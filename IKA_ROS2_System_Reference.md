# İKA — ROS 2 Jazzy Tabanlı Otonom Kara Aracı
## Tam Sistem Geliştirme Referans Dokümanı

> **Versiyon:** 1.0-dev  
> **Platform:** Raspberry Pi 5 (16 GB) · Ubuntu Server 24.04 LTS · ROS 2 Jazzy  
> **Simülasyon:** Gazebo Harmonic  
> **Durum:** Geliştirme aşaması — simülasyon önce doğrulanacak, ardından gerçek donanıma taşınacak

---

## İçindekiler

1. [Proje Genel Bakış](#1-proje-genel-bakış)
2. [Donanım Mimarisi](#2-donanım-mimarisi)
3. [Yazılım Mimarisi](#3-yazılım-mimarisi)
4. [ROS 2 Paket Yapısı](#4-ros-2-paket-yapısı)
5. [Düşük Seviye Kontrol — Arduino Katmanı](#5-düşük-seviye-kontrol--arduino-katmanı)
6. [Odometri Stratejisi — Encoder Yok / Lidar Odometri](#6-odometri-stratejisi--encoder-yok--lidar-odometri)
7. [Sensör Entegrasyonu](#7-sensör-entegrasyonu)
8. [Lokalizasyon ve Haritalama](#8-lokalizasyon-ve-haritalama)
9. [Nav2 Yapılandırması](#9-nav2-yapılandırması)
10. [Terrain Perception Node](#10-terrain-perception-node)
11. [Safety Supervisor Node](#11-safety-supervisor-node)
12. [Collision Monitor Yapılandırması](#12-collision-monitor-yapılandırması)
13. [Görev Yöneticisi](#13-görev-yöneticisi)
14. [Gazebo Simülasyon Kurulumu](#14-gazebo-simülasyon-kurulumu)
15. [Gerçek Araca Geçiş Prosedürü](#15-gerçek-araca-geçiş-prosedürü)
16. [Parametre Referansı](#16-parametre-referansı)
17. [Tanılama ve İzleme](#17-tanılama-ve-izleme)
18. [Güvenlik Protokolleri](#18-güvenlik-protokolleri)
19. [Test Prosedürleri](#19-test-prosedürleri)
20. [Bilinen Kısıtlar ve Açık Konular](#20-bilinen-kısıtlar-ve-açık-konular)

---

## 1. Proje Genel Bakış

### 1.1 Sistem Tanımı

İKA, iç ve dış ortamda özerk hareket edebilen dört tekerlekli skid-steer bir kara aracıdır. Sistem; 2D lidar, depth kamera, IMU ve GPS sensörlerini birleştirerek SLAM tabanlı haritalama, Nav2 ile yol planlama ve özel terrain/safety katmanları üzerinden karar verme yeteneğine sahiptir.

Bu doküman, sistemin her katmanını geliştirici düzeyinde açıklamayı hedefler. Kod örnekleri, parametre açıklamaları ve entegrasyon notları bu dokümanda birlikte yer almaktadır.

### 1.2 Temel Yetenekler

| Yetenek | Durum | Notlar |
|---|---|---|
| SLAM haritalama | Planlandı | SLAM Toolbox, lidar odometri ile |
| Nav2 otonom navigasyon | Planlandı | Encoder yok, lidar odom ile düşük hızda |
| Engelden kaçınma | Planlandı | Collision Monitor + local planner |
| Terrain değerlendirme | Planlandı | Özel node, depth kamera + IMU |
| Çukur / düşme algılama | Planlandı | Depth kamera zemin analizi |
| Rampa sınıflandırması | Planlandı | Eğim + IMU pitch füzyonu |
| GPS waypoint görevi | Planlandı | navsat_transform + Nav2 |
| Güvenli duruş (failsafe) | Planlandı | Safety Supervisor, lifecycle yönetimi |
| Keepout zone uyumu | Planlandı | Nav2 costmap keepout layer |

### 1.3 Mimari Felsefesi

- **Katmanlı güvenlik:** Collision Monitor → Safety Supervisor → Arduino E-Stop zincirleme güvenlik.
- **Parametrik tasarım:** Araç boyutu, hız sınırları, zemin eşikleri hiçbir zaman kodda sabit olmayacak; YAML parametreleri ile yönetilecek.
- **Simülasyon önce:** Her davranış önce Gazebo Harmonic'te doğrulanmadan gerçek araca taşınmayacak.
- **Modüler node'lar:** Her işlevsel birim ayrı ROS 2 lifecycle node'u olarak implemente edilecek.

---

## 2. Donanım Mimarisi

### 2.1 Bileşen Listesi

| Bileşen | Model / Detay | Arayüz |
|---|---|---|
| Ana bilgisayar | Raspberry Pi 5, 16 GB RAM | — |
| İşletim sistemi | Ubuntu Server 24.04 LTS, 64-bit | — |
| 2D Lidar | SLAMTEC RPLIDAR C1 | USB Serial |
| Depth kamera | Luxonis OAK-D Lite (öncelikli) | USB 3.0 |
| IMU | TBD — MPU-9250 veya eşdeğer | I2C / SPI |
| GPS | TBD — UBLOX NEO-M8N veya eşdeğer | USB / UART |
| Mikrodenetleyici | Arduino Uno | USB Serial |
| Motor sürücüler | TBD — L298N veya eşdeğer | Arduino GPIO |
| Motorlar | 4× DC motor (skid-steer) | Motor sürücü |
| Encoder | **Mevcut değil** | — |
| Acil durdurma | Fiziksel anahtar veya enerji kesme | Doğrudan motor/güç hattı |

> **Not:** Encoder yokluğu odometri güvenilirliğini kısıtlar. Bkz. Bölüm 6.

### 2.2 Skid-Steer Motor Gruplaması

Araç differential-drive / skid-steer mantığında çalışmaktadır. Motor gruplaması sağ-sol olmalıdır:

```
Sol grup:   Sol ön motor + Sol arka motor  →  left_wheel_velocity
Sağ grup:  Sağ ön motor + Sağ arka motor  →  right_wheel_velocity
```

ROS Nav2'den gelen `cmd_vel` (linear.x, angular.z) değerleri şu formülle sol/sağ hıza dönüştürülür:

```
v_left  = linear.x - (angular.z × wheel_base / 2)
v_right = linear.x + (angular.z × wheel_base / 2)
```

`wheel_base` parametresi Arduino tarafında YAML'dan beslenmelidir. Gerçek araç değeri bantla ölçülerek kalibre edilecektir.

### 2.3 Donanım Bağlantı Şeması

```
Raspberry Pi 5
├── USB 3.0  ── OAK-D Lite (depth kamera)
├── USB      ── RPLIDAR C1 (lidar)  →  /dev/ttyUSBx
├── USB      ── Arduino Uno         →  /dev/ttyACMx
├── USB/UART ── GPS modülü          →  /dev/ttyUSBx
└── I2C/SPI  ── IMU

Arduino Uno
├── Motor Sürücü A ── Sol ön + sol arka motor
└── Motor Sürücü B ── Sağ ön + sağ arka motor

Güç Hattı
└── Fiziksel E-Stop anahtarı → Motor sürücü güç hattı
```

### 2.4 Acil Durdurma Gereksinimleri

- Fiziksel E-Stop anahtarı motor sürücülerin güç hattını doğrudan kesmeli; yazılıma bağımlı olmamalıdır.
- Arduino da kendi tarafında `cmd_vel` zaman aşımı (watchdog) ile motorları durdurabilmelidir.
- Raspberry Pi tarafında Safety Supervisor `/cmd_vel_safe` üzerinden sıfır hız yayımlayabilmelidir.

---

## 3. Yazılım Mimarisi

### 3.1 Tam Sistem Veri Akışı

```
                    ┌──────────────────────────┐
                    │      Görev Yöneticisi     │
                    │  GPS hedef / RViz hedefi  │
                    └────────────┬─────────────┘
                                 │ /goal_pose veya /waypoints
                                 ▼
                       ┌───────────────────┐
                       │       Nav2        │
                       │  Planner Server   │
                       │  Controller Server│
                       │  BT Navigator     │
                       └────────┬──────────┘
                                │ /cmd_vel_nav
                                ▼
                    ┌────────────────────────┐
                    │   Collision Monitor    │
                    │  (Nav2 bileşeni)       │
                    └────────────┬───────────┘
                                 │ /cmd_vel_collision
                                 ▼
                    ┌────────────────────────┐
                    │   Safety Supervisor    │
                    │  (özel IKA node)       │
                    └────────────┬───────────┘
                                 │ /cmd_vel_safe
                                 ▼
                    ┌────────────────────────┐
                    │  Arduino Base          │
                    │  Controller (serial)   │
                    └────────────┬───────────┘
                                 │ Serial: JSON veya basit protokol
                                 ▼
                              Motorlar


Sensör Akışları:
─────────────────────────────────────────────────────────────
RPLIDAR C1      →  /scan               →  SLAM / Costmap / Collision Monitor
OAK-D Lite      →  /depth/image_raw    →  Voxel Layer
                →  /depth/points       →  Terrain Perception Node
IMU             →  /imu/data           →  robot_localization / Terrain Node
GPS             →  /gps/fix            →  navsat_transform / Görev Yöneticisi
Lidar Odom      →  /odom               →  robot_localization (encoder yok)

Çıktı Akışları:
─────────────────────────────────────────────────────────────
Terrain Node    →  /terrain_markers       →  Costmap Custom Layer
                →  /terrain_obstacles     →  Safety Supervisor
                →  /terrain_state         →  Görev Yöneticisi / RViz
Safety Supervisor → /safety_status        →  Diagnostics / RViz
                  → /cmd_vel_safe         →  Arduino Controller
```

### 3.2 Topic Listesi

| Topic | Tür | Yayımlayan | Dinleyen |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | sllidar_ros2 | SLAM, costmap, collision |
| `/depth/image_raw` | `sensor_msgs/Image` | depthai | voxel layer |
| `/depth/points` | `sensor_msgs/PointCloud2` | depthai | terrain node |
| `/imu/data` | `sensor_msgs/Imu` | IMU driver | robot_localization, terrain |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS driver | navsat_transform |
| `/odom` | `nav_msgs/Odometry` | lidar_odometry | robot_localization |
| `/odometry/filtered` | `nav_msgs/Odometry` | robot_localization | Nav2 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Nav2 global costmap |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 controller | Collision Monitor |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | Safety Supervisor | Arduino controller |
| `/terrain_obstacles` | `nav_msgs/OccupancyGrid` | terrain node | costmap custom layer |
| `/terrain_state` | `std_msgs/String` (JSON) | terrain node | görev yöneticisi |
| `/safety_status` | `std_msgs/String` (JSON) | safety supervisor | diagnostics, RViz |
| `/e_stop` | `std_msgs/Bool` | safety supervisor | arduino controller |

### 3.3 Frame Ağacı (TF)

```
map
└── odom
    └── base_link
        ├── base_footprint
        ├── laser_frame       (RPLIDAR C1)
        ├── camera_frame      (OAK-D Lite)
        │   └── camera_depth_optical_frame
        ├── imu_frame
        └── gps_frame
```

Tüm statik transform'lar `robot_state_publisher` üzerinden URDF/Xacro aracılığıyla yayımlanacaktır.

---

## 4. ROS 2 Paket Yapısı

### 4.1 Önerilen Workspace Düzeni

```
ika_ws/
├── src/
│   ├── ika_bringup/            # Launch dosyaları, üst düzey başlatma
│   ├── ika_description/        # URDF/Xacro, mesh dosyaları
│   ├── ika_navigation/         # Nav2 parametreleri, costmap yapılandırması
│   ├── ika_terrain/            # Terrain Perception Node
│   ├── ika_safety/             # Safety Supervisor Node
│   ├── ika_base_controller/    # Arduino serial köprüsü (ROS 2 tarafı)
│   ├── ika_mission/            # Görev Yöneticisi
│   └── ika_simulation/         # Gazebo world dosyaları, model konfigürasyonları
├── config/
│   ├── robot_params.yaml       # Araç boyutu, kinematik parametreler
│   ├── nav2_params.yaml        # Nav2 tam yapılandırma
│   ├── slam_params.yaml        # SLAM Toolbox parametreleri
│   ├── ekf_params.yaml         # robot_localization EKF parametreleri
│   ├── terrain_params.yaml     # Terrain algılama eşikleri
│   └── safety_params.yaml      # Güvenlik zone mesafeleri, eşikler
└── launch/
    ├── simulation.launch.py    # Gazebo + tüm sistem
    ├── real_robot.launch.py    # Gerçek araç başlatma
    ├── slam.launch.py          # SLAM modu
    ├── navigation.launch.py    # Navigasyon modu (harita yüklü)
    └── sensors.launch.py       # Yalnız sensör testi
```

### 4.2 Bağımlılıklar

```bash
# Temel ROS 2 Jazzy paketleri
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-lifecycle-msgs

# Sensör sürücüleri
sudo apt install ros-jazzy-sllidar-ros2   # RPLIDAR C1

# Gazebo Harmonic
sudo apt install ros-jazzy-ros-gz

# OAK-D Lite (depthai)
pip3 install depthai
sudo apt install ros-jazzy-depthai-ros    # veya kaynaktan derleme
```

---

## 5. Düşük Seviye Kontrol — Arduino Katmanı

### 5.1 İletişim Protokolü

Arduino Uno ile Raspberry Pi arasındaki iletişim **USB Serial üzerinden** gerçekleşir. İletişim için iki seçenek değerlendirilecektir:

**Seçenek A — micro-ROS (önerilen):**
- Arduino doğrudan ROS 2 ekosistemiyle entegre olur.
- `/cmd_vel` subscriber ve `/odom` publisher (encoder eklendikten sonra) Arduino üzerinde çalışır.
- Raspberry Pi'de micro-ROS Agent çalıştırılır.

**Seçenek B — Özel Serial Protokol:**
- Raspberry Pi'de ROS 2 düğümü Arduino ile JSON veya basit ikili protokol üzerinden haberleşir.
- Daha kolay debug edilir, micro-ROS kütüphanesine bağımlılık yoktur.
- Encoder eklendiğinde odometri verisini de alabilir.

> **Şu anki karar:** Seçenek B ile başlanacak, encoder entegrasyonu planlandığında micro-ROS değerlendirilebilir.

### 5.2 Raspberry Pi Tarafı — Base Controller Node

```python
# ika_base_controller/base_controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import json

class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('ika_base_controller')

        # Parametreler — gerçek araç değerleriyle kalibre edilecek
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.30)        # metre — ölçülecek
        self.declare_parameter('max_linear_speed', 0.3)   # m/s — kalibre edilecek
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s — kalibre edilecek
        self.declare_parameter('cmd_vel_timeout', 0.5)    # saniye

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Arduino bağlantısı kuruldu: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Arduino bağlantı hatası: {e}')
            self.serial = None

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_safe', self.cmd_vel_callback, 10)
        self.e_stop_sub = self.create_subscription(
            Bool, '/e_stop', self.e_stop_callback, 10)

        # Watchdog — cmd_vel gelmezse motorları durdur
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        wheel_base = self.get_parameter('wheel_base').value
        v_left  = msg.linear.x - (msg.angular.z * wheel_base / 2.0)
        v_right = msg.linear.x + (msg.angular.z * wheel_base / 2.0)
        self._send_command(v_left, v_right)

    def e_stop_callback(self, msg: Bool):
        if msg.data:
            self._send_command(0.0, 0.0)
            self.get_logger().warn('E-STOP aktif — motorlar durduruldu')

    def watchdog_callback(self):
        timeout = self.get_parameter('cmd_vel_timeout').value
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > timeout:
            self._send_command(0.0, 0.0)

    def _send_command(self, v_left: float, v_right: float):
        if self.serial is None:
            return
        max_v = self.get_parameter('max_linear_speed').value
        v_left  = max(-max_v, min(max_v, v_left))
        v_right = max(-max_v, min(max_v, v_right))
        cmd = json.dumps({'l': round(v_left, 3), 'r': round(v_right, 3)}) + '\n'
        try:
            self.serial.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial yazma hatası: {e}')
```

### 5.3 Arduino Tarafı — Motor Kontrol Kodu

```cpp
// arduino/ika_motor_controller/ika_motor_controller.ino

#include <ArduinoJson.h>

// --- Pin Tanımları ---
// Sol grup (sol ön + sol arka motorlar aynı sürücüye)
const int LEFT_PWM  = 5;
const int LEFT_IN1  = 6;
const int LEFT_IN2  = 7;

// Sağ grup (sağ ön + sağ arka motorlar aynı sürücüye)
const int RIGHT_PWM = 10;
const int RIGHT_IN1 = 8;
const int RIGHT_IN2 = 9;

// --- Parametreler ---
const float MAX_SPEED_MPS    = 0.30;   // Gerçek araç ölçümüyle kalibre edilecek
const int   MAX_PWM          = 200;    // 0-255 arası; motor sürücüye göre ayarlanacak
const int   MIN_PWM          = 60;     // Ölü bant — motor hareket ettiği en düşük PWM
const unsigned long TIMEOUT_MS = 500; // Bu sürede komut gelmezse dur

unsigned long lastCmdTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();
}

void loop() {
  // Watchdog
  if (millis() - lastCmdTime > TIMEOUT_MS) {
    stopMotors();
  }

  // Serial komut oku
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (!err) {
      float v_left  = doc["l"] | 0.0f;
      float v_right = doc["r"] | 0.0f;
      setMotor(LEFT_PWM,  LEFT_IN1,  LEFT_IN2,  v_left);
      setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, v_right);
      lastCmdTime = millis();
    }
  }
}

void setMotor(int pwmPin, int in1, int in2, float speed_mps) {
  int dir = (speed_mps >= 0) ? 1 : -1;
  float absSpeed = abs(speed_mps);
  int pwm = 0;

  if (absSpeed > 0.001) {
    // Hız → PWM lineer dönüşüm + ölü bant tazminatı
    pwm = (int)(MIN_PWM + (absSpeed / MAX_SPEED_MPS) * (MAX_PWM - MIN_PWM));
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  }

  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir > 0 ? LOW  : HIGH);
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(LEFT_PWM,  0);
  analogWrite(RIGHT_PWM, 0);
}
```

> **Kalibrasyon notu:** `MAX_SPEED_MPS`, `MAX_PWM`, `MIN_PWM` değerleri gerçek araçta ölçülerek ayarlanmalıdır. İlk çalıştırmada düşük PWM değerleriyle başlanmalıdır.

---

## 6. Odometri Stratejisi — Encoder Yok / Lidar Odometri

### 6.1 Encoder Yokluğunun Sonuçları

Tekerlek encoder'ı olmadan doğru tekometri üretmek mümkün değildir. Bu durum şu kısıtları doğurur:

| Kısıt | Açıklama |
|---|---|
| Hız tahmini yok | Motorlara gönderilen komuttan hız tahmini yapılabilir ancak kayma ve yük etkisi hesaba katılamaz |
| Nav2 güvenilirliği düşer | Lokal planner'ın hız geri bildirimi sınırlı kalır |
| Loop closure bağımlılığı artar | SLAM doğruluğu lidar loop closure'a daha fazla bağımlı hale gelir |
| Yüksek hız tehlikeli | Hız arttıkça odometri hatası birikir; maksimum 0.2–0.3 m/s önerilir |

### 6.2 Lidar Odometri ile Çalışma

Encoder yokken lidar tabanlı odometri hesaplanacaktır. Bunun için iki yaygın yaklaşım mevcuttur:

**rf2o_laser_odometry (önerilen):**
```bash
sudo apt install ros-jazzy-rf2o-laser-odometry
```

Temel yapılandırma (`config/rf2o_params.yaml`):
```yaml
rf2o_laser_odometry:
  ros__parameters:
    laser_scan_topic: /scan
    odom_topic: /odom
    publish_tf: false          # TF'i robot_localization yayımlayacak
    base_frame_id: base_link
    odom_frame_id: odom
    init_pose_from_topic: ""
    freq: 10.0                 # Hz — lidar frekansına göre ayarlanacak
```

**slam_toolbox'ın dahili odom tahmini:**
SLAM Toolbox, `localization_mode: false` ile çalışırken kendi iç odometrisini üretebilir; ancak bu haritalama modunda daha güvenilirdir.

### 6.3 robot_localization EKF Yapılandırması (Encoder Yok)

```yaml
# config/ekf_params.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true

    odom0: /odom                   # lidar odometri kaynağı
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false

    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true

    base_link_frame: base_link
    world_frame: odom
    odom_frame: odom
    map_frame: map

    # Encoder yokken lidar odom güveni artırılmış
    process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.06, 0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.03, 0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.03, 0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.06]
```

> **Önemli:** Encoder eklendiğinde bu yapılandırma önemli ölçüde değişecektir. `odom0` lidar odomdan encoder odometrisine geçirilecek ve kovariyans değerleri yeniden ayarlanacaktır.

---

## 7. Sensör Entegrasyonu

### 7.1 RPLIDAR C1

```bash
# Sürücü başlatma
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:=/dev/ttyUSB0 \
  frame_id:=laser_frame
```

URDF'de statik transform:
```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_frame"/>
  <origin xyz="0.15 0.0 0.12" rpy="0 0 0"/>
  <!-- xyz: araç merkezinden öne 15cm, yerden 12cm -->
  <!-- Gerçek montaj pozisyonuyla güncellenecek -->
</joint>
```

> **Önemli kısıt:** RPLIDAR C1 yatay düzlemde tarama yapar. Negatif engeller (çukur, merdiven inişi) bu sensörle algılanamaz. Çukur algılama tamamen depth kameraya devredilmektedir.

### 7.2 OAK-D Lite Depth Kamera

```bash
# depthai-ros başlatma
ros2 launch depthai_ros_driver camera.launch.py \
  camera_model:=OAK-D-LITE \
  camera_name:=oak \
  base_frame:=camera_frame \
  parent_frame:=base_link \
  cam_pos_x:=0.10 \
  cam_pos_y:=0.0 \
  cam_pos_z:=0.15 \
  cam_roll:=0.0 \
  cam_pitch:=0.15 \  # ~8.6 derece aşağı eğim — zemin görüşü için
  cam_yaw:=0.0
```

Yayımlanan başlıca topic'ler:
- `/oak/stereo/image_raw` — ham stereo görüntü
- `/oak/stereo/camera_info`
- `/oak/depth/image_raw` — 16-bit depth görüntüsü (mm)
- `/oak/points` — PointCloud2

> **Kamera açısı notu:** `cam_pitch` değeri terrain perception için kritiktir. Kamera öne ve hafif aşağıya bakmalıdır. Gerçek araçta montaj açısı ölçülüp bu değer güncellenecektir.

### 7.3 IMU

IMU sürücüsü platforma göre değişecektir (MPU-9250, BNO055 vb.). Standart çıktı topic'i:

```
/imu/data  →  sensor_msgs/Imu
```

IMU kalibrasyonu için robotik kalibrasyon prosedürü uygulanacaktır:
1. Düz zeminde statik kalibrasyon — bias hesaplama.
2. Yavaş dönüş testi — gyro ölçeği doğrulama.
3. Eğim testi — pitch/roll doğruluğu.

### 7.4 GPS

```bash
# nmea_navsat_driver veya ublox_gps_node
ros2 run nmea_navsat_driver nmea_serial_driver \
  --ros-args \
  -p port:=/dev/ttyUSB1 \
  -p baud:=9600
```

navsat_transform_node yapılandırması:
```yaml
navsat_transform:
  ros__parameters:
    frequency: 10.0
    delay: 3.0
    magnetic_declination_radians: 0.0   # Konum için hesaplanacak
    yaw_offset: 0.0
    zero_altitude: true
    broadcast_utm_transform: false
    publish_filtered_gps: true
    use_odometry_yaw: false
    wait_for_datum: false
```

---

## 8. Lokalizasyon ve Haritalama

### 8.1 SLAM Toolbox

```yaml
# config/slam_params.yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan

    mode: mapping              # mapping veya localization

    # Lidar odometriye güven artırılmış (encoder yok)
    minimum_travel_distance: 0.3   # metre — daha sık güncelleme
    minimum_travel_heading: 0.3    # radyan

    resolution: 0.05               # harita çözünürlüğü — 5cm

    max_laser_range: 12.0          # RPLIDAR C1 menzili
    min_laser_range: 0.15

    use_scan_matching: true
    use_scan_barycenter: true

    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Encoder yokken daha agresif loop closure
    do_loop_closing: true
    loop_search_space_dimension: 8.0
```

### 8.2 Haritalama ve Navigasyon Modları

**Haritalama modu** (`slam_params.yaml` → `mode: mapping`):
- Ortam keşfedilirken harita oluşturulur.
- `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePosegraph` ile kaydedilir.

**Navigasyon modu** (`mode: localization`):
- Önceden kaydedilmiş harita yüklenir.
- SLAM Toolbox haritada lokalizasyon yapar.
- Nav2 bu harita üzerinde yol planlama yapar.

---

## 9. Nav2 Yapılandırması

### 9.1 Temel nav2_params.yaml Yapısı

```yaml
# config/nav2_params.yaml

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: ""  # Varsayılan BT kullanılacak

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0       # Hz — Raspberry Pi kapasitesine göre
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    odom_topic: /odometry/filtered
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.25        # m/s — encoder yokken düşük tutulmalı
      max_vel_y: 0.0
      max_vel_theta: 0.8
      min_speed_xy: 0.0
      max_speed_xy: 0.25
      min_speed_theta: 0.0
      acc_lim_x: 1.5         # m/s^2 — araç ataletine göre ayarlanacak
      acc_lim_y: 0.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.5
      decel_lim_y: 0.0
      decel_lim_theta: -2.0
      vx_samples: 15
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      critics:
        - "RotateToGoal"
        - "Oscillation"
        - "BaseObstacle"
        - "GoalAlign"
        - "PathAlign"
        - "PathDist"
        - "GoalDist"
      PathAlign.scale: 32.0
      PathDist.scale: 32.0
      GoalAlign.scale: 24.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false       # Dijkstra varsayılan; A* mümkün
      allow_unknown: true
```

### 9.2 Global Costmap

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.25       # metre — araç yarıçapı + güvenlik payı
      resolution: 0.05
      track_unknown_space: true
      plugins:
        - "static_layer"
        - "obstacle_layer"
        - "keepout_filter"
        - "terrain_layer"      # özel terrain costmap layer
        - "inflation_layer"

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan depth_points
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 8.0
          raytrace_min_range: 0.0
          obstacle_max_range: 6.0
          obstacle_min_range: 0.0
        depth_points:
          topic: /oak/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.05
          clearing: true
          marking: true
          data_type: "PointCloud2"
          raytrace_max_range: 4.0
          obstacle_max_range: 3.5

      terrain_layer:
        plugin: "nav2_costmap_2d::CostmapLayer"  # özel layer ile değiştirilecek
        enabled: true
        topic: /terrain_obstacles

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: /costmap_filter_info

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45   # robot_radius + güvenlik payı
```

### 9.3 Local Costmap

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 4            # metre
      height: 4
      resolution: 0.05
      robot_radius: 0.25
      plugins:
        - "voxel_layer"
        - "terrain_layer"
        - "inflation_layer"

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan depth_points
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 6.0
          obstacle_max_range: 5.5
        depth_points:
          topic: /oak/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.02
          clearing: true
          marking: true
          data_type: "PointCloud2"
          raytrace_max_range: 3.0
          obstacle_max_range: 2.5

      terrain_layer:
        plugin: "nav2_costmap_2d::CostmapLayer"
        topic: /terrain_obstacles
        enabled: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45
```

---

## 10. Terrain Perception Node

### 10.1 Amaç ve Kapsam

`ika_terrain` node'u aşağıdaki algılama işlevlerini yerine getirir:

- **Çukur / düşme kenarı algılama:** Depth kameradan zemin düzlemi analizi.
- **Rampa sınıflandırması:** Zemin eğimi ve IMU pitch füzyonu.
- **Alçak engel tespiti:** Lidar'ın göremeyeceği zemin seviyesi engeller.
- **Sonuçları costmap'e yayımlama:** `/terrain_obstacles` topic'i.

### 10.2 Parametre Dosyası

```yaml
# config/terrain_params.yaml
terrain_perception:
  ros__parameters:
    # Çukur / kenar algılama
    dropoff_depth_threshold_m: 0.15      # Zeminden bu kadar aşağı düşme = tehlike
    dropoff_detection_width_m: 0.40      # Araç genişliği boyunca kaç cm taranacak
    dropoff_lookout_distance_m: 0.60     # Aracın önünde ne kadar mesafeye bakılacak
    ground_plane_fit_tolerance_m: 0.04   # RANSAC zemin düzlemi uyum toleransı

    # Rampa sınıflandırması
    max_safe_slope_deg: 15.0             # Bu eğime kadar güvenli — kalibrasyon gerekli
    max_caution_slope_deg: 25.0          # Yavaşlama bölgesi — kalibrasyon gerekli
    # max_caution_slope_deg'in üstü = geçilemez

    # Adım / engel yüksekliği
    max_step_height_m: 0.04              # Tırmanılabilir maksimum adım — kalibrasyon gerekli

    # Genel
    terrain_confidence_threshold: 0.6    # Düşük güven → belirsiz, geçilemez say
    terrain_slowdown_speed_mps: 0.10     # Riskli alanda uygulanan hız sınırı

    # Yayımlama
    costmap_resolution: 0.05             # terrain obstacle grid çözünürlüğü
    obstacle_decay_time_s: 2.0           # Eski terrain engeli ne kadar süre tutulacak

    # Sensör
    depth_topic: /oak/depth/image_raw
    points_topic: /oak/points
    imu_topic: /imu/data
    camera_info_topic: /oak/stereo/camera_info
```

### 10.3 Node İskeleti

```python
# ika_terrain/terrain_perception_node.py
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image, PointCloud2, Imu, CameraInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np

class TerrainPerceptionNode(LifecycleNode):
    """
    Terrain Perception Node:
    - Depth kamera + IMU verilerinden zemin analizi yapar.
    - Çukur, rampa, alçak engel bilgilerini costmap'e yayımlar.
    - Safety Supervisor ile terrain_state paylaşır.
    """

    def __init__(self):
        super().__init__('ika_terrain_perception')
        self._load_parameters()

    def _load_parameters(self):
        self.declare_parameter('dropoff_depth_threshold_m', 0.15)
        self.declare_parameter('max_safe_slope_deg', 15.0)
        self.declare_parameter('max_caution_slope_deg', 25.0)
        self.declare_parameter('max_step_height_m', 0.04)
        self.declare_parameter('dropoff_lookout_distance_m', 0.60)
        self.declare_parameter('terrain_confidence_threshold', 0.6)
        self.declare_parameter('terrain_slowdown_speed_mps', 0.10)
        self.declare_parameter('obstacle_decay_time_s', 2.0)

    def on_configure(self, state):
        self.get_logger().info('TerrainNode: konfigüre ediliyor')

        # Subscriber'lar
        self.depth_sub = self.create_subscription(
            Image, '/oak/depth/image_raw', self._depth_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, 10)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/oak/stereo/camera_info', self._cam_info_callback, 1)

        # Publisher'lar
        self.terrain_pub = self.create_publisher(
            OccupancyGrid, '/terrain_obstacles', 10)
        self.state_pub = self.create_publisher(
            String, '/terrain_state', 10)

        self.latest_imu: Imu = None
        self.camera_info: CameraInfo = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('TerrainNode: aktif')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        return TransitionCallbackReturn.SUCCESS

    def _imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def _cam_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def _depth_callback(self, msg: Image):
        """
        Depth görüntüsünden:
        1. Zemin düzlemini tahmin et (RANSAC benzeri).
        2. Ön bölgede zemin kaybı veya anormal derinlik düşüşü kontrol et → çukur.
        3. Eğim hesapla → rampa sınıflandır.
        4. Alçak engeller → işaretle.
        """
        if self.camera_info is None:
            return

        # Ham depth verisini numpy array'e dönüştür
        depth_array = self._decode_depth(msg)

        # Çukur algılama
        dropoff_risk, dropoff_confidence = self._detect_dropoff(depth_array)

        # Rampa / eğim
        slope_deg, slope_confidence = self._estimate_slope(depth_array)

        # Durum sınıflandırması
        terrain_class = self._classify_terrain(dropoff_risk, slope_deg, slope_confidence)

        # Costmap güncellemesi
        grid = self._build_obstacle_grid(dropoff_risk, slope_deg, terrain_class)
        self.terrain_pub.publish(grid)

        # Durum yayımla
        import json
        state_msg = String()
        state_msg.data = json.dumps({
            'class': terrain_class,
            'slope_deg': round(slope_deg, 2),
            'dropoff_risk': dropoff_risk,
            'confidence': round(slope_confidence, 2)
        })
        self.state_pub.publish(state_msg)

    def _decode_depth(self, msg: Image) -> np.ndarray:
        """16-bit depth görüntüsünü mm'den metreye çevir."""
        import struct
        raw = np.frombuffer(msg.data, dtype=np.uint16)
        return raw.reshape(msg.height, msg.width).astype(np.float32) / 1000.0

    def _detect_dropoff(self, depth: np.ndarray):
        """
        Kameranın önündeki bölgede zemin seviyesinin aniden düştüğünü tespit et.
        Basit yaklaşım: beklenen zemin mesafesi ile alınan mesafeyi karşılaştır.
        Gerçek implementasyon için RANSAC tabanlı zemin düzlemi uydurma önerilir.
        """
        threshold = self.get_parameter('dropoff_depth_threshold_m').value
        # ROI: görüntünün ön-orta bölgesi
        h, w = depth.shape
        roi = depth[h//2:h*3//4, w//4:w*3//4]
        valid = roi[roi > 0.1]
        if len(valid) < 10:
            return False, 0.0
        mean_depth = float(np.median(valid))
        bottom_strip = depth[h*3//4:, w//4:w*3//4]
        valid_bottom = bottom_strip[bottom_strip > 0.1]
        if len(valid_bottom) < 10:
            return True, 0.8   # Veri yoksa risk var say
        mean_bottom = float(np.median(valid_bottom))
        drop = mean_bottom - mean_depth
        confidence = min(1.0, drop / (threshold * 2))
        return drop > threshold, confidence

    def _estimate_slope(self, depth: np.ndarray):
        """
        Depth görüntüsünden basit eğim tahmini.
        IMU pitch ile desteklenirse güvenilirlik artar.
        """
        if self.latest_imu is None:
            return 0.0, 0.3  # IMU yoksa düşük güven

        import math
        q = self.latest_imu.orientation
        # Quaternion'dan pitch açısını hesapla
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_rad = math.asin(sinp)
        pitch_deg = math.degrees(pitch_rad)
        return pitch_deg, 0.85

    def _classify_terrain(self, dropoff_risk: bool, slope_deg: float, confidence: float) -> str:
        if dropoff_risk:
            return 'DROPOFF_DANGER'
        safe = self.get_parameter('max_safe_slope_deg').value
        caution = self.get_parameter('max_caution_slope_deg').value
        thresh = self.get_parameter('terrain_confidence_threshold').value
        if confidence < thresh:
            return 'UNKNOWN'
        if abs(slope_deg) <= safe:
            return 'SAFE'
        elif abs(slope_deg) <= caution:
            return 'CAUTION'
        else:
            return 'IMPASSABLE'

    def _build_obstacle_grid(self, dropoff_risk: bool, slope_deg: float, terrain_class: str):
        """Terrain durumunu nav_msgs/OccupancyGrid olarak yayımla."""
        grid = OccupancyGrid()
        grid.header.frame_id = 'base_link'
        grid.header.stamp = self.get_clock().now().to_msg()
        # Basit 1×1 metrelik ön bölge grid'i
        res = self.get_parameter('costmap_resolution').value
        size = int(1.0 / res)
        grid.info.resolution = res
        grid.info.width = size
        grid.info.height = size
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = -0.5
        cost = 0
        if terrain_class == 'DROPOFF_DANGER':
            cost = 100
        elif terrain_class == 'IMPASSABLE':
            cost = 100
        elif terrain_class == 'CAUTION':
            cost = 60
        elif terrain_class == 'UNKNOWN':
            cost = 50
        grid.data = [cost] * (size * size)
        return grid
```

> **Not:** `_detect_dropoff` ve `_estimate_slope` işlevleri iskelet implementasyondur. Gerçek araçta RANSAC tabanlı zemin düzlemi uydurma ve IMU füzyonu ile geliştirilecektir.

---

## 11. Safety Supervisor Node

### 11.1 Sorumluluklar

Safety Supervisor, tüm güvenlik koşullarını merkezi olarak yönetir:

- Terrain Node'dan gelen `DROPOFF_DANGER` veya `IMPASSABLE` durumunda aracı durdurur.
- Sensör topic zaman aşımlarını izler (lidar, depth kamera, IMU).
- Collision Monitor'ın atladığı durumlar için son kalkan görevi görür.
- `/e_stop` yayımlayarak Arduino katmanını da etkiler.
- Tüm güvenlik kararlarını `/safety_status` ile yayımlar.

### 11.2 Parametre Dosyası

```yaml
# config/safety_params.yaml
safety_supervisor:
  ros__parameters:
    # Sensör zaman aşımı eşikleri
    lidar_timeout_s: 1.0
    depth_timeout_s: 1.5
    imu_timeout_s: 0.5

    # Engel mesafe zonları
    stop_zone_distance_m: 0.25       # Bu mesafede dur
    slowdown_zone_distance_m: 0.55   # Bu mesafede yavaşla
    slowdown_speed_factor: 0.3       # Yavaşlama bölgesinde hız çarpanı

    # Terrain entegrasyonu
    terrain_stop_classes:
      - "DROPOFF_DANGER"
      - "IMPASSABLE"
    terrain_slow_classes:
      - "CAUTION"
      - "UNKNOWN"

    # Genel
    watchdog_rate_hz: 20.0
    recovery_wait_s: 3.0             # Hata sonrası kurtarma bekleme süresi
```

### 11.3 Node İskeleti

```python
# ika_safety/safety_supervisor_node.py
import rclpy
import json
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

class SafetySupervisorNode(LifecycleNode):
    """
    Güvenlik katmanı: terrain, sensör zaman aşımı ve genel watchdog.
    /cmd_vel_collision → filtreler → /cmd_vel_safe
    """

    def __init__(self):
        super().__init__('ika_safety_supervisor')
        self._declare_params()

    def _declare_params(self):
        self.declare_parameter('lidar_timeout_s', 1.0)
        self.declare_parameter('depth_timeout_s', 1.5)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('stop_zone_distance_m', 0.25)
        self.declare_parameter('slowdown_zone_distance_m', 0.55)
        self.declare_parameter('slowdown_speed_factor', 0.3)
        self.declare_parameter('watchdog_rate_hz', 20.0)

    def on_configure(self, state):
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_collision', self._cmd_vel_callback, 10)
        self.terrain_sub = self.create_subscription(
            String, '/terrain_state', self._terrain_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.e_stop_pub  = self.create_publisher(Bool, '/e_stop', 10)
        self.status_pub  = self.create_publisher(String, '/safety_status', 10)

        self.terrain_class = 'UNKNOWN'
        self.last_lidar_time  = self.get_clock().now()
        self.last_depth_time  = self.get_clock().now()
        self.last_imu_time    = self.get_clock().now()
        self.e_stop_active = False

        rate = self.get_parameter('watchdog_rate_hz').value
        self.create_timer(1.0 / rate, self._watchdog)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('SafetySupervisor: aktif')
        return TransitionCallbackReturn.SUCCESS

    def _terrain_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.terrain_class = data.get('class', 'UNKNOWN')
        except json.JSONDecodeError:
            self.terrain_class = 'UNKNOWN'

    def _cmd_vel_callback(self, msg: Twist):
        """Gelen cmd_vel'i güvenlik durumuna göre filtrele."""
        if self.e_stop_active:
            self._publish_stop()
            return

        stop_classes = ['DROPOFF_DANGER', 'IMPASSABLE']
        slow_classes  = ['CAUTION', 'UNKNOWN']

        if self.terrain_class in stop_classes:
            self._publish_stop()
            self.get_logger().warn(f'Terrain dur: {self.terrain_class}')
            return

        if self.terrain_class in slow_classes:
            factor = self.get_parameter('slowdown_speed_factor').value
            filtered = Twist()
            filtered.linear.x  = msg.linear.x  * factor
            filtered.angular.z = msg.angular.z  * factor
            self.cmd_vel_pub.publish(filtered)
            return

        self.cmd_vel_pub.publish(msg)

    def _watchdog(self):
        """Sensör zaman aşımlarını kontrol et."""
        now = self.get_clock().now()

        def elapsed(t):
            return (now - t).nanoseconds / 1e9

        lidar_ok = elapsed(self.last_lidar_time) < self.get_parameter('lidar_timeout_s').value
        depth_ok = elapsed(self.last_depth_time) < self.get_parameter('depth_timeout_s').value
        imu_ok   = elapsed(self.last_imu_time)   < self.get_parameter('imu_timeout_s').value

        if not lidar_ok or not depth_ok or not imu_ok:
            self.e_stop_active = True
            self._publish_stop()
            e_stop_msg = Bool(); e_stop_msg.data = True
            self.e_stop_pub.publish(e_stop_msg)
            self.get_logger().error(
                f'Sensör zaman aşımı — lidar:{lidar_ok} depth:{depth_ok} imu:{imu_ok}')
        else:
            self.e_stop_active = False
            e_stop_msg = Bool(); e_stop_msg.data = False
            self.e_stop_pub.publish(e_stop_msg)

        status = {
            'e_stop': self.e_stop_active,
            'terrain_class': self.terrain_class,
            'lidar_ok': lidar_ok,
            'depth_ok': depth_ok,
            'imu_ok': imu_ok
        }
        msg = String(); msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _publish_stop(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
```

---

## 12. Collision Monitor Yapılandırması

Collision Monitor, Nav2'nin dahili güvenlik bileşenidir. `/cmd_vel_nav` komutunu lidar/depth verilerine göre filtreler.

```yaml
# nav2_params.yaml içinde collision_monitor bölümü
collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: base_link
    odom_frame_id: odom
    cmd_vel_in_topic: /cmd_vel_nav
    cmd_vel_out_topic: /cmd_vel_collision
    state_topic: /collision_monitor_state
    transform_tolerance: 0.5
    source_timeout: 2.0
    base_shift_correction: true
    stop_pub_timeout: 2.0

    polygons:
      - name: FootprintApproach
        type: polygon
        points: "[[0.30, 0.20], [0.30, -0.20], [-0.20, -0.20], [-0.20, 0.20]]"
        action_type: approach
        min_points: 4
        slowdown_ratio: 0.5
        time_before_collision: 2.0
        simulation_time_step: 0.02
        visualize: true
        polygon_pub_topic: /collision_poly_approach

      - name: StopZone
        type: circle
        radius: 0.25     # stop_zone_distance_m ile eşleşmeli
        action_type: stop
        min_points: 4
        visualize: true
        polygon_pub_topic: /collision_poly_stop

    observation_sources:
      - name: scan
        type: scan
        topic: /scan
      - name: depth_pointcloud
        type: pointcloud
        topic: /oak/points
        min_height: 0.02
        max_height: 1.5
```

---

## 13. Görev Yöneticisi

### 13.1 GPS Waypoint Görevi

```python
# ika_mission/gps_waypoint_mission.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml

class GPSWaypointMission(Node):
    """
    GPS koordinatlarından oluşan waypoint listesini sırayla nav2'ye gönderir.
    navsat_transform çıktısı olan /gps/fix → UTM → map frame dönüşümünü kullanır.
    """

    def __init__(self):
        super().__init__('ika_gps_mission')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_wp_idx = 0
        self.waypoints = []

    def load_waypoints(self, yaml_path: str):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.get_logger().info(f'{len(self.waypoints)} waypoint yüklendi')

    def start(self):
        if not self.waypoints:
            self.get_logger().error('Waypoint listesi boş')
            return
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info('Görev tamamlandı')
            return

        wp = self.waypoints[self.current_wp_idx]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}: '
            f'x={wp["x"]:.2f} y={wp["y"]:.2f}')

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Hedef reddedildi')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.current_wp_idx += 1
        self._send_next_waypoint()
```

Waypoint YAML formatı:
```yaml
# missions/test_mission.yaml
waypoints:
  - x: 2.0
    y: 0.5
    label: "hedef_1"
  - x: 5.0
    y: 1.2
    label: "hedef_2"
  - x: 0.0
    y: 0.0
    label: "baz"
```

---

## 14. Gazebo Simülasyon Kurulumu

### 14.1 Simülasyon Önceliği

Her davranış önce Gazebo Harmonic simülasyonunda doğrulanacaktır:

1. Collision Monitor ve Costmap doğrulama
2. Terrain Node çukur algılama testi
3. Safety Supervisor sensör zaman aşımı testi
4. GPS waypoint görev testi
5. Encoder yok senaryosu (lidar odom) doğrulama

### 14.2 Basit Simülasyon Launch

```python
# launch/simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
            PathJoinSubstitution([FindPackageShare('ika_simulation'), 'models'])),

        # Gazebo Harmonic
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ]),
            launch_arguments={
                'gz_args': PathJoinSubstitution([
                    FindPackageShare('ika_simulation'), 'worlds', 'test_world.sdf'
                ])
            }.items()
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': open('ika_description/urdf/ika.urdf.xacro').read()
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('ika_bringup'), 'rviz', 'ika_sim.rviz'
            ])]
        ),
    ])
```

### 14.3 Test World İçeriği

Test dünyası şu unsurları barındırmalıdır:

- Düz zemin — temel hareket testi
- Statik engeller (kutular) — collision avoidance
- Bir rampa — terrain sınıflandırma
- Çukur / platform kenarı simülasyonu — düşme algılama (köpük dolgulu veya keepout ile)
- Dar geçit — genişlik kısıtı testi
- Keepout zone — yasak alan uyumu

---

## 15. Gerçek Araca Geçiş Prosedürü

### 15.1 Geçiş Kontrol Listesi

Her madde simülasyonda doğrulandıktan sonra işaretlenmelidir:

**Donanım hazırlığı:**
- [ ] Fiziksel E-Stop test edildi (motorlar kesildi mi?)
- [ ] Arduino watchdog test edildi (USB çekilince motorlar durdu mu?)
- [ ] Tüm USB portları `/dev/ttyXXX` atamaları yapıldı (udev kuralları)
- [ ] Arduino kodu yüklendi ve `minicom` ile test edildi
- [ ] Lidar `/scan` yayımlanıyor ve RViz'de görünüyor
- [ ] OAK-D Lite `/oak/depth/image_raw` yayımlanıyor
- [ ] IMU verisi `/imu/data` üzerinde geliyor
- [ ] GPS fix alınıyor

**Yazılım hazırlığı:**
- [ ] `use_sim_time: false` tüm node'larda
- [ ] TF ağacı `ros2 run tf2_tools view_frames` ile doğrulandı
- [ ] EKF `/odometry/filtered` üretiliyor
- [ ] SLAM haritası oluşturuluyor (RViz'de görünüyor)
- [ ] Nav2 hedef kabul ediyor ve rota üretiyor
- [ ] Safety Supervisor `/safety_status` yayımlıyor

**İlk hareket testi (düşük hız, açık alan):**
- [ ] `max_vel_x: 0.10` ile başla
- [ ] `teleop_twist_keyboard` ile manuel kontrol test edildi
- [ ] Engel önünde araç duruyor mu?
- [ ] E-Stop tuşuna basıldığında araç durdu mu?

### 15.2 udev Kuralları

```bash
# /etc/udev/rules.d/99-ika-usb.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="ika_lidar", MODE="0666"

SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", \
  SYMLINK+="ika_arduino", MODE="0666"

# GPS modül ID'leri platforma göre güncellenecek
```

```bash
# Kuralları yenile
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 16. Parametre Referansı

### 16.1 robot_params.yaml — Araç Kinematiği

```yaml
# config/robot_params.yaml
# TÜM DEĞERLER gerçek araç ölçümleriyle kalibre edilecek
robot:
  wheel_base: 0.30            # Sol-sağ teker aralığı — ölçülecek (m)
  wheel_radius: 0.05          # Teker yarıçapı — ölçülecek (m)
  robot_width: 0.35           # Toplam araç genişliği — ölçülecek (m)
  robot_length: 0.45          # Toplam araç uzunluğu — ölçülecek (m)
  robot_height: 0.20          # Yerden kameraya — ölçülecek (m)
  ground_clearance: 0.03      # Min. yerden yükseklik — ölçülecek (m)
  max_linear_speed: 0.30      # m/s — başlangıç değeri, kalibre edilecek
  max_angular_speed: 1.0      # rad/s — kalibre edilecek
  mass_kg: 3.0                # Yaklaşık kütle — ölçülecek
```

### 16.2 Tüm Kalibrasyon Gerektiren Parametreler

| Parametre | Dosya | Varsayılan | Kalibrasyon Yöntemi |
|---|---|---|---|
| `wheel_base` | robot_params | 0.30 | Metre ile ölç |
| `wheel_radius` | robot_params | 0.05 | Teker kumpasla ölç |
| `max_safe_slope_deg` | terrain_params | 15.0 | Gerçek rampada test |
| `max_caution_slope_deg` | terrain_params | 25.0 | Gerçek rampada test |
| `max_step_height_m` | terrain_params | 0.04 | Engel tırmanma testi |
| `dropoff_depth_threshold_m` | terrain_params | 0.15 | Gerçek kenar testi |
| `stop_zone_distance_m` | safety_params | 0.25 | Fren mesafesi ölçümü |
| `slowdown_zone_distance_m` | safety_params | 0.55 | Frenleme testi |
| `MAX_PWM` (Arduino) | arduino kodu | 200 | Motor doğrusal test |
| `MIN_PWM` (Arduino) | arduino kodu | 60 | Ölü bant testi |

---

## 17. Tanılama ve İzleme

### 17.1 Diagnostics Yapılandırması

```python
# Her node'da diagnostics updater kullanımı
from diagnostic_updater import Updater, DiagnosticStatusWrapper

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.updater = Updater(self)
        self.updater.setHardwareID('ika_robot')
        self.updater.add('Sensör Durumu', self._check_sensors)

    def _check_sensors(self, stat: DiagnosticStatusWrapper):
        if self.sensor_ok:
            stat.summary(DiagnosticStatusWrapper.OK, 'Normal')
        else:
            stat.summary(DiagnosticStatusWrapper.ERROR, 'Sensör verisi yok')
        return stat
```

### 17.2 RViz2 İzleme Paneli

RViz'de izlenmesi gereken görselleştirmeler:

- `/scan` — LaserScan
- `/oak/points` — PointCloud2
- `/map` — OccupancyGrid
- `/local_costmap/costmap` — OccupancyGrid
- `/global_costmap/costmap` — OccupancyGrid
- `/terrain_obstacles` — OccupancyGrid
- `/collision_poly_stop` — Polygon
- `/collision_poly_approach` — Polygon
- `/safety_status` — String display
- `/odometry/filtered` — Odometry (arrow)
- TF ağacı

### 17.3 rosbag2 Kayıt

```bash
# Test sırasında kritik topic'leri kaydet
ros2 bag record \
  /scan \
  /oak/depth/image_raw \
  /imu/data \
  /gps/fix \
  /odometry/filtered \
  /cmd_vel_safe \
  /terrain_state \
  /safety_status \
  -o ika_test_$(date +%Y%m%d_%H%M%S)
```

---

## 18. Güvenlik Protokolleri

### 18.1 Yazılım Güvenlik Zinciri

```
Nav2 Controller
    ↓ /cmd_vel_nav
Collision Monitor      ← /scan, /oak/points
    ↓ /cmd_vel_collision
Safety Supervisor      ← /terrain_state, sensör watchdog
    ↓ /cmd_vel_safe
Base Controller Node   ← /e_stop
    ↓ Serial
Arduino Watchdog       ← USB timeout
    ↓
Motor Sürücüler
    ↓
Fiziksel E-Stop Anahtarı
```

Her katman bir alt katman başarısız olsa da çalışmaya devam etmelidir.

### 18.2 Acil Durum Kuralları

1. **İlk gerçek araç testleri** daima açık ve engelsiz alanda yapılacak.
2. **Çukur testi** gerçek boşluk ile simülasyon doğrulanmadan yapılmayacak.
3. **Maksimum hız** encoder eklenmeden 0.25 m/s ile sınırlı tutulacak.
4. **Test sırasında** bir kişi her zaman fiziksel E-Stop anahtarına yakın konumda bulunacak.
5. **Rampa testi** önce düşük açılı (5°) rampadan başlanacak, kademeli artırılacak.

### 18.3 Lifecycle Node Yönetimi

Tüm özel node'lar (`terrain`, `safety`, `base_controller`) lifecycle node olarak implemente edilecektir. Bu sayede:

- Konfigürasyon ve aktivasyon ayrı aşamada yapılabilir.
- Hata durumunda node deactivate → cleanup → re-configure döngüsü çalışabilir.
- Tüm sistem `lifecycle_manager` üzerinden yönetilebilir.

```yaml
# nav2_params.yaml içinde lifecycle_manager bölümü
lifecycle_manager:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names:
      - controller_server
      - planner_server
      - behavior_server
      - bt_navigator
      - collision_monitor
      - ika_terrain_perception
      - ika_safety_supervisor
      - ika_base_controller
```

---

## 19. Test Prosedürleri

### 19.1 Birim Testler

Her node için ayrı test dosyası oluşturulacak:

```bash
# Terrain node testi
ros2 run ika_terrain test_terrain_perception

# Safety supervisor testi
ros2 run ika_safety test_safety_supervisor

# Base controller testi (Arduino bağlı değilken mock ile)
ros2 run ika_base_controller test_base_controller_mock
```

### 19.2 Entegrasyon Test Senaryoları

| Senaryo | Test Yöntemi | Başarı Kriteri |
|---|---|---|
| Statik engel önünde durma | Engel koy, hedef ver | Araç <25cm'de durmalı |
| Çukur kenarında durma | Keepout zone → depth kamera | `/terrain_state: DROPOFF_DANGER` |
| Rampa geçişi | %10 eğimli platform | Normal hızda geçmeli |
| Dik engel → rota değişimi | Yolu kapat | Alternatif rota bulunmalı |
| Lidar zaman aşımı | Lidar node öldür | E-Stop, araç durmalı |
| GPS waypoint | 3 noktalı mission | Tüm noktalar ziyaret edilmeli |
| Dar geçit | Robot genişliği - 10cm boşluk | Geçmemeli / uyarı vermeli |

### 19.3 Kalibrasyon Test Sırası

Gerçek araçta şu sırayla kalibrasyon yapılacaktır:

1. **E-Stop testi** — Donanım, watchdog, yazılım E-Stop.
2. **Düz hareket testi** — 1m ileri, dön, geri.
3. **Motor ölü bant** — `MIN_PWM` belirleme.
4. **Hız-PWM lineerlik** — `MAX_PWM` belirleme.
5. **Dönüş testi** — `wheel_base` kalibrasyonu.
6. **Lidar odom doğruluğu** — 5m gidip başa dönüş, konum hatası ölçümü.
7. **IMU kalibrasyonu** — Statik bias, pitch/roll doğruluğu.
8. **Terrain eşikleri** — Rampa ve adım kalibrasyonu.

---

## 20. Bilinen Kısıtlar ve Açık Konular

### 20.1 Encoder Yokluğundan Kaynaklanan Kısıtlar

- Lidar odometrisi kayma, yük değişimi ve yumuşak zeminde bozulabilir.
- Nav2'nin hız geri bildirimi sınırlı; lokal planner tutarlı sonuç üretmeyebilir.
- Uzun görevlerde konum hatası birikir; düzenli relocalization veya GPS korreksiyonu gerekebilir.
- **Öneri:** Teker encoder eklenmesi güçlü biçimde önerilir. QRE1113 veya benzeri optik encoder, mevcut tekerlere nispeten kolay entegre edilebilir.

### 20.2 Açık Geliştirme Konuları

| Konu | Öncelik | Not |
|---|---|---|
| RANSAC tabanlı zemin düzlemi | Yüksek | Mevcut terrain node iskelet |
| IMU + depth kamera füzyonu | Yüksek | Rampa güvenilirliği için |
| Encoder entegrasyonu | Yüksek | Nav2 kalitesini belirleyici |
| Dar geçit kontrolü | Orta | Robot genişliği + güvenlik payı |
| Keepout zone harita editörü | Orta | RViz plugin veya YAML editörü |
| Görev iptal ve kurtarma BT | Orta | Nav2 Behavior Tree |
| micro-ROS geçişi | Düşük | Encoder sonrası değerlendirilebilir |
| Tüm kalibrasyon değerlerinin ölçülmesi | Kritik | Gerçek test öncesi zorunlu |

### 20.3 Simülasyonda Doğrulanması Gereken Davranışlar

Aşağıdaki davranışlar gerçek araca geçişten önce Gazebo'da çalışır hale gelmeli:

- [ ] Lidar odom + SLAM Toolbox harita üretimi
- [ ] Nav2 A'dan B'ye rota bulma ve takip
- [ ] Collision Monitor engelde durma
- [ ] Terrain Node DROPOFF_DANGER → Safety Supervisor durma
- [ ] Terrain Node CAUTION → hız sınırlama
- [ ] Sensör zaman aşımı → E-Stop
- [ ] GPS waypoint sırası tamamlama

---

*Bu doküman İKA projesi geliştirme sürecinde yaşayan bir referans belgedir. Kalibrasyon değerleri, parametre güncellemeleri ve test sonuçları doğrultusunda revize edilecektir.*
