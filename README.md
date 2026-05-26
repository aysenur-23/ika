# İKA — ROS 2 Jazzy Tabanlı Otonom Kara Aracı

> **4 tekerlekli skid-steer otonom kara aracı.** Raspberry Pi 5 (16 GB) + Arduino Uno
> + RPLIDAR C1 + OAK-D Lite + IMU + GPS donanımı; ROS 2 Jazzy yazılım yığını
> üzerinde SLAM tabanlı haritalama, Nav2 ile otonom navigasyon, depth tabanlı
> terrain analizi (RANSAC) ve katmanlı güvenlik mimarisi sunar.

[![Apache 2.0](https://img.shields.io/badge/license-Apache_2.0-blue.svg)](LICENSE)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange.svg)](https://ubuntu.com/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-yellow.svg)](https://gazebosim.org/)

---

## İçindekiler

1. [Proje Özeti](#1-proje-özeti)
2. [Donanım](#2-donanım)
3. [Yazılım Mimarisi](#3-yazılım-mimarisi)
4. [Repo Yapısı](#4-repo-yapısı)
5. [Hızlı Başlangıç](#5-hızlı-başlangıç)
6. [Detaylı Kurulum (sıfırdan Pi)](#6-detaylı-kurulum)
7. [Simülasyonu Çalıştırma](#7-simülasyonu-çalıştırma)
8. [Gerçek Araçta Çalıştırma](#8-gerçek-araçta-çalıştırma)
9. [Test Senaryoları](#9-test-senaryoları)
10. [Geliştirici Komutları](#10-geliştirici-komutları)
11. [Sorun Giderme](#11-sorun-giderme)
12. [Daha Fazla Doküman](#12-daha-fazla-doküman)
13. [Lisans + Katkı](#13-lisans-katkı)

---

## 1. Proje Özeti

**Hedef:** İç ve dış ortamda otonom hareket edebilen, GPS waypoint takibi yapabilen,
engelden kaçınan, çukur/rampa algılayabilen ve fiziksel + yazılım katmanlı güvenliğe
sahip bir kara aracı.

**Tasarım felsefesi:**

- **Önce simülasyon:** Her davranış Gazebo Harmonic'te doğrulanmadan gerçek araca taşınmaz.
- **Parametrik:** Araç boyutları, hız sınırları, eşikler — hiçbir şey kodda hard-coded değil, hepsi YAML.
- **Modüler:** Her işlevsel birim ayrı ROS 2 paketi + lifecycle node.
- **Encoder yok:** Maliyet/karmaşıklık nedeniyle teker encoder'ları kullanılmıyor; odometri lidar tabanlı (`rf2o_laser_odometry`). Maksimum hız 0.25 m/s ile sınırlı.

**Temel yetenekler:**

| Yetenek | Durum |
|---|---|
| SLAM tabanlı haritalama | ✅ slam_toolbox |
| Otonom navigasyon | ✅ Nav2 |
| Engelden kaçınma | ✅ Collision Monitor + Voxel/Obstacle Layer |
| Çukur/düşme algılama | ✅ RANSAC zemin uydurma |
| Rampa sınıflandırması | ✅ SAFE / CAUTION / IMPASSABLE |
| GPS waypoint görevi | ✅ pause / resume / cancel / skip / restart |
| Güvenli duruş (failsafe) | ✅ Safety Supervisor + watchdog |
| Keepout zone uyumu | ✅ Config hazır (KEEPOUT.md ile aktive edilir) |
| Tanılama (diagnostics) | ✅ `/diagnostics` topic + rqt_robot_monitor |

---

## 2. Donanım

| Bileşen | Model | Arayüz |
|---|---|---|
| Ana bilgisayar | Raspberry Pi 5, 16 GB RAM | — |
| İşletim sistemi | Ubuntu Server 24.04.4 LTS, 64-bit | — |
| 2D Lidar | SLAMTEC RPLIDAR C1 (0.15-12m) | USB Serial |
| Depth kamera | Luxonis OAK-D Lite | USB 3.0 |
| IMU | TBD (BNO055 / MPU-9250 önerilen) | I²C / SPI |
| GPS | TBD (UBLOX NEO-M8N önerilen) | USB / UART |
| Mikrodenetleyici | Arduino Uno | USB Serial |
| Motor sürücüler | L298N veya eşdeğer | Arduino GPIO |
| Motorlar | 4× DC motor (skid-steer) | Sürücü |
| **Encoder** | **Yok** | — |
| E-Stop | Fiziksel anahtar | Motor güç hattı |

**Skid-steer kinematik:**

```
       Sol grup (sol ön + sol arka motorlar)
         ┃
         ▼
    v_left  = linear.x - (angular.z × wheel_base / 2)
    v_right = linear.x + (angular.z × wheel_base / 2)
         ▲
         ┃
       Sağ grup (sağ ön + sağ arka motorlar)
```

---

## 3. Yazılım Mimarisi

### 3.1 Sistem Veri Akışı

```
                  ┌──────────────────────────┐
                  │      Görev Yöneticisi     │
                  │  /goal_pose veya GPS WP   │
                  └────────────┬─────────────┘
                               │
                               ▼
                     ┌───────────────────┐
                     │       Nav2        │
                     │  Planner + DWB    │
                     │  BT Navigator     │
                     └────────┬──────────┘
                              │ /cmd_vel_nav
                              ▼
                  ┌────────────────────────┐
                  │   Collision Monitor    │  ← /scan, /oak/points
                  └────────────┬───────────┘
                               │ /cmd_vel_collision
                               ▼
                  ┌────────────────────────┐
                  │   Safety Supervisor    │  ← /terrain_state, sensör watchdog
                  └────────────┬───────────┘
                               │ /cmd_vel_safe
                               ▼
                  ┌────────────────────────┐
                  │  Base Controller       │
                  │  (Pi-Arduino seri)     │
                  └────────────┬───────────┘
                               │ JSON over USB
                               ▼
                            Arduino
                               │
                               ▼
                            Motorlar
```

**Sensör akışları:**

```
RPLIDAR C1   →  /scan              →  SLAM / Costmap / Collision Monitor
OAK-D Lite   →  /oak/depth/image_raw   →  Terrain Perception (RANSAC)
             →  /oak/points        →  Voxel Layer
IMU          →  /imu/data          →  EKF / Terrain
GPS          →  /gps/fix           →  navsat_transform / Mission
rf2o lidar   →  /odom              →  EKF
EKF          →  /odometry/filtered →  Nav2 + TF(odom→base_link)
SLAM         →  /map               →  Global Costmap
Terrain      →  /terrain_obstacles →  Global Costmap (static layer)
             →  /terrain_state     →  Safety Supervisor
Safety       →  /safety_status     →  Diagnostics, RViz
             →  /e_stop            →  Arduino
```

### 3.2 TF (Transform) Ağacı

```
map
└── odom                    (EKF yayar)
    └── base_link
        ├── base_footprint
        ├── laser_frame      (RPLIDAR C1)
        ├── camera_frame     (OAK-D Lite)
        │   └── camera_depth_optical_frame
        ├── imu_frame
        └── gps_frame
```

Tüm statik TF'ler `robot_state_publisher` ile URDF/Xacro üzerinden yayımlanır.
Dinamik `odom → base_link` TF'i `robot_localization` EKF'i yayar.

### 3.3 Güvenlik Zinciri

```
Nav2 Controller
    ↓ /cmd_vel_nav
Collision Monitor      ← /scan, /oak/points  (Nav2 dahili)
    ↓ /cmd_vel_collision
Safety Supervisor      ← /terrain_state, sensör watchdog  (IKA özel)
    ↓ /cmd_vel_safe
Base Controller Node   ← /e_stop
    ↓ Serial JSON
Arduino Watchdog       ← USB timeout (500ms)
    ↓ PWM
Motor Sürücüleri
    ↓
Fiziksel E-Stop Anahtarı (güç kesme)
```

Her katman üst katman çökse de bağımsız çalışmalıdır.

---

## 4. Repo Yapısı

```
ika/
├── README.md                       # Bu dosya
├── KURULUM.md                      # Detaylı kurulum rehberi
├── DENEMELER.md                    # 12 test senaryosu
├── KEEPOUT.md                      # Keepout zone aktivasyonu
├── IKA_ROS2_System_Reference.md    # Sistem referans dokümanı (Türkçe)
├── TEZ.md                          # Tez yazımı için içerik
├── LICENSE                         # Apache-2.0
├── .gitattributes                  # LF zorunlu (cross-platform)
├── .gitignore
├── ika_ws/
│   └── src/
│       ├── ika_bringup/            # Üst düzey launch + RViz config
│       ├── ika_description/        # URDF/Xacro + Gazebo plugin'leri
│       ├── ika_navigation/         # Nav2 + SLAM + EKF + rf2o configleri
│       ├── ika_terrain/            # RANSAC tabanlı terrain perception
│       ├── ika_safety/             # Safety Supervisor (watchdog + filtre)
│       ├── ika_base_controller/    # Pi-Arduino seri köprü + .ino
│       ├── ika_mission/            # GPS waypoint görev yöneticisi
│       └── ika_simulation/         # Gazebo Harmonic worlds + bridge
└── scripts/
    ├── install_pi.sh               # Pi'yi sıfırdan kurar (7 faz)
    ├── deploy_sim.sh               # Build + sim launch + verify
    ├── stop_sim.sh                 # Temiz kapatma
    ├── verify_sim.sh               # Topic/TF akış denetimi
    ├── check_workspace.sh          # Pre-push lint + test
    ├── teleop_safe.sh              # Güvenlik zincirinden geçen manuel
    └── teleop_raw.sh               # Sim'de direkt Gazebo'ya manuel
```

8 ROS 2 paketi:

| Paket | Tip | İçerik |
|---|---|---|
| `ika_description` | ament_cmake | URDF/Xacro, mesh dizini, Gazebo plugin'leri |
| `ika_simulation` | ament_cmake | test_world.sdf, ros_gz_bridge config, simulation.launch.py |
| `ika_navigation` | ament_cmake | nav2/slam/ekf/rf2o yaml + slam.launch.py + navigation.launch.py |
| `ika_bringup` | ament_cmake | real_robot.launch.py, sim_full.launch.py, RViz |
| `ika_base_controller` | ament_python | base_controller_node + Arduino .ino |
| `ika_terrain` | ament_python | terrain_perception_node + ground_plane.py (RANSAC) |
| `ika_safety` | ament_python | safety_supervisor_node |
| `ika_mission` | ament_python | gps_waypoint_mission |

---

## 5. Hızlı Başlangıç

> Bu bölüm **bilgisi olanlar için kısa yol.** İlk kez kuruyorsan [Bölüm 6](#6-detaylı-kurulum)'a geç.

```bash
# Pi'de (Ubuntu 24.04 fresh):
sudo apt update && sudo apt install -y git
git clone https://github.com/aysenur-23/ika.git ~/ika
cd ~/ika
chmod +x scripts/*.sh

# Sıfırdan kur (~30-40 dk)
./scripts/install_pi.sh

# Grupları aktive et
exit && ssh ubuntu@<PI_IP>    # veya yeni terminal aç
cd ~/ika

# İlk sim denemesi
./scripts/deploy_sim.sh

# Tam stack sim (Nav2 + safety + terrain)
./scripts/deploy_sim.sh full

# Durdur
./scripts/stop_sim.sh
```

---

## 6. Detaylı Kurulum

### 6.1 Ön Koşullar

- Raspberry Pi 5 (16 GB RAM önerilir)
- Boş **32 GB+** SD/NVMe disk (NVMe HAT öneririm — SD yavaş)
- Ubuntu Server 24.04.4 LTS (64-bit) **yeni kurulmuş**
- İnternet bağlantısı (Ethernet veya WiFi)
- Bir başka bilgisayardan SSH erişimi (opsiyonel ama önerilir)

### 6.2 İlk açılış (Pi'de)

İlk girişte `ubuntu` kullanıcı + `ubuntu` parola (parolayı değiştirmeni ister).

**WiFi (gerekirse):**
```bash
sudo nmcli device wifi connect "WIFI_ADI" password "PAROLA"
ip -4 addr show | grep inet     # IP'yi öğren
```

**SSH (önerilir):**
```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
```

Artık `ssh ubuntu@<PI_IP>` ile uzaktan bağlanabilirsin.

**Saat dilimi:**
```bash
sudo timedatectl set-timezone Europe/Istanbul
```

**Locale (ROS 2 için kritik):**
```bash
sudo apt install -y locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 6.3 Repo'yu klonla

```bash
sudo apt install -y git
cd ~
git clone https://github.com/aysenur-23/ika.git
cd ika
ls          # README.md, KURULUM.md, ika_ws/, scripts/, ...
```

### 6.4 Otomatik kurulum

```bash
chmod +x scripts/*.sh
./scripts/install_pi.sh
```

Bu script 7 fazda her şeyi sırayla kurar:

| Faz | Ne yapar |
|---|---|
| 1. system | apt update, temel araçlar (git, curl, vim, htop), locale, kullanıcı grupları (dialout, plugdev, video) |
| 2. ros | ROS 2 Jazzy apt anahtarı + deposu, `ros-jazzy-desktop`, `ros-dev-tools` |
| 3. packages | Nav2, SLAM, robot_localization, rf2o, Gazebo Harmonic, xacro, teleop, diagnostics, rqt. Apt'te olmayan paketler `ika_ws/src/third_party/` altına git clone'lanır |
| 4. python | python3-serial, python3-yaml, python3-numpy, python3-pytest |
| 5. udev | `/etc/udev/rules.d/99-ika-usb.rules` — lidar, arduino, GPS için sembolik link |
| 6. build | `colcon build --symlink-install --parallel-workers $(nproc)` |
| 7. verify | `ros2 doctor`, paket listesi, birim testler |

**İlk koşumda sudo parolası ister**, sonrasında 60s'de bir refresh eder — beklemen gerekmez.

**Tek faz çalıştırma:**
```bash
./scripts/install_pi.sh system     # sadece sistem
./scripts/install_pi.sh ros        # sadece ROS 2
./scripts/install_pi.sh packages   # sadece ROS paketleri
./scripts/install_pi.sh build      # sadece workspace build
./scripts/install_pi.sh verify     # sadece doğrulama
```

Script idempotent — kurulu paketleri atlar, sorunsuz tekrar çalışır.

### 6.5 Grupları aktive et (ÇOK ÖNEMLİ)

`install_pi.sh` seni `dialout`, `plugdev`, `video` gruplarına ekler ama **mevcut oturumda aktif olmazlar.** Logout + login gerek:

```bash
exit                          # SSH oturumunu kapat
ssh ubuntu@<PI_IP>            # tekrar bağlan

groups                        # 'dialout' görmelisin
```

### 6.6 ~/.bashrc kontrolü

`install_pi.sh` aşağıdaki iki satırı `~/.bashrc`'a ekledi — kontrol et:
```bash
grep -E "ros/jazzy|ika_ws/install" ~/.bashrc
```

Beklenen çıktı:
```
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ika/ika_ws/install/setup.bash
```

Yoksa elle ekle:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source $HOME/ika/ika_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6.7 Donanım hazırlığı (USB cihazlar takılıysa)

Lidar, Arduino, GPS taktıysan udev kuralları sembolik link oluşturur:
```bash
ls -l /dev/ika_*
# Beklenen:
# /dev/ika_lidar   -> ttyUSB0
# /dev/ika_arduino -> ttyACM0
# /dev/ika_gps     -> ttyUSB1
```

Görünmüyorsa **VID:PID'ler farklı olabilir.** `lsusb` ile öğren, `/etc/udev/rules.d/99-ika-usb.rules` dosyasını güncelle, `sudo udevadm control --reload-rules && sudo udevadm trigger`.

### 6.8 Doğrulama

```bash
./scripts/install_pi.sh verify
```

Beklenen:
- `[OK] ros2 cli`
- `[OK] gz cli (8.x.x)` ← Gazebo Harmonic
- `[OK] ros2 pkg: ika_bringup` (×8)
- Birim testler 22/22

Kırmızı `[!!]` çıkarsa [Bölüm 11](#11-sorun-giderme) → Sorun Giderme.

---

## 7. Simülasyonu Çalıştırma

### 7.1 Bare sim (sadece Gazebo + URDF)

```bash
cd ~/ika
./scripts/deploy_sim.sh
```

Bu komut paralel olarak:
1. `colcon build` (gerekiyorsa) — ~1 dk
2. Gazebo Harmonic'i `test_world.sdf` ile arka planda başlatır
3. URDF'i robot olarak spawn eder
4. ros_gz_bridge ile topic köprüsü kurar
5. RViz açar
6. 15s sonra `verify_sim.sh` ile akışı kontrol eder
7. Sana özet verir, prompt'a döner

**Pi'nin fiziksel ekranı varsa** Gazebo penceresi açılır.
**SSH'tan geliyorsan** Gazebo GUI gelmez (X forwarding gerek veya VNC kur).

Hızlı doğrulamalar:
```bash
ros2 topic hz /scan         # ~10 Hz
ros2 topic hz /imu/data     # ~100 Hz
ros2 topic hz /clock        # >100 Hz
ros2 topic list             # 15+ topic
```

### 7.2 Tam stack sim (Nav2 + safety + terrain)

```bash
./scripts/deploy_sim.sh full
```

Buna ek olarak Nav2, SLAM Toolbox (mapping mode), Safety Supervisor, Terrain Perception, navsat_transform de başlar.

RViz'de:
1. **Fixed Frame:** `map`
2. **2D Nav Goal** aracını seç
3. Haritada bir noktayı tıkla + sürükle → robot oraya gider

### 7.3 SLAM modu (sadece haritalama)

```bash
ros2 launch ika_navigation slam.launch.py use_sim_time:=true
```

Aracı `./scripts/teleop_raw.sh` ile sür, RViz'de haritanın oluştuğunu izle. Sonra kaydet:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ika/ika_ws/src/ika_navigation/maps/test_map
```

### 7.4 Manuel sürüş

```bash
# Güvenlik zincirinden geçer (sim_full + Nav2 ile birlikte güvenli):
./scripts/teleop_safe.sh

# Sim'de direkt Gazebo'ya (sadece bare sim):
./scripts/teleop_raw.sh
```

Klavye tuşları:
- `i / k / ,` — ileri / dur / geri
- `j / l` — sol / sağ
- `u / o / m / .` — diyagonal
- `q / z` — hız artır / azalt

### 7.5 Durdurma

```bash
./scripts/stop_sim.sh
```

Process group + yetim `gz`/`rviz` süreçlerini temizler. ROS daemon'u da yeniler.

---

## 8. Gerçek Araçta Çalıştırma

> ⚠️ **Önce simülasyonda tüm test senaryoları geçmeli.** Gerçek araçta encoder yokluğu nedeniyle maksimum hız 0.25 m/s ile sınırlandırılmıştır.

### 8.1 Arduino kurulumu

Arduino IDE veya `arduino-cli` ile motor kontrolcüsünü yükle:
```bash
sudo apt install -y arduino-cli
arduino-cli core install arduino:avr
arduino-cli lib install "ArduinoJson"
cd ~/ika/ika_ws/src/ika_base_controller/arduino
arduino-cli compile --fqbn arduino:avr:uno ika_motor_controller
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ika_arduino ika_motor_controller
```

### 8.2 Donanım bağlantı sırası

1. **E-Stop anahtarını test et** — motor sürücülerinin güç hattını fiziksel olarak kesmeli.
2. Arduino + motor sürücü + motorları bağla (sol/sağ grup ayrımı önemli — Doc §2.2).
3. Lidar, kamera, IMU, GPS, Arduino'yu Pi'ye USB ile tak.
4. `ls -l /dev/ika_*` ile sembolik link kontrolü.

### 8.3 İlk hareket testi (düşük hız, açık alan)

```bash
ros2 launch ika_bringup real_robot.launch.py
```

Başka terminalde manuel kontrol:
```bash
./scripts/teleop_safe.sh
```

İlk denemede:
- `q` ile hızı en düşüğe çek
- `i` ile ileri çok kısa bir hareket
- E-Stop tuşuna bas — motorlar duruyor mu?
- Engele yaklaş — duruyor mu?

### 8.4 Kalibrasyon

`config/robot_params.yaml` ve `Arduino .ino` dosyasındaki şu değerleri gerçek araç ölçümleriyle güncelle:

| Parametre | Yer | Yöntem |
|---|---|---|
| `wheel_base` | robot_params | Metre ile sol-sağ teker arası |
| `wheel_radius` | robot_params | Kumpasla teker yarıçapı |
| `MIN_PWM` | Arduino .ino | Motor dönmeye başladığı en düşük PWM |
| `MAX_PWM` | Arduino .ino | Sürücünün güvenli üst sınırı |
| `MAX_SPEED_MPS` | Arduino .ino | Maksimum lineer hız ölçümü |
| `max_safe_slope_deg` | terrain_params | Test rampasında deneme |
| `max_caution_slope_deg` | terrain_params | Test rampasında deneme |
| `max_step_height_m` | terrain_params | Engel tırmanma testi |

Detay: [IKA_ROS2_System_Reference.md](IKA_ROS2_System_Reference.md) §16.

---

## 9. Test Senaryoları

12 deneme — detayı [DENEMELER.md](DENEMELER.md):

| # | Senaryo | Süre |
|---|---|---|
| 0 | Build doğrulama | 2 dk |
| 1 | URDF görüntü (display.launch.py) | 1 dk |
| 2 | Bare simülasyon | 2 dk |
| 3 | Topic akışı (verify_sim.sh) | 1 dk |
| 4 | Manuel sürüş | 3 dk |
| 5 | Lidar odom + EKF + SLAM | 10 dk |
| 6 | Tam stack sim | 5 dk |
| 7 | Navigasyon hedefi | 5 dk |
| 8 | Terrain testleri (rampa, dropoff) | 10 dk |
| 9 | Safety zinciri + E-Stop | 5 dk |
| 10 | GPS waypoint mission | 10 dk |
| 11 | Diagnostics izleme | 3 dk |
| 12 | Keepout zone (opsiyonel) | 15 dk |

Her deneme:
- **Komut** — kopyalanabilir bash komutu
- **Beklenen** — başarılı çıktının ne olması gerektiği
- **Başarısızlık** — sorun çıkarsa nereye bakacağın

Sırayla yap — bir öncekinin sonucu sonrakini test edebilmen için gerekli.

---

## 10. Geliştirici Komutları

### Workspace build

```bash
cd ~/ika/ika_ws
colcon build --symlink-install                         # tüm paketler
colcon build --packages-select ika_terrain             # tek paket
colcon build --symlink-install --parallel-workers 4    # CPU sınırla
```

`--symlink-install` flag'i Python kaynak değişikliklerini build olmadan yansıtır.

### Birim testler (Pi'siz, sadece Python + numpy)

```bash
cd ~/ika/ika_ws/src/ika_terrain && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_safety && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_base_controller && python3 -m pytest test/ -v
# Toplam: 22 test
```

### Pre-push doğrulama

```bash
cd ~/ika
./scripts/check_workspace.sh
# Python syntax + YAML + XML + shell + unit testler
```

### Tek katman test

```bash
ros2 launch ika_description display.launch.py            # sadece URDF görüntü
ros2 launch ika_simulation simulation.launch.py          # bare Gazebo
ros2 launch ika_navigation slam.launch.py                # rf2o+EKF+SLAM mapping
ros2 launch ika_navigation navigation.launch.py slam_mode:=localization  # mevcut harita üstü
ros2 launch ika_bringup sim_full.launch.py               # tam sim stack
ros2 launch ika_bringup real_robot.launch.py             # gerçek araç
ros2 launch ika_bringup sensors.launch.py                # sadece sensör sürücüler
```

### Mission yönetimi

```bash
# Görev başlat
ros2 run ika_mission gps_waypoint_mission \
  --waypoints ~/ika/ika_ws/src/ika_mission/missions/test_mission.yaml \
  --frame map

# Dış komutlar
ros2 topic pub --once /mission_cmd std_msgs/String "data: pause"
ros2 topic pub --once /mission_cmd std_msgs/String "data: resume"
ros2 topic pub --once /mission_cmd std_msgs/String "data: skip"
ros2 topic pub --once /mission_cmd std_msgs/String "data: cancel"
ros2 topic pub --once /mission_cmd std_msgs/String "data: restart"

# Durumu izle
ros2 topic echo /mission_state
```

### Tanılama izleme

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor    # GUI
ros2 topic echo /diagnostics --once             # CLI
ros2 run rqt_graph rqt_graph                    # node bağlantı grafiği
```

### Rosbag kayıt

```bash
ros2 bag record \
  /scan /oak/depth/image_raw /imu/data /gps/fix \
  /odometry/filtered /cmd_vel_safe \
  /terrain_state /safety_status \
  -o ~/ika_test_$(date +%Y%m%d_%H%M%S)
```

### Kod güncellemesi geldiğinde

```bash
cd ~/ika
git pull
./scripts/deploy_sim.sh bare clean       # clean = build temizle + rebuild
```

---

## 11. Sorun Giderme

### Build hataları

| Belirti | Çözüm |
|---|---|
| `Package 'X' not found` | `cd ~/ika/ika_ws && rosdep install --from-paths src --ignore-src -r -y` |
| `Failed to find ament_python` | `sudo apt install ros-jazzy-ament-cmake-python` |
| Disk dolu (`No space left`) | `df -h`; `rm -rf ~/ika/ika_ws/build install log` |
| Build çok yavaş | SD kart yavaş; NVMe HAT öneri |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |

### Runtime hataları

| Belirti | Çözüm |
|---|---|
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash`; `~/.bashrc`'da kontrol et |
| `Permission denied: /dev/ttyACM0` | `groups` → dialout yok mu; `exit + ssh` (grupları yenile) |
| `gz: command not found` | `sudo apt install ros-jazzy-ros-gz` |
| Gazebo siyah ekran / segfault | `./scripts/deploy_sim.sh` headless dene; OGRE2 yerine ogre1: `ros2 launch ika_simulation simulation.launch.py headless:=true` |
| Sim arka planda kaldı | `./scripts/stop_sim.sh && ros2 daemon stop && ros2 daemon start` |
| `/scan` 0 Hz | Bridge çalışmıyor: `ros2 node list \| grep bridge`; gz tarafında: `gz topic -l \| grep scan` |
| `/terrain_state: UNKNOWN` hep | Depth verisi gelmiyor: `ros2 topic hz /oak/depth/image_raw` 0 ise bridge sorunu |
| Robot Nav2 hedefi reddediyor | Hedef harita dışında veya engel içinde; RViz'de costmap göster |
| Nav2 plan oluşturuyor ama robot durmuyor | Collision Monitor stop zone'da engel; polygon'u RViz'de etkinleştir |
| `lifecycle_manager` TIMEOUT | `ros2 lifecycle nodes`; `ros2 lifecycle get /<node>` |

### Donanım

| Belirti | Çözüm |
|---|---|
| Pi aşırı ısınıyor / throttle | `vcgencmd measure_temp` (80°C+ kritik), active cooler tak; SDK 4 fan da olmalı |
| Robot hareket etmiyor (real) | Arduino bağlı mı: `ls -l /dev/ika_arduino`; USB log: `minicom -D /dev/ika_arduino` ile JSON görmeli |
| Robot bir yöne kayıyor | Sol/sağ motor grupları doğru bağlı mı; `wheel_base` kalibre değil |
| RViz model boş | `Fixed Frame` `base_link` veya `map` olmalı |

### Network / DDS

| Belirti | Çözüm |
|---|---|
| Topic'ler diğer makineden görünmüyor | `ROS_DOMAIN_ID` aynı mı; multicast firewall'u açık mı |
| ROS daemon takılı | `ros2 daemon stop && ros2 daemon start` |
| Birden fazla makine, topic çakışıyor | `export ROS_DOMAIN_ID=42` (0-101 arası) — herkes aynı ID'de |

### Genel kurtarma — sıfırdan başla

```bash
cd ~/ika/ika_ws
rm -rf build install log
./scripts/install_pi.sh build           # rebuild
source install/setup.bash
./scripts/stop_sim.sh
ros2 daemon stop
./scripts/deploy_sim.sh
```

Daha fazlası: [KURULUM.md](KURULUM.md) §10, [DENEMELER.md](DENEMELER.md) Genel Sorun Giderme.

---

## 12. Daha Fazla Doküman

| Dosya | İçerik |
|---|---|
| [KURULUM.md](KURULUM.md) | Manuel adım adım kurulum (script kullanmadan) + 10 bölüm sorun giderme |
| [DENEMELER.md](DENEMELER.md) | 12 test senaryosu, her birinde komut/beklenen/başarısızlık |
| [KEEPOUT.md](KEEPOUT.md) | Keepout zone maskesi oluşturma + Nav2 entegrasyonu |
| [IKA_ROS2_System_Reference.md](IKA_ROS2_System_Reference.md) | Sistem referansı — geliştirici düzeyinde mimari, kod örnekleri, parametre referansı |
| [TEZ.md](TEZ.md) | Tez yazımı için bölüm bölüm içerik |

---

## 13. Lisans + Katkı

Apache-2.0 — bkz. [LICENSE](LICENSE).

**Katkı:**

```bash
git checkout -b feature/yeni-ozellik
# değişiklikleri yap
./scripts/check_workspace.sh     # pre-push kontrol
git add . && git commit -m "feat: yeni özellik"
git push origin feature/yeni-ozellik
# GitHub'da Pull Request aç
```

Tüm testler yeşil olmadan PR merge edilmemeli.

**Commit konvansiyonu:**

```
feat: yeni özellik
fix: bug düzeltme
docs: dokümantasyon
refactor: kod yeniden yapılandırma
test: test ekleme/güncelleme
chore: build/config
```

---

## Son Not

Sorularını [GitHub Issues](https://github.com/aysenur-23/ika/issues) üzerinden aç.
Hata raporlarında lütfen şunları ekle:

1. Hangi denemede (`./scripts/deploy_sim.sh` mi, `ros2 launch ...` mi)?
2. Tam komut + son 50 satır log (`/tmp/ika_sim.log` veya `/tmp/ika_build.log`)
3. `ros2 doctor --report` çıktısı
4. Donanım (sim mi gerçek araç mı)

Başarılar.
