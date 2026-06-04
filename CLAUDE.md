# CLAUDE.md — İKA Projesi Onboarding

> **Bu dosyayı yeni bir Claude oturumu otomatik okur.** Yeni hesapta da
> tüm bağlam buradan + aşağıdaki referans dosyalardan kazanılır.
> Buraya **kalıcı bilgi** yaz, ephemeral notlar `ILERLEME.md`'ye.

---

## Proje Özeti

**İKA**: 4-tekerlek skid-steer otonom kara aracı. ROS 2 Jazzy, Gazebo Harmonic.
Donanım: Raspberry Pi 5 (16 GB) + Arduino Uno + RPLIDAR C1 + Pi Camera (CSI) +
TBD IMU/GPS. **Encoder yok** — odometri lidar tabanlı.

Tez kapsamı: hibrit DL+RANSAC algılama, ayrı füzyon node, Nav2 (DWB↔MPPI)
karşılaştırması, sim doğrulama + sınırlı saha testi.

**Hız sınırı:** 0.25 m/s (encoder eklenene kadar).

---

## İş Akışı

- **Geliştirme:** Windows (`C:\Users\aslan\Desktop\ikasu\`) — kaynak kod, config,
  launch.
- **Build + Run:** Raspberry Pi 5 VEYA WSL2 Ubuntu-24.04 (`~/ika` ayrı klon).
  Pi production hedefi; WSL hızlı sim/test için.
- **Test edilebilir mantık:** ROS'suz saf-Python çekirdeklerde tutulur (pytest
  Windows'ta da koşar). ROS node'ları lifecycle olarak.
- **Parametreler:** Tümü YAML'den (kodda sabit yok).

Path biçimi POSIX (`/dev/ttyACM0`, `~/ika_ws/...`). LF satır sonu.

---

## Paket Haritası (`ika_ws/src/`)

| Paket | Görev |
|---|---|
| `ika_description` | URDF/xacro, RViz config, robot modeli |
| `ika_simulation` | Gazebo world, simülasyon launch, ros_gz bridge |
| `ika_bringup` | Üst-seviye launch'lar (real_robot, sim_full, sensors) |
| `ika_base_controller` | Arduino seri köprü (motor) |
| `ika_navigation` | Nav2 + slam_toolbox + EKF, DWB/MPPI switch |
| `ika_terrain` | Derinlik tabanlı zemin RANSAC, terrain layer |
| `ika_perception_dl` | DL detector (ONNX) + sim_detection_node |
| `ika_fusion` | hazard fusion (terrain + detection → /hazard_state) |
| `ika_safety` | safety_supervisor, sensör watchdog, e-stop |
| `ika_mission` | GPS waypoint mission |
| `ika_rl_planner` | Planner metrik harness (DWB/MPPI/RL karşılaştırma) |

---

## ⚠️ Kritik Bilinen Hatalar / Çözüldü

### 1) xacro `use_sim` arg'i Python bool'a coerce ediliyor

`<xacro:property name="X" value="$(arg X)"/>` ile alınan property, arg değerine
göre Python tipine castlanıyor (`true` → `True` bool). String karşılaştırması
`X == 'true'` daima False döner.

**Düzeltme:** `<xacro:if value="${use_sim}">` (truthy değerlendirme).

**Etki:** 2026-06-02'den önceki tüm Pi sim koşumları büyük olasılıkla
**sensörsüz spawn** olmuştur (URDF'te 0 `<gazebo>` tag'i). Pi'de sim'i pull +
rebuild'den sonra topic akışı yeniden doğrulanmalı.

### 2) WSL2'de Gazebo GUI + RViz + sensör render → COPY MODE bug

WSLg d3d12-mesa shared surface pool taşar; main pencere `[WARN:COPY MODE]`'a
düşer, Windows tarafına aktarılamaz (taskbar'da görünür, tıklayınca öne
gelmez). Çözüm yaklaşımları:

- **Geliştirme/test:** `headless:=true rviz:=true` — tek pencere RViz; Gazebo
  offscreen, sensörler --headless-rendering ile çalışır.
- **Tez screenshot'u:** `headless:=false rviz:=false render_engine:=ogre`
  → tek pencere Gazebo (OGRE1, daha az surface). ✅ Test edildi, çalışıyor.

### 3) `scripts/deploy_sim.sh` `set -u` + ROS setup.bash çatışması

ROS `setup.bash` tanımsız değişken okur (`AMENT_TRACE_SETUP_FILES`).
**Düzeltme:** Sourcing'i `set +u; source ...; set -u` ile sar.

---

## Hızlı Komutlar

### Pi'de build + sim (production)

```bash
cd ~/ika
./scripts/deploy_sim.sh bare       # Gazebo + bridge + RViz
./scripts/deploy_sim.sh full       # + Nav2 + safety + fusion
./scripts/stop_sim.sh
```

### WSL'de geliştirme / test

```bash
# Yeni terminal: PowerShell → wsl -d Ubuntu-24.04
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash

# Geliştirme (tek pencere RViz, Gazebo offscreen):
ros2 launch ika_simulation simulation.launch.py headless:=true rviz:=true render_engine:=ogre2

# Tez screenshot'u (tek pencere Gazebo, OGRE1):
ros2 launch ika_simulation simulation.launch.py headless:=false rviz:=false render_engine:=ogre

# Sürüş testi (ayrı terminal):
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}' -r 10
~/ika/scripts/teleop_safe.sh

# Tam sim (Nav2 dahil):
ros2 launch ika_bringup sim_full.launch.py headless:=true rviz:=true
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header:{frame_id: "map"}, pose:{position:{x: 2.0, y: 0.5}, orientation:{w: 1.0}}}'
```

### Birim testler (Windows'ta da koşar)

```bash
PYTHONPATH=ika_ws/src/ika_terrain:ika_ws/src/ika_perception_dl:ika_ws/src/ika_fusion:ika_ws/src/ika_safety:ika_ws/src/ika_rl_planner \
  pytest ika_ws/src/{ika_terrain,ika_perception_dl,ika_fusion,ika_safety,ika_rl_planner}/test/ \
  --import-mode=importlib
```

Pi'de: `colcon test --packages-select ika_terrain ika_perception_dl ika_fusion ika_safety ika_rl_planner`.

---

## Referans Proje (Plucky baseline port — 2026-06-04)

**slgrobotics/articubot_one** Jazzy fork — Josh Newans (Articulated Robotics)
projesinin aktif geliştirilen Pi5 + Gazebo Harmonic fork'u. Donanım/yazılım
birebir bizimle uyumlu (Pi5 + Arduino + RPLIDAR + Pi Cam + ROS 2 Jazzy + Nav2
+ slam_toolbox + EKF). `~/refs/articubot_one`'a clone'lu (read-only referans).

**plucky** robot variantı config'leri bizim mevcut stack'e cerrahi minimal port
edildi. Port edilen kalibrasyonlar:

| Parametre | Değer | Etki |
|---|---|---|
| Global costmap `track_unknown_space` | `true` | **ABORTED 102 false alarm fix** |
| Controller `controller_frequency` | `5 Hz` | Sim time step uyumu |
| Inflation `radius` / `scaling` | `0.40 / 2.58` | Yumuşak gradient |
| SLAM solver | Ceres explicit (SCHUR_JACOBI + LEVENBERG_MARQUARDT) | Stabil scan match |
| SLAM `transform_publish_period` | `0.02` (50 Hz) | RViz smooth TF |
| BT `bt_loop_duration` | `10` (geri alındı — Plucky'nin 5'i bizde çalışmadı) | — |

**Yedek tag:** `classic-nav2-backup-2026-06-04` (port öncesi son durum).
Port hatası durumunda: `git reset --hard classic-nav2-backup-2026-06-04`.

**Port sonrası benchmark** (sim, 4 waypoint):
- Classic baseline: 0/4 SUCCEEDED, robot 0.23m (takıldı)
- Plucky port: **1/4 SUCCEEDED, robot 7.56m otonom, SLAM 390 hücre**

---

## Mimari Kararlar (özet — detay için `TEZ.md` ve proje belleği)

- **Pi ↔ Arduino:** özel JSON seri protokol (micro-ROS değil ilk fazda).
- **Sim önce, sonra gerçek:** her davranış önce Gazebo'da doğrulanır.
- **Güvenlik zinciri:** Nav2 → Collision Monitor → safety_supervisor →
  Arduino → fiziksel E-Stop.
- **Lifecycle node + saf-Python çekirdek deseni:** ROS'suz çekirdek pytest'le
  test edilir, ROS sarmalı ince.
- **Mesaj kontratı:** `/detected_objects` → `vision_msgs/Detection3DArray`,
  `class_id="label:hazard"` (örn. `person:DYNAMIC`); füzyon parse eder.
- **Safety:** `/terrain_state` yerine fusion'ın `/hazard_state`'ini tüketir.
- **MPPI:** ayrı paket değil; `nav2_params.yaml` + `mppi_controller.yaml`
  overlay, `navigation.launch.py local_planner:=dwb|mppi` arg'ı ile.
- **`ika_rl_planner`:** şimdilik DWB↔MPPI metrik harness; öğrenilmiş PPO/SAC
  sonra.

---

## Tez Engel Taksonomisi (7 Sınıf)

`test_world.sdf`'te her sınıf için somut model var. Tam tablo:
`TEST_PROTOKOLU.md §1`. Özet:

1. **Sabit fiziksel engel** — `obstacle_box_*`, `wall_*`, `thin_pole_1`
2. **Yakın mesafe kritik** — `nav2_collision_monitor` reflex
3. **Negatif engel / çukur** — `dropoff_platform`, `pit_platform_forward`,
   `pothole_visual_*`, `trench_visual_*`
4. **Yükselti / rampa / eşik** — `ramp_safe`, `ramp_caution`, `kerb_step_1`
5. **Dinamik engel** — `person_static_1` + `sim_detection_node`
6. **Dar geçit / kör koridor** — `wall_left+right`, `l_corridor_wall_a/b`
7. **Riskli zemin** — `surface_patch_{safe,caution,risky}` (yol haritası §5.1)

---

## Donanım Notları

- **OAK-D → Pi Camera (CSI) değişimi (2026-06-01):** Gerçek robotta RGB-only
  (derinlik yok). Sim'de hâlâ RGBD modellenir (terrain RANSAC sim testi için).
  Gerçek robotta DL detector + IPM (yer-düzlemi projeksiyonu, z=0).
- Lidar SLAMTEC RPLIDAR C1 — 2B, negatif engel göremez.
- Encoder yok — odometri `rf2o_laser_odometry`.
- **EKF füzyon kaynakları:**
  - odom0: `/odom` (rf2o lidar odom) — x,y,yaw
  - imu0: `/imu/data` (BNO055) — roll/pitch/yaw + angular vel + accel_x
  - odom1: `/odometry/gps` (navsat_transform'dan) — x,y düzeltme (drift telafisi)
- **DL detector (MobileNet-SSD VOC):** person/bicycle/car/bus/motorbike/dog/cat
  /horse/cow/sheep/bird → DYNAMIC; chair/sofa/diningtable/pottedplant/bottle
  /tvmonitor → STATIC. `sim_detection_node` sim'de ground-truth tabanlı
  sentetik tespit üretir.
- **3D Harita (octomap_server):** `/oak/points` → 3D occupancy octree;
  /octomap_full topic'inde yayınlanır.

## Otonom Sürüş Modları (`sim_full.launch.py autonomous_mode:=...`)

| Mod | Açıklama | Node | /cmd_vel_nav kaynak |
|---|---|---|---|
| `avoider` (default) | Bug-tarzı reaktif: ileri sür, engelde dön, 2m sonra dur | `ika_mission/obstacle_avoider` | reaktif |
| `nav2` | Klasik goal-based Nav2 + DWB/MPPI | `controller_server` | planlanmış |
| `off` | Sadece perception, manuel teleop | — | yok |

Sim ve gerçek robotta aynı arg. Avoider hem WSL sim'de hem Pi'de test edildi.

---

## Çalışma Akışı (yeni Claude için sıra)

1. **Bu dosyayı oku** (otomatik).
2. **`ILERLEME.md`'yi oku** — şu an aktif iş + sıradakiler.
3. **İhtiyaç olursa derinleştir:**
   - `TEST_PROTOKOLU.md` — senaryo + saha test detayları
   - `TEZ.md` — tez içerik şablonu
   - `IKA_ROS2_System_Reference.md` — sistem referansı
   - `KURULUM.md` — Pi kurulumu
   - `DENEMELER.md` — adım adım deneme rehberi
4. **Çalışma sonunda `ILERLEME.md`'yi güncelle** — yapılanları işaretle,
   sıradakileri ekle, durum bilgisi yaz.
5. **Mimari karar veya kritik bug çözdüysen:** önce `CLAUDE.md`'ye ekle
   (kalıcı), sonra `ILERLEME.md`'ye not düş.

---

## Repo Yapısı

```
ikasu/
├── CLAUDE.md                     ← bu dosya, yeni Claude onboarding
├── ILERLEME.md                   ← canlı durum/yapılacak listesi
├── README.md                     ← klasik proje README
├── KURULUM.md                    ← Pi kurulum scripti rehberi
├── DENEMELER.md                  ← adım adım deneme rehberi
├── TEST_PROTOKOLU.md             ← tez doğrulama protokolü
├── TEZ.md                        ← tez içerik şablonu
├── IKA_ROS2_System_Reference.md  ← sistem referansı
├── KEEPOUT.md                    ← keepout zone notları
├── LICENSE
├── ika_ws/                       ← ROS 2 workspace
│   └── src/
│       └── ika_*/                ← 11 paket (yukarıdaki harita)
└── scripts/
    ├── install_pi.sh             ← Pi sıfırdan kurulum
    ├── deploy_sim.sh             ← build + sim launch
    ├── stop_sim.sh
    ├── verify_sim.sh             ← topic akışı doğrulama
    ├── check_workspace.sh
    ├── teleop_raw.sh
    └── teleop_safe.sh
```
