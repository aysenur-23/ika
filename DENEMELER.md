# İKA — Denemeler Rehberi

Bu doküman İKA'yı **adım adım test etmek** için tasarlanmış bir senaryo
serisidir. Her deneme bir öncekinin sonucunu varsayar; sırayla ilerlemek
sorunları erken yakalamana ve nereye baktığını bilmene yardımcı olur.

**Önce [KURULUM.md](KURULUM.md)'yi tamamla.** Bu rehber kurulum bittikten
sonra başlar.

## İçindekiler

- [Çalışma kuralları](#çalışma-kuralları)
- [Deneme 0 — Build doğrulama](#deneme-0--build-doğrulama)
- [Deneme 1 — URDF görüntü](#deneme-1--urdf-görüntü-display)
- [Deneme 2 — Bare simülasyon (Gazebo + URDF)](#deneme-2--bare-simülasyon)
- [Deneme 3 — Topic akışı (verify_sim.sh)](#deneme-3--topic-akışı)
- [Deneme 4 — Manuel sürüş (teleop_raw)](#deneme-4--manuel-sürüş)
- [Deneme 5 — Lidar odom + EKF + SLAM](#deneme-5--lidar-odom--ekf--slam)
- [Deneme 6 — Tam stack (sim_full)](#deneme-6--tam-stack)
- [Deneme 7 — Navigasyon hedefi (Nav2)](#deneme-7--navigasyon-hedefi)
- [Deneme 8 — Terrain testleri (rampa, dropoff)](#deneme-8--terrain-testleri)
- [Deneme 9 — Safety zinciri ve E-Stop](#deneme-9--safety-zinciri-ve-e-stop)
- [Deneme 10 — Mission yöneticisi (GPS waypoint)](#deneme-10--mission-yöneticisi)
- [Deneme 11 — Diagnostics izleme](#deneme-11--diagnostics-i̇zleme)
- [Deneme 12 — Keepout zone (opsiyonel)](#deneme-12--keepout-zone-opsiyonel)
- [Genel sorun giderme](#genel-sorun-giderme)

---

## Çalışma Kuralları

1. Her deneme **terminal başlığını** belirtir: `Terminal A`, `Terminal B`...
2. Her deneme **beklenen** sonucu listeler. Görmedigin şeye atlama.
3. **Başarısızlık** bölümü her denemenin sonunda. Önce orayı oku.
4. **Sim'i temiz başlat:** her deneme öncesi `./scripts/stop_sim.sh`.
5. Logları kaydet: `/tmp/ika_sim.log` her sim koşusunun çıktısı.
6. **Pi GUI:** Gazebo ve RViz Pi'nin fiziksel ekranını veya VNC kullanır.

Standart başlangıç (her yeni terminalde):
```bash
cd ~/ika
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
```

`~/.bashrc` doğru kurulmuşsa son iki satır otomatik koşar.

---

## Deneme 0 — Build Doğrulama

**Amaç:** Workspace derlendi, paketler ROS ortamına kayıtlı, birim testler yeşil.

### Komut
```bash
cd ~/ika/ika_ws
colcon build --symlink-install --parallel-workers $(nproc)
source install/setup.bash
ros2 pkg list | grep ika_
```

### Beklenen
- `colcon build` hata vermeden tamamlanır.
- `ros2 pkg list | grep ika_` çıktısında 8 paket:
  ```
  ika_base_controller
  ika_bringup
  ika_description
  ika_mission
  ika_navigation
  ika_safety
  ika_simulation
  ika_terrain
  ```
- Birim testler 22/22 yeşil:
  ```bash
  for p in ika_terrain ika_safety ika_base_controller; do
    cd ~/ika/ika_ws/src/$p && python3 -m pytest test/ -q
  done
  ```

### Başarısızlık
- **`Package 'X' not found`** → `rosdep install --from-paths src --ignore-src -r -y`
- **`error: invalid argument` / mimari uyumsuz** → Pi 5 arm64 mimaride; apt'in arm64 paketi olmayabilir, source build gerek.
- **Build sürüyor, donmuş gibi** → SD kart yavaş; bekle (10-20 dk) veya NVMe/SSD'ye geç.
- **Disk dolu** (`No space left on device`) → `df -h`. `build/`, `log/`, `~/.ros/log` temizle.

---

## Deneme 1 — URDF Görüntü (display)

**Amaç:** URDF/Xacro modelinin XML'i parse oluyor, RViz'de görünüyor.

### Komut (Pi GUI veya VNC ile)
```bash
ros2 launch ika_description display.launch.py
```

### Beklenen
- RViz açılır.
- Sol panelde **RobotModel** görünür.
- Aracın 4 tekerleği, kutu gövdesi, lidar (kırmızı silindir), kamera, IMU, GPS frame'leri 3B'de render olur.
- TF tree:
  ```
  base_footprint → base_link → {laser_frame, camera_frame, imu_frame, gps_frame}
  ```

### Başarısızlık
- **`xacro` syntax hatası** → `xacro ~/ika/ika_ws/src/ika_description/urdf/ika.urdf.xacro` ile manuel parse et, hata satırını incele.
- **RViz açıldı ama model boş** → `Fixed Frame: base_link` olarak ayarla.
- **`Could not find resource: package://ika_description/...`** → `source ~/ika/ika_ws/install/setup.bash`
- **Pi'de GUI açılmıyor** → SSH'tan geliyorsan `ssh -X` veya VNC kullan, ya da `display.launch.py rviz:=false` ile sadece robot_state_publisher test et.

---

## Deneme 2 — Bare Simülasyon

**Amaç:** Gazebo Harmonic açılıyor, aracın modeli dünya içine yerleştiriliyor, sensörler çalışıyor.

### Komut
```bash
cd ~/ika
./scripts/deploy_sim.sh             # bare sim
```

veya tek tek:
```bash
ros2 launch ika_simulation simulation.launch.py
```

### Beklenen
- Gazebo GUI açılır, `test_world.sdf` yüklenir.
- Dünya içinde: zemin, 3 kırmızı kutu, 2 rampa, 1 mavi platform, 2 duvar (dar geçit).
- İKA aracı (0, 0, 0.1) etrafında zemine düşer ve durur.
- `ros2 topic list` çıktısında en az:
  ```
  /clock
  /cmd_vel
  /scan
  /imu/data
  /gps/fix
  /oak/points
  /oak/depth/image_raw
  /joint_states
  /odom_truth
  /robot_description
  /tf_static
  ```

### Sim arka planda devam ederken denetle (Terminal B)
```bash
ros2 topic hz /scan         # ~10 Hz olmalı
ros2 topic hz /imu/data     # ~100 Hz
ros2 topic hz /clock        # >100 Hz
ros2 topic echo /scan --once | head -20   # ranges[] verisi gelmeli
```

### Başarısızlık
- **Gazebo siyah ekran / segfault** → OGRE2/GPU sorunu. Headless dene:
  ```bash
  ros2 launch ika_simulation simulation.launch.py headless:=true rviz:=false
  ```
- **Araç düşmüyor, hiç görünmüyor** → spawn yüksekliği:
  ```bash
  ros2 launch ika_simulation simulation.launch.py z:=0.5
  ```
- **`gz: command not found`** → `sudo apt install ros-jazzy-ros-gz` yapılmamış.
- **`Failed to load plugin gz-sim-diff-drive-system`** → Gazebo Harmonic'in core plugin'leri eksik:
  ```bash
  sudo apt install --reinstall ros-jazzy-ros-gz-sim
  ```
- **Topic'ler boş, hz 0** → bridge çalışmıyor olabilir:
  ```bash
  ros2 node list | grep bridge
  ros2 node info /ros_gz_bridge
  ```

---

## Deneme 3 — Topic Akışı (verify_sim.sh)

**Amaç:** Tüm beklenen topic'ler ve TF aktif.

### Komut (sim çalışırken, başka terminalde)
```bash
cd ~/ika
./scripts/verify_sim.sh
```

### Beklenen
- Renkli `[OK]` satırları gz topic'leri için: `/scan`, `/imu/data`, `/gps/fix`, `/oak/points`, `/oak/depth_image`, `/sim/odom`, `/joint_states`, `/clock`
- Renkli `[OK]` satırları ROS topic'leri için: `/scan`, `/imu/data`, `/gps/fix`, `/oak/points`, `/odom_truth`, `/joint_states`, `/robot_description`, `/tf_static`
- `ros2 topic hz` çıktıları 5+ Hz
- `view_frames` ile `frames.pdf` üretilir (TF tree görselleştirme)

### Başarısızlık (her `[!!]` satırı)
- **gz `/scan` eksik** → URDF içindeki `<sensor name="rplidar_c1">` blokunda topic adı kontrolü. `gz topic -l` ile gerçek isim.
- **ros `/scan` eksik, gz var** → `config/ros_gz_bridge.yaml`'da topic eşleştirmesi yanlış. `ros2 node info /ros_gz_bridge`
- **`/tf_static` eksik** → robot_state_publisher çalışmıyor: `ros2 node list | grep state_publisher`

---

## Deneme 4 — Manuel Sürüş

**Amaç:** Robot komutla hareket ediyor.

### Komut (sim çalışırken Terminal B)
```bash
cd ~/ika
./scripts/teleop_raw.sh         # /cmd_vel - direkt Gazebo'ya
```

### Beklenen
- Terminal'de tuş kullanım listesi:
  ```
  i / k / , / j / l ...
  ```
- `i` tuşu (ileri): robot ileri hareket eder.
- `j` / `l`: sola/sağa döner.
- `k`: durur.
- RViz'de robot konumu güncellenir (open up RViz, Fixed Frame: `odom`).

### Doğrula:
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom_truth --once
```

### Başarısızlık
- **Robot hiç hareket etmiyor** → cmd_vel topic'i Gazebo'ya gitmiyor:
  ```bash
  ros2 topic info /cmd_vel             # subscriber count > 0 olmalı
  gz topic -t /cmd_vel -i              # gz tarafında da gelmeli
  ```
- **Robot kayıyor, dönmüyor** → Skid-steer sürtünme ayarı düşük olabilir. `urdf/ika_gazebo.xacro` içinde `mu1`/`mu2`.

---

## Deneme 5 — Lidar Odom + EKF + SLAM

**Amaç:** rf2o lidar odometrisi + robot_localization EKF + slam_toolbox birlikte çalışır, harita oluşur.

### Komut (sim çalışırken)
```bash
ros2 launch ika_navigation slam.launch.py use_sim_time:=true
```

### Beklenen
- 3 node ayağa kalkar:
  - `rf2o_laser_odometry` — `/odom` yayar
  - `ekf_filter_node` — `/odometry/filtered` yayar, TF `odom→base_link`
  - `slam_toolbox` — `/map` yayar
- `ros2 topic hz /odom` ~10 Hz
- `ros2 topic hz /map` ~0.1 Hz (5 saniyede bir)

### Harita oluşturma (Terminal C — teleop)
Aracı manuel sürerek farklı yerleri gezdir. Engellerden uzak dur.
```bash
./scripts/teleop_raw.sh
```

### RViz'de göster (Terminal D)
```bash
rviz2
```
Ekle:
- Map → topic: `/map`
- LaserScan → `/scan`
- Odometry → `/odometry/filtered`
- TF
- Fixed Frame: `map`

### Beklenen RViz görüntüsü
- Robot odometri okuyla map içinde hareket ediyor.
- Lidar tarama noktaları engellerin sınırlarını çiziyor.
- `/map` zamanla doluyor (gri = bilinmiyor, beyaz = boş, siyah = engel).

### Haritayı kaydet
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ika/maps/test_map
```

### Başarısızlık
- **`/odom` 0 Hz** → rf2o lidar verisi alamıyor. `/scan` 10 Hz mi kontrol et. rf2o `base_frame_id` `base_link` olmalı.
- **EKF spam log: `TF lookup failed`** → URDF static TF (laser→base_link) yayımlanmamış. `ros2 run tf2_ros tf2_echo base_link laser_frame`
- **Harita boş kalıyor / titreşiyor** → Lidar odom kayıyor (encoder yok). Aracı yavaş sür (`./scripts/teleop_raw.sh`, hız çok ufak tutarak).
- **`slam_toolbox` crash** → `stack_size_to_use` parametresine bak; Pi'de yüksekse düşür.

---

## Deneme 6 — Tam Stack (sim_full)

**Amaç:** Nav2 + Collision Monitor + Safety Supervisor + Terrain + SLAM hepsi birlikte.

### Komut
```bash
./scripts/stop_sim.sh                       # önceki sim'i temizle
./scripts/deploy_sim.sh full
```

### Beklenen
- `simulation.launch.py` + `navigation.launch.py` paralel başlar.
- ~30 saniye içinde lifecycle_manager_navigation tüm Nav2 node'larını aktive eder.
- `ros2 node list` çıktısında:
  ```
  /controller_server
  /planner_server
  /behavior_server
  /bt_navigator
  /collision_monitor
  /ros_gz_bridge
  /robot_state_publisher
  /rf2o_laser_odometry
  /ekf_filter_node
  /navsat_transform
  /slam_toolbox
  /terrain_perception
  /safety_supervisor
  /lifecycle_manager_navigation
  /lifecycle_manager_ika
  /cmd_vel_relay
  ```

### Doğrulama
```bash
ros2 topic hz /odometry/filtered     # ~30 Hz
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
ros2 topic echo /safety_status --once
ros2 topic echo /terrain_state --once
```

### Başarısızlık
- **lifecycle_manager TIMEOUT** → Hangi node aktive olamadı? Logdan bak:
  ```bash
  ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger
  ros2 lifecycle nodes
  ros2 lifecycle get /controller_server
  ```
- **`/cmd_vel_safe` topic'i yok** → Safety supervisor başlamadı. `ros2 node info /safety_supervisor`
- **`/global_costmap/costmap` boş** → SLAM henüz `/map` yayımlamamış. Aracı biraz hareket ettirip haritayı oluştur (Deneme 5).
- **Çok yavaş** → Pi 5 16 GB için bile bu yoğun bir stack. CPU kullanımını izle:
  ```bash
  htop
  ```

---

## Deneme 7 — Navigasyon Hedefi (Nav2)

**Amaç:** Aracın bir noktaya kendi planlayıp gitmesi.

> Önce **Deneme 6** tam stack'i koşturmuş olmalısın. Bir önceki denemeden SLAM map'i `/map` topic'inde olmalı.

### Yol 1: RViz'den (önerilen)
1. RViz'de **2D Nav Goal** aracını seç.
2. Haritada bir noktaya tıkla, sürükle (yön ver), bırak.
3. Robot rotayı çizer ve takip etmeye başlar.

### Yol 2: CLI'dan
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}'
```

### Beklenen
- Path RViz'de mor çizgi olarak çizilir (`/plan` topic'i).
- Robot yavaşça hedefe doğru hareket eder.
- Engellerin etrafından dolaşır.
- Hedefe ulaşınca `/cmd_vel_nav` sıfırlanır, robot durur.

### Doğrula
```bash
ros2 topic echo /plan --once | head
ros2 action list
ros2 action info /navigate_to_pose
```

### Başarısızlık
- **`Goal rejected`** → Hedef harita dışında veya engelin içinde. Hariç frame mi kontrol et:
  ```bash
  ros2 topic echo /goal_pose --once
  ```
- **`Failed to compute path`** → Planlayıcı yol bulamadı. Costmap'i RViz'de göster, mavi/sarı bölge varsa yol oradan geçmiyordur.
- **Robot başlatıp hemen duruyor** → Collision Monitor `stop` polygon'unda engel görüyor. Stop zone polygon'unu RViz'de etkinleştir, görsel olarak bak.
- **Robot oscillating (sallanıp duruyor)** → `dwb_local_planner` parametrelerinde `RotateToGoal.slowing_factor` artır.

---

## Deneme 8 — Terrain Testleri (Rampa, Dropoff)

**Amaç:** Terrain Perception Node rampa ve çukuru doğru sınıflıyor.

### Hazırlık
sim_full çalışıyor olmalı (Deneme 6).

### Rampa testi (terminalden hedef ver)
Aracın `test_world.sdf` içindeki rampa konumlarına gitmesini sağla:
```bash
# Güvenli rampa konumu (%10 eğim — 5.7°):
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.5, y: -3.0}, orientation: {w: 1.0}}}'
```

Aracı izle, `/terrain_state` çıktısını oku:
```bash
ros2 topic echo /terrain_state
```

### Beklenen
- Düz zeminde: `{"class": "SAFE", "slope_deg": 0.x, ...}`
- Güvenli rampada: `class: "SAFE"` veya `"CAUTION"` (~6° ölçülecek)
- Dik rampada (%30): `class: "CAUTION"` veya `"IMPASSABLE"`
- Mavi platforma yaklaşırken kenara gelince: `class: "DROPOFF_DANGER"`

### Dropoff testi
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: -3.0, y: 0.0}, orientation: {w: 1.0}}}'
```
Robot mavi platforma çıkar, kenara gelince `/terrain_state` → `DROPOFF_DANGER`. Safety Supervisor `/cmd_vel_safe` topic'ini sıfırlar, robot durur.

### Doğrula
```bash
ros2 topic echo /terrain_state
ros2 topic echo /safety_status
ros2 topic echo /cmd_vel_safe              # DROPOFF anında 0 yayılmalı
```

### Başarısızlık
- **`/terrain_state` hep `UNKNOWN`** → Depth verisi gelmiyor. `ros2 topic hz /oak/depth/image_raw` 0 ise bridge sorunu.
- **`slope_deg` saçma sapan (>90°)** → IMU oryantasyon yanlış kalibre edilmiş veya yokluğu var. `ros2 topic echo /imu/data --once`
- **DROPOFF tetiklenmiyor** → `dropoff_depth_threshold_m` çok yüksek. `config/terrain_params.yaml`'da düşür (örn. 0.10).

---

## Deneme 9 — Safety Zinciri ve E-Stop

**Amaç:** Sensör kaybedildiğinde araç güvenle durur.

### Hazırlık
sim_full çalışıyor.

### Test 1: Lidar topic kesme
```bash
# Terminal B
ros2 node kill /ros_gz_bridge       # bridge'i öldür, /scan kesilir
```
Veya simüler:
```bash
ros2 topic pub /scan sensor_msgs/LaserScan ... &     # boş yayın
```

### Beklenen
- 1 saniye içinde `/safety_status` JSON'unda `lidar_ok: false`
- `/e_stop` topic'ine `True` yayımlanır
- `/cmd_vel_safe` sıfırlanır → robot durur
- RViz'de robot hareket etmiyor

### Doğrula
```bash
ros2 topic echo /safety_status
ros2 topic echo /e_stop
ros2 topic echo /diagnostics --once   # ERROR seviyesinde sensor watchdog
```

### Test 2: Terrain DROPOFF → durma
Deneme 8'deki dropoff senaryosu zaten bu zinciri test eder.

### Geri kurtarma
```bash
# Bridge'i geri başlat
./scripts/deploy_sim.sh full
# veya sadece bridge:
ros2 launch ika_simulation simulation.launch.py
```

### Başarısızlık
- **`/e_stop` yayımlanmıyor** → Safety Supervisor lifecycle aktif değil:
  ```bash
  ros2 lifecycle get /safety_supervisor
  ros2 lifecycle set /safety_supervisor activate
  ```
- **`watchdog_rate_hz` sıkıntısı** → `config/safety_params.yaml`'da değeri kontrol et.

---

## Deneme 10 — Mission Yöneticisi

**Amaç:** GPS waypoint listesi sırayla yürütülür, dış komutlarla yönlendirilebilir.

### Hazırlık
sim_full çalışıyor, SLAM map'i mevcut.

### Komut
```bash
ros2 run ika_mission gps_waypoint_mission \
  --waypoints ~/ika/ika_ws/src/ika_mission/missions/test_mission.yaml \
  --frame map
```

> İlk koşumda Nav2 hazır değilse 15 saniye bekleyebilir.

### Beklenen
- Log:
  ```
  [...] 3 waypoint yuklendi
  [...] -> Waypoint 1/3 [hedef_1] x=2.00 y=0.50 yaw=0.00
  ```
- Robot hedeflere sırayla gider.
- Her hedef ulaşıldığında bir sonrakine geçer.
- 3 waypoint sonunda `Tum waypoint'ler tamamlandi` log'u.

### Mission durumunu izle (Terminal B)
```bash
ros2 topic echo /mission_state
```

### Dış komutlar (Terminal C)
```bash
# Geçici durdur:
ros2 topic pub --once /mission_cmd std_msgs/String "data: pause"

# Devam:
ros2 topic pub --once /mission_cmd std_msgs/String "data: resume"

# Mevcut waypoint'i atla:
ros2 topic pub --once /mission_cmd std_msgs/String "data: skip"

# İptal:
ros2 topic pub --once /mission_cmd std_msgs/String "data: cancel"

# Yeniden başla:
ros2 topic pub --once /mission_cmd std_msgs/String "data: restart"
```

### Başarısızlık
- **`navigate_to_pose action server bulunamadi`** → Nav2 lifecycle hazır değil. Önce Deneme 6 doğrulanmış olmalı.
- **Goal kabul ediliyor ama hareket yok** → Nav2 hedefe yol bulamıyor. RViz'de costmap kontrol et.
- **Bir hedef başarısız, sonrakine geçmiyor** → max_retries değeri 0 olabilir; `--max-retries 1` argümanı.

---

## Deneme 11 — Diagnostics İzleme

**Amaç:** Sistemin sağlık durumunu canlı görselleştir.

### GUI ile (önerilen)
```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

### Beklenen
- 3 ana grup: `ika_base_controller`, `ika_safety_supervisor`, `ika_terrain`
- Tüm satırlar **YEŞIL** (OK).
- Sensör kesilse → ilgili satır KIRMIZI (ERROR), nedeni gösterilir.

### CLI ile
```bash
ros2 topic echo /diagnostics --once | head -50
```

### rqt_graph ile node ilişkilerini gör
```bash
ros2 run rqt_graph rqt_graph
```
Filtreleri açıp `topics` modunda incele: tüm topic'lerin yayın/abone ilişkilerini grafik halinde görürsün.

### Başarısızlık
- **`/diagnostics` boş** → diagnostics_msgs/Updater Pi'de yüklü değil:
  ```bash
  sudo apt install ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-msgs
  ```
- **rqt_robot_monitor açılmıyor** → `sudo apt install ros-jazzy-rqt-robot-monitor`

---

## Deneme 12 — Keepout Zone (Opsiyonel)

**Amaç:** Nav2'nin "yasak alan" maskesini takip etmesi.

> Bu deneme **opsiyoneldir**. Maske dosyası (`keepout_mask.yaml + .pgm`) önce
> oluşturulmalı. Detaylı talimat: [KEEPOUT.md](KEEPOUT.md).

### Hazırlık
1. SLAM ile bir harita üret ve kaydet (Deneme 5 sonu).
2. Haritadan keepout maske dosyaları üret (KEEPOUT.md Bölüm 1).
3. `navigation.launch.py` içinde `costmap_filter_info_server` ve `filter_mask_server` node'larını aktif et (KEEPOUT.md Bölüm 2).
4. Workspace'i yeniden build et: `colcon build --symlink-install`

### Test
```bash
./scripts/deploy_sim.sh full
ros2 topic echo /keepout_filter_mask --once
ros2 topic echo /costmap_filter_info --once
```

### Beklenen
- Her iki topic'te de veri yayını var.
- RViz'de Map display'i `/keepout_filter_mask` topic'ine bağlanınca siyah/beyaz maske görünür.
- Global costmap'te yasak bölgeler **kırmızı (lethal)** görünür.
- Nav2 hedefi yasak bölgenin arkasına verirsen yol etrafından dolaşır.
- Nav2 hedefi yasak bölgenin içine verirsen `Goal rejected` veya plan bulunamaz.

### Başarısızlık
- KEEPOUT.md "Sorun Giderme" bölümüne bak.

---

## Genel Sorun Giderme

### Sim bir kez başarılı oldu, ikincide saçma davranıyor
ROS daemon ve yetim süreçler kalmış olabilir:
```bash
./scripts/stop_sim.sh
ros2 daemon stop
sleep 2
ros2 daemon start
```

### `colcon build` her seferinde uzun sürüyor
`--symlink-install` flag'i Python paketleri için kaynak değişikliği colcon olmadan yansıtır. Sadece launch/config değiştirdiysen yeniden build gerekmeyebilir.

### Disk yer kalmadı
```bash
df -h
# build, install temizle
rm -rf ~/ika/ika_ws/build ~/ika/ika_ws/install ~/ika/ika_ws/log
# rosbag temizle
rm -f ~/ika*.db3 ~/ika*.mcap
# apt cache temizle
sudo apt clean
```

### Pi aşırı ısınıyor / throttle
```bash
vcgencmd measure_temp           # 80°C üstü kritik
vcgencmd get_throttled          # 0x0 ideal
```
Pi 5 için active cooler + heatsink şart. `sim_full` koşarken CPU 100%'e çıkar.

### Log dosyaları nereye?
- Build: `/tmp/ika_build.log`
- Sim: `/tmp/ika_sim.log`
- ROS 2 her node için: `~/.ros/log/<timestamp>/`
- Crash dumps: `~/.ros/log/latest/<node>/stderr.log`

### Network tarafından kaynaklı sorun
```bash
hostname -I
ping 8.8.8.8
ros2 daemon stop && ros2 daemon start
```
Pi'nin DDS multicast'i blocklanmış olabilir; gerekirse:
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Hiçbir şey çalışmıyor, yeniden başlayayım
```bash
./scripts/stop_sim.sh
ros2 daemon stop
cd ~/ika/ika_ws
rm -rf build install log
colcon build --symlink-install --parallel-workers $(nproc)
source install/setup.bash
./scripts/deploy_sim.sh
```

---

## Bir Sonraki Adım

Tüm 11 deneme yeşilse:

1. Gerçek araç tarafına geçmeye hazırsın. [IKA_ROS2_System_Reference.md](IKA_ROS2_System_Reference.md) **Bölüm 15** "Gerçek Araca Geçiş Prosedürü" listesini takip et.
2. Arduino yüklemesi: `ika_ws/src/ika_base_controller/arduino/ika_motor_controller/`
3. Kalibrasyon: `IKA_ROS2_System_Reference.md` **Bölüm 16**.

İyi denemeler. Hata varsa: log dosyasını + hangi denemede çıktığını + ekran görüntüsünü paylaş.
