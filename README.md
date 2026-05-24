# İKA — ROS 2 Jazzy Otonom Kara Aracı

İKA, ROS 2 Jazzy üstünde geliştirilen 4 tekerlekli **skid-steer otonom kara
aracıdır.** Raspberry Pi 5 + Arduino + RPLIDAR C1 + OAK-D Lite + IMU + GPS
donanımıyla SLAM tabanlı haritalama, Nav2 ile otonom navigasyon ve özel
terrain/safety katmanlarına sahiptir.

> Proje "**önce simülasyon**" felsefesini takip eder: her davranış önce
> Gazebo Harmonic'te doğrulanır, sonra gerçek araca taşınır.

## Hızlı Başlangıç

| Adım | Doküman |
|---|---|
| 1. Pi'yi sıfırdan kur | [**KURULUM.md**](KURULUM.md) — Ubuntu 24.04'ten ROS 2 Jazzy + Gazebo + tüm paketlere |
| 2. Adım adım sim ve test yap | [**DENEMELER.md**](DENEMELER.md) — 11 deneme senaryosu, başarısızlık çözümleri ile |
| 3. Sistem mimarisi ve referans | [**IKA_ROS2_System_Reference.md**](IKA_ROS2_System_Reference.md) — geliştirici düzeyinde detay |

## GitHub Üzerinden Transfer Akışı

Kod Windows'ta geliştirilip GitHub'a push'lanır, Pi'de clone edilir.

### Windows tarafı — ilk kerelik

GitHub'da boş bir repo aç (önce **https://github.com/new**, **README/lisans EKLEME**).

PowerShell veya Git Bash'te:
```bash
cd C:\Users\aslan\Desktop\ikasu

# Git identity (kendi bilgilerinle)
git config user.name "Adın Soyadın"
git config user.email "you@example.com"

# İlk commit
git commit -m "Initial commit: IKA workspace"

# Remote bağla ve push
git remote add origin https://github.com/KULLANICI/ika-ros2.git
git push -u origin main
```

> Her şey hazır: `.gitignore`, `.gitattributes` (LF zorunlu), tüm dosyalar stage'li. Yalnız `git commit` ve `push` kaldı.

### Sonraki güncellemeler

Windows'ta:
```bash
git add .
git commit -m "açıklama"
git push
```

Pi'de:
```bash
cd ~/ika
git pull
./scripts/deploy_sim.sh bare clean   # rebuild + sim
```

### Pi tarafı — ilk kerelik

[KURULUM.md](KURULUM.md) Bölüm 3 detaylı anlatıyor. Kısa hali:
```bash
sudo apt update && sudo apt install -y git
git clone https://github.com/KULLANICI/ika-ros2.git ~/ika
cd ~/ika
chmod +x scripts/install_pi.sh
./scripts/install_pi.sh                 # ~30-40 dk, paralel kurulum
source ~/.bashrc
./scripts/deploy_sim.sh                 # ilk sim denemesi
```

## Workspace Yapısı

```
ikasu/
├── KURULUM.md                # Detaylı kurulum rehberi
├── DENEMELER.md              # 11 test senaryosu
├── IKA_ROS2_System_Reference.md
├── README.md
├── ika_ws/
│   └── src/
│       ├── ika_bringup/      # Üst düzey launch, RViz, robot_params
│       ├── ika_description/  # URDF/Xacro, sensör frameleri, Gazebo plugin
│       ├── ika_navigation/   # Nav2 + SLAM + EKF + rf2o config + launch
│       ├── ika_terrain/      # Terrain Perception (RANSAC + slope)
│       ├── ika_safety/       # Safety Supervisor (filter + watchdog)
│       ├── ika_base_controller/  # Pi tarafı serial köprü + arduino/
│       ├── ika_mission/      # GPS waypoint görev yöneticisi
│       └── ika_simulation/   # Gazebo Harmonic worlds + bridge + launch
└── scripts/
    ├── install_pi.sh   # Sıfırdan Pi kurulumu (tek komut)
    ├── deploy_sim.sh   # Build + sim launch + verify (Pi)
    ├── stop_sim.sh     # Temiz kapatma
    ├── verify_sim.sh   # Topic/TF akış denetimi
    ├── teleop_safe.sh  # /cmd_vel_nav ile (safety zincirinden geçer)
    └── teleop_raw.sh   # /cmd_vel direkt (sim bypass)
```

## Önemli Tasarım Kararları

- **Encoder yok:** Odometri `rf2o_laser_odometry` (lidar tabanlı). Max hız 0.25 m/s.
- **Skid-steer:** Sol/sağ teker grupları paralel; `wheel_base` YAML'dan.
- **Güvenlik zinciri:** `Nav2 → Collision Monitor → Safety Supervisor → Arduino → Fiziksel E-Stop`. Her katman üst katman olmadan da çalışır.
- **Pi ↔ Arduino:** USB Serial üzerinden JSON protokol (`{"l":0.12,"r":-0.05}\n`). micro-ROS değil (ilk faz).
- **Tüm parametreler YAML'da:** Araç boyutları, hız sınırları, terrain eşikleri hiçbir yerde kodda hard-code değil.
- **Lifecycle node'lar:** Terrain, Safety, Base Controller hepsi lifecycle node — `lifecycle_manager` ile yönetilir.

## Test Durumu

| Katman | Test | Durum |
|---|---|---|
| Python syntax | 26 dosya `ast.parse` | ✅ |
| YAML/XML | 9 yaml + 8 package.xml + 5 xacro + 1 sdf | ✅ |
| Shell scriptler | 6 dosya `bash -n` | ✅ |
| Birim testler | RANSAC + terrain + safety + kinematik | ✅ 22/22 |
| Sim entegrasyon | Gazebo + URDF + bridge | ⏳ Pi'de doğrulanacak |
| Full stack | Nav2 + safety + terrain | ⏳ Pi'de doğrulanacak |

## Geliştirici Komutları (Pi'de)

```bash
# Workspace içinden build
cd ~/ika/ika_ws && colcon build --symlink-install

# Belirli paket build
colcon build --packages-select ika_terrain --symlink-install

# Birim testler
cd ~/ika/ika_ws/src/ika_terrain && python3 -m pytest test/ -v

# Tek sim katman test
ros2 launch ika_description display.launch.py        # URDF görüntü
ros2 launch ika_simulation simulation.launch.py      # bare Gazebo
ros2 launch ika_navigation slam.launch.py            # rf2o+EKF+SLAM
ros2 launch ika_bringup sim_full.launch.py           # tam stack

# Tek komut workflow
./scripts/deploy_sim.sh           # bare
./scripts/deploy_sim.sh full      # tam stack
./scripts/stop_sim.sh             # temiz kapat
./scripts/verify_sim.sh           # topic akış denetimi
```

## Lisans

Apache-2.0.

## Daha Fazla

Geliştirici düzeyinde mimari, parametre referansı ve gerçek araca geçiş
prosedürü için [IKA_ROS2_System_Reference.md](IKA_ROS2_System_Reference.md)'a bak.
