# İKA — Kurulum Rehberi

Bu doküman **fresh Ubuntu Server 24.04.4** kurulmuş bir Raspberry Pi 5'i sıfırdan
İKA çalıştırılabilir hale getirmek için adım adım rehberdir.

İki yol vardır:

- **Hızlı yol (önerilen):** [`scripts/install_pi.sh`](scripts/install_pi.sh) tek komutta her şeyi kurar. Bkz. [Bölüm 5](#5-otomatik-kurulum-tek-komut).
- **Manuel yol:** Her adımı kendin yürütmek istersen [Bölüm 2-4](#2-pi-ön-hazırlık) takip et.

> Sorun çıkarsa: en alttaki [SORUN GİDERME](#10-sorun-giderme) bölümüne bak.

---

## İçindekiler

1. [Önkoşullar](#1-önkoşullar)
2. [Pi ön-hazırlık (manuel)](#2-pi-ön-hazırlık)
3. [GitHub üzerinden kodu Pi'ye getirme](#3-githubdan-kodu-piye-getirme)
4. [Manuel kurulum adımları](#4-manuel-kurulum-adımları)
5. [Otomatik kurulum (tek komut)](#5-otomatik-kurulum-tek-komut)
6. [İlk derleme (colcon build)](#6-i̇lk-derleme-colcon-build)
7. [Kabuk (bash) yapılandırması](#7-kabuk-bash-yapılandırması)
8. [Donanım hazırlığı (udev)](#8-donanım-hazırlığı-udev)
9. [Kurulum doğrulama](#9-kurulum-doğrulama)
10. [Sorun giderme](#10-sorun-giderme)

---

## 1. Önkoşullar

- Raspberry Pi 5 (16 GB RAM önerilir)
- En az **32 GB** boş SD/NVMe disk
- İnternet bağlantısı (apt ve git için, Ethernet veya WiFi)
- Bir başka bilgisayardan SSH erişimi (zorunlu değil ama önerilir)
- GitHub repo hazır (kodu push'ladığın repo)

> Bu rehberde GitHub repo adresi `https://github.com/KULLANICI/ika-ros2.git` örnek olarak geçer. Kendi reponun URL'i ile değiştir.

---

## 2. Pi Ön-Hazırlık

> İlk açılışta `ubuntu` kullanıcısıyla giriş yapılır (default parola `ubuntu`, ilk girişte değişmeni ister).

### 2.1 Network bağlantısı

Ethernet kullanıyorsan kablo yeter. WiFi için:
```bash
sudo nmcli device wifi connect "WIFI_ADI" password "PAROLA"
# veya
sudo nmtui            # tekstil grafik araç
```

IP adresini öğren:
```bash
ip -4 addr show | grep inet
```

### 2.2 SSH (opsiyonel, çok yararlı)

```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
```

Artık başka bir makineden:
```bash
ssh ubuntu@<PI_IP>
```

### 2.3 Hostname (opsiyonel)

```bash
sudo hostnamectl set-hostname ika
sudo reboot
```

### 2.4 Sistem güncellemesi

```bash
sudo apt update
sudo apt upgrade -y
```

### 2.5 Zaman dilimi / NTP

```bash
sudo timedatectl set-timezone Europe/Istanbul
timedatectl                # 'NTP service: active' olmalı
```

### 2.6 Locale (ROS 2 için kritik)

ROS 2 UTF-8 locale gerektirir.
```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
echo "export LANG=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
locale                      # LC_ALL=en_US.UTF-8 olmalı
```

### 2.7 Temel araçlar

```bash
sudo apt install -y \
  curl gnupg2 lsb-release ca-certificates \
  software-properties-common \
  git vim tmux htop \
  build-essential cmake python3-pip \
  usbutils v4l-utils
sudo add-apt-repository -y universe
```

### 2.8 Kullanıcı gruplarına ekleme (seri port + USB)

```bash
sudo usermod -aG dialout,plugdev,video $USER
# ÖNEMLİ: Log-out / log-in gerekli, yoksa gruplar aktif olmaz
```

---

## 3. GitHub'dan Kodu Pi'ye Getirme

> Kod Windows'ta `C:\Users\aslan\Desktop\ikasu\` altında geliştirildi. GitHub'a push'lanıp Pi tarafında clone edilecek.

### 3.1 Windows tarafı (bir kerelik)

PowerShell veya Git Bash'te:
```bash
cd C:\Users\aslan\Desktop\ikasu
git init
git add .
git commit -m "Initial commit: IKA workspace"
git branch -M main
git remote add origin https://github.com/KULLANICI/ika-ros2.git
git push -u origin main
```

> Hatırlatma: GitHub'da reponun **önce oluşturulması** gerek (https://github.com/new). Boş bir repo (README/lisans eklemeden) oluştur.

Sonraki güncellemelerde:
```bash
git add .
git commit -m "açıklama"
git push
```

### 3.2 Pi tarafı

```bash
cd ~
git clone https://github.com/KULLANICI/ika-ros2.git ika
cd ika
ls           # ika_ws/ scripts/ README.md ... görmelisin
```

> Sonraki güncellemelerde Pi'de: `cd ~/ika && git pull && ./scripts/deploy_sim.sh bare clean` (rebuild dahil)

---

## 4. Manuel Kurulum Adımları

> `install_pi.sh` ile otomatik kurulumu tercih ediyorsan [Bölüm 5](#5-otomatik-kurulum-tek-komut)'e atla.

### 4.1 ROS 2 Jazzy apt deposu

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
```

### 4.2 ROS 2 Jazzy desktop

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

> ~3 GB indirir. WiFi'de zaman alabilir.

### 4.3 Gazebo Harmonic + bridge

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-image
```

### 4.4 Navigation, SLAM, lokalizasyon

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-rf2o-laser-odometry
```

> `nav2-mppi-controller`: klasik DWB'ye alternatif MPPI yerel planlayıcısı
> (tez kapsamındaki planlayıcı karşılaştırması; `local_planner:=mppi`).

### 4.5 Robot tanım + TF

```bash
sudo apt install -y \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-ros
```

### 4.6 Teleop + diagnostics + rqt

```bash
sudo apt install -y \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-topic-tools \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-diagnostic-aggregator \
  ros-jazzy-rqt-robot-monitor \
  ros-jazzy-rqt-graph \
  ros-jazzy-rqt-plot
```

### 4.7 GPS sürücüsü

```bash
sudo apt install -y ros-jazzy-nmea-navsat-driver || echo "apt'te yoksa source build gerekli"
```

### 4.8 Lidar sürücüsü (RPLIDAR C1)

```bash
sudo apt install -y ros-jazzy-sllidar-ros2 2>/dev/null || \
  ( cd ~/ika/ika_ws/src && mkdir -p third_party && cd third_party && \
    git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git )
```

### 4.9 Algılama: vision_msgs + Depth kamera (OAK-D Lite)

DL nesne tespiti `/detected_objects`'i **vision_msgs/Detection3DArray** olarak
yayar; `ika_perception_dl` ve `ika_fusion` için bu mesaj paketi **zorunlu**
(sim dahil):
```bash
sudo apt install -y ros-jazzy-vision-msgs
```

OAK-D Lite sürücüsü (yalnız gerçek araç):
```bash
sudo apt install -y ros-jazzy-depthai-ros-driver 2>/dev/null || \
  echo "depthai apt'te yok. Sim için gerek yok; gerçek araç gerektiğinde manuel kur: https://github.com/luxonis/depthai-ros"
```

VPU üzerinde spatial detection için **depthai Python** kütüphanesi gerekir
(yalnız gerçek araç; sim'de `sim_detection_node` kullanıldığından gerekmez):
```bash
python3 -m pip install --user --break-system-packages depthai
```

### 4.10 Python paketleri

```bash
sudo apt install -y python3-serial python3-yaml python3-numpy python3-pytest
```

### 4.11 ROS 2 setup'ı bash'e ekle

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Otomatik Kurulum (Tek Komut)

Bölüm 2.8'e kadar bitirdiysen (kullanıcı grupları + locale), repo'yu da klonladıysan:

```bash
cd ~/ika
chmod +x scripts/install_pi.sh
./scripts/install_pi.sh
```

Bu script yukarıdaki Bölüm 4'ün tüm adımlarını + udev + build + verify aşamasını sırayla yapar. Renkli log yazar.

**Tek faz çalıştırma:**
```bash
./scripts/install_pi.sh system          # FAZ 1: sistem + araçlar
./scripts/install_pi.sh ros             # FAZ 2: ROS 2 Jazzy
./scripts/install_pi.sh packages        # FAZ 3: ROS paketleri
./scripts/install_pi.sh python          # FAZ 4: Python paketleri
./scripts/install_pi.sh udev            # FAZ 5: udev kuralları
./scripts/install_pi.sh build           # FAZ 6: workspace build
./scripts/install_pi.sh verify          # FAZ 7: doğrulama
```

Script idempotent: zaten kurulu paketleri atlar, sorunsuz tekrar koşulur.

> İlk koşumda `sudo` parolası ister, sonrasında dokunmaz (60 saniyede bir refresh eder).

---

## 6. İlk Derleme (colcon build)

`install_pi.sh build` zaten yapar. Manuel yapmak istersen:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ika/ika_ws

# rosdep ilk çalıştırma (yalnız bir kere)
sudo rosdep init
rosdep update --rosdistro=jazzy

# Eksik bağımlılıkları tara/kur
rosdep install --from-paths src --ignore-src -r -y

# Build (tüm çekirdekleri kullan)
colcon build --symlink-install --parallel-workers $(nproc)
```

Build tamamen yeşil bittikten sonra:
```bash
source ~/ika/ika_ws/install/setup.bash
```

---

## 7. Kabuk (bash) Yapılandırması

`~/.bashrc` sonuna şu iki satırı ekle (yoksa):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
```

Faydalı alias'lar (opsiyonel):
```bash
echo "alias cb='cd ~/ika/ika_ws && colcon build --symlink-install'"  >> ~/.bashrc
echo "alias sb='source ~/ika/ika_ws/install/setup.bash'"             >> ~/.bashrc
echo "alias ika='cd ~/ika'"                                          >> ~/.bashrc
source ~/.bashrc
```

---

## 8. Donanım Hazırlığı (udev)

`install_pi.sh udev` zaten yapar. Manuel yapacaksan:

```bash
sudo tee /etc/udev/rules.d/99-ika-usb.rules >/dev/null <<'EOF'
# CP210x (SLAMTEC RPLIDAR C1)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ika_lidar",   MODE="0666"
# Arduino Uno (orijinal)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="ika_arduino", MODE="0666"
# Arduino Uno klon (CH340)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ika_arduino", MODE="0666"
# u-blox GPS
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="ika_gps",     MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

> Donanımını taktıktan sonra `lsusb` ile VID:PID'i öğren, gerekirse kuralları kendi donanımına göre güncelle.

Test:
```bash
ls -l /dev/ika_*    # ika_lidar, ika_arduino, ika_gps sembolik linkler görünmeli
```

---

## 9. Kurulum Doğrulama

Tek komut:
```bash
./scripts/install_pi.sh verify
```

Manuel kontroller:

### 9.1 ROS 2 cli

```bash
ros2 --help
ros2 doctor --report          # genel durum
```

### 9.2 Gazebo Harmonic

```bash
gz --version                  # 8.x.x görmeliyiz (Harmonic)
gz sim --help
```

### 9.3 Workspace paketleri görünüyor mu?

```bash
ros2 pkg list | grep ika_     # 11 paket listelenmeli
ros2 pkg prefix ika_simulation
```

> Paketler: ika_bringup, ika_description, ika_simulation, ika_navigation,
> ika_terrain, ika_safety, ika_base_controller, ika_mission,
> **ika_perception_dl**, **ika_fusion**, **ika_rl_planner**.

### 9.4 Birim testler (Pi'de gerek yoktur ama hızlı doğrulama)

```bash
# Tek komutta tüm paketlerin testleri (önerilen)
cd ~/ika/ika_ws
colcon test --packages-select \
  ika_terrain ika_safety ika_base_controller \
  ika_perception_dl ika_fusion ika_rl_planner
colcon test-result --verbose

# veya tek tek pytest ile:
cd ~/ika/ika_ws/src/ika_terrain        && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_safety         && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_base_controller && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_perception_dl  && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_fusion         && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_rl_planner     && python3 -m pytest test/ -v
```

> Toplam 76 test, hepsi PASSED olmalı.

### 9.5 URDF doğrulama

```bash
xacro ~/ika/ika_ws/src/ika_description/urdf/ika.urdf.xacro use_sim:=true > /tmp/ika.urdf
check_urdf /tmp/ika.urdf       # liburdfdom-tools paketinden gelir
```

---

## 10. Sorun Giderme

### 10.1 `ros2: command not found`

```bash
source /opt/ros/jazzy/setup.bash
```
`~/.bashrc`'a eklenmiş mi kontrol et:
```bash
grep ros/jazzy ~/.bashrc
```

### 10.2 `Permission denied: /dev/ttyACM0` veya `/dev/ttyUSB0`

```bash
groups | grep dialout         # dialout görünmüyorsa:
sudo usermod -aG dialout $USER
# Logout/login zorunlu, ya da geçici çözüm:
sudo chmod 666 /dev/ttyACM0
```

### 10.3 `gz sim` açılmıyor / SIGSEGV (Pi 5'te)

OGRE2 / GPU ile ilgili olabilir. Düşük seviye render:
```bash
export GZ_GUI_TILE_DEBUG=0
gz sim --render-engine ogre2
```

Hâlâ sorun varsa headless dene:
```bash
./scripts/deploy_sim.sh         # default
# veya:
ros2 launch ika_simulation simulation.launch.py headless:=true
```

### 10.4 colcon build başarısız: paket bulunamadı

```bash
cd ~/ika/ika_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 10.5 `Failed to launch nodelet`, `mismatched signature` vs.

Çakışan eski build var olabilir:
```bash
cd ~/ika/ika_ws
rm -rf build install log
colcon build --symlink-install --parallel-workers $(nproc)
```

### 10.6 RViz açıldı ama 3D model yok

`Fixed Frame` ayarı `base_link` veya `map` olmalı. URDF'in `/robot_description` topic'ine yayımlandığını kontrol et:
```bash
ros2 topic echo /robot_description --once | head
```

### 10.7 Gazebo'da robot düşüyor / kayboluyor

`-z` spawn yüksekliğini artır:
```bash
ros2 launch ika_simulation simulation.launch.py z:=0.5
```

### 10.8 SSH disconnect / GUI uzaktan

GUI uygulamalar (RViz, Gazebo GUI) Pi'nin **fiziksel ekranı** veya **X11 forwarding** gerektirir. SSH ile gelirsen:
```bash
ssh -X ubuntu@<PI_IP>          # X forwarding (ağır)
```
Daha iyisi: Pi'ye monitor bağla veya VNC kullan.

VNC kurulumu:
```bash
sudo apt install -y tigervnc-standalone-server
vncserver :1 -geometry 1280x800
# Sonra başka makineden: vncviewer <PI_IP>:5901
```

### 10.9 SD kart yavaş build

Önerilen: USB 3.0 SSD veya NVMe HAT kullan. SD kartla colcon build 10-20 dk sürebilir.

### 10.10 `KeyError: 'use_sim'` veya benzer launch hatası

Genelde xacro arg geçişi sorunu. Doğru komut:
```bash
xacro ~/ika/ika_ws/src/ika_description/urdf/ika.urdf.xacro use_sim:=true
```

### 10.11 `Could not find a package configuration file provided by "<X>"`

`rosdep install` çalıştır. Bulunmuyorsa apt deposunda yoktur → source build gerekir veya alternatif paket bul.

### 10.12 Daha derin sorun → log

- Build hatası: `cat /tmp/ika_build.log`
- Sim runtime: `tail -f /tmp/ika_sim.log`
- ROS 2 daemon: `ros2 daemon stop && ros2 daemon start`

---

## Kurulum Tamamlandı — Sonraki Adım

Tüm doğrulamalar yeşilse [DENEMELER.md](DENEMELER.md) dosyasına geç. Orada
11 farklı test senaryosu var, ilk sim denemesinden tam navigasyona kadar.

İlk komut için kısa yol:
```bash
cd ~/ika
./scripts/deploy_sim.sh
```
