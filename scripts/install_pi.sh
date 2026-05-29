#!/usr/bin/env bash
# IKA - Raspberry Pi 5 (Ubuntu 24.04) sifirdan kurulum scripti.
#
# Calistirma:
#   chmod +x scripts/install_pi.sh
#   ./scripts/install_pi.sh                 # tum fazlari sirayla
#   ./scripts/install_pi.sh system          # yalniz system update + temel araclar
#   ./scripts/install_pi.sh ros             # yalniz ROS 2 Jazzy
#   ./scripts/install_pi.sh packages        # yalniz ROS paketleri
#   ./scripts/install_pi.sh python          # yalniz Python paketleri
#   ./scripts/install_pi.sh udev            # yalniz udev kurallari
#   ./scripts/install_pi.sh build           # yalniz workspace build
#   ./scripts/install_pi.sh verify          # kurulum dogrulama
#
# Idempotent: re-run guvenli. Kurulu paketleri atlar.
# Sudo refresh icin baslangic sirasinda parola istenir.

set -uo pipefail

# ---- Argumanlar ------------------------------------------------------
PHASE="${1:-all}"

# ---- Renkler ---------------------------------------------------------
G="\033[0;32m"; R="\033[0;31m"; Y="\033[0;33m"; B="\033[0;34m"; M="\033[0;35m"; NC="\033[0m"

phase() { echo -e "\n${M}######  $1  ######${NC}"; }
step()  { echo -e "${B}== $1 ==${NC}"; }
ok()    { echo -e "${G}[OK]${NC} $1"; }
warn()  { echo -e "${Y}[WW]${NC} $1"; }
err()   { echo -e "${R}[!!]${NC} $1"; }

# ---- Yardimcilar -----------------------------------------------------
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

apt_install() {
  # Toplu kurulum - tek paket eksikse tum batch hata verir.
  # Garanti var olan paketler icin kullan.
  local pkgs=("$@")
  local need=()
  for p in "${pkgs[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || need+=("$p")
  done
  if (( ${#need[@]} == 0 )); then
    ok "Zaten kurulu: ${pkgs[*]}"
    return 0
  fi
  echo "Kurulacak: ${need[*]}"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${need[@]}"
}

apt_install_safe() {
  # Tolerantli kurulum - her paketi tek tek dener, bulunmayan paketi atlar.
  # Olabilirligi belirsiz ROS paketleri icin kullan (jazzy apt deposu eksik olabilir).
  local missing=()
  for p in "$@"; do
    if dpkg -s "$p" >/dev/null 2>&1; then
      ok "Zaten kurulu: $p"
    elif sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$p" 2>/dev/null; then
      ok "Kuruldu: $p"
    else
      warn "Bulunamadi (apt): $p - source clone gerekebilir"
      missing+=("$p")
    fi
  done
  if [[ ${#missing[@]} -gt 0 ]]; then
    APT_MISSING+=("${missing[@]}")
  fi
}

# Eksik paketleri biriktirme listesi (apt_install_safe doldurur)
declare -a APT_MISSING=()

clone_into_workspace() {
  # ws/src/third_party altina git clone (idempotent)
  local url="$1" name="$2" branch="${3:-}"
  local target="$PROJECT_ROOT/ika_ws/src/third_party/$name"
  if [[ -d "$target/.git" ]]; then
    ok "Zaten clone: $name"
    return 0
  fi
  mkdir -p "$PROJECT_ROOT/ika_ws/src/third_party"
  local args=(--depth 1)
  [[ -n "$branch" ]] && args+=(--branch "$branch")
  if git clone "${args[@]}" "$url" "$target" 2>/dev/null; then
    ok "Clone OK: $name"
  else
    warn "Clone basarisiz: $name ($url)"
  fi
}

ensure_line() {
  # bir satiri dosyada yoksa ekler
  local line="$1" file="$2"
  grep -qxF "$line" "$file" 2>/dev/null || echo "$line" >> "$file"
}

require_ubuntu_2404() {
  if [[ ! -f /etc/os-release ]]; then
    err "/etc/os-release yok - desteklenmeyen sistem"; exit 1
  fi
  # shellcheck disable=SC1091
  . /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
    err "Ubuntu 24.04 bekleniyor, bulunan: ${ID:-}-${VERSION_ID:-}"
    err "Devam etmek istiyorsan IKA_SKIP_OS_CHECK=1 ile tekrar dene"
    [[ "${IKA_SKIP_OS_CHECK:-}" == "1" ]] || exit 1
  fi
}

# ---- Fazlar ----------------------------------------------------------

phase_system() {
  phase "FAZ 1: Sistem hazirlik"
  require_ubuntu_2404
  ok "Ubuntu 24.04 dogrulandi"

  step "Locale -> en_US.UTF-8 (ROS 2 gereksinimi)"
  apt_install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  ok "Locale ayarlandi"

  step "Temel araclar"
  apt_install \
    curl gnupg2 lsb-release ca-certificates \
    software-properties-common \
    git vim nano tmux htop \
    build-essential cmake python3-pip \
    usbutils v4l-utils
  ok "Temel araclar kurulu"

  step "Universe repo etkin"
  sudo add-apt-repository -y universe
  ok "universe repo"

  step "Sistem guncellemesi"
  sudo apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get -y upgrade
  ok "Sistem guncel"

  step "Kullanici gruplari (dialout, plugdev, video)"
  for g in dialout plugdev video input; do
    if id -nG "$USER" | grep -qw "$g"; then
      ok "$USER zaten $g grubunda"
    else
      sudo usermod -aG "$g" "$USER"
      warn "$USER -> $g eklendi. Logout/login gerekli."
    fi
  done
}

phase_ros() {
  phase "FAZ 2: ROS 2 Jazzy kurulumu"

  step "ROS 2 apt anahtari"
  if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    ok "ros.key indirildi"
  else
    ok "ros anahtari mevcut"
  fi

  step "ROS 2 apt deposu"
  local codename
  # shellcheck disable=SC1091
  codename=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
  local line
  line="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $codename main"
  echo "$line" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt-get update
  ok "ROS 2 deposu: $codename"

  step "ros-jazzy-desktop (RViz dahil) + ros-dev-tools"
  apt_install ros-jazzy-desktop ros-dev-tools
  ok "ROS 2 Jazzy core kurulu"

  step "Bash setup: ROS 2 setup.bash"
  ensure_line "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc"
  ok "~/.bashrc icin ROS 2 source eklendi"
}

phase_packages() {
  phase "FAZ 3: Proje ROS paketleri"

  step "Nav2 + SLAM + lokalizasyon (core)"
  # ros-jazzy-navigation2 metapaketi Jazzy'de YOK - nav2-bringup yeterli
  # nav2-mppi-controller: DWB'ye alternatif MPPI yerel planlayici (tez karsilastirmasi)
  apt_install_safe \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-mppi-controller \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization

  step "rf2o lidar odom (apt yoksa source)"
  if ! apt_install_safe ros-jazzy-rf2o-laser-odometry; then :; fi
  if ! dpkg -s ros-jazzy-rf2o-laser-odometry >/dev/null 2>&1; then
    clone_into_workspace https://github.com/MAPIRlab/rf2o_laser_odometry.git rf2o_laser_odometry
  fi

  step "Gazebo Harmonic + bridge"
  apt_install_safe \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image

  step "Robot tanim / kontrol"
  apt_install_safe \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-ros

  step "Teleop + topic tools + diagnostics"
  apt_install_safe \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-topic-tools \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-diagnostic-aggregator \
    ros-jazzy-rqt-robot-monitor \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-plot

  step "GPS surucusu (apt yoksa source)"
  if ! apt_install_safe ros-jazzy-nmea-navsat-driver; then :; fi
  if ! dpkg -s ros-jazzy-nmea-navsat-driver >/dev/null 2>&1; then
    clone_into_workspace https://github.com/ros-drivers/nmea_navsat_driver.git nmea_navsat_driver ros2
  fi

  step "Lidar surucusu (RPLIDAR C1) - jazzy apt'te genelde yok"
  if ! apt_install_safe ros-jazzy-sllidar-ros2; then :; fi
  if ! dpkg -s ros-jazzy-sllidar-ros2 >/dev/null 2>&1; then
    clone_into_workspace https://github.com/Slamtec/sllidar_ros2.git sllidar_ros2
  fi

  step "Algilama mesajlari (vision_msgs) - DL tespit + fuzyon icin zorunlu"
  # ika_perception_dl /detected_objects (Detection3DArray) yayar, ika_fusion tuketir.
  apt_install_safe ros-jazzy-vision-msgs

  step "Depth kamera surucusu (OAK-D Lite / depthai)"
  if ! apt_install_safe ros-jazzy-depthai-ros-driver; then
    warn "depthai-ros apt'te yok. Sim icin gerekli degil."
    warn "Gercek arac icin manuel: https://github.com/luxonis/depthai-ros"
  fi
  # NOT: ika_perception_dl'in VPU spatial tespiti depthai Python kutuphanesine
  # baglidir (pip ile FAZ 4'te kurulur). Sim'de sim_detection_node kullanilir,
  # depthai gerekmez.

  step "Costmap filtreleri (keepout zone icin)"
  apt_install_safe ros-jazzy-nav2-map-server ros-jazzy-nav2-costmap-2d

  # Eksik paket raporu
  if (( ${#APT_MISSING[@]} > 0 )); then
    warn "Apt'te bulunamayan paketler (workspace build sirasinda source'tan derlenebilir):"
    for p in "${APT_MISSING[@]}"; do echo "  - $p"; done
  fi
}

phase_python() {
  phase "FAZ 4: Python paketleri"
  step "pyserial, pyyaml, numpy"
  # Sistem genelinde:
  apt_install python3-serial python3-yaml python3-numpy python3-pytest

  step "Kullanici icin yedek pip kurulum"
  python3 -m pip install --user --break-system-packages \
    pyserial pyyaml numpy pytest 2>/dev/null || \
    warn "pip kurulumu atlandi (apt'tekiler yeterli olmali)"

  step "depthai (OAK-D Lite VPU - yalniz gercek arac, sim'de gerekmez)"
  if python3 -c "import depthai" >/dev/null 2>&1; then
    ok "depthai zaten kurulu"
  elif python3 -m pip install --user --break-system-packages depthai 2>/dev/null; then
    ok "depthai kuruldu"
  else
    warn "depthai kurulamadi - sim icin gerekli degil; gercek arac icin: pip install depthai"
  fi
}

phase_udev() {
  phase "FAZ 5: udev kurallari (lidar, arduino, gps)"
  step "/etc/udev/rules.d/99-ika-usb.rules"
  sudo tee /etc/udev/rules.d/99-ika-usb.rules >/dev/null <<'RULES'
# IKA - USB seri cihazlar
# CP210x (SLAMTEC RPLIDAR C1)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ika_lidar",   MODE="0666"
# Arduino Uno (1)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="ika_arduino", MODE="0666"
# Arduino Uno (klon, CH340)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ika_arduino", MODE="0666"
# u-blox GPS (UBLOX-NEO seri)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="ika_gps",     MODE="0666"
RULES
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ok "udev kurallari yuklendi"
  warn "Gercek donanim takiliyken VID/PID kontrolu: lsusb"
}

phase_build() {
  phase "FAZ 6: Workspace build"
  if [[ ! -d "$PROJECT_ROOT/ika_ws/src" ]]; then
    err "ika_ws/src bulunamadi: $PROJECT_ROOT/ika_ws/src"
    err "Repo'yu klonlamis ve dogru dizinde calistiriyor musun?"
    return 1
  fi
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash

  step "rosdep init + update"
  if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
    sudo rosdep init 2>/dev/null || warn "rosdep init zaten yapilmis olabilir"
  fi
  rosdep update --rosdistro=jazzy
  ok "rosdep guncel"

  step "rosdep install (workspace bagimliliklari)"
  cd "$PROJECT_ROOT/ika_ws"
  rosdep install --from-paths src --ignore-src -r -y || \
    warn "rosdep install bazi paketleri bulamadi - asagidaki build bunlarsiz devam edecek"

  step "colcon build (--parallel-workers $(nproc))"
  colcon build \
    --symlink-install \
    --parallel-workers "$(nproc)" \
    --event-handlers console_cohesion+ \
    2>&1 | tee /tmp/ika_build.log
  local rc=${PIPESTATUS[0]}
  if [[ $rc -ne 0 ]]; then
    err "Build basarisiz. Detay: /tmp/ika_build.log"
    return $rc
  fi
  ok "Build OK"

  step "~/.bashrc -> install/setup.bash source"
  ensure_line "source $PROJECT_ROOT/ika_ws/install/setup.bash" "$HOME/.bashrc"
  ok "Workspace setup eklendi"

  step "Script'ler chmod +x"
  chmod +x "$SCRIPT_DIR"/*.sh
  ok "Script'ler calistirilabilir"
}

phase_verify() {
  phase "FAZ 7: Kurulum dogrulama"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash 2>/dev/null || true
  if [[ -f "$PROJECT_ROOT/ika_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/ika_ws/install/setup.bash"
  fi

  step "ROS 2 cli"
  ros2 --help >/dev/null 2>&1 && ok "ros2 cli" || err "ros2 cli BULUNAMADI"

  step "ros2 doctor (ozet)"
  ros2 doctor --report 2>&1 | head -n 30 || warn "ros2 doctor calismadi"

  step "Gazebo gz cli"
  gz --version >/dev/null 2>&1 && ok "gz cli ($(gz --version 2>&1 | head -1))" || err "gz cli BULUNAMADI"

  step "Workspace paketleri"
  for p in ika_bringup ika_description ika_simulation ika_navigation \
           ika_terrain ika_safety ika_base_controller ika_mission \
           ika_perception_dl ika_fusion ika_rl_planner; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then
      ok "ros2 pkg: $p"
    else
      err "EKSIK ros2 pkg: $p"
    fi
  done

  step "Birim testler"
  for pkg in ika_terrain ika_safety ika_base_controller \
             ika_perception_dl ika_fusion ika_rl_planner; do
    cd "$PROJECT_ROOT/ika_ws/src/$pkg" 2>/dev/null || continue
    if python3 -m pytest test/ -q 2>&1 | tail -3 | head -1; then :; fi
  done
}

# ---- main ------------------------------------------------------------
main() {
  echo -e "${M}IKA Pi kurulum scripti - faz: $PHASE${NC}"
  echo "Proje dizini : $PROJECT_ROOT"
  echo "Tarih        : $(date)"
  echo

  # sudo cache'i tut
  if ! sudo -v; then err "sudo gerekli"; exit 1; fi
  ( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) &
  SUDO_REFRESHER=$!
  trap 'kill $SUDO_REFRESHER 2>/dev/null || true' EXIT

  case "$PHASE" in
    all)
      phase_system
      phase_ros
      phase_packages
      phase_python
      phase_udev
      phase_build
      phase_verify
      ;;
    system)   phase_system ;;
    ros)      phase_ros ;;
    packages) phase_packages ;;
    python)   phase_python ;;
    udev)     phase_udev ;;
    build)    phase_build ;;
    verify)   phase_verify ;;
    *)        err "Bilinmeyen faz: $PHASE"; exit 2 ;;
  esac

  echo
  ok "Faz '$PHASE' tamamlandi."
  if [[ "$PHASE" == "all" ]]; then
    echo
    echo -e "${G}Sonraki adim:${NC}"
    echo "  source ~/.bashrc          # veya yeni terminal ac"
    echo "  ./scripts/deploy_sim.sh   # ilk sim denemesi"
    echo
    echo "Detayli denemeler icin: DENEMELER.md"
  fi
}

main "$@"
