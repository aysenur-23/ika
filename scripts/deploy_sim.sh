#!/usr/bin/env bash
# IKA - Pi tarafi tek-komut deploy + sim launch.
#
# Calistirma:
#   chmod +x scripts/deploy_sim.sh scripts/stop_sim.sh scripts/verify_sim.sh
#   ./scripts/deploy_sim.sh              # default: 'bare' simulation
#   ./scripts/deploy_sim.sh full         # sim_full (Gazebo + tum nav + safety)
#   ./scripts/deploy_sim.sh bare clean   # build artifact'lerini sil + rebuild
#
# Yaptiklari:
#   1) Build (colcon paralel, tum cekirdekler)
#   2) Sim launch arka planda baslatir, log /tmp/ika_sim.log
#   3) 15s bekler, verify_sim.sh ile akisi dogrular
#   4) PID'i /tmp/ika_sim.pid altinda birakir, exit
#
# Durdurmak icin: ./scripts/stop_sim.sh

set -u

# ---- Renkler ---------------------------------------------------------
G="\033[0;32m"; R="\033[0;31m"; Y="\033[0;33m"; B="\033[0;34m"; NC="\033[0m"
step() { echo -e "\n${B}== $1 ==${NC}"; }
ok()   { echo -e "${G}[OK]${NC} $1"; }
warn() { echo -e "${Y}[WW]${NC} $1"; }
err()  { echo -e "${R}[!!]${NC} $1"; }

# ---- Argumanlar ------------------------------------------------------
MODE="${1:-bare}"        # bare | full
CLEAN="${2:-}"           # clean | (empty)

case "$MODE" in
  bare) LAUNCH_PKG="ika_simulation"; LAUNCH_FILE="simulation.launch.py";;
  full) LAUNCH_PKG="ika_bringup";    LAUNCH_FILE="sim_full.launch.py";;
  *) err "Bilinmeyen mod: $MODE (bare|full)"; exit 2;;
esac

# ---- Pre-flight ------------------------------------------------------
step "Pre-flight kontroller"

# Workspace kokunu bul: scripts/ icindeysek bir ust
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# ika_ws/ src/ varligini kontrol et
if [[ -d "$PROJECT_ROOT/ika_ws/src" ]]; then
  WS="$PROJECT_ROOT/ika_ws"
elif [[ -d "$PROJECT_ROOT/src" ]]; then
  WS="$PROJECT_ROOT"
elif [[ -d "$HOME/ika_ws/src" ]]; then
  WS="$HOME/ika_ws"
else
  err "ika_ws/src bulunamadi. PROJECT_ROOT=$PROJECT_ROOT"
  err "Bu scripti workspace icinde veya scripts/ alt klasorunde calistir."
  exit 1
fi
ok "Workspace: $WS"

if [[ ! -f "/opt/ros/jazzy/setup.bash" ]]; then
  err "ROS 2 Jazzy bulunamadi: /opt/ros/jazzy/setup.bash"
  exit 1
fi
ok "ROS 2 Jazzy mevcut"

# Gazebo Harmonic
if ! command -v gz >/dev/null 2>&1; then
  warn "gz komutu yok - Gazebo Harmonic kurulu olmayabilir (sudo apt install ros-jazzy-ros-gz)"
fi

# Eski sim varsa once durdur
if [[ -f /tmp/ika_sim.pid ]]; then
  warn "Onceki sim hala calisiyor olabilir. Durduruluyor..."
  "$SCRIPT_DIR/stop_sim.sh" || true
fi

# ---- Build -----------------------------------------------------------
step "Build (colcon, $(nproc) cekirdek paralel)"

# shellcheck disable=SC1091
# ROS setup.bash AMENT_TRACE_SETUP_FILES gibi tanimsiz degiskenleri okur;
# script'in `set -u`'su ile catisir -> sourcing sirasinda gecici devre disi.
set +u; source /opt/ros/jazzy/setup.bash; set -u

cd "$WS"

if [[ "$CLEAN" == "clean" ]]; then
  warn "clean modu: build/ install/ log/ silinecek"
  rm -rf build install log
fi

BUILD_START=$SECONDS
if ! colcon build \
       --symlink-install \
       --parallel-workers "$(nproc)" \
       --event-handlers console_cohesion+ \
       2>&1 | tee /tmp/ika_build.log | grep -E '^(\[|Starting|Finished|Failed|Summary|ERROR)' ; then
  err "Build basarisiz. Detay: /tmp/ika_build.log"
  exit 1
fi
BUILD_TIME=$((SECONDS - BUILD_START))
ok "Build tamamlandi: ${BUILD_TIME}s"

# shellcheck disable=SC1091
set +u; source "$WS/install/setup.bash"; set -u

# ---- Sim launch (background) -----------------------------------------
step "Sim launch: ros2 launch $LAUNCH_PKG $LAUNCH_FILE (arka plan)"

: > /tmp/ika_sim.log
nohup ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
  > /tmp/ika_sim.log 2>&1 &
SIM_PID=$!
echo "$SIM_PID" > /tmp/ika_sim.pid

# pgid dosyasi: process group'u temiz oldurebilmek icin
ps -o pgid= -p "$SIM_PID" | tr -d ' ' > /tmp/ika_sim.pgid 2>/dev/null || true

ok "PID=$SIM_PID  Log=/tmp/ika_sim.log"
ok "Gazebo + bridge ayaga kalkmasi icin 15s bekleniyor..."
sleep 15

# Surec hala canli mi?
if ! kill -0 "$SIM_PID" 2>/dev/null; then
  err "Sim sureci olmus gorunuyor. Son loglar:"
  tail -n 40 /tmp/ika_sim.log
  exit 1
fi
ok "Sim sureci canli"

# ---- Verify ----------------------------------------------------------
step "Verify (topic + TF akisi)"

if [[ -x "$SCRIPT_DIR/verify_sim.sh" ]]; then
  "$SCRIPT_DIR/verify_sim.sh" || warn "verify_sim.sh bazi eksikler bildirdi (yukariya bak)"
else
  warn "verify_sim.sh bulunamadi/calistirilamadi"
fi

# ---- Ozet ------------------------------------------------------------
step "Hazir"
cat <<EOF
${G}Sim arka planda calisiyor.${NC}
  Mod      : $MODE
  PID      : $SIM_PID
  Log      : tail -f /tmp/ika_sim.log
  Build log: /tmp/ika_build.log
  Durdur   : ./scripts/stop_sim.sh

Hizli kontroller:
  ros2 topic list
  ros2 topic hz /scan
  ros2 topic hz /imu/data
  ros2 run rqt_robot_monitor rqt_robot_monitor    # diagnostics

RViz acik kalmadiysa ayri terminalde:
  source $WS/install/setup.bash
  ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix ika_description)/share/ika_description/rviz/ika_view.rviz

Hedef vermek icin (sim_full modunda):
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\
    '{header:{frame_id: "map"}, pose:{position:{x: 2.0, y: 0.5}, orientation:{w: 1.0}}}'
EOF
