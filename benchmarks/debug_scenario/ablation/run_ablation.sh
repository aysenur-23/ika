#!/usr/bin/env bash
# Faz 2 ablasyon orchestrator.
#
# Kullanım (WSL):
#   ./run_ablation.sh A2.1 10
#   ./run_ablation.sh A2.2 10
#   ./run_ablation.sh A2.3 10
#   ./run_ablation.sh A2.4 10        # baseline (yama yok)
#
# Yaptığı:
#   1) patch_nav2_params.py apply <name>  — nav2_params.yaml backup + edit
#   2) colcon build --packages-select ika_navigation (yeni params install/'a kopyalansin)
#   3) bypass_collision_monitor LAUNCH_EXTRA'sı A2.2/A2.3 için aktif
#   4) repeat_trial.sh N ablation_<name>_nN.csv
#   5) HER ZAMAN restore (trap)

set -u

NAME="${1:?Kullanim: $0 <A2.1|A2.2|A2.3|A2.4> <N>}"
N="${2:?Kullanim: $0 <name> <N>}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DEBUG_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/../../.." && pwd )"

G="\033[0;32m"; R="\033[0;31m"; Y="\033[0;33m"; B="\033[0;34m"; NC="\033[0m"
step() { echo -e "\n${B}== $1 ==${NC}"; }
ok()   { echo -e "${G}[OK]${NC} $1"; }
err()  { echo -e "${R}[!!]${NC} $1"; }

# Always restore on exit
restore_on_exit() {
  step "RESTORE — nav2_params.yaml geri yaz"
  python3 "$SCRIPT_DIR/patch_nav2_params.py" restore || err "Restore basarisiz!"
  (cd "$PROJECT_ROOT/ika_ws" && colcon build --packages-select ika_navigation \
    --symlink-install --event-handlers console_cohesion- 2>&1 \
    | tail -3 || err "Restore sonrası build başarısız!")
}
trap restore_on_exit EXIT

# ---- ROS source ----
set +u; source /opt/ros/jazzy/setup.bash; set -u
[[ -f "$PROJECT_ROOT/ika_ws/install/setup.bash" ]] \
  && { set +u; source "$PROJECT_ROOT/ika_ws/install/setup.bash"; set -u; }

# ---- 1. Patch ----
step "1. Yama uygula: $NAME"
python3 "$SCRIPT_DIR/patch_nav2_params.py" apply "$NAME"

# ---- 2. Rebuild ----
step "2. ika_navigation rebuild (yeni params install/ icin)"
cd "$PROJECT_ROOT/ika_ws"
colcon build --packages-select ika_navigation --symlink-install \
  --event-handlers console_cohesion- 2>&1 | tail -3

set +u; source "$PROJECT_ROOT/ika_ws/install/setup.bash"; set -u

# ---- 3. Launch extra args ----
case "$NAME" in
  A2.2|A2.3)
    export LAUNCH_EXTRA="bypass_collision_monitor:=true"
    ok "LAUNCH_EXTRA=$LAUNCH_EXTRA"
    ;;
  *)
    export LAUNCH_EXTRA=""
    ;;
esac

# ---- 4. Trials ----
step "4. Trials: N=$N"
OUT_CSV="ablation_${NAME}_n${N}_$(date +%H%M%S).csv"
bash "$DEBUG_DIR/repeat_trial.sh" "$N" "$OUT_CSV"

ok "Bitti: $DEBUG_DIR/results/$OUT_CSV"
