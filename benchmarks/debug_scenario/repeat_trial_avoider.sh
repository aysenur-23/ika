#!/usr/bin/env bash
# IKA — Avoider modu trial repeat'er (N kere).
#
# Kullanım (WSL):
#   ./repeat_trial_avoider.sh 10                 # default debug_world
#   ./repeat_trial_avoider.sh 5 my_test.csv      # özel ad
#   WORLD=test_world OBSTACLES="3,0;5.5,0.4;8,-0.4;10.5,0;13.5,0.3;16,0" \
#     ./repeat_trial_avoider.sh 5 parkur_test.csv
#
# Çıktı CSV: trial_id,status,final_x,final_y,min_obs_dist,distance_clear_m,
#            avoider_phase,duration,reason

set -u

N_TRIALS="${1:-${N_TRIALS:-10}}"
OUT_NAME="${2:-avoider_$(date +%Y%m%d_%H%M%S).csv}"
TIMEOUT="${TIMEOUT:-45}"
READY_WAIT="${READY_WAIT:-15}"
BETWEEN_DELAY="${BETWEEN_DELAY:-3}"
WORLD="${WORLD:-debug_world}"
OBSTACLES="${OBSTACLES:-1.5,0.0}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"
RESULTS_DIR="$SCRIPT_DIR/results"
mkdir -p "$RESULTS_DIR"
OUT="$RESULTS_DIR/$OUT_NAME"

set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ -f "$PROJECT_ROOT/ika_ws/install/setup.bash" ]]; then
  set +u; source "$PROJECT_ROOT/ika_ws/install/setup.bash"; set -u
elif [[ -f "$HOME/ika/ika_ws/install/setup.bash" ]]; then
  set +u; source "$HOME/ika/ika_ws/install/setup.bash"; set -u
fi

G="\033[0;32m"; R="\033[0;31m"; Y="\033[0;33m"; B="\033[0;34m"; NC="\033[0m"
step() { echo -e "\n${B}== $1 ==${NC}"; }
ok()   { echo -e "${G}[OK]${NC} $1"; }
err()  { echo -e "${R}[!!]${NC} $1"; }

STOP_SH="$PROJECT_ROOT/scripts/stop_sim.sh"
[[ -x "$STOP_SH" ]] || STOP_SH="$HOME/ika/scripts/stop_sim.sh"

# CSV header
echo "trial_id,status,final_x,final_y,min_obs_dist,distance_clear_m,avoider_phase,duration,reason" > "$OUT"
ok "Çıktı: $OUT"
ok "WORLD=$WORLD  OBSTACLES=$OBSTACLES  TIMEOUT=${TIMEOUT}s"

PASS_CNT=0
FAIL_CNT=0
for i in $(seq 1 "$N_TRIALS"); do
  step "Trial $i / $N_TRIALS"

  "$STOP_SH" >/dev/null 2>&1 || true
  sleep 1

  SIM_LOG="/tmp/ika_avoider_trial_${i}.log"
  : > "$SIM_LOG"
  setsid bash -c "exec ros2 launch ika_bringup sim_full.launch.py \
        headless:=true rviz:=false world:=$WORLD \
        autonomous_mode:=avoider \
        > '$SIM_LOG' 2>&1" < /dev/null &
  SIM_PID=$!
  echo "$SIM_PID" > /tmp/ika_sim.pid
  echo "$SIM_PID" > /tmp/ika_sim.pgid

  ok "Launch PID=$SIM_PID, hazırlık $READY_WAIT s bekleniyor..."
  sleep "$READY_WAIT"

  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    err "Sim öldü. Son log:"
    tail -n 30 "$SIM_LOG"
    echo "$i,FAIL_LAUNCH,nan,nan,-1,0,UNKNOWN,0,launch_died" >> "$OUT"
    FAIL_CNT=$((FAIL_CNT + 1))
    continue
  fi

  TRIAL_OUT=$(python3 "$SCRIPT_DIR/run_trial_avoider.py" --trial-id "$i" \
              --timeout "$TIMEOUT" \
              --obstacles "$OBSTACLES" \
              2>&1 | tail -n 1)
  echo "$TRIAL_OUT" >> "$OUT"
  echo "  -> $TRIAL_OUT"
  if [[ "$TRIAL_OUT" == *",PASS,"* ]]; then
    PASS_CNT=$((PASS_CNT + 1))
  else
    FAIL_CNT=$((FAIL_CNT + 1))
  fi

  "$STOP_SH" >/dev/null 2>&1 || true
  sleep "$BETWEEN_DELAY"
done

step "Özet"
TOTAL=$((PASS_CNT + FAIL_CNT))
echo "  PASS : $PASS_CNT / $TOTAL"
echo "  FAIL : $FAIL_CNT / $TOTAL"
if [[ "$TOTAL" -gt 0 ]]; then
  PCT=$(awk "BEGIN{printf \"%.1f\", 100 * $PASS_CNT / $TOTAL}")
  echo "  Oran : %$PCT"
fi
echo "  CSV  : $OUT"
