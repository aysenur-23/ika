#!/usr/bin/env bash
# IKA — Avoider modu trial repeat'er (N kere).
#
# Kullanım (WSL):
#   ./repeat_trial_avoider.sh 10                 # default debug_world
#   ./repeat_trial_avoider.sh 5 my_test.csv      # özel ad
#   WORLD=test_world OBSTACLES="3,0;5.5,0.4;8,-0.4;10.5,0;13.5,0.3;16,0" \
#     ./repeat_trial_avoider.sh 5 parkur_test.csv
#
# Çıktı CSV (TASK-2 sonrası): eski alanlar + telemetri.
#   trial_id,status,final_x,final_y,min_obs_dist,distance_clear_m,
#   avoider_phase,duration,reason,
#   finish_reached,collision,min_obstacle_distance,max_y_deviation,
#   final_y_error,state_transition_count,stuck_time,
#   cmd_vel_oscillation_score,trial_duration,pass_strict

set -u

N_TRIALS="${1:-${N_TRIALS:-10}}"
OUT_NAME="${2:-avoider_$(date +%Y%m%d_%H%M%S).csv}"
# Robot 0.25 m/s hizla 2.5m mesafeyi ~10s'de katar; engel kacinma sapma
# eklenince 20-25s yeter. Eski 45s gereksizdi.
TIMEOUT="${TIMEOUT:-30}"
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

# CSV header (TASK-2)
echo "trial_id,status,final_x,final_y,min_obs_dist,distance_clear_m,avoider_phase,duration,reason,finish_reached,collision,min_obstacle_distance,max_y_deviation,final_y_error,state_transition_count,stuck_time,cmd_vel_oscillation_score,trial_duration,pass_strict" > "$OUT"
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
    # TASK-2: FAIL_LAUNCH satırı da yeni kolonları içermeli (boş + 0).
    echo "$i,FAIL_LAUNCH,,,,,UNKNOWN,0,launch_died,0,0,,,,0,0,0,0,0" >> "$OUT"
    FAIL_CNT=$((FAIL_CNT + 1))
    continue
  fi

  PASS_X="${PASS_X:-2.5}"
  # TASK-2: trial koşumu — strict PASS=false artık exit 1 verir.
  # if/then/else, exit kodunu yutar; ileride `set -e` eklense bile batch
  # durdurulmaz. `set +e`/`set -e` KULLANILMAZ (mevcut shell opts korunur).
  TRIAL_TMP="/tmp/ika_trial_${i}.out"
  if python3 "$SCRIPT_DIR/run_trial_avoider.py" --trial-id "$i" \
        --timeout "$TIMEOUT" \
        --obstacles "$OBSTACLES" \
        --pass-x "$PASS_X" \
        > "$TRIAL_TMP" 2>&1; then
    TRIAL_RC=0
  else
    TRIAL_RC=$?
  fi

  TRIAL_OUT=$(tail -n 1 "$TRIAL_TMP" 2>/dev/null || true)
  TRIAL_OUT="${TRIAL_OUT%$'\r'}"
  TRIAL_OUT="$(echo -n "$TRIAL_OUT" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"

  # 19 kolon doğrulaması — Python csv ile (split-on-comma değil).
  # Geçersizse 0, geçerli ise 1 basılır.
  TRIAL_VALID=$(python3 - "$TRIAL_OUT" <<'PY'
import csv, io, sys
line = sys.argv[1] if len(sys.argv) > 1 else ""
if not line.strip():
    print(0); sys.exit(0)
try:
    row = next(csv.reader(io.StringIO(line)))
except Exception:
    print(0); sys.exit(0)
print(1 if len(row) == 19 else 0)
PY
)

  if [[ "$TRIAL_VALID" != "1" ]]; then
    err "Trial $i: çıktı 19 kolon değil (rc=$TRIAL_RC). Son 20 satır log:"
    tail -n 20 "$TRIAL_TMP" 2>/dev/null || true
    TRIAL_OUT="$i,FAIL_TRIAL,,,,,UNKNOWN,0,invalid_output,0,0,,,,0,0,0,0,0"
  fi

  echo "$TRIAL_OUT" >> "$OUT"
  echo "  -> $TRIAL_OUT (rc=$TRIAL_RC)"

  # TASK-2: PASS sayımı CSV'nin son sütununa (pass_strict) göre — exit kodu DEĞİL.
  LAST_COL="${TRIAL_OUT##*,}"
  LAST_COL="${LAST_COL%$'\r'}"
  LAST_COL="$(echo -n "$LAST_COL" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
  if [[ "$LAST_COL" == "1" ]]; then
    PASS_CNT=$((PASS_CNT + 1))
  else
    FAIL_CNT=$((FAIL_CNT + 1))
  fi

  "$STOP_SH" >/dev/null 2>&1 || true
  sleep "$BETWEEN_DELAY"
done

step "Özet (strict)"
TOTAL=$((PASS_CNT + FAIL_CNT))
echo "  STRICT PASS : $PASS_CNT / $TOTAL"
echo "  FAIL        : $FAIL_CNT / $TOTAL"
if [[ "$TOTAL" -gt 0 ]]; then
  PCT=$(awk "BEGIN{printf \"%.1f\", 100 * $PASS_CNT / $TOTAL}")
  echo "  Oran : %$PCT"
fi
echo "  CSV  : $OUT"
