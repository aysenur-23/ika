#!/usr/bin/env bash
# IKA Faz 0 — N kere debug_world senaryosunu koş, baseline.csv'ye yaz.
#
# Kullanım (WSL Ubuntu-24.04 içinde, ~/ika kökünde):
#   chmod +x benchmarks/debug_scenario/repeat_trial.sh
#   ./benchmarks/debug_scenario/repeat_trial.sh 10              # N=10 trial
#   ./benchmarks/debug_scenario/repeat_trial.sh 10 baseline.csv # özel ad
#   N_TRIALS=5 TIMEOUT=90 ./benchmarks/debug_scenario/repeat_trial.sh
#
# Her trial:
#   1) sim_full.launch.py headless=true rviz=false world=debug_world  (arka plan)
#   2) 25 s bekle (Nav2 + SLAM aktif olsun)
#   3) run_trial.py --trial-id N --timeout T
#   4) stop_sim.sh
#   5) 3 s temizlik beklemesi
#
# Çıktı: benchmarks/debug_scenario/results/baseline_<timestamp>.csv
#   trial_id,status,final_x,final_y,dist_to_goal,min_obs_dist,duration,nav2_result

set -u

N_TRIALS="${1:-${N_TRIALS:-10}}"
OUT_NAME="${2:-baseline_$(date +%Y%m%d_%H%M%S).csv}"
TIMEOUT="${TIMEOUT:-60}"
# run_trial.py'da aktif probe var; bu sleep sadece launch process'lerinin
# subprocess.spawn olmasını + ROS daemon registration'ını kolaylaştırır.
# Asıl ready-check Python tarafında.
READY_WAIT="${READY_WAIT:-15}"
BETWEEN_DELAY="${BETWEEN_DELAY:-3}"

# ---- Yol kurulumu ----
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"
RESULTS_DIR="$SCRIPT_DIR/results"
mkdir -p "$RESULTS_DIR"
OUT="$RESULTS_DIR/$OUT_NAME"

# ROS source
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ -f "$PROJECT_ROOT/ika_ws/install/setup.bash" ]]; then
  set +u; source "$PROJECT_ROOT/ika_ws/install/setup.bash"; set -u
elif [[ -f "$HOME/ika/ika_ws/install/setup.bash" ]]; then
  set +u; source "$HOME/ika/ika_ws/install/setup.bash"; set -u
else
  echo "[!!] Workspace install/setup.bash bulunamadı" >&2
  exit 1
fi

G="\033[0;32m"; R="\033[0;31m"; Y="\033[0;33m"; B="\033[0;34m"; NC="\033[0m"
step() { echo -e "\n${B}== $1 ==${NC}"; }
ok()   { echo -e "${G}[OK]${NC} $1"; }
warn() { echo -e "${Y}[WW]${NC} $1"; }
err()  { echo -e "${R}[!!]${NC} $1"; }

STOP_SH="$PROJECT_ROOT/scripts/stop_sim.sh"
[[ -x "$STOP_SH" ]] || STOP_SH="$HOME/ika/scripts/stop_sim.sh"

# CSV header
echo "trial_id,status,final_x,final_y,dist_to_goal,min_obs_dist,duration,nav2_result" > "$OUT"
ok "Çıktı: $OUT"

# ---- Trial döngüsü ----
PASS_CNT=0
FAIL_CNT=0
for i in $(seq 1 "$N_TRIALS"); do
  step "Trial $i / $N_TRIALS"

  # Eski sim varsa temizle
  "$STOP_SH" >/dev/null 2>&1 || true
  sleep 1

  # Launch (background) — KRITIK: setsid ile YENI PGID oluştur.
  # Aksi halde launch script'le aynı PGID'yi paylaşır; stop_sim.sh
  # `kill -INT -- -PGID` çağırınca script'in KENDİSİ de ölür ve
  # döngü trial 1'den sonra durur. setsid bunu engeller.
  SIM_LOG="/tmp/ika_trial_${i}.log"
  : > "$SIM_LOG"
  # LAUNCH_EXTRA env var ile ablasyon launch arg'leri eklenebilir:
  # LAUNCH_EXTRA="bypass_collision_monitor:=true" ./repeat_trial.sh ...
  LAUNCH_EXTRA="${LAUNCH_EXTRA:-}"
  # LOCAL_PLANNER env: dwb (default) veya mppi
  LOCAL_PLANNER="${LOCAL_PLANNER:-dwb}"
  setsid bash -c "exec ros2 launch ika_bringup sim_full.launch.py \
        headless:=true rviz:=false world:=debug_world \
        local_planner:=$LOCAL_PLANNER $LAUNCH_EXTRA \
        > '$SIM_LOG' 2>&1" < /dev/null &
  SIM_PID=$!
  echo "$SIM_PID" > /tmp/ika_sim.pid
  # setsid'den sonra child kendi PGID'sine sahip; pgid=pid olur
  echo "$SIM_PID" > /tmp/ika_sim.pgid

  ok "Launch PID=$SIM_PID, hazırlık $READY_WAIT s bekleniyor..."
  sleep "$READY_WAIT"

  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    err "Sim launch öldü. Son log:"
    tail -n 30 "$SIM_LOG"
    echo "$i,FAIL_LAUNCH,nan,nan,nan,-1,0,launch_died" >> "$OUT"
    FAIL_CNT=$((FAIL_CNT + 1))
    continue
  fi

  # Trial koşumu (COLLISION_THRESHOLD/GOAL_X/GOAL_Y env ile override edilebilir)
  COLL_THRESH="${COLLISION_THRESHOLD:-0.05}"
  GOAL_X="${GOAL_X:-3.0}"
  GOAL_Y="${GOAL_Y:-0.0}"
  TRIAL_OUT=$(python3 "$SCRIPT_DIR/run_trial.py" --trial-id "$i" \
              --timeout "$TIMEOUT" --collision-threshold "$COLL_THRESH" \
              --goal-x "$GOAL_X" --goal-y "$GOAL_Y" \
              2>&1 | tail -n 1)
  echo "$TRIAL_OUT" >> "$OUT"
  echo "  -> $TRIAL_OUT"
  if [[ "$TRIAL_OUT" == *",PASS,"* ]]; then
    PASS_CNT=$((PASS_CNT + 1))
  else
    FAIL_CNT=$((FAIL_CNT + 1))
  fi

  # Kapat
  "$STOP_SH" >/dev/null 2>&1 || true
  sleep "$BETWEEN_DELAY"
done

# ---- Özet ----
step "Özet"
TOTAL=$((PASS_CNT + FAIL_CNT))
echo "  PASS : $PASS_CNT / $TOTAL"
echo "  FAIL : $FAIL_CNT / $TOTAL"
if [[ "$TOTAL" -gt 0 ]]; then
  PCT=$(awk "BEGIN{printf \"%.1f\", 100 * $PASS_CNT / $TOTAL}")
  echo "  Oran : %$PCT"
fi
echo "  CSV  : $OUT"
