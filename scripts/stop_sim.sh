#!/usr/bin/env bash
# IKA - Sim ve baglantili surecleri temiz kapatir.
#
#   - /tmp/ika_sim.pgid icindeki process group'a SIGINT
#   - 3sn bekle, hala canliysa SIGTERM, sonra SIGKILL
#   - Yetim Gazebo (gz sim) sureclerini de toplar

set -u

G="\033[0;32m"; R="\033[0;31m"; Y="\033[0;33m"; NC="\033[0m"
ok()   { echo -e "${G}[OK]${NC} $1"; }
warn() { echo -e "${Y}[WW]${NC} $1"; }
err()  { echo -e "${R}[!!]${NC} $1"; }

PGID_FILE=/tmp/ika_sim.pgid
PID_FILE=/tmp/ika_sim.pid

kill_pg() {
  local pgid="$1" sig="$2"
  kill -s "$sig" -- "-${pgid}" 2>/dev/null || return 1
}

if [[ -f "$PGID_FILE" ]]; then
  PGID=$(cat "$PGID_FILE")
  if [[ -n "$PGID" ]]; then
    if kill_pg "$PGID" INT; then
      ok "Process group $PGID -> SIGINT gonderildi"
      sleep 3
      if kill -0 -- "-${PGID}" 2>/dev/null; then
        kill_pg "$PGID" TERM && warn "Hala canli, SIGTERM"
        sleep 2
      fi
      if kill -0 -- "-${PGID}" 2>/dev/null; then
        kill_pg "$PGID" KILL && warn "Hala canli, SIGKILL"
      fi
    else
      warn "Process group bulunamadi: $PGID"
    fi
  fi
  rm -f "$PGID_FILE"
fi

if [[ -f "$PID_FILE" ]]; then
  PID=$(cat "$PID_FILE")
  if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
    kill -INT "$PID" 2>/dev/null || true
    sleep 1
    kill -TERM "$PID" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
fi

# Yetim Gazebo surecleri
for pat in 'gz sim' 'gz-sim' 'ruby.*gz' 'ros_gz_bridge' 'parameter_bridge' 'rviz2'; do
  pkill -INT -f "$pat" 2>/dev/null && warn "pkill -INT $pat" || true
done
sleep 2
for pat in 'gz sim' 'gz-sim' 'ruby.*gz' 'ros_gz_bridge' 'parameter_bridge' 'rviz2'; do
  pkill -KILL -f "$pat" 2>/dev/null && warn "pkill -KILL $pat" || true
done

# ROS daemon'u da temizle (sonraki sim icin temiz baslangic)
ros2 daemon stop >/dev/null 2>&1 || true

ok "Temiz."
