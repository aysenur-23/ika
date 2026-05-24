#!/usr/bin/env bash
# IKA - Sim dogrulama yardimcisi.
# Gazebo + bridge calistiktan SONRA bu scripti calistir; her sey akiyor mu kontrol et.
#
# Kullanim:
#   chmod +x scripts/verify_sim.sh
#   ./scripts/verify_sim.sh

set -u

GREEN="\033[0;32m"; RED="\033[0;31m"; YEL="\033[0;33m"; NC="\033[0m"

step() { echo -e "${YEL}== $1 ==${NC}"; }
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
bad()  { echo -e "${RED}[!! ]${NC} $1"; }

require() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    bad "$cmd bulunamadi"; exit 1
  fi
}

require ros2
require gz

step "Gazebo topic listesi"
GZ_TOPICS=$(gz topic -l 2>/dev/null || true)
echo "$GZ_TOPICS"
for t in /clock /scan /imu/data /gps/fix /oak/points /oak/depth_image /sim/odom /joint_states; do
  if echo "$GZ_TOPICS" | grep -qx "$t"; then ok "gz: $t"; else bad "gz EKSIK: $t"; fi
done

step "ROS topic listesi"
ROS_TOPICS=$(ros2 topic list 2>/dev/null || true)
echo "$ROS_TOPICS"
for t in /clock /scan /imu/data /gps/fix /oak/points /oak/depth/image_raw /odom_truth /joint_states /tf /tf_static /robot_description; do
  if echo "$ROS_TOPICS" | grep -qx "$t"; then ok "ros: $t"; else bad "ros EKSIK: $t"; fi
done

step "ROS topic hz olcumleri (3sn)"
for t in /scan /imu/data /oak/points; do
  echo -n "  $t -> "
  timeout 3 ros2 topic hz "$t" 2>/dev/null | tail -n 2 | head -n 1 || bad "hz okunamadi"
done

step "TF agaci (5sn)"
timeout 5 ros2 run tf2_tools view_frames 2>/dev/null && ok "frames.pdf uretildi (mevcut dizinde)" || bad "view_frames basarisiz"

step "Lifecycle node'lari"
ros2 lifecycle nodes 2>/dev/null || bad "lifecycle nodes alinamadi"

echo
echo "Bitti. Eksik [!!] satirlari varsa bridge config veya gz sensor tanimi kontrol edilmeli."
