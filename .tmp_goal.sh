#!/bin/bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
set -u

echo "=== Goal gonderiliyor ==="
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

echo ""
echo "=== 15 saniye boyunca /plan akıyor mu (planner_server cikisi) ==="
timeout 15 ros2 topic hz /plan 2>&1 | tail -3

echo ""
echo "=== /cmd_vel hz (avoider/nav2 cikisi) ==="
timeout 5 ros2 topic hz /cmd_vel 2>&1 | tail -3

echo ""
echo "=== Robot konum (5 sn icinde 3 olcum) ==="
for i in 1 2 3; do
    timeout 3 ros2 topic echo --once /odom 2>&1 | grep -A4 "position:" | head -5
    sleep 2
done

echo ""
echo "=== Map size ==="
timeout 5 ros2 topic echo --once /map 2>&1 | head -20 | grep -E "width|height|resolution"
