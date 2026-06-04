#!/bin/bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
set -u

echo "=== /goal_pose'a abone var mi? ==="
ros2 topic info /goal_pose

echo ""
echo "=== Action server var mi? ==="
ros2 action list | head -10

echo ""
echo "=== Goal action gonder ==="
timeout 25 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback 2>&1 | tail -30
