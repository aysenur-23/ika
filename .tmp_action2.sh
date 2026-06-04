#!/bin/bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
set -u

echo "=== Goal: (1.5, 0) - yakin, lidar gormus alan ==="
timeout 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback 2>&1 | tail -35
