#!/bin/bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
set -u

echo "=== ROS Nodes ==="
ros2 node list 2>&1 | sort | head -30

echo ""
echo "=== Lifecycle States ==="
for n in /slam_toolbox /controller_server /planner_server /behavior_server /bt_navigator /safety_supervisor /hazard_fusion /terrain_perception; do
    printf "%-25s " "$n"
    ros2 lifecycle get "$n" 2>&1 | head -1
done

echo ""
echo "=== /map publisher ==="
ros2 topic info /map 2>&1 | head -5

echo ""
echo "=== /scan hz (3s) ==="
timeout 3 ros2 topic hz /scan 2>&1 | tail -2
