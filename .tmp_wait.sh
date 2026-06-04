#!/bin/bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
set -u

START=$(date +%s)
for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
    elapsed=$(($(date +%s) - START))
    bt=$(ros2 lifecycle get /bt_navigator 2>&1 | head -1)
    slam=$(ros2 lifecycle get /slam_toolbox 2>&1 | head -1)
    plan=$(ros2 lifecycle get /planner_server 2>&1 | head -1)
    printf "%03ds  slam=%-25s  planner=%-25s  bt_nav=%s\n" "$elapsed" "$slam" "$plan" "$bt"
    if [ "$bt" = "active [3]" ]; then
        echo "READY"
        break
    fi
    sleep 10
done

echo ""
echo "=== Final Lifecycle ==="
for n in /slam_toolbox /controller_server /planner_server /behavior_server /bt_navigator /safety_supervisor /hazard_fusion /terrain_perception; do
    printf "%-25s " "$n"
    ros2 lifecycle get "$n" 2>&1 | head -1
done
echo ""
echo "=== /map publisher ==="
ros2 topic info /map 2>&1 | head -5
echo ""
echo "=== /scan hz (4s) ==="
timeout 4 ros2 topic hz /scan 2>&1 | tail -2
