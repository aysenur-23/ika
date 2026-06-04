#!/bin/bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
set -u

# Sim'i temiz arka planda baslat
exec ros2 launch ika_bringup sim_full.launch.py > /tmp/sim_test.log 2>&1
