#!/usr/bin/env bash
# Manuel kontrol - guvenlik zincirini AT-LA-MA-DAN.
# Klavye -> /cmd_vel_nav -> Collision Monitor -> Safety Sup -> /cmd_vel_safe -> arac
#
# Kullanim: ./scripts/teleop_safe.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_nav
