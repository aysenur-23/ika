#!/usr/bin/env bash
# DIKKAT: Guvenlik zincirini bypass eder. Yalniz simde test icin.
# Klavye -> /cmd_vel (Gazebo direkt)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
