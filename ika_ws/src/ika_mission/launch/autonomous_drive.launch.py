"""IKA — Tam otonom reaktif engel kacinma launch (plain Node).

Calistirma:
    ros2 launch ika_mission autonomous_drive.launch.py

obstacle_avoider plain Node oldugu icin lifecycle_manager gerekmez.
on init + start_delay sonra otomatik DRIVING phase'e gecer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='ika_mission', executable='obstacle_avoider',
            name='obstacle_avoider', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'forward_speed_mps': 0.20,
                'turn_speed_rps': 0.5,
                'obstacle_distance_m': 0.80,
                'front_arc_deg': 60.0,
                'target_distance_m': 2.0,
                'yaw_tolerance_rad': 0.05,
                'control_rate_hz': 20.0,
                'start_delay_s': 3.0,
            }],
        ),
    ])
