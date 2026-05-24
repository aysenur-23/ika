"""Gazebo + tum yazilim katmanlari (navigation + terrain + safety).

Gercek araci taklit eden tam sistem simulasyonu. Donanim disinda her sey calisir.

Onemli: Simde Gazebo diff_drive plugin /cmd_vel topic'ini dinler.
Bu launch /cmd_vel_safe -> /cmd_vel remap'i yaparak tum guvenlik
zincirini Gazebo'ya bagar.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare('ika_simulation')
    nav_pkg = FindPackageShare('ika_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Gazebo + URDF + bridge + rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim_pkg, 'launch', 'simulation.launch.py'])),
        ),

        # Tum navigation + terrain + safety stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav_pkg, 'launch', 'navigation.launch.py'])),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # Safety chain'in cikisini Gazebo'nun cmd_vel'ine baglamak icin
        # /cmd_vel_safe -> /cmd_vel ucuncu parti rolu
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_safe', '/cmd_vel'],
            parameters=[{'use_sim_time': True}],
        ),
    ])
