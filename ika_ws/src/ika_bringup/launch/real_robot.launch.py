"""Gercek araci tam baslatma launch'i.

Akis:
  1) robot_state_publisher (URDF)
  2) Sensor surucu node'lari (lidar, kamera, GPS)
  3) Base controller (Arduino seri kopru)
  4) Navigation stack (rf2o + EKF + SLAM + Nav2 + terrain + safety)
  5) RViz

Manuel kontrol icin teleop_twist_keyboard ayri terminalde calistirilabilir;
ancak guvenlik zincirinden gecmesi icin /cmd_vel_nav'a yayinlamali veya
safety chain'i bypass eden test launch'i kullanilmali.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare('ika_description')
    bringup_pkg = FindPackageShare('ika_bringup')
    nav_pkg = FindPackageShare('ika_navigation')

    xacro_path = PathJoinSubstitution([desc_pkg, 'urdf', 'ika.urdf.xacro'])
    robot_yaml = PathJoinSubstitution([bringup_pkg, 'config', 'robot_params.yaml'])
    rviz_cfg = PathJoinSubstitution([bringup_pkg, 'rviz', 'ika_full.rviz'])

    # Jazzy: Command(...) URDF ciktisini ParameterValue ile string olarak ver.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path, ' use_sim:=false']),
            value_type=str,
        ),
        'use_sim_time': False,
    }

    return LaunchDescription([
        DeclareLaunchArgument('with_rviz', default_value='true'),

        # URDF
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[robot_description],
        ),

        # Sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_pkg, 'launch', 'sensors.launch.py'])
            ),
        ),

        # Base controller (Arduino kopru)
        Node(
            package='ika_base_controller', executable='base_controller_node',
            name='base_controller', output='screen',
            parameters=[robot_yaml, {'use_sim_time': False}],
        ),

        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav_pkg, 'launch', 'navigation.launch.py'])
            ),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # RViz
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': False}],
            output='log',
        ),
    ])
