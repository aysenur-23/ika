"""URDF'i robot_state_publisher ile yayimla, RViz acversiyon (sim disi).

Kullanim:
  ros2 launch ika_description display.launch.py
  ros2 launch ika_description display.launch.py use_sim:=true rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('ika_description')
    xacro_path = PathJoinSubstitution([pkg, 'urdf', 'ika.urdf.xacro'])

    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('rviz')

    # Jazzy: Command(...) URDF ciktisini ParameterValue ile string olarak ver.
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                TextSubstitution(text='xacro '), xacro_path,
                TextSubstitution(text=' use_sim:='), use_sim,
            ]),
            value_type=str,
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim}],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'ika_view.rviz'])],
            condition=IfCondition(use_rviz),
        ),
    ])
