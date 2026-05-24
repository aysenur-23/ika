"""IKA Gazebo Harmonic simulasyon launch dosyasi.

Baslat:
  - Gazebo Harmonic + test_world.sdf
  - robot_state_publisher (URDF, use_sim:=true)
  - URDF'i /robot_description -> Gazebo create
  - ros_gz_bridge (topic koprusu)
  - RViz2

Kullanim:
  ros2 launch ika_simulation simulation.launch.py
  ros2 launch ika_simulation simulation.launch.py headless:=true rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ika_sim = FindPackageShare('ika_simulation')
    ika_desc = FindPackageShare('ika_description')
    ros_gz_sim = FindPackageShare('ros_gz_sim')

    world_path = PathJoinSubstitution([ika_sim, 'worlds', 'test_world.sdf'])
    xacro_path = PathJoinSubstitution([ika_desc, 'urdf', 'ika.urdf.xacro'])
    bridge_yaml = PathJoinSubstitution([ika_sim, 'config', 'ros_gz_bridge.yaml'])
    rviz_cfg = PathJoinSubstitution([ika_desc, 'rviz', 'ika_view.rviz'])

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    robot_description = {
        'robot_description': Command([
            TextSubstitution(text='xacro '), xacro_path,
            TextSubstitution(text=' use_sim:=true'),
        ]),
        'use_sim_time': True,
    }

    # gz_args: '-r <world>' veya '-r -s --headless-rendering <world>'
    gz_args_normal = [TextSubstitution(text='-r '), world_path]
    gz_args_headless = [TextSubstitution(text='-r -s --headless-rendering '), world_path]

    gazebo_normal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args_normal}.items(),
        condition=UnlessCondition(headless),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args_headless}.items(),
        condition=IfCondition(headless),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # URDF'i Gazebo'ya spawn et
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ika',
            '-topic', '/robot_description',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        gazebo_normal,
        gazebo_headless,
        rsp,
        spawn,
        bridge,
        rviz,
    ])
