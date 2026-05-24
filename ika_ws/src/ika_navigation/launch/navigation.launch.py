"""Navigasyon modu: rf2o + EKF + SLAM-Toolbox(localization) + Nav2 + custom node'lar."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_pkg = FindPackageShare('ika_navigation')
    terrain_pkg = FindPackageShare('ika_terrain')
    safety_pkg = FindPackageShare('ika_safety')

    rf2o_yaml = PathJoinSubstitution([nav_pkg, 'config', 'rf2o_params.yaml'])
    ekf_yaml = PathJoinSubstitution([nav_pkg, 'config', 'ekf_params.yaml'])
    slam_yaml = PathJoinSubstitution([nav_pkg, 'config', 'slam_params.yaml'])
    nav2_yaml = PathJoinSubstitution([nav_pkg, 'config', 'nav2_params.yaml'])
    terrain_yaml = PathJoinSubstitution([terrain_pkg, 'config', 'terrain_params.yaml'])
    safety_yaml = PathJoinSubstitution([safety_pkg, 'config', 'safety_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'collision_monitor',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Lidar odom
        Node(
            package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry', output='screen',
            parameters=[rf2o_yaml, {'use_sim_time': use_sim_time}],
        ),

        # EKF + GPS donusumu
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', output='screen',
            parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),

        # SLAM Toolbox - localization modu (mevcut harita uzerinde)
        Node(
            package='slam_toolbox', executable='localization_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[slam_yaml, {'use_sim_time': use_sim_time, 'mode': 'localization'}],
        ),

        # Nav2 core
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen',
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),

        # Nav2 lifecycle
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': nav2_lifecycle_nodes,
            }],
        ),

        # IKA ozel node'lari
        Node(package='ika_terrain', executable='terrain_perception_node',
             name='terrain_perception', output='screen',
             parameters=[terrain_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='ika_safety', executable='safety_supervisor_node',
             name='safety_supervisor', output='screen',
             parameters=[safety_yaml, {'use_sim_time': use_sim_time}]),

        # IKA lifecycle manager
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_ika', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['terrain_perception', 'safety_supervisor'],
            }],
        ),
    ])
