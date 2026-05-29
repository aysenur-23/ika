"""Navigasyon modu: rf2o + EKF + SLAM-Toolbox + Nav2 + custom node'lar.

SLAM modu argumana bagli:
  slam_mode:=mapping       (varsayilan, ilk koşum/sim icin)
  slam_mode:=localization  (mevcut harita uzerinde)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_pkg = FindPackageShare('ika_navigation')
    terrain_pkg = FindPackageShare('ika_terrain')
    safety_pkg = FindPackageShare('ika_safety')
    dl_pkg = FindPackageShare('ika_perception_dl')
    fusion_pkg = FindPackageShare('ika_fusion')

    rf2o_yaml = PathJoinSubstitution([nav_pkg, 'config', 'rf2o_params.yaml'])
    ekf_yaml = PathJoinSubstitution([nav_pkg, 'config', 'ekf_params.yaml'])
    slam_yaml = PathJoinSubstitution([nav_pkg, 'config', 'slam_params.yaml'])
    nav2_yaml = PathJoinSubstitution([nav_pkg, 'config', 'nav2_params.yaml'])
    mppi_yaml = PathJoinSubstitution([nav_pkg, 'config', 'mppi_controller.yaml'])
    terrain_yaml = PathJoinSubstitution([terrain_pkg, 'config', 'terrain_params.yaml'])
    safety_yaml = PathJoinSubstitution([safety_pkg, 'config', 'safety_params.yaml'])
    dl_yaml = PathJoinSubstitution([dl_pkg, 'config', 'dl_params.yaml'])
    fusion_yaml = PathJoinSubstitution([fusion_pkg, 'config', 'fusion_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode = LaunchConfiguration('slam_mode')
    local_planner = LaunchConfiguration('local_planner')

    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'collision_monitor',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description="'mapping' (varsayilan) veya 'localization'"),
        DeclareLaunchArgument(
            'local_planner', default_value='dwb',
            description="Yerel planlayici: 'dwb' (klasik) veya 'mppi' (tez karsilastirmasi)"),

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

        # SLAM Toolbox - mapping modu (varsayilan, ilk sim koşumu)
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            condition=LaunchConfigurationEquals('slam_mode', 'mapping'),
            parameters=[slam_yaml, {'use_sim_time': use_sim_time, 'mode': 'mapping'}],
        ),
        # SLAM Toolbox - localization modu (mevcut harita uzerinde)
        Node(
            package='slam_toolbox', executable='localization_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            condition=LaunchConfigurationEquals('slam_mode', 'localization'),
            parameters=[slam_yaml, {'use_sim_time': use_sim_time, 'mode': 'localization'}],
        ),

        # Nav2 core
        # controller_server - klasik DWB (varsayilan)
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             condition=LaunchConfigurationEquals('local_planner', 'dwb'),
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        # controller_server - MPPI (tez karsilastirmasi). mppi_yaml, nav2_yaml
        # uzerine yuklenip FollowPath bloğunu DWB -> MPPI olarak ezer.
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             condition=LaunchConfigurationEquals('local_planner', 'mppi'),
             parameters=[nav2_yaml, mppi_yaml, {'use_sim_time': use_sim_time}]),
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

        # IKA ozel node'lari (veri akisi: terrain + DL -> fusion -> safety)
        Node(package='ika_terrain', executable='terrain_perception_node',
             name='terrain_perception', output='screen',
             parameters=[terrain_yaml, {'use_sim_time': use_sim_time}]),
        # DL nesne tespiti (OAK-D VPU). depthai/kamera yoksa fail-safe no-op
        # (orn. sim'de) - lifecycle aktivasyonunu bloklamaz.
        Node(package='ika_perception_dl', executable='dl_perception_node',
             name='dl_perception', output='screen',
             parameters=[dl_yaml, {'use_sim_time': use_sim_time}]),
        # Hibrit fuzyon: terrain + DL -> /hazard_state + /detection_obstacles
        Node(package='ika_fusion', executable='fusion_node',
             name='hazard_fusion', output='screen',
             parameters=[fusion_yaml, {'use_sim_time': use_sim_time}]),
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
                'node_names': [
                    'terrain_perception',
                    'dl_perception',
                    'hazard_fusion',
                    'safety_supervisor',
                ],
            }],
        ),
    ])
