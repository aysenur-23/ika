"""Navigasyon modu: rf2o + EKF + SLAM-Toolbox + Nav2 + custom node'lar.

SLAM modu argumana bagli:
  slam_mode:=mapping       (varsayilan, ilk koşum/sim icin)
  slam_mode:=localization  (mevcut harita uzerinde)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    enable_nav2 = LaunchConfiguration('enable_nav2')

    # NEDEN 2 AYRI LIFECYCLE_MANAGER:
    # slam_toolbox aktif olduktan sonra map_frame transform yayinlamasi
    # icin 3-5 sn lazim. lifecycle_manager autostart hemen siradakini
    # aktive eder, controller_server bu siralamada map_frame bulamayip
    # CONFIGURE_FAILURE'a duser ve lifecycle_manager zinciri durdurur.
    # Cozum: slam icin ayri lifecycle_manager (hemen baslar), Nav2 icin
    # ayri (TimerAction ile 30 sn gecikmeli baslar; slam'in scan toplayip
    # haritayi /map'e yayinlamasi icin yeterli zaman).
    slam_lifecycle_nodes = ['slam_toolbox']
    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        # collision_monitor lifecycle disinda; surekli configure failure
        # nav2 bringup'i kilitliyordu. safety_supervisor zaten safety zinciri
        # icin yeterli (cmd_vel filtreleme + sensor watchdog).
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description="'mapping' (varsayilan) veya 'localization'"),
        DeclareLaunchArgument(
            'local_planner', default_value='dwb',
            description="Yerel planlayici: 'dwb' (klasik) veya 'mppi' (tez karsilastirmasi)"),
        DeclareLaunchArgument(
            'enable_nav2', default_value='true',
            description="Nav2 controller/planner/bt zincirini yukle. "
                        "Avoider modunda 'false' (otomatik tam-reaktif suris)."),
        DeclareLaunchArgument(
            'enable_octomap', default_value='false',
            description="3D octomap_server'i yukle. WSL'de default false "
                        "(WSLg surface budget'i Gazebo GUI'yi COPY MODE'a "
                        "dusuruyor). Pi'de true verilebilir."),

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

        # Nav2 core (enable_nav2=true ise yuklenir; avoider modunda 'false')
        # controller_server - klasik DWB (varsayilan)
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             condition=IfCondition(PythonExpression([
                 "'", local_planner, "' == 'dwb' and '", enable_nav2, "' == 'true'"])),
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        # controller_server - MPPI (tez karsilastirmasi)
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             condition=IfCondition(PythonExpression([
                 "'", local_planner, "' == 'mppi' and '", enable_nav2, "' == 'true'"])),
             parameters=[nav2_yaml, mppi_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             condition=IfCondition(enable_nav2),
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             condition=IfCondition(enable_nav2),
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             condition=IfCondition(enable_nav2),
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen',
             condition=IfCondition(enable_nav2),
             parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]),

        # SLAM lifecycle - HEMEN baslar, slam_toolbox'i aktive eder
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_slam', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'bond_timeout': 10.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 20.0,
                'node_names': slam_lifecycle_nodes,
            }],
        ),

        # Nav2 lifecycle - 30 SN GECIKMELI (slam'in /map yayinlamasi icin yeterli).
        # bond_timeout 10 sn (default 4 cok kisaydi); respawn_reconnection true
        # (configure failed olursa retry); attempt_respawn_reconnection true ile
        # gecici bag sorunlarinda kurtulur.
        TimerAction(
            period=30.0,
            actions=[Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_navigation', output='screen',
                condition=IfCondition(enable_nav2),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'bond_timeout': 10.0,
                    'attempt_respawn_reconnection': True,
                    'bond_respawn_max_duration': 20.0,
                    'node_names': nav2_lifecycle_nodes,
                }],
            )],
        ),

        # IKA ozel node'lari (veri akisi: terrain + DL -> fusion -> safety)
        Node(package='ika_terrain', executable='terrain_perception_node',
             name='terrain_perception', output='screen',
             parameters=[terrain_yaml, {'use_sim_time': use_sim_time}]),
        # DL nesne tespiti — gerçek robotta OAK-D ile çalışır.
        Node(package='ika_perception_dl', executable='dl_perception_node',
             name='dl_perception', output='screen',
             condition=UnlessCondition(use_sim_time),
             parameters=[dl_yaml, {'use_sim_time': use_sim_time}]),
        # Sim sentetik DL tespiti: world ground-truth'tan /detected_objects
        # üretir (test_world.sdf icinde person/chair/bicycle/car sahnesi).
        Node(package='ika_perception_dl', executable='sim_detection_node',
             name='sim_detection', output='screen',
             condition=IfCondition(use_sim_time),
             parameters=[
                 PathJoinSubstitution([
                     FindPackageShare('ika_perception_dl'),
                     'config', 'sim_detection_params.yaml']),
                 {'use_sim_time': use_sim_time},
             ]),

        # 3D Occupancy mapping (octomap) - derinlik bulutundan 3B harita.
        # Ground filter KAPALI (filter_ground_plane=false) — yeniden ac'mak
        # icin base_frame_id ve camera_depth_optical_frame TF tree'de baglanti
        # gerekli; sim'de RGBD kamera frame'i ayri tree'de oldugu icin segfault
        # ediyordu. RViz'de tum noktalari occupied gorur, yer dahil — tezdeki
        # 3D harita gosterimi icin yeterli.
        Node(package='octomap_server', executable='octomap_server_node',
             name='octomap_server', output='log',
             condition=IfCondition(LaunchConfiguration('enable_octomap')),
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'resolution': 0.10,
                 'frame_id': 'map',
                 'base_frame_id': 'base_footprint',
                 'sensor_model.max_range': 6.0,
                 'sensor_model.hit': 0.7,
                 'sensor_model.miss': 0.4,
                 'sensor_model.min': 0.12,
                 'sensor_model.max': 0.97,
                 'pointcloud_min_z': 0.05,
                 'pointcloud_max_z': 2.0,
                 'filter_ground_plane': False,
             }],
             remappings=[('cloud_in', '/oak/points')]),
        # Hibrit fuzyon: terrain + DL -> /hazard_state + /detection_obstacles
        Node(package='ika_fusion', executable='fusion_node',
             name='hazard_fusion', output='screen',
             parameters=[fusion_yaml, {'use_sim_time': use_sim_time}]),
        Node(package='ika_safety', executable='safety_supervisor_node',
             name='safety_supervisor', output='screen',
             parameters=[safety_yaml, {'use_sim_time': use_sim_time}]),

        # IKA lifecycle manager - sim'de dl_perception YOK (sim_detection plain
        # Node, lifecycle disindadir). Gercek robotta dl_perception eklenir.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_ika', output='screen',
            condition=IfCondition(use_sim_time),
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'terrain_perception',
                    'hazard_fusion',
                    'safety_supervisor',
                ],
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_ika', output='screen',
            condition=UnlessCondition(use_sim_time),
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
