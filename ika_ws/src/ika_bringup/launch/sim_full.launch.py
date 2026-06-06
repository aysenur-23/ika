"""Gazebo + tum yazilim katmanlari + suris modu seçimi.

WSL TEZDEKI DEMO icin default'lar (parametre vermeden direkt 'ros2 launch
ika_bringup sim_full.launch.py' yazarsan):
  autonomous_mode = nav2           — Nav2 + SLAM + yol planlama (gercek profesyonel)
  headless        = true           — Gazebo GUI yok (WSL surface budget rahat)
  rviz            = true           — RViz acik (harita + plan + lidar gorulur)
  enable_octomap  = true           — 3D harita (octomap, tezdeki '3-eksenli')
  render_engine   = ogre           — WSL'de OGRE1

Suris modlari (`autonomous_mode` arg):
    nav2     (DEFAULT) — Klasik goal-based Nav2 + DWB/MPPI + planlama gorseli.
    avoider  — Tam-reaktif engel kacinma (basit bug algoritmasi).
    off      — Sadece slam + perception + safety; suris komutu kullanici (teleop).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare('ika_simulation')
    nav_pkg = FindPackageShare('ika_navigation')
    mission_pkg = FindPackageShare('ika_mission')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    render_engine = LaunchConfiguration('render_engine')
    local_planner = LaunchConfiguration('local_planner')
    slam_mode = LaunchConfiguration('slam_mode')
    autonomous_mode = LaunchConfiguration('autonomous_mode')
    enable_octomap = LaunchConfiguration('enable_octomap')
    world_name = LaunchConfiguration('world')
    bypass_coll = LaunchConfiguration('bypass_collision_monitor')

    # enable_nav2 = (autonomous_mode == 'nav2')
    enable_nav2 = PythonExpression(["'", autonomous_mode, "' == 'nav2'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # WSL TEZDEKI DEMO icin optimize edilmis default'lar:
        DeclareLaunchArgument('headless', default_value='true',
                              description="Gazebo GUI gizle (WSL: true onerilir, "
                                          "surface budget RViz icin korunur)."),
        DeclareLaunchArgument('rviz', default_value='true',
                              description="RViz ac (harita + plan + lidar gorulur)."),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description="Gazebo render engine (WSL: ogre, Pi: ogre2)."),
        DeclareLaunchArgument('local_planner', default_value='dwb',
                              description="(nav2 modunda) dwb veya mppi."),
        DeclareLaunchArgument('slam_mode', default_value='mapping',
                              description="mapping veya localization."),
        DeclareLaunchArgument(
            'autonomous_mode', default_value='nav2',
            description="Suris modu: 'nav2' (default, goal-based + planlama) | "
                        "'avoider' (reaktif) | 'dynamic' (TASK-4B: local "
                        "planner) | 'off' (sadece perception)."),
        # TASK-3.1: trial harness için kapı. Varsayılan true.
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description="avoider auto-start. False ise /avoider/start servisi "
                        "çağrılana kadar robot hareket etmez (trial harness için)."),
        DeclareLaunchArgument('enable_octomap', default_value='false',
                              description="3D octomap server (tezdeki 3-eksenli "
                                          "haritalama). Default false — RViz'de "
                                          "buyuk mavi duvarlari onler. Acmak icin "
                                          "enable_octomap:=true."),
        DeclareLaunchArgument('world', default_value='test_world',
                              description="Sahne (worlds/<ad>.sdf). 'test_world' "
                                          "(parkur) | 'debug_world' (1 engelli "
                                          "kalibrasyon)."),
        DeclareLaunchArgument('bypass_collision_monitor', default_value='false',
                              description="ABLATION: true ise /cmd_vel_nav -> "
                                          "/cmd_vel direkt relay; collision_monitor "
                                          "spawn olsa bile cikisi kullanilmaz. "
                                          "DWB veya planner ablasyonu icin."),

        # Gazebo + URDF + bridge + rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim_pkg, 'launch', 'simulation.launch.py'])),
            launch_arguments={
                'headless': headless,
                'rviz': use_rviz,
                'render_engine': render_engine,
                'world': world_name,
            }.items(),
        ),

        # Perception + safety + slam (her zaman). Nav2 ise enable_nav2'ye bagli.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav_pkg, 'launch', 'navigation.launch.py'])),
            launch_arguments={
                'use_sim_time': 'true',
                'local_planner': local_planner,
                'slam_mode': slam_mode,
                'enable_nav2': enable_nav2,
                'enable_octomap': enable_octomap,
            }.items(),
        ),

        # Avoider (sadece autonomous_mode == 'avoider')
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([mission_pkg, 'launch', 'autonomous_drive.launch.py'])),
            launch_arguments={
                'use_sim_time': 'true',
                # TASK-3.1: arg passthrough
                'auto_start': LaunchConfiguration('auto_start'),
            }.items(),
            condition=LaunchConfigurationEquals('autonomous_mode', 'avoider'),
        ),

        # TASK-4B-1: Dynamic local planner (autonomous_mode == 'dynamic')
        # Aynı /cmd_vel_nav + /avoider/* API'sini sağlar (harness uyumlu).
        Node(
            package='ika_local_planner',
            executable='dynamic_local_planner_node',
            name='dynamic_local_planner', output='screen',
            parameters=[{
                'use_sim_time': True,
                'auto_start': LaunchConfiguration('auto_start'),
                'control_rate_hz': 20.0,
                'target_x': 22.0, 'target_y': 0.0,
                'path_y': 0.0, 'target_heading_rad': 0.0,
                'default_speed_mps': 0.22,
                'slow_speed_mps': 0.12,
                'max_angular_rps': 0.55,
                'reflex_stop_distance_m': 0.20,
                'safety_cost_threshold': 0.65,
                'lookahead_m': 1.2,
                'front_arc_deg': 60.0,
                'costmap_width_m': 4.0,
                'costmap_height_m': 4.0,
                'costmap_res_m': 0.10,
                'inflation_radius_m': 0.30,
            }],
            condition=LaunchConfigurationEquals('autonomous_mode', 'dynamic'),
        ),

        # cmd_vel relay — KRITIK (2026-06-04): collision_monitor cikisindan
        # geriye relay. Doğru zincir:
        #   DWB     -> /cmd_vel_nav         (planlanan hiz)
        #   coll_mon -> /cmd_vel_collision  (engel yakininsa stop/slowdown uygular)
        #   relay   -> /cmd_vel             (robota gider, engelde durur)
        #
        # Onceki /cmd_vel_nav -> /cmd_vel direkt = collision_monitor'u atliyordu,
        # robot engellere carpiyordu (Gazebo screenshot kaniti). Düzeltildi.
        # nav2 modu — collision_monitor cikisindan relay (default,
        # 5 katmanli savunmanin son katmani).
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_collision', '/cmd_vel'],
            condition=IfCondition(PythonExpression([
                "'", autonomous_mode, "' == 'nav2' and '",
                bypass_coll, "' != 'true'"])),
            parameters=[{'use_sim_time': True}],
        ),
        # nav2 modu + bypass — DWB cikisindan direkt relay (ABLATION).
        # collision_monitor hala spawn olur (lifecycle'da) ama cikisi
        # kullanilmaz. A2.2 (DWB only) ve A2.3 (planner only) icin.
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_nav', '/cmd_vel'],
            condition=IfCondition(PythonExpression([
                "'", autonomous_mode, "' == 'nav2' and '",
                bypass_coll, "' == 'true'"])),
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_nav', '/cmd_vel'],
            condition=LaunchConfigurationEquals('autonomous_mode', 'avoider'),
            parameters=[{'use_sim_time': True}],
        ),
        # TASK-4B-1: dynamic mode relay — avoider ile aynı (cmd_vel_nav → cmd_vel)
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_nav', '/cmd_vel'],
            condition=LaunchConfigurationEquals('autonomous_mode', 'dynamic'),
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_safe', '/cmd_vel'],
            condition=LaunchConfigurationEquals('autonomous_mode', 'off'),
            parameters=[{'use_sim_time': True}],
        ),
    ])
