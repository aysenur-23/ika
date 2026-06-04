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
                        "'avoider' (reaktif) | 'off' (sadece perception)."),
        DeclareLaunchArgument('enable_octomap', default_value='false',
                              description="3D octomap server (tezdeki 3-eksenli "
                                          "haritalama). Default false — RViz'de "
                                          "buyuk mavi duvarlari onler. Acmak icin "
                                          "enable_octomap:=true."),

        # Gazebo + URDF + bridge + rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim_pkg, 'launch', 'simulation.launch.py'])),
            launch_arguments={
                'headless': headless,
                'rviz': use_rviz,
                'render_engine': render_engine,
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
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=LaunchConfigurationEquals('autonomous_mode', 'avoider'),
        ),

        # cmd_vel relay — KRITIK (2026-06-04): collision_monitor cikisindan
        # geriye relay. Doğru zincir:
        #   DWB     -> /cmd_vel_nav         (planlanan hiz)
        #   coll_mon -> /cmd_vel_collision  (engel yakininsa stop/slowdown uygular)
        #   relay   -> /cmd_vel             (robota gider, engelde durur)
        #
        # Onceki /cmd_vel_nav -> /cmd_vel direkt = collision_monitor'u atliyordu,
        # robot engellere carpiyordu (Gazebo screenshot kaniti). Düzeltildi.
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_collision', '/cmd_vel'],
            condition=LaunchConfigurationEquals('autonomous_mode', 'nav2'),
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_nav', '/cmd_vel'],
            condition=LaunchConfigurationEquals('autonomous_mode', 'avoider'),
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
