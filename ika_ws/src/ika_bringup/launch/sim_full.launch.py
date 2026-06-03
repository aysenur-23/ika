"""Gazebo + tum yazilim katmanlari + suris modu seçimi.

Suris modlari (`autonomous_mode` arg):
    avoider  (DEFAULT) — Tam-reaktif engel kacinma. Robot dumduz baslar, engelle
                         karsilasinca sola/saga doner, engel bitince ev yonune
                         doner, 2 m engelsiz mesafe sonra durur. Nav2 yuklenmez.
    nav2     — Klasik goal-based Nav2 + DWB/MPPI. /goal_pose ile hedef gonder.
    off      — Sadece slam + perception + safety; suris komutu kullanici (teleop).

Gercek arac (real_robot.launch.py) de ayni arg'i destekler.

Diger arg'ler:
    headless:=true|false       Gazebo GUI gizle (WSL Mod C default false)
    rviz:=true|false           RViz ac (WSL default false)
    render_engine:=ogre|ogre2  WSL'de 'ogre' onerilir.
    local_planner:=dwb|mppi    (sadece autonomous_mode=nav2 icin)
    slam_mode:=mapping|localization
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

    # enable_nav2 = (autonomous_mode == 'nav2')
    enable_nav2 = PythonExpression(["'", autonomous_mode, "' == 'nav2'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description="Gazebo GUI gizle. WSL Mod C default false."),
        DeclareLaunchArgument('rviz', default_value='false',
                              description="RViz ac. WSL'de Gazebo + RViz birlikte surface budget asar."),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description="Gazebo render engine (WSL: ogre, Pi: ogre2)."),
        DeclareLaunchArgument('local_planner', default_value='dwb',
                              description="(nav2 modunda) dwb veya mppi."),
        DeclareLaunchArgument('slam_mode', default_value='mapping',
                              description="mapping veya localization."),
        DeclareLaunchArgument(
            'autonomous_mode', default_value='avoider',
            description="Suris modu: 'avoider' (reaktif, default) | "
                        "'nav2' (goal-based) | 'off' (sadece perception)."),

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
            }.items(),
        ),

        # Avoider (sadece autonomous_mode == 'avoider')
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([mission_pkg, 'launch', 'autonomous_drive.launch.py'])),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=LaunchConfigurationEquals('autonomous_mode', 'avoider'),
        ),

        # /cmd_vel_safe -> /cmd_vel relay (her zaman)
        Node(
            package='topic_tools', executable='relay',
            name='cmd_vel_relay', output='log',
            arguments=['/cmd_vel_safe', '/cmd_vel'],
            parameters=[{'use_sim_time': True}],
        ),
    ])
