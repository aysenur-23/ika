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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ika_sim = FindPackageShare('ika_simulation')
    ika_desc = FindPackageShare('ika_description')
    ros_gz_sim = FindPackageShare('ros_gz_sim')

    # world arg ile sahne degisir: test_world (default, parkur) | debug_world
    # (kalibrasyon icin tek engelli minimal). Yeni dunya icin: world:=ad
    # (uzanti yok). 'worlds/<ad>.sdf' aranır.
    world_name = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([ika_sim, 'worlds',
                                       [world_name, TextSubstitution(text='.sdf')]])
    xacro_path = PathJoinSubstitution([ika_desc, 'urdf', 'ika.urdf.xacro'])
    bridge_yaml = PathJoinSubstitution([ika_sim, 'config', 'ros_gz_bridge.yaml'])
    # Full sim config: Map + Costmaps + Global Plan + Local Plan + LaserScan + TF
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('ika_bringup'), 'rviz', 'ika_full.rviz'])

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    render_engine = LaunchConfiguration('render_engine')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    # Jazzy: Command(...) ciktisi (URDF XML) parametre olarak verilince launch
    # YAML olarak parse etmeye calisir -> '<?xml' YAML'da gecersiz. ParameterValue
    # ile string tipini acikca belirtmek gerekiyor.
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                TextSubstitution(text='xacro '), xacro_path,
                TextSubstitution(text=' use_sim:=true'),
            ]),
            value_type=str,
        ),
        'use_sim_time': True,
    }

    # gz_args: '-r [--render-engine <eng>] <world>' veya headless varyanti.
    # render_engine default 'ogre2' (Pi). WSL2'de yazilim OpenGL (LLVMpipe) ile
    # uyumsuz -> 'ogre' (OGRE 1) gerekir, ayrica LIBGL_ALWAYS_SOFTWARE=1.
    re_arg = [TextSubstitution(text='--render-engine '), render_engine, TextSubstitution(text=' ')]
    gz_args_normal = [TextSubstitution(text='-r '), *re_arg, world_path]
    gz_args_headless = [TextSubstitution(text='-r -s --headless-rendering '), *re_arg, world_path]

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
        DeclareLaunchArgument(
            'render_engine', default_value='ogre2',
            description="Gazebo rendering engine: 'ogre2' (Pi default) | 'ogre' (WSL2/yazilim OGL)"),
        DeclareLaunchArgument(
            'world', default_value='test_world',
            description="Sahne adi (worlds/<ad>.sdf). 'test_world' (parkur) | "
                        "'debug_world' (1 engelli minimal kalibrasyon sahnesi)."),
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
