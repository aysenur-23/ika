"""Sim sentetik nesne tespiti - ana sim stack'inin yaninda calistirilir.

Kullanim (ayri terminalde, sim_full.launch.py calisirken):
  ros2 launch ika_perception_dl sim_detection.launch.py

Gercek dl_perception_node sim'de hicbir sey yayinlamadigi icin bu node
/detected_objects'in tek yayincisi olur ve DL yolunu (fusion -> costmap ->
safety -> planlayici) test eder.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dl_pkg = FindPackageShare('ika_perception_dl')
    sim_yaml = PathJoinSubstitution([dl_pkg, 'config', 'sim_detection_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ika_perception_dl', executable='sim_detection_node',
            name='sim_detection', output='screen',
            parameters=[
                sim_yaml,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
