"""Yalniz sensor surucu launch (test icin).
RPLIDAR C1 + OAK-D Lite + GPS surucu node'larini baslatir."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    gps_port = LaunchConfiguration('gps_port')
    gps_baud = LaunchConfiguration('gps_baud')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ika_lidar'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('gps_baud', default_value='9600'),

        # RPLIDAR C1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch', 'sllidar_c1_launch.py'])),
            launch_arguments={
                'serial_port': lidar_port,
                'frame_id': 'laser_frame',
            }.items(),
        ),

        # OAK-D Lite (depthai)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('depthai_ros_driver'),
                    'launch', 'camera.launch.py'])),
            launch_arguments={
                'camera_model': 'OAK-D-LITE',
                'camera_name': 'oak',
                'parent_frame': 'base_link',
            }.items(),
        ),

        # GPS - nmea
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            output='screen',
            parameters=[{
                'port': gps_port,
                'baud': gps_baud,
                'frame_id': 'gps_frame',
            }],
        ),
    ])
