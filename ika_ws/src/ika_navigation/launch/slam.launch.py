"""SLAM modu: rf2o lidar odom -> EKF -> SLAM Toolbox."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('ika_navigation')
    rf2o_yaml = PathJoinSubstitution([pkg, 'config', 'rf2o_params.yaml'])
    ekf_yaml = PathJoinSubstitution([pkg, 'config', 'ekf_params.yaml'])
    slam_yaml = PathJoinSubstitution([pkg, 'config', 'slam_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[rf2o_yaml, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/odometry/filtered', '/odometry/filtered')],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_yaml, {'use_sim_time': use_sim_time}],
        ),
    ])
