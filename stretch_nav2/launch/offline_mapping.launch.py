import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    navigation_package = str(get_package_share_path("stretch_nav2"))

    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'gamepad', 'broadcast_odom_tf': 'True'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/rviz_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([navigation_package, 'config', 'mapper_params_online_async.yaml']),
        ]
    )

    ld = LaunchDescription([
        rviz_param,
        declare_use_sim_time_argument,
        stretch_driver_launch,
        rplidar_launch,
        rviz_launch,
        slam_toolbox,
    ])

    return ld