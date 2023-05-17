import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_octomap_path = get_package_share_directory('stretch_octomap')
    
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])
    rviz_config = DeclareLaunchArgument('rviz_config',
                                        default_value=stretch_octomap_path + '/' + 'rviz/octomap.rviz')
     
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'True'}.items())

    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/d435i_high_resolution.launch.py']))

    # keyboard_teleop_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([stretch_core_path, '/launch/keyboard_teleop.launch.py']))

    octomap_mapping_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([stretch_octomap_path, '/launch/octomap_mapper.launch.xml']))
    
    rviz_launch = Node(package='rviz2', executable='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        respawn=True,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        )

    return LaunchDescription([
        rviz_param,
        rviz_config,
        stretch_driver_launch,
        d435i_launch,
        # keyboard_teleop_launch,
        octomap_mapping_launch,
        rviz_launch,
    ])
