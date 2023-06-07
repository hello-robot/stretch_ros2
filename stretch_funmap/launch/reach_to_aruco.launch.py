import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_funmap_path = get_package_share_directory('stretch_funmap')

    stretch_funmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_funmap_path, '/launch/stretch_funmap.launch.py']))
    
    aruco_head_scan_server = Node(
        package='stretch_funmap',
        executable='aruco_head_scan_action.py',
        output='screen',
        )
    
    reach_to_aruco = Node(
        package='stretch_funmap',
        executable='reach_to_aruco.py',
        output='screen',
        )
    

    return LaunchDescription([
        stretch_funmap_launch,
        aruco_head_scan_server,
        reach_to_aruco,
    ])
