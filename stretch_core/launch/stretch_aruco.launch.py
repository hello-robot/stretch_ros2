import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    stretch_marker_dict = Command(['ros2 param load ',
                                         str(get_package_share_directory('stretch_core') / 'config' / 'stretch_marker_dict.yaml')])

    detect_aruco_markers = Node(
        package='stretch_core',
        executable='detect_aruco_markers',
        output='screen',
        )

    return LaunchDescription([
        stretch_marker_dict,
        detect_aruco_markers,
        ])