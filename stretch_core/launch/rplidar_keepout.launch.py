import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'True', 'fail_out_of_range_goal': 'False'}.items(),
    )

    lidar_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/rplidar.launch.py'])
          )

    avoider = Node(
            package='stretch_core',
            executable='avoider',
            output='screen'
            )

    rviz_config_path = os.path.join(stretch_core_path, 'rviz', 'stretch_simple_test.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )

    return LaunchDescription([
        stretch_driver,
        lidar_launch,
        avoider,
        rviz_node,
    ])