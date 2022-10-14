import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'manipulation', 'broadcast_odom_tf': 'False', 'fail_out_of_range_goal': 'True'}.items(),
    )

    d435i_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_realsense.launch.py'])
          )

    stretch_aruco = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_aruco.launch.py'])
          )

    align_to_aruco = Node(
        package='stretch_core',
        executable='align_to_aruco',
        output='screen',
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
        d435i_launch,
        stretch_aruco,
        align_to_aruco,
        rviz_node,
        ])