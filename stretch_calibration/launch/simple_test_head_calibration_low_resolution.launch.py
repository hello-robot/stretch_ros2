import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    d435i_low_res_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/d435i_low_resolution.launch.py']),
        )

    d435i_configure = Node(
        package='stretch_core',
        executable='d435i_configure',
        parameters=[
            {'initial_mode': 'High Accuracy'}
            ],
        output='screen',
        )

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'true'}.items()
        )
    
    aruco_marker_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/stretch_aruco.launch.py']),
        )

    keyboar_teleop = Node(
        package='stretch_core',
        executable='keyboard_teleop',
        output='screen',
        )

    rviz_config_path = os.path.join(get_package_share_directory('stretch_core'), 'rviz', 'stretch_simple_test.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )

    return LaunchDescription([
        d435i_low_res_launch,
        # d435i_configure,
        stretch_driver,
        aruco_marker_detector,
        # keyboar_teleop,
        rviz_node,
        ])
