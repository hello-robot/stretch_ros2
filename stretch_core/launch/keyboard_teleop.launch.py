from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'manipulation', 'broadcast_odom_tf': 'True', 'fail_out_of_range_goal': 'False'}.items(),
    )

    keyboard_teleop = Node(
            package='stretch_core',
            executable='keyboard_teleop',
            output='screen'
            )

    return LaunchDescription([
        stretch_driver,
        keyboard_teleop,
    ])