from launch import LaunchDescription

from launch_ros.actions import Node

# Supported Scan Modes:
# Standard: max_distance: 12.0 m, Point number: 2.0K
# Express:  max_distance: 12.0 m, Point number: 4.0K
# Boost:    max_distance: 12.0 m, Point number: 8.0K


def generate_launch_description():
    """RPLIDAR A1."""
    return LaunchDescription([
        Node(
            node_name='rplidar_composition',
            package='rplidar_ros',
            node_executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/hello-lrf',
                'serial_baudrate': 115200,  # A1
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Boost',
            }],
        ),
    ])
