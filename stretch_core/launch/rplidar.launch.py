import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown

configurable_parameters = [{'name': 'serial_port',      'default': '/dev/hello-lrf',   'description':"'Specifying usb port to connected lidar'"},
                           {'name': 'serial_baudrate',  'default': '115200',           'description':"'Specifying usb port baudrate to connected lidar'"},
                           {'name': 'frame_id',         'default': 'laser',            'description':"'Specifying frame_id of lidar'"},
                           {'name': 'inverted',         'default': 'false',            'description':"'Specifying whether or not to invert scan data'"},
                           {'name': 'angle_compensate', 'default': 'true',             'description':"'Specifying whether or not to enable angle_compensate of scan data'"},
                           {'name': 'scan_mode',        'default': 'Boost',            'description':"''"}, # Check if this is supported
                           ]

# lidar supported modes
# Standard: max_distance: 12.0 m, Point number: 2.0K
# Express: max_distance: 12.0 m, Point number: 4.0K
# Boost: max_distance: 12.0 m, Point number: 8.0K

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    laser_filter_config = os.path.join(
        get_package_share_directory('stretch_core'),
        'config',
        'laser_filter_params.yaml'
        )

    lidar_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[set_configurable_parameters(configurable_parameters)],
            output='screen')

    laser_filters = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[laser_filter_config]
            )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        lidar_node,
        laser_filters,
    ])
