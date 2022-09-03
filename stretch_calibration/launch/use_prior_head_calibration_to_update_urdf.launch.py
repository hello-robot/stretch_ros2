import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


uncalibrated_urdf_path = os.path.join(get_package_share_directory('stretch_description'), 'urdf', 'stretch_uncalibrated.urdf')
uncalibrated_controller_yaml_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'controller_calibration_head_factory_default.yaml')
calibration_directory_path = "{0}/{1}/calibration_ros/".format(os.getenv('HELLO_FLEET_PATH'), os.getenv('HELLO_FLEET_ID'))

configurable_parameters = [{'name': 'uncalibrated_urdf_filename',                          'default': uncalibrated_urdf_path, 'description': 'directory path of the uncalibrated urdf file'},
                           {'name': 'uncalibrated_controller_calibration_filename',               'default': uncalibrated_controller_yaml_path, 'description': 'directory path of the uncalibrated controller yaml file'},
                           {'name': 'calibration_directory',                           'default': calibration_directory_path, 'description': 'Path of the calibration_ros directory'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def generate_launch_description():
    
    head_calibration_options = os.path.join(get_package_share_directory('stretch_calibration'), 'config', 'head_calibration_options.yaml')
    
    process_calibration_data = Node(
        package='stretch_calibration',
        executable='process_head_calibration_data',
        parameters=[
            head_calibration_options,
            {
                'calibration_directory': LaunchConfiguration('calibration_directory'),
                'uncalibrated_controller_calibration_filename': LaunchConfiguration('uncalibrated_controller_calibration_filename'),
                'uncalibrated_urdf_filename': LaunchConfiguration('uncalibrated_urdf_filename')
            }],
        arguments=['--no_vis', '--load_prev'],
        output='screen',
        )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        process_calibration_data,
        ])
