import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command

uncalibrated_controller_yaml_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'controller_calibration_head_factory_default.yaml')
debug_directory_path = os.path.join(os.getenv('HELLO_FLEET_PATH'), 'debug') + '/' if os.getenv('HELLO_FLEET_PATH') else ''

configurable_parameters = [{'name': 'uncalibrated_controller_yaml_file',               'default': uncalibrated_controller_yaml_path, 'description': 'directory path of the uncalibrated controller yaml file'},
                           {'name': 'debug_directory',  'default': debug_directory_path, 'description': 'directory where debug imagery is saved'}]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def generate_launch_description():
    debug_directory = LaunchConfiguration('debug_directory')
    grasp_object_params = [
        {'debug_directory': debug_directory,
         'dryrun': False}
    ]

    d435i_high_res_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/d435i_high_resolution.launch.py']),
        )

    stretch_driver = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/stretch_driver.launch.py']),
               launch_arguments={'broadcast_odom_tf': 'True'}.items()
          )

    stretch_funmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_funmap'), 'launch'),
            '/funmap.launch.py']),
        )

    grasp_object = Node(
            package='stretch_demos',
            executable='grasp_object',
            output='screen',
            parameters=grasp_object_params,
    )

    static_tf_straight_gripper_aligned = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '1.5707963', '--roll', '1.5707963', '--frame-id', 'link_straight_gripper', '--child-frame-id', 'link_straight_gripper_aligned']
        )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        d435i_high_res_launch,
        stretch_driver,
        stretch_funmap,
        grasp_object,
        static_tf_straight_gripper_aligned
        ])
