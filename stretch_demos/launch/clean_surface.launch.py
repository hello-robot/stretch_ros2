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

configurable_parameters = [{'name': 'uncalibrated_controller_yaml_file',               'default': uncalibrated_controller_yaml_path, 'description': 'directory path of the uncalibrated controller yaml file'},]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def generate_launch_description():
    d435i_high_res_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/d435i_high_resolution.launch.py']),
        )

    d435i_configure = Node(
        package='stretch_core',
        executable='d435i_configure',
        parameters=[
            {'initial_mode': 'High Accuracy'}
            ],
        output='screen',
        )

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 output='log',
                                 parameters=[{'source_list': ['/stretch/joint_states']},
                                             {'rate': 15}])

    stretch_description_path = get_package_share_directory('stretch_description')
    robot_description_content = Command(
        ['xacro ', os.path.join(stretch_description_path, 'urdf', 'stretch_uncalibrated.urdf')]
    )

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content,
                                            #   'robot_description': LaunchConfiguration('uncalibrated_urdf_file'), // Raises empty file error with URDF parser
                                              'publish_frequency': 15.0}])

    stretch_driver_params = [
        {'rate': 25.0,
         'timeout': 0.5,
         'controller_calibration_file': LaunchConfiguration('uncalibrated_controller_yaml_file'),
         'fail_out_of_range_goal': 'False'
         }
    ]

    stretch_driver = Node(package='stretch_core',
                          executable='stretch_driver',
                          emulate_tty=True,
                          output='screen',
                          remappings=[('cmd_vel', '/stretch/cmd_vel'),
                                      ('joint_states', '/stretch/joint_states')],
                          parameters=stretch_driver_params)

    clean_surface = Node(
        package='stretch_demos',
        executable='clean_surface',
        parameters=[],
        output='screen',
        )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        d435i_high_res_launch,
        # d435i_configure, // Uncomment when node is merged
        joint_state_publisher,
        robot_state_publisher,
        stretch_driver,
        clean_surface
        ])
