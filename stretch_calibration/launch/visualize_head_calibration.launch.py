import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


uncalibrated_urdf_path = os.path.join(get_package_share_directory('stretch_description'), 'urdf', 'stretch_uncalibrated.urdf')
uncalibrated_controller_yaml_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'controller_calibration_head_factory_default.yaml')
calibration_directory_path = "{0}/{1}/calibration_ros/".format(os.getenv('HELLO_FLEET_PATH'), os.getenv('HELLO_FLEET_ID'))

configurable_parameters = [{'name': 'optimization_result_yaml_file', 'default': '', 'description': ''},
                           {'name': 'calibrated_controller_yaml_file', 'default': '', 'description': ''},
                           {'name': 'calibrated_urdf_file', 'default': '', 'description': ''},
                           {'name': 'uncalibrated_urdf_filename',                          'default': uncalibrated_urdf_path, 'description': 'directory path of the uncalibrated urdf file'},
                           {'name': 'uncalibrated_controller_calibration_filename',               'default': uncalibrated_controller_yaml_path, 'description': 'directory path of the uncalibrated controller yaml file'},
                           {'name': 'calibration_directory',                           'default': calibration_directory_path, 'description': 'Path of the calibration_ros directory'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def generate_launch_description():
    head_calibration_options = os.path.join(get_package_share_directory('stretch_calibration'), 'config', 'head_calibration_options.yaml')

    optimization_result_yaml_file = LaunchConfiguration('optimization_result_yaml_file')

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
        arguments=['--only_vis', '--load', optimization_result_yaml_file],
        output='screen',
        ) 
    
    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 parameters=[{'source_list': ['/stretch/joint_states']},
                                             {'rate': 15},
                                             {'use_gui': 'false'}
                                             ])

    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('calibrated_urdf_file')]
    )
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content,
                                              'publish_frequency': 15.0}])

    stretch_driver_params = [
        {'rate': 25.0,
         'timeout': 0.5,
         'controller_calibration_file': LaunchConfiguration('calibrated_controller_yaml_file'),
         'fail_out_of_range_goal': 'false'
         }
    ]

    stretch_driver = Node(package='stretch_core',
                          executable='stretch_driver',
                          emulate_tty=True,
                          output='screen',
                          remappings=[('cmd_vel', '/stretch/cmd_vel'),
                                      ('joint_states', '/stretch/joint_states')],
                          parameters=stretch_driver_params)
    
    rviz_config_path = os.path.join(get_package_share_directory('stretch_calibration'), 'rviz', 'head_calibration.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        process_calibration_data,
        joint_state_publisher,
        robot_state_publisher,
        stretch_driver,
        rviz_node,
        ])
