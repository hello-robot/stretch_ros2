import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command


uncalibrated_urdf_path = os.path.join(get_package_share_directory('stretch_description'), 'urdf', 'stretch_uncalibrated.urdf')
uncalibrated_controller_yaml_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'controller_calibration_head_factory_default.yaml')
calibration_directory_path = "{0}/{1}/calibration_ros/".format(os.getenv('HELLO_FLEET_PATH'), os.getenv('HELLO_FLEET_ID'))

configurable_parameters = [{'name': 'uncalibrated_urdf_file',                          'default': uncalibrated_urdf_path, 'description': 'directory path of the uncalibrated urdf file'},
                           {'name': 'uncalibrated_controller_yaml_file',               'default': uncalibrated_controller_yaml_path, 'description': 'directory path of the uncalibrated controller yaml file'},
                           {'name': 'calibration_directory',                           'default': calibration_directory_path, 'description': 'Path of the calibration_ros directory'},
                          ]

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
    
    aruco_marker_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/stretch_aruco.launch.py']),
        )

    dict_file_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'stretch_marker_dict.yaml')
    collect_calibration_data = Node(
        package='stretch_calibration',
        executable='collect_head_calibration_data',
        parameters=[
            dict_file_path,
            {
                'controller_calibration_file': LaunchConfiguration('uncalibrated_controller_yaml_file'),
                'calibration_directory': LaunchConfiguration('calibration_directory')
            }],
        output='screen',
        )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        d435i_high_res_launch,
        # d435i_configure, // Uncomment when node is merged
        joint_state_publisher,
        robot_state_publisher,
        stretch_driver,
        aruco_marker_detector,
        collect_calibration_data,
        ])
