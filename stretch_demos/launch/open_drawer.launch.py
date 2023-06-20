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
    
    # rplidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('stretch_core'), 'launch'),
    #         '/rplidar.launch.py']),
    #     )

    # joint_state_publisher = Node(package='joint_state_publisher',
    #                              executable='joint_state_publisher',
    #                              output='log',
    #                              parameters=[{'source_list': ['/stretch/joint_states']},
    #                                          {'rate': 15}])

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

    stretch_description = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_description'), 'launch'),
               '/display.launch.py'])
          )

    stretch_driver = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/stretch_driver.launch.py']),
               launch_arguments={'broadcast_odom_tf': 'True'}.items()
          )

    keyboard_teleop_params = [
        {'handover_object_on': True,
        }
    ]

    keyboard_teleop = Node(
            package='stretch_core',
            executable='keyboard_teleop',
            output='screen',
            prefix='xterm -e',
            parameters=keyboard_teleop_params
    )

    stretch_funmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_funmap'), 'launch'),
            '/funmap.launch.py']),
        )

    open_drawer = Node(
            package='stretch_demos',
            executable='open_drawer',
            output='screen',
    )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        d435i_high_res_launch,
        # rplidar,
        # d435i_configure, #// Uncomment when node is merged
        # joint_state_publisher,
        # robot_state_publisher,
        stretch_driver,
        # stretch_description,
        stretch_funmap,
        keyboard_teleop,
        open_drawer
        ])
