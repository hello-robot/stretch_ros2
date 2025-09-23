import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


uncalibrated_controller_yaml_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'controller_calibration_head_factory_default.yaml')

configurable_parameters = [{'name': 'uncalibrated_controller_yaml_file',               'default': uncalibrated_controller_yaml_path, 'description': 'directory path of the uncalibrated controller yaml file'},]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')

    """ stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'position', 'broadcast_odom_tf': 'False', 'fail_out_of_range_goal': 'True'}.items(),
    )  """# base right

    stretch_driver = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/stretch_driver.launch.py']),
               launch_arguments={'broadcast_odom_tf': 'True'}.items()
          )  # nav_for_handover

    """ d435i_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_realsense.launch.py'])
          ) """
    d435i_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/d435i_high_resolution.launch.py'])
          )
    
    stretch_funmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_funmap'), 'launch'),
            '/funmap.launch.py']),
        )

    stretch_aruco = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_aruco.launch.py'])
          )
	
    aruco_tag_locator = Node(
        package='stretch_core',
        executable='aruco_tag_locator',
		#name='base_right_locator',
        output='screen',
        )
	
    #rviz_config_path = os.path.join(stretch_core_path, 'rviz', 'stretch_simple_test.rviz')
    # Updated RViz node with the specified configuration file
    rviz_config_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/rviz/aruco_detector_example.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )
	
    return LaunchDescription([
    stretch_driver,
    d435i_launch,
    stretch_funmap,
    stretch_aruco,
    rviz_node,
    aruco_tag_locator,
    ])
