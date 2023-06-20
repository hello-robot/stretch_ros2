import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_joint_limits_from_config(moveit_config_path, mode='default'):
    return str(moveit_config_path / 'config/joint_limits.yaml')

def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_unified_path = get_package_share_directory('stretch_unified')

    stretch_unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_unified_path, '/launch/stretch_unified.launch.py']))
    
    aruco_head_scan_server = Node(
        package='stretch_unified',
        executable='aruco_head_scan_action.py',
        output='screen',
        )
    
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_aruco.launch.py']))
    
    # ************************************************************************************************
    # MoveIt Parameters
    # ************************************************************************************************

    moveit_config_path = get_package_share_path('stretch_moveit_config')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('debug', default_value='false', choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('pipeline', default_value='ompl', choices=['ompl', 'chomp']))
    ld.add_action(DeclareLaunchArgument('allow_trajectory_execution', default_value='true'))
    ld.add_action(DeclareLaunchArgument('fake_execution', default_value='false'))
    ld.add_action(DeclareLaunchArgument('max_safe_path_cost', default_value='1'))
    ld.add_action(DeclareLaunchArgument('jiggle_fraction', default_value='0.05'))
    ld.add_action(DeclareLaunchArgument('publish_monitored_planning_scene', default_value='true'))
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument('capabilities', default_value=''))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument('disable_capabilities', default_value=''))

    planning_parameters = [str(moveit_config_path / 'config') + '/', LaunchConfiguration('pipeline'), '_planning.yaml']
    kinematics_config = str(moveit_config_path / 'config/kinematics.yaml')
    sensors_config = str(moveit_config_path / 'config/sensors_3d.yaml')
    trajectory_config = str(moveit_config_path / 'config/trajectory_execution.yaml')

    controller_parameters = [str(moveit_config_path / 'config') + '/',
                             'ros_controllers.yaml']

    # Load the URDF, SRDF and other .yaml configuration files
    robot_description_content = Command(['xacro ',
                                         str(get_package_share_path('stretch_description') / 'urdf' / 'stretch.urdf')])
    with open(moveit_config_path / 'config/stretch_description.srdf', 'r') as f:
        semantic_content = f.read()

    move_group_configuration = {
        'robot_description': robot_description_content,
        'robot_description_semantic': semantic_content,
        'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
        'max_safe_path_cost': LaunchConfiguration('max_safe_path_cost'),
        'jiggle_fraction': LaunchConfiguration('jiggle_fraction'),
        'capabilities': LaunchConfiguration('capabilities'),
        'disable_capabilities': LaunchConfiguration('disable_capabilities'),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        'publish_planning_scene': LaunchConfiguration('publish_monitored_planning_scene'),
        'publish_geometry_updates': LaunchConfiguration('publish_monitored_planning_scene'),
        'publish_state_updates': LaunchConfiguration('publish_monitored_planning_scene'),
        'publish_transforms_updates': LaunchConfiguration('publish_monitored_planning_scene'),
    }

    move_group_params = [
        planning_parameters,
        sensors_config,
        trajectory_config,
        controller_parameters,
        kinematics_config,
        load_joint_limits_from_config(moveit_config_path),
        move_group_configuration,
    ]

    moveit_cpp_config = load_yaml("stretch_moveit_config", "config/motion_planning_python.yaml")
    # ************************************************************************************************

    reach_to_aruco = Node(
        package='stretch_unified',
        executable='reach_to_aruco.py',
        parameters=[moveit_cpp_config] + move_group_params,
        output='screen',
        )
    
    ld.add_action(stretch_unified_launch)
    ld.add_action(aruco_head_scan_server)
    ld.add_action(aruco_launch)
    ld.add_action(reach_to_aruco)
    
    return ld
