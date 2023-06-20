from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    moveit_config_path = get_package_share_path('stretch_moveit_config')
    stretch_core_path = get_package_share_path('stretch_core')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('pipeline', default_value='ompl', description='specify the planning pipeline'))
    ld.add_action(DeclareLaunchArgument('debug', default_value='false', choices=['true', 'false'],
                                        description='By default, we are not in debug mode'))

    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('moveit_py_file', default_value='moveit2_planning.py'))

    # Load the URDF, SRDF and other .yaml configuration files
    robot_description_content = Command(['xacro ',
                                         str(get_package_share_path('stretch_description') / 'urdf' / 'stretch.urdf')])
    with open(moveit_config_path / 'config/stretch_description.srdf', 'r') as f:
        semantic_content = f.read()

    # Run the main MoveIt executable
    moveit2_py_node_launch_args = {
        'allow_trajectory_execution': 'true',
        'fake_execution': 'false',
        'info': 'true',
        'moveit_py_file': LaunchConfiguration('moveit_py_file'),
        'debug': LaunchConfiguration('debug'),
        'pipeline': LaunchConfiguration('pipeline'),
        'robot_description': robot_description_content,
        'semantic_config': semantic_content,
        "publish_robot_description": 'true', 
        "publish_robot_description_semantic": 'true',
    }
    # moveit2_py_node_launch_py = PythonLaunchDescriptionSource(str(moveit_config_path / 'launch/moveit2_py_node.launch.py'))
    # moveit2_py_node_launch = IncludeLaunchDescription(moveit2_py_node_launch_py,
    #                                              launch_arguments=moveit2_py_node_launch_args.items())

    move_group = PythonLaunchDescriptionSource(str(moveit_config_path / 'launch/move_group.launch.py'))
    move_group_launch = IncludeLaunchDescription(move_group,
                                                 launch_arguments=moveit2_py_node_launch_args.items())
    ld.add_action(move_group_launch)
    # ld.add_action(moveit2_py_node_launch)

    return ld
