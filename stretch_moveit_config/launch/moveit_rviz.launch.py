from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    moveit_config_path = get_package_share_path('stretch_moveit_config')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('robot_description'))
    ld.add_action(DeclareLaunchArgument('semantic_config'))

    ld.add_action(DeclareLaunchArgument('debug', default_value='false', choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('rviz_config',
                                        default_value=str(moveit_config_path / 'launch/moveit.rviz')))

    planning_parameters = [str(moveit_config_path / 'config') + '/', LaunchConfiguration('pipeline'), '_planning.yaml']

    rviz_parameters = [
        {'robot_description': LaunchConfiguration('robot_description')},
        {'robot_description_semantic': LaunchConfiguration('semantic_config')},
        {'robot_description_planning': planning_parameters},
        str(moveit_config_path / 'config/kinematics.yaml'),
    ]

    rviz_node = Node(package='rviz2', executable='rviz2',
                     output='log',
                     condition=UnlessCondition(LaunchConfiguration('debug')),
                     respawn=True,
                     arguments=['-d', LaunchConfiguration('rviz_config')],
                     parameters=rviz_parameters,
                     )
    ld.add_action(rviz_node)

    # rviz_debug_node is Identical to rviz_node except for the prefix (and the condition)
    rviz_debug_node = Node(package='rviz2', executable='rviz2',
                           output='log',
                           arguments=['-d', LaunchConfiguration('rviz_config')],
                           condition=IfCondition(LaunchConfiguration('debug')),
                           parameters=rviz_parameters,
                           prefix=['gdb --ex run --args'],
                           )
    ld.add_action(rviz_debug_node)
    return ld
