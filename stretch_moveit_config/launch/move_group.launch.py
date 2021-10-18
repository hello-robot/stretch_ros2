from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# Mapping from entry in the stretch_body configuration to joints in the SRDF
CONFIGURATION_TRANSLATION = {
    'lift': ['joint_lift'],
    'wrist_yaw': ['joint_wrist_yaw'],
    'stretch_gripper': ['joint_gripper_finger_left', 'joint_gripper_finger_right'],
    'base': ['position/x', 'position/theta'],
    'arm': ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'],
    'head_pan': ['joint_head_pan'],
    'head_tilt': ['joint_head_tilt'],
}

def load_joint_limits_from_config(moveit_config_path, mode='default'):
    return str(moveit_config_path / 'config/joint_limits.yaml')


def generate_launch_description():
    moveit_config_path = get_package_share_path('stretch_moveit_config')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('robot_description'))
    ld.add_action(DeclareLaunchArgument('semantic_config'))

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

    move_group_configuration = {
        'robot_description': LaunchConfiguration('robot_description'),
        'robot_description_semantic': LaunchConfiguration('semantic_config'),
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
    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                           output='screen',
                           parameters=move_group_params,
                           condition=UnlessCondition(LaunchConfiguration('debug')),
                           # args="$(arg command_args)"
                           )
    ld.add_action(move_group_node)

    debug_move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                                 output='screen',
                                 parameters=move_group_params,
                                 condition=IfCondition(LaunchConfiguration('debug')),
                                 # args="$(arg command_args)"
                                 prefix=['gdb --ex run --args'],
                                 )
    # TODO: prefix=['gdb -x $(find stretch_moveit_config)/launch/gdb_settings.gdb --ex run --args'],
    ld.add_action(debug_move_group_node)

    return ld
