from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("stretch").to_moveit_configs()

    # MoveGroupInterface demo executable
    movegroup_test = Node(
        name="movegroup_test",
        package="stretch_moveit_config",
        executable="movegroup_test",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([movegroup_test])