from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    stretch_moveit2_path = get_package_share_path('stretch_moveit2')
    stretch_description_path = get_package_share_path('stretch_description')

    # Load ROS2 Control backed XACRO and YAML
    fake_description_content = Command(['xacro ', str(stretch_moveit2_path / 'urdf' / 'fake_description.xacro')])
    fake_description = {'robot_description': fake_description_content}
    fake_ros2_controllers = str(stretch_moveit2_path / 'config' / 'fake_ros2_controllers.yaml')

    # Launch args
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False', choices=['True', 'False'],
        description='Whether to show Rviz'
    )

    declare_rviz_cfg_fpath_arg = DeclareLaunchArgument(
        'rviz_cfg_fpath',
        default_value=str(stretch_description_path / 'rviz' / 'stretch.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    # Nodes
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     arguments=['-d', LaunchConfiguration('rviz_cfg_fpath')],
                     condition=IfCondition(LaunchConfiguration("rviz")))

    static_odom_tf = Node(package='tf2_ros',
                          executable='static_transform_publisher',
                          arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'])

    fake_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[fake_description])

    controller_manager_node = Node(package='controller_manager',
                                   executable='ros2_control_node',
                                   parameters=[fake_description, fake_ros2_controllers])

    spawn_jsb_node = Node(package='controller_manager',
                          executable='spawner',
                          arguments=['joint_state_broadcaster'])

    spawn_sc_node = Node(package='controller_manager',
                          executable='spawner',
                          arguments=['stretch_controller'])

    return LaunchDescription([declare_rviz_arg,
                              declare_rviz_cfg_fpath_arg,
                              rviz_node,
                              static_odom_tf,
                              fake_state_publisher_node,
                              controller_manager_node,
                              spawn_jsb_node,
                              spawn_sc_node])
