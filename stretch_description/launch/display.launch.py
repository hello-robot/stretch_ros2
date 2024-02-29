from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():
    stretch_description_path = get_package_share_path('stretch_description')
    robot_description_content = Command(['xacro ',
                                         str(stretch_description_path / 'urdf' / 'stretch.urdf')])

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'zeros.joint_lift': 0.2, 'zeros.joint_wrist_yaw': 3.4}],
    )

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      output='both',
                                      parameters=[{'robot_description': robot_description_content,
                                                   'publish_frequency': 15.0}])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(stretch_description_path / 'rviz' / 'stretch.rviz')]
    )

    return LaunchDescription([joint_state_publisher_gui_node,
                              robot_state_publisher_node,
                              rviz_node,
                              ])
