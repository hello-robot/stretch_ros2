import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    ld = LaunchDescription()

    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('stretch_moveit_config'),
                                                               'config',
                                                               'stretch.xacro'))
    robot_description = {'robot_description': robot_description_config.toxml()}

    rviz_config_file = get_package_share_directory('stretch_core') + '/rviz/stretch_simple_test.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])
    ld.add_action(rviz_node)
    return ld
