from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.descriptions
from launch_ros.actions import Node
import launch_ros

# import importlib.resources as importlib_resources
# pkg_path = str(importlib_resources.files("stretch_urdf"))
# model_name = "SE3"  # RE1V0, RE2V0, SE3
# tool_name = "eoa_wrist_dw3_tool_sg3"  # eoa_wrist_dw3_tool_sg3, tool_stretch_gripper, etc
# urdf_file_path = pkg_path + f"/{model_name}/stretch_description_{model_name}_{tool_name}.urdf"
# mesh_files_directory_path = pkg_path + f"/{model_name}/meshes"

def generate_launch_description():
    package_share_path = get_package_share_path('stretch_mujoco_ros2')

    declare_broadcast_odom_tf_arg = DeclareLaunchArgument(
        'broadcast_odom_tf',
        default_value='False', choices=['True', 'False'],
        description='Whether to broadcast the odom TF'
    )

    # declare_fail_out_of_range_goal_arg = DeclareLaunchArgument(
    #     'fail_out_of_range_goal',
    #     default_value='False', choices=['True', 'False'],
    #     description='Whether the motion action servers fail on out-of-range commands'
    # )

    # declare_mode_arg = DeclareLaunchArgument(
    #     'mode',
    #     default_value='position', choices=['position', 'navigation', 'trajectory', 'gamepad'],
    #     description='The mode in which the ROS driver commands the robot'
    # )

    # declare_controller_arg = DeclareLaunchArgument(
    #     'calibrated_controller_yaml_file',
    #     default_value=str(stretch_core_path / 'config' / 'controller_calibration_head.yaml'),
    #     description='Path to the calibrated controller args file'
    # )

    robot_description_content = launch_ros.parameter_descriptions.ParameterValue( Command(['xacro ', str(package_share_path / 'urdf' / 'stretch_sim.urdf')]), value_type=str)

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 output='log',
                                 parameters=[{'source_list': ['/stretch/joint_states']},
                                             {'rate': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content},
                                             {'publish_frequency': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)

    stretch_driver_params = [
        {
        'rate': 30.0,
         'timeout': 0.5,
        #  'controller_calibration_file': LaunchConfiguration('calibrated_controller_yaml_file'),
         'broadcast_odom_tf': LaunchConfiguration('broadcast_odom_tf'),
        #  'fail_out_of_range_goal': LaunchConfiguration('fail_out_of_range_goal'),
        #  'mode': LaunchConfiguration('mode')
        }
    ]

    stretch_driver = Node(package='stretch_mujoco_ros2',
                          executable='stretch_sim_driver',
                          emulate_tty=True,
                          output='screen',
                          remappings=[('cmd_vel', '/stretch/cmd_vel'),
                                      ('joint_states', '/stretch/joint_states'),
                                      ],
                          parameters=stretch_driver_params)
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(package_share_path / 'rviz' / 'stretch_sim.rviz')]
    )

    return LaunchDescription([declare_broadcast_odom_tf_arg,
                            #   declare_fail_out_of_range_goal_arg,
                            #   declare_mode_arg,
                            #   declare_controller_arg,
                              joint_state_publisher,
                              robot_state_publisher,
                              stretch_driver,
                              rviz_node,
                              ])
