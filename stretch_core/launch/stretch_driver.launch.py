from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.descriptions
from launch_ros.actions import Node
import launch_ros
import stretch_body.robot_params as params
import importlib.resources
import os
import sys

# this fixes rviz launch issue
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'


def generate_launch_description():
    # Check is robot
    if 'HELLO_FLEET_ID' not in os.environ:
        print("\nERROR: Must be run on a robot.")
        sys.exit(1)

    stretch_core_path = get_package_share_path('stretch_core')
    ld = LaunchDescription()

    declare_broadcast_odom_tf_arg = DeclareLaunchArgument(
        'broadcast_odom_tf',
        default_value='False', choices=['True', 'False'],
        description='Whether to broadcast the odom TF'
    )
    ld.add_action(declare_broadcast_odom_tf_arg)

    declare_fail_out_of_range_goal_arg = DeclareLaunchArgument(
        'fail_out_of_range_goal',
        default_value='False', choices=['True', 'False'],
        description='Whether the motion action servers fail on out-of-range commands'
    )
    ld.add_action(declare_fail_out_of_range_goal_arg)

    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='position', choices=['position', 'navigation', 'trajectory', 'gamepad'],
        description='The mode in which the ROS driver commands the robot'
    )
    ld.add_action(declare_mode_arg)


    calibrated_backlash = stretch_core_path / 'config' / 'controller_calibration_head.yaml'
    uncalibrated_backlash = stretch_core_path / 'config' / 'controller_calibration_head_factory_default.yaml'
    if calibrated_backlash.is_file():
        backlash_fpath = calibrated_backlash
    else:
        ld.add_action(LogInfo(msg='\n\nWARNING: Calibrated backlash params not available. Using uncalibrated params.\n'))
        backlash_fpath = uncalibrated_backlash
    declare_controller_arg = DeclareLaunchArgument(
        'calibrated_controller_yaml_file',
        default_value=str(backlash_fpath),
        description='Path to the calibrated controller args file'
    )
    ld.add_action(declare_controller_arg)

    _, r = params.RobotParams.get_params()
    model_name = r['robot']['model_name']
    tool_name = r['robot']['tool']
    uncalibrated_urdf = importlib.resources.files("stretch_urdf") / model_name / f"stretch_description_{model_name}_{tool_name}.urdf"
    calibrated_urdf = get_package_share_path('stretch_description') / 'urdf' / 'stretch.urdf'
    if calibrated_urdf.is_file():
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue( Command(['xacro ', str(calibrated_urdf)]), value_type=str)
    else:
        ld.add_action(LogInfo(msg='\n\nWARNING: Calibrated URDF not available. Using uncalibrated URDF.\n'))
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue( Command(['xacro ', str(uncalibrated_urdf)]), value_type=str)

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 output='log',
                                 parameters=[{'source_list': ['/stretch/joint_states']},
                                             {'rate': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)
    ld.add_action(joint_state_publisher)

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content},
                                             {'publish_frequency': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)
    ld.add_action(robot_state_publisher)

    stretch_driver_params = [
        {'rate': 30.0,
         'timeout': 0.5,
         'controller_calibration_file': LaunchConfiguration('calibrated_controller_yaml_file'),
         'broadcast_odom_tf': LaunchConfiguration('broadcast_odom_tf'),
         'fail_out_of_range_goal': LaunchConfiguration('fail_out_of_range_goal'),
         'mode': LaunchConfiguration('mode')}
    ]

    stretch_driver = Node(package='stretch_core',
                          executable='stretch_driver',
                          emulate_tty=True,
                          output='screen',
                          remappings=[('cmd_vel', '/stretch/cmd_vel'),
                                      ('joint_states', '/stretch/joint_states')],
                          parameters=stretch_driver_params)
    ld.add_action(stretch_driver)

    return ld
