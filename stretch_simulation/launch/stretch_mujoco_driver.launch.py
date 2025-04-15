from platform import system
import sys

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.parameter_descriptions
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
import launch_ros
import os
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from stretch_mujoco.robocasa_gen import choose_layout, choose_style, get_styles, layouts

if system() == "Linux":
    # this fixes rviz launch issue
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = (
        "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so"
    )
    os.environ["GTK_PATH"] = ""


def generate_launch_description():

    stretch_simulation_path = get_package_share_path("stretch_simulation")
    ld = LaunchDescription()

    declare_broadcast_odom_tf_arg = DeclareLaunchArgument(
        "broadcast_odom_tf",
        default_value="True",
        choices=["True", "False"],
        description="Whether to broadcast the odom TF",
    )
    ld.add_action(declare_broadcast_odom_tf_arg)

    declare_fail_out_of_range_goal_arg = DeclareLaunchArgument(
        "fail_out_of_range_goal",
        default_value="False",
        choices=["True", "False"],
        description="Whether the motion action servers fail on out-of-range commands",
    )
    ld.add_action(declare_fail_out_of_range_goal_arg)

    ld.add_action(
        DeclareLaunchArgument(
            "mode",
            default_value="position",
            choices=["position", "navigation", "trajectory", "gamepad"],
            description="The mode in which the ROS driver commands the robot",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz", default_value="true", choices=["true", "false"]
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_mujoco_viewer", default_value="true", choices=["true", "false"]
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_cameras", default_value="false", choices=["true", "false"]
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_robocasa", default_value="true", choices=["true", "false"]
        )
    )
    ld.add_action(
        DeclareLaunchArgument("robocasa_task", default_value="PnPCounterToCab")
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robocasa_layout", default_value="Random", choices=["Random"] + list(layouts.values())
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robocasa_style",
            default_value="Random",
            choices=["Random"] + list(get_styles().values()),
        )
    )

    use_robocasa = "use_robocasa:=false" not in sys.argv
    robocasa_layout = None
    robocasa_style = None
    if use_robocasa and "--show-args" not in sys.argv:
        args_string = " ".join(sys.argv)
        if not "robocasa_layout" in args_string:
            print("\n\nYou have not specified a `robocasa_layout` argument, choose a layout:\n")
            robocasa_layout = choose_layout()
            robocasa_layout = layouts[robocasa_layout]
            print(f"{robocasa_layout=}")
        if not "robocasa_style" in args_string:
            print("\n\nYou have not specified a `robocasa_style` argument, choose a style:\n")
            robocasa_style = choose_style()
            robocasa_style = get_styles()[robocasa_style]
            print(f"{robocasa_style=}")

    # calibrated_backlash = stretch_simulation_path / 'config' / 'controller_calibration_head.yaml'
    # uncalibrated_backlash = stretch_simulation_path / 'config' / 'controller_calibration_head_factory_default.yaml'
    # if calibrated_backlash.is_file():
    #     backlash_fpath = calibrated_backlash
    # else:
    #     ld.add_action(LogInfo(msg='\n\nWARNING: Calibrated backlash params not available. Using uncalibrated params.\n'))
    #     backlash_fpath = uncalibrated_backlash
    # declare_controller_arg = DeclareLaunchArgument(
    #     'calibrated_controller_yaml_file',
    #     default_value=str(backlash_fpath),
    #     description='Path to the calibrated controller args file'
    # )
    # ld.add_action(declare_controller_arg)

    uncalibrated_urdf = (
        get_package_share_path("stretch_description")
        / "urdf"
        / "stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro"
    )
    calibrated_urdf = (
        get_package_share_path("stretch_description") / "urdf" / "stretch.urdf"
    )
    if calibrated_urdf.is_file():
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
            Command(["xacro ", str(calibrated_urdf)]), value_type=str
        )
    else:
        ld.add_action(
            LogInfo(
                msg=f"\n\nWARNING: Calibrated URDF not available. Using uncalibrated URDF.\n"
            )
        )
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
            Command(["xacro ", str(uncalibrated_urdf)]), value_type=str
        )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="log",
        parameters=[
            {"source_list": ["/stretch/joint_states"]},
            {"rate": 30.0},
            {"robot_description": robot_description_content},
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )
    ld.add_action(joint_state_publisher)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            {"publish_frequency": 30.0},
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )
    ld.add_action(robot_state_publisher)

    stretch_driver_params = [
        {
            "rate": 30.0,
            "timeout": 0.5,
            #  'controller_calibration_file': LaunchConfiguration('calibrated_controller_yaml_file'),
            "broadcast_odom_tf": LaunchConfiguration("broadcast_odom_tf"),
            "fail_out_of_range_goal": LaunchConfiguration("fail_out_of_range_goal"),
            "mode": LaunchConfiguration("mode"),
            "use_mujoco_viewer": LaunchConfiguration("use_mujoco_viewer"),
            "use_cameras": LaunchConfiguration("use_cameras"),
            "use_robocasa": LaunchConfiguration("use_robocasa"),
            "robocasa_task": LaunchConfiguration("robocasa_task"),
            "robocasa_layout": (
                robocasa_layout
                if robocasa_layout is not None
                else LaunchConfiguration("robocasa_layout")
            ),
            "robocasa_style": (
                robocasa_style
                if robocasa_style is not None
                else LaunchConfiguration("robocasa_style")
            ),
        }
    ]

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                str(stretch_simulation_path / "rviz" / "stretch_sim.rviz")
            ],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        Node(
            package="stretch_simulation",
            executable="stretch_mujoco_driver",
            emulate_tty=True,
            output="screen",
            remappings=[
                ("cmd_vel", "/stretch/cmd_vel"),
                ("joint_states", "/stretch/joint_states"),
            ],
            parameters=stretch_driver_params,
            # arguments=["--ros-args", "--log-level", "debug"],
        )
    )

    return ld
