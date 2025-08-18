import os

from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from stretch_mujoco_driver.stretch_mujoco_driver import DEFAULT_SIM_TOOL


def generate_launch_description():
    teleop_interface_package = str(get_package_share_path("stretch_web_teleop"))
    rosbridge_package = str(get_package_share_path("rosbridge_server"))
    stretch_navigation_path = str(get_package_share_directory("stretch_nav2"))

    # Declare launch arguments
    params_file = DeclareLaunchArgument(
        "params",
        default_value=[
            PathJoinSubstitution(
                [
                    teleop_interface_package,
                    "config",
                    "configure_video_streams_params.yaml",
                ]
            )
        ],
    )
    map = DeclareLaunchArgument(
        "map", description="filepath to previously captured map", default_value=""
    )
    tts_engine = DeclareLaunchArgument(
        "tts_engine",
        description="name of the TTS engine. Either pyttsx3 or gtts.",
        default_value="gtts",
    )
    certfile_arg = DeclareLaunchArgument("certfile", default_value="server.crt")
    keyfile_arg = DeclareLaunchArgument("keyfile", default_value="server.key")
    nav2_params_file_param = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(
            stretch_navigation_path, "config", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # Start collecting nodes to launch
    ld = LaunchDescription(
        [
            map,
            tts_engine,
            nav2_params_file_param,
            params_file,
            certfile_arg,
            keyfile_arg,
        ]
    )
    
    tf2_web_republisher_node = Node(
        package="tf2_web_republisher_py",
        executable="tf2_web_republisher",
        name="tf2_web_republisher_node",
    )
    ld.add_action(tf2_web_republisher_node)

    # Rosbridge Websocket
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution(
                [rosbridge_package, "launch", "rosbridge_websocket_launch.xml"]
            )
        ),
        launch_arguments={
            "port": "9090",
            "address": "localhost",
            "ssl": "true",
            "certfile": PathJoinSubstitution(
                [
                    teleop_interface_package,
                    "certificates",
                    LaunchConfiguration("certfile"),
                ]
            ),
            "keyfile": PathJoinSubstitution(
                [
                    teleop_interface_package,
                    "certificates",
                    LaunchConfiguration("keyfile"),
                ]
            ),
            "authenticate": "false",
            "call_services_in_new_thread": "true",
        }.items(),
    )
    ld.add_action(rosbridge_launch)

    # Configure Video Streams
    configure_video_streams_node = Node(
        package="stretch_web_teleop",
        executable="configure_video_streams.py",
        name=f"configure_video_streams_gripper",
        output="screen",
        arguments=[
            LaunchConfiguration("params"),
            str(False),
            "True",  # overhead"
            "True",  # "realsense"
            "True",  # "gripper"
        ],
        parameters=[
            {
                "has_beta_teleop_kit": False,
                "stretch_tool": DEFAULT_SIM_TOOL,
                "use_sim_time": True
            }
        ],
        remappings=[("/gripper_camera/color/camera_info","/gripper_camera/camera_info")]
    )
    ld.add_action(configure_video_streams_node)

    navigation_bringup_launch = GroupAction(
        condition=LaunchConfigurationNotEquals("map", ""),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [stretch_navigation_path, "/launch/bringup_launch.py"]
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("nav2_params_file"),
                    "use_rviz": "false",
                }.items(),
            ),
        ],
    )
    ld.add_action(navigation_bringup_launch)

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/reinitialize_global_localization ",
                    "std_srvs/srv/Empty ",
                    '"{}"',
                ]
            ],
            shell=True,
        ),
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " param set ",
                    "/rosbridge_websocket ",
                    "std_msgs/msg/Bool ",
                    "true",
                ]
            ],
            shell=True,
        )
    )

    # ld.add_action(
    #     ExecuteProcess(
    #         cmd=[
    #             [
    #                 FindExecutable(name="ros2"),
    #                 " param set ",
    #                 "/gripper_camera ",
    #                 "depth_module.enable_auto_exposure ",
    #                 "true",
    #             ]
    #         ],
    #         shell=True,
    #     )
    # )

    # Move To Pre-grasp Action Server
    move_to_pregrasp_node = Node(
        package="stretch_web_teleop",
        executable="move_to_pregrasp.py",
        output="screen",
        arguments=[LaunchConfiguration("params")],
        parameters=[{
                "use_sim_time": True}],
        remappings=[
            ("/camera/aligned_depth_to_color/camera_info", "/camera/depth/camera_info"),
            ("/camera/aligned_depth_to_color/image_raw/compressedDepth", "/camera/depth/image_rect_raw/compressed")
        ]
    )
    ld.add_action(move_to_pregrasp_node)

    # Text to speech
    text_to_speech_node = Node(
        package="stretch_web_teleop",
        executable="text_to_speech.py",
        output="screen",
        arguments=[LaunchConfiguration("tts_engine")],
        parameters=[],
    )
    ld.add_action(text_to_speech_node)

    # if stretch_tool == "eoa_wrist_dw3_tool_tablet_12in":
    #     detect_body_landmarks_node = Node(
    #         package="stretch_show_tablet",
    #         executable="detect_body_landmarks",
    #         output="screen",
    #     )
    #     ld.add_action(detect_body_landmarks_node)

    #     plan_tablet_pose_node = Node(
    #         package="stretch_show_tablet",
    #         executable="plan_tablet_pose_service",
    #         output="screen",
    #     )
    #     ld.add_action(plan_tablet_pose_node)

    #     show_tablet_node = Node(
    #         package="stretch_show_tablet",
    #         executable="show_tablet_server",
    #         output="screen",
    #     )
    #     ld.add_action(show_tablet_node)

    return ld
