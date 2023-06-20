from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'position', 'broadcast_odom_tf': 'True', 'fail_out_of_range_goal': 'False'}.items(),
    )

    mapping_on = DeclareLaunchArgument(
        'mapping_on',
        default_value='False', choices=['True', 'False'],
        description='Whether to turn FUNMAP mapping on/off'
    )

    hello_world_on = DeclareLaunchArgument(
        'hello_world_on',
        default_value='False', choices=['True', 'False'],
        description='Whether to turn hello world demo on/off'
    )

    open_drawer_on = DeclareLaunchArgument(
        'open_drawer_on',
        default_value='False', choices=['True', 'False'],
        description='Whether to turn open drawer demo on/off'
    )

    clean_surface_on = DeclareLaunchArgument(
        'clean_surface_on',
        default_value='False', choices=['True', 'False'],
        description='Whether to turn clean surface demo on/off'
    )

    grasp_object_on = DeclareLaunchArgument(
        'grasp_object_on',
        default_value='False', choices=['True', 'False'],
        description='Whether to turn grasp object demo on/off'
    )

    handover_object_on = DeclareLaunchArgument(
        'handover_object_on',
        default_value='False', choices=['True', 'False'],
        description='Whether to turn handover object on/off'
    )

    keyboard_teleop_params = [
        {'mapping_on': LaunchConfiguration('mapping_on'),
         'hello_world_on': LaunchConfiguration('hello_world_on'),
         'open_drawer_on': LaunchConfiguration('open_drawer_on'),
         'clean_surface_on': LaunchConfiguration('clean_surface_on'),
         'grasp_object_on': LaunchConfiguration('grasp_object_on'),
         'handover_object_on': LaunchConfiguration('handover_object_on')}
    ]

    keyboard_teleop = Node(
            package='stretch_core',
            executable='keyboard_teleop',
            output='screen',
            prefix='xterm -e',
            parameters=keyboard_teleop_params
    )

    return LaunchDescription([
        mapping_on,
        hello_world_on,
        open_drawer_on,
        clean_surface_on,
        grasp_object_on,
        handover_object_on,
        stretch_driver,
        keyboard_teleop,
    ])