import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_rtabmap_path = get_package_share_directory('stretch_rtabmap')
    stretch_navigation_path = get_package_share_directory('stretch_nav2')

    teleop_type_param = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])
    rviz_config = DeclareLaunchArgument('rviz_config',
                                        default_value=stretch_rtabmap_path + '/' + 'rviz/rtabmap_nav.rviz')
     
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'True'}.items())

    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/d435i_high_resolution.launch.py']))
    
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    rtabmap_navigation_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('grid_map', 'map')
        ],
        parameters=[{'Mem/IncrementalMemory': 'false'}],
        output='screen',
        )

    params_file_param = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(stretch_navigation_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(stretch_navigation_path, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': LaunchConfiguration('params_file')}.items())
    
    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())
    
    rviz_launch = Node(package='rviz2', executable='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        respawn=True,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        )

    return LaunchDescription([
        teleop_type_param,
        rviz_param,
        rviz_config,
        stretch_driver_launch,
        d435i_launch,
        rplidar_launch,
        rtabmap_navigation_node,
        params_file_param,
        nav2_launch,
        base_teleop_launch,
        rviz_launch,
    ])
