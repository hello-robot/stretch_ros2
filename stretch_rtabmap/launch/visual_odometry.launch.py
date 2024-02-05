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
    
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])
    rviz_config = DeclareLaunchArgument('rviz_config',
                                        default_value=stretch_rtabmap_path + '/' + 'rviz/rtabmap.rviz')
     
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation'}.items())

    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/d435i_high_resolution.launch.py']))

    rtabmap_mapping_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('grid_map', 'map')
        ],
        output='screen',
        )
    
    rviz_launch = Node(package='rviz2', executable='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        respawn=True,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        )

    return LaunchDescription([
        rviz_param,
        rviz_config,
        stretch_driver_launch,
        d435i_launch,
        rtabmap_mapping_node,
        rviz_launch,
    ])
