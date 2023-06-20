import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_unified_path = get_package_share_directory('stretch_unified')
    stretch_navigation_path = get_package_share_directory('stretch_nav2')

    map_path = "{0}/maps/".format(os.getenv('HELLO_FLEET_PATH'))
    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(map_path, 'hq.yaml'),
        description='Full path to the map yaml file to use for navigation')
    
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(stretch_unified_path, 'rviz/stretch_unified.rviz'),
        description='Full path to the map yaml file to use for navigation')
    
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'trajectory', 'broadcast_odom_tf': 'True'}.items())

    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/d435i_high_resolution.launch.py']))
    
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))
    
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_unified_path, '/launch/manipulation.launch.py']))

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_unified_path, '/launch/navigation.launch.py']),
        launch_arguments={'map': LaunchConfiguration('map')}.items())
    
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_unified_path, '/launch/perception.launch.py']))
    
    rviz_node = Node(package='rviz2', executable='rviz2',
                     output='log',
                     respawn=True,
                     arguments=['-d', LaunchConfiguration('rviz_config')],
                    #  parameters=rviz_parameters,
                     )

    return LaunchDescription([
        map_path_param,
        rviz_config,
        stretch_driver_launch,
        d435i_launch,
        rplidar_launch,
        manipulation_launch,
        navigation_launch,
        perception_launch,
        rviz_node,
    ])
