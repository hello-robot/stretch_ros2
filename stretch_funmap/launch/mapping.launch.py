import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

debug_directory_path = os.path.join(os.getenv('HELLO_FLEET_PATH'), 'debug') + '/' if os.getenv('HELLO_FLEET_PATH') else ''

configurable_parameters = [{'name': 'map_yaml',         'default': '', 'description': 'previously captured FUNMAP map (optional)'},                           
                           {'name': 'debug_directory',  'default': debug_directory_path, 'description': 'directory where debug imagery is saved'},
                           ]
                           
def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    debug_directory = LaunchConfiguration('debug_directory')

    funmap_params = [
        {'map_yaml': map_yaml,
         'debug_directory': debug_directory,
        }
    ]

    d435i_high_resolution = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/d435i_high_resolution.launch.py'])
          )
    
    stretch_driver = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/stretch_driver.launch.py']),
               launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'position'}.items()
          )

    funmap = Node(package='stretch_funmap',
                  executable='funmap',
                  output='screen',
                  parameters=funmap_params,
                  remappings=[
            ('/move_base_simple/goal', '/goal_pose'),
        ])

    rviz_config_path = os.path.join(get_package_share_directory('stretch_funmap'), 'rviz', 'stretch_mapping.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        d435i_high_resolution,
        stretch_driver,
        funmap,
        rviz_node
    ])
