import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

configurable_parameters = [{'name': 'map_yaml',         'default': '', 'description': 'previously captured FUNMAP map (optional)'},                           
                           {'name': 'debug_directory',  'default': '', 'description': 'directory where debug imagery is saved'},
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
        launch_arguments={'mode': 'manipulation', 'broadcast_odom_tf': 'True', 'fail_out_of_range_goal': 'False'}.items(),
    )
    
    stretch_description = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_description'), 'launch'),
               '/display.launch.py'])
            #    launch_arguments={'broadcast_odom_tf': 'True'}.items()
          )
    
    keyboard_teleop = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/keyboard_teleop.launch.py'])
          )

    funmap = Node(package='stretch_funmap',
                  executable='funmap',
                  output='screen',
                  parameters=funmap_params)

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        d435i_high_resolution,
        stretch_driver,
        # stretch_description,
        keyboard_teleop,
        funmap,
    ])
