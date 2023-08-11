from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    funmap = Node(package='stretch_funmap',
                  executable='funmap',
                  output='screen',
                  parameters=funmap_params)

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        funmap,
    ])
