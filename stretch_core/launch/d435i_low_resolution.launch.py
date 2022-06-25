import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals

configurable_parameters = [{'name': 'depth_module.profile',         'default': '424x240x15', 'description': 'depth module profile'},                           
                           {'name': 'rgb_camera.profile',           'default': '424x240x15', 'description': 'color image width'},
                           ]
                           
def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def generate_launch_description():     
     d435i_basic_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/d435i_basic.launch.py'])
          )

     logger = LogInfo(msg='D435i launched in low resolution')

     return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
          d435i_basic_launch,
          logger,
          ])
