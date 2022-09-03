import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node

configurable_parameters = [{'name': 'resolution',                          'default': 'high', 'description': 'resolution of the color/depth images low or high'},
                           {'name': 'publish_frustum_viz',                 'default': 'false', 'description': 'whether to pub viz of camera frustums'},
                           {'name': 'publish_upright_img',                 'default': 'true', 'description': 'whether to pub rotated upright color image'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def generate_launch_description():
     publish_frustum_viz = LaunchConfiguration('publish_frustum_viz')
     publish_upright_img = LaunchConfiguration('publish_upright_img')
     resolution = LaunchConfiguration('resolution')

     d435i_high_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/d435i_high_resolution.launch.py']),
               condition=LaunchConfigurationEquals('resolution', 'high')
          )

     d435i_low_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('stretch_core'), 'launch'),
               '/d435i_low_resolution.launch.py']),
               condition=LaunchConfigurationEquals('resolution', 'low')
          )

     d435i_configure = Node(
          package='stretch_core',
          executable='d435i_configure',
          output='screen',
          )

     d435i_frustum_visualizer = Node(
          condition=IfCondition(
               PythonExpression([
               publish_frustum_viz
               ])),
          package='stretch_core',
          executable='d435i_frustum_visualizer',
          output='screen',
          )
     
     upright_rotater = Node(
          condition=IfCondition(
               PythonExpression([
               publish_upright_img
               ])),
          package='image_rotate',
          executable='upright_rotater',
          remappings=[
               ('image', '/camera/color/image_raw'),
               ('rotated/image', '/camera/color/upright_image_raw'),
          ],
          parameters=[
               {'target_frame_id': "", 
               'target_x': '-1.5708'}
               ],
          )

     return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
          d435i_high_launch,
          d435i_low_launch,
          # d435i_configure,
          # d435i_frustum_visualizer,
          # upright_rotater,
          ])
