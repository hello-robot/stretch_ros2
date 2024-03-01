import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

json_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'HighAccuracyPreset.json')

configurable_parameters = [{'name': 'camera_namespace1',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'camera_name1',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'device_type1',                  'default': 'd435', 'description': 'camera unique name'},
                           {'name': 'json_file_path1',               'default': json_path, 'description': 'allows advanced configuration'},
                           {'name': 'depth_module.profile1',         'default': '424x240x15', 'description': 'depth module profile'},                           
                           {'name': 'enable_depth1',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile1',           'default': '424x240x15', 'description': 'color image width'},
                           {'name': 'enable_color1',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra11',                'default': 'true', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra21',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb1',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_confidence1',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps1',                     'default': '200', 'description': "''"},                           
                           {'name': 'accel_fps1',                    'default': '100', 'description': "''"},                           
                           {'name': 'enable_gyro1',                  'default': 'true', 'description': "''"},                           
                           {'name': 'enable_accel1',                 'default': 'true', 'description': "''"},                           
                           {'name': 'pointcloud.enable1',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter1',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter1','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync1',                  'default': 'true', 'description': "''"},                           
                           {'name': 'align_depth.enable1',           'default': 'true', 'description': "''"},                           
                           {'name': 'initial_reset1',                'default': 'true', 'description': "''"},                           
                           {'name': 'allow_no_texture_points1',      'default': 'true', 'description': "''"},
                           {'name': 'camera_namespace2',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'camera_name2',                  'default': 'gripper_camera', 'description': 'camera unique name'},
                           {'name': 'device_type2',                  'default': 'd405', 'description': 'camera unique name'},
                           {'name': 'json_file_path2',               'default': json_path, 'description': 'allows advanced configuration'},
                           {'name': 'depth_module.profile2',         'default': '480x270x15', 'description': 'depth module profile'},                           
                           {'name': 'depth_module.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'enable_depth2',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile2',           'default': '424x240x15', 'description': 'color image width'},
                           {'name': 'rgb_camera.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_color2',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra12',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra22',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb2',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_confidence2',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps2',                     'default': '200', 'description': "''"},                           
                           {'name': 'accel_fps2',                    'default': '100', 'description': "''"},                           
                           {'name': 'enable_gyro2',                  'default': 'true', 'description': "''"},                           
                           {'name': 'enable_accel2',                 'default': 'true', 'description': "''"},                           
                           {'name': 'pointcloud.enable2',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter2',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter2','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync2',                  'default': 'true', 'description': "''"},                           
                           {'name': 'align_depth.enable2',           'default': 'true', 'description': "''"},                           
                           {'name': 'initial_reset2',                'default': 'false', 'description': "''"},                           
                           {'name': 'allow_no_texture_points2',      'default': 'true', 'description': "''"},                            
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
     realsense_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('realsense2_camera'), 'launch'),
               '/rs_multi_camera_launch.py'])
          )

     d435i_accel_correction = Node(
          package='stretch_core',
          executable='d435i_accel_correction',
          output='screen',
     )

     return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
          realsense_launch,
          d435i_accel_correction,
     ])
