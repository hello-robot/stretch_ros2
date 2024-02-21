from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    print_debug_arg = DeclareLaunchArgument('print_debug', default_value='False', description='Print debug info')
    camera_params = [
        {
            'camera_port': '/dev/hello-nav-head-camera',
            'publish_topic':'/navigation_camera/image_raw',
            'print_debug': LaunchConfiguration('print_debug'),

            # Properties
            'format': 'MJPG',
            'size': [800, 600],
            'fps': 100,
            'brightness': 10,
            'contrast': 30,
            'saturation': 80,
            'hue': 0,
            'gamma': 80,
            'gain': 10,
            'white_balence_temp': 4600,
            'sharpness': 3,
            'backlight': 1
        }
    ]
    usb_cam = Node(package='stretch_core',
                   executable='usb_cam',
                   emulate_tty=True,
                   output='screen',
                   parameters=camera_params,
                   name='navigation_camera')
    return LaunchDescription([print_debug_arg, usb_cam])