from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    print_debug_arg = DeclareLaunchArgument('print_debug', default_value='False', description='Print debug info')
    camera_params = [
        {
            'camera_port': '/dev/hello-navigation-camera',
            'publish_topic':'/navigation_camera/image_raw',
            'print_debug': LaunchConfiguration('print_debug'),

            # Properties
            'format': 'MJPG',
            'size': [1024, 768],
            'fps': 100,
            'brightness': -40,
            'contrast': 40,
            'saturation': 60,
            'hue': 0,
            'gamma': 80,
            'gain': 50,
            'white_balence_temp': 4250,
            'sharpness': 100,
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