from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_params = [
        {
            'camera_port': '/dev/hello-gripper-camera',
            'publish_topic':'/gripper_camera/image_raw',

            # Properties
            # 'format': 'MJPG',
            # 'size': [1280, 800],
            # 'fps': 100,
            # 'brightness': 10,
            # 'contrast': 30,
            # 'saturation': 80,
            # 'hue': 0,
            # 'gamma': 80,
            # 'gain': 10,
            # 'white_balence_temp': 4600,
            # 'sharpness': 3,
            # 'backlight': 1
        }
    ]
    usb_cam = Node(package='stretch_core',
                   executable='usb_cam',
                   emulate_tty=True,
                   output='screen',
                   parameters=camera_params,
                   name='gripper_camera')
    return LaunchDescription([usb_cam])