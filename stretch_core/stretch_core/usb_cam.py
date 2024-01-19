#! /usr/bin/env python3

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import threading
from rclpy.executors import MultiThreadedExecutor
import os

UVC_COLOR_SIZE = [1280, 800] # [3840,2880] [1920, 1080] [1280, 720] [1280, 800] [640, 480]
UVC_FPS = 100

UVC_VIDEO_INDEX = '/dev/hello-navigation-camera'
UVC_VIDEO_FORMAT = 'MJPG' # MJPG YUYV 

UVC_BRIGHTNESS =  10         # brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
UVC_CONTRAST = 30            # contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32
UVC_SATURATION = 80          # saturation 0x00980902 (int)    : min=0 max=128 step=1 default=90 value=90
UVC_HUE = 0                  # hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
UVC_GAMMA = 80               # gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
UVC_GAIN = 10                 # gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
UVC_WB_TEMP = 4600           # white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
UVC_SHARPNESS = 3            # sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3
UVC_BACKLIT_COMP = 1         # backlight_compensation 0x0098091c (int)    : min=0 max=2 step=1 default=1 value=1
UVC_EXPOSURE_TIME = 157      # exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=157 flags=inactive

# More UVC Video capture properties here:
# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
# 
# Arducam wiki info site
# https://docs.arducam.com/UVC-Camera/Appilcation-Note/OpenCV-Python-GStreamer-on-linux/ 
# 
# Setting Video formates using v4l2
# http://trac.gateworks.com/wiki/linux/v4l2
#
#


def setup_uvc_camera(device_index, size, fps):
    cap = cv2.VideoCapture(device_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap



UVC_SETTINGS = {'brightness':UVC_BRIGHTNESS,
                'contrast':UVC_CONTRAST,
                'hue':UVC_HUE,
                'gamma':UVC_GAMMA,
                'gain':UVC_GAIN,
                'white_balance_temperature':UVC_WB_TEMP,
                'sharpness':UVC_SHARPNESS,
                'backlight_compensation':UVC_BACKLIT_COMP,
                'exposure_time_absolute':UVC_EXPOSURE_TIME}

# Set video format and size
cmd = f"v4l2-ctl --device {UVC_VIDEO_INDEX} --set-fmt-video=width={UVC_COLOR_SIZE[0]},height={UVC_COLOR_SIZE[1]}"
os.system(cmd)

# Set UVC SettingsNone
for k in list(UVC_SETTINGS.keys()):
    cmd = f"v4l2-ctl --device {UVC_VIDEO_INDEX} --set-ctrl={k}={UVC_SETTINGS[k]}"
    os.system(cmd)




class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(Image, '/navigation_camera/image_raw', 15)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
        self.cv_bridge = CvBridge()
        self.uvc_camera = setup_uvc_camera(UVC_VIDEO_INDEX, UVC_COLOR_SIZE, UVC_FPS)
        self.image_msg = None

    def timer_callback(self):
        if self.image_msg is not None:
            self.publisher.publish(self.image_msg)

    def timer_callback2(self):
        try:
            ret, image_uvc = self.uvc_camera.read()
            # Convert the OpenCV image to a ROS Image message
            self.image_msg = self.cv_bridge.cv2_to_imgmsg(image_uvc, encoding='bgr8')
            # print("Updated")
        except Exception as e:
            print(f"Error UVC Cam: {e}")



def main(args=None):
    rclpy.init(args=args)

    image_publisher_node = ImagePublisherNode()

    executor = MultiThreadedExecutor()
    executor.add_node(image_publisher_node)
    try:
        rclpy.spin(image_publisher_node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()