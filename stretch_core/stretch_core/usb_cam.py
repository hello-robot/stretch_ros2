#! /usr/bin/env python3

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import threading
from rclpy.executors import MultiThreadedExecutor
import os


# More UVC Video capture properties here:
# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
# 
# Arducam wiki info site
# https://docs.arducam.com/UVC-Camera/Appilcation-Note/OpenCV-Python-GStreamer-on-linux/ 
# 
# Setting Video formates using v4l2
# http://trac.gateworks.com/wiki/linux/v4l2


class USBCamNode(Node):

    def __init__(self):
        super().__init__('usb_cam_node')

        self.declare_parameter('camera_port','/dev/hello-navigation-camera')
        self.declare_parameter('publish_topic','/usb_cam/image_raw')

        # Properties
        self.declare_parameter('format', 'MJPG')
        self.declare_parameter('size', [1280, 800])
        self.declare_parameter('fps', 100)
        self.declare_parameter('brightness', 10)
        self.declare_parameter('contrast', 30)
        self.declare_parameter('saturation', 80)
        self.declare_parameter('hue', 0)
        self.declare_parameter('gamma', 80)
        self.declare_parameter('gain', 10)
        self.declare_parameter('white_balence_temp', 4600)
        self.declare_parameter('sharpness', 3)
        self.declare_parameter('backlight', 1)
        
        self.publisher = self.create_publisher(Image, self.get_parameter('publish_topic').value, 15)
        timer_period = 0.001  # seconds
        self.cv_bridge = CvBridge()
        self.image_msg = None

        self.usb_cam_properties = {
            'format' : self.get_parameter('format').value,
            'size' : self.get_parameter('size').value,
            'fps' : self.get_parameter('fps').value,
            'brightness' : self.get_parameter('brightness').value,
            'contrast' : self.get_parameter('contrast').value,
            'saturation' : self.get_parameter('saturation').value,
            'hue' : self.get_parameter('hue').value,
            'gamma' : self.get_parameter('gamma').value,
            'gain' : self.get_parameter('gain').value,
            'white_balence_temp' : self.get_parameter('white_balence_temp').value,
            'sharpness' : self.get_parameter('sharpness').value,
            'backlight' : self.get_parameter('backlight').value,
        }
        self.uvc_camera = self.setup_uvc_camera()
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)


    def setup_uvc_camera(self):
        cap = cv2.VideoCapture(self.get_parameter('camera_port').value)

        if self.usb_cam_properties['format']:
            fourcc_value = cv2.VideoWriter_fourcc(*self.usb_cam_properties['format'])
            cap.set(cv2.CAP_PROP_FOURCC, fourcc_value)
        if self.usb_cam_properties['fps']:
            cap.set(cv2.CAP_PROP_FPS, self.usb_cam_properties['fps'])
        if self.usb_cam_properties['size']:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.usb_cam_properties['size'][0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.usb_cam_properties['size'][1])
        if self.usb_cam_properties['brightness']:
            cap.set(cv2.CAP_PROP_BRIGHTNESS,self.usb_cam_properties['brightness'])
        if self.usb_cam_properties['contrast']:
            cap.set(cv2.CAP_PROP_CONTRAST,self.usb_cam_properties['contrast'])
        if self.usb_cam_properties['hue']:
            cap.set(cv2.CAP_PROP_HUE,self.usb_cam_properties['hue'])
        if self.usb_cam_properties['gamma']:
            cap.set(cv2.CAP_PROP_GAMMA,self.usb_cam_properties['gamma'])
        if self.usb_cam_properties['white_balence_temp']:
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE,self.usb_cam_properties['white_balence_temp'])
        if self.usb_cam_properties['backlight']:
            cap.set(cv2.CAP_PROP_BACKLIGHT,self.usb_cam_properties['backlight'])
        
        if cap:
            self.get_logger().info("Starting USB Camera Node...")
            self.get_logger().info(f"Camera Port: {self.get_parameter('camera_port').value}")
            self.get_logger().info(f"Published to topic: {self.get_parameter('publish_topic').value}")
            self.get_logger().info(f"Camera Properties Applied: {self.usb_cam_properties}")
            
        return cap

    def timer_callback(self):
        if self.image_msg is not None:
            self.publisher.publish(self.image_msg)

    def timer_callback2(self):
        try:
            ret, image_uvc = self.uvc_camera.read()
            # Convert the OpenCV image to a ROS Image message
            self.image_msg = self.cv_bridge.cv2_to_imgmsg(image_uvc, encoding='bgr8')
        except Exception as e:
            print(f"Error UVC Cam: {e}")



def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor()
        image_publisher_node = USBCamNode()
        executor.add_node(image_publisher_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            image_publisher_node.destroy_node()
    except KeyboardInterrupt:
        executor.shutdown()
        image_publisher_node.destroy_node()

if __name__ == '__main__':
    main()