#! /usr/bin/env python3

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# More UVC Video capture properties here:
# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
# 
# Arducam wiki info site
# https://docs.arducam.com/UVC-Camera/Appilcation-Note/OpenCV-Python-GStreamer-on-linux/ 
# 
# Setting Video formates using v4l2
# http://trac.gateworks.com/wiki/linux/v4l2

class USBCamNode(Node):
    """
    Custom USB Cam node that uses OpenCV library to capture and publish video output from USB cam devices.

    Default camera_port = /dev/hello-navigation-camera
    Default publish_topic = /usb_cam/image_raw

    Optional Configurable Properties = [format, size, fps, brightness, contrast, saturation, hue, gamma, 
                           gain, white_balence_temp, sharpness, backlight]
    """
    def __init__(self):
        super().__init__('usb_cam_node')

        self.declare_parameter('camera_port','/dev/hello-navigation-camera') # Default
        self.declare_parameter('publish_topic','/usb_cam/image_raw') # Default

        # Uninitialized Optional Camera Properties
        self.declare_parameter('format', rclpy.Parameter.Type.STRING)
        self.declare_parameter('size', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('fps', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('brightness', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('contrast', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('saturation', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('hue', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('gamma', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('gain', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('white_balence_temp', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('sharpness', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('backlight', rclpy.Parameter.Type.INTEGER)

        self.properties = {'format':None,
                            'size':None,
                            'fps':None,
                            'brightness':None,
                            'contrast':None,
                            'saturation':None,
                            'hue':None,
                            'gamma':None,
                            'gain':None,
                            'white_balence_temp':None,
                            'sharpness':None,
                            'backlight':None}

        self.publisher = self.create_publisher(Image, self.get_parameter('publish_topic').value, 15)
        timer_period = 0.001  # seconds
        self.cv_bridge = CvBridge()
        self.image_msg = None
        self.uvc_camera = self.setup_uvc_camera()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
    
    def is_parameter_initialized(self, param):
        # Ro2 does not have an API to check if an param is uninitalized
        try:
            value = self.get_parameter(param).value
            return True
        except rclpy.exceptions.ParameterUninitializedException:
            return False

    def setup_uvc_camera(self):
        cap = cv2.VideoCapture(self.get_parameter('camera_port').value)
        if self.is_parameter_initialized('format'):
            fourcc_value = cv2.VideoWriter_fourcc(*self.get_parameter('format').value)
            cap.set(cv2.CAP_PROP_FOURCC, fourcc_value)
            self.properties['format'] = self.get_parameter('format').value

        if self.is_parameter_initialized('fps'):
            cap.set(cv2.CAP_PROP_FPS, self.get_parameter('fps').value)
            self.properties['fps'] = self.get_parameter('fps').value

        if self.is_parameter_initialized('size'):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('size').value[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('size').value[1])
            self.properties['size'] = self.get_parameter('size').value

        if self.is_parameter_initialized('brightness'):
            cap.set(cv2.CAP_PROP_BRIGHTNESS,self.get_parameter('brightness').value)
            self.properties['brightness'] = self.get_parameter('brightness').value

        if self.is_parameter_initialized('contrast'):
            cap.set(cv2.CAP_PROP_CONTRAST,self.get_parameter('contrast').value)
            self.properties['contrast'] = self.get_parameter('contrast').value

        if self.is_parameter_initialized('hue'):
            cap.set(cv2.CAP_PROP_HUE,self.get_parameter('hue').value)
            self.properties['hue'] = self.get_parameter('hue').value

        if self.is_parameter_initialized('gamma'):
            cap.set(cv2.CAP_PROP_GAMMA,self.get_parameter('gamma').value)
            self.properties['gamma'] = self.get_parameter('gamma').value

        if self.is_parameter_initialized('gain'):
            cap.set(cv2.CAP_PROP_GAIN,self.get_parameter('gain').value)
            self.properties['gain'] = self.get_parameter('gain').value

        if self.is_parameter_initialized('sharpness'):
            cap.set(cv2.CAP_PROP_SHARPNESS,self.get_parameter('sharpness').value)
            self.properties['sharpness'] = self.get_parameter('sharpness').value

        if self.is_parameter_initialized('saturation'):
            cap.set(cv2.CAP_PROP_SATURATION,self.get_parameter('saturation').value)
            self.properties['saturation'] = self.get_parameter('saturation').value

        if self.is_parameter_initialized('white_balence_temp'):
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE,self.get_parameter('white_balence_temp').value)
            self.properties['white_balence_temp'] = self.get_parameter('white_balence_temp').value

        if self.is_parameter_initialized('backlight'):
            cap.set(cv2.CAP_PROP_BACKLIGHT,self.get_parameter('backlight').value)
            self.properties['backlight'] = self.get_parameter('backlight').value
        
        if cap:
            self.get_logger().info("Starting USB Camera Node...")
            self.get_logger().info(f"Camera Port: {self.get_parameter('camera_port').value}")
            self.get_logger().info(f"Published to topic: {self.get_parameter('publish_topic').value}")
            self.get_logger().info(f"Camera Properties Applied: {self.properties}")
        else:
            self.get_logger().error("Unable to start USB Camera Node")
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