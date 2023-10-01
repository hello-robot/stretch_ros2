import os
import time

import launch
import launch_pytest
import pytest

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from sensor_msgs.msg import Image, Imu

from threading import Thread
from threading import Event


@pytest.fixture
def d435i_high_res_proc():
    # Launch a process to test
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/d435i_high_resolution.launch.py'])
        )

# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(d435i_high_res_proc):
    """Launch process to run aruco detection."""
    return launch.LaunchDescription([
        d435i_high_res_proc,
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])

@pytest.mark.launch(fixture=launch_description)
def test_camera_topics_published():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_camera_subscribers()

        camera_depth_msgs_received_flag = node.camera_depth_msg_event_object.wait(timeout=20.0)  # Higher delay to allow camera driver to load
        assert camera_depth_msgs_received_flag, 'Did not receive camera depth msg!'
        
        camera_color_msgs_received_flag = node.camera_color_msg_event_object.wait(timeout=5.0) 
        assert camera_color_msgs_received_flag, 'Did not receive camera color msg!'
        
        camera_accel_msgs_received_flag = node.camera_accel_msg_event_object.wait(timeout=5.0)
        assert camera_accel_msgs_received_flag, 'Did not receive camera accel msg!'
    
    finally:
        rclpy.shutdown()

@pytest.mark.launch(fixture=launch_description)
def test_camera_detection_rate():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_camera_subscribers()

        camera_depth_msgs_received_flag = node.camera_depth_msg_event_object.wait(timeout=20.0) # Higher delay to allow camera driver to load
        assert camera_depth_msgs_received_flag, 'Did not receive camera depth msg!'
    
        ts = time.time()
        node.count_accel = 0     
        while rclpy.ok() and time.time() - ts < 1.0:
            pass
        count_accel = node.count_accel
        
        ts = time.time()
        node.count_rgb = 0
        while rclpy.ok() and time.time() - ts < 1.0:
            pass
        count_rgb = node.count_rgb

        ts = time.time()
        node.count_depth = 0
        while rclpy.ok() and time.time() - ts < 1.0:
            pass
        count_depth = node.count_depth
        
        assert count_accel >= 14, 'Camera accel detection frequency is: {0}Hz'.format(count_accel)
        assert count_rgb >= 14, 'Camera rgb image detection frequency is: {0}Hz'.format(count_rgb)
        assert count_depth >= 14, 'Camera depth image detection frequency is: {0}Hz'.format(count_depth)
    
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.camera_accel_msg_event_object = Event()
        self.camera_color_msg_event_object = Event()
        self.camera_depth_msg_event_object = Event()
        self.count_accel = 0
        self.count_rgb = 0
        self.count_depth = 0
    
    def start_camera_subscribers(self):
        # Create subscribers
        
        self.camera_accel_subscription = self.create_subscription(
            Imu,
            '/camera/accel/sample_corrected',
            self.camera_accel_subscriber_callback,
            10
        )
        
        self.camera_color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_color_subscriber_callback,
            10
        )

        self.camera_depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.camera_depth_subscriber_callback,
            10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()
    
    def camera_accel_subscriber_callback(self, data):
        self.camera_accel_msg_event_object.set()
        self.count_accel += 1
    
    def camera_color_subscriber_callback(self, data):
        self.camera_color_msg_event_object.set()
        self.count_rgb += 1

    def camera_depth_subscriber_callback(self, data):
        self.camera_depth_msg_event_object.set()
        self.count_depth += 1
