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

from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import MarkerArray

from threading import Thread
from threading import Event


@pytest.fixture
def aruco_detection_proc():
    # Launch a process to test
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/stretch_aruco.launch.py'])
        )

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
def launch_description(aruco_detection_proc, d435i_high_res_proc):
    """Launch process to run aruco detection."""
    return launch.LaunchDescription([
        aruco_detection_proc,
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

        camera_info_msgs_received_flag = node.camera_info_msg_event_object.wait(timeout=20.0) # Higher delay to allow camera driver to load
        assert camera_info_msgs_received_flag, 'Did not receive camera info msg!'

        camera_color_msgs_received_flag = node.camera_color_msg_event_object.wait(timeout=5.0)
        assert camera_color_msgs_received_flag, 'Did not receive camera color msg!'

        camera_aligned_depth_msgs_received_flag = node.camera_aligned_depth_msg_event_object.wait(timeout=5.0)
        assert camera_aligned_depth_msgs_received_flag, 'Did not receive camera aligned depth msg!'
    
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=launch_description)
def test_aruco_topics_published():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_aruco_subscribers()

        aruco_marker_array_msgs_received_flag = node.aruco_marker_array_msg_event_object.wait(timeout=20.0) # Higher delay to allow camera driver to load
        assert aruco_marker_array_msgs_received_flag, 'Did not receive aruco marker array msg!'

        aruco_axes_msgs_received_flag = node.aruco_axes_msg_event_object.wait(timeout=5.0)
        assert aruco_axes_msgs_received_flag, 'Did not receive aruco axes msg!'

    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=launch_description)
def test_aruco_detection_rate():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_aruco_subscribers()

        aruco_marker_array_msgs_received_flag = node.aruco_marker_array_msg_event_object.wait(timeout=20.0) # Higher delay to allow camera driver to load
        assert aruco_marker_array_msgs_received_flag, 'Did not receive aruco marker array msg!'
    
        ts = time.time()
        node.count = 0
        while rclpy.ok() and time.time() - ts < 1.0:
            pass
        count = node.count
        
        assert count > 12, 'Detection frequency is: {0}Hz'.format(count)
    
    finally:
        rclpy.shutdown()

class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.camera_info_msg_event_object = Event()
        self.camera_color_msg_event_object = Event()
        self.camera_aligned_depth_msg_event_object = Event()
        self.aruco_marker_array_msg_event_object = Event()
        self.aruco_axes_msg_event_object = Event()
        self.count = 0
    
    def start_camera_subscribers(self):
        # Create subscribers
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_subscriber_callback,
            10
        )
        
        self.camera_color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_color_subscriber_callback,
            10
        )

        self.camera_aligned_depth_subscription = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.camera_aligned_depth_subscriber_callback,
            10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def start_aruco_subscribers(self):   
        self.aruco_marker_array_subscription = self.create_subscription(
            MarkerArray,
            '/aruco/marker_array',
            self.aruco_marker_array_subscriber_callback,
            10
        )

        self.aruco_axes_subscription = self.create_subscription(
            MarkerArray,
            '/aruco/axes',
            self.aruco_axes_subscriber_callback,
            10
        )
 
        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def camera_info_subscriber_callback(self, data):
        self.camera_info_msg_event_object.set()
    
    def camera_color_subscriber_callback(self, data):
        self.camera_color_msg_event_object.set()

    def camera_aligned_depth_subscriber_callback(self, data):
        self.camera_aligned_depth_msg_event_object.set()

    def aruco_marker_array_subscriber_callback(self, data):
        self.aruco_marker_array_msg_event_object.set()
        self.count += 1

    def aruco_axes_subscriber_callback(self, data):
        self.aruco_axes_msg_event_object.set()
