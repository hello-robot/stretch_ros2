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

from sensor_msgs.msg import LaserScan

from threading import Thread
from threading import Event


@pytest.fixture
def rplidar_proc():
    # Launch a process to test
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/rplidar.launch.py'])
        )

# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(rplidar_proc):
    """Launch process to run aruco detection."""
    return launch.LaunchDescription([
        rplidar_proc,
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])

@pytest.mark.launch(fixture=launch_description)
def test_rplidar_topics_published():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_rplidar_subscribers()

        scan_msgs_received_flag = node.scan_msg_event_object.wait(timeout=5.0)
        assert scan_msgs_received_flag, 'Did not receive scan msg!'
        
        filtered_scan_msgs_received_flag = node.filtered_scan_msg_event_object.wait(timeout=5.0) 
        assert filtered_scan_msgs_received_flag, 'Did not receive filtered scan msg!'
        
    finally:
        rclpy.shutdown()

@pytest.mark.launch(fixture=launch_description)
def test_rplidar_detection_rate():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_rplidar_subscribers()

        scan_msgs_received_flag = node.scan_msg_event_object.wait(timeout=5.0)
        assert scan_msgs_received_flag, 'Did not receive scan msg!'
    
        ts = time.time()
        node.count_scan = 0     
        while rclpy.ok() and time.time() - ts < 1.0:
            pass
        count_scan = node.count_scan
        
        ts = time.time()
        node.count_filtered_scan = 0
        while rclpy.ok() and time.time() - ts < 1.0:
            pass
        count_filtered_scan = node.count_filtered_scan
        
        assert count_scan >= 7, 'Scan detection frequency is: {0}Hz'.format(count_scan)
        assert count_filtered_scan >= 7, 'Filtered scan detection frequency is: {0}Hz'.format(count_filtered_scan)
    
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.scan_msg_event_object = Event()
        self.filtered_scan_msg_event_object = Event()
        self.count_scan = 0
        self.count_filtered_scan = 0
    
    def start_rplidar_subscribers(self):
        # Create subscribers
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_subscriber_callback,
            10
        )
        
        self.filtered_scan_subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.filtered_scan_subscriber_callback,
            10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()
    
    def scan_subscriber_callback(self, data):
        self.scan_msg_event_object.set()
        self.count_scan += 1
    
    def filtered_scan_subscriber_callback(self, data):
        self.filtered_scan_msg_event_object.set()
        self.count_filtered_scan += 1
