import unittest
import pytest

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
import launch_testing.markers

from std_srvs.srv import Trigger, SetBool

import time
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    stretch_core_path = get_package_share_path('stretch_core')

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation'}.items()
    )

    return LaunchDescription([stretch_driver_launch,
                              ReadyToTest()])


class TestServices(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        time.sleep(3) # wait for launch file to load

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_services')

    def tearDown(self):
        self.node.destroy_node()

    def test_switch_to_position_mode(self, proc_output):
        mode_client = self.node.create_client(Trigger, '/switch_to_position_mode')
        self.assertTrue(mode_client.wait_for_service(timeout_sec=1.0))
        request = Trigger.Request()
        promise = mode_client.call_async(request)
        ts = time.time()
        while rclpy.ok() and time.time() - ts < 3.0 and not promise.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(promise.done())
        response = promise.result()
        self.node.get_logger().info(response.message)
        self.assertTrue(response.success)

    def test_switch_to_navigation_mode(self, proc_output):
        mode_client = self.node.create_client(Trigger, '/switch_to_navigation_mode')
        self.assertTrue(mode_client.wait_for_service(timeout_sec=1.0))
        request = Trigger.Request()
        promise = mode_client.call_async(request)
        ts = time.time()
        while rclpy.ok() and time.time() - ts < 3.0 and not promise.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(promise.done())
        response = promise.result()
        self.node.get_logger().info(response.message)
        self.assertTrue(response.success)

    def test_switch_to_manipulation_mode(self, proc_output):
        mode_client = self.node.create_client(Trigger, '/switch_to_manipulation_mode')
        self.assertTrue(mode_client.wait_for_service(timeout_sec=1.0))
        request = Trigger.Request()
        promise = mode_client.call_async(request)
        ts = time.time()
        while rclpy.ok() and time.time() - ts < 3.0 and not promise.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(promise.done())
        response = promise.result()
        self.node.get_logger().info(response.message)
        self.assertTrue(response.success)

    def test_stop_the_robot(self, proc_output):
        mode_client = self.node.create_client(Trigger, '/stop_the_robot')
        self.assertTrue(mode_client.wait_for_service(timeout_sec=1.0))
        request = Trigger.Request()
        promise = mode_client.call_async(request)
        ts = time.time()
        while rclpy.ok() and time.time() - ts < 3.0 and not promise.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(promise.done())
        response = promise.result()
        self.node.get_logger().info(response.message)
        self.assertTrue(response.success)

    def test_runstop(self, proc_output):
        mode_client = self.node.create_client(SetBool, '/runstop')
        self.assertTrue(mode_client.wait_for_service(timeout_sec=1.0))
        request = SetBool.Request()
        request.data = True
        promise = mode_client.call_async(request)
        ts = time.time()
        while rclpy.ok() and time.time() - ts < 3.0 and not promise.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(promise.done())
        response = promise.result()
        self.node.get_logger().info(response.message)
        self.assertTrue(response.success)

        request.data = False
        promise = mode_client.call_async(request)
        ts = time.time()
        while rclpy.ok() and time.time() - ts < 3.0 and not promise.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(promise.done())
        response = promise.result()
        self.node.get_logger().info(response.message)
        self.assertTrue(response.success)
