import unittest
import pytest

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
import launch_testing.markers

import time
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    stretch_core_path = get_package_share_path('stretch_core')

    stretch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_realsense.launch.py']),
    )

    stretch_aruco_detect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_aruco.launch.py']),
        launch_arguments={'mode': 'manipulation'}.items()
    )

    return LaunchDescription([stretch_realsense,
                              stretch_aruco_detect,
                              ReadyToTest()])


class TestAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        time.sleep(20) # wait for launch file to load

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_aruco_detect')
        self.rgb_flag = None
        self.depth_flag = None
        self.info_flag = None
        self.marker_flag = None

    def tearDown(self):
        self.node.destroy_node()

    def rgb_img_cb(self, msg):
        image = msg
        if(image.header.stamp is not None):
            self.rgb_flag = True

    def depth_img_cb(self, msg):
        image = msg
        if(image.header.stamp is not None):
            self.depth_flag = True

    def cam_info_cb(self, msg):
        info = msg
        if(info.header.stamp is not None):
            self.info_flag = True

    def marker_cb(self, msg):
        marker_array = msg
        self.marker_flag = False
        if(len(marker_array.markers)):
            self.marker_flag = True

    def test_camera_feed(self, proc_output):
        rgb_image_subscriber = self.node.create_subscription(Image, '/camera/color/image_raw', self.rgb_img_cb, 1)
        depth_image_subscriber = self.node.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_img_cb, 1)
        camera_info_subscriber = self.node.create_subscription(CameraInfo, '/camera/color/camera_info', self.cam_info_cb, 1)
        
        ts = time.time()
        while rclpy.ok() and time.time() - ts < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.rgb_flag)
        self.assertTrue(self.depth_flag)
        self.assertTrue(self.info_flag)

    def test_aruco_detection(self, proc_output):
        aruco_marker_subscriber = self.node.create_subscription(MarkerArray, '/aruco/marker_array', self.marker_cb, 1)

        ts = time.time()
        count = 0
        while rclpy.ok() and time.time() - ts < 1.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if(self.marker_flag == True):
                count += 1
        
        print('Detection frequency is: {0}Hz'.format(count))
        self.assertTrue(self.marker_flag)
        self.assertGreaterEqual(count, 10)