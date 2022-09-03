import unittest
import pytest

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

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'manipulation'}.items()
    )

    return LaunchDescription([stretch_driver_launch,
                              ReadyToTest()])


class TestAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        time.sleep(3) # wait for launch file to load

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_action')

    def tearDown(self):
        self.node.destroy_node()

    def test_action_server_exists(self, proc_output):
        action_client = ActionClient(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        self.assertTrue(action_client.wait_for_server(timeout_sec=1.0))
        self.assertTrue(action_client.server_is_ready())
        goal = FollowJointTrajectory.Goal()
        send_goal_promise = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_promise, timeout_sec=1.0)
        self.assertTrue(send_goal_promise.done())
        client_goal_handle = send_goal_promise.result()
        self.assertTrue(client_goal_handle.accepted)
        result_promise = client_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_promise, timeout_sec=1.0)
        self.assertTrue(result_promise.done())
        status = result_promise.result().status
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        result = result_promise.result().result
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.INVALID_JOINTS)

    def test_feedback_correctly_formed(self, proc_output):
        action_client = ActionClient(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        self.assertTrue(action_client.wait_for_server(timeout_sec=1.0))
        self.assertTrue(action_client.server_is_ready())
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_lift']
        # goal.trajectory.
        goal.multi_dof_trajectory.joint_names = ['position']
