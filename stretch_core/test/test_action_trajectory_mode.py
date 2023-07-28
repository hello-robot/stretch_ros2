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
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    stretch_core_path = get_package_share_path('stretch_core')

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'trajectory'}.items()
    )

    return LaunchDescription([stretch_driver_launch,
                              ReadyToTest()])


class TestAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        time.sleep(3) # wait for launch file to load

    def setUp(self):
        self.node = rclpy.create_node('test_action')
        self.action_client = ActionClient(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')    
        self.action_client.wait_for_server(timeout_sec=1.0)

        self.joint_states_sub = self.node.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.joint_states_sub
        
        self.joint_state = None
    
    def tearDown(self):
        self.node.destroy_node()
        time.sleep(1.0)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def joint_states_callback(self, msg):
        self.joint_state = msg

    def test_action_server_exists(self, proc_output):
        self.assertTrue(self.action_client.server_is_ready())

    def test_feedback_correctly_formed(self, proc_output):
        goal = FollowJointTrajectory.Goal()
        
        # Send a blank trajectory goal
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        self.assertTrue(send_goal.done()) # Check if goal finished executing

        client_goal_handle = send_goal.result()
        self.assertTrue(client_goal_handle.accepted) # Check if goal request was accepted

        result_promise = client_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_promise, timeout_sec=1.0)
        self.assertTrue(result_promise.done()) # Check if result received

        status = result_promise.result().status
        self.assertEqual(status, GoalStatus.STATUS_ABORTED) # Check status code

        result = result_promise.result().result
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.INVALID_JOINTS) # Check result error code

    def test_arm_trajectory_goal_replaced(self, proc_output):
        # Test if a new goal replaces the current goal
        # x --------- x --------- x ....................... current trajectory
        # ....................... o --------- o --------- o new trajectory
        # x --------- x --------- o --------- o --------- o resultant trajectory
        print("########### Executing test_arm_trajectory_goal_replaced ###########")
        goal = FollowJointTrajectory.Goal()
        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=1.0)
        point1.time_from_start = duration1.to_msg()
        point2.time_from_start = duration2.to_msg()
        rclpy.spin_once(self.node)
        arm_index = self.joint_state.name.index('wrist_extension')
        arm_pos = self.joint_state.position[arm_index]
        print("arm pos is: {}".format(arm_pos))
        point1.positions = [arm_pos]
        point2.positions = [arm_pos+0.04]
        goal.trajectory.joint_names = ['wrist_extension']
        goal.trajectory.points = [point1, point2]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(0.5)
        self.assertTrue(send_goal.done()) # Check if goal finished executing

        for i in range(1,30):
            rclpy.spin_once(self.node)
        arm_pos = self.joint_state.position[arm_index]
        print("arm pos is: {}".format(arm_pos))
        point1.positions = [arm_pos]
        point2.positions = [arm_pos+0.04]
        goal.trajectory.joint_names = ['wrist_extension']
        goal.trajectory.points = [point1, point2]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(2.0)

    def test_trajectory_goal_queued(self, proc_output):
        # Test if a new goal in the future gets queued up for execution after the current goal
        # x --------- x --------- x ............................ current trajectory
        # .............................. o --------- o --------- o new trajectory
        # x --------- x --------- x .... o --------- o --------- o resultant trajectory
        print("########### Executing test_trajectory_goal_queued ###########")
        goal = FollowJointTrajectory.Goal()
        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()
        point3 = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=2.0)
        duration3 = Duration(seconds=5.0)
        point1.time_from_start = duration1.to_msg()
        point2.time_from_start = duration2.to_msg()
        point3.time_from_start = duration3.to_msg()
        point1.positions = [0.0]
        point2.positions = [0.5]
        point3.positions = [0.0]
        goal.trajectory.joint_names = ['joint_head_pan']
        goal.trajectory.points = [point1, point2, point3]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(2.0)

        # Send another goal to be executed 6 seconds from now
        delayed_time = self.node.get_clock().now().to_msg()
        delayed_time.sec += 6 # Add a four second delay
        goal.trajectory.header.stamp = delayed_time
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(6.0)

    def test_trajectory_goal_merged(self, proc_output):
        # Test if a new goal with some waypoints in the past replaces the current goal
        # by discarding the old waypoints while preserving the waypoints in the future
        # x --------- x --------- x ................. current trajectory
        # ................. o --------- o --------- o new trajectory
        # x --------- x --------- x --- o --------- o resultant trajectory
        print("########### Executing test_trajectory_goal_merged ###########")
        goal = FollowJointTrajectory.Goal()
        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()
        point3 = JointTrajectoryPoint()
        point4 = JointTrajectoryPoint()
        point5 = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=2.0)
        duration3 = Duration(seconds=5.0)
        duration4 = Duration(seconds=7.0)
        duration5 = Duration(seconds=10.0)
        point1.time_from_start = duration1.to_msg()
        point2.time_from_start = duration2.to_msg()
        point3.time_from_start = duration3.to_msg()
        point4.time_from_start = duration4.to_msg()
        point5.time_from_start = duration5.to_msg()
        point1.positions = [0.0]
        point2.positions = [0.5]
        point3.positions = [0.0]
        point4.positions = [-0.5]
        point5.positions = [0.0]
        goal.trajectory.joint_names = ['joint_head_pan']
        goal.trajectory.points = [point1, point2, point3, point4, point5]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(5.0)

        # Send another goal to be executed 5 seconds in the past
        past_time = self.node.get_clock().now().to_msg()
        past_time.sec -= 5 # Subtract a five second delay
        goal.trajectory.header.stamp = past_time
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(10.0)

