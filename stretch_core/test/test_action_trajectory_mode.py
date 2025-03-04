import unittest

import time

import launch_pytest
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory,
    MultiDOFJointTrajectoryPoint,
)
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, Vector3, Quaternion
from rclpy.action.client import ClientGoalHandle

import pytest
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest


class FixtureData:
    """
    This fixture data class provides the instances, ros subscriptions and data necessary to carry out the tests.
    """
    def __init__(self) -> None:

        rclpy.init()

        self.node = rclpy.create_node(node_name="test_action")

        self.action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )
        self.action_client.wait_for_server(timeout_sec=1.0)

        self.joint_states_sub = self.node.create_subscription(
            JointState, "/stretch/joint_states", self.joint_states_callback, 1
        )

        self.joint_state: JointState | None = None

    def joint_states_callback(self, msg: JointState):
        self.joint_state = msg

    def destory(self):
        print("Cleaning up tests")

        self.node.destroy_node()
        self.joint_states_sub.destroy()
        rclpy.shutdown()


@launch_pytest.fixture
def launch_description():
    """
    Starts the StretchDriver in Trajectory mode
    """
    stretch_core_path = get_package_share_path("stretch_core")

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(stretch_core_path), "/launch/stretch_driver.launch.py"]
        ),
        launch_arguments={"mode": "trajectory"}.items(),
    )

    return LaunchDescription([stretch_driver_launch, ReadyToTest()])


@pytest.fixture(scope="module", autouse=True)
def fixture_data():
    """
    Sets up a test node, then cleans up subscribers after tests complete.
    """
    print("WARNING: these tests may move the real robot!")

    fixture_data = FixtureData()

    yield fixture_data

    # These will be called after tests finish:
    fixture_data.destory()


def _sleep_until_joint_states_available(
    fixture_data: FixtureData,
):
    """
    When we subscribe, we don't immediately get access to joint states, this waits.
    """
    while fixture_data.joint_state is None:
        rclpy.spin_once(fixture_data.node)


def _sleep_not_blocking_ros(
    fixture_data: FixtureData, sleep_amount: Duration
):
    """
    Using time.sleep does bad things.
    """
    start_time = fixture_data.node.get_clock().now()

    while (fixture_data.node.get_clock().now() - start_time) <= sleep_amount:
        rclpy.spin_once(fixture_data.node)


def _test_linear_motion(
    fixture_data: FixtureData, joint_names: list[str]
):
    """
    Tests a linear trajectory.

    False-positive note: if the robot is runstopped or the joint does not move at all,
    but it's already at the goal, this test will pass.
    """

    goal = FollowJointTrajectory.Goal()

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    trajectory.points = [
        JointTrajectoryPoint(
            positions=[0.2], time_from_start=Duration(seconds=0).to_msg()
        ),
        JointTrajectoryPoint(
            positions=[0.5], time_from_start=Duration(seconds=10).to_msg()
        ),
    ]

    goal.trajectory = trajectory

    fixture_data.action_client.wait_for_server()

    send_goal = fixture_data.action_client.send_goal_async(goal)

    rclpy.spin_until_future_complete(fixture_data.node, send_goal, timeout_sec=10.0)

    _sleep_not_blocking_ros(
        fixture_data, Duration.from_msg(trajectory.points[-1].time_from_start)
    )

    _sleep_not_blocking_ros(
        fixture_data, Duration(seconds=2)
    )  # sleep an extra 2 seconds for fun

    _sleep_until_joint_states_available(fixture_data)

    fixture_data.action_client.wait_for_server()

    goal_handle: ClientGoalHandle = send_goal.result()

    assert goal_handle.status == GoalStatus.STATUS_SUCCEEDED, "The goal status is not SUCCEEDED."

    for joint_name in trajectory.joint_names:

        assert fixture_data.joint_state
        state_joint_names: list[str] = fixture_data.joint_state.name
        index = state_joint_names.index(joint_name)
        position = fixture_data.joint_state.position[index]

        goal = trajectory.points[-1].positions[0]

        distance_from_goal = abs(position - goal)

        distance_from_goal_tolerance = 0.01  # in meters

        print(
            f"{joint_name} position is: {position}, goal is {goal}. Distance from goal is {distance_from_goal}"
        )

        assert (
            distance_from_goal < distance_from_goal_tolerance
        ), f"Goal tolerance not met. {joint_name} position is: {position}, goal is {goal}. Distance from goal is {distance_from_goal}"


@pytest.mark.launch(fixture=launch_description)
def test_lift(fixture_data):
    _test_linear_motion(fixture_data, ["joint_lift"])


@pytest.mark.launch(fixture=launch_description)
def test_base(fixture_data):
    goal = FollowJointTrajectory.Goal()

    # joint_name should be 'position' and not one generated by command i.e. 'translate/rotate_mobile_base'
    joint_name = "position"
    goal.multi_dof_trajectory.joint_names = [joint_name]

    goal.multi_dof_trajectory.points = [
        MultiDOFJointTrajectoryPoint(
            time_from_start=Duration(seconds=0).to_msg(),
            transforms=[
                Transform(translation=Vector3(x=0.0), rotation=Quaternion(w=1.0))
            ],
        ),
        MultiDOFJointTrajectoryPoint(
            time_from_start=Duration(seconds=5).to_msg(),
            transforms=[
                Transform(translation=Vector3(x=0.5), rotation=Quaternion(w=1.0))
            ],
        ),
    ]

    fixture_data.action_client.send_goal_async(goal)


@pytest.mark.launch(fixture=launch_description)
def test_wrist_extension(fixture_data):
    _test_linear_motion(fixture_data, ["wrist_extension"])


class TestActionTrajectoryMode(unittest.TestCase):
    """
    This is a unittest test. It should probably be converted into a pytest test to work better with colcon and the rest of the test suite.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        time.sleep(3)  # wait for launch file to load

    def setUp(self):
        self.node = rclpy.create_node("test_action")
        self.action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )
        self.action_client.wait_for_server(timeout_sec=1.0)

        self.joint_states_sub = self.node.create_subscription(
            JointState, "/stretch/joint_states", self.joint_states_callback, 1
        )

        self.joint_state = None

    def tearDown(self):
        self.node.destroy_node()
        self.joint_states_sub.destroy()
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def joint_states_callback(self, msg: JointState):
        self.joint_state = msg

    def test_action_server_exists(self, proc_output):
        self.assertTrue(self.action_client.server_is_ready())

    def test_feedback_correctly_formed(self, proc_output):
        goal = FollowJointTrajectory.Goal()

        # Send a blank trajectory goal
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        self.assertTrue(send_goal.done())  # Check if goal finished executing

        client_goal_handle = send_goal.result()
        self.assertTrue(
            client_goal_handle.accepted
        )  # Check if goal request was accepted

        result_promise = client_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_promise, timeout_sec=1.0)
        self.assertTrue(result_promise.done())  # Check if result received

        status = result_promise.result().status
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)  # Check status code

        result = result_promise.result().result
        self.assertEqual(
            result.error_code, FollowJointTrajectory.Result.INVALID_JOINTS
        )  # Check result error code

    def test_arm_trajectory_goal_replaced(self):
        # Test if a new goal replaces the current goal
        # x --------- x --------- x ....................... current trajectory
        # ....................... o --------- o --------- o new trajectory
        # x --------- x --------- o --------- o --------- o resultant trajectory
        print("########### Executing test_arm_trajectory_goal_replaced ###########")
        goal = FollowJointTrajectory.Goal()
        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()
        duration1 = Duration(seconds=0)
        duration2 = Duration(seconds=1)
        point1.time_from_start = duration1.to_msg()
        point2.time_from_start = duration2.to_msg()
        rclpy.spin_once(self.node)
        arm_index = self.joint_state.name.index("wrist_extension")
        arm_pos = self.joint_state.position[arm_index]
        print("arm pos is: {}".format(arm_pos))
        point1.positions = [arm_pos]
        point2.positions = [arm_pos + 0.04]
        goal.trajectory.joint_names = ["wrist_extension"]
        goal.trajectory.points = [point1, point2]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(0.5)
        self.assertTrue(send_goal.done())  # Check if goal finished executing

        for i in range(1, 30):
            rclpy.spin_once(self.node)
        arm_pos = self.joint_state.position[arm_index]
        print("arm pos is: {}".format(arm_pos))
        point1.positions = [arm_pos]
        point2.positions = [arm_pos + 0.04]
        goal.trajectory.joint_names = ["wrist_extension"]
        goal.trajectory.points = [point1, point2]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(2.0)

    def test_trajectory_goal_queued(self):
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
        goal.trajectory.joint_names = ["joint_head_pan"]
        goal.trajectory.points = [point1, point2, point3]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(2.0)

        # Send another goal to be executed 6 seconds from now
        delayed_time = self.node.get_clock().now().to_msg()
        delayed_time.sec += 6  # Add a four second delay
        goal.trajectory.header.stamp = delayed_time
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(6.0)

    def test_trajectory_goal_merged(self):
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
        duration1 = Duration(seconds=0)
        duration2 = Duration(seconds=2)
        duration3 = Duration(seconds=5)
        duration4 = Duration(seconds=7)
        duration5 = Duration(seconds=10)
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
        goal.trajectory.joint_names = ["joint_head_pan"]
        goal.trajectory.points = [point1, point2, point3, point4, point5]
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(5.0)

        # Send another goal to be executed 5 seconds in the past
        past_time = self.node.get_clock().now().to_msg()
        past_time.sec -= 5  # Subtract a five second delay
        goal.trajectory.header.stamp = past_time
        send_goal = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal, timeout_sec=1.0)
        time.sleep(10.0)
