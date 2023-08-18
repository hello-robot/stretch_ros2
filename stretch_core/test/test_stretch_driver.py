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

from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from threading import Thread
from threading import Event

from hello_helpers.hello_misc import HelloNode


@pytest.fixture
def stretch_driver_proc():
    # Launch a process to test
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('stretch_core'), 'launch'),
            '/stretch_driver.launch.py'])
        )

# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(stretch_driver_proc):
    """Launch a process to run stretch driver."""
    return launch.LaunchDescription([
        stretch_driver_proc,
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])

@pytest.mark.launch(fixture=launch_description)
def test_core_topics_published():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_subscribers()
        
        battery_msgs_received_flag = node.battery_msg_event_object.wait(timeout=5.0)
        assert battery_msgs_received_flag, 'Did not receive battery msg!'
        
        mode_msgs_received_flag = node.mode_msg_event_object.wait(timeout=5.0)
        assert mode_msgs_received_flag, 'Did not receive mode msg!'

        joint_state_msgs_received_flag = node.joint_state_msg_event_object.wait(timeout=5.0)
        assert joint_state_msgs_received_flag, 'Did not receive joint state msg!'

        is_homed_msgs_received_flag = node.is_homed_msg_event_object.wait(timeout=5.0)
        assert is_homed_msgs_received_flag, 'Did not receive is homed msg!'

        is_runstopped_msgs_received_flag = node.is_runstopped_msg_event_object.wait(timeout=5.0)
        assert is_runstopped_msgs_received_flag, 'Did not receive is runstopped msg!'

        odom_msgs_received_flag = node.odom_msg_event_object.wait(timeout=5.0)
        assert odom_msgs_received_flag, 'Did not receive odom msg!'

        tool_msgs_received_flag = node.tool_msg_event_object.wait(timeout=5.0)
        assert tool_msgs_received_flag, 'Did not receive tool msg!'

        tf_msgs_received_flag = node.tf_msg_event_object.wait(timeout=5.0)
        assert tf_msgs_received_flag, 'Did not receive tf msg!'

    finally:
        rclpy.shutdown()

@pytest.mark.launch(fixture=launch_description)
def test_joint_states_rate():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_subscribers()

        joint_state_msgs_received_flag = node.joint_state_msg_event_object.wait(timeout=5.0)
        assert joint_state_msgs_received_flag, 'Did not receive aruco marker array msg!'
    
        ts = time.time()
        node.count_joint_state = 0
        while rclpy.ok() and time.time() - ts < 1.0:
            pass
        count_joint_state = node.count_joint_state
        
        assert count_joint_state >= 14, 'Joint states detection frequency is: {0}Hz'.format(count_joint_state)

    finally:
        rclpy.shutdown()

@pytest.mark.launch(fixture=launch_description)
def test_mode_sevices_callable():
    try:
        node = HelloNode.quick_create('test_node')

        navigation_mode_srv_call = node.switch_to_navigation_mode()
        assert navigation_mode_srv_call == True

        trajectory_mode_srv_call = node.switch_to_trajectory_mode()
        assert trajectory_mode_srv_call == True

        position_mode_srv_call = node.switch_to_position_mode()
        assert position_mode_srv_call == True

    finally:
        rclpy.shutdown()

@pytest.mark.launch(fixture=launch_description)
def test_robot_sevices_callable():
    try:
        node = HelloNode.quick_create('test_node')

        home_robot_srv_call = node.home_the_robot()
        assert home_robot_srv_call == True

        stow_robot_srv_call = node.stow_the_robot()
        assert stow_robot_srv_call == True

        stop_robot_srv_call = node.stop_the_robot()
        assert stop_robot_srv_call == True
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.battery_msg_event_object = Event()
        self.mode_msg_event_object = Event()
        self.joint_state_msg_event_object = Event()
        self.is_homed_msg_event_object = Event()
        self.is_runstopped_msg_event_object = Event()
        self.odom_msg_event_object = Event()
        self.tool_msg_event_object = Event()
        self.tf_msg_event_object = Event()
        self.count_joint_state = 0
    
    def start_subscribers(self):
        # Create subscribers
        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery',
            self.battery_subscriber_callback,
            10
        )
        
        self.mode_subscription = self.create_subscription(
            String,
            'mode',
            self.mode_subscriber_callback,
            10
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/stretch/joint_states',
            self.joint_state_subscriber_callback,
            10
        )

        self.is_homed_subscription = self.create_subscription(
            Bool,
            '/is_homed',
            self.is_homed_subscriber_callback,
            10
        )

        self.is_runstopped_subscription = self.create_subscription(
            Bool,
            '/is_runstopped',
            self.is_runstopped_subscriber_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_subscriber_callback,
            10
        )

        self.tool_subscription = self.create_subscription(
            String,
            '/tool',
            self.tool_subscriber_callback,
            10
        )

        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_subscriber_callback,
            10
        )
        
        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def battery_subscriber_callback(self, data):
        self.battery_msg_event_object.set()
    
    def mode_subscriber_callback(self, data):
        self.mode_msg_event_object.set()

    def joint_state_subscriber_callback(self, data):
        self.joint_state_msg_event_object.set()
        self.count_joint_state += 1

    def is_homed_subscriber_callback(self, data):
        self.is_homed_msg_event_object.set()

    def is_runstopped_subscriber_callback(self, data):
        self.is_runstopped_msg_event_object.set()

    def odom_subscriber_callback(self, data):
        self.odom_msg_event_object.set()

    def tool_subscriber_callback(self, data):
        self.tool_msg_event_object.set()

    def tf_subscriber_callback(self, data):
        self.tf_msg_event_object.set()
