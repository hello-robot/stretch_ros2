#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.duration import Duration
import rclpy.logging
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import os
import sys
import threading
import time

class JointNode(Node):
    def __init__(self):
        super().__init__('joint_node')
        self.rate = 10.0

        self.joint_states = None
        self.joint_states_lock = threading.Lock()

        self.logger = self.get_logger()

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states

    def send_wrist_yaw(self, yaw):
        pass

    def main(self):
        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')

        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
            sys.exit()

        self.logger.info("Test joints node is ready!")

def main():
    rclpy.init()

    try:
        node = JointNode()
        node.main()
        
        while rclpy.ok():
            rclpy.spin_once(node)

            node.send_wrist_yaw(0.5)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('joint_node').info('interrupt received, so shutting down')

if __name__ == "__main__":
    main()
