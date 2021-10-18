#! /usr/bin/env python3

import pickle
from pathlib import Path
import stretch_body.hello_utils as hu

from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory


class JointTrajectoryAction(Node):

    def __init__(self, node, action_server_rate_hz):
        super().__init__('joint_trajectory_action')
        self.node = node
        self.action_server_rate = self.node.create_rate(action_server_rate_hz)
        self.server = ActionServer(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
                                   self.execute_cb)
        self.debug_dir = Path(hu.get_stretch_directory('goals'))
        if not self.debug_dir.exists():
            self.debug_dir.mkdir()

    def execute_cb(self, goal_handle):
        if self.node.robot_mode != 'manipulation':
            self.node.get_logger().warn('Only manipulation mode support currently. Use /switch_to_manipulation_mode service.')

        # save goal to log directory
        goal_fpath = self.debug_dir / f'goal_{hu.create_time_string()}.pickle'
        with goal_fpath.open('wb') as s:
            pickle.dump(goal_handle.request, s)

        return self.success_callback(goal_handle, 'nothing happened')

    def success_callback(self, goal_handle, success_str):
        self.node.get_logger().info("{0} joint_traj action: {1}".format(self.node.node_name, success_str))
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        result.error_string = success_str
        goal_handle.succeed()
        return result
