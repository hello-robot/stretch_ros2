#! /usr/bin/env python3

import time
import copy
import pickle
from pathlib import Path
from hello_helpers.hello_misc import *
from hello_helpers.simple_command_group import SimpleCommandGroup
from rclpy.action.server import ServerGoalHandle

from control_msgs.action import FollowJointTrajectory

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectory, JointTrajectory

import hello_helpers.hello_misc as hm

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from stretch_mujoco_driver.stretch_mujoco_driver import StretchMujocoDriver

class JointTrajectoryAction:

    def __init__(self, node: "StretchMujocoDriver", action_server_rate_hz:int):
        self.node = node
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.server = ActionServer(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
                                   execute_callback=self.execute_cb,
                                   cancel_callback=self.cancel_cb,
                                   goal_callback=self.goal_cb,
                                   handle_accepted_callback=self.handle_accepted_cb,
                                   callback_group=node.main_group)
        
    def goal_cb(self, goal_request: FollowJointTrajectory.Goal): raise NotImplementedError()
    def handle_accepted_cb(self, goal_handle:ServerGoalHandle):raise NotImplementedError()
    def execute_cb(self, goal_handle:ServerGoalHandle): raise NotImplementedError()
    def cancel_cb(self, goal_handle:ServerGoalHandle):raise NotImplementedError()