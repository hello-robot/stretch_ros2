#! /usr/bin/env python3

import time
import pickle
import numpy as np
from pathlib import Path
from serial import SerialException
import stretch_body.hello_utils as hu
from hello_helpers.hello_misc import *
from .trajectory_components import get_trajectory_components

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory


class JointTrajectoryAction(Node):

    def __init__(self, node, action_server_rate_hz):
        super().__init__('joint_trajectory_action')
        self.node = node
        self.action_server_rate = self.node.create_rate(action_server_rate_hz)
        self.server = ActionServer(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
                                   execute_callback=self.execute_cb,
                                   cancel_callback=self.cancel_cb)
        self.joints = get_trajectory_components(self.node.robot)
        self.node.robot._update_trajectory_dynamixel = lambda : None
        self.node.robot._update_trajectory_non_dynamixel = lambda : None
        self.debug_dir = Path(hu.get_stretch_directory('goals'))
        if not self.debug_dir.exists():
            self.debug_dir.mkdir()

    def execute_cb(self, goal_handle):
        if self.node.robot_mode != 'manipulation':
            self.node.get_logger().warn('Only manipulation mode support currently. Use /switch_to_manipulation_mode service.')

        # save goal to log directory
        goal = goal_handle.request
        goal_fpath = self.debug_dir / f'goal_{hu.create_time_string()}.pickle'
        with goal_fpath.open('wb') as s:
            pickle.dump(goal, s)

        # pre-process
        try:
            goal.trajectory = merge_arm_joints(goal.trajectory)
            goal.trajectory = preprocess_gripper_trajectory(goal.trajectory)
        except FollowJointTrajectoryException as e:
            self.error_callback(goal_handle, e.CODE, str(e))
        path_tolerance_dict = {tol.name: tol for tol in goal.path_tolerance}
        goal_tolerance_dict = {tol.name: tol for tol in goal.goal_tolerance}

        # load and start trajectory
        trajectories = [goal.trajectory, goal.multi_dof_trajectory]
        trajectories = [trajectory for trajectory in trajectories if len(trajectory.points) >= 2]
        if len(trajectories) == 0:
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS, 'no trajectory in goal contains enough waypoints')
        n_points = max([len(trajectory.points) for trajectory in trajectories])
        duration = max([Duration.from_msg(trajectory.points[-1].time_from_start) for trajectory in trajectories])
        self.node.get_logger().info(f"{self.node.node_name} joint_traj action: new traj with {n_points} points over {to_sec(duration.to_msg())} seconds")
        for joint in self.joints:
            self.joints[joint].trajectory_manager.trajectory.clear()
        for trajectory in trajectories:
            for joint_index, joint_name in enumerate(trajectory.joint_names):
                try:
                    self.joints[joint_name].add_waypoints(trajectory.points, joint_index)
                except KeyError as e:
                    return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, str(e))
        self.node.robot.stop_trajectory()
        if not self.node.robot.follow_trajectory():
            self.node.robot.stop_trajectory()
            return self.error_callback(goal_handle, -100, 'hardware failed to start trajectory')

        # update trajectory and publish feedback
        ts = self.node.get_clock().now()
        while rclpy.ok() and self.node.get_clock().now() - ts <= duration:
            self._update_trajectory_dynamixel()
            self._update_trajectory_non_dynamixel()
            self.feedback_callback(goal_handle, ts)
            self.action_server_rate.sleep()

        time.sleep(0.1)
        self._update_trajectory_dynamixel()
        self._update_trajectory_non_dynamixel()
        self.node.robot.stop_trajectory()
        return self.success_callback(goal_handle, 'traj succeeded!')

    def cancel_cb(self, goal_handle):
        self.node.get_logger().info("{0} joint_traj action: received cancel request".format(self.node.node_name))
        self.node.robot.stop_trajectory()
        return 2 # Accepting cancel request

    def error_callback(self, goal_handle, error_code, error_str):
        self.node.get_logger().info("{0} joint_traj action: {1}".format(self.node.node_name, error_str))
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_str
        goal_handle.abort()
        return result

    def feedback_callback(self, goal_handle, start_time):
        goal = goal_handle.request
        time_along_traj = (self.node.get_clock().now() - start_time).to_msg()

        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.node.get_clock().now().to_msg()
        feedback.joint_names = goal.trajectory.joint_names
        feedback.desired.time_from_start = time_along_traj
        feedback.actual.time_from_start = time_along_traj
        feedback.error.time_from_start = time_along_traj
        for joint_name in feedback.joint_names:
            joint = self.joints[joint_name]
            actual_pos = joint.get_position()

        feedback.multi_dof_joint_names = goal.multi_dof_trajectory.joint_names
        goal_handle.publish_feedback(feedback)

    def success_callback(self, goal_handle, success_str):
        self.node.get_logger().info("{0} joint_traj action: {1}".format(self.node.node_name, success_str))
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        result.error_string = success_str
        goal_handle.succeed()
        return result

    def _check_trajectories_valid(self, goal_handle):
        """TODO: re-evaluating after determining chosen dynamic limits
        """
        valid, error_str = self.joints['joint_head_pan'].trajectory_manager.trajectory.is_valid(4.0, 8.0)
        if not valid and error_str != "must have atleast two waypoints":
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"joint_head_pan: {error_str}")

        valid, error_str = self.joints['joint_head_tilt'].trajectory_manager.trajectory.is_valid(4.0, 8.0)
        if not valid and error_str != "must have atleast two waypoints":
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"joint_head_tilt: {error_str}")

        valid, error_str = self.joints['joint_wrist_yaw'].trajectory_manager.trajectory.is_valid(4.0, 8.0)
        if not valid and error_str != "must have atleast two waypoints":
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"joint_wrist_yaw: {error_str}")

        valid, error_str = self.joints['stretch_gripper'].trajectory_manager.trajectory.is_valid(50.0, 100.0)
        if not valid and error_str != "must have atleast two waypoints":
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"stretch_gripper: {error_str}")

        valid, error_str = self.joints['wrist_extension'].trajectory_manager.trajectory.is_valid(0.1, 0.15)
        if not valid and error_str != "must have atleast two waypoints":
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"wrist_extension: {error_str}")

        valid, error_str = self.joints['joint_lift'].trajectory_manager.trajectory.is_valid(0.1, 0.15)
        if not valid and error_str != "must have atleast two waypoints":
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"joint_lift: {error_str}")

        valid, error_str = self.joints['position'].trajectory_manager.trajectory.is_valid(25.0, 10.0,
            self.node.robot.base.translate_to_motor_rad, self.node.robot.base.rotate_to_motor_rad)
        if not valid and error_str != "must have atleast two waypoints":
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"position: {error_str}")

        return None

    def _update_trajectory_dynamixel(self):
        try:
            self.node.robot.end_of_arm.update_trajectory()
            self.node.robot.head.update_trajectory()
        except SerialException:
            self.get_logger().warn(f'{self.node.node_name} joint_traj action: Serial Exception on updating dynamixel waypoint trajectories')

    def _update_trajectory_non_dynamixel(self):
        self.node.robot.arm.motor.pull_status()
        self.node.robot.arm.update_trajectory()
        self.node.robot.lift.motor.pull_status()
        self.node.robot.lift.update_trajectory()
        self.node.robot.base.left_wheel.pull_status()
        self.node.robot.base.right_wheel.pull_status()
        self.node.robot.base.update_trajectory()
