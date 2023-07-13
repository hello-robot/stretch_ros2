#! /usr/bin/env python3

import importlib
import time
import pickle
import numpy as np
from pathlib import Path
from serial import SerialException
import stretch_body.hello_utils as hu
from hello_helpers.hello_misc import *
from .trajectory_components import get_trajectory_components

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .command_groups import HeadPanCommandGroup, HeadTiltCommandGroup, \
                           WristYawCommandGroup, GripperCommandGroup, \
                           ArmCommandGroup, LiftCommandGroup, \
                           MobileBaseCommandGroup

import hello_helpers.hello_misc as hm

class JointTrajectoryAction(Node):

    def __init__(self, node, action_server_rate_hz):
        super().__init__('joint_trajectory_action')
        self.node = node
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.action_server_rate = self.node.create_rate(action_server_rate_hz)
        self.server = ActionServer(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
                                   execute_callback=self.execute_cb,
                                   cancel_callback=self.cancel_cb,
                                   goal_callback=self.goal_cb,
                                   handle_accepted_callback=self.handle_accepted_cb,
                                   callback_group=ReentrantCallbackGroup())
        
        self.debug_dir = Path(hu.get_stretch_directory('goals'))
        if not self.debug_dir.exists():
            self.debug_dir.mkdir()
        
        # Position mode init
        self.head_pan_cg = HeadPanCommandGroup(node=self.node) \
            if 'head_pan' in self.node.robot.head.joints else None
        self.head_tilt_cg = HeadTiltCommandGroup(node=self.node) \
            if 'head_tilt' in self.node.robot.head.joints else None
        self.wrist_yaw_cg = WristYawCommandGroup(node=self.node) \
            if 'wrist_yaw' in self.node.robot.end_of_arm.joints else None
        self.gripper_cg = GripperCommandGroup(node=self.node) \
            if 'stretch_gripper' in self.node.robot.end_of_arm.joints else None
        self.arm_cg = ArmCommandGroup(node=self.node)
        self.lift_cg = LiftCommandGroup(node=self.node)
        self.mobile_base_cg = MobileBaseCommandGroup(node=self.node)
        self.command_groups = [self.arm_cg, self.lift_cg, self.mobile_base_cg, self.head_pan_cg,
                               self.head_tilt_cg, self.wrist_yaw_cg, self.gripper_cg]
        self.command_groups = [cg for cg in self.command_groups if cg is not None]

        for joint in self.node.robot.end_of_arm.joints:
            module_name = self.node.robot.end_of_arm.params['devices'][joint].get('ros_py_module_name')
            class_name = self.node.robot.end_of_arm.params['devices'][joint].get('ros_py_class_name')
            if module_name and class_name:
                endofarm_cg = getattr(importlib.import_module(module_name), class_name)(node=self.node)
                self.command_groups.append(endofarm_cg)
        
        # Trajectory mode init
        self.joints = get_trajectory_components(self.node.robot)
        self.node.robot._update_trajectory_dynamixel = lambda : None
        self.node.robot._update_trajectory_non_dynamixel = lambda : None

        self.timeout = 0.2 # seconds
        self.last_goal_time = self.get_clock().now().to_msg()

        self.latest_goal_id = 0

    def goal_cb(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        new_goal_time = self.get_clock().now().to_msg()
        time_duration = (new_goal_time.sec + new_goal_time.nanosec*pow(10,-9)) - (self.last_goal_time.sec + self.last_goal_time.nanosec*pow(10,-9))
        # If incoming commands received above 5 Hz, they are rejected
        if self._goal_handle is not None and self._goal_handle.is_active and (time_duration < self.timeout):
            return GoalResponse.REJECT # Reject goal if another goal is currently active

        self.last_goal_time = self.get_clock().now().to_msg()
        return GoalResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                # self._goal_handle.abort() \TODO(@hello-atharva): This is causing state transition issues.
            self._goal_handle = goal_handle

        goal_handle.execute()
    
    def execute_cb(self, goal_handle):
        # save goal to log directory
        goal = goal_handle.request
        goal_fpath = self.debug_dir / f'goal_{hu.create_time_string()}.pickle'
        with goal_fpath.open('wb') as s:
            pickle.dump(goal, s)

        # Register this goal's ID
        goal_id = self.latest_goal_id + 1
        self.latest_goal_id += 1
        
        with self.node.robot_stop_lock:
            # Escape stopped mode to execute trajectory
            self.node.stop_the_robot = False
        self.node.robot_mode_rwlock.acquire_read()
        if self.node.robot_mode not in ['position', 'trajectory', 'navigation']:
            self.node.robot_mode_rwlock.release_read()
            return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, "Cannot execute goals while in mode={0}".format(self.node.robot_mode))
        
        if self.node.robot_mode == 'position':
            # For now, ignore goal time and configuration tolerances.
            commanded_joint_names = goal.trajectory.joint_names
            self.node.get_logger().info(("{0} joint_traj action: New trajectory received with joint_names = "
                        "{1}").format(self.node.node_name, commanded_joint_names))

            ###################################################
            # Decide what to do based on the commanded joints.
            updates = [c.update(commanded_joint_names, self.invalid_joints_callback,
                    robot_mode=self.node.robot_mode)
                    for c in self.command_groups]
            if not all(updates):
                # The joint names violated at least one of the command
                # group's requirements. The command group should have
                # reported the error.
                self.node.robot_mode_rwlock.release_read()
                return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS, "joint names violated one of command group's requirements")

            num_valid_points = sum([c.get_num_valid_commands() for c in self.command_groups])
            if num_valid_points <= 0:
                err_str = ("Received a command without any valid joint names. "
                        "Received joint names = {0}").format(commanded_joint_names)
                self.node.robot_mode_rwlock.release_read()
                return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS, err_str)
            elif num_valid_points != len(commanded_joint_names):
                err_str = ("Received only {0} valid joints out of {1} total joints. Received joint names = "
                        "{2}").format(num_valid_points, len(commanded_joint_names), commanded_joint_names)
                self.node.robot_mode_rwlock.release_read()
                return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS, err_str)

            ###################################################
            # Try to reach each of the goals in sequence until
            # an error is detected or success is achieved.
            for pointi, point in enumerate(goal.trajectory.points):
                if not goal_handle.is_active:
                        self.get_logger().info('Goal aborted')
                        self.node.robot.stop_trajectory()
                        return FollowJointTrajectory.Result()

                self.node.get_logger().debug(("{0} joint_traj action: "
                                "target point #{1} = <{2}>").format(self.node.node_name, pointi, point))

                valid_goals = [c.set_goal(point, self.invalid_goal_callback, self.node.fail_out_of_range_goal)
                            for c in self.command_groups]
                if not all(valid_goals):
                    # At least one of the goals violated the requirements
                    # of a command group. Any violations should have been
                    # reported as errors by the command groups.
                    self.node.robot_mode_rwlock.release_read()
                    return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS, "joint names violated one of command group's requirements")

                robot_status = self.node.robot.get_status() # uses lock held by robot
                for c in self.command_groups:
                    c.init_execution(self.node.robot, robot_status)
                self.node.robot.push_command()

                goals_reached = [c.goal_reached() for c in self.command_groups]
                goal_start_time = self.node.get_clock().now()

                while not all(goals_reached):
                    # If goal is flagged as no longer active (ie. another goal was accepted),
                    # then stop executing
                    if not goal_handle.is_active:
                        self.get_logger().info('Goal aborted')
                        self.node.robot.stop_trajectory()
                        return FollowJointTrajectory.Result()
                    
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.node.robot.stop_trajectory()
                        self.get_logger().info('Goal canceled')
                        self.node.robot_mode_rwlock.release_read()
                        return FollowJointTrajectory.Result()
                    
                    if (self.node.get_clock().now() - goal_start_time) > self.node.default_goal_timeout_duration:
                        err_str = ("Time to execute the current goal point = <{0}> exceeded the "
                                "default_goal_timeout = {1}").format(point, self.node.default_goal_timeout_s)
                        self.node.robot_mode_rwlock.release_read()
                        return self.error_callback(goal_handle, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, err_str)
                    
                    # Check if a premption request has been received.
                    with self.node.robot_stop_lock:
                        if self.node.stop_the_robot or goal_id != self.latest_goal_id:
                            self.node.get_logger().info("{0} joint_traj action: PREEMPTION REQUESTED, but not stopping current motions to allow smooth interpolation between old and new commands.".format(self.node.node_name))
                            self.node.stop_the_robot = False
                            self.node.robot_mode_rwlock.release_read()
                            goal_handle.abort()
                            return FollowJointTrajectory.Result()

                    robot_status = self.node.robot.get_status()
                    named_errors = [c.update_execution(robot_status, contact_detected_callback=self.contact_detected_callback)
                                    for c in self.command_groups]
                    # It's not clear how this could ever happen. The
                    # groups in command_groups.py seem to return
                    # (self.name, self.error) or None, rather than True.
                    if any(ret == True for ret in named_errors):
                        self.node.robot_mode_rwlock.release_read()
                        # TODO: Check when this condtion is met
                        return self.error_callback(goal_handle, 100, "--")

                    self.feedback_callback(goal_handle, desired_point=point, named_errors=named_errors)
                    goals_reached = [c.goal_reached() for c in self.command_groups]
                    time.sleep(0.01)
                    # self.action_server_rate.sleep()

                self.node.get_logger().debug("{0} joint_traj action: Achieved target point.".format(self.node.node_name))

            self.node.robot_mode_rwlock.release_read()

            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                self.node.robot.stop_trajectory()
                return FollowJointTrajectory.Result()
            return self.success_callback(goal_handle, "Achieved all target points.")
        
        elif self.node.robot_mode == 'trajectory':
             # pre-process
            try:
                goal.trajectory = hm.merge_arm_joints(goal.trajectory)
                goal.trajectory = hm.preprocess_gripper_trajectory(goal.trajectory)
            except Exception as e:
                self.error_callback(goal_handle, 100, str(e))
            path_tolerance_dict = {tol.name: tol for tol in goal.path_tolerance}
            goal_tolerance_dict = {tol.name: tol for tol in goal.goal_tolerance}

            # load and start trajectory
            trajectories = [goal.trajectory, goal.multi_dof_trajectory]
            trajectories = [trajectory for trajectory in trajectories if len(trajectory.points) >= 2]
            if len(trajectories) == 0:
                return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS, 'no trajectory in goal contains enough waypoints')
            n_points = max([len(trajectory.points) for trajectory in trajectories])
            duration = max([Duration.from_msg(trajectory.points[-1].time_from_start) for trajectory in trajectories])
            self.node.get_logger().info(f"{self.node.node_name} joint_traj action: new traj with {n_points} points over {round(duration.nanoseconds/1e9, 2)} seconds")
            self.node.robot.stop_trajectory()
            for joint in self.joints:
                self.joints[joint].trajectory_manager.trajectory.clear()
            for trajectory in trajectories:
                for joint_index, joint_name in enumerate(trajectory.joint_names):
                    try:
                        self.joints[joint_name].add_waypoints(trajectory.points, joint_index)
                    except KeyError as e:
                        return self.error_callback(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, str(e))
            if not self.node.robot.follow_trajectory():
                self.node.robot.stop_trajectory()
                return self.error_callback(goal_handle, -100, 'hardware failed to start trajectory')

            # update trajectory and publish feedback
            ts = self.node.get_clock().now()
            while rclpy.ok() and self.node.get_clock().now() - ts <= duration:
                # Check if a premption request has been received.
                with self.node.robot_stop_lock:
                    if self.node.stop_the_robot or goal_id != self.latest_goal_id:
                        self.node.get_logger().info("{0} joint_traj action: PREEMPTION REQUESTED, but not stopping current motions to allow smooth interpolation between old and new commands.".format(self.node.node_name))
                        self.node.stop_the_robot = False
                        self.node.robot_mode_rwlock.release_read()
                        goal_handle.abort()
                        return FollowJointTrajectory.Result()

                self._update_trajectory_dynamixel()
                self._update_trajectory_non_dynamixel()
                self.feedback_callback(goal_handle, start_time=ts)
                # self.action_server_rate.sleep()

            time.sleep(0.1)
            self.node.robot_mode_rwlock.release_read()
            self._update_trajectory_dynamixel()
            self._update_trajectory_non_dynamixel()
            return self.success_callback(goal_handle, 'traj succeeded!')
        
        else:
            self.node.robot_mode_rwlock.release_read()
            return self.error_callback(goal_handle, -100, 'Joint Trajectory Server only accepts goals in position or trajectory mode')


    def contact_detected_callback(self, err_str):
        self.node.get_logger().warn(err_str)
    
    def invalid_joints_callback(self, err_str):
        self.node.get_logger().warn(err_str)
    
    def invalid_goal_callback(self, err_str):
        self.node.get_logger().warn(err_str)
    
    def error_callback(self, goal_handle, error_code, error_str):
        print("-------------------------------------------")
        print("Errored goal")
        self.node.get_logger().info("{0} joint_traj action: {1}".format(self.node.node_name, error_str))
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_str
        goal_handle.abort()
        return result

    def cancel_cb(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.node.get_logger().info("{0} joint_traj action: received cancel request".format(self.node.node_name))
        # self.node.robot.stop_trajectory()
        return CancelResponse.ACCEPT # Accepting cancel request

    def feedback_callback(self, goal_handle, desired_point=None, named_errors=None, start_time=None):
        goal = goal_handle.request
        feedback = FollowJointTrajectory.Feedback()
        commanded_joint_names = goal.trajectory.joint_names

        if self.node.robot_mode == 'position':
            clean_named_errors = []
            for named_error in named_errors:
                if type(named_error) == tuple:
                    clean_named_errors.append(named_error)
                elif type(named_error) == list:
                    clean_named_errors += named_error
            clean_named_errors_dict = dict((k, v) for k, v in clean_named_errors)

            actual_point = JointTrajectoryPoint()
            error_point = JointTrajectoryPoint()
            for i, commanded_joint_name in enumerate(commanded_joint_names):
                error_point.positions.append(clean_named_errors_dict[commanded_joint_name])
                actual_point.positions.append(desired_point.positions[i] - clean_named_errors_dict[commanded_joint_name])

            self.node.get_logger().debug("{0} joint_traj action: sending feedback".format(self.node.node_name))

            feedback.desired = desired_point
            feedback.actual = actual_point
            feedback.error = error_point

        elif self.node.robot_mode == 'trajectory':
            time_along_traj = (self.node.get_clock().now() - start_time).to_msg()

            feedback.desired.time_from_start = time_along_traj
            feedback.actual.time_from_start = time_along_traj
            feedback.error.time_from_start = time_along_traj
            for joint_name in feedback.joint_names:
                joint = self.joints[joint_name]
                actual_pos = joint.get_position()

            feedback.multi_dof_joint_names = goal.multi_dof_trajectory.joint_names
        
        feedback.joint_names = commanded_joint_names
        feedback.header.stamp = self.node.get_clock().now().to_msg()
        goal_handle.publish_feedback(feedback)

    def success_callback(self, goal_handle, success_str):
        print("-------------------------------------------")
        print("Finished goal")
        self.node.get_logger().info("{0} joint_traj action: {1}".format(self.node.node_name, success_str))
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        result.error_string = success_str
        goal_handle.succeed()
        return result

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
