#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger

import math
import time
import threading
import sys

import tf2_ros
import argparse as ap

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

class OpenDrawerNode(Node):

    def __init__(self):
        super().__init__('open_drawer')
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.logger = rclpy.logging.get_logger('open_drawer')
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf2_buffer, self)
        self.move_to_pose_complete = False
        self.unsuccessful_status = [-100, 100, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, FollowJointTrajectory.Result.INVALID_JOINTS, FollowJointTrajectory.Result.INVALID_GOAL, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP]
        self._get_result_future = None

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position
    
    def goal_response(self, future: rclpy.task.Future):
        if not future or not future.result():
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.move_to_pose_complete = True
            return

        self._get_result_future = goal_handle.get_result_async()

    def get_result(self, future: rclpy.task.Future):
        if not future or not future.result():
            return

        result = future.result().result
        error_code = result.error_code
        self.move_to_pose_complete = True

    def move_to_pose(self, pose, return_before_done=False, custom_contact_thresholds=False):
        self.move_to_pose_complete = False
        joint_names = [key for key in pose]
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0.0).to_msg()

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        trajectory_goal.trajectory.joint_names = joint_names
        if not custom_contact_thresholds: 
            joint_positions = [pose[key] for key in joint_names]
            point.positions = joint_positions
            trajectory_goal.trajectory.points = [point]
        else:
            pose_correct = all([len(pose[key])==2 for key in joint_names])
            if not pose_correct:
                self.logger.error("HelloNode.move_to_pose: Not sending trajectory due to improper pose. custom_contact_thresholds requires 2 values (pose_target, contact_threshold_effort) for each joint name, but pose = {0}".format(pose))
                return
            joint_positions = [pose[key][0] for key in joint_names]
            joint_efforts = [pose[key][1] for key in joint_names]
            point.positions = joint_positions
            point.effort = joint_efforts
            trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        self._send_goal_future = self.trajectory_client.send_goal_async(trajectory_goal)

        if not return_before_done:
            time_start = time.time()
            self._get_result_future = None

            while not self._get_result_future and (time.time() - time_start) < 10:
                self.goal_response(self._send_goal_future)

            if not self._get_result_future:
                return self._send_goal_future

            time_start = time.time()
            while not self.move_to_pose_complete and (time.time() - time_start) < 10:
                self.get_result(self._get_result_future)
        
        return self._send_goal_future

    def align_to_surface(self):
        self.logger.info('align_to_surface')
        trigger_request = Trigger.Request()
        trigger_result = self.trigger_align_with_nearest_cliff_service.call_async(trigger_request)
        self.logger.info('trigger_result = {0}'.format(trigger_result))

    def extend_hook_until_contact(self):
        self.logger.info('extend_hook_until_contact')
        max_extension_m = 0.5
        max_reach_m = 0.4
        extension_m = self.wrist_position + max_reach_m
        extension_m = min(extension_m, max_extension_m)
        extension_contact_effort = 42.0 #18.5 #effort_pct #42.0 #40.0 from funmap
        pose = {'wrist_extension': (extension_m, extension_contact_effort)}
        self.move_to_pose(pose, custom_contact_thresholds=True)

    def lower_hook_until_contact(self):
        self.logger.info('lower_hook_until_contact')
        max_drop_m = 0.15
        lift_m = self.lift_position - max_drop_m
        lift_contact_effort = 42.0 #32.5 #effort_pct #18.0 #20.0 #20.0 from funmap
        pose = {'joint_lift': (lift_m, lift_contact_effort)}
        self.move_to_pose(pose, custom_contact_thresholds=True)

        use_correction = True
        if use_correction:
            # raise due to drop down after contact detection
            time.sleep(0.2) # wait for new lift position
            lift_m = self.lift_position + 0.015
            pose = {'joint_lift': lift_m}
            self.move_to_pose(pose)
            time.sleep(0.2) # wait for new lift position

    def raise_hook_until_contact(self):
        self.logger.info('raise_hook_until_contact')
        max_raise_m = 0.15
        lift_m = self.lift_position + max_raise_m
        lift_contact_effort = 42.0 #effort_pct
        pose = {'joint_lift': (lift_m, lift_contact_effort)}
        self.move_to_pose(pose, custom_contact_thresholds=True)

        use_correction = True
        if use_correction:
            # raise due to drop down after contact detection
            time.sleep(0.5) # wait for new lift position
            lift_m = self.lift_position + 0.01 #0.015
            pose = {'joint_lift': lift_m}
            self.move_to_pose(pose)
            time.sleep(0.5) # wait for new lift position

    def backoff_from_surface(self):
        self.logger.info('backoff_from_surface')
        if self.wrist_position is not None:
            wrist_target_m = self.wrist_position - 0.005
            pose = {'wrist_extension': wrist_target_m}
            self.move_to_pose(pose)
            return True
        else:
            self.logger.error('backoff_from_surface: self.wrist_position is None!')
            return False

    def pull_open(self):
        self.logger.info('pull_open')
        if self.wrist_position is not None:
            max_extension_m = 0.5
            extension_m = self.wrist_position - 0.2
            extension_m = min(extension_m, max_extension_m)
            extension_m = max(0.01, extension_m)
            extension_contact_effort = 64.4 #effort_pct #100.0 #40.0 from funmap
            pose = {'wrist_extension': (extension_m, extension_contact_effort)}
            self.move_to_pose(pose, custom_contact_thresholds=True)
            return True
        else:
            self.logger.error('pull_open: self.wrist_position is None!')
            return False

    def push_closed(self):
        self.logger.info('push_closed')
        if self.wrist_position is not None:
            wrist_target_m = self.wrist_position + 0.22
            pose = {'wrist_extension': wrist_target_m}
            self.move_to_pose(pose)
            return True
        else:
            self.logger.error('pull_open: self.wrist_position is None!')
            return False

    def move_to_initial_configuration(self):
        initial_pose = {'wrist_extension': 0.01,
                        'joint_wrist_yaw': 1.570796327,
                        'gripper_aperture': 0.0}
        self.logger.info('Move to the initial configuration for drawer opening.')
        self.move_to_pose(initial_pose)

    def trigger_open_drawer_down_callback(self, request, response):
        return self.open_drawer('down')

    def trigger_open_drawer_up_callback(self, request, response):
        return self.open_drawer('up')


    def open_drawer(self, direction):
        self.move_to_initial_configuration()

        self.extend_hook_until_contact()
        success = self.backoff_from_surface()
        if not success:
            return Trigger.Response(
                success=False,
                message='Failed to backoff from the surface.'
            )

        if direction == 'down':
            self.lower_hook_until_contact()
        elif direction == 'up':
            self.raise_hook_until_contact()

        success = self.pull_open()
        if not success:
            return Trigger.Response(
                success=False,
                message='Failed to pull open the drawer.'
            )

        push_drawer_closed = False
        if push_drawer_closed:
            time.sleep(3.0)
            self.push_closed()

        return Trigger.Response(
            success=True,
            message='Completed opening the drawer!'
            )


    def main(self):
        self.callback_group = ReentrantCallbackGroup()

        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, qos_profile=1, callback_group=self.callback_group)

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory', callback_group=self.callback_group)
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
            sys.exit()

        self.trigger_open_drawer_service = self.create_service(Trigger, '/open_drawer/trigger_open_drawer_down',
                                                         self.trigger_open_drawer_down_callback, callback_group=self.callback_group)

        self.trigger_open_drawer_service = self.create_service(Trigger, '/open_drawer/trigger_open_drawer_up',
                                                         self.trigger_open_drawer_up_callback, callback_group=self.callback_group)

        self.trigger_align_with_nearest_cliff_service = self.create_client(Trigger, '/funmap/trigger_align_with_nearest_cliff', callback_group=self.callback_group)
        self.trigger_align_with_nearest_cliff_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_align_with_nearest_cliff.')

        self.trigger_reach_until_contact_service = self.create_client(Trigger, '/funmap/trigger_reach_until_contact', callback_group=self.callback_group)
        self.trigger_reach_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_reach_until_contact.')

        self.trigger_lower_until_contact_service = self.create_client(Trigger, '/funmap/trigger_lower_until_contact', callback_group=self.callback_group)
        self.trigger_lower_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_lower_until_contact.')


def main():
    rclpy.init()
    try:
        node = OpenDrawerNode()
        node.main()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(node=node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('open_drawer').info('interrupt received, so shutting down')

if __name__ == '__main__':
    main()
