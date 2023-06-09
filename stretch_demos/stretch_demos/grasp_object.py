#!/usr/bin/env python

from __future__ import print_function

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

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger

import math
import time
import threading
import sys
import tf2_ros
import argparse as ap        
import numpy as np
import os

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


class GraspObjectNode(Node):

    def __init__(self):
        super().__init__('grasp_node')
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf2_buffer, self)
        self.logger = self.get_logger()
        
    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position
        self.left_finger_position, temp1, temp2 = hm.get_left_finger_state(joint_states)

    def point_cloud_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def lower_tool_until_contact(self):
        self.logger.info('lower_tool_until_contact')
        trigger_request = Trigger.Request() 
        trigger_result = self.trigger_lower_until_contact_service.call_async(trigger_request)
        self.logger.info('trigger_result = {0}'.format(trigger_result))
        
    def move_to_initial_configuration(self):
        initial_pose = {'wrist_extension': 0.01,
                        'joint_wrist_yaw': 0.0,
                        'gripper_aperture': 0.125}

        self.logger.info('Move to the initial configuration for drawer opening.')
        self.move_to_pose(initial_pose)

    def move_to_pose(self, pose, _async=False, custom_contact_thresholds=False):
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
                self.log.error("HelloNode.move_to_pose: Not sending trajectory due to improper pose. custom_contact_thresholds requires 2 values (pose_target, contact_threshold_effort) for each joint name, but pose = {0}".format(pose))
                return
            joint_positions = [pose[key][0] for key in joint_names]
            joint_efforts = [pose[key][1] for key in joint_names]
            point.positions = joint_positions
            point.effort = joint_efforts
            trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        future = self.trajectory_client.send_goal_async(trajectory_goal)
        
        if _async:
            rclpy.spin_until_future_complete(self.trajectory_client, future)

    def look_at_surface(self, scan_time_s=None):
        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory)
        manip = self.manipulation_view
        head_settle_time_s = 1.0
        manip.move_head(self.move_to_pose)
        time.sleep(head_settle_time_s)
        if scan_time_s is None:
            manip.update(self.point_cloud, self.tf2_buffer)
        else:
            start_time_s = time.time()
            while ((time.time() - start_time_s) < scan_time_s): 
                manip.update(self.point_cloud, self.tf2_buffer)
        if self.debug_directory is not None:
            dirname = self.debug_directory + 'grasp_object/'
            # If the directory does not already exist, create it.
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            filename = 'look_at_surface_' + hm.create_time_string()
            manip.save_scan(dirname + filename)
        else:
            self.logger.info('GraspObjectNode: No debug directory provided, so debugging data will not be saved.')

    def drive(self, forward_m):
        tolerance_distance_m = 0.005
        if forward_m > 0: 
            at_goal = self.move_base.forward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        else:
            at_goal = self.move_base.backward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        
    def trigger_grasp_object_callback(self, request, response):
        actually_move = True
        max_lift_m = 1.09
        min_extension_m = 0.01
        max_extension_m = 0.5

        if actually_move:
            self.logger.info('Retract the tool.')
            pose = {'wrist_extension': 0.01}
            self.move_to_pose(pose)
            time.sleep(5)

            self.logger.info('Reorient the wrist.')
            pose = {'joint_wrist_yaw': 0.0}
            self.move_to_pose(pose)
            time.sleep(5)
            
        self.look_at_surface(scan_time_s = 3.0)
        
        grasp_target = self.manipulation_view.get_grasp_target(self.tf2_buffer)

        self.logger.info(f"Got grasp target as {grasp_target}")
        
        if grasp_target is not None: 
            pregrasp_lift_m = self.manipulation_view.get_pregrasp_lift(grasp_target, self.tf2_buffer)

            if (self.lift_position is None):
                return Trigger.Response(
                    success=False,
                    message='lift position unavailable'
                )

            if actually_move:
                self.logger.info('Raise tool to pregrasp height.')
                lift_to_pregrasp_m = max(self.lift_position + pregrasp_lift_m, 0.1)
                lift_to_pregrasp_m = min(lift_to_pregrasp_m, max_lift_m)
                pose = {'joint_lift': lift_to_pregrasp_m}
                self.move_to_pose(pose)
                time.sleep(5)

            pregrasp_yaw = self.manipulation_view.get_pregrasp_yaw(grasp_target, self.tf2_buffer)
            self.logger.info('pregrasp_yaw = {0:.2f} rad'.format(pregrasp_yaw))
            self.logger.info('pregrasp_yaw = {0:.2f} deg'.format(pregrasp_yaw * (180.0/np.pi)))

            if actually_move:
                self.logger.info('Rotate the gripper for grasping.')
                pose = {'joint_wrist_yaw': pregrasp_yaw}
                self.move_to_pose(pose)
                time.sleep(5)
                
                self.logger.info('Open the gripper.')
                pose = {'gripper_aperture': 0.125}
                self.move_to_pose(pose)
                time.sleep(5)

            pregrasp_mobile_base_m, pregrasp_wrist_extension_m = self.manipulation_view.get_pregrasp_planar_translation(grasp_target, self.tf2_buffer)
            
            self.logger.info('pregrasp_mobile_base_m = {0:.3f} m'.format(pregrasp_mobile_base_m))
            self.logger.info('pregrasp_wrist_extension_m = {0:.3f} m'.format(pregrasp_wrist_extension_m))

            if actually_move:
                self.logger.info('Drive to pregrasp location.')
                self.drive(pregrasp_mobile_base_m)
                time.sleep(5)

                if pregrasp_wrist_extension_m > 0.0:
                    extension_m = max(self.wrist_position + pregrasp_wrist_extension_m, min_extension_m)
                    extension_m = min(extension_m, max_extension_m)
                    self.logger.info('Extend tool above surface.')
                    pose = {'wrist_extension': extension_m} 
                    self.move_to_pose(pose)
                    time.sleep(5)
                else:
                    self.logger.info('negative wrist extension for pregrasp, so not extending or retracting.')

            grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m = self.manipulation_view.get_grasp_from_pregrasp(grasp_target, self.tf2_buffer)
            self.logger.info('grasp_mobile_base_m = {0:3f} m, grasp_lift_m = {1:3f} m, grasp_wrist_extension_m = {2:3f} m'.format(grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m))

            if actually_move: 
                self.logger.info('Move the grasp pose from the pregrasp pose.')

                lift_m = max(self.lift_position + grasp_lift_m, 0.1)
                lift_m = min(lift_m, max_lift_m)
                
                extension_m = max(self.wrist_position + grasp_wrist_extension_m, min_extension_m)
                extension_m = min(extension_m, max_extension_m)
                
                pose = {'translate_mobile_base': grasp_mobile_base_m,
                        'joint_lift': lift_m,  
                        'wrist_extension': extension_m}
                self.move_to_pose(pose)
                time.sleep(5)

                self.logger.info('Attempt to close the gripper on the object.')
                gripper_aperture_m = grasp_target['width_m'] - 0.18
                pose = {'gripper_aperture': gripper_aperture_m}
                self.move_to_pose(pose)
                time.sleep(5)
                
                # Lifting appears to happen before the gripper has
                # finished unless there is this sleep. Need to look
                # into this issue.
                time.sleep(3.0)

                self.logger.info('Attempt to lift the object.')
                object_lift_height_m = 0.1

                lift_m = max(self.lift_position + object_lift_height_m, 0.2)
                lift_m = min(lift_m, max_lift_m)
                
                pose = {'joint_lift': lift_m}
                self.move_to_pose(pose)
                time.sleep(5)

                self.logger.info('Open the gripper a little to avoid overtorquing and overheating the gripper motor.')
                pose = {'gripper_aperture': gripper_aperture_m + 0.005}
                self.move_to_pose(pose)
                time.sleep(5)


            if actually_move:
                self.logger.info('Retract the tool.')
                pose = {'wrist_extension': 0.01}
                self.move_to_pose(pose)
                time.sleep(5)

                self.logger.info('Reorient the wrist.')
                pose = {'joint_wrist_yaw': 0.0}
                self.move_to_pose(pose)
                time.sleep(5)

        return Trigger.Response(
            success=True,
            message='Completed object grasp!'
            )

    
    def main(self):
        # hm.HelloNode.main(self, 'grasp_object', 'grasp_object', wait_for_first_pointcloud=False)

        self.debug_directory = '/home/hello-robot/demos/' #self.get_parameter_or('~debug_directory').value
        self.logger.info('Using the following directory for debugging files: {0}'.format(self.debug_directory))


        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)

        self.point_cloud_subscriber = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 1)

        self.trigger_grasp_object_service = self.create_service(Trigger,
                                                                '/grasp_object/trigger_grasp_object',
                                                                self.trigger_grasp_object_callback)

        self.trigger_reach_until_contact_service = self.create_client(Trigger, '/funmap/trigger_reach_until_contact')
        self.logger.info("Waiting for /funmap/trigger_reach_until_contact' service")
        self.trigger_reach_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_reach_until_contact.')


        self.trigger_lower_until_contact_service = self.create_client(Trigger, '/funmap/trigger_lower_until_contact')
        self.logger.info("Waiting for /funmap/trigger_lower_until_contact' service")
        self.trigger_lower_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_lower_until_contact.')

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
            sys.exit()

        self.logger.info("Grasp object node is ready!")

def main():
    rclpy.init()
    try:
        node = GraspObjectNode()
        node.main()
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('grasp_object').info('interrupt received, so shutting down')
        
if __name__ == '__main__':
    main()
