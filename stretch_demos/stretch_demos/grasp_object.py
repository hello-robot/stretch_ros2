#!/usr/bin/env python3

from sensor_msgs.msg import JointState

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.logging

from std_srvs.srv import Trigger

import time
import threading
import numpy as np
import os

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


class GraspObjectNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.tool = None
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None
        
    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position
        self.left_finger_position, temp1, temp2 = hm.get_left_finger_state(joint_states)

    def lower_tool_until_contact(self):
        if (self.dryrun):
            return

        self.logger.info('lower_tool_until_contact')
        trigger_request = Trigger.Request() 
        trigger_result = self.trigger_lower_until_contact_service.call_async(trigger_request)
        self.logger.info('trigger_result = {0}'.format(trigger_result))

    def look_at_surface(self, scan_time_s=None):
        if self.dryrun:
            return

        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory, self.get_tool())
        manip = self.manipulation_view
        head_settle_time_s = 0.02 #1.0
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
        if (self.dryrun):
            return

        tolerance_distance_m = 0.005
        if forward_m > 0: 
            at_goal = self.move_base.forward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        else:
            at_goal = self.move_base.backward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)

    def trigger_grasp_object_callback(self, request, response):
        max_lift_m = 1.09
        min_extension_m = 0.01
        max_extension_m = 0.5

        self.logger.info('Stow the arm.')
        self.stow_the_robot()

        # 1. Scan surface and find grasp target
        self.look_at_surface(scan_time_s = 4.0)
        grasp_target = self.manipulation_view.get_grasp_target(self.tf2_buffer)
        if grasp_target is None:
            return Trigger.Response(
                success=False,
                message='Failed to find grasp target'
            )

        # 2. Move to pregrasp pose
        pregrasp_lift_m = self.manipulation_view.get_pregrasp_lift(grasp_target, self.tf2_buffer)
        if self.tool == "tool_stretch_dex_wrist":
            pregrasp_lift_m += 0.02
        if (self.lift_position is None):
            return Trigger.Response(
                success=False,
                message='lift position unavailable'
            )
        self.logger.info('Raise tool to pregrasp height.')
        lift_to_pregrasp_m = max(self.lift_position + pregrasp_lift_m, 0.1)
        lift_to_pregrasp_m = min(lift_to_pregrasp_m, max_lift_m)
        pose = {'joint_lift': lift_to_pregrasp_m}
        self.move_to_pose(pose)

        if self.tool == "tool_stretch_dex_wrist":
            self.logger.info('Rotate pitch/roll for grasping.')
            pose = {'joint_wrist_pitch': -0.3, 'joint_wrist_roll': 0.0}
            self.move_to_pose(pose)

        pregrasp_yaw = self.manipulation_view.get_pregrasp_yaw(grasp_target, self.tf2_buffer)
        self.logger.info('pregrasp_yaw = {0:.2f} rad'.format(pregrasp_yaw))
        self.logger.info('pregrasp_yaw = {0:.2f} deg'.format(pregrasp_yaw * (180.0/np.pi)))
        self.logger.info('Rotate the gripper for grasping.')
        pose = {'joint_wrist_yaw': pregrasp_yaw}
        self.move_to_pose(pose)

        self.logger.info('Open the gripper.')
        pose = {'gripper_aperture': 0.07}
        self.move_to_pose(pose)

        pregrasp_mobile_base_m, pregrasp_wrist_extension_m = self.manipulation_view.get_pregrasp_planar_translation(grasp_target, self.tf2_buffer)
        self.logger.info('pregrasp_mobile_base_m = {0:.3f} m'.format(pregrasp_mobile_base_m))
        self.logger.info('pregrasp_wrist_extension_m = {0:.3f} m'.format(pregrasp_wrist_extension_m))
        self.logger.info('Drive to pregrasp location.')
        self.drive(pregrasp_mobile_base_m)

        if pregrasp_wrist_extension_m > 0.0:
            extension_m = max(self.wrist_position + pregrasp_wrist_extension_m, min_extension_m)
            extension_m = min(extension_m, max_extension_m)
            self.logger.info('Extend tool above surface.')
            pose = {'wrist_extension': extension_m}
            self.move_to_pose(pose)
        else:
            self.logger.info('negative wrist extension for pregrasp, so not extending or retracting.')

        # 3. Grasp the object and lift it
        grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m = self.manipulation_view.get_grasp_from_pregrasp(grasp_target, self.tf2_buffer)
        self.logger.info('grasp_mobile_base_m = {0:3f} m, grasp_lift_m = {1:3f} m, grasp_wrist_extension_m = {2:3f} m'.format(grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m))
        self.logger.info('Move the grasp pose from the pregrasp pose.')

        lift_m = max(self.lift_position + grasp_lift_m, 0.1)
        lift_m = min(lift_m, max_lift_m)
        extension_m = max(self.wrist_position + grasp_wrist_extension_m, min_extension_m)
        extension_m = min(extension_m, max_extension_m)
        pose = {'translate_mobile_base': grasp_mobile_base_m,
                'joint_lift': lift_m,
                'wrist_extension': extension_m}
        self.move_to_pose(pose)

        self.logger.info('Attempt to close the gripper on the object.')
        gripper_aperture_m = grasp_target['width_m'] - 0.18
        pose = {'gripper_aperture': gripper_aperture_m}
        self.move_to_pose(pose)

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

        self.logger.info('Open the gripper a little to avoid overtorquing and overheating the gripper motor.')
        pose = {'gripper_aperture': gripper_aperture_m + 0.005}
        self.move_to_pose(pose)

        self.logger.info('Retract the tool.')
        pose = {'wrist_extension': 0.01}
        self.move_to_pose(pose)

        self.logger.info('Reorient the wrist.')
        pose = {'joint_wrist_yaw': 0.0}
        self.move_to_pose(pose)

        return Trigger.Response(
            success=True,
            message='Completed object grasp!'
        )
    
    def main(self):
        hm.HelloNode.main(self, 'grasp_object', 'grasp_object', wait_for_first_pointcloud=False)
        self.logger = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()

        self.debug_directory = self.get_parameter('debug_directory').value
        self.logger.info('Using the following directory for debugging files: {0}'.format(self.debug_directory))

        self.dryrun = self.get_parameter('dryrun').value

        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', callback=self.joint_states_callback, qos_profile=1, callback_group=self.callback_group)

        self.trigger_grasp_object_service = self.create_service(Trigger,
                                                                '/grasp_object/trigger_grasp_object',
                                                                callback=self.trigger_grasp_object_callback,
                                                                callback_group=self.callback_group)

        self.trigger_reach_until_contact_service = self.create_client(Trigger, '/funmap/trigger_reach_until_contact', callback_group=self.callback_group)
        self.logger.info("Waiting for /funmap/trigger_reach_until_contact' service")
        self.trigger_reach_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_reach_until_contact.')


        self.trigger_lower_until_contact_service = self.create_client(Trigger, '/funmap/trigger_lower_until_contact', callback_group=self.callback_group)
        self.logger.info("Waiting for /funmap/trigger_lower_until_contact' service")
        self.trigger_lower_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_lower_until_contact.')

        self.logger.info("Grasp object node is ready!")

def main():
    try:
        node = GraspObjectNode()
        node.main()
        
        node.new_thread.join()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('grasp_object').info('interrupt received, so shutting down')

if __name__ == '__main__':
    main()
