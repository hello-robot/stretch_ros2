#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.logging

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import time
import threading

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

class OpenDrawerNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.wrist_position = None
        self.lift_position = None

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position

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
        hm.HelloNode.main(self, 'open_drawer', 'open_drawer', wait_for_first_pointcloud=False)
        
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, qos_profile=1, callback_group=self.callback_group)

        self.trigger_open_drawer_service = self.create_service(Trigger, '/open_drawer/trigger_open_drawer_down',
                                                         self.trigger_open_drawer_down_callback, callback_group=self.callback_group)

        self.trigger_open_drawer_service = self.create_service(Trigger, '/open_drawer/trigger_open_drawer_up',
                                                         self.trigger_open_drawer_up_callback, callback_group=self.callback_group)

        self.trigger_reach_until_contact_service = self.create_client(Trigger, '/funmap/trigger_reach_until_contact', callback_group=self.callback_group)
        self.trigger_reach_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_reach_until_contact.')

        self.trigger_lower_until_contact_service = self.create_client(Trigger, '/funmap/trigger_lower_until_contact', callback_group=self.callback_group)
        self.trigger_lower_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_lower_until_contact.')


def main():
    try:
        node = OpenDrawerNode()
        node.main()

        node.new_thread.join()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('open_drawer').info('interrupt received, so shutting down')

if __name__ == '__main__':
    main()
