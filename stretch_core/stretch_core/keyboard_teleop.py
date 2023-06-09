#!/usr/bin/env python3

import argparse as ap
from functools import partial
import math
import sys

from .keyboard import KBHit

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform

import hello_helpers.hello_misc as hm


class GetKeyboardCommands:

    def __init__(self, node, mapping_on=False, hello_world_on=False, open_drawer_on=False, clean_surface_on=False, grasp_object_on=False, deliver_object_on=False):
        self.kb = KBHit()
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on
        self.node = node
        self.step_size = 'medium'
        self.rad_per_deg = math.pi/180.0
        self.small_deg = 3.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.005  #0.02
        self.medium_deg = 6.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.04
        self.big_deg = 12.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.06
        self.mode = 'position' #'trajectory' #'navigation'

    def get_deltas(self):
        if self.step_size == 'small':
            deltas = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            deltas = {'rad': self.medium_rad, 'translate': self.medium_translate} 
        if self.step_size == 'big':
            deltas = {'rad': self.big_rad, 'translate': self.big_translate} 
        return deltas

    def print_commands(self):
        print('---------- KEYBOARD TELEOP MENU -----------')
        print('                                           ')
        print('              i HEAD UP                    ')
        print(' j HEAD LEFT            l HEAD RIGHT       ')
        print('              , HEAD DOWN                  ')
        print('                                           ')
        print('                                           ')
        print(' 7 BASE ROTATE LEFT     9 BASE ROTATE RIGHT')
        print(' home                   page-up            ')
        print('                                           ')
        print('                                           ')
        print('              8 LIFT UP                    ')
        print('              up-arrow                     ')
        print(' 4 BASE FORWARD         6 BASE BACK        ')
        print(' left-arrow             right-arrow        ')
        print('              2 LIFT DOWN                  ')
        print('              down-arrow                   ')
        print('                                           ')
        print('                                           ')
        print('              w ARM OUT                    ')
        print(' a WRIST FORWARD        d WRIST BACK       ')
        print('              x ARM IN                     ')
        print('                                           ')
        print('                                           ')
        print('              5 GRIPPER CLOSE              ')
        print('              0 GRIPPER OPEN               ')
        print('                                           ')
        print('  step size:  b BIG, m MEDIUM, s SMALL     ')
        print('                                           ')
        print('              q QUIT                       ')
        print('                                           ')
        print('-------------------------------------------')

    def get_command(self, node):
        command = None
        c = None

        if self.kb.kbhit(): # Returns True if any key pressed
            c = self.kb.getch()

        ####################################################
        ## COMMANDS TO TRIGGER STRETCH DEMOS
        ####################################################
        # Trigger clean surface demo
        if ((c == '/') or (c == '?')) and self.clean_surface_on:
            trigger_request = Trigger.Request() 
            future = node.trigger_clean_surface_service.call_async(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(future))
            # future.add_done_callback(partial(self.done_callback))

        # Trigger grasp object demo
        if ((c == '\\') or (c == '\"')) and self.grasp_object_on:
            trigger_request = Trigger.Request() 
            future = node.trigger_grasp_object_service.call_async(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(future))
            # future.add_done_callback(partial(self.done_callback))

        ####################################################
        ## BASIC KEYBOARD TELEOPERATION COMMANDS
        ####################################################
        
        # 8 or up arrow
        if c == '8' or c == '\x1b[A':
            command = {'joint': 'joint_lift', 'delta': self.get_deltas()['translate']}
        # 2 or down arrow
        if c == '2' or c == '\x1b[B':
            command = {'joint': 'joint_lift', 'delta': -self.get_deltas()['translate']}
        if self.mode == 'trajectory':
            # 4 or left arrow
            if c == '4' or c == '\x1b[D':
                command = {'joint': 'joint_mobile_base_translation', 'delta': self.get_deltas()['translate']}
            # 6 or right arrow
            if c == '6' or c == '\x1b[C':
                command = {'joint': 'joint_mobile_base_translation', 'delta': -self.get_deltas()['translate']}
        elif self.mode == 'position':
            # 4 or left arrow
            if c == '4' or c == '\x1b[D':
                command = {'joint': 'translate_mobile_base', 'inc': self.get_deltas()['translate']}
            # 6 or right arrow
            if c == '6' or c == '\x1b[C':
                command = {'joint': 'translate_mobile_base', 'inc': -self.get_deltas()['translate']}
            # 1 or end key 
            if c == '7' or c == '\x1b[H':
                command = {'joint': 'rotate_mobile_base', 'inc': self.get_deltas()['rad']}
            # 3 or pg down 5~
            if c == '9' or c == '\x1b[5':
                command = {'joint': 'rotate_mobile_base', 'inc': -self.get_deltas()['rad']}
        elif self.mode == 'navigation':
            node.get_logger().info('ERROR: Navigation mode is not currently supported.')

        if c == 'w' or c == 'W':
            command = {'joint': 'wrist_extension', 'delta': self.get_deltas()['translate']}
        if c == 'x' or c == 'X':
            command = {'joint': 'wrist_extension', 'delta': -self.get_deltas()['translate']}
        if c == 'd' or c == 'D':
            command = {'joint': 'joint_wrist_yaw', 'delta': -self.get_deltas()['rad']}
        if c == 'a' or c == 'A':
            command = {'joint': 'joint_wrist_yaw', 'delta': self.get_deltas()['rad']}
        if c == '5' or c == '\x1b[E' or c == 'g' or c == 'G':
            # grasp
            command = {'joint': 'joint_gripper_finger_left', 'delta': -self.get_deltas()['rad']}
        if c == '0' or c == '\x1b[2' or c == 'r' or c == 'R':
            # release
            command = {'joint': 'joint_gripper_finger_left', 'delta': self.get_deltas()['rad']}
        if c == 'i' or c == 'I':
            command = {'joint': 'joint_head_tilt', 'delta': (2.0 * self.get_deltas()['rad'])}
        if c == ',' or c == '<':
            command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if c == 'j' or c == 'J':
            command = {'joint': 'joint_head_pan', 'delta': (2.0 * self.get_deltas()['rad'])}
        if c == 'l' or c == 'L':
            command = {'joint': 'joint_head_pan', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if c == 'b' or c == 'B':
            node.get_logger().info('process_keyboard.py: changing to BIG step size')
            self.step_size = 'big'
        if c == 'm' or c == 'M':
            node.get_logger().info('process_keyboard.py: changing to MEDIUM step size')
            self.step_size = 'medium'
        if c == 's' or c == 'S':
            node.get_logger().info('process_keyboard.py: changing to SMALL step size')
            self.step_size = 'small'
        if c == 'q' or c == 'Q':
            node.get_logger().info('keyboard_teleop exiting...')
            node.get_logger().info('Received quit character (q), so exiting')

        ####################################################

        return command


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.mapping_on = self.get_parameter_or('mapping_on',
                                                Parameter('mapping', Parameter.Type.BOOL, False)).value

        self.hello_world_on = self.get_parameter_or('hello_world_on',
                                                Parameter('hello_world', Parameter.Type.BOOL, False)).value
        self.open_drawer_on = self.get_parameter_or('open_drawer_on',
                                                Parameter('open_drawer', Parameter.Type.BOOL, False)).value
        self.clean_surface_on = self.get_parameter_or('clean_surface_on',
                                                Parameter('clean_surface', Parameter.Type.BOOL, False)).value
        self.grasp_object_on = self.get_parameter_or('grasp_object_on',
                                                Parameter('grasp_object', Parameter.Type.BOOL, True)).value
        self.deliver_object_on = self.get_parameter_or('deliver_object_on',
                                                Parameter('deliver_object', Parameter.Type.BOOL, False)).value

        self.keys = GetKeyboardCommands(self,
                                        self.mapping_on,
                                        self.hello_world_on,
                                        self.open_drawer_on,
                                        self.clean_surface_on,
                                        self.grasp_object_on,
                                        self.deliver_object_on)

        self.joint_state = JointState()
        self.robot_mode = String()

        if self.clean_surface_on:
            self.get_logger().info("Clean surface demo enabled.")
        
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def mode_callback(self, mode):
        self.robot_mode = mode.data

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            if self.robot_mode == 'position':
                point = JointTrajectoryPoint()
                duration = Duration(seconds=0.0)
                point.time_from_start = duration.to_msg()
                trajectory_goal = FollowJointTrajectory.Goal()
                # trajectory_goal.goal_time_tolerance = rclpy.time.Time()
                joint_name = command['joint']
                trajectory_goal.trajectory.joint_names = [joint_name]

                # for base joints
                if 'inc' in command:
                    inc = command['inc']
                    new_value = inc

                # for non-base joints
                elif 'delta' in command:
                    joint_index = joint_state.name.index(joint_name)
                    joint_value = joint_state.position[joint_index]
                    delta = command['delta']
                    new_value = joint_value + delta
                
                point.positions = [new_value]
                trajectory_goal.trajectory.points = [point]
                trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                self.trajectory_client.send_goal_async(trajectory_goal)

            elif self.robot_mode == 'trajectory':
                trajectory_goal = FollowJointTrajectory.Goal()
                # trajectory_goal.goal_time_tolerance = rclpy.time.Time()
                joint_name = command['joint']
                duration1 = Duration(seconds=0.0)
                duration2 = Duration(seconds=1.0)

                # for base joints
                if 'inc' in command:
                    inc = command['inc']
                    point1 = MultiDOFJointTrajectoryPoint()
                    point2 = MultiDOFJointTrajectoryPoint()
                    point1.time_from_start = duration1.to_msg()
                    point2.time_from_start = duration2.to_msg()
                    transform1 = Transform()
                    transform2 = Transform()
                    transform1.translation.x = 0.0
                    transform1.rotation.w = 1.0
                    transform2.translation.x = inc
                    transform2.rotation.w = 1.0
                    point1.transforms = [transform1]
                    point2.transforms = [transform2]
                    # joint_name should be 'position' and not one generated by command i.e. 'translate/rotate_mobile_base'
                    joint_name = 'position'
                    trajectory_goal.multi_dof_trajectory.joint_names = [joint_name]
                    trajectory_goal.multi_dof_trajectory.points = [point1, point2]
                    trajectory_goal.multi_dof_trajectory.header.stamp = self.get_clock().now().to_msg()
                    self.trajectory_client.send_goal_async(trajectory_goal)

                # for non-base joints
                elif 'delta' in command:
                    point1 = JointTrajectoryPoint()
                    point2 = JointTrajectoryPoint()
                    point1.time_from_start = duration1.to_msg()
                    point2.time_from_start = duration2.to_msg()
                    joint_index = joint_state.name.index(joint_name)
                    joint_value = joint_state.position[joint_index]
                    delta = command['delta']
                    new_value = joint_value + delta
                    point1.positions = [joint_value]
                    point2.positions = [new_value]
                    trajectory_goal.trajectory.joint_names = [joint_name]
                    trajectory_goal.trajectory.points = [point1, point2]
                    trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                    self.trajectory_client.send_goal_async(trajectory_goal)

            else:
                self.get_logger().warn('Keyboard teleoperation available only in position or manipulaiton mode')


    def main(self):
        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.joint_states_sub

        self.robot_mode_sub = self.create_subscription(String, 'mode', self.mode_callback, 10)
        self.robot_mode_sub

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        if self.clean_surface_on:
            self.trigger_clean_surface_service = self.create_client(Trigger,
                                                                    '/clean_surface/trigger_clean_surface')
            self.trigger_clean_surface_service.wait_for_service()
            self.get_logger().info('Node ' + self.get_name() + ' connected to /clean_surface/trigger_clean_surface.')

        if self.grasp_object_on:
            self.trigger_grasp_object_service = self.create_client(Trigger,
                                                                    '/grasp_object/trigger_grasp_object')
            self.get_logger().info("Waiting for /grasp_object/trigger_grasp_object' service")
            self.trigger_grasp_object_service.wait_for_service()
            self.get_logger().info('Node ' + self.get_name() + ' connected to /grasp_object/trigger_grasp_object.')
        
        self.keys.print_commands()
        while rclpy.ok():
            rclpy.spin_once(self)
            command = self.keys.get_command(self)
            self.send_command(command)

        self.keys.kb.set_normal_term()
        self.destroy_node()
        rclpy.shutdown()

def main():
    try:
        rclpy.init()

        node = KeyboardTeleopNode()
        node.main()
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

if __name__ == '__main__':
    main()