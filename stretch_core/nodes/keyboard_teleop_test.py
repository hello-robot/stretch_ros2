#!/usr/bin/env python3

import math
import keyboard as kb
import argparse as ap

import rclpy
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import hello_helpers.hello_misc as hm


class GetKeyboardCommands:

    def __init__(self, mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on):
        
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
        self.mode = 'position' #'manipulation' #'navigation'

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

        c = kb.getch()
            
        ####################################################
        ## BASIC KEYBOARD TELEOPERATION COMMANDS
        ####################################################
        
        # 8 or up arrow
        if c == '8' or c == '\x1b[A':
            command = {'joint': 'joint_lift', 'delta': self.get_deltas()['translate']}
        # 2 or down arrow
        if c == '2' or c == '\x1b[B':
            command = {'joint': 'joint_lift', 'delta': -self.get_deltas()['translate']}
        if self.mode == 'manipulation':
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

        if c == 'q' or c == 'Q':
            node.get_logger().info('keyboard_teleop exiting...')
            node.get_logger().info('Received quit character (q), so exiting')

        ####################################################

        return command


class KeyboardTeleopNode(Node):

    def __init__(self, mapping_on=False, hello_world_on=False, open_drawer_on=False, clean_surface_on=False, grasp_object_on=False, deliver_object_on=False):
        self.keys = GetKeyboardCommands(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on)
        self.rate = 10.0
        self.joint_state = JointState()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
        # self.get_logger().info(self.joint_state.name[0])

    def send_command(self, command):
        print('command received')
        joint_state = self.joint_state
        # self.get_logger().info('joint_name = {0}, inc = {1}'.format(command['joint'], command['inc']))
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            trajectory_goal = FollowJointTrajectory.Goal()
            # trajectory_goal.goal_time_tolerance = rclpy.Time(1.0)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            if 'inc' in command:
                inc = command['inc']
                new_value = inc
            elif 'delta' in command:
                joint_index = joint_state.name.index(joint_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            current_time = self.get_clock().now()
            trajectory_goal.trajectory.header.stamp = current_time.to_msg()
            self.trajectory_client.send_goal_async(trajectory_goal)
            # self.get_logger().info('Done sending pose.')
            # self.keys.print_commands()

    def main(self):
        rclpy.init()
        super().__init__('keyboard_test')

        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 10)
        self.subscription  # prevent unused variable warning

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        while not self.trajectory_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/stretch_controller/follow_joint_trajectory' action server...")

        rate = self.create_rate(self.rate)

        self.keys.print_commands()
        while rclpy.ok():
            rclpy.spin_once(self)
            command = self.keys.get_command(self)
            self.send_command(command)
        self.destroy_node()
        rclpy.shutdown()

def main():
    try:
        node = KeyboardTeleopNode()
        node.main()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')

if __name__ == '__main__':
    main()