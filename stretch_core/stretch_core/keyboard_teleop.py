#!/usr/bin/env python3

import math
from .keyboard import KBHit

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import hello_helpers.hello_misc as hm


class GetKeyboardCommands:

    def __init__(self):
        self.kb = KBHit()
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
        c = None

        if self.kb.kbhit(): # Returns True if any key pressed
            c = self.kb.getch()
            
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


class KeyboardTeleopNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.keys = GetKeyboardCommands()
        self.joint_state = JointState()
        
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
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

    def main(self):
        rclpy.init()
        hm.HelloNode.main(self, 'keyboard_teleop', 'keyboard_teleop', wait_for_first_pointcloud=False)

        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.subscription
        
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
        node = KeyboardTeleopNode()
        node.main()
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

if __name__ == '__main__':
    main()