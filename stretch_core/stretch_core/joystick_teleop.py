from sensor_msgs.msg import Joy

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
import math
import sys
import time


class GetJoystickCommands:

    def __init__(self, node):
        self.joy = Joy()
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
        self.assisted_teleop = 0

    def get_deltas(self):
        if self.step_size == 'small':
            deltas = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            deltas = {'rad': self.medium_rad, 'translate': self.medium_translate} 
        if self.step_size == 'big':
            deltas = {'rad': self.big_rad, 'translate': self.big_translate} 
        return deltas

    def print_commands(self):
        print('---------- JOYSTICK TELEOP MENU -----------')
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

    def get_command(self, node, joy):
        self.joy = joy
        command = None
        c = None

        if(not self.joy or self.joy == None):
            return command

        ####################################################
        ## KEY MAPPINGS
        ####################################################
        
        # self.joy.axes[0] = left joy horizontal, right = negative (potentiometer)
        # self.joy.axes[1] = left joy vertical, down = negative (potentiometer)
        # self.joy.axes[2] = front left unpressed is 1, pressed is -1 (potentiometer)
        # self.joy.axes[3] = rigth joy horizontal, right is negative (potentiometer)
        # self.joy.axes[4] = right joy vertical, down is negative (potentiometer)
        # self.joy.axes[5] = front right unpressed is 1, pressed is -1 (potentiometer)
        # self.joy.axes[6] = plus right press is -1, plus left press is 1 (switch)
        # self.joy.axes[7] = plus up press is 1, plus down press is -1 (switch)

        # self.joy.buttons[0] = Green Button A, press is 1
        # self.joy.buttons[1] = Red Button B, press is 1
        # self.joy.buttons[2] = Blue Button X, press is 1
        # self.joy.buttons[3] = Yellow Button Y, press is 1
        # self.joy.buttons[4] = Front left button, press is 1
        # self.joy.buttons[5] = Front right button, press is 1
        # self.joy.buttons[6] = Back Button, press is 1
        # self.joy.buttons[7] = Start Button, press is 1
        # self.joy.buttons[8] = Home Button, press is 1
        # self.joy.buttons[9] = Joy left Button, press is 1
        # self.joy.buttons[10] = Joy right Button, press is 1
        
        ####################################################
        ## DEMO RELATED CAPABILITIES
        ####################################################
        # Trigger Assisted Teleop demo
        if ((self.joy.buttons[3])): # Yellow button pressed
            self.assisted_teleop = 1
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_write_hello_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))
        
        # Choose Red object
        if (self.joy.buttons[1] and self.assisted_teleop): # Red button pressed
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_local_localization_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Choose Green object
        if (self.joy.buttons[0] and self.assisted_teleop): # Green button pressed
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_align_with_nearest_cliff_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Choose Blue object   
        if (self.joy.buttons[2] and self.assisted_teleop): # Blue button pressed
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_grasp_object_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))
            
        ####################################################
        ## JOYSTICK TELEOPERATION COMMANDS
        ####################################################
        
        # 8 or up arrow
        if self.joy.axes[4] > 0.25:
            command = {'joint': 'joint_lift', 'delta': self.get_deltas()['translate']}
        # 2 or down arrow
        if self.joy.axes[4] < -0.25:
            command = {'joint': 'joint_lift', 'delta': -self.get_deltas()['translate']}
        
        if self.mode == 'manipulation':
            # 4 or left arrow
            if self.joy.axes[1] > 0.25:
                command = {'joint': 'joint_mobile_base_translation', 'delta': self.get_deltas()['translate']}
            # 6 or right arrow
            if self.joy.axes[1] < -0.25:
                command = {'joint': 'joint_mobile_base_translation', 'delta': -self.get_deltas()['translate']}
        
        elif self.mode == 'position':
            # 4 or left arrow
            if self.joy.axes[1] > 0.25:
                command = {'joint': 'translate_mobile_base', 'inc': self.get_deltas()['translate']}
            # 6 or right arrow
            if self.joy.axes[1] < -0.25:
                command = {'joint': 'translate_mobile_base', 'inc': -self.get_deltas()['translate']}
            # 1 or end key 
            if self.joy.axes[0] > 0.25:
                command = {'joint': 'rotate_mobile_base', 'inc': self.get_deltas()['rad']}
            # 3 or pg down 5~
            if self.joy.axes[0] < -0.25:
                command = {'joint': 'rotate_mobile_base', 'inc': -self.get_deltas()['rad']}
        
        elif self.mode == 'navigation':
            node.get_logger().info('ERROR: Navigation mode is not currently supported.')

        # arm out
        if self.joy.axes[3] < -0.25:
            command = {'joint': 'wrist_extension', 'delta': self.get_deltas()['translate']}
        # arm in
        if self.joy.axes[3] > 0.25:
            command = {'joint': 'wrist_extension', 'delta': -self.get_deltas()['translate']}
        if self.joy.buttons[4]:
            command = {'joint': 'joint_wrist_yaw', 'delta': -self.get_deltas()['rad']}
        if self.joy.buttons[5]:
            command = {'joint': 'joint_wrist_yaw', 'delta': self.get_deltas()['rad']}
        if self.joy.buttons[1]:
            # grasp
            command = {'joint': 'joint_gripper_finger_left', 'delta': -self.get_deltas()['rad']}
        if self.joy.buttons[0]:
            # release
            command = {'joint': 'joint_gripper_finger_left', 'delta': self.get_deltas()['rad']}
        if self.joy.axes[7] == 1:
            command = {'joint': 'joint_head_tilt', 'delta': (2.0 * self.get_deltas()['rad'])}
        if self.joy.axes[7] == -1:
            command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if self.joy.axes[6] == 1:
            command = {'joint': 'joint_head_pan', 'delta': (2.0 * self.get_deltas()['rad'])}
        if self.joy.axes[6] == -1:
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
        if self.joy.buttons[6]:
            node.get_logger().info('joystick_teleop exiting...')
            node.get_logger().info('Received quit character (q), so exiting')

        ####################################################

        return command


class JoystickTeleopNode(Node):

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("joystick_teleop")
        self.keys = GetJoystickCommands(self.node)
        self.joint_state = JointState()
        self.joy = Joy()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def joystick_callback(self, joy):
        self.joy = joy

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
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
                trajectory_goal.multi_dof_trajectory.header.stamp = self.node.get_clock().now().to_msg()
                self.trajectory_client.send_goal_async(trajectory_goal)
                # time.sleep(1.0)

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
                trajectory_goal.trajectory.header.stamp = self.node.get_clock().now().to_msg()
                self.trajectory_client.send_goal_async(trajectory_goal)
                # time.sleep(1.0)

    def command(self):
        command = self.keys.get_command(self.node, self.joy)
        self.send_command(command)
    
    def main(self):

        self.trajectory_client = ActionClient(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.node.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()
        
        self.joint_states_sub = self.node.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 10)
        self.joint_states_sub

        self.joystick_sub = self.node.create_subscription(Joy, '/joy', self.joystick_callback, 10)
        self.joystick_sub

        self.keys.print_commands()

        for i in range(10):
            rclpy.spin_once(self.node)

        timer_period = 1.0  # seconds
        self.timer = self.node.create_timer(timer_period, self.command)

        rclpy.spin(self.node)

        self.node.destroy_node()
        rclpy.shutdown()

def main():
    try:
        joy = JoystickTeleopNode()
        joy.main()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')

if __name__ == '__main__':
    main()