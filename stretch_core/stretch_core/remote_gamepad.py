#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from hello_helpers.gamepad_conversion import *
from pprint import pprint
from stretch_body.gamepad_controller import GamePadController
from stretch_body.hello_utils import ThreadServiceExit

PRINT_DEBUG = True

class StretchRemoteGamepad(Node):
    def __init__(self):
        super().__init__('stretch_remote_gamepad')
        print("Starting Stretch Remote Gamepad....")
        self.joy_publisher = self.create_publisher(Joy, 'gamepad_joy', 10)  
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.gamepad_controller = GamePadController()
        self.gamepad_controller.start()
    
    def gamepad_state_to_joy(self,gamepad_state):
        current_clock = self.get_clock().now()
        current_time = current_clock.to_msg()
        out_msg = unpack_gamepad_state_to_joy(gamepad_state)
        out_msg.header.stamp = current_time
        if PRINT_DEBUG:
            print(pprint(unpack_joy_to_gamepad_state(out_msg)))
        return out_msg
    
    def publish_message(self):
        msg = self.gamepad_state_to_joy(self.gamepad_controller.gamepad_state)
        self.joy_publisher.publish(msg)

def main():
    try:
        rclpy.init()
        node = StretchRemoteGamepad()
        try:
            rclpy.spin(node)
        finally:
            node.gamepad_controller.stop()
            node.destroy_node()
    except (KeyboardInterrupt, ThreadServiceExit):
        node.gamepad_controller.stop()
        node.destroy_node()

if __name__ == '__main__':
    main()