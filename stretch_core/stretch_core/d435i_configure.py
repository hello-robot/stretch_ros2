#!/usr/bin/env python3

import rclpy
import time
import dynamic_reconfigure.client
from std_srvs.srv import Trigger, Trigger_Response
import threading

class D435iConfigureNode:
    def __init__(self):
        self.rate = 1.0
        self.visual_preset = None
        self.mode_lock = threading.Lock()
        
    def turn_on_default_mode(self):
        with self.mode_lock: 
            self.locked_mode_id = 1
            self.locked_mode_name = 'Default'
            self.parameter_client.update_configuration({'visual_preset' : self.locked_mode_name})
            self.node.get_logger().info("Set D435i to {0} mode".format(self.locked_mode_name))
        
    def turn_on_high_accuracy_mode(self):
        with self.mode_lock: 
            self.locked_mode_id = 3
            self.locked_mode_name = 'High Accuracy'
            self.parameter_client.update_configuration({'visual_preset' : self.locked_mode_name})
            self.node.get_logger().info("Set D435i to {0} mode".format(self.locked_mode_name))
        
    def default_mode_service_callback(self, request):
        self.turn_on_default_mode()
        return Trigger_Response(
            success=True,
            message='Default mode enabled.'
            )

    def high_accuracy_mode_service_callback(self, request):
        self.turn_on_high_accuracy_mode()
        return Trigger_Response(
            success=True,
            message='High Accuracy mode enabled.'
            )

    def main(self):
        time.sleep(3) # sleep for 3 seconds to wait for D435i to be ready
        rclpy.init()
        self.node = rclpy.create_node('configure_d435i', 
                                       allow_undeclared_parameters=True,
                                       automatically_declare_parameters_from_overrides=True)
        self.node_name = self.node.get_name()
        self.node.get_logger().info("{0} started".format(self.node_name))

        self.parameter_client = dynamic_reconfigure.client.Client('/camera/stereo_module/')

        self.switch_to_default_mode_service = self.node.create_service(Trigger,
                                                                '/camera/switch_to_default_mode',
                                                                 self.default_mode_service_callback)

        self.switch_to_high_accuracy_mode_service = self.node.create_service(Trigger,
                                                                '/camera/switch_to_high_accuracy_mode',
                                                                self.high_accuracy_mode_service_callback)

        initial_mode = self.node.get_parameter_or('~initial_mode')
        
        self.node.get_logger().info("initial_mode = {0}".format(initial_mode))

        if initial_mode == 'High Accuracy':
            self.turn_on_high_accuracy_mode()
        elif initial_mode == 'Default':
            self.turn_on_default_mode()
        else:
            error_string = 'initial_mode = {0} not recognized. Setting to D435i to Default mode.'.format(initial_mode)
            self.node.get_logger().error(error_string)
            self.turn_on_default_mode()
        
        rate = self.node.create_rate(self.rate)
        while rclpy.ok():
            rate.sleep()

            
if __name__ == '__main__':
    try: 
        node = D435iConfigureNode()
        node.main()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
