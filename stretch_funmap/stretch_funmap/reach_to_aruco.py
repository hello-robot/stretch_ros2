#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient

from rclpy.node import Node
from stretch_funmap.action import ArucoHeadScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from std_srvs.srv import Trigger

class Reach2Aruco(Node):
    def __init__(self):
        super().__init__('head_scan_client')
        self._action_client = ActionClient(self, ArucoHeadScan, 'aruco_head_scan')
        self._service_client = self.create_client(Trigger, 'switch_to_trajectory_mode')
        while not self._service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def switch_to_trajectory_mode(self):
        self.future = self._service_client.call_async(self.req)
        return self.future.result()
    
    def send_head_scan_goal(self):
        response = self.switch_to_trajectory_mode()

        goal_msg = ArucoHeadScan.Goal()
        
        goal_msg.aruco_id = 245
        goal_msg.tilt_angle = 0.0
        goal_msg.publish_to_map = True
        goal_msg.fill_in_blindspot_with_second_scan = False
        goal_msg.fast_scan = True

        self.get_logger().info("Waiting for aruco head scan server to start")
        self._action_client.wait_for_server()

        self.get_logger().info("Sending aruco head scan goal")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.aruco_head_scan_goal_response_cb)

    def aruco_head_scan_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.aruco_head_scan_get_result_cb)

    def aruco_head_scan_get_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Aruco head scan completed')

    def navigate_to_pose(self):
        pass

    def move_to_pose(self):
        pass

    def main(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    reach_to_aruco = Reach2Aruco()

    reach_to_aruco.send_head_scan_goal()

    rclpy.spin(reach_to_aruco)

if __name__ == '__main__':
    main()