#!/usr/bin/env python3

from copy import deepcopy
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from stretch_funmap.action import ArucoHeadScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult


class Reach2Aruco(Node):
    def __init__(self):
        super().__init__('head_scan_client')
        self.aruco_head_scan_action_client = ActionClient(self, ArucoHeadScan, 'aruco_head_scan')
        self.switch_to_trajectory_mode_service_client = self.create_client(Trigger, 'switch_to_trajectory_mode')

    def switch_to_trajectory_mode(self):
        self.future = self.switch_to_trajectory_mode_service_client.call_async(self.req)
        return self.future.result()
    
    def switch_to_navigation_mode(self):
        self.future = self.switch_to_navigation_mode_service_client.call_async(self.req)
        return self.future.result()
    
    def send_head_scan_goal(self):
        response = self.switch_to_trajectory_mode()

        goal_msg = ArucoHeadScan.Goal()
        
        goal_msg.aruco_id = 245
        goal_msg.tilt_angle = -0.85
        goal_msg.publish_to_map = True
        goal_msg.fill_in_blindspot_with_second_scan = False
        goal_msg.fast_scan = True

        self.get_logger().info("Waiting for aruco head scan server to start")
        self.aruco_head_scan_action_client.wait_for_server()

        self.get_logger().info("Sending aruco head scan goal")
        self._send_goal_future = self.aruco_head_scan_action_client.send_goal_async(goal_msg)

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

    def nav_to_pose(self, goal_pose):
        response = self.switch_to_navigation_mode()

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
                time.sleep(2.0)
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def move_to_pose(self):
        response = self.switch_to_trajectory_mode()
        pass

    def main(self):
        while not self.switch_to_trajectory_mode_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.switch_to_navigation_mode_service_client = self.create_client(Trigger, 'switch_to_navigation_mode')
        while not self.switch_to_navigation_mode_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = Trigger.Request()
        
        self.navigator = BasicNavigator()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

def main(args=None):
    rclpy.init(args=args)

    reach_to_aruco = Reach2Aruco()
    reach_to_aruco.main()
    reach_to_aruco.send_head_scan_goal()

    rclpy.spin(reach_to_aruco)

if __name__ == '__main__':
    main()