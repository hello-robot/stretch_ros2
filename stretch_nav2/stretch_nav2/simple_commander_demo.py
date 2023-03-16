#! /usr/bin/env python3

# Adapted from the simple commander demo examples on 
# https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/demo_security.py

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


"""
Basic security route patrol demo. In this demonstration, we use the D435i camera
mounted on the robot to relay the camera feed back to us that can be monitored
using RViz.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    security_route = [
        [0.0, 0.0],
        [1.057, 1.3551],
        [1.5828, 5.0823],
        [-0.5390, 5.6623],
        [0.8975, 9.7033]]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Do security route until dead
    while rclpy.ok():
        # Send our route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in security_route[1:]:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(route_poses)

        # Do something during our route (e.x. AI detection on camera images for anomalies)
        # Simply print ETA for the demonstation
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                navigator.get_logger().info('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(route_poses)))
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600.0):
                    navigator.cancelTask()

        # If at end of route, reverse the route to restart
        security_route.reverse()

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('Route complete! Restarting...')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().info('Security route was canceled, exiting.')
            rclpy.shutdown()
        elif result == TaskResult.FAILED:
            navigator.get_logger().info('Security route failed! Restarting from other side...')

    rclpy.shutdown()


if __name__ == '__main__':
    main()