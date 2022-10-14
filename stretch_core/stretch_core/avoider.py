#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from numpy import linspace, inf, tanh
from math import sin, isnan
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Avoider(Node):
    def __init__(self):
        super().__init__('stretch_avoider')
        
        self.width = 1
        self.extent = self.width / 2.0
        self.distance = 0.75 # robot turns at this distance
        self.keepout = 0.4 # robot stops at this distance

        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        self.publisher_ = self.create_publisher(Twist, '/stretch/cmd_vel', 1) #/stretch_diff_drive_controller/cmd_vel for gazebo
        self.subscriber_ = self.create_subscription(LaserScan, '/scan_filtered', self.lidar_callback, 10)

    def set_speed(self, lin_vel, rot_vel):
        self.twist.linear.x = lin_vel
        self.twist.angular.z = rot_vel
        self.publisher_.publish(self.twist)
    
    def lidar_callback(self, msg):
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        # Invalid values converted to inf so that they do not affect min_range
        all_points = [r if (not isnan(r)) else inf for r in msg.ranges]
        front_points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]
        front_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, front_points)]

        min_front = min(front_ranges)
        min_all = min(all_points)
        if(min_all < self.keepout):
            lin_vel = 0.0
            rot_vel = 0.0
        elif(min_front < self.distance):
            lin_vel = 0.0
            rot_vel = 0.25
        else:
            lin_vel = 0.5
            rot_vel = 0.0

        self.set_speed(lin_vel, rot_vel)


def main(args=None):
    rclpy.init(args=args)

    avoider = Avoider()

    rclpy.spin(avoider)

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

