#!/usr/bin/env python

from __future__ import print_function

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

class D435iAccelCorrectionNode(Node):
    def __init__(self):
        super().__init__('D435iAccelCorrectionNode')
        self.num_samples_to_skip = 4
        self.sample_count = 0

        self.ros_setup()

    def accel_callback(self, accel):
        self.accel = accel
        self.sample_count += 1
        if (self.sample_count % self.num_samples_to_skip) == 0:
            # This can avoid issues with the D435i's time stamps being too
            # far ahead for TF.
            self.accel.header.stamp = self.get_clock().now().to_msg()
            x = self.accel.linear_acceleration.x
            y = self.accel.linear_acceleration.y
            z = self.accel.linear_acceleration.z

            self.accel.linear_acceleration.x = x
            self.accel.linear_acceleration.y = y
            self.accel.linear_acceleration.z = z

            self.corrected_accel_pub.publish(self.accel)

    def ros_setup(self):
        self.node_name = self.get_name()
        self.get_logger().info('{0} started'.format(self.node_name))

        self.topic_name = '/camera/accel/sample'
        self.accel_subscriber = self.create_subscription(Imu, self.topic_name, self.accel_callback, 1)

        self.corrected_accel_pub = self.create_publisher(Imu, '/camera/accel/sample_corrected', 1)


def main():
    try:
        rclpy.init()
        node = D435iAccelCorrectionNode()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
