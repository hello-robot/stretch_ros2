#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class D435iAccelCorrectionNode(Node):
    def __init__(self):
        super().__init__('d435i_accel_correction_node')
        self.node_name = self.get_name()
        self.get_logger().info("{0} started".format(self.node_name))

        self.accel = Imu()
        self.num_samples_to_skip = 4
        self.sample_count = 0

        self.topic_name = '/camera/accel/sample'
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.accel_subscriber = self.create_subscription(Imu, self.topic_name, self.accel_callback, qos_policy)
        
        self.corrected_accel_pub = self.create_publisher(Imu, '/camera/accel/sample_corrected', 10)
        
    def accel_callback(self, accel):
        self.accel = accel
        self.sample_count += 1
        if((self.sample_count % self.num_samples_to_skip) == 0): 
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
        

def main():
    rclpy.init()
    node = D435iAccelCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
