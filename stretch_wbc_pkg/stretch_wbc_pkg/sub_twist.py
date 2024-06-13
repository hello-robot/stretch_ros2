import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import csv

class SineTwistSubscriber(Node):
    def __init__(self):
        super().__init__('sine_twist_subscriber')
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        
        self.save = True
        self.time_start = self.get_clock().now().nanoseconds
        
        
        self.x_data = []
        self.linear_data = []
        self.angular_data = []
        
        if self.save:
            # save to a csv file where each row is is (time, linear.x, angular.z)
            self.csv_file = open('/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/twist_data.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['time', 'linear.x', 'angular.z'])

            
    # def save(self):
        
    def twist_callback(self, msg):
        current_time_sec = (self.get_clock().now().nanoseconds - self.time_start) / 1e9  # convert ns to s
        # current_time_sec = msg.header.stamp.sec
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Write data to CSV file
        self.csv_writer.writerow([current_time_sec, linear_x, angular_z])

        # Log the data
        self.get_logger().info(f'Time: {current_time_sec}, linear.x: {linear_x}, angular.z: {angular_z}')

        

def main(args=None):
    rclpy.init(args=args)
    node = SineTwistSubscriber()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()