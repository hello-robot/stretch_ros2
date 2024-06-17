import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import csv
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class SineVelPublisher(Node):
    def __init__(self):
        super().__init__('sine_vel_publisher')
        
        # Load the configuration from the YAML file
        package_share_directory = get_package_share_directory('stretch_wbc_pkg')
        config_file = os.path.join(package_share_directory, 'config', 'joint_vel_pub_config.yaml')
        
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
            
        #Sanity check the config file
        # self.print_joint_limits_from_config()
        
        # command velcoity publisher
        self.base_publisher_ = self.create_publisher(TwistStamped, '/base_cmd_vel', 10)
        self.arm_publisher_ = self.create_publisher(TwistStamped, '/arm_cmd_vel', 10)
        self.lift_publisher_ = self.create_publisher(TwistStamped, '/lift_cmd_vel', 10)
        self.wrist_publisher_ = self.create_publisher(TwistStamped, '/wrist_cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time_start = self.get_clock().now().nanoseconds
        
        self.plot = False
        self.save = True

        
        if self.config["base"]["save"]:
            self.base_csv_file = open('/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/base_cmd_vel_data.csv', 'w', newline='')
            self.base_csv_writer = csv.writer(self.base_csv_file)
            self.base_csv_writer.writerow(['time', 'base_linear.x', 'base_angular.z'])
        if self.config["lift"]["save"]:
            self.lift_csv_file = open('/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/lift_cmd_vel_data.csv', 'w', newline='')
            self.lift_csv_writer = csv.writer(self.lift_csv_file)
            self.lift_csv_writer.writerow(['time', 'lift_linear'])
        if self.config["arm"]["save"]:
            self.arm_csv_file = open('/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/arm_cmd_vel_data.csv', 'w', newline='')
            self.arm_csv_writer = csv.writer(self.arm_csv_file)
            self.arm_csv_writer.writerow(['time', 'arm_linear'])
        if self.config["wrist"]["save"]:
            self.wrist_csv_file = open('/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/wrist_cmd_vel_data.csv', 'w', newline='')
            self.wrist_csv_writer = csv.writer(self.wrist_csv_file)
            self.wrist_csv_writer.writerow(['time', 'wrist_angular'])
        
    def plot_init(self):
        self.ax.set_xlim(0, 10) 
        self.ax.set_ylim(-2, 2)
        return self.ln,
        
    def update_plot(self):
        # self.ax.set_xlim(0, max(self.x_data[-1], 10))  # Dynamically adjust x limit
        self.ln.set_data(self.x_data, self.linear_data)
        return self.ln,
    
    def l_vel_profile(self, time_elapsed, freq, limits):
        min, max = limits
        amp = (max - min) / 2
        offset = (max + min) / 2
        raw_vel = np.sin(time_elapsed * 2 * np.pi * freq)
        l_vel = amp * raw_vel + offset
        return l_vel
    
    def ang_vel_profile(self, time_elapsed, freq, limits):
        min, max = limits
        amp = (max - min) / 2
        offset = (max + min) / 2
        raw_vel = np.sin(time_elapsed * 2 * np.pi * freq)
        a_vel = amp * raw_vel + offset
        # may need to do clamping or somehting here
        return a_vel
    
    def print_joint_limits_from_config(self,):
        print("Joint Limits:")
        for joint, params in self.config.items():
            if 'limits' in params:
                print(f"{joint.capitalize()} Limits:")
                for limit_type, values in params['limits'].items():
                    print(f"  {limit_type.capitalize()}: {values}")
            else:
                print(f"{joint.capitalize()} has no limits defined.")
    
    def timer_callback(self,):
        
        current_time = self.get_clock().now()
        current_time_sec = (self.get_clock().now().nanoseconds - self.time_start) / 1e9
        time_elapsed = (current_time.nanoseconds - self.time_start) / 1e9
        
        if self.config["base"]["pub"]:
            base_msg = TwistStamped()
            base_freq = self.config["base"]["freq"]
            # base_amp = self.config["base"]["amp"]
            l_limits = self.config["base"]["limits"]["linear"]
            a_limits = self.config["base"]["limits"]["angular"]
            
            #compute the linear and angular velocity for base joint
            linear_vel = self.l_vel_profile(time_elapsed, base_freq, l_limits)
            angular_vel = self.ang_vel_profile(time_elapsed, base_freq, a_limits)
            
            if self.config["base"]["limits"]["linear"][0] == 0.0 and self.config["base"]["limits"]["linear"][1] == 0.0:
                linear_vel = 0.0
            if self.config["base"]["limits"]["angular"][0] == 0.0 and self.config["base"]["limits"]["angular"][1] == 0.0:
                angular_vel = 0.0
            
            base_msg.header.stamp = current_time.to_msg()
            base_msg.header.frame_id = 'base_link'
            base_msg.twist.linear.x = linear_vel
            base_msg.twist.linear.y = 0.0
            base_msg.twist.linear.z = 0.0
            base_msg.twist.angular.x = 0.0
            base_msg.twist.angular.y = 0.0
            base_msg.twist.angular.z = angular_vel
            
            # save to csv file
            if self.config["base"]["save"]:
                self.base_csv_writer.writerow([current_time_sec, base_msg.twist.linear.x, base_msg.twist.angular.z])
            
            
            # Publish base cmd vel to /base_cmd_vel topic
            self.base_publisher_.publish(base_msg)
            
        if self.config["lift"]["pub"]:
            lift_msg = TwistStamped()
            lift_freq = self.config["base"]["freq"]
            # lift_amp = self.config["base"]["amp"]\
            lift_limits = self.config["lift"]["limits"]["linear"]
            
            #compute the linear velocity for lift joint
            linear_vel = self.l_vel_profile(time_elapsed, lift_freq, lift_limits)

            
            lift_msg.header.stamp = current_time.to_msg()
            lift_msg.header.frame_id = 'base_link'
            lift_msg.twist.linear.x = linear_vel
            lift_msg.twist.linear.y = 0.0
            lift_msg.twist.linear.z = 0.0
            lift_msg.twist.angular.x = 0.0
            lift_msg.twist.angular.y = 0.0
            lift_msg.twist.angular.z = 0.0
            
            if self.config["lift"]["save"]:
                self.lift_csv_writer.writerow([current_time_sec, lift_msg.twist.linear.x])
            
            # Publish base cmd vel to /base_cmd_vel topic
            self.lift_publisher_.publish(lift_msg)
            
        if self.config["arm"]["pub"]:
            arm_msg = TwistStamped()
            arm_freq = self.config["arm"]["freq"]
            # arm_amp = self.config["base"]["amp"]
            arm_limits = self.config["arm"]["limits"]["linear"]
            
            #compute the linear velocity for lift joint
            linear_vel = self.l_vel_profile(time_elapsed, arm_freq, arm_limits)

            
            arm_msg.header.stamp = current_time.to_msg()
            arm_msg.header.frame_id = 'base_link'
            arm_msg.twist.linear.x = linear_vel
            arm_msg.twist.linear.y = 0.0
            arm_msg.twist.linear.z = 0.0
            arm_msg.twist.angular.x = 0.0
            arm_msg.twist.angular.y = 0.0
            arm_msg.twist.angular.z = 0.0
            
            if self.config["arm"]["save"]:
                self.arm_csv_writer.writerow([current_time_sec, arm_msg.twist.linear.x])
            
            # Publish base cmd vel to /base_cmd_vel topic
            self.arm_publisher_.publish(arm_msg)
            
        if self.config["wrist"]["pub"]:
            wrist_msg = TwistStamped()
            wrist_freq = self.config["wrist"]["freq"]
            # wrist_amp = self.config["wrist"]["amp"]
            wrist_limits = self.config["wrist"]["limits"]["angular"]
            
            #compute the linear velocity for lift joint
            ang_vel = self.ang_vel_profile(time_elapsed, wrist_freq, wrist_limits)

            
            wrist_msg.header.stamp = current_time.to_msg()
            wrist_msg.header.frame_id = 'base_link'
            wrist_msg.twist.linear.x = 0.0
            wrist_msg.twist.linear.y = 0.0
            wrist_msg.twist.linear.z = 0.0
            wrist_msg.twist.angular.x = 0.0
            wrist_msg.twist.angular.y = 0.0
            wrist_msg.twist.angular.z = ang_vel
            
            if self.config["wrist"]["save"]:
                self.wrist_csv_writer.writerow([current_time_sec, wrist_msg.twist.angular.z])
            
            # Publish base cmd vel to /base_cmd_vel topic
            self.wrist_publisher_.publish(wrist_msg)
      

def main(args=None):
    
    rclpy.init(args=args)
    node = SineVelPublisher()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()