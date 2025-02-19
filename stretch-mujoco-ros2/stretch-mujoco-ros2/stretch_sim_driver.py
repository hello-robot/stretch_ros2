#! /usr/bin/env python3

# stretch_ros2
import copy
import yaml
import numpy as np
import threading
from .rwlock import RWLock

import tf2_ros
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy
from std_msgs.msg import Bool, String, Float64MultiArray

from ament_index_python.packages import get_package_share_path


# stretch_mujoco
from stretch_mujoco import StretchMujocoSimulator
import stretch_mujoco.config as config
import stretch_mujoco.utils as utils



class StretchSimDriver(Node):
    def __init__(self, scene_xml_path: str = './scene.xml'):
        super().__init__('stretch_sim_driver')
        
        # TODO: initialization
        # hardcoding robo model SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf
        self.robot_sim = StretchMujocoSimulator(scene_xml_path)
        self.robot_sim.start()
        
        self.robot_mode_rwlock = RWLock()
        
        self.ros_setup()
    
    
    def command_mobile_base_velocity_and_publish_state(self):
        self.robot_mode_rwlock.acquire_read()
        

        current_clock = self.get_clock().now()  # update ros time
        current_time = current_clock.to_msg()

        
        # TODO: pull robot status and publish joint_state
        robot_status = self.robot_sim._pull_status()      # update with ros status
        # robot_status = self.status.copy()     # update with mujoco status
        joint_state = JointState()
        joint_state.header.stamp = current_time
        joint_state.name = self.robot_sim.urdf_model.joints_names
        
        
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self.joint_state_pub.publish(joint_state)
        
        # TODO: pull camera data and publish images
        camera_data = self.robot_sim.pull_camera_data()
        
        
    def ros_setup(self):
        self.node_name = self.get_name()
        
        
        self.mutex_group = MutuallyExclusiveCallbackGroup() # only one callback can be executing
        
        self.declare_parameter('rate', 30.0)
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.joint_limits_pub = self.create_publisher(JointState, 'joint_limits', 1)
        
        
        timer_period = 1.0 / self.joint_state_rate
        self.timer = self.create_timer(timer_period, self.command_mobile_base_velocity_and_publish_state, callback_group=self.mutex_group)