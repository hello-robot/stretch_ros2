#! /usr/bin/env python3

# stretch_ros2
import copy
import yaml
import numpy as np
import threading

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
        
        self.ros_setup()
        
    def ros_setup(self):
        self.node_name = self.get_name()
        