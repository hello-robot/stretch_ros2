#!/usr/bin/env python3

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

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy, Image
from std_msgs.msg import Bool, String, Float64MultiArray

from hello_helpers.gripper_conversion import GripperConversion
# from hello_helpers.joint_qpos_conversion import get_Idx, UnsupportedToolError
# from hello_helpers.hello_misc import LoopTimer
# from hello_helpers.gamepad_conversion import unpack_joy_to_gamepad_state, unpack_gamepad_state_to_joy, get_default_joy_msg

from ament_index_python.packages import get_package_share_path
# from ament_index_python.packages import get_package_share_directory

import cv2
from cv_bridge import CvBridge

# stretch_mujoco
# from stretch_mujoco import StretchMujocoSimulator
from .stretch_mujoco_wrapper import StretchMujocoSimulator
import stretch_mujoco.config as config
import stretch_mujoco.utils as utils


# local
import time
import os
package_name = 'stretch_mujoco_ros2'
package_path = get_package_share_path(package_name)
default_scene_xml_path = str(package_path / 'scene' / 'scene.xml')

class StretchSimDriver(Node):
    def __init__(self, robot_sim: StretchMujocoSimulator, scene_xml_path: str = default_scene_xml_path):
        super().__init__('stretch_sim_driver')
        self.use_robotis_head = True
        self.use_robotis_end_of_arm = True
        
        # TODO: initialization
        # hardcoding robo model stretch_urdf/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf

        self.robot_sim = robot_sim  # pass in the simulator
        # self.robot_sim = StretchMujocoSimulator(scene_xml_path)
        # self.robot_sim.start()
        self.get_logger().info('Started the robot simulator.')
        # self.actuator_names = self.get_actuator_names(self.robot_sim.mjmodel)
        
        self.gripper_conversion = GripperConversion()
        
        self.robot_stop_lock = threading.Lock()
        
        self.robot_mode_rwlock = RWLock()
        # self.control_modes = ['position', 'navigation', 'trajectory', 'gamepad']
        
        # camera_data = self.robot_sim.pull_camera_data()
        # self.get_logger().info(f"Pulled camera_data: {camera_data.keys()}")
        
        self.ros_setup()    
    
    # MOBILE BASE VELOCITY METHODS ############
    
    def set_mobile_base_velocity_callback(self, twist):
        self.robot_mode_rwlock.acquire_read()
        
        self.linear_velocity_mps = twist.linear.x       # vel_x
        self.angular_velocity_radps = twist.angular.z   # vel_theta
        self.last_twist_time = self.get_clock().now()
        self.robot_mode_rwlock.release_read()
        
    def set_robot_streaming_position_callback(self, msg):
        self.robot_mode_rwlock.acquire_read()
        
        qpos = msg.data
        self.move_to_position(qpos)
        self.robot_mode_rwlock.release_read()
        
    def move_to_position(self, qpos: list):
        allowed_actuators = config.allowed_position_actuators
        for i, actuator_name in enumerate(allowed_actuators):
            pos = qpos[i]
            if actuator_name not in ['base_translate', 'base_rotate']:
                self.robot_sim.move_to(actuator_name=actuator_name, pos=pos)

    
    def command_mobile_base_velocity_and_publish_state(self):
        self.robot_mode_rwlock.acquire_read()
        

        current_clock = self.get_clock().now()  # ros time
        current_time = current_clock.to_msg()

        time_since_last_twist = self.get_clock().now() - self.last_twist_time
        if time_since_last_twist < self.timeout:
            self.robot_sim.set_base_velocity(self.linear_velocity_mps, self.angular_velocity_radps)
            # self.robot.push_command() #Moved to main
        elif time_since_last_twist < Duration(seconds=self.timeout_s+1.0):
            self.robot_sim.move_by(actuator_name="base_translate", pos=0.0)
            # self.robot.push_command() #Moved to main
        else:
            self.robot_sim.set_base_velocity(0.0, 0.0)
            # self.robot.push_command() #Moved to main
                
        # TODO: pull robot status and publish joint_state
        robot_status = self.robot_sim._pull_status().copy()     # update with ros status (rw lock?)
        # robot_status = self.robot_sim.status.copy()           # update with mujoco status
        
        # obtain odometry
        # assign relevant base status to variables
        base_status = robot_status['base']
        x = base_status['x']
        y = base_status['y']
        theta = base_status['theta']
        x_vel = base_status['x_vel']
        y_vel = base_status['y_vel']    # 0.0
        theta_vel = base_status['theta_vel']
        
        # assign relevant arm status to variables
        arm_status = robot_status['arm']
        pos_out = arm_status['pos'] # + arm_backlash_correction
        vel_out = arm_status['vel']
        # eff_out = arm_status['motor']['effort_pct']
        eff_out = arm_status['effort']
        
        lift_status = robot_status['lift']
        pos_up = lift_status['pos']
        vel_up = lift_status['vel']
        # eff_up = lift_status['motor']['effort_pct']
        eff_up = lift_status['effort']
        
        
        if self.use_robotis_end_of_arm:
            # assign relevant wrist status to variables
            wrist_yaw_status = robot_status['end_of_arm']['wrist_yaw']
            wrist_yaw_rad = wrist_yaw_status['pos']
            wrist_yaw_vel = wrist_yaw_status['vel']
            wrist_yaw_effort = wrist_yaw_status['effort']

            dex_wrist_attached = False
            if 'wrist_pitch' in robot_status['end_of_arm']:
                dex_wrist_attached = True
                
            if dex_wrist_attached:
                wrist_pitch_status = robot_status['end_of_arm']['wrist_pitch']
                wrist_pitch_rad = wrist_pitch_status['pos']
                wrist_pitch_vel = wrist_pitch_status['vel']
                wrist_pitch_effort = wrist_pitch_status['effort']
                
                wrist_roll_status = robot_status['end_of_arm']['wrist_roll']
                wrist_roll_rad = wrist_roll_status['pos']
                wrist_roll_vel = wrist_roll_status['vel']
                wrist_roll_effort = wrist_roll_status['effort']
            
            gripper_status = robot_status['end_of_arm']['stretch_gripper']
            gripper_status['pos_pct'] = gripper_status['effort']
            gripper_aperture_m, gripper_finger_rad, gripper_finger_effort, gripper_finger_vel = \
                        self.gripper_conversion.status_to_all(gripper_status)     # no gripper effort data
    
        if self.use_robotis_head:
            # assign relevant head pan status to variables
            head_pan_status = robot_status['head']['head_pan']
            head_pan_rad = head_pan_status['pos'] # + self.head_pan_calibrated_offset_rad + pan_backlash_correction
            head_pan_vel = head_pan_status['vel']
            head_pan_effort = head_pan_status['effort']
            
            # assign relevant head tilt status to variables
            head_tilt_status = robot_status['head']['head_tilt']
            head_tilt_rad = head_tilt_status['pos'] # + self.head_tilt_calibrated_offset_rad + tilt_backlash_correction
            head_tilt_vel = head_tilt_status['vel']
            head_tilt_effort = head_tilt_status['effort']
        
        q = quaternion_from_euler(0.0, 0.0, theta)
        
        if self.broadcast_odom_tf:
            # publish odometry via TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

            b = TransformStamped()
            b.header.stamp = current_time
            b.header.frame_id = self.base_frame_id
            b.child_frame_id = "base_footprint"
            b.transform.translation.x = 0.0
            b.transform.translation.y = 0.0
            b.transform.translation.z = 0.0
            b.transform.rotation.x = 0.0
            b.transform.rotation.y = 0.0
            b.transform.rotation.z = 0.0
            b.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(b)
        
        # publish odometry via the odom topic
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.linear.y = y_vel
        odom.twist.twist.angular.z = theta_vel
        self.odom_pub.publish(odom)
        
        # publish joint state for the arm
        joint_state = JointState()
        joint_state.header.stamp = current_time
        
        joint_state.name = ['wrist_extension', 'joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0']

        # set positions of the telescoping joints
        positions = [pos_out / 4.0 for i in range(4)]
        # set lift position
        positions.insert(0, pos_up)
        # set wrist_extension position
        positions.insert(0, pos_out)

        # set velocities of the telescoping joints
        velocities = [vel_out / 4.0 for i in range(4)]
        # set lift velocity
        velocities.insert(0, vel_up)
        # set wrist_extension velocity
        velocities.insert(0, vel_out)

        # set efforts of the telescoping joints     # No effort in simulator
        efforts = [eff_out for i in range(4)]
        # set lift effort
        efforts.insert(0, eff_up)
        # set wrist_extension effort
        efforts.insert(0, eff_out)

        if self.use_robotis_head:
            head_joint_names = ['joint_head_pan', 'joint_head_tilt']
            joint_state.name.extend(head_joint_names)

            positions.append(head_pan_rad)
            velocities.append(head_pan_vel)
            efforts.append(head_pan_effort)

            positions.append(head_tilt_rad)
            velocities.append(head_tilt_vel)
            efforts.append(head_tilt_effort)

        if self.use_robotis_end_of_arm:
            if dex_wrist_attached:
                end_of_arm_joint_names = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
                if 'gripper' in config.actuator_names:
                    end_of_arm_joint_names = end_of_arm_joint_names + ['joint_gripper_finger_left', 'joint_gripper_finger_right']
            else:
                if 'gripper' in config.actuator_names:
                    end_of_arm_joint_names = ['joint_wrist_yaw', 'joint_gripper_finger_left', 'joint_gripper_finger_right']
            
            joint_state.name.extend(end_of_arm_joint_names)

            positions.append(wrist_yaw_rad)
            velocities.append(wrist_yaw_vel)
            efforts.append(wrist_yaw_effort)

            if dex_wrist_attached:
                positions.append(wrist_pitch_rad)
                velocities.append(wrist_pitch_vel)
                efforts.append(wrist_pitch_effort)

                positions.append(wrist_roll_rad)
                velocities.append(wrist_roll_vel)
                efforts.append(wrist_roll_effort)
            if 'gripper' in config.actuator_names:
                positions.append(gripper_finger_rad)
                velocities.append(gripper_finger_vel)
                efforts.append(gripper_finger_effort)
                positions.append(gripper_finger_rad)
                velocities.append(gripper_finger_vel)
                efforts.append(gripper_finger_effort)

        # set joint_state
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self.joint_state_pub.publish(joint_state)
        
        current_clock2 = self.get_clock().now()  # ros time
        self.get_logger().info(f"Time taken for command_mobile_base_velocity_and_publish_state: {current_clock2 - current_clock}")
        
        # # TODO: pull camera data and publish
        # camera_data = self.robot_sim.pull_camera_data()
        # # camera_data has cam_d405_rgb, cam_d405_depth, cam_d435i_rgb, cam_d435i_depth, cam_nav_rgb

        # for cam_name, cam_publisher in self.camera_pub.items():
        #     if cam_name in camera_data.keys():
        #         img = camera_data[cam_name]

        #         # Convert depth images (assumed to be grayscale)
        #         if "depth" in cam_name:
        #             img_msg = self.bridge.cv2_to_imgmsg(img, encoding="32FC1")  # Float32 depth
        #         else:
        #             img_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), encoding="bgr8")

        #         cam_publisher.publish(img_msg)
        #         # self.get_logger().info(f"Published {cam_name}")
        
        self.robot_mode_rwlock.release_read()

    
    def stop_the_robot_callback(self, request, response):
        with self.robot_stop_lock:
            for allowed_position_actuator in config.allowed_position_actuators:
                self.robot_sim.move_to(allowed_position_actuator, 0.0)

        self.get_logger().info('Received stop_the_robot service call, so commanded all actuators to stop.')
        response.success = True
        response.message = 'Stopped the robot.'
        return response
    
    def home_the_robot_callback(self, request, response):
        self.get_logger().info('Received home_the_robot service call.')
        self.robot_sim.home()
        response.success = True
        response.message = 'Homed the robot.'
        return response
    
    def stow_the_robot_callback(self, request, response):
        self.get_logger().info('Received stow_the_robot service call.')
        success, message = self.robot_sim.stow()
        response.success = True
        response.message = 'Stowed the robot.'
        return response
    
    def runstop_the_robot(self, runstopped, just_change_mode=False):
        if runstopped:
            self.robot_sim.stop()
    
    def ros_setup(self):
        self.node_name = self.get_name()
        self.bridge = CvBridge()

        self.declare_parameter('broadcast_odom_tf', False)
        self.broadcast_odom_tf = self.get_parameter('broadcast_odom_tf').value
        self.get_logger().info('broadcast_odom_tf = ' + str(self.broadcast_odom_tf))
        if self.broadcast_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            
        self.linear_velocity_mps = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        self.angular_velocity_radps = 0.0  # rad/s ROS SI standard for cmd_vel (REP 103)

        self.max_arm_height = 1.1
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        self.main_group = ReentrantCallbackGroup()
        self.mutex_group = MutuallyExclusiveCallbackGroup() # only one callback can be executing
        self.create_subscription(Twist, "cmd_vel", self.set_mobile_base_velocity_callback, 1, callback_group=self.main_group)
        
        # self.create_subscription(Float64MultiArray, "joint_pose_cmd", self.set_robot_streaming_position_callback, 1, callback_group=self.main_group)
        
        self.declare_parameter('rate', 30.0)
        self.joint_state_rate = self.get_parameter('rate').value
        self.declare_parameter('timeout', 0.5, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Timeout (sec) after which Twist/Joy commands are considered stale',
        ))
        self.timeout_s = self.get_parameter('timeout').value
        self.timeout = Duration(seconds=self.timeout_s)
        # self.declare_parameter('default_goal_timeout_s', 10.0, ParameterDescriptor(
        #     type=ParameterType.PARAMETER_DOUBLE,
        #     description='Default timeout (sec) for goal execution',
        # ))
        # self.default_goal_timeout_s = self.get_parameter('default_goal_timeout_s').value
        # self.default_goal_timeout_duration = Duration(seconds=self.default_goal_timeout_s)
        self.get_logger().info(f"rate = {self.joint_state_rate} Hz")
        self.get_logger().info(f"twist timeout = {self.timeout_s} s")
        
        self.base_frame_id = 'base_link'
        self.get_logger().info(f"base_frame_id = {self.base_frame_id}")
        self.odom_frame_id = 'odom'
        self.get_logger().info(f"odom_frame_id = {self.odom_frame_id}")
        
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        # self.joint_limits_pub = self.create_publisher(JointState, 'joint_limits', 1)
        
        self.last_twist_time = self.get_clock().now()
        
        # start action server for joint trajectories
        # self.declare_parameter('fail_out_of_range_goal', False)
        # self.fail_out_of_range_goal = self.get_parameter('fail_out_of_range_goal').value
        
        # self.declare_parameter('action_server_rate', 30.0)
        # self.action_server_rate = self.get_parameter('action_server_rate').value
        
        
        self.stop_the_robot_service = self.create_service(Trigger,
                                                          '/stop_the_robot',
                                                          self.stop_the_robot_callback,
                                                          callback_group=self.main_group)

        self.home_the_robot_service = self.create_service(Trigger,
                                                          '/home_the_robot',
                                                          self.home_the_robot_callback,
                                                          callback_group=self.main_group)

        self.stow_the_robot_service = self.create_service(Trigger,
                                                           '/stow_the_robot',
                                                           self.stow_the_robot_callback,
                                                           callback_group=self.main_group)

        # self.runstop_service = self.create_service(SetBool,
        #                                            '/runstop',
        #                                            self.runstop_service_callback,
        #                                            callback_group=self.main_group)

        # self.get_joint_states = self.create_service(Trigger,
        #                                             '/get_joint_states',
        #                                             self.get_joint_states_callback,
        #                                             callback_group=self.main_group)
        
        # start loop to command the mobile base velocity, publish
        # odometry, and publish joint states
        
        self.camera_pub = {
            "cam_d405_rgb": self.create_publisher(Image, "/camera/d405/color", 10),
            "cam_d405_depth": self.create_publisher(Image, "/camera/d405/depth", 10),
            "cam_d435i_rgb": self.create_publisher(Image, "/camera/d435i/color", 10),
            "cam_d435i_depth": self.create_publisher(Image, "/camera/d435i/depth", 10),
            "cam_nav_rgb": self.create_publisher(Image, "/camera/nav/color", 10),
        }
        
        timer_period = 1.0 / self.joint_state_rate
        self.timer = self.create_timer(timer_period, self.command_mobile_base_velocity_and_publish_state, callback_group=self.mutex_group)
        
        

def main():
    # multi thread version but fail due to opengl thread safety
    # try:
    #     rclpy.init()
    #     executor = MultiThreadedExecutor(num_threads=8)
    #     node = StretchSimDriver()
        
    #     executor.add_node(node)
    #     try:
    #         executor.spin()
    #     finally:
    #         executor.shutdown()
    #         node.robot_sim.stop()
    #         node.destroy_node()
    # except (KeyboardInterrupt):
    #     rclpy.shutdown()
    
    # multi thread but spin_once in while loop

    # os.environ["MUJOCO_GL"] = "GLFW"

    robot_sim = StretchMujocoSimulator(default_scene_xml_path)
    robot_sim.start()
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=8)
        node = StretchSimDriver(
            robot_sim=robot_sim,
            )
        executor.add_node(node)
        
        try:
            # rate = node.create_rate(1000)
            node.get_logger().info('Started ros manual spin loop at given rate')
            
            while rclpy.ok():
                t1 = time.time()
                
                camera_data = robot_sim.pull_camera_data()
                t2 = time.time()
                node.get_logger().info(f"Time taken for camera rendering: {t2 - t1}")
                # camera_data has cam_d405_rgb, cam_d405_depth, cam_d435i_rgb, cam_d435i_depth, cam_nav_rgb
        
                for cam_name, cam_publisher in node.camera_pub.items():
                    if cam_name in camera_data.keys():
                        img = camera_data[cam_name]
                        # Convert depth images (assumed to be grayscale)
                        if "depth" in cam_name:
                            img_msg = node.bridge.cv2_to_imgmsg(img, encoding="32FC1")  # Float32 depth
                        else:
                            img_msg = node.bridge.cv2_to_imgmsg(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), encoding="bgr8")
                        cam_publisher.publish(img_msg)
                        node.get_logger().info(f"Published {cam_name}")
                        
                # t2 = time.time()
                # node.get_logger().info(f"Time taken for camera publishing: {t2 - t1}")
                
                executor.spin_once()
                t3 = time.time()
                node.get_logger().info(f"Time taken for executor spin_once: {t3 - t1}")
                node.get_logger().info(f"Joint State publishing period: {1.0 / node.joint_state_rate}")
                node.get_logger().info(f"####################################")
                time.sleep(1/1000)
    
        except KeyboardInterrupt:
            print("####################################")
            print("\nShutting down gracefully...")
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()

    except KeyboardInterrupt:
        print("####################################")
        print("\nShutting down...")    
        rclpy.shutdown()
        
    # single thread version
    # try:
    #     rclpy.init()
    #     node = StretchSimDriver()
    #     try:
    #         rclpy.spin(node)
    #     finally:
    #         # node.robot_sim.stop()
    #         node.destroy_node()
    # except (KeyboardInterrupt):
    #     rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()