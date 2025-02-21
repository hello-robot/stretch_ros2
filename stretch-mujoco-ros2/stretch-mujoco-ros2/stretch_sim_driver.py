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

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy
from std_msgs.msg import Bool, String, Float64MultiArray

from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy
from std_msgs.msg import Bool, String, Float64MultiArray

from hello_helpers.gripper_conversion import GripperConversion
from hello_helpers.joint_qpos_conversion import get_Idx, UnsupportedToolError
from hello_helpers.hello_misc import LoopTimer
from hello_helpers.gamepad_conversion import unpack_joy_to_gamepad_state, unpack_gamepad_state_to_joy, get_default_joy_msg

from ament_index_python.packages import get_package_share_path


# stretch_mujoco
from stretch_mujoco import StretchMujocoSimulator
import stretch_mujoco.config as config
import stretch_mujoco.utils as utils



class StretchSimDriver(Node):
    def __init__(self, scene_xml_path: str = './scene.xml'):
        super().__init__('stretch_sim_driver')
        self.use_robotis_head = True
        self.use_robotis_end_of_arm = True
        
        # TODO: initialization
        # hardcoding robo model stretch_urdf/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf
        self.robot_sim = StretchMujocoSimulator(scene_xml_path)
        self.robot_sim.start()
        
        self.robot_mode_rwlock = RWLock()
        
        self.ros_setup()
    
    
    def command_mobile_base_velocity_and_publish_state(self):
        self.robot_mode_rwlock.acquire_read()
        

        current_clock = self.get_clock().now()  # ros time
        current_time = current_clock.to_msg()

        
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
        y_vel = base_status['y_vel']
        theta_vel = base_status['theta_vel']
        
        # assign relevant arm status to variables
        arm_status = robot_status['arm']
        pos_out = arm_status['pos'] # + arm_backlash_correction
        vel_out = arm_status['vel']
        # eff_out = arm_status['motor']['effort_pct']
        
        lift_status = robot_status['lift']
        pos_up = lift_status['pos']
        vel_up = lift_status['vel']
        # eff_up = lift_status['motor']['effort_pct']
        
        if self.use_robotis_end_of_arm:
            # end_of_arm collection
            robot_status['end_of_arm'] = dict(
                wrist_yaw = robot_status["wrist_yaw"],
                wrist_pitch = robot_status["wrist_pitch"],
                wrist_roll = robot_status["wrist_roll"],
                stretch_gripper = robot_status["gripper"],
            )
            # assign relevant wrist status to variables
            wrist_yaw_status = robot_status['end_of_arm']['wrist_yaw']
            wrist_yaw_rad = wrist_yaw_status['pos']
            wrist_yaw_vel = wrist_yaw_status['vel']
            # wrist_yaw_effort = wrist_yaw_status['effort']

            dex_wrist_attached = False
            if 'wrist_pitch' in robot_status['end_of_arm']:
                dex_wrist_attached = True
                
            if dex_wrist_attached:
                wrist_pitch_status = robot_status['end_of_arm']['wrist_pitch']
                wrist_pitch_rad = wrist_pitch_status['pos']
                wrist_pitch_vel = wrist_pitch_status['vel']
                # wrist_pitch_effort = wrist_pitch_status['effort']
                
                wrist_roll_status = robot_status['end_of_arm']['wrist_roll']
                wrist_roll_rad = wrist_roll_status['pos']
                wrist_roll_vel = wrist_roll_status['vel']
                # wrist_roll_effort = wrist_roll_status['effort']
            
            gripper_status = robot_status['end_of_arm']['stretch_gripper']  # NOTE: gripper only has pos and vel
            gripper_status['effort'] = 0    # we assume gripper effort is 0
            gripper_aperture_m, gripper_finger_rad, gripper_finger_effort, gripper_finger_vel = \
                        self.gripper_conversion.status_to_all(gripper_status)     # no gripper effort data
    
        if self.use_robotis_head:
            # head collection
            robot_status['head'] = dict(
                head_pan = robot_status["head_pan"],
                head_tilt = robot_status["head_tilt"],
            )
            # assign relevant head pan status to variables
            head_pan_status = robot_status['head']['head_pan']
            head_pan_rad = head_pan_status['pos'] # + self.head_pan_calibrated_offset_rad + pan_backlash_correction
            head_pan_vel = head_pan_status['vel']
            # head_pan_effort = head_pan_status['effort']
            
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

        # set efforts of the telescoping joints
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
                if 'stretch_gripper' in self.robot.end_of_arm.joints:
                    end_of_arm_joint_names = end_of_arm_joint_names + ['joint_gripper_finger_left', 'joint_gripper_finger_right']
            else:
                if 'stretch_gripper' in self.robot.end_of_arm.joints:
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
            if 'stretch_gripper' in self.robot.end_of_arm.joints:
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
        
        
    def ros_setup(self):
        self.node_name = self.get_name()
        
        self.declare_parameter('broadcast_odom_tf', False)
        self.broadcast_odom_tf = self.get_parameter('broadcast_odom_tf').value
        self.get_logger().info('broadcast_odom_tf = ' + str(self.broadcast_odom_tf))
        if self.broadcast_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            
        self.linear_velocity_mps = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        self.angular_velocity_radps = 0.0  # rad/s ROS SI standard for cmd_vel (REP 103)

        self.max_arm_height = 1.1

        self.mutex_group = MutuallyExclusiveCallbackGroup() # only one callback can be executing
        self.create_subscription(Twist, "cmd_vel", self.set_mobile_base_velocity_callback, 1, callback_group=self.main_group)
        
        self.create_subscription(Float64MultiArray, "joint_pose_cmd", self.set_robot_streaming_position_callback, 1, callback_group=self.main_group)
        
        self.declare_parameter('rate', 30.0)
        self.joint_state_rate = self.get_parameter('rate').value
        self.declare_parameter('timeout', 0.5, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Timeout (sec) after which Twist/Joy commands are considered stale',
        ))
        self.timeout_s = self.get_parameter('timeout').value
        self.timeout = Duration(seconds=self.timeout_s)
        self.declare_parameter('default_goal_timeout_s', 10.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for goal execution',
        ))
        self.default_goal_timeout_s = self.get_parameter('default_goal_timeout_s').value
        self.default_goal_timeout_duration = Duration(seconds=self.default_goal_timeout_s)
        self.get_logger().info(f"rate = {self.joint_state_rate} Hz")
        self.get_logger().info(f"twist timeout = {self.timeout_s} s")
        
        self.base_frame_id = 'base_link'
        self.get_logger().info(f"base_frame_id = {self.base_frame_id}")
        self.odom_frame_id = 'odom'
        self.get_logger().info(f"odom_frame_id = {self.odom_frame_id}")
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.joint_limits_pub = self.create_publisher(JointState, 'joint_limits', 1)
        
        self.last_twist_time = self.get_clock().now()
        
        # start action server for joint trajectories
        self.declare_parameter('fail_out_of_range_goal', False)
        self.fail_out_of_range_goal = self.get_parameter('fail_out_of_range_goal').value
        
        self.declare_parameter('action_server_rate', 30.0)
        self.action_server_rate = self.get_parameter('action_server_rate').value
        
        
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

        self.runstop_service = self.create_service(SetBool,
                                                   '/runstop',
                                                   self.runstop_service_callback,
                                                   callback_group=self.main_group)

        self.get_joint_states = self.create_service(Trigger,
                                                    '/get_joint_states',
                                                    self.get_joint_states_callback,
                                                    callback_group=self.main_group)

        self.self_collision_avoidance = self.create_service(SetBool,
                                                            '/self_collision_avoidance',
                                                            self.self_collision_avoidance_callback,
                                                            callback_group=self.main_group)
        
        # start loop to command the mobile base velocity, publish
        # odometry, and publish joint states
        timer_period = 1.0 / self.joint_state_rate
        self.timer = self.create_timer(timer_period, self.command_mobile_base_velocity_and_publish_state, callback_group=self.mutex_group)
        

def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=8)
        node = StretchSimDriver()
        
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.robot_sim.stop()
            node.destroy_node()
    except (KeyboardInterrupt):
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()