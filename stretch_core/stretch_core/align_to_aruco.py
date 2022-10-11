#!/usr/bin/env python3

# This node uses the tf_listener tutorial from Stretch Tutorials
# Head should always point at ArUco marker even while moving and aligning

import sys
import time
from math import atan2, sqrt

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_matrix)
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


class FrameListener(Node):

    def __init__(self):
        super().__init__('align_to_aruco')

        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 0.2 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def get_transforms(self):
        return self.trans_base, self.trans_camera

    def on_timer(self):
        marker_frame_rel = 'base_right' # 'aruco_tag'
        base_frame_rel = 'base_link'
        camera_frame_rel = 'camera_link'

        try:
            now = Time()
            self.trans_base = self.tf_buffer.lookup_transform(
                base_frame_rel,
                marker_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {marker_frame_rel} to {base_frame_rel}: {ex}')
            return

        try:
            now = Time()
            self.trans_camera = self.tf_buffer.lookup_transform(
                camera_frame_rel,
                marker_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {marker_frame_rel} to {base_frame_rel}: {ex}')
            return

# Align x-axis of base_link to x-axis of marker
# Robot should first align x-axes then minimize distance along x-axis and then along y-axis
class AlignToAruco(FrameListener):
    def __init__(self, node, offset=0.75):
        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()
        self.joint_state = JointState()
        self.offset = offset
        self.node = node

        self.trajectory_client = ActionClient(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.node.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def compute_difference(self):
        self.trans_base, self.trans_camera = self.node.get_transforms()

        x = self.trans_base.transform.rotation.x
        y = self.trans_base.transform.rotation.y
        z = self.trans_base.transform.rotation.z
        w = self.trans_base.transform.rotation.w

        ############## changes for offset ###############
        R = quaternion_matrix((x, y, z, w))
        P_dash = np.array([[0], [-self.offset], [0], [1]])
        P = np.array([[self.trans_base.transform.translation.x], [self.trans_base.transform.translation.y], [0], [1]])

        X = np.matmul(R, P_dash)
        P_base = X + P

        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]
        #################################################

        # base_position_x = self.trans_base.transform.translation.x
        # base_position_y = self.trans_base.transform.translation.y # - self.offset

        phi = atan2(base_position_y, base_position_x)
        self.node.get_logger().info("Angle phi is: {}".format(phi))
        dist = sqrt(pow(base_position_x, 2) + pow(base_position_y, 2))
        self.node.get_logger().info("Distance x is: {}".format(dist))

        x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x, y, z, w])
        z_rot_base = -phi + z_rot_base + 3.14159
        self.node.get_logger().info("Angle z_rot_base is: {}".format(z_rot_base))
        # camera_position_x = self.trans_base.transform.translation.x
        # camera_position_y = self.trans_base.transform.translation.y
        # camera_position_z = self.trans_base.transform.translation.z

        return phi, dist, z_rot_base

    def align_to_marker(self):
        # Turn phi
        # Travel dist
        # -Turn phi + x_rot_base + 3.14159

        phi, dist, z_rot_base = self.compute_difference()

        x, y, z, w = quaternion_from_euler(0, 0, phi)

        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=10.0)

        # for base joints
        point1 = MultiDOFJointTrajectoryPoint()
        point2 = MultiDOFJointTrajectoryPoint()
        point1.time_from_start = duration1.to_msg()
        point2.time_from_start = duration2.to_msg()
        transform1 = Transform()
        transform2 = Transform()

        trajectory_goal = FollowJointTrajectory.Goal()
        # trajectory_goal.goal_time_tolerance = rclpy.time.Time()
        
        joint_state = self.joint_state
        if (joint_state is not None):

            transform1.translation.x = 0.0
            transform1.rotation.w = 1.0
            transform2.translation.x = 0.0
            transform2.rotation.x = x
            transform2.rotation.y = y
            transform2.rotation.z = z
            transform2.rotation.w = w
            point1.transforms = [transform1]
            point2.transforms = [transform2]
            # joint_name should be 'position' and not one generated by command i.e. 'translate/rotate_mobile_base'
            joint_name = 'position'
            trajectory_goal.multi_dof_trajectory.joint_names = [joint_name]
            trajectory_goal.multi_dof_trajectory.points = [point1, point2]
            trajectory_goal.multi_dof_trajectory.header.stamp = self.node.get_clock().now().to_msg()
            self.trajectory_client.send_goal_async(trajectory_goal)
            self.node.get_logger().info("Executing first goal")
            time.sleep(15)

        joint_state = self.joint_state
        if (joint_state is not None):
            transform1.translation.x = 0.0
            transform1.rotation.w = 1.0
            transform2.translation.x = dist
            transform2.rotation.x = 0.0
            transform2.rotation.y = 0.0
            transform2.rotation.z = 0.0
            transform2.rotation.w = 1.0
            point1.transforms = [transform1]
            point2.transforms = [transform2]
            # joint_name should be 'position' and not one generated by command i.e. 'translate/rotate_mobile_base'
            joint_name = 'position'
            trajectory_goal.multi_dof_trajectory.joint_names = [joint_name]
            trajectory_goal.multi_dof_trajectory.points = [point1, point2]
            trajectory_goal.multi_dof_trajectory.header.stamp = self.node.get_clock().now().to_msg()
            self.trajectory_client.send_goal_async(trajectory_goal)
            self.node.get_logger().info("Executing second goal")
            time.sleep(15)

        joint_state = self.joint_state
        x, y, z, w = quaternion_from_euler(0, 0, z_rot_base)
        if (joint_state is not None):
            transform1.translation.x = 0.0
            transform1.rotation.w = 1.0
            transform2.translation.x = 0.0
            transform2.rotation.x = x
            transform2.rotation.y = y
            transform2.rotation.z = z
            transform2.rotation.w = w
            point1.transforms = [transform1]
            point2.transforms = [transform2]
            # joint_name should be 'position' and not one generated by command i.e. 'translate/rotate_mobile_base'
            joint_name = 'position'
            trajectory_goal.multi_dof_trajectory.joint_names = [joint_name]
            trajectory_goal.multi_dof_trajectory.points = [point1, point2]
            trajectory_goal.multi_dof_trajectory.header.stamp = self.node.get_clock().now().to_msg()
            self.trajectory_client.send_goal_async(trajectory_goal)
            self.node.get_logger().info("Executing third goal")


def main():
    time.sleep(20) # Allows time for realsense camera to boot up before this node becomes active
    rclpy.init()
    node = FrameListener()
    
    for i in range(10):
        rclpy.spin_once(node)

    align = AlignToAruco(node, offset=0.75)
    align.align_to_marker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
