#!/usr/bin/env python3

import sys
import time
from math import atan2, sqrt
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException, Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_matrix
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

class AlignToAruco(Node):
    def __init__(
        self, node, trans_base: TransformStamped, offset=0.75
    ):
        self.trans_base = trans_base
        self.offset = offset  # Desired y-offset of base from marker
        self.node = node

        self.trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        if not self.trajectory_client.wait_for_server(timeout_sec=60.0):
            self.node.get_logger().error("Unable to connect to trajectory server.")
            sys.exit()

    def compute_difference(self):
        # Extract quaternion and rotation matrix of marker in base_link frame
        x, y, z, w = (
            self.trans_base.transform.rotation.x,
            self.trans_base.transform.rotation.y,
            self.trans_base.transform.rotation.z,
            self.trans_base.transform.rotation.w,
        )
        R = quaternion_matrix((x, y, z, w))

        # Apply rotation to the offset vector
        P_dash = np.array([[0], [-self.offset], [0], [1]])
        P = np.array(
            [
                [self.trans_base.transform.translation.x],
                [self.trans_base.transform.translation.y],
                [0],
                [1],
            ]
        )
        X = np.matmul(R, P_dash)

        # Compute the marker position with offset in base_link frame
        P_base = X + P
        P_base[3, 0] = 1  # Homogeneous coordinate

        # Extract adjusted position
        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]

        # Compute rotation and translation needed
        phi = atan2(base_position_y, base_position_x)
        dist = sqrt(base_position_x**2 + base_position_y**2)

        _, _, z_rot_base = euler_from_quaternion([x, y, z, w])
        # Calculate final rotation: -phi (cancel rotation needed to align),
        # + z_rot_base (original marker rotation),
        # + pi (such that the base and the marker axis are aligned as shown in tutorial)
        z_rot_base = -phi + z_rot_base + np.pi

        return phi, dist, z_rot_base

    def align_to_marker(self):
        phi, dist, final_theta = self.compute_difference()

        def send_base_goal_blocking(joint_name, inc):
            point = JointTrajectoryPoint()
            point.positions = [inc]
            point.time_from_start = Duration(seconds=5.0).to_msg()

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [joint_name]
            goal.trajectory.points = [point]

            self.node.get_logger().info(f"[{joint_name}] Sending goal: {inc:.3f}")
            send_goal_future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.node.get_logger().error(f"Goal for {joint_name} was rejected!")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            result = result_future.result()

            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().warn(
                    f"Goal for {joint_name} did not succeed: status {result.status}"
                )
            else:
                self.node.get_logger().info(f"Goal for {joint_name} succeeded.")

        send_base_goal_blocking("rotate_mobile_base", phi)
        send_base_goal_blocking("translate_mobile_base", dist)
        send_base_goal_blocking("rotate_mobile_base", final_theta)


def main():
    rclpy.init()
    node = Node("align_to_aruco_node")

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    timeout_sec = 120.0
    start_time = time.time()

    node.get_logger().info("Waiting for required transforms...")
    trans_base = None

    # Get the ArUco tag name from the launch file parameter (default: "base_right")
    node.declare_parameter("aruco_tag_name", "base_right")
    aruco_tag_name_in_stretch_marker_dict = node.get_parameter("aruco_tag_name").get_parameter_value().string_value

    missing = []

    while rclpy.ok():
        # Block while searching for the marker:
        try:
            if tf_buffer.can_transform(
                "base_link", aruco_tag_name_in_stretch_marker_dict, Time()
            ):
                trans_base = tf_buffer.lookup_transform(
                    "base_link", aruco_tag_name_in_stretch_marker_dict, Time()
                )
                node.get_logger().info("Found the ArUco tag!")
                node.get_logger().warning("WARNING: The robot will now move!")
                break
            else:
                missing.append(f"base_link → {aruco_tag_name_in_stretch_marker_dict}")
        except TransformException as ex:
            node.get_logger().warn(f"Error during transform lookup: {ex}")
            missing.append("exception")
            
        node.get_logger().error(
                "Could not detect the ArUco marker. "
                "Please make sure the camera is manually pointed at the marker. "
                "Please also make sure that the marker is upright - an incorrect orientation "
                "will result in an incorrect movement. "
                "The robot will move when the marker is found."
            )

        elapsed = time.time() - start_time
        if elapsed > timeout_sec:
            break

        rclpy.spin_once(node)
        time.sleep(1)

    if trans_base is None:
        node.get_logger().error(
            f"Timeout waiting for transforms: {', '.join(missing)}. Exitting."
        )
        rclpy.shutdown()
        return

    try:
        # Create aligner and run
        align = AlignToAruco(node=node, trans_base=trans_base)
        align.align_to_marker()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
