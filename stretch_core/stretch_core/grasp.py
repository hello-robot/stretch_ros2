#!/usr/bin/env python3

import math
import time
from typing import Optional
from pprint import pprint


import rclpy
import rclpy.time
from rclpy.duration import Duration
from hello_helpers.hello_misc import HelloNode
from geometry_msgs.msg import Twist, PoseStamped, Transform
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class GraspCommand(HelloNode):
    def __init__(self) -> None:
        super().__init__()
        super().main("grasp_command", "grasp_command", wait_for_first_pointcloud=False)

        self.CUSTOM_JOINT_EFFORT: float = 60.0
        self.ARM_MAX_LENGTH = 0.513
        self.LIFT_MAX_HEIGHT = 1.098
        self.GRASP_X_OFFSET = 0.225  # gripper center wrt base
        self.LIFT_Z_OFFSET = 0.185  # difference between base and real lift joint value
        self.ARM_Y_OFFSET = 0.159  # difference between base and real arm joint value
        self.GRIPPER_Z_OFFSET = (
            0.11  # height difference between gripper center and lift
        )

        self.joint_wrist_yaw_in = 3.0  # gripper faces inside
        self.joint_wrist_yaw_forward = 3.141592 / 2  # gripper faces towards the front
        self.joint_wrist_yaw_out = 0.0  # gripper faces towards the human
        self.gripper_open = 0.2
        self.gripper_close = -0.01

    def get_latest_tf(
        self,
        target_frame: str,
        source_frame: str,
    ) -> Optional[Transform]:
        try:
            return self.tf2_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            ).transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TODO: hardcode something if the motion capture is not running

            self.get_logger().error(
                f"Failed to get transform from {source_frame} to {target_frame}: {str(e)}"
            )
            return None

    # Move arm to the desired height without stretching arm
    def lift_arm(self) -> bool:
        if not self.joint_state:
            self.get_logger().info("Waiting for joint states message to arrive")
            return False

        T_base_link__object = self.get_latest_tf("base_link", "Pringles")
        if T_base_link__object is None:
            return False

        target_lift_height = (
            T_base_link__object.translation.z
            - self.LIFT_Z_OFFSET
            + self.GRIPPER_Z_OFFSET * 0
        )

        self.get_logger().info("Moving the lift ...")
        joint_positions_dict: dict[str, tuple[float, float]] = {
            "joint_lift": (target_lift_height, self.CUSTOM_JOINT_EFFORT),
            "wrist_extension": (0.1, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_yaw": (self.joint_wrist_yaw_forward, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_pitch": (0.0, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_roll": (0.0, self.CUSTOM_JOINT_EFFORT),
            "gripper_aperture": (self.gripper_open, self.CUSTOM_JOINT_EFFORT),
        }

        pprint(joint_positions_dict)
        self.move_to_pose(joint_positions_dict, custom_contact_thresholds=True)

        self.get_logger().info("Lift is ready!")
        return True

    # Move arm to the pre-grasping position
    def align_end_effector(self) -> bool:
        HACKY_GRASP_CENTER_Y_OFFSET = -0.05

        if not self.joint_state:
            self.get_logger().info("Waiting for joint states message to arrive")
            return False

        T_link_grasp_center__object = self.get_latest_tf(
            "link_grasp_center", "Pringles"
        )
        if T_link_grasp_center__object is None:
            return False
        T_link_grasp_center__object.translation.y += HACKY_GRASP_CENTER_Y_OFFSET

        T_base_link__link_grasp_center = self.get_latest_tf(
            "base_link", "link_grasp_center"
        )
        if T_base_link__link_grasp_center is None:
            return False

        def magic_function(dx, dy, L, X):
            return 2 * math.atan2(
                X + dx - math.sqrt(-2 * L * dy + X**2 + 2 * X * dx + dx**2 + dy**2),
                2 * L - dy,
            )

        theta_offset = magic_function(
            T_link_grasp_center__object.translation.x,
            T_link_grasp_center__object.translation.y,
            -T_base_link__link_grasp_center.translation.y,
            T_base_link__link_grasp_center.translation.x,
        )

        joint_positions_dict: dict[str, float] = {
            "rotate_mobile_base": theta_offset,  # relative
        }
        # pprint(T_link_grasp_center__object.translation)
        # pprint(joint_positions_dict)

        self.move_to_pose(joint_positions_dict, custom_contact_thresholds=False)

        T_link_grasp_center__object = self.get_latest_tf(
            "link_grasp_center", "Pringles"
        )
        if T_link_grasp_center__object is None:
            return False
        T_link_grasp_center__object.translation.y += HACKY_GRASP_CENTER_Y_OFFSET

        pprint(T_link_grasp_center__object.translation)

        return True

    def move_base_forward(self, distance: float) -> bool:
        if not self.joint_state:
            self.get_logger().info("Waiting for joint states message to arrive")
            return False

        self.get_logger().info(f"move_base_forward: d: {distance}")

        joint_positions_dict: dict[str, float] = {
            "translate_mobile_base": distance
        }
        self.move_to_pose(joint_positions_dict)

        return True

    # close the gripper and lift the object
    def issue_grasp_command(self) -> bool:
        if not self.joint_state:
            self.get_logger().info("Waiting for joint states message to arrive")
            return False

        self.get_logger().info("Grasping ...")
        self.move_to_pose({"gripper_aperture": self.gripper_close})

        current_lift_pos = self.joint_state.position[self.joint_state.name.index("joint_lift")]

        # lift and retract the arm
        joint_positions_dict: dict[str, tuple[float, float]] = {
            "joint_lift": (current_lift_pos + 0.05, self.CUSTOM_JOINT_EFFORT),
            "wrist_extension": (0.0, self.CUSTOM_JOINT_EFFORT),
        }

        self.move_to_pose(joint_positions_dict, custom_contact_thresholds=True)

        return True

    def drive_to_human(self) -> bool:
        T_base_link__human = self.get_latest_tf(
            "base_link", "human"
        )
        if T_base_link__human is None:
            return False

        theta_offset = math.atan2(
            T_base_link__human.translation.y, T_base_link__human.translation.x
        )
        joint_positions_dict: dict[str, float] = {
            "rotate_mobile_base": theta_offset,  # relative
        }
        self.move_to_pose(joint_positions_dict)

        T_base_link__human = self.get_latest_tf(
            "base_link", "human"
        )
        if T_base_link__human is None:
            return False

        distance = max(0.0, T_base_link__human.translation.x - 1.0)
        self.move_base_forward(distance)

        return True

    # lift the arm and open the gripper
    def handover(self) -> bool:
        if not self.joint_state:
            self.get_logger().info("Waiting for joint states message to arrive")
            return False

        self.get_logger().info("Lifting and stretching ...")

        joint_positions_dict: dict[str, float] = {
            "joint_wrist_yaw": self.joint_wrist_yaw_out,
            "rotate_mobile_base": 3.1415 / 2,  # relative
        }
        self.move_to_pose(joint_positions_dict)

        joint_positions_dict: dict[str, tuple[float, float]] = {
            "joint_lift": (0.8, self.CUSTOM_JOINT_EFFORT),
            "wrist_extension": (0.3, self.CUSTOM_JOINT_EFFORT),
        }
        self.move_to_pose(joint_positions_dict, custom_contact_thresholds=True)

        # TODO: open gripper

        self.get_logger().info("Opening the gripper ...")
        time.sleep(2)
        self.move_to_pose({"gripper_aperture": self.gripper_open})

        return True

    def main(self):
        while True:
            if not self.lift_arm():
                self.get_logger().warn("lift_arm failed")
                continue

            if not self.align_end_effector():
                self.get_logger().warn("align_end_effector failed")
                continue

            T_link_grasp_center__object = self.get_latest_tf(
                "link_grasp_center", "Pringles"
            )
            if T_link_grasp_center__object is None:
                continue
            distance = max(0.0, T_link_grasp_center__object.translation.x - 1)

            if distance > 0.02:
                self.move_base_forward(distance)
                continue

            distance = max(0.0, T_link_grasp_center__object.translation.x - 0.12)
            if distance > 0.02:
                self.move_base_forward(distance)
                self.align_end_effector()
                continue

            distance = max(0.0, T_link_grasp_center__object.translation.x - 0.02)
            if distance > 0.2:
                self.get_logger().error(
                    f"WTF: {distance} // {T_link_grasp_center__object.translation}"
                )
                break
            else:
                self.move_base_forward(distance)

            self.issue_grasp_command()
            time.sleep(1)
            self.move_base_forward(-0.3)

            while not self.drive_to_human():
                time.sleep(1)

            self.drive_to_human()

            self.handover()

            break

        # self.move_base_forward()
        # self.issue_grasp_command()
        # self.handover()

        rclpy.shutdown()


def main():
    # # gripper position
    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = "base_link"
    # pose_goal.pose.orientation.x = 0.0
    # pose_goal.pose.orientation.y = 0.0
    # pose_goal.pose.orientation.z = 0.0
    # pose_goal.pose.orientation.w = 1.0
    # pose_goal.pose.position.x = 0.328
    # pose_goal.pose.position.y = -0.305
    # pose_goal.pose.position.z = 0.683

    try:
        node = GraspCommand()
        # node.home_the_robot()

        # node.move_to_pose(
        #     {"joint_lift": (0.2, node.CUSTOM_JOINT_EFFORT)},
        #     custom_contact_thresholds=True,
        # )
        node.main()

        node.new_thread.join()
    except Exception as e:
        node.get_logger().info(e)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
