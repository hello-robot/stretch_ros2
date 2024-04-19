#!/usr/bin/env python3

import math
import time
from typing import Optional
from pprint import pformat

import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Transform, Vector3
from hello_helpers.hello_misc import HelloNode
from control_msgs.action._follow_joint_trajectory import (
    FollowJointTrajectory_GetResult_Response,
    FollowJointTrajectory_Result,
)
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from loguru import logger


class GraspCommand(HelloNode):
    def __init__(self) -> None:
        super().__init__()
        super().main("grasp_command", "grasp_command", wait_for_first_pointcloud=False)

        # Constants
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

            logger.error(
                f"Failed to get transform from {source_frame} to {target_frame}: {str(e)}"
            )
            return None

    def status_dict_reverse_lookup(self, status_dict: dict[str, int], val: int) -> str:
        # It would be more efficient to cache the reversed dictionary, but ignore for now
        for k, v in status_dict.items():
            if v == val:
                return k
        return "Unknown"

    def check_follow_joint_trajectory_response(
        self,
        response: FollowJointTrajectory_GetResult_Response,
    ) -> bool:
        if (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and response.result.error_code == FollowJointTrajectory_Result.SUCCESSFUL
        ):
            return True

        status_str = self.status_dict_reverse_lookup(
            GoalStatus.__prepare__(None, None),
            response.status,
        )
        logger.error(f"GoalStatus: error id {response.status} ('{status_str}')")

        status_str = self.status_dict_reverse_lookup(
            FollowJointTrajectory_Result.__prepare__(None, None),
            response.result.error_code,
        )
        logger.error(
            f"JointTrajectory: error id {response.result.error_code} ('{status_str}'; {response.result.error_string})"
        )

        return False

    def stow(self) -> bool:
        logger.info("Stowing the arm...")

        joint_positions_dict: dict[str, tuple[float, float]] = {
            "joint_lift": (0.25, self.CUSTOM_JOINT_EFFORT),
            "wrist_extension": (0.025, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_yaw": (self.joint_wrist_yaw_in, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_pitch": (0.0, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_roll": (0.0, self.CUSTOM_JOINT_EFFORT),
        }
        action_response = self.move_to_pose(
            joint_positions_dict, custom_contact_thresholds=True
        )
        returncode = self.check_follow_joint_trajectory_response(action_response)
        if returncode:
            logger.success("Stowed the arm")
        return returncode

    # Move arm to the desired height without stretching arm
    def lift_arm(self) -> bool:
        logger.info("Moving the lift...")

        T_base_link__object = self.get_latest_tf("base_link", "Pringles")
        if T_base_link__object is None:
            return False

        target_lift_height = (
            T_base_link__object.translation.z
            - self.LIFT_Z_OFFSET
            + self.GRIPPER_Z_OFFSET * 0
        )

        joint_positions_dict: dict[str, tuple[float, float]] = {
            "joint_lift": (target_lift_height, self.CUSTOM_JOINT_EFFORT),
            "wrist_extension": (0.1, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_yaw": (self.joint_wrist_yaw_forward, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_pitch": (0.0, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_roll": (0.0, self.CUSTOM_JOINT_EFFORT),
            "gripper_aperture": (self.gripper_open, self.CUSTOM_JOINT_EFFORT),
        }
        logger.info("\n" + pformat(joint_positions_dict))
        action_response = self.move_to_pose(
            joint_positions_dict, custom_contact_thresholds=True
        )

        returncode = self.check_follow_joint_trajectory_response(action_response)
        if returncode:
            logger.success("Moved the lift")
        return returncode

    # Move arm to the pre-grasping position
    def align_end_effector(self) -> bool:
        logger.info("Aligning the end effector...")

        T_base_link__link_grasp_center = self.get_latest_tf(
            "base_link", "link_grasp_center"
        )
        if T_base_link__link_grasp_center is None:
            return False

        T_base_link__object = self.get_latest_tf("base_link", "Pringles")
        if T_base_link__object is None:
            return False

        def calculate_base_rotation_angle(
            p_base_link__object: Vector3, p_base_link__link_grasp_center: Vector3
        ) -> float:
            # alias variables to make things more readable
            p_R_obj = p_base_link__object
            p_R_EE = p_base_link__link_grasp_center

            return -2.0 * math.atan2(
                p_R_obj.x - math.sqrt(p_R_obj.x**2 + p_R_obj.y**2 - p_R_EE.y**2),
                3 * p_R_EE.y - p_R_obj.y
            )

        theta_offset = calculate_base_rotation_angle(
            T_base_link__object.translation,
            T_base_link__link_grasp_center.translation
        )
        joint_positions_dict: dict[str, float] = {
            "rotate_mobile_base": theta_offset,  # relative
        }
        logger.info("\n" + pformat(joint_positions_dict))
        action_response = self.move_to_pose(
            joint_positions_dict, custom_contact_thresholds=False
        )

        returncode = self.check_follow_joint_trajectory_response(action_response)
        if returncode:
            logger.success("Aligned the end effector")
        return returncode

    def move_base_forward(self, distance: float) -> bool:
        logger.info(f"Driving the base {distance:.2f} m")

        joint_positions_dict: dict[str, float] = {"translate_mobile_base": distance}
        action_response = self.move_to_pose(joint_positions_dict)

        returncode = self.check_follow_joint_trajectory_response(action_response)
        if returncode:
            logger.success("Drove the base")
        return returncode

    def drive_base_link_to_frame(
        self, target_frame: str, stopping_distance: float
    ) -> bool:
        logger.info(f"Driving base_link to {target_frame}")

        T_base_link__target = self.get_latest_tf("base_link", target_frame)
        if T_base_link__target is None:
            return False

        # Align yaw
        theta_offset = math.atan2(
            T_base_link__target.translation.y, T_base_link__target.translation.x
        )
        joint_positions_dict: dict[str, float] = {
            "rotate_mobile_base": theta_offset,  # relative
        }
        logger.info("\n" + pformat(joint_positions_dict))
        action_response = self.move_to_pose(joint_positions_dict)
        if not self.check_follow_joint_trajectory_response(action_response):
            logger.error("Failed to rotate base")

        T_base_link__target = self.get_latest_tf("base_link", target_frame)
        if T_base_link__target is None:
            return False
        logger.debug("New relative pose\n" + pformat(T_base_link__target.translation))

        # Drive forward
        distance_offset = max(
            0.0, T_base_link__target.translation.x - stopping_distance
        )

        returncode = (distance_offset == 0.0) or self.move_base_forward(distance_offset)
        if returncode:
            logger.success(f"Drove base_link to {target_frame}")
        return returncode

    # close the gripper and lift the object
    def issue_grasp_command(self) -> bool:
        logger.info("Closing the gripper...")

        T_gripper__object = self.get_latest_tf("link_grasp_center", "Pringles")
        if T_gripper__object is not None:
            logger.debug(f"Object pose pre-grasp:\n{T_gripper__object.translation}")

        action_response = self.move_to_pose({"gripper_aperture": self.gripper_close})
        if not self.check_follow_joint_trajectory_response(action_response):
            logger.error("Failed to close the gripper")
            return False

        current_lift_pos = self.joint_state.position[
            self.joint_state.name.index("joint_lift")
        ]

        # lift and retract the arm
        joint_positions_dict: dict[str, tuple[float, float]] = {
            "joint_lift": (current_lift_pos + 0.05, self.CUSTOM_JOINT_EFFORT),
            "wrist_extension": (0.0, self.CUSTOM_JOINT_EFFORT),
            "joint_wrist_yaw": (2.7, self.CUSTOM_JOINT_EFFORT),
        }
        action_response = self.move_to_pose(
            joint_positions_dict, custom_contact_thresholds=True
        )
        returncode = self.check_follow_joint_trajectory_response(action_response)
        if returncode:
            logger.success("Finished grasp command")
        return returncode

    # lift the arm and open the gripper
    def handover(self) -> bool:
        logger.info("Executing handover...")

        joint_positions_dict: dict[str, float] = {
            "rotate_mobile_base": 3.1415 / 2,  # relative
        }
        self.move_to_pose(joint_positions_dict)

        joint_positions_dict: dict[str, tuple[float, float]] = {
            "joint_lift": (0.9, self.CUSTOM_JOINT_EFFORT),
        }
        self.move_to_pose(joint_positions_dict, custom_contact_thresholds=True)

        joint_positions_dict: dict[str, float] = {
            "joint_wrist_yaw": self.joint_wrist_yaw_out,
        }
        self.move_to_pose(joint_positions_dict)

        joint_positions_dict: dict[str, tuple[float, float]] = {
            "wrist_extension": (0.2, self.CUSTOM_JOINT_EFFORT),
        }
        self.move_to_pose(joint_positions_dict, custom_contact_thresholds=True)

        # Open the gripper
        logger.info("Opening the gripper...")
        time.sleep(1)
        self.move_to_pose({"gripper_aperture": self.gripper_open})

        logger.success("Completed handover.")
        return True

    def main(self) -> None:
        # Ensure that the joint states are initialized
        while not self.joint_state:
            logger.info("Waiting for joint state initialization.")
            time.sleep(0.5)

        self.stow()

        while not self.drive_base_link_to_frame("Pringles", 0.5):
            continue

        while True:
            if not self.lift_arm():
                continue

            if not self.align_end_effector():
                continue

            T_link_grasp_center__object = self.get_latest_tf(
                "link_grasp_center", "Pringles"
            )
            if T_link_grasp_center__object is None:
                continue

            # distance = max(0.0, T_link_grasp_center__object.translation.x - 1)
            # if distance > 0.02:
            #     self.move_base_forward(distance)
            #     continue

            distance = max(0.0, T_link_grasp_center__object.translation.x - 0.12)
            if distance > 0.02:
                self.move_base_forward(distance)
                self.align_end_effector()
                continue

            # The gripper shortens in length as it closes, so we need to add a
            # couple of centimeters to the drive distance to compensate.
            distance = max(0.0, T_link_grasp_center__object.translation.x + 0.02)
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

            while not self.drive_base_link_to_frame("human", 1.5):
                time.sleep(1)

            self.handover()
            time.sleep(1)
            self.stow()
            break

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
