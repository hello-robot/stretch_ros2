#!/usr/bin/env python3

import math
import time
from typing import Optional
from pprint import pformat

import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Transform
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
        self.HACKY_GRASP_CENTER_ALIGN_Y_OFFSET = -0.05

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

    def check_move_to_pose(self, positions: dict[str, tuple[float, float]]) -> None:
        action_response = self.move_to_pose(
            positions,
            custom_contact_thresholds=True,
        )

        returncode = self.check_follow_joint_trajectory_response(action_response)
        if returncode:
            time.sleep(1)
            T_bl_ee = self.get_latest_tf("base_link", "link_grasp_center")
            assert T_bl_ee is not None
            tx = T_bl_ee.translation
            logger.success(f"{tx.x:.6f},{tx.y:.6f},{tx.z:.6f}")


def main() -> None:
    try:
        node = GraspCommand()

        node.check_move_to_pose(
            {
                "joint_wrist_pitch": (0.0, node.CUSTOM_JOINT_EFFORT),
                "joint_wrist_roll": (0.0, node.CUSTOM_JOINT_EFFORT),
                "gripper_aperture": (0.00, node.CUSTOM_JOINT_EFFORT),
                #
                "joint_lift": (0.75, node.CUSTOM_JOINT_EFFORT),
                "wrist_extension": (0.25, node.CUSTOM_JOINT_EFFORT),
                "joint_wrist_yaw": (4.0, node.CUSTOM_JOINT_EFFORT),
            }
        )

        node.new_thread.join()
        node.destroy_node()
    except Exception as e:
        node.get_logger().info(e)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
