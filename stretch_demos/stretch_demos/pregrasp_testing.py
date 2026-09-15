#!/usr/bin/env python3

import rclpy
import rclpy.logging

import hello_helpers.hello_misc as hm
from stretch_demos.grasp_object_visual_servo import (
    PREGRASP_FIXED_WRIST_YAW_RAD,
    PREGRASP_FIXED_WRIST_PITCH_RAD,
    PREGRASP_FIXED_WRIST_ROLL_RAD,
    PREGRASP_GRIPPER_OPEN_APERTURE_M,
)

# Captured from /stretch/joint_states after a real pregrasp run -- re-capture
# these two if the test object's placement (height/depth) changes.
PREGRASP_TEST_LIFT_M = 0.848
PREGRASP_TEST_WRIST_EXTENSION_M = 0.287


class PregraspTestingNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'pregrasp_testing', 'pregrasp_testing', wait_for_first_pointcloud=False)
        self.logger = self.get_logger()

        self.logger.info('Moving lift to {0:.3f} m.'.format(PREGRASP_TEST_LIFT_M))
        self.move_to_pose({'joint_lift': PREGRASP_TEST_LIFT_M})

        self.logger.info('Extending wrist to {0:.3f} m.'.format(PREGRASP_TEST_WRIST_EXTENSION_M))
        self.move_to_pose({'wrist_extension': PREGRASP_TEST_WRIST_EXTENSION_M})

        self.logger.info('Setting wrist yaw/pitch/roll to {0:.3f}/{1:.3f}/{2:.3f} rad.'.format(
            PREGRASP_FIXED_WRIST_YAW_RAD, PREGRASP_FIXED_WRIST_PITCH_RAD, PREGRASP_FIXED_WRIST_ROLL_RAD))
        self.move_to_pose({'joint_wrist_yaw': PREGRASP_FIXED_WRIST_YAW_RAD,
                            'joint_wrist_pitch': PREGRASP_FIXED_WRIST_PITCH_RAD,
                            'joint_wrist_roll': PREGRASP_FIXED_WRIST_ROLL_RAD})

        self.logger.info('Opening the gripper to {0:.3f} m.'.format(PREGRASP_GRIPPER_OPEN_APERTURE_M))
        self.move_to_pose({'gripper_aperture': PREGRASP_GRIPPER_OPEN_APERTURE_M})

        self.logger.info(
            'Pregrasp-testing pose reached. Place the graspable object between the fingers, '
            'then launch the visual servoing node.')


def main():
    try:
        node = PregraspTestingNode()
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('pregrasp_testing').info('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
