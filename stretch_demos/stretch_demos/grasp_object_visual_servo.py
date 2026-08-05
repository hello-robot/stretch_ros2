#!/usr/bin/env python3

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.logging

from std_srvs.srv import Trigger

import hello_helpers.hello_misc as hm
import stretch_funmap.manipulation_planning as mp

# Increment 1 of the incremental build-out described in
# grasp-demo-status.md / the approved implementation plan: Stage 1
# targeting only (reuses the existing FUNMAP scan-and-select pipeline
# unchanged, exactly as grasp_object.py already does), converted to a
# real 3D point and logged. No D405, no velocity mode, no motion beyond
# the stow + head scan yet -- those arrive in later increments.


class GraspObjectVisualServoNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.debug_directory = None
        self.manipulation_view = None

    def find_grasp_target(self):
        """Stage 1: one-shot FUNMAP scan + surface/largest-blob object
        selection (stretch_funmap.manipulation_planning.ManipulationView,
        unmodified). Returns (grasp_target, object_xyz_base_link), or
        (None, None) if no object was found.
        """
        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory, self.get_tool())
        manip = self.manipulation_view
        manip.move_head(self.move_to_pose)

        scan_time_s = 4.0
        start_time_s = time.time()
        while (time.time() - start_time_s) < scan_time_s:
            manip.update(self.point_cloud, self.tf2_buffer)

        grasp_target = manip.get_grasp_target(self.tf2_buffer)
        if grasp_target is None:
            return None, None

        h = manip.max_height_im
        xyz_pix = [grasp_target['location_xy_pix'][0],
                   grasp_target['location_xy_pix'][1],
                   grasp_target['location_z_pix']]
        object_xyz_base_link = h.get_pix_in_frame(xyz_pix, 'base_link', self.tf2_buffer)

        return grasp_target, object_xyz_base_link

    def trigger_grasp_object_visual_servo_callback(self, request, response):
        self.logger.info('Stow the arm.')
        self.stow_the_robot()

        self.logger.info('Stage 1: scanning for a grasp target.')
        grasp_target, object_xyz_base_link = self.find_grasp_target()
        if grasp_target is None:
            return Trigger.Response(
                success=False,
                message='Stage 1 failed to find a grasp target'
            )

        self.logger.info(
            'Stage 1 object location (base_link): x={0:.3f}, y={1:.3f}, z={2:.3f} m'.format(
                object_xyz_base_link[0], object_xyz_base_link[1], object_xyz_base_link[2]))
        self.logger.info('Stage 1 object width_m = {0:.3f} m'.format(grasp_target['width_m']))

        return Trigger.Response(
            success=True,
            message='Stage 1 found a grasp target (Increment 1: no further motion yet).'
        )

    def main(self):
        hm.HelloNode.main(self, 'grasp_object_visual_servo', 'grasp_object_visual_servo', wait_for_first_pointcloud=False)
        self.logger = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('debug_directory', '')
        self.debug_directory = self.get_parameter('debug_directory').value or None
        self.logger.info('Using the following directory for debugging files: {0}'.format(self.debug_directory))

        self.declare_parameter('dryrun', False)
        self.dryrun = self.get_parameter('dryrun').value

        self.trigger_grasp_object_visual_servo_service = self.create_service(
            Trigger,
            '/grasp_object_visual_servo/trigger_grasp_object_visual_servo',
            callback=self.trigger_grasp_object_visual_servo_callback,
            callback_group=self.callback_group)

        self.logger.info('Grasp object visual servo node is ready! (Increment 1: Stage 1 targeting only)')


def main():
    try:
        node = GraspObjectVisualServoNode()
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('grasp_object_visual_servo').info('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
