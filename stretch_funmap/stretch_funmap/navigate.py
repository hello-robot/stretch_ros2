#!/usr/bin/env python3

import rclpy
import rclpy.logging
import rclpy.task

from control_msgs.action import FollowJointTrajectory
import ros2_numpy as rn
from std_srvs.srv import Trigger

import hello_helpers.hello_misc as hm

import os
import time

import cv2
import numpy as np

from . import navigation_planning as na
from . import ros_max_height_image as rm

class ForwardMotionObstacleDetector():
    def __init__(self):
        # Define the volume of interest for obstacle detection while
        # moving forward. stretch's mast shakes while moving forward,
        # which can result in false positives if the sensitivity is
        # too high.
        
        # How far to look ahead.
        voi_side_x_m = 0.15
        # Robot's width plus a safety margin.
        voi_side_y_m = 0.4
        voi_height_m = 0.07 #0.06
        robot_front_x_m = 0.08
        voi_axes = np.identity(3)
        voi_origin = np.array([robot_front_x_m, -voi_side_y_m/2.0, -0.03])
        self.voi = rm.ROSVolumeOfInterest('base_link', voi_origin, voi_axes, voi_side_x_m, voi_side_y_m, voi_height_m)

        self.obstacle_pixel_thresh = 100

        m_per_pix = 0.006
        pixel_dtype = np.uint8
        self.max_height_im = rm.ROSMaxHeightImage(self.voi, m_per_pix, pixel_dtype)
        self.max_height_im.print_info()
        self.logger = rclpy.logging.get_logger('stretch_funmap')

    def detect(self, point_cloud_msg, tf2_buffer):
        return (self.count_obstacle_pixels(point_cloud_msg, tf2_buffer) > self.obstacle_pixel_thresh)
        
    def count_obstacle_pixels(self, point_cloud_msg, tf2_buffer):
        self.max_height_im.clear()
        cloud_time = point_cloud_msg.header.stamp
        cloud_frame = point_cloud_msg.header.frame_id
        point_cloud = rn.numpify(point_cloud_msg)
        only_xyz = False
        if only_xyz:
            xyz = rn.point_cloud2.get_xyz_points(point_cloud)
            self.max_height_im.from_points_with_tf2(xyz, cloud_frame, tf2_buffer)
        else: 
            rgb_points = rn.point_cloud2.split_rgb_field(point_cloud)
            self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, tf2_buffer)
        obstacle_im = self.max_height_im.image == 0
        num_obstacle_pix = np.sum(obstacle_im)
        self.logger.info(f'num_obstacle_pix = {num_obstacle_pix}')
        return num_obstacle_pix

    def publish_visualizations(self, voi_marker_pub, point_cloud_pub):
        marker = self.voi.get_ros_marker(duration=1000.0)
        voi_marker_pub.publish(marker)
        point_cloud = self.max_height_im.to_point_cloud()
        point_cloud_pub.publish(point_cloud)


class FastSingleViewPlanner():
    def __init__(self, debug_directory=None):
        self.logger = rclpy.logging.get_logger('stretch_funmap')
        if debug_directory is not None: 
            self.debug_directory = debug_directory + 'fast_single_view_planner/'
            self.logger.info(f'MoveBase __init__: self.debug_directory = {str(self.debug_directory)}')
        else:
            self.debug_directory = debug_directory
        
        # Define the volume of interest for planning using the current
        # view.
        
        # How far to look ahead.
        look_ahead_distance_m = 2.0
        # Robot's width plus a safety margin.
        look_to_side_distance_m = 1.3

        m_per_pix = 0.006
        pixel_dtype = np.uint8

        robot_head_above_ground = 1.13
        lowest_distance_below_ground = 0.03

        voi_height_m = robot_head_above_ground + lowest_distance_below_ground
        robot_front_x_m = -0.1
        voi_side_x_m = abs(robot_front_x_m) + look_ahead_distance_m
        voi_side_y_m = 2.0 * look_to_side_distance_m
        voi_axes = np.identity(3)
        voi_origin = np.array([robot_front_x_m, -voi_side_y_m/2.0, -lowest_distance_below_ground])
        self.frame_id = 'base_link'
        self.voi = rm.ROSVolumeOfInterest(self.frame_id, voi_origin, voi_axes, voi_side_x_m, voi_side_y_m, voi_height_m)
        self.max_height_im = rm.ROSMaxHeightImage(self.voi, m_per_pix, pixel_dtype)
        self.max_height_im.print_info()
        self.updated = False

    def check_line_path(self, end_xyz, end_frame_id, tf2_buffer, floor_mask=None):
        if self.updated: 
            robot_xy_pix, robot_ang_rad, timestamp = self.max_height_im.get_robot_pose_in_image(tf2_buffer)        
            robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]
            end_xy_pix = self.max_height_im.get_point_in_image(end_xyz, end_frame_id, tf2_buffer)[:2]
            
            debug = True
            if debug and (self.debug_directory is not None): 
                # Save the new scan to disk.
                dirname = self.debug_directory + 'check_line_path/'
                filename = 'check_line_path_' + hm.create_time_string()
                self.logger.info(f'FastSingleViewPlanner check_line_path : directory = {dirname}')
                self.logger.info(f'FastSingleViewPlanner check_line_path : filename = {filename}')
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                self.max_height_im.save(dirname + filename)
                im = self.max_height_im.image.copy()
                rpix = np.int64(np.round(robot_xya_pix))
                radius = 20
                color = 255
                cv2.circle(im, tuple(rpix[:2]), radius, color, 2)
                self.logger.info('end_xy_pix = {0}'.format(end_xy_pix))
                epix = np.int64(np.round(end_xy_pix))
                cv2.circle(im, tuple(epix), radius, color, 2)
                cv2.imwrite(dirname + filename + '_with_start_and_end.png', im)

            check_result = na.check_line_path(self.max_height_im, robot_xya_pix, end_xy_pix, floor_mask=floor_mask)
            return check_result
        self.logger.error('FastSingleViewPlanner.check_line_path called without first updating the scan with a pointcloud.')
        return False
        
    def plan_a_path(self, end_xyz, end_frame_id, tf2_buffer, floor_mask=None):
        if self.updated: 
            robot_xy_pix, robot_ang_rad, timestamp = self.max_height_im.get_robot_pose_in_image(tf2_buffer)        
            robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]
            end_xy_pix = self.max_height_im.get_point_in_image(end_xyz, end_frame_id, tf2_buffer)[:2]
            
            debug = True
            if debug and (self.debug_directory is not None): 
                # Save the new scan to disk.
                dirname = self.debug_directory + 'plan_a_path/'
                filename = 'plan_a_path_' + hm.create_time_string()
                self.logger.info(f'FastSingleViewPlanner plan_a_path : directory = {dirname}')
                self.logger.info(f'FastSingleViewPlanner plan_a_path : filename = {filename}')
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                self.save_scan(dirname + filename)
                im = self.max_height_im.image.copy()
                rpix = np.int64(np.round(robot_xya_pix))
                radius = 20
                color = 255
                cv2.circle(im, tuple(rpix[:2]), radius, color, 2)
                self.logger.info('end_xy_pix = {0}'.format(end_xy_pix))
                epix = np.int64(np.round(end_xy_pix))
                cv2.circle(im, tuple(epix), radius, color, 2)
                cv2.imwrite(dirname + filename + '_with_start_and_end.png', im)
            
            line_segment_path, message = na.plan_a_path(self.max_height_im, robot_xya_pix, end_xy_pix, floor_mask=floor_mask)
            if line_segment_path is not None: 
                output_frame = 'base_link'
                image_to_points_mat, ip_timestamp = self.max_height_im.get_image_to_points_mat(output_frame, tf2_buffer)
                path = [np.matmul(image_to_points_mat, np.array([p[0], p[1], 0.0, 1.0])) for p in line_segment_path]
                path = [[p[0], p[1], 0.0] for p in path]
                return path, 'base_link'

        return None, None
        
    def update(self, point_cloud_msg, tf2_buffer):
        self.max_height_im.clear()
        cloud_time = point_cloud_msg.header.stamp
        cloud_frame = point_cloud_msg.header.frame_id
        point_cloud = rn.numpify(point_cloud_msg)
        only_xyz = False
        if only_xyz:
            xyz = rn.point_cloud2.get_xyz_points(point_cloud)
            self.max_height_im.from_points_with_tf2(xyz, cloud_frame, tf2_buffer)
        else: 
            rgb_points = rn.point_cloud2.split_rgb_field(point_cloud)
            self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, tf2_buffer)
        obstacle_im = self.max_height_im.image == 0
        self.updated = True

    def save_scan(self, filename):
        # Save the new scan to disk.
        self.max_height_im.save(filename)

    def publish_visualizations(self, voi_marker_pub, point_cloud_pub):
        marker = self.voi.get_ros_marker(duration=1000.0)
        voi_marker_pub.publish(marker)
        point_cloud = self.max_height_im.to_point_cloud()
        point_cloud_pub.publish(point_cloud)


class MoveBase():
    def __init__(self, node, debug_directory=None):
        self.logger = rclpy.logging.get_logger('stretch_funmap')
        self.debug_directory = debug_directory
        self.logger.info(f"MoveBase __init__: self.debug_directory = {str(self.debug_directory)}")
 
        self.forward_obstacle_detector = ForwardMotionObstacleDetector()
        self.local_planner = FastSingleViewPlanner()
        self.node = node
        self.unsuccessful_status = [-100, 100, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, FollowJointTrajectory.Result.INVALID_JOINTS, FollowJointTrajectory.Result.INVALID_GOAL, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP]  
        self.at_goal = False
        self.unsuccessful_action = False
        self._get_result_future = None

    def head_to_forward_motion_pose(self):
        # Move head to navigation pose.
        #pose = {'joint_head_pan': 0.1, 'joint_head_tilt': -0.9}
        pose = {'joint_head_pan': 0.1, 'joint_head_tilt': -1.1}
        self.node.move_to_pose(pose)

    def goal_response(self, future: rclpy.task.Future):
        self._get_result_future = None
        if not future or not future.result():
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info('Goal rejected')
            return True

        self._get_result_future = goal_handle.get_result_async()
        return True

    def get_result(self, future: rclpy.task.Future):
        if not future or not future.result():
            return

        result = future.result().result
        error_code = result.error_code
        self.logger.info('The Action Server has finished, it returned: "%s"' % str(error_code))

        if (error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.at_goal = True
            self.unsuccessful_action = False
            self.logger.info("Goal point reached.")
        elif (error_code in self.unsuccessful_status):
            self.at_goal = False
            self.unsuccessful_action = True
            self.logger.info("Goal unsuccessful.")
        else:
            self.at_goal = False
            self.unsuccessful_action = False
            self.logger.info("Goal aborted.")

    # def check_move_state(self, result_future: rclpy.task.Future):
    #     at_goal = False
    #     unsuccessful_action = False
    #     if result_future.done():
    #         self.logger.info('Move succeeded!')
    #         # wait for the motion to come to a complete stop
    #         time.sleep(0.5)
    #         at_goal = True
    #     elif result_future.cancelled() or result_future.exception():
    #         self.logger.info('Move action terminated without success.')
    #         unsuccessful_action = True
    #     return at_goal, unsuccessful_action
    
    def local_plan(self, end_xyz, end_frame_id):
        self.local_planner.update(self.node.point_cloud, self.node.tf2_buffer)
        line_segment_path, frame_id = self.local_planner.plan_a_path(end_xyz, end_frame_id, self.node.tf2_buffer, floor_mask=None)
        return line_segment_path, frame_id

    def check_line_path(self, end_xyz, end_frame_id):
        self.local_planner.update(self.node.point_cloud, self.node.tf2_buffer)
        check_result = self.local_planner.check_line_path(end_xyz, end_frame_id, self.node.tf2_buffer, floor_mask=None)
        return check_result

    def backward(self, distance_m, publish_visualizations=True,
                 tolerance_distance_m=0.1, max_forward_attempts=4, detect_obstacles=False):
        self.forward(distance_m, publish_visualizations=publish_visualizations,
                     tolerance_distance_m=tolerance_distance_m, max_forward_attempts=max_forward_attempts, detect_obstacles=detect_obstacles)
    
    def forward(self, distance_m, publish_visualizations=True,
                tolerance_distance_m=0.1, max_forward_attempts=4, detect_obstacles=True):
        # The head needs to have been moved forward prior to calling
        # this function. Consider checking that the head's pose is
        # correct before proceeding
        
        # obtain the initial position of the robot
        xya, timestamp = hm.get_robot_floor_pose_xya(self.node.tf2_buffer)
        start_position_m = xya[:2]
        start_angle_rad = xya[2]
        start_direction = np.array([np.cos(start_angle_rad), np.sin(start_angle_rad)])
        target_position_m = start_position_m + (distance_m * start_direction)
        
        at_goal = False
        unsuccessful_action = False
        if detect_obstacles: 
            obstacle_detected = self.forward_obstacle_detector.detect(self.node.point_cloud,
                                                                      self.node.tf2_buffer)
        else:
            obstacle_detected = False
        
        if detect_obstacles and publish_visualizations: 
            self.forward_obstacle_detector.publish_visualizations(self.node.voi_marker_pub,
                                                                  self.node.obstacle_point_cloud_pub)
        forward_attempts = 0
        forward_distance_m = distance_m

        # Make at least one move if no obtacle was detected.
        while (not obstacle_detected) and (((forward_attempts <= 0) or
                                            ((forward_distance_m > tolerance_distance_m) and
                                             (forward_attempts < max_forward_attempts)))):
            # no obstacles detected, so start moving
            trigger_request = Trigger.Request() 

            pose = {'translate_mobile_base': forward_distance_m}
            self.at_goal = False
            self.unsuccessful_action = False
            self._future_goal = self.node.move_to_pose(pose, blocking=False)

            # Check if goal has been successfully sent to the joint trajectory server
            pose_sent = False
            time_start = time.time()
            timeout = 10

            while not pose_sent and ((time.time() - time_start) < timeout):
                pose_sent = self.goal_response(self._future_goal)

            if not pose_sent or self._get_result_future == None:
                self.logger.info("Failed to drive robot.")
                return self.at_goal

            while (not self.at_goal) and (not obstacle_detected) and (not self.unsuccessful_action):
                if detect_obstacles: 
                    obstacle_detected = self.forward_obstacle_detector.detect(self.node.point_cloud, self.node.tf2_buffer)
                    if obstacle_detected:
                        trigger_result = self.node.stop_the_robot_service.call_async(trigger_request)
                        self.logger.info('trigger_result = ' + str(trigger_result))
                        self.logger.info('Obstacle detected near the front of the robot, so stopping!')
                    if publish_visualizations: 
                        self.forward_obstacle_detector.publish_visualizations(self.node.voi_marker_pub, self.node.obstacle_point_cloud_pub)
                self.get_result(self._get_result_future)

            # obtain the new position of the robot
            xya, timestamp = hm.get_robot_floor_pose_xya(self.node.tf2_buffer)
            end_position_m = xya[:2]
            end_angle_rad = xya[2]
            end_direction = np.array([np.cos(end_angle_rad), np.sin(end_angle_rad)])
            position_error_m = target_position_m - end_position_m
            distance_to_closest_point_m = np.dot(end_direction, position_error_m)
            
            forward_attempts += 1
            forward_distance_m = distance_to_closest_point_m

        if obstacle_detected: 
            self.logger.info('Obstacle detected near the front of the robot, so not starting to move forward.')

        if abs(forward_distance_m) < tolerance_distance_m:
            self.at_goal = True
        else:
            self.at_goal = False
            
        return self.at_goal

    
    def turn(self, angle_rad, publish_visualizations=True, tolerance_angle_rad=0.1, max_turn_attempts=6):
        # obtain the initial angle of the robot
        xya, timestamp = hm.get_robot_floor_pose_xya(self.node.tf2_buffer)
        start_angle_rad = xya[2]
        target_angle_rad = start_angle_rad + angle_rad

        turn_attempts = 0
        # Make a minimum of one turn attempt, even if the magnitude of
        # the turn is very small.
        turn_angle_error_rad = angle_rad
        while ((turn_attempts <= 0) or
               ((abs(turn_angle_error_rad) > tolerance_angle_rad) and
                (turn_attempts < max_turn_attempts))):

            pose = {'rotate_mobile_base': turn_angle_error_rad}
            self.at_goal = False
            self.unsuccessful_action = False
            self._future_goal = self.node.move_to_pose(pose, blocking=False)

            # Check if goal has been successfully sent to the joint trajectory server
            pose_sent = False
            time_start = time.time()
            timeout = 10

            while not pose_sent and ((time.time() - time_start) < timeout):
                pose_sent = self.goal_response(self._future_goal)

            if not pose_sent or self._get_result_future == None:
                self.logger.info("Failed to drive robot.")
                return self.at_goal

            while (not self.at_goal) and (not self.unsuccessful_action):
                # at_goal, unsuccessful_action = self.check_move_state(self.node.trajectory_client)
                self.get_result(self._get_result_future)
                time.sleep(0.01)

            # obtain the new angle of the robot
            xya, timestamp = hm.get_robot_floor_pose_xya(self.node.tf2_buffer)
            end_angle_rad = xya[2]

            # Find the angle that the robot should turn in order
            # to achieve the target.
            turn_angle_error_rad = hm.angle_diff_rad(target_angle_rad, end_angle_rad)
            turn_attempts += 1

        if (abs(turn_angle_error_rad) < tolerance_angle_rad):
            self.at_goal = True
        else:
            self.at_goal = False
            
        return self.at_goal
