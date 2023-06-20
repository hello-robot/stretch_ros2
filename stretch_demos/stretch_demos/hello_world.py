#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

import math
import time
import threading
import sys
import tf2_ros
import argparse as ap        

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv


class HelloWorldNode(Node):

    def __init__(self):
        super().__init__('hello_world')
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.letter_top_lift_m = 1.08
        self.logger = self.get_logger()
        
    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
    
    def move_to_pose(self, pose, return_before_done=False, custom_contact_thresholds=False):
        self.move_to_pose_complete = False
        joint_names = [key for key in pose]
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0.0).to_msg()

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        trajectory_goal.trajectory.joint_names = joint_names
        if not custom_contact_thresholds: 
            joint_positions = [pose[key] for key in joint_names]
            point.positions = joint_positions
            trajectory_goal.trajectory.points = [point]
        else:
            pose_correct = all([len(pose[key])==2 for key in joint_names])
            if not pose_correct:
                self.logger.error("HelloNode.move_to_pose: Not sending trajectory due to improper pose. custom_contact_thresholds requires 2 values (pose_target, contact_threshold_effort) for each joint name, but pose = {0}".format(pose))
                return
            joint_positions = [pose[key][0] for key in joint_names]
            joint_efforts = [pose[key][1] for key in joint_names]
            point.positions = joint_positions
            point.effort = joint_efforts
            trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        self._send_goal_future = self.trajectory_client.send_goal_async(trajectory_goal)

        if not return_before_done:
            time_start = time.time()
            self._get_result_future = None

            while self._get_result_future == None and (time.time() - time_start) < 10:
                self.goal_response(self._send_goal_future)

            if self._get_result_future == None:
                return self._send_goal_future

            time_start = time.time()
            while not self.move_to_pose_complete and (time.time() - time_start) < 10:
                self.get_result(self._get_result_future)
        
        return self._send_goal_future

    def align_to_surface(self):
        self.logger.info('align_to_surface')
        trigger_request = Trigger.Request() 
        trigger_result = self.trigger_align_with_nearest_cliff_service.call_async(trigger_request)
        self.logger.info('trigger_result = {0}'.format(trigger_result))


    def backoff_after_contact(self):
        self.logger.info('backoff after contact')
        with self.joint_states_lock:
            wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(self.joint_states)
        if wrist_position is not None:
            wrist_target_m = wrist_position - 0.005
            pose = {'wrist_extension': wrist_target_m}
            self.move_to_pose(pose)
            return True
        else:
            self.logger.error('backoff_from_surface: self.wrist_position is None!')
            return False
        
    def pen_down(self):
        self.logger.info('pen_down')
        max_extension_m = 0.5
        max_reach_m = 0.4
        with self.joint_states_lock:
            wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(self.joint_states)
        extension_m = wrist_position + max_reach_m
        extension_m = min(extension_m, max_extension_m)
        extension_contact_effort = 18.5 #effort_pct
        pose = {'wrist_extension': (extension_m, extension_contact_effort)}
        self.move_to_pose(pose, custom_contact_thresholds=True)
        
    def pen_up(self):
        self.logger.info('pen_up')
        with self.joint_states_lock:
            wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(self.joint_states)
        max_extension_m = 0.5
        min_extension_m = 0.01
        backoff_m = 0.1
        extension_m = wrist_position - backoff_m
        extension_m = min(extension_m, max_extension_m)
        extension_m = max(min_extension_m, extension_m)
        extension_contact_effort = 32.9 #effort_pct # to avoid stopping due to contact
        pose = {'wrist_extension': (extension_m, extension_contact_effort)}
        self.move_to_pose(pose, custom_contact_thresholds=True)
        time.sleep(1.0) # Give it time to backoff before the next move. Otherwise it may not backoff.

        
    def vertical_line(self, move_down, half=False):
        if move_down: 
            self.logger.info('vertical_line moving down')
        else: 
            self.logger.info('vertical_line moving up')
        with self.joint_states_lock: 
            i = self.joint_states.name.index('joint_lift')
            lift_position = self.joint_states.position[i]
        if not half: 
            length_m = self.letter_height_m
        else:
            length_m = self.letter_height_m / 2.0
            
        if move_down:
            new_lift_position = lift_position - length_m
        else:
            new_lift_position = lift_position + length_m
        pose = {'joint_lift': new_lift_position}
        self.move_to_pose(pose)                

    def horizontal_line(self, move_right, half=False):
        if move_right: 
            self.logger.info('horizontal_line moving right')
        else:
            self.logger.info('horizontal_line moving left')
        length_m = self.letter_height_m / 2.0
        if half:
            length_m = length_m / 2.0
        if move_right:
            length_m = -length_m
        use_simple_move = False
        if use_simple_move:
            pose = {'translate_mobile_base': length_m}
            self.move_to_pose(pose)
        else: 
            if length_m > 0: 
                at_goal = self.move_base.forward(length_m, detect_obstacles=False)
            else:
                at_goal = self.move_base.backward(length_m, detect_obstacles=False)
        time.sleep(1.0) # Give it time to finish the base move before the next move.

    def space(self):
        self.logger.info('space')
        self.align_to_surface()
        self.horizontal_line(move_right=True, half=True)


    def move_arm_out_of_the_way(self):
        initial_pose = {'wrist_extension': 0.01,
                        'joint_lift': 0.8,
                        'joint_wrist_yaw': 0.0}

        self.logger.info('Move arm out of the way of the camera.')
        self.move_to_pose(initial_pose)

        
    def move_to_initial_configuration(self):
        initial_pose = {'wrist_extension': 0.01,
                        'joint_lift': self.letter_top_lift_m,
                        'joint_wrist_yaw': 0.0}

        self.logger.info('Move to initial arm pose for writing.')
        self.move_to_pose(initial_pose)

    def letter_h(self):
        self.logger.info('Letter H')
        #write a letter "H" down and to the right
        self.align_to_surface()
        self.pen_down()
        self.vertical_line(move_down=True)
        self.pen_up()
        self.vertical_line(move_down=False, half=True)
        self.pen_down()
        self.horizontal_line(move_right=True)
        self.pen_up()
        self.vertical_line(move_down=False, half=True)
        self.pen_down()
        self.vertical_line(move_down=True)
        self.pen_up()
        self.vertical_line(move_down=False)

    def letter_e(self):
        self.logger.info('Letter E')
        # write a letter "E" down and to the right
        self.align_to_surface()
        self.pen_down()
        self.vertical_line(move_down=True)
        self.pen_up()
        self.pen_down()
        self.horizontal_line(move_right=True)
        self.pen_up()
        self.vertical_line(move_down=False)
        self.pen_down()
        self.horizontal_line(move_right=False)
        self.pen_up()
        self.vertical_line(move_down=True, half=True)
        self.pen_down()
        self.horizontal_line(move_right=True)
        self.pen_up()
        self.vertical_line(move_down=False, half=True)

    def letter_l(self):
        self.logger.info('Letter L')
        # write a letter "L" down and to the right
        self.align_to_surface()
        self.pen_down()
        self.vertical_line(move_down=True)
        self.pen_up()
        self.pen_down()
        self.horizontal_line(move_right=True)
        self.pen_up()
        self.vertical_line(move_down=False)

    def letter_o(self):
        self.logger.info('Letter O')
        # write a letter "O" down and to the right
        self.align_to_surface()
        self.pen_down()
        self.vertical_line(move_down=True)
        self.pen_up()
        self.pen_down()
        self.horizontal_line(move_right=True)
        self.pen_up()
        self.pen_down()
        self.vertical_line(move_down=False)
        self.pen_up()
        self.pen_down()
        self.horizontal_line(move_right=False)
        self.pen_up()
        self.horizontal_line(move_right=True)
                
    def trigger_write_hello_callback(self, request):

        #self.move_arm_out_of_the_way()

        self.move_to_initial_configuration()

        self.align_to_surface()
        
        self.letter_h()
        self.space()
        self.letter_e()
        self.space()
        self.letter_l()
        self.space()
        self.letter_l()
        self.space()
        self.letter_o()
        self.space()

        return Trigger.Response(
            success=True,
            message='Completed writing hello!'
            )

    
    def main(self):
        self.callback_group = ReentrantCallbackGroup()

        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', callback=self.joint_states_callback, callback_group=self.callback_group)
        
        self.trigger_write_hello_service = self.create_service(Trigger, '/hello_world/trigger_write_hello',
                                                         callback=self.trigger_write_hello_callback, callback_group=self.callback_group)

        self.trigger_align_with_nearest_cliff_service = self.create_client(Trigger, '/funmap/trigger_align_with_nearest_cliff', callback_group=self.callback_group)
        self.trigger_align_with_nearest_cliff_service.wait_for_service()
        self.logger.info('Node ' + self.node_name + ' connected to /funmap/trigger_align_with_nearest_cliff.')

        self.trigger_reach_until_contact_service = self.create_client(Trigger, '/funmap/trigger_reach_until_contact', callback_group=self.callback_group)
        self.trigger_reach_until_contact_service.wait_for_service()
        self.logger.info('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
        
if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Hello World demo for stretch.')
        args, unknown = parser.parse_known_args()
        node = HelloWorldNode()
        node.main()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(node=node)
        executor.spin()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('hello_world').info('interrupt received, so shutting down')
