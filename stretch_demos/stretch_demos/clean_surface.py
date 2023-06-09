#!/usr/bin/env python3

from __future__ import print_function

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
# import actionlib
# from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger

import math
import time
import threading
import sys
import os
import tf2_ros
import argparse as ap

import hello_helpers.hello_misc as hm
# import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


# class CleanSurfaceNode(hm.HelloNode):
class CleanSurfaceNode(Node):
    def __init__(self):
        super().__init__('clean_surface')
        # hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.point_cloud = None
        # self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf2_buffer, self)
        self.log = self.get_logger()
        
    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position

    def point_cloud_callback(self, point_cloud):
        self.point_cloud = point_cloud

    # @staticmethod
    def move_to_pose(self, pose, _async=False, custom_contact_thresholds=False):
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
                self.log.error("HelloNode.move_to_pose: Not sending trajectory due to improper pose. custom_contact_thresholds requires 2 values (pose_target, contact_threshold_effort) for each joint name, but pose = {0}".format(pose))
                return
            joint_positions = [pose[key][0] for key in joint_names]
            joint_efforts = [pose[key][1] for key in joint_names]
            point.positions = joint_positions
            point.effort = joint_efforts
            trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_client.send_goal(trajectory_goal)
        # if not _async: 
        #     self.trajectory_client.wait_for_result()

    def wipe_in(self):
        self.log.info('wipe_in')
        if self.wrist_position is not None:
            wrist_target_m = self.wrist_position - 0.2
            pose = {'wrist_extension': wrist_target_m}
            self.move_to_pose(pose)
            return True
        else:
            self.log.error('wipe_in: self.wrist_position is None!')
            return False

    def wipe_out(self):
        self.log.info('wipe_out')
        if self.wrist_position is not None:
            wrist_target_m = self.wrist_position + 0.22
            pose = {'wrist_extension': wrist_target_m}
            self.move_to_pose(pose)
            return True
        else:
            self.log.error('wipe_out: self.wrist_position is None!')
            return False

    def look_at_surface(self):
        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory)
        manip = self.manipulation_view
        manip.move_head(self.move_to_pose)
        manip.update(self.point_cloud, self.tf2_buffer)
        if self.debug_directory is not None:
            dirname = self.debug_directory + 'clean_surface/'
            # If the directory does not already exist, create it.
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            filename = 'look_at_surface_' + hm.create_time_string()
            manip.save_scan(dirname + filename)
        else:
            self.log.info('CleanSurfaceNode: No debug directory provided, so debugging data will not be saved.')
        
    def trigger_clean_surface_callback(self, request, response):
        self.log.info("Cleaning initiating!")

        tool_width_m = 0.08 #0.06
        tool_length_m = 0.08 #0.06
        step_size_m = 0.04 #0.06 #0.1 #0.02
        min_extension_m = 0.01
        max_extension_m = 0.5
        
        self.look_at_surface()
        strokes, simple_plan, lift_to_surface_m = self.manipulation_view.get_surface_wiping_plan(self.tf2_buffer, tool_width_m, tool_length_m, step_size_m)
        self.log.info("Plan:" + str(simple_plan))

        print('********* lift_to_surface_m = {0} **************'.format(lift_to_surface_m))

        if True and (strokes is not None) and (len(strokes) > 0):

            if (self.lift_position is not None) and (self.wrist_position is not None): 

                above_surface_m = 0.1
                
                lift_above_surface_m = self.lift_position + lift_to_surface_m + above_surface_m
                pose = {'joint_lift': lift_above_surface_m}
                self.log.info('Raise tool above surface.')
                self.move_to_pose(pose)

                start = simple_plan[0]
                forward_m = start['mobile_base_forward_m']
                pose = {'translate_mobile_base': forward_m}
                self.log.info('Drive to the start of the cleaning region.')
                self.move_to_pose(pose)

                initial_wrist_position = self.wrist_position 
                extension_m = start['start_wrist_extension_m']

                #start_extension_m = initial_wrist_position + extension_m + (tool_length_m/2.0)
                start_extension_m = initial_wrist_position + extension_m
                pose = {'wrist_extension': start_extension_m}
                self.log.info('Extend tool above surface.')
                self.move_to_pose(pose)

                use_old_code = False
                if use_old_code: 
                    self.lower_tool_until_contact()
                else: 
                    lift_m = self.lift_position - (above_surface_m - 0.02)
                    lift_contact_effort = 33.7 #effort_pct #20.0 from funmap
                    extension_contact_effort = 16.45 #effort_pct #40.0 from funmap
                    pose = {'joint_lift': (lift_m, lift_contact_effort)}
                    self.move_to_pose(pose, custom_contact_thresholds=True)
                    
                    use_correction = True
                    if use_correction:
                        # raise due to drop down after contact detection
                        #rospy.sleep(0.5) # wait for new lift position
                        # rospy.sleep(0.2) # wait for new lift position
                        lift_m = self.lift_position + 0.01 #0.015
                        pose = {'joint_lift': lift_m}
                        self.move_to_pose(pose)
                        # rospy.sleep(0.2) # wait for new lift position

                extension_m = start['end_wrist_extension_m']
                #end_extension_m = initial_wrist_position + extension_m - (tool_length_m/2.0)
                end_extension_m = initial_wrist_position + extension_m
                pose = {'wrist_extension': end_extension_m}
                self.log.info('Extend tool to end of surface.')
                self.move_to_pose(pose)

                extension_m = start['start_wrist_extension_m']
                pose = {'wrist_extension': start_extension_m}
                self.log.info('Retract tool to beginning.')
                self.move_to_pose(pose)

                for m in simple_plan[1:]:
                    forward_m = m['mobile_base_forward_m']
                    pose = {'translate_mobile_base': forward_m}
                    self.log.info('Drive to the start of the cleaning region.')
                    self.move_to_pose(pose)

                    extension_m = m['start_wrist_extension_m']
                    #start_extension_m = initial_wrist_position + extension_m + (tool_length_m/2.0)
                    start_extension_m = initial_wrist_position + extension_m
                    start_extension_m = max(start_extension_m, min_extension_m)
                    pose = {'wrist_extension': start_extension_m}
                    self.log.info('Extend tool above surface.')
                    self.move_to_pose(pose)

                    extension_m = m['end_wrist_extension_m']
                    #end_extension_m = initial_wrist_position + extension_m - (tool_length_m/2.0)
                    end_extension_m = initial_wrist_position + extension_m
                    end_extension_m = min(end_extension_m, max_extension_m)
                    pose = {'wrist_extension': end_extension_m}
                    self.log.info('Extend tool to end of surface.')
                    self.move_to_pose(pose)

                    pose = {'wrist_extension': start_extension_m}
                    self.log.info('Retract tool to beginning.')
                    self.move_to_pose(pose)

                pose = {'joint_lift': lift_above_surface_m}
                self.log.info('Raise tool above surface.')
                self.move_to_pose(pose)

                pose = {'wrist_extension': initial_wrist_position}
                self.log.info('Retract tool to initial position.')
                self.move_to_pose(pose)

        response = Trigger.Response(success=True, message='Completed surface cleaning!')

        return response
    
    def hello_world_callback(self, request, response):
        self.get_logger().info("Hello world!")
        response = Trigger.Response(success=True, message='Logged!')
        return response
    
    def main(self):
        self.debug_directory = self.get_parameter_or('~debug_directory').value
        self.log.info('Using the following directory for debugging files: {0}'.format(self.debug_directory))

        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 10)

        self.point_cloud_subscriber = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 10)

        self.trigger_clean_surface_service = self.create_service(Trigger,
                                                                '/clean_surface/trigger_clean_surface',
                                                                self.trigger_clean_surface_callback)

        self.hello_world_service = self.create_service(Trigger,
                                                        '/clean_surface/hello_world',
                                                        self.hello_world_callback)

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
            sys.exit()

        self.log.info("Clean surface node is ready!")

def main():
    rclpy.init()
    try:
        node = CleanSurfaceNode()
        node.main()
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger("clean_surface").info('interrupt received, so shutting down')
        
if __name__ == '__main__':
    main()
