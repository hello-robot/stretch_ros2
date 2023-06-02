#!/usr/bin/env python3

import rclpy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Transform, TransformStamped, Pose
import ros2_numpy
import numpy as np
import tf2_ros
import time

from rclpy.duration import Duration
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from stretch_funmap.msg import ArucoHeadScan
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
import hello_helpers.hello_misc as hm

class ArucoHeadScan(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'aruco_head_scan', 'aruco_head_scan', wait_for_first_pointcloud=True)
        self.server = ActionServer(self, ArucoHeadScan, 'ArucoHeadScan', self.execute_cb)
        self.aruco_marker_array = self.create_subscription(MarkerArray, 'aruco/marker_array', self.aruco_callback)
        self.aruco_id = 1000 # Placeholder value
        self.aruco_found = False
        self.markers = MarkerArray().markers

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.aruco_id = goal.aruco_id
        self.tilt_angle = goal.tilt_angle
        self.fill_in_blindspot_with_second_scan = goal.fill_in_blindspot_with_second_scan
        self.fast_scan = goal.fast_scan
        self.publish_to_map = goal.publish_to_map
        self.scan_and_detect(goal_handle)

    def scan_and_detect(self, goal_handle):
        self.aruco_tf = None
        pan_angle = -3.69
        
        markers = []
        scan_point = JointTrajectoryPoint()
        start_point = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=3.0)
        start_point.time_from_start = duration1.to_msg()
        scan_point.time_from_start = duration2.to_msg()
        start_point.positions = [self.joint_state[7], self.joint_state[6]] # [joint_head_tilt, joint_head_pan]
        scan_point.positions = [self.tilt_angle, pan_angle]

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_head_tilt', 'joint_head_pan']
        trajectory_goal.trajectory.points = [start_point, scan_point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal_async(trajectory_goal)

        self.feedback = FollowJointTrajectory.Feedback()
        self.feedback.pan_angle = pan_angle
        goal_handle.publish_feedback(self.feedback)

        for pan_angle in np.arange(-3.69, 1.66, 0.7):
            time.sleep(3.0)
            for i in range(20):
                if self.markers:
                    markers = self.markers
                    break

            self.get_logger().info("Markers found: {}".format(markers))
            
            if markers != []:
                for marker in markers:
                    if marker.id == self.aruco_id:
                        self.aruco_found = True
                        self.aruco_name = marker.text
                        if self.publish_to_map:
                            trans = self.tf2_buffer.lookup_transform('map', self.aruco_name, 0)
                            self.aruco_tf = self.broadcast_tf(trans.transform, self.aruco_name, 'map')
                            self.get_logger().info("{} pose published to tf".format(self.aruco_name))

            if not self.aruco_found:
                self.feedback.pan_angle = pan_angle
                goal_handle.publish_feedback(self.feedback)
                duration1 = Duration(seconds=0.0)
                duration2 = Duration(seconds=3.0)
                start_point.time_from_start = duration1.to_msg()
                scan_point.time_from_start = duration2.to_msg()
                start_point.positions = [self.joint_state[7], self.joint_state[6]] # [joint_head_tilt, joint_head_pan]
                scan_point.positions = [self.tilt_angle, pan_angle]
                trajectory_goal.trajectory.points = [start_point, scan_point]
                trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                trajectory_goal.trajectory.header.frame_id = 'base_link'

                self.trajectory_client.send_goal_async(trajectory_goal)
            else:
                break
        
        time.sleep(2.0)    
        self.result_cb(goal_handle, self.aruco_found)

    def result_cb(self, goal_handle, aruco_found):
        self.result.aruco_found = aruco_found
        result = FollowJointTrajectory.Result()
        if aruco_found:    
            success_str = "Aruco marker found"
            self.get_logger().info(success_str)
            goal_handle.succeed()
            result.error_code = result.SUCCESSFUL
            result.error_string = success_str
            return result
        else:
            error_str = "Could not find aruco marker"
            self.get_logger().info(error_str)
            result.error_code = -1
            result.error_string = error_str
            goal_handle.abort()
            return result

    def broadcast_tf(self, trans, name, ref):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ref
        t.child_frame_id = name
        t.transform = trans
        return t

    def aruco_callback(self, msg):
        self.markers = msg.markers

    def main(self):
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        dock_pose = Pose()
        predock_pose = Pose()
        while rclpy.ok():
            try:
                self.aruco_tf.header.stamp = self.get_clock().now().to_msg()
                self.tf2_broadcaster.sendTransform(self.aruco_tf)
            except AttributeError:
                pass
            time.sleep(0.2)


def main():
    try:
        node = ArucoHeadScan()
        node.main()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
