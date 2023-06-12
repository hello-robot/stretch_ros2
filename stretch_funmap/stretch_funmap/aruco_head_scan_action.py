#!/usr/bin/env python3

import rclpy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Transform, TransformStamped, Pose
import ros2_numpy
import numpy as np
from tf2_ros import TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import time

from rclpy.duration import Duration
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from stretch_funmap.action import ArucoHeadScan
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
import hello_helpers.hello_misc as hm

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ArucoHeadScanClass(hm.HelloNode):
    def __init__(self):
        # TODO: remove the dependency on HelloNode
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'aruco_head_scan', 'aruco_head_scan', wait_for_first_pointcloud=True)
        self.get_logger().info("Initializing aruco head scan action server")
        self.cb_group = ReentrantCallbackGroup()
        self.server = ActionServer(self, ArucoHeadScan, 'aruco_head_scan', self.execute_cb, callback_group=self.cb_group)
        self.aruco_marker_array = self.create_subscription(MarkerArray, 'aruco/marker_array', self.aruco_callback, 10, callback_group=self.cb_group)
        self.aruco_id = 1000 # Placeholder value
        self.aruco_found = False
        self.marker_array = MarkerArray()
        self.markers = []
        self.joint_state = JointState()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def execute_cb(self, goal_handle):
        self.get_logger().info("Received goal to perform aruco head scan")
        goal = goal_handle.request
        self.aruco_id = goal.aruco_id
        self.tilt_angle = goal.tilt_angle
        self.fill_in_blindspot_with_second_scan = goal.fill_in_blindspot_with_second_scan
        self.fast_scan = goal.fast_scan
        self.publish_to_map = goal.publish_to_map
        self.scan_and_detect(goal_handle)

    def scan_and_detect(self, goal_handle):
        self.get_logger().info("Sending scan goal to head pan and tilt joints")
        self.aruco_tf = None
        pan_angle = -3.14
   
        markers = []
        scan_point = JointTrajectoryPoint()
        start_point = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=3.0)
        start_point.time_from_start = duration1.to_msg()
        scan_point.time_from_start = duration2.to_msg()
        start_point.positions = [self.joint_state.position[7], self.joint_state.position[6]] # [joint_head_tilt, joint_head_pan]
        scan_point.positions = [self.tilt_angle, pan_angle]

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_head_tilt', 'joint_head_pan']
        trajectory_goal.trajectory.points = [start_point, scan_point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal_async(trajectory_goal)

        self.feedback = ArucoHeadScan.Feedback()
        self.feedback.pan_angle = pan_angle
        goal_handle.publish_feedback(self.feedback)

        for pan_angle in np.arange(-3.14, 1.39, 0.7):
            time.sleep(5.0)
            
            rclpy.spin_once(self)
            if self.markers:
                markers = self.markers

            if markers != []:
                for marker in markers:
                    if marker.id == self.aruco_id:
                        self.aruco_found = True
                        self.aruco_name = marker.text
                        if self.publish_to_map:
                            try:
                                trans = self.tf2_buffer.lookup_transform('map', self.aruco_name, rclpy.time.Time())
                                self.aruco_tf = self.broadcast_tf(trans.transform, 'static_{}'.format(self.aruco_name), 'map')
                                self.get_logger().info("{} pose published to tf".format(self.aruco_name))
                            except TransformException as ex:
                                pass

            if not self.aruco_found:
                self.feedback.pan_angle = pan_angle
                goal_handle.publish_feedback(self.feedback)
                duration1 = Duration(seconds=0.0)
                duration2 = Duration(seconds=3.0)
                start_point.time_from_start = duration1.to_msg()
                scan_point.time_from_start = duration2.to_msg()
                start_point.positions = [self.joint_state.position[7], self.joint_state.position[6]] # [joint_head_tilt, joint_head_pan]
                scan_point.positions = [self.tilt_angle, pan_angle]
                trajectory_goal.trajectory.points = [start_point, scan_point]
                trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                trajectory_goal.trajectory.header.frame_id = 'base_link'

                self.trajectory_client.send_goal_async(trajectory_goal)
            else:
                break

        time.sleep(2.0)
        self.result_cb(goal_handle)

    def result_cb(self, goal_handle):
        self.get_logger().info("Finished performing aruco head scan")
        result = ArucoHeadScan.Result()
        result.aruco_found = self.aruco_found
        if self.aruco_found:
            success_str = "Aruco marker found"
            self.get_logger().info(success_str)
            goal_handle.succeed()
            return result
        else:
            error_str = "Could not find aruco marker"
            self.get_logger().info(error_str)
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
        self.marker_array = msg
        self.markers = self.marker_array.markers

    def main(self):
        self.get_logger().info("Initializing the tf buffer, listener and broadcaster")
        self.tf2_buffer = Buffer()
        self.listener = TransformListener(self.tf2_buffer, self)
        self.tf2_static_broadcaster = StaticTransformBroadcaster(self)

        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.joint_states_sub

        while rclpy.ok():
            try:
                self.aruco_tf.header.stamp = self.get_clock().now().to_msg()
                self.tf2_broadcaster.sendTransform(self.aruco_tf)
            except AttributeError:
                pass
            time.sleep(0.2)
            rclpy.spin_once(self)


def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor()
        node = ArucoHeadScanClass()
        node.main()
        executor.add_node(node)
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
