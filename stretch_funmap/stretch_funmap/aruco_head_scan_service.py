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
        self.predock_tf = None
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
                            if self.aruco_name == 'docking_station':
                                # Transform the docking station frame such that x-axis points out of the aruco plane and 0.5 m in the front of the dock
                                # This facilitates passing the goal pose as this predock frame so that the robot can back up into the dock
                                saved_pose = Transform()
                                saved_pose.translation.x = 0.0
                                saved_pose.translation.y = -0.45
                                saved_pose.translation.z = 0.47
                                saved_pose.rotation.x = -0.382
                                saved_pose.rotation.y = -0.352
                                saved_pose.rotation.z = -0.604
                                saved_pose.rotation.w = 0.604
                                tran = self.broadcast_tf(saved_pose, 'predock_pose', 'docking_station')
                                self.tf2_broadcaster.sendTransform(tran)
                                try:
                                    trans = self.tf2_buffer.lookup_transform('map', 'predock_pose', 0, rospy.Duration(1.0))
                                    # Bring predock_frame at base_link level
                                    angles = euler_from_quaternion([trans.transform.rotation.x,
                                                                    trans.transform.rotation.y,
                                                                    trans.transform.rotation.z,
                                                                    trans.transform.rotation.w])
                                    q = quaternion_from_euler(0, 0, angles[2])
                                    trans.transform.translation.z = 0
                                    trans.transform.rotation.x = q[0]
                                    trans.transform.rotation.y = q[1]
                                    trans.transform.rotation.z = q[2]
                                    trans.transform.rotation.w = q[3]
                                    trans.header.stamp = self.get_clock().now().to_msg()
                                    self.predock_tf = trans
                                    self.get_logger().info("Published predock pose")
                                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                    self.get_logger().info("Could not publish pose to tf")
                                    pass

            if not self.aruco_found:
                self.feedback.pan_angle = pan_angle
                self.server.publish_feedback(self.feedback)

                scan_point.time_from_start = rospy.Duration(3.0)
                scan_point.positions = [self.tilt_angle, pan_angle]
                trajectory_goal.trajectory.points = [scan_point]
                trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                trajectory_goal.trajectory.header.frame_id = 'base_link'

                self.trajectory_client.send_goal(trajectory_goal)
                self.trajectory_client.wait_for_result()
            else:
                break
        
        time.sleep(2.0)    
        self.result_cb(self.aruco_found, "after headscan")

    def result_cb(self, aruco_found, str=None):
        self.result.aruco_found = aruco_found
        if aruco_found:    
            self.get_logger().info("Aruco marker found")
            self.server.set_succeeded(self.result)
        else:
            self.get_logger().info("Could not find aruco marker {}".format(str))
            self.server.set_aborted(self.result)

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
        self.rate = 5.0
        rate = rospy.Rate(self.rate)
        
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        dock_pose = Pose()
        predock_pose = Pose()
        while rclpy.ok():
            try:
                self.aruco_tf.header.stamp = self.get_clock().now().to_msg()
                self.predock_tf.header.stamp = self.get_clock().now().to_msg()
                self.tf2_broadcaster.sendTransform(self.aruco_tf)
                if self.aruco_name == 'docking_station':
                    dock_pose.position.x = self.aruco_tf.transform.translation.x
                    dock_pose.position.y = self.aruco_tf.transform.translation.y
                    dock_pose.position.z = self.aruco_tf.transform.translation.z
                    dock_pose.orientation.x = self.aruco_tf.transform.rotation.x
                    dock_pose.orientation.y = self.aruco_tf.transform.rotation.y
                    dock_pose.orientation.z = self.aruco_tf.transform.rotation.z
                    dock_pose.orientation.w = self.aruco_tf.transform.rotation.w
                    self.dock_pose_pub.publish(dock_pose)
                    self.tf2_broadcaster.sendTransform(self.predock_tf)
                    predock_pose.position.x = self.predock_tf.transform.translation.x
                    predock_pose.position.y = self.predock_tf.transform.translation.y
                    predock_pose.position.z = self.predock_tf.transform.translation.z
                    predock_pose.orientation.x = self.predock_tf.transform.rotation.x
                    predock_pose.orientation.y = self.predock_tf.transform.rotation.y
                    predock_pose.orientation.z = self.predock_tf.transform.rotation.z
                    predock_pose.orientation.w = self.predock_tf.transform.rotation.w
                    self.predock_pose_pub.publish(predock_pose)
            except AttributeError:
                pass
            rate.sleep()


def main():
    try:
        node = ArucoHeadScan()
        node.main()
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')


if __name__ == '__main__':
    main()