#!/usr/bin/env python3

# Import modules
import rclpy
import time
import tf2_ros
import math
from tf2_ros import TransformException
from rclpy.time import Time
from math import atan2, sqrt, pi
from tf_transformations import euler_from_quaternion, quaternion_matrix

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
from std_msgs.msg import String


from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory


from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import TransformStamped

class LocateArUcoTag(hm.HelloNode):
    
    def __init__(self):

        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'aruco_tag_locator', 'aruco_tag_locator', wait_for_first_pointcloud=False)
        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)

        self.joint_state = None
        self.current_mode = None
        self.move_base = nv.MoveBase(self)  
        #self.joint_states_lock = threading.Lock()
        #self.move_base = None  # Initialize later after node setup
        gap = 0.5
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.pub_name_dk = None  # Initialize publisher later
        self.logger = None  # Initialize logger later

        self.min_pan_position = -3.8
        self.max_pan_position =  1.50

        self.pan_num_steps = 10
        self.pan_step_size = abs(self.min_pan_position - self.max_pan_position)/self.pan_num_steps

        self.min_tilt_position = -0.75
        self.tilt_num_steps = 3
        self.tilt_step_size = pi/16

        self.rot_vel = 0.25 # radians/sec

    # to move regardless of detection
    """ def move_to_handover_pose(self):   
        self.get_logger().info('Moving to handover pose...')

        self.move_base.turn(0.5)
        self.mobile_base_forward_m = 0.5
        tolerance_distance_m = 0.01
        self.move_base.forward(self.mobile_base_forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        self.get_logger().info('Finished moving to handover pose.') """

    def send_command(self, command):
        
        if (self.joint_state is not None) and (command is not None):

            joint_name = command['joint']

            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [joint_name]

            point = JointTrajectoryPoint()

            if 'delta' in command:
                # get the current position of the joint and add the delta as # new position value
                joint_index = self.joint_state.name.index(joint_name)
                joint_value = self.joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point.positions = [new_value]

            elif 'position' in command:
                # extract the head position value from the `position` key
                point.positions = [command['position']]

            point.velocities = [self.rot_vel]

            trajectory_goal.trajectory.points = [point]

            trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory_goal.trajectory.header.frame_id = 'base_link'

            self.trajectory_client.send_goal(trajectory_goal)

    def find_tag(self, tag_name='handover'):

        ### Ensure navigation mode  ###
        self.get_logger().info(f"Current mode: {self.mode.data}")
        pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
        self.send_command(pan_command)
        tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
        self.send_command(tilt_command)

        tilt_step_count = 0
        current_tilt = self.min_tilt_position

        # Sweep until tilt reaches desired range
        while tilt_step_count < self.tilt_num_steps:
            self.get_logger().info(f"Tilt: {current_tilt:.3f}")
            for j in range(self.pan_num_steps):
                # Update pan position
                pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
                self.send_command(pan_command)
                current_pan = self.min_pan_position + (j + 1) * self.pan_step_size
                self.get_logger().info(f"Head Position - Pan: {current_pan:.3f}, Tilt: {current_tilt:.3f}")
                time.sleep(0.2) 

                # Try to find the tag
                try:
                    now = Time()
                    transform = self.tf_buffer.lookup_transform('base_link', tag_name, now)
                    self.get_logger().info(f"Found Requested Tag: {tag_name}")
                    self.get_logger().info(f"Found Requested Tag: \n{transform}")

                    x = transform.transform.translation.x
                    y = transform.transform.translation.y
                    z = transform.transform.translation.z

                    quat = [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
                    ]
                    roll, pitch, yaw = euler_from_quaternion(quat)
                    self.get_logger().info(f"Tag position: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}")

                    # Step 1: rotate to face the tag
                    angle_to_tag = atan2(y, x)  
                    self.get_logger().info(f"Rotating by {angle_to_tag:.3f} radians")
                    self.move_base.turn(angle_to_tag)

                    # Step 2: move forward/backward to match x
                    distance_to_tag = sqrt(x**2 + y**2)  # Distance in the xy-plane
                    final_distance = distance_to_tag - 0.65
                    self.get_logger().info(f"Moving forward {distance_to_tag:.3f} meters")
                    self.move_base.forward(final_distance, detect_obstacles=False, tolerance_distance_m=0.01)
                    
                    # Step 3: adjust lift to match z
                    self.get_logger().info(f"Adjusting lift to {z:.3f} meters")
                    self.move_to_pose({'joint_lift': z}, duration=5.0)

                    # Step 4: rotate the mobile base by 90 degrees to grasp
                    rotate_to_grasp = math.pi / 2  
                    self.get_logger().info(f"Step 4: Rotating mobile base by {rotate_to_grasp:.3f} radians (90 degrees)")
                    self.move_base.turn(rotate_to_grasp)

                    self.get_logger().info(f"Now ready to grasp")

                    return transform
                    
                except TransformException:
                    pass  # continue to next steps

                if j == self.pan_num_steps - 1:
                    current_tilt += self.tilt_step_size
                    tilt_command = {'joint': 'joint_head_tilt', 'position': current_tilt}
                    self.send_command(tilt_command)
                    time.sleep(0.5)
                    tilt_step_count += 1
                    self.get_logger().info(f"Tilt incremented at Pan = 1.50 to: {current_tilt:.3f}")
                    if tilt_step_count >= self.tilt_num_steps:
                        break

            if tilt_step_count >= self.tilt_num_steps:
                break

            # reset pan and increment tilt
            pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
            self.send_command(pan_command)
            current_tilt += self.tilt_step_size
            tilt_command = {'joint': 'joint_head_tilt', 'position': current_tilt}
            self.send_command(tilt_command)
            time.sleep(0.5)
            tilt_step_count += 1
            self.get_logger().info(f"Tilt incremented after pan reset to: {current_tilt:.3f}")

        self.get_logger().info(f"The requested tag '{tag_name}' was not found")
        # to move the robot 0.5 meters regardless of tag detection
        #self.move_to_handover_pose()
        #self.move_base.turn(0.5)

    def main(self, node_name, node_topic_namespace, wait_for_first_pointcloud=True):

        # Initialize node if not already done by quick_create
        if not hasattr(self, 'node_name'):
            hm.HelloNode.main(self, node_name, node_topic_namespace, wait_for_first_pointcloud=False)
        
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        time.sleep(1.0)
        self.get_logger().info('Searching for docking ArUco tag')
        pose = self.find_tag("handover")
        # to execute the handover pose regardless detection
        #self.move_to_handover_pose()


def main(args=None):
    
    node = None
    try:
        node = LocateArUcoTag.quick_create('Locate_nav', wait_for_first_pointcloud=False)
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.logger.info('Interrupt received, shutting down')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
