#!/usr/bin/env python3

from copy import deepcopy
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.logging import get_logger

from stretch_funmap.action import ArucoHeadScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import (
    MoveItPy,
)

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, blocking=True, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


class Reach2Aruco(Node):
    def __init__(self):
        super().__init__('head_scan_client')
        self.aruco_head_scan_action_client = ActionClient(self, ArucoHeadScan, 'aruco_head_scan')
        self.switch_to_trajectory_mode_service_client = self.create_client(Trigger, 'switch_to_trajectory_mode')

    def switch_to_trajectory_mode(self):
        self.future = self.switch_to_trajectory_mode_service_client.call_async(self.req)
        return self.future.result()
    
    def switch_to_navigation_mode(self):
        self.future = self.switch_to_navigation_mode_service_client.call_async(self.req)
        return self.future.result()
    
    def send_head_scan_goal(self):
        response = self.switch_to_trajectory_mode()

        goal_msg = ArucoHeadScan.Goal()
        
        goal_msg.aruco_id = 245
        goal_msg.tilt_angle = -0.85
        goal_msg.publish_to_map = True
        goal_msg.fill_in_blindspot_with_second_scan = False
        goal_msg.fast_scan = True

        self.get_logger().info("Waiting for aruco head scan server to start")
        self.aruco_head_scan_action_client.wait_for_server()

        self.get_logger().info("Sending aruco head scan goal")
        self._send_goal_future = self.aruco_head_scan_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.aruco_head_scan_goal_response_cb)

    def aruco_head_scan_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.aruco_head_scan_get_result_cb)

    def aruco_head_scan_get_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Aruco head scan completed')

    def nav_to_pose(self, goal_pose):
        response = self.switch_to_navigation_mode()

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
                time.sleep(2.0)
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def move_to_pose(self, x=0.0, y=0.0, z=0.45):
        response = self.switch_to_trajectory_mode()
        
        # set plan start state to current state
        self.stretch_arm.set_start_state_to_current_state()

        joint_values = {
            "joint_lift": z,
            "joint_arm_l3": y/4,
            "joint_arm_l2": y/4,
            "joint_arm_l1": y/4,
            "joint_arm_l0": y/4,
            "joint_wrist_yaw": 0.0,
            "joint_wrist_pitch": 0.0,
            "joint_wrist_roll": 0.0,
        }
        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.stretch.get_robot_model().get_joint_model_group("stretch_arm"),
        )
        self.stretch_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        plan_and_execute(self.stretch, self.stretch_arm, self.logger, sleep_time=3.0)

    def main(self):
        while not self.switch_to_trajectory_mode_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.switch_to_navigation_mode_service_client = self.create_client(Trigger, 'switch_to_navigation_mode')
        while not self.switch_to_navigation_mode_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = Trigger.Request()
        
        self.navigator = BasicNavigator()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # instantiate MoveItPy instance and get planning component
        self.stretch = MoveItPy(node_name="moveit_py_commander")
        self.stretch_arm = self.stretch.get_planning_component("stretch_arm")

        # instantiate a RobotState instance using the current robot model
        robot_model = self.stretch.get_robot_model()
        self.robot_state = RobotState(robot_model)

def main(args=None):
    rclpy.init(args=args)

    reach_to_aruco = Reach2Aruco()
    reach_to_aruco.main()
    reach_to_aruco.send_head_scan_goal()

    rclpy.spin(reach_to_aruco)

if __name__ == '__main__':
    main()