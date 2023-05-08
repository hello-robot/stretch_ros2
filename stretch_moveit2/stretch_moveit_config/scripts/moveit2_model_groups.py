#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
This tutorial has been adapted for Stretch from the official MoveIt 2 Python bindigs
tutorial on https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
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


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    stretch = MoveItPy(node_name="moveit_py")
    stretch_arm = stretch.get_planning_component("stretch_arm")
    stretch_head = stretch.get_planning_component("stretch_head")
    stretch_gripper = stretch.get_planning_component("stretch_gripper")
    stretch_base_arm = stretch.get_planning_component("mobile_base_arm")
    logger.info("MoveItPy instance created")

    # ###########################################################################
    # # Plan 1 - use the stretch_arm planning group
    # ###########################################################################

    # # # set plan start state using predefined state
    # # stretch_arm.set_start_state(configuration_name="arm_in")

    # # set plan start state to current state
    # stretch_arm.set_start_state_to_current_state()

    # # set pose goal using predefined state
    # stretch_arm.set_goal_state(configuration_name="extended")

    # # plan to goal
    # plan_and_execute(stretch, stretch_arm, logger, sleep_time=3.0)

    # ###########################################################################
    # # Plan 2 - use the stretch_head planning group
    # ###########################################################################

    # # set plan start state to current state
    # stretch_head.set_start_state_to_current_state()

    # # set pose goal using predefined state
    # stretch_head.set_goal_state(configuration_name="look_right")

    # # plan to goal
    # plan_and_execute(stretch, stretch_head, logger, sleep_time=3.0)

    # ###########################################################################
    # # Plan 3 - use the stretch_gripper planning group
    # ###########################################################################

    # # set plan start state to current state
    # stretch_gripper.set_start_state_to_current_state()

    # # set pose goal using predefined state
    # stretch_gripper.set_goal_state(configuration_name="open")

    # # plan to goal
    # plan_and_execute(stretch, stretch_gripper, logger, sleep_time=3.0)

    # ###########################################################################
    # # Plan 4 - use the mobile_base_arm planning group
    # ###########################################################################

    #  # set plan start state to current state
    # stretch_base_arm.set_start_state_to_current_state()

    # # set pose goal using predefined state
    # stretch_base_arm.set_goal_state(configuration_name="forward_base")

    # # plan to goal
    # plan_and_execute(stretch, stretch_base_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 5 - Pose goal for mobile_base_arm planning group
    ###########################################################################

    # instantiate a RobotState instance using the current robot model
    robot_model = stretch.get_robot_model()
    robot_state = RobotState(robot_model)

    # set plan start state to current state
    stretch_base_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped, Pose

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = -0.023
    pose_goal.pose.orientation.y = 0.010
    pose_goal.pose.orientation.z = -0.709
    pose_goal.pose.orientation.w = 0.704
    pose_goal.pose.position.x = -0.018
    pose_goal.pose.position.y = -0.60
    pose_goal.pose.position.z = 0.63
    
    # set the robot goal state from IK
    # goal_state = robot_state.set_from_ik(joint_model_group_name='mobile_base_arm', geometry_pose=pose_goal, tip_name='link_grasp_center', timeout=5.0)
    # print('*******************************************************************************')
    # print(goal_state)
    # print(type(goal_state))
    # print(type(robot_state))
    stretch_base_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_grasp_center")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        stretch, ["ompl_rrtc", "pilz_lin", "chomp", "ompl_rrt_star"]
    )

    # plan to goal
    plan_and_execute(
        stretch,
        stretch_base_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0)

if __name__ == "__main__":
    main()
