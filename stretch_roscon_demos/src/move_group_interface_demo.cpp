/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/macros/console_colors.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_interface_demo");


void poseMsgToEigen(const geometry_msgs::msg::Pose& msg, Eigen::Isometry3d& out)
{
  Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if ((quaternion.x() == 0) && (quaternion.y() == 0) && (quaternion.z() == 0) && (quaternion.w() == 0))
  {
    RCLCPP_WARN(LOGGER, "Empty quaternion found in pose message. Setting to neutral orientation.");
    quaternion.setIdentity();
  }
  else
  {
    quaternion.normalize();
  }
  out = translation * quaternion;
}

void initializeMoveGroup(moveit::planning_interface::MoveGroupInterface& move_group)
{
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_demo", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, "gripper");
  moveit::planning_interface::MoveGroupInterface move_group_base_arm(move_group_node, "mobile_base_arm");
  moveit::planning_interface::MoveGroupInterface move_group_head(move_group_node, "stretch_head");

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  namespace rvt = rviz_visual_tools;
  rviz_visual_tools::RvizVisualTools visual_tools("panda_link0", "move_group_tutorial", move_group_node);
  /* moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0"); */
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_base_arm.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_base_arm.getJointModelGroupNames().begin(), move_group_base_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  // Setting Initial Planning Parameters
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  initializeMoveGroup(move_group_base_arm);
  initializeMoveGroup(move_group_gripper);
  move_group_head.setMaxVelocityScalingFactor(0.1);
  move_group_head.setMaxAccelerationScalingFactor(0.1);

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  // Part 0: Turn your head to the target
  geometry_msgs::msg::Pose target_pose;
  move_group_head.setStartStateToCurrentState();
  target_pose.position.x = -0.0019189747981727123;
  target_pose.position.y = -0.03920292109251022;
  target_pose.position.z = 1.297531008720398;
  target_pose.orientation.x = -0.5433773994445801;
  target_pose.orientation.y = 0.45249122381210327;
  target_pose.orientation.z = 0.5637422800064087;
  target_pose.orientation.w = -0.42683398723602295;
  Eigen::Isometry3d approx_target = Eigen::Isometry3d::Identity();
  poseMsgToEigen(target_pose, approx_target);

  move_group_head.setApproximateJointValueTarget(approx_target, move_group_head.getEndEffectorLink());

  bool success = (move_group_head.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Moving head to target %s", success ? "" : "FAILED");

  // Part 1: Move Lift to height
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_base_arm.setStartStateToCurrentState();  
  target_pose.position.x = 0.0;
  target_pose.position.y = -0.2;
  target_pose.position.z = 1.18;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.7071067810149885;
  target_pose.orientation.w = 0.7071067810149885;
  poseMsgToEigen(target_pose, approx_target);

  move_group_base_arm.setApproximateJointValueTarget(approx_target, move_group_base_arm.getEndEffectorLink());

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are planning and asking move_group
  // to actually move the robot. 
  // If we wanted to plan only we would use move.group.plan(plan) instead
  success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Moving arm to height execution %s", success ? "" : "FAILED");


  // Part 2: Open Gripper
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_gripper.setStartStateToCurrentState();
  move_group_gripper.setNamedTarget("open");
  success = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Opening gripper execution %s", success ? "" : "FAILED");
  
  // Part 3: Move to grabbing pose
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_base_arm.setStartStateToCurrentState();
  target_pose.position.y = -0.405;
  poseMsgToEigen(target_pose, approx_target);
  
  move_group_base_arm.setApproximateJointValueTarget(approx_target, move_group_base_arm.getEndEffectorLink());
  success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Move to grabbing pose execution %s", success ? "" : "FAILED");

  // Part 4: Close Gripper and grab the object
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_gripper.setStartStateToCurrentState();
  move_group_gripper.setNamedTarget("closed");
  success = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Closing gripper execution %s", success ? "" : "FAILED");

  // Part 5: Lift the object slightly
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_base_arm.setStartStateToCurrentState();
  target_pose.position.z += 0.15;
  poseMsgToEigen(target_pose, approx_target);
  
  move_group_base_arm.setApproximateJointValueTarget(approx_target, move_group_base_arm.getEndEffectorLink());
  success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Lift the object execution %s", success ? "" : "FAILED");

  // Part 6: Move the robot towards the other table and make a turn
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_base_arm.setStartStateToCurrentState();
  move_group_base_arm.setJointValueTarget("position", {1, 0, 0});
  success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Move the robot execution %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_base_arm.setStartStateToCurrentState();
  move_group_base_arm.setJointValueTarget("position", {1, 0, 3});
  success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Turn the robot execution %s", success ? "" : "FAILED");

  // Part 7: Open gripper and drop the object
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_gripper.setStartStateToCurrentState();
  move_group_gripper.setNamedTarget("open");
  success = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Opening gripper execution %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
  rclcpp::shutdown();
  return 0;
}
