#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_draw_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto moveit_draw_node = rclcpp::Node::make_shared("moveit_draw", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(moveit_draw_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // ########################### Setup ##########################
  static const std::string PLANNING_GROUP_ARM = "stretch_arm";

  moveit::planning_interface::MoveGroupInterface move_group_arm(moveit_draw_node, PLANNING_GROUP_ARM);

  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);

  const moveit::core::JointModelGroup* joint_model_group;
  const moveit::core::LinkModel* ee_parent_link = 
      move_group_arm.getCurrentState()->getLinkModel("link_grasp_center"); // the link name can be changed here to visualize the trajectory of the corresponding link

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(moveit_draw_node, "odom", "move_group_visualization",
                                                      move_group_arm.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_arm.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan disp_plan;

  bool success;
  std::vector<double> joint_group_positions;
  moveit::core::RobotStatePtr current_state;

  // ########################### Pose Goal ##########################

  joint_model_group = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  ee_parent_link = move_group_arm.getCurrentState()->getLinkModel("link_grasp_center");

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  geometry_msgs::msg::PoseStamped currentPose;
  currentPose = move_group_arm.getCurrentPose();

  move_group_arm.setStartStateToCurrentState();

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = currentPose.pose.orientation.x;
  target_pose1.orientation.y = currentPose.pose.orientation.y;
  target_pose1.orientation.z = currentPose.pose.orientation.z;
  target_pose1.orientation.w = currentPose.pose.orientation.w;
  target_pose1.position.x = currentPose.pose.position.x;
  target_pose1.position.y = currentPose.pose.position.y;
  target_pose1.position.z = 1.05;

  move_group_arm.setApproximateJointValueTarget(target_pose1, "link_wrist_yaw");

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(disp_plan.trajectory_, ee_parent_link, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  move_group_arm.move();

  // ##############################################################
  currentPose = move_group_arm.getCurrentPose();
  move_group_arm.setStartStateToCurrentState();
  
  disp_plan.trajectory_ = my_plan.trajectory_;

  target_pose1.orientation.x = currentPose.pose.orientation.x;
  target_pose1.orientation.y = currentPose.pose.orientation.y;
  target_pose1.orientation.z = currentPose.pose.orientation.z;
  target_pose1.orientation.w = currentPose.pose.orientation.w;
  target_pose1.position.x = currentPose.pose.position.x;
  target_pose1.position.y = -0.53;
  target_pose1.position.z = currentPose.pose.position.z;

  move_group_arm.setApproximateJointValueTarget(target_pose1, "link_wrist_yaw");

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> a, b;
  a = disp_plan.trajectory_.joint_trajectory.points;
  b = my_plan.trajectory_.joint_trajectory.points;
  a.insert(std::end(a), std::begin(b), std::end(b));
  disp_plan.trajectory_.joint_trajectory.points = a;

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(disp_plan.trajectory_, ee_parent_link, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  move_group_arm.move();

  // ###

  currentPose = move_group_arm.getCurrentPose();
  move_group_arm.setStartStateToCurrentState();
  
  target_pose1.orientation.x = currentPose.pose.orientation.x;
  target_pose1.orientation.y = currentPose.pose.orientation.y;
  target_pose1.orientation.z = currentPose.pose.orientation.z;
  target_pose1.orientation.w = currentPose.pose.orientation.w;
  target_pose1.position.x = currentPose.pose.position.x;
  target_pose1.position.y = currentPose.pose.position.y;
  target_pose1.position.z = 0.75;

  move_group_arm.setApproximateJointValueTarget(target_pose1, "link_wrist_yaw");

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  a = disp_plan.trajectory_.joint_trajectory.points;
  b = my_plan.trajectory_.joint_trajectory.points;
  a.insert(std::end(a), std::begin(b), std::end(b));
  disp_plan.trajectory_.joint_trajectory.points = a;

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(disp_plan.trajectory_, ee_parent_link, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  move_group_arm.move();

  // ###

  currentPose = move_group_arm.getCurrentPose();
  move_group_arm.setStartStateToCurrentState();
  
  target_pose1.orientation.x = currentPose.pose.orientation.x;
  target_pose1.orientation.y = currentPose.pose.orientation.y;
  target_pose1.orientation.z = currentPose.pose.orientation.z;
  target_pose1.orientation.w = currentPose.pose.orientation.w;
  target_pose1.position.x = currentPose.pose.position.x;
  target_pose1.position.y = -0.23;
  target_pose1.position.z = currentPose.pose.position.z;

  move_group_arm.setApproximateJointValueTarget(target_pose1, "link_wrist_yaw");

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  a = disp_plan.trajectory_.joint_trajectory.points;
  b = my_plan.trajectory_.joint_trajectory.points;
  a.insert(std::end(a), std::begin(b), std::end(b));
  disp_plan.trajectory_.joint_trajectory.points = a;

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(disp_plan.trajectory_, ee_parent_link, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  move_group_arm.move();
  // ##############################################################

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
