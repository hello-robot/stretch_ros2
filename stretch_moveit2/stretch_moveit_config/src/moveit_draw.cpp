#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#include <vector>
#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_draw_demo");

std::vector<std::pair<float, float>> get_heart_xy(std::pair<float, float> range, int slices){
  float x, y, t, slice;
  slice = (range.second-range.first)/slices;
  std::vector<std::pair<float, float>> result;
  
  for(t=range.first; t<=range.second; t+=slice){
    std::pair<float, float> temp;
    x = pow(sin(t),3)/4 + 0.25;
    y = (13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t))/64 + 0.8;
    temp.first = x;
    temp.second = y;
    result.push_back(temp);
  }
  return result;
}

std::vector<std::pair<float, float>> get_star_xy(std::pair<float, float> range, int slices){
  float x, y, t, slice;
  slice = (range.second-range.first)/slices;
  std::vector<std::pair<float, float>> result;
  
  for(t=range.first; t<=range.second; t+=slice){
    std::pair<float, float> temp;
    x = 0.25*pow(cos(t),3) + 0.25;
    y = 0.25*pow(sin(t),3) + 0.5;
    temp.first = x;
    temp.second = y;
    result.push_back(temp);
  }
  return result;
}

std::vector<std::pair<float, float>> get_circle_xy(std::pair<float, float> range, int slices){
  float x, y, t, slice;
  slice = (range.second-range.first)/slices;
  std::vector<std::pair<float, float>> result;
  
  for(t=range.first; t<=range.second; t+=slice){
    std::pair<float, float> temp;
    x = 0.25*cos(t) + 0.25;
    y = 0.25*sin(t) + 0.5;
    temp.first = x;
    temp.second = y;
    result.push_back(temp);
  }
  return result;
}

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

  joint_model_group = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  ee_parent_link = move_group_arm.getCurrentState()->getLinkModel("link_grasp_center");

  // ########################### Joint Goal Draw ##########################
  std::pair<float, float> range = {-3.142, 3.142};
  int slices = 20;
  std::vector<std::pair<float, float>> points;

  points = get_heart_xy(range, slices);

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  int num = points.size();

  for(int i=0; i<num; i++){
    std::cout << "Executing waypoint number " << i+1 << std::endl; 
    current_state = move_group_arm.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = points[i].second;   // 0 to 1.1, joint_lift
    joint_group_positions[1] = points[i].first/4; // 0 to 0.130, joint_arm_l3
    joint_group_positions[2] = points[i].first/4; // 0 to 0.130, joint_arm_l2
    joint_group_positions[3] = points[i].first/4; // 0 to 0.130, joint_arm_l1
    joint_group_positions[4] = points[i].first/4; // 0 to 0.130, joint_arm_l0
    
    move_group_arm.setJointValueTarget(joint_group_positions);

    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

    if(i <= 1){
      disp_plan.trajectory_ = my_plan.trajectory_;
    }
    else{
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> a, b;
      a = disp_plan.trajectory_.joint_trajectory.points;
      b = my_plan.trajectory_.joint_trajectory.points;
      a.insert(std::end(a), std::begin(b), std::end(b));
      disp_plan.trajectory_.joint_trajectory.points = a;

      visual_tools.deleteAllMarkers();
      visual_tools.publishTrajectoryLine(disp_plan.trajectory_, ee_parent_link, joint_model_group);
      visual_tools.trigger();
    }

    move_group_arm.move();
  }

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
