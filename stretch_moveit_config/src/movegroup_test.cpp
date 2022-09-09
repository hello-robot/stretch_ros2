// This is a test script to demonstrate the move group API for MoveIt 2 with the Stretch RE1 robot
// It is based on the MoveGroup C++ tutorial for MoveIt 2

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("movegroup_test", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // ########################### Setup ##########################
  // Setup
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably
  static const std::string PLANNING_GROUP_HEAD = "stretch_head";
  static const std::string PLANNING_GROUP_GRIPPER = "stretch_gripper";
  static const std::string PLANNING_GROUP_ARM = "stretch_arm";
  static const std::string PLANNING_GROUP_BASE = "mobile_base_arm";
  static const std::string PLANNING_GROUP_BASE_ARM = "mobile_base_arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group_head(move_group_node, PLANNING_GROUP_HEAD);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER);
  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_base(move_group_node, PLANNING_GROUP_BASE);
  moveit::planning_interface::MoveGroupInterface move_group_base_arm(move_group_node, PLANNING_GROUP_BASE_ARM);

  // We lower the allowed maximum velocity and acceleration to 80% of their maximum.
  // The default values are 10% (0.1)
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster
  move_group_head.setMaxVelocityScalingFactor(1.0);
  move_group_head.setMaxAccelerationScalingFactor(1.0);
  move_group_gripper.setMaxVelocityScalingFactor(1.0);
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);
  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);
  move_group_base.setMaxVelocityScalingFactor(1.0);
  move_group_base.setMaxAccelerationScalingFactor(1.0);
  move_group_base_arm.setMaxVelocityScalingFactor(1.0);
  move_group_base_arm.setMaxAccelerationScalingFactor(1.0);

  // Raw pointers are frequently used to refer to the planning group for improved performance
  const moveit::core::JointModelGroup* joint_model_group;
  const moveit::core::LinkModel* ee_parent_link = 
      move_group_head.getCurrentState()->getLinkModel("link_grasp_center"); // the link name can be changed here to visualize the trajectory of the corresponding link

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "odom", "move_group_visualization",
                                                      move_group_head.getRobotModel());
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // Start the demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Getting Basic Information
  // We can print the name of the reference frame for this robot
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_head.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Now, we call the planner to compute the plan and visualize it
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;
  std::vector<double> joint_group_positions;

  // We'll create an pointer that references the current robot's state
  // RobotState is the object that contains all the current position/velocity/acceleration data
  moveit::core::RobotStatePtr current_state;

  // ########################### Test ##########################

  // ########################### Step 1 ##########################

  // Planning to a joint-space goal
  // Let's set a joint space goal and move towards it.
  // We'll create an pointer that references the current robot's state.
  current_state = move_group_head.getCurrentState(10);

  joint_model_group =
      move_group_head.getCurrentState()->getJointModelGroup(PLANNING_GROUP_HEAD);

  // Next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Head_Group", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Now, let's modify the joints to turn the head camera, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 80; // joint_head_pan, -220 to 80 faces all the way to the left and right
  joint_group_positions[1] = 0; // joint_head_tilt, -45 faces downwards
  move_group_head.setJointValueTarget(joint_group_positions);

  success = (move_group_head.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_head.move();

  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to turn the camera the other way, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = -220; // joint_head_pan, -220 to 80 faces all the way to the left and right
  joint_group_positions[1] = 0; // joint_head_tilt, -45 faces downwards
  move_group_head.setJointValueTarget(joint_group_positions);

  success = (move_group_head.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_head.move();

  // Next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to face the camera downwards, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = -220; // joint_head_pan, -220 to 80 faces all the way to the left and right
  joint_group_positions[1] = -45; // joint_head_tilt, -45 faces downwards
  move_group_head.setJointValueTarget(joint_group_positions);

  success = (move_group_head.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_head.move();

  // Next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to return the camera to its default position, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0; // joint_head_pan, -220 to 80 faces all the way to the left and right
  joint_group_positions[1] = 0; // joint_head_tilt, -45 faces downwards
  move_group_head.setJointValueTarget(joint_group_positions);

  success = (move_group_head.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_head.move();

  // ########################### Step 2 ##########################
  joint_model_group = move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Gripper_Group", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_gripper.getPlanningFrame().c_str());

  current_state = move_group_gripper.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to close the gripper, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 34; // joint_gripper_finger_left
  joint_group_positions[1] = 34; // joint_gripper_finger_right
  move_group_gripper.setJointValueTarget(joint_group_positions);

  success = (move_group_gripper.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_gripper.move();

  current_state = move_group_gripper.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to open the gripper, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0;
  joint_group_positions[1] = 0;
  move_group_gripper.setJointValueTarget(joint_group_positions);

  success = (move_group_gripper.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_gripper.move();

  // ########################### Step 3 ##########################
  joint_model_group = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Arm_Group", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());

  current_state = move_group_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to raise and stretch the arm, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 1.1;   // 0 to 1.1, joint_lift
  joint_group_positions[1] = 0.130; // 0 to 0.130, joint_arm_l3
  joint_group_positions[2] = 0.130; // 0 to 0.130, joint_arm_l2
  joint_group_positions[3] = 0.130; // 0 to 0.130, joint_arm_l1
  joint_group_positions[4] = 0.130; // 0 to 0.130, joint_arm_l0
  joint_group_positions[5] = 229;   // 0 to 229, joint_wrist_yaw
  move_group_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_arm.move();

  current_state = move_group_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to lower the arm, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0.595;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 0;
  move_group_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_arm.move();

  // ########################### Step 4 ##########################
  joint_model_group = move_group_base.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE);

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Mobile_Base_Group", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base.getPlanningFrame().c_str());

  current_state = move_group_base.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to move the base to the right and forward, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0.5;   // 0 to inf, position/x
  joint_group_positions[1] = -0.5;    // 0 to inf, position/y
  joint_group_positions[2] = 0;  // 0 to 3.14, position/theta
  move_group_base.setJointValueTarget(joint_group_positions);

  success = (move_group_base.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base.move();

  current_state = move_group_base.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to move the base to the initial position, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  move_group_base.setJointValueTarget(joint_group_positions);

  success = (move_group_base.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base.move();

  // ########################### Step 5 ##########################
  joint_model_group = move_group_base_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE_ARM);

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Mobile_Base_Arm_Group", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base.getPlanningFrame().c_str());

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to stow the arm and move forward, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0.5;   // 0 to inf, position/x
  joint_group_positions[1] = 0;      // 0 to inf, position/y
  joint_group_positions[2] = 0;      // -2 to 4, position/theta
  joint_group_positions[3] = 0.2;    // 0 to 1.1, joint_lift
  joint_group_positions[4] = 0;      // 0 to 0.130, joint_arm_l3
  joint_group_positions[5] = 0;      // 0 to 0.130, joint_arm_l2
  joint_group_positions[6] = 0;      // 0 to 0.130, joint_arm_l1
  joint_group_positions[7] = 0;      // 0 to 0.130, joint_arm_l0
  joint_group_positions[8] = 229;    // 0 to 229, joint_wrist_yaw
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to return to the initial position, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0.595;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 0;
  joint_group_positions[6] = 0;
  joint_group_positions[7] = 0;
  joint_group_positions[8] = 0;
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  // ########################### Step 6 ##########################

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Add_Object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Now, let's define a collision object ROS message for the robot to avoid
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_base_arm.getPlanningFrame();

  // The id of the object is used to identify it
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.025;
  primitive.dimensions[primitive.BOX_Y] = 0.2;
  primitive.dimensions[primitive.BOX_Z] = 0.2;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to move the base back, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0.8;   // 0 to inf, position/x
  joint_group_positions[1] = 0;      // 0 to inf, position/y
  joint_group_positions[2] = 0;      // -2 to 4, position/theta
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  collision_object.operation = collision_object.REMOVE;
  collision_objects.push_back(collision_object);

  // Now, let's remove the collision object from the world
  // (using a vector that could contain additional objects)
  RCLCPP_INFO(LOGGER, "Remove object from the world");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Remove_Object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  planning_scene_interface.addCollisionObjects(collision_objects);

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to return the base to initial position, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0;      // 0 to inf, position/x
  joint_group_positions[1] = 0;      // 0 to inf, position/y
  joint_group_positions[2] = 0;      // -2 to 4, position/theta
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  // ########################### Step 7 ##########################

  joint_model_group = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Getting Basic Information
  // We can print the name of the reference frame for this robot
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // We'll save the current pose to use it as a reference point so that we don't have to
  // enter the pose values that must remain constant manually
  geometry_msgs::msg::PoseStamped currentPose;
  currentPose = move_group_arm.getCurrentPose();

  move_group_arm.setStartStateToCurrentState();

  // We create the pose goal
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = currentPose.pose.orientation.x;
  target_pose1.orientation.y = currentPose.pose.orientation.y;
  target_pose1.orientation.z = currentPose.pose.orientation.z;
  target_pose1.orientation.w = currentPose.pose.orientation.w;
  target_pose1.position.x = currentPose.pose.position.x;
  target_pose1.position.y = currentPose.pose.position.y;
  target_pose1.position.z = 1.25;

  // We use the approximate IK solver to get the joint positions
  move_group_arm.setApproximateJointValueTarget(target_pose1, "link_wrist_yaw");

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group_arm.move();

  // ########################### Step 8 ##########################

  joint_model_group = move_group_base_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE_ARM);
  ee_parent_link = move_group_base_arm.getCurrentState()->getLinkModel("link_grasp_center"); // the link name can be changed here to visualize the trajectory of the corresponding link

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Visualize_Trajectory", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  current_state = move_group_base_arm.getCurrentState(10);

  joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to stow the arm, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0.5;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0.2;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 0;
  joint_group_positions[6] = 0;
  joint_group_positions[7] = 0;
  joint_group_positions[8] = 229;
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, ee_parent_link, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group_base_arm.move();

  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
