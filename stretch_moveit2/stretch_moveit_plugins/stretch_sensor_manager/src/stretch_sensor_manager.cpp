#include <stretch_sensor_manager/stretch_sensor_manager.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("stretch_sensor_manager");

/**
 * @brief Return the skew symmetric matrix of the input vector
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& vector)
{
  Eigen::Matrix3d skew_matrix;
  // clang-format off
  skew_matrix <<          0, -vector(2),  vector(1),
                  vector(2),          0, -vector(0),
                 -vector(1),  vector(0),          0;
  // clang-format on
  return skew_matrix;
}

namespace stretch_sensor_manager
{
void StretchSensorManager::getSensorsList(std::vector<std::string>& names) const
{
  names = { sensor_name_ };
}

bool StretchSensorManager::initialize(const rclcpp::Node::SharedPtr& node)
{
  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node);
  robot_model_ = robot_model_loader->getModel();

  const std::string parameter_ns = "stretch_sensor_manager";
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(node, parameter_ns + ".sensor_name", sensor_name_);
  error += !rosparam_shortcuts::get(node, parameter_ns + "." + sensor_name_ + ".planning_group", planning_group_);
  if (!robot_model_->hasJointModelGroup(planning_group_))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Robot model doesn't have the following planning_group: " << planning_group_);
    return false;
  }

  /**
   * stretch_sensor_manager:
   *  sensor_name: name (string)
   *  name:
   *    planning_group: string
   *    min_dist: double
   *    max_dist: double
   *    x_angle: double
   *    y_angle: double
   */
  planning_pipeline_ = std::make_unique<planning_pipeline::PlanningPipeline>(robot_model_, node, "move_group");
  error += !rosparam_shortcuts::get(node, parameter_ns + "." + sensor_name_ + ".min_dist", sensor_info_.min_dist);
  error += !rosparam_shortcuts::get(node, parameter_ns + "." + sensor_name_ + ".max_dist", sensor_info_.max_dist);
  error += !rosparam_shortcuts::get(node, parameter_ns + "." + sensor_name_ + ".x_angle", sensor_info_.x_angle);
  error += !rosparam_shortcuts::get(node, parameter_ns + "." + sensor_name_ + ".y_angle", sensor_info_.y_angle);

  planning_scene_monitor_ = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>(node, robot_model_loader);
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

  if (planning_scene)
  {
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->requestPlanningSceneState();
    // Wait for complete state to be received
    planning_scene_monitor_->getStateMonitor()->waitForCurrentState(node->now());
  }
  return error == 0;
}

moveit_sensor_manager::SensorInfo StretchSensorManager::getSensorInfo(const std::string& name) const
{
  if (name == sensor_name_)
    return sensor_info_;
  RCLCPP_ERROR_STREAM(LOGGER, "Unknown sensor: " << name);
  return moveit_sensor_manager::SensorInfo();
}

bool StretchSensorManager::hasSensors() const
{
  return !sensor_name_.empty();
}

bool StretchSensorManager::pointSensorTo(const std::string& name, const geometry_msgs::msg::PointStamped& target,
                                         moveit_msgs::msg::RobotTrajectory& sensor_trajectory)
{
  if (name != sensor_name_)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unknown sensor: " << name);
    return false;
  }

  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene_locked(planning_scene_monitor_);
  auto planning_scene = planning_scene::PlanningScene::clone(planning_scene_locked);
  const std::string& link_name = robot_model_->getJointModelGroup(planning_group_)->getLinkModelNames().back();
  const Eigen::Isometry3d link_frame = planning_scene->getFrameTransform(link_name);
  // Calculating the frame the align the vector pointing from the link origin to the target point with one of the
  // link's basis vectors Currently hard coded for y-axis (the axis pointing outside the camera)
  // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
  const size_t axis_index = 0;
  const Eigen::Vector3d axis = link_frame.rotation().col(axis_index);
  Eigen::Vector3d point_eigen;
  tf2::fromMsg(target.point, point_eigen);
  point_eigen = planning_scene->getFrameTransform(target.header.frame_id) * point_eigen;
  const Eigen::Vector3d link_to_point_direction = (point_eigen - link_frame.translation()).normalized();
  const Eigen::Vector3d v = axis.cross(link_to_point_direction);
  const double s = v.norm();
  const double c = axis.dot(link_to_point_direction);
  Eigen::Isometry3d desired_pose(link_frame);
  if (std::abs(s) > 0.01)
  {
    const auto v_skew = skew(v);
    desired_pose =
        desired_pose * Eigen::Quaterniond(Eigen::Matrix3d::Identity() + v_skew + v_skew * v_skew * (1 - c) / (s * s));
  }

  planning_interface::MotionPlanRequest req;
  req.group_name = planning_group_;
  geometry_msgs::msg::PoseStamped target_orientation;
  target_orientation.pose = tf2::toMsg(desired_pose);
  target_orientation.header.frame_id = planning_scene->getPlanningFrame();
  req.goal_constraints.emplace_back(kinematic_constraints::constructGoalConstraints(link_name, target_orientation));
  req.allowed_planning_time = 1.0;
  req.num_planning_attempts = 2;
  moveit::core::robotStateToRobotStateMsg(planning_scene->getCurrentState(), req.start_state);
  planning_interface::MotionPlanResponse res;
  if (planning_pipeline_->generatePlan(planning_scene, req, res))
  {
    res.trajectory_->getRobotTrajectoryMsg(sensor_trajectory);
    return true;
  }

  RCLCPP_ERROR_STREAM(LOGGER, "Failed to find a solution");
  return false;
}
}  // namespace stretch_sensor_manager

// register StretchSensorManager as a MoveItSensorManager implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(stretch_sensor_manager::StretchSensorManager, moveit_sensor_manager::MoveItSensorManager)
