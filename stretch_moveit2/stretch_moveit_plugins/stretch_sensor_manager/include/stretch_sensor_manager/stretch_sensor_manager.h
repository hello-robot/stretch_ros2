#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/sensor_manager/sensor_manager.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

namespace stretch_sensor_manager
{
class StretchSensorManager : public moveit_sensor_manager::MoveItSensorManager
{
public:
  void getSensorsList(std::vector<std::string>& names) const override;

  bool initialize(const rclcpp::Node::SharedPtr& node) override;

  [[nodiscard]] moveit_sensor_manager::SensorInfo getSensorInfo(const std::string& name) const override;

  [[nodiscard]] bool hasSensors() const override;

  bool pointSensorTo(const std::string& name, const geometry_msgs::msg::PointStamped& target,
                     moveit_msgs::msg::RobotTrajectory& sensor_trajectory) override;

protected:
  moveit_sensor_manager::SensorInfo sensor_info_;
  std::string sensor_name_;
  std::string planning_group_;
  planning_pipeline::PlanningPipelineUniquePtr planning_pipeline_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  // planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene_;
  moveit::core::RobotModelPtr robot_model_;
};
}  // namespace stretch_sensor_manager
