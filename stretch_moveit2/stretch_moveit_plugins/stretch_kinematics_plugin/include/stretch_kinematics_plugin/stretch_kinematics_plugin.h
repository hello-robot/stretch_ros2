#pragma once

// ROS
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

// ROS msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/kinematic_solver_info.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace stretch_kinematics_plugin
{
class StretchKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool
  getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const override;

  /**
   * @brief  Return all the link names in the order they are represented
   * internally
   */
  const std::vector<std::string>& getLinkNames() const override;

private:
  [[nodiscard]] bool timedOut(const rclcpp::Time& start_time, double duration) const;

  bool initialized_ = false;  ///< Internal variable that indicates whether solver is
                              ///< configured and ready

  unsigned int dimension_;                             ///< Dimension of the group
  moveit_msgs::msg::KinematicSolverInfo solver_info_;  ///< Stores information for the inverse kinematics solver

  const moveit::core::JointModelGroup* joint_model_group_;
  moveit::core::RobotStatePtr state_;

  Eigen::VectorXd joint_min_, joint_max_;  ///< joint limits

  const moveit::core::JointModelGroup* arm_jmg_;
  const moveit::core::JointModelGroup* mobile_base_jmg_;
  const moveit::core::JointModel* mobile_base_joint_;
  // Index that separate the joint values for the arm joint model group from the mobile base joint values in the
  // kinematics solver i.e. the index of the first variable of the mobile base JMG {v_0 v_1 v_2 ............. v_n-4
  // v_n-3 v_n-2 v_n-1} where n is dimension of the group
  //  <- variables for the arm JMG -> | variables for the mobile base JMG (X/Y/theta)
  //                                  -> mobile_base_index_ = n-3
  std::size_t mobile_base_index_;
};
}  // namespace stretch_kinematics_plugin
