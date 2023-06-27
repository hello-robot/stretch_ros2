#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/task.h>

class PickPlaceTask {
public:
  struct Parameters {
    std::string object_name;
//    std::string arm_group_name;
    std::string hand_group_name;
    std::string end_effector_name;
    std::string hand_open_pose;
    std::string hand_close_pose;
    std::string hand_frame;
    std::string mobile_base_arm_group_name;
    geometry_msgs::msg::PoseStamped place_pose;

    void loadParameters(const rclcpp::Node::SharedPtr &node);
  };
  PickPlaceTask(const rclcpp::Node::SharedPtr &node,
                const Parameters &parameters);
  bool plan();
  bool execute();

private:
  moveit::task_constructor::TaskUniquePtr task_;
};
