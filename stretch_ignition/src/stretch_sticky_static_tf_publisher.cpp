#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/**
 * @brief Simple class that broadcasts tf from the odometry message
 *
 */
class StretchStaticTfPublisher : public rclcpp::Node
{
public:
  StretchStaticTfPublisher() : Node("tf_broadcaster")
  {
    // Publish every 10ms -> 100 Hz
    timer_ = this->create_wall_timer(
      10ms, std::bind(&StretchStaticTfPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    broadcastTf("laser", "stretch/link_laser/gpu_lidar");
    broadcastTf("base_link", "stretch/base_link/imu");
    broadcastTf("base_link", "stretch/base_link/magnetometer");
    broadcastTf("link_wrist_yaw", "stretch/link_wrist_yaw/wrist_imu");
    broadcastTf("camera_gyro_frame", "stretch/camera_gyro_frame/realsense_imu");
    broadcastTf("camera_color_optical_frame", "stretch/camera_color_optical_frame/realsense_d435_color");
    broadcastTf("camera_depth_optical_frame", "stretch/camera_depth_optical_frame/realsense_d435");
    broadcastTf("camera_infra1_optical_frame", "stretch/camera_infra1_optical_frame/realsense_d435_ir");
    broadcastTf("camera_infra2_optical_frame", "stretch/camera_infra2_optical_frame/realsense_d435_ir2");
    broadcastTf("position", "odom");
  }

  void broadcastTf(std::string from, std::string to)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.frame_id = from;
    transformStamped.header.stamp = this->now();
    transformStamped.child_frame_id = to;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transformStamped);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StretchStaticTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
