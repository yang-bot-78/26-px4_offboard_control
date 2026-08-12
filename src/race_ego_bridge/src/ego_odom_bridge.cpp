#include <cmath>
#include <memory>
#include <string>

#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "race_ego_bridge/frame_utils.hpp"

// Relay the project's already-aligned map odometry into the yaw-only form EGO
// consumes. World alignment belongs to race_mapping and must not be repeated
// here. The input twist is body FLU, so it is still rotated into map axes.
//
// This node is deliberately kept instead of remapping EGO straight onto the
// MAVROS topic: it drops roll/pitch and republishes a yaw-only orientation,
// which EGO's flat (level-cruise) flight mode relies on.
class EgoOdomBridge : public rclcpp::Node
{
public:
  EgoOdomBridge()
  : Node("race_ego_odom_bridge")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/race/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/race/ego/odom");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    // MAVROS carries no per-sample estimator validity flags, so strict health
    // degenerates to "is the FCU link up".
    require_strict_health_ = declare_parameter<bool>("require_strict_health", false);

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&EgoOdomBridge::callback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](mavros_msgs::msg::State::SharedPtr msg) {connected_ = msg->connected;});

    RCLCPP_INFO(
      get_logger(),
      "EGO odom bridge: %s -> %s frame=%s pose=map_passthrough "
      "velocity=bodyFLU->map strict=%s",
      input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
      require_strict_health_ ? "true" : "false");
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const race_ego_bridge::Vec3 map{
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    if (msg->header.frame_id != frame_id_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FRAME_MISMATCH odom=%s expected=%s", msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    // /race/odom follows REP-147: twist is in child_frame_id (base_link FLU).
    // Rotate with the full attitude; yaw-only is wrong during bank or pitch.
    tf2::Quaternion body_to_world_q;
    tf2::fromMsg(msg->pose.pose.orientation, body_to_world_q);
    const tf2::Matrix3x3 body_to_world(body_to_world_q);
    const tf2::Vector3 velocity_world = body_to_world * tf2::Vector3(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const race_ego_bridge::Vec3 velocity_map{
      velocity_world.x(), velocity_world.y(), velocity_world.z()};
    const double yaw_map = tf2::getYaw(msg->pose.pose.orientation);

    if (!race_ego_bridge::finite(map) || !race_ego_bridge::finite(velocity_map) ||
      !std::isfinite(yaw_map))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Reject non-finite MAVROS odometry");
      return;
    }
    if (require_strict_health_ && !connected_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "ODOM_UNHEALTHY mavros_connected=false");
      return;
    }

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_map);

    nav_msgs::msg::Odometry output;
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = frame_id_;
    output.child_frame_id = child_frame_id_;
    output.pose.pose.position.x = map.x;
    output.pose.pose.position.y = map.y;
    output.pose.pose.position.z = map.z;
    output.pose.pose.orientation.x = orientation.x();
    output.pose.pose.orientation.y = orientation.y();
    output.pose.pose.orientation.z = orientation.z();
    output.pose.pose.orientation.w = orientation.w();
    output.twist.twist.linear.x = velocity_map.x;
    output.twist.twist.linear.y = velocity_map.y;
    output.twist.twist.linear.z = velocity_map.z;
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool require_strict_health_{false};
  bool connected_{false};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoOdomBridge>());
  rclcpp::shutdown();
  return 0;
}
