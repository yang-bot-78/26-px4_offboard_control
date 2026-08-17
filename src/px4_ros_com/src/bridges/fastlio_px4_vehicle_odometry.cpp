#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

using nav_msgs::msg::Odometry;
using px4_msgs::msg::VehicleOdometry;

namespace
{
using Matrix6d = Eigen::Matrix<double, 6, 6>;

std::array<double, 36> rotate_covariance_xy(
  const std::array<double, 36> & input, double linear_yaw, double angular_yaw)
{
  Matrix6d covariance;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      covariance(row, column) = input[row * 6 + column];
    }
  }
  Matrix6d jacobian = Matrix6d::Identity();
  jacobian.block<2, 2>(0, 0) = Eigen::Rotation2Dd(linear_yaw).toRotationMatrix();
  jacobian.block<2, 2>(3, 3) = Eigen::Rotation2Dd(angular_yaw).toRotationMatrix();
  const Matrix6d rotated = jacobian * covariance * jacobian.transpose();
  std::array<double, 36> output{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      output[row * 6 + column] = rotated(row, column);
    }
  }
  return output;
}

float variance_or_nan(const std::array<double, 36> & covariance, std::size_t index)
{
  return std::isfinite(covariance[index]) ? static_cast<float>(covariance[index]) : NAN;
}

}  // namespace

class FastlioPx4VehicleOdometry : public rclcpp::Node
{
public:
  FastlioPx4VehicleOdometry()
  : Node("fastlio_px4_vehicle_odometry")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry/healthy");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/fmu/in/vehicle_visual_odometry");
    reset_topic_ = declare_parameter<std::string>(
      "reset_counter_topic", "/ev_health/reset_counter");
    quality_ = declare_parameter<int>("quality", 100);
    position_yaw_offset_rad_ = declare_parameter<double>("position_yaw_offset_rad", 0.0);
    yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);
    max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 50.0);
    if (!std::isfinite(position_yaw_offset_rad_) || !std::isfinite(yaw_offset_rad_) ||
      !std::isfinite(max_publish_rate_hz_) || max_publish_rate_hz_ < 0.0)
    {
      throw std::invalid_argument("PX4 VehicleOdometry bridge parameters must be finite");
    }
    if (max_publish_rate_hz_ > 0.0) {
      min_publish_interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / max_publish_rate_hz_));
    }

    publisher_ = create_publisher<VehicleOdometry>(output_topic_, rclcpp::SensorDataQoS());
    const auto odom_qos = rclcpp::SensorDataQoS().keep_last(5);
    subscription_ = create_subscription<Odometry>(
      input_topic_, odom_qos,
      std::bind(&FastlioPx4VehicleOdometry::odometry_callback, this, std::placeholders::_1));
    const auto reset_qos = rclcpp::QoS(1).reliable().transient_local();
    reset_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      reset_topic_, reset_qos,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        reset_counter_.store(msg->data, std::memory_order_release);
        // The next sample is the first one allowed in the new EV frame.
        publish_blocked_.store(true, std::memory_order_release);
        RCLCPP_WARN(get_logger(), "[EV_RESET_COUNTER] counter=%u", msg->data);
      });
    RCLCPP_INFO(
      get_logger(), "PX4 VehicleOdometry EV writer %s -> %s at %.1f Hz; reset=%s",
      input_topic_.c_str(), output_topic_.c_str(), max_publish_rate_hz_, reset_topic_.c_str());
  }

private:
  rclcpp::Publisher<VehicleOdometry>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr reset_subscription_;
  std::string input_topic_, output_topic_, reset_topic_;
  int quality_{100};
  double position_yaw_offset_rad_{0.0};
  double yaw_offset_rad_{0.0};
  double max_publish_rate_hz_{50.0};
  std::chrono::steady_clock::duration min_publish_interval_{};
  std::chrono::steady_clock::time_point last_publish_time_{};
  std::atomic<std::uint8_t> reset_counter_{0};
  std::atomic<bool> publish_blocked_{false};

  void odometry_callback(const Odometry::SharedPtr msg)
  {
    if (publish_blocked_.load(std::memory_order_acquire)) {
      publish_blocked_.store(false, std::memory_order_release);
      return;
    }
    const auto publish_time = std::chrono::steady_clock::now();
    if (max_publish_rate_hz_ > 0.0 && last_publish_time_ != std::chrono::steady_clock::time_point{} &&
      publish_time - last_publish_time_ < min_publish_interval_)
    {
      return;
    }
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const auto & v = msg->twist.twist.linear;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Rejecting non-finite EV sample");
      return;
    }

    const Eigen::Vector3d position_enu(p.x, p.y, p.z);
    const Eigen::Vector3d aligned_position =
      Eigen::AngleAxisd(position_yaw_offset_rad_, Eigen::Vector3d::UnitZ()) * position_enu;
    const Eigen::Vector3d position_ned =
      px4_ros_com::frame_transforms::enu_to_ned_local_frame(aligned_position);
    const Eigen::Quaterniond ros_q(q.w, q.x, q.y, q.z);
    const Eigen::Quaterniond aligned_q =
      Eigen::AngleAxisd(yaw_offset_rad_, Eigen::Vector3d::UnitZ()) * ros_q;
    const Eigen::Quaterniond px4_q =
      px4_ros_com::frame_transforms::ros_to_px4_orientation(aligned_q).normalized();
    const Eigen::Vector3d velocity_enu(v.x, v.y, v.z);
    const Eigen::Vector3d aligned_velocity =
      Eigen::AngleAxisd(position_yaw_offset_rad_, Eigen::Vector3d::UnitZ()) * velocity_enu;
    const Eigen::Vector3d velocity_ned =
      px4_ros_com::frame_transforms::enu_to_ned_local_frame(aligned_velocity);
    const auto pose_cov = rotate_covariance_xy(msg->pose.covariance, position_yaw_offset_rad_, yaw_offset_rad_);
    const auto twist_cov = rotate_covariance_xy(msg->twist.covariance, position_yaw_offset_rad_, 0.0);

    VehicleOdometry out{};
    out.timestamp = static_cast<std::uint64_t>(now().nanoseconds() / 1000ULL);
    out.timestamp_sample = static_cast<std::uint64_t>(
      msg->header.stamp.sec) * 1000000ULL + msg->header.stamp.nanosec / 1000ULL;
    out.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    out.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
    out.position = {static_cast<float>(position_ned.x()), static_cast<float>(position_ned.y()),
      static_cast<float>(position_ned.z())};
    px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(px4_q, out.q);
    out.velocity = {static_cast<float>(velocity_ned.x()), static_cast<float>(velocity_ned.y()),
      static_cast<float>(velocity_ned.z())};
    out.angular_velocity = {0.0F, 0.0F, 0.0F};
    out.position_variance = {variance_or_nan(pose_cov, 0), variance_or_nan(pose_cov, 7),
      variance_or_nan(pose_cov, 14)};
    out.orientation_variance = {variance_or_nan(pose_cov, 21), variance_or_nan(pose_cov, 28),
      variance_or_nan(pose_cov, 35)};
    out.velocity_variance = {variance_or_nan(twist_cov, 0), variance_or_nan(twist_cov, 7),
      variance_or_nan(twist_cov, 14)};
    out.reset_counter = reset_counter_.load(std::memory_order_acquire);
    out.quality = static_cast<std::int8_t>(std::clamp(quality_, 0, 100));
    publisher_->publish(out);
    last_publish_time_ = publish_time;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioPx4VehicleOdometry>());
  rclcpp::shutdown();
  return 0;
}
