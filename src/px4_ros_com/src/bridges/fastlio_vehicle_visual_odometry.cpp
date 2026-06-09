#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

using nav_msgs::msg::Odometry;
using px4_msgs::msg::VehicleOdometry;

class FastlioVehicleVisualOdometry : public rclcpp::Node
{
public:
	FastlioVehicleVisualOdometry() : Node("fastlio_vehicle_visual_odometry")
	{
		input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
		output_topic_ = declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");
		quality_ = declare_parameter<int>("quality", 100);

		publisher_ = create_publisher<VehicleOdometry>(output_topic_, 10);
		subscription_ = create_subscription<Odometry>(
			input_topic_, 10,
			std::bind(&FastlioVehicleVisualOdometry::odometry_callback, this, std::placeholders::_1));

		RCLCPP_INFO(get_logger(), "Bridging %s -> %s as px4_msgs/VehicleOdometry (quality=%d)",
			    input_topic_.c_str(), output_topic_.c_str(), quality_);
	}

private:
	rclcpp::Subscription<Odometry>::SharedPtr subscription_;
	rclcpp::Publisher<VehicleOdometry>::SharedPtr publisher_;

	std::string input_topic_;
	std::string output_topic_;
	int quality_{100};

	static bool is_finite(double value)
	{
		return std::isfinite(value);
	}

	static float variance_or_nan(const std::array<double, 36> &covariance, std::size_t index)
	{
		return is_finite(covariance[index]) ? static_cast<float>(covariance[index]) : NAN;
	}

	void odometry_callback(const Odometry::SharedPtr msg) const
	{
		VehicleOdometry out{};
		out.timestamp = now().nanoseconds() / 1000;
		out.timestamp_sample =
			static_cast<uint64_t>(msg->header.stamp.sec) * 1000000ULL + msg->header.stamp.nanosec / 1000ULL;
		out.pose_frame = VehicleOdometry::POSE_FRAME_NED;
		out.velocity_frame = VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

		const Eigen::Vector3d position_enu(
			msg->pose.pose.position.x,
			msg->pose.pose.position.y,
			msg->pose.pose.position.z);
		const Eigen::Vector3d position_ned =
			px4_ros_com::frame_transforms::enu_to_ned_local_frame(position_enu);
		out.position = {
			static_cast<float>(position_ned.x()),
			static_cast<float>(position_ned.y()),
			static_cast<float>(position_ned.z()),
		};

		const Eigen::Quaterniond ros_q(
			msg->pose.pose.orientation.w,
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z);
		const Eigen::Quaterniond px4_q =
			px4_ros_com::frame_transforms::ros_to_px4_orientation(ros_q).normalized();
		px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(px4_q, out.q);

		const Eigen::Vector3d linear_vel_flu(
			msg->twist.twist.linear.x,
			msg->twist.twist.linear.y,
			msg->twist.twist.linear.z);
		const Eigen::Vector3d linear_vel_frd =
			px4_ros_com::frame_transforms::baselink_to_aircraft_body_frame(linear_vel_flu);
		out.velocity = {
			static_cast<float>(linear_vel_frd.x()),
			static_cast<float>(linear_vel_frd.y()),
			static_cast<float>(linear_vel_frd.z()),
		};

		const Eigen::Vector3d angular_vel_flu(
			msg->twist.twist.angular.x,
			msg->twist.twist.angular.y,
			msg->twist.twist.angular.z);
		const Eigen::Vector3d angular_vel_frd =
			px4_ros_com::frame_transforms::baselink_to_aircraft_body_frame(angular_vel_flu);
		out.angular_velocity = {
			static_cast<float>(angular_vel_frd.x()),
			static_cast<float>(angular_vel_frd.y()),
			static_cast<float>(angular_vel_frd.z()),
		};

		const auto pose_cov_ned =
			px4_ros_com::frame_transforms::enu_to_ned_local_frame(msg->pose.covariance);
		const auto twist_cov_frd =
			px4_ros_com::frame_transforms::baselink_to_aircraft_body_frame(msg->twist.covariance);

		out.position_variance = {
			variance_or_nan(pose_cov_ned, 0),
			variance_or_nan(pose_cov_ned, 7),
			variance_or_nan(pose_cov_ned, 14),
		};
		out.orientation_variance = {
			variance_or_nan(pose_cov_ned, 21),
			variance_or_nan(pose_cov_ned, 28),
			variance_or_nan(pose_cov_ned, 35),
		};
		out.velocity_variance = {
			variance_or_nan(twist_cov_frd, 0),
			variance_or_nan(twist_cov_frd, 7),
			variance_or_nan(twist_cov_frd, 14),
		};

		out.reset_counter = 0;
		out.quality = static_cast<int8_t>(std::clamp(quality_, 0, 100));

		publisher_->publish(out);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FastlioVehicleVisualOdometry>());
	rclcpp::shutdown();
	return 0;
}
