/****************************************************************************
 *
 * Copyright 2026.
 *
 ****************************************************************************/

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Quaternion;

namespace
{

bool is_finite_quaternion(const Quaternion &q)
{
	return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

double quaternion_norm(const Quaternion &q)
{
	return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

Quaternion normalize_quaternion(const Quaternion &q)
{
	const double norm = quaternion_norm(q);

	if (norm <= 1e-9) {
		Quaternion identity{};
		identity.w = 1.0;
		return identity;
	}

	Quaternion out{};
	out.x = q.x / norm;
	out.y = q.y / norm;
	out.z = q.z / norm;
	out.w = q.w / norm;
	return out;
}

Quaternion quaternion_conjugate(const Quaternion &q)
{
	Quaternion out{};
	out.x = -q.x;
	out.y = -q.y;
	out.z = -q.z;
	out.w = q.w;
	return out;
}

Quaternion yaw_quaternion(double yaw_rad)
{
	Quaternion out{};
	out.z = std::sin(yaw_rad / 2.0);
	out.w = std::cos(yaw_rad / 2.0);
	return out;
}

Quaternion quaternion_multiply(const Quaternion &a, const Quaternion &b)
{
	Quaternion out{};
	out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return out;
}

double quaternion_to_yaw_rad(const Quaternion &q)
{
	const Quaternion q_norm = normalize_quaternion(q);
	const double siny_cosp = 2.0 * (q_norm.w * q_norm.z + q_norm.x * q_norm.y);
	const double cosy_cosp = 1.0 - 2.0 * (q_norm.y * q_norm.y + q_norm.z * q_norm.z);
	return std::atan2(siny_cosp, cosy_cosp);
}

bool covariance_is_all_zero(const std::array<double, 36> &covariance)
{
	for (double value : covariance) {
		if (std::fabs(value) > 1e-12) {
			return false;
		}
	}

	return true;
}

void apply_covariance_floor(
	std::array<double, 36> &covariance,
	double position_variance_floor,
	double orientation_variance_floor)
{
	covariance[0] = std::max(covariance[0], position_variance_floor);
	covariance[7] = std::max(covariance[7], position_variance_floor);
	covariance[14] = std::max(covariance[14], position_variance_floor);
	covariance[21] = std::max(covariance[21], orientation_variance_floor);
	covariance[28] = std::max(covariance[28], orientation_variance_floor);
	covariance[35] = std::max(covariance[35], orientation_variance_floor);
}

bool twist_is_all_zero(const Odometry &msg)
{
	return std::fabs(msg.twist.twist.linear.x) < 1e-9 &&
	       std::fabs(msg.twist.twist.linear.y) < 1e-9 &&
	       std::fabs(msg.twist.twist.linear.z) < 1e-9 &&
	       std::fabs(msg.twist.twist.angular.x) < 1e-9 &&
	       std::fabs(msg.twist.twist.angular.y) < 1e-9 &&
	       std::fabs(msg.twist.twist.angular.z) < 1e-9;
}

} // namespace

class FastlioMavrosOdometryBridge : public rclcpp::Node
{
public:
	FastlioMavrosOdometryBridge() : Node("fastlio_mavros_odometry_bridge")
	{
		input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
		output_topic_ = declare_parameter<std::string>("output_topic", "/mavros/odometry/out");
		frame_id_ = declare_parameter<std::string>("frame_id", "odom");
		child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
		force_frame_ids_ = declare_parameter<bool>("force_frame_ids", true);
		restamp_message_ = declare_parameter<bool>("restamp_message", true);
		attitude_yaw_offset_rad_ = declare_parameter<double>("attitude_yaw_offset_rad", 0.0);
		body_yaw_offset_rad_ = declare_parameter<double>("body_yaw_offset_rad", 0.0);
		default_position_variance_ = declare_parameter<double>("default_position_variance", 0.01);
		default_orientation_variance_ = declare_parameter<double>("default_orientation_variance", 0.02);
		default_linear_velocity_variance_ = declare_parameter<double>("default_linear_velocity_variance", 0.01);
		default_angular_velocity_variance_ = declare_parameter<double>("default_angular_velocity_variance", 0.02);
		min_position_variance_ = declare_parameter<double>("min_position_variance", 0.01);
		min_orientation_variance_ = declare_parameter<double>("min_orientation_variance", 0.02);
		min_linear_velocity_variance_ = declare_parameter<double>("min_linear_velocity_variance", 0.01);
		min_angular_velocity_variance_ = declare_parameter<double>("min_angular_velocity_variance", 0.02);

		odometry_publisher_ = create_publisher<Odometry>(output_topic_, 10);
		odometry_subscription_ = create_subscription<Odometry>(
			input_topic_, 10,
			std::bind(&FastlioMavrosOdometryBridge::odometry_callback, this, std::placeholders::_1));

		RCLCPP_INFO(
			get_logger(),
			"Bridging %s -> %s as nav_msgs/Odometry for MAVROS odometry input",
			input_topic_.c_str(),
			output_topic_.c_str());
	}

private:
	rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
	rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;

	std::string input_topic_;
	std::string output_topic_;
	std::string frame_id_;
	std::string child_frame_id_;
	bool force_frame_ids_{true};
	bool restamp_message_{true};
	double attitude_yaw_offset_rad_{0.0};
	double body_yaw_offset_rad_{0.0};
	double default_position_variance_{0.01};
	double default_orientation_variance_{0.02};
	double default_linear_velocity_variance_{0.01};
	double default_angular_velocity_variance_{0.02};
	double min_position_variance_{0.01};
	double min_orientation_variance_{0.02};
	double min_linear_velocity_variance_{0.01};
	double min_angular_velocity_variance_{0.02};
	mutable std::optional<Odometry> previous_msg_;
	mutable int64_t last_yaw_log_ns_{0};

	void odometry_callback(const Odometry::SharedPtr msg) const
	{
		Odometry out = *msg;

		// MAVROS odometry expects the standard local frame pair odom/base_link.
		if (force_frame_ids_) {
			out.header.frame_id = frame_id_;
			out.child_frame_id = child_frame_id_;
		}

		if (restamp_message_) {
			out.header.stamp = now();
		}

		apply_yaw_offsets(out);
		fill_missing_pose_covariance(out);
		fill_missing_twist(out);
		fill_missing_twist_covariance(out);

		odometry_publisher_->publish(out);
		previous_msg_ = out;
	}

	void fill_missing_twist(Odometry &out) const
	{
		if (!twist_is_all_zero(out) || !previous_msg_.has_value()) {
			return;
		}

		const auto &prev = previous_msg_.value();
		const rclcpp::Time current_stamp(out.header.stamp);
		const rclcpp::Time previous_stamp(prev.header.stamp);
		const double dt = (current_stamp - previous_stamp).seconds();

		if (!(dt > 1e-4)) {
			return;
		}

		out.twist.twist.linear.x =
			(out.pose.pose.position.x - prev.pose.pose.position.x) / dt;
		out.twist.twist.linear.y =
			(out.pose.pose.position.y - prev.pose.pose.position.y) / dt;
		out.twist.twist.linear.z =
			(out.pose.pose.position.z - prev.pose.pose.position.z) / dt;

		if (!is_finite_quaternion(out.pose.pose.orientation) ||
		    !is_finite_quaternion(prev.pose.pose.orientation)) {
			return;
		}

		const Quaternion q_prev = normalize_quaternion(prev.pose.pose.orientation);
		const Quaternion q_curr = normalize_quaternion(out.pose.pose.orientation);
		const Quaternion q_delta = normalize_quaternion(
			quaternion_multiply(q_curr, quaternion_conjugate(q_prev)));

		const double angle = 2.0 * std::atan2(
			std::sqrt(q_delta.x * q_delta.x + q_delta.y * q_delta.y + q_delta.z * q_delta.z),
			std::fabs(q_delta.w));

		if (angle <= 1e-9) {
			return;
		}

		const double sin_half = std::sqrt(q_delta.x * q_delta.x + q_delta.y * q_delta.y + q_delta.z * q_delta.z);
		if (sin_half <= 1e-9) {
			return;
		}

		const double axis_x = q_delta.x / sin_half;
		const double axis_y = q_delta.y / sin_half;
		const double axis_z = q_delta.z / sin_half;

		out.twist.twist.angular.x = axis_x * angle / dt;
		out.twist.twist.angular.y = axis_y * angle / dt;
		out.twist.twist.angular.z = axis_z * angle / dt;
	}

	void fill_missing_twist_covariance(Odometry &out) const
	{
		if (covariance_is_all_zero(out.twist.covariance)) {
			out.twist.covariance[0] = default_linear_velocity_variance_;
			out.twist.covariance[7] = default_linear_velocity_variance_;
			out.twist.covariance[14] = default_linear_velocity_variance_;
			out.twist.covariance[21] = default_angular_velocity_variance_;
			out.twist.covariance[28] = default_angular_velocity_variance_;
			out.twist.covariance[35] = default_angular_velocity_variance_;
		}

		apply_covariance_floor(
			out.twist.covariance,
			min_linear_velocity_variance_,
			min_angular_velocity_variance_);
	}

	void apply_yaw_offsets(Odometry &out) const
	{
		if (!is_finite_quaternion(out.pose.pose.orientation)) {
			return;
		}

		const Quaternion q_in = normalize_quaternion(out.pose.pose.orientation);
		const double input_yaw_rad = quaternion_to_yaw_rad(q_in);
		Quaternion q_out = q_in;

		// attitude_yaw_offset_rad rotates the reported attitude in the world/reference
		// frame. Use this when the EV yaw zero/reference is biased by a fixed angle.
		if (std::fabs(attitude_yaw_offset_rad_) > 1e-9) {
			const Quaternion q_attitude_offset = yaw_quaternion(attitude_yaw_offset_rad_);
			q_out = normalize_quaternion(quaternion_multiply(q_attitude_offset, q_out));
		}

		// body_yaw_offset_rad rotates the body frame itself. Use this for fixed
		// installation offsets between the lidar/VIO body frame and PX4 body frame.
		if (std::fabs(body_yaw_offset_rad_) > 1e-9) {
			const Quaternion q_body_offset = yaw_quaternion(body_yaw_offset_rad_);
			q_out = normalize_quaternion(quaternion_multiply(q_out, q_body_offset));
		}

		if (std::fabs(attitude_yaw_offset_rad_) <= 1e-9 &&
		    std::fabs(body_yaw_offset_rad_) <= 1e-9) {
			return;
		}

		const double corrected_yaw_rad = quaternion_to_yaw_rad(q_out);
		out.pose.pose.orientation = q_out;
		const int64_t now_ns =
			static_cast<int64_t>(out.header.stamp.sec) * 1000000000LL +
			static_cast<int64_t>(out.header.stamp.nanosec);

		if (now_ns - last_yaw_log_ns_ >= 1000000000LL) {
			last_yaw_log_ns_ = now_ns;
			RCLCPP_INFO(
				get_logger(),
				"Yaw correction: input_yaw=%.3f rad (%.1f deg), corrected_yaw=%.3f rad (%.1f deg), attitude_offset=%.3f rad (%.1f deg), body_offset=%.3f rad (%.1f deg)",
				input_yaw_rad,
				input_yaw_rad * 180.0 / M_PI,
				corrected_yaw_rad,
				corrected_yaw_rad * 180.0 / M_PI,
				attitude_yaw_offset_rad_,
				attitude_yaw_offset_rad_ * 180.0 / M_PI,
				body_yaw_offset_rad_,
				body_yaw_offset_rad_ * 180.0 / M_PI);
		}
	}

	void fill_missing_pose_covariance(Odometry &out) const
	{
		if (covariance_is_all_zero(out.pose.covariance)) {
			out.pose.covariance[0] = default_position_variance_;
			out.pose.covariance[7] = default_position_variance_;
			out.pose.covariance[14] = default_position_variance_;
			out.pose.covariance[21] = default_orientation_variance_;
			out.pose.covariance[28] = default_orientation_variance_;
			out.pose.covariance[35] = default_orientation_variance_;
		}

		apply_covariance_floor(
			out.pose.covariance,
			min_position_variance_,
			min_orientation_variance_);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FastlioMavrosOdometryBridge>());
	rclcpp::shutdown();
	return 0;
}
