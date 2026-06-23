#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

using nav_msgs::msg::Odometry;

namespace
{

double stamp_to_seconds(const builtin_interfaces::msg::Time &stamp)
{
	return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool is_finite_value(double value)
{
	return std::isfinite(value);
}

bool quaternion_is_finite(const geometry_msgs::msg::Quaternion &q)
{
	return is_finite_value(q.x) && is_finite_value(q.y) &&
	       is_finite_value(q.z) && is_finite_value(q.w);
}

double quaternion_norm(const geometry_msgs::msg::Quaternion &q)
{
	return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

} // namespace

class FastlioOdometryGuard : public rclcpp::Node
{
public:
	FastlioOdometryGuard() : Node("fastlio_odometry_guard")
	{
		input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
		output_topic_ = declare_parameter<std::string>("output_topic", "/Odometry/guarded");
		max_dt_s_ = declare_parameter<double>("max_dt_s", 1.0);
		min_dt_s_ = declare_parameter<double>("min_dt_s", 0.001);
		max_position_jump_m_ = declare_parameter<double>("max_position_jump_m", 0.60);
		max_xy_jump_m_ = declare_parameter<double>("max_xy_jump_m", 0.45);
		max_z_jump_m_ = declare_parameter<double>("max_z_jump_m", 0.25);
		max_computed_speed_mps_ = declare_parameter<double>("max_computed_speed_mps", 3.0);
		max_computed_z_speed_mps_ = declare_parameter<double>("max_computed_z_speed_mps", 1.5);
		max_reported_speed_mps_ = declare_parameter<double>("max_reported_speed_mps", 4.0);
		max_reported_z_speed_mps_ = declare_parameter<double>("max_reported_z_speed_mps", 2.0);
		min_quaternion_norm_ = declare_parameter<double>("min_quaternion_norm", 0.5);
		max_quaternion_norm_ = declare_parameter<double>("max_quaternion_norm", 1.5);
		reject_log_period_s_ = declare_parameter<double>("reject_log_period_s", 1.0);

		publisher_ = create_publisher<Odometry>(output_topic_, 10);
		subscription_ = create_subscription<Odometry>(
			input_topic_, 10,
			std::bind(&FastlioOdometryGuard::odometry_callback, this, std::placeholders::_1));

		RCLCPP_INFO(
			get_logger(),
			"Guarding odometry %s -> %s (jump=%.2fm, xy=%.2fm, z=%.2fm, speed=%.2fm/s)",
			input_topic_.c_str(),
			output_topic_.c_str(),
			max_position_jump_m_,
			max_xy_jump_m_,
			max_z_jump_m_,
			max_computed_speed_mps_);
	}

private:
	rclcpp::Subscription<Odometry>::SharedPtr subscription_;
	rclcpp::Publisher<Odometry>::SharedPtr publisher_;

	std::string input_topic_;
	std::string output_topic_;
	double max_dt_s_{1.0};
	double min_dt_s_{0.001};
	double max_position_jump_m_{0.60};
	double max_xy_jump_m_{0.45};
	double max_z_jump_m_{0.25};
	double max_computed_speed_mps_{3.0};
	double max_computed_z_speed_mps_{1.5};
	double max_reported_speed_mps_{4.0};
	double max_reported_z_speed_mps_{2.0};
	double min_quaternion_norm_{0.5};
	double max_quaternion_norm_{1.5};
	double reject_log_period_s_{1.0};

	std::optional<Odometry> last_accepted_;
	std::size_t accepted_count_{0};
	std::size_t rejected_count_{0};
	rclcpp::Time last_reject_log_time_{0, 0, RCL_ROS_TIME};

	bool message_is_finite(const Odometry &msg, std::string &reason) const
	{
		const auto &p = msg.pose.pose.position;
		const auto &q = msg.pose.pose.orientation;
		const auto &v = msg.twist.twist.linear;
		const auto &w = msg.twist.twist.angular;

		if (!is_finite_value(p.x) || !is_finite_value(p.y) || !is_finite_value(p.z)) {
			reason = "non-finite position";
			return false;
		}

		if (!quaternion_is_finite(q)) {
			reason = "non-finite orientation";
			return false;
		}

		const double q_norm = quaternion_norm(q);
		if (!is_finite_value(q_norm) || q_norm < min_quaternion_norm_ || q_norm > max_quaternion_norm_) {
			std::ostringstream ss;
			ss << "bad quaternion norm " << q_norm;
			reason = ss.str();
			return false;
		}

		if (!is_finite_value(v.x) || !is_finite_value(v.y) || !is_finite_value(v.z) ||
		    !is_finite_value(w.x) || !is_finite_value(w.y) || !is_finite_value(w.z)) {
			reason = "non-finite twist";
			return false;
		}

		const double reported_speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		if (reported_speed > max_reported_speed_mps_) {
			std::ostringstream ss;
			ss << "reported speed " << reported_speed << " m/s";
			reason = ss.str();
			return false;
		}

		if (std::fabs(v.z) > max_reported_z_speed_mps_) {
			std::ostringstream ss;
			ss << "reported z speed " << v.z << " m/s";
			reason = ss.str();
			return false;
		}

		return true;
	}

	bool passes_motion_gate(const Odometry &msg, std::string &reason) const
	{
		if (!last_accepted_.has_value()) {
			return true;
		}

		const auto &prev = last_accepted_.value();
		const double current_time_s = stamp_to_seconds(msg.header.stamp);
		const double previous_time_s = stamp_to_seconds(prev.header.stamp);
		const double dt = current_time_s - previous_time_s;

		if (!(dt > min_dt_s_)) {
			std::ostringstream ss;
			ss << "bad dt " << dt << " s";
			reason = ss.str();
			return false;
		}

		if (dt > max_dt_s_) {
			std::ostringstream ss;
			ss << "large dt " << dt << " s";
			reason = ss.str();
			return false;
		}

		const auto &p = msg.pose.pose.position;
		const auto &p_prev = prev.pose.pose.position;
		const double dx = p.x - p_prev.x;
		const double dy = p.y - p_prev.y;
		const double dz = p.z - p_prev.z;
		const double xy_jump = std::sqrt(dx * dx + dy * dy);
		const double position_jump = std::sqrt(dx * dx + dy * dy + dz * dz);
		const double computed_speed = position_jump / dt;
		const double computed_z_speed = std::fabs(dz) / dt;

		if (position_jump > max_position_jump_m_) {
			std::ostringstream ss;
			ss << "position jump " << position_jump << " m";
			reason = ss.str();
			return false;
		}

		if (xy_jump > max_xy_jump_m_) {
			std::ostringstream ss;
			ss << "xy jump " << xy_jump << " m";
			reason = ss.str();
			return false;
		}

		if (std::fabs(dz) > max_z_jump_m_) {
			std::ostringstream ss;
			ss << "z jump " << dz << " m";
			reason = ss.str();
			return false;
		}

		if (computed_speed > max_computed_speed_mps_) {
			std::ostringstream ss;
			ss << "computed speed " << computed_speed << " m/s";
			reason = ss.str();
			return false;
		}

		if (computed_z_speed > max_computed_z_speed_mps_) {
			std::ostringstream ss;
			ss << "computed z speed " << computed_z_speed << " m/s";
			reason = ss.str();
			return false;
		}

		return true;
	}

	void log_reject(const std::string &reason)
	{
		const auto now_time = now();
		if ((now_time - last_reject_log_time_).seconds() < reject_log_period_s_) {
			return;
		}

		last_reject_log_time_ = now_time;
		RCLCPP_WARN(
			get_logger(),
			"Rejected FAST-LIO odometry: %s (accepted=%zu rejected=%zu)",
			reason.c_str(),
			accepted_count_,
			rejected_count_);
	}

	void odometry_callback(const Odometry::SharedPtr msg)
	{
		std::string reason;
		if (!message_is_finite(*msg, reason) || !passes_motion_gate(*msg, reason)) {
			++rejected_count_;
			log_reject(reason);
			return;
		}

		publisher_->publish(*msg);
		last_accepted_ = *msg;
		++accepted_count_;
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FastlioOdometryGuard>());
	rclcpp::shutdown();
	return 0;
}
