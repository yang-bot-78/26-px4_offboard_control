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

enum class MotionGateResult
{
	Accept,
	Reject,
	RejectAndRebaseline,
};

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
		max_xy_jump_m_ = declare_parameter<double>("max_xy_jump_m", 0.20);
		max_z_jump_m_ = declare_parameter<double>("max_z_jump_m", 0.10);
		max_computed_speed_mps_ = declare_parameter<double>("max_computed_speed_mps", 1.2);
		dynamic_speed_initial_mps_ = declare_parameter<double>(
			"dynamic_speed_initial_mps", max_computed_speed_mps_);
		dynamic_speed_middle_mps_ = declare_parameter<double>(
			"dynamic_speed_middle_mps", max_computed_speed_mps_);
		dynamic_speed_final_mps_ = declare_parameter<double>(
			"dynamic_speed_final_mps", max_computed_speed_mps_);
		dynamic_speed_initial_duration_s_ = declare_parameter<double>(
			"dynamic_speed_initial_duration_s", 0.0);
		dynamic_speed_middle_duration_s_ = declare_parameter<double>(
			"dynamic_speed_middle_duration_s", 0.0);
		max_computed_z_speed_mps_ = declare_parameter<double>("max_computed_z_speed_mps", 1.2);
		max_reported_speed_mps_ = declare_parameter<double>("max_reported_speed_mps", 4.0);
		max_reported_z_speed_mps_ = declare_parameter<double>("max_reported_z_speed_mps", 2.0);
		min_unknown_angular_variance_ = declare_parameter<double>(
			"min_unknown_angular_variance", 1.0e5);
		min_quaternion_norm_ = declare_parameter<double>("min_quaternion_norm", 0.5);
		max_quaternion_norm_ = declare_parameter<double>("max_quaternion_norm", 1.5);
		reject_log_period_s_ = declare_parameter<double>("reject_log_period_s", 1.0);

		dynamic_speed_initial_mps_ = std::max(0.01, dynamic_speed_initial_mps_);
		dynamic_speed_middle_mps_ = std::max(dynamic_speed_initial_mps_, dynamic_speed_middle_mps_);
		dynamic_speed_final_mps_ = std::max(dynamic_speed_middle_mps_, dynamic_speed_final_mps_);
		dynamic_speed_initial_duration_s_ = std::max(0.0, dynamic_speed_initial_duration_s_);
		dynamic_speed_middle_duration_s_ = std::max(0.0, dynamic_speed_middle_duration_s_);

		const auto odom_qos = rclcpp::SensorDataQoS().keep_last(5);
		publisher_ = create_publisher<Odometry>(output_topic_, odom_qos);
		subscription_ = create_subscription<Odometry>(
			input_topic_, odom_qos,
			std::bind(&FastlioOdometryGuard::odometry_callback, this, std::placeholders::_1));

		RCLCPP_INFO(
			get_logger(),
			"Guarding odometry %s -> %s (jump=%.2fm, xy=%.2fm, z=%.2fm, speed=%.2f->%.2f->%.2fm/s over %.1f+%.1fs, unknown angular variance>=%.0f)",
			input_topic_.c_str(),
			output_topic_.c_str(),
			max_position_jump_m_,
			max_xy_jump_m_,
			max_z_jump_m_,
			dynamic_speed_initial_mps_,
			dynamic_speed_middle_mps_,
			dynamic_speed_final_mps_,
			dynamic_speed_initial_duration_s_,
			dynamic_speed_middle_duration_s_,
			min_unknown_angular_variance_);
	}

private:
	rclcpp::Subscription<Odometry>::SharedPtr subscription_;
	rclcpp::Publisher<Odometry>::SharedPtr publisher_;

	std::string input_topic_;
	std::string output_topic_;
	double max_dt_s_{1.0};
	double min_dt_s_{0.001};
	double max_position_jump_m_{0.60};
	double max_xy_jump_m_{0.20};
	double max_z_jump_m_{0.10};
	double max_computed_speed_mps_{1.2};
	double dynamic_speed_initial_mps_{1.2};
	double dynamic_speed_middle_mps_{1.2};
	double dynamic_speed_final_mps_{1.2};
	double dynamic_speed_initial_duration_s_{0.0};
	double dynamic_speed_middle_duration_s_{0.0};
	double max_computed_z_speed_mps_{1.2};
	double max_reported_speed_mps_{4.0};
	double max_reported_z_speed_mps_{2.0};
	double min_unknown_angular_variance_{1.0e5};
	double min_quaternion_norm_{0.5};
	double max_quaternion_norm_{1.5};
	double reject_log_period_s_{1.0};

	std::optional<Odometry> last_accepted_;
	rclcpp::Time dynamic_speed_started_at_{0, 0, RCL_ROS_TIME};
	std::size_t accepted_count_{0};
	std::size_t rejected_count_{0};
	rclcpp::Time last_reject_log_time_{0, 0, RCL_ROS_TIME};

	bool angular_velocity_is_unknown_with_high_covariance(const Odometry &msg) const
	{
		const auto &w = msg.twist.twist.angular;
		const auto &covariance = msg.twist.covariance;
		const bool all_unknown =
			!is_finite_value(w.x) && !is_finite_value(w.y) && !is_finite_value(w.z);
		return all_unknown &&
		       is_finite_value(covariance[21]) &&
		       is_finite_value(covariance[28]) &&
		       is_finite_value(covariance[35]) &&
		       covariance[21] >= min_unknown_angular_variance_ &&
		       covariance[28] >= min_unknown_angular_variance_ &&
		       covariance[35] >= min_unknown_angular_variance_;
	}

	bool message_is_valid(const Odometry &msg, std::string &reason) const
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

		if (!is_finite_value(v.x) || !is_finite_value(v.y) || !is_finite_value(v.z)) {
			reason = "non-finite linear velocity";
			return false;
		}

		const bool angular_velocity_is_finite =
			is_finite_value(w.x) && is_finite_value(w.y) && is_finite_value(w.z);
		if (!angular_velocity_is_finite && !angular_velocity_is_unknown_with_high_covariance(msg)) {
			reason = "non-finite angular velocity without high covariance";
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

	double current_computed_speed_limit_mps() const
	{
		if (dynamic_speed_started_at_.nanoseconds() == 0) {
			return dynamic_speed_initial_mps_;
		}

		const double elapsed_s = std::max(0.0, (now() - dynamic_speed_started_at_).seconds());
		if (elapsed_s < dynamic_speed_initial_duration_s_) {
			return dynamic_speed_initial_mps_;
		}
		if (elapsed_s < dynamic_speed_initial_duration_s_ + dynamic_speed_middle_duration_s_) {
			return dynamic_speed_middle_mps_;
		}
		return dynamic_speed_final_mps_;
	}

	MotionGateResult check_motion_gate(const Odometry &msg, std::string &reason) const
	{
		if (!last_accepted_.has_value()) {
			return MotionGateResult::Accept;
		}

		const auto &prev = last_accepted_.value();
		const double current_time_s = stamp_to_seconds(msg.header.stamp);
		const double previous_time_s = stamp_to_seconds(prev.header.stamp);
		const double dt = current_time_s - previous_time_s;

		if (dt <= 0.0) {
			std::ostringstream ss;
			ss << "non-monotonic dt " << dt << " s";
			reason = ss.str();
			// Do not move the baseline backwards for duplicate or out-of-order data.
			return MotionGateResult::Reject;
		}

		if (dt <= min_dt_s_) {
			std::ostringstream ss;
			ss << "closely spaced dt " << dt << " s";
			reason = ss.str();
			// Positive sub-threshold samples are benign burst jitter. Drop the frame
			// without moving the baseline; the health monitor does not enter recovery.
			return MotionGateResult::Reject;
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
			return MotionGateResult::Reject;
		}

		if (xy_jump > max_xy_jump_m_) {
			std::ostringstream ss;
			ss << "xy jump " << xy_jump << " m";
			reason = ss.str();
			return MotionGateResult::Reject;
		}

		if (std::fabs(dz) > max_z_jump_m_) {
			std::ostringstream ss;
			ss << "z jump " << dz << " m";
			reason = ss.str();
			return MotionGateResult::Reject;
		}

		if (dt > max_dt_s_) {
			std::ostringstream ss;
			ss << "large dt " << dt << " s; safely rebaseline near last accepted pose";
			reason = ss.str();
			return MotionGateResult::RejectAndRebaseline;
		}

		const double computed_speed_limit_mps = current_computed_speed_limit_mps();
		if (computed_speed > computed_speed_limit_mps) {
			std::ostringstream ss;
			ss << "computed speed " << computed_speed << " m/s exceeds dynamic limit "
			   << computed_speed_limit_mps << " m/s";
			reason = ss.str();
			return MotionGateResult::Reject;
		}

		if (computed_z_speed > max_computed_z_speed_mps_) {
			std::ostringstream ss;
			ss << "computed z speed " << computed_z_speed << " m/s";
			reason = ss.str();
			return MotionGateResult::Reject;
		}

		return MotionGateResult::Accept;
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
		if (!message_is_valid(*msg, reason)) {
			++rejected_count_;
			log_reject(reason);
			return;
		}

		const MotionGateResult gate_result = check_motion_gate(*msg, reason);
		if (gate_result != MotionGateResult::Accept) {
			++rejected_count_;
			if (gate_result == MotionGateResult::RejectAndRebaseline) {
				last_accepted_ = *msg;
			}
			log_reject(reason);
			return;
		}

		publisher_->publish(*msg);
		last_accepted_ = *msg;
		if (dynamic_speed_started_at_.nanoseconds() == 0) {
			dynamic_speed_started_at_ = now();
		}
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
