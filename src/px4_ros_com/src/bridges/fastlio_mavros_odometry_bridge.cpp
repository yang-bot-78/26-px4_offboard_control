#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros_com/lever_arm_compensation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

using nav_msgs::msg::Odometry;

namespace
{

double steady_clock_seconds()
{
	return std::chrono::duration<double>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

double stamp_to_seconds(const builtin_interfaces::msg::Time &stamp)
{
	return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw_rad)
{
	geometry_msgs::msg::Quaternion q{};
	q.z = std::sin(yaw_rad * 0.5);
	q.w = std::cos(yaw_rad * 0.5);
	return q;
}

geometry_msgs::msg::Quaternion quaternion_multiply(
	const geometry_msgs::msg::Quaternion &a,
	const geometry_msgs::msg::Quaternion &b)
{
	geometry_msgs::msg::Quaternion out{};
	out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return out;
}

std::array<double, 36> rotate_covariance_xy(
	const std::array<double, 36> &input,
	double linear_yaw_rad,
	double angular_yaw_rad)
{
	double jacobian[6][6]{};
	for (std::size_t i = 0; i < 6; ++i) {
		jacobian[i][i] = 1.0;
	}

	const double linear_c = std::cos(linear_yaw_rad);
	const double linear_s = std::sin(linear_yaw_rad);
	jacobian[0][0] = linear_c;
	jacobian[0][1] = -linear_s;
	jacobian[1][0] = linear_s;
	jacobian[1][1] = linear_c;

	const double angular_c = std::cos(angular_yaw_rad);
	const double angular_s = std::sin(angular_yaw_rad);
	jacobian[3][3] = angular_c;
	jacobian[3][4] = -angular_s;
	jacobian[4][3] = angular_s;
	jacobian[4][4] = angular_c;

	std::array<double, 36> output{};
	for (std::size_t row = 0; row < 6; ++row) {
		for (std::size_t column = 0; column < 6; ++column) {
			for (std::size_t left = 0; left < 6; ++left) {
				for (std::size_t right = 0; right < 6; ++right) {
					output[row * 6 + column] += jacobian[row][left] *
						input[left * 6 + right] * jacobian[column][right];
				}
			}
		}
	}
	return output;
}

std::array<double, 36> rotate_twist_covariance_linear(
	const std::array<double, 36> &input,
	const px4_ros_com::lever_arm::Matrix3 &rotation)
{
	double jacobian[6][6]{};
	for (std::size_t i = 0; i < 3; ++i) {
		jacobian[i + 3][i + 3] = 1.0;
		for (std::size_t j = 0; j < 3; ++j) {
			jacobian[i][j] = rotation[i][j];
		}
	}

	std::array<double, 36> output{};
	for (std::size_t row = 0; row < 6; ++row) {
		for (std::size_t column = 0; column < 6; ++column) {
			for (std::size_t left = 0; left < 6; ++left) {
				for (std::size_t right = 0; right < 6; ++right) {
					output[row * 6 + column] += jacobian[row][left] *
						input[left * 6 + right] * jacobian[column][right];
				}
			}
		}
	}
	return output;
}

} // namespace

class FastlioMavrosOdometryBridge : public rclcpp::Node
{
public:
	FastlioMavrosOdometryBridge() : Node("fastlio_mavros_odometry_bridge")
	{
		input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
		output_topic_ = declare_parameter<std::string>("output_topic", "/mavros/odometry/out");
		world_frame_id_ = declare_parameter<std::string>("world_frame_id", "odom");
		body_frame_id_ = declare_parameter<std::string>("body_frame_id", "base_link");
		restamp_message_ = declare_parameter<bool>("restamp_message", false);
		max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 50.0);
		world_yaw_alignment_rad_ = declare_parameter<double>("world_yaw_alignment_rad", 0.0);
		body_to_sensor_x_m_ = declare_parameter<double>("body_to_sensor_x_m", 0.0);
		body_to_sensor_y_m_ = declare_parameter<double>("body_to_sensor_y_m", 0.0);
		body_to_sensor_z_m_ = declare_parameter<double>("body_to_sensor_z_m", 0.0);
		body_to_fastlio_yaw_rad_ = declare_parameter<double>("body_to_fastlio_yaw_rad", 0.0);
		timing_alignment_enabled_ = declare_parameter<bool>("timing_alignment_enabled", false);
		frlio_anchor_age_topic_ = declare_parameter<std::string>(
			"frlio_anchor_age_topic", "/frlio/high_rate_odom/anchor_age");

		const px4_ros_com::lever_arm::Vector3 lever_arm{
			body_to_sensor_x_m_, body_to_sensor_y_m_, body_to_sensor_z_m_};
		if (!px4_ros_com::lever_arm::finite(lever_arm) ||
			!std::isfinite(world_yaw_alignment_rad_) ||
			!std::isfinite(body_to_fastlio_yaw_rad_) ||
			!std::isfinite(max_publish_rate_hz_) || max_publish_rate_hz_ < 0.0) {
			throw std::invalid_argument("ODOMETRY installation and rate parameters must be finite");
		}
		if (max_publish_rate_hz_ > 0.0) {
			min_publish_interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
				std::chrono::duration<double>(1.0 / max_publish_rate_hz_));
		}

		odometry_publisher_ = create_publisher<Odometry>(output_topic_, 10);
		const auto odom_qos = rclcpp::SensorDataQoS().keep_last(5);
		odometry_subscription_ = create_subscription<Odometry>(
			input_topic_, odom_qos,
			std::bind(&FastlioMavrosOdometryBridge::odometry_callback, this, std::placeholders::_1));
		if (timing_alignment_enabled_) {
			frlio_anchor_age_subscription_ = create_subscription<std_msgs::msg::Float64>(
				frlio_anchor_age_topic_, odom_qos,
				[this](const std_msgs::msg::Float64::SharedPtr msg) {
					last_frlio_anchor_age_s_.store(msg->data);
				});
		}
		uniqueness_timer_ = create_wall_timer(
			std::chrono::seconds(1),
			std::bind(&FastlioMavrosOdometryBridge::check_single_ev_writer, this));

		RCLCPP_INFO(get_logger(),
			"Bridging %s -> %s as MAVLink ODOMETRY at %.1f Hz (timing_alignment=%s)",
			input_topic_.c_str(), output_topic_.c_str(), max_publish_rate_hz_,
			timing_alignment_enabled_ ? "true" : "false");
	}

private:
	rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr frlio_anchor_age_subscription_;
	rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
	rclcpp::TimerBase::SharedPtr uniqueness_timer_;
	std::string input_topic_;
	std::string output_topic_;
	std::string world_frame_id_;
	std::string body_frame_id_;
	std::string frlio_anchor_age_topic_;
	bool restamp_message_{false};
	double max_publish_rate_hz_{50.0};
	std::chrono::steady_clock::duration min_publish_interval_{};
	std::chrono::steady_clock::time_point last_publish_time_{};
	double world_yaw_alignment_rad_{0.0};
	double body_to_sensor_x_m_{0.0};
	double body_to_sensor_y_m_{0.0};
	double body_to_sensor_z_m_{0.0};
	double body_to_fastlio_yaw_rad_{0.0};
	bool duplicate_writer_{false};
	bool timing_alignment_enabled_{false};
	std::atomic<double> last_frlio_anchor_age_s_{0.0};
	std::uint64_t timing_sequence_{0};
	double last_timing_publish_steady_s_{0.0};

	void check_single_ev_writer()
	{
		if (duplicate_writer_) {
			return;
		}
		if (count_publishers(output_topic_) > 1 ||
			count_publishers("/mavros/vision_pose/pose_cov") > 0 ||
			count_publishers("/fmu/in/vehicle_visual_odometry") > 0) {
			duplicate_writer_ = true;
			RCLCPP_FATAL(get_logger(), "Duplicate PX4 external-vision writer detected; stopping ODOMETRY output");
			odometry_publisher_.reset();
		}
	}

	void odometry_callback(const Odometry::SharedPtr msg)
	{
		if (duplicate_writer_ || !odometry_publisher_) {
			return;
		}
		const auto publish_time = std::chrono::steady_clock::now();
		if (max_publish_rate_hz_ > 0.0 &&
			last_publish_time_ != std::chrono::steady_clock::time_point{} &&
			publish_time - last_publish_time_ < min_publish_interval_) {
			return;
		}
		last_publish_time_ = publish_time;

		const px4_ros_com::lever_arm::Vector3 velocity_fastlio{
			msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
		if (!px4_ros_com::lever_arm::finite(velocity_fastlio)) {
			RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
				"Rejecting EV ODOMETRY with non-finite linear velocity");
			return;
		}

		Odometry output{};
		output.header = msg->header;
		if (restamp_message_) {
			output.header.stamp = now();
		}
		output.header.frame_id = world_frame_id_;
		output.child_frame_id = body_frame_id_;
		output.pose = msg->pose;
		output.twist = msg->twist;

		const px4_ros_com::lever_arm::Vector3 body_to_sensor{
			body_to_sensor_x_m_, body_to_sensor_y_m_, body_to_sensor_z_m_};
		const auto body_to_fastlio = px4_ros_com::lever_arm::rotate_z(
			body_to_sensor, body_to_fastlio_yaw_rad_);
		const bool compensate_lever_arm =
			std::hypot(std::hypot(body_to_fastlio[0], body_to_fastlio[1]), body_to_fastlio[2]) > 1e-9;
		if (compensate_lever_arm) {
			try {
				const auto &position = msg->pose.pose.position;
				const auto &orientation = msg->pose.pose.orientation;
				const auto body_position = px4_ros_com::lever_arm::body_position_from_sensor(
					{{position.x, position.y, position.z}},
					{orientation.x, orientation.y, orientation.z, orientation.w}, body_to_fastlio);
				output.pose.pose.position.x = body_position[0];
				output.pose.pose.position.y = body_position[1];
				output.pose.pose.position.z = body_position[2];
				output.pose.covariance = px4_ros_com::lever_arm::body_pose_covariance_from_sensor(
					msg->pose.covariance,
					{orientation.x, orientation.y, orientation.z, orientation.w}, body_to_fastlio);
			} catch (const std::invalid_argument &error) {
				RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
					"Rejecting EV ODOMETRY pose: %s", error.what());
				return;
			}
		}

		if (std::fabs(world_yaw_alignment_rad_) > 1e-9) {
			const double c = std::cos(world_yaw_alignment_rad_);
			const double s = std::sin(world_yaw_alignment_rad_);
			const double x = output.pose.pose.position.x;
			const double y = output.pose.pose.position.y;
			output.pose.pose.position.x = c * x - s * y;
			output.pose.pose.position.y = s * x + c * y;
			output.pose.pose.orientation = quaternion_multiply(
				yaw_quaternion(world_yaw_alignment_rad_), output.pose.pose.orientation);
			output.pose.covariance = rotate_covariance_xy(
				output.pose.covariance, world_yaw_alignment_rad_, world_yaw_alignment_rad_);
		}

		if (std::fabs(body_to_fastlio_yaw_rad_) > 1e-9) {
			try {
				const auto body_orientation = px4_ros_com::lever_arm::postmultiply_yaw(
					{output.pose.pose.orientation.x, output.pose.pose.orientation.y,
					 output.pose.pose.orientation.z, output.pose.pose.orientation.w},
					body_to_fastlio_yaw_rad_);
				output.pose.pose.orientation.x = body_orientation.x;
				output.pose.pose.orientation.y = body_orientation.y;
				output.pose.pose.orientation.z = body_orientation.z;
				output.pose.pose.orientation.w = body_orientation.w;
				output.pose.covariance = rotate_covariance_xy(
					output.pose.covariance, 0.0, -body_to_fastlio_yaw_rad_);
			} catch (const std::invalid_argument &error) {
				RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
					"Rejecting EV ODOMETRY attitude: %s", error.what());
				return;
			}
		}

		// ODOMETRY twist is in child_frame_id.  Convert FAST-LIO child FLU to the
		// compensated aircraft base_link FLU.  MAVROS then applies the same final
		// pose to obtain the world velocity published by the old vision_speed path.
		const double c = std::cos(body_to_fastlio_yaw_rad_);
		const double s = std::sin(body_to_fastlio_yaw_rad_);
		const px4_ros_com::lever_arm::Matrix3 fastlio_to_body{{
			{{c, s, 0.0}},
			{{-s, c, 0.0}},
			{{0.0, 0.0, 1.0}},
		}};
		const auto velocity_body = px4_ros_com::lever_arm::multiply(
			fastlio_to_body, velocity_fastlio);
		output.twist.twist.linear.x = velocity_body[0];
		output.twist.twist.linear.y = velocity_body[1];
		output.twist.twist.linear.z = velocity_body[2];
		output.twist.covariance = rotate_twist_covariance_linear(
			output.twist.covariance, fastlio_to_body);

		odometry_publisher_->publish(output);
		if (timing_alignment_enabled_) {
			const double published_steady_s = steady_clock_seconds();
			const auto gap_us = static_cast<std::uint64_t>(
				last_timing_publish_steady_s_ > 0.0 ?
				(published_steady_s - last_timing_publish_steady_s_) * 1e6 : 0.0);
			last_timing_publish_steady_s_ = published_steady_s;
			RCLCPP_INFO(
				get_logger(),
				"FRLIO_TIMING stage=mavros_odometry_publish steady_clock_now=%.9f "
				"sensor_timestamp=%.9f sequence=%llu anchor_age=%.6f queue_size=0 "
				"lidar_queue=0 imu_queue=0 mutex_wait_us=0 mutex_hold_us=0 "
				"gap_us=%llu buffer_mutex_wait_us=0 buffer_mutex_hold_us=0",
				published_steady_s, stamp_to_seconds(output.header.stamp),
				static_cast<unsigned long long>(++timing_sequence_),
				last_frlio_anchor_age_s_.load(),
				static_cast<unsigned long long>(gap_us));
		}
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FastlioMavrosOdometryBridge>());
	rclcpp::shutdown();
	return 0;
}
