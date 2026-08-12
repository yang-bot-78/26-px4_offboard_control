#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros_com/lever_arm_compensation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;

namespace
{

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
			double value = 0.0;
			for (std::size_t left = 0; left < 6; ++left) {
				for (std::size_t right = 0; right < 6; ++right) {
					value += jacobian[row][left] * input[left * 6 + right] *
						jacobian[column][right];
				}
			}
			output[row * 6 + column] = value;
		}
	}
	return output;
}

// Rotate the linear triple of a twist covariance with a full 3D rotation.  The
// jacobian is block diagonal: R for the linear triple, identity for the angular
// one.  FAST-LIO reports NaN angular rates behind an isotropic 1e6 variance, so
// rotating that block would add no information while risking that the "unknown"
// marker stops looking isotropic.  Keeping it identity leaves the unknown
// marked unknown, and still transforms the linear-angular cross terms
// correctly.
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
			double value = 0.0;
			for (std::size_t left = 0; left < 6; ++left) {
				for (std::size_t right = 0; right < 6; ++right) {
					value += jacobian[row][left] * input[left * 6 + right] *
						jacobian[column][right];
				}
			}
			output[row * 6 + column] = value;
		}
	}
	return output;
}

} // namespace

class FastlioMavrosVisionBridge : public rclcpp::Node
{
public:
	FastlioMavrosVisionBridge() : Node("fastlio_mavros_vision_bridge")
	{
		input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
		pose_topic_ = declare_parameter<std::string>("pose_topic", "/mavros/vision_pose/pose_cov");
		speed_topic_ = declare_parameter<std::string>("speed_topic", "/mavros/vision_speed/speed_twist_cov");
		force_pose_frame_id_ = declare_parameter<std::string>("force_pose_frame_id", "odom");
		// The published twist is rotated into the local world frame, so the
		// declared frame must be the world frame, not base_link.  MAVROS
		// vision_speed interprets the vector as local ENU either way; a
		// base_link default would only mislead the next reader.
		force_twist_frame_id_ = declare_parameter<std::string>("force_twist_frame_id", "odom");
		restamp_message_ = declare_parameter<bool>("restamp_message", false);
		publish_speed_ = declare_parameter<bool>("publish_speed", false);
		world_yaw_alignment_rad_ = declare_parameter<double>(
			"world_yaw_alignment_rad", 0.0);
		body_to_sensor_x_m_ = declare_parameter<double>("body_to_sensor_x_m", 0.0);
		body_to_sensor_y_m_ = declare_parameter<double>("body_to_sensor_y_m", 0.0);
		body_to_sensor_z_m_ = declare_parameter<double>("body_to_sensor_z_m", 0.0);
		body_to_fastlio_yaw_rad_ = declare_parameter<double>(
			"body_to_fastlio_yaw_rad", 0.0);
		const px4_ros_com::lever_arm::Vector3 lever_arm{
			body_to_sensor_x_m_, body_to_sensor_y_m_, body_to_sensor_z_m_};
		if (!px4_ros_com::lever_arm::finite(lever_arm) ||
		    !std::isfinite(body_to_fastlio_yaw_rad_) ||
		    !std::isfinite(world_yaw_alignment_rad_)) {
			throw std::invalid_argument(
				"installation and world_yaw_alignment_rad parameters must be finite");
		}

		pose_publisher_ = create_publisher<PoseWithCovarianceStamped>(pose_topic_, 10);

		if (publish_speed_) {
			speed_publisher_ = create_publisher<TwistWithCovarianceStamped>(speed_topic_, 10);
		}

		odometry_subscription_ = create_subscription<Odometry>(
			input_topic_, 10,
			std::bind(&FastlioMavrosVisionBridge::odometry_callback, this, std::placeholders::_1));

		RCLCPP_INFO(
			get_logger(),
			"Bridging %s -> %s%s for MAVROS vision input; body->FAST-LIO origin "
			"in body FLU=(%.4f, %.4f, %.4f) m, R_FASTLIO_body yaw=%.6f rad, "
			"world yaw alignment=%.6f rad",
			input_topic_.c_str(),
			pose_topic_.c_str(),
			publish_speed_ ? " + vision_speed" : "",
			body_to_sensor_x_m_,
			body_to_sensor_y_m_,
			body_to_sensor_z_m_,
			body_to_fastlio_yaw_rad_,
			world_yaw_alignment_rad_);
	}

private:
	rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
	rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
	rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr speed_publisher_;

	std::string input_topic_;
	std::string pose_topic_;
	std::string speed_topic_;
	std::string force_pose_frame_id_;
	std::string force_twist_frame_id_;
	bool restamp_message_{false};
	bool publish_speed_{false};
	double world_yaw_alignment_rad_{0.0};
	double body_to_sensor_x_m_{0.0};
	double body_to_sensor_y_m_{0.0};
	double body_to_sensor_z_m_{0.0};
	double body_to_fastlio_yaw_rad_{0.0};

	void odometry_callback(const Odometry::SharedPtr msg)
	{
		PoseWithCovarianceStamped pose_msg{};
		pose_msg.header = msg->header;

		if (restamp_message_) {
			pose_msg.header.stamp = now();
		}

		if (!force_pose_frame_id_.empty()) {
			pose_msg.header.frame_id = force_pose_frame_id_;
		}

		pose_msg.pose = msg->pose;

		const px4_ros_com::lever_arm::Vector3 body_to_sensor{
			body_to_sensor_x_m_, body_to_sensor_y_m_, body_to_sensor_z_m_};
		// The measured lever arm is in aircraft body FLU.  FAST-LIO's child
		// axes have a fixed installation rotation, so use the same
		// R_FASTLIO_body for both the lever-arm position and output attitude.
		const auto body_to_fastlio = px4_ros_com::lever_arm::rotate_z(
			body_to_sensor, body_to_fastlio_yaw_rad_);
		const bool compensate_lever_arm =
			std::hypot(std::hypot(body_to_fastlio[0], body_to_fastlio[1]), body_to_fastlio[2]) > 1e-9;
		if (compensate_lever_arm) {
			const auto &position = msg->pose.pose.position;
			const auto &orientation = msg->pose.pose.orientation;
			try {
				const auto body_position = px4_ros_com::lever_arm::body_position_from_sensor(
					{{position.x, position.y, position.z}},
					{orientation.x, orientation.y, orientation.z, orientation.w},
					body_to_fastlio);
				pose_msg.pose.pose.position.x = body_position[0];
				pose_msg.pose.pose.position.y = body_position[1];
				pose_msg.pose.pose.position.z = body_position[2];
				pose_msg.pose.covariance =
					px4_ros_com::lever_arm::body_pose_covariance_from_sensor(
						msg->pose.covariance,
						{orientation.x, orientation.y, orientation.z, orientation.w},
						body_to_fastlio);
			} catch (const std::invalid_argument &error) {
				RCLCPP_ERROR_THROTTLE(
					get_logger(), *get_clock(), 1000,
					"Rejecting EV pose: %s", error.what());
				return;
			}
		}

		if (std::fabs(world_yaw_alignment_rad_) > 1e-9) {
			const double c = std::cos(world_yaw_alignment_rad_);
			const double s = std::sin(world_yaw_alignment_rad_);
			const double x = pose_msg.pose.pose.position.x;
			const double y = pose_msg.pose.pose.position.y;
			pose_msg.pose.pose.position.x = c * x - s * y;
			pose_msg.pose.pose.position.y = s * x + c * y;
		}

		if (std::fabs(world_yaw_alignment_rad_) > 1e-9) {
			const auto offset_q = yaw_quaternion(world_yaw_alignment_rad_);
			pose_msg.pose.pose.orientation =
				quaternion_multiply(offset_q, pose_msg.pose.pose.orientation);
		}

		if (std::fabs(body_to_fastlio_yaw_rad_) > 1e-9) {
			// R_world_body = R_world_fastlio * R_fastlio_body.
			// This is a child-frame installation rotation, hence the
			// postmultiplication; world alignment remains a world-frame offset.
			try {
				const auto body_orientation = px4_ros_com::lever_arm::postmultiply_yaw(
					{pose_msg.pose.pose.orientation.x,
					 pose_msg.pose.pose.orientation.y,
					 pose_msg.pose.pose.orientation.z,
					 pose_msg.pose.pose.orientation.w},
					body_to_fastlio_yaw_rad_);
				pose_msg.pose.pose.orientation.x = body_orientation.x;
				pose_msg.pose.pose.orientation.y = body_orientation.y;
				pose_msg.pose.pose.orientation.z = body_orientation.z;
				pose_msg.pose.pose.orientation.w = body_orientation.w;
			} catch (const std::invalid_argument &error) {
				RCLCPP_ERROR_THROTTLE(
					get_logger(), *get_clock(), 1000,
					"Rejecting EV attitude: %s", error.what());
				return;
			}
		}

		if (std::fabs(world_yaw_alignment_rad_) > 1e-9) {
			pose_msg.pose.covariance = rotate_covariance_xy(
				pose_msg.pose.covariance,
				world_yaw_alignment_rad_,
				world_yaw_alignment_rad_);
		}

		if (std::fabs(body_to_fastlio_yaw_rad_) > 1e-9) {
			// Orientation covariance is expressed in the child frame by the
			// MAVROS contract; rotate it into the body frame with R_body_fastlio.
			pose_msg.pose.covariance = rotate_covariance_xy(
				pose_msg.pose.covariance, 0.0, -body_to_fastlio_yaw_rad_);
		}

		pose_publisher_->publish(pose_msg);

		if (publish_speed_ && speed_publisher_) {
			TwistWithCovarianceStamped speed_msg{};
			speed_msg.header = msg->header;

			if (restamp_message_) {
				speed_msg.header.stamp = pose_msg.header.stamp;
			}

			if (!force_twist_frame_id_.empty()) {
				speed_msg.header.frame_id = force_twist_frame_id_;
			}

			speed_msg.twist = msg->twist;

			// FAST-LIO publishes twist in child_frame_id per REP-147, so the
			// linear velocity arrives in the FAST-LIO child frame while the
			// MAVROS vision_speed plugin interprets the vector as local world
			// ENU.  Rotate with the raw child->world attitude from this same
			// message: one sample carries both, so there is no synchronization
			// error, and the child-frame installation yaw cancels out (the
			// twist never entered body FLU, so R_FASTLIO_body must not be
			// applied here the way the pose path applies it to attitude).
			// The world yaw alignment is applied afterwards, exactly as for
			// position.  Angular rates stay untouched: FAST-LIO marks them NaN
			// on purpose and rotating an unknown produces another unknown.
			try {
				const auto child_to_world =
					px4_ros_com::lever_arm::child_to_world_rotation(
						{msg->pose.pose.orientation.x,
						 msg->pose.pose.orientation.y,
						 msg->pose.pose.orientation.z,
						 msg->pose.pose.orientation.w});
				const px4_ros_com::lever_arm::Vector3 velocity_child{
					msg->twist.twist.linear.x,
					msg->twist.twist.linear.y,
					msg->twist.twist.linear.z};
				if (!px4_ros_com::lever_arm::finite(velocity_child)) {
					throw std::invalid_argument(
						"non-finite linear velocity");
				}
				const auto velocity_world = px4_ros_com::lever_arm::multiply(
					child_to_world, velocity_child);
				speed_msg.twist.twist.linear.x = velocity_world[0];
				speed_msg.twist.twist.linear.y = velocity_world[1];
				speed_msg.twist.twist.linear.z = velocity_world[2];
				speed_msg.twist.covariance = rotate_twist_covariance_linear(
					speed_msg.twist.covariance, child_to_world);
			} catch (const std::invalid_argument &error) {
				// Drop the sample rather than hand EKF2 a velocity whose frame
				// is unknown.  The pose path above already published, so
				// position/height/yaw fusion is unaffected.
				RCLCPP_ERROR_THROTTLE(
					get_logger(), *get_clock(), 1000,
					"Rejecting EV velocity: %s", error.what());
				return;
			}

			if (std::fabs(world_yaw_alignment_rad_) > 1e-9) {
				const double c = std::cos(world_yaw_alignment_rad_);
				const double s = std::sin(world_yaw_alignment_rad_);
				const double vx = speed_msg.twist.twist.linear.x;
				const double vy = speed_msg.twist.twist.linear.y;
				speed_msg.twist.twist.linear.x = c * vx - s * vy;
				speed_msg.twist.twist.linear.y = s * vx + c * vy;
				speed_msg.twist.covariance = rotate_covariance_xy(
					speed_msg.twist.covariance,
					world_yaw_alignment_rad_,
					0.0);
			}

			speed_publisher_->publish(speed_msg);
		}
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FastlioMavrosVisionBridge>());
	rclcpp::shutdown();
	return 0;
}
