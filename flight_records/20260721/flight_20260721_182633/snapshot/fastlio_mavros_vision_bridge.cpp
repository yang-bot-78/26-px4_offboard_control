#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <memory>
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
		force_twist_frame_id_ = declare_parameter<std::string>("force_twist_frame_id", "base_link");
		restamp_message_ = declare_parameter<bool>("restamp_message", true);
		publish_speed_ = declare_parameter<bool>("publish_speed", false);
		yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);
		position_yaw_offset_rad_ = declare_parameter<double>("position_yaw_offset_rad", 0.0);

		pose_publisher_ = create_publisher<PoseWithCovarianceStamped>(pose_topic_, 10);

		if (publish_speed_) {
			speed_publisher_ = create_publisher<TwistWithCovarianceStamped>(speed_topic_, 10);
		}

		odometry_subscription_ = create_subscription<Odometry>(
			input_topic_, 10,
			std::bind(&FastlioMavrosVisionBridge::odometry_callback, this, std::placeholders::_1));

		RCLCPP_INFO(
			get_logger(),
			"Bridging %s -> %s%s for MAVROS vision input",
			input_topic_.c_str(),
			pose_topic_.c_str(),
			publish_speed_ ? " + vision_speed" : "");
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
	bool restamp_message_{true};
	bool publish_speed_{false};
	double yaw_offset_rad_{0.0};
	double position_yaw_offset_rad_{0.0};

	void odometry_callback(const Odometry::SharedPtr msg) const
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

		if (std::fabs(position_yaw_offset_rad_) > 1e-9) {
			const double c = std::cos(position_yaw_offset_rad_);
			const double s = std::sin(position_yaw_offset_rad_);
			const double x = pose_msg.pose.pose.position.x;
			const double y = pose_msg.pose.pose.position.y;
			pose_msg.pose.pose.position.x = c * x - s * y;
			pose_msg.pose.pose.position.y = s * x + c * y;
		}

		if (std::fabs(yaw_offset_rad_) > 1e-9) {
			const auto offset_q = yaw_quaternion(yaw_offset_rad_);
			pose_msg.pose.pose.orientation =
				quaternion_multiply(offset_q, pose_msg.pose.pose.orientation);
		}

		if (std::fabs(position_yaw_offset_rad_) > 1e-9 ||
		    std::fabs(yaw_offset_rad_) > 1e-9) {
			pose_msg.pose.covariance = rotate_covariance_xy(
				pose_msg.pose.covariance,
				position_yaw_offset_rad_,
				yaw_offset_rad_);
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

			if (std::fabs(position_yaw_offset_rad_) > 1e-9) {
				const double c = std::cos(position_yaw_offset_rad_);
				const double s = std::sin(position_yaw_offset_rad_);
				const double vx = speed_msg.twist.twist.linear.x;
				const double vy = speed_msg.twist.twist.linear.y;
				speed_msg.twist.twist.linear.x = c * vx - s * vy;
				speed_msg.twist.twist.linear.y = s * vx + c * vy;
				speed_msg.twist.covariance = rotate_covariance_xy(
					speed_msg.twist.covariance,
					position_yaw_offset_rad_,
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
