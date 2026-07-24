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

namespace
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;

std::array<double, 36> rotate_covariance_xy(
	const std::array<double, 36> &input,
	double linear_yaw_rad,
	double angular_yaw_rad)
{
	Matrix6d covariance;
	for (std::size_t row = 0; row < 6; ++row) {
		for (std::size_t column = 0; column < 6; ++column) {
			covariance(row, column) = input[row * 6 + column];
		}
	}

	Matrix6d jacobian = Matrix6d::Identity();
	const Eigen::Matrix2d linear_rotation =
		Eigen::Rotation2Dd(linear_yaw_rad).toRotationMatrix();
	const Eigen::Matrix2d angular_rotation =
		Eigen::Rotation2Dd(angular_yaw_rad).toRotationMatrix();
	jacobian.block<2, 2>(0, 0) = linear_rotation;
	jacobian.block<2, 2>(3, 3) = angular_rotation;
	const Matrix6d rotated = jacobian * covariance * jacobian.transpose();

	std::array<double, 36> output{};
	for (std::size_t row = 0; row < 6; ++row) {
		for (std::size_t column = 0; column < 6; ++column) {
			output[row * 6 + column] = rotated(row, column);
		}
	}
	return output;
}

Eigen::Vector3d rotate_enu_xy(const Eigen::Vector3d &vector, double yaw_rad)
{
	return Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) * vector;
}

} // namespace

class FastlioVehicleVisualOdometry : public rclcpp::Node
{
public:
	FastlioVehicleVisualOdometry() : Node("fastlio_vehicle_visual_odometry")
	{
		input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
		output_topic_ = declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");
		quality_ = declare_parameter<int>("quality", 100);
		position_yaw_offset_rad_ = declare_parameter<double>("position_yaw_offset_rad", 0.0);
		yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);

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
	double position_yaw_offset_rad_{0.0};
	double yaw_offset_rad_{0.0};

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
		out.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;

		const Eigen::Vector3d position_enu(
			msg->pose.pose.position.x,
			msg->pose.pose.position.y,
			msg->pose.pose.position.z);
		const Eigen::Vector3d aligned_position_enu =
			rotate_enu_xy(position_enu, position_yaw_offset_rad_);
		const Eigen::Vector3d position_ned =
			px4_ros_com::frame_transforms::enu_to_ned_local_frame(aligned_position_enu);
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
		const Eigen::Quaterniond aligned_ros_q =
			Eigen::AngleAxisd(yaw_offset_rad_, Eigen::Vector3d::UnitZ()) * ros_q;
		const Eigen::Quaterniond px4_q =
			px4_ros_com::frame_transforms::ros_to_px4_orientation(aligned_ros_q).normalized();
		px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(px4_q, out.q);

		const Eigen::Vector3d linear_vel_enu(
			msg->twist.twist.linear.x,
			msg->twist.twist.linear.y,
			msg->twist.twist.linear.z);
		const Eigen::Vector3d aligned_linear_vel_enu =
			rotate_enu_xy(linear_vel_enu, position_yaw_offset_rad_);
		const Eigen::Vector3d linear_vel_ned =
			px4_ros_com::frame_transforms::enu_to_ned_local_frame(aligned_linear_vel_enu);
		out.velocity = {
			static_cast<float>(linear_vel_ned.x()),
			static_cast<float>(linear_vel_ned.y()),
			static_cast<float>(linear_vel_ned.z()),
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

		const auto aligned_pose_cov_enu = rotate_covariance_xy(
			msg->pose.covariance,
			position_yaw_offset_rad_,
			yaw_offset_rad_);
		const auto pose_cov_ned =
			px4_ros_com::frame_transforms::enu_to_ned_local_frame(aligned_pose_cov_enu);
		const auto aligned_twist_cov_enu = rotate_covariance_xy(
			msg->twist.covariance,
			position_yaw_offset_rad_,
			0.0);
		const auto twist_cov_ned =
			px4_ros_com::frame_transforms::enu_to_ned_local_frame(aligned_twist_cov_enu);

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
			variance_or_nan(twist_cov_ned, 0),
			variance_or_nan(twist_cov_ned, 7),
			variance_or_nan(twist_cov_ned, 14),
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
