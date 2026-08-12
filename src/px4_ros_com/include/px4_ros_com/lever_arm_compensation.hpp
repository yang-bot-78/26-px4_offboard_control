#pragma once

#include <array>
#include <cstddef>
#include <cmath>
#include <stdexcept>

namespace px4_ros_com
{
namespace lever_arm
{

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;
using Covariance6 = std::array<double, 36>;

struct QuaternionXyzw
{
	double x;
	double y;
	double z;
	double w;
};

inline bool finite(const Vector3 &value)
{
	return std::isfinite(value[0]) && std::isfinite(value[1]) &&
	       std::isfinite(value[2]);
}

inline bool same_world_yaw_alignment(
	double position_yaw_rad,
	double attitude_yaw_rad,
	double tolerance = 1e-12)
{
	return std::isfinite(position_yaw_rad) && std::isfinite(attitude_yaw_rad) &&
	       std::isfinite(tolerance) && tolerance >= 0.0 &&
	       std::fabs(position_yaw_rad - attitude_yaw_rad) <= tolerance;
}

inline QuaternionXyzw quaternion_multiply(
	const QuaternionXyzw &a,
	const QuaternionXyzw &b)
{
	return {
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline QuaternionXyzw postmultiply_yaw(
	const QuaternionXyzw &input,
	double yaw_rad)
{
	if (!std::isfinite(yaw_rad)) {
		throw std::invalid_argument("fixed attitude rotation requires a finite yaw");
	}
	const double norm = std::sqrt(
		input.x * input.x + input.y * input.y + input.z * input.z + input.w * input.w);
	if (!std::isfinite(norm) || norm <= 1e-12) {
		throw std::invalid_argument("fixed attitude rotation requires a finite quaternion");
	}
	const QuaternionXyzw normalized{
		input.x / norm, input.y / norm, input.z / norm, input.w / norm};
	const QuaternionXyzw fixed_yaw{
		0.0, 0.0, std::sin(yaw_rad * 0.5), std::cos(yaw_rad * 0.5)};
	return quaternion_multiply(normalized, fixed_yaw);
}

inline Matrix3 child_to_world_rotation(const QuaternionXyzw &input)
{
	const double norm = std::sqrt(
		input.x * input.x + input.y * input.y + input.z * input.z + input.w * input.w);
	if (!std::isfinite(norm) || norm <= 1e-12) {
		throw std::invalid_argument("lever-arm compensation requires a finite quaternion");
	}

	const double x = input.x / norm;
	const double y = input.y / norm;
	const double z = input.z / norm;
	const double w = input.w / norm;
	return {{
		{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)}},
		{{2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)}},
		{{2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}},
	}};
}

inline Vector3 multiply(const Matrix3 &matrix, const Vector3 &vector)
{
	Vector3 output{};
	for (std::size_t row = 0; row < 3; ++row) {
		for (std::size_t column = 0; column < 3; ++column) {
			output[row] += matrix[row][column] * vector[column];
		}
	}
	return output;
}

// Rotate a body-frame vector into the FAST-LIO child frame using the fixed
// installation yaw R_FASTLIO_body.
inline Vector3 rotate_z(const Vector3 &vector, double yaw_rad)
{
	if (!finite(vector) || !std::isfinite(yaw_rad)) {
		throw std::invalid_argument("fixed frame rotation requires finite inputs");
	}
	const double c = std::cos(yaw_rad);
	const double s = std::sin(yaw_rad);
	return {
		c * vector[0] - s * vector[1],
		s * vector[0] + c * vector[1],
		vector[2]};
}

// body_to_sensor is the control-origin to pose-origin lever arm expressed in
// the odometry child frame. The caller converts any body-FLU measurement into
// that child frame before using this function. FAST-LIO reports the child
// origin, so subtract its world-frame lever arm to recover the control origin.
inline Vector3 body_position_from_sensor(
	const Vector3 &sensor_position_world,
	const QuaternionXyzw &sensor_orientation_world,
	const Vector3 &body_to_sensor)
{
	if (!finite(sensor_position_world) || !finite(body_to_sensor)) {
		throw std::invalid_argument("lever-arm compensation requires finite vectors");
	}
	const Vector3 lever_world = multiply(
		child_to_world_rotation(sensor_orientation_world), body_to_sensor);
	return {
		sensor_position_world[0] - lever_world[0],
		sensor_position_world[1] - lever_world[1],
		sensor_position_world[2] - lever_world[2],
	};
}

inline Covariance6 body_pose_covariance_from_sensor(
	const Covariance6 &sensor_covariance,
	const QuaternionXyzw &sensor_orientation_world,
	const Vector3 &body_to_sensor)
{
	if (!finite(body_to_sensor)) {
		throw std::invalid_argument("lever-arm compensation requires a finite translation");
	}
	for (double value : sensor_covariance) {
		if (!std::isfinite(value)) {
			throw std::invalid_argument("lever-arm compensation requires finite covariance");
		}
	}

	const Matrix3 rotation = child_to_world_rotation(sensor_orientation_world);
	const double x = body_to_sensor[0];
	const double y = body_to_sensor[1];
	const double z = body_to_sensor[2];
	const Matrix3 skew{{
		{{0.0, -z, y}},
		{{z, 0.0, -x}},
		{{-y, x, 0.0}},
	}};

	// For p_body = p_sensor - R * r, a small child-frame attitude error gives
	// dp_body = dp_sensor + R * skew(r) * dtheta.
	double jacobian[6][6]{};
	for (std::size_t index = 0; index < 6; ++index) {
		jacobian[index][index] = 1.0;
	}
	for (std::size_t row = 0; row < 3; ++row) {
		for (std::size_t column = 0; column < 3; ++column) {
			for (std::size_t inner = 0; inner < 3; ++inner) {
				jacobian[row][column + 3] += rotation[row][inner] * skew[inner][column];
			}
		}
	}

	Covariance6 output{};
	for (std::size_t row = 0; row < 6; ++row) {
		for (std::size_t column = 0; column < 6; ++column) {
			for (std::size_t left = 0; left < 6; ++left) {
				for (std::size_t right = 0; right < 6; ++right) {
					output[row * 6 + column] += jacobian[row][left] *
						sensor_covariance[left * 6 + right] * jacobian[column][right];
				}
			}
		}
	}
	return output;
}

} // namespace lever_arm
} // namespace px4_ros_com
