#include <gtest/gtest.h>

#include "px4_ros_com/lever_arm_compensation.hpp"

#include <cmath>

namespace lever = px4_ros_com::lever_arm;

TEST(LeverArmCompensation, IdentityOrientationSubtractsMeasuredBodyToSensorVector)
{
	const auto body = lever::body_position_from_sensor(
		{{1.25, -0.10, 0.40}},
		{0.0, 0.0, 0.0, 1.0},
		{{0.25, -0.10, 0.40}});
	EXPECT_NEAR(body[0], 1.0, 1e-12);
	EXPECT_NEAR(body[1], 0.0, 1e-12);
	EXPECT_NEAR(body[2], 0.0, 1e-12);
}

TEST(LeverArmCompensation, YawRotationRemovesCircularSensorMotion)
{
	const double half_yaw = std::acos(-1.0) / 4.0;
	const auto body = lever::body_position_from_sensor(
		{{0.0, 0.25, 0.0}},
		{0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)},
		{{0.25, 0.0, 0.0}});
	EXPECT_NEAR(body[0], 0.0, 1e-12);
	EXPECT_NEAR(body[1], 0.0, 1e-12);
	EXPECT_NEAR(body[2], 0.0, 1e-12);
}

TEST(LeverArmCompensation, FixedFastlioYawRotatesBodyLeverArm)
{
	const double yaw = std::acos(-1.0) / 2.0;
	const auto fastlio = lever::rotate_z({{1.0, 0.0, 0.25}}, yaw);
	EXPECT_NEAR(fastlio[0], 0.0, 1e-12);
	EXPECT_NEAR(fastlio[1], 1.0, 1e-12);
	EXPECT_NEAR(fastlio[2], 0.25, 1e-12);
}

TEST(LeverArmCompensation, CurrentZeroYawPreservesVerticalLeverArm)
{
	const auto fastlio = lever::rotate_z({{0.0, 0.0, 0.08}}, 0.0);
	EXPECT_NEAR(fastlio[0], 0.0, 1e-12);
	EXPECT_NEAR(fastlio[1], 0.0, 1e-12);
	EXPECT_NEAR(fastlio[2], 0.08, 1e-12);
}

TEST(LeverArmCompensation, FixedFastlioYawIsPostmultipliedIntoAttitude)
{
	const double half_right_angle = std::acos(-1.0) / 4.0;
	const lever::QuaternionXyzw roll_90{
		std::sin(half_right_angle), 0.0, 0.0, std::cos(half_right_angle)};
	const auto body = lever::postmultiply_yaw(roll_90, std::acos(-1.0) / 2.0);
	EXPECT_NEAR(body.x, 0.5, 1e-12);
	EXPECT_NEAR(body.y, -0.5, 1e-12);
	EXPECT_NEAR(body.z, 0.5, 1e-12);
	EXPECT_NEAR(body.w, 0.5, 1e-12);
}

TEST(LeverArmCompensation, WorldPositionAndAttitudeYawMustMatch)
{
	EXPECT_TRUE(lever::same_world_yaw_alignment(0.25, 0.25));
	EXPECT_FALSE(lever::same_world_yaw_alignment(0.25, -0.25));
	EXPECT_FALSE(lever::same_world_yaw_alignment(NAN, NAN));
}

TEST(LeverArmCompensation, PoseCovarianceIncludesYawLeverArmUncertainty)
{
	lever::Covariance6 covariance{};
	for (std::size_t index = 0; index < 6; ++index) {
		covariance[index * 6 + index] = 0.01;
	}
	covariance[35] = 0.04;

	const auto shifted = lever::body_pose_covariance_from_sensor(
		covariance,
		{0.0, 0.0, 0.0, 1.0},
		{{1.0, 0.0, 0.0}});
	EXPECT_NEAR(shifted[0], 0.01, 1e-12);
	EXPECT_NEAR(shifted[7], 0.05, 1e-12);
	EXPECT_NEAR(shifted[35], 0.04, 1e-12);
}

TEST(LeverArmCompensation, InvalidQuaternionFailsClosed)
{
	EXPECT_THROW(
		lever::body_position_from_sensor(
			{{0.0, 0.0, 0.0}},
			{0.0, 0.0, 0.0, 0.0},
			{{0.25, 0.0, 0.0}}),
		std::invalid_argument);
}
