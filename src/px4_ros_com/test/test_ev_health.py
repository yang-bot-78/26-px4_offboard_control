import math

import pytest

from px4_ros_com.ev_health import (
    compensate_sensor_to_body_lever_arm,
    covariance_3x3_is_symmetric_psd,
    EvHealthMonitorCore,
    HealthConfig,
    HealthState,
    ev_enu_to_px4_ned,
    quaternion_rotation_matrix_xyzw,
    rotate_covariance_3x3,
    rotate_vector,
)


def config(**overrides):
    values = dict(
        recovery_healthy_s=0.0,
        max_input_age_s=0.25,
        max_position_jump_m=0.15,
        max_horizontal_velocity_difference_mps=0.45,
        anomaly_to_fault_s=0.3,
        velocity_lowpass_cutoff_hz=0.0,
    )
    values.update(overrides)
    return HealthConfig(**values)


def add_frame(core, stamp, x, *, px4_velocity_enu=(0.0, 0.0, 0.0), age=0.01):
    receive = stamp + age
    core.update_px4_velocity(
        px4_velocity_enu,
        receive,
        input_is_enu=True,
        sample_stamp_s=stamp,
    )
    return core.process_ev(
        stamp_s=stamp,
        receive_time_s=receive,
        raw_position_enu=(x, 0.0, 0.0),
        covariance_xyz=(0.01, 0.01, 0.01),
    )


def test_normal_forward_motion_has_matching_positive_sign():
    core = EvHealthMonitorCore(config())
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(0.5, 0.0, 0.0))
    result = add_frame(core, 1.1, 0.05, px4_velocity_enu=(0.5, 0.0, 0.0))

    # With the current zero yaw offset, EV ENU +X is PX4 NED +Y.
    assert result.velocity_px4_ned[1] == pytest.approx(0.5)
    assert result.metrics.px4_velocity_px4_ned[1] == pytest.approx(0.5)
    assert result.state == HealthState.HEALTHY


def test_normal_backward_motion_has_matching_negative_sign():
    core = EvHealthMonitorCore(config())
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(-0.4, 0.0, 0.0))
    result = add_frame(core, 1.1, -0.04, px4_velocity_enu=(-0.4, 0.0, 0.0))

    assert result.velocity_px4_ned[1] == pytest.approx(-0.4)
    assert result.metrics.px4_velocity_px4_ned[1] == pytest.approx(-0.4)
    assert result.state == HealthState.HEALTHY


def test_internal_velocity_matches_position_derivative_in_common_frame():
    core = EvHealthMonitorCore(config())
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(0.5, 0.0, 0.0))
    core.update_px4_velocity(
        (0.5, 0.0, 0.0), 1.11, input_is_enu=True, sample_stamp_s=1.1
    )
    result = core.process_ev(
        stamp_s=1.1,
        receive_time_s=1.11,
        raw_position_enu=(0.05, 0.0, 0.0),
        internal_velocity_px4_ned=(0.0, 0.5, 0.0),
    )
    assert result.state == HealthState.HEALTHY
    assert result.metrics.internal_velocity_difference_mps == pytest.approx(0.0)


def test_internal_velocity_opposite_to_position_derivative_is_suspect():
    core = EvHealthMonitorCore(config(max_internal_velocity_difference_mps=0.4))
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(0.5, 0.0, 0.0))
    core.update_px4_velocity(
        (0.5, 0.0, 0.0), 1.11, input_is_enu=True, sample_stamp_s=1.1
    )
    result = core.process_ev(
        stamp_s=1.1,
        receive_time_s=1.11,
        raw_position_enu=(0.05, 0.0, 0.0),
        internal_velocity_px4_ned=(0.0, -0.5, 0.0),
    )
    assert result.state == HealthState.SUSPECT
    assert "internal_velocity_mismatch" in result.reason


def test_px4_moves_backward_while_ev_drifts_forward_faults_after_0p3s():
    core = EvHealthMonitorCore(config())
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(-0.2, 0.0, 0.0))
    states = []
    for index in range(1, 5):
        result = add_frame(
            core,
            1.0 + 0.1 * index,
            0.06 * index,
            px4_velocity_enu=(-0.2, 0.0, 0.0),
        )
        states.append(result.state)

    assert states[:3] == [HealthState.SUSPECT] * 3
    assert states[3] == HealthState.FAULT
    assert "velocity_mismatch" in result.reason
    assert not result.publish


def test_single_0p2m_jump_is_rejected_and_resynchronised():
    core = EvHealthMonitorCore(config())
    add_frame(core, 1.0, 0.0)
    jump = add_frame(core, 1.1, 0.2)
    following = add_frame(core, 1.2, 0.2)

    assert not jump.accepted
    assert not jump.publish
    assert "position_jump" in jump.reason
    assert following.accepted
    assert following.metrics.single_frame_displacement_m == pytest.approx(0.0)


@pytest.mark.parametrize(
    "second_stamp,second_x,reason",
    [
        (1.0, 0.0, "non_monotonic_stamp"),
        (0.9, 0.0, "non_monotonic_stamp"),
        (1.1, math.nan, "non_finite_ev_message"),
    ],
)
def test_zero_dt_timestamp_regression_and_nan_are_rejected(
    second_stamp, second_x, reason
):
    core = EvHealthMonitorCore(config())
    add_frame(core, 1.0, 0.0)
    result = add_frame(core, second_stamp, second_x)

    assert not result.accepted
    assert not result.publish
    assert reason in result.reason


def test_message_interruption_transitions_to_fault():
    core = EvHealthMonitorCore(config(message_timeout_s=0.2))
    add_frame(core, 1.0, 0.0)

    assert core.check_timeout(1.22) == HealthState.SUSPECT
    assert core.check_timeout(1.52) == HealthState.FAULT


def test_active_anomaly_reaches_fault_deadline_without_another_ev_frame():
    core = EvHealthMonitorCore(config())
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(0.0, 0.0, 0.0))
    mismatch = add_frame(
        core,
        1.1,
        0.06,
        px4_velocity_enu=(-0.2, 0.0, 0.0),
    )

    assert mismatch.state == HealthState.SUSPECT
    assert core.check_timeout(1.41) == HealthState.FAULT


def test_filtered_velocity_returns_near_zero_when_stationary():
    core = EvHealthMonitorCore(
        config(
            velocity_lowpass_cutoff_hz=2.0,
            max_horizontal_velocity_difference_mps=2.0,
        )
    )
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(0.5, 0.0, 0.0))
    add_frame(core, 1.1, 0.05, px4_velocity_enu=(0.5, 0.0, 0.0))
    for index in range(2, 15):
        result = add_frame(core, 1.0 + 0.1 * index, 0.05)

    assert math.hypot(*result.velocity_px4_ned[:2]) < 0.02


def test_coordinate_conversion_includes_nonzero_yaw_offset():
    vector = ev_enu_to_px4_ned((1.0, 0.0, 0.0), math.pi / 4.0)
    assert vector == pytest.approx((math.sqrt(0.5), math.sqrt(0.5), 0.0))


@pytest.mark.parametrize(
    "yaw_deg,body_velocity,expected_world",
    [
        (0.0, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        (90.0, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        (180.0, (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
    ],
)
def test_body_flu_velocity_rotates_to_world_at_key_yaws(
    yaw_deg, body_velocity, expected_world
):
    yaw = math.radians(yaw_deg)
    quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    rotation = quaternion_rotation_matrix_xyzw(quaternion)
    assert rotate_vector(rotation, body_velocity) == pytest.approx(
        expected_world, abs=1e-12
    )


def test_velocity_covariance_rotation_preserves_cross_terms_and_psd():
    yaw = math.pi / 2.0
    rotation = quaternion_rotation_matrix_xyzw(
        (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    )
    covariance = ((0.04, 0.01, 0.0), (0.01, 0.09, 0.0), (0.0, 0.0, 0.16))
    rotated = rotate_covariance_3x3(rotation, covariance)
    assert rotated[0][0] == pytest.approx(0.09)
    assert rotated[1][1] == pytest.approx(0.04)
    assert rotated[0][1] == pytest.approx(-0.01)
    assert rotated[1][0] == pytest.approx(-0.01)
    assert covariance_3x3_is_symmetric_psd(rotated)


def test_invalid_velocity_covariance_is_rejected():
    assert not covariance_3x3_is_symmetric_psd(
        ((0.01, 0.2, 0.0), (0.2, 0.01, 0.0), (0.0, 0.0, 0.01))
    )
    assert not covariance_3x3_is_symmetric_psd(
        ((0.0, 0.0, 0.0), (0.0, 0.01, 0.0), (0.0, 0.0, 0.01))
    )


def test_sensor_reference_point_lever_arm_compensation():
    # Body origin is 0.2 m ahead of the sensor. At +1 rad/s yaw, its velocity
    # differs by omega x r = +0.2 m/s along body-left.
    shifted = compensate_sensor_to_body_lever_arm(
        (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.2, 0.0, 0.0)
    )
    assert shifted == pytest.approx((1.0, 0.2, 0.0))


def test_unknown_angular_velocity_cannot_be_used_for_lever_arm():
    with pytest.raises(ValueError):
        compensate_sensor_to_body_lever_arm(
            (0.0, 0.0, 0.0), (math.nan, math.nan, math.nan), (0.1, 0.0, 0.0)
        )


def test_lidar_forward_pointing_vehicle_right_installation_mapping():
    # Flight yaw is about +90 deg, so body forward is PX4 local +Y and body
    # right is PX4 local -X.  If raw FAST-LIO +X really follows a lidar whose
    # front points body-right, a -90 deg position alignment maps +X to -NED X.
    installed = ev_enu_to_px4_ned((1.0, 0.0, 0.0), -math.pi / 2.0)
    assert installed == pytest.approx((-1.0, 0.0, 0.0), abs=1e-12)

    # This separately records what the current zero launch offset means.  It
    # maps raw lidar-forward to PX4 local +Y (aircraft-forward at yaw +90 deg),
    # and is only physically correct if FAST-LIO/extrinsics already align axes.
    current_zero_offset = ev_enu_to_px4_ned((1.0, 0.0, 0.0), 0.0)
    assert current_zero_offset == pytest.approx((0.0, 1.0, 0.0))


def test_stale_measurement_age_is_rejected_and_faults():
    core = EvHealthMonitorCore(config(max_input_age_s=0.25))
    results = []
    for index in range(4):
        stamp = 1.0 + 0.1 * index
        receive = stamp + 1.3
        core.update_px4_velocity(
            (0.0, 0.0, 0.0),
            receive,
            input_is_enu=True,
            sample_stamp_s=stamp,
        )
        results.append(
            core.process_ev(
                stamp_s=stamp,
                receive_time_s=receive,
                raw_position_enu=(0.0, 0.0, 0.0),
            )
        )

    assert all(not result.accepted for result in results)
    assert results[-1].state == HealthState.FAULT
    assert "stale_input" in results[-1].reason


def test_velocity_comparison_uses_measurement_time_not_latest_sample():
    core = EvHealthMonitorCore(config(max_velocity_alignment_s=0.05))
    core.update_px4_velocity(
        (0.5, 0.0, 0.0),
        receive_time_s=1.01,
        sample_stamp_s=1.0,
        input_is_enu=True,
    )
    core.process_ev(
        stamp_s=1.0,
        receive_time_s=1.01,
        raw_position_enu=(0.0, 0.0, 0.0),
    )
    # Add source samples in monotonic order. A newer, opposite velocity must not
    # be compared with the older EV interval ending at 1.1 s.
    core.update_px4_velocity(
        (0.5, 0.0, 0.0),
        receive_time_s=1.11,
        sample_stamp_s=1.1,
        input_is_enu=True,
    )
    core.update_px4_velocity(
        (-0.5, 0.0, 0.0),
        receive_time_s=1.21,
        sample_stamp_s=1.2,
        input_is_enu=True,
    )
    result = core.process_ev(
        stamp_s=1.1,
        receive_time_s=1.21,
        raw_position_enu=(0.05, 0.0, 0.0),
    )

    assert result.metrics.px4_velocity_px4_ned[1] == pytest.approx(0.5)
    assert result.state == HealthState.HEALTHY


def test_default_lowpass_does_not_create_false_fault_during_acceleration_and_reversal():
    core = EvHealthMonitorCore(
        config(velocity_lowpass_cutoff_hz=3.0)
    )
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(0.0, 0.0, 0.0))

    # Linear acceleration 0 -> +0.5 m/s has a +0.25 m/s interval average.
    result = add_frame(core, 1.1, 0.025, px4_velocity_enu=(0.5, 0.0, 0.0))
    assert result.metrics.raw_velocity_px4_ned[1] == pytest.approx(0.25)
    assert result.metrics.px4_velocity_px4_ned[1] == pytest.approx(0.25)
    assert result.state == HealthState.HEALTHY

    # A +0.5 -> -0.5 reversal averages to zero over this interval. The filtered
    # publication may lag, but health comparison must remain interval-consistent.
    result = add_frame(core, 1.2, 0.025, px4_velocity_enu=(-0.5, 0.0, 0.0))
    assert result.metrics.raw_velocity_px4_ned[1] == pytest.approx(0.0)
    assert result.metrics.px4_velocity_px4_ned[1] == pytest.approx(0.0)
    assert result.state == HealthState.HEALTHY


@pytest.mark.parametrize(
    "velocity,stamp,reason",
    [
        ((math.nan, 0.0, 0.0), 1.1, "non_finite_px4_velocity"),
        ((0.0, 0.0, 0.0), 0.0, "invalid_px4_velocity_stamp"),
        ((0.0, 0.0, 0.0), 0.9, "non_monotonic_px4_velocity_stamp"),
    ],
)
def test_bad_px4_velocity_frames_are_rejected_immediately(velocity, stamp, reason):
    core = EvHealthMonitorCore(config())
    assert core.update_px4_velocity(
        (0.0, 0.0, 0.0),
        receive_time_s=1.01,
        sample_stamp_s=1.0,
    )

    accepted = core.update_px4_velocity(
        velocity,
        receive_time_s=1.11,
        sample_stamp_s=stamp,
    )
    assert not accepted
    assert core.state == HealthState.SUSPECT
    assert reason in core.reason


def test_stale_frame_still_reports_converted_px4_position():
    core = EvHealthMonitorCore(config(max_input_age_s=0.25))
    result = core.process_ev(
        stamp_s=1.0,
        receive_time_s=2.0,
        raw_position_enu=(1.0, 2.0, 3.0),
    )

    assert not result.accepted
    assert result.metrics.position_px4_ned == pytest.approx((2.0, 1.0, -3.0))


def test_effective_points_status_must_remain_fresh_when_enabled():
    core = EvHealthMonitorCore(
        config(
            require_effective_points_status=True,
            effective_points_timeout_s=0.2,
        )
    )
    core.update_effective_points(True, 1.0)
    add_frame(core, 1.0, 0.0)

    assert core.check_timeout(1.21) == HealthState.SUSPECT
    assert "effective_points_status_timeout" in core.reason


def test_fault_requires_continuous_healthy_hysteresis_to_recover():
    core = EvHealthMonitorCore(config(recovery_healthy_s=0.5))
    add_frame(core, 1.0, 0.0, px4_velocity_enu=(-0.2, 0.0, 0.0))
    for index in range(1, 5):
        result = add_frame(
            core,
            1.0 + index * 0.1,
            index * 0.06,
            px4_velocity_enu=(-0.2, 0.0, 0.0),
        )
    assert result.state == HealthState.FAULT

    # A single healthy frame must not clear a latched fault.
    result = add_frame(core, 1.5, 0.24, px4_velocity_enu=(0.0, 0.0, 0.0))
    assert result.state == HealthState.FAULT
    for index in range(6, 11):
        result = add_frame(core, 1.0 + index * 0.1, 0.24)
    assert result.state == HealthState.HEALTHY
