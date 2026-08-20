import importlib.util
import math
from pathlib import Path
import subprocess
import sys

from nav_msgs.msg import Odometry
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
CONTRACT_HELPER_PATH = PACKAGE_ROOT / "test" / "ev_entry_contract_static.py"
AUTOFIX_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "fastlio_mavros_autofix.launch.py"
STACK_PATH = WORKSPACE_ROOT / "tools" / "flight" / "一键启动起飞栈.sh"


def load_health_node_module():
    path = PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py"
    spec = importlib.util.spec_from_file_location("fastlio_ev_health_monitor", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_contract_module():
    spec = importlib.util.spec_from_file_location(
        "ev_entry_contract_static", CONTRACT_HELPER_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def replace_once(source, old, new):
    assert source.count(old) == 1, old
    return source.replace(old, new, 1)


def write_text(tmp_path, name, source):
    path = tmp_path / name
    path.write_text(source, encoding="utf-8")
    return path


def valid_odometry():
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = 0.2
    message.twist.twist.linear.y = -0.1
    message.twist.twist.linear.z = 0.05
    for index, value in ((0, 0.04), (7, 0.09), (14, 0.16)):
        message.twist.covariance[index] = value
    message.twist.twist.angular.x = math.nan
    message.twist.twist.angular.y = math.nan
    message.twist.twist.angular.z = math.nan
    return message


def test_unknown_angular_velocity_is_not_mistaken_for_bad_linear_measurement():
    module = load_health_node_module()
    assert module.FastlioEvHealthMonitor._message_is_finite(valid_odometry())


def test_zero_or_non_psd_linear_velocity_covariance_is_rejected():
    module = load_health_node_module()
    message = valid_odometry()
    message.twist.covariance[0] = 0.0
    assert not module.FastlioEvHealthMonitor._message_is_finite(message)

    message = valid_odometry()
    message.twist.covariance[1] = 0.2
    message.twist.covariance[6] = 0.2
    assert not module.FastlioEvHealthMonitor._message_is_finite(message)


def test_default_launch_selects_only_mavros_vision_path():
    source = AUTOFIX_LAUNCH_PATH.read_text()
    assert (
        'DeclareLaunchArgument("start_mavros_vision_bridge", default_value="true")'
        in source
    )
    assert '"input_topic": healthy_odom_topic' in source
    assert (
        'DeclareLaunchArgument("healthy_odom_topic", default_value="/Odometry/healthy")'
        in source
    )
    assert '"pose_topic": "/mavros/vision_pose/pose_cov"' in source
    assert '"restamp_message": False' in source
    assert (
        '"camera_init_ned_yaw", default_value="1.5707963267948966"'
        in source
    )
    # The two alternative EV writers were deleted at the source. Their launch
    # adapters must be gone too, so no argument can turn a second writer on.
    # Comments still explain why they were removed, so only executable code is
    # checked here -- the contract helper enforces the same rule structurally.
    code = "\n".join(
        line for line in source.splitlines() if not line.lstrip().startswith("#")
    )
    assert "start_bridge" not in code.replace("start_mavros_vision_bridge", "")
    assert "start_px4_ev_bridge" not in code
    assert "fastlio_mavros_odometry_bridge" not in code
    assert "fastlio_vehicle_visual_odometry" not in code


def test_internal_velocity_difference_threshold_is_launch_configurable():
    source = AUTOFIX_LAUNCH_PATH.read_text()
    assert (
        'ev_max_internal_velocity_difference_mps = LaunchConfiguration(\n'
        '        "ev_max_internal_velocity_difference_mps"\n'
        "    )"
    ) in source
    assert (
        '"max_internal_velocity_difference_mps":\n'
        "                    ev_max_internal_velocity_difference_mps"
    ) in source
    assert (
        '"ev_max_internal_velocity_difference_mps", default_value="0.50"'
        in source
    )


def test_velocity_comparison_window_is_launch_configurable():
    source = AUTOFIX_LAUNCH_PATH.read_text()
    assert (
        'ev_velocity_comparison_window_s = LaunchConfiguration(\n'
        '        "ev_velocity_comparison_window_s"\n'
        "    )"
    ) in source
    assert (
        '"velocity_comparison_window_s": ev_velocity_comparison_window_s'
        in source
    )
    assert (
        '"ev_velocity_comparison_window_s", default_value="0.15"'
        in source
    )


def test_one_click_stack_explicitly_selects_only_mavros_vision_path():
    source = STACK_PATH.read_text()
    assert "start_mavros_vision_bridge:=true" in source
    # Passing an argument the launch file no longer declares would abort the
    # launch, so the stack must not mention the deleted adapters.
    assert "start_bridge:=" not in source
    assert "start_px4_ev_bridge:=" not in source


def test_structural_contract_accepts_current_selected_entry():
    contract = load_contract_module()
    report = contract.validate_contract(AUTOFIX_LAUNCH_PATH, STACK_PATH)

    assert report["identity"] == (
        "package=px4_ros_com,"
        "executable=fastlio_mavros_vision_bridge,"
        "name=fastlio_mavros_vision_bridge"
    )
    assert report["condition"] == "start_mavros_vision_bridge"
    assert report["input"] == "/Odometry/healthy"
    assert report["restamp"] == "false"
    assert report["validation_order"] == "single_ev_node_in_source"


@pytest.mark.parametrize(
    "old,new,error",
    [
        (
            'DeclareLaunchArgument("start_mavros_vision_bridge", default_value="true")',
            'DeclareLaunchArgument("start_mavros_vision_bridge", default_value="false")',
            "start_mavros_vision_bridge",
        ),
        (
            '"input_topic": healthy_odom_topic,',
            '"input_topic": fastlio_odom_topic,',
            "input_topic",
        ),
        (
            '"restamp_message": False,',
            '"restamp_message": True,',
            "restamp_message",
        ),
        # Reintroducing a deleted adapter must fail closed even though the
        # argument no longer exists in the real launch file.
        (
            'DeclareLaunchArgument("start_odom_guard", default_value="true"),',
            'DeclareLaunchArgument("start_odom_guard", default_value="true"),\n'
            '            DeclareLaunchArgument("start_bridge", default_value="false"),',
            "start_bridge",
        ),
        (
            'DeclareLaunchArgument("start_odom_guard", default_value="true"),',
            'DeclareLaunchArgument("start_odom_guard", default_value="true"),\n'
            '            DeclareLaunchArgument("start_px4_ev_bridge", default_value="false"),',
            "start_px4_ev_bridge",
        ),
    ],
)
def test_launch_negative_mutation_matrix(tmp_path, old, new, error):
    contract = load_contract_module()
    source = AUTOFIX_LAUNCH_PATH.read_text(encoding="utf-8")
    if old in ('"input_topic": healthy_odom_topic,', '"restamp_message": False,'):
        assert source.count(old) >= 1
        mutated = source.replace(old, new, 1)
    else:
        mutated = replace_once(source, old, new)
    launch_path = write_text(tmp_path, "mutated.launch.py", mutated)

    with pytest.raises(contract.ContractError, match=error):
        contract.validate_launch(launch_path)


def test_decoy_literals_comments_and_other_bridges_cannot_satisfy_selected_node(tmp_path):
    contract = load_contract_module()
    source = AUTOFIX_LAUNCH_PATH.read_text(encoding="utf-8")
    mutated = source.replace(
        '"input_topic": healthy_odom_topic,',
        '"input_topic": fastlio_odom_topic,',
        1,
    )
    mutated = mutated.replace('"restamp_message": False,', '"restamp_message": True,', 1)
    mutated += """
# Decoys must not satisfy the selected Node contract:
# \"input_topic\": healthy_odom_topic, \"restamp_message\": False
UNRELATED_CONTRACT_LITERALS = (
    \"input_topic\", \"healthy_odom_topic\", \"restamp_message\", False
)
"""
    launch_path = write_text(tmp_path, "decoy.launch.py", mutated)

    with pytest.raises(contract.ContractError, match="input_topic"):
        contract.validate_launch(launch_path)


@pytest.mark.parametrize(
    "original,wrong,late,error",
    [
        (
            'healthy_odom_topic = LaunchConfiguration("healthy_odom_topic")',
            'healthy_odom_topic = LaunchConfiguration("fastlio_odom_topic")',
            'healthy_odom_topic = LaunchConfiguration("healthy_odom_topic")',
            "input_topic",
        ),
    ],
)
def test_selected_node_rejects_correct_rebinding_after_capture(
    tmp_path, original, wrong, late, error
):
    """A correct rebinding placed *after* the Node captured the wrong value
    must not launder the contract: the value the Node actually receives is the
    one bound at capture time."""
    contract = load_contract_module()
    source = AUTOFIX_LAUNCH_PATH.read_text(encoding="utf-8")
    mutated = replace_once(source, original, wrong)
    mutated = replace_once(
        mutated,
        "\n\n    fastlio_mavros_vision_bridge = Node(",
        f"\n\n    {late}\n\n    fastlio_mavros_vision_bridge = Node(",
    )
    launch_path = write_text(tmp_path, "late-rebinding.launch.py", mutated)

    with pytest.raises(contract.ContractError, match=error):
        contract.validate_launch(launch_path)


def test_nested_early_launch_return_fails_closed(tmp_path):
    contract = load_contract_module()
    source = AUTOFIX_LAUNCH_PATH.read_text(encoding="utf-8")
    mutated = replace_once(
        source,
        "def generate_launch_description():\n",
        "def generate_launch_description():\n"
        "    if False:\n"
        "        return LaunchDescription([])\n",
    )
    launch_path = write_text(tmp_path, "nested-early-return.launch.py", mutated)

    with pytest.raises(contract.ContractError, match="only Return"):
        contract.validate_launch(launch_path)


def test_launch_decorator_cannot_replace_return_path(tmp_path):
    contract = load_contract_module()
    source = AUTOFIX_LAUNCH_PATH.read_text(encoding="utf-8")
    mutated = replace_once(
        source,
        "def generate_launch_description():",
        "@(lambda function: (lambda: LaunchDescription([])))\n"
        "def generate_launch_description():",
    )
    launch_path = write_text(tmp_path, "decorated-generate.launch.py", mutated)

    with pytest.raises(contract.ContractError, match="must be undecorated"):
        contract.validate_launch(launch_path)


def test_stack_tokens_split_across_commands_and_comments_fail_closed(tmp_path):
    """A deleted adapter smuggled into a nested command must fail closed, and a
    commented-out token must not be mistaken for a real one."""
    contract = load_contract_module()
    source = STACK_PATH.read_text(encoding="utf-8")
    mutated = replace_once(
        source,
        "start_mavros_vision_bridge:=true",
        "start_mavros_vision_bridge:=true start_px4_ev_bridge:=false",
    )
    stack_path = write_text(tmp_path, "split-stack.sh", mutated)

    with pytest.raises(contract.ContractError, match="start_px4_ev_bridge"):
        contract.validate_stack(stack_path)


def test_stack_conflicting_duplicate_argument_fails_closed(tmp_path):
    contract = load_contract_module()
    source = STACK_PATH.read_text(encoding="utf-8")
    mutated = replace_once(
        source,
        "start_mavros_vision_bridge:=true",
        "start_mavros_vision_bridge:=true start_mavros_vision_bridge:=false",
    )
    stack_path = write_text(tmp_path, "conflicting-stack.sh", mutated)

    with pytest.raises(contract.ContractError, match="start_mavros_vision_bridge"):
        contract.validate_stack(stack_path)


def test_missing_inputs_and_parse_errors_fail_closed(tmp_path):
    contract = load_contract_module()
    with pytest.raises(contract.ContractError, match="cannot read"):
        contract.validate_contract(tmp_path / "missing.launch.py", STACK_PATH)

    invalid_launch = write_text(tmp_path, "invalid.launch.py", "def broken(:\n")
    with pytest.raises(contract.ContractError, match="AST parse failed"):
        contract.validate_launch(invalid_launch)

    invalid_stack = write_text(
        tmp_path,
        "invalid-stack.sh",
        'open_window "PX4" "px4" "unterminated\n',
    )
    with pytest.raises(contract.ContractError, match="shell parse failed"):
        contract.validate_stack(invalid_stack)


def test_helper_cli_and_missing_helper_fail_closed(tmp_path):
    invalid_launch = write_text(tmp_path, "invalid.launch.py", "def broken(:\n")
    result = subprocess.run(
        [
            sys.executable,
            str(CONTRACT_HELPER_PATH),
            "--launch",
            str(invalid_launch),
            "--stack",
            str(STACK_PATH),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "STRUCTURAL_EV_ENTRY_CONTRACT=FAIL" in result.stderr

    missing = subprocess.run(
        [sys.executable, str(tmp_path / "missing-helper.py")],
        check=False,
        capture_output=True,
        text=True,
    )
    assert missing.returncode != 0


def test_readiness_invokes_structural_helper_fail_closed():
    source = (
        WORKSPACE_ROOT / "tools" / "checks" / "起飞前自检.sh"
    ).read_text(
        encoding="utf-8"
    )
    # The script lives at tools/checks/, so the workspace root is two levels up.
    assert 'project_root="$(cd -- "${script_dir}/../.." && pwd -P)"' in source
    assert 'require_file "${ev_contract_helper}"' in source
    assert (
        'python3 "${ev_contract_helper}" --launch "${launch_source}" '
        '--stack "${stack_script}"'
    ) in source
    assert "require_literal" not in source


# The deleted fastlio_mavros_odometry_bridge used to carry these guarantees.
# They did not disappear with it -- each moved to the component that owns it on
# the single EV path, and the tests below pin them there:
#   duplicate / out-of-order samples -> fastlio_odometry_guard motion gate
#   covariance floor + finite payload -> fastlio_ev_health_monitor
#   source timestamp preservation     -> fastlio_mavros_vision_bridge


def test_guard_rejects_duplicate_and_non_monotonic_samples():
    source = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_odometry_guard.cpp"
    ).read_text()
    # Duplicate or out-of-order frames must not advance the baseline.
    assert "if (dt <= 0.0) {" in source
    assert "if (dt <= min_dt_s_) {" in source
    assert 'ss << "closely spaced dt "' in source
    assert "return MotionGateResult::Reject;" in source
    assert "Do not move the baseline backwards" in source
    # A long gap may rebaseline only after the absolute jump checks pass.
    assert "if (dt > max_dt_s_) {" in source
    assert "return MotionGateResult::RejectAndRebaseline;" in source
    assert source.index("if (position_jump > max_position_jump_m_)") < source.index(
        "if (dt > max_dt_s_)"
    )
    assert 'reason += "; resynced guard baseline"' not in source


def test_vision_bridge_preserves_source_timestamp_by_default():
    source = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_mavros_vision_bridge.cpp"
    ).read_text()
    # 坐标转换.md requires the FAST-LIO measurement time to reach PX4 unchanged.
    # The default must be the documented behaviour, not something only a launch
    # file supplies -- a bare `ros2 run` has to be correct too.
    assert 'declare_parameter<bool>("restamp_message", false)' in source
    assert "bool restamp_message_{false};" in source


def test_vision_bridge_limits_the_complete_ev_pair_to_50_hz_by_default():
    source = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_mavros_vision_bridge.cpp"
    ).read_text()
    launch = AUTOFIX_LAUNCH_PATH.read_text()
    assert 'declare_parameter<double>("max_publish_rate_hz", 50.0)' in source
    assert "std::chrono::steady_clock" in source
    assert "publish_time - last_publish_time_ < min_publish_interval_" in source
    assert 'DeclareLaunchArgument("ev_publish_rate_hz", default_value="50.0")' in launch
    assert '"max_publish_rate_hz": ParameterValue(' in launch


def test_health_gate_applies_covariance_floor_above_configurable_minimum():
    source = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    assert '"healthy_position_variance_floor_m2", 0.01' in source
    assert '"healthy_orientation_variance_floor_rad2", 0.02' in source
    # Position (0,7,14) and orientation (21,28,35) diagonals are both floored,
    # then scaled by the health multiplier.
    assert "for index in (0, 7, 14):" in source
    assert "for index in (21, 28, 35):" in source
    assert "self.core.config.min_position_variance," in source
    assert "self.core.config.min_orientation_variance," in source

    floor = 0.01
    assert max(0.02, floor) == 0.02
    assert max(0.005, floor) == 0.01


def test_health_gate_declares_calibrated_world_velocity_covariance_floors():
    source = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    launch = AUTOFIX_LAUNCH_PATH.read_text()
    assert '"velocity_variance_floor_x_m2ps2", 0.0016' in source
    assert '"velocity_variance_floor_y_m2ps2", 0.0013' in source
    assert '"velocity_variance_floor_z_m2ps2", 0.0' in source
    assert '"velocity_variance_floor_x_m2ps2": velocity_variance_floor_x_m2ps2' in launch
    assert '"velocity_variance_floor_y_m2ps2": velocity_variance_floor_y_m2ps2' in launch
    assert '"velocity_variance_floor_z_m2ps2": velocity_variance_floor_z_m2ps2' in launch
    assert "apply_world_velocity_covariance_floors(" in source
    assert "linear_covariance_body[row][column]" in source


def test_health_gate_rejects_nonfinite_payload():
    source = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    assert "finite_payload=self._message_is_finite(msg)" in source
    assert "def _message_is_finite(cls, msg: Odometry) -> bool:" in source
    assert "all(math.isfinite(value) for value in values)" in source

    assert not math.isfinite(math.nan)
    assert not math.isfinite(math.inf)


def test_health_gate_preserves_internal_velocity_and_inflates_full_covariance():
    source = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    assert "output.twist.twist.linear.x =" not in source
    assert "for row in range(3):" in source
    assert "result.covariance_multiplier" in source
    assert "if result.publish and not frlio_block:" in source


def test_realtime_odometry_chain_uses_sensor_qos_depth_five():
    guard = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_odometry_guard.cpp"
    ).read_text()
    bridge = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_mavros_vision_bridge.cpp"
    ).read_text()
    monitor = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    assert "rclcpp::SensorDataQoS().keep_last(5)" in guard
    assert "input_topic_, odom_qos" in guard
    assert "input_topic_, odom_qos" in bridge
    assert "depth=5" in monitor
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in monitor
    assert "Odometry, self.output_topic, odom_qos" in monitor


def test_frlio_anchor_gate_rejects_unknown_and_stale_status():
    module = load_health_node_module()
    evaluate = module._evaluate_frlio_anchor_gate
    common = dict(
        required=True,
        status_seen=True,
        status_message_age_s=0.01,
        anchor_message_age_s=0.01,
        message_timeout_s=0.5,
        max_anchor_age_s=0.60,
    )
    assert evaluate(status="HEALTHY", anchor_age_s=0.10, **common) == (None, False)
    reason, blocked = evaluate(status="probe", anchor_age_s=0.10, **common)
    assert reason == "frlio_status_probe"
    assert blocked
    assert evaluate(status="HEALTHY", anchor_age_s=0.60, **common) == (None, False)
    reason, blocked = evaluate(status="HEALTHY", anchor_age_s=0.601, **common)
    assert reason.startswith("frlio_anchor_stale")
    assert blocked
    reason, blocked = evaluate(
        status="SUSPECT_STALE_LIDAR", anchor_age_s=0.20, **common
    )
    assert reason.startswith("frlio_anchor_suspect")
    assert not blocked


def test_frlio_soft_suspect_remains_diagnostic_only():
    source = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    assert "_exposed_health_state" not in source
    assert 'key="frlio_gate_reason"' in source


def test_frlio_soft_suspect_does_not_feed_the_hard_fault_timer():
    source = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    assert "if frlio_reason is not None and frlio_block:" in source
    assert "A short LiDAR-anchor warning is diagnostic-only" in source
    assert "output_covariance_multiplier = max(" not in source


def test_flight_ready_keeps_raw_suspect_diagnostic_only():
    module = load_health_node_module()
    evaluate = module._evaluate_flight_ready
    common = dict(
        core_state=module.HealthState.SUSPECT,
        frlio_reason="frlio_anchor_suspect age=0.200s",
        frlio_block=False,
        last_healthy_output_s=10.0,
        healthy_output_timeout_s=0.10,
    )

    ready, reason = evaluate(now_s=10.0, **common)
    assert ready
    assert reason == "ready"

    ready, reason = evaluate(
        core_state=module.HealthState.FAULT,
        now_s=10.01,
        **{key: value for key, value in common.items() if key != "core_state"},
    )
    assert not ready
    assert reason == "core_state_fault"


def test_flight_ready_fails_closed_for_hard_gate_and_stale_output():
    module = load_health_node_module()
    evaluate = module._evaluate_flight_ready
    common = dict(
        core_state=module.HealthState.HEALTHY,
        now_s=5.0,
        healthy_output_timeout_s=0.10,
    )

    ready, reason = evaluate(
        frlio_reason="frlio_anchor_stale age=0.601s",
        frlio_block=True,
        last_healthy_output_s=5.0,
        **common,
    )
    assert not ready
    assert reason.startswith("frlio_anchor_stale")

    ready, reason = evaluate(
        frlio_reason=None,
        frlio_block=False,
        last_healthy_output_s=4.89,
        **common,
    )
    assert not ready
    assert reason.startswith("healthy_odom_output_stale")


def test_actual_fastlio_source_publishes_internal_velocity_before_message():
    source_path = Path(
        "/home/robot/livox_mid360_env/ws_fastlio/src/fast_lio/src/laserMapping.cpp"
    )
    assert source_path.is_file(), "actual FAST-LIO source branch is missing"
    source = source_path.read_text()
    function = source[source.index("void publish_odometry"):source.index("void publish_path")]
    assert "state_point.vel" in function
    assert "P.block<3, 3>(12, 12)" in function
    assert "quiet_NaN" in function
    assert function.index("twist.twist.linear.x") < function.index(
        "pubOdomAftMapped->publish"
    )
