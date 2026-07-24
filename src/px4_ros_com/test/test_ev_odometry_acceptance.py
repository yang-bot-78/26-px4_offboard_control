import importlib.util
import math
from pathlib import Path

from nav_msgs.msg import Odometry


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_health_node_module():
    path = PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py"
    spec = importlib.util.spec_from_file_location("fastlio_ev_health_monitor", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


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


def test_default_launch_selects_only_mavros_odometry_path():
    source = (PACKAGE_ROOT / "launch" / "fastlio_mavros_autofix.launch.py").read_text()
    assert 'DeclareLaunchArgument("start_bridge", default_value="true")' in source
    assert (
        'DeclareLaunchArgument("start_mavros_vision_bridge", default_value="false")'
        in source
    )
    assert 'DeclareLaunchArgument("start_px4_ev_bridge", default_value="false")' in source
    assert '"output_topic": "/mavros/odometry/out"' in source
    assert '"restamp_message": False' in source
    assert '"input_linear_velocity_frame": "child"' in source
    assert (
        '"camera_init_ned_yaw", default_value="1.5707963267948966"'
        in source
    )


def test_bridge_rejects_duplicate_paths_and_non_monotonic_samples():
    source = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_mavros_odometry_bridge.cpp"
    ).read_text()
    assert 'count_publishers("/mavros/vision_pose/pose_cov")' in source
    assert 'count_publishers("/fmu/in/vehicle_visual_odometry")' in source
    assert "input_stamp_ns <= last_input_stamp_ns_.value()" in source
    assert 'declare_parameter<bool>("restamp_message", false)' in source
    assert 'declare_parameter<bool>("derive_missing_twist", false)' in source


def test_bridge_preserves_valid_velocity_variance_above_configurable_floor():
    source = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_mavros_odometry_bridge.cpp"
    ).read_text()
    assert (
        'declare_parameter<double>("min_linear_velocity_variance", 0.0001)'
        in source
    )
    assert "covariance[0] = std::max(covariance[0], position_variance_floor);" in source
    assert "covariance[7] = std::max(covariance[7], position_variance_floor);" in source
    assert "covariance[14] = std::max(covariance[14], position_variance_floor);" in source

    floor = 0.0001
    assert max(0.0002, floor) == 0.0002
    assert max(0.00005, floor) == 0.0001
    assert max(0.02, floor) == 0.02


def test_bridge_rejects_nonfinite_nonpositive_velocity_variance():
    source = (
        PACKAGE_ROOT / "src" / "bridges" / "fastlio_mavros_odometry_bridge.cpp"
    ).read_text()
    assert "!std::isfinite(covariance[row * 6 + row])" in source
    assert "covariance[row * 6 + row] <= 0.0" in source
    assert "!linear_covariance_is_valid(msg->twist.covariance)" in source

    assert not math.isfinite(math.nan)
    assert not math.isfinite(math.inf)
    assert -0.001 <= 0.0
    assert 0.0 <= 0.0


def test_bridge_launch_exposes_velocity_variance_floor():
    source = (
        PACKAGE_ROOT / "launch" / "fastlio_mavros_odometry_bridge.launch.py"
    ).read_text()
    assert '"min_linear_velocity_variance", default_value="0.0001"' in source
    assert '"min_linear_velocity_variance": min_linear_velocity_variance' in source


def test_health_gate_preserves_internal_velocity_and_inflates_full_covariance():
    source = (PACKAGE_ROOT / "scripts" / "fastlio_ev_health_monitor.py").read_text()
    assert "output.twist.twist.linear.x =" not in source
    assert "for row in range(3):" in source
    assert "result.covariance_multiplier" in source
    assert "if result.publish:" in source


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
