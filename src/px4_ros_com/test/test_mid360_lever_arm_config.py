from pathlib import Path
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
VALIDATOR = WORKSPACE_ROOT / "tools" / "fastlio" / "校验杆臂配置.sh"
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "mid360_lever_arm.conf"


def run_validator(path):
    return subprocess.run(
        [str(VALIDATOR), str(path)],
        check=False,
        capture_output=True,
        text=True,
    )


def test_repository_config_passes_validation_with_adopted_measurement():
    """坐标转换.md is the single authority for these values. It records
    MID360_INSTALLATION_YAW_VERIFIED=1 deliberately: the config passes preflight
    and enters the takeoff chain. That flag is explicitly *not* a claim that the
    360-degree rotation residual is closed. The adopted X=Y=0, Z=+0.08 m
    remains unchanged; the strict 2026-08-12 propeller-off bag measured raw
    body +X forward and +Y left, so the installation yaw is zero."""
    result = run_validator(DEFAULT_CONFIG)
    assert result.returncode == 0, result.stderr
    config_text = DEFAULT_CONFIG.read_text(encoding="ascii")
    assert "MID360_LEVER_ARM_CALIBRATED=1" in config_text
    assert "MID360_INSTALLATION_YAW_VERIFIED=1" in config_text
    assert "MID360_BODY_TO_SENSOR_X_M=0.0" in config_text
    assert "MID360_BODY_TO_SENSOR_Y_M=0.0" in config_text
    assert "MID360_BODY_TO_SENSOR_Z_M=0.08" in config_text
    assert "MID360_BODY_TO_FASTLIO_YAW_RAD=0.0" in config_text


def test_uncalibrated_config_blocks_flight(tmp_path):
    config = tmp_path / "uncalibrated.conf"
    config.write_text(
        "MID360_LEVER_ARM_CALIBRATED=0\n"
        "MID360_INSTALLATION_YAW_VERIFIED=0\n"
        "MID360_BODY_TO_SENSOR_X_M=0.198\n"
        "MID360_BODY_TO_SENSOR_Y_M=-0.143\n"
        "MID360_BODY_TO_SENSOR_Z_M=0.08\n"
        "MID360_BODY_TO_FASTLIO_YAW_RAD=3.141592653589793\n",
        encoding="ascii",
    )
    result = run_validator(config)
    assert result.returncode != 0
    assert "尚未标定" in result.stderr


def test_measured_nonzero_body_flu_vector_is_accepted(tmp_path):
    config = tmp_path / "lever_arm.conf"
    config.write_text(
        "MID360_LEVER_ARM_CALIBRATED=1\n"
        "MID360_INSTALLATION_YAW_VERIFIED=1\n"
        "MID360_BODY_TO_SENSOR_X_M=0.21\n"
        "MID360_BODY_TO_SENSOR_Y_M=-0.08\n"
        "MID360_BODY_TO_SENSOR_Z_M=0.11\n"
        "MID360_BODY_TO_FASTLIO_YAW_RAD=0.0\n",
        encoding="ascii",
    )
    result = run_validator(config)
    assert result.returncode == 0, result.stderr
    assert "x=0.21 m y=-0.08 m z=0.11 m" in result.stdout


def test_zero_or_non_numeric_vector_is_rejected(tmp_path):
    zero = tmp_path / "zero.conf"
    zero.write_text(
        "MID360_LEVER_ARM_CALIBRATED=1\n"
        "MID360_INSTALLATION_YAW_VERIFIED=1\n"
        "MID360_BODY_TO_SENSOR_X_M=0.0\n"
        "MID360_BODY_TO_SENSOR_Y_M=0.0\n"
        "MID360_BODY_TO_SENSOR_Z_M=0.0\n"
        "MID360_BODY_TO_FASTLIO_YAW_RAD=0.0\n",
        encoding="ascii",
    )
    assert run_validator(zero).returncode != 0

    invalid = tmp_path / "invalid.conf"
    invalid.write_text(
        "MID360_LEVER_ARM_CALIBRATED=1\n"
        "MID360_INSTALLATION_YAW_VERIFIED=1\n"
        "MID360_BODY_TO_SENSOR_X_M=nan\n"
        "MID360_BODY_TO_SENSOR_Y_M=0.1\n"
        "MID360_BODY_TO_SENSOR_Z_M=0.1\n"
        "MID360_BODY_TO_FASTLIO_YAW_RAD=0.0\n",
        encoding="ascii",
    )
    assert run_validator(invalid).returncode != 0
