"""
Tests for security utilities.

Tests cover:
- Input validation (joint angles, speeds, addresses, names)
- Privilege checking (GPIO, I2C, SPI, serial, CAN)
- Sanitization (names, serial commands)
"""

from __future__ import annotations

import math
import os
import sys
from unittest.mock import patch

import pytest

from robo_infra.utils.security import (
    AddressRange,
    HardwareAccess,
    InputValidator,
    JointLimits,
    PrivilegeError,
    SpeedLimits,
    ValidationError,
    check_all_hardware_access,
    check_can_access,
    check_gpio_access,
    check_i2c_access,
    check_serial_access,
    check_spi_access,
    get_required_groups,
    sanitize_name,
    sanitize_serial_command,
    validate_acceleration,
    validate_can_id,
    validate_i2c_address,
    validate_joint_angle,
    validate_joint_angles,
    validate_port_name,
    validate_speed,
)


# ===========================================================================
# Test Fixtures
# ===========================================================================


@pytest.fixture
def robot_validator() -> InputValidator:
    """Create a validator with typical robot limits."""
    return InputValidator(
        joint_limits=[
            JointLimits(-math.pi, math.pi, "shoulder"),
            JointLimits(-math.pi / 2, math.pi / 2, "elbow"),
            JointLimits(-2.0, 2.0, "wrist"),
        ],
        max_speed=5.0,
        max_accel=20.0,
    )


# ===========================================================================
# Joint Angle Validation Tests
# ===========================================================================


class TestValidateJointAngle:
    """Tests for validate_joint_angle()."""

    def test_valid_angle(self) -> None:
        """Test validation of valid angles."""
        assert validate_joint_angle(0.0) is True
        assert validate_joint_angle(1.5) is True
        assert validate_joint_angle(-1.5) is True
        assert validate_joint_angle(math.pi) is True

    def test_angle_at_limits(self) -> None:
        """Test angles exactly at limits are valid."""
        assert validate_joint_angle(-1.0, min_angle=-1.0, max_angle=1.0) is True
        assert validate_joint_angle(1.0, min_angle=-1.0, max_angle=1.0) is True

    def test_angle_below_minimum(self) -> None:
        """Test angle below minimum raises ValidationError."""
        with pytest.raises(ValidationError) as exc_info:
            validate_joint_angle(-2.0, min_angle=-1.0, max_angle=1.0)

        assert exc_info.value.field == "joint"
        assert exc_info.value.value == -2.0
        assert "below minimum" in str(exc_info.value)

    def test_angle_above_maximum(self) -> None:
        """Test angle above maximum raises ValidationError."""
        with pytest.raises(ValidationError) as exc_info:
            validate_joint_angle(2.0, min_angle=-1.0, max_angle=1.0)

        assert exc_info.value.field == "joint"
        assert exc_info.value.value == 2.0
        assert "exceeds maximum" in str(exc_info.value)

    def test_non_finite_angle(self) -> None:
        """Test non-finite angles raise ValidationError."""
        with pytest.raises(ValidationError):
            validate_joint_angle(float("inf"))

        with pytest.raises(ValidationError):
            validate_joint_angle(float("-inf"))

        with pytest.raises(ValidationError):
            validate_joint_angle(float("nan"))

    def test_non_numeric_angle(self) -> None:
        """Test non-numeric angles raise ValidationError."""
        with pytest.raises(ValidationError):
            validate_joint_angle("1.5")  # type: ignore

        with pytest.raises(ValidationError):
            validate_joint_angle(None)  # type: ignore

    def test_custom_joint_name(self) -> None:
        """Test custom joint name appears in error."""
        with pytest.raises(ValidationError) as exc_info:
            validate_joint_angle(10.0, joint_name="shoulder")

        assert "shoulder" in str(exc_info.value)


class TestValidateJointAngles:
    """Tests for validate_joint_angles()."""

    def test_valid_angles(self) -> None:
        """Test validation of valid angle sequences."""
        assert validate_joint_angles([0.0, 1.0, -1.0]) is True
        assert validate_joint_angles([0.5]) is True
        assert validate_joint_angles([]) is True  # Empty is valid

    def test_with_custom_limits(self) -> None:
        """Test validation with custom limits per joint."""
        limits = [
            JointLimits(-1.0, 1.0, "j1"),
            JointLimits(-2.0, 2.0, "j2"),
        ]
        assert validate_joint_angles([0.5, 1.5], limits) is True

    def test_length_mismatch(self) -> None:
        """Test length mismatch raises ValidationError."""
        limits = [JointLimits(-1.0, 1.0, "j1")]

        with pytest.raises(ValidationError) as exc_info:
            validate_joint_angles([0.5, 1.0], limits)

        assert "Expected 1" in str(exc_info.value)

    def test_one_invalid_angle(self) -> None:
        """Test one invalid angle fails the whole sequence."""
        limits = [
            JointLimits(-1.0, 1.0, "j1"),
            JointLimits(-1.0, 1.0, "j2"),
        ]

        with pytest.raises(ValidationError) as exc_info:
            validate_joint_angles([0.5, 2.0], limits)

        assert exc_info.value.field == "j2"


# ===========================================================================
# Speed and Acceleration Validation Tests
# ===========================================================================


class TestValidateSpeed:
    """Tests for validate_speed()."""

    def test_valid_speed(self) -> None:
        """Test validation of valid speeds."""
        assert validate_speed(0.0) is True
        assert validate_speed(5.0) is True
        assert validate_speed(10.0) is True

    def test_negative_speed(self) -> None:
        """Test negative speeds raise ValidationError."""
        with pytest.raises(ValidationError) as exc_info:
            validate_speed(-1.0)

        assert "below minimum" in str(exc_info.value)

    def test_speed_above_max(self) -> None:
        """Test speed above max raises ValidationError."""
        with pytest.raises(ValidationError):
            validate_speed(15.0, max_speed=10.0)

    def test_custom_limits(self) -> None:
        """Test custom speed limits."""
        assert validate_speed(2.0, min_speed=1.0, max_speed=5.0) is True

        with pytest.raises(ValidationError):
            validate_speed(0.5, min_speed=1.0, max_speed=5.0)


class TestValidateAcceleration:
    """Tests for validate_acceleration()."""

    def test_valid_acceleration(self) -> None:
        """Test validation of valid accelerations."""
        assert validate_acceleration(0.0) is True
        assert validate_acceleration(20.0) is True
        assert validate_acceleration(50.0) is True

    def test_negative_acceleration(self) -> None:
        """Test negative acceleration raises ValidationError."""
        with pytest.raises(ValidationError):
            validate_acceleration(-5.0)

    def test_above_max_acceleration(self) -> None:
        """Test acceleration above max raises ValidationError."""
        with pytest.raises(ValidationError):
            validate_acceleration(100.0, max_accel=50.0)


# ===========================================================================
# Address Validation Tests
# ===========================================================================


class TestValidateI2CAddress:
    """Tests for validate_i2c_address()."""

    def test_valid_addresses(self) -> None:
        """Test validation of valid I2C addresses."""
        assert validate_i2c_address(0x20) is True
        assert validate_i2c_address(0x40) is True
        assert validate_i2c_address(0x68) is True  # Common IMU address
        assert validate_i2c_address(0x77) is True  # Max non-reserved

    def test_reserved_addresses(self) -> None:
        """Test reserved addresses are rejected by default."""
        # Low reserved range (0x00-0x07)
        with pytest.raises(ValidationError):
            validate_i2c_address(0x00)
        with pytest.raises(ValidationError):
            validate_i2c_address(0x07)

        # High reserved range (0x78-0x7F)
        with pytest.raises(ValidationError):
            validate_i2c_address(0x78)
        with pytest.raises(ValidationError):
            validate_i2c_address(0x7F)

    def test_allow_reserved(self) -> None:
        """Test allow_reserved flag permits reserved addresses."""
        assert validate_i2c_address(0x00, allow_reserved=True) is True
        assert validate_i2c_address(0x7F, allow_reserved=True) is True

    def test_out_of_range(self) -> None:
        """Test out-of-range addresses raise ValidationError."""
        with pytest.raises(ValidationError):
            validate_i2c_address(-1)
        with pytest.raises(ValidationError):
            validate_i2c_address(0x80)
        with pytest.raises(ValidationError):
            validate_i2c_address(0xFF)

    def test_non_integer(self) -> None:
        """Test non-integer address raises ValidationError."""
        with pytest.raises(ValidationError):
            validate_i2c_address(32.5)  # type: ignore
        with pytest.raises(ValidationError):
            validate_i2c_address("0x20")  # type: ignore


class TestValidateCanId:
    """Tests for validate_can_id()."""

    def test_valid_standard_id(self) -> None:
        """Test validation of valid standard CAN IDs."""
        assert validate_can_id(0x000) is True
        assert validate_can_id(0x100) is True
        assert validate_can_id(0x7FF) is True  # Max standard

    def test_standard_id_too_large(self) -> None:
        """Test standard IDs > 0x7FF are rejected."""
        with pytest.raises(ValidationError) as exc_info:
            validate_can_id(0x800)

        assert "standard" in str(exc_info.value)

    def test_extended_id(self) -> None:
        """Test extended 29-bit IDs."""
        assert validate_can_id(0x800, extended=True) is True
        assert validate_can_id(0x1FFFFFFF, extended=True) is True

        with pytest.raises(ValidationError):
            validate_can_id(0x20000000, extended=True)

    def test_negative_id(self) -> None:
        """Test negative IDs are rejected."""
        with pytest.raises(ValidationError):
            validate_can_id(-1)


# ===========================================================================
# Name and Port Validation Tests
# ===========================================================================


class TestValidatePortName:
    """Tests for validate_port_name()."""

    def test_valid_linux_ports(self) -> None:
        """Test valid Linux serial port names."""
        assert validate_port_name("/dev/ttyUSB0", platform="linux") is True
        assert validate_port_name("/dev/ttyACM0", platform="linux") is True
        assert validate_port_name("/dev/ttyAMA0", platform="linux") is True

    def test_valid_macos_ports(self) -> None:
        """Test valid macOS serial port names."""
        assert validate_port_name("/dev/tty.usbserial-1234", platform="darwin") is True
        assert validate_port_name("/dev/cu.usbmodem14101", platform="darwin") is True

    def test_valid_windows_ports(self) -> None:
        """Test valid Windows COM ports."""
        assert validate_port_name("COM1", platform="windows") is True
        assert validate_port_name("COM10", platform="windows") is True

    def test_path_traversal(self) -> None:
        """Test path traversal is rejected."""
        with pytest.raises(ValidationError):
            validate_port_name("/dev/../etc/passwd", platform="linux")
        with pytest.raises(ValidationError):
            validate_port_name("../something", platform="linux")

    def test_invalid_port_name(self) -> None:
        """Test invalid port names are rejected."""
        with pytest.raises(ValidationError):
            validate_port_name("/etc/passwd", platform="linux")
        with pytest.raises(ValidationError):
            validate_port_name("/dev/sda1", platform="linux")

    def test_empty_port_name(self) -> None:
        """Test empty port name is rejected."""
        with pytest.raises(ValidationError):
            validate_port_name("")


class TestSanitizeName:
    """Tests for sanitize_name()."""

    def test_valid_names(self) -> None:
        """Test valid names pass through unchanged."""
        assert sanitize_name("robot") == "robot"
        assert sanitize_name("robot_1") == "robot_1"
        assert sanitize_name("my-robot") == "my-robot"
        assert sanitize_name("Robot123") == "Robot123"

    def test_invalid_start_character(self) -> None:
        """Test names must start with a letter."""
        with pytest.raises(ValidationError):
            sanitize_name("1robot")
        with pytest.raises(ValidationError):
            sanitize_name("_robot")
        with pytest.raises(ValidationError):
            sanitize_name("-robot")

    def test_invalid_characters(self) -> None:
        """Test invalid characters are rejected."""
        with pytest.raises(ValidationError):
            sanitize_name("robot@home")
        with pytest.raises(ValidationError):
            sanitize_name("robot/1")
        with pytest.raises(ValidationError):
            sanitize_name("../etc/passwd")

    def test_max_length(self) -> None:
        """Test max length enforcement."""
        long_name = "a" * 100
        with pytest.raises(ValidationError):
            sanitize_name(long_name, max_length=64)

    def test_empty_name(self) -> None:
        """Test empty name is rejected."""
        with pytest.raises(ValidationError):
            sanitize_name("")

    def test_allow_dots(self) -> None:
        """Test allow_dots parameter."""
        assert sanitize_name("robot.v1", allow_dots=True) == "robot.v1"

        with pytest.raises(ValidationError):
            sanitize_name("robot.v1", allow_dots=False)


class TestSanitizeSerialCommand:
    """Tests for sanitize_serial_command()."""

    def test_valid_commands(self) -> None:
        """Test valid serial commands pass through."""
        assert sanitize_serial_command("G1 X10 Y20") == "G1 X10 Y20"
        assert sanitize_serial_command("M104 S200") == "M104 S200"
        assert sanitize_serial_command("?") == "?"

    def test_forbidden_patterns(self) -> None:
        """Test forbidden shell patterns are rejected."""
        with pytest.raises(ValidationError):
            sanitize_serial_command("echo; rm -rf /")
        with pytest.raises(ValidationError):
            sanitize_serial_command("cat && ls")
        with pytest.raises(ValidationError):
            sanitize_serial_command("cmd || other")
        with pytest.raises(ValidationError):
            sanitize_serial_command("$(whoami)")

    def test_control_characters(self) -> None:
        """Test control characters are rejected."""
        with pytest.raises(ValidationError):
            sanitize_serial_command("cmd\x00null")  # Null byte
        with pytest.raises(ValidationError):
            sanitize_serial_command("cmd\x1bescape")  # ESC

    def test_max_length(self) -> None:
        """Test max length enforcement."""
        long_cmd = "G" * 300
        with pytest.raises(ValidationError):
            sanitize_serial_command(long_cmd, max_length=256)

    def test_allowed_chars(self) -> None:
        """Test allowed_chars restriction."""
        gcode_chars = "GMTSPF0123456789. \r\n"
        assert sanitize_serial_command("G1 F100", allowed_chars=gcode_chars) == "G1 F100"

        with pytest.raises(ValidationError):
            sanitize_serial_command("G1 X10", allowed_chars=gcode_chars)  # X not allowed


# ===========================================================================
# Privilege Checking Tests
# ===========================================================================


class TestPrivilegeChecking:
    """Tests for privilege checking functions."""

    def test_check_gpio_access_non_linux(self) -> None:
        """Test GPIO check is skipped on non-Linux."""
        with patch.object(sys, "platform", "darwin"):
            # Should not raise on non-Linux
            check_gpio_access()

    def test_check_i2c_access_non_linux(self) -> None:
        """Test I2C check is skipped on non-Linux."""
        with patch.object(sys, "platform", "darwin"):
            check_i2c_access()

    def test_check_serial_access_non_linux(self) -> None:
        """Test serial check is skipped on non-Linux."""
        with patch.object(sys, "platform", "darwin"):
            check_serial_access()

    def test_check_spi_access_non_linux(self) -> None:
        """Test SPI check is skipped on non-Linux."""
        with patch.object(sys, "platform", "darwin"):
            check_spi_access()

    def test_check_can_access_non_linux(self) -> None:
        """Test CAN check is skipped on non-Linux."""
        with patch.object(sys, "platform", "darwin"):
            check_can_access()

    @pytest.mark.skipif(sys.platform != "linux", reason="Linux-only test")
    def test_check_gpio_access_linux_no_device(self) -> None:
        """Test GPIO check when no device exists."""
        with patch.object(os.path, "exists", return_value=False):
            # Should not raise when device doesn't exist
            check_gpio_access()

    @pytest.mark.skipif(sys.platform != "linux", reason="Linux-only test")
    def test_check_i2c_access_linux_permission_denied(self) -> None:
        """Test I2C check raises when permission denied."""
        with (
            patch.object(os.path, "exists", return_value=True),
            patch.object(os, "access", return_value=False),
        ):
            with pytest.raises(PrivilegeError) as exc_info:
                check_i2c_access(bus=1)

            assert exc_info.value.required_group == "i2c"
            assert exc_info.value.fix_command is not None
            assert "i2c" in exc_info.value.fix_command

    def test_check_all_hardware_access(self) -> None:
        """Test check_all_hardware_access returns dict."""
        with patch.object(sys, "platform", "darwin"):
            result = check_all_hardware_access()

            assert isinstance(result, dict)
            assert HardwareAccess.GPIO in result
            assert HardwareAccess.I2C in result
            assert all(isinstance(v, bool) for v in result.values())

    def test_get_required_groups(self) -> None:
        """Test get_required_groups returns correct mapping."""
        groups = get_required_groups()

        assert isinstance(groups, dict)
        assert "i2c" in groups
        assert "dialout" in groups
        assert HardwareAccess.I2C in groups["i2c"]


class TestPrivilegeError:
    """Tests for PrivilegeError exception."""

    def test_privilege_error_attributes(self) -> None:
        """Test PrivilegeError stores attributes correctly."""
        error = PrivilegeError(
            "Cannot access GPIO",
            resource="gpio",
            required_group="gpio",
            fix_command="sudo usermod -a -G gpio $USER",
        )

        assert str(error) == "Cannot access GPIO"
        assert error.resource == "gpio"
        assert error.required_group == "gpio"
        assert error.fix_command == "sudo usermod -a -G gpio $USER"


# ===========================================================================
# Data Class Tests
# ===========================================================================


class TestJointLimits:
    """Tests for JointLimits dataclass."""

    def test_creation(self) -> None:
        """Test JointLimits creation."""
        limits = JointLimits(-1.0, 1.0, "elbow")
        assert limits.min_angle == -1.0
        assert limits.max_angle == 1.0
        assert limits.name == "elbow"

    def test_defaults(self) -> None:
        """Test JointLimits defaults."""
        limits = JointLimits()
        assert limits.min_angle < 0
        assert limits.max_angle > 0
        assert limits.name == "joint"

    def test_invalid_range(self) -> None:
        """Test invalid range raises ValueError."""
        with pytest.raises(ValueError):
            JointLimits(1.0, -1.0)  # min > max
        with pytest.raises(ValueError):
            JointLimits(1.0, 1.0)  # min == max


class TestSpeedLimits:
    """Tests for SpeedLimits dataclass."""

    def test_creation(self) -> None:
        """Test SpeedLimits creation."""
        limits = SpeedLimits(0.0, 5.0, "m/s")
        assert limits.min_speed == 0.0
        assert limits.max_speed == 5.0
        assert limits.unit == "m/s"

    def test_negative_min_speed(self) -> None:
        """Test negative min_speed raises ValueError."""
        with pytest.raises(ValueError):
            SpeedLimits(-1.0, 5.0)


class TestAddressRange:
    """Tests for AddressRange dataclass."""

    def test_creation(self) -> None:
        """Test AddressRange creation."""
        range_ = AddressRange(0x10, 0x70, "custom")
        assert range_.min_address == 0x10
        assert range_.max_address == 0x70
        assert range_.name == "custom"


# ===========================================================================
# InputValidator Tests
# ===========================================================================


class TestInputValidator:
    """Tests for InputValidator composite class."""

    def test_validate_angle_by_index(self, robot_validator: InputValidator) -> None:
        """Test validating single angle by index."""
        assert robot_validator.validate_angle(0, 0.5) is True
        assert robot_validator.validate_angle(1, 0.5) is True

    def test_validate_angle_invalid_index(self, robot_validator: InputValidator) -> None:
        """Test invalid joint index raises ValidationError."""
        with pytest.raises(ValidationError):
            robot_validator.validate_angle(10, 0.5)
        with pytest.raises(ValidationError):
            robot_validator.validate_angle(-1, 0.5)

    def test_validate_angles(self, robot_validator: InputValidator) -> None:
        """Test validating all joint angles."""
        assert robot_validator.validate_angles([0.5, 0.3, -0.5]) is True

    def test_validate_speed(self, robot_validator: InputValidator) -> None:
        """Test speed validation with limits."""
        assert robot_validator.validate_speed(3.0) is True

        with pytest.raises(ValidationError):
            robot_validator.validate_speed(10.0)  # Above max_speed=5.0

    def test_validate_accel(self, robot_validator: InputValidator) -> None:
        """Test acceleration validation with limits."""
        assert robot_validator.validate_accel(10.0) is True

        with pytest.raises(ValidationError):
            robot_validator.validate_accel(30.0)  # Above max_accel=20.0

    def test_validate_i2c_address_with_allowed_list(self) -> None:
        """Test I2C validation with allowed addresses."""
        validator = InputValidator(
            allowed_i2c_addresses={0x20, 0x40, 0x68},
        )

        assert validator.validate_i2c_address(0x20) is True

        with pytest.raises(ValidationError) as exc_info:
            validator.validate_i2c_address(0x50)

        assert "not in allowed list" in str(exc_info.value)


# ===========================================================================
# ValidationError Tests
# ===========================================================================


class TestValidationError:
    """Tests for ValidationError exception."""

    def test_validation_error_attributes(self) -> None:
        """Test ValidationError stores attributes correctly."""
        error = ValidationError(
            "Invalid angle",
            field="joint_1",
            value=10.0,
            constraint=">= -3.14",
        )

        assert str(error) == "Invalid angle"
        assert error.field == "joint_1"
        assert error.value == 10.0
        assert error.constraint == ">= -3.14"

    def test_validation_error_optional_attributes(self) -> None:
        """Test ValidationError with only message."""
        error = ValidationError("Something wrong")

        assert str(error) == "Something wrong"
        assert error.field is None
        assert error.value is None
        assert error.constraint is None
