"""Unit tests for BNO055 9-DOF IMU driver.

Tests cover:
- BNO055 configuration
- Reading types and calibration
- Operation modes
- Register operations
"""

from __future__ import annotations

from robo_infra.core.types import Quaternion, Vector3
from robo_infra.drivers.bno055 import (
    BNO055CalibrationStatus,
    BNO055Config,
    BNO055Driver,
    BNO055OperationMode,
    BNO055PowerMode,
    BNO055Reading,
    BNO055Register,
    BNO055SystemStatus,
)


# =============================================================================
# BNO055Reading Tests
# =============================================================================


class TestBNO055Reading:
    """Tests for BNO055Reading dataclass."""

    def test_create_reading_full(self) -> None:
        """Test creating a complete reading."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(
                system=3, gyroscope=3, accelerometer=3, magnetometer=3
            ),
            temperature=25.0,
            timestamp=1234567890.0,
        )

        assert reading.euler.x == 0.0
        assert reading.quaternion.w == 1.0
        assert reading.acceleration.z == 9.81
        assert reading.temperature == 25.0


class TestBNO055CalibrationStatus:
    """Tests for BNO055CalibrationStatus."""

    def test_create_calibration_status(self) -> None:
        """Test creating calibration status."""
        status = BNO055CalibrationStatus(
            system=3, gyroscope=3, accelerometer=3, magnetometer=3
        )

        assert status.system == 3
        assert status.gyroscope == 3
        assert status.accelerometer == 3
        assert status.magnetometer == 3

    def test_calibration_status_fully_calibrated(self) -> None:
        """Test fully calibrated status."""
        status = BNO055CalibrationStatus(
            system=3, gyroscope=3, accelerometer=3, magnetometer=3
        )
        assert status.is_calibrated

    def test_calibration_status_not_calibrated(self) -> None:
        """Test not calibrated status."""
        status = BNO055CalibrationStatus(
            system=0, gyroscope=0, accelerometer=0, magnetometer=0
        )
        assert not status.is_calibrated


class TestBNO055Config:
    """Tests for BNO055Config."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = BNO055Config()

        assert config.address == 0x28
        assert config.mode == BNO055OperationMode.NDOF
        assert config.power_mode == BNO055PowerMode.NORMAL

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = BNO055Config(
            address=0x29,
            mode=BNO055OperationMode.IMU,
            power_mode=BNO055PowerMode.LOW_POWER,
        )

        assert config.address == 0x29
        assert config.mode == BNO055OperationMode.IMU


class TestBNO055OperationMode:
    """Tests for BNO055OperationMode enum."""

    def test_ndof_mode(self) -> None:
        """Test NDOF operation mode."""
        assert BNO055OperationMode.NDOF.value == 0x0C

    def test_imu_mode(self) -> None:
        """Test IMU operation mode."""
        assert BNO055OperationMode.IMU.value == 0x08

    def test_config_mode(self) -> None:
        """Test CONFIG operation mode."""
        assert BNO055OperationMode.CONFIG.value == 0x00


class TestBNO055PowerMode:
    """Tests for BNO055PowerMode enum."""

    def test_normal_power(self) -> None:
        """Test NORMAL power mode."""
        assert BNO055PowerMode.NORMAL.value == 0x00

    def test_low_power(self) -> None:
        """Test LOW_POWER mode."""
        assert BNO055PowerMode.LOW_POWER.value == 0x01


class TestBNO055Register:
    """Tests for BNO055Register enum."""

    def test_chip_id_register(self) -> None:
        """Test CHIP_ID register."""
        assert BNO055Register.CHIP_ID.value == 0x00

    def test_euler_registers(self) -> None:
        """Test Euler angle registers exist."""
        assert hasattr(BNO055Register, "EUL_HEADING_LSB")


class TestBNO055SystemStatus:
    """Tests for BNO055SystemStatus enum."""

    def test_idle_status(self) -> None:
        """Test IDLE status."""
        assert BNO055SystemStatus.IDLE.value == 0x00


class TestBNO055Driver:
    """Tests for BNO055Driver class structure."""

    def test_driver_is_abstract(self) -> None:
        """Verify BNO055Driver is abstract and requires concrete implementation."""
        import inspect

        # Verify it has abstract methods
        assert inspect.isabstract(BNO055Driver)

    def test_driver_config(self) -> None:
        """Test driver configuration."""
        config = BNO055Config()

        assert config.address == 0x28
        assert config.mode == OperationMode.NDOF
