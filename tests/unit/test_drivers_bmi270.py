"""Unit tests for BMI270 6-DOF IMU driver.

Tests cover:
- BMI270 configuration
- Reading types
- Status types
- Power modes
- Simulated driver behavior
"""

from __future__ import annotations

import pytest

from robo_infra.drivers.bmi270 import (
    BMI270Config,
    BMI270Driver,
    BMI270Reading,
    BMI270Register,
    BMI270Status,
    AccelODR,
    AccelBWP,
    AccelRange,
    GyroODR,
    GyroBWP,
    GyroRange,
    PowerMode,
)
from robo_infra.core.types import Vector3


# =============================================================================
# BMI270Reading Tests
# =============================================================================


class TestBMI270Reading:
    """Tests for BMI270Reading dataclass."""

    def test_create_reading(self) -> None:
        """Test creating a reading."""
        reading = BMI270Reading(
            acceleration=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            temperature=25.0,
            timestamp=1234567890.0,
        )

        assert reading.acceleration.z == 9.81
        assert reading.temperature == 25.0


class TestBMI270Status:
    """Tests for BMI270Status dataclass."""

    def test_create_status(self) -> None:
        """Test creating a status."""
        status = BMI270Status(
            accel_data_ready=True,
            gyro_data_ready=True,
            aux_data_ready=False,
            cmd_ready=True,
            drdy_aux=False,
            drdy_gyr=True,
            drdy_acc=True,
        )

        assert status.accel_data_ready is True
        assert status.gyro_data_ready is True
        assert status.aux_data_ready is False


class TestBMI270Config:
    """Tests for BMI270Config."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = BMI270Config()

        assert config.address == 0x68
        assert config.accel_odr == AccelODR.ODR_100HZ
        assert config.accel_range == AccelRange.RANGE_4G
        assert config.gyro_odr == GyroODR.ODR_100HZ
        assert config.gyro_range == GyroRange.RANGE_500DPS
        assert config.power_mode == PowerMode.NORMAL

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = BMI270Config(
            address=0x69,
            accel_odr=AccelODR.ODR_200HZ,
            accel_range=AccelRange.RANGE_8G,
            gyro_odr=GyroODR.ODR_200HZ,
            gyro_range=GyroRange.RANGE_1000DPS,
            power_mode=PowerMode.PERFORMANCE,
        )

        assert config.address == 0x69
        assert config.accel_odr == AccelODR.ODR_200HZ


class TestAccelODR:
    """Tests for AccelODR enum."""

    def test_accel_odr_values(self) -> None:
        """Test accelerometer ODR values."""
        assert AccelODR.ODR_0_78HZ is not None
        assert AccelODR.ODR_100HZ is not None
        assert AccelODR.ODR_1600HZ is not None


class TestAccelBWP:
    """Tests for AccelBWP enum."""

    def test_accel_bwp_values(self) -> None:
        """Test accelerometer BWP values."""
        assert AccelBWP.OSR4_AVG1 is not None
        assert AccelBWP.NORMAL_AVG4 is not None


class TestAccelRange:
    """Tests for AccelRange enum."""

    def test_accel_range_values(self) -> None:
        """Test accelerometer range values."""
        assert AccelRange.RANGE_2G is not None
        assert AccelRange.RANGE_4G is not None
        assert AccelRange.RANGE_8G is not None
        assert AccelRange.RANGE_16G is not None


class TestGyroODR:
    """Tests for GyroODR enum."""

    def test_gyro_odr_values(self) -> None:
        """Test gyroscope ODR values."""
        assert GyroODR.ODR_100HZ is not None
        assert GyroODR.ODR_3200HZ is not None


class TestGyroBWP:
    """Tests for GyroBWP enum."""

    def test_gyro_bwp_values(self) -> None:
        """Test gyroscope BWP values."""
        assert GyroBWP.OSR4 is not None
        assert GyroBWP.NORMAL is not None


class TestGyroRange:
    """Tests for GyroRange enum."""

    def test_gyro_range_values(self) -> None:
        """Test gyroscope range values."""
        assert GyroRange.RANGE_125DPS is not None
        assert GyroRange.RANGE_500DPS is not None
        assert GyroRange.RANGE_2000DPS is not None


class TestPowerMode:
    """Tests for PowerMode enum."""

    def test_power_mode_values(self) -> None:
        """Test power mode values."""
        assert PowerMode.SUSPEND is not None
        assert PowerMode.NORMAL is not None
        assert PowerMode.LOW_POWER is not None
        assert PowerMode.PERFORMANCE is not None


class TestBMI270Register:
    """Tests for BMI270Register enum."""

    def test_chip_id_register(self) -> None:
        """Test CHIP_ID register."""
        assert BMI270Register.CHIP_ID.value == 0x00

    def test_acc_conf_register(self) -> None:
        """Test ACC_CONF register."""
        assert BMI270Register.ACC_CONF.value == 0x40


class TestBMI270Driver:
    """Tests for BMI270Driver class structure."""

    def test_driver_is_abstract(self) -> None:
        """Verify BMI270Driver is abstract and requires concrete implementation."""
        import inspect

        # Verify it has abstract methods
        assert inspect.isabstract(BMI270Driver)

    def test_driver_config(self) -> None:
        """Test driver configuration."""
        config = BMI270Config()

        assert config.address == 0x68
        assert config.accel_range == AccelRange.RANGE_4G
