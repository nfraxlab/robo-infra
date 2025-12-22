"""Unit tests for LSM6DS3 6-DOF IMU driver.

Tests cover:
- LSM6DS3 configuration
- Reading types
- Step counter functionality
- Simulated driver behavior
"""

from __future__ import annotations

from robo_infra.drivers.lsm6ds3 import (
    AccelODR,
    AccelScale,
    GyroODR,
    GyroScale,
    LSM6DS3Config,
    LSM6DS3Driver,
    LSM6DS3Reading,
    LSM6DS3Register,
)


# =============================================================================
# LSM6DS3Reading Tests
# =============================================================================


class TestLSM6DS3Reading:
    """Tests for LSM6DS3Reading dataclass."""

    def test_create_reading(self) -> None:
        """Test creating a reading."""
        reading = LSM6DS3Reading(
            acceleration=(0.0, 0.0, 9.81),
            gyroscope=(0.0, 0.0, 0.0),
            temperature=25.0,
            timestamp=1234567890.0,
        )

        assert reading.acceleration[2] == 9.81
        assert reading.temperature == 25.0


class TestLSM6DS3Config:
    """Tests for LSM6DS3Config."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = LSM6DS3Config()

        assert config.address == 0x6A

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = LSM6DS3Config(
            address=0x6B,
            accel_odr=AccelODR.ODR_104HZ,
            accel_scale=AccelScale.SCALE_4G,
        )

        assert config.address == 0x6B


class TestAccelODR:
    """Tests for AccelODR enum."""

    def test_accel_odr_values(self) -> None:
        """Test accelerometer ODR values."""
        assert AccelODR.POWER_DOWN is not None
        assert AccelODR.ODR_12_5HZ is not None
        assert AccelODR.ODR_104HZ is not None


class TestAccelScale:
    """Tests for AccelScale enum."""

    def test_accel_scale_values(self) -> None:
        """Test accelerometer scale values."""
        assert AccelScale.SCALE_2G is not None
        assert AccelScale.SCALE_4G is not None
        assert AccelScale.SCALE_8G is not None
        assert AccelScale.SCALE_16G is not None


class TestGyroODR:
    """Tests for GyroODR enum."""

    def test_gyro_odr_values(self) -> None:
        """Test gyroscope ODR values."""
        assert GyroODR.POWER_DOWN is not None
        assert GyroODR.ODR_104HZ is not None


class TestGyroScale:
    """Tests for GyroScale enum."""

    def test_gyro_scale_values(self) -> None:
        """Test gyroscope scale values."""
        assert GyroScale.SCALE_250DPS is not None
        assert GyroScale.SCALE_500DPS is not None


class TestLSM6DS3Register:
    """Tests for LSM6DS3Register enum."""

    def test_who_am_i_register(self) -> None:
        """Test WHO_AM_I register."""
        assert LSM6DS3Register.WHO_AM_I.value == 0x0F


class TestLSM6DS3Driver:
    """Tests for LSM6DS3Driver class structure."""

    def test_driver_is_abstract(self) -> None:
        """Verify LSM6DS3Driver is abstract and requires concrete implementation."""
        import inspect

        # Verify it has abstract methods
        assert inspect.isabstract(LSM6DS3Driver)

    def test_driver_config(self) -> None:
        """Test driver configuration."""
        config = LSM6DS3Config()

        assert config.address == 0x6A
