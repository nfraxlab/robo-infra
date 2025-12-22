"""Unit tests for ICM-20948 9-DOF IMU driver.

Tests cover:
- ICM20948 configuration
- Reading types
- Register banks
- Simulated driver behavior
"""

from __future__ import annotations

from robo_infra.drivers.icm20948 import (
    AccelRange,
    GyroRange,
    ICM20948Config,
    ICM20948Driver,
    ICM20948Reading,
    ICM20948Register,
    MagMode,
)


# =============================================================================
# ICM20948Reading Tests
# =============================================================================


class TestICM20948Reading:
    """Tests for ICM20948Reading dataclass."""

    def test_create_reading(self) -> None:
        """Test creating a reading."""
        reading = ICM20948Reading(
            acceleration=(0.0, 0.0, 9.81),
            gyroscope=(0.0, 0.0, 0.0),
            magnetometer=(30.0, 0.0, 40.0),
            temperature=25.0,
            timestamp=1234567890.0,
        )

        assert reading.acceleration[2] == 9.81
        assert reading.temperature == 25.0


class TestICM20948Config:
    """Tests for ICM20948Config."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = ICM20948Config()

        assert config.address == 0x68

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = ICM20948Config(
            address=0x69,
            accel_range=AccelRange.RANGE_8G,
            gyro_range=GyroRange.RANGE_1000DPS,
        )

        assert config.address == 0x69


class TestAccelRange:
    """Tests for AccelRange enum."""

    def test_accel_ranges(self) -> None:
        """Test accelerometer ranges."""
        assert AccelRange.RANGE_2G is not None
        assert AccelRange.RANGE_4G is not None
        assert AccelRange.RANGE_8G is not None
        assert AccelRange.RANGE_16G is not None


class TestGyroRange:
    """Tests for GyroRange enum."""

    def test_gyro_ranges(self) -> None:
        """Test gyroscope ranges."""
        assert GyroRange.RANGE_250DPS is not None
        assert GyroRange.RANGE_500DPS is not None
        assert GyroRange.RANGE_1000DPS is not None
        assert GyroRange.RANGE_2000DPS is not None


class TestMagMode:
    """Tests for MagMode enum."""

    def test_mag_modes(self) -> None:
        """Test magnetometer modes."""
        assert MagMode.POWER_DOWN is not None
        assert MagMode.SINGLE is not None
        assert MagMode.CONTINUOUS_10HZ is not None


class TestICM20948Register:
    """Tests for ICM20948Register enum."""

    def test_who_am_i_register(self) -> None:
        """Test WHO_AM_I register."""
        assert ICM20948Register.WHO_AM_I.value == 0x00


class TestICM20948Driver:
    """Tests for ICM20948Driver class structure."""

    def test_driver_is_abstract(self) -> None:
        """Verify ICM20948Driver is abstract and requires concrete implementation."""
        import inspect

        # Verify it has abstract methods
        assert inspect.isabstract(ICM20948Driver)

    def test_driver_config(self) -> None:
        """Test driver configuration."""
        config = ICM20948Config()

        assert config.address == 0x68
        assert config.accel_range == AccelRange.RANGE_4G
