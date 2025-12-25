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


# =============================================================================
# Phase 5.8.3.2 - ICM20948 Comprehensive Tests
# =============================================================================


class TestICM20948Initialization:
    """Tests for ICM20948 initialization and configuration."""

    def test_default_address(self) -> None:
        """Test default I2C address."""
        config = ICM20948Config()
        assert config.address == 0x68

    def test_alternate_address(self) -> None:
        """Test alternate I2C address (AD0 pin high)."""
        config = ICM20948Config(address=0x69)
        assert config.address == 0x69

    def test_default_accel_range(self) -> None:
        """Test default accelerometer range."""
        config = ICM20948Config()
        assert config.accel_range == AccelRange.RANGE_4G

    def test_default_gyro_range(self) -> None:
        """Test default gyroscope range."""
        config = ICM20948Config()
        assert config.gyro_range == GyroRange.RANGE_500DPS

    def test_default_mag_mode(self) -> None:
        """Test default magnetometer mode."""
        config = ICM20948Config()
        assert config.mag_mode == MagMode.CONTINUOUS_100HZ

    def test_default_sample_rate(self) -> None:
        """Test default sample rate."""
        config = ICM20948Config()
        assert config.sample_rate_hz == 100

    def test_sample_rate_range(self) -> None:
        """Test sample rate range validation."""
        config = ICM20948Config(sample_rate_hz=500)
        assert config.sample_rate_hz == 500


class TestICM20948ReadAccel:
    """Tests for ICM20948 accelerometer reading."""

    def test_accel_reading_structure(self) -> None:
        """Test acceleration reading structure."""
        from robo_infra.core.types import Vector3
        reading = ICM20948Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            magnetometer=Vector3(x=30.0, y=0.0, z=40.0),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.acceleration.z == 9.81

    def test_accel_ranges_all(self) -> None:
        """Test all accelerometer ranges are defined."""
        assert AccelRange.RANGE_2G.value == 0
        assert AccelRange.RANGE_4G.value == 1
        assert AccelRange.RANGE_8G.value == 2
        assert AccelRange.RANGE_16G.value == 3

    def test_accel_scale_factors(self) -> None:
        """Test accelerometer scale factors exist."""
        assert ICM20948Driver.ACCEL_SCALE[AccelRange.RANGE_2G] == 16384.0
        assert ICM20948Driver.ACCEL_SCALE[AccelRange.RANGE_4G] == 8192.0
        assert ICM20948Driver.ACCEL_SCALE[AccelRange.RANGE_8G] == 4096.0
        assert ICM20948Driver.ACCEL_SCALE[AccelRange.RANGE_16G] == 2048.0


class TestICM20948ReadGyro:
    """Tests for ICM20948 gyroscope reading."""

    def test_gyro_reading_structure(self) -> None:
        """Test gyroscope reading structure."""
        from robo_infra.core.types import Vector3
        reading = ICM20948Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=10.0, y=20.0, z=30.0),
            magnetometer=Vector3(x=30.0, y=0.0, z=40.0),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.gyroscope.x == 10.0
        assert reading.gyroscope.y == 20.0
        assert reading.gyroscope.z == 30.0

    def test_gyro_ranges_all(self) -> None:
        """Test all gyroscope ranges are defined."""
        assert GyroRange.RANGE_250DPS.value == 0
        assert GyroRange.RANGE_500DPS.value == 1
        assert GyroRange.RANGE_1000DPS.value == 2
        assert GyroRange.RANGE_2000DPS.value == 3

    def test_gyro_scale_factors(self) -> None:
        """Test gyroscope scale factors exist."""
        assert ICM20948Driver.GYRO_SCALE[GyroRange.RANGE_250DPS] == 131.0
        assert ICM20948Driver.GYRO_SCALE[GyroRange.RANGE_500DPS] == 65.5
        assert ICM20948Driver.GYRO_SCALE[GyroRange.RANGE_1000DPS] == 32.8
        assert ICM20948Driver.GYRO_SCALE[GyroRange.RANGE_2000DPS] == 16.4


class TestICM20948ReadMag:
    """Tests for ICM20948 magnetometer reading."""

    def test_mag_reading_structure(self) -> None:
        """Test magnetometer reading structure."""
        from robo_infra.core.types import Vector3
        reading = ICM20948Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            magnetometer=Vector3(x=30.0, y=-10.0, z=45.0),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.magnetometer.x == 30.0
        assert reading.magnetometer.y == -10.0
        assert reading.magnetometer.z == 45.0

    def test_mag_modes_all(self) -> None:
        """Test all magnetometer modes are defined."""
        assert MagMode.POWER_DOWN.value == 0x00
        assert MagMode.SINGLE.value == 0x01
        assert MagMode.CONTINUOUS_10HZ.value == 0x02
        assert MagMode.CONTINUOUS_20HZ.value == 0x04
        assert MagMode.CONTINUOUS_50HZ.value == 0x06
        assert MagMode.CONTINUOUS_100HZ.value == 0x08
        assert MagMode.SELF_TEST.value == 0x10

    def test_mag_scale_factor(self) -> None:
        """Test magnetometer scale factor."""
        assert ICM20948Driver.MAG_SCALE == 0.15  # µT/LSB


class TestICM20948WhoAmI:
    """Tests for ICM20948 WHO_AM_I identification."""

    def test_who_am_i_register(self) -> None:
        """Test WHO_AM_I register address."""
        assert ICM20948Register.WHO_AM_I.value == 0x00

    def test_who_am_i_expected_value(self) -> None:
        """Test expected WHO_AM_I value."""
        from robo_infra.drivers.icm20948 import ICM20948_WHO_AM_I
        assert ICM20948_WHO_AM_I == 0xEA

    def test_ak09916_who_am_i(self) -> None:
        """Test AK09916 magnetometer WHO_AM_I value."""
        from robo_infra.drivers.icm20948 import AK09916_WHO_AM_I
        assert AK09916_WHO_AM_I == 0x09


class TestICM20948Calibration:
    """Tests for ICM20948 calibration configuration."""

    def test_dlpf_config_accel(self) -> None:
        """Test accelerometer DLPF configuration."""
        config = ICM20948Config(accel_dlpf=5)
        assert config.accel_dlpf == 5

    def test_dlpf_config_gyro(self) -> None:
        """Test gyroscope DLPF configuration."""
        config = ICM20948Config(gyro_dlpf=6)
        assert config.gyro_dlpf == 6

    def test_dlpf_range_validation(self) -> None:
        """Test DLPF value range (0-7)."""
        config = ICM20948Config(accel_dlpf=0, gyro_dlpf=7)
        assert 0 <= config.accel_dlpf <= 7
        assert 0 <= config.gyro_dlpf <= 7


class TestICM20948RegisterBanks:
    """Tests for ICM20948 register bank switching."""

    def test_bank_0_registers(self) -> None:
        """Test Bank 0 register addresses."""
        assert ICM20948Register.WHO_AM_I == 0x00
        assert ICM20948Register.USER_CTRL == 0x03
        assert ICM20948Register.PWR_MGMT_1 == 0x06
        assert ICM20948Register.ACCEL_XOUT_H == 0x2D
        assert ICM20948Register.GYRO_XOUT_H == 0x33

    def test_bank_select_register(self) -> None:
        """Test bank select register address."""
        assert ICM20948Register.REG_BANK_SEL == 0x7F

    def test_bank_enum_values(self) -> None:
        """Test bank enum values."""
        from robo_infra.drivers.icm20948 import ICM20948Bank
        assert ICM20948Bank.BANK_0 == 0x00
        assert ICM20948Bank.BANK_1 == 0x10
        assert ICM20948Bank.BANK_2 == 0x20
        assert ICM20948Bank.BANK_3 == 0x30


class TestICM20948ChannelMapping:
    """Tests for ICM20948 channel mapping constants."""

    def test_channel_constants_exist(self) -> None:
        """Test channel mapping constants are defined."""
        assert hasattr(ICM20948Driver, "CHANNEL_ACCEL_X")
        assert hasattr(ICM20948Driver, "CHANNEL_ACCEL_Y")
        assert hasattr(ICM20948Driver, "CHANNEL_ACCEL_Z")
        assert hasattr(ICM20948Driver, "CHANNEL_GYRO_X")
        assert hasattr(ICM20948Driver, "CHANNEL_GYRO_Y")
        assert hasattr(ICM20948Driver, "CHANNEL_GYRO_Z")
        assert hasattr(ICM20948Driver, "CHANNEL_MAG_X")
        assert hasattr(ICM20948Driver, "CHANNEL_MAG_Y")
        assert hasattr(ICM20948Driver, "CHANNEL_MAG_Z")
        assert hasattr(ICM20948Driver, "CHANNEL_TEMP")

    def test_channel_values_unique(self) -> None:
        """Test channel values are unique."""
        channels = [
            ICM20948Driver.CHANNEL_ACCEL_X,
            ICM20948Driver.CHANNEL_ACCEL_Y,
            ICM20948Driver.CHANNEL_ACCEL_Z,
            ICM20948Driver.CHANNEL_GYRO_X,
            ICM20948Driver.CHANNEL_GYRO_Y,
            ICM20948Driver.CHANNEL_GYRO_Z,
            ICM20948Driver.CHANNEL_MAG_X,
            ICM20948Driver.CHANNEL_MAG_Y,
            ICM20948Driver.CHANNEL_MAG_Z,
            ICM20948Driver.CHANNEL_TEMP,
        ]
        assert len(channels) == len(set(channels))


class TestICM20948TemperatureSensor:
    """Tests for ICM20948 temperature sensor."""

    def test_temperature_reading(self) -> None:
        """Test temperature reading structure."""
        from robo_infra.core.types import Vector3
        reading = ICM20948Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            magnetometer=Vector3(x=30.0, y=0.0, z=40.0),
            temperature=27.5,
            timestamp=0.0,
        )
        assert reading.temperature == 27.5

    def test_temperature_constants(self) -> None:
        """Test temperature conversion constants."""
        assert ICM20948Driver.TEMP_SENSITIVITY == 333.87
        assert ICM20948Driver.TEMP_OFFSET == 21.0


class TestICM20948ReadingTimestamp:
    """Tests for ICM20948 reading timestamps."""

    def test_reading_has_timestamp(self) -> None:
        """Test reading includes timestamp."""
        from robo_infra.core.types import Vector3
        reading = ICM20948Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            magnetometer=Vector3(x=30.0, y=0.0, z=40.0),
            temperature=25.0,
            timestamp=1234567890.123,
        )
        assert reading.timestamp == 1234567890.123

    def test_timestamp_positive(self) -> None:
        """Test timestamp is positive."""
        from robo_infra.core.types import Vector3
        reading = ICM20948Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            magnetometer=Vector3(x=30.0, y=0.0, z=40.0),
            temperature=25.0,
            timestamp=100.0,
        )
        assert reading.timestamp >= 0
