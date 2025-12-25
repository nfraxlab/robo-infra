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


# =============================================================================
# Phase 5.8.3.3 - LSM6DS3 Comprehensive Tests
# =============================================================================


class TestLSM6DS3Initialization:
    """Tests for LSM6DS3 initialization and configuration."""

    def test_default_address(self) -> None:
        """Test default I2C address."""
        config = LSM6DS3Config()
        assert config.address == 0x6A

    def test_alternate_address(self) -> None:
        """Test alternate I2C address (SA0 pin high)."""
        config = LSM6DS3Config(address=0x6B)
        assert config.address == 0x6B

    def test_default_accel_odr(self) -> None:
        """Test default accelerometer output data rate."""
        config = LSM6DS3Config()
        assert config.accel_odr == AccelODR.ODR_104HZ

    def test_default_accel_scale(self) -> None:
        """Test default accelerometer scale."""
        config = LSM6DS3Config()
        assert config.accel_scale == AccelScale.SCALE_4G

    def test_default_gyro_odr(self) -> None:
        """Test default gyroscope output data rate."""
        config = LSM6DS3Config()
        assert config.gyro_odr == GyroODR.ODR_104HZ

    def test_default_gyro_scale(self) -> None:
        """Test default gyroscope scale."""
        config = LSM6DS3Config()
        assert config.gyro_scale == GyroScale.SCALE_500DPS

    def test_block_data_update_default(self) -> None:
        """Test block data update is enabled by default."""
        config = LSM6DS3Config()
        assert config.bdu is True


class TestLSM6DS3ReadAccel:
    """Tests for LSM6DS3 accelerometer reading."""

    def test_accel_reading_structure(self) -> None:
        """Test acceleration reading structure."""
        from robo_infra.core.types import Vector3
        reading = LSM6DS3Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.acceleration.z == 9.81

    def test_accel_odr_all_values(self) -> None:
        """Test all accelerometer ODR values are defined."""
        assert AccelODR.POWER_DOWN.value == 0
        assert AccelODR.ODR_12_5HZ.value == 1
        assert AccelODR.ODR_26HZ.value == 2
        assert AccelODR.ODR_52HZ.value == 3
        assert AccelODR.ODR_104HZ.value == 4
        assert AccelODR.ODR_208HZ.value == 5
        assert AccelODR.ODR_416HZ.value == 6
        assert AccelODR.ODR_833HZ.value == 7
        assert AccelODR.ODR_1660HZ.value == 8
        assert AccelODR.ODR_3330HZ.value == 9
        assert AccelODR.ODR_6660HZ.value == 10

    def test_accel_scale_all_values(self) -> None:
        """Test all accelerometer scale values are defined."""
        assert AccelScale.SCALE_2G.value == 0
        assert AccelScale.SCALE_16G.value == 1
        assert AccelScale.SCALE_4G.value == 2
        assert AccelScale.SCALE_8G.value == 3

    def test_accel_sensitivity_factors(self) -> None:
        """Test accelerometer sensitivity factors."""
        assert LSM6DS3Driver.ACCEL_SENSITIVITY[AccelScale.SCALE_2G] == 16384.0
        assert LSM6DS3Driver.ACCEL_SENSITIVITY[AccelScale.SCALE_4G] == 8192.0
        assert LSM6DS3Driver.ACCEL_SENSITIVITY[AccelScale.SCALE_8G] == 4096.0
        assert LSM6DS3Driver.ACCEL_SENSITIVITY[AccelScale.SCALE_16G] == 1365.33


class TestLSM6DS3ReadGyro:
    """Tests for LSM6DS3 gyroscope reading."""

    def test_gyro_reading_structure(self) -> None:
        """Test gyroscope reading structure."""
        from robo_infra.core.types import Vector3
        reading = LSM6DS3Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=10.0, y=-5.0, z=15.0),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.gyroscope.x == 10.0
        assert reading.gyroscope.y == -5.0
        assert reading.gyroscope.z == 15.0

    def test_gyro_odr_all_values(self) -> None:
        """Test all gyroscope ODR values are defined."""
        assert GyroODR.POWER_DOWN.value == 0
        assert GyroODR.ODR_12_5HZ.value == 1
        assert GyroODR.ODR_26HZ.value == 2
        assert GyroODR.ODR_52HZ.value == 3
        assert GyroODR.ODR_104HZ.value == 4
        assert GyroODR.ODR_208HZ.value == 5
        assert GyroODR.ODR_416HZ.value == 6
        assert GyroODR.ODR_833HZ.value == 7
        assert GyroODR.ODR_1660HZ.value == 8

    def test_gyro_scale_all_values(self) -> None:
        """Test all gyroscope scale values are defined."""
        assert GyroScale.SCALE_250DPS.value == 0
        assert GyroScale.SCALE_500DPS.value == 1
        assert GyroScale.SCALE_1000DPS.value == 2
        assert GyroScale.SCALE_2000DPS.value == 3

    def test_gyro_sensitivity_factors(self) -> None:
        """Test gyroscope sensitivity factors."""
        assert LSM6DS3Driver.GYRO_SENSITIVITY[GyroScale.SCALE_250DPS] == 131.0
        assert LSM6DS3Driver.GYRO_SENSITIVITY[GyroScale.SCALE_500DPS] == 65.5
        assert LSM6DS3Driver.GYRO_SENSITIVITY[GyroScale.SCALE_1000DPS] == 32.8
        assert LSM6DS3Driver.GYRO_SENSITIVITY[GyroScale.SCALE_2000DPS] == 16.4


class TestLSM6DS3FIFO:
    """Tests for LSM6DS3 FIFO functionality."""

    def test_fifo_registers_exist(self) -> None:
        """Test FIFO register addresses are defined."""
        assert LSM6DS3Register.FIFO_CTRL1 == 0x06
        assert LSM6DS3Register.FIFO_CTRL2 == 0x07
        assert LSM6DS3Register.FIFO_CTRL3 == 0x08
        assert LSM6DS3Register.FIFO_CTRL4 == 0x09
        assert LSM6DS3Register.FIFO_CTRL5 == 0x0A

    def test_fifo_status_registers(self) -> None:
        """Test FIFO status register addresses."""
        assert LSM6DS3Register.FIFO_STATUS1 == 0x3A
        assert LSM6DS3Register.FIFO_STATUS2 == 0x3B
        assert LSM6DS3Register.FIFO_STATUS3 == 0x3C
        assert LSM6DS3Register.FIFO_STATUS4 == 0x3D

    def test_fifo_data_registers(self) -> None:
        """Test FIFO data register addresses."""
        assert LSM6DS3Register.FIFO_DATA_OUT_L == 0x3E
        assert LSM6DS3Register.FIFO_DATA_OUT_H == 0x3F


class TestLSM6DS3RegisterAddresses:
    """Tests for LSM6DS3 register addresses."""

    def test_who_am_i_register(self) -> None:
        """Test WHO_AM_I register address."""
        assert LSM6DS3Register.WHO_AM_I == 0x0F

    def test_who_am_i_expected_value(self) -> None:
        """Test expected WHO_AM_I value."""
        from robo_infra.drivers.lsm6ds3 import LSM6DS3_WHO_AM_I
        assert LSM6DS3_WHO_AM_I == 0x69

    def test_control_registers(self) -> None:
        """Test control register addresses."""
        assert LSM6DS3Register.CTRL1_XL == 0x10  # Accelerometer
        assert LSM6DS3Register.CTRL2_G == 0x11   # Gyroscope
        assert LSM6DS3Register.CTRL3_C == 0x12   # Common

    def test_data_output_registers(self) -> None:
        """Test data output register addresses."""
        assert LSM6DS3Register.OUT_TEMP_L == 0x20
        assert LSM6DS3Register.OUTX_L_G == 0x22   # Gyro X
        assert LSM6DS3Register.OUTX_L_XL == 0x28  # Accel X

    def test_status_register(self) -> None:
        """Test status register address."""
        assert LSM6DS3Register.STATUS_REG == 0x1E


class TestLSM6DS3TemperatureSensor:
    """Tests for LSM6DS3 temperature sensor."""

    def test_temperature_reading(self) -> None:
        """Test temperature reading structure."""
        from robo_infra.core.types import Vector3
        reading = LSM6DS3Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            temperature=26.5,
            timestamp=0.0,
        )
        assert reading.temperature == 26.5

    def test_temperature_constants(self) -> None:
        """Test temperature conversion constants."""
        assert LSM6DS3Driver.TEMP_SENSITIVITY == 16.0
        assert LSM6DS3Driver.TEMP_OFFSET == 25.0


class TestLSM6DS3StepCounter:
    """Tests for LSM6DS3 step counter (pedometer) functionality."""

    def test_step_counter_registers(self) -> None:
        """Test step counter register addresses."""
        assert LSM6DS3Register.STEP_COUNTER_L == 0x4B
        assert LSM6DS3Register.STEP_COUNTER_H == 0x4C
        assert LSM6DS3Register.STEP_TIMESTAMP_L == 0x49
        assert LSM6DS3Register.STEP_TIMESTAMP_H == 0x4A

    def test_pedometer_config_register(self) -> None:
        """Test pedometer configuration registers."""
        assert LSM6DS3Register.CTRL10_C == 0x19
        assert LSM6DS3Register.TAP_CFG == 0x58

    def test_func_src_register(self) -> None:
        """Test function source register."""
        assert LSM6DS3Register.FUNC_SRC == 0x53


class TestLSM6DS3ChannelMapping:
    """Tests for LSM6DS3 channel mapping constants."""

    def test_channel_constants_exist(self) -> None:
        """Test channel mapping constants are defined."""
        assert hasattr(LSM6DS3Driver, "CHANNEL_ACCEL_X")
        assert hasattr(LSM6DS3Driver, "CHANNEL_ACCEL_Y")
        assert hasattr(LSM6DS3Driver, "CHANNEL_ACCEL_Z")
        assert hasattr(LSM6DS3Driver, "CHANNEL_GYRO_X")
        assert hasattr(LSM6DS3Driver, "CHANNEL_GYRO_Y")
        assert hasattr(LSM6DS3Driver, "CHANNEL_GYRO_Z")
        assert hasattr(LSM6DS3Driver, "CHANNEL_TEMP")

    def test_channel_values_sequential(self) -> None:
        """Test channel values are sequential."""
        assert LSM6DS3Driver.CHANNEL_ACCEL_X == 0
        assert LSM6DS3Driver.CHANNEL_ACCEL_Y == 1
        assert LSM6DS3Driver.CHANNEL_ACCEL_Z == 2
        assert LSM6DS3Driver.CHANNEL_GYRO_X == 3
        assert LSM6DS3Driver.CHANNEL_GYRO_Y == 4
        assert LSM6DS3Driver.CHANNEL_GYRO_Z == 5
        assert LSM6DS3Driver.CHANNEL_TEMP == 6


class TestLSM6DS3InterruptConfig:
    """Tests for LSM6DS3 interrupt configuration."""

    def test_interrupt_registers(self) -> None:
        """Test interrupt register addresses."""
        assert LSM6DS3Register.INT1_CTRL == 0x0D
        assert LSM6DS3Register.INT2_CTRL == 0x0E

    def test_wake_up_registers(self) -> None:
        """Test wake-up register addresses."""
        assert LSM6DS3Register.WAKE_UP_SRC == 0x1B
        assert LSM6DS3Register.WAKE_UP_THS == 0x5B
        assert LSM6DS3Register.WAKE_UP_DUR == 0x5C

    def test_tap_registers(self) -> None:
        """Test tap detection register addresses."""
        assert LSM6DS3Register.TAP_SRC == 0x1C
        assert LSM6DS3Register.TAP_THS_6D == 0x59
        assert LSM6DS3Register.INT_DUR2 == 0x5A

    def test_free_fall_register(self) -> None:
        """Test free fall register address."""
        assert LSM6DS3Register.FREE_FALL == 0x5D


class TestLSM6DS3ReadingTimestamp:
    """Tests for LSM6DS3 reading timestamps."""

    def test_reading_has_timestamp(self) -> None:
        """Test reading includes timestamp."""
        from robo_infra.core.types import Vector3
        reading = LSM6DS3Reading(
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            temperature=25.0,
            timestamp=1234567890.456,
        )
        assert reading.timestamp == 1234567890.456

    def test_timestamp_registers(self) -> None:
        """Test timestamp register addresses."""
        assert LSM6DS3Register.TIMESTAMP0_REG == 0x40
        assert LSM6DS3Register.TIMESTAMP1_REG == 0x41
        assert LSM6DS3Register.TIMESTAMP2_REG == 0x42


class TestLSM6DS3ConfigOptions:
    """Tests for LSM6DS3 configuration options."""

    def test_custom_accel_config(self) -> None:
        """Test custom accelerometer configuration."""
        config = LSM6DS3Config(
            accel_odr=AccelODR.ODR_416HZ,
            accel_scale=AccelScale.SCALE_8G,
        )
        assert config.accel_odr == AccelODR.ODR_416HZ
        assert config.accel_scale == AccelScale.SCALE_8G

    def test_custom_gyro_config(self) -> None:
        """Test custom gyroscope configuration."""
        config = LSM6DS3Config(
            gyro_odr=GyroODR.ODR_833HZ,
            gyro_scale=GyroScale.SCALE_2000DPS,
        )
        assert config.gyro_odr == GyroODR.ODR_833HZ
        assert config.gyro_scale == GyroScale.SCALE_2000DPS

    def test_block_data_update_disabled(self) -> None:
        """Test block data update can be disabled."""
        config = LSM6DS3Config(bdu=False)
        assert config.bdu is False

    def test_metadata_field(self) -> None:
        """Test metadata field in config."""
        config = LSM6DS3Config(metadata={"sensor_id": "imu_1"})
        assert config.metadata["sensor_id"] == "imu_1"
