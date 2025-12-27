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
        status = BNO055CalibrationStatus(system=3, gyroscope=3, accelerometer=3, magnetometer=3)

        assert status.system == 3
        assert status.gyroscope == 3
        assert status.accelerometer == 3
        assert status.magnetometer == 3

    def test_calibration_status_fully_calibrated(self) -> None:
        """Test fully calibrated status."""
        status = BNO055CalibrationStatus(system=3, gyroscope=3, accelerometer=3, magnetometer=3)
        assert status.is_calibrated

    def test_calibration_status_not_calibrated(self) -> None:
        """Test not calibrated status."""
        status = BNO055CalibrationStatus(system=0, gyroscope=0, accelerometer=0, magnetometer=0)
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
        assert config.mode == BNO055OperationMode.NDOF


# =============================================================================
# Phase 5.8.3.1 - BNO055 Comprehensive Tests
# =============================================================================


class TestBNO055Initialization:
    """Tests for BNO055 initialization and configuration."""

    def test_default_address(self) -> None:
        """Test default I2C address."""
        config = BNO055Config()
        assert config.address == 0x28

    def test_alternate_address(self) -> None:
        """Test alternate I2C address (ADR pin high)."""
        config = BNO055Config(address=0x29)
        assert config.address == 0x29

    def test_config_mode_default(self) -> None:
        """Test default operation mode is NDOF."""
        config = BNO055Config()
        assert config.mode == BNO055OperationMode.NDOF

    def test_config_power_mode_default(self) -> None:
        """Test default power mode is NORMAL."""
        config = BNO055Config()
        assert config.power_mode == BNO055PowerMode.NORMAL

    def test_config_units_default(self) -> None:
        """Test default unit settings."""
        config = BNO055Config()
        assert config.use_degrees is True
        assert config.use_celsius is True
        assert config.use_mps2 is True

    def test_config_units_custom(self) -> None:
        """Test custom unit settings (radians, Fahrenheit, mg)."""
        config = BNO055Config(
            use_degrees=False,
            use_celsius=False,
            use_mps2=False,
        )
        assert config.use_degrees is False
        assert config.use_celsius is False
        assert config.use_mps2 is False

    def test_config_timeouts(self) -> None:
        """Test timeout configuration."""
        config = BNO055Config(
            reset_timeout=2.0,
            mode_switch_timeout=0.2,
        )
        assert config.reset_timeout == 2.0
        assert config.mode_switch_timeout == 0.2


class TestBNO055ReadEuler:
    """Tests for BNO055 Euler angle reading."""

    def test_euler_reading_structure(self) -> None:
        """Test Euler reading is Vector3 with heading, roll, pitch."""
        reading = BNO055Reading(
            euler=Vector3(x=45.0, y=10.0, z=-5.0),  # heading, roll, pitch
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.euler.x == 45.0  # heading
        assert reading.euler.y == 10.0  # roll
        assert reading.euler.z == -5.0  # pitch

    def test_euler_heading_range(self) -> None:
        """Test Euler heading in valid range (0-360)."""
        reading = BNO055Reading(
            euler=Vector3(x=359.0, y=0.0, z=0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert 0 <= reading.euler.x <= 360

    def test_euler_roll_pitch_range(self) -> None:
        """Test Euler roll/pitch in valid range (-180 to 180)."""
        reading = BNO055Reading(
            euler=Vector3(x=0.0, y=-90.0, z=45.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert -180 <= reading.euler.y <= 180
        assert -180 <= reading.euler.z <= 180


class TestBNO055ReadQuaternion:
    """Tests for BNO055 quaternion reading."""

    def test_quaternion_reading_structure(self) -> None:
        """Test quaternion reading has w, x, y, z components."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(w=0.707, x=0.0, y=0.707, z=0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.quaternion.w == 0.707
        assert reading.quaternion.x == 0.0
        assert reading.quaternion.y == 0.707
        assert reading.quaternion.z == 0.0

    def test_quaternion_identity(self) -> None:
        """Test identity quaternion (no rotation)."""
        quat = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        assert quat.w == 1.0
        assert quat.x == 0.0
        assert quat.y == 0.0
        assert quat.z == 0.0

    def test_quaternion_normalized(self) -> None:
        """Test quaternion components form unit quaternion."""
        # Unit quaternion: w^2 + x^2 + y^2 + z^2 = 1
        quat = Quaternion(w=0.707, x=0.0, y=0.707, z=0.0)
        magnitude = quat.w**2 + quat.x**2 + quat.y**2 + quat.z**2
        assert abs(magnitude - 1.0) < 0.01  # Approximately normalized


class TestBNO055ReadAccelerometer:
    """Tests for BNO055 accelerometer reading."""

    def test_acceleration_at_rest(self) -> None:
        """Test acceleration reading at rest (gravity only)."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert abs(reading.acceleration.z - 9.81) < 0.1

    def test_linear_acceleration_at_rest(self) -> None:
        """Test linear acceleration (gravity removed) is near zero at rest."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(x=0.0, y=0.0, z=9.81),
            linear_acceleration=Vector3(x=0.0, y=0.0, z=0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert abs(reading.linear_acceleration.x) < 0.5
        assert abs(reading.linear_acceleration.y) < 0.5
        assert abs(reading.linear_acceleration.z) < 0.5

    def test_gravity_vector_magnitude(self) -> None:
        """Test gravity vector has correct magnitude."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(x=0.0, y=0.0, z=9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        magnitude = (reading.gravity.x**2 + reading.gravity.y**2 + reading.gravity.z**2) ** 0.5
        assert abs(magnitude - 9.81) < 0.5


class TestBNO055ReadGyroscope:
    """Tests for BNO055 gyroscope reading."""

    def test_gyroscope_at_rest(self) -> None:
        """Test gyroscope reading at rest is near zero."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=0.0),
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert abs(reading.gyroscope.x) < 1.0
        assert abs(reading.gyroscope.y) < 1.0
        assert abs(reading.gyroscope.z) < 1.0

    def test_gyroscope_rotation(self) -> None:
        """Test gyroscope reading during rotation."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(x=0.0, y=0.0, z=90.0),  # 90 deg/s around Z
            magnetometer=Vector3(30.0, 0.0, 40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.gyroscope.z == 90.0


class TestBNO055ReadMagnetometer:
    """Tests for BNO055 magnetometer reading."""

    def test_magnetometer_reading(self) -> None:
        """Test magnetometer reading has x, y, z components."""
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(x=20.0, y=-5.0, z=45.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        assert reading.magnetometer.x == 20.0
        assert reading.magnetometer.y == -5.0
        assert reading.magnetometer.z == 45.0

    def test_magnetometer_field_strength(self) -> None:
        """Test magnetometer field strength is in reasonable range."""
        # Earth's magnetic field is typically 25-65 µT
        reading = BNO055Reading(
            euler=Vector3(0.0, 0.0, 0.0),
            quaternion=Quaternion(1.0, 0.0, 0.0, 0.0),
            acceleration=Vector3(0.0, 0.0, 9.81),
            linear_acceleration=Vector3(0.0, 0.0, 0.0),
            gravity=Vector3(0.0, 0.0, 9.81),
            gyroscope=Vector3(0.0, 0.0, 0.0),
            magnetometer=Vector3(x=30.0, y=0.0, z=40.0),
            calibration=BNO055CalibrationStatus(3, 3, 3, 3),
            temperature=25.0,
            timestamp=0.0,
        )
        magnitude = (
            reading.magnetometer.x**2 + reading.magnetometer.y**2 + reading.magnetometer.z**2
        ) ** 0.5
        assert 20.0 < magnitude < 70.0


class TestBNO055CalibrationStatusAdvanced:
    """Advanced tests for BNO055 calibration status."""

    def test_partially_calibrated(self) -> None:
        """Test partially calibrated status."""
        status = BNO055CalibrationStatus(system=2, gyroscope=3, accelerometer=1, magnetometer=2)
        assert status.is_partially_calibrated is True
        assert status.is_calibrated is False

    def test_uncalibrated(self) -> None:
        """Test completely uncalibrated status."""
        status = BNO055CalibrationStatus(system=0, gyroscope=1, accelerometer=0, magnetometer=0)
        # At least one sensor has value >= 2
        assert status.is_partially_calibrated is False

    def test_calibration_level_range(self) -> None:
        """Test calibration levels are in range 0-3."""
        for level in range(4):
            status = BNO055CalibrationStatus(
                system=level, gyroscope=level, accelerometer=level, magnetometer=level
            )
            assert 0 <= status.system <= 3
            assert 0 <= status.gyroscope <= 3


class TestBNO055ModeSwitching:
    """Tests for BNO055 operation mode switching."""

    def test_fusion_modes(self) -> None:
        """Test fusion mode values."""
        assert BNO055OperationMode.IMU == 0x08
        assert BNO055OperationMode.COMPASS == 0x09
        assert BNO055OperationMode.NDOF == 0x0C

    def test_non_fusion_modes(self) -> None:
        """Test non-fusion mode values."""
        assert BNO055OperationMode.ACCONLY == 0x01
        assert BNO055OperationMode.MAGONLY == 0x02
        assert BNO055OperationMode.GYROONLY == 0x03
        assert BNO055OperationMode.AMG == 0x07

    def test_config_mode_value(self) -> None:
        """Test config mode value."""
        assert BNO055OperationMode.CONFIG == 0x00

    def test_mode_enum_completeness(self) -> None:
        """Test all expected modes exist."""
        modes = [
            "CONFIG",
            "ACCONLY",
            "MAGONLY",
            "GYROONLY",
            "ACCMAG",
            "ACCGYRO",
            "MAGGYRO",
            "AMG",
            "IMU",
            "COMPASS",
            "M4G",
            "NDOF_FMC_OFF",
            "NDOF",
        ]
        for mode in modes:
            assert hasattr(BNO055OperationMode, mode)


class TestBNO055RegisterAddresses:
    """Tests for BNO055 register addresses."""

    def test_data_registers(self) -> None:
        """Test data register addresses."""
        assert BNO055Register.ACC_DATA_X_LSB == 0x08
        assert BNO055Register.MAG_DATA_X_LSB == 0x0E
        assert BNO055Register.GYR_DATA_X_LSB == 0x14
        assert BNO055Register.EUL_HEADING_LSB == 0x1A
        assert BNO055Register.QUA_DATA_W_LSB == 0x20

    def test_control_registers(self) -> None:
        """Test control register addresses."""
        assert BNO055Register.OPR_MODE == 0x3D
        assert BNO055Register.PWR_MODE == 0x3E
        assert BNO055Register.SYS_TRIGGER == 0x3F

    def test_calibration_registers(self) -> None:
        """Test calibration register addresses."""
        assert BNO055Register.CALIB_STAT == 0x35
        assert BNO055Register.ACC_OFFSET_X_LSB == 0x55
        assert BNO055Register.MAG_OFFSET_X_LSB == 0x5B
        assert BNO055Register.GYR_OFFSET_X_LSB == 0x61


class TestBNO055ChannelMapping:
    """Tests for BNO055 channel mapping constants."""

    def test_channel_constants_exist(self) -> None:
        """Test channel mapping constants are defined."""
        assert hasattr(BNO055Driver, "CHANNEL_ACCEL_X")
        assert hasattr(BNO055Driver, "CHANNEL_GYRO_X")
        assert hasattr(BNO055Driver, "CHANNEL_MAG_X")
        assert hasattr(BNO055Driver, "CHANNEL_EULER_HEADING")
        assert hasattr(BNO055Driver, "CHANNEL_QUAT_W")
        assert hasattr(BNO055Driver, "CHANNEL_TEMP")

    def test_channel_values_unique(self) -> None:
        """Test channel values are unique."""
        channels = [
            BNO055Driver.CHANNEL_ACCEL_X,
            BNO055Driver.CHANNEL_ACCEL_Y,
            BNO055Driver.CHANNEL_ACCEL_Z,
            BNO055Driver.CHANNEL_GYRO_X,
            BNO055Driver.CHANNEL_GYRO_Y,
            BNO055Driver.CHANNEL_GYRO_Z,
            BNO055Driver.CHANNEL_MAG_X,
            BNO055Driver.CHANNEL_MAG_Y,
            BNO055Driver.CHANNEL_MAG_Z,
            BNO055Driver.CHANNEL_EULER_HEADING,
            BNO055Driver.CHANNEL_EULER_ROLL,
            BNO055Driver.CHANNEL_EULER_PITCH,
            BNO055Driver.CHANNEL_QUAT_W,
            BNO055Driver.CHANNEL_QUAT_X,
            BNO055Driver.CHANNEL_QUAT_Y,
            BNO055Driver.CHANNEL_QUAT_Z,
            BNO055Driver.CHANNEL_TEMP,
        ]
        assert len(channels) == len(set(channels))
