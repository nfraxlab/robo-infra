"""BNO055 9-DOF Absolute Orientation Sensor driver.

The BNO055 from Bosch Sensortec is a System-in-Package (SiP) combining:
- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer
- 32-bit ARM Cortex M0+ microcontroller with sensor fusion algorithms

The sensor provides:
- Absolute orientation (Euler angles or Quaternion)
- Acceleration (linear and gravity)
- Angular velocity (gyroscope)
- Magnetic field strength

Key Features:
- On-chip sensor fusion (no external processing required)
- Multiple operation modes (NDOF, IMU, COMPASS, etc.)
- Auto-calibration with calibration status reporting
- I2C and UART interfaces

Notes:
- Default I2C address is 0x28 (or 0x29 with ADR pin high)
- Requires calibration for accurate readings
- Temperature-compensated sensor data
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.driver import Driver
from robo_infra.core.exceptions import (
    CommunicationError,
)
from robo_infra.core.types import Quaternion, Vector3


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

BNO055_DEFAULT_ADDRESS = 0x28
BNO055_ALT_ADDRESS = 0x29
BNO055_CHIP_ID = 0xA0


# =============================================================================
# Register Definitions
# =============================================================================


class BNO055Register(IntEnum):
    """BNO055 register addresses (Page 0)."""

    # Chip identification
    CHIP_ID = 0x00
    ACC_ID = 0x01
    MAG_ID = 0x02
    GYR_ID = 0x03
    SW_REV_ID_LSB = 0x04
    SW_REV_ID_MSB = 0x05
    BL_REV_ID = 0x06

    # Page ID
    PAGE_ID = 0x07

    # Accelerometer data
    ACC_DATA_X_LSB = 0x08
    ACC_DATA_X_MSB = 0x09
    ACC_DATA_Y_LSB = 0x0A
    ACC_DATA_Y_MSB = 0x0B
    ACC_DATA_Z_LSB = 0x0C
    ACC_DATA_Z_MSB = 0x0D

    # Magnetometer data
    MAG_DATA_X_LSB = 0x0E
    MAG_DATA_X_MSB = 0x0F
    MAG_DATA_Y_LSB = 0x10
    MAG_DATA_Y_MSB = 0x11
    MAG_DATA_Z_LSB = 0x12
    MAG_DATA_Z_MSB = 0x13

    # Gyroscope data
    GYR_DATA_X_LSB = 0x14
    GYR_DATA_X_MSB = 0x15
    GYR_DATA_Y_LSB = 0x16
    GYR_DATA_Y_MSB = 0x17
    GYR_DATA_Z_LSB = 0x18
    GYR_DATA_Z_MSB = 0x19

    # Euler angles
    EUL_HEADING_LSB = 0x1A
    EUL_HEADING_MSB = 0x1B
    EUL_ROLL_LSB = 0x1C
    EUL_ROLL_MSB = 0x1D
    EUL_PITCH_LSB = 0x1E
    EUL_PITCH_MSB = 0x1F

    # Quaternion data
    QUA_DATA_W_LSB = 0x20
    QUA_DATA_W_MSB = 0x21
    QUA_DATA_X_LSB = 0x22
    QUA_DATA_X_MSB = 0x23
    QUA_DATA_Y_LSB = 0x24
    QUA_DATA_Y_MSB = 0x25
    QUA_DATA_Z_LSB = 0x26
    QUA_DATA_Z_MSB = 0x27

    # Linear acceleration (without gravity)
    LIA_DATA_X_LSB = 0x28
    LIA_DATA_X_MSB = 0x29
    LIA_DATA_Y_LSB = 0x2A
    LIA_DATA_Y_MSB = 0x2B
    LIA_DATA_Z_LSB = 0x2C
    LIA_DATA_Z_MSB = 0x2D

    # Gravity vector
    GRV_DATA_X_LSB = 0x2E
    GRV_DATA_X_MSB = 0x2F
    GRV_DATA_Y_LSB = 0x30
    GRV_DATA_Y_MSB = 0x31
    GRV_DATA_Z_LSB = 0x32
    GRV_DATA_Z_MSB = 0x33

    # Temperature
    TEMP = 0x34

    # Calibration status
    CALIB_STAT = 0x35
    ST_RESULT = 0x36
    INT_STA = 0x37
    SYS_CLK_STATUS = 0x38
    SYS_STATUS = 0x39
    SYS_ERR = 0x3A

    # Unit selection
    UNIT_SEL = 0x3B

    # Operation mode
    OPR_MODE = 0x3D
    PWR_MODE = 0x3E

    # System trigger
    SYS_TRIGGER = 0x3F

    # Temperature source
    TEMP_SOURCE = 0x40

    # Axis mapping
    AXIS_MAP_CONFIG = 0x41
    AXIS_MAP_SIGN = 0x42

    # Accelerometer offset
    ACC_OFFSET_X_LSB = 0x55
    ACC_OFFSET_X_MSB = 0x56
    ACC_OFFSET_Y_LSB = 0x57
    ACC_OFFSET_Y_MSB = 0x58
    ACC_OFFSET_Z_LSB = 0x59
    ACC_OFFSET_Z_MSB = 0x5A

    # Magnetometer offset
    MAG_OFFSET_X_LSB = 0x5B
    MAG_OFFSET_X_MSB = 0x5C
    MAG_OFFSET_Y_LSB = 0x5D
    MAG_OFFSET_Y_MSB = 0x5E
    MAG_OFFSET_Z_LSB = 0x5F
    MAG_OFFSET_Z_MSB = 0x60

    # Gyroscope offset
    GYR_OFFSET_X_LSB = 0x61
    GYR_OFFSET_X_MSB = 0x62
    GYR_OFFSET_Y_LSB = 0x63
    GYR_OFFSET_Y_MSB = 0x64
    GYR_OFFSET_Z_LSB = 0x65
    GYR_OFFSET_Z_MSB = 0x66

    # Accelerometer radius
    ACC_RADIUS_LSB = 0x67
    ACC_RADIUS_MSB = 0x68

    # Magnetometer radius
    MAG_RADIUS_LSB = 0x69
    MAG_RADIUS_MSB = 0x6A


class BNO055PowerMode(IntEnum):
    """BNO055 power modes."""

    NORMAL = 0x00
    LOW_POWER = 0x01
    SUSPEND = 0x02


class BNO055OperationMode(IntEnum):
    """BNO055 operation modes."""

    # Configuration mode
    CONFIG = 0x00

    # Non-fusion modes
    ACCONLY = 0x01  # Accelerometer only
    MAGONLY = 0x02  # Magnetometer only
    GYROONLY = 0x03  # Gyroscope only
    ACCMAG = 0x04  # Accelerometer + Magnetometer
    ACCGYRO = 0x05  # Accelerometer + Gyroscope
    MAGGYRO = 0x06  # Magnetometer + Gyroscope
    AMG = 0x07  # All sensors, no fusion

    # Fusion modes
    IMU = 0x08  # Accelerometer + Gyroscope fusion
    COMPASS = 0x09  # Accelerometer + Magnetometer fusion
    M4G = 0x0A  # Accelerometer + Magnetometer (gravity-based)
    NDOF_FMC_OFF = 0x0B  # Full fusion without fast magnetometer calibration
    NDOF = 0x0C  # Full fusion with all sensors


class BNO055AxisRemap(IntEnum):
    """Axis remapping configuration."""

    X = 0x00
    Y = 0x01
    Z = 0x02


class BNO055AxisSign(IntEnum):
    """Axis sign configuration."""

    POSITIVE = 0x00
    NEGATIVE = 0x01


class BNO055SystemStatus(IntEnum):
    """System status codes."""

    IDLE = 0x00
    SYSTEM_ERROR = 0x01
    INITIALIZING_PERIPHERALS = 0x02
    SYSTEM_INITIALIZATION = 0x03
    EXECUTING_SELF_TEST = 0x04
    FUSION_RUNNING = 0x05
    NO_FUSION = 0x06


class BNO055Error(IntEnum):
    """System error codes."""

    NO_ERROR = 0x00
    PERIPHERAL_INIT = 0x01
    SYSTEM_INIT = 0x02
    SELF_TEST_FAIL = 0x03
    REG_MAP_VAL_RANGE = 0x04
    REG_MAP_ADDR_RANGE = 0x05
    REG_MAP_WRITE_ERR = 0x06
    LOW_POWER_NOT_AVAILABLE = 0x07
    ACCEL_POWER_NOT_AVAILABLE = 0x08
    FUSION_CONFIG_ERR = 0x09
    SENSOR_CONFIG_ERR = 0x0A


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class BNO055CalibrationStatus:
    """Calibration status for each component.

    Each value ranges from 0 (uncalibrated) to 3 (fully calibrated).
    """

    system: int
    gyroscope: int
    accelerometer: int
    magnetometer: int

    @property
    def is_calibrated(self) -> bool:
        """Check if all components are fully calibrated."""
        return all(
            val >= 3 for val in [self.system, self.gyroscope, self.accelerometer, self.magnetometer]
        )

    @property
    def is_partially_calibrated(self) -> bool:
        """Check if any component is calibrated."""
        return any(
            val >= 2 for val in [self.system, self.gyroscope, self.accelerometer, self.magnetometer]
        )


@dataclass
class BNO055CalibrationData:
    """Calibration offsets and radii for saving/restoring calibration."""

    accel_offset: Vector3
    mag_offset: Vector3
    gyro_offset: Vector3
    accel_radius: int
    mag_radius: int


@dataclass
class BNO055SystemInfo:
    """System identification and version information."""

    chip_id: int
    accel_id: int
    mag_id: int
    gyro_id: int
    sw_revision: int
    bootloader_revision: int


@dataclass
class BNO055Reading:
    """Complete reading from BNO055."""

    # Orientation
    euler: Vector3  # heading, roll, pitch in degrees
    quaternion: Quaternion

    # Accelerometer data
    acceleration: Vector3  # Raw accelerometer (m/s²)
    linear_acceleration: Vector3  # Acceleration minus gravity (m/s²)
    gravity: Vector3  # Gravity vector (m/s²)

    # Other sensors
    gyroscope: Vector3  # Angular velocity (°/s or rad/s)
    magnetometer: Vector3  # Magnetic field (µT)

    # Calibration
    calibration: BNO055CalibrationStatus

    # Temperature
    temperature: float  # °C

    # Timestamp
    timestamp: float


# =============================================================================
# Configuration
# =============================================================================


class BNO055Config(BaseModel):
    """Configuration for BNO055 driver."""

    model_config = {"frozen": False, "extra": "allow"}

    # I2C settings
    address: int = Field(default=BNO055_DEFAULT_ADDRESS, description="I2C address")

    # Operation mode
    mode: BNO055OperationMode = Field(
        default=BNO055OperationMode.NDOF,
        description="Operation mode",
    )

    # Power mode
    power_mode: BNO055PowerMode = Field(
        default=BNO055PowerMode.NORMAL,
        description="Power mode",
    )

    # Units
    use_degrees: bool = Field(default=True, description="Use degrees for angles")
    use_celsius: bool = Field(default=True, description="Use Celsius for temperature")
    use_mps2: bool = Field(default=True, description="Use m/s² for acceleration")

    # Axis remapping (default is P1 configuration)
    axis_map_x: BNO055AxisRemap = Field(default=BNO055AxisRemap.X, description="X-axis mapping")
    axis_map_y: BNO055AxisRemap = Field(default=BNO055AxisRemap.Y, description="Y-axis mapping")
    axis_map_z: BNO055AxisRemap = Field(default=BNO055AxisRemap.Z, description="Z-axis mapping")
    sign_x: BNO055AxisSign = Field(default=BNO055AxisSign.POSITIVE, description="X-axis sign")
    sign_y: BNO055AxisSign = Field(default=BNO055AxisSign.POSITIVE, description="Y-axis sign")
    sign_z: BNO055AxisSign = Field(default=BNO055AxisSign.POSITIVE, description="Z-axis sign")

    # Timing
    reset_timeout: float = Field(default=1.0, description="Reset timeout in seconds")
    mode_switch_timeout: float = Field(default=0.1, description="Mode switch timeout")

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# BNO055 Driver
# =============================================================================


class BNO055Driver(Driver):
    """Driver for BNO055 9-DOF Absolute Orientation Sensor.

    Provides access to:
    - Absolute orientation (quaternion or Euler angles)
    - Linear acceleration (gravity removed)
    - Angular velocity
    - Magnetic field
    - Gravity vector

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> imu = BNO055Driver(bus=bus)
        >>> imu.begin()
        >>> reading = imu.read_all()
        >>> print(f"Heading: {reading.euler.x:.1f}°")
    """

    # Channel mapping for Driver interface
    CHANNEL_ACCEL_X = 0
    CHANNEL_ACCEL_Y = 1
    CHANNEL_ACCEL_Z = 2
    CHANNEL_GYRO_X = 3
    CHANNEL_GYRO_Y = 4
    CHANNEL_GYRO_Z = 5
    CHANNEL_MAG_X = 6
    CHANNEL_MAG_Y = 7
    CHANNEL_MAG_Z = 8
    CHANNEL_EULER_HEADING = 9
    CHANNEL_EULER_ROLL = 10
    CHANNEL_EULER_PITCH = 11
    CHANNEL_QUAT_W = 12
    CHANNEL_QUAT_X = 13
    CHANNEL_QUAT_Y = 14
    CHANNEL_QUAT_Z = 15
    CHANNEL_TEMP = 16

    def __init__(
        self,
        bus: I2CBus,
        config: BNO055Config | None = None,
    ) -> None:
        """Initialize BNO055 driver.

        Args:
            bus: I2C bus for communication.
            config: Configuration options.
        """
        self._bus = bus
        self._config = config or BNO055Config()
        self._mode = BNO055OperationMode.CONFIG
        self._info: BNO055SystemInfo | None = None
        self._connected = False
        self._last_reading: BNO055Reading | None = None

        # Channel cache
        self._channel_values: dict[int, float] = {}

    @property
    def config(self) -> BNO055Config:
        """Get driver configuration."""
        return self._config

    @property
    def mode(self) -> BNO055OperationMode:
        """Get current operation mode."""
        return self._mode

    @property
    def info(self) -> BNO055SystemInfo | None:
        """Get system information."""
        return self._info

    @property
    def is_connected(self) -> bool:
        """Check if driver is connected."""
        return self._connected

    # -------------------------------------------------------------------------
    # Driver Interface
    # -------------------------------------------------------------------------

    def set_channel(self, channel: int, value: float) -> None:
        """Set channel value (not applicable for read-only sensor)."""
        raise NotImplementedError("BNO055 is a read-only sensor")

    def get_channel(self, channel: int) -> float:
        """Get channel value.

        Args:
            channel: Channel number (see CHANNEL_* constants).

        Returns:
            Channel value.
        """
        if not self._connected:
            return 0.0

        # Update all readings
        self._update_readings()

        return self._channel_values.get(channel, 0.0)

    def _update_readings(self) -> None:
        """Update all channel values from sensor."""
        try:
            reading = self.read_all()

            self._channel_values[self.CHANNEL_ACCEL_X] = reading.acceleration.x
            self._channel_values[self.CHANNEL_ACCEL_Y] = reading.acceleration.y
            self._channel_values[self.CHANNEL_ACCEL_Z] = reading.acceleration.z
            self._channel_values[self.CHANNEL_GYRO_X] = reading.gyroscope.x
            self._channel_values[self.CHANNEL_GYRO_Y] = reading.gyroscope.y
            self._channel_values[self.CHANNEL_GYRO_Z] = reading.gyroscope.z
            self._channel_values[self.CHANNEL_MAG_X] = reading.magnetometer.x
            self._channel_values[self.CHANNEL_MAG_Y] = reading.magnetometer.y
            self._channel_values[self.CHANNEL_MAG_Z] = reading.magnetometer.z
            self._channel_values[self.CHANNEL_EULER_HEADING] = reading.euler.x
            self._channel_values[self.CHANNEL_EULER_ROLL] = reading.euler.y
            self._channel_values[self.CHANNEL_EULER_PITCH] = reading.euler.z
            self._channel_values[self.CHANNEL_QUAT_W] = reading.quaternion.w
            self._channel_values[self.CHANNEL_QUAT_X] = reading.quaternion.x
            self._channel_values[self.CHANNEL_QUAT_Y] = reading.quaternion.y
            self._channel_values[self.CHANNEL_QUAT_Z] = reading.quaternion.z
            self._channel_values[self.CHANNEL_TEMP] = reading.temperature

        except Exception as e:
            logger.warning("Failed to update BNO055 readings: %s", e)

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def begin(self) -> bool:
        """Initialize the BNO055 sensor.

        Returns:
            True if initialization successful.

        Raises:
            CommunicationError: If sensor not found.
            ConfigurationError: If configuration fails.
        """
        # Verify chip ID
        chip_id = self._read_byte(BNO055Register.CHIP_ID)
        if chip_id != BNO055_CHIP_ID:
            raise CommunicationError(
                f"BNO055 not found at address 0x{self._config.address:02X}. "
                f"Expected chip ID 0x{BNO055_CHIP_ID:02X}, got 0x{chip_id:02X}"
            )

        # Read system info
        self._info = BNO055SystemInfo(
            chip_id=chip_id,
            accel_id=self._read_byte(BNO055Register.ACC_ID),
            mag_id=self._read_byte(BNO055Register.MAG_ID),
            gyro_id=self._read_byte(BNO055Register.GYR_ID),
            sw_revision=(
                self._read_byte(BNO055Register.SW_REV_ID_MSB) << 8
                | self._read_byte(BNO055Register.SW_REV_ID_LSB)
            ),
            bootloader_revision=self._read_byte(BNO055Register.BL_REV_ID),
        )

        # Set to config mode
        self._set_mode(BNO055OperationMode.CONFIG)

        # Reset
        self._write_byte(BNO055Register.SYS_TRIGGER, 0x20)
        time.sleep(self._config.reset_timeout)

        # Wait for chip to come back
        timeout = time.time() + 2.0
        while time.time() < timeout:
            try:
                if self._read_byte(BNO055Register.CHIP_ID) == BNO055_CHIP_ID:
                    break
            except Exception:
                pass
            time.sleep(0.01)
        else:
            raise CommunicationError("BNO055 reset timeout")

        # Set power mode
        self._write_byte(BNO055Register.PWR_MODE, self._config.power_mode)

        # Configure units
        unit_sel = 0
        if not self._config.use_degrees:
            unit_sel |= 0x02  # Radians for gyro
            unit_sel |= 0x04  # Radians for euler
        if not self._config.use_celsius:
            unit_sel |= 0x10  # Fahrenheit
        if not self._config.use_mps2:
            unit_sel |= 0x01  # mg for acceleration
        self._write_byte(BNO055Register.UNIT_SEL, unit_sel)

        # Configure axis remapping
        self._configure_axis_remap()

        # Set operation mode
        self._set_mode(self._config.mode)

        self._connected = True
        logger.info(
            "BNO055 initialized (SW rev: %d, mode: %s)",
            self._info.sw_revision,
            self._config.mode.name,
        )

        return True

    def reset(self) -> None:
        """Perform a soft reset."""
        self._write_byte(BNO055Register.SYS_TRIGGER, 0x20)
        time.sleep(self._config.reset_timeout)
        self.begin()

    # -------------------------------------------------------------------------
    # Mode Control
    # -------------------------------------------------------------------------

    def _set_mode(self, mode: BNO055OperationMode) -> None:
        """Set operation mode.

        Args:
            mode: Target operation mode.
        """
        self._write_byte(BNO055Register.OPR_MODE, mode)
        time.sleep(self._config.mode_switch_timeout)
        self._mode = mode

    def set_mode(self, mode: BNO055OperationMode) -> None:
        """Set operation mode.

        Args:
            mode: Target operation mode.
        """
        if mode == self._mode:
            return

        # Must go through CONFIG mode to change fusion modes
        if self._mode >= BNO055OperationMode.IMU or mode >= BNO055OperationMode.IMU:
            self._set_mode(BNO055OperationMode.CONFIG)

        self._set_mode(mode)

    def enter_config_mode(self) -> None:
        """Enter configuration mode."""
        self._set_mode(BNO055OperationMode.CONFIG)

    def enter_fusion_mode(self) -> None:
        """Enter full fusion (NDOF) mode."""
        self._set_mode(BNO055OperationMode.NDOF)

    # -------------------------------------------------------------------------
    # Reading Data
    # -------------------------------------------------------------------------

    def read_all(self) -> BNO055Reading:
        """Read all sensor data.

        Returns:
            Complete BNO055Reading with all sensor values.
        """
        reading = BNO055Reading(
            euler=self.read_euler(),
            quaternion=self.read_quaternion(),
            acceleration=self.read_acceleration(),
            linear_acceleration=self.read_linear_acceleration(),
            gravity=self.read_gravity(),
            gyroscope=self.read_gyroscope(),
            magnetometer=self.read_magnetometer(),
            calibration=self.get_calibration_status(),
            temperature=self.read_temperature(),
            timestamp=time.time(),
        )
        self._last_reading = reading
        return reading

    def read_euler(self) -> Vector3:
        """Read Euler angles (heading, roll, pitch).

        Returns:
            Vector3 with angles in degrees (or radians if configured).
        """
        data = self._read_bytes(BNO055Register.EUL_HEADING_LSB, 6)
        heading = self._unpack_int16(data[0], data[1]) / 16.0
        roll = self._unpack_int16(data[2], data[3]) / 16.0
        pitch = self._unpack_int16(data[4], data[5]) / 16.0
        return Vector3(x=heading, y=roll, z=pitch)

    def read_quaternion(self) -> Quaternion:
        """Read orientation quaternion.

        Returns:
            Quaternion representing orientation.
        """
        data = self._read_bytes(BNO055Register.QUA_DATA_W_LSB, 8)
        scale = 1.0 / (1 << 14)  # 2^14 = 16384
        w = self._unpack_int16(data[0], data[1]) * scale
        x = self._unpack_int16(data[2], data[3]) * scale
        y = self._unpack_int16(data[4], data[5]) * scale
        z = self._unpack_int16(data[6], data[7]) * scale
        return Quaternion(w=w, x=x, y=y, z=z)

    def read_acceleration(self) -> Vector3:
        """Read raw accelerometer data.

        Returns:
            Vector3 with acceleration in m/s² (or mg if configured).
        """
        data = self._read_bytes(BNO055Register.ACC_DATA_X_LSB, 6)
        scale = 0.01 if self._config.use_mps2 else 1.0  # 1 LSB = 0.01 m/s² or 1 mg
        x = self._unpack_int16(data[0], data[1]) * scale
        y = self._unpack_int16(data[2], data[3]) * scale
        z = self._unpack_int16(data[4], data[5]) * scale
        return Vector3(x=x, y=y, z=z)

    def read_linear_acceleration(self) -> Vector3:
        """Read linear acceleration (gravity removed).

        Returns:
            Vector3 with linear acceleration in m/s².
        """
        data = self._read_bytes(BNO055Register.LIA_DATA_X_LSB, 6)
        scale = 0.01 if self._config.use_mps2 else 1.0
        x = self._unpack_int16(data[0], data[1]) * scale
        y = self._unpack_int16(data[2], data[3]) * scale
        z = self._unpack_int16(data[4], data[5]) * scale
        return Vector3(x=x, y=y, z=z)

    def read_gravity(self) -> Vector3:
        """Read gravity vector.

        Returns:
            Vector3 with gravity in m/s².
        """
        data = self._read_bytes(BNO055Register.GRV_DATA_X_LSB, 6)
        scale = 0.01 if self._config.use_mps2 else 1.0
        x = self._unpack_int16(data[0], data[1]) * scale
        y = self._unpack_int16(data[2], data[3]) * scale
        z = self._unpack_int16(data[4], data[5]) * scale
        return Vector3(x=x, y=y, z=z)

    def read_gyroscope(self) -> Vector3:
        """Read angular velocity.

        Returns:
            Vector3 with angular velocity in °/s (or rad/s if configured).
        """
        data = self._read_bytes(BNO055Register.GYR_DATA_X_LSB, 6)
        scale = 1.0 / 16.0 if self._config.use_degrees else 1.0 / 900.0
        x = self._unpack_int16(data[0], data[1]) * scale
        y = self._unpack_int16(data[2], data[3]) * scale
        z = self._unpack_int16(data[4], data[5]) * scale
        return Vector3(x=x, y=y, z=z)

    def read_magnetometer(self) -> Vector3:
        """Read magnetic field.

        Returns:
            Vector3 with magnetic field in µT.
        """
        data = self._read_bytes(BNO055Register.MAG_DATA_X_LSB, 6)
        scale = 1.0 / 16.0  # 1 LSB = 1/16 µT
        x = self._unpack_int16(data[0], data[1]) * scale
        y = self._unpack_int16(data[2], data[3]) * scale
        z = self._unpack_int16(data[4], data[5]) * scale
        return Vector3(x=x, y=y, z=z)

    def read_temperature(self) -> float:
        """Read temperature.

        Returns:
            Temperature in °C (or °F if configured).
        """
        temp = self._read_byte(BNO055Register.TEMP)
        if temp > 127:
            temp -= 256
        return float(temp)

    # -------------------------------------------------------------------------
    # Calibration
    # -------------------------------------------------------------------------

    def get_calibration_status(self) -> BNO055CalibrationStatus:
        """Get calibration status for all components.

        Returns:
            BNO055CalibrationStatus with 0-3 values for each component.
        """
        status = self._read_byte(BNO055Register.CALIB_STAT)
        return BNO055CalibrationStatus(
            system=(status >> 6) & 0x03,
            gyroscope=(status >> 4) & 0x03,
            accelerometer=(status >> 2) & 0x03,
            magnetometer=status & 0x03,
        )

    def is_calibrated(self) -> bool:
        """Check if sensor is fully calibrated.

        Returns:
            True if all components have calibration level 3.
        """
        return self.get_calibration_status().is_calibrated

    def wait_for_calibration(self, timeout: float = 60.0) -> bool:
        """Wait for sensor to be fully calibrated.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if calibration achieved, False if timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_calibrated():
                return True
            time.sleep(0.1)
        return False

    def get_calibration_data(self) -> BNO055CalibrationData:
        """Get calibration offsets for saving.

        Returns:
            BNO055CalibrationData with all offsets.
        """
        # Must be in config mode to read offsets
        current_mode = self._mode
        self._set_mode(BNO055OperationMode.CONFIG)

        # Read accelerometer offset
        data = self._read_bytes(BNO055Register.ACC_OFFSET_X_LSB, 6)
        accel_offset = Vector3(
            x=self._unpack_int16(data[0], data[1]),
            y=self._unpack_int16(data[2], data[3]),
            z=self._unpack_int16(data[4], data[5]),
        )

        # Read magnetometer offset
        data = self._read_bytes(BNO055Register.MAG_OFFSET_X_LSB, 6)
        mag_offset = Vector3(
            x=self._unpack_int16(data[0], data[1]),
            y=self._unpack_int16(data[2], data[3]),
            z=self._unpack_int16(data[4], data[5]),
        )

        # Read gyroscope offset
        data = self._read_bytes(BNO055Register.GYR_OFFSET_X_LSB, 6)
        gyro_offset = Vector3(
            x=self._unpack_int16(data[0], data[1]),
            y=self._unpack_int16(data[2], data[3]),
            z=self._unpack_int16(data[4], data[5]),
        )

        # Read radii
        data = self._read_bytes(BNO055Register.ACC_RADIUS_LSB, 4)
        accel_radius = self._unpack_int16(data[0], data[1])
        mag_radius = self._unpack_int16(data[2], data[3])

        # Restore mode
        self._set_mode(current_mode)

        return BNO055CalibrationData(
            accel_offset=accel_offset,
            mag_offset=mag_offset,
            gyro_offset=gyro_offset,
            accel_radius=accel_radius,
            mag_radius=mag_radius,
        )

    def set_calibration_data(self, calibration: BNO055CalibrationData) -> None:
        """Restore saved calibration offsets.

        Args:
            calibration: Calibration data to restore.
        """
        # Must be in config mode
        current_mode = self._mode
        self._set_mode(BNO055OperationMode.CONFIG)

        # Write accelerometer offset
        self._write_bytes(
            BNO055Register.ACC_OFFSET_X_LSB,
            self._pack_int16s(
                int(calibration.accel_offset.x),
                int(calibration.accel_offset.y),
                int(calibration.accel_offset.z),
            ),
        )

        # Write magnetometer offset
        self._write_bytes(
            BNO055Register.MAG_OFFSET_X_LSB,
            self._pack_int16s(
                int(calibration.mag_offset.x),
                int(calibration.mag_offset.y),
                int(calibration.mag_offset.z),
            ),
        )

        # Write gyroscope offset
        self._write_bytes(
            BNO055Register.GYR_OFFSET_X_LSB,
            self._pack_int16s(
                int(calibration.gyro_offset.x),
                int(calibration.gyro_offset.y),
                int(calibration.gyro_offset.z),
            ),
        )

        # Write radii
        self._write_bytes(
            BNO055Register.ACC_RADIUS_LSB,
            self._pack_int16s(calibration.accel_radius, calibration.mag_radius),
        )

        # Restore mode
        self._set_mode(current_mode)

        logger.info("Calibration data restored")

    # -------------------------------------------------------------------------
    # System Status
    # -------------------------------------------------------------------------

    def get_system_status(self) -> tuple[BNO055SystemStatus, BNO055Error]:
        """Get system status and error code.

        Returns:
            Tuple of (status, error).
        """
        status = BNO055SystemStatus(self._read_byte(BNO055Register.SYS_STATUS))
        error = BNO055Error(self._read_byte(BNO055Register.SYS_ERR))
        return status, error

    def run_self_test(self) -> dict[str, bool]:
        """Run self-test and return results.

        Returns:
            Dict with test results for each component.
        """
        # Must be in config mode
        current_mode = self._mode
        self._set_mode(BNO055OperationMode.CONFIG)

        # Trigger self-test
        self._write_byte(BNO055Register.SYS_TRIGGER, 0x01)
        time.sleep(0.5)

        # Read results
        result = self._read_byte(BNO055Register.ST_RESULT)

        # Restore mode
        self._set_mode(current_mode)

        return {
            "accelerometer": bool(result & 0x01),
            "magnetometer": bool(result & 0x02),
            "gyroscope": bool(result & 0x04),
            "mcu": bool(result & 0x08),
        }

    # -------------------------------------------------------------------------
    # Axis Remapping
    # -------------------------------------------------------------------------

    def _configure_axis_remap(self) -> None:
        """Configure axis remapping."""
        config = (
            (self._config.axis_map_z << 4)
            | (self._config.axis_map_y << 2)
            | self._config.axis_map_x
        )
        sign = (self._config.sign_x << 2) | (self._config.sign_y << 1) | self._config.sign_z
        self._write_byte(BNO055Register.AXIS_MAP_CONFIG, config)
        self._write_byte(BNO055Register.AXIS_MAP_SIGN, sign)

    # -------------------------------------------------------------------------
    # Low-Level I2C
    # -------------------------------------------------------------------------

    def _read_byte(self, register: int) -> int:
        """Read a single byte from register."""
        data = self._bus.read_register(self._config.address, register, 1)
        return data[0]

    def _read_bytes(self, register: int, length: int) -> bytes:
        """Read multiple bytes from register."""
        return self._bus.read_register(self._config.address, register, length)

    def _write_byte(self, register: int, value: int) -> None:
        """Write a single byte to register."""
        self._bus.write_register(self._config.address, register, bytes([value]))

    def _write_bytes(self, register: int, data: bytes) -> None:
        """Write multiple bytes to register."""
        self._bus.write_register(self._config.address, register, data)

    @staticmethod
    def _unpack_int16(lsb: int, msb: int) -> int:
        """Unpack a signed 16-bit integer from LSB and MSB bytes."""
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    @staticmethod
    def _pack_int16s(*values: int) -> bytes:
        """Pack signed 16-bit integers into bytes (LSB first)."""
        result = []
        for value in values:
            if value < 0:
                value += 65536
            result.extend([value & 0xFF, (value >> 8) & 0xFF])
        return bytes(result)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "BNO055_ALT_ADDRESS",
    "BNO055_CHIP_ID",
    # Constants
    "BNO055_DEFAULT_ADDRESS",
    "BNO055AxisRemap",
    "BNO055AxisSign",
    "BNO055CalibrationData",
    # Data classes
    "BNO055CalibrationStatus",
    # Config
    "BNO055Config",
    # Driver
    "BNO055Driver",
    "BNO055Error",
    "BNO055OperationMode",
    "BNO055PowerMode",
    "BNO055Reading",
    # Enums
    "BNO055Register",
    "BNO055SystemInfo",
    "BNO055SystemStatus",
]
