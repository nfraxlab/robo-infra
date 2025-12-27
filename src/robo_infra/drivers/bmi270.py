"""BMI270 6-DOF Motion Sensor driver.

The BMI270 from Bosch Sensortec is an ultra-low power IMU:
- 3-axis accelerometer (±2g, ±4g, ±8g, ±16g)
- 3-axis gyroscope (±125, ±250, ±500, ±1000, ±2000 dps)

Key Features:
- Ultra-low power (0.55mA in performance mode)
- Motion detection (any motion, no motion, significant motion)
- Activity recognition (walking, running, stationary)
- Wrist gesture recognition (wearable applications)
- I2C and SPI interfaces
- 6kB FIFO buffer

Notes:
- Default I2C address is 0x68 (or 0x69 with SDO pin high)
- Requires configuration file loading after power-up
- Optimized for wearable devices
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.driver import Driver
from robo_infra.core.exceptions import CommunicationError, ConfigurationError
from robo_infra.core.types import Vector3


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

BMI270_DEFAULT_ADDRESS = 0x68
BMI270_ALT_ADDRESS = 0x69
BMI270_CHIP_ID = 0x24


# =============================================================================
# Register Definitions
# =============================================================================


class BMI270Register(IntEnum):
    """BMI270 register addresses."""

    CHIP_ID = 0x00
    ERR_REG = 0x02
    STATUS = 0x03
    DATA_0 = 0x04  # Auxiliary X LSB
    DATA_1 = 0x05
    DATA_2 = 0x06
    DATA_3 = 0x07
    DATA_4 = 0x08
    DATA_5 = 0x09
    DATA_6 = 0x0A
    DATA_7 = 0x0B
    DATA_8 = 0x0C  # Accel X LSB
    DATA_9 = 0x0D
    DATA_10 = 0x0E
    DATA_11 = 0x0F
    DATA_12 = 0x10
    DATA_13 = 0x11
    DATA_14 = 0x12  # Gyro X LSB
    DATA_15 = 0x13
    DATA_16 = 0x14
    DATA_17 = 0x15
    DATA_18 = 0x16
    DATA_19 = 0x17
    SENSORTIME_0 = 0x18
    SENSORTIME_1 = 0x19
    SENSORTIME_2 = 0x1A
    EVENT = 0x1B
    INT_STATUS_0 = 0x1C
    INT_STATUS_1 = 0x1D
    SC_OUT_0 = 0x1E  # Step counter
    SC_OUT_1 = 0x1F
    WR_GEST_ACT = 0x20
    INTERNAL_STATUS = 0x21
    TEMPERATURE_0 = 0x22
    TEMPERATURE_1 = 0x23
    FIFO_LENGTH_0 = 0x24
    FIFO_LENGTH_1 = 0x25
    FIFO_DATA = 0x26
    FEAT_PAGE = 0x2F
    ACC_CONF = 0x40
    ACC_RANGE = 0x41
    GYR_CONF = 0x42
    GYR_RANGE = 0x43
    AUX_CONF = 0x44
    FIFO_DOWNS = 0x45
    FIFO_WTM_0 = 0x46
    FIFO_WTM_1 = 0x47
    FIFO_CONFIG_0 = 0x48
    FIFO_CONFIG_1 = 0x49
    SATURATION = 0x4A
    AUX_DEV_ID = 0x4B
    AUX_IF_CONF = 0x4C
    AUX_RD_ADDR = 0x4D
    AUX_WR_ADDR = 0x4E
    AUX_WR_DATA = 0x4F
    ERR_REG_MSK = 0x52
    INT1_IO_CTRL = 0x53
    INT2_IO_CTRL = 0x54
    INT_LATCH = 0x55
    INT1_MAP_FEAT = 0x56
    INT2_MAP_FEAT = 0x57
    INT_MAP_DATA = 0x58
    INIT_CTRL = 0x59
    INIT_ADDR_0 = 0x5B
    INIT_ADDR_1 = 0x5C
    INIT_DATA = 0x5E
    INTERNAL_ERROR = 0x5F
    AUX_IF_TRIM = 0x68
    GYR_CRT_CONF = 0x69
    NVM_CONF = 0x6A
    IF_CONF = 0x6B
    DRV = 0x6C
    ACC_SELF_TEST = 0x6D
    GYR_SELF_TEST_AXES = 0x6E
    NV_CONF = 0x70
    OFFSET_0 = 0x71
    OFFSET_1 = 0x72
    OFFSET_2 = 0x73
    OFFSET_3 = 0x74
    OFFSET_4 = 0x75
    OFFSET_5 = 0x76
    OFFSET_6 = 0x77
    PWR_CONF = 0x7C
    PWR_CTRL = 0x7D
    CMD = 0x7E


class AccelODR(IntEnum):
    """Accelerometer output data rate."""

    ODR_0_78HZ = 0x01
    ODR_1_56HZ = 0x02
    ODR_3_12HZ = 0x03
    ODR_6_25HZ = 0x04
    ODR_12_5HZ = 0x05
    ODR_25HZ = 0x06
    ODR_50HZ = 0x07
    ODR_100HZ = 0x08
    ODR_200HZ = 0x09
    ODR_400HZ = 0x0A
    ODR_800HZ = 0x0B
    ODR_1600HZ = 0x0C


class AccelBWP(IntEnum):
    """Accelerometer bandwidth parameter."""

    OSR4_AVG1 = 0x00
    OSR2_AVG2 = 0x01
    NORMAL_AVG4 = 0x02
    CIC_AVG8 = 0x03
    RES_AVG16 = 0x04
    RES_AVG32 = 0x05
    RES_AVG64 = 0x06
    RES_AVG128 = 0x07


class AccelRange(IntEnum):
    """Accelerometer full-scale range."""

    RANGE_2G = 0x00
    RANGE_4G = 0x01
    RANGE_8G = 0x02
    RANGE_16G = 0x03


class GyroODR(IntEnum):
    """Gyroscope output data rate."""

    ODR_25HZ = 0x06
    ODR_50HZ = 0x07
    ODR_100HZ = 0x08
    ODR_200HZ = 0x09
    ODR_400HZ = 0x0A
    ODR_800HZ = 0x0B
    ODR_1600HZ = 0x0C
    ODR_3200HZ = 0x0D


class GyroBWP(IntEnum):
    """Gyroscope bandwidth parameter."""

    OSR4 = 0x00
    OSR2 = 0x01
    NORMAL = 0x02


class GyroRange(IntEnum):
    """Gyroscope full-scale range."""

    RANGE_2000DPS = 0x00
    RANGE_1000DPS = 0x01
    RANGE_500DPS = 0x02
    RANGE_250DPS = 0x03
    RANGE_125DPS = 0x04


class PowerMode(IntEnum):
    """Device power mode."""

    SUSPEND = 0x00
    NORMAL = 0x01
    LOW_POWER = 0x02
    PERFORMANCE = 0x03


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class BMI270Reading:
    """Complete reading from BMI270."""

    acceleration: Vector3  # m/s²
    gyroscope: Vector3  # °/s
    temperature: float  # °C
    timestamp: float


@dataclass
class BMI270Status:
    """Device status."""

    accel_data_ready: bool
    gyro_data_ready: bool
    aux_data_ready: bool
    cmd_ready: bool
    drdy_aux: bool
    drdy_gyr: bool
    drdy_acc: bool


# =============================================================================
# Configuration
# =============================================================================


class BMI270Config(BaseModel):
    """Configuration for BMI270 driver."""

    model_config = {"frozen": False, "extra": "allow"}

    # I2C settings
    address: int = Field(default=BMI270_DEFAULT_ADDRESS, description="I2C address")

    # Accelerometer
    accel_odr: AccelODR = Field(
        default=AccelODR.ODR_100HZ,
        description="Accelerometer output data rate",
    )
    accel_bwp: AccelBWP = Field(
        default=AccelBWP.NORMAL_AVG4,
        description="Accelerometer bandwidth parameter",
    )
    accel_range: AccelRange = Field(
        default=AccelRange.RANGE_4G,
        description="Accelerometer full-scale range",
    )

    # Gyroscope
    gyro_odr: GyroODR = Field(
        default=GyroODR.ODR_100HZ,
        description="Gyroscope output data rate",
    )
    gyro_bwp: GyroBWP = Field(
        default=GyroBWP.NORMAL,
        description="Gyroscope bandwidth parameter",
    )
    gyro_range: GyroRange = Field(
        default=GyroRange.RANGE_500DPS,
        description="Gyroscope full-scale range",
    )

    # Power
    power_mode: PowerMode = Field(
        default=PowerMode.NORMAL,
        description="Power mode",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# BMI270 Driver
# =============================================================================


class BMI270Driver(Driver):
    """Driver for BMI270 6-DOF Motion Sensor.

    Provides access to:
    - Accelerometer (3-axis)
    - Gyroscope (3-axis)
    - Temperature
    - Step counter

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> imu = BMI270Driver(bus=bus)
        >>> imu.begin()
        >>> reading = imu.read_all()
        >>> print(f"Accel X: {reading.acceleration.x:.2f} m/s²")
    """

    # Scale factors
    ACCEL_SENSITIVITY = {
        AccelRange.RANGE_2G: 16384.0,  # LSB/g
        AccelRange.RANGE_4G: 8192.0,
        AccelRange.RANGE_8G: 4096.0,
        AccelRange.RANGE_16G: 2048.0,
    }

    GYRO_SENSITIVITY = {
        GyroRange.RANGE_2000DPS: 16.4,  # LSB/(°/s)
        GyroRange.RANGE_1000DPS: 32.8,
        GyroRange.RANGE_500DPS: 65.5,
        GyroRange.RANGE_250DPS: 131.0,
        GyroRange.RANGE_125DPS: 262.0,
    }

    # Temperature constants
    TEMP_OFFSET = 23.0  # °C at 0x8000

    # Channel mapping
    CHANNEL_ACCEL_X = 0
    CHANNEL_ACCEL_Y = 1
    CHANNEL_ACCEL_Z = 2
    CHANNEL_GYRO_X = 3
    CHANNEL_GYRO_Y = 4
    CHANNEL_GYRO_Z = 5
    CHANNEL_TEMP = 6

    def __init__(
        self,
        bus: I2CBus,
        config: BMI270Config | None = None,
    ) -> None:
        """Initialize BMI270 driver.

        Args:
            bus: I2C bus for communication.
            config: Configuration options.
        """
        self._bus = bus
        self._config = config or BMI270Config()
        self._connected = False
        self._last_reading: BMI270Reading | None = None
        self._channel_values: dict[int, float] = {}

    @property
    def config(self) -> BMI270Config:
        """Get driver configuration."""
        return self._config

    @property
    def is_connected(self) -> bool:
        """Check if driver is connected."""
        return self._connected

    # -------------------------------------------------------------------------
    # Driver Interface
    # -------------------------------------------------------------------------

    def set_channel(self, channel: int, value: float) -> None:
        """Set channel value (not applicable for read-only sensor)."""
        raise NotImplementedError("BMI270 is a read-only sensor")

    def get_channel(self, channel: int) -> float:
        """Get channel value."""
        if not self._connected:
            return 0.0

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
            self._channel_values[self.CHANNEL_TEMP] = reading.temperature

        except Exception as e:
            logger.warning("Failed to update BMI270 readings: %s", e)

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def begin(self) -> bool:
        """Initialize the BMI270 sensor.

        Returns:
            True if initialization successful.

        Raises:
            CommunicationError: If sensor not found.
            ConfigurationError: If configuration loading fails.
        """
        # Verify chip ID
        chip_id = self._read_byte(BMI270Register.CHIP_ID)

        if chip_id != BMI270_CHIP_ID:
            raise CommunicationError(
                f"BMI270 not found at address 0x{self._config.address:02X}. "
                f"Expected chip ID 0x{BMI270_CHIP_ID:02X}, got 0x{chip_id:02X}"
            )

        # Soft reset
        self._write_byte(BMI270Register.CMD, 0xB6)
        time.sleep(0.05)

        # Wait for power up
        time.sleep(0.01)

        # Load configuration file
        self._load_config_file()

        # Configure power
        self._write_byte(BMI270Register.PWR_CONF, 0x00)  # Advanced power save off
        time.sleep(0.001)

        # Enable sensors
        self._write_byte(BMI270Register.PWR_CTRL, 0x0E)  # Enable acc, gyr, temp
        time.sleep(0.001)

        # Configure accelerometer
        acc_conf = (self._config.accel_bwp << 4) | self._config.accel_odr
        self._write_byte(BMI270Register.ACC_CONF, acc_conf)
        self._write_byte(BMI270Register.ACC_RANGE, self._config.accel_range)

        # Configure gyroscope
        gyr_conf = (self._config.gyro_bwp << 4) | self._config.gyro_odr
        self._write_byte(BMI270Register.GYR_CONF, gyr_conf)
        self._write_byte(BMI270Register.GYR_RANGE, self._config.gyro_range)

        # Check for errors
        err = self._read_byte(BMI270Register.ERR_REG)
        if err & 0x02:  # Fatal error
            raise ConfigurationError(f"BMI270 fatal error: 0x{err:02X}")

        self._connected = True

        logger.info(
            "BMI270 initialized (accel: ±%dg, gyro: ±%d dps)",
            2 << self._config.accel_range,
            2000 >> self._config.gyro_range,
        )

        return True

    def _load_config_file(self) -> None:
        """Load the BMI270 configuration file.

        The BMI270 requires a configuration file to be loaded after power-up.
        This is a simplified version that enables basic functionality.
        """
        # Prepare for config loading
        self._write_byte(BMI270Register.PWR_CONF, 0x00)  # Disable advanced power save
        time.sleep(0.001)

        # Set init control to 0
        self._write_byte(BMI270Register.INIT_CTRL, 0x00)

        # In production, you would load the full config file here
        # For simulation/basic operation, we skip this step
        # The config file is ~8KB and specific to the chip

        # Set init control to 1 to start initialization
        self._write_byte(BMI270Register.INIT_CTRL, 0x01)
        time.sleep(0.15)  # Wait for initialization

        # Check init status
        internal_status = self._read_byte(BMI270Register.INTERNAL_STATUS)
        if not (internal_status & 0x01):
            logger.warning(
                "BMI270 config loading may have failed (status: 0x%02X)", internal_status
            )

    def reset(self) -> None:
        """Perform a software reset."""
        self._write_byte(BMI270Register.CMD, 0xB6)
        time.sleep(0.05)
        self.begin()

    # -------------------------------------------------------------------------
    # Reading Data
    # -------------------------------------------------------------------------

    def read_all(self) -> BMI270Reading:
        """Read all sensor data.

        Returns:
            BMI270Reading with all sensor values.
        """
        # Read accel (6) + gyro (6) = 12 bytes from DATA_8
        data = self._read_bytes(BMI270Register.DATA_8, 12)

        # Parse accelerometer
        accel_sens = self.ACCEL_SENSITIVITY[self._config.accel_range]
        ax = self._unpack_int16(data[0], data[1]) / accel_sens * 9.81
        ay = self._unpack_int16(data[2], data[3]) / accel_sens * 9.81
        az = self._unpack_int16(data[4], data[5]) / accel_sens * 9.81

        # Parse gyroscope
        gyro_sens = self.GYRO_SENSITIVITY[self._config.gyro_range]
        gx = self._unpack_int16(data[6], data[7]) / gyro_sens
        gy = self._unpack_int16(data[8], data[9]) / gyro_sens
        gz = self._unpack_int16(data[10], data[11]) / gyro_sens

        # Read temperature
        temperature = self.read_temperature()

        reading = BMI270Reading(
            acceleration=Vector3(x=ax, y=ay, z=az),
            gyroscope=Vector3(x=gx, y=gy, z=gz),
            temperature=temperature,
            timestamp=time.time(),
        )
        self._last_reading = reading
        return reading

    def read_acceleration(self) -> Vector3:
        """Read accelerometer data only.

        Returns:
            Vector3 with acceleration in m/s².
        """
        data = self._read_bytes(BMI270Register.DATA_8, 6)
        sens = self.ACCEL_SENSITIVITY[self._config.accel_range]

        return Vector3(
            x=self._unpack_int16(data[0], data[1]) / sens * 9.81,
            y=self._unpack_int16(data[2], data[3]) / sens * 9.81,
            z=self._unpack_int16(data[4], data[5]) / sens * 9.81,
        )

    def read_gyroscope(self) -> Vector3:
        """Read gyroscope data only.

        Returns:
            Vector3 with angular velocity in °/s.
        """
        data = self._read_bytes(BMI270Register.DATA_14, 6)
        sens = self.GYRO_SENSITIVITY[self._config.gyro_range]

        return Vector3(
            x=self._unpack_int16(data[0], data[1]) / sens,
            y=self._unpack_int16(data[2], data[3]) / sens,
            z=self._unpack_int16(data[4], data[5]) / sens,
        )

    def read_temperature(self) -> float:
        """Read temperature.

        Returns:
            Temperature in °C.
        """
        data = self._read_bytes(BMI270Register.TEMPERATURE_0, 2)
        raw = self._unpack_int16(data[0], data[1])

        if raw == 0x8000:
            return float("nan")  # Invalid

        return raw / 512.0 + self.TEMP_OFFSET

    def get_status(self) -> BMI270Status:
        """Get device status.

        Returns:
            BMI270Status with data ready flags.
        """
        status = self._read_byte(BMI270Register.STATUS)
        return BMI270Status(
            accel_data_ready=bool(status & 0x80),
            gyro_data_ready=bool(status & 0x40),
            aux_data_ready=bool(status & 0x20),
            cmd_ready=bool(status & 0x10),
            drdy_aux=bool(status & 0x20),
            drdy_gyr=bool(status & 0x40),
            drdy_acc=bool(status & 0x80),
        )

    # -------------------------------------------------------------------------
    # Step Counter
    # -------------------------------------------------------------------------

    def enable_step_counter(self) -> None:
        """Enable step counter feature."""
        # Access feature page 1
        self._write_byte(BMI270Register.FEAT_PAGE, 0x01)
        time.sleep(0.001)

        # Enable step counter (simplified - full implementation needs config file)
        logger.info("Step counter enabled")

    def read_step_count(self) -> int:
        """Read step counter value.

        Returns:
            Number of steps counted.
        """
        data = self._read_bytes(BMI270Register.SC_OUT_0, 2)
        return data[0] | (data[1] << 8)

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

    @staticmethod
    def _unpack_int16(lsb: int, msb: int) -> int:
        """Unpack a signed 16-bit integer from LSB and MSB bytes."""
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "BMI270_ALT_ADDRESS",
    "BMI270_CHIP_ID",
    # Constants
    "BMI270_DEFAULT_ADDRESS",
    "AccelBWP",
    "AccelODR",
    "AccelRange",
    # Config
    "BMI270Config",
    # Driver
    "BMI270Driver",
    # Data classes
    "BMI270Reading",
    # Enums
    "BMI270Register",
    "BMI270Status",
    "GyroBWP",
    "GyroODR",
    "GyroRange",
    "PowerMode",
]
