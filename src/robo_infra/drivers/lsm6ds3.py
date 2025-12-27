"""LSM6DS3 6-DOF Motion Sensor driver.

The LSM6DS3 from STMicroelectronics is a system-in-package featuring:
- 3-axis accelerometer (±2g, ±4g, ±8g, ±16g)
- 3-axis gyroscope (±125, ±245, ±500, ±1000, ±2000 dps)

Key Features:
- Ultra-low power (0.65mA in high-performance mode)
- 8kB FIFO buffer
- Embedded temperature sensor
- I2C and SPI interfaces
- Programmable interrupts
- Step counter and tilt detection

Notes:
- Default I2C address is 0x6A (or 0x6B with SA0 pin high)
- Often used in wearables and mobile devices
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.driver import Driver
from robo_infra.core.exceptions import CommunicationError
from robo_infra.core.types import Vector3


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

LSM6DS3_DEFAULT_ADDRESS = 0x6A
LSM6DS3_ALT_ADDRESS = 0x6B
LSM6DS3_WHO_AM_I = 0x69


# =============================================================================
# Register Definitions
# =============================================================================


class LSM6DS3Register(IntEnum):
    """LSM6DS3 register addresses."""

    FUNC_CFG_ACCESS = 0x01
    SENSOR_SYNC_TIME_FRAME = 0x04
    FIFO_CTRL1 = 0x06
    FIFO_CTRL2 = 0x07
    FIFO_CTRL3 = 0x08
    FIFO_CTRL4 = 0x09
    FIFO_CTRL5 = 0x0A
    ORIENT_CFG_G = 0x0B
    INT1_CTRL = 0x0D
    INT2_CTRL = 0x0E
    WHO_AM_I = 0x0F
    CTRL1_XL = 0x10
    CTRL2_G = 0x11
    CTRL3_C = 0x12
    CTRL4_C = 0x13
    CTRL5_C = 0x14
    CTRL6_C = 0x15
    CTRL7_G = 0x16
    CTRL8_XL = 0x17
    CTRL9_XL = 0x18
    CTRL10_C = 0x19
    MASTER_CONFIG = 0x1A
    WAKE_UP_SRC = 0x1B
    TAP_SRC = 0x1C
    D6D_SRC = 0x1D
    STATUS_REG = 0x1E
    OUT_TEMP_L = 0x20
    OUT_TEMP_H = 0x21
    OUTX_L_G = 0x22
    OUTX_H_G = 0x23
    OUTY_L_G = 0x24
    OUTY_H_G = 0x25
    OUTZ_L_G = 0x26
    OUTZ_H_G = 0x27
    OUTX_L_XL = 0x28
    OUTX_H_XL = 0x29
    OUTY_L_XL = 0x2A
    OUTY_H_XL = 0x2B
    OUTZ_L_XL = 0x2C
    OUTZ_H_XL = 0x2D
    SENSORHUB1_REG = 0x2E
    FIFO_STATUS1 = 0x3A
    FIFO_STATUS2 = 0x3B
    FIFO_STATUS3 = 0x3C
    FIFO_STATUS4 = 0x3D
    FIFO_DATA_OUT_L = 0x3E
    FIFO_DATA_OUT_H = 0x3F
    TIMESTAMP0_REG = 0x40
    TIMESTAMP1_REG = 0x41
    TIMESTAMP2_REG = 0x42
    STEP_TIMESTAMP_L = 0x49
    STEP_TIMESTAMP_H = 0x4A
    STEP_COUNTER_L = 0x4B
    STEP_COUNTER_H = 0x4C
    FUNC_SRC = 0x53
    TAP_CFG = 0x58
    TAP_THS_6D = 0x59
    INT_DUR2 = 0x5A
    WAKE_UP_THS = 0x5B
    WAKE_UP_DUR = 0x5C
    FREE_FALL = 0x5D
    MD1_CFG = 0x5E
    MD2_CFG = 0x5F


class AccelODR(IntEnum):
    """Accelerometer output data rate."""

    POWER_DOWN = 0
    ODR_12_5HZ = 1
    ODR_26HZ = 2
    ODR_52HZ = 3
    ODR_104HZ = 4
    ODR_208HZ = 5
    ODR_416HZ = 6
    ODR_833HZ = 7
    ODR_1660HZ = 8
    ODR_3330HZ = 9
    ODR_6660HZ = 10


class AccelScale(IntEnum):
    """Accelerometer full-scale range."""

    SCALE_2G = 0
    SCALE_16G = 1
    SCALE_4G = 2
    SCALE_8G = 3


class GyroODR(IntEnum):
    """Gyroscope output data rate."""

    POWER_DOWN = 0
    ODR_12_5HZ = 1
    ODR_26HZ = 2
    ODR_52HZ = 3
    ODR_104HZ = 4
    ODR_208HZ = 5
    ODR_416HZ = 6
    ODR_833HZ = 7
    ODR_1660HZ = 8


class GyroScale(IntEnum):
    """Gyroscope full-scale range."""

    SCALE_250DPS = 0
    SCALE_500DPS = 1
    SCALE_1000DPS = 2
    SCALE_2000DPS = 3


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class LSM6DS3Reading:
    """Complete reading from LSM6DS3."""

    acceleration: Vector3  # m/s²
    gyroscope: Vector3  # °/s
    temperature: float  # °C
    timestamp: float


# =============================================================================
# Configuration
# =============================================================================


class LSM6DS3Config(BaseModel):
    """Configuration for LSM6DS3 driver."""

    model_config = {"frozen": False, "extra": "allow"}

    # I2C settings
    address: int = Field(default=LSM6DS3_DEFAULT_ADDRESS, description="I2C address")

    # Accelerometer
    accel_odr: AccelODR = Field(
        default=AccelODR.ODR_104HZ,
        description="Accelerometer output data rate",
    )
    accel_scale: AccelScale = Field(
        default=AccelScale.SCALE_4G,
        description="Accelerometer full-scale range",
    )

    # Gyroscope
    gyro_odr: GyroODR = Field(
        default=GyroODR.ODR_104HZ,
        description="Gyroscope output data rate",
    )
    gyro_scale: GyroScale = Field(
        default=GyroScale.SCALE_500DPS,
        description="Gyroscope full-scale range",
    )

    # Block data update
    bdu: bool = Field(
        default=True,
        description="Block data update until MSB and LSB are read",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# LSM6DS3 Driver
# =============================================================================


class LSM6DS3Driver(Driver):
    """Driver for LSM6DS3 6-DOF Motion Sensor.

    Provides access to:
    - Accelerometer (3-axis)
    - Gyroscope (3-axis)
    - Temperature

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> imu = LSM6DS3Driver(bus=bus)
        >>> imu.begin()
        >>> reading = imu.read_all()
        >>> print(f"Accel X: {reading.acceleration.x:.2f} m/s²")
    """

    # Scale factors (sensitivity in LSB/unit)
    ACCEL_SENSITIVITY = {
        AccelScale.SCALE_2G: 16384.0,  # LSB/g
        AccelScale.SCALE_4G: 8192.0,
        AccelScale.SCALE_8G: 4096.0,
        AccelScale.SCALE_16G: 1365.33,
    }

    GYRO_SENSITIVITY = {
        GyroScale.SCALE_250DPS: 131.0,  # LSB/(°/s)
        GyroScale.SCALE_500DPS: 65.5,
        GyroScale.SCALE_1000DPS: 32.8,
        GyroScale.SCALE_2000DPS: 16.4,
    }

    # Temperature constants
    TEMP_SENSITIVITY = 16.0  # LSB/°C
    TEMP_OFFSET = 25.0  # °C at 0 LSB

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
        config: LSM6DS3Config | None = None,
    ) -> None:
        """Initialize LSM6DS3 driver.

        Args:
            bus: I2C bus for communication.
            config: Configuration options.
        """
        self._bus = bus
        self._config = config or LSM6DS3Config()
        self._connected = False
        self._last_reading: LSM6DS3Reading | None = None
        self._channel_values: dict[int, float] = {}

    @property
    def config(self) -> LSM6DS3Config:
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
        raise NotImplementedError("LSM6DS3 is a read-only sensor")

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
            logger.warning("Failed to update LSM6DS3 readings: %s", e)

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def begin(self) -> bool:
        """Initialize the LSM6DS3 sensor.

        Returns:
            True if initialization successful.

        Raises:
            CommunicationError: If sensor not found.
        """
        # Verify chip ID
        who_am_i = self._read_byte(LSM6DS3Register.WHO_AM_I)

        if who_am_i != LSM6DS3_WHO_AM_I:
            raise CommunicationError(
                f"LSM6DS3 not found at address 0x{self._config.address:02X}. "
                f"Expected WHO_AM_I 0x{LSM6DS3_WHO_AM_I:02X}, got 0x{who_am_i:02X}"
            )

        # Software reset
        ctrl3 = self._read_byte(LSM6DS3Register.CTRL3_C)
        self._write_byte(LSM6DS3Register.CTRL3_C, ctrl3 | 0x01)
        time.sleep(0.05)

        # Configure CTRL3_C (block data update, auto-increment)
        ctrl3_val = 0x04  # IF_INC
        if self._config.bdu:
            ctrl3_val |= 0x40  # BDU
        self._write_byte(LSM6DS3Register.CTRL3_C, ctrl3_val)

        # Configure accelerometer (CTRL1_XL)
        ctrl1_xl = (self._config.accel_odr << 4) | (self._config.accel_scale << 2)
        self._write_byte(LSM6DS3Register.CTRL1_XL, ctrl1_xl)

        # Configure gyroscope (CTRL2_G)
        ctrl2_g = (self._config.gyro_odr << 4) | (self._config.gyro_scale << 2)
        self._write_byte(LSM6DS3Register.CTRL2_G, ctrl2_g)

        self._connected = True

        logger.info(
            "LSM6DS3 initialized (accel: %s, gyro: %s)",
            self._config.accel_scale.name,
            self._config.gyro_scale.name,
        )

        return True

    def reset(self) -> None:
        """Perform a software reset."""
        ctrl3 = self._read_byte(LSM6DS3Register.CTRL3_C)
        self._write_byte(LSM6DS3Register.CTRL3_C, ctrl3 | 0x01)
        time.sleep(0.05)
        self.begin()

    # -------------------------------------------------------------------------
    # Reading Data
    # -------------------------------------------------------------------------

    def read_all(self) -> LSM6DS3Reading:
        """Read all sensor data.

        Returns:
            LSM6DS3Reading with all sensor values.
        """
        # Read temp (2) + gyro (6) + accel (6) = 14 bytes
        data = self._read_bytes(LSM6DS3Register.OUT_TEMP_L, 14)

        # Parse temperature
        temp_raw = self._unpack_int16(data[0], data[1])
        temperature = temp_raw / self.TEMP_SENSITIVITY + self.TEMP_OFFSET

        # Parse gyroscope
        gyro_sens = self.GYRO_SENSITIVITY[self._config.gyro_scale]
        gx = self._unpack_int16(data[2], data[3]) / gyro_sens
        gy = self._unpack_int16(data[4], data[5]) / gyro_sens
        gz = self._unpack_int16(data[6], data[7]) / gyro_sens

        # Parse accelerometer
        accel_sens = self.ACCEL_SENSITIVITY[self._config.accel_scale]
        ax = self._unpack_int16(data[8], data[9]) / accel_sens * 9.81
        ay = self._unpack_int16(data[10], data[11]) / accel_sens * 9.81
        az = self._unpack_int16(data[12], data[13]) / accel_sens * 9.81

        reading = LSM6DS3Reading(
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
        data = self._read_bytes(LSM6DS3Register.OUTX_L_XL, 6)
        sens = self.ACCEL_SENSITIVITY[self._config.accel_scale]

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
        data = self._read_bytes(LSM6DS3Register.OUTX_L_G, 6)
        sens = self.GYRO_SENSITIVITY[self._config.gyro_scale]

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
        data = self._read_bytes(LSM6DS3Register.OUT_TEMP_L, 2)
        raw = self._unpack_int16(data[0], data[1])
        return raw / self.TEMP_SENSITIVITY + self.TEMP_OFFSET

    def data_ready(self) -> tuple[bool, bool]:
        """Check if new data is available.

        Returns:
            Tuple of (accel_ready, gyro_ready).
        """
        status = self._read_byte(LSM6DS3Register.STATUS_REG)
        return bool(status & 0x01), bool(status & 0x02)

    # -------------------------------------------------------------------------
    # Step Counter
    # -------------------------------------------------------------------------

    def enable_step_counter(self) -> None:
        """Enable the pedometer (step counter)."""
        # Enable embedded functions
        ctrl10 = self._read_byte(LSM6DS3Register.CTRL10_C)
        self._write_byte(LSM6DS3Register.CTRL10_C, ctrl10 | 0x04)  # PEDO_EN

        # Configure tap settings for pedometer
        tap_cfg = self._read_byte(LSM6DS3Register.TAP_CFG)
        self._write_byte(LSM6DS3Register.TAP_CFG, tap_cfg | 0x40)  # PEDO_EN

    def read_step_count(self) -> int:
        """Read step counter value.

        Returns:
            Number of steps counted.
        """
        data = self._read_bytes(LSM6DS3Register.STEP_COUNTER_L, 2)
        return data[0] | (data[1] << 8)

    def reset_step_counter(self) -> None:
        """Reset step counter to zero."""
        ctrl10 = self._read_byte(LSM6DS3Register.CTRL10_C)
        self._write_byte(LSM6DS3Register.CTRL10_C, ctrl10 | 0x02)  # PEDO_RST_STEP
        time.sleep(0.01)
        self._write_byte(LSM6DS3Register.CTRL10_C, ctrl10)

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
    "LSM6DS3_ALT_ADDRESS",
    # Constants
    "LSM6DS3_DEFAULT_ADDRESS",
    "LSM6DS3_WHO_AM_I",
    "AccelODR",
    "AccelScale",
    "GyroODR",
    "GyroScale",
    # Config
    "LSM6DS3Config",
    # Driver
    "LSM6DS3Driver",
    # Data classes
    "LSM6DS3Reading",
    # Enums
    "LSM6DS3Register",
]
