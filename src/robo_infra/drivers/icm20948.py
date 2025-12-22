"""ICM-20948 9-DOF Motion Sensor driver.

The ICM-20948 from TDK InvenSense is a low-power 9-axis motion sensor:
- 3-axis accelerometer (±2g, ±4g, ±8g, ±16g)
- 3-axis gyroscope (±250, ±500, ±1000, ±2000 dps)
- 3-axis magnetometer (AK09916 - ±4900µT)

Key Features:
- Ultra-low power consumption
- 1kB FIFO buffer
- Digital Motion Processor (DMP)
- I2C and SPI interfaces
- Integrated magnetometer (AK09916)

Notes:
- Default I2C address is 0x68 (or 0x69 with AD0 pin high)
- Magnetometer is on separate internal I2C bus (auxiliary)
- Temperature sensor included
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

ICM20948_DEFAULT_ADDRESS = 0x68
ICM20948_ALT_ADDRESS = 0x69
ICM20948_WHO_AM_I = 0xEA

# AK09916 magnetometer constants
AK09916_ADDRESS = 0x0C
AK09916_WHO_AM_I = 0x09


# =============================================================================
# Register Definitions
# =============================================================================


class ICM20948Bank(IntEnum):
    """Register bank selection."""

    BANK_0 = 0x00
    BANK_1 = 0x10
    BANK_2 = 0x20
    BANK_3 = 0x30


class ICM20948Register(IntEnum):
    """ICM-20948 register addresses (Bank 0)."""

    # Bank 0 registers
    WHO_AM_I = 0x00
    USER_CTRL = 0x03
    LP_CONFIG = 0x05
    PWR_MGMT_1 = 0x06
    PWR_MGMT_2 = 0x07
    INT_PIN_CFG = 0x0F
    INT_ENABLE = 0x10
    INT_ENABLE_1 = 0x11
    INT_ENABLE_2 = 0x12
    INT_ENABLE_3 = 0x13
    I2C_MST_STATUS = 0x17
    INT_STATUS = 0x19
    INT_STATUS_1 = 0x1A
    INT_STATUS_2 = 0x1B
    INT_STATUS_3 = 0x1C
    DELAY_TIMEH = 0x28
    DELAY_TIMEL = 0x29
    ACCEL_XOUT_H = 0x2D
    ACCEL_XOUT_L = 0x2E
    ACCEL_YOUT_H = 0x2F
    ACCEL_YOUT_L = 0x30
    ACCEL_ZOUT_H = 0x31
    ACCEL_ZOUT_L = 0x32
    GYRO_XOUT_H = 0x33
    GYRO_XOUT_L = 0x34
    GYRO_YOUT_H = 0x35
    GYRO_YOUT_L = 0x36
    GYRO_ZOUT_H = 0x37
    GYRO_ZOUT_L = 0x38
    TEMP_OUT_H = 0x39
    TEMP_OUT_L = 0x3A
    EXT_SLV_SENS_DATA_00 = 0x3B

    # Bank 0 - FIFO
    FIFO_EN_1 = 0x66
    FIFO_EN_2 = 0x67
    FIFO_RST = 0x68
    FIFO_MODE = 0x69
    FIFO_COUNTH = 0x70
    FIFO_COUNTL = 0x71
    FIFO_R_W = 0x72
    DATA_RDY_STATUS = 0x74
    FIFO_CFG = 0x76

    # Bank select
    REG_BANK_SEL = 0x7F


class ICM20948Bank2Register(IntEnum):
    """ICM-20948 register addresses (Bank 2)."""

    GYRO_SMPLRT_DIV = 0x00
    GYRO_CONFIG_1 = 0x01
    GYRO_CONFIG_2 = 0x02
    XG_OFFS_USRH = 0x03
    XG_OFFS_USRL = 0x04
    YG_OFFS_USRH = 0x05
    YG_OFFS_USRL = 0x06
    ZG_OFFS_USRH = 0x07
    ZG_OFFS_USRL = 0x08
    ODR_ALIGN_EN = 0x09
    ACCEL_SMPLRT_DIV_1 = 0x10
    ACCEL_SMPLRT_DIV_2 = 0x11
    ACCEL_INTEL_CTRL = 0x12
    ACCEL_WOM_THR = 0x13
    ACCEL_CONFIG = 0x14
    ACCEL_CONFIG_2 = 0x15
    FSYNC_CONFIG = 0x52
    TEMP_CONFIG = 0x53
    MOD_CTRL_USR = 0x54


class ICM20948Bank3Register(IntEnum):
    """ICM-20948 register addresses (Bank 3) - I2C Master."""

    I2C_MST_ODR_CONFIG = 0x00
    I2C_MST_CTRL = 0x01
    I2C_MST_DELAY_CTRL = 0x02
    I2C_SLV0_ADDR = 0x03
    I2C_SLV0_REG = 0x04
    I2C_SLV0_CTRL = 0x05
    I2C_SLV0_DO = 0x06
    I2C_SLV1_ADDR = 0x07
    I2C_SLV1_REG = 0x08
    I2C_SLV1_CTRL = 0x09
    I2C_SLV1_DO = 0x0A
    I2C_SLV2_ADDR = 0x0B
    I2C_SLV2_REG = 0x0C
    I2C_SLV2_CTRL = 0x0D
    I2C_SLV2_DO = 0x0E
    I2C_SLV3_ADDR = 0x0F
    I2C_SLV3_REG = 0x10
    I2C_SLV3_CTRL = 0x11
    I2C_SLV3_DO = 0x12
    I2C_SLV4_ADDR = 0x13
    I2C_SLV4_REG = 0x14
    I2C_SLV4_CTRL = 0x15
    I2C_SLV4_DO = 0x16
    I2C_SLV4_DI = 0x17


class AK09916Register(IntEnum):
    """AK09916 magnetometer registers."""

    WIA2 = 0x01
    ST1 = 0x10
    HXL = 0x11
    HXH = 0x12
    HYL = 0x13
    HYH = 0x14
    HZL = 0x15
    HZH = 0x16
    ST2 = 0x18
    CNTL2 = 0x31
    CNTL3 = 0x32


class AccelRange(IntEnum):
    """Accelerometer full-scale range."""

    RANGE_2G = 0
    RANGE_4G = 1
    RANGE_8G = 2
    RANGE_16G = 3


class GyroRange(IntEnum):
    """Gyroscope full-scale range."""

    RANGE_250DPS = 0
    RANGE_500DPS = 1
    RANGE_1000DPS = 2
    RANGE_2000DPS = 3


class MagMode(IntEnum):
    """Magnetometer operating mode."""

    POWER_DOWN = 0x00
    SINGLE = 0x01
    CONTINUOUS_10HZ = 0x02
    CONTINUOUS_20HZ = 0x04
    CONTINUOUS_50HZ = 0x06
    CONTINUOUS_100HZ = 0x08
    SELF_TEST = 0x10


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class ICM20948Reading:
    """Complete reading from ICM-20948."""

    acceleration: Vector3  # m/s²
    gyroscope: Vector3  # °/s
    magnetometer: Vector3  # µT
    temperature: float  # °C
    timestamp: float


# =============================================================================
# Configuration
# =============================================================================


class ICM20948Config(BaseModel):
    """Configuration for ICM-20948 driver."""

    model_config = {"frozen": False, "extra": "allow"}

    # I2C settings
    address: int = Field(default=ICM20948_DEFAULT_ADDRESS, description="I2C address")

    # Accelerometer
    accel_range: AccelRange = Field(
        default=AccelRange.RANGE_4G,
        description="Accelerometer full-scale range",
    )
    accel_dlpf: int = Field(
        default=3,
        ge=0,
        le=7,
        description="Accelerometer DLPF config",
    )

    # Gyroscope
    gyro_range: GyroRange = Field(
        default=GyroRange.RANGE_500DPS,
        description="Gyroscope full-scale range",
    )
    gyro_dlpf: int = Field(
        default=3,
        ge=0,
        le=7,
        description="Gyroscope DLPF config",
    )

    # Magnetometer
    mag_mode: MagMode = Field(
        default=MagMode.CONTINUOUS_100HZ,
        description="Magnetometer operating mode",
    )

    # Sample rate
    sample_rate_hz: int = Field(
        default=100,
        ge=1,
        le=1125,
        description="Sample rate in Hz",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# ICM-20948 Driver
# =============================================================================


class ICM20948Driver(Driver):
    """Driver for ICM-20948 9-DOF Motion Sensor.

    Provides access to:
    - Accelerometer (3-axis)
    - Gyroscope (3-axis)
    - Magnetometer (3-axis, via AK09916)
    - Temperature

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> imu = ICM20948Driver(bus=bus)
        >>> imu.begin()
        >>> reading = imu.read_all()
        >>> print(f"Accel X: {reading.acceleration.x:.2f} m/s²")
    """

    # Scale factors
    ACCEL_SCALE = {
        AccelRange.RANGE_2G: 16384.0,
        AccelRange.RANGE_4G: 8192.0,
        AccelRange.RANGE_8G: 4096.0,
        AccelRange.RANGE_16G: 2048.0,
    }

    GYRO_SCALE = {
        GyroRange.RANGE_250DPS: 131.0,
        GyroRange.RANGE_500DPS: 65.5,
        GyroRange.RANGE_1000DPS: 32.8,
        GyroRange.RANGE_2000DPS: 16.4,
    }

    MAG_SCALE = 0.15  # µT/LSB

    # Temperature constants
    TEMP_SENSITIVITY = 333.87
    TEMP_OFFSET = 21.0

    # Channel mapping
    CHANNEL_ACCEL_X = 0
    CHANNEL_ACCEL_Y = 1
    CHANNEL_ACCEL_Z = 2
    CHANNEL_GYRO_X = 3
    CHANNEL_GYRO_Y = 4
    CHANNEL_GYRO_Z = 5
    CHANNEL_MAG_X = 6
    CHANNEL_MAG_Y = 7
    CHANNEL_MAG_Z = 8
    CHANNEL_TEMP = 9

    def __init__(
        self,
        bus: I2CBus,
        config: ICM20948Config | None = None,
    ) -> None:
        """Initialize ICM-20948 driver.

        Args:
            bus: I2C bus for communication.
            config: Configuration options.
        """
        self._bus = bus
        self._config = config or ICM20948Config()
        self._connected = False
        self._current_bank = 255  # Force initial bank switch
        self._last_reading: ICM20948Reading | None = None
        self._channel_values: dict[int, float] = {}

    @property
    def config(self) -> ICM20948Config:
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
        raise NotImplementedError("ICM-20948 is a read-only sensor")

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
            self._channel_values[self.CHANNEL_MAG_X] = reading.magnetometer.x
            self._channel_values[self.CHANNEL_MAG_Y] = reading.magnetometer.y
            self._channel_values[self.CHANNEL_MAG_Z] = reading.magnetometer.z
            self._channel_values[self.CHANNEL_TEMP] = reading.temperature

        except Exception as e:
            logger.warning("Failed to update ICM-20948 readings: %s", e)

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def begin(self) -> bool:
        """Initialize the ICM-20948 sensor.

        Returns:
            True if initialization successful.

        Raises:
            CommunicationError: If sensor not found.
        """
        # Switch to bank 0 and verify chip ID
        self._select_bank(ICM20948Bank.BANK_0)
        who_am_i = self._read_byte(ICM20948Register.WHO_AM_I)

        if who_am_i != ICM20948_WHO_AM_I:
            raise CommunicationError(
                f"ICM-20948 not found at address 0x{self._config.address:02X}. "
                f"Expected WHO_AM_I 0x{ICM20948_WHO_AM_I:02X}, got 0x{who_am_i:02X}"
            )

        # Reset device
        self._write_byte(ICM20948Register.PWR_MGMT_1, 0x81)  # Reset
        time.sleep(0.1)

        # Wake up
        self._write_byte(ICM20948Register.PWR_MGMT_1, 0x01)  # Auto-select clock
        time.sleep(0.05)

        # Enable all sensors
        self._write_byte(ICM20948Register.PWR_MGMT_2, 0x00)

        # Configure accelerometer (Bank 2)
        self._select_bank(ICM20948Bank.BANK_2)
        accel_config = (self._config.accel_range << 1) | 0x01  # Enable DLPF
        self._write_byte(ICM20948Bank2Register.ACCEL_CONFIG, accel_config)
        self._write_byte(ICM20948Bank2Register.ACCEL_CONFIG_2, self._config.accel_dlpf)

        # Configure gyroscope (Bank 2)
        gyro_config = (self._config.gyro_range << 1) | 0x01  # Enable DLPF
        self._write_byte(ICM20948Bank2Register.GYRO_CONFIG_1, gyro_config)
        self._write_byte(ICM20948Bank2Register.GYRO_CONFIG_2, self._config.gyro_dlpf)

        # Configure sample rate
        sample_div = int(1125 / self._config.sample_rate_hz) - 1
        self._write_byte(ICM20948Bank2Register.ACCEL_SMPLRT_DIV_2, sample_div & 0xFF)
        self._write_byte(ICM20948Bank2Register.GYRO_SMPLRT_DIV, sample_div & 0xFF)

        # Initialize magnetometer
        self._init_magnetometer()

        self._select_bank(ICM20948Bank.BANK_0)
        self._connected = True

        logger.info(
            "ICM-20948 initialized (accel: ±%dg, gyro: ±%d dps, rate: %d Hz)",
            2 << self._config.accel_range,
            250 << self._config.gyro_range,
            self._config.sample_rate_hz,
        )

        return True

    def _init_magnetometer(self) -> None:
        """Initialize AK09916 magnetometer via I2C master."""
        # Enable I2C master (Bank 0)
        self._select_bank(ICM20948Bank.BANK_0)

        # Configure INT_PIN_CFG to bypass mode for direct magnetometer access
        self._write_byte(ICM20948Register.INT_PIN_CFG, 0x02)

        # Enable I2C master
        self._write_byte(ICM20948Register.USER_CTRL, 0x20)

        # Configure I2C master (Bank 3)
        self._select_bank(ICM20948Bank.BANK_3)
        self._write_byte(ICM20948Bank3Register.I2C_MST_CTRL, 0x07)  # 400kHz

        # Reset magnetometer
        self._write_mag_register(AK09916Register.CNTL3, 0x01)
        time.sleep(0.01)

        # Verify magnetometer ID
        mag_id = self._read_mag_register(AK09916Register.WIA2)
        if mag_id != AK09916_WHO_AM_I:
            logger.warning("AK09916 magnetometer not found (got 0x%02X)", mag_id)
            return

        # Set magnetometer mode
        self._write_mag_register(AK09916Register.CNTL2, self._config.mag_mode)

        # Configure slave 0 to read magnetometer data
        self._write_byte(ICM20948Bank3Register.I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80)  # Read
        self._write_byte(ICM20948Bank3Register.I2C_SLV0_REG, AK09916Register.HXL)
        self._write_byte(ICM20948Bank3Register.I2C_SLV0_CTRL, 0x88)  # Enable, 8 bytes

        logger.debug("AK09916 magnetometer initialized")

    def _write_mag_register(self, register: int, value: int) -> None:
        """Write to magnetometer register via I2C master."""
        self._select_bank(ICM20948Bank.BANK_3)
        self._write_byte(ICM20948Bank3Register.I2C_SLV0_ADDR, AK09916_ADDRESS)  # Write
        self._write_byte(ICM20948Bank3Register.I2C_SLV0_REG, register)
        self._write_byte(ICM20948Bank3Register.I2C_SLV0_DO, value)
        self._write_byte(ICM20948Bank3Register.I2C_SLV0_CTRL, 0x81)  # Enable, 1 byte
        time.sleep(0.01)

    def _read_mag_register(self, register: int) -> int:
        """Read from magnetometer register via I2C master."""
        self._select_bank(ICM20948Bank.BANK_3)
        self._write_byte(ICM20948Bank3Register.I2C_SLV4_ADDR, AK09916_ADDRESS | 0x80)  # Read
        self._write_byte(ICM20948Bank3Register.I2C_SLV4_REG, register)
        self._write_byte(ICM20948Bank3Register.I2C_SLV4_CTRL, 0x80)  # Enable
        time.sleep(0.01)
        return self._read_byte(ICM20948Bank3Register.I2C_SLV4_DI)

    # -------------------------------------------------------------------------
    # Reading Data
    # -------------------------------------------------------------------------

    def read_all(self) -> ICM20948Reading:
        """Read all sensor data.

        Returns:
            ICM20948Reading with all sensor values.
        """
        self._select_bank(ICM20948Bank.BANK_0)

        # Read accel (6 bytes) + gyro (6 bytes) + temp (2 bytes) + mag (8 bytes)
        data = self._read_bytes(ICM20948Register.ACCEL_XOUT_H, 14)

        # Parse accelerometer
        accel_scale = self.ACCEL_SCALE[self._config.accel_range]
        ax = self._unpack_int16(data[1], data[0]) / accel_scale * 9.81
        ay = self._unpack_int16(data[3], data[2]) / accel_scale * 9.81
        az = self._unpack_int16(data[5], data[4]) / accel_scale * 9.81

        # Parse gyroscope
        gyro_scale = self.GYRO_SCALE[self._config.gyro_range]
        gx = self._unpack_int16(data[7], data[6]) / gyro_scale
        gy = self._unpack_int16(data[9], data[8]) / gyro_scale
        gz = self._unpack_int16(data[11], data[10]) / gyro_scale

        # Parse temperature
        temp_raw = self._unpack_int16(data[13], data[12])
        temperature = (temp_raw - self.TEMP_OFFSET) / self.TEMP_SENSITIVITY + self.TEMP_OFFSET

        # Read magnetometer from external sensor data
        mag_data = self._read_bytes(ICM20948Register.EXT_SLV_SENS_DATA_00, 8)
        mx = self._unpack_int16(mag_data[0], mag_data[1]) * self.MAG_SCALE
        my = self._unpack_int16(mag_data[2], mag_data[3]) * self.MAG_SCALE
        mz = self._unpack_int16(mag_data[4], mag_data[5]) * self.MAG_SCALE

        reading = ICM20948Reading(
            acceleration=Vector3(x=ax, y=ay, z=az),
            gyroscope=Vector3(x=gx, y=gy, z=gz),
            magnetometer=Vector3(x=mx, y=my, z=mz),
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
        self._select_bank(ICM20948Bank.BANK_0)
        data = self._read_bytes(ICM20948Register.ACCEL_XOUT_H, 6)

        scale = self.ACCEL_SCALE[self._config.accel_range]
        return Vector3(
            x=self._unpack_int16(data[1], data[0]) / scale * 9.81,
            y=self._unpack_int16(data[3], data[2]) / scale * 9.81,
            z=self._unpack_int16(data[5], data[4]) / scale * 9.81,
        )

    def read_gyroscope(self) -> Vector3:
        """Read gyroscope data only.

        Returns:
            Vector3 with angular velocity in °/s.
        """
        self._select_bank(ICM20948Bank.BANK_0)
        data = self._read_bytes(ICM20948Register.GYRO_XOUT_H, 6)

        scale = self.GYRO_SCALE[self._config.gyro_range]
        return Vector3(
            x=self._unpack_int16(data[1], data[0]) / scale,
            y=self._unpack_int16(data[3], data[2]) / scale,
            z=self._unpack_int16(data[5], data[4]) / scale,
        )

    def read_magnetometer(self) -> Vector3:
        """Read magnetometer data only.

        Returns:
            Vector3 with magnetic field in µT.
        """
        self._select_bank(ICM20948Bank.BANK_0)
        data = self._read_bytes(ICM20948Register.EXT_SLV_SENS_DATA_00, 6)

        return Vector3(
            x=self._unpack_int16(data[0], data[1]) * self.MAG_SCALE,
            y=self._unpack_int16(data[2], data[3]) * self.MAG_SCALE,
            z=self._unpack_int16(data[4], data[5]) * self.MAG_SCALE,
        )

    def read_temperature(self) -> float:
        """Read temperature.

        Returns:
            Temperature in °C.
        """
        self._select_bank(ICM20948Bank.BANK_0)
        data = self._read_bytes(ICM20948Register.TEMP_OUT_H, 2)

        raw = self._unpack_int16(data[1], data[0])
        return (raw - self.TEMP_OFFSET) / self.TEMP_SENSITIVITY + self.TEMP_OFFSET

    # -------------------------------------------------------------------------
    # Low-Level I2C
    # -------------------------------------------------------------------------

    def _select_bank(self, bank: ICM20948Bank) -> None:
        """Select register bank."""
        if bank != self._current_bank:
            self._bus.write_register(
                self._config.address,
                ICM20948Register.REG_BANK_SEL,
                bytes([bank]),
            )
            self._current_bank = bank

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
    "ICM20948_ALT_ADDRESS",
    # Constants
    "ICM20948_DEFAULT_ADDRESS",
    "ICM20948_WHO_AM_I",
    "AK09916Register",
    "AccelRange",
    "GyroRange",
    # Enums
    "ICM20948Bank",
    "ICM20948Bank2Register",
    "ICM20948Bank3Register",
    # Config
    "ICM20948Config",
    # Driver
    "ICM20948Driver",
    # Data classes
    "ICM20948Reading",
    "ICM20948Register",
    "MagMode",
]
