"""Power monitor drivers for INA219 and INA226.

This module provides drivers for I2C-based power monitor ICs commonly used
in robotics for battery monitoring and power management.

Supported Devices:
- INA219: 12-bit bidirectional current/power monitor (up to 26V, ±3.2A)
- INA226: 16-bit bidirectional current/power monitor (up to 36V, ±80A)

Example:
    >>> from robo_infra.power.drivers import INA219Driver, INA219Config
    >>> config = INA219Config(address=0x40, shunt_ohms=0.1)
    >>> driver = INA219Driver(config=config)
    >>> driver.enable()
    >>> print(f"Voltage: {driver.read_voltage():.2f}V")
    >>> print(f"Current: {driver.read_current():.3f}A")
"""

from __future__ import annotations

import logging
import struct
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

# INA219 Register Addresses
INA219_REG_CONFIG = 0x00
INA219_REG_SHUNT_VOLTAGE = 0x01
INA219_REG_BUS_VOLTAGE = 0x02
INA219_REG_POWER = 0x03
INA219_REG_CURRENT = 0x04
INA219_REG_CALIBRATION = 0x05

# INA226 Register Addresses
INA226_REG_CONFIG = 0x00
INA226_REG_SHUNT_VOLTAGE = 0x01
INA226_REG_BUS_VOLTAGE = 0x02
INA226_REG_POWER = 0x03
INA226_REG_CURRENT = 0x04
INA226_REG_CALIBRATION = 0x05
INA226_REG_MASK_ENABLE = 0x06
INA226_REG_ALERT_LIMIT = 0x07
INA226_REG_MANUFACTURER_ID = 0xFE
INA226_REG_DIE_ID = 0xFF

# Default I2C addresses
INA219_DEFAULT_ADDRESS = 0x40
INA226_DEFAULT_ADDRESS = 0x40


# =============================================================================
# Enums
# =============================================================================


class INA219BusVoltageRange(IntEnum):
    """INA219 bus voltage range configuration."""

    RANGE_16V = 0  # 16V range
    RANGE_32V = 1  # 32V range (default)


class INA219Gain(IntEnum):
    """INA219 PGA gain configuration."""

    GAIN_1_40MV = 0  # ±40mV range
    GAIN_2_80MV = 1  # ±80mV range
    GAIN_4_160MV = 2  # ±160mV range
    GAIN_8_320MV = 3  # ±320mV range (default)


class INA219ADCResolution(IntEnum):
    """INA219 ADC resolution/averaging configuration."""

    ADC_9BIT_1S = 0  # 9-bit, 1 sample, 84μs
    ADC_10BIT_1S = 1  # 10-bit, 1 sample, 148μs
    ADC_11BIT_1S = 2  # 11-bit, 1 sample, 276μs
    ADC_12BIT_1S = 3  # 12-bit, 1 sample, 532μs (default)
    ADC_12BIT_2S = 9  # 12-bit, 2 samples, 1.06ms
    ADC_12BIT_4S = 10  # 12-bit, 4 samples, 2.13ms
    ADC_12BIT_8S = 11  # 12-bit, 8 samples, 4.26ms
    ADC_12BIT_16S = 12  # 12-bit, 16 samples, 8.51ms
    ADC_12BIT_32S = 13  # 12-bit, 32 samples, 17.02ms
    ADC_12BIT_64S = 14  # 12-bit, 64 samples, 34.05ms
    ADC_12BIT_128S = 15  # 12-bit, 128 samples, 68.10ms


class INA226AveragingMode(IntEnum):
    """INA226 averaging mode configuration."""

    AVG_1 = 0  # 1 sample
    AVG_4 = 1  # 4 samples
    AVG_16 = 2  # 16 samples
    AVG_64 = 3  # 64 samples
    AVG_128 = 4  # 128 samples
    AVG_256 = 5  # 256 samples
    AVG_512 = 6  # 512 samples
    AVG_1024 = 7  # 1024 samples


class INA226ConversionTime(IntEnum):
    """INA226 conversion time configuration."""

    TIME_140US = 0  # 140μs
    TIME_204US = 1  # 204μs
    TIME_332US = 2  # 332μs
    TIME_588US = 3  # 588μs
    TIME_1100US = 4  # 1.1ms (default)
    TIME_2116US = 5  # 2.116ms
    TIME_4156US = 6  # 4.156ms
    TIME_8244US = 7  # 8.244ms


# =============================================================================
# Base Class
# =============================================================================


class PowerMonitorDriver(ABC):
    """Abstract base class for power monitor drivers.

    All power monitor drivers must implement:
    - enable(): Initialize and start the driver
    - disable(): Stop and cleanup the driver
    - read_voltage(): Read bus voltage
    - read_current(): Read current
    - read_power(): Read power
    """

    @abstractmethod
    def enable(self) -> None:
        """Enable the power monitor."""
        ...

    @abstractmethod
    def disable(self) -> None:
        """Disable the power monitor."""
        ...

    @abstractmethod
    def read_voltage(self) -> float:
        """Read bus voltage in volts.

        Returns:
            Bus voltage in V.
        """
        ...

    @abstractmethod
    def read_current(self) -> float:
        """Read current in amperes.

        Returns:
            Current in A (positive = into load).
        """
        ...

    @abstractmethod
    def read_power(self) -> float:
        """Read power in watts.

        Returns:
            Power in W.
        """
        ...

    @abstractmethod
    def is_enabled(self) -> bool:
        """Check if driver is enabled."""
        ...


# =============================================================================
# Reading Data Class
# =============================================================================


@dataclass
class PowerReading:
    """Power monitor reading."""

    voltage: float  # Bus voltage (V)
    current: float  # Current (A)
    power: float  # Power (W)
    shunt_voltage: float  # Shunt voltage (mV)
    timestamp: float


# =============================================================================
# INA219 Configuration
# =============================================================================


class INA219Config(BaseModel):
    """Configuration for INA219 power monitor."""

    model_config = {"frozen": False, "extra": "allow"}

    # I2C configuration
    address: int = Field(
        default=INA219_DEFAULT_ADDRESS,
        ge=0x00,
        le=0x7F,
        description="I2C address",
    )

    # Shunt resistor
    shunt_ohms: float = Field(
        default=0.1,
        gt=0.0,
        description="Shunt resistor value in ohms",
    )

    # Expected maximum current
    max_expected_current: float = Field(
        default=3.2,
        gt=0.0,
        description="Maximum expected current in A",
    )

    # Configuration options
    bus_voltage_range: INA219BusVoltageRange = Field(
        default=INA219BusVoltageRange.RANGE_32V,
        description="Bus voltage range",
    )
    gain: INA219Gain = Field(
        default=INA219Gain.GAIN_8_320MV,
        description="PGA gain",
    )
    bus_adc_resolution: INA219ADCResolution = Field(
        default=INA219ADCResolution.ADC_12BIT_1S,
        description="Bus ADC resolution",
    )
    shunt_adc_resolution: INA219ADCResolution = Field(
        default=INA219ADCResolution.ADC_12BIT_1S,
        description="Shunt ADC resolution",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# INA219 Driver
# =============================================================================


class INA219Driver(PowerMonitorDriver):
    """Driver for INA219 I2C power monitor.

    The INA219 is a 12-bit bidirectional current/power monitor IC that
    monitors voltage, current, and power on a circuit by measuring
    the voltage across a shunt resistor.

    Features:
    - 12-bit ADC resolution
    - Up to 26V bus voltage range
    - ±3.2A current measurement (with 0.1Ω shunt)
    - Programmable gain
    - I2C interface

    Example:
        >>> from robo_infra.power.drivers import INA219Driver, INA219Config
        >>> config = INA219Config(address=0x40, shunt_ohms=0.1)
        >>> driver = INA219Driver(config=config)
        >>> driver.enable()
        >>> print(f"Voltage: {driver.read_voltage():.2f}V")
        >>> print(f"Current: {driver.read_current():.3f}A")
    """

    def __init__(
        self,
        config: INA219Config | None = None,
        bus: I2CBus | None = None,
        address: int = INA219_DEFAULT_ADDRESS,
        shunt_ohms: float = 0.1,
    ) -> None:
        """Initialize INA219 driver.

        Args:
            config: Configuration object (overrides other args).
            bus: I2C bus instance for communication.
            address: I2C address (default: 0x40).
            shunt_ohms: Shunt resistor value in ohms.
        """
        if config is not None:
            self._config = config
        else:
            self._config = INA219Config(address=address, shunt_ohms=shunt_ohms)

        self._bus = bus
        self._enabled = False

        # Calculated calibration values
        self._current_lsb: float = 0.0
        self._power_lsb: float = 0.0
        self._calibration: int = 0

        # Simulated values (when no bus)
        self._simulated_voltage = 12.0
        self._simulated_current = 0.5

        logger.debug(
            "INA219Driver initialized: addr=0x%02X, shunt=%.3fΩ",
            self._config.address,
            self._config.shunt_ohms,
        )

    @property
    def config(self) -> INA219Config:
        """Get configuration."""
        return self._config

    def is_enabled(self) -> bool:
        """Check if driver is enabled."""
        return self._enabled

    def enable(self) -> None:
        """Enable and configure the INA219."""
        if self._enabled:
            return

        # Calculate calibration
        self._calculate_calibration()

        # Configure the device
        if self._bus is not None:
            # Reset
            self._write_register(INA219_REG_CONFIG, 0x8000)
            time.sleep(0.001)

            # Write configuration
            config_value = self._build_config_register()
            self._write_register(INA219_REG_CONFIG, config_value)

            # Write calibration
            self._write_register(INA219_REG_CALIBRATION, self._calibration)

        self._enabled = True
        logger.info(
            "INA219 enabled: addr=0x%02X, calibration=%d",
            self._config.address,
            self._calibration,
        )

    def disable(self) -> None:
        """Disable the INA219."""
        if not self._enabled:
            return

        # Power down mode
        if self._bus is not None:
            self._write_register(INA219_REG_CONFIG, 0x0000)

        self._enabled = False
        logger.info("INA219 disabled: addr=0x%02X", self._config.address)

    def read_voltage(self) -> float:
        """Read bus voltage.

        Returns:
            Bus voltage in V.
        """
        if not self._enabled:
            raise RuntimeError("INA219 is not enabled")

        if self._bus is not None:
            raw = self._read_register(INA219_REG_BUS_VOLTAGE)
            # Shift right 3 bits, multiply by 4mV LSB
            return (raw >> 3) * 0.004
        else:
            return self._simulated_voltage

    def read_shunt_voltage(self) -> float:
        """Read shunt voltage.

        Returns:
            Shunt voltage in mV.
        """
        if not self._enabled:
            raise RuntimeError("INA219 is not enabled")

        if self._bus is not None:
            raw = self._read_register_signed(INA219_REG_SHUNT_VOLTAGE)
            # LSB is 10μV
            return raw * 0.01
        else:
            # Calculate from simulated current
            return self._simulated_current * self._config.shunt_ohms * 1000

    def read_current(self) -> float:
        """Read current.

        Returns:
            Current in A (positive = into load).
        """
        if not self._enabled:
            raise RuntimeError("INA219 is not enabled")

        if self._bus is not None:
            raw = self._read_register_signed(INA219_REG_CURRENT)
            return raw * self._current_lsb
        else:
            return self._simulated_current

    def read_power(self) -> float:
        """Read power.

        Returns:
            Power in W.
        """
        if not self._enabled:
            raise RuntimeError("INA219 is not enabled")

        if self._bus is not None:
            raw = self._read_register(INA219_REG_POWER)
            return raw * self._power_lsb
        else:
            return self._simulated_voltage * self._simulated_current

    def read_all(self) -> PowerReading:
        """Read all values.

        Returns:
            PowerReading with voltage, current, power, and shunt voltage.
        """
        return PowerReading(
            voltage=self.read_voltage(),
            current=self.read_current(),
            power=self.read_power(),
            shunt_voltage=self.read_shunt_voltage(),
            timestamp=time.time(),
        )

    def set_simulated_values(self, voltage: float, current: float) -> None:
        """Set simulated values for testing.

        Args:
            voltage: Simulated bus voltage.
            current: Simulated current.
        """
        self._simulated_voltage = voltage
        self._simulated_current = current

    def _calculate_calibration(self) -> None:
        """Calculate calibration register value."""
        # Calculate current LSB = max_current / 2^15
        self._current_lsb = self._config.max_expected_current / 32768

        # Calculate calibration = trunc(0.04096 / (current_lsb * shunt))
        self._calibration = int(0.04096 / (self._current_lsb * self._config.shunt_ohms))

        # Clamp to 16-bit
        self._calibration = min(0xFFFF, self._calibration)

        # Power LSB = 20 * current_lsb
        self._power_lsb = 20 * self._current_lsb

        logger.debug(
            "INA219 calibration: cal=%d, current_lsb=%.6f, power_lsb=%.6f",
            self._calibration,
            self._current_lsb,
            self._power_lsb,
        )

    def _build_config_register(self) -> int:
        """Build configuration register value."""
        config = 0

        # Bus voltage range (bit 13)
        config |= (self._config.bus_voltage_range & 0x01) << 13

        # PGA gain (bits 11-12)
        config |= (self._config.gain & 0x03) << 11

        # Bus ADC resolution (bits 7-10)
        config |= (self._config.bus_adc_resolution & 0x0F) << 7

        # Shunt ADC resolution (bits 3-6)
        config |= (self._config.shunt_adc_resolution & 0x0F) << 3

        # Operating mode: continuous shunt and bus (bits 0-2 = 0x7)
        config |= 0x07

        return config

    def _write_register(self, reg: int, value: int) -> None:
        """Write 16-bit register."""
        if self._bus is not None:
            data = struct.pack(">H", value)
            self._bus.write(self._config.address, bytes([reg]) + data)

    def _read_register(self, reg: int) -> int:
        """Read 16-bit unsigned register."""
        if self._bus is not None:
            self._bus.write(self._config.address, bytes([reg]))
            data = self._bus.read(self._config.address, 2)
            return struct.unpack(">H", data)[0]
        return 0

    def _read_register_signed(self, reg: int) -> int:
        """Read 16-bit signed register."""
        if self._bus is not None:
            self._bus.write(self._config.address, bytes([reg]))
            data = self._bus.read(self._config.address, 2)
            return struct.unpack(">h", data)[0]
        return 0


# =============================================================================
# INA226 Configuration
# =============================================================================


class INA226Config(BaseModel):
    """Configuration for INA226 power monitor."""

    model_config = {"frozen": False, "extra": "allow"}

    # I2C configuration
    address: int = Field(
        default=INA226_DEFAULT_ADDRESS,
        ge=0x00,
        le=0x7F,
        description="I2C address",
    )

    # Shunt resistor
    shunt_ohms: float = Field(
        default=0.1,
        gt=0.0,
        description="Shunt resistor value in ohms",
    )

    # Expected maximum current
    max_expected_current: float = Field(
        default=10.0,
        gt=0.0,
        description="Maximum expected current in A",
    )

    # Configuration options
    averaging_mode: INA226AveragingMode = Field(
        default=INA226AveragingMode.AVG_1,
        description="Averaging mode",
    )
    bus_conversion_time: INA226ConversionTime = Field(
        default=INA226ConversionTime.TIME_1100US,
        description="Bus voltage conversion time",
    )
    shunt_conversion_time: INA226ConversionTime = Field(
        default=INA226ConversionTime.TIME_1100US,
        description="Shunt voltage conversion time",
    )

    # Alert thresholds
    over_voltage_limit: float | None = Field(
        default=None,
        description="Over voltage alert threshold (V)",
    )
    under_voltage_limit: float | None = Field(
        default=None,
        description="Under voltage alert threshold (V)",
    )
    over_current_limit: float | None = Field(
        default=None,
        description="Over current alert threshold (A)",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# INA226 Driver
# =============================================================================


class INA226Driver(PowerMonitorDriver):
    """Driver for INA226 I2C power monitor.

    The INA226 is a 16-bit bidirectional current/power monitor IC with
    higher resolution and wider voltage range than the INA219.

    Features:
    - 16-bit ADC resolution
    - Up to 36V bus voltage range
    - High-side or low-side sensing
    - Programmable averaging
    - Alert functions
    - I2C interface

    Example:
        >>> from robo_infra.power.drivers import INA226Driver, INA226Config
        >>> config = INA226Config(address=0x40, shunt_ohms=0.01)
        >>> driver = INA226Driver(config=config)
        >>> driver.enable()
        >>> print(f"Voltage: {driver.read_voltage():.2f}V")
        >>> print(f"Current: {driver.read_current():.3f}A")
    """

    def __init__(
        self,
        config: INA226Config | None = None,
        bus: I2CBus | None = None,
        address: int = INA226_DEFAULT_ADDRESS,
        shunt_ohms: float = 0.01,
    ) -> None:
        """Initialize INA226 driver.

        Args:
            config: Configuration object (overrides other args).
            bus: I2C bus instance for communication.
            address: I2C address (default: 0x40).
            shunt_ohms: Shunt resistor value in ohms.
        """
        if config is not None:
            self._config = config
        else:
            self._config = INA226Config(address=address, shunt_ohms=shunt_ohms)

        self._bus = bus
        self._enabled = False

        # Calculated calibration values
        self._current_lsb: float = 0.0
        self._power_lsb: float = 0.0
        self._calibration: int = 0

        # Simulated values (when no bus)
        self._simulated_voltage = 24.0
        self._simulated_current = 1.0

        logger.debug(
            "INA226Driver initialized: addr=0x%02X, shunt=%.3fΩ",
            self._config.address,
            self._config.shunt_ohms,
        )

    @property
    def config(self) -> INA226Config:
        """Get configuration."""
        return self._config

    def is_enabled(self) -> bool:
        """Check if driver is enabled."""
        return self._enabled

    def enable(self) -> None:
        """Enable and configure the INA226."""
        if self._enabled:
            return

        # Calculate calibration
        self._calculate_calibration()

        # Configure the device
        if self._bus is not None:
            # Reset
            self._write_register(INA226_REG_CONFIG, 0x8000)
            time.sleep(0.001)

            # Write configuration
            config_value = self._build_config_register()
            self._write_register(INA226_REG_CONFIG, config_value)

            # Write calibration
            self._write_register(INA226_REG_CALIBRATION, self._calibration)

            # Configure alerts if set
            self._configure_alerts()

        self._enabled = True
        logger.info(
            "INA226 enabled: addr=0x%02X, calibration=%d",
            self._config.address,
            self._calibration,
        )

    def disable(self) -> None:
        """Disable the INA226."""
        if not self._enabled:
            return

        # Power down mode
        if self._bus is not None:
            self._write_register(INA226_REG_CONFIG, 0x0000)

        self._enabled = False
        logger.info("INA226 disabled: addr=0x%02X", self._config.address)

    def read_voltage(self) -> float:
        """Read bus voltage.

        Returns:
            Bus voltage in V.
        """
        if not self._enabled:
            raise RuntimeError("INA226 is not enabled")

        if self._bus is not None:
            raw = self._read_register(INA226_REG_BUS_VOLTAGE)
            # LSB is 1.25mV
            return raw * 0.00125
        else:
            return self._simulated_voltage

    def read_shunt_voltage(self) -> float:
        """Read shunt voltage.

        Returns:
            Shunt voltage in mV.
        """
        if not self._enabled:
            raise RuntimeError("INA226 is not enabled")

        if self._bus is not None:
            raw = self._read_register_signed(INA226_REG_SHUNT_VOLTAGE)
            # LSB is 2.5μV
            return raw * 0.0025
        else:
            return self._simulated_current * self._config.shunt_ohms * 1000

    def read_current(self) -> float:
        """Read current.

        Returns:
            Current in A (positive = into load).
        """
        if not self._enabled:
            raise RuntimeError("INA226 is not enabled")

        if self._bus is not None:
            raw = self._read_register_signed(INA226_REG_CURRENT)
            return raw * self._current_lsb
        else:
            return self._simulated_current

    def read_power(self) -> float:
        """Read power.

        Returns:
            Power in W.
        """
        if not self._enabled:
            raise RuntimeError("INA226 is not enabled")

        if self._bus is not None:
            raw = self._read_register(INA226_REG_POWER)
            return raw * self._power_lsb
        else:
            return self._simulated_voltage * self._simulated_current

    def read_all(self) -> PowerReading:
        """Read all values.

        Returns:
            PowerReading with voltage, current, power, and shunt voltage.
        """
        return PowerReading(
            voltage=self.read_voltage(),
            current=self.read_current(),
            power=self.read_power(),
            shunt_voltage=self.read_shunt_voltage(),
            timestamp=time.time(),
        )

    def get_manufacturer_id(self) -> int:
        """Read manufacturer ID register.

        Returns:
            Manufacturer ID (should be 0x5449 for TI).
        """
        if self._bus is not None:
            return self._read_register(INA226_REG_MANUFACTURER_ID)
        return 0x5449  # Simulated TI ID

    def get_die_id(self) -> int:
        """Read die ID register.

        Returns:
            Die ID (should be 0x2260 for INA226).
        """
        if self._bus is not None:
            return self._read_register(INA226_REG_DIE_ID)
        return 0x2260  # Simulated INA226 ID

    def set_simulated_values(self, voltage: float, current: float) -> None:
        """Set simulated values for testing.

        Args:
            voltage: Simulated bus voltage.
            current: Simulated current.
        """
        self._simulated_voltage = voltage
        self._simulated_current = current

    def _calculate_calibration(self) -> None:
        """Calculate calibration register value."""
        # Calculate current LSB = max_current / 2^15
        self._current_lsb = self._config.max_expected_current / 32768

        # Calculate calibration = 0.00512 / (current_lsb * shunt)
        self._calibration = int(0.00512 / (self._current_lsb * self._config.shunt_ohms))

        # Clamp to 16-bit
        self._calibration = min(0xFFFF, self._calibration)

        # Power LSB = 25 * current_lsb
        self._power_lsb = 25 * self._current_lsb

        logger.debug(
            "INA226 calibration: cal=%d, current_lsb=%.6f, power_lsb=%.6f",
            self._calibration,
            self._current_lsb,
            self._power_lsb,
        )

    def _build_config_register(self) -> int:
        """Build configuration register value."""
        config = 0

        # Averaging mode (bits 9-11)
        config |= (self._config.averaging_mode & 0x07) << 9

        # Bus conversion time (bits 6-8)
        config |= (self._config.bus_conversion_time & 0x07) << 6

        # Shunt conversion time (bits 3-5)
        config |= (self._config.shunt_conversion_time & 0x07) << 3

        # Operating mode: continuous shunt and bus (bits 0-2 = 0x7)
        config |= 0x07

        return config

    def _configure_alerts(self) -> None:
        """Configure alert thresholds."""
        # For now, alerts are not implemented
        # Would need to write to MASK_ENABLE and ALERT_LIMIT registers
        pass

    def _write_register(self, reg: int, value: int) -> None:
        """Write 16-bit register."""
        if self._bus is not None:
            data = struct.pack(">H", value)
            self._bus.write(self._config.address, bytes([reg]) + data)

    def _read_register(self, reg: int) -> int:
        """Read 16-bit unsigned register."""
        if self._bus is not None:
            self._bus.write(self._config.address, bytes([reg]))
            data = self._bus.read(self._config.address, 2)
            return struct.unpack(">H", data)[0]
        return 0

    def _read_register_signed(self, reg: int) -> int:
        """Read 16-bit signed register."""
        if self._bus is not None:
            self._bus.write(self._config.address, bytes([reg]))
            data = self._bus.read(self._config.address, 2)
            return struct.unpack(">h", data)[0]
        return 0


# =============================================================================
# Simulated Power Monitor
# =============================================================================


class SimulatedPowerMonitor(PowerMonitorDriver):
    """Simulated power monitor for testing.

    Provides configurable simulated readings without hardware.
    """

    def __init__(
        self,
        voltage: float = 12.0,
        current: float = 0.5,
        name: str = "simulated_power_monitor",
    ) -> None:
        """Initialize simulated power monitor.

        Args:
            voltage: Initial simulated voltage.
            current: Initial simulated current.
            name: Monitor name.
        """
        self._voltage = voltage
        self._current = current
        self._name = name
        self._enabled = False

        logger.debug("SimulatedPowerMonitor initialized: %s", name)

    @property
    def name(self) -> str:
        """Get monitor name."""
        return self._name

    def is_enabled(self) -> bool:
        """Check if enabled."""
        return self._enabled

    def enable(self) -> None:
        """Enable monitor."""
        self._enabled = True
        logger.info("SimulatedPowerMonitor '%s' enabled", self._name)

    def disable(self) -> None:
        """Disable monitor."""
        self._enabled = False
        logger.info("SimulatedPowerMonitor '%s' disabled", self._name)

    def read_voltage(self) -> float:
        """Read simulated voltage."""
        if not self._enabled:
            raise RuntimeError("SimulatedPowerMonitor is not enabled")
        return self._voltage

    def read_current(self) -> float:
        """Read simulated current."""
        if not self._enabled:
            raise RuntimeError("SimulatedPowerMonitor is not enabled")
        return self._current

    def read_power(self) -> float:
        """Read simulated power."""
        if not self._enabled:
            raise RuntimeError("SimulatedPowerMonitor is not enabled")
        return self._voltage * self._current

    def set_values(self, voltage: float, current: float) -> None:
        """Set simulated values.

        Args:
            voltage: Simulated voltage.
            current: Simulated current.
        """
        self._voltage = voltage
        self._current = current


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Constants
    "INA219_DEFAULT_ADDRESS",
    "INA226_DEFAULT_ADDRESS",
    "INA219ADCResolution",
    "INA219BusVoltageRange",
    # INA219
    "INA219Config",
    "INA219Driver",
    "INA219Gain",
    "INA226AveragingMode",
    # INA226
    "INA226Config",
    "INA226ConversionTime",
    "INA226Driver",
    # Base class
    "PowerMonitorDriver",
    # Data class
    "PowerReading",
    # Simulated
    "SimulatedPowerMonitor",
]
