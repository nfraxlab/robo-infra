"""Electrical sensor implementations.

Phase 4.6 provides common electrical sensors:
- CurrentSensor (shunt resistor, hall effect)
- VoltageSensor (direct, voltage divider)
- PowerSensor (combined current + voltage)

All sensors extend `Sensor` and return a `Reading` with appropriate units.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core import Reading
from robo_infra.core.sensor import Sensor
from robo_infra.core.types import Limits, Unit


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import AnalogPin


# =============================================================================
# Current Sensor
# =============================================================================


class CurrentSensorType(Enum):
    """Type of current sensor."""

    SHUNT = "shunt"  # Shunt resistor (e.g., INA219)
    HALL_EFFECT = "hall_effect"  # Hall effect (e.g., ACS712)
    I2C = "i2c"  # I2C power monitor (e.g., INA219, INA260)


class CurrentSensorConfig(BaseModel):
    """Configuration for current sensors."""

    name: str = "Current"
    sensor_type: CurrentSensorType = CurrentSensorType.HALL_EFFECT
    unit: Unit = Unit.AMPS

    # ADC parameters
    adc_resolution: int = 4095  # 12-bit ADC
    reference_voltage: float = 3.3

    # Hall effect sensor parameters (e.g., ACS712)
    # Sensitivity in mV/A (ACS712-05B: 185, ACS712-20A: 100, ACS712-30A: 66)
    sensitivity_mv_per_amp: float = 100.0
    zero_current_voltage: float = 2.5  # Voltage at 0A (typically Vcc/2)

    # Shunt resistor parameters
    shunt_resistance: float = 0.1  # ohms

    # Current range
    max_current: float = 30.0  # Amps
    min_current: float = -30.0  # Amps (negative for bidirectional)

    # Calibration
    offset: float = 0.0
    scale: float = 1.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class CurrentSensorStatus:
    """Runtime status for current sensor."""

    last_raw: int | None = None
    last_voltage: float | None = None
    last_current: float | None = None
    readings_count: int = 0


class CurrentSensor(Sensor):
    """Current sensor.

    Measures electrical current using shunt resistor or hall effect sensor.

    Supported sensor types:
    - Hall effect (ACS712, ACS758): Measures current via magnetic field
    - Shunt resistor (INA219): Measures voltage drop across known resistance
    - I2C power monitors: Digital current sensors

    Example:
        ```python
        pin = AnalogPin(0)
        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,  # ACS712-20A
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        print(f"Current: {reading.value:.2f} A")
        ```
    """

    def __init__(
        self,
        *,
        pin: AnalogPin | None = None,
        bus: I2CBus | None = None,
        address: int = 0x40,  # INA219 default
        driver: Driver | None = None,
        config: CurrentSensorConfig | None = None,
    ) -> None:
        """Initialize current sensor.

        Args:
            pin: Analog pin for hall effect or shunt voltage
            bus: I2C bus for I2C sensors
            address: I2C address (default 0x40 for INA219)
            driver: Optional driver for hardware abstraction
            config: Sensor configuration
        """
        self._pin = pin
        self._bus = bus
        self._address = address
        self._driver = driver
        self._config = config or CurrentSensorConfig()
        self._status = CurrentSensorStatus()

        limits = Limits(
            min=self._config.min_current,
            max=self._config.max_current,
        )

        super().__init__(
            name=self._config.name,
            unit=self._config.unit,
            limits=limits,
        )

    @property
    def config(self) -> CurrentSensorConfig:
        """Get sensor configuration."""
        return self._config

    @property
    def status(self) -> CurrentSensorStatus:  # type: ignore[override]
        """Get sensor status."""
        return self._status

    def _read_raw(self) -> int:
        """Read raw sensor value."""
        if self._driver is not None:
            return int(self._driver.get_channel(0))

        if self._pin is not None:
            raw = self._pin.read_raw()
            self._status.last_raw = raw
            return raw

        if self._bus is not None:
            # Read current register from I2C sensor (e.g., INA219)
            # Register 0x04 is typically the current register
            data = self._bus.read(self._address, 2)
            raw = (data[0] << 8) | data[1]
            # Handle signed value
            if raw & 0x8000:
                raw -= 0x10000
            self._status.last_raw = raw
            return raw

        return 0

    def read(self) -> Reading:
        """Read current with proper conversion.

        Returns:
            Reading with current in configured unit.
        """
        if not self._is_enabled:
            from robo_infra.core.exceptions import DisabledError

            raise DisabledError(self._name)

        raw = self._read_raw()
        value = self._convert_value(raw)

        # Clamp to limits
        value = max(self._limits.min, min(self._limits.max, value))

        return Reading(
            value=value,
            unit=self._unit,
            timestamp=time.time(),
            raw=raw,
        )

    def _convert_value(self, raw: int) -> float:
        """Convert raw reading to current in Amps."""
        sensor_type = self._config.sensor_type

        if sensor_type == CurrentSensorType.HALL_EFFECT:
            current = self._hall_effect_to_current(raw)
        elif sensor_type == CurrentSensorType.SHUNT:
            current = self._shunt_to_current(raw)
        elif sensor_type == CurrentSensorType.I2C:
            # INA219: Current LSB typically configured for specific resolution
            # Default: raw value in mA
            current = raw / 1000.0
        else:
            current = float(raw)

        # Apply calibration
        current = current * self._config.scale + self._config.offset

        self._status.last_current = current
        self._status.readings_count += 1

        return current

    def _hall_effect_to_current(self, raw: int) -> float:
        """Convert hall effect sensor reading to current.

        Hall effect sensors output a voltage proportional to current,
        centered around Vcc/2 for bidirectional measurement.
        """
        # Convert raw ADC to voltage
        voltage = raw / self._config.adc_resolution * self._config.reference_voltage
        self._status.last_voltage = voltage

        # Calculate current from voltage difference
        voltage_diff = voltage - self._config.zero_current_voltage

        # Convert mV/A to V/A
        sensitivity_v_per_amp = self._config.sensitivity_mv_per_amp / 1000.0

        if sensitivity_v_per_amp == 0:
            return 0.0

        current = voltage_diff / sensitivity_v_per_amp
        return current

    def _shunt_to_current(self, raw: int) -> float:
        """Convert shunt resistor voltage reading to current.

        Uses Ohm's law: I = V / R
        """
        # Convert raw ADC to voltage across shunt
        voltage = raw / self._config.adc_resolution * self._config.reference_voltage
        self._status.last_voltage = voltage

        if self._config.shunt_resistance == 0:
            return 0.0

        current = voltage / self._config.shunt_resistance
        return current

    def read_milliamps(self) -> float:
        """Read current in milliamps."""
        reading = self.read()
        return reading.value * 1000.0


# =============================================================================
# Voltage Sensor
# =============================================================================


class VoltageSensorType(Enum):
    """Type of voltage sensor."""

    DIRECT = "direct"  # Direct ADC measurement
    DIVIDER = "divider"  # Voltage divider
    I2C = "i2c"  # I2C sensor (e.g., INA219)


class VoltageSensorConfig(BaseModel):
    """Configuration for voltage sensors."""

    name: str = "Voltage"
    sensor_type: VoltageSensorType = VoltageSensorType.DIVIDER
    unit: Unit = Unit.VOLTS

    # ADC parameters
    adc_resolution: int = 4095  # 12-bit ADC
    reference_voltage: float = 3.3

    # Voltage divider parameters: ratio = (R1 + R2) / R2
    r1: float = 30000.0  # Top resistor (ohms)
    r2: float = 7500.0  # Bottom resistor (ohms)

    # Voltage range
    max_voltage: float = 25.0  # Volts
    min_voltage: float = 0.0  # Volts

    # Calibration
    offset: float = 0.0
    scale: float = 1.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class VoltageSensorStatus:
    """Runtime status for voltage sensor."""

    last_raw: int | None = None
    last_adc_voltage: float | None = None
    last_voltage: float | None = None
    readings_count: int = 0


class VoltageSensor(Sensor):
    """Voltage sensor.

    Measures electrical voltage directly or through a voltage divider.

    Voltage Divider:
        For measuring voltages higher than the ADC reference, use a
        voltage divider with R1 (top) and R2 (bottom) resistors.
        Vin = Vadc * (R1 + R2) / R2

    Example:
        ```python
        pin = AnalogPin(0)
        config = VoltageSensorConfig(
            sensor_type=VoltageSensorType.DIVIDER,
            r1=30000.0,  # 30k top resistor
            r2=7500.0,   # 7.5k bottom resistor
            # Allows measuring 0-16.5V with 3.3V ADC
        )
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        print(f"Voltage: {reading.value:.2f} V")
        ```
    """

    def __init__(
        self,
        *,
        pin: AnalogPin | None = None,
        bus: I2CBus | None = None,
        address: int = 0x40,  # INA219 default
        driver: Driver | None = None,
        config: VoltageSensorConfig | None = None,
    ) -> None:
        """Initialize voltage sensor.

        Args:
            pin: Analog pin for voltage measurement
            bus: I2C bus for I2C sensors
            address: I2C address
            driver: Optional driver for hardware abstraction
            config: Sensor configuration
        """
        self._pin = pin
        self._bus = bus
        self._address = address
        self._driver = driver
        self._config = config or VoltageSensorConfig()
        self._status = VoltageSensorStatus()

        limits = Limits(
            min=self._config.min_voltage,
            max=self._config.max_voltage,
        )

        super().__init__(
            name=self._config.name,
            unit=self._config.unit,
            limits=limits,
        )

    @property
    def config(self) -> VoltageSensorConfig:
        """Get sensor configuration."""
        return self._config

    @property
    def status(self) -> VoltageSensorStatus:  # type: ignore[override]
        """Get sensor status."""
        return self._status

    @property
    def divider_ratio(self) -> float:
        """Get voltage divider ratio (Vin/Vadc)."""
        if self._config.r2 == 0:
            return 1.0
        return (self._config.r1 + self._config.r2) / self._config.r2

    def _read_raw(self) -> int:
        """Read raw sensor value."""
        if self._driver is not None:
            return int(self._driver.get_channel(0))

        if self._pin is not None:
            raw = self._pin.read_raw()
            self._status.last_raw = raw
            return raw

        if self._bus is not None:
            # Read voltage register from I2C sensor
            # INA219: Register 0x02 is bus voltage
            data = self._bus.read(self._address, 2)
            raw = (data[0] << 8) | data[1]
            raw >>= 3  # INA219 bus voltage is 13-bit, right-aligned
            self._status.last_raw = raw
            return raw

        return 0

    def read(self) -> Reading:
        """Read voltage with proper conversion.

        Returns:
            Reading with voltage in configured unit.
        """
        if not self._is_enabled:
            from robo_infra.core.exceptions import DisabledError

            raise DisabledError(self._name)

        raw = self._read_raw()
        value = self._convert_value(raw)

        # Clamp to limits
        value = max(self._limits.min, min(self._limits.max, value))

        return Reading(
            value=value,
            unit=self._unit,
            timestamp=time.time(),
            raw=raw,
        )

    def _convert_value(self, raw: int) -> float:
        """Convert raw reading to voltage."""
        sensor_type = self._config.sensor_type

        if sensor_type == VoltageSensorType.DIRECT:
            voltage = self._direct_to_voltage(raw)
        elif sensor_type == VoltageSensorType.DIVIDER:
            voltage = self._divider_to_voltage(raw)
        elif sensor_type == VoltageSensorType.I2C:
            # INA219: Bus voltage LSB = 4mV
            voltage = raw * 0.004
        else:
            voltage = float(raw)

        # Apply calibration
        voltage = voltage * self._config.scale + self._config.offset

        self._status.last_voltage = voltage
        self._status.readings_count += 1

        return voltage

    def _direct_to_voltage(self, raw: int) -> float:
        """Convert direct ADC reading to voltage."""
        adc_voltage = raw / self._config.adc_resolution * self._config.reference_voltage
        self._status.last_adc_voltage = adc_voltage
        return adc_voltage

    def _divider_to_voltage(self, raw: int) -> float:
        """Convert voltage divider reading to actual voltage."""
        adc_voltage = raw / self._config.adc_resolution * self._config.reference_voltage
        self._status.last_adc_voltage = adc_voltage

        # Apply divider ratio
        voltage = adc_voltage * self.divider_ratio
        return voltage

    def read_millivolts(self) -> float:
        """Read voltage in millivolts."""
        reading = self.read()
        return reading.value * 1000.0


# =============================================================================
# Power Sensor (Combined Current + Voltage)
# =============================================================================


class PowerSensorConfig(BaseModel):
    """Configuration for power sensors."""

    name: str = "Power"
    unit: Unit = Unit.WATTS

    # I2C parameters (for integrated power monitors)
    address: int = 0x40  # INA219 default

    # Current sense parameters (for INA219)
    shunt_resistance: float = 0.1  # ohms
    max_expected_current: float = 3.2  # Amps

    # Calculated calibration value
    current_lsb: float = 0.0001  # 0.1mA per bit

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class PowerSensorStatus:
    """Runtime status for power sensor."""

    last_raw: int | None = None
    last_voltage: float | None = None
    last_current: float | None = None
    last_power: float | None = None
    readings_count: int = 0


class PowerSensor(Sensor):
    """Power sensor for measuring voltage, current, and power.

    Typically uses integrated I2C power monitors like INA219 or INA260
    that can measure all three values simultaneously.

    Example:
        ```python
        bus = I2CBus(1)
        config = PowerSensorConfig(
            shunt_resistance=0.1,
            max_expected_current=3.2,
        )
        sensor = PowerSensor(bus=bus, config=config)
        sensor.enable()

        reading = sensor.read()
        print(f"Power: {reading.value:.2f} W")
        print(f"Voltage: {sensor.read_voltage():.2f} V")
        print(f"Current: {sensor.read_current():.2f} A")
        ```
    """

    def __init__(
        self,
        *,
        bus: I2CBus | None = None,
        driver: Driver | None = None,
        config: PowerSensorConfig | None = None,
    ) -> None:
        """Initialize power sensor.

        Args:
            bus: I2C bus for sensor
            driver: Optional driver for hardware abstraction
            config: Sensor configuration
        """
        self._bus = bus
        self._driver = driver
        self._config = config or PowerSensorConfig()
        self._status = PowerSensorStatus()

        # Power limits based on voltage and current
        max_power = 26.0 * self._config.max_expected_current  # Assuming max 26V

        limits = Limits(min=0.0, max=max_power)

        super().__init__(
            name=self._config.name,
            unit=self._config.unit,
            limits=limits,
        )

    @property
    def config(self) -> PowerSensorConfig:
        """Get sensor configuration."""
        return self._config

    @property
    def status(self) -> PowerSensorStatus:  # type: ignore[override]
        """Get sensor status."""
        return self._status

    def _read_raw(self) -> int:
        """Read raw power value."""
        if self._driver is not None:
            return int(self._driver.get_channel(0))

        if self._bus is not None:
            # INA219: Power register is 0x03
            data = self._bus.read(self._config.address, 2)
            raw = (data[0] << 8) | data[1]
            self._status.last_raw = raw
            return raw

        return 0

    def read(self) -> Reading:
        """Read power measurement.

        Returns:
            Reading with power in Watts.
        """
        if not self._is_enabled:
            from robo_infra.core.exceptions import DisabledError

            raise DisabledError(self._name)

        raw = self._read_raw()
        value = self._convert_value(raw)

        # Clamp to limits
        value = max(self._limits.min, min(self._limits.max, value))

        return Reading(
            value=value,
            unit=self._unit,
            timestamp=time.time(),
            raw=raw,
        )

    def _convert_value(self, raw: int) -> float:
        """Convert raw reading to power in Watts."""
        # INA219: Power LSB = 20 * Current LSB
        power_lsb = 20 * self._config.current_lsb
        power = raw * power_lsb

        self._status.last_power = power
        self._status.readings_count += 1

        return power

    def read_voltage(self) -> float:
        """Read bus voltage in Volts."""
        if self._bus is None:
            return 0.0

        # INA219: Bus voltage register is 0x02
        # Need to select register first, then read
        data = self._bus.read(self._config.address, 2)
        raw = (data[0] << 8) | data[1]
        raw >>= 3  # 13-bit value

        voltage = raw * 0.004  # 4mV per bit
        self._status.last_voltage = voltage
        return voltage

    def read_current(self) -> float:
        """Read current in Amps."""
        if self._bus is None:
            return 0.0

        # INA219: Current register is 0x04
        data = self._bus.read(self._config.address, 2)
        raw = (data[0] << 8) | data[1]

        # Handle signed value
        if raw & 0x8000:
            raw -= 0x10000

        current = raw * self._config.current_lsb
        self._status.last_current = current
        return current

    def read_all(self) -> tuple[float, float, float]:
        """Read voltage, current, and power.

        Returns:
            Tuple of (voltage, current, power) in (V, A, W).
        """
        voltage = self.read_voltage()
        current = self.read_current()
        power = self.read().value
        return (voltage, current, power)
