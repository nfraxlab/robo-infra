"""Environmental sensor implementations.

Phase 4.5 provides common environmental sensors:
- Temperature (thermistor, DS18B20, thermocouple)
- Humidity (capacitive, resistive)
- Pressure (barometric)
- Light (ambient light, lux meter)

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
    from robo_infra.core.pin import AnalogPin, DigitalPin


# =============================================================================
# Temperature Sensor
# =============================================================================


class TemperatureSensorType(Enum):
    """Type of temperature sensor."""

    THERMISTOR = "thermistor"
    DS18B20 = "ds18b20"
    THERMOCOUPLE = "thermocouple"
    I2C = "i2c"  # Generic I2C (e.g., TMP102, LM75)


class TemperatureConfig(BaseModel):
    """Configuration for temperature sensors."""

    name: str = "Temperature"
    sensor_type: TemperatureSensorType = TemperatureSensorType.THERMISTOR
    unit: Unit = Unit.CELSIUS

    # Thermistor parameters (Steinhart-Hart)
    nominal_resistance: float = 10000.0  # ohms at 25°C
    nominal_temp: float = 25.0  # °C
    beta_coefficient: float = 3950.0  # B coefficient
    series_resistor: float = 10000.0  # ohms

    # ADC parameters
    adc_resolution: int = 4095  # 12-bit ADC

    # Limits
    min_temp: float = -40.0
    max_temp: float = 125.0

    # Calibration offset
    offset: float = 0.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class TemperatureStatus:
    """Runtime status for temperature sensor."""

    last_raw: int | None = None
    last_celsius: float | None = None
    readings_count: int = 0


class Temperature(Sensor):
    """Temperature sensor.

    Supports multiple sensor types:
    - Thermistor: Uses Steinhart-Hart equation for NTC thermistors
    - DS18B20: 1-Wire digital sensor (via driver)
    - Thermocouple: Via analog or dedicated IC
    - I2C: Generic I2C temperature sensors

    Example:
        ```python
        pin = AnalogPin(0)
        config = TemperatureConfig(
            sensor_type=TemperatureSensorType.THERMISTOR,
            beta_coefficient=3950,
        )
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        print(f"Temperature: {reading.value:.1f}°C")
        ```
    """

    def __init__(
        self,
        *,
        pin: AnalogPin | DigitalPin | None = None,
        bus: I2CBus | None = None,
        address: int = 0x48,  # Common I2C address
        driver: Driver | None = None,
        config: TemperatureConfig | None = None,
    ) -> None:
        """Initialize temperature sensor.

        Args:
            pin: Analog pin for thermistor/thermocouple, digital for DS18B20
            bus: I2C bus for I2C sensors
            address: I2C address (default 0x48)
            driver: Optional driver for hardware abstraction
            config: Sensor configuration
        """
        self._pin = pin
        self._bus = bus
        self._address = address
        self._driver = driver
        self._config = config or TemperatureConfig()
        self._status = TemperatureStatus()

        # Limits based on config
        limits = Limits(
            min=self._config.min_temp,
            max=self._config.max_temp,
        )

        super().__init__(
            name=self._config.name,
            unit=self._config.unit,
            limits=limits,
        )

    @property
    def config(self) -> TemperatureConfig:
        """Get sensor configuration."""
        return self._config

    @property
    def status(self) -> TemperatureStatus:  # type: ignore[override]
        """Get sensor status."""
        return self._status

    def _read_raw(self) -> int:
        """Read raw ADC value or digital reading."""
        if self._driver is not None:
            return int(self._driver.get_channel(0))

        if self._pin is not None:
            # For analog pins (thermistor), digital pins need driver
            raw = self._pin.read_raw() if hasattr(self._pin, "read_raw") else 0
            self._status.last_raw = raw
            return raw

        if self._bus is not None:
            # Read 2 bytes from I2C sensor
            data = self._bus.read(self._address, 2)
            raw = (data[0] << 8) | data[1]
            self._status.last_raw = raw
            return raw

        return 0

    def read(self) -> Reading:
        """Read temperature with proper conversion.

        Overrides base Sensor.read() to apply non-linear conversion.

        Returns:
            Reading with temperature in configured unit.
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
        """Convert raw reading to temperature in configured unit."""
        sensor_type = self._config.sensor_type

        if sensor_type == TemperatureSensorType.THERMISTOR:
            celsius = self._thermistor_to_celsius(raw)
        elif sensor_type == TemperatureSensorType.DS18B20:
            # DS18B20 returns 12-bit value, 0.0625°C per bit
            celsius = raw * 0.0625
        elif sensor_type == TemperatureSensorType.THERMOCOUPLE:
            # Simplified: assume linear conversion from millivolts
            # Real implementation would use lookup tables
            celsius = raw * 0.25
        elif sensor_type == TemperatureSensorType.I2C:
            # Common format: 12-bit, 0.0625°C per LSB
            celsius = raw / 256.0
        else:
            celsius = float(raw)

        # Apply calibration offset
        celsius += self._config.offset

        self._status.last_celsius = celsius
        self._status.readings_count += 1

        # Convert to target unit
        return self._convert_unit(celsius)

    def _thermistor_to_celsius(self, raw: int) -> float:
        """Convert thermistor ADC reading to Celsius using Steinhart-Hart."""
        if raw <= 0:
            return self._config.min_temp
        if raw >= self._config.adc_resolution:
            return self._config.max_temp

        # Calculate resistance from voltage divider
        # Assuming thermistor is connected to ground
        resistance = self._config.series_resistor * raw / (self._config.adc_resolution - raw)

        if resistance <= 0:
            return self._config.min_temp

        # Simplified Steinhart-Hart (B-parameter equation)
        import math

        steinhart = math.log(resistance / self._config.nominal_resistance)
        steinhart /= self._config.beta_coefficient
        steinhart += 1.0 / (self._config.nominal_temp + 273.15)

        if steinhart <= 0:
            return self._config.max_temp

        celsius = 1.0 / steinhart - 273.15
        return celsius

    def _convert_unit(self, celsius: float) -> float:
        """Convert Celsius to configured unit."""
        if self._config.unit == Unit.CELSIUS:
            return celsius
        elif self._config.unit == Unit.FAHRENHEIT:
            return celsius * 9.0 / 5.0 + 32.0
        elif self._config.unit == Unit.KELVIN:
            return celsius + 273.15
        return celsius

    def read_celsius(self) -> float:
        """Read temperature in Celsius."""
        reading = self.read()
        # Convert back to Celsius if needed
        if self._config.unit == Unit.FAHRENHEIT:
            return (reading.value - 32.0) * 5.0 / 9.0
        elif self._config.unit == Unit.KELVIN:
            return reading.value - 273.15
        return reading.value

    def read_fahrenheit(self) -> float:
        """Read temperature in Fahrenheit."""
        celsius = self.read_celsius()
        return celsius * 9.0 / 5.0 + 32.0

    def read_kelvin(self) -> float:
        """Read temperature in Kelvin."""
        celsius = self.read_celsius()
        return celsius + 273.15


# =============================================================================
# Humidity Sensor
# =============================================================================


class HumiditySensorType(Enum):
    """Type of humidity sensor."""

    CAPACITIVE = "capacitive"
    RESISTIVE = "resistive"
    I2C = "i2c"  # e.g., DHT22, SHT31, BME280


class HumidityConfig(BaseModel):
    """Configuration for humidity sensors."""

    name: str = "Humidity"
    sensor_type: HumiditySensorType = HumiditySensorType.I2C
    unit: Unit = Unit.PERCENT

    # ADC parameters for analog sensors
    adc_resolution: int = 4095
    min_voltage: float = 0.0  # Voltage at 0% RH
    max_voltage: float = 3.3  # Voltage at 100% RH
    reference_voltage: float = 3.3

    # Calibration
    offset: float = 0.0
    scale: float = 1.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class HumidityStatus:
    """Runtime status for humidity sensor."""

    last_raw: int | None = None
    last_percent: float | None = None
    readings_count: int = 0


class Humidity(Sensor):
    """Humidity sensor.

    Measures relative humidity (RH) as a percentage.

    Example:
        ```python
        bus = I2CBus(1)
        config = HumidityConfig(sensor_type=HumiditySensorType.I2C)
        humidity = Humidity(bus=bus, address=0x44, config=config)
        humidity.enable()

        reading = humidity.read()
        print(f"Humidity: {reading.value:.1f}%")
        ```
    """

    def __init__(
        self,
        *,
        pin: AnalogPin | None = None,
        bus: I2CBus | None = None,
        address: int = 0x44,  # SHT31 default
        driver: Driver | None = None,
        config: HumidityConfig | None = None,
    ) -> None:
        """Initialize humidity sensor."""
        self._pin = pin
        self._bus = bus
        self._address = address
        self._driver = driver
        self._config = config or HumidityConfig()
        self._status = HumidityStatus()

        limits = Limits(min=0.0, max=100.0)

        super().__init__(
            name=self._config.name,
            unit=self._config.unit,
            limits=limits,
        )

    @property
    def config(self) -> HumidityConfig:
        """Get sensor configuration."""
        return self._config

    @property
    def status(self) -> HumidityStatus:  # type: ignore[override]
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
            # Read from I2C sensor
            data = self._bus.read(self._address, 2)
            raw = (data[0] << 8) | data[1]
            self._status.last_raw = raw
            return raw

        return 0

    def read(self) -> Reading:
        """Read humidity with proper conversion.

        Returns:
            Reading with humidity percentage.
        """
        if not self._is_enabled:
            from robo_infra.core.exceptions import DisabledError

            raise DisabledError(self._name)

        raw = self._read_raw()
        value = self._convert_value(raw)

        return Reading(
            value=value,
            unit=self._unit,
            timestamp=time.time(),
            raw=raw,
        )

    def _convert_value(self, raw: int) -> float:
        """Convert raw reading to humidity percentage."""
        sensor_type = self._config.sensor_type

        if sensor_type == HumiditySensorType.I2C:
            # Common I2C format: 16-bit, 0-100%
            percent = raw / 65535.0 * 100.0
        else:
            # Analog sensor: linear mapping
            voltage = raw / self._config.adc_resolution * self._config.reference_voltage
            voltage_range = self._config.max_voltage - self._config.min_voltage
            if voltage_range > 0:
                percent = (voltage - self._config.min_voltage) / voltage_range * 100.0
            else:
                percent = 0.0

        # Apply calibration
        percent = percent * self._config.scale + self._config.offset

        # Clamp to valid range
        percent = max(0.0, min(100.0, percent))

        self._status.last_percent = percent
        self._status.readings_count += 1

        return percent


# =============================================================================
# Pressure Sensor
# =============================================================================


class PressureSensorType(Enum):
    """Type of pressure sensor."""

    BAROMETRIC = "barometric"  # e.g., BMP280, BME280
    DIFFERENTIAL = "differential"
    GAUGE = "gauge"
    ABSOLUTE = "absolute"


class PressureConfig(BaseModel):
    """Configuration for pressure sensors."""

    name: str = "Pressure"
    sensor_type: PressureSensorType = PressureSensorType.BAROMETRIC
    unit: Unit = Unit.HECTOPASCALS

    # Range for barometric sensors
    min_pressure: float = 300.0  # hPa
    max_pressure: float = 1100.0  # hPa

    # Calibration
    offset: float = 0.0
    scale: float = 1.0

    # For altitude calculation
    sea_level_pressure: float = 1013.25  # hPa

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class PressureStatus:
    """Runtime status for pressure sensor."""

    last_raw: int | None = None
    last_pressure: float | None = None
    readings_count: int = 0


class Pressure(Sensor):
    """Pressure sensor.

    Measures atmospheric or differential pressure.

    Example:
        ```python
        bus = I2CBus(1)
        config = PressureConfig(sensor_type=PressureSensorType.BAROMETRIC)
        pressure = Pressure(bus=bus, address=0x76, config=config)
        pressure.enable()

        reading = pressure.read()
        print(f"Pressure: {reading.value:.1f} hPa")
        altitude = pressure.estimate_altitude()
        print(f"Altitude: {altitude:.0f} m")
        ```
    """

    def __init__(
        self,
        *,
        pin: AnalogPin | None = None,
        bus: I2CBus | None = None,
        address: int = 0x76,  # BMP280/BME280 default
        driver: Driver | None = None,
        config: PressureConfig | None = None,
    ) -> None:
        """Initialize pressure sensor."""
        self._pin = pin
        self._bus = bus
        self._address = address
        self._driver = driver
        self._config = config or PressureConfig()
        self._status = PressureStatus()

        limits = Limits(
            min=self._config.min_pressure,
            max=self._config.max_pressure,
        )

        super().__init__(
            name=self._config.name,
            unit=self._config.unit,
            limits=limits,
        )

    @property
    def config(self) -> PressureConfig:
        """Get sensor configuration."""
        return self._config

    @property
    def status(self) -> PressureStatus:  # type: ignore[override]
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
            # Read 3 bytes for 20-bit pressure (common format)
            data = self._bus.read(self._address, 3)
            raw = (data[0] << 16) | (data[1] << 8) | data[2]
            raw >>= 4  # 20-bit value
            self._status.last_raw = raw
            return raw

        return 0

    def read(self) -> Reading:
        """Read pressure with proper conversion.

        Returns:
            Reading with pressure in configured unit.
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
        """Convert raw reading to pressure in configured unit."""
        # Common conversion: raw to hPa
        # This is sensor-specific; using BMP280-like conversion
        pressure_hpa = raw / 256.0  # Simplified

        # Apply calibration
        pressure_hpa = pressure_hpa * self._config.scale + self._config.offset

        self._status.last_pressure = pressure_hpa
        self._status.readings_count += 1

        # Convert to target unit
        return self._convert_unit(pressure_hpa)

    def _convert_unit(self, hpa: float) -> float:
        """Convert hPa to configured unit."""
        if self._config.unit == Unit.HECTOPASCALS:
            return hpa
        elif self._config.unit == Unit.PASCALS:
            return hpa * 100.0
        elif self._config.unit == Unit.KILOPASCALS:
            return hpa / 10.0
        elif self._config.unit == Unit.BAR:
            return hpa / 1000.0
        elif self._config.unit == Unit.PSI:
            return hpa * 0.0145038
        elif self._config.unit == Unit.ATM:
            return hpa / 1013.25
        return hpa

    def read_hpa(self) -> float:
        """Read pressure in hectopascals."""
        reading = self.read()
        # Convert back to hPa
        return self._to_hpa(reading.value)

    def _to_hpa(self, value: float) -> float:
        """Convert configured unit back to hPa."""
        if self._config.unit == Unit.HECTOPASCALS:
            return value
        elif self._config.unit == Unit.PASCALS:
            return value / 100.0
        elif self._config.unit == Unit.KILOPASCALS:
            return value * 10.0
        elif self._config.unit == Unit.BAR:
            return value * 1000.0
        elif self._config.unit == Unit.PSI:
            return value / 0.0145038
        elif self._config.unit == Unit.ATM:
            return value * 1013.25
        return value

    def estimate_altitude(self, sea_level_pressure: float | None = None) -> float:
        """Estimate altitude based on pressure.

        Uses the barometric formula:
        altitude = 44330 * (1 - (P/P0)^0.1903)

        Args:
            sea_level_pressure: Reference pressure at sea level (hPa).
                               Defaults to config value or 1013.25 hPa.

        Returns:
            Estimated altitude in meters.
        """
        import math

        p0 = sea_level_pressure or self._config.sea_level_pressure
        p = self.read_hpa()

        if p <= 0 or p0 <= 0:
            return 0.0

        altitude = 44330.0 * (1.0 - math.pow(p / p0, 0.1903))
        return altitude


# =============================================================================
# Light Sensor
# =============================================================================


class LightSensorType(Enum):
    """Type of light sensor."""

    PHOTORESISTOR = "photoresistor"  # LDR
    PHOTODIODE = "photodiode"
    PHOTOTRANSISTOR = "phototransistor"
    I2C = "i2c"  # e.g., BH1750, TSL2561, VEML7700


class LightConfig(BaseModel):
    """Configuration for light sensors."""

    name: str = "Light"
    sensor_type: LightSensorType = LightSensorType.I2C
    unit: Unit = Unit.LUX

    # ADC parameters for analog sensors
    adc_resolution: int = 4095
    reference_voltage: float = 3.3

    # LDR parameters
    ldr_resistance_dark: float = 1_000_000.0  # ohms in darkness
    ldr_resistance_light: float = 100.0  # ohms in bright light
    series_resistor: float = 10000.0  # ohms

    # Calibration
    offset: float = 0.0
    scale: float = 1.0

    # Range
    max_lux: float = 65535.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class LightStatus:
    """Runtime status for light sensor."""

    last_raw: int | None = None
    last_lux: float | None = None
    readings_count: int = 0


class Light(Sensor):
    """Light sensor.

    Measures ambient light level in lux or as a normalized value.

    Example:
        ```python
        bus = I2CBus(1)
        config = LightConfig(sensor_type=LightSensorType.I2C)
        light = Light(bus=bus, address=0x23, config=config)
        light.enable()

        reading = light.read()
        print(f"Light: {reading.value:.0f} lux")
        ```
    """

    def __init__(
        self,
        *,
        pin: AnalogPin | None = None,
        bus: I2CBus | None = None,
        address: int = 0x23,  # BH1750 default
        driver: Driver | None = None,
        config: LightConfig | None = None,
    ) -> None:
        """Initialize light sensor."""
        self._pin = pin
        self._bus = bus
        self._address = address
        self._driver = driver
        self._config = config or LightConfig()
        self._status = LightStatus()

        limits = Limits(min=0.0, max=self._config.max_lux)

        super().__init__(
            name=self._config.name,
            unit=self._config.unit,
            limits=limits,
        )

    @property
    def config(self) -> LightConfig:
        """Get sensor configuration."""
        return self._config

    @property
    def status(self) -> LightStatus:  # type: ignore[override]
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
            # Read 2 bytes from I2C sensor
            data = self._bus.read(self._address, 2)
            raw = (data[0] << 8) | data[1]
            self._status.last_raw = raw
            return raw

        return 0

    def read(self) -> Reading:
        """Read light level with proper conversion.

        Returns:
            Reading with lux value.
        """
        if not self._is_enabled:
            from robo_infra.core.exceptions import DisabledError

            raise DisabledError(self._name)

        raw = self._read_raw()
        value = self._convert_value(raw)

        return Reading(
            value=value,
            unit=self._unit,
            timestamp=time.time(),
            raw=raw,
        )

    def _convert_value(self, raw: int) -> float:
        """Convert raw reading to lux."""
        sensor_type = self._config.sensor_type

        if sensor_type == LightSensorType.I2C:
            # BH1750 format: raw / 1.2 = lux
            lux = raw / 1.2
        elif sensor_type == LightSensorType.PHOTORESISTOR:
            lux = self._ldr_to_lux(raw)
        else:
            # Photodiode/phototransistor: linear with light
            normalized = raw / self._config.adc_resolution
            lux = normalized * self._config.max_lux

        # Apply calibration
        lux = lux * self._config.scale + self._config.offset

        # Clamp to valid range
        lux = max(0.0, min(self._config.max_lux, lux))

        self._status.last_lux = lux
        self._status.readings_count += 1

        return lux

    def _ldr_to_lux(self, raw: int) -> float:
        """Convert LDR reading to approximate lux.

        Uses logarithmic relationship between resistance and lux.
        This is an approximation; real LDRs require calibration.
        """
        import math

        if raw <= 0:
            return self._config.max_lux
        if raw >= self._config.adc_resolution:
            return 0.0

        # Calculate LDR resistance from voltage divider
        # Assuming LDR connected to VCC, series resistor to ground
        voltage = raw / self._config.adc_resolution * self._config.reference_voltage
        if voltage >= self._config.reference_voltage:
            return 0.0

        # LDR resistance
        ldr_resistance = (
            self._config.series_resistor * (self._config.reference_voltage - voltage) / voltage
        )

        # Approximate lux from resistance (logarithmic)
        # lux ≈ 500000 / R (very rough approximation)
        if ldr_resistance <= 0:
            return self._config.max_lux

        # Use gamma correction formula: R = R_ref * (lux / lux_ref)^-gamma
        # Simplified: lux ≈ constant / R^(1/gamma)
        gamma = 0.7  # Typical for LDRs
        lux = 500000.0 * math.pow(ldr_resistance, -gamma)

        return lux

    def read_lux(self) -> float:
        """Read light level in lux."""
        return self.read().value

    def read_normalized(self) -> float:
        """Read light as normalized 0.0-1.0 value."""
        lux = self.read_lux()
        return min(1.0, lux / self._config.max_lux)

    def is_bright(self, threshold: float = 500.0) -> bool:
        """Check if light level is above threshold.

        Args:
            threshold: Lux threshold (default 500 = typical indoor lighting)

        Returns:
            True if current lux is above threshold.
        """
        return self.read_lux() > threshold

    def is_dark(self, threshold: float = 10.0) -> bool:
        """Check if light level is below threshold.

        Args:
            threshold: Lux threshold (default 10 = dim lighting)

        Returns:
            True if current lux is below threshold.
        """
        return self.read_lux() < threshold
