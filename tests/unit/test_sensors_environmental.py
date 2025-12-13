"""Unit tests for environmental sensors (Phase 4.5)."""

from __future__ import annotations

import pytest

from robo_infra.core.pin import SimulatedAnalogPin
from robo_infra.core.types import Unit
from robo_infra.sensors.environmental import (
    Humidity,
    HumidityConfig,
    HumiditySensorType,
    Light,
    LightConfig,
    LightSensorType,
    Pressure,
    PressureConfig,
    PressureSensorType,
    Temperature,
    TemperatureConfig,
    TemperatureSensorType,
)


# =============================================================================
# Temperature Sensor Tests
# =============================================================================


class TestTemperatureConfig:
    """Tests for TemperatureConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = TemperatureConfig()
        assert config.name == "Temperature"
        assert config.sensor_type == TemperatureSensorType.THERMISTOR
        assert config.unit == Unit.CELSIUS
        assert config.nominal_resistance == 10000.0
        assert config.beta_coefficient == 3950.0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = TemperatureConfig(
            name="TempSensor1",
            sensor_type=TemperatureSensorType.DS18B20,
            unit=Unit.FAHRENHEIT,
            offset=0.5,
        )
        assert config.name == "TempSensor1"
        assert config.sensor_type == TemperatureSensorType.DS18B20
        assert config.unit == Unit.FAHRENHEIT
        assert config.offset == 0.5


class TestTemperature:
    """Tests for Temperature sensor."""

    def test_thermistor_creation(self) -> None:
        """Test creating a thermistor-based temperature sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        config = TemperatureConfig(sensor_type=TemperatureSensorType.THERMISTOR)
        temp = Temperature(pin=pin, config=config)

        assert temp.name == "Temperature"
        assert temp.unit == Unit.CELSIUS
        assert temp.is_enabled is False

    def test_thermistor_read(self) -> None:
        """Test reading from thermistor sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # Set to approximately room temperature (25°C)
        # At 25°C, resistance = nominal = 10k, so voltage divider gives ~0.5
        pin.set_simulated_normalized(0.5)

        config = TemperatureConfig(sensor_type=TemperatureSensorType.THERMISTOR)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        # Should be close to nominal temp (25°C)
        assert 20 < reading.value < 30

    def test_read_celsius(self) -> None:
        """Test reading temperature in Celsius."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = TemperatureConfig(unit=Unit.CELSIUS)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        celsius = temp.read_celsius()
        assert isinstance(celsius, float)
        assert 20 < celsius < 30

    def test_read_fahrenheit(self) -> None:
        """Test reading temperature in Fahrenheit."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = TemperatureConfig(unit=Unit.CELSIUS)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        fahrenheit = temp.read_fahrenheit()
        # 25°C ≈ 77°F
        assert 68 < fahrenheit < 86

    def test_read_kelvin(self) -> None:
        """Test reading temperature in Kelvin."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = TemperatureConfig(unit=Unit.CELSIUS)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        kelvin = temp.read_kelvin()
        # 25°C ≈ 298K
        assert 293 < kelvin < 303

    def test_fahrenheit_output_unit(self) -> None:
        """Test sensor configured to output Fahrenheit."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = TemperatureConfig(unit=Unit.FAHRENHEIT)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        assert reading.unit == Unit.FAHRENHEIT
        # Should be in Fahrenheit range
        assert 68 < reading.value < 86

    def test_calibration_offset(self) -> None:
        """Test temperature offset calibration."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        # Without offset
        config1 = TemperatureConfig(offset=0.0)
        temp1 = Temperature(pin=pin, config=config1)
        temp1.enable()
        reading1 = temp1.read()

        # With offset
        config2 = TemperatureConfig(offset=2.0)
        temp2 = Temperature(pin=pin, config=config2)
        temp2.enable()
        reading2 = temp2.read()

        assert reading2.value == pytest.approx(reading1.value + 2.0, abs=0.1)

    def test_status_tracking(self) -> None:
        """Test sensor status tracking."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        temp = Temperature(pin=pin)
        temp.enable()

        assert temp.status.readings_count == 0
        _ = temp.read()
        assert temp.status.readings_count == 1
        assert temp.status.last_raw is not None
        assert temp.status.last_celsius is not None

    def test_no_input_returns_zero(self) -> None:
        """Test sensor with no input source."""
        temp = Temperature()
        temp.enable()

        reading = temp.read()
        # With raw=0, should hit min_temp
        assert reading.value == pytest.approx(-40.0, abs=0.1)


# =============================================================================
# Humidity Sensor Tests
# =============================================================================


class TestHumidityConfig:
    """Tests for HumidityConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = HumidityConfig()
        assert config.name == "Humidity"
        assert config.sensor_type == HumiditySensorType.I2C
        assert config.unit == Unit.PERCENT

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = HumidityConfig(
            name="RH Sensor",
            sensor_type=HumiditySensorType.CAPACITIVE,
            offset=1.5,
            scale=0.98,
        )
        assert config.name == "RH Sensor"
        assert config.sensor_type == HumiditySensorType.CAPACITIVE
        assert config.offset == 1.5
        assert config.scale == 0.98


class TestHumidity:
    """Tests for Humidity sensor."""

    def test_creation(self) -> None:
        """Test creating humidity sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        humidity = Humidity(pin=pin)

        assert humidity.name == "Humidity"
        assert humidity.unit == Unit.PERCENT

    def test_analog_read(self) -> None:
        """Test reading from analog humidity sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # 50% of voltage range
        pin.set_simulated_normalized(0.5)

        config = HumidityConfig(sensor_type=HumiditySensorType.CAPACITIVE)
        humidity = Humidity(pin=pin, config=config)
        humidity.enable()

        reading = humidity.read()
        # Linear mapping: 50% voltage = ~50% humidity
        assert 40 < reading.value < 60

    def test_humidity_clamping(self) -> None:
        """Test humidity is clamped to 0-100%."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(1.0)  # Max voltage

        config = HumidityConfig(
            sensor_type=HumiditySensorType.CAPACITIVE,
            scale=1.5,  # Would exceed 100%
        )
        humidity = Humidity(pin=pin, config=config)
        humidity.enable()

        reading = humidity.read()
        assert reading.value <= 100.0

    def test_status_tracking(self) -> None:
        """Test humidity sensor status tracking."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        humidity = Humidity(pin=pin)
        humidity.enable()

        assert humidity.status.readings_count == 0
        _ = humidity.read()
        assert humidity.status.readings_count == 1
        assert humidity.status.last_percent is not None


# =============================================================================
# Pressure Sensor Tests
# =============================================================================


class TestPressureConfig:
    """Tests for PressureConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = PressureConfig()
        assert config.name == "Pressure"
        assert config.sensor_type == PressureSensorType.BAROMETRIC
        assert config.unit == Unit.HECTOPASCALS
        assert config.sea_level_pressure == 1013.25

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = PressureConfig(
            unit=Unit.KILOPASCALS,
            sea_level_pressure=1020.0,
        )
        assert config.unit == Unit.KILOPASCALS
        assert config.sea_level_pressure == 1020.0


class TestPressure:
    """Tests for Pressure sensor."""

    def test_creation(self) -> None:
        """Test creating pressure sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        pressure = Pressure(pin=pin)

        assert pressure.name == "Pressure"
        assert pressure.unit == Unit.HECTOPASCALS

    def test_read_hpa(self) -> None:
        """Test reading pressure in hectopascals."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # Simulate raw value that would give ~1000 hPa
        pin.set_simulated_normalized(0.5)

        pressure = Pressure(pin=pin)
        pressure.enable()

        hpa = pressure.read_hpa()
        assert isinstance(hpa, float)

    def test_unit_conversion_pascals(self) -> None:
        """Test pressure unit conversion to Pascals."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = PressureConfig(unit=Unit.PASCALS)
        pressure = Pressure(pin=pin, config=config)
        pressure.enable()

        reading = pressure.read()
        assert reading.unit == Unit.PASCALS

    def test_unit_conversion_kilopascals(self) -> None:
        """Test pressure unit conversion to kilopascals."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = PressureConfig(unit=Unit.KILOPASCALS)
        pressure = Pressure(pin=pin, config=config)
        pressure.enable()

        reading = pressure.read()
        assert reading.unit == Unit.KILOPASCALS

    def test_estimate_altitude(self) -> None:
        """Test altitude estimation from pressure."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # Set to give roughly sea level pressure
        pin.set_simulated_normalized(0.98)

        config = PressureConfig(sea_level_pressure=1013.25)
        pressure = Pressure(pin=pin, config=config)
        pressure.enable()

        altitude = pressure.estimate_altitude()
        assert isinstance(altitude, float)

    def test_status_tracking(self) -> None:
        """Test pressure sensor status tracking."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        pressure = Pressure(pin=pin)
        pressure.enable()

        assert pressure.status.readings_count == 0
        _ = pressure.read()
        assert pressure.status.readings_count == 1
        assert pressure.status.last_pressure is not None


# =============================================================================
# Light Sensor Tests
# =============================================================================


class TestLightConfig:
    """Tests for LightConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = LightConfig()
        assert config.name == "Light"
        assert config.sensor_type == LightSensorType.I2C
        assert config.unit == Unit.LUX
        assert config.max_lux == 65535.0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = LightConfig(
            name="LDR",
            sensor_type=LightSensorType.PHOTORESISTOR,
            max_lux=10000.0,
        )
        assert config.name == "LDR"
        assert config.sensor_type == LightSensorType.PHOTORESISTOR
        assert config.max_lux == 10000.0


class TestLight:
    """Tests for Light sensor."""

    def test_creation(self) -> None:
        """Test creating light sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        light = Light(pin=pin)

        assert light.name == "Light"
        assert light.unit == Unit.LUX

    def test_read_lux(self) -> None:
        """Test reading light level in lux."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = LightConfig(sensor_type=LightSensorType.PHOTODIODE)
        light = Light(pin=pin, config=config)
        light.enable()

        lux = light.read_lux()
        assert isinstance(lux, float)
        assert lux >= 0

    def test_read_normalized(self) -> None:
        """Test reading normalized light level."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = LightConfig(sensor_type=LightSensorType.PHOTODIODE)
        light = Light(pin=pin, config=config)
        light.enable()

        normalized = light.read_normalized()
        assert 0.0 <= normalized <= 1.0

    def test_is_bright(self) -> None:
        """Test brightness threshold check."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        config = LightConfig(
            sensor_type=LightSensorType.PHOTODIODE,
            max_lux=1000.0,
        )
        light = Light(pin=pin, config=config)
        light.enable()

        # Low light
        pin.set_simulated_normalized(0.1)
        assert not light.is_bright(threshold=500.0)

        # High light
        pin.set_simulated_normalized(0.9)
        assert light.is_bright(threshold=500.0)

    def test_is_dark(self) -> None:
        """Test darkness threshold check."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        config = LightConfig(
            sensor_type=LightSensorType.PHOTODIODE,
            max_lux=1000.0,
        )
        light = Light(pin=pin, config=config)
        light.enable()

        # Very low light
        pin.set_simulated_normalized(0.005)
        assert light.is_dark(threshold=10.0)

        # Higher light
        pin.set_simulated_normalized(0.1)
        assert not light.is_dark(threshold=10.0)

    def test_ldr_sensor(self) -> None:
        """Test photoresistor (LDR) sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = LightConfig(sensor_type=LightSensorType.PHOTORESISTOR)
        light = Light(pin=pin, config=config)
        light.enable()

        lux = light.read_lux()
        assert isinstance(lux, float)
        assert lux >= 0

    def test_lux_clamping(self) -> None:
        """Test lux value clamping."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(1.0)

        config = LightConfig(
            sensor_type=LightSensorType.PHOTODIODE,
            max_lux=1000.0,
            scale=2.0,  # Would exceed max
        )
        light = Light(pin=pin, config=config)
        light.enable()

        lux = light.read_lux()
        assert lux <= 1000.0

    def test_status_tracking(self) -> None:
        """Test light sensor status tracking."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        light = Light(pin=pin)
        light.enable()

        assert light.status.readings_count == 0
        _ = light.read()
        assert light.status.readings_count == 1
        assert light.status.last_lux is not None


# =============================================================================
# Sensor Type Enum Tests
# =============================================================================


class TestSensorTypeEnums:
    """Tests for sensor type enumerations."""

    def test_temperature_sensor_types(self) -> None:
        """Test TemperatureSensorType enum values."""
        assert TemperatureSensorType.THERMISTOR.value == "thermistor"
        assert TemperatureSensorType.DS18B20.value == "ds18b20"
        assert TemperatureSensorType.THERMOCOUPLE.value == "thermocouple"
        assert TemperatureSensorType.I2C.value == "i2c"

    def test_humidity_sensor_types(self) -> None:
        """Test HumiditySensorType enum values."""
        assert HumiditySensorType.CAPACITIVE.value == "capacitive"
        assert HumiditySensorType.RESISTIVE.value == "resistive"
        assert HumiditySensorType.I2C.value == "i2c"

    def test_pressure_sensor_types(self) -> None:
        """Test PressureSensorType enum values."""
        assert PressureSensorType.BAROMETRIC.value == "barometric"
        assert PressureSensorType.DIFFERENTIAL.value == "differential"
        assert PressureSensorType.GAUGE.value == "gauge"
        assert PressureSensorType.ABSOLUTE.value == "absolute"

    def test_light_sensor_types(self) -> None:
        """Test LightSensorType enum values."""
        assert LightSensorType.PHOTORESISTOR.value == "photoresistor"
        assert LightSensorType.PHOTODIODE.value == "photodiode"
        assert LightSensorType.PHOTOTRANSISTOR.value == "phototransistor"
        assert LightSensorType.I2C.value == "i2c"
