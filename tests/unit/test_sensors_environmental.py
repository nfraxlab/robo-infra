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


# =============================================================================
# Phase 5.5.5.1 - Enhanced Environmental Sensor Tests
# =============================================================================


class TestTemperatureConversions:
    """Tests for temperature unit conversions (5.5.5.1)."""

    def test_celsius_to_fahrenheit_freezing(self) -> None:
        """Test Celsius to Fahrenheit conversion at freezing point."""
        # 0°C = 32°F
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # Need raw value that gives 0°C
        # Thermistor at 0°C has higher resistance, lower voltage
        pin.set_simulated_normalized(0.2)

        config = TemperatureConfig(unit=Unit.CELSIUS, offset=-25.0)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        celsius = temp.read_celsius()
        fahrenheit = temp.read_fahrenheit()

        # Verify conversion formula: F = C * 9/5 + 32
        expected_f = celsius * 9.0 / 5.0 + 32.0
        assert fahrenheit == pytest.approx(expected_f, abs=0.1)

    def test_celsius_to_fahrenheit_boiling(self) -> None:
        """Test Celsius to Fahrenheit conversion formula validation."""
        # Verify 100°C = 212°F using the formula
        celsius = 100.0
        fahrenheit = celsius * 9.0 / 5.0 + 32.0
        assert fahrenheit == pytest.approx(212.0, abs=0.01)

    def test_fahrenheit_to_celsius_room_temp(self) -> None:
        """Test Fahrenheit to Celsius conversion at room temperature."""
        # 77°F = 25°C
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = TemperatureConfig(unit=Unit.FAHRENHEIT)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        # Convert back to Celsius
        celsius = (reading.value - 32.0) * 5.0 / 9.0
        assert reading.unit == Unit.FAHRENHEIT
        # Verify it's a valid temperature reading
        assert -40 < celsius < 125

    def test_fahrenheit_to_celsius_formula(self) -> None:
        """Test Fahrenheit to Celsius conversion formula."""
        # 68°F = 20°C
        fahrenheit = 68.0
        celsius = (fahrenheit - 32.0) * 5.0 / 9.0
        assert celsius == pytest.approx(20.0, abs=0.01)

    def test_thermistor_calculation_cold(self) -> None:
        """Test thermistor calculation at cold temperature.

        For NTC thermistor with thermistor connected to ground:
        - Higher raw value = higher thermistor resistance = colder
        """
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # High voltage/raw = high resistance = cold (NTC thermistor)
        pin.set_simulated_normalized(0.85)

        config = TemperatureConfig(
            sensor_type=TemperatureSensorType.THERMISTOR,
            beta_coefficient=3950,
            nominal_resistance=10000.0,
            nominal_temp=25.0,
        )
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        # High normalized value should give cold temperature for NTC
        assert reading.value < 10

    def test_thermistor_calculation_hot(self) -> None:
        """Test thermistor calculation at hot temperature.

        For NTC thermistor with thermistor connected to ground:
        - Lower raw value = lower thermistor resistance = hotter
        """
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # Low voltage/raw = low resistance = hot (NTC thermistor)
        pin.set_simulated_normalized(0.15)

        config = TemperatureConfig(
            sensor_type=TemperatureSensorType.THERMISTOR,
            beta_coefficient=3950,
            nominal_resistance=10000.0,
            nominal_temp=25.0,
        )
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        # Low normalized value should give hot temperature for NTC
        assert reading.value > 30

    def test_thermistor_beta_coefficient_effect(self) -> None:
        """Test that beta coefficient affects thermistor calculation."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.3)

        # Lower beta = steeper curve
        config1 = TemperatureConfig(beta_coefficient=3000)
        temp1 = Temperature(pin=pin, config=config1)
        temp1.enable()
        reading1 = temp1.read()

        # Higher beta = shallower curve
        config2 = TemperatureConfig(beta_coefficient=4500)
        temp2 = Temperature(pin=pin, config=config2)
        temp2.enable()
        reading2 = temp2.read()

        # Different beta values should give different temperatures
        assert reading1.value != reading2.value


class TestHumidityCalculations:
    """Tests for humidity-related calculations (5.5.5.1)."""

    def test_dewpoint_calculation_formula(self) -> None:
        """Test dewpoint calculation using Magnus formula.

        Magnus formula:
        γ(T,RH) = ln(RH/100) + (a*T)/(b+T)
        Td = (b * γ) / (a - γ)
        where a = 17.27, b = 237.7
        """
        # At 25°C and 50% RH, dewpoint should be around 13.8°C
        import math

        temp_c = 25.0
        rh = 50.0
        a = 17.27
        b = 237.7

        gamma = math.log(rh / 100.0) + (a * temp_c) / (b + temp_c)
        dewpoint = (b * gamma) / (a - gamma)

        assert dewpoint == pytest.approx(13.8, abs=0.5)

    def test_dewpoint_at_high_humidity(self) -> None:
        """Test dewpoint approaches temperature at high humidity."""
        import math

        temp_c = 20.0
        rh = 95.0  # Very high humidity
        a = 17.27
        b = 237.7

        gamma = math.log(rh / 100.0) + (a * temp_c) / (b + temp_c)
        dewpoint = (b * gamma) / (a - gamma)

        # At 95% RH, dewpoint should be very close to temperature
        assert abs(dewpoint - temp_c) < 2.0

    def test_dewpoint_at_low_humidity(self) -> None:
        """Test dewpoint is much lower than temp at low humidity."""
        import math

        temp_c = 25.0
        rh = 20.0  # Low humidity
        a = 17.27
        b = 237.7

        gamma = math.log(rh / 100.0) + (a * temp_c) / (b + temp_c)
        dewpoint = (b * gamma) / (a - gamma)

        # At 20% RH, dewpoint should be much lower than temperature
        assert dewpoint < temp_c - 15

    def test_absolute_humidity_from_relative(self) -> None:
        """Test absolute humidity calculation from relative humidity.

        Absolute humidity (g/m³) = (RH × ρw) / 100
        where ρw is saturation vapor density at temperature
        """
        import math

        temp_c = 20.0
        rh = 50.0

        # Saturation vapor pressure (hPa) using simplified formula
        es = 6.112 * math.exp((17.67 * temp_c) / (temp_c + 243.5))

        # Saturation vapor density (g/m³)
        # ρw = 216.7 * es / (temp_c + 273.15)
        rho_w = 216.7 * es / (temp_c + 273.15)

        # Absolute humidity
        abs_humidity = (rh * rho_w) / 100.0

        # At 20°C, 50% RH, absolute humidity should be ~8.6 g/m³
        assert abs_humidity == pytest.approx(8.6, abs=0.5)

    def test_humidity_sensor_calibration(self) -> None:
        """Test humidity sensor with calibration scale and offset."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = HumidityConfig(
            sensor_type=HumiditySensorType.CAPACITIVE,
            scale=0.95,
            offset=2.0,
        )
        humidity = Humidity(pin=pin, config=config)
        humidity.enable()

        reading = humidity.read()
        # Calibration should be applied
        assert reading.value > 0
        assert reading.value <= 100


class TestPressureCalculations:
    """Tests for pressure-related calculations (5.5.5.1)."""

    def test_pressure_altitude_calculation_sea_level(self) -> None:
        """Test altitude calculation at sea level pressure."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # Set raw value for approximately 1013 hPa
        pin.set_simulated_normalized(0.99)

        config = PressureConfig(sea_level_pressure=1013.25)
        pressure = Pressure(pin=pin, config=config)
        pressure.enable()

        altitude = pressure.estimate_altitude()
        # Near sea level pressure should give near-zero altitude
        assert isinstance(altitude, float)

    def test_pressure_altitude_calculation_high(self) -> None:
        """Test altitude calculation at reduced pressure (higher altitude)."""
        import math

        # At ~900 hPa, altitude should be around 1000m
        p = 900.0
        p0 = 1013.25
        altitude = 44330.0 * (1.0 - math.pow(p / p0, 0.1903))

        assert altitude == pytest.approx(1000, abs=100)

    def test_pressure_altitude_formula(self) -> None:
        """Test barometric altitude formula accuracy."""
        import math

        # Known altitude: Denver, CO is at ~1609m with avg pressure ~840 hPa
        p = 840.0
        p0 = 1013.25
        altitude = 44330.0 * (1.0 - math.pow(p / p0, 0.1903))

        assert altitude == pytest.approx(1609, abs=200)

    def test_pressure_sea_level_correction(self) -> None:
        """Test sea level pressure correction for altitude."""
        import math

        # If we're at 500m with measured pressure of 955 hPa,
        # calculate what sea level pressure would be
        measured_pressure = 955.0
        altitude = 500.0

        # Inverse of altitude formula to get P0
        # P = P0 * (1 - altitude/44330)^5.255
        # P0 = P / (1 - altitude/44330)^5.255
        sea_level_pressure = measured_pressure / math.pow(1.0 - altitude / 44330.0, 5.255)

        # Sea level pressure should be higher than measured
        assert sea_level_pressure > measured_pressure
        assert sea_level_pressure == pytest.approx(1013, abs=20)

    def test_pressure_unit_conversions(self) -> None:
        """Test pressure unit conversion accuracy."""
        # 1013.25 hPa = 101325 Pa = 101.325 kPa = 1.01325 bar = 14.696 psi
        hpa = 1013.25

        assert hpa * 100 == pytest.approx(101325, abs=1)  # Pa
        assert hpa / 10 == pytest.approx(101.325, abs=0.01)  # kPa
        assert hpa / 1000 == pytest.approx(1.01325, abs=0.001)  # bar
        assert hpa * 0.0145038 == pytest.approx(14.696, abs=0.01)  # psi


class TestTemperatureEdgeCases:
    """Edge case tests for temperature sensors (5.5.5.1)."""

    def test_temperature_min_limit(self) -> None:
        """Test temperature at minimum limit."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.0)  # Extreme cold

        config = TemperatureConfig(min_temp=-40.0)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        assert reading.value >= -40.0

    def test_temperature_max_limit(self) -> None:
        """Test temperature at maximum limit."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(1.0)  # Extreme hot

        config = TemperatureConfig(max_temp=125.0)
        temp = Temperature(pin=pin, config=config)
        temp.enable()

        reading = temp.read()
        assert reading.value <= 125.0

    def test_ds18b20_sensor_type(self) -> None:
        """Test DS18B20 digital temperature sensor."""
        config = TemperatureConfig(
            sensor_type=TemperatureSensorType.DS18B20,
            name="DS18B20",
        )
        temp = Temperature(config=config)

        assert temp.config.sensor_type == TemperatureSensorType.DS18B20

    def test_thermocouple_sensor_type(self) -> None:
        """Test thermocouple temperature sensor."""
        config = TemperatureConfig(
            sensor_type=TemperatureSensorType.THERMOCOUPLE,
            name="TypeK",
        )
        temp = Temperature(config=config)

        assert temp.config.sensor_type == TemperatureSensorType.THERMOCOUPLE

    def test_i2c_temperature_sensor(self) -> None:
        """Test I2C temperature sensor configuration."""
        config = TemperatureConfig(
            sensor_type=TemperatureSensorType.I2C,
            name="TMP102",
        )
        temp = Temperature(config=config)

        assert temp.config.sensor_type == TemperatureSensorType.I2C
