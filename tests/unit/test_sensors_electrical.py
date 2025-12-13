"""Unit tests for electrical sensors (Phase 4.6)."""

from __future__ import annotations

import pytest

from robo_infra.core.pin import SimulatedAnalogPin
from robo_infra.core.types import Unit
from robo_infra.sensors.electrical import (
    CurrentSensor,
    CurrentSensorConfig,
    CurrentSensorType,
    PowerSensor,
    PowerSensorConfig,
    VoltageSensor,
    VoltageSensorConfig,
    VoltageSensorType,
)


# =============================================================================
# Current Sensor Tests
# =============================================================================


class TestCurrentSensorConfig:
    """Tests for CurrentSensorConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = CurrentSensorConfig()
        assert config.name == "Current"
        assert config.sensor_type == CurrentSensorType.HALL_EFFECT
        assert config.unit == Unit.AMPS
        assert config.sensitivity_mv_per_amp == 100.0
        assert config.zero_current_voltage == 2.5

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = CurrentSensorConfig(
            name="MotorCurrent",
            sensor_type=CurrentSensorType.SHUNT,
            shunt_resistance=0.05,
            max_current=50.0,
        )
        assert config.name == "MotorCurrent"
        assert config.sensor_type == CurrentSensorType.SHUNT
        assert config.shunt_resistance == 0.05
        assert config.max_current == 50.0


class TestCurrentSensor:
    """Tests for CurrentSensor."""

    def test_creation(self) -> None:
        """Test creating current sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        sensor = CurrentSensor(pin=pin)

        assert sensor.name == "Current"
        assert sensor.unit == Unit.AMPS

    def test_hall_effect_zero_current(self) -> None:
        """Test hall effect sensor at zero current (Vcc/2)."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        # At 0A, voltage = 2.5V = 50% of 5V reference
        pin.set_simulated_normalized(0.5)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(0.0, abs=0.1)

    def test_hall_effect_positive_current(self) -> None:
        """Test hall effect sensor with positive current."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        # At 10A with 100mV/A sensitivity: V = 2.5 + (10 * 0.1) = 3.5V
        # 3.5V / 5.0V = 0.7
        pin.set_simulated_normalized(0.7)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(10.0, abs=0.5)

    def test_hall_effect_negative_current(self) -> None:
        """Test hall effect sensor with negative (reverse) current."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        # At -5A with 100mV/A sensitivity: V = 2.5 + (-5 * 0.1) = 2.0V
        # 2.0V / 5.0V = 0.4
        pin.set_simulated_normalized(0.4)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(-5.0, abs=0.5)

    def test_shunt_resistor(self) -> None:
        """Test shunt resistor current sensing."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # 0.1 ohm shunt, 2A current = 0.2V drop
        # 0.2V / 3.3V = 0.0606
        pin.set_simulated_normalized(0.0606)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.SHUNT,
            shunt_resistance=0.1,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(2.0, abs=0.2)

    def test_read_milliamps(self) -> None:
        """Test reading current in milliamps."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        pin.set_simulated_normalized(0.5)  # Zero current

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        milliamps = sensor.read_milliamps()
        assert milliamps == pytest.approx(0.0, abs=100)

    def test_calibration_offset(self) -> None:
        """Test current sensor calibration offset."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        pin.set_simulated_normalized(0.5)  # Zero current

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
            offset=0.5,  # Add 0.5A offset
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(0.5, abs=0.1)

    def test_status_tracking(self) -> None:
        """Test current sensor status tracking."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        sensor = CurrentSensor(pin=pin)
        sensor.enable()

        assert sensor.status.readings_count == 0
        _ = sensor.read()
        assert sensor.status.readings_count == 1
        assert sensor.status.last_current is not None


# =============================================================================
# Voltage Sensor Tests
# =============================================================================


class TestVoltageSensorConfig:
    """Tests for VoltageSensorConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = VoltageSensorConfig()
        assert config.name == "Voltage"
        assert config.sensor_type == VoltageSensorType.DIVIDER
        assert config.unit == Unit.VOLTS
        assert config.r1 == 30000.0
        assert config.r2 == 7500.0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = VoltageSensorConfig(
            name="BatteryVoltage",
            sensor_type=VoltageSensorType.DIRECT,
            max_voltage=5.0,
        )
        assert config.name == "BatteryVoltage"
        assert config.sensor_type == VoltageSensorType.DIRECT
        assert config.max_voltage == 5.0


class TestVoltageSensor:
    """Tests for VoltageSensor."""

    def test_creation(self) -> None:
        """Test creating voltage sensor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        sensor = VoltageSensor(pin=pin)

        assert sensor.name == "Voltage"
        assert sensor.unit == Unit.VOLTS

    def test_direct_measurement(self) -> None:
        """Test direct voltage measurement."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # Set to 1.65V (half of 3.3V reference)
        pin.set_simulated_normalized(0.5)

        config = VoltageSensorConfig(sensor_type=VoltageSensorType.DIRECT)
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(1.65, abs=0.1)

    def test_voltage_divider(self) -> None:
        """Test voltage divider measurement."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # With 30k/7.5k divider, ratio = 5.0
        # To measure 12V: ADC sees 12V / 5 = 2.4V
        # 2.4V / 3.3V = 0.727
        pin.set_simulated_normalized(0.727)

        config = VoltageSensorConfig(
            sensor_type=VoltageSensorType.DIVIDER,
            r1=30000.0,
            r2=7500.0,
        )
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(12.0, abs=0.5)

    def test_divider_ratio(self) -> None:
        """Test voltage divider ratio calculation."""
        config = VoltageSensorConfig(
            r1=30000.0,
            r2=7500.0,
        )
        sensor = VoltageSensor(config=config)

        # Ratio = (30k + 7.5k) / 7.5k = 5.0
        assert sensor.divider_ratio == pytest.approx(5.0, abs=0.01)

    def test_read_millivolts(self) -> None:
        """Test reading voltage in millivolts."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = VoltageSensorConfig(sensor_type=VoltageSensorType.DIRECT)
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        millivolts = sensor.read_millivolts()
        assert millivolts == pytest.approx(1650, abs=50)

    def test_calibration_scale(self) -> None:
        """Test voltage sensor calibration scale."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)  # 1.65V

        config = VoltageSensorConfig(
            sensor_type=VoltageSensorType.DIRECT,
            scale=1.02,  # 2% calibration adjustment
        )
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(1.65 * 1.02, abs=0.1)

    def test_status_tracking(self) -> None:
        """Test voltage sensor status tracking."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        sensor = VoltageSensor(pin=pin)
        sensor.enable()

        assert sensor.status.readings_count == 0
        _ = sensor.read()
        assert sensor.status.readings_count == 1
        assert sensor.status.last_voltage is not None

    def test_zero_r2_divider(self) -> None:
        """Test voltage divider with r2=0 returns ratio of 1."""
        config = VoltageSensorConfig(r1=10000.0, r2=0.0)
        sensor = VoltageSensor(config=config)

        assert sensor.divider_ratio == 1.0


# =============================================================================
# Power Sensor Tests
# =============================================================================


class TestPowerSensorConfig:
    """Tests for PowerSensorConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = PowerSensorConfig()
        assert config.name == "Power"
        assert config.unit == Unit.WATTS
        assert config.shunt_resistance == 0.1
        assert config.max_expected_current == 3.2

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = PowerSensorConfig(
            name="MotorPower",
            shunt_resistance=0.05,
            max_expected_current=10.0,
        )
        assert config.name == "MotorPower"
        assert config.shunt_resistance == 0.05
        assert config.max_expected_current == 10.0


class TestPowerSensor:
    """Tests for PowerSensor."""

    def test_creation(self) -> None:
        """Test creating power sensor."""
        sensor = PowerSensor()

        assert sensor.name == "Power"
        assert sensor.unit == Unit.WATTS

    def test_status_tracking(self) -> None:
        """Test power sensor status tracking."""
        sensor = PowerSensor()
        sensor.enable()

        assert sensor.status.readings_count == 0
        _ = sensor.read()
        assert sensor.status.readings_count == 1

    def test_read_without_bus(self) -> None:
        """Test reading without I2C bus returns zero."""
        sensor = PowerSensor()
        sensor.enable()

        assert sensor.read_voltage() == 0.0
        assert sensor.read_current() == 0.0


# =============================================================================
# Sensor Type Enum Tests
# =============================================================================


class TestSensorTypeEnums:
    """Tests for sensor type enumerations."""

    def test_current_sensor_types(self) -> None:
        """Test CurrentSensorType enum values."""
        assert CurrentSensorType.SHUNT.value == "shunt"
        assert CurrentSensorType.HALL_EFFECT.value == "hall_effect"
        assert CurrentSensorType.I2C.value == "i2c"

    def test_voltage_sensor_types(self) -> None:
        """Test VoltageSensorType enum values."""
        assert VoltageSensorType.DIRECT.value == "direct"
        assert VoltageSensorType.DIVIDER.value == "divider"
        assert VoltageSensorType.I2C.value == "i2c"


# =============================================================================
# Integration Tests
# =============================================================================


class TestElectricalSensorIntegration:
    """Integration tests for electrical sensors."""

    def test_battery_monitoring(self) -> None:
        """Test typical battery monitoring scenario."""
        # Voltage sensing with divider for 12V battery
        voltage_pin = SimulatedAnalogPin(0)
        voltage_pin.setup()

        voltage_config = VoltageSensorConfig(
            name="BatteryVoltage",
            sensor_type=VoltageSensorType.DIVIDER,
            r1=30000.0,
            r2=7500.0,
            max_voltage=20.0,
        )
        voltage_sensor = VoltageSensor(pin=voltage_pin, config=voltage_config)
        voltage_sensor.enable()

        # Current sensing for load
        current_pin = SimulatedAnalogPin(1, reference_voltage=5.0)
        current_pin.setup()

        current_config = CurrentSensorConfig(
            name="LoadCurrent",
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
            max_current=20.0,
        )
        current_sensor = CurrentSensor(pin=current_pin, config=current_config)
        current_sensor.enable()

        # Simulate 12V battery, 5A load
        voltage_pin.set_simulated_normalized(0.727)  # 12V through divider
        current_pin.set_simulated_normalized(0.6)  # 5A with hall effect

        voltage = voltage_sensor.read()
        current = current_sensor.read()

        assert voltage.value == pytest.approx(12.0, abs=0.5)
        assert current.value == pytest.approx(5.0, abs=0.5)

        # Calculate power
        power = voltage.value * current.value
        assert power == pytest.approx(60.0, abs=5.0)
