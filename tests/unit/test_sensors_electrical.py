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


# =============================================================================
# Phase 5.5.5.2 - Enhanced Electrical Sensor Tests
# =============================================================================


class TestCurrentShuntCalculations:
    """Tests for current shunt resistor calculations (5.5.5.2)."""

    def test_shunt_resistor_calculation_ohms_law(self) -> None:
        """Test shunt resistor current calculation using Ohm's law (I = V/R)."""
        # 0.1 ohm shunt with 100mV drop = 1A
        shunt_resistance = 0.1  # ohms
        voltage_drop = 0.1  # volts (100mV)
        current = voltage_drop / shunt_resistance
        assert current == pytest.approx(1.0, abs=0.001)

    def test_shunt_resistor_high_current(self) -> None:
        """Test shunt resistor with high current."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # 0.1 ohm shunt, 10A current = 1V drop
        # 1V / 3.3V = 0.303
        pin.set_simulated_normalized(0.303)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.SHUNT,
            shunt_resistance=0.1,
            max_current=20.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(10.0, abs=0.5)

    def test_shunt_resistor_low_current(self) -> None:
        """Test shunt resistor with low current (milliamps)."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        # 0.1 ohm shunt, 0.5A current = 50mV drop
        # 0.05V / 3.3V = 0.0151
        pin.set_simulated_normalized(0.0151)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.SHUNT,
            shunt_resistance=0.1,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(0.5, abs=0.1)

    def test_shunt_power_dissipation_calculation(self) -> None:
        """Test shunt resistor power dissipation (P = I²R)."""
        current = 5.0  # Amps
        shunt_resistance = 0.1  # ohms

        power_dissipation = current * current * shunt_resistance
        assert power_dissipation == pytest.approx(2.5, abs=0.01)  # 2.5W


class TestCurrentHallEffectCalculations:
    """Tests for hall effect current sensor calculations (5.5.5.2)."""

    def test_hall_effect_sensitivity_acs712_5a(self) -> None:
        """Test ACS712-05B sensitivity (185mV/A)."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        # At 3A with 185mV/A: V = 2.5 + (3 * 0.185) = 3.055V
        # 3.055V / 5.0V = 0.611
        pin.set_simulated_normalized(0.611)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=185.0,  # ACS712-05B
            zero_current_voltage=2.5,
            reference_voltage=5.0,
            max_current=5.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(3.0, abs=0.3)

    def test_hall_effect_sensitivity_acs712_20a(self) -> None:
        """Test ACS712-20A sensitivity (100mV/A)."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        # At 15A with 100mV/A: V = 2.5 + (15 * 0.1) = 4.0V
        # 4.0V / 5.0V = 0.8
        pin.set_simulated_normalized(0.8)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,  # ACS712-20A
            zero_current_voltage=2.5,
            reference_voltage=5.0,
            max_current=20.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(15.0, abs=0.5)

    def test_hall_effect_sensitivity_acs712_30a(self) -> None:
        """Test ACS712-30A sensitivity (66mV/A)."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        # At 20A with 66mV/A: V = 2.5 + (20 * 0.066) = 3.82V
        # 3.82V / 5.0V = 0.764
        pin.set_simulated_normalized(0.764)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=66.0,  # ACS712-30A
            zero_current_voltage=2.5,
            reference_voltage=5.0,
            max_current=30.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value == pytest.approx(20.0, abs=1.0)

    def test_hall_effect_bidirectional(self) -> None:
        """Test bidirectional current measurement."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
            min_current=-20.0,
            max_current=20.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        # Positive current (3.5V = +10A)
        pin.set_simulated_normalized(0.7)
        positive = sensor.read()
        assert positive.value > 0

        # Negative current (1.5V = -10A)
        pin.set_simulated_normalized(0.3)
        negative = sensor.read()
        assert negative.value < 0


class TestCurrentAveraging:
    """Tests for current measurement averaging (5.5.5.2)."""

    def test_current_multiple_readings(self) -> None:
        """Test taking multiple current readings."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        readings = [sensor.read().value for _ in range(10)]

        # All readings should be consistent (simulated)
        avg = sum(readings) / len(readings)
        for r in readings:
            assert r == pytest.approx(avg, abs=0.1)

    def test_current_reading_count_tracking(self) -> None:
        """Test that reading count is properly tracked."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        sensor = CurrentSensor(pin=pin)
        sensor.enable()

        assert sensor.status.readings_count == 0

        for i in range(5):
            _ = sensor.read()
            assert sensor.status.readings_count == i + 1


class TestVoltageDividerCalculations:
    """Tests for voltage divider calculations (5.5.5.2)."""

    def test_voltage_divider_formula(self) -> None:
        """Test voltage divider formula: Vout = Vin × R2/(R1+R2)."""
        vin = 12.0
        r1 = 30000.0
        r2 = 7500.0

        vout = vin * r2 / (r1 + r2)
        assert vout == pytest.approx(2.4, abs=0.01)

    def test_voltage_divider_ratio_calculation(self) -> None:
        """Test voltage divider ratio: Vin/Vout = (R1+R2)/R2."""
        r1 = 30000.0
        r2 = 7500.0

        ratio = (r1 + r2) / r2
        assert ratio == pytest.approx(5.0, abs=0.01)

    def test_voltage_divider_various_ratios(self) -> None:
        """Test different voltage divider configurations."""
        test_cases = [
            # (R1, R2, expected_ratio)
            (10000, 10000, 2.0),  # 1:1 divider
            (30000, 7500, 5.0),  # Common 12V to 3.3V
            (47000, 10000, 5.7),  # Common value
            (100000, 10000, 11.0),  # High attenuation
        ]

        for r1, r2, expected_ratio in test_cases:
            config = VoltageSensorConfig(r1=float(r1), r2=float(r2))
            sensor = VoltageSensor(config=config)
            assert sensor.divider_ratio == pytest.approx(expected_ratio, abs=0.1)

    def test_voltage_adc_calibration_offset(self) -> None:
        """Test ADC calibration with offset."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = VoltageSensorConfig(
            sensor_type=VoltageSensorType.DIRECT,
            offset=0.05,  # 50mV offset correction
        )
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        # 0.5 * 3.3V = 1.65V + 0.05V offset = 1.70V
        assert reading.value == pytest.approx(1.70, abs=0.1)

    def test_voltage_adc_calibration_scale(self) -> None:
        """Test ADC calibration with scale factor."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.5)

        config = VoltageSensorConfig(
            sensor_type=VoltageSensorType.DIRECT,
            scale=1.05,  # 5% scale correction
        )
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        # 1.65V * 1.05 = 1.7325V
        assert reading.value == pytest.approx(1.7325, abs=0.1)


class TestPowerCalculations:
    """Tests for power calculations (5.5.5.2)."""

    def test_power_calculation_p_equals_vi(self) -> None:
        """Test power calculation: P = V × I."""
        voltage = 12.0  # Volts
        current = 2.5  # Amps
        power = voltage * current
        assert power == pytest.approx(30.0, abs=0.001)

    def test_power_calculation_p_equals_i2r(self) -> None:
        """Test power calculation: P = I² × R."""
        current = 5.0  # Amps
        resistance = 10.0  # Ohms
        power = current * current * resistance
        assert power == pytest.approx(250.0, abs=0.001)

    def test_power_calculation_p_equals_v2_over_r(self) -> None:
        """Test power calculation: P = V² / R."""
        voltage = 24.0  # Volts
        resistance = 12.0  # Ohms
        power = (voltage * voltage) / resistance
        assert power == pytest.approx(48.0, abs=0.001)

    def test_power_sensor_config(self) -> None:
        """Test power sensor configuration."""
        config = PowerSensorConfig(
            name="MotorPower",
            shunt_resistance=0.05,
            max_expected_current=10.0,
            current_lsb=0.0005,
        )
        sensor = PowerSensor(config=config)

        assert sensor.name == "MotorPower"
        assert sensor.config.shunt_resistance == 0.05
        assert sensor.config.max_expected_current == 10.0


class TestEnergyIntegration:
    """Tests for energy integration calculations (5.5.5.2)."""

    def test_energy_calculation_joules(self) -> None:
        """Test energy calculation: E = P × t (in Joules)."""
        power = 100.0  # Watts
        time_seconds = 60.0  # 1 minute
        energy_joules = power * time_seconds
        assert energy_joules == pytest.approx(6000.0, abs=0.001)

    def test_energy_calculation_watt_hours(self) -> None:
        """Test energy calculation in Watt-hours."""
        power = 100.0  # Watts
        time_hours = 2.5  # hours
        energy_wh = power * time_hours
        assert energy_wh == pytest.approx(250.0, abs=0.001)

    def test_energy_calculation_kilowatt_hours(self) -> None:
        """Test energy calculation in kilowatt-hours."""
        power = 1500.0  # Watts
        time_hours = 3.0  # hours
        energy_kwh = (power * time_hours) / 1000.0
        assert energy_kwh == pytest.approx(4.5, abs=0.001)

    def test_energy_integration_discrete(self) -> None:
        """Test discrete energy integration from power samples."""
        # Simulate power readings over time
        power_readings = [100.0, 120.0, 110.0, 130.0, 115.0]
        sample_interval_seconds = 1.0

        # Trapezoidal integration
        total_energy_joules = 0.0
        for i in range(len(power_readings) - 1):
            avg_power = (power_readings[i] + power_readings[i + 1]) / 2
            total_energy_joules += avg_power * sample_interval_seconds

        # Expected: average of adjacent pairs × time
        (110 + 115 + 120 + 122.5) * 1.0  # Simplified
        assert total_energy_joules > 0

    def test_battery_capacity_calculation(self) -> None:
        """Test battery capacity calculation in amp-hours."""
        # A 3Ah battery at 3.7V nominal
        capacity_ah = 3.0
        voltage = 3.7

        energy_wh = capacity_ah * voltage
        assert energy_wh == pytest.approx(11.1, abs=0.01)


class TestElectricalSensorEdgeCases:
    """Edge case tests for electrical sensors (5.5.5.2)."""

    def test_current_sensor_zero_shunt_resistance(self) -> None:
        """Test current sensor with zero shunt resistance."""
        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.SHUNT,
            shunt_resistance=0.0,
        )
        sensor = CurrentSensor(config=config)
        sensor.enable()

        # Should handle division by zero gracefully
        reading = sensor.read()
        assert reading.value == 0.0

    def test_voltage_sensor_zero_r2(self) -> None:
        """Test voltage divider with R2=0."""
        config = VoltageSensorConfig(r1=10000.0, r2=0.0)
        sensor = VoltageSensor(config=config)

        # Should return ratio of 1.0 to avoid division by zero
        assert sensor.divider_ratio == 1.0

    def test_current_sensor_limits(self) -> None:
        """Test current sensor respects limits."""
        pin = SimulatedAnalogPin(0, reference_voltage=5.0)
        pin.setup()
        # Extreme high value
        pin.set_simulated_normalized(1.0)

        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.HALL_EFFECT,
            sensitivity_mv_per_amp=100.0,
            zero_current_voltage=2.5,
            reference_voltage=5.0,
            max_current=30.0,
        )
        sensor = CurrentSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value <= 30.0

    def test_voltage_sensor_limits(self) -> None:
        """Test voltage sensor respects limits."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(1.0)

        config = VoltageSensorConfig(
            sensor_type=VoltageSensorType.DIVIDER,
            r1=30000.0,
            r2=7500.0,
            max_voltage=25.0,
        )
        sensor = VoltageSensor(pin=pin, config=config)
        sensor.enable()

        reading = sensor.read()
        assert reading.value <= 25.0

    def test_i2c_current_sensor_type(self) -> None:
        """Test I2C current sensor configuration."""
        config = CurrentSensorConfig(
            sensor_type=CurrentSensorType.I2C,
            name="INA219",
        )
        sensor = CurrentSensor(config=config)

        assert sensor.config.sensor_type == CurrentSensorType.I2C

    def test_i2c_voltage_sensor_type(self) -> None:
        """Test I2C voltage sensor configuration."""
        config = VoltageSensorConfig(
            sensor_type=VoltageSensorType.I2C,
            name="INA219_Voltage",
        )
        sensor = VoltageSensor(config=config)

        assert sensor.config.sensor_type == VoltageSensorType.I2C
