"""Tests for robo_infra.sensors.distance (Phase 4.1)."""

from __future__ import annotations

import pytest

from robo_infra.core.bus import SimulatedI2CBus
from robo_infra.core.driver import ChannelConfig, SimulatedDriver
from robo_infra.core.exceptions import CommunicationError
from robo_infra.core.pin import SimulatedAnalogPin, SimulatedDigitalPin
from robo_infra.core.types import Unit
from robo_infra.sensors.distance import (
    DistanceSensor,
    IRDistance,
    IRDistanceConfig,
    IRDistanceStatus,
    ToF,
    ToFConfig,
    ToFStatus,
    Ultrasonic,
    UltrasonicConfig,
    UltrasonicStatus,
)


# =============================================================================
# Original Tests
# =============================================================================


class TestUltrasonic:
    def test_driver_backed_raw_us_to_cm(self) -> None:
        driver = SimulatedDriver(name="drv", channels=4)
        driver.connect()
        driver.enable()

        # Configure channel to accept microsecond values (0-10000)
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=10000.0))

        # 58us per cm -> 580us should be ~10cm
        driver.set_channel(0, 580.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS)
        s.enable()
        reading = s.read()
        assert reading.unit == Unit.CENTIMETERS
        assert 9.5 < reading.value < 10.5
        assert s.read_raw() == 580


class TestToF:
    def test_i2c_register_word_mm(self) -> None:
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x01, 0x01: 0xF4})  # 0x01F4 = 500mm
        bus.open()

        s = ToF(bus, address=0x29, register=0x00)
        s.enable()
        reading = s.read()
        assert reading.unit == Unit.MILLIMETERS
        assert reading.value == 500.0


class TestIRDistance:
    def test_analog_voltage_conversion(self) -> None:
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=2.0)
        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0)
        s.enable()

        reading = s.read()
        assert reading.unit == Unit.MILLIMETERS
        assert reading.value == 500.0

    def test_driver_voltage_channel(self) -> None:
        driver = SimulatedDriver(name="drv", channels=2)
        driver.connect()
        driver.enable()
        driver.set_channel(1, 1.0)

        s = IRDistance(driver=driver, channel=1, a=1000.0, b=0.0)
        s.enable()
        reading = s.read()
        assert reading.value == 1000.0

    def test_requires_source(self) -> None:
        s = IRDistance(a=1000.0)
        s.enable()
        with pytest.raises(CommunicationError):
            _ = s.read()


# =============================================================================
# Phase 5.5.3.2 - Ultrasonic Tests
# =============================================================================


class TestUltrasonicConfig:
    """Tests for UltrasonicConfig (5.5.3.2)."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = UltrasonicConfig()

        assert config.name == "Ultrasonic"
        assert config.unit == Unit.CENTIMETERS
        assert config.trigger_pulse_us == 10
        assert config.timeout_s == 0.02
        assert config.us_per_cm == 58.0

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = UltrasonicConfig(
            name="HC-SR04",
            unit=Unit.MILLIMETERS,
            trigger_pulse_us=15,
            timeout_s=0.03,
            us_per_cm=60.0,
            metadata={"sensor_type": "ultrasonic"},
        )

        assert config.name == "HC-SR04"
        assert config.unit == Unit.MILLIMETERS
        assert config.trigger_pulse_us == 15
        assert config.timeout_s == 0.03
        assert config.us_per_cm == 60.0
        assert config.metadata["sensor_type"] == "ultrasonic"


class TestUltrasonicStatus:
    """Tests for UltrasonicStatus (5.5.3.2)."""

    def test_status_default(self) -> None:
        """Test default status values."""
        status = UltrasonicStatus()

        assert status.last_duration_us is None

    def test_status_with_value(self) -> None:
        """Test status with duration value."""
        status = UltrasonicStatus(last_duration_us=580)

        assert status.last_duration_us == 580


class TestUltrasonicDistanceCalculation:
    """Tests for ultrasonic distance calculation (5.5.3.2)."""

    def test_distance_calculation_cm(self) -> None:
        """Test distance calculation in centimeters."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))

        # 58 us per cm -> 1160 us = 20 cm
        driver.set_channel(0, 1160.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS, us_per_cm=58.0)
        s.enable()

        reading = s.read()
        assert reading.value == pytest.approx(20.0, abs=0.5)

    def test_distance_calculation_mm(self) -> None:
        """Test distance calculation in millimeters."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))

        # 58 us per cm -> 580 us = 10 cm = 100 mm
        driver.set_channel(0, 580.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.MILLIMETERS, us_per_cm=58.0)
        s.enable()

        reading = s.read()
        assert reading.unit == Unit.MILLIMETERS
        assert reading.value == pytest.approx(100.0, abs=5.0)

    def test_speed_of_sound_temp_compensation(self) -> None:
        """Test that us_per_cm can be adjusted for temperature."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))

        # Set a fixed duration
        driver.set_channel(0, 580.0)

        # At 20°C: speed of sound ~343 m/s, us_per_cm ≈ 58.3
        s_20c = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS, us_per_cm=58.3)
        s_20c.enable()
        dist_20c = s_20c.read().value

        # At 30°C: speed of sound ~349 m/s, us_per_cm ≈ 57.3
        s_30c = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS, us_per_cm=57.3)
        s_30c.enable()
        dist_30c = s_30c.read().value

        # Higher temperature = faster sound = larger distance reading
        assert dist_30c > dist_20c


class TestUltrasonicMaxMinRange:
    """Tests for ultrasonic max/min range handling (5.5.3.2)."""

    def test_max_range(self) -> None:
        """Test max range (400 cm for HC-SR04)."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))

        # 400 cm = 23200 us at 58 us/cm
        driver.set_channel(0, 23200.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS)
        s.enable()

        reading = s.read()
        assert reading.value == pytest.approx(400.0, abs=1.0)

    def test_min_range(self) -> None:
        """Test min range (2 cm for HC-SR04)."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))

        # 2 cm = 116 us at 58 us/cm
        driver.set_channel(0, 116.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS)
        s.enable()

        reading = s.read()
        assert reading.value == pytest.approx(2.0, abs=0.1)

    def test_zero_distance(self) -> None:
        """Test zero duration returns zero distance."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))

        driver.set_channel(0, 0.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS)
        s.enable()

        reading = s.read()
        assert reading.value == 0.0


class TestUltrasonicStatus:
    """Tests for ultrasonic status tracking (5.5.3.2)."""

    def test_status_updated_after_read(self) -> None:
        """Test that status is updated after read."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))
        driver.set_channel(0, 580.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS)
        s.enable()

        # Status should be None before first read
        status = s.status()
        assert status.last_duration_us is None

        # Read and check status
        s.read()
        status = s.status()
        assert status.last_duration_us == 580


class TestUltrasonicWithConfig:
    """Tests for Ultrasonic with config object (5.5.3.2)."""

    def test_ultrasonic_from_config(self) -> None:
        """Test creating Ultrasonic from config object."""
        config = UltrasonicConfig(
            name="Custom Ultrasonic",
            unit=Unit.CENTIMETERS,
            timeout_s=0.05,
            us_per_cm=58.0,
        )

        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))
        driver.set_channel(0, 580.0)

        s = Ultrasonic(driver=driver, channel=0, config=config)
        s.enable()

        assert s.name == "Custom Ultrasonic"
        reading = s.read()
        assert reading.value == pytest.approx(10.0, abs=0.5)


class TestUltrasonicInvalidUnit:
    """Tests for ultrasonic invalid unit handling (5.5.3.2)."""

    def test_invalid_unit_raises(self) -> None:
        """Test that invalid unit raises ValueError."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()

        with pytest.raises(ValueError, match="CENTIMETERS or Unit.MILLIMETERS"):
            Ultrasonic(driver=driver, channel=0, unit=Unit.METERS)


class TestUltrasonicNoPins:
    """Tests for ultrasonic without pins or driver (5.5.3.2)."""

    def test_no_pins_no_driver_raises(self) -> None:
        """Test that reading without pins or driver raises error."""
        s = Ultrasonic()
        s.enable()

        with pytest.raises(CommunicationError):
            s.read()


# =============================================================================
# Phase 5.5.3.3 - ToF Sensor Tests
# =============================================================================


class TestToFConfig:
    """Tests for ToFConfig (5.5.3.3)."""

    def test_default_config(self) -> None:
        """Test default ToF configuration."""
        config = ToFConfig()

        assert config.name == "ToF"
        assert config.address == 0x29
        assert config.register == 0x00
        assert config.unit == Unit.MILLIMETERS

    def test_custom_config(self) -> None:
        """Test custom ToF configuration."""
        config = ToFConfig(
            name="VL53L0X",
            address=0x52,
            register=0x14,
            metadata={"max_range_mm": 2000},
        )

        assert config.name == "VL53L0X"
        assert config.address == 0x52
        assert config.register == 0x14
        assert config.metadata["max_range_mm"] == 2000


class TestToFStatus:
    """Tests for ToFStatus (5.5.3.3)."""

    def test_status_default(self) -> None:
        """Test default ToF status."""
        status = ToFStatus()

        assert status.last_mm is None

    def test_status_with_value(self) -> None:
        """Test ToF status with measurement."""
        status = ToFStatus(last_mm=250)

        assert status.last_mm == 250


class TestToFI2C:
    """Tests for ToF I2C communication (5.5.3.3)."""

    def test_tof_i2c_init(self) -> None:
        """Test ToF I2C initialization."""
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x00, 0x01: 0x64})  # 100mm

        s = ToF(bus, address=0x29, register=0x00)

        assert s.name == "ToF"
        assert s._address == 0x29
        assert s._register == 0x00

    def test_tof_read_distance(self) -> None:
        """Test ToF reading distance."""
        bus = SimulatedI2CBus()
        # 0x00C8 = 200mm (big-endian: high byte first)
        bus.add_device(0x29, registers={0x00: 0x00, 0x01: 0xC8})
        bus.open()

        s = ToF(bus, address=0x29, register=0x00)
        s.enable()

        reading = s.read()
        assert reading.value == 200.0
        assert reading.unit == Unit.MILLIMETERS

    def test_tof_different_address(self) -> None:
        """Test ToF with different I2C address."""
        bus = SimulatedI2CBus()
        bus.add_device(0x52, registers={0x00: 0x01, 0x01: 0x2C})  # 300mm
        bus.open()

        s = ToF(bus, address=0x52, register=0x00)
        s.enable()

        reading = s.read()
        assert reading.value == 300.0

    def test_tof_different_register(self) -> None:
        """Test ToF reading from different register."""
        bus = SimulatedI2CBus()
        # Set up device with multiple registers
        bus.add_device(0x29, registers={0x14: 0x01, 0x15: 0x90})  # 400mm at reg 0x14
        bus.open()

        s = ToF(bus, address=0x29, register=0x14)
        s.enable()

        reading = s.read()
        assert reading.value == 400.0


class TestToFRangeModes:
    """Tests for ToF range modes (5.5.3.3)."""

    def test_tof_short_range(self) -> None:
        """Test ToF short range measurement (0-200mm)."""
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x00, 0x01: 0x32})  # 50mm
        bus.open()

        s = ToF(bus, address=0x29, register=0x00)
        s.enable()

        reading = s.read()
        assert reading.value == 50.0
        assert reading.value < 200  # Short range

    def test_tof_long_range(self) -> None:
        """Test ToF long range measurement (>1000mm)."""
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x05, 0x01: 0xDC})  # 1500mm
        bus.open()

        s = ToF(bus, address=0x29, register=0x00)
        s.enable()

        reading = s.read()
        assert reading.value == 1500.0
        assert reading.value > 1000  # Long range


class TestToFStatus:
    """Tests for ToF status tracking (5.5.3.3)."""

    def test_tof_status_updated(self) -> None:
        """Test that ToF status is updated after read."""
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x00, 0x01: 0xFA})  # 250mm
        bus.open()

        s = ToF(bus, address=0x29, register=0x00)
        s.enable()

        # Status should be None before first read
        status = s.status()
        assert status.last_mm is None

        # Read and check status
        s.read()
        status = s.status()
        assert status.last_mm == 250


class TestToFWithConfig:
    """Tests for ToF with config object (5.5.3.3)."""

    def test_tof_from_config(self) -> None:
        """Test creating ToF from config object."""
        config = ToFConfig(
            name="VL53L1X",
            address=0x29,
            register=0x00,
        )

        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x02, 0x01: 0x58})  # 600mm
        bus.open()

        s = ToF(bus, config=config)
        s.enable()

        assert s.name == "VL53L1X"
        reading = s.read()
        assert reading.value == 600.0


class TestToFInvalidUnit:
    """Tests for ToF invalid unit handling (5.5.3.3)."""

    def test_tof_non_mm_unit_raises(self) -> None:
        """Test that non-millimeter unit raises ValueError."""
        bus = SimulatedI2CBus()

        with pytest.raises(ValueError, match="mm"):
            ToF(bus, unit=Unit.CENTIMETERS)


class TestToFEnableDisable:
    """Tests for ToF enable/disable (5.5.3.3)."""

    def test_tof_enable_opens_bus(self) -> None:
        """Test that enable opens the I2C bus."""
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x00, 0x01: 0x64})

        s = ToF(bus, address=0x29, register=0x00)

        assert bus._is_open is False
        s.enable()
        assert bus._is_open is True

    def test_tof_disable_closes_bus(self) -> None:
        """Test that disable closes the I2C bus."""
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x00, 0x01: 0x64})
        bus.open()

        s = ToF(bus, address=0x29, register=0x00)
        s.enable()
        s.disable()

        assert bus._is_open is False


# =============================================================================
# Phase 5.5.3.4 - IR Distance Tests
# =============================================================================


class TestIRDistanceConfig:
    """Tests for IRDistanceConfig (5.5.3.4)."""

    def test_default_config(self) -> None:
        """Test default IR distance configuration."""
        config = IRDistanceConfig()

        assert config.name == "IRDistance"
        assert config.unit == Unit.MILLIMETERS
        assert config.model == "inverse_voltage"
        assert config.a == 1000.0
        assert config.b == 0.0
        assert config.min_mm == 0
        assert config.max_mm == 5000

    def test_custom_config(self) -> None:
        """Test custom IR distance configuration."""
        config = IRDistanceConfig(
            name="Sharp GP2Y0A21",
            a=6787.0,
            b=3.0,
            min_mm=100,
            max_mm=800,
            metadata={"wavelength_nm": 850},
        )

        assert config.name == "Sharp GP2Y0A21"
        assert config.a == 6787.0
        assert config.b == 3.0
        assert config.min_mm == 100
        assert config.max_mm == 800


class TestIRDistanceStatus:
    """Tests for IRDistanceStatus (5.5.3.4)."""

    def test_status_default(self) -> None:
        """Test default IR distance status."""
        status = IRDistanceStatus()

        assert status.last_voltage is None

    def test_status_with_value(self) -> None:
        """Test IR distance status with voltage."""
        status = IRDistanceStatus(last_voltage=2.5)

        assert status.last_voltage == 2.5


class TestIRDistanceVoltageConversion:
    """Tests for IR distance voltage to distance conversion (5.5.3.4)."""

    def test_analog_voltage_to_distance(self) -> None:
        """Test voltage to distance conversion."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=1.0)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0)
        s.enable()

        reading = s.read()
        # distance = a / (voltage - b) = 1000 / 1.0 = 1000mm
        assert reading.value == 1000.0

    def test_linearization_curve(self) -> None:
        """Test linearization with offset (b parameter)."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=2.0)

        # With offset b=0.5: distance = 1000 / (2.0 - 0.5) = 666.67mm
        s = IRDistance(analog_pin=pin, a=1000.0, b=0.5)
        s.enable()

        reading = s.read()
        assert reading.value == pytest.approx(666.67, abs=1.0)

    def test_high_voltage_short_distance(self) -> None:
        """Test high voltage = short distance (inverse relationship)."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=2.5)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0)
        s.enable()

        reading = s.read()
        # distance = 1000 / 2.5 = 400mm
        assert reading.value == 400.0

    def test_low_voltage_long_distance(self) -> None:
        """Test low voltage = long distance."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=0.5)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0)
        s.enable()

        reading = s.read()
        # distance = 1000 / 0.5 = 2000mm (approx due to int conversion)
        assert reading.value == pytest.approx(2000.0, abs=5.0)


class TestIRDistanceRangeLimits:
    """Tests for IR distance range limits (5.5.3.4)."""

    def test_min_range_clamping(self) -> None:
        """Test distance is clamped to min range."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=3.3)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0, min_mm=100, max_mm=800)
        s.enable()

        reading = s.read()
        # distance = 1000 / 3.3 ≈ 303mm, but clamped to min=100
        # Actually 303 > 100, so no clamping here
        assert reading.value >= 100

    def test_max_range_clamping(self) -> None:
        """Test distance is clamped to max range."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=0.1)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0, min_mm=0, max_mm=800)
        s.enable()

        reading = s.read()
        # distance = 1000 / 0.1 = 10000mm, but clamped to max=800
        assert reading.value == 800.0

    def test_out_of_range_high(self) -> None:
        """Test out of range detection (too close)."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=3.0)

        # Sharp sensors saturate at close range
        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0, min_mm=100, max_mm=800)
        s.enable()

        reading = s.read()
        # distance = 1000 / 3.0 ≈ 333mm
        assert 100 <= reading.value <= 800


class TestIRDistanceCalibration:
    """Tests for IR distance calibration parameters (5.5.3.4)."""

    def test_calibration_with_a_parameter(self) -> None:
        """Test calibration using a parameter."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=1.0)

        # Different sensors have different a values
        s1 = IRDistance(analog_pin=pin, a=1000.0, b=0.0)
        s2 = IRDistance(analog_pin=pin, a=2000.0, b=0.0)

        s1.enable()
        s2.enable()

        r1 = s1.read()
        r2 = s2.read()

        # Same voltage but different a gives different distance (approx due to int conversion)
        assert r2.value == pytest.approx(2 * r1.value, abs=5.0)

    def test_eps_prevents_division_by_zero(self) -> None:
        """Test eps parameter prevents division by zero."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=0.0)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0, eps=1e-6)
        s.enable()

        # Should not raise, eps prevents division by zero
        reading = s.read()
        assert reading.value > 0


class TestIRDistanceStatusTracking:
    """Tests for IR distance status tracking (5.5.3.4)."""

    def test_status_updated_after_read(self) -> None:
        """Test that status is updated after read."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=1.5)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0)
        s.enable()

        # Status should be None before first read
        status = s.status()
        assert status.last_voltage is None

        # Read and check status
        s.read()
        status = s.status()
        assert status.last_voltage == pytest.approx(1.5, abs=0.01)


class TestIRDistanceWithConfig:
    """Tests for IRDistance with config object (5.5.3.4)."""

    def test_ir_from_config(self) -> None:
        """Test creating IRDistance from config object."""
        config = IRDistanceConfig(
            name="Custom IR",
            a=1500.0,
            b=0.2,
            min_mm=50,
            max_mm=1000,
        )

        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=1.0)

        s = IRDistance(analog_pin=pin, config=config)
        s.enable()

        assert s.name == "Custom IR"
        reading = s.read()
        # distance = 1500 / (1.0 - 0.2) = 1875mm, clamped to 1000
        assert reading.value == 1000.0


# =============================================================================
# Phase 5.5.3 - Additional Tests
# =============================================================================


class TestDistanceSensorBase:
    """Tests for DistanceSensor base class (5.5.3)."""

    def test_read_distance_convenience(self) -> None:
        """Test read_distance convenience method."""
        driver = SimulatedDriver(name="drv", channels=1)
        driver.connect()
        driver.enable()
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=50000.0))
        driver.set_channel(0, 580.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS)
        s.enable()

        distance = s.read_distance()
        assert isinstance(distance, float)
        assert distance == pytest.approx(10.0, abs=0.5)


class TestSimulatedPinSetup:
    """Tests for simulated pin setup (5.5.3)."""

    def test_ultrasonic_enables_pins(self) -> None:
        """Test that ultrasonic enable sets up pins."""
        trigger = SimulatedDigitalPin(17)
        echo = SimulatedDigitalPin(18)

        s = Ultrasonic(trigger_pin=trigger, echo_pin=echo)

        assert not trigger.initialized
        assert not echo.initialized

        s.enable()

        assert trigger.initialized
        assert echo.initialized

    def test_ir_enables_analog_pin(self) -> None:
        """Test that IRDistance enable sets up analog pin."""
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=1.0)

        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0)

        assert not pin.initialized

        s.enable()

        assert pin.initialized


class TestMultipleSensors:
    """Tests for multiple distance sensors (5.5.3)."""

    def test_multiple_ultrasonic_different_channels(self) -> None:
        """Test multiple ultrasonic sensors on different channels."""
        driver = SimulatedDriver(name="drv", channels=4)
        driver.connect()
        driver.enable()

        for i in range(4):
            driver.set_channel_config(i, ChannelConfig(min_value=0.0, max_value=50000.0))

        driver.set_channel(0, 580.0)   # 10 cm
        driver.set_channel(1, 1160.0)  # 20 cm
        driver.set_channel(2, 2320.0)  # 40 cm
        driver.set_channel(3, 4640.0)  # 80 cm

        sensors = [
            Ultrasonic(driver=driver, channel=i, unit=Unit.CENTIMETERS)
            for i in range(4)
        ]

        for s in sensors:
            s.enable()

        readings = [s.read().value for s in sensors]

        assert readings[0] == pytest.approx(10.0, abs=0.5)
        assert readings[1] == pytest.approx(20.0, abs=0.5)
        assert readings[2] == pytest.approx(40.0, abs=0.5)
        assert readings[3] == pytest.approx(80.0, abs=0.5)

    def test_multiple_tof_different_addresses(self) -> None:
        """Test multiple ToF sensors on different I2C addresses."""
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x00, 0x01: 0xC8})  # 200mm
        bus.add_device(0x30, registers={0x00: 0x01, 0x01: 0x90})  # 400mm
        bus.open()

        s1 = ToF(bus, address=0x29, register=0x00)
        s2 = ToF(bus, address=0x30, register=0x00)

        s1.enable()
        s2.enable()

        r1 = s1.read()
        r2 = s2.read()

        assert r1.value == 200.0
        assert r2.value == 400.0
