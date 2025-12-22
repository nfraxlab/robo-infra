"""Unit tests for ESP32 platform implementation.

Tests ESP32Platform, ESP32DigitalPin, ESP32PWMPin, ESP32AnalogPin,
ESP32DACPin, ESP32TouchPin, and ESP32HallSensor in simulation mode.

Run: pytest tests/unit/test_platforms_esp32.py -v
"""

from __future__ import annotations

import pytest

from robo_infra.core.exceptions import HardwareNotFoundError
from robo_infra.core.pin import PinMode, PinState
from robo_infra.platforms.esp32 import (
    DEFAULT_ESP_CAPABILITIES,
    ESP32_CAPABILITIES,
    ESP32AnalogPin,
    ESP32Chip,
    ESP32ConnectionMode,
    ESP32DACPin,
    ESP32DigitalPin,
    ESP32HallSensor,
    ESP32Platform,
    ESP32PWMPin,
    ESP32TouchPin,
    MicroPythonREPL,
)


# =============================================================================
# ESP32 Chip Enum Tests
# =============================================================================


class TestESP32Chip:
    """Tests for ESP32Chip enum."""

    def test_chip_values(self) -> None:
        """Test chip value strings."""
        assert ESP32Chip.ESP8266.value == "esp8266"
        assert ESP32Chip.ESP32.value == "esp32"
        assert ESP32Chip.ESP32_S2.value == "esp32-s2"
        assert ESP32Chip.ESP32_S3.value == "esp32-s3"
        assert ESP32Chip.ESP32_C3.value == "esp32-c3"
        assert ESP32Chip.UNKNOWN.value == "unknown"

    def test_all_chips_defined(self) -> None:
        """Test all expected chips are defined."""
        expected_chips = ["ESP8266", "ESP32", "ESP32_S2", "ESP32_S3", "ESP32_C3", "UNKNOWN"]
        for chip_name in expected_chips:
            assert hasattr(ESP32Chip, chip_name)


class TestESP32ConnectionMode:
    """Tests for ESP32ConnectionMode enum."""

    def test_connection_modes(self) -> None:
        """Test connection mode values."""
        assert ESP32ConnectionMode.SERIAL.value == "serial"
        assert ESP32ConnectionMode.WEBREPL.value == "webrepl"
        assert ESP32ConnectionMode.BLE.value == "ble"


# =============================================================================
# Chip Capabilities Tests
# =============================================================================


class TestESP32Capabilities:
    """Tests for ESP32 chip capability constants."""

    def test_esp32_capabilities(self) -> None:
        """Test ESP32 chip capabilities."""
        caps = ESP32_CAPABILITIES[ESP32Chip.ESP32]
        assert len(caps["gpio_pins"]) == 40
        assert 32 in caps["adc_pins"]
        assert 25 in caps["dac_pins"]
        assert 4 in caps["touch_pins"]
        assert caps["pwm_channels"] == 16
        assert caps["adc_resolution"] == 12
        assert caps["dac_resolution"] == 8

    def test_esp32_s2_capabilities(self) -> None:
        """Test ESP32-S2 chip capabilities."""
        caps = ESP32_CAPABILITIES[ESP32Chip.ESP32_S2]
        assert len(caps["gpio_pins"]) == 46
        assert caps["adc_resolution"] == 13
        assert len(caps["dac_pins"]) == 2

    def test_esp32_s3_capabilities(self) -> None:
        """Test ESP32-S3 chip capabilities (no DAC)."""
        caps = ESP32_CAPABILITIES[ESP32Chip.ESP32_S3]
        assert len(caps["gpio_pins"]) == 49
        assert len(caps["dac_pins"]) == 0  # S3 has no DAC

    def test_esp32_c3_capabilities(self) -> None:
        """Test ESP32-C3 chip capabilities (no touch, no DAC)."""
        caps = ESP32_CAPABILITIES[ESP32Chip.ESP32_C3]
        assert len(caps["gpio_pins"]) == 22
        assert len(caps["dac_pins"]) == 0
        assert len(caps["touch_pins"]) == 0

    def test_esp8266_capabilities(self) -> None:
        """Test ESP8266 chip capabilities."""
        caps = ESP32_CAPABILITIES[ESP32Chip.ESP8266]
        assert len(caps["gpio_pins"]) == 11  # Limited GPIO
        assert len(caps["adc_pins"]) == 1  # Only ADC0
        assert caps["adc_resolution"] == 10

    def test_default_capabilities(self) -> None:
        """Test default capabilities is ESP32."""
        assert ESP32_CAPABILITIES[ESP32Chip.ESP32] == DEFAULT_ESP_CAPABILITIES


# =============================================================================
# ESP32DigitalPin Tests
# =============================================================================


class TestESP32DigitalPin:
    """Tests for ESP32DigitalPin class."""

    def test_create_output_pin(self) -> None:
        """Test creating an output pin."""
        pin = ESP32DigitalPin(2, mode=PinMode.OUTPUT, simulation=True)
        assert pin.number == 2
        assert pin.mode == PinMode.OUTPUT
        assert not pin.initialized

    def test_setup_simulation(self) -> None:
        """Test setup in simulation mode."""
        pin = ESP32DigitalPin(2, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_write_high(self) -> None:
        """Test writing HIGH to pin."""
        pin = ESP32DigitalPin(2, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(True)
        assert pin.state == PinState.HIGH
        assert pin.read() is True

    def test_write_low(self) -> None:
        """Test writing LOW to pin."""
        pin = ESP32DigitalPin(2, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(False)
        assert pin.state == PinState.LOW
        assert pin.read() is False

    def test_input_pulldown(self) -> None:
        """Test input pulldown mode (ESP32 supports this)."""
        pin = ESP32DigitalPin(4, mode=PinMode.INPUT_PULLDOWN, simulation=True)
        assert pin.mode == PinMode.INPUT_PULLDOWN

    def test_inverted_pin(self) -> None:
        """Test inverted pin logic."""
        pin = ESP32DigitalPin(2, mode=PinMode.OUTPUT, inverted=True, simulation=True)
        pin.setup()
        pin.write(True)
        # Read should return True (inverted back)

    def test_no_repl_raises_error(self) -> None:
        """Test that no REPL raises error in non-simulation mode."""
        pin = ESP32DigitalPin(2, mode=PinMode.OUTPUT, simulation=False)
        with pytest.raises(HardwareNotFoundError):
            pin.setup()


# =============================================================================
# ESP32PWMPin Tests
# =============================================================================


class TestESP32PWMPin:
    """Tests for ESP32PWMPin class."""

    def test_create_pwm_pin(self) -> None:
        """Test creating a PWM pin."""
        pin = ESP32PWMPin(5, simulation=True)
        assert pin.number == 5
        assert pin.duty_cycle == 0.0

    def test_setup(self) -> None:
        """Test PWM pin setup."""
        pin = ESP32PWMPin(5, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_set_duty_cycle(self) -> None:
        """Test setting duty cycle."""
        pin = ESP32PWMPin(5, simulation=True)
        pin.setup()
        pin.set_duty_cycle(0.75)
        assert pin.duty_cycle == 0.75

    def test_set_frequency(self) -> None:
        """Test setting frequency."""
        pin = ESP32PWMPin(5, frequency=5000, simulation=True)
        assert pin.frequency == 5000

        pin.set_frequency(10000)
        assert pin.frequency == 10000

    def test_frequency_clamped(self) -> None:
        """Test frequency clamping."""
        pin = ESP32PWMPin(5, simulation=True)
        pin.setup()

        pin.frequency = 50000000  # Too high
        assert pin.frequency == 40000000  # Clamped to max

        pin.frequency = -100  # Negative
        assert pin.frequency == 1  # Minimum

    def test_deinit(self) -> None:
        """Test PWM deinit."""
        pin = ESP32PWMPin(5, simulation=True)
        pin.setup()
        pin.deinit()  # Should not raise


# =============================================================================
# ESP32AnalogPin Tests
# =============================================================================


class TestESP32AnalogPin:
    """Tests for ESP32AnalogPin class."""

    def test_create_analog_pin(self) -> None:
        """Test creating an analog pin."""
        pin = ESP32AnalogPin(32, simulation=True)
        assert pin.number == 32

    def test_setup(self) -> None:
        """Test analog pin setup."""
        pin = ESP32AnalogPin(32, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_read_raw(self) -> None:
        """Test reading raw ADC value."""
        pin = ESP32AnalogPin(32, resolution=12, simulation=True)
        pin.setup()
        raw = pin.read_raw()
        assert 0 <= raw <= 4095  # 12-bit ADC

    def test_attenuation(self) -> None:
        """Test ADC attenuation setting."""
        pin = ESP32AnalogPin(32, attenuation=3, simulation=True)
        assert pin._attenuation == 3

    def test_3v3_reference(self) -> None:
        """Test 3.3V reference voltage."""
        pin = ESP32AnalogPin(32, reference_voltage=3.3, simulation=True)
        assert pin._reference_voltage == 3.3


# =============================================================================
# ESP32DACPin Tests
# =============================================================================


class TestESP32DACPin:
    """Tests for ESP32DACPin class."""

    def test_create_dac_pin_25(self) -> None:
        """Test creating DAC on GPIO25."""
        dac = ESP32DACPin(25, simulation=True)
        assert dac.number == 25

    def test_create_dac_pin_26(self) -> None:
        """Test creating DAC on GPIO26."""
        dac = ESP32DACPin(26, simulation=True)
        assert dac.number == 26

    def test_invalid_dac_pin(self) -> None:
        """Test that invalid DAC pin raises error."""
        with pytest.raises(ValueError, match="DAC only available"):
            ESP32DACPin(13, simulation=False)

    def test_setup(self) -> None:
        """Test DAC setup."""
        dac = ESP32DACPin(25, simulation=True)
        dac.setup()
        assert dac.initialized

    def test_write_value(self) -> None:
        """Test writing DAC value."""
        dac = ESP32DACPin(25, simulation=True)
        dac.setup()
        dac.write(128)
        assert dac.value == 128

    def test_value_clamped(self) -> None:
        """Test DAC value is clamped to 0-255."""
        dac = ESP32DACPin(25, simulation=True)
        dac.setup()

        dac.write(300)
        assert dac.value == 255

        dac.write(-10)
        assert dac.value == 0

    def test_write_voltage(self) -> None:
        """Test writing voltage."""
        dac = ESP32DACPin(25, simulation=True)
        dac.setup()

        dac.write_voltage(1.65, vref=3.3)  # Half voltage
        assert 125 <= dac.value <= 130

    def test_custom_name(self) -> None:
        """Test custom DAC name."""
        dac = ESP32DACPin(25, name="audio_out", simulation=True)
        assert dac.name == "audio_out"


# =============================================================================
# ESP32TouchPin Tests
# =============================================================================


class TestESP32TouchPin:
    """Tests for ESP32TouchPin class."""

    def test_create_touch_pin(self) -> None:
        """Test creating a touch pin."""
        touch = ESP32TouchPin(4, simulation=True)
        assert touch.number == 4

    def test_setup(self) -> None:
        """Test touch pin setup."""
        touch = ESP32TouchPin(4, simulation=True)
        touch.setup()
        assert touch.initialized

    def test_read(self) -> None:
        """Test reading touch value."""
        touch = ESP32TouchPin(4, simulation=True)
        touch.setup()
        value = touch.read()
        assert value == 500  # Simulated untouched value

    def test_is_touched_false(self) -> None:
        """Test is_touched returns False when not touched."""
        touch = ESP32TouchPin(4, simulation=True)
        touch.setup()
        # Simulated value is 500, threshold 100 -> not touched
        assert touch.is_touched() is False

    def test_is_touched_with_custom_threshold(self) -> None:
        """Test is_touched with custom threshold."""
        touch = ESP32TouchPin(4, threshold=600, simulation=True)
        touch.setup()
        # Simulated value is 500, threshold 600 -> touched
        assert touch.is_touched() is True

    def test_calibrate(self) -> None:
        """Test touch calibration."""
        touch = ESP32TouchPin(4, simulation=True)
        touch.setup()
        threshold = touch.calibrate(samples=5)
        assert threshold == 250  # Half of 500

    def test_custom_name(self) -> None:
        """Test custom touch pin name."""
        touch = ESP32TouchPin(4, name="button", simulation=True)
        assert touch.name == "button"


# =============================================================================
# ESP32HallSensor Tests
# =============================================================================


class TestESP32HallSensor:
    """Tests for ESP32HallSensor class."""

    def test_create_hall_sensor(self) -> None:
        """Test creating hall sensor."""
        hall = ESP32HallSensor(simulation=True)
        assert not hall.initialized

    def test_setup(self) -> None:
        """Test hall sensor setup."""
        hall = ESP32HallSensor(simulation=True)
        hall.setup()
        assert hall.initialized

    def test_read(self) -> None:
        """Test reading hall sensor value."""
        hall = ESP32HallSensor(simulation=True)
        hall.setup()
        value = hall.read()
        assert value == 0  # Simulated: no magnetic field

    def test_cleanup(self) -> None:
        """Test hall sensor cleanup."""
        hall = ESP32HallSensor(simulation=True)
        hall.setup()
        hall.cleanup()
        assert not hall.initialized


# =============================================================================
# MicroPythonREPL Tests
# =============================================================================


class TestMicroPythonREPL:
    """Tests for MicroPythonREPL class."""

    def test_create_repl(self) -> None:
        """Test creating REPL object."""
        repl = MicroPythonREPL(port="/dev/ttyUSB0")
        assert repl._port == "/dev/ttyUSB0"
        assert not repl.is_connected

    def test_webrepl_config(self) -> None:
        """Test WebREPL configuration."""
        repl = MicroPythonREPL(host="192.168.1.100", webrepl_password="secret")
        assert repl._host == "192.168.1.100"
        assert repl._password == "secret"

    def test_mode_initially_none(self) -> None:
        """Test mode is None before connection."""
        repl = MicroPythonREPL()
        assert repl.mode is None


# =============================================================================
# ESP32Platform Tests
# =============================================================================


class TestESP32Platform:
    """Tests for ESP32Platform class."""

    @pytest.fixture
    def platform(self, monkeypatch: pytest.MonkeyPatch) -> ESP32Platform:
        """Create platform in simulation mode."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        return ESP32Platform()

    def test_create_platform_simulation(self, platform: ESP32Platform) -> None:
        """Test creating platform in simulation mode."""
        assert platform.is_available
        assert platform._simulation

    def test_chip_property(self, platform: ESP32Platform) -> None:
        """Test chip property."""
        assert platform.chip == ESP32Chip.ESP32

    def test_explicit_chip(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """Test explicit chip setting."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        platform = ESP32Platform(chip=ESP32Chip.ESP32_S3)
        assert platform.chip == ESP32Chip.ESP32_S3

    def test_get_chip_capabilities(self, platform: ESP32Platform) -> None:
        """Test getting chip capabilities."""
        caps = platform.get_chip_capabilities()
        assert "gpio_pins" in caps
        assert "adc_pins" in caps
        assert "dac_pins" in caps
        assert "touch_pins" in caps

    def test_get_pin_output(self, platform: ESP32Platform) -> None:
        """Test getting an output pin."""
        pin = platform.get_pin(2, mode=PinMode.OUTPUT)
        assert isinstance(pin, ESP32DigitalPin)
        assert pin.mode == PinMode.OUTPUT

    def test_get_pin_pwm(self, platform: ESP32Platform) -> None:
        """Test getting a PWM pin."""
        pin = platform.get_pin(5, mode=PinMode.PWM)
        assert isinstance(pin, ESP32PWMPin)

    def test_get_analog_pin(self, platform: ESP32Platform) -> None:
        """Test getting an analog pin."""
        pin = platform.get_analog_pin(32)
        assert isinstance(pin, ESP32AnalogPin)

    def test_get_dac_pin(self, platform: ESP32Platform) -> None:
        """Test getting a DAC pin."""
        dac = platform.get_dac_pin(25)
        assert isinstance(dac, ESP32DACPin)

    def test_get_dac_cached(self, platform: ESP32Platform) -> None:
        """Test that DAC pins are cached."""
        dac1 = platform.get_dac_pin(25)
        dac2 = platform.get_dac_pin(25)
        assert dac1 is dac2

    def test_get_touch_pin(self, platform: ESP32Platform) -> None:
        """Test getting a touch pin."""
        touch = platform.get_touch_pin(4)
        assert isinstance(touch, ESP32TouchPin)

    def test_get_touch_cached(self, platform: ESP32Platform) -> None:
        """Test that touch pins are cached."""
        touch1 = platform.get_touch_pin(4)
        touch2 = platform.get_touch_pin(4)
        assert touch1 is touch2

    def test_get_hall_sensor(self, platform: ESP32Platform) -> None:
        """Test getting hall sensor."""
        hall = platform.get_hall_sensor()
        assert isinstance(hall, ESP32HallSensor)

    def test_get_hall_sensor_cached(self, platform: ESP32Platform) -> None:
        """Test that hall sensor is cached."""
        hall1 = platform.get_hall_sensor()
        hall2 = platform.get_hall_sensor()
        assert hall1 is hall2

    def test_connection_mode(self, platform: ESP32Platform) -> None:
        """Test connection mode property."""
        # In simulation, no actual connection
        assert platform.connection_mode is None

    def test_soft_reset_simulation(self, platform: ESP32Platform) -> None:
        """Test soft reset in simulation."""
        platform.soft_reset()  # Should not raise

    def test_cleanup(self, platform: ESP32Platform) -> None:
        """Test platform cleanup."""
        pin = platform.get_pin(2, mode=PinMode.OUTPUT)
        pin.setup()
        dac = platform.get_dac_pin(25)
        dac.setup()
        touch = platform.get_touch_pin(4)
        touch.setup()

        platform.cleanup()
        # Should not raise


# =============================================================================
# Integration-style Tests (Simulation)
# =============================================================================


class TestESP32Integration:
    """Integration-style tests for ESP32 platform."""

    @pytest.fixture
    def platform(self, monkeypatch: pytest.MonkeyPatch) -> ESP32Platform:
        """Create platform in simulation mode."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        return ESP32Platform()

    def test_led_control(self, platform: ESP32Platform) -> None:
        """Test LED control on GPIO2 (common ESP32 LED pin)."""
        led = platform.get_pin(2, mode=PinMode.OUTPUT)

        led.high()
        assert led.read() is True

        led.low()
        assert led.read() is False

    def test_pwm_motor_control(self, platform: ESP32Platform) -> None:
        """Test PWM motor control."""
        motor = platform.get_pin(5, mode=PinMode.PWM, frequency=5000)
        assert isinstance(motor, ESP32PWMPin)

        for speed in [0.0, 0.25, 0.5, 0.75, 1.0]:
            motor.set_duty_cycle(speed)
            assert motor.duty_cycle == speed

    def test_dac_audio_output(self, platform: ESP32Platform) -> None:
        """Test DAC for audio-like output."""
        dac = platform.get_dac_pin(25)
        dac.setup()

        # Simulate waveform
        for value in [0, 64, 128, 192, 255, 192, 128, 64]:
            dac.write(value)
            assert dac.value == value

    def test_touch_button(self, platform: ESP32Platform) -> None:
        """Test touch button reading."""
        touch = platform.get_touch_pin(4)
        touch.setup()

        # Read and check threshold
        value = touch.read()
        assert isinstance(value, int)
        assert value > 0

    def test_hall_magnetic_detection(self, platform: ESP32Platform) -> None:
        """Test hall sensor for magnetic field detection."""
        hall = platform.get_hall_sensor()
        hall.setup()

        value = hall.read()
        assert isinstance(value, int)

    def test_multi_pin_control(self, platform: ESP32Platform) -> None:
        """Test controlling multiple pins."""
        led = platform.get_pin(2, mode=PinMode.OUTPUT)
        pwm = platform.get_pin(5, mode=PinMode.PWM)
        adc = platform.get_analog_pin(32)
        dac = platform.get_dac_pin(25)
        touch = platform.get_touch_pin(4)

        # Control all pins
        led.high()
        pwm.set_duty_cycle(0.5)
        adc.setup()
        adc.read()
        dac.write(128)
        touch.setup()
        touch.read()

        assert led.read() is True
        assert pwm.duty_cycle == 0.5
        assert dac.value == 128
