"""Unit tests for Arduino platform implementation.

Tests ArduinoPlatform, ArduinoDigitalPin, ArduinoPWMPin, ArduinoAnalogPin,
and ArduinoServoPin classes in simulation mode.

Run: pytest tests/unit/test_platforms_arduino.py -v
"""

from __future__ import annotations

import pytest

from robo_infra.core.exceptions import HardwareNotFoundError
from robo_infra.core.pin import PinMode, PinState
from robo_infra.platforms.arduino import (
    ARDUINO_USB_IDS,
    BOARD_CAPABILITIES,
    DEFAULT_CAPABILITIES,
    ArduinoAnalogPin,
    ArduinoBoard,
    ArduinoDigitalPin,
    ArduinoPlatform,
    ArduinoPWMPin,
    ArduinoServoPin,
    FirmataCommand,
)


# =============================================================================
# Arduino Board Enum Tests
# =============================================================================


class TestArduinoBoard:
    """Tests for ArduinoBoard enum."""

    def test_board_values(self) -> None:
        """Test board value strings."""
        assert ArduinoBoard.UNO.value == "uno"
        assert ArduinoBoard.NANO.value == "nano"
        assert ArduinoBoard.MEGA.value == "mega"
        assert ArduinoBoard.LEONARDO.value == "leonardo"
        assert ArduinoBoard.DUE.value == "due"
        assert ArduinoBoard.UNKNOWN.value == "unknown"

    def test_all_boards_defined(self) -> None:
        """Test all expected boards are defined."""
        expected_boards = ["UNO", "NANO", "MINI", "MEGA", "MEGA_ADK", "LEONARDO", "MICRO", "DUE", "ZERO", "MKR1000", "UNKNOWN"]
        for board_name in expected_boards:
            assert hasattr(ArduinoBoard, board_name)


class TestFirmataCommand:
    """Tests for FirmataCommand enum."""

    def test_command_values(self) -> None:
        """Test Firmata protocol command values."""
        assert FirmataCommand.DIGITAL_MESSAGE.value == 0x90
        assert FirmataCommand.ANALOG_MESSAGE.value == 0xE0
        assert FirmataCommand.SET_PIN_MODE.value == 0xF4
        assert FirmataCommand.SYSTEM_RESET.value == 0xFF
        assert FirmataCommand.START_SYSEX.value == 0xF0
        assert FirmataCommand.END_SYSEX.value == 0xF7


# =============================================================================
# Board Capabilities Tests
# =============================================================================


class TestBoardCapabilities:
    """Tests for board capability constants."""

    def test_uno_capabilities(self) -> None:
        """Test Uno board capabilities."""
        caps = BOARD_CAPABILITIES[ArduinoBoard.UNO]
        assert len(caps["digital_pins"]) == 14  # D0-D13
        assert len(caps["analog_pins"]) == 6  # A0-A5
        assert 9 in caps["pwm_pins"]
        assert caps["adc_resolution"] == 10
        assert caps["pwm_resolution"] == 8

    def test_mega_capabilities(self) -> None:
        """Test Mega board capabilities."""
        caps = BOARD_CAPABILITIES[ArduinoBoard.MEGA]
        assert len(caps["digital_pins"]) == 54
        assert len(caps["analog_pins"]) == 16
        assert len(caps["pwm_pins"]) == 15
        assert caps["adc_resolution"] == 10

    def test_due_capabilities(self) -> None:
        """Test Due board capabilities."""
        caps = BOARD_CAPABILITIES[ArduinoBoard.DUE]
        assert len(caps["digital_pins"]) == 54
        assert caps["adc_resolution"] == 12  # 12-bit ADC
        assert "dac_pins" in caps
        assert len(caps["dac_pins"]) == 2

    def test_default_capabilities(self) -> None:
        """Test default capabilities fallback."""
        assert len(DEFAULT_CAPABILITIES["digital_pins"]) == 14
        assert len(DEFAULT_CAPABILITIES["analog_pins"]) == 6
        assert DEFAULT_CAPABILITIES["adc_resolution"] == 10


class TestArduinoUSBIDs:
    """Tests for Arduino USB VID/PID detection."""

    def test_uno_usb_ids(self) -> None:
        """Test Uno USB IDs are registered."""
        assert (0x2341, 0x0043) in ARDUINO_USB_IDS
        assert ARDUINO_USB_IDS[(0x2341, 0x0043)] == ArduinoBoard.UNO

    def test_mega_usb_ids(self) -> None:
        """Test Mega USB IDs are registered."""
        assert (0x2341, 0x0010) in ARDUINO_USB_IDS
        assert ARDUINO_USB_IDS[(0x2341, 0x0010)] == ArduinoBoard.MEGA

    def test_generic_ch340_id(self) -> None:
        """Test generic CH340 ID maps to UNKNOWN."""
        assert (0x1A86, 0x7523) in ARDUINO_USB_IDS
        assert ARDUINO_USB_IDS[(0x1A86, 0x7523)] == ArduinoBoard.UNKNOWN


# =============================================================================
# ArduinoDigitalPin Tests
# =============================================================================


class TestArduinoDigitalPin:
    """Tests for ArduinoDigitalPin class."""

    def test_create_output_pin(self) -> None:
        """Test creating an output pin."""
        pin = ArduinoDigitalPin(13, mode=PinMode.OUTPUT, simulation=True)
        assert pin.number == 13
        assert pin.mode == PinMode.OUTPUT
        assert not pin.initialized

    def test_setup_simulation(self) -> None:
        """Test setup in simulation mode."""
        pin = ArduinoDigitalPin(13, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_write_high(self) -> None:
        """Test writing HIGH to pin."""
        pin = ArduinoDigitalPin(13, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(True)
        assert pin.state == PinState.HIGH
        assert pin.read() is True

    def test_write_low(self) -> None:
        """Test writing LOW to pin."""
        pin = ArduinoDigitalPin(13, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(False)
        assert pin.state == PinState.LOW
        assert pin.read() is False

    def test_high_low_methods(self) -> None:
        """Test high() and low() convenience methods."""
        pin = ArduinoDigitalPin(13, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()

        pin.high()
        assert pin.read() is True

        pin.low()
        assert pin.read() is False

    def test_inverted_pin(self) -> None:
        """Test inverted pin logic."""
        pin = ArduinoDigitalPin(13, mode=PinMode.OUTPUT, inverted=True, simulation=True)
        pin.setup()

        pin.write(True)  # Write True but inverted
        # Internal state should be LOW because inverted
        # But read should return True

    def test_initial_state_high(self) -> None:
        """Test pin with initial state HIGH."""
        pin = ArduinoDigitalPin(
            13, mode=PinMode.OUTPUT, initial=PinState.HIGH, simulation=True
        )
        pin.setup()
        assert pin.read() is True

    def test_input_pin(self) -> None:
        """Test input pin mode."""
        pin = ArduinoDigitalPin(7, mode=PinMode.INPUT, simulation=True)
        pin.setup()
        # In simulation, default is False
        assert pin.read() is False

    def test_input_pullup(self) -> None:
        """Test input pullup mode."""
        pin = ArduinoDigitalPin(7, mode=PinMode.INPUT_PULLUP, simulation=True)
        assert pin.mode == PinMode.INPUT_PULLUP

    def test_custom_name(self) -> None:
        """Test custom pin name."""
        pin = ArduinoDigitalPin(13, name="LED", simulation=True)
        assert pin.name == "LED"

    def test_default_name(self) -> None:
        """Test default pin name."""
        pin = ArduinoDigitalPin(13, simulation=True)
        assert "13" in pin.name

    def test_cleanup(self) -> None:
        """Test pin cleanup."""
        pin = ArduinoDigitalPin(13, simulation=True)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized

    def test_no_board_raises_error(self) -> None:
        """Test that no board raises error in non-simulation mode."""
        pin = ArduinoDigitalPin(13, mode=PinMode.OUTPUT, simulation=False)
        with pytest.raises(HardwareNotFoundError):
            pin.setup()


# =============================================================================
# ArduinoPWMPin Tests
# =============================================================================


class TestArduinoPWMPin:
    """Tests for ArduinoPWMPin class."""

    def test_create_pwm_pin(self) -> None:
        """Test creating a PWM pin."""
        pin = ArduinoPWMPin(9, simulation=True)
        assert pin.number == 9
        assert pin.duty_cycle == 0.0

    def test_setup(self) -> None:
        """Test PWM pin setup."""
        pin = ArduinoPWMPin(9, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_set_duty_cycle(self) -> None:
        """Test setting duty cycle."""
        pin = ArduinoPWMPin(9, simulation=True)
        pin.setup()
        pin.set_duty_cycle(0.5)
        assert pin.duty_cycle == 0.5

    def test_duty_cycle_clamped(self) -> None:
        """Test duty cycle is clamped to 0-1."""
        pin = ArduinoPWMPin(9, simulation=True)
        pin.setup()

        pin.duty_cycle = 1.5
        assert pin.duty_cycle == 1.0

        pin.duty_cycle = -0.5
        assert pin.duty_cycle == 0.0

    def test_set_frequency(self) -> None:
        """Test setting frequency."""
        pin = ArduinoPWMPin(9, frequency=1000, simulation=True)
        assert pin.frequency == 1000

        pin.set_frequency(2000)
        assert pin.frequency == 2000

    def test_set_pulse_width(self) -> None:
        """Test setting pulse width in microseconds."""
        pin = ArduinoPWMPin(9, frequency=1000, simulation=True)  # 1kHz = 1ms period
        pin.setup()
        pin.set_pulse_width(500)  # 500us = 50% duty
        assert 0.49 < pin.duty_cycle < 0.51

    def test_start_stop(self) -> None:
        """Test start and stop methods."""
        pin = ArduinoPWMPin(9, duty_cycle=0.5, simulation=True)
        pin.start()  # Should setup and run
        assert pin.initialized

        pin.stop()  # Should set duty to 0
        assert pin.duty_cycle == 0.0

    def test_custom_name(self) -> None:
        """Test custom pin name."""
        pin = ArduinoPWMPin(9, name="motor", simulation=True)
        assert pin.name == "motor"

    def test_cleanup(self) -> None:
        """Test PWM cleanup."""
        pin = ArduinoPWMPin(9, simulation=True)
        pin.setup()
        pin.cleanup()
        assert not pin.initialized


# =============================================================================
# ArduinoAnalogPin Tests
# =============================================================================


class TestArduinoAnalogPin:
    """Tests for ArduinoAnalogPin class."""

    def test_create_analog_pin(self) -> None:
        """Test creating an analog pin."""
        pin = ArduinoAnalogPin(0, simulation=True)
        assert pin.number == 0

    def test_setup(self) -> None:
        """Test analog pin setup."""
        pin = ArduinoAnalogPin(0, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_read_raw(self) -> None:
        """Test reading raw ADC value."""
        pin = ArduinoAnalogPin(0, resolution=10, simulation=True)
        pin.setup()
        raw = pin.read_raw()
        assert 0 <= raw <= 1023  # 10-bit ADC

    def test_read_voltage(self) -> None:
        """Test reading voltage."""
        pin = ArduinoAnalogPin(0, reference_voltage=5.0, simulation=True)
        pin.setup()
        voltage = pin.read()
        assert 0.0 <= voltage <= 5.0

    def test_custom_resolution(self) -> None:
        """Test custom ADC resolution."""
        pin = ArduinoAnalogPin(0, resolution=12, simulation=True)
        assert pin._resolution == 12
        assert pin._max_value == 4095

    def test_custom_reference(self) -> None:
        """Test custom reference voltage."""
        pin = ArduinoAnalogPin(0, reference_voltage=3.3, simulation=True)
        assert pin._reference_voltage == 3.3

    def test_cleanup(self) -> None:
        """Test analog pin cleanup."""
        pin = ArduinoAnalogPin(0, simulation=True)
        pin.setup()
        pin.cleanup()
        assert not pin.initialized


# =============================================================================
# ArduinoServoPin Tests
# =============================================================================


class TestArduinoServoPin:
    """Tests for ArduinoServoPin class."""

    def test_create_servo_pin(self) -> None:
        """Test creating a servo pin."""
        servo = ArduinoServoPin(9, simulation=True)
        assert servo.number == 9
        assert servo.angle == 90.0  # Default center

    def test_setup(self) -> None:
        """Test servo pin setup."""
        servo = ArduinoServoPin(9, simulation=True)
        servo.setup()
        assert servo.initialized

    def test_write_angle(self) -> None:
        """Test writing angle."""
        servo = ArduinoServoPin(9, simulation=True)
        servo.setup()
        servo.write(45)
        assert servo.angle == 45.0

    def test_angle_clamped(self) -> None:
        """Test angle is clamped to 0-180."""
        servo = ArduinoServoPin(9, simulation=True)
        servo.setup()

        servo.write(200)
        assert servo.angle == 180.0

        servo.write(-20)
        assert servo.angle == 0.0

    def test_write_microseconds(self) -> None:
        """Test writing pulse width in microseconds."""
        servo = ArduinoServoPin(9, min_pulse=544, max_pulse=2400, simulation=True)
        servo.setup()

        # Center pulse (middle of range)
        servo.write_microseconds(1472)  # (544 + 2400) / 2
        assert 88 < servo.angle < 92  # ~90 degrees

    def test_custom_name(self) -> None:
        """Test custom servo name."""
        servo = ArduinoServoPin(9, name="arm", simulation=True)
        assert servo.name == "arm"

    def test_detach(self) -> None:
        """Test servo detach."""
        servo = ArduinoServoPin(9, simulation=True)
        servo.setup()
        servo.detach()  # Should not raise

    def test_cleanup(self) -> None:
        """Test servo cleanup."""
        servo = ArduinoServoPin(9, simulation=True)
        servo.setup()
        servo.cleanup()
        assert not servo.initialized


# =============================================================================
# ArduinoPlatform Tests
# =============================================================================


class TestArduinoPlatform:
    """Tests for ArduinoPlatform class."""

    @pytest.fixture
    def platform(self, monkeypatch: pytest.MonkeyPatch) -> ArduinoPlatform:
        """Create platform in simulation mode."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        return ArduinoPlatform()

    def test_create_platform_simulation(self, platform: ArduinoPlatform) -> None:
        """Test creating platform in simulation mode."""
        assert platform.is_available
        assert platform._simulation

    def test_board_type(self, platform: ArduinoPlatform) -> None:
        """Test board type property."""
        # In simulation, defaults to UNO
        assert platform.board_type == ArduinoBoard.UNO

    def test_explicit_board_type(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """Test explicit board type setting."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        platform = ArduinoPlatform(board_type=ArduinoBoard.MEGA)
        assert platform.board_type == ArduinoBoard.MEGA

    def test_get_board_capabilities(self, platform: ArduinoPlatform) -> None:
        """Test getting board capabilities."""
        caps = platform.get_board_capabilities()
        assert "digital_pins" in caps
        assert "analog_pins" in caps
        assert "pwm_pins" in caps

    def test_get_pin_output(self, platform: ArduinoPlatform) -> None:
        """Test getting an output pin."""
        pin = platform.get_pin(13, mode=PinMode.OUTPUT)
        assert isinstance(pin, ArduinoDigitalPin)
        assert pin.mode == PinMode.OUTPUT

    def test_get_pin_input(self, platform: ArduinoPlatform) -> None:
        """Test getting an input pin."""
        pin = platform.get_pin(7, mode=PinMode.INPUT)
        assert isinstance(pin, ArduinoDigitalPin)
        assert pin.mode == PinMode.INPUT

    def test_get_pin_pwm(self, platform: ArduinoPlatform) -> None:
        """Test getting a PWM pin."""
        pin = platform.get_pin(9, mode=PinMode.PWM)
        assert isinstance(pin, ArduinoPWMPin)

    def test_get_analog_pin(self, platform: ArduinoPlatform) -> None:
        """Test getting an analog pin."""
        pin = platform.get_analog_pin(0)
        assert isinstance(pin, ArduinoAnalogPin)
        assert pin.number == 0

    def test_get_analog_pin_by_string(self, platform: ArduinoPlatform) -> None:
        """Test getting analog pin by string like 'A0'."""
        pin = platform.get_pin("A0")
        assert isinstance(pin, ArduinoAnalogPin)

    def test_get_servo(self, platform: ArduinoPlatform) -> None:
        """Test getting a servo pin."""
        servo = platform.get_servo(9)
        assert isinstance(servo, ArduinoServoPin)
        assert servo.number == 9

    def test_get_servo_cached(self, platform: ArduinoPlatform) -> None:
        """Test that servos are cached."""
        servo1 = platform.get_servo(9)
        servo2 = platform.get_servo(9)
        assert servo1 is servo2

    def test_platform_info(self, platform: ArduinoPlatform) -> None:
        """Test getting platform info."""
        info = platform.info
        assert info.model == "uno"

    def test_cleanup(self, platform: ArduinoPlatform) -> None:
        """Test platform cleanup."""
        pin = platform.get_pin(13, mode=PinMode.OUTPUT)
        pin.setup()
        servo = platform.get_servo(9)
        servo.setup()

        platform.cleanup()
        # Should not raise

    def test_bus_not_supported(self, platform: ArduinoPlatform) -> None:
        """Test that bus creation raises error."""
        with pytest.raises(HardwareNotFoundError):
            platform._create_bus("i2c")


# =============================================================================
# Integration-style Tests (Simulation)
# =============================================================================


class TestArduinoIntegration:
    """Integration-style tests for Arduino platform."""

    @pytest.fixture
    def platform(self, monkeypatch: pytest.MonkeyPatch) -> ArduinoPlatform:
        """Create platform in simulation mode."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        return ArduinoPlatform()

    def test_led_blink_sequence(self, platform: ArduinoPlatform) -> None:
        """Test LED blink sequence."""
        led = platform.get_pin(13, mode=PinMode.OUTPUT)

        # Blink sequence
        led.high()
        assert led.read() is True

        led.low()
        assert led.read() is False

        led.toggle()
        assert led.read() is True

    def test_motor_speed_control(self, platform: ArduinoPlatform) -> None:
        """Test motor speed control with PWM."""
        motor = platform.get_pin(9, mode=PinMode.PWM)
        assert isinstance(motor, ArduinoPWMPin)

        # Speed ramp
        for speed in [0.0, 0.25, 0.5, 0.75, 1.0]:
            motor.set_duty_cycle(speed)
            assert motor.duty_cycle == speed

    def test_servo_sweep(self, platform: ArduinoPlatform) -> None:
        """Test servo sweep motion."""
        servo = platform.get_servo(10)

        # Sweep from 0 to 180
        for angle in range(0, 181, 30):
            servo.write(angle)
            assert servo.angle == float(angle)

    def test_analog_sensor_read(self, platform: ArduinoPlatform) -> None:
        """Test reading analog sensor."""
        sensor = platform.get_analog_pin(0)

        # Read multiple times
        readings = [sensor.read() for _ in range(5)]
        assert all(0.0 <= r <= 5.0 for r in readings)

    def test_multiple_pins(self, platform: ArduinoPlatform) -> None:
        """Test using multiple pins simultaneously."""
        led1 = platform.get_pin(12, mode=PinMode.OUTPUT)
        led2 = platform.get_pin(13, mode=PinMode.OUTPUT)
        button = platform.get_pin(7, mode=PinMode.INPUT)
        motor = platform.get_pin(9, mode=PinMode.PWM)
        servo = platform.get_servo(10)

        # Control all
        led1.high()
        led2.low()
        button.read()
        motor.set_duty_cycle(0.5)
        servo.write(90)

        assert led1.read() is True
        assert led2.read() is False
        assert motor.duty_cycle == 0.5
        assert servo.angle == 90.0
