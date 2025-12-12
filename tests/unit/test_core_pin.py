"""Unit tests for pin abstractions."""

from robo_infra.core.pin import (
    AnalogPin,
    DigitalPin,
    PinMode,
    PinState,
    PWMPin,
    SimulatedAnalogPin,
    SimulatedDigitalPin,
    SimulatedPWMPin,
    detect_platform,
    get_analog_pin,
    get_digital_pin,
    get_pwm_pin,
)


class TestPinMode:
    """Test PinMode enum."""

    def test_modes_exist(self) -> None:
        """Test all modes exist."""
        assert PinMode.INPUT
        assert PinMode.OUTPUT
        assert PinMode.INPUT_PULLUP
        assert PinMode.INPUT_PULLDOWN
        assert PinMode.PWM
        assert PinMode.ANALOG


class TestPinState:
    """Test PinState enum."""

    def test_states_exist(self) -> None:
        """Test states exist."""
        assert PinState.LOW.value == 0
        assert PinState.HIGH.value == 1


class TestSimulatedDigitalPin:
    """Test SimulatedDigitalPin."""

    def test_creation(self) -> None:
        """Test pin creation."""
        pin = SimulatedDigitalPin(17, PinMode.OUTPUT, name="led")
        assert pin.number == 17
        assert pin.mode == PinMode.OUTPUT
        assert pin.name == "led"
        assert not pin.initialized

    def test_setup_cleanup(self) -> None:
        """Test setup and cleanup."""
        pin = SimulatedDigitalPin(17)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized

    def test_write_read(self) -> None:
        """Test write and read operations."""
        pin = SimulatedDigitalPin(17, PinMode.OUTPUT)
        pin.setup()

        pin.write(True)
        assert pin.read() is True
        assert pin.state == PinState.HIGH

        pin.write(False)
        assert pin.read() is False
        assert pin.state == PinState.LOW

    def test_high_low(self) -> None:
        """Test high() and low() methods."""
        pin = SimulatedDigitalPin(17)
        pin.setup()

        pin.high()
        assert pin.read() is True

        pin.low()
        assert pin.read() is False

    def test_toggle(self) -> None:
        """Test toggle method."""
        pin = SimulatedDigitalPin(17)
        pin.setup()

        pin.low()
        assert pin.read() is False

        pin.toggle()
        assert pin.read() is True

        pin.toggle()
        assert pin.read() is False

    def test_inverted(self) -> None:
        """Test inverted logic."""
        pin = SimulatedDigitalPin(17, inverted=True)
        pin.setup()

        # Write True, but inverted means internal state is False
        pin.write(True)
        assert pin.read() is True  # Read should invert back

        pin.write(False)
        assert pin.read() is False

    def test_on_off_aliases(self) -> None:
        """Test on() and off() aliases."""
        pin = SimulatedDigitalPin(17)
        pin.setup()

        pin.on()
        assert pin.read() is True

        pin.off()
        assert pin.read() is False

    def test_repr(self) -> None:
        """Test string representation."""
        pin = SimulatedDigitalPin(17, name="test_pin")
        repr_str = repr(pin)
        assert "17" in repr_str
        assert "output" in repr_str
        assert "test_pin" in repr_str


class TestSimulatedPWMPin:
    """Test SimulatedPWMPin."""

    def test_creation(self) -> None:
        """Test PWM pin creation."""
        pin = SimulatedPWMPin(18, name="servo", frequency=50)
        assert pin.number == 18
        assert pin.mode == PinMode.PWM
        assert pin.frequency == 50
        assert pin.duty_cycle == 0.0

    def test_setup_cleanup(self) -> None:
        """Test setup and cleanup."""
        pin = SimulatedPWMPin(18)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized

    def test_duty_cycle(self) -> None:
        """Test duty cycle operations."""
        pin = SimulatedPWMPin(18)
        pin.setup()

        pin.set_duty_cycle(0.5)
        assert pin.duty_cycle == 0.5

        pin.set_duty_cycle(1.0)
        assert pin.duty_cycle == 1.0

        # Clamping
        pin.set_duty_cycle(1.5)
        assert pin.duty_cycle == 1.0

        pin.set_duty_cycle(-0.5)
        assert pin.duty_cycle == 0.0

    def test_frequency(self) -> None:
        """Test frequency setting."""
        pin = SimulatedPWMPin(18, frequency=50)
        pin.setup()

        pin.set_frequency(100)
        assert pin.frequency == 100

    def test_start_stop(self) -> None:
        """Test start and stop."""
        pin = SimulatedPWMPin(18)
        pin.setup()

        assert not pin.running

        pin.start()
        assert pin.running

        pin.stop()
        assert not pin.running

    def test_pulse_width(self) -> None:
        """Test pulse width setting (servo control)."""
        pin = SimulatedPWMPin(18, frequency=50)  # 50Hz = 20ms period
        pin.setup()

        # 1500us pulse at 50Hz should be 7.5% duty cycle
        pin.set_pulse_width(1500)
        assert abs(pin.duty_cycle - 0.075) < 0.001

        # 1000us = 5%
        pin.set_pulse_width(1000)
        assert abs(pin.duty_cycle - 0.05) < 0.001

        # 2000us = 10%
        pin.set_pulse_width(2000)
        assert abs(pin.duty_cycle - 0.1) < 0.001


class TestSimulatedAnalogPin:
    """Test SimulatedAnalogPin."""

    def test_creation(self) -> None:
        """Test analog pin creation."""
        pin = SimulatedAnalogPin(0, name="sensor", resolution=12, reference_voltage=3.3)
        assert pin.number == 0
        assert pin.mode == PinMode.ANALOG
        assert pin.resolution == 12
        assert pin.reference_voltage == 3.3
        assert pin.max_value == 4095  # 2^12 - 1

    def test_setup_cleanup(self) -> None:
        """Test setup and cleanup."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized

    def test_read_raw(self) -> None:
        """Test raw ADC reading."""
        pin = SimulatedAnalogPin(0, resolution=12, reference_voltage=3.3)
        pin.setup()

        pin.set_simulated_voltage(0.0)
        assert pin.read_raw() == 0

        pin.set_simulated_voltage(3.3)
        assert pin.read_raw() == 4095

        pin.set_simulated_voltage(1.65)  # Half of 3.3V
        assert abs(pin.read_raw() - 2047) <= 1  # Allow rounding

    def test_read_voltage(self) -> None:
        """Test voltage reading."""
        pin = SimulatedAnalogPin(0, resolution=12, reference_voltage=3.3)
        pin.setup()

        pin.set_simulated_voltage(1.65)
        voltage = pin.read()
        assert abs(voltage - 1.65) < 0.01

    def test_read_normalized(self) -> None:
        """Test normalized reading."""
        pin = SimulatedAnalogPin(0, resolution=12, reference_voltage=3.3)
        pin.setup()

        pin.set_simulated_voltage(0.0)
        assert pin.read_normalized() == 0.0

        pin.set_simulated_voltage(3.3)
        assert abs(pin.read_normalized() - 1.0) < 0.001

        pin.set_simulated_voltage(1.65)
        assert abs(pin.read_normalized() - 0.5) < 0.01

    def test_set_simulated_normalized(self) -> None:
        """Test setting value via normalized input."""
        pin = SimulatedAnalogPin(0, resolution=12, reference_voltage=3.3)
        pin.setup()

        pin.set_simulated_normalized(0.5)
        assert abs(pin.read() - 1.65) < 0.01

        pin.set_simulated_normalized(1.0)
        assert abs(pin.read() - 3.3) < 0.01


class TestPinFactories:
    """Test pin factory functions."""

    def test_detect_platform_returns_simulation(self) -> None:
        """Test platform detection defaults to simulation on non-embedded."""
        # On a regular computer, should return 'simulation'
        platform = detect_platform()
        assert platform in ("simulation", "raspberry_pi", "jetson", "beaglebone")

    def test_get_digital_pin(self) -> None:
        """Test get_digital_pin factory."""
        pin = get_digital_pin(17, PinMode.OUTPUT, name="test")
        assert isinstance(pin, DigitalPin)
        assert pin.number == 17

    def test_get_digital_pin_forced_simulation(self) -> None:
        """Test forcing simulation platform."""
        pin = get_digital_pin(17, platform="simulation")
        assert isinstance(pin, SimulatedDigitalPin)

    def test_get_pwm_pin(self) -> None:
        """Test get_pwm_pin factory."""
        pin = get_pwm_pin(18, frequency=50, name="servo")
        assert isinstance(pin, PWMPin)
        assert pin.frequency == 50

    def test_get_analog_pin(self) -> None:
        """Test get_analog_pin factory."""
        pin = get_analog_pin(0, resolution=12, reference_voltage=3.3)
        assert isinstance(pin, AnalogPin)
        assert pin.resolution == 12
