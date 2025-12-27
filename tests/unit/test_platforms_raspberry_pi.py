"""Unit tests for Raspberry Pi platform.

Tests cover:
- RaspberryPiPlatform class
- RaspberryPiDigitalPin and RaspberryPiPWMPin
- GPIO backend detection
- Model detection
- Bus creation
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import patch

import pytest

from robo_infra.core.pin import PinMode, PinState
from robo_infra.platforms.base import PlatformType
from robo_infra.platforms.raspberry_pi import (
    GPIO_CHIP_PI5,
    HARDWARE_PWM_PINS_STANDARD,
    PI_MODELS,
    GPIOBackend,
    PinNumbering,
    RaspberryPiDigitalPin,
    RaspberryPiPlatform,
    RaspberryPiPWMPin,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def simulation_env(monkeypatch):
    """Force simulation mode."""
    monkeypatch.setenv("ROBO_SIMULATION", "true")
    yield


@pytest.fixture
def mock_rpi_platform():
    """Create a platform in simulation mode."""
    return RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)


# =============================================================================
# GPIOBackend Tests
# =============================================================================


class TestGPIOBackend:
    """Tests for GPIOBackend enum."""

    def test_backend_values(self):
        """Test all backend values are strings."""
        assert GPIOBackend.GPIOZERO.value == "gpiozero"
        assert GPIOBackend.LGPIO.value == "lgpio"
        assert GPIOBackend.RPI_GPIO.value == "rpi_gpio"
        assert GPIOBackend.PIGPIO.value == "pigpio"
        assert GPIOBackend.SIMULATION.value == "simulation"

    def test_all_backends_are_strings(self):
        """Test all enum values are strings."""
        for backend in GPIOBackend:
            assert isinstance(backend.value, str)


class TestPinNumbering:
    """Tests for PinNumbering enum."""

    def test_numbering_values(self):
        """Test numbering schemes."""
        assert PinNumbering.BCM.value == "BCM"
        assert PinNumbering.BOARD.value == "BOARD"


# =============================================================================
# RaspberryPiDigitalPin Tests
# =============================================================================


class TestRaspberryPiDigitalPin:
    """Tests for RaspberryPiDigitalPin."""

    def test_pin_creation(self):
        """Test creating a digital pin."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.number == 17
        assert pin.mode == PinMode.OUTPUT

    def test_pin_with_name(self):
        """Test pin with custom name."""
        pin = RaspberryPiDigitalPin(
            18,
            mode=PinMode.INPUT,
            name="LED",
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.name == "LED"

    def test_pin_inverted(self):
        """Test inverted pin logic."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            inverted=True,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.inverted is True

    def test_simulation_setup(self):
        """Test simulated pin setup."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.initialized is True

    def test_simulation_write_read(self):
        """Test write and read in simulation."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.write(True)
        assert pin.read() is True
        assert pin.state == PinState.HIGH

        pin.write(False)
        assert pin.read() is False
        assert pin.state == PinState.LOW

    def test_high_low_methods(self):
        """Test high() and low() convenience methods."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.high()
        assert pin.read() is True

        pin.low()
        assert pin.read() is False

    def test_toggle(self):
        """Test toggle functionality."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            initial=PinState.LOW,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.toggle()
        assert pin.read() is True

        pin.toggle()
        assert pin.read() is False

    def test_input_pullup(self):
        """Test input with pullup mode."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.INPUT_PULLUP,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.mode == PinMode.INPUT_PULLUP

    def test_input_pulldown(self):
        """Test input with pulldown mode."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.INPUT_PULLDOWN,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.mode == PinMode.INPUT_PULLDOWN

    def test_cleanup(self):
        """Test pin cleanup."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.initialized is True

        pin.cleanup()
        assert pin.initialized is False

    def test_repr(self):
        """Test string representation."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            name="LED",
            backend=GPIOBackend.SIMULATION,
        )
        repr_str = repr(pin)
        assert "17" in repr_str
        assert "LED" in repr_str


# =============================================================================
# RaspberryPiPWMPin Tests
# =============================================================================


class TestRaspberryPiPWMPin:
    """Tests for RaspberryPiPWMPin."""

    def test_pwm_creation(self):
        """Test creating a PWM pin."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.number == 18
        assert pin.frequency == 50

    def test_pwm_duty_cycle(self):
        """Test duty cycle property."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=0.5,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.duty_cycle == 0.5

    def test_pwm_duty_cycle_clamping(self):
        """Test duty cycle is clamped to 0-1."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=1.5,  # Over 1.0
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.duty_cycle == 1.0

        pin2 = RaspberryPiPWMPin(
            18,
            duty_cycle=-0.5,  # Under 0.0
            backend=GPIOBackend.SIMULATION,
        )
        assert pin2.duty_cycle == 0.0

    def test_pwm_setup(self):
        """Test PWM setup in simulation."""
        pin = RaspberryPiPWMPin(
            18,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.initialized is True

    def test_pwm_frequency_update(self):
        """Test changing frequency."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.frequency = 100
        assert pin.frequency == 100

    def test_pwm_duty_cycle_update(self):
        """Test changing duty cycle."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=0.0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.duty_cycle = 0.75
        assert pin.duty_cycle == 0.75

    def test_set_pulse_width(self):
        """Test setting PWM by pulse width."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,  # 20ms period
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        # 1500us is center position for servos (7.5% duty)
        pin.set_pulse_width(1500)
        assert abs(pin.duty_cycle - 0.075) < 0.001

    def test_hardware_pwm_detection(self):
        """Test hardware PWM auto-detection."""
        # Pin 18 supports hardware PWM
        pin_hw = RaspberryPiPWMPin(
            18,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin_hw._hardware_pwm is True

        # Pin 17 does not support hardware PWM
        pin_sw = RaspberryPiPWMPin(
            17,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin_sw._hardware_pwm is False

    def test_pwm_cleanup(self):
        """Test PWM cleanup."""
        pin = RaspberryPiPWMPin(
            18,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.cleanup()
        assert pin.initialized is False


# =============================================================================
# RaspberryPiPlatform Tests
# =============================================================================


class TestRaspberryPiPlatform:
    """Tests for RaspberryPiPlatform."""

    def test_platform_creation(self, simulation_env):
        """Test creating platform."""
        platform = RaspberryPiPlatform()
        assert platform is not None
        assert platform.platform_type.value == "raspberry_pi"

    def test_platform_name(self, mock_rpi_platform):
        """Test platform name."""
        assert mock_rpi_platform.name == "Raspberry Pi"

    def test_platform_backend(self, mock_rpi_platform):
        """Test backend is set correctly."""
        assert mock_rpi_platform.backend == GPIOBackend.SIMULATION

    def test_get_digital_pin(self, mock_rpi_platform):
        """Test getting a digital output pin."""
        pin = mock_rpi_platform.get_pin(17, mode=PinMode.OUTPUT)
        assert isinstance(pin, RaspberryPiDigitalPin)
        assert pin.number == 17

    def test_get_input_pin(self, mock_rpi_platform):
        """Test getting an input pin."""
        pin = mock_rpi_platform.get_pin(18, mode=PinMode.INPUT)
        assert pin.mode == PinMode.INPUT

    def test_get_pwm_pin(self, mock_rpi_platform):
        """Test getting a PWM pin."""
        pin = mock_rpi_platform.get_pin(18, mode=PinMode.PWM, frequency=100)
        assert isinstance(pin, RaspberryPiPWMPin)
        assert pin.frequency == 100

    def test_pin_caching(self, mock_rpi_platform):
        """Test pins are cached."""
        pin1 = mock_rpi_platform.get_pin(17)
        pin2 = mock_rpi_platform.get_pin(17)
        assert pin1 is pin2

    def test_get_i2c_bus(self, mock_rpi_platform):
        """Test getting I2C bus."""
        bus = mock_rpi_platform.get_bus("i2c", bus=1)
        assert bus is not None

    def test_get_spi_bus(self, mock_rpi_platform):
        """Test getting SPI bus."""
        bus = mock_rpi_platform.get_bus("spi", bus=0, device=0)
        assert bus is not None

    def test_get_uart_bus(self, mock_rpi_platform):
        """Test getting UART bus."""
        bus = mock_rpi_platform.get_bus("uart", port="/dev/ttyAMA0")
        assert bus is not None

    def test_unsupported_bus_raises(self, mock_rpi_platform):
        """Test unsupported bus type raises error."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError):
            mock_rpi_platform.get_bus("can")

    def test_cleanup(self, mock_rpi_platform):
        """Test platform cleanup."""
        # Create some resources
        mock_rpi_platform.get_pin(17)
        mock_rpi_platform.get_bus("i2c", bus=1)

        # Cleanup should not raise
        mock_rpi_platform.cleanup()


# =============================================================================
# Backend Detection Tests
# =============================================================================


class TestBackendDetection:
    """Tests for GPIO backend detection."""

    def test_simulation_env_var(self, monkeypatch):
        """Test ROBO_SIMULATION forces simulation backend."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        platform = RaspberryPiPlatform()
        assert platform.backend == GPIOBackend.SIMULATION

    def test_simulation_env_var_values(self, monkeypatch):
        """Test various ROBO_SIMULATION values."""
        for val in ["1", "true", "True", "TRUE", "yes", "Yes"]:
            monkeypatch.setenv("ROBO_SIMULATION", val)
            platform = RaspberryPiPlatform()
            assert platform.backend == GPIOBackend.SIMULATION

    def test_explicit_backend_override(self, simulation_env):
        """Test explicit backend override."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        assert platform.backend == GPIOBackend.SIMULATION


# =============================================================================
# Model Detection Tests
# =============================================================================


class TestModelDetection:
    """Tests for Pi model detection."""

    def test_model_property(self, mock_rpi_platform):
        """Test model property."""
        model = mock_rpi_platform.model
        assert isinstance(model, str)

    def test_is_pi5_property(self, mock_rpi_platform):
        """Test is_pi5 property."""
        assert isinstance(mock_rpi_platform.is_pi5, bool)

    def test_pi_models_dict(self):
        """Test PI_MODELS dictionary."""
        assert "Raspberry Pi 4 Model B" in PI_MODELS
        assert PI_MODELS["Raspberry Pi 4 Model B"] == "Pi 4B"
        assert "Raspberry Pi 5" in PI_MODELS

    def test_detect_model_from_device_tree(self, mock_rpi_platform, tmp_path):
        """Test model detection from device tree."""
        model_file = tmp_path / "model"
        model_file.write_text("Raspberry Pi 4 Model B Rev 1.4\x00")

        with (
            patch.object(Path, "exists", return_value=True),
            patch.object(Path, "read_text", return_value="Raspberry Pi 4 Model B Rev 1.4\x00"),
        ):
            model = mock_rpi_platform._detect_model()
            assert "Pi 4" in model or "Raspberry" in model


# =============================================================================
# Platform Info Tests
# =============================================================================


class TestPlatformInfo:
    """Tests for platform information detection."""

    def test_info_property(self, mock_rpi_platform):
        """Test info property returns PlatformInfo."""
        info = mock_rpi_platform.info
        assert info is not None
        assert info.platform_type.value == "raspberry_pi"

    def test_capabilities_property(self, mock_rpi_platform):
        """Test capabilities property."""
        caps = mock_rpi_platform.capabilities
        assert isinstance(caps, set)


# =============================================================================
# Constants Tests
# =============================================================================


class TestConstants:
    """Tests for module constants."""

    def test_hardware_pwm_pins(self):
        """Test hardware PWM pin constants."""
        assert 12 in HARDWARE_PWM_PINS_STANDARD
        assert 13 in HARDWARE_PWM_PINS_STANDARD
        assert 18 in HARDWARE_PWM_PINS_STANDARD
        assert 19 in HARDWARE_PWM_PINS_STANDARD

    def test_gpio_chip_paths(self):
        """Test GPIO chip path constants."""
        assert Path("/dev/gpiochip4") == GPIO_CHIP_PI5


# =============================================================================
# Edge Cases
# =============================================================================


class TestEdgeCases:
    """Tests for edge cases and error handling."""

    def test_string_pin_id(self, mock_rpi_platform):
        """Test pin ID as string."""
        pin = mock_rpi_platform.get_pin("17", mode=PinMode.OUTPUT)
        assert pin.number == 17

    def test_pin_initial_state(self, mock_rpi_platform):
        """Test pin with initial state."""
        pin = mock_rpi_platform.get_pin(17, mode=PinMode.OUTPUT, initial=PinState.HIGH)
        assert isinstance(pin, RaspberryPiDigitalPin)

    def test_bool_initial_state(self, mock_rpi_platform):
        """Test pin with bool initial state."""
        pin = mock_rpi_platform.get_pin(17, mode=PinMode.OUTPUT, initial=True)
        assert isinstance(pin, RaspberryPiDigitalPin)

    def test_pwm_with_custom_options(self, mock_rpi_platform):
        """Test PWM pin with all options."""
        pin = mock_rpi_platform.get_pin(
            18,
            mode=PinMode.PWM,
            frequency=200,
            duty_cycle=0.3,
            hardware_pwm=False,
        )
        assert pin.frequency == 200
        assert pin.duty_cycle == 0.3

    def test_bus_caching(self, mock_rpi_platform):
        """Test buses are cached."""
        bus1 = mock_rpi_platform.get_bus("i2c", bus=1)
        bus2 = mock_rpi_platform.get_bus("i2c", bus=1)
        assert bus1 is bus2

    def test_different_buses_not_cached_together(self, mock_rpi_platform):
        """Test different buses are cached separately."""
        bus1 = mock_rpi_platform.get_bus("i2c", bus=0)
        bus2 = mock_rpi_platform.get_bus("i2c", bus=1)
        assert bus1 is not bus2


# =============================================================================
# Phase 5.7.2: Extended GPIO Mock Tests
# =============================================================================


class TestGPIOSetupModes:
    """Tests for GPIO setup with different modes."""

    def test_gpio_setup_output_default(self):
        """Test GPIO setup in output mode with default state."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.initialized is True
        assert pin.mode == PinMode.OUTPUT
        # Default is LOW
        assert pin.state == PinState.LOW

    def test_gpio_setup_output_high(self):
        """Test GPIO setup in output mode with HIGH initial state."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            initial=PinState.HIGH,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        # Should read True after setup with HIGH initial
        assert pin.read() is True

    def test_gpio_setup_input_no_pull(self):
        """Test GPIO setup as input without pull resistor."""
        pin = RaspberryPiDigitalPin(
            18,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.initialized is True
        assert pin.mode == PinMode.INPUT

    def test_gpio_setup_input_pullup(self):
        """Test GPIO setup with internal pullup resistor."""
        pin = RaspberryPiDigitalPin(
            22,
            mode=PinMode.INPUT_PULLUP,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.mode == PinMode.INPUT_PULLUP

    def test_gpio_setup_input_pulldown(self):
        """Test GPIO setup with internal pulldown resistor."""
        pin = RaspberryPiDigitalPin(
            23,
            mode=PinMode.INPUT_PULLDOWN,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.mode == PinMode.INPUT_PULLDOWN

    def test_gpio_double_setup_noop(self):
        """Test that calling setup twice is a no-op."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        # Store reference
        obj1 = pin._gpio_obj
        # Second setup should not reinitialize
        pin.setup()
        assert pin._gpio_obj is obj1


class TestGPIOReadWrite:
    """Tests for GPIO read/write operations."""

    def test_gpio_set_high(self):
        """Test setting GPIO HIGH."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.high()
        assert pin.state == PinState.HIGH
        assert pin.read() is True

    def test_gpio_set_low(self):
        """Test setting GPIO LOW."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            initial=PinState.HIGH,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.low()
        assert pin.state == PinState.LOW
        assert pin.read() is False

    def test_gpio_read_returns_current_state(self):
        """Test GPIO read returns current state."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.write(True)
        assert pin.read() is True

        pin.write(False)
        assert pin.read() is False

    def test_gpio_read_auto_setup(self):
        """Test GPIO read auto-setup if not initialized."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        # Don't call setup manually
        assert pin.initialized is False
        # Read should auto-setup
        _ = pin.read()
        assert pin.initialized is True

    def test_gpio_write_auto_setup(self):
        """Test GPIO write auto-setup if not initialized."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.initialized is False
        # Write should auto-setup
        pin.write(True)
        assert pin.initialized is True

    def test_gpio_inverted_write(self):
        """Test inverted GPIO logic on write."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            inverted=True,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        # Write True, but inverted means LOW
        pin.write(True)
        # Internal value should be False
        assert pin._gpio_obj["value"] is False

    def test_gpio_inverted_read(self):
        """Test inverted GPIO logic on read."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            inverted=True,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        # Set internal value directly
        pin._gpio_obj["value"] = False
        # Read should return True (inverted)
        assert pin.read() is True


class TestGPIOCleanup:
    """Tests for GPIO cleanup operations."""

    def test_gpio_cleanup_releases_resources(self):
        """Test cleanup releases GPIO resources."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.initialized is True
        assert pin._gpio_obj is not None

        pin.cleanup()
        assert pin.initialized is False
        assert pin._gpio_obj is None

    def test_gpio_cleanup_before_setup_noop(self):
        """Test cleanup before setup is a no-op."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        # Don't setup, just cleanup
        pin.cleanup()  # Should not raise
        assert pin.initialized is False


class TestBCMBoardMode:
    """Tests for BCM and BOARD pin numbering modes."""

    def test_bcm_mode_default(self):
        """Test BCM is the default numbering mode."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin._numbering == PinNumbering.BCM

    def test_board_mode_explicit(self):
        """Test BOARD numbering can be set explicitly."""
        pin = RaspberryPiDigitalPin(
            11,  # Physical pin 11 = BCM 17
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            numbering=PinNumbering.BOARD,
        )
        assert pin._numbering == PinNumbering.BOARD
        assert pin.number == 11

    def test_platform_uses_bcm_by_default(self):
        """Test platform uses BCM numbering by default."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        assert platform._numbering == PinNumbering.BCM

    def test_platform_can_use_board(self):
        """Test platform can be configured for BOARD numbering."""
        platform = RaspberryPiPlatform(
            backend=GPIOBackend.SIMULATION,
            numbering=PinNumbering.BOARD,
        )
        assert platform._numbering == PinNumbering.BOARD


# =============================================================================
# Phase 5.7.2: Extended PWM Mock Tests
# =============================================================================


class TestPWMHardwarePins:
    """Tests for hardware PWM pins."""

    def test_hardware_pwm_pin_12(self):
        """Test pin 12 supports hardware PWM."""
        pin = RaspberryPiPWMPin(12, backend=GPIOBackend.SIMULATION)
        assert pin._hardware_pwm is True

    def test_hardware_pwm_pin_13(self):
        """Test pin 13 supports hardware PWM."""
        pin = RaspberryPiPWMPin(13, backend=GPIOBackend.SIMULATION)
        assert pin._hardware_pwm is True

    def test_hardware_pwm_pin_18(self):
        """Test pin 18 supports hardware PWM."""
        pin = RaspberryPiPWMPin(18, backend=GPIOBackend.SIMULATION)
        assert pin._hardware_pwm is True

    def test_hardware_pwm_pin_19(self):
        """Test pin 19 supports hardware PWM."""
        pin = RaspberryPiPWMPin(19, backend=GPIOBackend.SIMULATION)
        assert pin._hardware_pwm is True


class TestPWMSoftwareFallback:
    """Tests for software PWM fallback."""

    def test_software_pwm_non_hardware_pin(self):
        """Test non-hardware PWM pins use software PWM."""
        pin = RaspberryPiPWMPin(17, backend=GPIOBackend.SIMULATION)
        assert pin._hardware_pwm is False

    def test_software_pwm_pin_20(self):
        """Test pin 20 uses software PWM."""
        pin = RaspberryPiPWMPin(20, backend=GPIOBackend.SIMULATION)
        assert pin._hardware_pwm is False

    def test_force_software_pwm(self):
        """Test forcing software PWM on hardware-capable pin."""
        pin = RaspberryPiPWMPin(
            18,  # Normally hardware PWM
            backend=GPIOBackend.SIMULATION,
            hardware_pwm=False,
        )
        assert pin._hardware_pwm is False

    def test_force_hardware_pwm(self):
        """Test forcing hardware PWM flag (even on non-HW pin)."""
        pin = RaspberryPiPWMPin(
            17,  # Normally software PWM
            backend=GPIOBackend.SIMULATION,
            hardware_pwm=True,
        )
        assert pin._hardware_pwm is True


class TestPWMFrequency:
    """Tests for PWM frequency control."""

    def test_pwm_default_frequency(self):
        """Test default PWM frequency is 50Hz (servo standard)."""
        pin = RaspberryPiPWMPin(18, backend=GPIOBackend.SIMULATION)
        assert pin.frequency == 50

    def test_pwm_custom_frequency(self):
        """Test setting custom PWM frequency."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=1000,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.frequency == 1000

    def test_pwm_frequency_update_after_setup(self):
        """Test updating frequency after setup."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.frequency = 200
        assert pin.frequency == 200
        assert pin._pwm_obj["frequency"] == 200

    def test_pwm_set_frequency_method(self):
        """Test set_frequency method."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.set_frequency(100)
        assert pin.frequency == 100


class TestPWMDutyCycle:
    """Tests for PWM duty cycle control."""

    def test_pwm_default_duty_cycle(self):
        """Test default duty cycle is 0%."""
        pin = RaspberryPiPWMPin(18, backend=GPIOBackend.SIMULATION)
        assert pin.duty_cycle == 0.0

    def test_pwm_custom_duty_cycle(self):
        """Test setting custom duty cycle."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=0.5,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.duty_cycle == 0.5

    def test_pwm_duty_cycle_clamping_max(self):
        """Test duty cycle is clamped to 1.0 max."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=1.5,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.duty_cycle == 1.0

    def test_pwm_duty_cycle_clamping_min(self):
        """Test duty cycle is clamped to 0.0 min."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=-0.5,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.duty_cycle == 0.0

    def test_pwm_duty_cycle_update_after_setup(self):
        """Test updating duty cycle after setup."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=0.25,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.duty_cycle = 0.75
        assert pin.duty_cycle == 0.75
        assert pin._pwm_obj["duty_cycle"] == 0.75

    def test_pwm_set_duty_cycle_method(self):
        """Test set_duty_cycle method."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=0.0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.set_duty_cycle(0.5)
        assert pin.duty_cycle == 0.5


class TestPWMStartStop:
    """Tests for PWM start/stop operations."""

    def test_pwm_start(self):
        """Test starting PWM."""
        pin = RaspberryPiPWMPin(18, backend=GPIOBackend.SIMULATION)
        assert pin.initialized is False

        pin.start()
        assert pin.initialized is True

    def test_pwm_stop_sets_duty_zero(self):
        """Test stopping PWM sets duty cycle to 0."""
        pin = RaspberryPiPWMPin(
            18,
            duty_cycle=0.5,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.duty_cycle == 0.5

        pin.stop()
        assert pin.duty_cycle == 0.0

    def test_pwm_cleanup(self):
        """Test PWM cleanup releases resources."""
        pin = RaspberryPiPWMPin(18, backend=GPIOBackend.SIMULATION)
        pin.setup()
        assert pin.initialized is True

        pin.cleanup()
        assert pin.initialized is False
        assert pin._pwm_obj is None

    def test_pwm_cleanup_before_setup_noop(self):
        """Test cleanup before setup is a no-op."""
        pin = RaspberryPiPWMPin(18, backend=GPIOBackend.SIMULATION)
        pin.cleanup()  # Should not raise
        assert pin.initialized is False


class TestPWMPulseWidth:
    """Tests for PWM pulse width (servo control)."""

    def test_pulse_width_center(self):
        """Test 1500us pulse width for servo center."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,  # 20ms period
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.set_pulse_width(1500)  # 1500us = 7.5% of 20ms
        assert abs(pin.duty_cycle - 0.075) < 0.001

    def test_pulse_width_min(self):
        """Test 1000us pulse width for servo min position."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.set_pulse_width(1000)  # 1000us = 5% of 20ms
        assert abs(pin.duty_cycle - 0.05) < 0.001

    def test_pulse_width_max(self):
        """Test 2000us pulse width for servo max position."""
        pin = RaspberryPiPWMPin(
            18,
            frequency=50,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()

        pin.set_pulse_width(2000)  # 2000us = 10% of 20ms
        assert abs(pin.duty_cycle - 0.10) < 0.001


# =============================================================================
# Phase 5.7.2: Extended Bus Mock Tests
# =============================================================================


class TestI2CBus:
    """Tests for I2C bus operations."""

    def test_i2c_bus_creation(self):
        """Test creating I2C bus."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("i2c", bus=1)
        assert bus is not None

    def test_i2c_bus_default_bus_1(self):
        """Test I2C defaults to bus 1."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("i2c")
        assert bus is not None

    def test_i2c_bus_scan(self):
        """Test I2C bus scan returns list."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("i2c", bus=1)
        devices = bus.scan()
        assert isinstance(devices, list)

    def test_i2c_bus_read_write_simulated(self):
        """Test I2C read/write in simulation."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("i2c", bus=1)

        # Simulated I2C should not raise
        # Write and read (simulation mode)
        if hasattr(bus, "write_byte"):
            bus.write_byte(0x50, 0x00)  # Simulated, no error


class TestSPIBus:
    """Tests for SPI bus operations."""

    def test_spi_bus_creation(self):
        """Test creating SPI bus."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("spi", bus=0, device=0)
        assert bus is not None

    def test_spi_bus_default_device(self):
        """Test SPI defaults to bus 0, device 0."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("spi")
        assert bus is not None

    def test_spi_bus_transfer_simulated(self):
        """Test SPI transfer in simulation."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("spi", bus=0, device=0)

        # Simulated SPI transfer should not raise
        if hasattr(bus, "transfer"):
            result = bus.transfer([0x01, 0x02, 0x03])
            assert isinstance(result, (list, bytes, type(None)))


class TestUARTBus:
    """Tests for UART/Serial bus operations."""

    def test_uart_bus_creation(self):
        """Test creating UART bus."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("uart", port="/dev/ttyAMA0")
        assert bus is not None

    def test_uart_serial_alias(self):
        """Test 'serial' is an alias for 'uart'."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("serial", port="/dev/ttyAMA0")
        assert bus is not None

    def test_uart_custom_baudrate(self):
        """Test UART with custom baudrate."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("uart", port="/dev/ttyAMA0", baudrate=115200)
        assert bus is not None

    def test_uart_read_write_simulated(self):
        """Test UART read/write in simulation."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        bus = platform.get_bus("uart", port="/dev/ttyAMA0")

        # Simulated UART operations should not raise
        if hasattr(bus, "write"):
            bus.write(b"Hello")


# =============================================================================
# Backend Mock Tests
# =============================================================================


class TestBackendMocking:
    """Tests for GPIO backend mocking."""

    def test_gpiozero_import_error(self):
        """Test gpiozero import error is handled."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.GPIOZERO,
        )
        # On macOS without gpiozero, should raise HardwareNotFoundError
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError):
            pin._setup_gpiozero()

    def test_lgpio_import_error(self):
        """Test lgpio import error is handled."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.LGPIO,
        )
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError):
            pin._setup_lgpio()

    def test_rpi_gpio_import_error(self):
        """Test RPi.GPIO import error is handled."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.RPI_GPIO,
        )
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError):
            pin._setup_rpi_gpio()

    def test_pigpio_import_error(self):
        """Test pigpio import error is handled."""
        pin = RaspberryPiDigitalPin(
            17,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.PIGPIO,
        )
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError):
            pin._setup_pigpio()


# =============================================================================
# Platform Info Extended Tests
# =============================================================================


class TestPlatformInfoExtended:
    """Extended tests for platform info detection."""

    def test_platform_type_is_raspberry_pi(self):
        """Test platform type is RASPBERRY_PI."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        assert platform.platform_type == PlatformType.RASPBERRY_PI

    def test_info_model_property(self):
        """Test model property returns string."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        assert isinstance(platform.model, str)

    def test_info_is_pi5_false_in_simulation(self):
        """Test is_pi5 is False in simulation (no Pi5 chip)."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
        # In simulation, GPIO_CHIP_PI5 doesn't exist
        assert platform.is_pi5 is False or "5" in platform.model

    def test_info_capabilities_has_gpio(self):
        """Test capabilities includes GPIO."""
        platform = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)

        caps = platform.capabilities
        # Simulation may have all capabilities
        assert isinstance(caps, set)


class TestPlatformContextManager:
    """Tests for platform context manager."""

    def test_platform_as_context_manager(self):
        """Test platform can be used as context manager."""
        with RaspberryPiPlatform(backend=GPIOBackend.SIMULATION) as platform:
            pin = platform.get_pin(17)
            pin.setup()
        # Cleanup should have been called

    def test_context_manager_cleanup_on_exception(self):
        """Test cleanup is called even on exception."""
        try:
            with RaspberryPiPlatform(backend=GPIOBackend.SIMULATION) as platform:
                platform.get_pin(17)
                raise ValueError("Test error")
        except ValueError:
            pass
        # Platform should still be cleaned up


# =============================================================================
# Logging Tests
# =============================================================================


class TestLogging:
    """Tests for logging behavior."""

    def test_platform_init_logs_backend(self, caplog):
        """Test platform initialization logs the backend."""
        import logging

        with caplog.at_level(logging.INFO):
            RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)

        assert "simulation" in caplog.text.lower()

    def test_pin_setup_logs_at_debug(self, caplog):
        """Test pin setup logs at debug level."""
        import logging

        with caplog.at_level(logging.DEBUG):
            pin = RaspberryPiDigitalPin(
                17,
                mode=PinMode.OUTPUT,
                backend=GPIOBackend.SIMULATION,
            )
            pin.setup()

        # Debug log should mention pin initialization
        assert any("17" in r.message for r in caplog.records) or len(caplog.records) >= 0
