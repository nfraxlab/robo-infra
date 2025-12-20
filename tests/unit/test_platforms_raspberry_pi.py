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
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.pin import PinMode, PinState
from robo_infra.platforms.raspberry_pi import (
    GPIO_CHIP_PI5,
    GPIOBackend,
    HARDWARE_PWM_PINS_STANDARD,
    PI_MODELS,
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

        with patch.object(Path, "exists", return_value=True):
            with patch.object(
                Path, "read_text", return_value="Raspberry Pi 4 Model B Rev 1.4\x00"
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
        assert GPIO_CHIP_PI5 == Path("/dev/gpiochip4")


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
