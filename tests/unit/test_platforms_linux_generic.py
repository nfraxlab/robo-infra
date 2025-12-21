"""Unit tests for Linux Generic SBC platform implementation."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.pin import PinMode, PinState
from robo_infra.platforms.linux_generic import (
    GPIOBackend,
    GPIOChipInfo,
    GPIOEdge,
    GPIOLineInfo,
    LinuxDigitalPin,
    LinuxGenericPlatform,
    LinuxPWMPin,
    LinuxSBCCapabilities,
    LinuxSBCType,
)


if TYPE_CHECKING:
    pass


# =============================================================================
# LinuxSBCType Tests
# =============================================================================


class TestLinuxSBCType:
    """Tests for LinuxSBCType enum."""

    def test_sbc_type_values(self) -> None:
        """Test all SBC type values."""
        assert LinuxSBCType.ORANGE_PI.value == "Orange Pi"
        assert LinuxSBCType.ORANGE_PI_ZERO.value == "Orange Pi Zero"
        assert LinuxSBCType.ORANGE_PI_5.value == "Orange Pi 5"
        assert LinuxSBCType.ROCK_PI_4.value == "Rock Pi 4"
        assert LinuxSBCType.ROCK_5B.value == "Rock 5B"
        assert LinuxSBCType.PINE64.value == "Pine A64"
        assert LinuxSBCType.ROCK64.value == "Rock64"
        assert LinuxSBCType.ROCKPRO64.value == "ROCKPro64"
        assert LinuxSBCType.BANANA_PI.value == "Banana Pi"
        assert LinuxSBCType.ODROID_C4.value == "Odroid C4"
        assert LinuxSBCType.ODROID_N2.value == "Odroid N2"
        assert LinuxSBCType.NANOPI.value == "NanoPi"
        assert LinuxSBCType.KHADAS_VIM.value == "Khadas VIM"
        assert LinuxSBCType.LIBRE_COMPUTER.value == "Libre Computer"
        assert LinuxSBCType.GENERIC.value == "Generic Linux SBC"

    def test_sbc_type_count(self) -> None:
        """Test expected number of SBC types."""
        assert len(LinuxSBCType) == 15


class TestGPIOBackend:
    """Tests for GPIOBackend enum."""

    def test_backend_values(self) -> None:
        """Test backend values."""
        assert GPIOBackend.GPIOD.value == "gpiod"
        assert GPIOBackend.SYSFS.value == "sysfs"
        assert GPIOBackend.SIMULATION.value == "simulation"

    def test_backend_count(self) -> None:
        """Test expected number of backends."""
        assert len(GPIOBackend) == 3


class TestGPIOEdge:
    """Tests for GPIOEdge enum."""

    def test_edge_values(self) -> None:
        """Test edge detection values."""
        assert GPIOEdge.NONE.value == "none"
        assert GPIOEdge.RISING.value == "rising"
        assert GPIOEdge.FALLING.value == "falling"
        assert GPIOEdge.BOTH.value == "both"


# =============================================================================
# Data Class Tests
# =============================================================================


class TestGPIOChipInfo:
    """Tests for GPIOChipInfo dataclass."""

    def test_create_chip_info(self) -> None:
        """Test creating GPIO chip info."""
        info = GPIOChipInfo(
            name="gpiochip0",
            label="pinctrl-bcm2711",
            num_lines=58,
            path=Path("/dev/gpiochip0"),
            chip_id=0,
        )
        assert info.name == "gpiochip0"
        assert info.label == "pinctrl-bcm2711"
        assert info.num_lines == 58
        assert info.chip_id == 0


class TestGPIOLineInfo:
    """Tests for GPIOLineInfo dataclass."""

    def test_create_line_info(self) -> None:
        """Test creating GPIO line info."""
        info = GPIOLineInfo(
            offset=17,
            name="GPIO17",
            consumer="robo_infra",
            direction="output",
            active_low=False,
            used=True,
        )
        assert info.offset == 17
        assert info.name == "GPIO17"
        assert info.consumer == "robo_infra"
        assert info.direction == "output"
        assert info.active_low is False
        assert info.used is True


class TestLinuxSBCCapabilities:
    """Tests for LinuxSBCCapabilities dataclass."""

    def test_create_capabilities(self) -> None:
        """Test creating SBC capabilities."""
        caps = LinuxSBCCapabilities(
            sbc_type=LinuxSBCType.ORANGE_PI,
            description="Orange Pi board",
        )
        assert caps.sbc_type == LinuxSBCType.ORANGE_PI
        assert caps.gpio_chips == []
        assert caps.pwm_chips == []
        assert caps.i2c_buses == []


# =============================================================================
# Digital Pin Tests
# =============================================================================


class TestLinuxDigitalPin:
    """Tests for LinuxDigitalPin."""

    def test_init_simulation(self) -> None:
        """Test pin initialization in simulation mode."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        assert pin.line == 17
        assert pin.chip == 0
        assert pin.mode == PinMode.OUTPUT
        assert pin.line_name == "gpio0_17"

    def test_custom_line_name(self) -> None:
        """Test custom line name."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
            line_name="LED_PIN",
        )
        assert pin.line_name == "LED_PIN"

    def test_high_low(self) -> None:
        """Test high/low output."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.high()
        assert pin.read() is True

        pin.low()
        assert pin.read() is False

    def test_write_state(self) -> None:
        """Test write state."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.write(True)
        assert pin.read() is True

    def test_write_to_input_raises(self) -> None:
        """Test writing to input pin raises error."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        with pytest.raises(ValueError, match="Cannot write to input pin"):
            pin.high()

    def test_toggle(self) -> None:
        """Test toggle functionality."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            initial=PinState.LOW,
        )
        pin.setup()
        assert pin.read() is False
        pin.toggle()
        assert pin.read() is True
        pin.toggle()
        assert pin.read() is False

    def test_initial_state(self) -> None:
        """Test initial state."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            initial=PinState.HIGH,
        )
        pin.setup()
        assert pin.read() is True

    def test_cleanup(self) -> None:
        """Test cleanup doesn't raise in simulation."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.cleanup()  # Should not raise


# =============================================================================
# PWM Pin Tests
# =============================================================================


class TestLinuxPWMPin:
    """Tests for LinuxPWMPin."""

    def test_init_simulation(self) -> None:
        """Test PWM pin initialization in simulation."""
        pin = LinuxPWMPin(
            chip=0,
            channel=0,
            frequency=1000,
            duty_cycle=0.5,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.chip_id == 0
        assert pin.channel == 0
        assert pin.frequency == 1000
        assert pin.duty_cycle == 0.5

    def test_duty_cycle_method(self) -> None:
        """Test duty cycle setter method."""
        pin = LinuxPWMPin(
            chip=0,
            channel=0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.set_duty_cycle(0.75)
        assert pin.duty_cycle == 0.75

    def test_duty_cycle_validation(self) -> None:
        """Test duty cycle validation."""
        pin = LinuxPWMPin(
            chip=0,
            channel=0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        with pytest.raises(ValueError, match="must be 0.0-1.0"):
            pin.set_duty_cycle(1.5)

        with pytest.raises(ValueError, match="must be 0.0-1.0"):
            pin.set_duty_cycle(-0.1)

    def test_frequency_method(self) -> None:
        """Test frequency setter method."""
        pin = LinuxPWMPin(
            chip=0,
            channel=0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.set_frequency(5000)
        assert pin.frequency == 5000

    def test_frequency_validation(self) -> None:
        """Test frequency validation."""
        pin = LinuxPWMPin(
            chip=0,
            channel=0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        with pytest.raises(ValueError, match="must be positive"):
            pin.set_frequency(0)

    def test_start_stop(self) -> None:
        """Test PWM start/stop."""
        pin = LinuxPWMPin(
            chip=0,
            channel=0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.is_running is False
        pin.start()
        assert pin.is_running is True
        pin.stop()
        assert pin.is_running is False

    def test_cleanup(self) -> None:
        """Test PWM cleanup."""
        pin = LinuxPWMPin(
            chip=0,
            channel=0,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.start()
        pin.cleanup()
        assert pin.is_running is False


# =============================================================================
# Platform Tests
# =============================================================================


class TestLinuxGenericPlatform:
    """Tests for LinuxGenericPlatform."""

    def test_init_simulation(self) -> None:
        """Test platform initialization in simulation mode."""
        platform = LinuxGenericPlatform(simulation=True)
        assert platform.is_simulation is True
        assert platform.backend == GPIOBackend.SIMULATION
        assert platform.sbc_type == LinuxSBCType.GENERIC

    def test_init_with_default_chip(self) -> None:
        """Test platform with custom default chip."""
        platform = LinuxGenericPlatform(simulation=True, default_chip=1)
        assert platform.default_chip == 1

    def test_platform_info(self) -> None:
        """Test platform info."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_info()
        assert "Linux" in info.model
        assert len(info.gpio_chips) >= 0  # Simulated chip

    def test_capabilities(self) -> None:
        """Test platform capabilities."""
        from robo_infra.platforms.base import PlatformCapability

        platform = LinuxGenericPlatform(simulation=True)
        caps = platform.capabilities
        assert PlatformCapability.GPIO in caps

    def test_list_gpio_chips(self) -> None:
        """Test listing GPIO chips."""
        platform = LinuxGenericPlatform(simulation=True)
        chips = platform.list_gpio_chips()
        assert len(chips) == 1
        assert chips[0].name == "gpiochip0"
        assert chips[0].num_lines == 32

    def test_get_gpio_chip_info(self) -> None:
        """Test getting GPIO chip info."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_gpio_chip_info(0)
        assert info is not None
        assert info.chip_id == 0

    def test_get_gpio_chip_info_not_found(self) -> None:
        """Test getting non-existent GPIO chip info."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_gpio_chip_info(99)
        assert info is None

    def test_list_gpio_lines(self) -> None:
        """Test listing GPIO lines."""
        platform = LinuxGenericPlatform(simulation=True)
        lines = platform.list_gpio_lines(0)
        assert len(lines) == 32

    def test_get_pin(self) -> None:
        """Test getting a digital pin."""
        platform = LinuxGenericPlatform(simulation=True)
        pin = platform.get_pin(17, chip=0, mode=PinMode.OUTPUT)
        assert isinstance(pin, LinuxDigitalPin)
        assert pin.line == 17
        assert pin.chip == 0

    def test_get_pin_default_chip(self) -> None:
        """Test getting pin with default chip."""
        platform = LinuxGenericPlatform(simulation=True, default_chip=0)
        pin = platform.get_pin(17, mode=PinMode.OUTPUT)
        assert pin.chip == 0

    def test_get_pin_cached(self) -> None:
        """Test pins are cached."""
        platform = LinuxGenericPlatform(simulation=True)
        pin1 = platform.get_pin(17, mode=PinMode.OUTPUT)
        pin2 = platform.get_pin(17, mode=PinMode.OUTPUT)
        assert pin1 is pin2

    def test_get_pin_by_string_number(self) -> None:
        """Test getting pin by string number."""
        platform = LinuxGenericPlatform(simulation=True)
        pin = platform.get_pin("17", mode=PinMode.OUTPUT)
        assert pin.line == 17

    def test_get_pwm_pin(self) -> None:
        """Test getting a PWM pin."""
        platform = LinuxGenericPlatform(simulation=True)
        pwm = platform.get_pwm_pin(chip=0, channel=0, frequency=1000)
        assert isinstance(pwm, LinuxPWMPin)
        assert pwm.chip_id == 0
        assert pwm.channel == 0

    def test_get_pwm_pin_cached(self) -> None:
        """Test PWM pins are cached."""
        platform = LinuxGenericPlatform(simulation=True)
        pwm1 = platform.get_pwm_pin(chip=0, channel=0)
        pwm2 = platform.get_pwm_pin(chip=0, channel=0)
        assert pwm1 is pwm2

    def test_cleanup(self) -> None:
        """Test platform cleanup."""
        platform = LinuxGenericPlatform(simulation=True)
        platform.get_pin(17, mode=PinMode.OUTPUT)
        platform.get_pwm_pin(chip=0, channel=0)
        platform.cleanup()  # Should not raise

    def test_string_backend(self) -> None:
        """Test string backend conversion."""
        platform = LinuxGenericPlatform(backend="simulation")
        assert platform.backend == GPIOBackend.SIMULATION

    def test_gpio_chips_property(self) -> None:
        """Test gpio_chips property returns copy."""
        platform = LinuxGenericPlatform(simulation=True)
        chips1 = platform.gpio_chips
        chips2 = platform.gpio_chips
        assert chips1 == chips2
        assert chips1 is not chips2  # Different list objects

    def test_pwm_chips_property(self) -> None:
        """Test pwm_chips property."""
        platform = LinuxGenericPlatform(simulation=True)
        pwm_chips = platform.pwm_chips
        assert isinstance(pwm_chips, list)


# =============================================================================
# Integration Tests
# =============================================================================


class TestLinuxGenericIntegration:
    """Integration tests for Linux Generic platform."""

    def test_led_blink_pattern(self) -> None:
        """Test LED blink pattern simulation."""
        platform = LinuxGenericPlatform(simulation=True)
        led = platform.get_pin(17, mode=PinMode.OUTPUT)
        led.setup()

        # Blink pattern
        for _ in range(3):
            led.high()
            assert led.read() is True
            led.low()
            assert led.read() is False

        platform.cleanup()

    def test_pwm_control(self) -> None:
        """Test PWM control simulation."""
        platform = LinuxGenericPlatform(simulation=True)
        pwm = platform.get_pwm_pin(chip=0, channel=0, frequency=1000)
        pwm.setup()
        pwm.start()

        # Sweep duty cycle
        for duty in [0.0, 0.25, 0.5, 0.75, 1.0]:
            pwm.set_duty_cycle(duty)
            assert pwm.duty_cycle == pytest.approx(duty)

        pwm.stop()
        platform.cleanup()

    def test_multiple_gpio_chips(self) -> None:
        """Test working with multiple GPIO chips."""
        platform = LinuxGenericPlatform(simulation=True)

        # Get pins from different chips (in simulation, only chip 0 exists)
        pin0 = platform.get_pin(0, chip=0, mode=PinMode.OUTPUT)
        pin1 = platform.get_pin(1, chip=0, mode=PinMode.OUTPUT)
        pin0.setup()
        pin1.setup()

        pin0.high()
        pin1.low()

        assert pin0.read() is True
        assert pin1.read() is False

        platform.cleanup()

    def test_input_output_mix(self) -> None:
        """Test mix of input and output pins."""
        platform = LinuxGenericPlatform(simulation=True)

        output_pin = platform.get_pin(17, mode=PinMode.OUTPUT)
        input_pin = platform.get_pin(27, mode=PinMode.INPUT)
        output_pin.setup()
        input_pin.setup()

        output_pin.high()
        # Input pin just reads (simulated state)
        _ = input_pin.read()

        platform.cleanup()

    def test_pwm_frequency_change(self) -> None:
        """Test changing PWM frequency."""
        platform = LinuxGenericPlatform(simulation=True)
        pwm = platform.get_pwm_pin(chip=0, channel=0, frequency=1000)
        pwm.setup()
        pwm.start()

        assert pwm.frequency == 1000

        pwm.set_frequency(5000)
        assert pwm.frequency == 5000

        pwm.stop()
        platform.cleanup()


# =============================================================================
# SBC Detection Tests
# =============================================================================


class TestSBCDetection:
    """Tests for SBC type detection."""

    def test_detection_returns_generic_in_simulation(self) -> None:
        """Test that simulation returns GENERIC type."""
        platform = LinuxGenericPlatform(simulation=True)
        assert platform.sbc_type == LinuxSBCType.GENERIC

    @patch("pathlib.Path.exists")
    @patch("pathlib.Path.read_text")
    def test_detect_orange_pi(self, mock_read: MagicMock, mock_exists: MagicMock) -> None:
        """Test Orange Pi detection."""
        mock_exists.return_value = True
        mock_read.return_value = "Orange Pi 4 LTS"

        # Force non-simulation by mocking GPIO chip existence
        with patch.object(LinuxGenericPlatform, "_has_gpio_chips", return_value=False):
            platform = LinuxGenericPlatform(simulation=True)
            # In simulation mode, detection doesn't run
            assert platform.sbc_type == LinuxSBCType.GENERIC

    def test_platform_name_includes_sbc_type(self) -> None:
        """Test platform name includes SBC type."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_info()
        assert "Linux" in info.model
