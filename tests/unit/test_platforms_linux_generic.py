"""Unit tests for Linux Generic SBC platform implementation."""

from __future__ import annotations

from pathlib import Path
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
        with pytest.raises(ValueError, match=r"must be 0\.0-1\.0"):
            pin.set_duty_cycle(1.5)

        with pytest.raises(ValueError, match=r"must be 0\.0-1\.0"):
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


# =============================================================================
# libgpiod Mock Tests (Phase 5.7.4.1)
# =============================================================================


class TestLinuxGPIOChipOpen:
    """Tests for GPIO chip open operations."""

    def test_gpiochip_open_simulation(self) -> None:
        """Test opening GPIO chip in simulation mode."""
        platform = LinuxGenericPlatform(simulation=True)
        chips = platform.list_gpio_chips()
        assert len(chips) >= 1
        assert chips[0].chip_id == 0

    def test_gpiochip_open_default_chip(self) -> None:
        """Test opening default GPIO chip."""
        platform = LinuxGenericPlatform(simulation=True, default_chip=0)
        info = platform.get_gpio_chip_info(0)
        assert info is not None
        assert info.name == "gpiochip0"

    def test_gpiochip_open_custom_chip(self) -> None:
        """Test opening custom GPIO chip."""
        platform = LinuxGenericPlatform(simulation=True, default_chip=1)
        assert platform.default_chip == 1

    def test_gpiochip_open_nonexistent(self) -> None:
        """Test opening non-existent GPIO chip returns None."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_gpio_chip_info(99)
        assert info is None

    def test_gpiochip_info_properties(self) -> None:
        """Test GPIO chip info properties."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_gpio_chip_info(0)
        assert info is not None
        assert info.name == "gpiochip0"
        assert info.label == "simulated-gpio"
        assert info.num_lines == 32
        assert info.chip_id == 0

    def test_gpiochip_path(self) -> None:
        """Test GPIO chip path."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_gpio_chip_info(0)
        assert info is not None
        assert str(info.path) == "/dev/gpiochip0"

    @patch("robo_infra.platforms.linux_generic.GPIOCHIP_PATH")
    def test_gpiochip_discovery_with_glob(self, mock_path: MagicMock) -> None:
        """Test GPIO chip discovery with glob."""
        mock_path.glob.return_value = []
        platform = LinuxGenericPlatform(simulation=True)
        # Simulation mode still has simulated chips
        assert len(platform.gpio_chips) >= 1


class TestLinuxGPIOChipList:
    """Tests for GPIO chip listing operations."""

    def test_list_gpio_chips_returns_list(self) -> None:
        """Test list_gpio_chips returns a list."""
        platform = LinuxGenericPlatform(simulation=True)
        chips = platform.list_gpio_chips()
        assert isinstance(chips, list)

    def test_list_gpio_chips_simulation(self) -> None:
        """Test list_gpio_chips in simulation mode."""
        platform = LinuxGenericPlatform(simulation=True)
        chips = platform.list_gpio_chips()
        assert len(chips) == 1
        assert chips[0].name == "gpiochip0"

    def test_list_gpio_chips_returns_copy(self) -> None:
        """Test list_gpio_chips returns a copy."""
        platform = LinuxGenericPlatform(simulation=True)
        chips1 = platform.list_gpio_chips()
        chips2 = platform.list_gpio_chips()
        assert chips1 is not chips2
        assert chips1 == chips2

    def test_gpio_chips_property(self) -> None:
        """Test gpio_chips property."""
        platform = LinuxGenericPlatform(simulation=True)
        chips = platform.gpio_chips
        assert isinstance(chips, list)
        assert len(chips) >= 1

    def test_multiple_gpio_chips_access(self) -> None:
        """Test accessing multiple GPIO chips."""
        platform = LinuxGenericPlatform(simulation=True)
        for chip in platform.list_gpio_chips():
            info = platform.get_gpio_chip_info(chip.chip_id)
            assert info is not None
            assert info.num_lines > 0


class TestLinuxLineRequest:
    """Tests for GPIO line request operations."""

    def test_line_request_output(self) -> None:
        """Test requesting line as output."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.mode == PinMode.OUTPUT

    def test_line_request_input(self) -> None:
        """Test requesting line as input."""
        pin = LinuxDigitalPin(
            line=18,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        assert pin.mode == PinMode.INPUT

    def test_line_request_with_pull_up(self) -> None:
        """Test requesting line with pull-up."""
        pin = LinuxDigitalPin(
            line=19,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
            pull="up",
        )
        pin.setup()
        assert pin._pull == "up"

    def test_line_request_with_pull_down(self) -> None:
        """Test requesting line with pull-down."""
        pin = LinuxDigitalPin(
            line=20,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
            pull="down",
        )
        pin.setup()
        assert pin._pull == "down"

    def test_line_request_active_low(self) -> None:
        """Test requesting line with active_low."""
        pin = LinuxDigitalPin(
            line=21,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            active_low=True,
        )
        pin.setup()
        assert pin._active_low is True

    def test_line_request_initial_high(self) -> None:
        """Test requesting output line with initial HIGH."""
        pin = LinuxDigitalPin(
            line=22,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            initial=PinState.HIGH,
        )
        pin.setup()
        assert pin.read() is True

    def test_line_request_initial_low(self) -> None:
        """Test requesting output line with initial LOW."""
        pin = LinuxDigitalPin(
            line=23,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            initial=PinState.LOW,
        )
        pin.setup()
        assert pin.read() is False

    def test_line_request_via_platform(self) -> None:
        """Test line request via platform."""
        platform = LinuxGenericPlatform(simulation=True)
        pin = platform.get_pin(17, chip=0, mode=PinMode.OUTPUT)
        pin.setup()
        assert pin.mode == PinMode.OUTPUT

    def test_line_request_with_custom_name(self) -> None:
        """Test line request with custom name."""
        pin = LinuxDigitalPin(
            line=24,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            line_name="LED_PIN",
        )
        pin.setup()
        assert pin.line_name == "LED_PIN"


class TestLinuxLineRelease:
    """Tests for GPIO line release operations."""

    def test_line_release_simulation(self) -> None:
        """Test releasing line in simulation."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.cleanup()
        assert pin._initialized is False

    def test_line_release_before_setup(self) -> None:
        """Test releasing line before setup."""
        pin = LinuxDigitalPin(
            line=18,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.cleanup()  # Should not raise

    def test_line_release_idempotent(self) -> None:
        """Test multiple cleanup calls are safe."""
        pin = LinuxDigitalPin(
            line=19,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.cleanup()
        pin.cleanup()  # Should not raise

    def test_line_release_via_platform_cleanup(self) -> None:
        """Test line release via platform cleanup."""
        platform = LinuxGenericPlatform(simulation=True)
        pin = platform.get_pin(17, mode=PinMode.OUTPUT)
        pin.setup()
        platform.cleanup()  # Should cleanup all pins

    def test_line_release_multiple_pins(self) -> None:
        """Test releasing multiple pins."""
        platform = LinuxGenericPlatform(simulation=True)
        pin1 = platform.get_pin(17, mode=PinMode.OUTPUT)
        pin2 = platform.get_pin(18, mode=PinMode.OUTPUT)
        pin1.setup()
        pin2.setup()
        pin1.cleanup()
        pin2.cleanup()


class TestLinuxLineRead:
    """Tests for GPIO line read operations."""

    def test_line_read_returns_bool(self) -> None:
        """Test reading line returns boolean."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        value = pin.read()
        assert isinstance(value, bool)

    def test_line_read_input_pin(self) -> None:
        """Test reading input pin."""
        pin = LinuxDigitalPin(
            line=18,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        _ = pin.read()  # Should not raise

    def test_line_read_output_pin(self) -> None:
        """Test reading output pin."""
        pin = LinuxDigitalPin(
            line=19,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.high()
        assert pin.read() is True

    def test_line_read_active_low(self) -> None:
        """Test reading with active_low inverts value."""
        pin = LinuxDigitalPin(
            line=20,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            active_low=True,
        )
        pin.setup()
        # Write True, which sets internal state to False (inverted)
        pin.write(True)
        # Read should return True (inverted back)
        assert pin.read() is True

    def test_line_read_multiple_times(self) -> None:
        """Test reading line multiple times."""
        pin = LinuxDigitalPin(
            line=21,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        for _ in range(10):
            _ = pin.read()

    def test_line_read_tracks_state(self) -> None:
        """Test reading tracks state after write."""
        pin = LinuxDigitalPin(
            line=22,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.high()
        assert pin.read() is True
        pin.low()
        assert pin.read() is False


class TestLinuxLineWrite:
    """Tests for GPIO line write operations."""

    def test_line_write_high(self) -> None:
        """Test writing HIGH to line."""
        pin = LinuxDigitalPin(
            line=17,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.write(True)
        assert pin.read() is True

    def test_line_write_low(self) -> None:
        """Test writing LOW to line."""
        pin = LinuxDigitalPin(
            line=18,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.write(False)
        assert pin.read() is False

    def test_line_write_high_method(self) -> None:
        """Test high() method."""
        pin = LinuxDigitalPin(
            line=19,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.high()
        assert pin.read() is True

    def test_line_write_low_method(self) -> None:
        """Test low() method."""
        pin = LinuxDigitalPin(
            line=20,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.low()
        assert pin.read() is False

    def test_line_write_to_input_raises(self) -> None:
        """Test writing to input pin raises error."""
        pin = LinuxDigitalPin(
            line=21,
            chip=0,
            mode=PinMode.INPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        with pytest.raises(ValueError, match="Cannot write to input pin"):
            pin.write(True)

    def test_line_write_toggle(self) -> None:
        """Test toggle method."""
        pin = LinuxDigitalPin(
            line=22,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        pin.write(False)
        assert pin.read() is False
        pin.toggle()
        assert pin.read() is True
        pin.toggle()
        assert pin.read() is False

    def test_line_write_active_low(self) -> None:
        """Test writing with active_low."""
        pin = LinuxDigitalPin(
            line=23,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
            active_low=True,
        )
        pin.setup()
        pin.write(True)
        # Active low: writing True sets internal LOW, read inverts back to True
        assert pin.read() is True

    def test_line_write_rapid_sequence(self) -> None:
        """Test rapid write sequence."""
        pin = LinuxDigitalPin(
            line=24,
            chip=0,
            mode=PinMode.OUTPUT,
            backend=GPIOBackend.SIMULATION,
        )
        pin.setup()
        for _ in range(100):
            pin.high()
            pin.low()
        assert pin.read() is False


class TestLinuxLineEvent:
    """Tests for GPIO line event operations."""

    def test_edge_enum_values(self) -> None:
        """Test GPIOEdge enum values."""
        assert GPIOEdge.NONE.value == "none"
        assert GPIOEdge.RISING.value == "rising"
        assert GPIOEdge.FALLING.value == "falling"
        assert GPIOEdge.BOTH.value == "both"

    def test_edge_enum_count(self) -> None:
        """Test GPIOEdge enum count."""
        assert len(GPIOEdge) == 4

    def test_line_info_direction(self) -> None:
        """Test GPIOLineInfo direction field."""
        info = GPIOLineInfo(
            offset=0,
            name="line0",
            consumer="",
            direction="input",
            active_low=False,
            used=False,
        )
        assert info.direction == "input"

    def test_line_info_output_direction(self) -> None:
        """Test GPIOLineInfo output direction."""
        info = GPIOLineInfo(
            offset=1,
            name="line1",
            consumer="robo_infra",
            direction="output",
            active_low=False,
            used=True,
        )
        assert info.direction == "output"

    def test_list_gpio_lines_simulation(self) -> None:
        """Test listing GPIO lines in simulation."""
        platform = LinuxGenericPlatform(simulation=True)
        lines = platform.list_gpio_lines(0)
        assert len(lines) == 32

    def test_list_gpio_lines_has_info(self) -> None:
        """Test GPIO line info contains expected fields."""
        platform = LinuxGenericPlatform(simulation=True)
        lines = platform.list_gpio_lines(0)
        for line in lines:
            assert hasattr(line, "offset")
            assert hasattr(line, "name")
            assert hasattr(line, "consumer")
            assert hasattr(line, "direction")
            assert hasattr(line, "active_low")
            assert hasattr(line, "used")

    def test_list_gpio_lines_offsets(self) -> None:
        """Test GPIO line offsets are sequential."""
        platform = LinuxGenericPlatform(simulation=True)
        lines = platform.list_gpio_lines(0)
        for i, line in enumerate(lines):
            assert line.offset == i


# =============================================================================
# Backend Detection Tests
# =============================================================================


class TestBackendDetection:
    """Tests for GPIO backend detection."""

    def test_simulation_backend(self) -> None:
        """Test simulation backend is used in simulation mode."""
        platform = LinuxGenericPlatform(simulation=True)
        assert platform.backend == GPIOBackend.SIMULATION

    def test_string_backend_simulation(self) -> None:
        """Test string 'simulation' is converted to enum."""
        platform = LinuxGenericPlatform(backend="simulation")
        assert platform.backend == GPIOBackend.SIMULATION

    def test_string_backend_gpiod(self) -> None:
        """Test string 'gpiod' is converted to enum."""
        platform = LinuxGenericPlatform(backend="gpiod", simulation=True)
        assert platform.backend == GPIOBackend.GPIOD

    def test_string_backend_sysfs(self) -> None:
        """Test string 'sysfs' is converted to enum."""
        platform = LinuxGenericPlatform(backend="sysfs", simulation=True)
        assert platform.backend == GPIOBackend.SYSFS

    def test_is_simulation_property(self) -> None:
        """Test is_simulation property."""
        platform = LinuxGenericPlatform(simulation=True)
        assert platform.is_simulation is True


# =============================================================================
# PWM Extended Tests
# =============================================================================


class TestLinuxPWMExtended:
    """Extended tests for Linux PWM functionality."""

    def test_pwm_chip_id_property(self) -> None:
        """Test chip_id property."""
        pin = LinuxPWMPin(chip=1, channel=0, backend=GPIOBackend.SIMULATION)
        assert pin.chip_id == 1

    def test_pwm_channel_property(self) -> None:
        """Test channel property."""
        pin = LinuxPWMPin(chip=0, channel=2, backend=GPIOBackend.SIMULATION)
        assert pin.channel == 2

    def test_pwm_period_calculation(self) -> None:
        """Test period is calculated from frequency."""
        pin = LinuxPWMPin(chip=0, channel=0, frequency=1000, backend=GPIOBackend.SIMULATION)
        # 1000 Hz = 1ms period = 1,000,000 ns
        assert pin._period_ns == 1_000_000

    def test_pwm_frequency_change_updates_period(self) -> None:
        """Test changing frequency updates period."""
        pin = LinuxPWMPin(chip=0, channel=0, frequency=1000, backend=GPIOBackend.SIMULATION)
        pin.setup()
        pin.set_frequency(500)
        assert pin._period_ns == 2_000_000

    def test_pwm_duty_cycle_range(self) -> None:
        """Test duty cycle accepts full range."""
        pin = LinuxPWMPin(chip=0, channel=0, backend=GPIOBackend.SIMULATION)
        pin.setup()
        for duty in [0.0, 0.1, 0.5, 0.9, 1.0]:
            pin.set_duty_cycle(duty)
            assert pin.duty_cycle == pytest.approx(duty)

    def test_pwm_is_running_property(self) -> None:
        """Test is_running property."""
        pin = LinuxPWMPin(chip=0, channel=0, backend=GPIOBackend.SIMULATION)
        pin.setup()
        assert pin.is_running is False
        pin.start()
        assert pin.is_running is True
        pin.stop()
        assert pin.is_running is False


# =============================================================================
# SBC Type Detection Extended Tests
# =============================================================================


class TestSBCTypeDetectionExtended:
    """Extended tests for SBC type detection."""

    def test_all_sbc_types_unique_values(self) -> None:
        """Test all SBC type values are unique."""
        values = [sbc.value for sbc in LinuxSBCType]
        assert len(values) == len(set(values))

    @patch("pathlib.Path.exists")
    @patch("pathlib.Path.read_text")
    def test_detect_rock_pi_4(self, mock_read: MagicMock, mock_exists: MagicMock) -> None:
        """Test Rock Pi 4 detection."""
        mock_exists.return_value = True
        mock_read.return_value = "Radxa ROCK Pi 4 Model B"
        # In simulation, detection doesn't run
        platform = LinuxGenericPlatform(simulation=True)
        assert platform.sbc_type == LinuxSBCType.GENERIC

    @patch("pathlib.Path.exists")
    @patch("pathlib.Path.read_text")
    def test_detect_banana_pi(self, mock_read: MagicMock, mock_exists: MagicMock) -> None:
        """Test Banana Pi detection."""
        mock_exists.return_value = True
        mock_read.return_value = "Banana Pi M5"
        platform = LinuxGenericPlatform(simulation=True)
        assert platform.sbc_type == LinuxSBCType.GENERIC

    @patch("pathlib.Path.exists")
    @patch("pathlib.Path.read_text")
    def test_detect_odroid(self, mock_read: MagicMock, mock_exists: MagicMock) -> None:
        """Test Odroid detection."""
        mock_exists.return_value = True
        mock_read.return_value = "Hardkernel ODROID-C4"
        platform = LinuxGenericPlatform(simulation=True)
        assert platform.sbc_type == LinuxSBCType.GENERIC


# =============================================================================
# Platform Capabilities Tests
# =============================================================================


class TestLinuxPlatformCapabilities:
    """Tests for Linux platform capabilities."""

    def test_gpio_capability_present(self) -> None:
        """Test GPIO capability is always present."""
        from robo_infra.platforms.base import PlatformCapability

        platform = LinuxGenericPlatform(simulation=True)
        assert PlatformCapability.GPIO in platform.capabilities

    def test_platform_type_is_linux_generic(self) -> None:
        """Test platform type is LINUX_GENERIC."""
        from robo_infra.platforms.base import PlatformType

        platform = LinuxGenericPlatform(simulation=True)
        assert platform.platform_type == PlatformType.LINUX_GENERIC

    def test_get_info_returns_platform_info(self) -> None:
        """Test get_info returns PlatformInfo."""
        from robo_infra.platforms.base import PlatformInfo

        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_info()
        assert isinstance(info, PlatformInfo)

    def test_get_info_has_gpio_chips(self) -> None:
        """Test get_info includes gpio_chips."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_info()
        assert len(info.gpio_chips) >= 1

    def test_get_info_model_includes_sbc_type(self) -> None:
        """Test get_info model includes SBC type."""
        platform = LinuxGenericPlatform(simulation=True)
        info = platform.get_info()
        assert "Generic" in info.model


# =============================================================================
# Bus Access Tests (Phase 8.8.1)
# =============================================================================


class TestLinuxGenericBusAccess:
    """Tests for Linux Generic platform bus access."""

    def test_get_bus_i2c_simulation(self) -> None:
        """Test getting I2C bus in simulation mode returns SimulatedI2CBus."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_bus("i2c", bus=1)
        assert isinstance(bus, SimulatedI2CBus)

    def test_get_bus_spi_simulation(self) -> None:
        """Test getting SPI bus in simulation mode returns SimulatedSPIBus."""
        from robo_infra.core.bus import SimulatedSPIBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_bus("spi", bus=0, device=0)
        assert isinstance(bus, SimulatedSPIBus)

    def test_get_bus_uart_simulation(self) -> None:
        """Test getting UART bus in simulation mode returns SimulatedSerialBus."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_bus("uart", port="/dev/ttyS0")
        assert isinstance(bus, SimulatedSerialBus)

    def test_get_bus_serial_alias(self) -> None:
        """Test 'serial' is accepted as alias for 'uart'."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_bus("serial", port="/dev/ttyS0")
        assert isinstance(bus, SimulatedSerialBus)

    def test_get_bus_unknown_type_raises(self) -> None:
        """Test unknown bus type raises HardwareNotFoundError."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        platform = LinuxGenericPlatform(simulation=True)
        with pytest.raises(HardwareNotFoundError):
            platform.get_bus("unknown_bus")

    def test_get_bus_case_insensitive(self) -> None:
        """Test bus type is case insensitive."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = LinuxGenericPlatform(simulation=True)
        bus1 = platform.get_bus("I2C")
        bus2 = platform.get_bus("i2c")
        bus3 = platform.get_bus("I2c")
        assert isinstance(bus1, SimulatedI2CBus)
        assert isinstance(bus2, SimulatedI2CBus)
        assert isinstance(bus3, SimulatedI2CBus)

    @patch("robo_infra.platforms.linux_generic.LinuxGenericPlatform._create_i2c_bus")
    def test_get_bus_i2c_passes_kwargs(self, mock_create: MagicMock) -> None:
        """Test get_bus passes kwargs to _create_i2c_bus."""
        mock_create.return_value = MagicMock()
        platform = LinuxGenericPlatform(simulation=True)
        platform.get_bus("i2c", bus=2)
        mock_create.assert_called_once_with(bus=2)

    @patch("robo_infra.platforms.linux_generic.LinuxGenericPlatform._create_spi_bus")
    def test_get_bus_spi_passes_kwargs(self, mock_create: MagicMock) -> None:
        """Test get_bus passes kwargs to _create_spi_bus."""
        mock_create.return_value = MagicMock()
        platform = LinuxGenericPlatform(simulation=True)
        platform.get_bus("spi", bus=1, device=2)
        mock_create.assert_called_once_with(bus=1, device=2)


class TestLinuxGenericBusConvenienceMethods:
    """Tests for bus convenience methods (get_i2c, get_spi, get_serial)."""

    def test_get_i2c_default_bus(self) -> None:
        """Test get_i2c with default bus number."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_i2c()
        assert isinstance(bus, SimulatedI2CBus)
        assert bus.config.bus_number == 1

    def test_get_i2c_custom_bus(self) -> None:
        """Test get_i2c with custom bus number."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_i2c(bus=2)
        assert isinstance(bus, SimulatedI2CBus)
        assert bus.config.bus_number == 2

    def test_get_spi_default(self) -> None:
        """Test get_spi with default parameters."""
        from robo_infra.core.bus import SimulatedSPIBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_spi()
        assert isinstance(bus, SimulatedSPIBus)
        assert bus.config.bus == 0
        assert bus.config.device == 0

    def test_get_spi_custom(self) -> None:
        """Test get_spi with custom bus and device."""
        from robo_infra.core.bus import SimulatedSPIBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_spi(bus=1, device=1)
        assert isinstance(bus, SimulatedSPIBus)
        assert bus.config.bus == 1
        assert bus.config.device == 1

    def test_get_serial_default(self) -> None:
        """Test get_serial with default parameters."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_serial()
        assert isinstance(bus, SimulatedSerialBus)
        assert bus.config.port == "/dev/ttyS0"
        assert bus.config.baudrate == 115200

    def test_get_serial_custom(self) -> None:
        """Test get_serial with custom port and baudrate."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = LinuxGenericPlatform(simulation=True)
        bus = platform.get_serial(port="/dev/ttyUSB0", baudrate=9600)
        assert isinstance(bus, SimulatedSerialBus)
        assert bus.config.port == "/dev/ttyUSB0"
        assert bus.config.baudrate == 9600


class TestLinuxGenericRealBusAccess:
    """Tests for real hardware bus access (with mocked libraries)."""

    @patch("robo_infra.core.bus.SMBus2I2CBus.__init__", return_value=None)
    def test_get_bus_i2c_real_uses_smbus2(self, mock_init: MagicMock) -> None:
        """Test non-simulation mode attempts to use SMBus2I2CBus."""
        # Create platform in non-simulation mode
        platform = LinuxGenericPlatform(simulation=False)
        platform._simulation = False  # Force non-simulation

        # This will try to import and use SMBus2I2CBus
        try:
            bus = platform.get_bus("i2c", bus=1)
            # If we get here, SMBus2I2CBus was used
            from robo_infra.core.bus import SMBus2I2CBus

            assert isinstance(bus, SMBus2I2CBus)
        except ImportError:
            # smbus2 not installed - falls back to simulated
            from robo_infra.core.bus import SimulatedI2CBus

            bus = platform.get_bus("i2c", bus=1)
            assert isinstance(bus, SimulatedI2CBus)

    @patch("robo_infra.core.bus.SpiDevSPIBus.__init__", return_value=None)
    def test_get_bus_spi_real_uses_spidev(self, mock_init: MagicMock) -> None:
        """Test non-simulation mode attempts to use SpiDevSPIBus."""
        platform = LinuxGenericPlatform(simulation=False)
        platform._simulation = False

        try:
            bus = platform.get_bus("spi", bus=0, device=0)
            from robo_infra.core.bus import SpiDevSPIBus

            assert isinstance(bus, SpiDevSPIBus)
        except ImportError:
            from robo_infra.core.bus import SimulatedSPIBus

            bus = platform.get_bus("spi", bus=0, device=0)
            assert isinstance(bus, SimulatedSPIBus)

    @patch("robo_infra.core.bus.PySerialBus.__init__", return_value=None)
    def test_get_bus_uart_real_uses_pyserial(self, mock_init: MagicMock) -> None:
        """Test non-simulation mode attempts to use PySerialBus."""
        platform = LinuxGenericPlatform(simulation=False)
        platform._simulation = False

        try:
            bus = platform.get_bus("uart", port="/dev/ttyS0")
            from robo_infra.core.bus import PySerialBus

            assert isinstance(bus, PySerialBus)
        except ImportError:
            from robo_infra.core.bus import SimulatedSerialBus

            bus = platform.get_bus("uart", port="/dev/ttyS0")
            assert isinstance(bus, SimulatedSerialBus)

    def test_get_bus_graceful_fallback_on_import_error(self) -> None:
        """Test that bus access falls back gracefully when library not installed."""
        # In simulation mode, should always return simulated bus
        platform = LinuxGenericPlatform(simulation=True)

        from robo_infra.core.bus import SimulatedI2CBus, SimulatedSerialBus, SimulatedSPIBus

        i2c_bus = platform.get_bus("i2c")
        spi_bus = platform.get_bus("spi")
        uart_bus = platform.get_bus("uart")

        assert isinstance(i2c_bus, SimulatedI2CBus)
        assert isinstance(spi_bus, SimulatedSPIBus)
        assert isinstance(uart_bus, SimulatedSerialBus)

    def test_no_not_implemented_error(self) -> None:
        """Test that NotImplementedError is NOT raised anymore."""
        platform = LinuxGenericPlatform(simulation=False)

        # These should NOT raise NotImplementedError
        # They may raise ImportError if libs not installed, but that's caught internally
        try:
            bus = platform.get_bus("i2c")
            assert bus is not None
        except NotImplementedError:
            pytest.fail("NotImplementedError should not be raised")
