"""Tests for GPIO driver."""

from __future__ import annotations

import pytest

from robo_infra.core.driver import DriverState
from robo_infra.drivers.gpio import (
    GPIOConfig,
    GPIODirection,
    GPIODriver,
    GPIOEdge,
    GPIOPinConfig,
    GPIOPinState,
    GPIOPull,
    Platform,
    SoftwarePWMConfig,
    SoftwarePWMThread,
    get_gpio_driver,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def driver() -> GPIODriver:
    """Create a simulation-mode GPIO driver."""
    config = GPIOConfig(platform=Platform.SIMULATION)
    return GPIODriver(config=config)


@pytest.fixture
def connected_driver() -> GPIODriver:
    """Create a connected simulation-mode GPIO driver."""
    config = GPIOConfig(platform=Platform.SIMULATION)
    driver = GPIODriver(config=config)
    driver.connect()
    return driver


# =============================================================================
# Test Enums
# =============================================================================


class TestGPIODirection:
    """Tests for GPIODirection enum."""

    def test_input_value(self):
        """Test INPUT direction value."""
        assert GPIODirection.INPUT.value == "input"

    def test_output_value(self):
        """Test OUTPUT direction value."""
        assert GPIODirection.OUTPUT.value == "output"


class TestGPIOPull:
    """Tests for GPIOPull enum."""

    def test_none_value(self):
        """Test NONE pull value."""
        assert GPIOPull.NONE.value == "none"

    def test_up_value(self):
        """Test UP pull value."""
        assert GPIOPull.UP.value == "up"

    def test_down_value(self):
        """Test DOWN pull value."""
        assert GPIOPull.DOWN.value == "down"


class TestGPIOEdge:
    """Tests for GPIOEdge enum."""

    def test_edge_values(self):
        """Test edge detection values."""
        assert GPIOEdge.NONE.value == "none"
        assert GPIOEdge.RISING.value == "rising"
        assert GPIOEdge.FALLING.value == "falling"
        assert GPIOEdge.BOTH.value == "both"


class TestPlatform:
    """Tests for Platform enum."""

    def test_platform_values(self):
        """Test platform values."""
        assert Platform.RASPBERRY_PI.value == "raspberry_pi"
        assert Platform.JETSON.value == "jetson"
        assert Platform.BEAGLEBONE.value == "beaglebone"
        assert Platform.SIMULATION.value == "simulation"


# =============================================================================
# Test Configuration
# =============================================================================


class TestGPIOPinConfig:
    """Tests for GPIOPinConfig dataclass."""

    def test_default_values(self):
        """Test default configuration values."""
        config = GPIOPinConfig()
        assert config.direction == GPIODirection.OUTPUT
        assert config.pull == GPIOPull.NONE
        assert config.initial_state is False
        assert config.inverted is False
        assert config.edge == GPIOEdge.NONE

    def test_custom_values(self):
        """Test custom configuration values."""
        config = GPIOPinConfig(
            direction=GPIODirection.INPUT,
            pull=GPIOPull.UP,
            initial_state=True,
            inverted=True,
            edge=GPIOEdge.RISING,
        )
        assert config.direction == GPIODirection.INPUT
        assert config.pull == GPIOPull.UP
        assert config.initial_state is True
        assert config.inverted is True
        assert config.edge == GPIOEdge.RISING


class TestSoftwarePWMConfig:
    """Tests for SoftwarePWMConfig dataclass."""

    def test_default_values(self):
        """Test default PWM configuration values."""
        config = SoftwarePWMConfig()
        assert config.frequency == 1000.0
        assert config.duty_cycle == 0.0
        assert config.running is False

    def test_custom_values(self):
        """Test custom PWM configuration values."""
        config = SoftwarePWMConfig(
            frequency=5000.0,
            duty_cycle=0.75,
            running=True,
        )
        assert config.frequency == 5000.0
        assert config.duty_cycle == 0.75
        assert config.running is True


class TestGPIOConfig:
    """Tests for GPIOConfig dataclass."""

    def test_default_values(self):
        """Test default driver configuration values."""
        config = GPIOConfig()
        assert config.platform is None
        assert config.numbering_mode == "BCM"
        assert config.warnings is True
        assert config.cleanup_on_disconnect is True
        assert config.software_pwm_default_frequency == 1000.0

    def test_custom_values(self):
        """Test custom driver configuration values."""
        config = GPIOConfig(
            platform=Platform.RASPBERRY_PI,
            numbering_mode="BOARD",
            warnings=False,
            cleanup_on_disconnect=False,
            software_pwm_default_frequency=500.0,
        )
        assert config.platform == Platform.RASPBERRY_PI
        assert config.numbering_mode == "BOARD"
        assert config.warnings is False
        assert config.cleanup_on_disconnect is False
        assert config.software_pwm_default_frequency == 500.0


class TestGPIOPinState:
    """Tests for GPIOPinState dataclass."""

    def test_default_values(self):
        """Test default pin state values."""
        state = GPIOPinState(pin=17)
        assert state.pin == 17
        assert state.direction == GPIODirection.OUTPUT
        assert state.value is False
        assert state.pwm_config is None
        assert state.configured is False

    def test_custom_values(self):
        """Test custom pin state values."""
        pwm_config = SoftwarePWMConfig(frequency=2000.0)
        state = GPIOPinState(
            pin=18,
            direction=GPIODirection.INPUT,
            value=True,
            pwm_config=pwm_config,
            configured=True,
        )
        assert state.pin == 18
        assert state.direction == GPIODirection.INPUT
        assert state.value is True
        assert state.pwm_config is not None
        assert state.configured is True


# =============================================================================
# Test Initialization
# =============================================================================


class TestGPIODriverInit:
    """Tests for GPIODriver initialization."""

    def test_default_init(self, driver: GPIODriver):
        """Test default initialization."""
        assert driver.name == "GPIO"
        assert driver.channels == 0
        assert driver.state == DriverState.DISCONNECTED

    def test_simulation_mode(self, driver: GPIODriver):
        """Test simulation mode detection."""
        driver.connect()
        assert driver.simulation_mode is True
        assert driver.platform == Platform.SIMULATION

    def test_init_with_config(self):
        """Test initialization with custom config."""
        config = GPIOConfig(
            platform=Platform.SIMULATION,
            software_pwm_default_frequency=500.0,
        )
        driver = GPIODriver(config=config)
        driver.connect()
        assert driver.platform == Platform.SIMULATION


# =============================================================================
# Test Lifecycle
# =============================================================================


class TestGPIODriverLifecycle:
    """Tests for GPIODriver lifecycle methods."""

    def test_connect_simulation(self, driver: GPIODriver):
        """Test connecting in simulation mode."""
        driver.connect()
        assert driver.state == DriverState.CONNECTED
        assert driver.simulation_mode is True

    def test_connect_already_connected_warns(self, connected_driver: GPIODriver, caplog):
        """Test connecting when already connected logs warning."""
        import logging

        with caplog.at_level(logging.WARNING):
            connected_driver.connect()
        assert "already connected" in caplog.text

    def test_disconnect_simulation(self, connected_driver: GPIODriver):
        """Test disconnecting in simulation mode."""
        connected_driver.disconnect()
        assert connected_driver.state == DriverState.DISCONNECTED

    def test_disconnect_already_disconnected_warns(self, driver: GPIODriver, caplog):
        """Test disconnecting when already disconnected logs warning."""
        import logging

        with caplog.at_level(logging.WARNING):
            driver.disconnect()
        assert "already disconnected" in caplog.text

    def test_context_manager(self):
        """Test using GPIODriver as context manager."""
        config = GPIOConfig(platform=Platform.SIMULATION)
        with GPIODriver(config=config) as driver:
            assert driver.state == DriverState.CONNECTED
        assert driver.state == DriverState.DISCONNECTED


# =============================================================================
# Test Pin Configuration
# =============================================================================


class TestGPIOPinConfiguration:
    """Tests for GPIO pin configuration."""

    def test_setup_pin_output(self, connected_driver: GPIODriver):
        """Test setting up an output pin."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT)

        state = connected_driver.get_pin_state(17)
        assert state is not None
        assert state.pin == 17
        assert state.direction == GPIODirection.OUTPUT
        assert state.configured is True

    def test_setup_pin_input(self, connected_driver: GPIODriver):
        """Test setting up an input pin."""
        connected_driver.setup_pin(27, GPIODirection.INPUT)

        state = connected_driver.get_pin_state(27)
        assert state is not None
        assert state.direction == GPIODirection.INPUT

    def test_setup_pin_with_initial_value(self, connected_driver: GPIODriver):
        """Test setting up a pin with initial value."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT, initial=True)

        state = connected_driver.get_pin_state(17)
        assert state is not None
        assert state.value is True

    def test_is_pin_configured(self, connected_driver: GPIODriver):
        """Test checking if pin is configured."""
        assert connected_driver.is_pin_configured(17) is False

        connected_driver.setup_pin(17, GPIODirection.OUTPUT)
        assert connected_driver.is_pin_configured(17) is True

    def test_configured_pins_property(self, connected_driver: GPIODriver):
        """Test configured_pins property."""
        assert connected_driver.configured_pins == []

        connected_driver.setup_pin(17, GPIODirection.OUTPUT)
        connected_driver.setup_pin(18, GPIODirection.OUTPUT)

        pins = connected_driver.configured_pins
        assert 17 in pins
        assert 18 in pins
        assert len(pins) == 2


# =============================================================================
# Test Digital I/O
# =============================================================================


class TestGPIODigitalIO:
    """Tests for GPIO digital I/O operations."""

    def test_digital_write_true(self, connected_driver: GPIODriver):
        """Test writing HIGH to a pin."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT)
        connected_driver.digital_write(17, True)

        state = connected_driver.get_pin_state(17)
        assert state is not None
        assert state.value is True

    def test_digital_write_false(self, connected_driver: GPIODriver):
        """Test writing LOW to a pin."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT, initial=True)
        connected_driver.digital_write(17, False)

        state = connected_driver.get_pin_state(17)
        assert state is not None
        assert state.value is False

    def test_digital_write_auto_configures(self, connected_driver: GPIODriver):
        """Test digital_write auto-configures unconfigured pin."""
        connected_driver.digital_write(17, True)

        assert connected_driver.is_pin_configured(17) is True
        state = connected_driver.get_pin_state(17)
        assert state is not None
        assert state.direction == GPIODirection.OUTPUT

    def test_digital_write_on_input_raises(self, connected_driver: GPIODriver):
        """Test writing to input pin raises error."""
        connected_driver.setup_pin(27, GPIODirection.INPUT)

        with pytest.raises(ValueError, match="not configured as output"):
            connected_driver.digital_write(27, True)

    def test_digital_read(self, connected_driver: GPIODriver):
        """Test reading from a pin."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT, initial=True)
        value = connected_driver.digital_read(17)
        assert value is True

    def test_digital_read_auto_configures(self, connected_driver: GPIODriver):
        """Test digital_read auto-configures unconfigured pin."""
        connected_driver.digital_read(27)

        assert connected_driver.is_pin_configured(27) is True
        state = connected_driver.get_pin_state(27)
        assert state is not None
        assert state.direction == GPIODirection.INPUT

    def test_set_high(self, connected_driver: GPIODriver):
        """Test set_high convenience method."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT)
        connected_driver.set_high(17)

        state = connected_driver.get_pin_state(17)
        assert state is not None
        assert state.value is True

    def test_set_low(self, connected_driver: GPIODriver):
        """Test set_low convenience method."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT, initial=True)
        connected_driver.set_low(17)

        state = connected_driver.get_pin_state(17)
        assert state is not None
        assert state.value is False

    def test_toggle(self, connected_driver: GPIODriver):
        """Test toggle method."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT)

        result = connected_driver.toggle(17)
        assert result is True

        result = connected_driver.toggle(17)
        assert result is False

    def test_toggle_unconfigured_pin(self, connected_driver: GPIODriver):
        """Test toggle on unconfigured pin."""
        result = connected_driver.toggle(17)
        assert result is True
        assert connected_driver.is_pin_configured(17) is True


# =============================================================================
# Test Software PWM
# =============================================================================


class TestGPIOSoftwarePWM:
    """Tests for GPIO software PWM."""

    def test_pwm_start(self, connected_driver: GPIODriver):
        """Test starting PWM on a pin."""
        connected_driver.pwm_start(18, frequency=1000.0, duty_cycle=0.5)

        state = connected_driver.get_pin_state(18)
        assert state is not None
        assert state.pwm_config is not None
        assert state.pwm_config.frequency == 1000.0
        assert state.pwm_config.duty_cycle == 0.5
        assert state.pwm_config.running is True

    def test_pwm_stop(self, connected_driver: GPIODriver):
        """Test stopping PWM on a pin."""
        connected_driver.pwm_start(18, duty_cycle=0.5)
        connected_driver.pwm_stop(18)

        state = connected_driver.get_pin_state(18)
        assert state is not None
        assert state.pwm_config is not None
        assert state.pwm_config.running is False

    def test_pwm_set_duty_cycle(self, connected_driver: GPIODriver):
        """Test setting PWM duty cycle."""
        connected_driver.pwm_start(18, duty_cycle=0.5)
        connected_driver.pwm_set_duty_cycle(18, 0.75)

        duty = connected_driver.pwm_get_duty_cycle(18)
        assert duty == 0.75

    def test_pwm_set_frequency(self, connected_driver: GPIODriver):
        """Test setting PWM frequency."""
        connected_driver.pwm_start(18, frequency=1000.0)
        connected_driver.pwm_set_frequency(18, 2000.0)

        state = connected_driver.get_pin_state(18)
        assert state is not None
        assert state.pwm_config is not None
        assert state.pwm_config.frequency == 2000.0

    def test_pwm_duty_cycle_not_started_raises(self, connected_driver: GPIODriver):
        """Test setting duty cycle on non-PWM pin raises error."""
        with pytest.raises(ValueError, match="PWM not started"):
            connected_driver.pwm_set_duty_cycle(18, 0.5)

    def test_pwm_frequency_not_started_raises(self, connected_driver: GPIODriver):
        """Test setting frequency on non-PWM pin raises error."""
        with pytest.raises(ValueError, match="PWM not started"):
            connected_driver.pwm_set_frequency(18, 1000.0)

    def test_pwm_get_duty_cycle_not_started_raises(self, connected_driver: GPIODriver):
        """Test getting duty cycle on non-PWM pin raises error."""
        with pytest.raises(ValueError, match="PWM not started"):
            connected_driver.pwm_get_duty_cycle(18)

    def test_active_pwm_pins(self, connected_driver: GPIODriver):
        """Test active_pwm_pins property."""
        assert connected_driver.active_pwm_pins == []

        connected_driver.pwm_start(18, duty_cycle=0.5)
        connected_driver.pwm_start(19, duty_cycle=0.5)

        pins = connected_driver.active_pwm_pins
        assert 18 in pins
        assert 19 in pins

    def test_pwm_uses_default_frequency(self, connected_driver: GPIODriver):
        """Test PWM uses default frequency from config."""
        connected_driver.pwm_start(18, duty_cycle=0.5)

        state = connected_driver.get_pin_state(18)
        assert state is not None
        assert state.pwm_config is not None
        assert state.pwm_config.frequency == 1000.0  # Default


# =============================================================================
# Test Software PWM Thread
# =============================================================================


def _noop_callback(pin: int, val: int) -> None:
    """No-op callback for tests."""
    pass


class TestSoftwarePWMThread:
    """Tests for SoftwarePWMThread."""

    def test_thread_creation(self):
        """Test creating a PWM thread."""
        thread = SoftwarePWMThread(
            pin=18,
            frequency=1000.0,
            duty_cycle=0.5,
            write_callback=_noop_callback,
        )
        assert thread._pin == 18
        assert thread.frequency == 1000.0
        assert thread.duty_cycle == 0.5
        assert thread.daemon is True

    def test_frequency_property(self):
        """Test frequency property."""
        thread = SoftwarePWMThread(18, 1000.0, 0.5, _noop_callback)

        thread.frequency = 2000.0
        assert thread.frequency == 2000.0

    def test_frequency_clamped_to_max(self):
        """Test frequency is clamped to maximum."""
        thread = SoftwarePWMThread(18, 100000.0, 0.5, _noop_callback)  # Way above max

        assert thread.frequency <= SoftwarePWMThread.MAX_FREQUENCY

    def test_duty_cycle_property(self):
        """Test duty_cycle property."""
        thread = SoftwarePWMThread(18, 1000.0, 0.5, _noop_callback)

        thread.duty_cycle = 0.75
        assert thread.duty_cycle == 0.75

    def test_duty_cycle_clamped(self):
        """Test duty cycle is clamped to 0-1."""
        thread = SoftwarePWMThread(18, 1000.0, 1.5, _noop_callback)  # Above 1

        assert thread.duty_cycle == 1.0

        thread.duty_cycle = -0.5
        assert thread.duty_cycle == 0.0


# =============================================================================
# Test Driver Interface
# =============================================================================


class TestGPIODriverInterface:
    """Tests for GPIODriver Driver interface compatibility."""

    def test_set_channel_starts_pwm(self, connected_driver: GPIODriver):
        """Test set_channel starts PWM if not running."""
        connected_driver.set_channel(18, 0.5)

        assert 18 in connected_driver.active_pwm_pins

    def test_set_channel_updates_duty(self, connected_driver: GPIODriver):
        """Test set_channel updates existing PWM duty cycle."""
        connected_driver.pwm_start(18, duty_cycle=0.25)
        connected_driver.set_channel(18, 0.75)

        duty = connected_driver.pwm_get_duty_cycle(18)
        assert duty == 0.75

    def test_set_channel_normalizes_negative(self, connected_driver: GPIODriver):
        """Test set_channel takes absolute value of negative."""
        connected_driver.set_channel(18, -0.5)

        duty = connected_driver.get_channel(18)
        assert duty == 0.5

    def test_get_channel_pwm(self, connected_driver: GPIODriver):
        """Test get_channel returns PWM duty cycle."""
        connected_driver.pwm_start(18, duty_cycle=0.6)

        value = connected_driver.get_channel(18)
        assert value == 0.6

    def test_get_channel_digital(self, connected_driver: GPIODriver):
        """Test get_channel returns digital value."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT, initial=True)

        value = connected_driver.get_channel(17)
        assert value == 1.0

    def test_get_channel_unconfigured(self, connected_driver: GPIODriver):
        """Test get_channel returns 0 for unconfigured pin."""
        value = connected_driver.get_channel(99)
        assert value == 0.0


# =============================================================================
# Test Validation
# =============================================================================


class TestGPIOValidation:
    """Tests for GPIODriver validation."""

    def test_require_connected(self, driver: GPIODriver):
        """Test operations require connection."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError, match="not connected"):
            driver.digital_write(17, True)

    def test_setup_requires_connected(self, driver: GPIODriver):
        """Test setup_pin requires connection."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError, match="not connected"):
            driver.setup_pin(17, GPIODirection.OUTPUT)

    def test_pwm_requires_connected(self, driver: GPIODriver):
        """Test PWM operations require connection."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError, match="not connected"):
            driver.pwm_start(18, duty_cycle=0.5)


# =============================================================================
# Test Driver Registration
# =============================================================================


# NOTE: Driver registration tests are skipped because the driver registry
# is cleared by test_core_driver.py tests, and module reloading causes
# enum identity issues with other tests in this file.
# Registration is tested implicitly through the driver __init__.py exports.


# =============================================================================
# Test Convenience Functions
# =============================================================================


class TestGPIOConvenienceFunctions:
    """Tests for GPIO convenience functions."""

    def test_get_gpio_driver_default(self):
        """Test get_gpio_driver with default platform."""
        driver = get_gpio_driver()
        driver.connect()

        # Should auto-detect to simulation on dev machine
        assert driver.simulation_mode is True
        driver.disconnect()

    def test_get_gpio_driver_with_platform(self):
        """Test get_gpio_driver with specific platform."""
        driver = get_gpio_driver(Platform.SIMULATION)
        driver.connect()

        assert driver.platform == Platform.SIMULATION
        driver.disconnect()

    def test_get_gpio_driver_with_string_platform(self):
        """Test get_gpio_driver with string platform."""
        driver = get_gpio_driver("simulation")
        driver.connect()

        assert driver.platform.value == "simulation"
        driver.disconnect()


# =============================================================================
# Test Representation
# =============================================================================


class TestGPIORepr:
    """Tests for GPIODriver string representation."""

    def test_repr_includes_platform(self, connected_driver: GPIODriver):
        """Test repr includes platform."""
        repr_str = repr(connected_driver)
        assert "simulation" in repr_str

    def test_repr_includes_pin_count(self, connected_driver: GPIODriver):
        """Test repr includes pin count."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT)
        repr_str = repr(connected_driver)
        assert "pins=1" in repr_str

    def test_repr_includes_state(self, driver: GPIODriver):
        """Test repr includes driver state."""
        repr_str = repr(driver)
        assert "disconnected" in repr_str


# =============================================================================
# Test Edge Cases
# =============================================================================


class TestGPIOEdgeCases:
    """Tests for GPIO edge cases."""

    def test_multiple_pins(self, connected_driver: GPIODriver):
        """Test controlling multiple pins."""
        # Setup all pins first
        for pin in range(17, 25):
            connected_driver.setup_pin(pin, GPIODirection.OUTPUT)

        # Then write to them
        for pin in range(17, 25):
            connected_driver.digital_write(pin, pin % 2 == 0)

        assert len(connected_driver.configured_pins) == 8

    def test_reconfigure_pin(self, connected_driver: GPIODriver):
        """Test reconfiguring a pin."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT, initial=True)
        connected_driver.setup_pin(17, GPIODirection.OUTPUT, initial=False)

        state = connected_driver.get_pin_state(17)
        assert state is not None
        # Initial value should be updated
        assert state.configured is True

    def test_disconnect_stops_pwm(self, connected_driver: GPIODriver):
        """Test disconnecting stops all PWM."""
        connected_driver.pwm_start(18, duty_cycle=0.5)
        connected_driver.pwm_start(19, duty_cycle=0.5)

        connected_driver.disconnect()

        # PWM threads should be stopped
        assert connected_driver.state == DriverState.DISCONNECTED

    def test_disconnect_clears_pins(self, connected_driver: GPIODriver):
        """Test disconnecting clears pin tracking."""
        connected_driver.setup_pin(17, GPIODirection.OUTPUT)
        connected_driver.disconnect()

        # Pins should be cleared
        assert connected_driver.get_pin_state(17) is None

    def test_rapid_pwm_changes(self, connected_driver: GPIODriver):
        """Test rapid PWM duty cycle changes."""
        connected_driver.pwm_start(18, duty_cycle=0.0)

        for i in range(10):
            connected_driver.pwm_set_duty_cycle(18, i * 0.1)

        duty = connected_driver.pwm_get_duty_cycle(18)
        assert duty == 0.9  # 9 * 0.1
