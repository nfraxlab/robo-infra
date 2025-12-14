"""Tests for L298N dual H-bridge motor driver."""

from __future__ import annotations

import pytest

from robo_infra.drivers.l298n import (
    BrakeMode,
    L298N,
    L298NConfig,
    MotorChannel,
    MotorConfig,
    MotorDirection,
    MotorState,
)
from robo_infra.core.driver import DriverState, get_driver, clear_driver_registry


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def driver() -> L298N:
    """Create a simulation-mode L298N driver."""
    return L298N()


@pytest.fixture
def connected_driver() -> L298N:
    """Create a connected simulation-mode L298N driver."""
    driver = L298N()
    driver.connect()
    return driver


@pytest.fixture(autouse=True)
def reset_registry():
    """Reset driver registry after each test."""
    yield
    # Don't clear, as L298N needs to stay registered


# =============================================================================
# Test Enums and Constants
# =============================================================================


class TestMotorDirection:
    """Tests for MotorDirection enum."""

    def test_forward_value(self):
        """Test FORWARD direction value."""
        assert MotorDirection.FORWARD.value == "forward"

    def test_reverse_value(self):
        """Test REVERSE direction value."""
        assert MotorDirection.REVERSE.value == "reverse"

    def test_brake_value(self):
        """Test BRAKE direction value."""
        assert MotorDirection.BRAKE.value == "brake"

    def test_coast_value(self):
        """Test COAST direction value."""
        assert MotorDirection.COAST.value == "coast"


class TestMotorChannel:
    """Tests for MotorChannel enum."""

    def test_channel_a_value(self):
        """Test motor channel A value."""
        assert MotorChannel.A == 0

    def test_channel_b_value(self):
        """Test motor channel B value."""
        assert MotorChannel.B == 1

    def test_channel_int_conversion(self):
        """Test channel can be converted to int."""
        assert int(MotorChannel.A) == 0
        assert int(MotorChannel.B) == 1


class TestBrakeMode:
    """Tests for BrakeMode enum."""

    def test_brake_mode_value(self):
        """Test BRAKE mode value."""
        assert BrakeMode.BRAKE.value == "brake"

    def test_coast_mode_value(self):
        """Test COAST mode value."""
        assert BrakeMode.COAST.value == "coast"


# =============================================================================
# Test Configuration
# =============================================================================


class TestMotorConfig:
    """Tests for MotorConfig dataclass."""

    def test_default_values(self):
        """Test default configuration values."""
        config = MotorConfig()
        assert config.inverted is False
        assert config.max_speed == 1.0
        assert config.min_speed == 0.0
        assert config.acceleration is None
        assert config.default_brake_mode == BrakeMode.COAST

    def test_custom_values(self):
        """Test custom configuration values."""
        config = MotorConfig(
            inverted=True,
            max_speed=0.8,
            min_speed=0.1,
            acceleration=2.0,
            default_brake_mode=BrakeMode.BRAKE,
        )
        assert config.inverted is True
        assert config.max_speed == 0.8
        assert config.min_speed == 0.1
        assert config.acceleration == 2.0
        assert config.default_brake_mode == BrakeMode.BRAKE


class TestL298NConfig:
    """Tests for L298NConfig dataclass."""

    def test_default_values(self):
        """Test default configuration values."""
        config = L298NConfig()
        assert config.pwm_frequency == 1000
        assert config.enable_on_connect is True
        assert isinstance(config.motor_a, MotorConfig)
        assert isinstance(config.motor_b, MotorConfig)

    def test_custom_values(self):
        """Test custom configuration values."""
        motor_a_config = MotorConfig(inverted=True)
        motor_b_config = MotorConfig(max_speed=0.5)
        config = L298NConfig(
            pwm_frequency=2000,
            motor_a=motor_a_config,
            motor_b=motor_b_config,
            enable_on_connect=False,
        )
        assert config.pwm_frequency == 2000
        assert config.motor_a.inverted is True
        assert config.motor_b.max_speed == 0.5
        assert config.enable_on_connect is False


class TestMotorState:
    """Tests for MotorState dataclass."""

    def test_default_values(self):
        """Test default state values."""
        state = MotorState()
        assert state.speed == 0.0
        assert state.direction == MotorDirection.COAST
        assert state.enabled is False
        assert state.in1_state is False
        assert state.in2_state is False

    def test_custom_values(self):
        """Test custom state values."""
        state = MotorState(
            speed=0.75,
            direction=MotorDirection.FORWARD,
            enabled=True,
            in1_state=True,
            in2_state=False,
        )
        assert state.speed == 0.75
        assert state.direction == MotorDirection.FORWARD
        assert state.enabled is True
        assert state.in1_state is True
        assert state.in2_state is False


# =============================================================================
# Test Initialization
# =============================================================================


class TestL298NInit:
    """Tests for L298N initialization."""

    def test_default_init_simulation_mode(self, driver: L298N):
        """Test default initialization creates simulation mode driver."""
        assert driver.simulation_mode is True
        assert driver.name == "L298N"
        assert driver.channels == 2
        assert driver.state == DriverState.DISCONNECTED

    def test_init_with_config(self):
        """Test initialization with custom config."""
        config = L298NConfig(pwm_frequency=500)
        driver = L298N(config=config)
        assert driver._pwm_frequency == 500

    def test_num_channels_constant(self):
        """Test NUM_CHANNELS constant."""
        assert L298N.NUM_CHANNELS == 2

    def test_default_pwm_frequency_constant(self):
        """Test DEFAULT_PWM_FREQUENCY constant."""
        assert L298N.DEFAULT_PWM_FREQUENCY == 1000


# =============================================================================
# Test Lifecycle
# =============================================================================


class TestL298NLifecycle:
    """Tests for L298N lifecycle methods."""

    def test_connect_simulation(self, driver: L298N):
        """Test connecting in simulation mode."""
        driver.connect()
        assert driver.state == DriverState.CONNECTED

    def test_connect_already_connected_warns(self, connected_driver: L298N, caplog):
        """Test connecting when already connected logs warning."""
        import logging
        with caplog.at_level(logging.WARNING):
            connected_driver.connect()
        assert "already connected" in caplog.text

    def test_disconnect_simulation(self, connected_driver: L298N):
        """Test disconnecting in simulation mode."""
        connected_driver.disconnect()
        assert connected_driver.state == DriverState.DISCONNECTED

    def test_disconnect_already_disconnected_warns(self, driver: L298N, caplog):
        """Test disconnecting when already disconnected logs warning."""
        import logging
        with caplog.at_level(logging.WARNING):
            driver.disconnect()
        assert "already disconnected" in caplog.text

    def test_context_manager(self):
        """Test using L298N as context manager."""
        with L298N() as driver:
            assert driver.state == DriverState.CONNECTED
        assert driver.state == DriverState.DISCONNECTED

    def test_motors_initialized_to_coast_on_connect(self, connected_driver: L298N):
        """Test motors are initialized to coast state on connect."""
        state_a = connected_driver.get_motor_state(MotorChannel.A)
        state_b = connected_driver.get_motor_state(MotorChannel.B)

        assert state_a.direction == MotorDirection.COAST
        assert state_a.speed == 0.0
        assert state_b.direction == MotorDirection.COAST
        assert state_b.speed == 0.0


# =============================================================================
# Test Motor Control
# =============================================================================


class TestL298NMotorControl:
    """Tests for L298N motor control methods."""

    def test_set_motor_forward(self, connected_driver: L298N):
        """Test setting motor to forward direction."""
        connected_driver.set_motor(0, 0.75, MotorDirection.FORWARD)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.75
        assert state.direction == MotorDirection.FORWARD
        assert state.in1_state is True
        assert state.in2_state is False

    def test_set_motor_reverse(self, connected_driver: L298N):
        """Test setting motor to reverse direction."""
        connected_driver.set_motor(1, 0.5, MotorDirection.REVERSE)
        state = connected_driver.get_motor_state(1)

        assert state.speed == 0.5
        assert state.direction == MotorDirection.REVERSE
        assert state.in1_state is False
        assert state.in2_state is True

    def test_set_motor_brake(self, connected_driver: L298N):
        """Test setting motor to brake mode."""
        connected_driver.set_motor(0, 0.0, MotorDirection.BRAKE)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 1.0  # Full brake
        assert state.direction == MotorDirection.BRAKE
        assert state.in1_state is True
        assert state.in2_state is True

    def test_set_motor_coast(self, connected_driver: L298N):
        """Test setting motor to coast mode."""
        # First set to forward
        connected_driver.set_motor(0, 0.5, MotorDirection.FORWARD)
        # Then coast
        connected_driver.set_motor(0, 0.0, MotorDirection.COAST)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.0
        assert state.direction == MotorDirection.COAST
        assert state.in1_state is False
        assert state.in2_state is False

    def test_forward_convenience_method(self, connected_driver: L298N):
        """Test forward() convenience method."""
        connected_driver.forward(0, 0.8)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.8
        assert state.direction == MotorDirection.FORWARD

    def test_reverse_convenience_method(self, connected_driver: L298N):
        """Test reverse() convenience method."""
        connected_driver.reverse(1, 0.6)
        state = connected_driver.get_motor_state(1)

        assert state.speed == 0.6
        assert state.direction == MotorDirection.REVERSE

    def test_brake_convenience_method(self, connected_driver: L298N):
        """Test brake() convenience method."""
        connected_driver.forward(0, 0.5)
        connected_driver.brake(0)
        state = connected_driver.get_motor_state(0)

        assert state.direction == MotorDirection.BRAKE

    def test_coast_convenience_method(self, connected_driver: L298N):
        """Test coast() convenience method."""
        connected_driver.forward(0, 0.5)
        connected_driver.coast(0)
        state = connected_driver.get_motor_state(0)

        assert state.direction == MotorDirection.COAST
        assert state.speed == 0.0


# =============================================================================
# Test Speed Limits
# =============================================================================


class TestL298NSpeedLimits:
    """Tests for L298N speed limiting."""

    def test_speed_clamped_to_max(self, connected_driver: L298N):
        """Test speed is clamped to 1.0 maximum."""
        connected_driver.set_motor(0, 1.5, MotorDirection.FORWARD)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 1.0

    def test_speed_clamped_to_min(self, connected_driver: L298N):
        """Test speed is clamped to 0.0 minimum."""
        connected_driver.set_motor(0, -0.5, MotorDirection.FORWARD)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 0.0

    def test_max_speed_config_applied(self):
        """Test max_speed from config is applied."""
        config = L298NConfig(motor_a=MotorConfig(max_speed=0.5))
        driver = L298N(config=config)
        driver.connect()

        driver.set_motor(0, 1.0, MotorDirection.FORWARD)
        state = driver.get_motor_state(0)
        assert state.speed == 0.5

    def test_min_speed_config_applied(self):
        """Test min_speed from config is applied."""
        config = L298NConfig(motor_a=MotorConfig(min_speed=0.2))
        driver = L298N(config=config)
        driver.connect()

        driver.set_motor(0, 0.1, MotorDirection.FORWARD)
        state = driver.get_motor_state(0)
        assert state.speed == 0.2

    def test_min_speed_not_applied_when_zero(self):
        """Test min_speed is not applied when speed is zero."""
        config = L298NConfig(motor_a=MotorConfig(min_speed=0.2))
        driver = L298N(config=config)
        driver.connect()

        driver.set_motor(0, 0.0, MotorDirection.FORWARD)
        state = driver.get_motor_state(0)
        assert state.speed == 0.0


# =============================================================================
# Test Direction Inversion
# =============================================================================


class TestL298NInversion:
    """Tests for L298N motor inversion."""

    def test_inverted_motor_forward_becomes_reverse(self):
        """Test inverted motor swaps forward/reverse directions."""
        config = L298NConfig(motor_a=MotorConfig(inverted=True))
        driver = L298N(config=config)
        driver.connect()

        driver.set_motor(0, 0.5, MotorDirection.FORWARD)
        state = driver.get_motor_state(0)

        # Direction stored is the original request
        assert state.direction == MotorDirection.FORWARD
        # But pins are swapped (reverse pattern)
        assert state.in1_state is False
        assert state.in2_state is True

    def test_inverted_motor_reverse_becomes_forward(self):
        """Test inverted motor swaps reverse/forward directions."""
        config = L298NConfig(motor_a=MotorConfig(inverted=True))
        driver = L298N(config=config)
        driver.connect()

        driver.set_motor(0, 0.5, MotorDirection.REVERSE)
        state = driver.get_motor_state(0)

        # Direction stored is the original request
        assert state.direction == MotorDirection.REVERSE
        # But pins are swapped (forward pattern)
        assert state.in1_state is True
        assert state.in2_state is False

    def test_inverted_motor_brake_unchanged(self):
        """Test inverted motor doesn't affect brake."""
        config = L298NConfig(motor_a=MotorConfig(inverted=True))
        driver = L298N(config=config)
        driver.connect()

        driver.set_motor(0, 0.0, MotorDirection.BRAKE)
        state = driver.get_motor_state(0)

        # Brake is the same regardless of inversion
        assert state.in1_state is True
        assert state.in2_state is True


# =============================================================================
# Test Stop Methods
# =============================================================================


class TestL298NStopMethods:
    """Tests for L298N stopping methods."""

    def test_stop_with_brake_mode(self, connected_driver: L298N):
        """Test stop() with brake mode."""
        connected_driver.forward(0, 0.5)
        connected_driver.stop(0, BrakeMode.BRAKE)
        state = connected_driver.get_motor_state(0)

        assert state.direction == MotorDirection.BRAKE

    def test_stop_with_coast_mode(self, connected_driver: L298N):
        """Test stop() with coast mode."""
        connected_driver.forward(0, 0.5)
        connected_driver.stop(0, BrakeMode.COAST)
        state = connected_driver.get_motor_state(0)

        assert state.direction == MotorDirection.COAST

    def test_stop_uses_default_brake_mode(self):
        """Test stop() uses motor's default_brake_mode."""
        config = L298NConfig(motor_a=MotorConfig(default_brake_mode=BrakeMode.BRAKE))
        driver = L298N(config=config)
        driver.connect()

        driver.forward(0, 0.5)
        driver.stop(0)  # No mode specified
        state = driver.get_motor_state(0)

        assert state.direction == MotorDirection.BRAKE

    def test_stop_all(self, connected_driver: L298N):
        """Test stop_all() stops both motors."""
        connected_driver.forward(0, 0.5)
        connected_driver.reverse(1, 0.7)
        connected_driver.stop_all()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.direction == MotorDirection.COAST
        assert state_b.direction == MotorDirection.COAST

    def test_emergency_stop(self, connected_driver: L298N):
        """Test emergency_stop() brakes all motors."""
        connected_driver.forward(0, 0.5)
        connected_driver.reverse(1, 0.7)
        connected_driver.emergency_stop()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.direction == MotorDirection.BRAKE
        assert state_b.direction == MotorDirection.BRAKE
        assert connected_driver.is_enabled is False


# =============================================================================
# Test Enable/Disable
# =============================================================================


class TestL298NEnableDisable:
    """Tests for L298N enable/disable methods."""

    def test_enable_motor(self, connected_driver: L298N):
        """Test enabling a motor."""
        connected_driver.disable_motor(0)
        connected_driver.enable_motor(0)
        state = connected_driver.get_motor_state(0)
        assert state.enabled is True

    def test_disable_motor(self, connected_driver: L298N):
        """Test disabling a motor."""
        connected_driver.disable_motor(0)
        state = connected_driver.get_motor_state(0)
        assert state.enabled is False

    def test_disabled_motor_cannot_be_controlled(self, connected_driver: L298N):
        """Test disabled motor raises error on control."""
        connected_driver.disable_motor(0)

        from robo_infra.core.exceptions import DisabledError
        with pytest.raises(DisabledError):
            connected_driver.set_motor(0, 0.5, MotorDirection.FORWARD)

    def test_enable_all(self, connected_driver: L298N):
        """Test enable_all() enables both motors."""
        connected_driver.disable_all()
        connected_driver.enable_all()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.enabled is True
        assert state_b.enabled is True

    def test_disable_all(self, connected_driver: L298N):
        """Test disable_all() disables both motors."""
        connected_driver.disable_all()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.enabled is False
        assert state_b.enabled is False

    def test_motors_enabled_on_connect_by_default(self, driver: L298N):
        """Test motors are enabled on connect by default."""
        driver.connect()

        state_a = driver.get_motor_state(0)
        state_b = driver.get_motor_state(1)

        assert state_a.enabled is True
        assert state_b.enabled is True

    def test_motors_not_enabled_on_connect_when_configured(self):
        """Test motors not enabled on connect when configured."""
        config = L298NConfig(enable_on_connect=False)
        driver = L298N(config=config)
        driver.connect()

        state_a = driver.get_motor_state(0)
        state_b = driver.get_motor_state(1)

        assert state_a.enabled is False
        assert state_b.enabled is False


# =============================================================================
# Test Channel Interface
# =============================================================================


class TestL298NChannelInterface:
    """Tests for L298N Driver channel interface compatibility."""

    def test_set_channel_positive_is_forward(self, connected_driver: L298N):
        """Test positive channel value is forward."""
        connected_driver.set_channel(0, 0.5)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.5
        assert state.direction == MotorDirection.FORWARD

    def test_set_channel_negative_is_reverse(self, connected_driver: L298N):
        """Test negative channel value is reverse."""
        connected_driver.set_channel(0, -0.5)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.5
        assert state.direction == MotorDirection.REVERSE

    def test_get_channel_forward(self, connected_driver: L298N):
        """Test get_channel returns positive for forward."""
        connected_driver.forward(0, 0.75)
        value = connected_driver.get_channel(0)
        assert value == 0.75

    def test_get_channel_reverse(self, connected_driver: L298N):
        """Test get_channel returns negative for reverse."""
        connected_driver.reverse(0, 0.6)
        value = connected_driver.get_channel(0)
        assert value == -0.6


# =============================================================================
# Test Validation
# =============================================================================


class TestL298NValidation:
    """Tests for L298N validation."""

    def test_invalid_channel_raises_error(self, connected_driver: L298N):
        """Test invalid channel raises ValueError."""
        with pytest.raises(ValueError, match="Invalid motor channel"):
            connected_driver.set_motor(2, 0.5, MotorDirection.FORWARD)

    def test_negative_channel_raises_error(self, connected_driver: L298N):
        """Test negative channel raises ValueError."""
        with pytest.raises(ValueError, match="Invalid motor channel"):
            connected_driver.set_motor(-1, 0.5, MotorDirection.FORWARD)

    def test_channel_enum_accepted(self, connected_driver: L298N):
        """Test MotorChannel enum is accepted."""
        connected_driver.set_motor(MotorChannel.A, 0.5, MotorDirection.FORWARD)
        state = connected_driver.get_motor_state(MotorChannel.A)
        assert state.speed == 0.5

    def test_require_connected(self, driver: L298N):
        """Test operations require connection."""
        from robo_infra.core.exceptions import HardwareNotFoundError
        with pytest.raises(HardwareNotFoundError, match="not connected"):
            driver.set_motor(0, 0.5, MotorDirection.FORWARD)


# =============================================================================
# Test Driver Registration
# =============================================================================


class TestL298NRegistration:
    """Tests for L298N driver registration."""

    def test_driver_registered(self):
        """Test L298N is registered as 'l298n'."""
        # Force re-import to ensure registration
        import importlib
        import robo_infra.drivers.l298n
        importlib.reload(robo_infra.drivers.l298n)

        driver_cls = get_driver("l298n")
        assert driver_cls.__name__ == "L298N"

    def test_create_from_registry(self):
        """Test creating L298N from registry."""
        # Force re-import to ensure registration
        import importlib
        import robo_infra.drivers.l298n
        importlib.reload(robo_infra.drivers.l298n)

        driver_cls = get_driver("l298n")
        driver = driver_cls()
        assert driver.__class__.__name__ == "L298N"
        assert hasattr(driver, "set_motor")
        assert hasattr(driver, "forward")
        assert hasattr(driver, "reverse")


# =============================================================================
# Test Properties
# =============================================================================


class TestL298NProperties:
    """Tests for L298N properties."""

    def test_motor_a_state_property(self, connected_driver: L298N):
        """Test motor_a_state property."""
        connected_driver.forward(0, 0.5)
        state = connected_driver.motor_a_state

        assert state.speed == 0.5
        assert state.direction.value == MotorDirection.FORWARD.value

    def test_motor_b_state_property(self, connected_driver: L298N):
        """Test motor_b_state property."""
        connected_driver.reverse(1, 0.7)
        state = connected_driver.motor_b_state

        assert state.speed == 0.7
        assert state.direction.value == MotorDirection.REVERSE.value

    def test_get_motor_config(self):
        """Test get_motor_config method."""
        config = L298NConfig(motor_a=MotorConfig(inverted=True))
        driver = L298N(config=config)

        motor_a_config = driver.get_motor_config(0)
        motor_b_config = driver.get_motor_config(1)

        assert motor_a_config.inverted is True
        assert motor_b_config.inverted is False


# =============================================================================
# Test Representation
# =============================================================================


class TestL298NRepr:
    """Tests for L298N string representation."""

    def test_repr_includes_state(self, driver: L298N):
        """Test repr includes driver state."""
        repr_str = repr(driver)
        assert "L298N" in repr_str
        assert "disconnected" in repr_str

    def test_repr_includes_simulation(self, driver: L298N):
        """Test repr includes simulation mode."""
        repr_str = repr(driver)
        assert "simulation=True" in repr_str


# =============================================================================
# Test Edge Cases
# =============================================================================


class TestL298NEdgeCases:
    """Tests for L298N edge cases."""

    def test_forward_with_default_speed(self, connected_driver: L298N):
        """Test forward() with default speed."""
        connected_driver.forward(0)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 1.0

    def test_reverse_with_default_speed(self, connected_driver: L298N):
        """Test reverse() with default speed."""
        connected_driver.reverse(0)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 1.0

    def test_rapid_direction_changes(self, connected_driver: L298N):
        """Test rapid direction changes work correctly."""
        for _ in range(10):
            connected_driver.forward(0, 0.5)
            connected_driver.reverse(0, 0.5)
            connected_driver.brake(0)
            connected_driver.coast(0)

        state = connected_driver.get_motor_state(0)
        assert state.direction.value == MotorDirection.COAST.value

    def test_both_motors_independent(self, connected_driver: L298N):
        """Test both motors operate independently."""
        connected_driver.forward(0, 0.3)
        connected_driver.reverse(1, 0.7)

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.speed == 0.3
        assert state_a.direction.value == MotorDirection.FORWARD.value
        assert state_b.speed == 0.7
        assert state_b.direction.value == MotorDirection.REVERSE.value

    def test_zero_speed_forward_still_sets_direction(self, connected_driver: L298N):
        """Test zero speed with forward still sets direction pins."""
        connected_driver.set_motor(0, 0.0, MotorDirection.FORWARD)
        state = connected_driver.get_motor_state(0)

        # With zero speed and forward direction, the direction pins are set
        # but since speed is 0, motor won't actually move
        assert state.speed == 0.0
        assert state.direction == MotorDirection.FORWARD
