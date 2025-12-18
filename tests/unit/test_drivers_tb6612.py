"""Tests for TB6612FNG dual motor driver."""

from __future__ import annotations

import pytest

from robo_infra.core.driver import DriverState, get_driver
from robo_infra.drivers.tb6612 import (
    TB6612,
    TB6612BrakeMode,
    TB6612Channel,
    TB6612Config,
    TB6612Direction,
    TB6612MotorConfig,
    TB6612MotorState,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def driver() -> TB6612:
    """Create a simulation-mode TB6612 driver."""
    return TB6612()


@pytest.fixture
def connected_driver() -> TB6612:
    """Create a connected simulation-mode TB6612 driver."""
    driver = TB6612()
    driver.connect()
    return driver


# =============================================================================
# Test Enums and Constants
# =============================================================================


class TestTB6612Direction:
    """Tests for TB6612Direction enum."""

    def test_forward_value(self):
        """Test FORWARD direction value."""
        assert TB6612Direction.FORWARD.value == "forward"

    def test_reverse_value(self):
        """Test REVERSE direction value."""
        assert TB6612Direction.REVERSE.value == "reverse"

    def test_brake_value(self):
        """Test BRAKE direction value."""
        assert TB6612Direction.BRAKE.value == "brake"

    def test_coast_value(self):
        """Test COAST direction value."""
        assert TB6612Direction.COAST.value == "coast"


class TestTB6612Channel:
    """Tests for TB6612Channel enum."""

    def test_channel_a_value(self):
        """Test motor channel A value."""
        assert TB6612Channel.A == 0

    def test_channel_b_value(self):
        """Test motor channel B value."""
        assert TB6612Channel.B == 1

    def test_channel_int_conversion(self):
        """Test channel can be converted to int."""
        assert int(TB6612Channel.A) == 0
        assert int(TB6612Channel.B) == 1


class TestTB6612BrakeMode:
    """Tests for TB6612BrakeMode enum."""

    def test_brake_mode_value(self):
        """Test BRAKE mode value."""
        assert TB6612BrakeMode.BRAKE.value == "brake"

    def test_coast_mode_value(self):
        """Test COAST mode value."""
        assert TB6612BrakeMode.COAST.value == "coast"


# =============================================================================
# Test Configuration
# =============================================================================


class TestTB6612MotorConfig:
    """Tests for TB6612MotorConfig dataclass."""

    def test_default_values(self):
        """Test default configuration values."""
        config = TB6612MotorConfig()
        assert config.inverted is False
        assert config.max_speed == 1.0
        assert config.min_speed == 0.0
        assert config.acceleration is None
        assert config.default_brake_mode == TB6612BrakeMode.COAST

    def test_custom_values(self):
        """Test custom configuration values."""
        config = TB6612MotorConfig(
            inverted=True,
            max_speed=0.8,
            min_speed=0.1,
            acceleration=2.0,
            default_brake_mode=TB6612BrakeMode.BRAKE,
        )
        assert config.inverted is True
        assert config.max_speed == 0.8
        assert config.min_speed == 0.1
        assert config.acceleration == 2.0
        assert config.default_brake_mode == TB6612BrakeMode.BRAKE


class TestTB6612Config:
    """Tests for TB6612Config dataclass."""

    def test_default_values(self):
        """Test default configuration values."""
        config = TB6612Config()
        assert config.pwm_frequency == 10000  # Higher than L298N
        assert config.enable_on_connect is True
        assert config.exit_standby_on_connect is True
        assert isinstance(config.motor_a, TB6612MotorConfig)
        assert isinstance(config.motor_b, TB6612MotorConfig)

    def test_custom_values(self):
        """Test custom configuration values."""
        motor_a_config = TB6612MotorConfig(inverted=True)
        motor_b_config = TB6612MotorConfig(max_speed=0.5)
        config = TB6612Config(
            pwm_frequency=20000,
            motor_a=motor_a_config,
            motor_b=motor_b_config,
            enable_on_connect=False,
            exit_standby_on_connect=False,
        )
        assert config.pwm_frequency == 20000
        assert config.motor_a.inverted is True
        assert config.motor_b.max_speed == 0.5
        assert config.enable_on_connect is False
        assert config.exit_standby_on_connect is False


class TestTB6612MotorState:
    """Tests for TB6612MotorState dataclass."""

    def test_default_values(self):
        """Test default state values."""
        state = TB6612MotorState()
        assert state.speed == 0.0
        assert state.direction == TB6612Direction.COAST
        assert state.enabled is False
        assert state.in1_state is False
        assert state.in2_state is False

    def test_custom_values(self):
        """Test custom state values."""
        state = TB6612MotorState(
            speed=0.75,
            direction=TB6612Direction.FORWARD,
            enabled=True,
            in1_state=True,
            in2_state=False,
        )
        assert state.speed == 0.75
        assert state.direction == TB6612Direction.FORWARD
        assert state.enabled is True
        assert state.in1_state is True
        assert state.in2_state is False


# =============================================================================
# Test Initialization
# =============================================================================


class TestTB6612Init:
    """Tests for TB6612 initialization."""

    def test_default_init_simulation_mode(self, driver: TB6612):
        """Test default initialization creates simulation mode driver."""
        assert driver.simulation_mode is True
        assert driver.name == "TB6612"
        assert driver.channels == 2
        assert driver.state == DriverState.DISCONNECTED

    def test_init_with_config(self):
        """Test initialization with custom config."""
        config = TB6612Config(pwm_frequency=5000)
        driver = TB6612(config=config)
        assert driver._pwm_frequency == 5000

    def test_num_channels_constant(self):
        """Test NUM_CHANNELS constant."""
        assert TB6612.NUM_CHANNELS == 2

    def test_default_pwm_frequency_constant(self):
        """Test DEFAULT_PWM_FREQUENCY constant is higher than L298N."""
        assert TB6612.DEFAULT_PWM_FREQUENCY == 10000


# =============================================================================
# Test Lifecycle
# =============================================================================


class TestTB6612Lifecycle:
    """Tests for TB6612 lifecycle methods."""

    def test_connect_simulation(self, driver: TB6612):
        """Test connecting in simulation mode."""
        driver.connect()
        assert driver.state == DriverState.CONNECTED

    def test_connect_exits_standby_by_default(self, driver: TB6612):
        """Test connecting exits standby mode by default."""
        driver.connect()
        assert driver.in_standby is False

    def test_connect_stays_in_standby_when_configured(self):
        """Test connecting stays in standby when configured."""
        config = TB6612Config(exit_standby_on_connect=False)
        driver = TB6612(config=config)
        driver.connect()
        assert driver.in_standby is True

    def test_connect_already_connected_warns(self, connected_driver: TB6612, caplog):
        """Test connecting when already connected logs warning."""
        import logging

        with caplog.at_level(logging.WARNING):
            connected_driver.connect()
        assert "already connected" in caplog.text

    def test_disconnect_simulation(self, connected_driver: TB6612):
        """Test disconnecting in simulation mode."""
        connected_driver.disconnect()
        assert connected_driver.state == DriverState.DISCONNECTED

    def test_disconnect_enters_standby(self, connected_driver: TB6612):
        """Test disconnecting enters standby mode."""
        connected_driver.disconnect()
        assert connected_driver.in_standby is True

    def test_disconnect_already_disconnected_warns(self, driver: TB6612, caplog):
        """Test disconnecting when already disconnected logs warning."""
        import logging

        with caplog.at_level(logging.WARNING):
            driver.disconnect()
        assert "already disconnected" in caplog.text

    def test_context_manager(self):
        """Test using TB6612 as context manager."""
        with TB6612() as driver:
            assert driver.state == DriverState.CONNECTED
        assert driver.state == DriverState.DISCONNECTED

    def test_motors_initialized_to_coast_on_connect(self, connected_driver: TB6612):
        """Test motors are initialized to coast state on connect."""
        state_a = connected_driver.get_motor_state(TB6612Channel.A)
        state_b = connected_driver.get_motor_state(TB6612Channel.B)

        assert state_a.direction.value == TB6612Direction.COAST.value
        assert state_a.speed == 0.0
        assert state_b.direction.value == TB6612Direction.COAST.value
        assert state_b.speed == 0.0


# =============================================================================
# Test Standby Mode
# =============================================================================


class TestTB6612Standby:
    """Tests for TB6612 standby mode."""

    def test_standby_enters_standby_mode(self, connected_driver: TB6612):
        """Test standby() enters standby mode."""
        connected_driver.standby()
        assert connected_driver.in_standby is True

    def test_wake_exits_standby_mode(self, connected_driver: TB6612):
        """Test wake() exits standby mode."""
        connected_driver.standby()
        connected_driver.wake()
        assert connected_driver.in_standby is False

    def test_motor_control_fails_in_standby(self, connected_driver: TB6612):
        """Test motor control raises error in standby mode."""
        connected_driver.standby()

        from robo_infra.core.exceptions import DisabledError

        with pytest.raises(DisabledError, match="standby"):
            connected_driver.set_motor(0, 0.5, TB6612Direction.FORWARD)

    def test_in_standby_property(self, connected_driver: TB6612):
        """Test in_standby property."""
        assert connected_driver.in_standby is False
        connected_driver.standby()
        assert connected_driver.in_standby is True


# =============================================================================
# Test Motor Control
# =============================================================================


class TestTB6612MotorControl:
    """Tests for TB6612 motor control methods."""

    def test_set_motor_forward(self, connected_driver: TB6612):
        """Test setting motor to forward direction."""
        connected_driver.set_motor(0, 0.75, TB6612Direction.FORWARD)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.75
        assert state.direction.value == TB6612Direction.FORWARD.value
        assert state.in1_state is True
        assert state.in2_state is False

    def test_set_motor_reverse(self, connected_driver: TB6612):
        """Test setting motor to reverse direction."""
        connected_driver.set_motor(1, 0.5, TB6612Direction.REVERSE)
        state = connected_driver.get_motor_state(1)

        assert state.speed == 0.5
        assert state.direction.value == TB6612Direction.REVERSE.value
        assert state.in1_state is False
        assert state.in2_state is True

    def test_set_motor_brake(self, connected_driver: TB6612):
        """Test setting motor to brake mode."""
        connected_driver.set_motor(0, 0.0, TB6612Direction.BRAKE)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 1.0  # Full brake
        assert state.direction.value == TB6612Direction.BRAKE.value
        assert state.in1_state is True
        assert state.in2_state is True

    def test_set_motor_coast(self, connected_driver: TB6612):
        """Test setting motor to coast mode."""
        # First set to forward
        connected_driver.set_motor(0, 0.5, TB6612Direction.FORWARD)
        # Then coast
        connected_driver.set_motor(0, 0.0, TB6612Direction.COAST)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.0
        assert state.direction.value == TB6612Direction.COAST.value
        assert state.in1_state is False
        assert state.in2_state is False

    def test_forward_convenience_method(self, connected_driver: TB6612):
        """Test forward() convenience method."""
        connected_driver.forward(0, 0.8)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.8
        assert state.direction.value == TB6612Direction.FORWARD.value

    def test_reverse_convenience_method(self, connected_driver: TB6612):
        """Test reverse() convenience method."""
        connected_driver.reverse(1, 0.6)
        state = connected_driver.get_motor_state(1)

        assert state.speed == 0.6
        assert state.direction.value == TB6612Direction.REVERSE.value

    def test_brake_convenience_method(self, connected_driver: TB6612):
        """Test brake() convenience method."""
        connected_driver.forward(0, 0.5)
        connected_driver.brake(0)
        state = connected_driver.get_motor_state(0)

        assert state.direction.value == TB6612Direction.BRAKE.value

    def test_coast_convenience_method(self, connected_driver: TB6612):
        """Test coast() convenience method."""
        connected_driver.forward(0, 0.5)
        connected_driver.coast(0)
        state = connected_driver.get_motor_state(0)

        assert state.direction.value == TB6612Direction.COAST.value
        assert state.speed == 0.0


# =============================================================================
# Test Speed Limits
# =============================================================================


class TestTB6612SpeedLimits:
    """Tests for TB6612 speed limiting."""

    def test_speed_clamped_to_max(self, connected_driver: TB6612):
        """Test speed is clamped to 1.0 maximum."""
        connected_driver.set_motor(0, 1.5, TB6612Direction.FORWARD)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 1.0

    def test_speed_clamped_to_min(self, connected_driver: TB6612):
        """Test speed is clamped to 0.0 minimum."""
        connected_driver.set_motor(0, -0.5, TB6612Direction.FORWARD)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 0.0

    def test_max_speed_config_applied(self):
        """Test max_speed from config is applied."""
        config = TB6612Config(motor_a=TB6612MotorConfig(max_speed=0.5))
        driver = TB6612(config=config)
        driver.connect()

        driver.set_motor(0, 1.0, TB6612Direction.FORWARD)
        state = driver.get_motor_state(0)
        assert state.speed == 0.5

    def test_min_speed_config_applied(self):
        """Test min_speed from config is applied."""
        config = TB6612Config(motor_a=TB6612MotorConfig(min_speed=0.2))
        driver = TB6612(config=config)
        driver.connect()

        driver.set_motor(0, 0.1, TB6612Direction.FORWARD)
        state = driver.get_motor_state(0)
        assert state.speed == 0.2


# =============================================================================
# Test Direction Inversion
# =============================================================================


class TestTB6612Inversion:
    """Tests for TB6612 motor inversion."""

    def test_inverted_motor_forward_becomes_reverse(self):
        """Test inverted motor swaps forward/reverse directions."""
        config = TB6612Config(motor_a=TB6612MotorConfig(inverted=True))
        driver = TB6612(config=config)
        driver.connect()

        driver.set_motor(0, 0.5, TB6612Direction.FORWARD)
        state = driver.get_motor_state(0)

        # Direction stored is the original request
        assert state.direction.value == TB6612Direction.FORWARD.value
        # But pins are swapped (reverse pattern)
        assert state.in1_state is False
        assert state.in2_state is True

    def test_inverted_motor_brake_unchanged(self):
        """Test inverted motor doesn't affect brake."""
        config = TB6612Config(motor_a=TB6612MotorConfig(inverted=True))
        driver = TB6612(config=config)
        driver.connect()

        driver.set_motor(0, 0.0, TB6612Direction.BRAKE)
        state = driver.get_motor_state(0)

        # Brake is the same regardless of inversion
        assert state.in1_state is True
        assert state.in2_state is True


# =============================================================================
# Test Stop Methods
# =============================================================================


class TestTB6612StopMethods:
    """Tests for TB6612 stopping methods."""

    def test_stop_with_brake_mode(self, connected_driver: TB6612):
        """Test stop() with brake mode."""
        connected_driver.forward(0, 0.5)
        connected_driver.stop(0, TB6612BrakeMode.BRAKE)
        state = connected_driver.get_motor_state(0)

        assert state.direction.value == TB6612Direction.BRAKE.value

    def test_stop_with_coast_mode(self, connected_driver: TB6612):
        """Test stop() with coast mode."""
        connected_driver.forward(0, 0.5)
        connected_driver.stop(0, TB6612BrakeMode.COAST)
        state = connected_driver.get_motor_state(0)

        assert state.direction.value == TB6612Direction.COAST.value

    def test_stop_all(self, connected_driver: TB6612):
        """Test stop_all() stops both motors."""
        connected_driver.forward(0, 0.5)
        connected_driver.reverse(1, 0.7)
        connected_driver.stop_all()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.direction.value == TB6612Direction.COAST.value
        assert state_b.direction.value == TB6612Direction.COAST.value

    def test_emergency_stop(self, connected_driver: TB6612):
        """Test emergency_stop() brakes all motors and enters standby."""
        connected_driver.forward(0, 0.5)
        connected_driver.reverse(1, 0.7)
        connected_driver.emergency_stop()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.direction.value == TB6612Direction.BRAKE.value
        assert state_b.direction.value == TB6612Direction.BRAKE.value
        assert connected_driver.is_enabled is False
        assert connected_driver.in_standby is True


# =============================================================================
# Test Enable/Disable
# =============================================================================


class TestTB6612EnableDisable:
    """Tests for TB6612 enable/disable methods."""

    def test_enable_motor(self, connected_driver: TB6612):
        """Test enabling a motor."""
        connected_driver.disable_motor(0)
        connected_driver.enable_motor(0)
        state = connected_driver.get_motor_state(0)
        assert state.enabled is True

    def test_disable_motor(self, connected_driver: TB6612):
        """Test disabling a motor."""
        connected_driver.disable_motor(0)
        state = connected_driver.get_motor_state(0)
        assert state.enabled is False

    def test_disabled_motor_cannot_be_controlled(self, connected_driver: TB6612):
        """Test disabled motor raises error on control."""
        connected_driver.disable_motor(0)

        from robo_infra.core.exceptions import DisabledError

        with pytest.raises(DisabledError):
            connected_driver.set_motor(0, 0.5, TB6612Direction.FORWARD)

    def test_enable_all(self, connected_driver: TB6612):
        """Test enable_all() enables both motors."""
        connected_driver.disable_all()
        connected_driver.enable_all()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.enabled is True
        assert state_b.enabled is True

    def test_disable_all(self, connected_driver: TB6612):
        """Test disable_all() disables both motors."""
        connected_driver.disable_all()

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.enabled is False
        assert state_b.enabled is False


# =============================================================================
# Test Channel Interface
# =============================================================================


class TestTB6612ChannelInterface:
    """Tests for TB6612 Driver channel interface compatibility."""

    def test_set_channel_positive_is_forward(self, connected_driver: TB6612):
        """Test positive channel value is forward."""
        connected_driver.set_channel(0, 0.5)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.5
        assert state.direction.value == TB6612Direction.FORWARD.value

    def test_set_channel_negative_is_reverse(self, connected_driver: TB6612):
        """Test negative channel value is reverse."""
        connected_driver.set_channel(0, -0.5)
        state = connected_driver.get_motor_state(0)

        assert state.speed == 0.5
        assert state.direction.value == TB6612Direction.REVERSE.value

    def test_get_channel_forward(self, connected_driver: TB6612):
        """Test get_channel returns positive for forward."""
        connected_driver.forward(0, 0.75)
        value = connected_driver.get_channel(0)
        assert value == 0.75

    def test_get_channel_reverse(self, connected_driver: TB6612):
        """Test get_channel returns negative for reverse."""
        connected_driver.reverse(0, 0.6)
        value = connected_driver.get_channel(0)
        assert value == -0.6


# =============================================================================
# Test Validation
# =============================================================================


class TestTB6612Validation:
    """Tests for TB6612 validation."""

    def test_invalid_channel_raises_error(self, connected_driver: TB6612):
        """Test invalid channel raises ValueError."""
        with pytest.raises(ValueError, match="Invalid motor channel"):
            connected_driver.set_motor(2, 0.5, TB6612Direction.FORWARD)

    def test_negative_channel_raises_error(self, connected_driver: TB6612):
        """Test negative channel raises ValueError."""
        with pytest.raises(ValueError, match="Invalid motor channel"):
            connected_driver.set_motor(-1, 0.5, TB6612Direction.FORWARD)

    def test_channel_enum_accepted(self, connected_driver: TB6612):
        """Test TB6612Channel enum is accepted."""
        connected_driver.set_motor(TB6612Channel.A, 0.5, TB6612Direction.FORWARD)
        state = connected_driver.get_motor_state(TB6612Channel.A)
        assert state.speed == 0.5

    def test_require_connected(self, driver: TB6612):
        """Test operations require connection."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        with pytest.raises(HardwareNotFoundError, match="not connected"):
            driver.set_motor(0, 0.5, TB6612Direction.FORWARD)


# =============================================================================
# Test Driver Registration
# =============================================================================


class TestTB6612Registration:
    """Tests for TB6612 driver registration."""

    def test_driver_registered(self):
        """Test TB6612 is registered as 'tb6612'."""
        # Force re-import to ensure registration
        import importlib

        import robo_infra.drivers.tb6612

        importlib.reload(robo_infra.drivers.tb6612)

        driver_cls = get_driver("tb6612")
        assert driver_cls.__name__ == "TB6612"

    def test_create_from_registry(self):
        """Test creating TB6612 from registry."""
        # Force re-import to ensure registration
        import importlib

        import robo_infra.drivers.tb6612

        importlib.reload(robo_infra.drivers.tb6612)

        driver_cls = get_driver("tb6612")
        driver = driver_cls()
        assert driver.__class__.__name__ == "TB6612"
        assert hasattr(driver, "standby")
        assert hasattr(driver, "wake")


# =============================================================================
# Test Properties
# =============================================================================


class TestTB6612Properties:
    """Tests for TB6612 properties."""

    def test_motor_a_state_property(self, connected_driver: TB6612):
        """Test motor_a_state property."""
        connected_driver.forward(0, 0.5)
        state = connected_driver.motor_a_state

        assert state.speed == 0.5
        assert state.direction.value == TB6612Direction.FORWARD.value

    def test_motor_b_state_property(self, connected_driver: TB6612):
        """Test motor_b_state property."""
        connected_driver.reverse(1, 0.7)
        state = connected_driver.motor_b_state

        assert state.speed == 0.7
        assert state.direction.value == TB6612Direction.REVERSE.value

    def test_get_motor_config(self):
        """Test get_motor_config method."""
        config = TB6612Config(motor_a=TB6612MotorConfig(inverted=True))
        driver = TB6612(config=config)

        motor_a_config = driver.get_motor_config(0)
        motor_b_config = driver.get_motor_config(1)

        assert motor_a_config.inverted is True
        assert motor_b_config.inverted is False


# =============================================================================
# Test Representation
# =============================================================================


class TestTB6612Repr:
    """Tests for TB6612 string representation."""

    def test_repr_includes_state(self, driver: TB6612):
        """Test repr includes driver state."""
        repr_str = repr(driver)
        assert "TB6612" in repr_str
        assert "disconnected" in repr_str

    def test_repr_includes_simulation(self, driver: TB6612):
        """Test repr includes simulation mode."""
        repr_str = repr(driver)
        assert "simulation=True" in repr_str

    def test_repr_includes_standby(self, connected_driver: TB6612):
        """Test repr includes standby state."""
        repr_str = repr(connected_driver)
        assert "standby=" in repr_str


# =============================================================================
# Test Edge Cases
# =============================================================================


class TestTB6612EdgeCases:
    """Tests for TB6612 edge cases."""

    def test_forward_with_default_speed(self, connected_driver: TB6612):
        """Test forward() with default speed."""
        connected_driver.forward(0)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 1.0

    def test_reverse_with_default_speed(self, connected_driver: TB6612):
        """Test reverse() with default speed."""
        connected_driver.reverse(0)
        state = connected_driver.get_motor_state(0)
        assert state.speed == 1.0

    def test_rapid_direction_changes(self, connected_driver: TB6612):
        """Test rapid direction changes work correctly."""
        for _ in range(10):
            connected_driver.forward(0, 0.5)
            connected_driver.reverse(0, 0.5)
            connected_driver.brake(0)
            connected_driver.coast(0)

        state = connected_driver.get_motor_state(0)
        assert state.direction.value == TB6612Direction.COAST.value

    def test_both_motors_independent(self, connected_driver: TB6612):
        """Test both motors operate independently."""
        connected_driver.forward(0, 0.3)
        connected_driver.reverse(1, 0.7)

        state_a = connected_driver.get_motor_state(0)
        state_b = connected_driver.get_motor_state(1)

        assert state_a.speed == 0.3
        assert state_a.direction.value == TB6612Direction.FORWARD.value
        assert state_b.speed == 0.7
        assert state_b.direction.value == TB6612Direction.REVERSE.value

    def test_standby_wake_cycle(self, connected_driver: TB6612):
        """Test multiple standby/wake cycles."""
        for _ in range(5):
            connected_driver.standby()
            assert connected_driver.in_standby is True
            connected_driver.wake()
            assert connected_driver.in_standby is False

    def test_motor_control_after_wake(self, connected_driver: TB6612):
        """Test motor control works after waking from standby."""
        connected_driver.forward(0, 0.5)
        connected_driver.standby()
        connected_driver.wake()
        connected_driver.reverse(0, 0.8)

        state = connected_driver.get_motor_state(0)
        assert state.speed == 0.8
        assert state.direction.value == TB6612Direction.REVERSE.value
