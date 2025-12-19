"""Tests for robo_infra.controllers.gripper module."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from robo_infra.actuators.servo import Servo
from robo_infra.controllers.gripper import (
    Gripper,
    GripperConfig,
    GripperState,
)
from robo_infra.core.sensor import Sensor
from robo_infra.core.types import Reading


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def gripper_servo() -> Servo:
    """Create a simulated gripper servo."""
    return Servo(name="gripper_servo", angle_range=(0, 90))


@pytest.fixture
def force_sensor() -> MagicMock:
    """Create a mock force sensor."""
    sensor = MagicMock(spec=Sensor)
    sensor.name = "force_sensor"
    sensor.read.return_value = Reading(value=0.0)
    return sensor


@pytest.fixture
def gripper_config() -> GripperConfig:
    """Create default gripper configuration."""
    return GripperConfig(
        name="test_gripper",
        open_position=0.0,
        closed_position=90.0,
        grip_threshold=5.0,
        grip_force_threshold=0.5,
    )


@pytest.fixture
def gripper(gripper_servo: Servo, gripper_config: GripperConfig) -> Gripper:
    """Create a Gripper controller with default config."""
    return Gripper(
        name="test_gripper",
        actuator=gripper_servo,
        config=gripper_config,
    )


@pytest.fixture
def enabled_gripper(gripper: Gripper) -> Gripper:
    """Create and enable a Gripper controller."""
    gripper.enable()
    return gripper


@pytest.fixture
def gripper_with_force_sensor(
    gripper_servo: Servo,
    gripper_config: GripperConfig,
    force_sensor: MagicMock,
) -> Gripper:
    """Create a Gripper controller with force sensor."""
    return Gripper(
        name="test_gripper",
        actuator=gripper_servo,
        config=gripper_config,
        force_sensor=force_sensor,
    )


# =============================================================================
# Initialization Tests
# =============================================================================


class TestGripperInit:
    """Tests for Gripper initialization."""

    def test_gripper_init(self, gripper_servo: Servo) -> None:
        """Test basic initialization."""
        gripper = Gripper(
            name="my_gripper",
            actuator=gripper_servo,
        )

        assert gripper.name == "my_gripper"
        assert gripper.actuator is gripper_servo
        assert gripper.force_sensor is None
        assert gripper.gripper_state == GripperState.DISABLED
        assert not gripper.is_enabled

    def test_gripper_init_with_config(
        self, gripper_servo: Servo, gripper_config: GripperConfig
    ) -> None:
        """Test initialization with custom config."""
        gripper = Gripper(
            name="custom_gripper",
            actuator=gripper_servo,
            config=gripper_config,
        )

        assert gripper.name == "custom_gripper"
        assert gripper.gripper_config.open_position == 0.0
        assert gripper.gripper_config.closed_position == 90.0
        assert gripper.gripper_config.grip_threshold == 5.0

    def test_gripper_init_with_force_sensor(
        self,
        gripper_servo: Servo,
        force_sensor: MagicMock,
    ) -> None:
        """Test initialization with force sensor."""
        gripper = Gripper(
            name="force_gripper",
            actuator=gripper_servo,
            force_sensor=force_sensor,
        )

        assert gripper.force_sensor is force_sensor
        assert gripper.name == "force_gripper"

    def test_gripper_init_without_actuator_raises(self) -> None:
        """Test that initialization without actuator raises ValueError."""
        with pytest.raises(ValueError, match="actuator is required"):
            Gripper(name="no_actuator", actuator=None)  # type: ignore[arg-type]


# =============================================================================
# Open/Close Tests
# =============================================================================


class TestGripperOpenClose:
    """Tests for Gripper open/close operations."""

    def test_open_moves_to_open_position(self, enabled_gripper: Gripper) -> None:
        """Test that open() moves to open position."""
        # Start closed
        enabled_gripper.close()
        assert enabled_gripper.is_closed

        # Open
        enabled_gripper.open()
        assert enabled_gripper.is_open
        assert enabled_gripper.gripper_state == GripperState.OPEN

    def test_close_moves_to_closed_position(self, enabled_gripper: Gripper) -> None:
        """Test that close() moves to closed position."""
        # Start open
        enabled_gripper.open()
        assert enabled_gripper.is_open

        # Close
        enabled_gripper.close()
        assert enabled_gripper.is_closed
        assert enabled_gripper.gripper_state == GripperState.CLOSED

    def test_set_partial_position(self, enabled_gripper: Gripper) -> None:
        """Test setting a partial position."""
        enabled_gripper.set(45.0)
        assert enabled_gripper.position == pytest.approx(45.0, abs=1.0)
        assert not enabled_gripper.is_open
        assert not enabled_gripper.is_closed

    def test_set_out_of_range_raises(self, enabled_gripper: Gripper) -> None:
        """Test that setting position out of range raises ValueError."""
        with pytest.raises(ValueError, match="out of range"):
            enabled_gripper.set(-10.0)

        with pytest.raises(ValueError, match="out of range"):
            enabled_gripper.set(100.0)

    def test_open_when_disabled_raises(self, gripper: Gripper) -> None:
        """Test that open() raises when gripper is disabled."""
        assert not gripper.is_enabled
        with pytest.raises(RuntimeError, match="not enabled"):
            gripper.open()

    def test_close_when_disabled_raises(self, gripper: Gripper) -> None:
        """Test that close() raises when gripper is disabled."""
        assert not gripper.is_enabled
        with pytest.raises(RuntimeError, match="not enabled"):
            gripper.close()


# =============================================================================
# State Tests
# =============================================================================


class TestGripperState:
    """Tests for Gripper state management."""

    def test_is_open_when_at_open_position(self, enabled_gripper: Gripper) -> None:
        """Test is_open returns True at open position."""
        enabled_gripper.open()
        assert enabled_gripper.is_open
        assert not enabled_gripper.is_closed

    def test_is_closed_when_at_closed_position(self, enabled_gripper: Gripper) -> None:
        """Test is_closed returns True at closed position."""
        enabled_gripper.close()
        assert enabled_gripper.is_closed
        assert not enabled_gripper.is_open

    def test_is_gripping_without_force_sensor(self, enabled_gripper: Gripper) -> None:
        """Test is_gripping without force sensor."""
        # Without force sensor, grip() returns False
        result = enabled_gripper.grip()
        assert result is False
        assert not enabled_gripper.is_gripping

    def test_is_gripping_with_force_sensor_below_threshold(
        self,
        gripper_with_force_sensor: Gripper,
        force_sensor: MagicMock,
    ) -> None:
        """Test is_gripping with force sensor below threshold."""
        gripper_with_force_sensor.enable()
        force_sensor.read.return_value = Reading(value=0.1)  # Below threshold (0.5)

        result = gripper_with_force_sensor.grip()
        assert result is False
        assert not gripper_with_force_sensor.is_gripping

    def test_is_gripping_with_force_sensor_above_threshold(
        self,
        gripper_with_force_sensor: Gripper,
        force_sensor: MagicMock,
    ) -> None:
        """Test is_gripping with force sensor above threshold."""
        gripper_with_force_sensor.enable()
        force_sensor.read.return_value = Reading(value=1.0)  # Above threshold (0.5)

        result = gripper_with_force_sensor.grip()
        assert result is True
        assert gripper_with_force_sensor.is_gripping
        assert gripper_with_force_sensor.gripper_state == GripperState.GRIPPING

    def test_release_clears_gripping_state(
        self,
        gripper_with_force_sensor: Gripper,
        force_sensor: MagicMock,
    ) -> None:
        """Test that release() clears gripping state."""
        gripper_with_force_sensor.enable()
        force_sensor.read.return_value = Reading(value=1.0)  # Force detected

        gripper_with_force_sensor.grip()
        assert gripper_with_force_sensor.is_gripping

        gripper_with_force_sensor.release()
        assert not gripper_with_force_sensor.is_gripping
        assert gripper_with_force_sensor.is_open


# =============================================================================
# Enable/Disable Tests
# =============================================================================


class TestGripperEnableDisable:
    """Tests for Gripper enable/disable."""

    def test_enable(self, gripper: Gripper) -> None:
        """Test enabling gripper."""
        assert not gripper.is_enabled
        gripper.enable()
        assert gripper.is_enabled
        assert gripper.gripper_state != GripperState.DISABLED

    def test_disable(self, enabled_gripper: Gripper) -> None:
        """Test disabling gripper."""
        assert enabled_gripper.is_enabled
        enabled_gripper.disable()
        assert not enabled_gripper.is_enabled
        assert enabled_gripper.gripper_state == GripperState.DISABLED

    def test_disable_clears_gripping(
        self,
        gripper_with_force_sensor: Gripper,
        force_sensor: MagicMock,
    ) -> None:
        """Test that disable clears gripping state."""
        gripper_with_force_sensor.enable()
        force_sensor.read.return_value = Reading(value=1.0)
        gripper_with_force_sensor.grip()
        assert gripper_with_force_sensor.is_gripping

        gripper_with_force_sensor.disable()
        assert not gripper_with_force_sensor.is_gripping


# =============================================================================
# Config Tests
# =============================================================================


class TestGripperConfig:
    """Tests for GripperConfig."""

    def test_config_defaults(self) -> None:
        """Test config default values."""
        config = GripperConfig(name="test")
        assert config.open_position == 0.0
        assert config.closed_position == 100.0
        assert config.grip_threshold == 0.1
        assert config.grip_force_threshold == 0.5
        assert config.force_sensor is None

    def test_config_range_property(self) -> None:
        """Test config range calculation."""
        config = GripperConfig(
            name="test",
            open_position=10.0,
            closed_position=90.0,
        )
        assert config.range == 80.0

    def test_config_is_inverted_property(self) -> None:
        """Test config inverted detection."""
        # Normal: open < closed
        normal = GripperConfig(name="normal", open_position=0.0, closed_position=90.0)
        assert not normal.is_inverted

        # Inverted: open > closed
        inverted = GripperConfig(name="inverted", open_position=90.0, closed_position=0.0)
        assert inverted.is_inverted

    def test_config_validation(self) -> None:
        """Test config validation."""
        # grip_threshold must be >= 0
        with pytest.raises(ValueError):
            GripperConfig(name="test", grip_threshold=-1.0)

        # grip_force_threshold must be > 0
        with pytest.raises(ValueError):
            GripperConfig(name="test", grip_force_threshold=0.0)


# =============================================================================
# Additional Method Tests
# =============================================================================


class TestGripperMethods:
    """Tests for additional Gripper methods."""

    def test_stop(self, enabled_gripper: Gripper) -> None:
        """Test stop method."""
        enabled_gripper.open()
        enabled_gripper.stop()
        # State should be updated based on position
        assert enabled_gripper.gripper_state == GripperState.OPEN

    def test_repr(self, gripper: Gripper) -> None:
        """Test string representation."""
        repr_str = repr(gripper)
        assert "Gripper" in repr_str
        assert "test_gripper" in repr_str

    def test_grip_when_disabled_raises(self, gripper: Gripper) -> None:
        """Test that grip() raises when gripper is disabled."""
        with pytest.raises(RuntimeError, match="not enabled"):
            gripper.grip()

    def test_set_when_disabled_raises(self, gripper: Gripper) -> None:
        """Test that set() raises when gripper is disabled."""
        with pytest.raises(RuntimeError, match="not enabled"):
            gripper.set(45.0)


__all__ = [
    "TestGripperInit",
    "TestGripperOpenClose",
    "TestGripperState",
    "TestGripperEnableDisable",
    "TestGripperConfig",
    "TestGripperMethods",
]
