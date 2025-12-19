"""Tests for robo_infra.controllers.differential module."""

from __future__ import annotations

import pytest

from robo_infra.actuators.dc_motor import DCMotor
from robo_infra.controllers.differential import (
    DifferentialDrive,
    DifferentialDriveConfig,
    DifferentialDriveState,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def left_motor() -> DCMotor:
    """Create a simulated left motor."""
    return DCMotor(name="left_wheel")


@pytest.fixture
def right_motor() -> DCMotor:
    """Create a simulated right motor."""
    return DCMotor(name="right_wheel")


@pytest.fixture
def differential_drive(left_motor: DCMotor, right_motor: DCMotor) -> DifferentialDrive:
    """Create a DifferentialDrive controller with default config."""
    return DifferentialDrive(
        name="test_rover",
        left=left_motor,
        right=right_motor,
    )


@pytest.fixture
def enabled_drive(differential_drive: DifferentialDrive) -> DifferentialDrive:
    """Create and enable a DifferentialDrive controller."""
    differential_drive.enable()
    return differential_drive


@pytest.fixture
def config_with_inversion() -> DifferentialDriveConfig:
    """Create a config with motor inversion."""
    return DifferentialDriveConfig(
        name="inverted_rover",
        wheel_diameter=0.1,
        track_width=0.3,
        invert_left=True,
        invert_right=False,
    )


@pytest.fixture
def custom_config() -> DifferentialDriveConfig:
    """Create a custom configuration."""
    return DifferentialDriveConfig(
        name="custom_rover",
        wheel_diameter=0.1,
        track_width=0.3,
        max_speed=2.0,
        deadband=0.1,
    )


# =============================================================================
# Initialization Tests
# =============================================================================


class TestDifferentialDriveInit:
    """Tests for DifferentialDrive initialization."""

    def test_differential_init(self, left_motor: DCMotor, right_motor: DCMotor) -> None:
        """Test basic initialization."""
        rover = DifferentialDrive(
            name="test_rover",
            left=left_motor,
            right=right_motor,
        )

        assert rover.name == "test_rover"
        assert rover.left_motor is left_motor
        assert rover.right_motor is right_motor
        assert rover.dd_state == DifferentialDriveState.DISABLED
        assert rover.current_speed == (0.0, 0.0)
        assert not rover.is_enabled

    def test_differential_init_with_config(
        self,
        left_motor: DCMotor,
        right_motor: DCMotor,
        custom_config: DifferentialDriveConfig,
    ) -> None:
        """Test initialization with custom config."""
        rover = DifferentialDrive(
            name="custom_rover",
            left=left_motor,
            right=right_motor,
            config=custom_config,
        )

        assert rover.dd_config.wheel_diameter == 0.1
        assert rover.dd_config.track_width == 0.3
        assert rover.dd_config.max_speed == 2.0
        assert rover.dd_config.deadband == 0.1

    def test_differential_init_inverted_motors(
        self,
        left_motor: DCMotor,
        right_motor: DCMotor,
        config_with_inversion: DifferentialDriveConfig,
    ) -> None:
        """Test initialization with inverted motors."""
        rover = DifferentialDrive(
            name="inverted_rover",
            left=left_motor,
            right=right_motor,
            config=config_with_inversion,
        )

        assert rover.dd_config.invert_left is True
        assert rover.dd_config.invert_right is False

    def test_differential_init_none_motors_raises(self) -> None:
        """Test that None motors raise ValueError."""
        with pytest.raises(ValueError, match="Both left and right motors are required"):
            DifferentialDrive(name="bad", left=None, right=None)  # type: ignore[arg-type]


# =============================================================================
# Basic Movement Tests
# =============================================================================


class TestDifferentialDriveMovement:
    """Tests for DifferentialDrive movement methods."""

    def test_forward_sets_both_motors(self, enabled_drive: DifferentialDrive) -> None:
        """Test forward() sets both motors to positive speed."""
        enabled_drive.forward(0.5)

        left, right = enabled_drive.current_speed
        assert left == 0.5
        assert right == 0.5
        assert enabled_drive.dd_state == DifferentialDriveState.MOVING

    def test_forward_full_speed(self, enabled_drive: DifferentialDrive) -> None:
        """Test forward() at full speed."""
        enabled_drive.forward(1.0)

        left, right = enabled_drive.current_speed
        assert left == 1.0
        assert right == 1.0

    def test_reverse_sets_both_motors_negative(self, enabled_drive: DifferentialDrive) -> None:
        """Test reverse() sets both motors to negative speed."""
        enabled_drive.reverse(0.5)

        left, right = enabled_drive.current_speed
        assert left == -0.5
        assert right == -0.5
        assert enabled_drive.dd_state == DifferentialDriveState.MOVING

    def test_stop_stops_both_motors(self, enabled_drive: DifferentialDrive) -> None:
        """Test stop() sets both motors to zero."""
        enabled_drive.forward(0.5)
        enabled_drive.stop()

        left, right = enabled_drive.current_speed
        assert left == 0.0
        assert right == 0.0
        assert enabled_drive.dd_state == DifferentialDriveState.IDLE

    def test_brake_brakes_both_motors(self, enabled_drive: DifferentialDrive) -> None:
        """Test brake() applies braking."""
        enabled_drive.forward(0.5)
        enabled_drive.brake()

        left, right = enabled_drive.current_speed
        assert left == 0.0
        assert right == 0.0
        assert enabled_drive.dd_state == DifferentialDriveState.BRAKING


# =============================================================================
# Turning Tests
# =============================================================================


class TestDifferentialDriveTurning:
    """Tests for DifferentialDrive turning methods."""

    def test_turn_left(self, enabled_drive: DifferentialDrive) -> None:
        """Test turn_left() reduces left wheel speed."""
        enabled_drive.turn_left(0.5)

        left, right = enabled_drive.current_speed
        assert left < right  # Left is slower for left turn
        assert left == 0.25  # 0.5 * 0.5
        assert right == 0.5
        assert enabled_drive.dd_state == DifferentialDriveState.TURNING

    def test_turn_right(self, enabled_drive: DifferentialDrive) -> None:
        """Test turn_right() reduces right wheel speed."""
        enabled_drive.turn_right(0.5)

        left, right = enabled_drive.current_speed
        assert left > right  # Right is slower for right turn
        assert left == 0.5
        assert right == 0.25  # 0.5 * 0.5
        assert enabled_drive.dd_state == DifferentialDriveState.TURNING

    def test_spin_clockwise(self, enabled_drive: DifferentialDrive) -> None:
        """Test spin() clockwise (left forward, right backward)."""
        enabled_drive.spin(0.5, clockwise=True)

        left, right = enabled_drive.current_speed
        assert left == 0.5
        assert right == -0.5
        assert enabled_drive.dd_state == DifferentialDriveState.SPINNING

    def test_spin_counterclockwise(self, enabled_drive: DifferentialDrive) -> None:
        """Test spin() counter-clockwise (left backward, right forward)."""
        enabled_drive.spin(0.5, clockwise=False)

        left, right = enabled_drive.current_speed
        assert left == -0.5
        assert right == 0.5
        assert enabled_drive.dd_state == DifferentialDriveState.SPINNING


# =============================================================================
# Arc Movement Tests
# =============================================================================


class TestDifferentialDriveArc:
    """Tests for DifferentialDrive arc movement."""

    def test_arc_left(self, enabled_drive: DifferentialDrive) -> None:
        """Test arc() with positive radius (turn left)."""
        enabled_drive.arc(0.5, 1.0)  # 1 meter radius left turn

        left, right = enabled_drive.current_speed
        assert left < right  # Left wheel is inner wheel
        assert enabled_drive.dd_state == DifferentialDriveState.MOVING

    def test_arc_right(self, enabled_drive: DifferentialDrive) -> None:
        """Test arc() with negative radius (turn right)."""
        enabled_drive.arc(0.5, -1.0)  # 1 meter radius right turn

        left, right = enabled_drive.current_speed
        assert left > right  # Right wheel is inner wheel
        assert enabled_drive.dd_state == DifferentialDriveState.MOVING

    def test_arc_tight_radius(self, enabled_drive: DifferentialDrive) -> None:
        """Test arc() with tight radius produces larger speed difference."""
        enabled_drive.arc(0.5, 0.2)  # 0.2 meter radius (very tight)

        left, right = enabled_drive.current_speed
        speed_diff_tight = abs(right - left)

        enabled_drive.arc(0.5, 2.0)  # 2 meter radius (wide)
        left2, right2 = enabled_drive.current_speed
        speed_diff_wide = abs(right2 - left2)

        # Tighter radius should have larger speed difference
        assert speed_diff_tight > speed_diff_wide

    def test_arc_wide_radius(self, enabled_drive: DifferentialDrive) -> None:
        """Test arc() with very wide radius is nearly straight."""
        enabled_drive.arc(0.5, 10.0)  # 10 meter radius (very wide)

        left, right = enabled_drive.current_speed
        # Should be nearly equal for large radius
        assert abs(left - right) < 0.05

    def test_arc_zero_radius_spins(self, enabled_drive: DifferentialDrive) -> None:
        """Test arc() with zero radius is spin in place."""
        enabled_drive.arc(0.5, 0.0)

        left, right = enabled_drive.current_speed
        assert left == 0.5
        assert right == -0.5


# =============================================================================
# Tank Drive Tests
# =============================================================================


class TestDifferentialDriveTank:
    """Tests for DifferentialDrive tank drive."""

    def test_tank_drive(self, enabled_drive: DifferentialDrive) -> None:
        """Test tank() sets wheel speeds independently."""
        enabled_drive.tank(0.3, 0.7)

        left, right = enabled_drive.current_speed
        assert left == 0.3
        assert right == 0.7

    def test_tank_drive_opposite_directions(self, enabled_drive: DifferentialDrive) -> None:
        """Test tank() with opposite wheel directions."""
        enabled_drive.tank(0.5, -0.5)

        left, right = enabled_drive.current_speed
        assert left == 0.5
        assert right == -0.5
        assert enabled_drive.dd_state == DifferentialDriveState.SPINNING

    def test_tank_drive_equal_speeds(self, enabled_drive: DifferentialDrive) -> None:
        """Test tank() with equal speeds sets MOVING state."""
        enabled_drive.tank(0.5, 0.5)

        assert enabled_drive.dd_state == DifferentialDriveState.MOVING

    def test_tank_drive_zero_speeds(self, enabled_drive: DifferentialDrive) -> None:
        """Test tank() with zero speeds sets IDLE state."""
        enabled_drive.tank(0.0, 0.0)

        assert enabled_drive.dd_state == DifferentialDriveState.IDLE


# =============================================================================
# Speed Helpers Tests
# =============================================================================


class TestDifferentialDriveSpeedHelpers:
    """Tests for DifferentialDrive speed helper methods."""

    def test_clamp_speed_within_range(self, differential_drive: DifferentialDrive) -> None:
        """Test _clamp_speed() within range returns unchanged."""
        assert differential_drive._clamp_speed(0.5) == 0.5
        assert differential_drive._clamp_speed(-0.5) == -0.5

    def test_clamp_speed_above_max(self, differential_drive: DifferentialDrive) -> None:
        """Test _clamp_speed() above max is clamped."""
        assert differential_drive._clamp_speed(1.5) == 1.0
        assert differential_drive._clamp_speed(10.0) == 1.0

    def test_clamp_speed_below_min(self, differential_drive: DifferentialDrive) -> None:
        """Test _clamp_speed() below min is clamped."""
        assert differential_drive._clamp_speed(-1.5) == -1.0
        assert differential_drive._clamp_speed(-10.0) == -1.0

    def test_apply_inversion(
        self,
        left_motor: DCMotor,
        right_motor: DCMotor,
        config_with_inversion: DifferentialDriveConfig,
    ) -> None:
        """Test _apply_inversion() inverts correct motors."""
        rover = DifferentialDrive(
            name="test",
            left=left_motor,
            right=right_motor,
            config=config_with_inversion,
        )

        left, right = rover._apply_inversion(0.5, 0.5)
        assert left == -0.5  # Inverted
        assert right == 0.5  # Not inverted

    def test_calculate_arc_speeds_straight(self, differential_drive: DifferentialDrive) -> None:
        """Test _calculate_arc_speeds() with infinite radius is straight."""
        left, right = differential_drive._calculate_arc_speeds(0.5, 10000)

        assert left == 0.5
        assert right == 0.5


# =============================================================================
# Enable/Disable Tests
# =============================================================================


class TestDifferentialDriveEnableDisable:
    """Tests for DifferentialDrive enable/disable."""

    def test_enable_enables_motors(self, differential_drive: DifferentialDrive) -> None:
        """Test enable() enables both motors."""
        differential_drive.enable()

        assert differential_drive.is_enabled
        assert differential_drive.dd_state == DifferentialDriveState.IDLE

    def test_disable_disables_motors(self, enabled_drive: DifferentialDrive) -> None:
        """Test disable() disables both motors."""
        enabled_drive.forward(0.5)
        enabled_drive.disable()

        assert not enabled_drive.is_enabled
        assert enabled_drive.dd_state == DifferentialDriveState.DISABLED
        assert enabled_drive.current_speed == (0.0, 0.0)


# =============================================================================
# Config Tests
# =============================================================================


class TestDifferentialDriveConfig:
    """Tests for DifferentialDriveConfig model."""

    def test_config_defaults(self) -> None:
        """Test config has sensible defaults."""
        config = DifferentialDriveConfig(name="test")

        assert config.name == "test"
        assert config.wheel_diameter == 0.065
        assert config.track_width == 0.15
        assert config.max_speed == 1.0
        assert config.invert_left is False
        assert config.invert_right is False
        assert config.deadband == 0.05

    def test_config_wheel_circumference(self) -> None:
        """Test wheel_circumference property."""
        import math

        config = DifferentialDriveConfig(name="test", wheel_diameter=0.1)

        expected = math.pi * 0.1
        assert abs(config.wheel_circumference - expected) < 0.0001

    def test_config_turning_radius(self) -> None:
        """Test turning_radius property."""
        config = DifferentialDriveConfig(name="test", track_width=0.3)

        assert config.turning_radius == 0.15  # track_width / 2


# =============================================================================
# Repr Tests
# =============================================================================


class TestDifferentialDriveRepr:
    """Tests for DifferentialDrive string representation."""

    def test_repr(self, differential_drive: DifferentialDrive) -> None:
        """Test __repr__ returns useful string."""
        repr_str = repr(differential_drive)

        assert "DifferentialDrive" in repr_str
        assert "test_rover" in repr_str
        assert "disabled" in repr_str
