"""Tests for robo_infra.controllers.joint_group module."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from robo_infra.controllers.joint_group import (
    JointGroup,
    JointGroupConfig,
    JointGroupState,
)
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.exceptions import DisabledError, LimitsExceededError
from robo_infra.core.types import Limits


if TYPE_CHECKING:
    from robo_infra.core.actuator import Actuator


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def simple_joints() -> dict[str, Actuator]:
    """Create a simple set of joints for testing."""
    return {
        "shoulder": SimulatedActuator(
            name="shoulder",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
        "elbow": SimulatedActuator(
            name="elbow",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
        "wrist": SimulatedActuator(
            name="wrist",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
    }


@pytest.fixture
def joint_group(simple_joints: dict[str, Actuator]) -> JointGroup:
    """Create a JointGroup with simple joints."""
    return JointGroup(name="test_arm", joints=simple_joints)


@pytest.fixture
def enabled_joint_group(joint_group: JointGroup) -> JointGroup:
    """Create and enable a JointGroup."""
    joint_group.enable()
    return joint_group


@pytest.fixture
def config_with_positions() -> JointGroupConfig:
    """Create a config with named positions."""
    return JointGroupConfig(
        name="arm_config",
        home_positions={"shoulder": 45, "elbow": 90, "wrist": 135},
        named_positions={
            "ready": {"shoulder": 30, "elbow": 60, "wrist": 90},
            "rest": {"shoulder": 0, "elbow": 0, "wrist": 0},
        },
    )


# =============================================================================
# Initialization Tests
# =============================================================================


class TestJointGroupInit:
    """Tests for JointGroup initialization."""

    def test_joint_group_init_with_servos(self, simple_joints: dict[str, Actuator]) -> None:
        """Test JointGroup initialization with servos."""
        arm = JointGroup(name="test_arm", joints=simple_joints)

        assert arm.name == "test_arm"
        assert len(arm.joints) == 3
        assert "shoulder" in arm.joints
        assert "elbow" in arm.joints
        assert "wrist" in arm.joints
        assert arm.jg_state == JointGroupState.DISABLED
        assert not arm.is_enabled

    def test_joint_group_init_empty_raises(self) -> None:
        """Test that empty joints dict raises ValueError."""
        with pytest.raises(ValueError, match="requires at least one joint"):
            JointGroup(name="empty_arm", joints={})

    def test_joint_group_init_with_config(
        self, simple_joints: dict[str, Actuator], config_with_positions: JointGroupConfig
    ) -> None:
        """Test JointGroup initialization with custom config."""
        arm = JointGroup(
            name="configured_arm",
            joints=simple_joints,
            config=config_with_positions,
        )

        assert arm.name == "configured_arm"
        assert arm.jg_config.home_positions == {"shoulder": 45, "elbow": 90, "wrist": 135}
        assert "ready" in arm.get_named_positions()
        assert "rest" in arm.get_named_positions()

    def test_joint_group_joint_names(self, joint_group: JointGroup) -> None:
        """Test joint_names property returns correct list."""
        assert set(joint_group.joint_names) == {"shoulder", "elbow", "wrist"}


# =============================================================================
# Movement Tests
# =============================================================================


class TestJointGroupMovement:
    """Tests for JointGroup movement methods."""

    def test_move_to_valid_positions(self, enabled_joint_group: JointGroup) -> None:
        """Test moving to valid positions."""
        enabled_joint_group.move_joints(
            {"shoulder": 45, "elbow": 90, "wrist": 135},
            interpolate=False,
        )

        positions = enabled_joint_group.get_positions()
        assert positions["shoulder"] == 45
        assert positions["elbow"] == 90
        assert positions["wrist"] == 135

    def test_move_to_partial_positions(self, enabled_joint_group: JointGroup) -> None:
        """Test moving only some joints."""
        # First set all joints
        enabled_joint_group.move_joints(
            {"shoulder": 45, "elbow": 45, "wrist": 45},
            interpolate=False,
        )

        # Now move only shoulder
        enabled_joint_group.move_joints({"shoulder": 90}, interpolate=False)

        positions = enabled_joint_group.get_positions()
        assert positions["shoulder"] == 90
        assert positions["elbow"] == 45  # Unchanged
        assert positions["wrist"] == 45  # Unchanged

    def test_move_to_invalid_joint_raises(self, enabled_joint_group: JointGroup) -> None:
        """Test that moving invalid joint raises KeyError."""
        with pytest.raises(KeyError, match="Unknown joint"):
            enabled_joint_group.move_joints({"nonexistent": 45})

    def test_move_to_exceeds_limits_clamps(self, enabled_joint_group: JointGroup) -> None:
        """Test that values exceeding limits are clamped."""
        # Default config has clamp_to_limits=True
        enabled_joint_group.move_joints({"shoulder": 200}, interpolate=False)

        positions = enabled_joint_group.get_positions()
        assert positions["shoulder"] == 180  # Clamped to max

        enabled_joint_group.move_joints({"elbow": -50}, interpolate=False)
        positions = enabled_joint_group.get_positions()
        assert positions["elbow"] == 0  # Clamped to min

    def test_move_to_exceeds_limits_raises_when_configured(
        self, simple_joints: dict[str, Actuator]
    ) -> None:
        """Test that values exceeding limits raise when clamp disabled."""
        config = JointGroupConfig(name="strict", clamp_to_limits=False)
        arm = JointGroup(name="strict_arm", joints=simple_joints, config=config)
        arm.enable()

        with pytest.raises(LimitsExceededError):
            arm.move_joints({"shoulder": 200}, interpolate=False)

    @pytest.mark.asyncio
    async def test_move_to_async(self, enabled_joint_group: JointGroup) -> None:
        """Test async movement."""
        await enabled_joint_group.amove_joints(
            {"shoulder": 60, "elbow": 120},
            interpolate=False,
        )

        positions = enabled_joint_group.get_positions()
        assert positions["shoulder"] == 60
        assert positions["elbow"] == 120

    def test_move_when_disabled_raises(self, joint_group: JointGroup) -> None:
        """Test that moving when disabled raises DisabledError."""
        with pytest.raises(DisabledError):
            joint_group.move_joints({"shoulder": 45})


# =============================================================================
# State Tests
# =============================================================================


class TestJointGroupState:
    """Tests for JointGroup state management."""

    def test_get_positions(self, enabled_joint_group: JointGroup) -> None:
        """Test get_positions returns all joint positions."""
        positions = enabled_joint_group.get_positions()

        assert len(positions) == 3
        assert "shoulder" in positions
        assert "elbow" in positions
        assert "wrist" in positions

    def test_get_joint_position(self, enabled_joint_group: JointGroup) -> None:
        """Test getting a single joint position."""
        enabled_joint_group.move_joints({"shoulder": 45}, interpolate=False)

        assert enabled_joint_group.get_joint_position("shoulder") == 45

    def test_get_joint_position_invalid_raises(self, enabled_joint_group: JointGroup) -> None:
        """Test getting invalid joint raises KeyError."""
        with pytest.raises(KeyError, match="Unknown joint"):
            enabled_joint_group.get_joint_position("nonexistent")

    def test_home_moves_to_defaults(
        self, simple_joints: dict[str, Actuator], config_with_positions: JointGroupConfig
    ) -> None:
        """Test home() moves to configured home positions."""
        arm = JointGroup(
            name="arm",
            joints=simple_joints,
            config=config_with_positions,
        )
        arm.enable()

        # Move to some position first
        arm.move_joints({"shoulder": 0, "elbow": 0, "wrist": 0}, interpolate=False)

        # Home
        arm.home()

        positions = arm.get_positions()
        assert positions["shoulder"] == 45
        assert positions["elbow"] == 90
        assert positions["wrist"] == 135

    def test_stop_halts_movement(self, enabled_joint_group: JointGroup) -> None:
        """Test stop() halts movement and sets state."""
        enabled_joint_group.stop()

        assert enabled_joint_group.jg_state == JointGroupState.IDLE
        assert not enabled_joint_group.is_moving

    def test_disable_stops_all_joints(self, enabled_joint_group: JointGroup) -> None:
        """Test disable() disables all joints."""
        enabled_joint_group.disable()

        assert not enabled_joint_group.is_enabled
        assert enabled_joint_group.jg_state == JointGroupState.DISABLED

    def test_enable_enables_controller(self, joint_group: JointGroup) -> None:
        """Test enable() enables the controller."""
        joint_group.enable()

        assert joint_group.is_enabled
        assert joint_group.jg_state == JointGroupState.IDLE


# =============================================================================
# Named Positions Tests
# =============================================================================


class TestJointGroupNamedPositions:
    """Tests for JointGroup named position management."""

    def test_save_position(self, enabled_joint_group: JointGroup) -> None:
        """Test saving current position."""
        enabled_joint_group.move_joints(
            {"shoulder": 45, "elbow": 90, "wrist": 135},
            interpolate=False,
        )

        position = enabled_joint_group.save_position("custom")

        assert position.name == "custom"
        assert position.values["shoulder"] == 45
        assert position.values["elbow"] == 90
        assert position.values["wrist"] == 135
        assert "custom" in enabled_joint_group.get_named_positions()

    def test_go_to_named_position(
        self, simple_joints: dict[str, Actuator], config_with_positions: JointGroupConfig
    ) -> None:
        """Test going to a named position."""
        arm = JointGroup(
            name="arm",
            joints=simple_joints,
            config=config_with_positions,
        )
        arm.enable()

        arm.go_to_named("ready")

        positions = arm.get_positions()
        assert positions["shoulder"] == 30
        assert positions["elbow"] == 60
        assert positions["wrist"] == 90

    def test_go_to_unknown_position_raises(self, enabled_joint_group: JointGroup) -> None:
        """Test going to unknown position raises KeyError."""
        with pytest.raises(KeyError, match="Unknown position"):
            enabled_joint_group.go_to_named("nonexistent")

    @pytest.mark.asyncio
    async def test_ago_to_named_position(
        self, simple_joints: dict[str, Actuator], config_with_positions: JointGroupConfig
    ) -> None:
        """Test async go to named position."""
        arm = JointGroup(
            name="arm",
            joints=simple_joints,
            config=config_with_positions,
        )
        arm.enable()

        await arm.ago_to_named("rest")

        positions = arm.get_positions()
        assert positions["shoulder"] == 0
        assert positions["elbow"] == 0
        assert positions["wrist"] == 0

    def test_get_named_positions(
        self, simple_joints: dict[str, Actuator], config_with_positions: JointGroupConfig
    ) -> None:
        """Test get_named_positions returns all positions."""
        arm = JointGroup(
            name="arm",
            joints=simple_joints,
            config=config_with_positions,
        )

        named = arm.get_named_positions()

        assert "ready" in named
        assert "rest" in named
        assert named["ready"]["shoulder"] == 30
        assert named["rest"]["shoulder"] == 0


# =============================================================================
# Interpolation Tests
# =============================================================================


class TestJointGroupInterpolation:
    """Tests for JointGroup motion interpolation."""

    def test_interpolate_move_generates_steps(self, enabled_joint_group: JointGroup) -> None:
        """Test _interpolate_move generates correct number of steps."""
        start = {"shoulder": 0, "elbow": 0}
        end = {"shoulder": 100, "elbow": 50}
        steps = 10

        positions = list(enabled_joint_group._interpolate_move(start, end, steps))

        assert len(positions) == 10

    def test_interpolate_move_final_position(self, enabled_joint_group: JointGroup) -> None:
        """Test _interpolate_move ends at target position."""
        start = {"shoulder": 0, "elbow": 0}
        end = {"shoulder": 100, "elbow": 50}
        steps = 10

        positions = list(enabled_joint_group._interpolate_move(start, end, steps))

        # Final position should match target
        assert positions[-1]["shoulder"] == 100
        assert positions[-1]["elbow"] == 50

    def test_interpolate_move_linear_progression(
        self, enabled_joint_group: JointGroup
    ) -> None:
        """Test _interpolate_move uses linear interpolation."""
        start = {"shoulder": 0}
        end = {"shoulder": 100}
        steps = 4

        positions = list(enabled_joint_group._interpolate_move(start, end, steps))

        # Should be 25, 50, 75, 100
        assert positions[0]["shoulder"] == 25
        assert positions[1]["shoulder"] == 50
        assert positions[2]["shoulder"] == 75
        assert positions[3]["shoulder"] == 100


# =============================================================================
# Config Tests
# =============================================================================


class TestJointGroupConfig:
    """Tests for JointGroupConfig model."""

    def test_config_defaults(self) -> None:
        """Test config has sensible defaults."""
        config = JointGroupConfig(name="test")

        assert config.name == "test"
        assert config.interpolation_steps == 50
        assert config.default_speed == 0.5
        assert config.move_timeout == 10.0
        assert config.clamp_to_limits is True

    def test_config_custom_values(self) -> None:
        """Test config accepts custom values."""
        config = JointGroupConfig(
            name="custom",
            interpolation_steps=100,
            default_speed=0.8,
            move_timeout=30.0,
            clamp_to_limits=False,
        )

        assert config.interpolation_steps == 100
        assert config.default_speed == 0.8
        assert config.move_timeout == 30.0
        assert config.clamp_to_limits is False


# =============================================================================
# Repr Tests
# =============================================================================


class TestJointGroupRepr:
    """Tests for JointGroup string representation."""

    def test_repr(self, joint_group: JointGroup) -> None:
        """Test __repr__ returns useful string."""
        repr_str = repr(joint_group)

        assert "JointGroup" in repr_str
        assert "test_arm" in repr_str
        assert "disabled" in repr_str
