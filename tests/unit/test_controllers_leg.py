"""Unit tests for Leg controller.

Tests for single robot leg with 3-DOF inverse kinematics.
"""

from __future__ import annotations

import math

import pytest

from robo_infra.controllers.leg import (
    FootPosition,
    JointAngles,
    JointType,
    Leg,
    LegConfig,
    LegDimensions,
    LegPosition,
    LegState,
    LegStatus,
    create_leg,
    forward_kinematics_3dof,
    inverse_kinematics_3dof,
)
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.exceptions import DisabledError, KinematicsError, LimitsExceededError


# =============================================================================
# Enum Tests
# =============================================================================


class TestLegState:
    """Tests for LegState enum."""

    def test_all_states_exist(self) -> None:
        """Test all expected states are defined."""
        assert LegState.DISABLED.value == "disabled"
        assert LegState.IDLE.value == "idle"
        assert LegState.HOMING.value == "homing"
        assert LegState.MOVING.value == "moving"
        assert LegState.STANCE.value == "stance"
        assert LegState.SWING.value == "swing"
        assert LegState.ERROR.value == "error"

    def test_state_count(self) -> None:
        """Test number of states."""
        assert len(LegState) == 7


class TestLegPosition:
    """Tests for LegPosition enum."""

    def test_hexapod_positions(self) -> None:
        """Test hexapod leg positions."""
        assert LegPosition.FRONT_LEFT.value == "front_left"
        assert LegPosition.FRONT_RIGHT.value == "front_right"
        assert LegPosition.MIDDLE_LEFT.value == "middle_left"
        assert LegPosition.MIDDLE_RIGHT.value == "middle_right"
        assert LegPosition.REAR_LEFT.value == "rear_left"
        assert LegPosition.REAR_RIGHT.value == "rear_right"

    def test_quadruped_aliases(self) -> None:
        """Test quadruped leg position aliases."""
        assert LegPosition.LEFT_FRONT.value == "left_front"
        assert LegPosition.RIGHT_FRONT.value == "right_front"
        assert LegPosition.LEFT_REAR.value == "left_rear"
        assert LegPosition.RIGHT_REAR.value == "right_rear"


class TestJointType:
    """Tests for JointType enum."""

    def test_joint_types(self) -> None:
        """Test all joint types."""
        assert JointType.COXA.value == "coxa"
        assert JointType.FEMUR.value == "femur"
        assert JointType.TIBIA.value == "tibia"

    def test_joint_count(self) -> None:
        """Test number of joints."""
        assert len(JointType) == 3


# =============================================================================
# Data Class Tests
# =============================================================================


class TestFootPosition:
    """Tests for FootPosition dataclass."""

    def test_default_values(self) -> None:
        """Test default constructor."""
        pos = FootPosition()
        assert pos.x == 0.0
        assert pos.y == 0.0
        assert pos.z == 0.0

    def test_with_values(self) -> None:
        """Test constructor with values."""
        pos = FootPosition(x=0.1, y=0.05, z=-0.15)
        assert pos.x == 0.1
        assert pos.y == 0.05
        assert pos.z == -0.15

    def test_iteration(self) -> None:
        """Test tuple unpacking."""
        pos = FootPosition(x=1.0, y=2.0, z=3.0)
        x, y, z = pos
        assert x == 1.0
        assert y == 2.0
        assert z == 3.0

    def test_as_tuple(self) -> None:
        """Test as_tuple method."""
        pos = FootPosition(x=1.0, y=2.0, z=3.0)
        assert pos.as_tuple() == (1.0, 2.0, 3.0)

    def test_distance_to(self) -> None:
        """Test distance calculation."""
        pos1 = FootPosition(x=0.0, y=0.0, z=0.0)
        pos2 = FootPosition(x=3.0, y=4.0, z=0.0)
        assert pos1.distance_to(pos2) == 5.0

    def test_distance_to_3d(self) -> None:
        """Test 3D distance calculation."""
        pos1 = FootPosition(x=0.0, y=0.0, z=0.0)
        pos2 = FootPosition(x=1.0, y=2.0, z=2.0)
        assert pos1.distance_to(pos2) == 3.0


class TestJointAngles:
    """Tests for JointAngles dataclass."""

    def test_default_values(self) -> None:
        """Test default constructor."""
        angles = JointAngles()
        assert angles.coxa == 0.0
        assert angles.femur == 0.0
        assert angles.tibia == 0.0

    def test_with_values(self) -> None:
        """Test constructor with values."""
        angles = JointAngles(coxa=10.0, femur=45.0, tibia=-90.0)
        assert angles.coxa == 10.0
        assert angles.femur == 45.0
        assert angles.tibia == -90.0

    def test_iteration(self) -> None:
        """Test tuple unpacking."""
        angles = JointAngles(coxa=1.0, femur=2.0, tibia=3.0)
        coxa, femur, tibia = angles
        assert coxa == 1.0
        assert femur == 2.0
        assert tibia == 3.0

    def test_as_tuple(self) -> None:
        """Test as_tuple method."""
        angles = JointAngles(coxa=1.0, femur=2.0, tibia=3.0)
        assert angles.as_tuple() == (1.0, 2.0, 3.0)

    def test_as_dict(self) -> None:
        """Test as_dict method."""
        angles = JointAngles(coxa=10.0, femur=20.0, tibia=30.0)
        d = angles.as_dict()
        assert d["coxa"] == 10.0
        assert d["femur"] == 20.0
        assert d["tibia"] == 30.0


class TestLegDimensions:
    """Tests for LegDimensions dataclass."""

    def test_creation(self) -> None:
        """Test creating leg dimensions."""
        dims = LegDimensions(coxa_length=0.03, femur_length=0.08, tibia_length=0.12)
        assert dims.coxa_length == 0.03
        assert dims.femur_length == 0.08
        assert dims.tibia_length == 0.12

    def test_total_length(self) -> None:
        """Test total length calculation."""
        dims = LegDimensions(coxa_length=0.03, femur_length=0.08, tibia_length=0.12)
        assert abs(dims.total_length - 0.23) < 1e-10

    def test_min_reach(self) -> None:
        """Test minimum reach calculation."""
        dims = LegDimensions(coxa_length=0.03, femur_length=0.08, tibia_length=0.12)
        assert abs(dims.min_reach - 0.04) < 1e-10  # |0.08 - 0.12|

    def test_max_reach(self) -> None:
        """Test maximum reach calculation."""
        dims = LegDimensions(coxa_length=0.03, femur_length=0.08, tibia_length=0.12)
        assert dims.max_reach == 0.20  # 0.08 + 0.12


class TestLegStatus:
    """Tests for LegStatus dataclass."""

    def test_creation(self) -> None:
        """Test creating leg status."""
        status = LegStatus(
            state=LegState.IDLE,
            is_enabled=True,
            is_homed=True,
            foot_position=FootPosition(x=0.1, y=0.0, z=-0.15),
            joint_angles=JointAngles(coxa=0.0, femur=45.0, tibia=-90.0),
        )
        assert status.state == LegState.IDLE
        assert status.is_enabled is True
        assert status.is_homed is True
        assert status.ground_contact is False
        assert status.load == 0.0

    def test_as_dict(self) -> None:
        """Test as_dict method."""
        status = LegStatus(
            state=LegState.STANCE,
            is_enabled=True,
            is_homed=True,
            foot_position=FootPosition(x=0.1, y=0.0, z=-0.15),
            joint_angles=JointAngles(coxa=0.0, femur=45.0, tibia=-90.0),
            ground_contact=True,
            load=0.5,
        )
        d = status.as_dict()
        assert d["state"] == "stance"
        assert d["is_enabled"] is True
        assert d["ground_contact"] is True
        assert d["load"] == 0.5


# =============================================================================
# Configuration Tests
# =============================================================================


class TestLegConfig:
    """Tests for LegConfig model."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = LegConfig()
        assert config.coxa_length == 0.03
        assert config.femur_length == 0.08
        assert config.tibia_length == 0.12

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = LegConfig(
            coxa_length=0.05,
            femur_length=0.10,
            tibia_length=0.15,
        )
        assert config.coxa_length == 0.05
        assert config.femur_length == 0.10
        assert config.tibia_length == 0.15

    def test_joint_limits(self) -> None:
        """Test joint limit defaults."""
        config = LegConfig()
        assert config.coxa_min == -45.0
        assert config.coxa_max == 45.0
        assert config.femur_min == -90.0
        assert config.femur_max == 90.0
        assert config.tibia_min == -135.0
        assert config.tibia_max == 0.0

    def test_home_positions(self) -> None:
        """Test home position defaults."""
        config = LegConfig()
        assert config.coxa_home == 0.0
        assert config.femur_home == 0.0
        assert config.tibia_home == -90.0

    def test_dimensions_property(self) -> None:
        """Test dimensions property."""
        config = LegConfig(coxa_length=0.03, femur_length=0.08, tibia_length=0.12)
        dims = config.dimensions
        assert isinstance(dims, LegDimensions)
        assert dims.coxa_length == 0.03

    def test_coxa_limits_property(self) -> None:
        """Test coxa_limits property."""
        config = LegConfig(coxa_min=-30.0, coxa_max=30.0, coxa_home=5.0)
        limits = config.coxa_limits
        assert limits.min == -30.0
        assert limits.max == 30.0
        assert limits.default == 5.0

    def test_inversion_flags(self) -> None:
        """Test inversion flag defaults."""
        config = LegConfig()
        assert config.invert_coxa is False
        assert config.invert_femur is False
        assert config.invert_tibia is False


# =============================================================================
# Inverse Kinematics Tests
# =============================================================================


class TestInverseKinematics:
    """Tests for inverse kinematics function."""

    def test_forward_position(self) -> None:
        """Test IK for position directly in front."""
        coxa, femur, tibia = inverse_kinematics_3dof(
            x=0.15,
            y=0.0,
            z=-0.1,
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )
        assert abs(coxa) < 0.1  # Should be ~0 for y=0
        # Angles should be finite and reasonable
        assert -180 <= femur <= 180
        assert -180 <= tibia <= 180

    def test_lateral_position(self) -> None:
        """Test IK for position to the side."""
        coxa, _femur, _tibia = inverse_kinematics_3dof(
            x=0.1,
            y=0.1,
            z=-0.1,
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )
        # Coxa should rotate to point toward (x, y)
        expected_coxa = math.degrees(math.atan2(0.1, 0.1))
        assert abs(coxa - expected_coxa) < 1.0

    def test_position_too_far(self) -> None:
        """Test IK raises error for unreachable position."""
        with pytest.raises(KinematicsError, match="too far"):
            inverse_kinematics_3dof(
                x=0.5,  # Way too far
                y=0.0,
                z=0.0,
                coxa_length=0.03,
                femur_length=0.08,
                tibia_length=0.12,
            )

    def test_position_too_close(self) -> None:
        """Test IK raises error for too close position."""
        with pytest.raises(KinematicsError, match="too close"):
            inverse_kinematics_3dof(
                x=0.03,  # At coxa joint
                y=0.0,
                z=0.0,  # At same height, too close
                coxa_length=0.03,
                femur_length=0.08,
                tibia_length=0.12,
            )

    def test_ik_fk_roundtrip(self) -> None:
        """Test that IK followed by FK returns original position."""
        # Original target position
        target_x, target_y, target_z = 0.12, 0.05, -0.12

        # Calculate joint angles
        coxa, femur, tibia = inverse_kinematics_3dof(
            x=target_x,
            y=target_y,
            z=target_z,
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )

        # Calculate foot position from angles
        x, y, z = forward_kinematics_3dof(
            coxa_angle=coxa,
            femur_angle=femur,
            tibia_angle=tibia,
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )

        # Should match original
        assert abs(x - target_x) < 0.01
        assert abs(y - target_y) < 0.01
        assert abs(z - target_z) < 0.01


class TestForwardKinematics:
    """Tests for forward kinematics function."""

    def test_home_position(self) -> None:
        """Test FK at a typical standing position.

        For a leg to point forward and down, the femur should angle upward
        and the tibia should bend back (negative tibia angle).
        """
        # Use IK to get angles for a position that's forward and down
        # Then verify FK produces that position
        from robo_infra.controllers.leg import inverse_kinematics_3dof

        target_x, target_y, target_z = 0.10, 0.0, -0.10
        coxa, femur, tibia = inverse_kinematics_3dof(
            x=target_x,
            y=target_y,
            z=target_z,
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )

        x, y, z = forward_kinematics_3dof(
            coxa_angle=coxa,
            femur_angle=femur,
            tibia_angle=tibia,
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )

        # FK should reproduce the target position
        assert abs(x - target_x) < 0.01
        assert abs(y - target_y) < 0.01
        assert abs(z - target_z) < 0.01

    def test_fully_extended(self) -> None:
        """Test FK with leg fully extended."""
        x, _y, z = forward_kinematics_3dof(
            coxa_angle=0.0,
            femur_angle=0.0,
            tibia_angle=0.0,  # Straight line with femur
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )
        # The leg extends in the XZ plane when coxa=0
        reach = math.sqrt(x * x + z * z)
        # Leg should have significant reach when extended
        assert reach > 0.1 or abs(x) > 0.1 or abs(z) > 0.1

    def test_coxa_rotation(self) -> None:
        """Test FK with coxa rotation."""
        x, y, _z = forward_kinematics_3dof(
            coxa_angle=90.0,  # Point to the side
            femur_angle=0.0,
            tibia_angle=-90.0,
            coxa_length=0.03,
            femur_length=0.08,
            tibia_length=0.12,
        )
        # Should be pointing to the side (positive y)
        assert y > 0
        assert abs(x) < y  # More to side than forward


# =============================================================================
# Leg Controller Tests
# =============================================================================


class TestLegController:
    """Tests for Leg controller class."""

    @pytest.fixture
    def leg(self) -> Leg:
        """Create a leg for testing."""
        return create_leg("test_leg", simulated=True)

    def test_creation(self, leg: Leg) -> None:
        """Test leg creation."""
        assert leg.name == "test_leg"
        assert leg.leg_state == LegState.DISABLED
        assert leg.is_enabled is False

    def test_leg_config(self, leg: Leg) -> None:
        """Test leg configuration access."""
        config = leg.leg_config
        assert isinstance(config, LegConfig)
        assert config.coxa_length == 0.03

    def test_dimensions(self, leg: Leg) -> None:
        """Test dimensions property."""
        dims = leg.dimensions
        assert isinstance(dims, LegDimensions)
        # Use approximate comparison for floating-point arithmetic
        assert abs(dims.total_length - 0.23) < 1e-10

    def test_enable_disable(self, leg: Leg) -> None:
        """Test enable and disable."""
        leg.enable()
        assert leg.is_enabled is True
        assert leg.leg_state == LegState.IDLE

        leg.disable()
        assert leg.is_enabled is False
        assert leg.leg_state == LegState.DISABLED

    def test_home(self, leg: Leg) -> None:
        """Test homing."""
        leg.enable()
        leg.home()
        assert leg.is_homed is True

        angles = leg.get_joint_angles()
        assert angles.coxa == 0.0
        assert angles.femur == 0.0
        assert angles.tibia == -90.0

    def test_home_disabled_error(self, leg: Leg) -> None:
        """Test homing while disabled raises error."""
        with pytest.raises(DisabledError):
            leg.home()

    def test_set_joint_angles(self, leg: Leg) -> None:
        """Test setting joint angles."""
        leg.enable()
        leg.set_joint_angles(coxa=10.0, femur=20.0, tibia=-45.0)

        angles = leg.get_joint_angles()
        assert abs(angles.coxa - 10.0) < 0.1
        assert abs(angles.femur - 20.0) < 0.1
        assert abs(angles.tibia - (-45.0)) < 0.1

    def test_set_joint_angles_partial(self, leg: Leg) -> None:
        """Test setting only some joint angles."""
        leg.enable()
        leg.set_joint_angles(coxa=15.0)
        leg.set_joint_angles(femur=30.0)

        angles = leg.get_joint_angles()
        assert abs(angles.coxa - 15.0) < 0.1
        assert abs(angles.femur - 30.0) < 0.1

    def test_set_joint_angles_limit_error(self, leg: Leg) -> None:
        """Test joint angle limit enforcement."""
        leg.enable()
        with pytest.raises(LimitsExceededError):
            leg.set_joint_angles(coxa=100.0)  # Exceeds limit

    def test_set_foot_position(self, leg: Leg) -> None:
        """Test setting foot position."""
        leg.enable()
        # Use a position that's reachable within default joint limits
        leg.set_foot_position(0.15, 0.0, -0.10)

        # Verify position changed
        pos = leg.get_foot_position()
        assert abs(pos.x - 0.15) < 0.02
        assert abs(pos.y - 0.0) < 0.02
        assert abs(pos.z - (-0.10)) < 0.02

    def test_set_foot_position_unreachable(self, leg: Leg) -> None:
        """Test error for unreachable position."""
        leg.enable()
        with pytest.raises(KinematicsError):
            leg.set_foot_position(0.5, 0.0, 0.0)  # Too far

    def test_get_foot_position(self, leg: Leg) -> None:
        """Test getting foot position."""
        leg.enable()
        leg.home()

        pos = leg.get_foot_position()
        assert isinstance(pos, FootPosition)
        # At home position, leg should be extended forward and down

    def test_stance_swing(self, leg: Leg) -> None:
        """Test stance and swing mode."""
        leg.enable()

        leg.set_stance()
        assert leg.leg_state == LegState.STANCE
        assert leg.is_grounded is True

        leg.set_swing()
        assert leg.leg_state == LegState.SWING
        assert leg.is_grounded is False

    def test_ground_contact(self, leg: Leg) -> None:
        """Test ground contact setting."""
        leg.enable()

        leg.set_ground_contact(True)
        assert leg.is_grounded is True

        leg.set_ground_contact(False)
        assert leg.is_grounded is False

    def test_load(self, leg: Leg) -> None:
        """Test load setting."""
        leg.enable()

        leg.set_load(0.5)
        status = leg.leg_status()
        assert status.load == 0.5

        # Test clamping
        leg.set_load(2.0)
        status = leg.leg_status()
        assert status.load == 1.0

        leg.set_load(-0.5)
        status = leg.leg_status()
        assert status.load == 0.0

    def test_leg_status(self, leg: Leg) -> None:
        """Test leg status."""
        leg.enable()
        leg.home()

        status = leg.leg_status()
        assert isinstance(status, LegStatus)
        assert status.state == LegState.IDLE
        assert status.is_enabled is True
        assert status.is_homed is True

    def test_stop(self, leg: Leg) -> None:
        """Test stop method."""
        leg.enable()
        leg.stop()
        assert leg.leg_state == LegState.IDLE

    def test_position_attribute(self) -> None:
        """Test leg position attribute."""
        leg = create_leg("test", position="front_left", simulated=True)
        assert leg.position == LegPosition.FRONT_LEFT

    def test_position_attribute_none(self) -> None:
        """Test leg with no position."""
        leg = create_leg("test", simulated=True)
        assert leg.position is None


class TestLegAsTools:
    """Tests for Leg.as_tools() method."""

    @pytest.fixture
    def leg(self) -> Leg:
        """Create a leg for testing."""
        leg = create_leg("test_leg", simulated=True)
        leg.enable()
        return leg

    def test_tools_returned(self, leg: Leg) -> None:
        """Test that tools are returned."""
        tools = leg.as_tools()
        assert isinstance(tools, list)
        assert len(tools) == 4

    def test_tool_names(self, leg: Leg) -> None:
        """Test tool function names."""
        tools = leg.as_tools()
        names = [t.__name__ for t in tools]
        assert "set_foot_position" in names
        assert "set_joint_angles" in names
        assert "get_leg_status" in names
        assert "home_leg" in names

    def test_set_foot_position_tool(self, leg: Leg) -> None:
        """Test set_foot_position tool."""
        tools = leg.as_tools()
        tool = next(t for t in tools if t.__name__ == "set_foot_position")

        # Use a position that's reachable within default joint limits
        result = tool(x=0.15, y=0.0, z=-0.10)
        assert "0.150" in result
        assert "0.000" in result
        assert "-0.100" in result

    def test_get_leg_status_tool(self, leg: Leg) -> None:
        """Test get_leg_status tool."""
        tools = leg.as_tools()
        tool = next(t for t in tools if t.__name__ == "get_leg_status")

        result = tool()
        assert isinstance(result, dict)
        assert "state" in result
        assert "is_enabled" in result

    def test_home_leg_tool(self, leg: Leg) -> None:
        """Test home_leg tool."""
        tools = leg.as_tools()
        tool = next(t for t in tools if t.__name__ == "home_leg")

        result = tool()
        assert "homed" in result.lower()


class TestCreateLeg:
    """Tests for create_leg factory function."""

    def test_default_creation(self) -> None:
        """Test creating leg with defaults."""
        leg = create_leg("test")
        assert leg.name == "test"
        assert leg.leg_config.coxa_length == 0.03

    def test_custom_lengths(self) -> None:
        """Test creating leg with custom lengths."""
        leg = create_leg(
            "test",
            coxa_length=0.05,
            femur_length=0.10,
            tibia_length=0.15,
        )
        assert leg.leg_config.coxa_length == 0.05
        assert leg.leg_config.femur_length == 0.10
        assert leg.leg_config.tibia_length == 0.15

    def test_with_position(self) -> None:
        """Test creating leg with position."""
        leg = create_leg("test", position=LegPosition.FRONT_LEFT)
        assert leg.position == LegPosition.FRONT_LEFT

    def test_with_string_position(self) -> None:
        """Test creating leg with string position."""
        leg = create_leg("test", position="rear_right")
        assert leg.position == LegPosition.REAR_RIGHT

    def test_simulated_flag(self) -> None:
        """Test simulated flag."""
        leg = create_leg("test", simulated=True)
        assert leg.coxa is not None
        assert leg.femur is not None
        assert leg.tibia is not None


class TestLegWithCustomActuators:
    """Tests for Leg with custom actuators."""

    def test_custom_actuators(self) -> None:
        """Test creating leg with custom actuators."""
        config = LegConfig()

        coxa = SimulatedActuator(name="coxa", limits=config.coxa_limits)
        femur = SimulatedActuator(name="femur", limits=config.femur_limits)
        tibia = SimulatedActuator(name="tibia", limits=config.tibia_limits)

        leg = Leg(
            name="custom_leg",
            coxa=coxa,
            femur=femur,
            tibia=tibia,
            config=config,
        )

        assert leg.coxa is coxa
        assert leg.femur is femur
        assert leg.tibia is tibia

    def test_actuator_access(self) -> None:
        """Test accessing actuators through properties."""
        leg = create_leg("test", simulated=True)

        assert leg.coxa.name == "test_coxa"
        assert leg.femur.name == "test_femur"
        assert leg.tibia.name == "test_tibia"


class TestLegMoveToMethod:
    """Tests for Leg.move_to() method."""

    @pytest.fixture
    def leg(self) -> Leg:
        """Create enabled leg."""
        leg = create_leg("test", simulated=True)
        leg.enable()
        return leg

    def test_move_to_all_joints(self, leg: Leg) -> None:
        """Test move_to with all joints."""
        leg.move_to({"coxa": 10.0, "femur": 20.0, "tibia": -60.0})

        angles = leg.get_joint_angles()
        assert abs(angles.coxa - 10.0) < 0.1
        assert abs(angles.femur - 20.0) < 0.1
        assert abs(angles.tibia - (-60.0)) < 0.1

    def test_move_to_partial(self, leg: Leg) -> None:
        """Test move_to with partial targets."""
        leg.move_to({"coxa": 15.0})

        angles = leg.get_joint_angles()
        assert abs(angles.coxa - 15.0) < 0.1
