"""Unit tests for Hexapod and Quadruped controllers.

Tests for multi-legged walking robot controllers with gait generation.
"""

from __future__ import annotations

import math

import pytest

from robo_infra.controllers.hexapod import (
    HEXAPOD_RIPPLE_PHASES,
    HEXAPOD_TRIPOD_PHASES,
    HEXAPOD_WAVE_PHASES,
    QUADRUPED_TROT_PHASES,
    QUADRUPED_WALK_PHASES,
    BodyAxis,
    BodyPose,
    GaitParameters,
    GaitType,
    Hexapod,
    HexapodConfig,
    LeggedRobotState,
    LeggedRobotStatus,
    Quadruped,
    QuadrupedConfig,
    calculate_foot_trajectory,
    create_hexapod,
    create_quadruped,
    get_hexapod_phases,
    get_quadruped_phases,
)
from robo_infra.controllers.leg import LegConfig, LegState
from robo_infra.core.exceptions import DisabledError


# =============================================================================
# Enum Tests
# =============================================================================


class TestGaitType:
    """Tests for GaitType enum."""

    def test_hexapod_gaits(self) -> None:
        """Test hexapod gait types."""
        assert GaitType.TRIPOD.value == "tripod"
        assert GaitType.WAVE.value == "wave"
        assert GaitType.RIPPLE.value == "ripple"

    def test_quadruped_gaits(self) -> None:
        """Test quadruped gait types."""
        assert GaitType.TROT.value == "trot"
        assert GaitType.WALK.value == "walk"
        assert GaitType.CREEP.value == "creep"
        assert GaitType.PACE.value == "pace"
        assert GaitType.BOUND.value == "bound"
        assert GaitType.GALLOP.value == "gallop"

    def test_gait_count(self) -> None:
        """Test number of gaits."""
        assert len(GaitType) == 9


class TestLeggedRobotState:
    """Tests for LeggedRobotState enum."""

    def test_all_states(self) -> None:
        """Test all states are defined."""
        assert LeggedRobotState.DISABLED.value == "disabled"
        assert LeggedRobotState.IDLE.value == "idle"
        assert LeggedRobotState.STANDING.value == "standing"
        assert LeggedRobotState.WALKING.value == "walking"
        assert LeggedRobotState.ROTATING.value == "rotating"
        assert LeggedRobotState.POSING.value == "posing"
        assert LeggedRobotState.TRANSITIONING.value == "transitioning"
        assert LeggedRobotState.EMERGENCY.value == "emergency"
        assert LeggedRobotState.ERROR.value == "error"

    def test_state_count(self) -> None:
        """Test number of states."""
        assert len(LeggedRobotState) == 9


class TestBodyAxis:
    """Tests for BodyAxis enum."""

    def test_axes(self) -> None:
        """Test all axes."""
        assert BodyAxis.ROLL.value == "roll"
        assert BodyAxis.PITCH.value == "pitch"
        assert BodyAxis.YAW.value == "yaw"


# =============================================================================
# Data Class Tests
# =============================================================================


class TestBodyPose:
    """Tests for BodyPose dataclass."""

    def test_default_values(self) -> None:
        """Test default constructor."""
        pose = BodyPose()
        assert pose.roll == 0.0
        assert pose.pitch == 0.0
        assert pose.yaw == 0.0
        assert pose.height == 0.0
        assert pose.x_offset == 0.0
        assert pose.y_offset == 0.0

    def test_with_values(self) -> None:
        """Test constructor with values."""
        pose = BodyPose(roll=5.0, pitch=-3.0, yaw=10.0, height=0.12)
        assert pose.roll == 5.0
        assert pose.pitch == -3.0
        assert pose.yaw == 10.0
        assert pose.height == 0.12

    def test_as_dict(self) -> None:
        """Test as_dict method."""
        pose = BodyPose(roll=5.0, pitch=-3.0, yaw=10.0, height=0.12)
        d = pose.as_dict()
        assert d["roll"] == 5.0
        assert d["pitch"] == -3.0
        assert d["yaw"] == 10.0
        assert d["height"] == 0.12


class TestGaitParameters:
    """Tests for GaitParameters dataclass."""

    def test_default_values(self) -> None:
        """Test default constructor."""
        params = GaitParameters()
        assert params.step_height == 0.03
        assert params.step_length == 0.06
        assert params.cycle_time == 1.0
        assert params.duty_factor == 0.5
        assert params.phase_offset == 0.0

    def test_custom_values(self) -> None:
        """Test constructor with values."""
        params = GaitParameters(
            step_height=0.05,
            step_length=0.1,
            cycle_time=0.8,
            duty_factor=0.6,
        )
        assert params.step_height == 0.05
        assert params.step_length == 0.1
        assert params.cycle_time == 0.8
        assert params.duty_factor == 0.6

    def test_validate_valid(self) -> None:
        """Test validation passes for valid params."""
        params = GaitParameters()
        params.validate()  # Should not raise

    def test_validate_invalid_step_height(self) -> None:
        """Test validation fails for invalid step height."""
        params = GaitParameters(step_height=-0.01)
        with pytest.raises(ValueError, match="step_height"):
            params.validate()

    def test_validate_invalid_duty_factor(self) -> None:
        """Test validation fails for invalid duty factor."""
        params = GaitParameters(duty_factor=1.5)
        with pytest.raises(ValueError, match="duty_factor"):
            params.validate()


class TestLeggedRobotStatus:
    """Tests for LeggedRobotStatus dataclass."""

    def test_creation(self) -> None:
        """Test status creation."""
        status = LeggedRobotStatus(
            state=LeggedRobotState.STANDING,
            is_enabled=True,
            is_standing=True,
            body_pose=BodyPose(height=0.1),
            gait_type=GaitType.TRIPOD,
            gait_phase=0.5,
            velocity=(0.1, 0.0, 0.0),
            leg_states={"front_left": LegState.STANCE},
        )
        assert status.state == LeggedRobotState.STANDING
        assert status.is_standing is True
        assert status.gait_phase == 0.5

    def test_as_dict(self) -> None:
        """Test as_dict method."""
        status = LeggedRobotStatus(
            state=LeggedRobotState.WALKING,
            is_enabled=True,
            is_standing=True,
            body_pose=BodyPose(height=0.1),
            gait_type=GaitType.TROT,
            gait_phase=0.25,
            velocity=(0.1, 0.05, 0.1),
            leg_states={"front_left": LegState.SWING},
        )
        d = status.as_dict()
        assert d["state"] == "walking"
        assert d["gait_type"] == "trot"
        assert d["velocity"]["vx"] == 0.1


# =============================================================================
# Gait Phase Tests
# =============================================================================


class TestGaitPhases:
    """Tests for gait phase constants and functions."""

    def test_tripod_phases(self) -> None:
        """Test tripod gait phases."""
        phases = HEXAPOD_TRIPOD_PHASES
        assert len(phases) == 6
        # Tripod: 3 legs at 0.0, 3 at 0.5
        assert phases["front_left"] == 0.0
        assert phases["front_right"] == 0.5
        assert phases["middle_left"] == 0.5
        assert phases["middle_right"] == 0.0

    def test_wave_phases(self) -> None:
        """Test wave gait phases."""
        phases = HEXAPOD_WAVE_PHASES
        assert len(phases) == 6
        # All phases should be different for wave
        unique_phases = set(phases.values())
        assert len(unique_phases) == 6

    def test_ripple_phases(self) -> None:
        """Test ripple gait phases."""
        phases = HEXAPOD_RIPPLE_PHASES
        assert len(phases) == 6
        # Ripple: 2 legs at each of 3 phase points
        assert phases["front_left"] == 0.0
        assert phases["middle_right"] == 0.0

    def test_trot_phases(self) -> None:
        """Test trot gait phases."""
        phases = QUADRUPED_TROT_PHASES
        assert len(phases) == 4
        # Diagonal pairs
        assert phases["front_left"] == phases["rear_right"]
        assert phases["front_right"] == phases["rear_left"]

    def test_walk_phases(self) -> None:
        """Test walk gait phases."""
        phases = QUADRUPED_WALK_PHASES
        assert len(phases) == 4
        # All different
        unique_phases = set(phases.values())
        assert len(unique_phases) == 4

    def test_get_hexapod_phases(self) -> None:
        """Test get_hexapod_phases function."""
        tripod = get_hexapod_phases(GaitType.TRIPOD)
        assert tripod == HEXAPOD_TRIPOD_PHASES

        wave = get_hexapod_phases(GaitType.WAVE)
        assert wave == HEXAPOD_WAVE_PHASES

        ripple = get_hexapod_phases(GaitType.RIPPLE)
        assert ripple == HEXAPOD_RIPPLE_PHASES

    def test_get_hexapod_phases_default(self) -> None:
        """Test get_hexapod_phases returns tripod for unknown gait."""
        phases = get_hexapod_phases(GaitType.TROT)  # Not a hexapod gait
        assert phases == HEXAPOD_TRIPOD_PHASES

    def test_get_quadruped_phases(self) -> None:
        """Test get_quadruped_phases function."""
        trot = get_quadruped_phases(GaitType.TROT)
        assert trot == QUADRUPED_TROT_PHASES

        walk = get_quadruped_phases(GaitType.WALK)
        assert walk == QUADRUPED_WALK_PHASES

    def test_get_quadruped_phases_default(self) -> None:
        """Test get_quadruped_phases returns trot for unknown gait."""
        phases = get_quadruped_phases(GaitType.TRIPOD)  # Not a quadruped gait
        assert phases == QUADRUPED_TROT_PHASES


# =============================================================================
# Foot Trajectory Tests
# =============================================================================


class TestFootTrajectory:
    """Tests for calculate_foot_trajectory function."""

    def test_stance_phase_start(self) -> None:
        """Test trajectory at start of stance phase."""
        x, y, z = calculate_foot_trajectory(
            phase=0.0,
            duty_factor=0.5,
            step_length=0.1,
            step_height=0.05,
            direction=0.0,
        )
        # At start of stance, foot is forward
        assert x > 0
        assert abs(y) < 0.001
        assert z == 0.0  # On ground

    def test_stance_phase_end(self) -> None:
        """Test trajectory at end of stance phase."""
        x, _y, z = calculate_foot_trajectory(
            phase=0.49,
            duty_factor=0.5,
            step_length=0.1,
            step_height=0.05,
            direction=0.0,
        )
        # At end of stance, foot is backward
        assert x < 0
        assert z == 0.0  # Still on ground

    def test_swing_phase_mid(self) -> None:
        """Test trajectory at middle of swing phase."""
        _x, _y, z = calculate_foot_trajectory(
            phase=0.75,  # Middle of swing (0.5 to 1.0)
            duty_factor=0.5,
            step_length=0.1,
            step_height=0.05,
            direction=0.0,
        )
        # At middle of swing, foot is at max height
        assert z > 0  # Lifted
        assert abs(z - 0.05) < 0.01  # Near max height

    def test_direction_forward(self) -> None:
        """Test trajectory for forward direction."""
        x, y, _z = calculate_foot_trajectory(
            phase=0.0,
            duty_factor=0.5,
            step_length=0.1,
            step_height=0.05,
            direction=0.0,  # Forward
        )
        assert x > 0  # Forward offset
        assert abs(y) < 0.001  # No lateral offset

    def test_direction_sideways(self) -> None:
        """Test trajectory for sideways direction."""
        x, y, _z = calculate_foot_trajectory(
            phase=0.0,
            duty_factor=0.5,
            step_length=0.1,
            step_height=0.05,
            direction=math.pi / 2,  # Left
        )
        assert abs(x) < 0.001  # No forward offset
        assert y > 0  # Left offset


# =============================================================================
# HexapodConfig Tests
# =============================================================================


class TestHexapodConfig:
    """Tests for HexapodConfig model."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = HexapodConfig()
        assert config.body_length == 0.2
        assert config.body_width == 0.15
        assert config.default_height == 0.1
        assert config.default_gait == GaitType.TRIPOD

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = HexapodConfig(
            body_length=0.3,
            body_width=0.2,
            default_height=0.15,
            default_gait=GaitType.WAVE,
        )
        assert config.body_length == 0.3
        assert config.default_gait == GaitType.WAVE

    def test_leg_config(self) -> None:
        """Test nested leg configuration."""
        leg_config = LegConfig(coxa_length=0.05)
        config = HexapodConfig(leg_config=leg_config)
        assert config.leg_config.coxa_length == 0.05

    def test_limits(self) -> None:
        """Test limit defaults."""
        config = HexapodConfig()
        assert config.max_speed == 0.3
        assert config.max_rotation_rate == 1.0
        assert config.max_body_roll == 20.0
        assert config.max_body_pitch == 20.0


# =============================================================================
# QuadrupedConfig Tests
# =============================================================================


class TestQuadrupedConfig:
    """Tests for QuadrupedConfig model."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = QuadrupedConfig()
        assert config.body_length == 0.4
        assert config.body_width == 0.2
        assert config.default_height == 0.25
        assert config.default_gait == GaitType.TROT

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = QuadrupedConfig(
            body_length=0.5,
            body_width=0.25,
            default_height=0.3,
            default_gait=GaitType.WALK,
        )
        assert config.body_length == 0.5
        assert config.default_gait == GaitType.WALK

    def test_quadruped_leg_config_defaults(self) -> None:
        """Test quadruped has different default leg lengths."""
        config = QuadrupedConfig()
        # Quadruped has longer legs than hexapod by default
        assert config.leg_config.femur_length == 0.15
        assert config.leg_config.tibia_length == 0.18

    def test_limits(self) -> None:
        """Test limit defaults (quadruped is faster)."""
        config = QuadrupedConfig()
        assert config.max_speed == 1.0
        assert config.max_rotation_rate == 2.0
        assert config.max_body_roll == 30.0


# =============================================================================
# Hexapod Controller Tests
# =============================================================================


class TestHexapodController:
    """Tests for Hexapod controller class."""

    @pytest.fixture
    def hexapod(self) -> Hexapod:
        """Create a hexapod for testing with reachable geometry."""
        # Use larger legs and wider coxa limits for hexapod leg positions
        leg_config = LegConfig(
            coxa_length=0.05,
            femur_length=0.10,
            tibia_length=0.15,  # Total reach = 0.30m
            coxa_min=-180.0,  # Wide limits for hexapod (legs point in all directions)
            coxa_max=180.0,
            femur_min=-135.0,
            femur_max=135.0,
        )
        return create_hexapod(
            "test_hexapod",
            body_length=0.15,  # Smaller body
            body_width=0.10,
            default_height=0.08,
            leg_config=leg_config,
            simulated=True,
        )

    def test_creation(self, hexapod: Hexapod) -> None:
        """Test hexapod creation."""
        assert hexapod.name == "test_hexapod"
        assert hexapod.robot_state == LeggedRobotState.DISABLED
        assert len(hexapod.legs) == 6

    def test_leg_names(self, hexapod: Hexapod) -> None:
        """Test all leg names present."""
        expected = {
            "front_left",
            "front_right",
            "middle_left",
            "middle_right",
            "rear_left",
            "rear_right",
        }
        assert set(hexapod.legs.keys()) == expected

    def test_config(self, hexapod: Hexapod) -> None:
        """Test configuration access."""
        config = hexapod.hexapod_config
        assert isinstance(config, HexapodConfig)

    def test_enable_disable(self, hexapod: Hexapod) -> None:
        """Test enable and disable."""
        hexapod.enable()
        assert hexapod.is_enabled is True
        assert hexapod.robot_state == LeggedRobotState.IDLE

        hexapod.disable()
        assert hexapod.is_enabled is False
        assert hexapod.robot_state == LeggedRobotState.DISABLED

    def test_home(self, hexapod: Hexapod) -> None:
        """Test homing all legs."""
        hexapod.enable()
        hexapod.home()
        assert hexapod.robot_state == LeggedRobotState.IDLE

    def test_home_disabled_error(self, hexapod: Hexapod) -> None:
        """Test homing while disabled raises error."""
        with pytest.raises(DisabledError):
            hexapod.home()

    def test_stand(self, hexapod: Hexapod) -> None:
        """Test standing."""
        hexapod.enable()
        hexapod.stand()

        assert hexapod.robot_state == LeggedRobotState.STANDING
        assert hexapod.body_pose.height == hexapod.hexapod_config.default_height

    def test_stand_custom_height(self, hexapod: Hexapod) -> None:
        """Test standing at custom height."""
        hexapod.enable()
        hexapod.stand(height=0.15)

        assert hexapod.body_pose.height == 0.15

    def test_sit(self, hexapod: Hexapod) -> None:
        """Test sitting."""
        hexapod.enable()
        hexapod.stand()
        hexapod.sit()

        assert hexapod.robot_state == LeggedRobotState.IDLE

    def test_set_gait(self, hexapod: Hexapod) -> None:
        """Test setting gait."""
        hexapod.set_gait(GaitType.WAVE)
        assert hexapod.current_gait == GaitType.WAVE

        hexapod.set_gait("ripple")
        assert hexapod.current_gait == GaitType.RIPPLE

    def test_set_gait_invalid(self, hexapod: Hexapod) -> None:
        """Test setting invalid gait."""
        with pytest.raises(ValueError, match="Invalid hexapod gait"):
            hexapod.set_gait(GaitType.TROT)  # Not valid for hexapod

    def test_walk(self, hexapod: Hexapod) -> None:
        """Test walking."""
        hexapod.enable()
        hexapod.walk(direction=0, speed=0.5)

        assert hexapod.robot_state == LeggedRobotState.WALKING
        assert hexapod.is_walking is True

    def test_walk_disabled_error(self, hexapod: Hexapod) -> None:
        """Test walking while disabled raises error."""
        with pytest.raises(DisabledError):
            hexapod.walk(direction=0, speed=0.5)

    def test_rotate(self, hexapod: Hexapod) -> None:
        """Test rotation."""
        hexapod.enable()
        hexapod.rotate(angular_speed=0.5)

        assert hexapod.robot_state == LeggedRobotState.ROTATING

    def test_stop(self, hexapod: Hexapod) -> None:
        """Test stopping."""
        hexapod.enable()
        hexapod.walk(direction=0, speed=0.5)
        hexapod.stop()

        assert hexapod.robot_state == LeggedRobotState.STANDING
        assert hexapod.is_walking is False

    def test_set_body_pose(self, hexapod: Hexapod) -> None:
        """Test setting body pose."""
        hexapod.enable()
        hexapod.set_body_pose(roll=5.0, pitch=-3.0, height=0.12)

        assert hexapod.body_pose.roll == 5.0
        assert hexapod.body_pose.pitch == -3.0
        assert hexapod.body_pose.height == 0.12

    def test_set_body_pose_clamped(self, hexapod: Hexapod) -> None:
        """Test body pose is clamped to limits."""
        hexapod.enable()
        hexapod.set_body_pose(roll=50.0)  # Exceeds max of 20

        assert hexapod.body_pose.roll == 20.0  # Clamped

    def test_update(self, hexapod: Hexapod) -> None:
        """Test update method."""
        hexapod.enable()
        hexapod.walk(direction=0, speed=0.5)

        # Update should not raise
        hexapod.update(dt=0.1)

    def test_hexapod_status(self, hexapod: Hexapod) -> None:
        """Test status reporting."""
        hexapod.enable()
        hexapod.stand()

        status = hexapod.hexapod_status()
        assert isinstance(status, LeggedRobotStatus)
        assert status.state == LeggedRobotState.STANDING
        assert status.is_enabled is True
        assert status.is_standing is True
        assert len(status.leg_states) == 6


class TestHexapodMissingLegs:
    """Tests for hexapod with missing legs."""

    def test_missing_leg_raises(self) -> None:
        """Test that missing leg raises error."""
        from robo_infra.controllers.leg import create_leg

        legs = {
            name: create_leg(name, simulated=True)
            for name in ["front_left", "front_right"]  # Missing 4 legs
        }

        with pytest.raises(ValueError, match="Missing legs"):
            Hexapod(name="incomplete", legs=legs)


class TestHexapodAsTools:
    """Tests for Hexapod.as_tools() method."""

    @pytest.fixture
    def hexapod(self) -> Hexapod:
        """Create enabled hexapod."""
        hexapod = create_hexapod("test", simulated=True)
        hexapod.enable()
        return hexapod

    def test_tools_returned(self, hexapod: Hexapod) -> None:
        """Test that tools are returned."""
        tools = hexapod.as_tools()
        assert isinstance(tools, list)
        assert len(tools) == 8

    def test_tool_names(self, hexapod: Hexapod) -> None:
        """Test tool function names."""
        tools = hexapod.as_tools()
        names = [t.__name__ for t in tools]
        assert "stand_up" in names
        assert "sit_down" in names
        assert "walk_forward" in names
        assert "walk_direction" in names
        assert "rotate_robot" in names
        assert "stop_motion" in names
        assert "set_gait_pattern" in names
        assert "get_hexapod_status" in names

    def test_stand_up_tool(self, hexapod: Hexapod) -> None:
        """Test stand_up tool."""
        tools = hexapod.as_tools()
        tool = next(t for t in tools if t.__name__ == "stand_up")

        result = tool()
        assert "Standing" in result

    def test_get_status_tool(self, hexapod: Hexapod) -> None:
        """Test get_hexapod_status tool."""
        tools = hexapod.as_tools()
        tool = next(t for t in tools if t.__name__ == "get_hexapod_status")

        result = tool()
        assert isinstance(result, dict)
        assert "state" in result


# =============================================================================
# Quadruped Controller Tests
# =============================================================================


class TestQuadrupedController:
    """Tests for Quadruped controller class."""

    @pytest.fixture
    def quadruped(self) -> Quadruped:
        """Create a quadruped for testing with reachable geometry."""
        # Use larger legs and wider limits for quadruped leg positions
        leg_config = LegConfig(
            coxa_length=0.05,
            femur_length=0.15,
            tibia_length=0.20,  # Total reach = 0.40m
            coxa_min=-180.0,  # Wide limits for quadruped (legs at corners)
            coxa_max=180.0,
            femur_min=-135.0,
            femur_max=135.0,
        )
        return create_quadruped(
            "test_quadruped",
            body_length=0.25,  # Smaller body
            body_width=0.15,
            default_height=0.15,
            leg_config=leg_config,
            simulated=True,
        )

    def test_creation(self, quadruped: Quadruped) -> None:
        """Test quadruped creation."""
        assert quadruped.name == "test_quadruped"
        assert quadruped.robot_state == LeggedRobotState.DISABLED
        assert len(quadruped.legs) == 4

    def test_leg_names(self, quadruped: Quadruped) -> None:
        """Test all leg names present."""
        expected = {"front_left", "front_right", "rear_left", "rear_right"}
        assert set(quadruped.legs.keys()) == expected

    def test_config(self, quadruped: Quadruped) -> None:
        """Test configuration access."""
        config = quadruped.quadruped_config
        assert isinstance(config, QuadrupedConfig)

    def test_enable_disable(self, quadruped: Quadruped) -> None:
        """Test enable and disable."""
        quadruped.enable()
        assert quadruped.is_enabled is True
        assert quadruped.robot_state == LeggedRobotState.IDLE

        quadruped.disable()
        assert quadruped.is_enabled is False
        assert quadruped.robot_state == LeggedRobotState.DISABLED

    def test_stand(self, quadruped: Quadruped) -> None:
        """Test standing."""
        quadruped.enable()
        quadruped.stand()

        assert quadruped.robot_state == LeggedRobotState.STANDING
        assert quadruped.body_pose.height == quadruped.quadruped_config.default_height

    def test_stand_custom_height(self, quadruped: Quadruped) -> None:
        """Test standing at custom height."""
        quadruped.enable()
        quadruped.stand(height=0.3)

        assert quadruped.body_pose.height == 0.3

    def test_sit(self, quadruped: Quadruped) -> None:
        """Test sitting."""
        quadruped.enable()
        quadruped.stand()
        quadruped.sit()

        assert quadruped.robot_state == LeggedRobotState.IDLE

    def test_set_gait(self, quadruped: Quadruped) -> None:
        """Test setting gait."""
        quadruped.set_gait(GaitType.WALK)
        assert quadruped.current_gait == GaitType.WALK

        quadruped.set_gait("pace")
        assert quadruped.current_gait == GaitType.PACE

    def test_set_gait_invalid(self, quadruped: Quadruped) -> None:
        """Test setting invalid gait."""
        with pytest.raises(ValueError, match="Invalid quadruped gait"):
            quadruped.set_gait(GaitType.TRIPOD)  # Not valid for quadruped

    def test_trot(self, quadruped: Quadruped) -> None:
        """Test trotting."""
        quadruped.enable()
        quadruped.trot(vx=0.5, vy=0.0, yaw_rate=0.0)

        assert quadruped.robot_state == LeggedRobotState.WALKING
        assert quadruped.current_gait == GaitType.TROT
        assert quadruped.is_walking is True

    def test_creep(self, quadruped: Quadruped) -> None:
        """Test creeping (slow stable gait)."""
        quadruped.enable()
        quadruped.creep(vx=0.3, vy=0.0)

        assert quadruped.robot_state == LeggedRobotState.WALKING
        assert quadruped.current_gait == GaitType.CREEP

    def test_stop(self, quadruped: Quadruped) -> None:
        """Test stopping."""
        quadruped.enable()
        quadruped.trot(vx=0.5, vy=0.0, yaw_rate=0.0)
        quadruped.stop()

        assert quadruped.robot_state == LeggedRobotState.STANDING
        assert quadruped.is_walking is False

    def test_set_body_pose(self, quadruped: Quadruped) -> None:
        """Test setting body pose."""
        quadruped.enable()
        quadruped.set_body_pose(roll=10.0, pitch=-5.0, height=0.28)

        assert quadruped.body_pose.roll == 10.0
        assert quadruped.body_pose.pitch == -5.0
        assert quadruped.body_pose.height == 0.28

    def test_set_body_pose_clamped(self, quadruped: Quadruped) -> None:
        """Test body pose is clamped to limits."""
        quadruped.enable()
        quadruped.set_body_pose(roll=50.0)  # Exceeds max of 30

        assert quadruped.body_pose.roll == 30.0  # Clamped

    def test_update(self, quadruped: Quadruped) -> None:
        """Test update method."""
        quadruped.enable()
        quadruped.trot(vx=0.5, vy=0.0, yaw_rate=0.0)

        # Update should not raise
        quadruped.update(dt=0.1)

    def test_quadruped_status(self, quadruped: Quadruped) -> None:
        """Test status reporting."""
        quadruped.enable()
        quadruped.stand()

        status = quadruped.quadruped_status()
        assert isinstance(status, LeggedRobotStatus)
        assert status.state == LeggedRobotState.STANDING
        assert status.is_enabled is True
        assert len(status.leg_states) == 4


class TestQuadrupedMissingLegs:
    """Tests for quadruped with missing legs."""

    def test_missing_leg_raises(self) -> None:
        """Test that missing leg raises error."""
        from robo_infra.controllers.leg import create_leg

        legs = {
            name: create_leg(name, simulated=True)
            for name in ["front_left", "front_right"]  # Missing 2 legs
        }

        with pytest.raises(ValueError, match="Missing legs"):
            Quadruped(name="incomplete", legs=legs)


class TestQuadrupedAsTools:
    """Tests for Quadruped.as_tools() method."""

    @pytest.fixture
    def quadruped(self) -> Quadruped:
        """Create enabled quadruped."""
        quadruped = create_quadruped("test", simulated=True)
        quadruped.enable()
        return quadruped

    def test_tools_returned(self, quadruped: Quadruped) -> None:
        """Test that tools are returned."""
        tools = quadruped.as_tools()
        assert isinstance(tools, list)
        assert len(tools) == 7

    def test_tool_names(self, quadruped: Quadruped) -> None:
        """Test tool function names."""
        tools = quadruped.as_tools()
        names = [t.__name__ for t in tools]
        assert "stand_up" in names
        assert "sit_down" in names
        assert "trot_move" in names
        assert "creep_move" in names
        assert "stop_motion" in names
        assert "set_gait_pattern" in names
        assert "get_quadruped_status" in names

    def test_trot_move_tool(self, quadruped: Quadruped) -> None:
        """Test trot_move tool."""
        tools = quadruped.as_tools()
        tool = next(t for t in tools if t.__name__ == "trot_move")

        result = tool(vx=0.5)
        assert "Trotting" in result

    def test_get_status_tool(self, quadruped: Quadruped) -> None:
        """Test get_quadruped_status tool."""
        tools = quadruped.as_tools()
        tool = next(t for t in tools if t.__name__ == "get_quadruped_status")

        result = tool()
        assert isinstance(result, dict)
        assert "state" in result


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreateHexapod:
    """Tests for create_hexapod factory function."""

    def test_default_creation(self) -> None:
        """Test creating hexapod with defaults."""
        hexapod = create_hexapod("test")
        assert hexapod.name == "test"
        assert len(hexapod.legs) == 6

    def test_custom_body_dimensions(self) -> None:
        """Test creating hexapod with custom dimensions."""
        hexapod = create_hexapod(
            "test",
            body_length=0.3,
            body_width=0.2,
            default_height=0.15,
        )
        config = hexapod.hexapod_config
        assert config.body_length == 0.3
        assert config.body_width == 0.2
        assert config.default_height == 0.15

    def test_custom_leg_config(self) -> None:
        """Test creating hexapod with custom leg config."""
        leg_config = LegConfig(coxa_length=0.05)
        hexapod = create_hexapod("test", leg_config=leg_config)
        assert hexapod.hexapod_config.leg_config.coxa_length == 0.05


class TestCreateQuadruped:
    """Tests for create_quadruped factory function."""

    def test_default_creation(self) -> None:
        """Test creating quadruped with defaults."""
        quadruped = create_quadruped("test")
        assert quadruped.name == "test"
        assert len(quadruped.legs) == 4

    def test_custom_body_dimensions(self) -> None:
        """Test creating quadruped with custom dimensions."""
        quadruped = create_quadruped(
            "test",
            body_length=0.5,
            body_width=0.25,
            default_height=0.3,
        )
        config = quadruped.quadruped_config
        assert config.body_length == 0.5
        assert config.body_width == 0.25
        assert config.default_height == 0.3

    def test_custom_leg_config(self) -> None:
        """Test creating quadruped with custom leg config."""
        leg_config = LegConfig(femur_length=0.2, tibia_length=0.22)
        quadruped = create_quadruped("test", leg_config=leg_config)
        assert quadruped.quadruped_config.leg_config.femur_length == 0.2
        assert quadruped.quadruped_config.leg_config.tibia_length == 0.22
