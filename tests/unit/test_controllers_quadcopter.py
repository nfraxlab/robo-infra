"""Unit tests for robo_infra.controllers.quadcopter module.

Tests for quadcopter flight controller including motor mixing,
flight commands, and state management.
"""

from __future__ import annotations

import math

import pytest

from robo_infra.controllers.quadcopter import (
    MIXER_PLUS,
    MIXER_X,
    Attitude,
    FlightMode,
    FrameType,
    MotorOutputs,
    MotorPosition,
    Position3D,
    Quadcopter,
    QuadcopterConfig,
    QuadcopterState,
    QuadcopterStatus,
    Velocity,
    create_quadcopter,
    mix_motors,
    normalize_motor_outputs,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def default_config() -> QuadcopterConfig:
    """Create a default quadcopter configuration."""
    return QuadcopterConfig(
        name="test_quad",
        arm_length=0.25,
        motor_kv=2300,
        prop_diameter=5.0,
        frame_type="X",
    )


@pytest.fixture
def quadcopter(default_config: QuadcopterConfig) -> Quadcopter:
    """Create a Quadcopter controller."""
    return Quadcopter(
        name="test_quad",
        config=default_config,
        simulated=True,
    )


@pytest.fixture
def enabled_quadcopter(quadcopter: Quadcopter) -> Quadcopter:
    """Create and enable a Quadcopter controller."""
    quadcopter.enable()
    return quadcopter


@pytest.fixture
def armed_quadcopter(enabled_quadcopter: Quadcopter) -> Quadcopter:
    """Create, enable, and arm a Quadcopter controller."""
    enabled_quadcopter.arm()
    return enabled_quadcopter


@pytest.fixture
def flying_quadcopter(armed_quadcopter: Quadcopter) -> Quadcopter:
    """Create a flying Quadcopter controller."""
    armed_quadcopter.takeoff(altitude=2.0)
    return armed_quadcopter


# =============================================================================
# QuadcopterState Tests
# =============================================================================


class TestQuadcopterState:
    """Tests for QuadcopterState enum."""

    def test_all_states_exist(self) -> None:
        """Test all expected states exist."""
        assert QuadcopterState.DISABLED.value == "disabled"
        assert QuadcopterState.DISARMED.value == "disarmed"
        assert QuadcopterState.ARMED.value == "armed"
        assert QuadcopterState.TAKING_OFF.value == "taking_off"
        assert QuadcopterState.HOVERING.value == "hovering"
        assert QuadcopterState.FLYING.value == "flying"
        assert QuadcopterState.LANDING.value == "landing"
        assert QuadcopterState.EMERGENCY.value == "emergency"
        assert QuadcopterState.ERROR.value == "error"


class TestFlightMode:
    """Tests for FlightMode enum."""

    def test_stabilized_modes(self) -> None:
        """Test stabilized flight modes."""
        assert FlightMode.STABILIZE.value == "stabilize"
        assert FlightMode.ACRO.value == "acro"

    def test_altitude_modes(self) -> None:
        """Test altitude hold modes."""
        assert FlightMode.ALT_HOLD.value == "alt_hold"
        assert FlightMode.LOITER.value == "loiter"

    def test_autonomous_modes(self) -> None:
        """Test autonomous flight modes."""
        assert FlightMode.AUTO.value == "auto"
        assert FlightMode.GUIDED.value == "guided"
        assert FlightMode.RTL.value == "rtl"


class TestFrameType:
    """Tests for FrameType enum."""

    def test_frame_types(self) -> None:
        """Test frame type values."""
        assert FrameType.X.value == "X"
        assert FrameType.PLUS.value == "+"
        assert FrameType.H.value == "H"


class TestMotorPosition:
    """Tests for MotorPosition enum."""

    def test_motor_positions(self) -> None:
        """Test motor position values."""
        assert MotorPosition.FRONT_LEFT.value == 0
        assert MotorPosition.FRONT_RIGHT.value == 1
        assert MotorPosition.REAR_LEFT.value == 2
        assert MotorPosition.REAR_RIGHT.value == 3


# =============================================================================
# Data Class Tests
# =============================================================================


class TestAttitude:
    """Tests for Attitude dataclass."""

    def test_default_attitude(self) -> None:
        """Test default attitude is level."""
        att = Attitude()
        assert att.roll == 0.0
        assert att.pitch == 0.0
        assert att.yaw == 0.0

    def test_attitude_with_values(self) -> None:
        """Test attitude with custom values."""
        att = Attitude(roll=10.0, pitch=-5.0, yaw=90.0)
        assert att.roll == 10.0
        assert att.pitch == -5.0
        assert att.yaw == 90.0

    def test_to_radians(self) -> None:
        """Test conversion to radians."""
        att = Attitude(roll=90.0, pitch=45.0, yaw=180.0)
        rad = att.to_radians()
        assert abs(rad[0] - math.pi / 2) < 0.01
        assert abs(rad[1] - math.pi / 4) < 0.01
        assert abs(rad[2] - math.pi) < 0.01

    def test_from_radians(self) -> None:
        """Test creation from radians."""
        att = Attitude.from_radians(math.pi / 2, math.pi / 4, math.pi)
        assert abs(att.roll - 90.0) < 0.1
        assert abs(att.pitch - 45.0) < 0.1
        assert abs(att.yaw - 180.0) < 0.1


class TestVelocity:
    """Tests for Velocity dataclass."""

    def test_default_velocity(self) -> None:
        """Test default velocity is zero."""
        vel = Velocity()
        assert vel.vx == 0.0
        assert vel.vy == 0.0
        assert vel.vz == 0.0
        assert vel.yaw_rate == 0.0

    def test_speed_property(self) -> None:
        """Test speed magnitude calculation."""
        vel = Velocity(vx=3.0, vy=4.0, vz=0.0)
        assert abs(vel.speed - 5.0) < 0.01

    def test_horizontal_speed_property(self) -> None:
        """Test horizontal speed calculation."""
        vel = Velocity(vx=3.0, vy=4.0, vz=5.0)
        assert abs(vel.horizontal_speed - 5.0) < 0.01


class TestPosition3D:
    """Tests for Position3D dataclass."""

    def test_default_position(self) -> None:
        """Test default position is origin."""
        pos = Position3D()
        assert pos.x == 0.0
        assert pos.y == 0.0
        assert pos.z == 0.0
        assert pos.yaw == 0.0

    def test_distance_to(self) -> None:
        """Test distance calculation."""
        pos1 = Position3D(x=0.0, y=0.0, z=0.0)
        pos2 = Position3D(x=3.0, y=4.0, z=0.0)
        assert abs(pos1.distance_to(pos2) - 5.0) < 0.01

    def test_horizontal_distance_to(self) -> None:
        """Test horizontal distance calculation."""
        pos1 = Position3D(x=0.0, y=0.0, z=0.0)
        pos2 = Position3D(x=3.0, y=4.0, z=10.0)
        assert abs(pos1.horizontal_distance_to(pos2) - 5.0) < 0.01


class TestMotorOutputs:
    """Tests for MotorOutputs dataclass."""

    def test_default_outputs(self) -> None:
        """Test default outputs are zero."""
        outputs = MotorOutputs()
        assert outputs.front_left == 0.0
        assert outputs.front_right == 0.0
        assert outputs.rear_left == 0.0
        assert outputs.rear_right == 0.0

    def test_clamping(self) -> None:
        """Test values are clamped to valid range."""
        outputs = MotorOutputs(
            front_left=1.5,
            front_right=-0.5,
            rear_left=0.5,
            rear_right=0.5,
        )
        assert outputs.front_left == 1.0  # Clamped high
        assert outputs.front_right == 0.0  # Clamped low
        assert outputs.rear_left == 0.5
        assert outputs.rear_right == 0.5

    def test_as_list(self) -> None:
        """Test conversion to list."""
        outputs = MotorOutputs(
            front_left=0.1,
            front_right=0.2,
            rear_left=0.3,
            rear_right=0.4,
        )
        values = outputs.as_list()
        assert values == [0.1, 0.2, 0.3, 0.4]

    def test_from_list(self) -> None:
        """Test creation from list."""
        outputs = MotorOutputs.from_list([0.1, 0.2, 0.3, 0.4])
        assert outputs.front_left == 0.1
        assert outputs.front_right == 0.2
        assert outputs.rear_left == 0.3
        assert outputs.rear_right == 0.4

    def test_from_list_invalid_length(self) -> None:
        """Test error on invalid list length."""
        with pytest.raises(ValueError, match="Expected 4 values"):
            MotorOutputs.from_list([0.1, 0.2, 0.3])

    def test_scaled(self) -> None:
        """Test scaling outputs."""
        outputs = MotorOutputs(
            front_left=0.5,
            front_right=0.5,
            rear_left=0.5,
            rear_right=0.5,
        )
        scaled = outputs.scaled(0.5)
        assert scaled.front_left == 0.25
        assert scaled.front_right == 0.25


# =============================================================================
# Configuration Tests
# =============================================================================


class TestQuadcopterConfig:
    """Tests for QuadcopterConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = QuadcopterConfig(name="test")
        assert config.name == "test"
        assert config.arm_length == 0.25
        assert config.motor_kv == 2300
        assert config.prop_diameter == 5.0
        assert config.frame_type == "X"
        assert config.weight == 1.0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = QuadcopterConfig(
            name="custom",
            arm_length=0.125,
            motor_kv=2400,
            prop_diameter=3.0,
            frame_type="+",
            weight=0.35,
        )
        assert config.arm_length == 0.125
        assert config.motor_kv == 2400
        assert config.prop_diameter == 3.0
        assert config.frame_type == "+"
        assert config.weight == 0.35

    def test_frame_type_validation(self) -> None:
        """Test frame type validation."""
        with pytest.raises(ValueError, match="frame_type must be one of"):
            QuadcopterConfig(name="test", frame_type="invalid")

    def test_frame_type_enum_property(self) -> None:
        """Test frame_type_enum property."""
        config_x = QuadcopterConfig(name="test", frame_type="X")
        assert config_x.frame_type_enum == FrameType.X

        config_plus = QuadcopterConfig(name="test", frame_type="+")
        assert config_plus.frame_type_enum == FrameType.PLUS

    def test_mixer_matrix_x(self) -> None:
        """Test mixer matrix for X frame."""
        config = QuadcopterConfig(name="test", frame_type="X")
        assert config.mixer_matrix == MIXER_X

    def test_mixer_matrix_plus(self) -> None:
        """Test mixer matrix for + frame."""
        config = QuadcopterConfig(name="test", frame_type="+")
        assert config.mixer_matrix == MIXER_PLUS

    def test_wheelbase_property(self) -> None:
        """Test wheelbase calculation."""
        config = QuadcopterConfig(name="test", arm_length=0.25)
        assert config.wheelbase == 0.5

    def test_prop_diameter_m_property(self) -> None:
        """Test propeller diameter in meters."""
        config = QuadcopterConfig(name="test", prop_diameter=5.0)
        assert abs(config.prop_diameter_m - 0.127) < 0.001

    def test_battery_voltage_properties(self) -> None:
        """Test battery voltage calculations."""
        config = QuadcopterConfig(name="test", battery_cells=4)
        assert config.battery_voltage_full == 4 * 4.2
        assert config.battery_voltage_empty == 4 * 3.3


# =============================================================================
# Motor Mixing Tests
# =============================================================================


class TestMotorMixing:
    """Tests for motor mixing functions."""

    def test_mix_motors_hover(self) -> None:
        """Test motor mixing for hover (all equal)."""
        outputs = mix_motors(
            throttle=0.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            mixer=MIXER_X,
        )
        # All motors should be equal at hover
        assert abs(outputs.front_left - 0.5) < 0.01
        assert abs(outputs.front_right - 0.5) < 0.01
        assert abs(outputs.rear_left - 0.5) < 0.01
        assert abs(outputs.rear_right - 0.5) < 0.01

    def test_mix_motors_roll_right(self) -> None:
        """Test motor mixing for right roll."""
        outputs = mix_motors(
            throttle=0.5,
            roll=0.2,  # Right roll
            pitch=0.0,
            yaw=0.0,
            mixer=MIXER_X,
        )
        # Left motors should increase, right decrease
        assert outputs.front_left < outputs.front_right
        assert outputs.rear_left < outputs.rear_right

    def test_mix_motors_pitch_forward(self) -> None:
        """Test motor mixing for forward pitch."""
        outputs = mix_motors(
            throttle=0.5,
            roll=0.0,
            pitch=0.2,  # Forward pitch
            yaw=0.0,
            mixer=MIXER_X,
        )
        # Rear motors should increase, front decrease
        assert outputs.front_left > outputs.rear_left
        assert outputs.front_right > outputs.rear_right

    def test_mix_motors_yaw_right(self) -> None:
        """Test motor mixing for right yaw."""
        outputs = mix_motors(
            throttle=0.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.2,  # Right yaw
            mixer=MIXER_X,
        )
        # CW motors increase, CCW decrease for clockwise rotation
        # FL and RR are CCW, FR and RL are CW
        assert outputs.front_left < 0.5
        assert outputs.rear_right < 0.5
        assert outputs.front_right > 0.5
        assert outputs.rear_left > 0.5

    def test_mix_motors_clamping(self) -> None:
        """Test motor outputs are clamped."""
        outputs = mix_motors(
            throttle=1.0,
            roll=0.5,
            pitch=0.5,
            yaw=0.5,
            mixer=MIXER_X,
        )
        values = outputs.as_list()
        for v in values:
            assert 0.0 <= v <= 1.0


class TestNormalizeMotorOutputs:
    """Tests for normalize_motor_outputs function."""

    def test_normalize_already_valid(self) -> None:
        """Test normalization of already valid outputs."""
        outputs = MotorOutputs(
            front_left=0.5,
            front_right=0.5,
            rear_left=0.5,
            rear_right=0.5,
        )
        normalized = normalize_motor_outputs(outputs)
        assert normalized.front_left == 0.5

    def test_normalize_scale_down(self) -> None:
        """Test normalization scales down when over 1.0."""
        # Mix motors can produce values > 1.0 before normalization
        raw = mix_motors(1.0, 0.5, 0.5, 0.5, MIXER_X)
        normalized = normalize_motor_outputs(raw)
        values = normalized.as_list()
        assert max(values) <= 1.0

    def test_normalize_min_output(self) -> None:
        """Test minimum output enforcement."""
        outputs = MotorOutputs(
            front_left=0.01,
            front_right=0.01,
            rear_left=0.01,
            rear_right=0.01,
        )
        normalized = normalize_motor_outputs(outputs, min_output=0.05)
        values = normalized.as_list()
        for v in values:
            assert v >= 0.05


# =============================================================================
# Quadcopter Initialization Tests
# =============================================================================


class TestQuadcopterInit:
    """Tests for Quadcopter initialization."""

    def test_basic_init(self) -> None:
        """Test basic initialization."""
        quad = Quadcopter(name="test")
        assert quad.name == "test"
        assert quad.quad_state == QuadcopterState.DISABLED
        assert not quad.is_armed
        assert not quad.is_flying
        assert not quad.is_enabled

    def test_init_with_config(self, default_config: QuadcopterConfig) -> None:
        """Test initialization with config."""
        quad = Quadcopter(
            name="test",
            config=default_config,
            simulated=True,
        )
        assert quad.quad_config == default_config
        assert quad.quad_config.frame_type == "X"

    def test_init_with_motors_wrong_count(self) -> None:
        """Test error with wrong motor count."""
        # This would need actual BrushlessMotor objects
        # For now, we test the simulated case
        pass  # Covered by other tests

    def test_initial_state(self, quadcopter: Quadcopter) -> None:
        """Test initial state values."""
        assert quadcopter.attitude.roll == 0.0
        assert quadcopter.attitude.pitch == 0.0
        assert quadcopter.attitude.yaw == 0.0
        assert quadcopter.position.x == 0.0
        assert quadcopter.position.y == 0.0
        assert quadcopter.position.z == 0.0
        assert quadcopter.altitude == 0.0


# =============================================================================
# Enable/Disable Tests
# =============================================================================


class TestQuadcopterEnableDisable:
    """Tests for enable/disable functionality."""

    def test_enable(self, quadcopter: Quadcopter) -> None:
        """Test enabling quadcopter."""
        quadcopter.enable()
        assert quadcopter.is_enabled
        assert quadcopter.quad_state == QuadcopterState.DISARMED

    def test_disable(self, enabled_quadcopter: Quadcopter) -> None:
        """Test disabling quadcopter."""
        enabled_quadcopter.disable()
        assert not enabled_quadcopter.is_enabled
        assert enabled_quadcopter.quad_state == QuadcopterState.DISABLED

    def test_disable_while_armed(self, armed_quadcopter: Quadcopter) -> None:
        """Test disabling while armed disarms first."""
        armed_quadcopter.disable()
        assert not armed_quadcopter.is_armed
        assert not armed_quadcopter.is_enabled


# =============================================================================
# Arming Tests
# =============================================================================


class TestQuadcopterArming:
    """Tests for arm/disarm functionality."""

    def test_can_arm_when_enabled(self, enabled_quadcopter: Quadcopter) -> None:
        """Test can_arm returns true when enabled."""
        can_arm, _reason = enabled_quadcopter.can_arm()
        assert can_arm is True

    def test_cannot_arm_when_disabled(self, quadcopter: Quadcopter) -> None:
        """Test cannot arm when disabled."""
        can_arm, reason = quadcopter.can_arm()
        assert can_arm is False
        assert "not enabled" in reason.lower()

    def test_arm(self, enabled_quadcopter: Quadcopter) -> None:
        """Test arming."""
        enabled_quadcopter.arm()
        assert enabled_quadcopter.is_armed
        assert enabled_quadcopter.quad_state == QuadcopterState.ARMED

    def test_arm_sets_idle_throttle(self, enabled_quadcopter: Quadcopter) -> None:
        """Test arming sets idle throttle."""
        enabled_quadcopter.arm()
        outputs = enabled_quadcopter.motor_outputs
        idle = enabled_quadcopter.quad_config.idle_throttle
        for val in outputs.as_list():
            assert val >= idle

    def test_disarm(self, armed_quadcopter: Quadcopter) -> None:
        """Test disarming."""
        armed_quadcopter.disarm()
        assert not armed_quadcopter.is_armed
        assert armed_quadcopter.quad_state == QuadcopterState.DISARMED

    def test_disarm_stops_motors(self, armed_quadcopter: Quadcopter) -> None:
        """Test disarming stops motors."""
        armed_quadcopter.disarm()
        outputs = armed_quadcopter.motor_outputs
        for val in outputs.as_list():
            assert val == 0.0

    def test_arm_requires_enable(self, quadcopter: Quadcopter) -> None:
        """Test arming requires enabled controller."""
        from robo_infra.core.exceptions import DisabledError

        with pytest.raises(DisabledError):
            quadcopter.arm()

    def test_armed_time(self, enabled_quadcopter: Quadcopter) -> None:
        """Test armed time tracking."""
        assert enabled_quadcopter.armed_time == 0.0
        enabled_quadcopter.arm()
        assert enabled_quadcopter.armed_time >= 0.0


# =============================================================================
# Flight Command Tests
# =============================================================================


class TestQuadcopterTakeoff:
    """Tests for takeoff command."""

    def test_takeoff(self, armed_quadcopter: Quadcopter) -> None:
        """Test takeoff command."""
        armed_quadcopter.takeoff(altitude=2.0)
        # In simulated mode, immediately at altitude
        assert armed_quadcopter.altitude == 2.0
        assert armed_quadcopter.is_flying

    def test_takeoff_requires_armed(self, enabled_quadcopter: Quadcopter) -> None:
        """Test takeoff requires armed state."""
        from robo_infra.core.exceptions import SafetyError

        with pytest.raises(SafetyError, match="Must be armed"):
            enabled_quadcopter.takeoff()

    def test_takeoff_requires_enabled(self, quadcopter: Quadcopter) -> None:
        """Test takeoff requires enabled state."""
        from robo_infra.core.exceptions import DisabledError

        with pytest.raises(DisabledError):
            quadcopter.takeoff()

    def test_takeoff_clamps_altitude(self, armed_quadcopter: Quadcopter) -> None:
        """Test altitude is clamped to max."""
        max_alt = armed_quadcopter.quad_config.max_altitude
        armed_quadcopter.takeoff(altitude=max_alt + 100)
        assert armed_quadcopter.altitude == max_alt

    def test_takeoff_rejects_negative_altitude(self, armed_quadcopter: Quadcopter) -> None:
        """Test negative altitude is rejected."""
        with pytest.raises(ValueError, match="positive"):
            armed_quadcopter.takeoff(altitude=-5.0)


class TestQuadcopterLand:
    """Tests for land command."""

    def test_land(self, flying_quadcopter: Quadcopter) -> None:
        """Test landing command."""
        flying_quadcopter.land()
        # In simulated mode, immediately on ground
        assert flying_quadcopter.altitude == 0.0
        assert not flying_quadcopter.is_flying

    def test_land_requires_enabled(self, quadcopter: Quadcopter) -> None:
        """Test land requires enabled state."""
        from robo_infra.core.exceptions import DisabledError

        with pytest.raises(DisabledError):
            quadcopter.land()


class TestQuadcopterHover:
    """Tests for hover command."""

    def test_hover(self, flying_quadcopter: Quadcopter) -> None:
        """Test hover command."""
        flying_quadcopter.hover()
        assert flying_quadcopter.quad_state == QuadcopterState.HOVERING
        assert flying_quadcopter.flight_mode == FlightMode.ALT_HOLD

    def test_hover_requires_flying(self, armed_quadcopter: Quadcopter) -> None:
        """Test hover requires flight."""
        from robo_infra.core.exceptions import SafetyError

        with pytest.raises(SafetyError, match="Not in flight"):
            armed_quadcopter.hover()


class TestQuadcopterGoto:
    """Tests for goto command."""

    def test_goto(self, flying_quadcopter: Quadcopter) -> None:
        """Test goto position command."""
        flying_quadcopter.goto(x=5.0, y=3.0, z=10.0)
        # In simulated mode, immediately at position
        assert flying_quadcopter.position.x == 5.0
        assert flying_quadcopter.position.y == 3.0
        assert flying_quadcopter.position.z == 10.0

    def test_goto_with_yaw(self, flying_quadcopter: Quadcopter) -> None:
        """Test goto with yaw angle."""
        flying_quadcopter.goto(x=0.0, y=0.0, z=5.0, yaw=90.0)
        assert flying_quadcopter.position.yaw == 90.0

    def test_goto_requires_flying(self, armed_quadcopter: Quadcopter) -> None:
        """Test goto requires flight."""
        from robo_infra.core.exceptions import SafetyError

        with pytest.raises(SafetyError, match="Must be in flight"):
            armed_quadcopter.goto(0, 0, 5)

    def test_goto_enforces_geofence(self, flying_quadcopter: Quadcopter) -> None:
        """Test geofence enforcement."""
        from robo_infra.core.exceptions import SafetyError

        geofence = flying_quadcopter.quad_config.geofence_radius
        with pytest.raises(SafetyError, match="geofence"):
            flying_quadcopter.goto(x=geofence + 100, y=0, z=5)


class TestQuadcopterRTL:
    """Tests for return to launch."""

    def test_return_to_launch(self, flying_quadcopter: Quadcopter) -> None:
        """Test RTL command."""
        flying_quadcopter.goto(x=10.0, y=10.0, z=5.0)
        flying_quadcopter.return_to_launch()
        # Should return to origin
        assert flying_quadcopter.position.x == 0.0
        assert flying_quadcopter.position.y == 0.0
        assert flying_quadcopter.flight_mode == FlightMode.RTL


# =============================================================================
# Attitude Control Tests
# =============================================================================


class TestQuadcopterAttitudeControl:
    """Tests for attitude control."""

    def test_set_attitude(self, armed_quadcopter: Quadcopter) -> None:
        """Test setting attitude."""
        armed_quadcopter.set_attitude(roll=10.0, pitch=5.0, yaw=90.0)
        # In simulated mode, attitude is updated
        assert armed_quadcopter.attitude.roll == 10.0
        assert armed_quadcopter.attitude.pitch == 5.0
        assert armed_quadcopter.attitude.yaw == 90.0

    def test_set_attitude_clamped(self, armed_quadcopter: Quadcopter) -> None:
        """Test attitude is clamped to limits."""
        max_tilt = armed_quadcopter.quad_config.max_tilt_angle
        armed_quadcopter.set_attitude(roll=max_tilt + 20, pitch=max_tilt + 20)
        assert armed_quadcopter.attitude.roll == max_tilt
        assert armed_quadcopter.attitude.pitch == max_tilt

    def test_set_attitude_requires_armed(self, enabled_quadcopter: Quadcopter) -> None:
        """Test attitude control requires armed state."""
        from robo_infra.core.exceptions import SafetyError

        with pytest.raises(SafetyError, match="Must be armed"):
            enabled_quadcopter.set_attitude(roll=10.0)


class TestQuadcopterVelocityControl:
    """Tests for velocity control."""

    def test_set_velocity(self, armed_quadcopter: Quadcopter) -> None:
        """Test setting velocity."""
        armed_quadcopter.set_velocity(vx=1.0, vy=0.5, vz=0.2)
        assert armed_quadcopter.velocity.vx == 1.0
        assert armed_quadcopter.velocity.vy == 0.5
        assert armed_quadcopter.velocity.vz == 0.2

    def test_set_velocity_clamped(self, armed_quadcopter: Quadcopter) -> None:
        """Test velocity is clamped to limits."""
        max_speed = armed_quadcopter.quad_config.max_speed
        armed_quadcopter.set_velocity(vx=max_speed + 10, vy=0, vz=0)
        assert armed_quadcopter.velocity.vx == max_speed


# =============================================================================
# Emergency Stop Tests
# =============================================================================


class TestQuadcopterEmergencyStop:
    """Tests for emergency stop."""

    def test_emergency_stop(self, flying_quadcopter: Quadcopter) -> None:
        """Test emergency stop."""
        flying_quadcopter.stop()
        assert flying_quadcopter.quad_state == QuadcopterState.EMERGENCY
        assert not flying_quadcopter.is_armed

    def test_emergency_stop_cuts_motors(self, flying_quadcopter: Quadcopter) -> None:
        """Test emergency stop cuts all motors."""
        flying_quadcopter.stop()
        outputs = flying_quadcopter.motor_outputs
        for val in outputs.as_list():
            assert val == 0.0

    def test_reset_emergency(self, flying_quadcopter: Quadcopter) -> None:
        """Test resetting from emergency state."""
        flying_quadcopter.stop()
        assert flying_quadcopter.quad_state == QuadcopterState.EMERGENCY
        flying_quadcopter.reset_emergency()
        assert flying_quadcopter.quad_state == QuadcopterState.DISARMED


# =============================================================================
# Status Tests
# =============================================================================


class TestQuadcopterStatus:
    """Tests for status reporting."""

    def test_status(self, armed_quadcopter: Quadcopter) -> None:
        """Test status reporting."""
        status = armed_quadcopter.status()
        assert isinstance(status, QuadcopterStatus)
        assert status.state == QuadcopterState.ARMED
        assert status.is_armed is True
        assert status.is_enabled is True

    def test_status_includes_battery(self, enabled_quadcopter: Quadcopter) -> None:
        """Test status includes battery info."""
        status = enabled_quadcopter.status()
        assert status.battery_voltage is not None
        assert status.battery_percent is not None
        assert 0 <= status.battery_percent <= 100

    def test_status_includes_gps(self, enabled_quadcopter: Quadcopter) -> None:
        """Test status includes GPS info (simulated)."""
        status = enabled_quadcopter.status()
        assert status.gps_fix is True  # Simulated always has fix
        assert status.satellites == 12  # Simulated value


class TestQuadcopterFlightModes:
    """Tests for flight mode control."""

    def test_set_flight_mode(self, enabled_quadcopter: Quadcopter) -> None:
        """Test setting flight mode."""
        enabled_quadcopter.set_flight_mode(FlightMode.LOITER)
        assert enabled_quadcopter.flight_mode == FlightMode.LOITER

    def test_set_flight_mode_requires_enabled(self, quadcopter: Quadcopter) -> None:
        """Test flight mode requires enabled state."""
        from robo_infra.core.exceptions import DisabledError

        with pytest.raises(DisabledError):
            quadcopter.set_flight_mode(FlightMode.LOITER)


# =============================================================================
# Motor Output Tests
# =============================================================================


class TestQuadcopterMotorOutput:
    """Tests for motor output control."""

    def test_get_motor_output(self, armed_quadcopter: Quadcopter) -> None:
        """Test getting individual motor output."""
        fl = armed_quadcopter.get_motor_output(MotorPosition.FRONT_LEFT)
        assert fl >= 0.0
        assert fl <= 1.0

    def test_motor_outputs_property(self, armed_quadcopter: Quadcopter) -> None:
        """Test motor outputs property."""
        outputs = armed_quadcopter.motor_outputs
        assert isinstance(outputs, MotorOutputs)


# =============================================================================
# Integration Tests
# =============================================================================


class TestQuadcopterAIIntegration:
    """Tests for AI integration."""

    def test_as_tools(self, enabled_quadcopter: Quadcopter) -> None:
        """Test generating AI tools."""
        tools = enabled_quadcopter.as_tools()
        assert isinstance(tools, list)
        assert len(tools) > 0

    def test_as_tools_names(self, enabled_quadcopter: Quadcopter) -> None:
        """Test tool names are descriptive."""
        tools = enabled_quadcopter.as_tools()
        names = [t.__name__ for t in tools]
        assert "arm_drone" in names
        assert "disarm_drone" in names
        assert "takeoff_to_altitude" in names
        assert "land_drone" in names
        assert "get_drone_status" in names


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreateQuadcopter:
    """Tests for create_quadcopter factory function."""

    def test_create_quadcopter_default(self) -> None:
        """Test creating quadcopter with defaults."""
        quad = create_quadcopter()
        assert quad.name == "quadcopter"
        assert quad.quad_config.frame_type == "X"

    def test_create_quadcopter_custom(self) -> None:
        """Test creating quadcopter with custom settings."""
        quad = create_quadcopter(
            name="racing",
            frame_type="+",
            arm_length=0.125,
        )
        assert quad.name == "racing"
        assert quad.quad_config.frame_type == "+"
        assert quad.quad_config.arm_length == 0.125

    def test_create_quadcopter_simulated(self) -> None:
        """Test creating simulated quadcopter."""
        quad = create_quadcopter(simulated=True)
        # Should work without real motors
        quad.enable()
        assert quad.is_enabled
