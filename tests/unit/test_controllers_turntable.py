"""Tests for robo_infra.controllers.turntable module."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from robo_infra.actuators.stepper import Stepper
from robo_infra.controllers.turntable import (
    RotationDirection,
    Turntable,
    TurntableConfig,
    TurntableState,
    TurntableStatus,
    create_turntable,
    turntable_status,
)
from robo_infra.core.exceptions import DisabledError
from robo_infra.core.sensor import Sensor


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def turntable_motor() -> Stepper:
    """Create a simulated turntable motor."""
    return Stepper(
        step_pin=0,
        dir_pin=1,
        name="turntable_motor",
    )


@pytest.fixture
def home_sensor() -> MagicMock:
    """Create a mock home sensor."""
    sensor = MagicMock(spec=Sensor)
    sensor.name = "home_sensor"
    sensor.read.return_value = MagicMock(value=1.0)
    return sensor


@pytest.fixture
def encoder_sensor() -> MagicMock:
    """Create a mock encoder sensor."""
    sensor = MagicMock(spec=Sensor)
    sensor.name = "encoder"
    sensor.read.return_value = MagicMock(value=0.0)
    return sensor


@pytest.fixture
def turntable_config() -> TurntableConfig:
    """Create default turntable configuration."""
    return TurntableConfig(
        name="test_turntable",
        gear_ratio=5.0,
        speed_max=360.0,
        speed_default=90.0,
        rpm_max=60.0,
    )


@pytest.fixture
def limited_config() -> TurntableConfig:
    """Create turntable configuration with angle limits."""
    return TurntableConfig(
        name="limited_turntable",
        angle_min=-180.0,
        angle_max=180.0,
        speed_max=360.0,
    )


@pytest.fixture
def indexed_config() -> TurntableConfig:
    """Create turntable configuration with index positions."""
    return TurntableConfig(
        name="indexed_turntable",
        index_positions=[0.0, 90.0, 180.0, 270.0],
    )


@pytest.fixture
def turntable(turntable_motor: Stepper, turntable_config: TurntableConfig) -> Turntable:
    """Create a Turntable controller with default config."""
    return Turntable(
        name="test_turntable",
        motor=turntable_motor,
        config=turntable_config,
    )


@pytest.fixture
def enabled_turntable(turntable: Turntable) -> Turntable:
    """Create and enable a Turntable controller."""
    turntable.enable()
    return turntable


@pytest.fixture
def limited_turntable(turntable_motor: Stepper, limited_config: TurntableConfig) -> Turntable:
    """Create a Turntable with angle limits."""
    tt = Turntable(
        name="limited",
        motor=turntable_motor,
        config=limited_config,
    )
    tt.enable()
    return tt


@pytest.fixture
def indexed_turntable(turntable_motor: Stepper, indexed_config: TurntableConfig) -> Turntable:
    """Create a Turntable with index positions."""
    tt = Turntable(
        name="indexed",
        motor=turntable_motor,
        config=indexed_config,
    )
    tt.enable()
    return tt


# =============================================================================
# Turntable Configuration Tests
# =============================================================================


class TestTurntableConfig:
    """Tests for TurntableConfig model."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = TurntableConfig(name="test")
        assert config.name == "test"
        assert config.gear_ratio == 1.0
        assert config.speed_max == 360.0
        assert config.rpm_max == 60.0
        assert config.angle_min is None
        assert config.angle_max is None
        assert config.is_limited is False

    def test_limited_config(self) -> None:
        """Test configuration with limits."""
        config = TurntableConfig(
            name="limited",
            angle_min=-90.0,
            angle_max=90.0,
        )
        assert config.is_limited is True
        assert config.angle_min == -90.0
        assert config.angle_max == 90.0

    def test_index_positions(self) -> None:
        """Test configuration with index positions."""
        config = TurntableConfig(
            name="indexed",
            index_positions=[0, 45, 90, 135, 180],
        )
        assert len(config.index_positions) == 5
        assert config.index_positions[2] == 90

    def test_config_validation(self) -> None:
        """Test configuration validation."""
        with pytest.raises(ValueError):
            TurntableConfig(name="test", gear_ratio=0)
        with pytest.raises(ValueError):
            TurntableConfig(name="test", speed_max=0)


# =============================================================================
# Turntable Initialization Tests
# =============================================================================


class TestTurntableInit:
    """Tests for Turntable initialization."""

    def test_create_turntable(self, turntable: Turntable) -> None:
        """Test basic turntable creation."""
        assert turntable.name == "test_turntable"
        assert turntable.turntable_state == TurntableState.DISABLED
        assert turntable.current_angle == 0.0
        assert turntable.current_speed == 0.0

    def test_motor_required(self) -> None:
        """Test that motor is required."""
        with pytest.raises(ValueError, match="motor is required"):
            Turntable(name="test", motor=None)  # type: ignore[arg-type]

    def test_default_config_creation(self, turntable_motor: Stepper) -> None:
        """Test turntable with no config uses defaults."""
        tt = Turntable(name="test", motor=turntable_motor)
        assert tt.turntable_config is not None
        assert tt.turntable_config.name == "test"

    def test_turntable_with_sensors(
        self, turntable_motor: Stepper, home_sensor: MagicMock, encoder_sensor: MagicMock
    ) -> None:
        """Test turntable with sensors."""
        tt = Turntable(
            name="test",
            motor=turntable_motor,
            home_sensor=home_sensor,
            encoder=encoder_sensor,
        )
        assert tt.home_sensor is not None
        assert tt.encoder is not None


# =============================================================================
# Turntable Lifecycle Tests
# =============================================================================


class TestTurntableLifecycle:
    """Tests for Turntable enable/disable lifecycle."""

    def test_enable(self, turntable: Turntable) -> None:
        """Test enabling the turntable."""
        assert turntable.turntable_state == TurntableState.DISABLED
        turntable.enable()
        assert turntable.turntable_state == TurntableState.IDLE
        assert turntable._is_enabled is True

    def test_disable(self, enabled_turntable: Turntable) -> None:
        """Test disabling the turntable."""
        enabled_turntable.disable()
        assert enabled_turntable.turntable_state == TurntableState.DISABLED
        assert enabled_turntable._is_enabled is False

    def test_home_without_sensor(self, enabled_turntable: Turntable) -> None:
        """Test homing without home sensor."""
        enabled_turntable._current_angle = 45.0
        enabled_turntable.home()
        assert enabled_turntable.current_angle == 0.0
        assert enabled_turntable._is_homed is True

    def test_home_with_sensor(self, turntable_motor: Stepper, home_sensor: MagicMock) -> None:
        """Test homing with home sensor."""
        config = TurntableConfig(name="test", home_offset=10.0)
        tt = Turntable(
            name="test",
            motor=turntable_motor,
            config=config,
            home_sensor=home_sensor,
        )
        tt.enable()
        tt.home()
        assert tt.current_angle == 10.0  # home_offset
        assert tt._is_homed is True


# =============================================================================
# Turntable Rotate To Tests
# =============================================================================


class TestTurntableRotateTo:
    """Tests for Turntable rotate_to operation."""

    def test_rotate_to_positive(self, enabled_turntable: Turntable) -> None:
        """Test rotating to positive angle."""
        enabled_turntable.rotate_to(90.0)
        assert enabled_turntable.current_angle == 90.0
        assert enabled_turntable.turntable_state == TurntableState.IDLE

    def test_rotate_to_negative(self, enabled_turntable: Turntable) -> None:
        """Test rotating to negative angle."""
        enabled_turntable.rotate_to(-45.0)
        assert enabled_turntable.current_angle == -45.0

    def test_rotate_to_with_speed(self, enabled_turntable: Turntable) -> None:
        """Test rotating with specific speed."""
        enabled_turntable.rotate_to(90.0, speed=180.0)
        assert enabled_turntable.current_angle == 90.0

    def test_rotate_to_exceeds_speed(self, enabled_turntable: Turntable) -> None:
        """Test that exceeding max speed raises error."""
        with pytest.raises(ValueError, match="exceeds maximum"):
            enabled_turntable.rotate_to(90.0, speed=500.0)

    def test_rotate_to_requires_enabled(self, turntable: Turntable) -> None:
        """Test that rotate_to requires enabled turntable."""
        with pytest.raises(DisabledError):
            turntable.rotate_to(90.0)

    def test_rotate_to_below_limit(self, limited_turntable: Turntable) -> None:
        """Test that rotating below minimum raises error."""
        with pytest.raises(ValueError, match="below minimum"):
            limited_turntable.rotate_to(-200.0)

    def test_rotate_to_above_limit(self, limited_turntable: Turntable) -> None:
        """Test that rotating above maximum raises error."""
        with pytest.raises(ValueError, match="above maximum"):
            limited_turntable.rotate_to(200.0)


# =============================================================================
# Turntable Rotate By Tests
# =============================================================================


class TestTurntableRotateBy:
    """Tests for Turntable rotate_by operation."""

    def test_rotate_by_positive(self, enabled_turntable: Turntable) -> None:
        """Test rotating by positive delta."""
        enabled_turntable.rotate_by(45.0)
        assert enabled_turntable.current_angle == 45.0

    def test_rotate_by_negative(self, enabled_turntable: Turntable) -> None:
        """Test rotating by negative delta."""
        enabled_turntable.rotate_by(-30.0)
        assert enabled_turntable.current_angle == -30.0

    def test_rotate_by_cumulative(self, enabled_turntable: Turntable) -> None:
        """Test multiple rotate_by operations."""
        enabled_turntable.rotate_by(45.0)
        enabled_turntable.rotate_by(45.0)
        assert enabled_turntable.current_angle == 90.0

    def test_rotate_by_requires_enabled(self, turntable: Turntable) -> None:
        """Test that rotate_by requires enabled turntable."""
        with pytest.raises(DisabledError):
            turntable.rotate_by(45.0)


# =============================================================================
# Turntable Continuous Rotation Tests
# =============================================================================


class TestTurntableContinuous:
    """Tests for Turntable continuous rotation."""

    def test_continuous_clockwise(self, enabled_turntable: Turntable) -> None:
        """Test continuous rotation clockwise."""
        enabled_turntable.continuous(rpm=30.0)
        assert enabled_turntable.turntable_state == TurntableState.CONTINUOUS
        assert enabled_turntable.direction == RotationDirection.CLOCKWISE

    def test_continuous_counterclockwise(self, enabled_turntable: Turntable) -> None:
        """Test continuous rotation counter-clockwise."""
        enabled_turntable.continuous(rpm=-30.0)
        assert enabled_turntable.turntable_state == TurntableState.CONTINUOUS
        assert enabled_turntable.direction == RotationDirection.COUNTERCLOCKWISE

    def test_continuous_exceeds_rpm(self, enabled_turntable: Turntable) -> None:
        """Test that exceeding max RPM raises error."""
        with pytest.raises(ValueError, match="exceeds maximum"):
            enabled_turntable.continuous(rpm=100.0)

    def test_continuous_not_allowed_with_limits(self, limited_turntable: Turntable) -> None:
        """Test that continuous rotation not allowed with limits."""
        with pytest.raises(ValueError, match="not allowed with angle limits"):
            limited_turntable.continuous(rpm=30.0)

    def test_continuous_requires_enabled(self, turntable: Turntable) -> None:
        """Test that continuous requires enabled turntable."""
        with pytest.raises(DisabledError):
            turntable.continuous(rpm=30.0)


# =============================================================================
# Turntable Stop Tests
# =============================================================================


class TestTurntableStop:
    """Tests for Turntable stop operation."""

    def test_stop_moving_turntable(self, enabled_turntable: Turntable) -> None:
        """Test stopping a moving turntable."""
        enabled_turntable.continuous(rpm=30.0)
        enabled_turntable.stop()
        assert enabled_turntable.turntable_state == TurntableState.IDLE
        assert enabled_turntable._is_continuous is False

    def test_stop_disabled_turntable(self, turntable: Turntable) -> None:
        """Test stopping a disabled turntable does nothing."""
        turntable.stop()  # Should not raise
        assert turntable.turntable_state == TurntableState.DISABLED

    def test_emergency_stop(self, enabled_turntable: Turntable) -> None:
        """Test emergency stop."""
        enabled_turntable.continuous(rpm=30.0)
        enabled_turntable.emergency_stop()
        assert enabled_turntable.turntable_state == TurntableState.IDLE
        assert enabled_turntable._is_continuous is False


# =============================================================================
# Turntable Index Position Tests
# =============================================================================


class TestTurntableIndex:
    """Tests for Turntable index position operations."""

    def test_go_to_index(self, indexed_turntable: Turntable) -> None:
        """Test going to index position."""
        indexed_turntable.go_to_index(2)
        assert indexed_turntable.current_angle == 180.0

    def test_go_to_index_out_of_range(self, indexed_turntable: Turntable) -> None:
        """Test going to invalid index raises error."""
        with pytest.raises(ValueError, match="out of range"):
            indexed_turntable.go_to_index(10)

    def test_go_to_index_no_positions(self, enabled_turntable: Turntable) -> None:
        """Test going to index with no positions defined raises error."""
        with pytest.raises(ValueError, match="No index positions"):
            enabled_turntable.go_to_index(0)

    def test_next_index(self, indexed_turntable: Turntable) -> None:
        """Test moving to next index."""
        indexed_turntable.next_index()
        assert indexed_turntable.current_angle == 90.0

    def test_next_index_wraps(self, indexed_turntable: Turntable) -> None:
        """Test next index wraps to first."""
        indexed_turntable.rotate_to(270.0)
        indexed_turntable.next_index()
        assert indexed_turntable.current_angle == 0.0

    def test_prev_index(self, indexed_turntable: Turntable) -> None:
        """Test moving to previous index."""
        indexed_turntable.rotate_to(180.0)
        indexed_turntable.prev_index()
        assert indexed_turntable.current_angle == 90.0

    def test_prev_index_wraps(self, indexed_turntable: Turntable) -> None:
        """Test prev index wraps to last."""
        indexed_turntable.prev_index()
        assert indexed_turntable.current_angle == 270.0


# =============================================================================
# Turntable Status Tests
# =============================================================================


class TestTurntableStatus:
    """Tests for Turntable status."""

    def test_status_idle(self, enabled_turntable: Turntable) -> None:
        """Test status when idle."""
        status = enabled_turntable.status()
        assert isinstance(status, TurntableStatus)
        assert status.state == TurntableState.IDLE
        assert status.current_angle == 0.0
        assert status.is_enabled is True

    def test_status_continuous(self, enabled_turntable: Turntable) -> None:
        """Test status during continuous rotation."""
        enabled_turntable.continuous(rpm=30.0)
        status = enabled_turntable.status()
        assert status.state == TurntableState.CONTINUOUS
        assert status.direction == RotationDirection.CLOCKWISE

    def test_turntable_status_function(self, enabled_turntable: Turntable) -> None:
        """Test turntable_status helper function."""
        enabled_turntable.rotate_to(45.0)
        status_dict = turntable_status(enabled_turntable)
        assert status_dict["name"] == "test_turntable"
        assert status_dict["state"] == "idle"
        assert status_dict["current_angle"] == 45.0


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreateTurntable:
    """Tests for create_turntable factory function."""

    def test_create_default(self) -> None:
        """Test creating turntable with defaults."""
        tt = create_turntable("test")
        assert tt.name == "test"
        assert tt.motor is not None
        assert tt.turntable_config.gear_ratio == 1.0

    def test_create_with_params(self) -> None:
        """Test creating turntable with custom parameters."""
        tt = create_turntable(
            "custom",
            gear_ratio=10.0,
            speed_max=180.0,
            index_positions=[0, 60, 120, 180, 240, 300],
        )
        assert tt.turntable_config.gear_ratio == 10.0
        assert tt.turntable_config.speed_max == 180.0
        assert len(tt.turntable_config.index_positions) == 6

    def test_create_with_motor(self, turntable_motor: Stepper) -> None:
        """Test creating turntable with provided motor."""
        tt = create_turntable("test", motor=turntable_motor)
        assert tt.motor is turntable_motor


# =============================================================================
# Tool Generation Tests
# =============================================================================


class TestTurntableTools:
    """Tests for Turntable tool generation."""

    def test_as_tools_returns_list(self, enabled_turntable: Turntable) -> None:
        """Test that as_tools returns a list of callables."""
        tools = enabled_turntable.as_tools()
        assert isinstance(tools, list)
        assert len(tools) >= 6
        for tool in tools:
            assert callable(tool)

    def test_rotate_to_tool(self, enabled_turntable: Turntable) -> None:
        """Test rotate_turntable_to tool."""
        tools = enabled_turntable.as_tools()
        rotate_tool = tools[0]
        result = rotate_tool(angle=90.0)
        assert "90" in result
        assert enabled_turntable.current_angle == 90.0

    def test_rotate_by_tool(self, enabled_turntable: Turntable) -> None:
        """Test rotate_turntable_by tool."""
        tools = enabled_turntable.as_tools()
        rotate_by_tool = tools[1]
        result = rotate_by_tool(delta=45.0)
        assert "45" in result
        assert enabled_turntable.current_angle == 45.0

    def test_home_tool(self, enabled_turntable: Turntable) -> None:
        """Test home_turntable tool."""
        tools = enabled_turntable.as_tools()
        enabled_turntable.rotate_to(90.0)
        home_tool = tools[2]
        result = home_tool()
        assert "homed" in result.lower()
        assert enabled_turntable.current_angle == 0.0

    def test_continuous_tool(self, enabled_turntable: Turntable) -> None:
        """Test continuous_rotation tool."""
        tools = enabled_turntable.as_tools()
        continuous_tool = tools[3]
        result = continuous_tool(rpm=30.0)
        assert "30" in result
        assert enabled_turntable.turntable_state == TurntableState.CONTINUOUS

    def test_stop_tool(self, enabled_turntable: Turntable) -> None:
        """Test stop_turntable tool."""
        tools = enabled_turntable.as_tools()
        enabled_turntable.continuous(rpm=30.0)
        stop_tool = tools[4]
        result = stop_tool()
        assert "stopped" in result.lower()

    def test_status_tool(self, enabled_turntable: Turntable) -> None:
        """Test get_turntable_status tool."""
        tools = enabled_turntable.as_tools()
        status_tool = tools[5]
        result = status_tool()
        assert isinstance(result, dict)
        assert "state" in result
        assert "current_angle" in result


# =============================================================================
# Rotation Direction Tests
# =============================================================================


class TestRotationDirection:
    """Tests for RotationDirection enum."""

    def test_direction_values(self) -> None:
        """Test direction enum values."""
        assert RotationDirection.CLOCKWISE.value == 1
        assert RotationDirection.COUNTERCLOCKWISE.value == -1


# =============================================================================
# Angle Normalization Tests
# =============================================================================


class TestAngleNormalization:
    """Tests for angle normalization."""

    def test_normalize_positive_angle(self) -> None:
        """Test normalizing large positive angle."""
        assert Turntable._normalize_angle(270.0) == -90.0

    def test_normalize_negative_angle(self) -> None:
        """Test normalizing large negative angle."""
        assert Turntable._normalize_angle(-270.0) == 90.0

    def test_normalize_small_angle(self) -> None:
        """Test normalizing small angle (no change)."""
        assert Turntable._normalize_angle(45.0) == 45.0
        assert Turntable._normalize_angle(-45.0) == -45.0

    def test_normalize_edge_cases(self) -> None:
        """Test normalizing edge case angles."""
        assert Turntable._normalize_angle(180.0) == 180.0
        assert Turntable._normalize_angle(-180.0) == -180.0
