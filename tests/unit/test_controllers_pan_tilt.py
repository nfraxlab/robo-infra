"""Tests for robo_infra.controllers.pan_tilt module."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from robo_infra.actuators.servo import Servo
from robo_infra.controllers.pan_tilt import (
    PanTilt,
    PanTiltConfig,
    PanTiltState,
    PanTiltStatus,
    create_pan_tilt,
    pan_tilt_status,
)
from robo_infra.core.exceptions import DisabledError
from robo_infra.core.sensor import Sensor


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def pan_servo() -> Servo:
    """Create a simulated pan servo."""
    return Servo(
        name="pan_servo",
        angle_range=(-90.0, 90.0),
    )


@pytest.fixture
def tilt_servo() -> Servo:
    """Create a simulated tilt servo."""
    return Servo(
        name="tilt_servo",
        angle_range=(-45.0, 45.0),
    )


@pytest.fixture
def wide_pan_servo() -> Servo:
    """Create a pan servo with wider range."""
    return Servo(
        name="pan_servo",
        angle_range=(-180.0, 180.0),
    )


@pytest.fixture
def imu_sensor() -> MagicMock:
    """Create a mock IMU sensor for tracking."""
    sensor = MagicMock(spec=Sensor)
    sensor.name = "imu"
    sensor.read.return_value = MagicMock(value=0.0)
    return sensor


@pytest.fixture
def pan_tilt_config() -> PanTiltConfig:
    """Create default pan-tilt configuration."""
    return PanTiltConfig(
        name="test_head",
        pan_range=(-90.0, 90.0),
        tilt_range=(-45.0, 45.0),
        pan_speed=45.0,
        tilt_speed=30.0,
    )


@pytest.fixture
def wide_config() -> PanTiltConfig:
    """Create pan-tilt configuration with wide range."""
    return PanTiltConfig(
        name="wide_head",
        pan_range=(-180.0, 180.0),
        tilt_range=(-90.0, 90.0),
    )


@pytest.fixture
def pan_tilt(
    pan_servo: Servo, tilt_servo: Servo, pan_tilt_config: PanTiltConfig
) -> PanTilt:
    """Create a PanTilt controller with default config."""
    return PanTilt(
        name="test_head",
        pan_actuator=pan_servo,
        tilt_actuator=tilt_servo,
        config=pan_tilt_config,
    )


@pytest.fixture
def enabled_pan_tilt(pan_tilt: PanTilt) -> PanTilt:
    """Create and enable a PanTilt controller."""
    pan_tilt.enable()
    return pan_tilt


@pytest.fixture
def wide_pan_tilt(
    wide_pan_servo: Servo, tilt_servo: Servo, wide_config: PanTiltConfig
) -> PanTilt:
    """Create a PanTilt with wide range."""
    pt = PanTilt(
        name="wide",
        pan_actuator=wide_pan_servo,
        tilt_actuator=tilt_servo,
        config=wide_config,
    )
    pt.enable()
    return pt


# =============================================================================
# PanTilt Configuration Tests
# =============================================================================


class TestPanTiltConfig:
    """Tests for PanTiltConfig model."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = PanTiltConfig(name="test")
        assert config.name == "test"
        assert config.pan_min == -90.0
        assert config.pan_max == 90.0
        assert config.tilt_min == -45.0
        assert config.tilt_max == 45.0
        assert config.pan_speed == 90.0  # Default value
        assert config.tilt_speed == 90.0  # Default value
        assert config.invert_pan is False
        assert config.invert_tilt is False

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = PanTiltConfig(
            name="custom",
            pan_range=(-180.0, 180.0),
            tilt_range=(-90.0, 90.0),
            pan_speed=60.0,
            tilt_speed=45.0,
            invert_pan=True,
            invert_tilt=True,
        )
        assert config.pan_min == -180.0
        assert config.pan_max == 180.0
        assert config.pan_speed == 60.0
        assert config.invert_pan is True

    def test_config_range_properties(self) -> None:
        """Test configuration range properties."""
        config = PanTiltConfig(
            name="test",
            pan_range=(-45.0, 45.0),
            tilt_range=(-30.0, 30.0),
        )
        # pan_range is a tuple, not the range value
        assert config.pan_range == (-45.0, 45.0)
        assert config.tilt_range == (-30.0, 30.0)

    def test_config_validation(self) -> None:
        """Test configuration validation."""
        with pytest.raises(ValueError):
            PanTiltConfig(name="test", pan_speed=0)
        with pytest.raises(ValueError):
            PanTiltConfig(name="test", tilt_speed=-10)


# =============================================================================
# PanTilt Initialization Tests
# =============================================================================


class TestPanTiltInit:
    """Tests for PanTilt initialization."""

    def test_create_pan_tilt(self, pan_tilt: PanTilt) -> None:
        """Test basic pan-tilt creation."""
        assert pan_tilt.name == "test_head"
        assert pan_tilt.pt_state == PanTiltState.DISABLED
        assert pan_tilt.pan_angle == 0.0
        assert pan_tilt.tilt_angle == 0.0

    def test_servos_required(self, pan_servo: Servo) -> None:
        """Test that both actuators are required."""
        with pytest.raises(ValueError, match="Pan actuator is required"):
            PanTilt(name="test", pan_actuator=None, tilt_actuator=pan_servo)  # type: ignore[arg-type]
        with pytest.raises(ValueError, match="Tilt actuator is required"):
            PanTilt(name="test", pan_actuator=pan_servo, tilt_actuator=None)  # type: ignore[arg-type]

    def test_default_config_creation(
        self, pan_servo: Servo, tilt_servo: Servo
    ) -> None:
        """Test pan-tilt with no config uses defaults."""
        pt = PanTilt(name="test", pan_actuator=pan_servo, tilt_actuator=tilt_servo)
        assert pt.pt_config is not None
        assert pt.pt_config.name == "test"


# =============================================================================
# PanTilt Lifecycle Tests
# =============================================================================


class TestPanTiltLifecycle:
    """Tests for PanTilt enable/disable lifecycle."""

    def test_enable(self, pan_tilt: PanTilt) -> None:
        """Test enabling the pan-tilt."""
        assert pan_tilt.pt_state == PanTiltState.DISABLED
        pan_tilt.enable()
        assert pan_tilt.pt_state == PanTiltState.IDLE
        assert pan_tilt._is_enabled is True

    def test_disable(self, enabled_pan_tilt: PanTilt) -> None:
        """Test disabling the pan-tilt."""
        enabled_pan_tilt.disable()
        assert enabled_pan_tilt.pt_state == PanTiltState.DISABLED
        assert enabled_pan_tilt._is_enabled is False

    def test_home(self, enabled_pan_tilt: PanTilt) -> None:
        """Test homing the pan-tilt."""
        enabled_pan_tilt.pan_to(45.0)
        enabled_pan_tilt.tilt_to(30.0)
        enabled_pan_tilt.home()
        assert enabled_pan_tilt.pan_angle == 0.0
        assert enabled_pan_tilt.tilt_angle == 0.0

    def test_center(self, enabled_pan_tilt: PanTilt) -> None:
        """Test centering the pan-tilt (same as home)."""
        enabled_pan_tilt.pan_to(-45.0)
        enabled_pan_tilt.tilt_to(-30.0)
        enabled_pan_tilt.center()
        assert enabled_pan_tilt.pan_angle == 0.0
        assert enabled_pan_tilt.tilt_angle == 0.0


# =============================================================================
# PanTilt Look At Tests
# =============================================================================


class TestPanTiltLookAt:
    """Tests for PanTilt look_at operation."""

    def test_look_at_basic(self, enabled_pan_tilt: PanTilt) -> None:
        """Test basic look_at operation."""
        enabled_pan_tilt.look_at(pan=45.0, tilt=30.0)
        assert enabled_pan_tilt.pan_angle == 45.0
        assert enabled_pan_tilt.tilt_angle == 30.0
        assert enabled_pan_tilt.pt_state == PanTiltState.IDLE

    def test_look_at_negative(self, enabled_pan_tilt: PanTilt) -> None:
        """Test look_at with negative angles."""
        enabled_pan_tilt.look_at(pan=-45.0, tilt=-30.0)
        assert enabled_pan_tilt.pan_angle == -45.0
        assert enabled_pan_tilt.tilt_angle == -30.0

    def test_look_at_pan_exceeds_limit(self, enabled_pan_tilt: PanTilt) -> None:
        """Test look_at with pan exceeding limits."""
        with pytest.raises(ValueError, match="above maximum"):
            enabled_pan_tilt.look_at(pan=180.0, tilt=0.0)

    def test_look_at_tilt_exceeds_limit(self, enabled_pan_tilt: PanTilt) -> None:
        """Test look_at with tilt exceeding limits."""
        with pytest.raises(ValueError, match="above maximum"):
            enabled_pan_tilt.look_at(pan=0.0, tilt=90.0)

    def test_look_at_requires_enabled(self, pan_tilt: PanTilt) -> None:
        """Test that look_at requires enabled pan-tilt."""
        with pytest.raises(DisabledError):
            pan_tilt.look_at(pan=45.0, tilt=30.0)


# =============================================================================
# PanTilt Pan To Tests
# =============================================================================


class TestPanTiltPanTo:
    """Tests for PanTilt pan_to operation."""

    def test_pan_to_positive(self, enabled_pan_tilt: PanTilt) -> None:
        """Test panning to positive angle."""
        enabled_pan_tilt.pan_to(60.0)
        assert enabled_pan_tilt.pan_angle == 60.0
        assert enabled_pan_tilt.tilt_angle == 0.0

    def test_pan_to_negative(self, enabled_pan_tilt: PanTilt) -> None:
        """Test panning to negative angle."""
        enabled_pan_tilt.pan_to(-60.0)
        assert enabled_pan_tilt.pan_angle == -60.0

    def test_pan_to_exceeds_max(self, enabled_pan_tilt: PanTilt) -> None:
        """Test panning beyond maximum."""
        with pytest.raises(ValueError, match="above maximum"):
            enabled_pan_tilt.pan_to(180.0)

    def test_pan_to_exceeds_min(self, enabled_pan_tilt: PanTilt) -> None:
        """Test panning below minimum."""
        with pytest.raises(ValueError, match="below minimum"):
            enabled_pan_tilt.pan_to(-180.0)

    def test_pan_to_requires_enabled(self, pan_tilt: PanTilt) -> None:
        """Test that pan_to requires enabled pan-tilt."""
        with pytest.raises(DisabledError):
            pan_tilt.pan_to(45.0)


# =============================================================================
# PanTilt Tilt To Tests
# =============================================================================


class TestPanTiltTiltTo:
    """Tests for PanTilt tilt_to operation."""

    def test_tilt_to_positive(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tilting to positive angle."""
        enabled_pan_tilt.tilt_to(30.0)
        assert enabled_pan_tilt.tilt_angle == 30.0
        assert enabled_pan_tilt.pan_angle == 0.0

    def test_tilt_to_negative(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tilting to negative angle."""
        enabled_pan_tilt.tilt_to(-30.0)
        assert enabled_pan_tilt.tilt_angle == -30.0

    def test_tilt_to_exceeds_max(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tilting beyond maximum."""
        with pytest.raises(ValueError, match="above maximum"):
            enabled_pan_tilt.tilt_to(90.0)

    def test_tilt_to_exceeds_min(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tilting below minimum."""
        with pytest.raises(ValueError, match="below minimum"):
            enabled_pan_tilt.tilt_to(-90.0)

    def test_tilt_to_requires_enabled(self, pan_tilt: PanTilt) -> None:
        """Test that tilt_to requires enabled pan-tilt."""
        with pytest.raises(DisabledError):
            pan_tilt.tilt_to(30.0)


# =============================================================================
# PanTilt Pan/Tilt By Tests
# =============================================================================


class TestPanTiltByOperations:
    """Tests for PanTilt pan_by and tilt_by operations."""

    def test_pan_by_positive(self, enabled_pan_tilt: PanTilt) -> None:
        """Test panning by positive delta."""
        enabled_pan_tilt.pan_by(30.0)
        assert enabled_pan_tilt.pan_angle == 30.0

    def test_pan_by_negative(self, enabled_pan_tilt: PanTilt) -> None:
        """Test panning by negative delta."""
        enabled_pan_tilt.pan_by(-30.0)
        assert enabled_pan_tilt.pan_angle == -30.0

    def test_pan_by_cumulative(self, enabled_pan_tilt: PanTilt) -> None:
        """Test cumulative pan_by operations."""
        enabled_pan_tilt.pan_by(20.0)
        enabled_pan_tilt.pan_by(20.0)
        assert enabled_pan_tilt.pan_angle == 40.0

    def test_tilt_by_positive(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tilting by positive delta."""
        enabled_pan_tilt.tilt_by(15.0)
        assert enabled_pan_tilt.tilt_angle == 15.0

    def test_tilt_by_negative(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tilting by negative delta."""
        enabled_pan_tilt.tilt_by(-15.0)
        assert enabled_pan_tilt.tilt_angle == -15.0

    def test_tilt_by_cumulative(self, enabled_pan_tilt: PanTilt) -> None:
        """Test cumulative tilt_by operations."""
        enabled_pan_tilt.tilt_by(10.0)
        enabled_pan_tilt.tilt_by(10.0)
        assert enabled_pan_tilt.tilt_angle == 20.0


# =============================================================================
# PanTilt Tracking Tests
# =============================================================================


class TestPanTiltTracking:
    """Tests for PanTilt track operation."""

    def test_track_moves_toward_target(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tracking mode moves toward target."""
        # Target at center (320, 240 default) means no movement
        enabled_pan_tilt.track(target=(320.0, 240.0))
        assert enabled_pan_tilt.pan_angle == 0.0
        assert enabled_pan_tilt.tilt_angle == 0.0

    def test_track_offset_target(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tracking moves toward offset target."""
        # Target right of center should pan right
        enabled_pan_tilt.track(target=(400.0, 240.0))
        # Pan should have moved positive (right)
        assert enabled_pan_tilt.pan_angle > 0.0

    def test_track_requires_enabled(self, pan_tilt: PanTilt) -> None:
        """Test that track requires enabled pan-tilt."""
        with pytest.raises(DisabledError):
            pan_tilt.track(target=(320.0, 240.0))

    def test_track_with_custom_gain(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tracking with custom gain."""
        enabled_pan_tilt.track(target=(400.0, 240.0), gain=0.5)
        assert enabled_pan_tilt.pan_angle > 0.0


# =============================================================================
# PanTilt Stop Tests
# =============================================================================


class TestPanTiltStop:
    """Tests for PanTilt stop operations."""

    def test_stop(self, enabled_pan_tilt: PanTilt) -> None:
        """Test stopping pan-tilt."""
        enabled_pan_tilt.look_at(pan=30.0, tilt=15.0)
        enabled_pan_tilt.stop()
        assert enabled_pan_tilt.pt_state == PanTiltState.IDLE

    def test_emergency_stop(self, enabled_pan_tilt: PanTilt) -> None:
        """Test emergency stop."""
        enabled_pan_tilt.look_at(pan=30.0, tilt=15.0)
        enabled_pan_tilt.emergency_stop()
        assert enabled_pan_tilt.pt_state == PanTiltState.IDLE


# =============================================================================
# PanTilt Status Tests
# =============================================================================


class TestPanTiltStatus:
    """Tests for PanTilt status."""

    def test_status_idle(self, enabled_pan_tilt: PanTilt) -> None:
        """Test status when idle."""
        status = enabled_pan_tilt.status()
        assert isinstance(status, PanTiltStatus)
        assert status.state == PanTiltState.IDLE
        assert status.pan_angle == 0.0
        assert status.tilt_angle == 0.0
        assert status.is_enabled is True

    def test_status_after_movement(self, enabled_pan_tilt: PanTilt) -> None:
        """Test status after movement."""
        enabled_pan_tilt.look_at(pan=30.0, tilt=15.0)
        status = enabled_pan_tilt.status()
        assert status.pan_angle == 30.0
        assert status.tilt_angle == 15.0

    def test_pan_tilt_status_function(self, enabled_pan_tilt: PanTilt) -> None:
        """Test pan_tilt_status helper function."""
        enabled_pan_tilt.look_at(pan=30.0, tilt=20.0)
        status_dict = pan_tilt_status(enabled_pan_tilt)
        assert status_dict["name"] == "test_head"
        assert status_dict["state"] == "idle"
        assert status_dict["pan_angle"] == 30.0
        assert status_dict["tilt_angle"] == 20.0


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreatePanTilt:
    """Tests for create_pan_tilt factory function."""

    def test_create_default(self) -> None:
        """Test creating pan-tilt with defaults."""
        pt = create_pan_tilt("test")
        assert pt.name == "test"
        assert pt.pan_actuator is not None
        assert pt.tilt_actuator is not None
        assert pt.pt_config.pan_min == -90.0
        assert pt.pt_config.pan_max == 90.0

    def test_create_with_params(self) -> None:
        """Test creating pan-tilt with custom parameters."""
        pt = create_pan_tilt(
            "custom",
            pan_range=(-180.0, 180.0),
            tilt_range=(-60.0, 60.0),
        )
        assert pt.pt_config.pan_min == -180.0
        assert pt.pt_config.pan_max == 180.0
        assert pt.pt_config.tilt_min == -60.0
        assert pt.pt_config.tilt_max == 60.0

    def test_create_with_servos(self, pan_servo: Servo, tilt_servo: Servo) -> None:
        """Test creating pan-tilt with provided servos."""
        pt = create_pan_tilt(
            "test",
            pan_actuator=pan_servo,
            tilt_actuator=tilt_servo,
        )
        assert pt.pan_actuator is pan_servo
        assert pt.tilt_actuator is tilt_servo


# =============================================================================
# Tool Generation Tests
# =============================================================================


class TestPanTiltTools:
    """Tests for PanTilt tool generation."""

    def test_as_tools_returns_list(self, enabled_pan_tilt: PanTilt) -> None:
        """Test that as_tools returns a list of callables."""
        tools = enabled_pan_tilt.as_tools()
        assert isinstance(tools, list)
        assert len(tools) >= 6
        for tool in tools:
            assert callable(tool)

    def test_look_at_tool(self, enabled_pan_tilt: PanTilt) -> None:
        """Test look_at tool."""
        tools = enabled_pan_tilt.as_tools()
        look_at_tool = tools[0]
        result = look_at_tool(pan=45.0, tilt=30.0)
        assert "45" in result
        assert "30" in result
        assert enabled_pan_tilt.pan_angle == 45.0
        assert enabled_pan_tilt.tilt_angle == 30.0

    def test_center_tool(self, enabled_pan_tilt: PanTilt) -> None:
        """Test center tool."""
        tools = enabled_pan_tilt.as_tools()
        enabled_pan_tilt.look_at(pan=45.0, tilt=30.0)
        center_tool = tools[1]
        result = center_tool()
        assert "centered" in result.lower()
        assert enabled_pan_tilt.pan_angle == 0.0
        assert enabled_pan_tilt.tilt_angle == 0.0

    def test_pan_to_tool(self, enabled_pan_tilt: PanTilt) -> None:
        """Test pan_to tool."""
        tools = enabled_pan_tilt.as_tools()
        pan_tool = tools[2]
        result = pan_tool(angle=60.0)
        assert "60" in result
        assert enabled_pan_tilt.pan_angle == 60.0

    def test_tilt_to_tool(self, enabled_pan_tilt: PanTilt) -> None:
        """Test tilt_to tool."""
        tools = enabled_pan_tilt.as_tools()
        tilt_tool = tools[3]
        result = tilt_tool(angle=30.0)
        assert "30" in result
        assert enabled_pan_tilt.tilt_angle == 30.0

    def test_track_tool(self, enabled_pan_tilt: PanTilt) -> None:
        """Test track tool."""
        tools = enabled_pan_tilt.as_tools()
        track_tool = tools[4]
        result = track_tool(x=400.0, y=240.0)
        assert "track" in result.lower()
        assert enabled_pan_tilt.pan_angle != 0.0  # Should have moved

    def test_status_tool(self, enabled_pan_tilt: PanTilt) -> None:
        """Test get_pan_tilt_status tool."""
        tools = enabled_pan_tilt.as_tools()
        status_tool = tools[5]
        result = status_tool()
        assert isinstance(result, dict)
        assert "state" in result
        assert "pan_angle" in result
        assert "tilt_angle" in result


# =============================================================================
# Inversion Tests
# =============================================================================


class TestPanTiltInversion:
    """Tests for pan/tilt inversion."""

    def test_inverted_pan(self, pan_servo: Servo, tilt_servo: Servo) -> None:
        """Test inverted pan direction."""
        config = PanTiltConfig(
            name="inverted",
            invert_pan=True,
        )
        pt = PanTilt(
            name="inverted",
            pan_actuator=pan_servo,
            tilt_actuator=tilt_servo,
            config=config,
        )
        pt.enable()
        pt.pan_to(45.0)
        # The pan angle should still report the requested angle (user-facing)
        assert pt.pan_angle == 45.0
        # But internal servo would be inverted (we just verify it worked)
        assert pt.pt_config.invert_pan is True

    def test_inverted_tilt(self, pan_servo: Servo, tilt_servo: Servo) -> None:
        """Test inverted tilt direction."""
        config = PanTiltConfig(
            name="inverted",
            invert_tilt=True,
        )
        pt = PanTilt(
            name="inverted",
            pan_actuator=pan_servo,
            tilt_actuator=tilt_servo,
            config=config,
        )
        pt.enable()
        pt.tilt_to(30.0)
        # The tilt angle should still report the requested angle (user-facing)
        assert pt.tilt_angle == 30.0
        # Verify inversion is configured
        assert pt.pt_config.invert_tilt is True


# =============================================================================
# Edge Case Tests
# =============================================================================


class TestPanTiltEdgeCases:
    """Tests for pan-tilt edge cases."""

    def test_look_at_at_limits(self, enabled_pan_tilt: PanTilt) -> None:
        """Test look_at at exact limits."""
        enabled_pan_tilt.look_at(pan=90.0, tilt=45.0)
        assert enabled_pan_tilt.pan_angle == 90.0
        assert enabled_pan_tilt.tilt_angle == 45.0

        enabled_pan_tilt.look_at(pan=-90.0, tilt=-45.0)
        assert enabled_pan_tilt.pan_angle == -90.0
        assert enabled_pan_tilt.tilt_angle == -45.0

    def test_rapid_direction_changes(self, enabled_pan_tilt: PanTilt) -> None:
        """Test rapid direction changes."""
        for i in range(5):
            enabled_pan_tilt.pan_to(45.0 if i % 2 == 0 else -45.0)
        # Should end at 45.0 (last call is i=4, even)
        assert enabled_pan_tilt.pan_angle == 45.0

    def test_zero_angles(self, enabled_pan_tilt: PanTilt) -> None:
        """Test moving to zero angles."""
        enabled_pan_tilt.look_at(pan=45.0, tilt=30.0)
        enabled_pan_tilt.look_at(pan=0.0, tilt=0.0)
        assert enabled_pan_tilt.pan_angle == 0.0
        assert enabled_pan_tilt.tilt_angle == 0.0
