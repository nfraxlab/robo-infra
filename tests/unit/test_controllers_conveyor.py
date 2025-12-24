"""Tests for robo_infra.controllers.conveyor module."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from robo_infra.actuators.dc_motor import DCMotor
from robo_infra.controllers.conveyor import (
    Conveyor,
    ConveyorConfig,
    ConveyorDirection,
    ConveyorState,
    ConveyorStatus,
    conveyor_status,
    create_conveyor,
)
from robo_infra.core.exceptions import DisabledError
from robo_infra.core.sensor import Sensor


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def conveyor_motor() -> DCMotor:
    """Create a simulated conveyor motor."""
    return DCMotor(
        pin_a=0,
        pin_b=1,
        enable=2,
        name="conveyor_motor",
    )


@pytest.fixture
def encoder_sensor() -> MagicMock:
    """Create a mock encoder sensor."""
    sensor = MagicMock(spec=Sensor)
    sensor.name = "encoder"
    sensor.read.return_value = MagicMock(value=0.0)
    return sensor


@pytest.fixture
def conveyor_config() -> ConveyorConfig:
    """Create default conveyor configuration."""
    return ConveyorConfig(
        name="test_conveyor",
        belt_length=2.0,
        speed_max=1.0,
        default_speed=0.5,
        index_distance=0.1,
    )


@pytest.fixture
def conveyor(conveyor_motor: DCMotor, conveyor_config: ConveyorConfig) -> Conveyor:
    """Create a Conveyor controller with default config."""
    return Conveyor(
        name="test_conveyor",
        motor=conveyor_motor,
        config=conveyor_config,
    )


@pytest.fixture
def enabled_conveyor(conveyor: Conveyor) -> Conveyor:
    """Create and enable a Conveyor controller."""
    conveyor.enable()
    return conveyor


@pytest.fixture
def conveyor_with_encoder(
    conveyor_motor: DCMotor,
    conveyor_config: ConveyorConfig,
    encoder_sensor: MagicMock,
) -> Conveyor:
    """Create a Conveyor controller with encoder."""
    return Conveyor(
        name="test_conveyor",
        motor=conveyor_motor,
        config=conveyor_config,
        encoder=encoder_sensor,
    )


# =============================================================================
# Conveyor Configuration Tests
# =============================================================================


class TestConveyorConfig:
    """Tests for ConveyorConfig model."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = ConveyorConfig(name="test")
        assert config.name == "test"
        assert config.belt_length == 1.0
        assert config.speed_max == 1.0
        assert config.default_speed == 0.5
        assert config.index_distance == 0.1
        assert config.soft_start is True

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = ConveyorConfig(
            name="custom",
            belt_length=5.0,
            speed_max=2.0,
            default_speed=1.0,
            index_distance=0.25,
            soft_start=False,
        )
        assert config.belt_length == 5.0
        assert config.speed_max == 2.0
        assert config.default_speed == 1.0
        assert config.index_distance == 0.25
        assert config.soft_start is False

    def test_config_validation(self) -> None:
        """Test configuration validation."""
        with pytest.raises(ValueError):
            ConveyorConfig(name="test", belt_length=-1.0)
        with pytest.raises(ValueError):
            ConveyorConfig(name="test", speed_max=0)


# =============================================================================
# Conveyor Initialization Tests
# =============================================================================


class TestConveyorInit:
    """Tests for Conveyor initialization."""

    def test_create_conveyor(self, conveyor: Conveyor) -> None:
        """Test basic conveyor creation."""
        assert conveyor.name == "test_conveyor"
        assert conveyor.conveyor_state == ConveyorState.DISABLED
        assert conveyor.current_speed == 0.0
        assert conveyor.distance_traveled == 0.0

    def test_motor_required(self) -> None:
        """Test that motor is required."""
        with pytest.raises(ValueError, match="motor is required"):
            Conveyor(name="test", motor=None)  # type: ignore[arg-type]

    def test_default_config_creation(self, conveyor_motor: DCMotor) -> None:
        """Test conveyor with no config uses defaults."""
        conveyor = Conveyor(name="test", motor=conveyor_motor)
        assert conveyor.conveyor_config is not None
        assert conveyor.conveyor_config.name == "test"

    def test_conveyor_with_encoder(
        self, conveyor_with_encoder: Conveyor, encoder_sensor: MagicMock
    ) -> None:
        """Test conveyor with encoder sensor."""
        assert conveyor_with_encoder.encoder is not None
        assert conveyor_with_encoder.encoder.name == "encoder"


# =============================================================================
# Conveyor Lifecycle Tests
# =============================================================================


class TestConveyorLifecycle:
    """Tests for Conveyor enable/disable lifecycle."""

    def test_enable(self, conveyor: Conveyor) -> None:
        """Test enabling the conveyor."""
        assert conveyor.conveyor_state == ConveyorState.DISABLED
        conveyor.enable()
        assert conveyor.conveyor_state == ConveyorState.IDLE
        assert conveyor._is_enabled is True

    def test_disable(self, enabled_conveyor: Conveyor) -> None:
        """Test disabling the conveyor."""
        enabled_conveyor.disable()
        assert enabled_conveyor.conveyor_state == ConveyorState.DISABLED
        assert enabled_conveyor._is_enabled is False

    def test_home_resets_distance(self, enabled_conveyor: Conveyor) -> None:
        """Test home resets distance counter."""
        enabled_conveyor._distance_traveled = 1.0
        enabled_conveyor.home()
        assert enabled_conveyor.distance_traveled == 0.0
        assert enabled_conveyor._is_homed is True


# =============================================================================
# Conveyor Run Tests
# =============================================================================


class TestConveyorRun:
    """Tests for Conveyor run operation."""

    def test_run_forward(self, enabled_conveyor: Conveyor) -> None:
        """Test running conveyor forward."""
        enabled_conveyor.run(speed=0.5)
        assert enabled_conveyor.conveyor_state == ConveyorState.RUNNING
        assert enabled_conveyor.current_speed == 0.5
        assert enabled_conveyor.direction == ConveyorDirection.FORWARD
        assert enabled_conveyor.is_running is True

    def test_run_reverse(self, enabled_conveyor: Conveyor) -> None:
        """Test running conveyor in reverse."""
        enabled_conveyor.run(speed=0.5, reverse=True)
        assert enabled_conveyor.direction == ConveyorDirection.REVERSE
        assert enabled_conveyor.is_running is True

    def test_run_default_speed(self, enabled_conveyor: Conveyor) -> None:
        """Test running with default speed."""
        enabled_conveyor.run()
        assert enabled_conveyor.current_speed == 0.5  # default_speed from config

    def test_run_exceeds_max_speed(self, enabled_conveyor: Conveyor) -> None:
        """Test that exceeding max speed raises error."""
        with pytest.raises(ValueError, match="exceeds maximum"):
            enabled_conveyor.run(speed=2.0)

    def test_run_requires_enabled(self, conveyor: Conveyor) -> None:
        """Test that run requires enabled conveyor."""
        with pytest.raises(DisabledError):
            conveyor.run(speed=0.5)


# =============================================================================
# Conveyor Stop Tests
# =============================================================================


class TestConveyorStop:
    """Tests for Conveyor stop operation."""

    def test_stop_running_conveyor(self, enabled_conveyor: Conveyor) -> None:
        """Test stopping a running conveyor."""
        enabled_conveyor.run(speed=0.5)
        enabled_conveyor.stop()
        assert enabled_conveyor.conveyor_state == ConveyorState.IDLE
        assert enabled_conveyor.current_speed == 0.0
        assert enabled_conveyor.is_running is False

    def test_stop_disabled_conveyor(self, conveyor: Conveyor) -> None:
        """Test stopping a disabled conveyor does nothing."""
        conveyor.stop()  # Should not raise
        assert conveyor.conveyor_state == ConveyorState.DISABLED

    def test_emergency_stop(self, enabled_conveyor: Conveyor) -> None:
        """Test emergency stop."""
        enabled_conveyor.run(speed=0.5)
        enabled_conveyor.emergency_stop()
        assert enabled_conveyor.conveyor_state == ConveyorState.IDLE
        assert enabled_conveyor.is_running is False


# =============================================================================
# Conveyor Jog Tests
# =============================================================================


class TestConveyorJog:
    """Tests for Conveyor jog operation."""

    def test_jog_forward(self, enabled_conveyor: Conveyor) -> None:
        """Test jogging forward."""
        enabled_conveyor.jog(distance=0.5)
        assert enabled_conveyor.distance_traveled == 0.5
        assert enabled_conveyor.conveyor_state == ConveyorState.IDLE

    def test_jog_reverse(self, enabled_conveyor: Conveyor) -> None:
        """Test jogging in reverse."""
        enabled_conveyor.jog(distance=-0.5)
        assert enabled_conveyor.distance_traveled == -0.5
        assert enabled_conveyor.conveyor_state == ConveyorState.IDLE

    def test_jog_with_speed(self, enabled_conveyor: Conveyor) -> None:
        """Test jogging with specific speed."""
        enabled_conveyor.jog(distance=0.5, speed=0.3)
        assert enabled_conveyor.distance_traveled == 0.5

    def test_jog_exceeds_max_speed(self, enabled_conveyor: Conveyor) -> None:
        """Test that exceeding max speed raises error."""
        with pytest.raises(ValueError, match="exceeds maximum"):
            enabled_conveyor.jog(distance=0.5, speed=2.0)

    def test_jog_requires_enabled(self, conveyor: Conveyor) -> None:
        """Test that jog requires enabled conveyor."""
        with pytest.raises(DisabledError):
            conveyor.jog(distance=0.5)


# =============================================================================
# Conveyor Index Tests
# =============================================================================


class TestConveyorIndex:
    """Tests for Conveyor index operation."""

    def test_index_single(self, enabled_conveyor: Conveyor) -> None:
        """Test indexing by one increment."""
        enabled_conveyor.index(count=1)
        assert enabled_conveyor.distance_traveled == pytest.approx(0.1)

    def test_index_multiple(self, enabled_conveyor: Conveyor) -> None:
        """Test indexing by multiple increments."""
        enabled_conveyor.index(count=5)
        assert enabled_conveyor.distance_traveled == pytest.approx(0.5)

    def test_index_reverse(self, enabled_conveyor: Conveyor) -> None:
        """Test indexing in reverse."""
        enabled_conveyor.index(count=-3)
        assert enabled_conveyor.distance_traveled == pytest.approx(-0.3)

    def test_index_with_speed(self, enabled_conveyor: Conveyor) -> None:
        """Test indexing with specific speed."""
        enabled_conveyor.index(count=2, speed=0.3)
        assert enabled_conveyor.distance_traveled == pytest.approx(0.2)

    def test_index_requires_enabled(self, conveyor: Conveyor) -> None:
        """Test that index requires enabled conveyor."""
        with pytest.raises(DisabledError):
            conveyor.index(count=1)


# =============================================================================
# Conveyor Speed Control Tests
# =============================================================================


class TestConveyorSpeedControl:
    """Tests for Conveyor speed control."""

    def test_set_speed(self, enabled_conveyor: Conveyor) -> None:
        """Test setting speed."""
        enabled_conveyor.run(speed=0.5)
        enabled_conveyor.set_speed(0.8)
        assert enabled_conveyor.target_speed == 0.8

    def test_set_speed_exceeds_max(self, enabled_conveyor: Conveyor) -> None:
        """Test that exceeding max speed raises error."""
        with pytest.raises(ValueError, match="exceeds maximum"):
            enabled_conveyor.set_speed(2.0)

    def test_reverse_direction(self, enabled_conveyor: Conveyor) -> None:
        """Test reversing direction."""
        enabled_conveyor.run(speed=0.5)
        assert enabled_conveyor.direction == ConveyorDirection.FORWARD
        enabled_conveyor.reverse()
        assert enabled_conveyor.direction == ConveyorDirection.REVERSE
        enabled_conveyor.reverse()
        assert enabled_conveyor.direction == ConveyorDirection.FORWARD

    def test_reset_distance(self, enabled_conveyor: Conveyor) -> None:
        """Test resetting distance counter."""
        enabled_conveyor.jog(distance=1.0)
        enabled_conveyor.reset_distance()
        assert enabled_conveyor.distance_traveled == 0.0


# =============================================================================
# Conveyor Status Tests
# =============================================================================


class TestConveyorStatus:
    """Tests for Conveyor status."""

    def test_status_idle(self, enabled_conveyor: Conveyor) -> None:
        """Test status when idle."""
        status = enabled_conveyor.status()
        assert isinstance(status, ConveyorStatus)
        assert status.state == ConveyorState.IDLE
        assert status.current_speed == 0.0
        assert status.is_enabled is True

    def test_status_running(self, enabled_conveyor: Conveyor) -> None:
        """Test status when running."""
        enabled_conveyor.run(speed=0.5)
        status = enabled_conveyor.status()
        assert status.state == ConveyorState.RUNNING
        assert status.current_speed == 0.5
        assert status.direction == ConveyorDirection.FORWARD

    def test_conveyor_status_function(self, enabled_conveyor: Conveyor) -> None:
        """Test conveyor_status helper function."""
        enabled_conveyor.run(speed=0.5)
        status_dict = conveyor_status(enabled_conveyor)
        assert status_dict["name"] == "test_conveyor"
        assert status_dict["state"] == "running"
        assert status_dict["current_speed"] == 0.5


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreateConveyor:
    """Tests for create_conveyor factory function."""

    def test_create_default(self) -> None:
        """Test creating conveyor with defaults."""
        conveyor = create_conveyor("test")
        assert conveyor.name == "test"
        assert conveyor.motor is not None
        assert conveyor.conveyor_config.belt_length == 1.0

    def test_create_with_params(self) -> None:
        """Test creating conveyor with custom parameters."""
        conveyor = create_conveyor(
            "custom",
            belt_length=5.0,
            speed_max=2.0,
            index_distance=0.25,
        )
        assert conveyor.conveyor_config.belt_length == 5.0
        assert conveyor.conveyor_config.speed_max == 2.0
        assert conveyor.conveyor_config.index_distance == 0.25

    def test_create_with_motor(self, conveyor_motor: DCMotor) -> None:
        """Test creating conveyor with provided motor."""
        conveyor = create_conveyor("test", motor=conveyor_motor)
        assert conveyor.motor is conveyor_motor


# =============================================================================
# Tool Generation Tests
# =============================================================================


class TestConveyorTools:
    """Tests for Conveyor tool generation."""

    def test_as_tools_returns_list(self, enabled_conveyor: Conveyor) -> None:
        """Test that as_tools returns a list of callables."""
        tools = enabled_conveyor.as_tools()
        assert isinstance(tools, list)
        assert len(tools) >= 5
        for tool in tools:
            assert callable(tool)

    def test_run_tool(self, enabled_conveyor: Conveyor) -> None:
        """Test run_conveyor tool."""
        tools = enabled_conveyor.as_tools()
        run_tool = tools[0]
        result = run_tool(speed=0.5)
        assert "running" in result.lower()
        assert enabled_conveyor.is_running is True

    def test_stop_tool(self, enabled_conveyor: Conveyor) -> None:
        """Test stop_conveyor tool."""
        tools = enabled_conveyor.as_tools()
        enabled_conveyor.run(speed=0.5)
        stop_tool = tools[1]
        result = stop_tool()
        assert "stopped" in result.lower()
        assert enabled_conveyor.is_running is False

    def test_jog_tool(self, enabled_conveyor: Conveyor) -> None:
        """Test jog_conveyor tool."""
        tools = enabled_conveyor.as_tools()
        jog_tool = tools[2]
        result = jog_tool(distance=0.5)
        assert "jogged" in result.lower()

    def test_index_tool(self, enabled_conveyor: Conveyor) -> None:
        """Test index_conveyor tool."""
        tools = enabled_conveyor.as_tools()
        index_tool = tools[3]
        result = index_tool(count=3)
        assert "indexed" in result.lower()

    def test_status_tool(self, enabled_conveyor: Conveyor) -> None:
        """Test get_conveyor_status tool."""
        tools = enabled_conveyor.as_tools()
        status_tool = tools[4]
        result = status_tool()
        assert isinstance(result, dict)
        assert "state" in result
        assert "current_speed" in result


# =============================================================================
# Conveyor Direction Tests
# =============================================================================


class TestConveyorDirection:
    """Tests for ConveyorDirection enum."""

    def test_direction_values(self) -> None:
        """Test direction enum values."""
        assert ConveyorDirection.FORWARD.value == 1
        assert ConveyorDirection.REVERSE.value == -1
