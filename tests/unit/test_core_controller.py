"""Tests for robo_infra.core.controller module."""

from __future__ import annotations

import pytest

from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.controller import (
    ControllerConfig,
    ControllerGroup,
    ControllerMode,
    ControllerState,
    MotionConfig,
    Position,
    SimulatedController,
    create_controller,
)
from robo_infra.core.exceptions import DisabledError, SafetyError
from robo_infra.core.sensor import SimulatedSensor
from robo_infra.core.types import Limits


# =============================================================================
# Test Enums
# =============================================================================


class TestControllerState:
    """Tests for ControllerState enum."""

    def test_all_states_defined(self) -> None:
        """All expected states should be defined."""
        assert ControllerState.DISABLED.value == "disabled"
        assert ControllerState.IDLE.value == "idle"
        assert ControllerState.HOMING.value == "homing"
        assert ControllerState.MOVING.value == "moving"
        assert ControllerState.RUNNING.value == "running"
        assert ControllerState.STOPPED.value == "stopped"
        assert ControllerState.ERROR.value == "error"

    def test_state_count(self) -> None:
        """Should have expected number of states."""
        assert len(ControllerState) == 7


class TestControllerMode:
    """Tests for ControllerMode enum."""

    def test_all_modes_defined(self) -> None:
        """All expected modes should be defined."""
        assert ControllerMode.MANUAL.value == "manual"
        assert ControllerMode.AUTOMATIC.value == "automatic"
        assert ControllerMode.REMOTE.value == "remote"
        assert ControllerMode.TEACH.value == "teach"

    def test_mode_count(self) -> None:
        """Should have expected number of modes."""
        assert len(ControllerMode) == 4


# =============================================================================
# Test ControllerConfig
# =============================================================================


class TestControllerConfig:
    """Tests for ControllerConfig Pydantic model."""

    def test_minimal_config(self) -> None:
        """Should create config with just name."""
        config = ControllerConfig(name="test")
        assert config.name == "test"
        assert config.mode == ControllerMode.MANUAL
        assert config.home_on_enable is False
        assert config.disable_on_error is True
        assert config.default_speed == 0.5

    def test_full_config(self) -> None:
        """Should create config with all options."""
        config = ControllerConfig(
            name="robot_arm",
            description="6-DOF Arm",
            mode=ControllerMode.AUTOMATIC,
            home_on_enable=True,
            disable_on_error=False,
            enable_safety_limits=True,
            default_speed=0.8,
            default_acceleration=0.6,
            loop_rate=200.0,
            watchdog_timeout=2.0,
            metadata={"version": "1.0"},
        )
        assert config.name == "robot_arm"
        assert config.description == "6-DOF Arm"
        assert config.mode == ControllerMode.AUTOMATIC
        assert config.home_on_enable is True
        assert config.disable_on_error is False
        assert config.default_speed == 0.8
        assert config.loop_rate == 200.0

    def test_from_dict(self) -> None:
        """Should create from dictionary."""
        data = {
            "name": "arm",
            "mode": "automatic",
            "default_speed": 0.7,
        }
        config = ControllerConfig.from_dict(data)
        assert config.name == "arm"
        assert config.mode == ControllerMode.AUTOMATIC
        assert config.default_speed == 0.7

    def test_validation_speed_range(self) -> None:
        """Should validate speed in range."""
        with pytest.raises(ValueError):
            ControllerConfig(name="test", default_speed=1.5)

    def test_validation_loop_rate_positive(self) -> None:
        """Should require positive loop rate."""
        with pytest.raises(ValueError):
            ControllerConfig(name="test", loop_rate=0)


# =============================================================================
# Test Position and MotionConfig
# =============================================================================


class TestPosition:
    """Tests for Position dataclass."""

    def test_create_position(self) -> None:
        """Should create position with values."""
        pos = Position(
            name="home",
            values={"joint1": 0.0, "joint2": 90.0},
            metadata={"description": "Home position"},
        )
        assert pos.name == "home"
        assert pos.values["joint1"] == 0.0
        assert pos.values["joint2"] == 90.0
        assert pos.metadata["description"] == "Home position"

    def test_repr(self) -> None:
        """Should have useful repr."""
        pos = Position(name="test", values={"a": 1.0})
        r = repr(pos)
        assert "test" in r
        assert "values" in r


class TestMotionConfig:
    """Tests for MotionConfig dataclass."""

    def test_defaults(self) -> None:
        """Should have sensible defaults."""
        config = MotionConfig()
        assert config.speed == 1.0
        assert config.acceleration == 1.0
        assert config.interpolation == "linear"
        assert config.timeout == 30.0
        assert config.blocking is True

    def test_custom_values(self) -> None:
        """Should accept custom values."""
        config = MotionConfig(
            speed=0.5,
            acceleration=0.3,
            interpolation="cubic",
            timeout=10.0,
            blocking=False,
        )
        assert config.speed == 0.5
        assert config.interpolation == "cubic"
        assert config.blocking is False


# =============================================================================
# Test SimulatedController
# =============================================================================


class TestSimulatedController:
    """Tests for SimulatedController."""

    def test_create_basic(self) -> None:
        """Should create with minimal args."""
        controller = SimulatedController(name="test")
        assert controller.name == "test"
        assert controller.state == ControllerState.DISABLED
        assert controller.is_enabled is False
        assert controller.is_homed is False

    def test_create_with_config(self) -> None:
        """Should create with config."""
        config = ControllerConfig(name="arm", default_speed=0.8)
        controller = SimulatedController(name="arm", config=config)
        assert controller.config.default_speed == 0.8

    def test_enable_disable(self) -> None:
        """Should enable and disable."""
        controller = SimulatedController(name="test")
        assert controller.is_enabled is False

        controller.enable()
        assert controller.is_enabled is True
        assert controller.state == ControllerState.IDLE

        controller.disable()
        assert controller.is_enabled is False
        assert controller.state == ControllerState.DISABLED

    def test_add_remove_actuator(self) -> None:
        """Should add and remove actuators."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(name="joint", limits=Limits(min=0, max=180))

        controller.add_actuator("joint1", actuator)
        assert "joint1" in controller.actuators
        assert controller.get_actuator("joint1") is actuator

        removed = controller.remove_actuator("joint1")
        assert removed is actuator
        assert "joint1" not in controller.actuators

    def test_add_remove_sensor(self) -> None:
        """Should add and remove sensors."""
        controller = SimulatedController(name="test")
        sensor = SimulatedSensor(name="encoder", limits=Limits(min=0, max=360))

        controller.add_sensor("enc1", sensor)
        assert "enc1" in controller.sensors
        assert controller.get_sensor("enc1") is sensor

        removed = controller.remove_sensor("enc1")
        assert removed is sensor
        assert "enc1" not in controller.sensors

    def test_enable_enables_components(self) -> None:
        """Should enable all actuators and sensors on enable."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(name="joint", limits=Limits(min=0, max=180))
        sensor = SimulatedSensor(name="enc", limits=Limits(min=0, max=360))

        controller.add_actuator("joint", actuator)
        controller.add_sensor("enc", sensor)

        assert actuator.is_enabled is False
        assert sensor.is_enabled is False

        controller.enable()

        assert actuator.is_enabled is True
        assert sensor.is_enabled is True

    def test_disable_disables_components(self) -> None:
        """Should disable all components on disable."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(name="joint", limits=Limits(min=0, max=180))
        controller.add_actuator("joint", actuator)

        controller.enable()
        assert actuator.is_enabled is True

        controller.disable()
        assert actuator.is_enabled is False

    def test_home(self) -> None:
        """Should home controller."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(
            name="joint",
            limits=Limits(min=0, max=180, default=90),
        )
        controller.add_actuator("joint", actuator)

        controller.enable()
        actuator.set(45.0)
        assert actuator.get() == 45.0

        controller.home()

        assert controller.is_homed is True
        assert actuator.get() == 90.0  # Default position

    def test_home_when_disabled(self) -> None:
        """Should raise DisabledError when homing disabled controller."""
        controller = SimulatedController(name="test")
        with pytest.raises(DisabledError):
            controller.home()

    def test_stop(self) -> None:
        """Should emergency stop."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(name="joint", limits=Limits(min=0, max=180))
        controller.add_actuator("joint", actuator)

        controller.enable()
        assert actuator.is_enabled is True

        controller.stop()

        assert controller.state == ControllerState.STOPPED
        assert actuator.is_enabled is False

    def test_reset_stop(self) -> None:
        """Should reset from stop state."""
        controller = SimulatedController(name="test")
        controller.enable()
        controller.stop()

        assert controller.state == ControllerState.STOPPED

        controller.reset_stop()

        assert controller.state == ControllerState.IDLE
        assert controller.is_homed is False  # Requires rehoming

    def test_context_manager(self) -> None:
        """Should work as context manager."""
        controller = SimulatedController(name="test")

        with controller:
            assert controller.is_enabled is True

        assert controller.is_enabled is False


# =============================================================================
# Test Motion
# =============================================================================


class TestControllerMotion:
    """Tests for controller motion commands."""

    def test_move_to(self) -> None:
        """Should move actuators to targets."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(
            name="joint",
            limits=Limits(min=0, max=180, default=90),
        )
        controller.add_actuator("joint", actuator)
        controller.enable()

        controller.move_to({"joint": 45.0})

        assert actuator.get() == 45.0

    def test_move_to_multiple(self) -> None:
        """Should move multiple actuators."""
        controller = SimulatedController(name="test")
        j1 = SimulatedActuator(name="j1", limits=Limits(min=0, max=180))
        j2 = SimulatedActuator(name="j2", limits=Limits(min=0, max=180))
        controller.add_actuator("joint1", j1)
        controller.add_actuator("joint2", j2)
        controller.enable()

        controller.move_to({"joint1": 30.0, "joint2": 60.0})

        assert j1.get() == 30.0
        assert j2.get() == 60.0

    def test_move_to_when_disabled(self) -> None:
        """Should raise DisabledError when disabled."""
        controller = SimulatedController(name="test")
        with pytest.raises(DisabledError):
            controller.move_to({"joint": 45.0})

    def test_move_to_when_stopped(self) -> None:
        """Should raise SafetyError when in stop state."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(name="j", limits=Limits(min=0, max=180))
        controller.add_actuator("joint", actuator)
        controller.enable()
        controller.stop()

        with pytest.raises(SafetyError):
            controller.move_to({"joint": 45.0})

    def test_move_to_unknown_actuator(self) -> None:
        """Should raise ValueError for unknown actuator."""
        controller = SimulatedController(name="test")
        controller.enable()

        with pytest.raises(ValueError, match="Unknown actuator"):
            controller.move_to({"nonexistent": 45.0})

    def test_move_to_position(self) -> None:
        """Should move to named position."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(name="j", limits=Limits(min=0, max=180))
        controller.add_actuator("joint", actuator)
        controller.enable()

        # Save a position
        controller.add_position(Position(name="up", values={"joint": 90.0}))

        controller.move_to_position("up")
        assert actuator.get() == 90.0

    def test_move_to_position_not_found(self) -> None:
        """Should raise KeyError for unknown position."""
        controller = SimulatedController(name="test")
        controller.enable()

        with pytest.raises(KeyError):
            controller.move_to_position("nonexistent")


# =============================================================================
# Test Position Management
# =============================================================================


class TestPositionManagement:
    """Tests for position save/load functionality."""

    def test_save_position(self) -> None:
        """Should save current position."""
        controller = SimulatedController(name="test")
        j1 = SimulatedActuator(name="j1", limits=Limits(min=0, max=180, default=0))
        j2 = SimulatedActuator(name="j2", limits=Limits(min=0, max=180, default=0))
        controller.add_actuator("joint1", j1)
        controller.add_actuator("joint2", j2)
        controller.enable()

        j1.set(30.0)
        j2.set(60.0)

        pos = controller.save_position("waypoint1", metadata={"note": "test"})

        assert pos.name == "waypoint1"
        assert pos.values["joint1"] == 30.0
        assert pos.values["joint2"] == 60.0
        assert pos.metadata["note"] == "test"
        assert "waypoint1" in controller.positions

    def test_delete_position(self) -> None:
        """Should delete position."""
        controller = SimulatedController(name="test")
        controller.add_position(Position(name="test", values={}))

        assert "test" in controller.positions

        result = controller.delete_position("test")
        assert result is True
        assert "test" not in controller.positions

    def test_delete_position_not_found(self) -> None:
        """Should return False for missing position."""
        controller = SimulatedController(name="test")
        result = controller.delete_position("nonexistent")
        assert result is False


# =============================================================================
# Test Status
# =============================================================================


class TestControllerStatus:
    """Tests for controller status."""

    def test_status_disabled(self) -> None:
        """Should return disabled status."""
        controller = SimulatedController(name="test")
        status = controller.status()

        assert status.state == ControllerState.DISABLED
        assert status.is_enabled is False
        assert status.is_homed is False
        assert status.uptime == 0.0

    def test_status_enabled(self) -> None:
        """Should return enabled status."""
        controller = SimulatedController(name="test")
        actuator = SimulatedActuator(name="j", limits=Limits(min=0, max=180))
        sensor = SimulatedSensor(name="s", limits=Limits(min=0, max=360))
        controller.add_actuator("joint", actuator)
        controller.add_sensor("sensor", sensor)

        controller.enable()
        status = controller.status()

        assert status.state == ControllerState.IDLE
        assert status.is_enabled is True
        assert status.actuator_count == 1
        assert status.sensor_count == 1
        assert status.uptime > 0

    def test_get_actuator_statuses(self) -> None:
        """Should get all actuator statuses."""
        controller = SimulatedController(name="test")
        j1 = SimulatedActuator(name="j1", limits=Limits(min=0, max=180))
        j2 = SimulatedActuator(name="j2", limits=Limits(min=0, max=180))
        controller.add_actuator("joint1", j1)
        controller.add_actuator("joint2", j2)
        controller.enable()

        statuses = controller.get_actuator_statuses()

        assert "joint1" in statuses
        assert "joint2" in statuses
        assert statuses["joint1"].is_enabled is True

    def test_get_sensor_statuses(self) -> None:
        """Should get all sensor statuses."""
        controller = SimulatedController(name="test")
        s1 = SimulatedSensor(name="s1", limits=Limits(min=0, max=360))
        controller.add_sensor("sensor1", s1)
        controller.enable()

        statuses = controller.get_sensor_statuses()

        assert "sensor1" in statuses
        assert statuses["sensor1"].is_enabled is True

    def test_get_actuator_values(self) -> None:
        """Should get all actuator values."""
        controller = SimulatedController(name="test")
        j1 = SimulatedActuator(name="j1", limits=Limits(min=0, max=180, default=45))
        j2 = SimulatedActuator(name="j2", limits=Limits(min=0, max=180, default=90))
        controller.add_actuator("joint1", j1)
        controller.add_actuator("joint2", j2)
        controller.enable()

        values = controller.get_actuator_values()

        assert values["joint1"] == 45.0
        assert values["joint2"] == 90.0

    def test_read_sensors(self) -> None:
        """Should read all sensor values."""
        controller = SimulatedController(name="test")
        s1 = SimulatedSensor(name="s1", limits=Limits(min=0, max=100, default=50))
        s2 = SimulatedSensor(name="s2", limits=Limits(min=0, max=100, default=75))
        controller.add_sensor("sensor1", s1)
        controller.add_sensor("sensor2", s2)
        controller.enable()

        values = controller.read_sensors()

        assert values["sensor1"] == 50.0
        assert values["sensor2"] == 75.0


# =============================================================================
# Test Callbacks
# =============================================================================


class TestControllerCallbacks:
    """Tests for controller callbacks."""

    def test_on_state_change(self) -> None:
        """Should call state change callbacks."""
        controller = SimulatedController(name="test")
        states: list[ControllerState] = []

        controller.on_state_change(lambda s: states.append(s))

        controller.enable()
        controller.disable()

        assert ControllerState.IDLE in states
        assert ControllerState.DISABLED in states

    def test_on_error(self) -> None:
        """Should call error callbacks."""
        controller = SimulatedController(name="test")
        errors: list[str] = []

        controller.on_error(lambda e: errors.append(e))

        # Trigger an error by trying to home when disabled
        # (This won't call the callback directly since exception is raised)
        # Let's test via internal error handling
        controller.enable()
        controller._handle_error("Test error")

        assert len(errors) == 1
        assert "Test error" in errors[0]


# =============================================================================
# Test Control Loop
# =============================================================================


class TestControlLoop:
    """Tests for async control loop."""

    @pytest.mark.asyncio
    async def test_run_loop(self) -> None:
        """Should run control loop."""
        controller = SimulatedController(name="test")
        controller._config.loop_rate = 100.0  # Fast rate for test
        controller.enable()

        # Track loop iterations
        iterations = [0]

        original_step = controller._control_loop_step

        def counting_step():
            iterations[0] += 1
            if iterations[0] >= 5:
                controller.stop_loop()
            original_step()

        controller._control_loop_step = counting_step

        await controller.run()

        assert iterations[0] >= 5
        assert controller.is_running is False

    @pytest.mark.asyncio
    async def test_run_when_disabled(self) -> None:
        """Should raise DisabledError."""
        controller = SimulatedController(name="test")

        with pytest.raises(DisabledError):
            await controller.run()


# =============================================================================
# Test Integration Hooks
# =============================================================================


class TestIntegrationHooks:
    """Tests for ai-infra and svc-infra integration hooks."""

    def test_as_tools(self) -> None:
        """Should generate tool definitions."""
        controller = SimulatedController(name="arm")
        j1 = SimulatedActuator(name="j1", limits=Limits(min=0, max=180))
        controller.add_actuator("joint1", j1)

        tools = controller.as_tools()

        assert len(tools) >= 4  # move, home, stop, status
        tool_names = [t["name"] for t in tools]
        assert "arm_move" in tool_names
        assert "arm_home" in tool_names
        assert "arm_stop" in tool_names
        assert "arm_status" in tool_names

    def test_as_router(self) -> None:
        """Should generate router definition."""
        controller = SimulatedController(name="arm")

        router = controller.as_router()

        assert router["prefix"] == "/arm"
        assert "arm" in router["tags"]
        assert len(router["endpoints"]) >= 8


# =============================================================================
# Test ControllerGroup
# =============================================================================


class TestControllerGroup:
    """Tests for ControllerGroup."""

    def test_create_group(self) -> None:
        """Should create empty group."""
        group = ControllerGroup("robot")
        assert group.name == "robot"
        assert len(group) == 0

    def test_add_remove(self) -> None:
        """Should add and remove controllers."""
        group = ControllerGroup("robot")
        c1 = SimulatedController(name="arm")
        c2 = SimulatedController(name="gripper")

        group.add("arm", c1)
        group.add("gripper", c2)

        assert len(group) == 2
        assert "arm" in group
        assert group.get("arm") is c1

        removed = group.remove("arm")
        assert removed is c1
        assert len(group) == 1

    def test_enable_disable_all(self) -> None:
        """Should enable/disable all controllers."""
        group = ControllerGroup("robot")
        c1 = SimulatedController(name="arm")
        c2 = SimulatedController(name="gripper")
        group.add("arm", c1)
        group.add("gripper", c2)

        group.enable_all()
        assert c1.is_enabled is True
        assert c2.is_enabled is True

        group.disable_all()
        assert c1.is_enabled is False
        assert c2.is_enabled is False

    def test_home_all(self) -> None:
        """Should home all enabled controllers."""
        group = ControllerGroup("robot")
        c1 = SimulatedController(name="arm")
        c2 = SimulatedController(name="gripper")
        a1 = SimulatedActuator(name="j", limits=Limits(min=0, max=180, default=90))
        c1.add_actuator("joint", a1)
        group.add("arm", c1)
        group.add("gripper", c2)

        group.enable_all()
        a1.set(45.0)

        group.home_all()

        assert c1.is_homed is True
        assert a1.get() == 90.0

    def test_stop_all(self) -> None:
        """Should emergency stop all."""
        group = ControllerGroup("robot")
        c1 = SimulatedController(name="arm")
        c2 = SimulatedController(name="gripper")
        group.add("arm", c1)
        group.add("gripper", c2)

        group.enable_all()
        group.stop_all()

        assert c1.state == ControllerState.STOPPED
        assert c2.state == ControllerState.STOPPED

    def test_status_all(self) -> None:
        """Should get status of all controllers."""
        group = ControllerGroup("robot")
        c1 = SimulatedController(name="arm")
        c2 = SimulatedController(name="gripper")
        group.add("arm", c1)
        group.add("gripper", c2)

        c1.enable()

        statuses = group.status_all()

        assert statuses["arm"].is_enabled is True
        assert statuses["gripper"].is_enabled is False

    def test_context_manager(self) -> None:
        """Should work as context manager."""
        group = ControllerGroup("robot")
        c1 = SimulatedController(name="arm")
        group.add("arm", c1)

        with group:
            assert c1.is_enabled is True

        assert c1.is_enabled is False

    def test_iteration(self) -> None:
        """Should iterate over names."""
        group = ControllerGroup("robot")
        group.add("a", SimulatedController(name="a"))
        group.add("b", SimulatedController(name="b"))

        names = list(group)
        assert "a" in names
        assert "b" in names


# =============================================================================
# Test Factory Function
# =============================================================================


class TestFactoryFunction:
    """Tests for create_controller factory."""

    def test_create_basic(self) -> None:
        """Should create basic controller."""
        controller = create_controller("test")
        assert controller.name == "test"
        assert len(controller.actuators) == 0
        assert len(controller.sensors) == 0

    def test_create_with_components(self) -> None:
        """Should create with actuators and sensors."""
        actuators = {
            "joint1": SimulatedActuator(name="j1", limits=Limits(min=0, max=180)),
            "joint2": SimulatedActuator(name="j2", limits=Limits(min=0, max=180)),
        }
        sensors = {
            "enc1": SimulatedSensor(name="e1", limits=Limits(min=0, max=360)),
        }

        controller = create_controller(
            "arm",
            actuators=actuators,
            sensors=sensors,
        )

        assert "joint1" in controller.actuators
        assert "joint2" in controller.actuators
        assert "enc1" in controller.sensors

    def test_create_with_config(self) -> None:
        """Should create with config."""
        config = ControllerConfig(name="arm", default_speed=0.9)
        controller = create_controller("arm", config=config)

        assert controller.config.default_speed == 0.9


# =============================================================================
# Test Repr
# =============================================================================


class TestControllerRepr:
    """Tests for string representation."""

    def test_repr(self) -> None:
        """Should have useful repr."""
        controller = SimulatedController(name="arm")
        j1 = SimulatedActuator(name="j", limits=Limits(min=0, max=180))
        s1 = SimulatedSensor(name="s", limits=Limits(min=0, max=360))
        controller.add_actuator("joint", j1)
        controller.add_sensor("sensor", s1)

        r = repr(controller)

        assert "SimulatedController" in r
        assert "arm" in r
        assert "actuators=1" in r
        assert "sensors=1" in r
