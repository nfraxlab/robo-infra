"""Tests for robo_infra.core.actuator module."""

from __future__ import annotations

import pytest

from robo_infra.core.actuator import (
    Actuator,
    ActuatorConfig,
    ActuatorGroup,
    ActuatorState,
    ActuatorStatus,
    ActuatorType,
    SimulatedActuator,
    create_actuator,
    create_motor,
    create_servo,
)
from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import (
    DisabledError,
    LimitsExceededError,
    NotCalibratedError,
)
from robo_infra.core.types import Limits


class TestEnums:
    """Tests for actuator enums."""

    def test_actuator_state_values(self) -> None:
        """Test ActuatorState enum values."""
        assert ActuatorState.DISABLED.value == "disabled"
        assert ActuatorState.IDLE.value == "idle"
        assert ActuatorState.MOVING.value == "moving"
        assert ActuatorState.HOLDING.value == "holding"
        assert ActuatorState.ERROR.value == "error"
        assert ActuatorState.CALIBRATING.value == "calibrating"

    def test_actuator_type_values(self) -> None:
        """Test ActuatorType enum values."""
        assert ActuatorType.SERVO.value == "servo"
        assert ActuatorType.DC_MOTOR.value == "dc_motor"
        assert ActuatorType.STEPPER.value == "stepper"
        assert ActuatorType.LINEAR.value == "linear"
        assert ActuatorType.SOLENOID.value == "solenoid"
        assert ActuatorType.PNEUMATIC.value == "pneumatic"
        assert ActuatorType.HYDRAULIC.value == "hydraulic"
        assert ActuatorType.GENERIC.value == "generic"


class TestActuatorConfig:
    """Tests for ActuatorConfig model."""

    def test_default_config(self) -> None:
        """Test ActuatorConfig default values."""
        config = ActuatorConfig()
        assert config.name == "Actuator"
        assert config.actuator_type == ActuatorType.GENERIC
        assert config.channel == 0
        assert config.unit == "units"
        assert config.inverted is False
        assert config.offset == 0.0
        assert config.scale == 1.0
        assert config.speed_limit is None
        assert config.acceleration is None
        assert config.require_calibration is False
        assert config.metadata == {}

    def test_custom_config(self) -> None:
        """Test ActuatorConfig with custom values."""
        config = ActuatorConfig(
            name="servo1",
            actuator_type=ActuatorType.SERVO,
            channel=5,
            limits=Limits(0, 180, 90),
            unit="degrees",
            inverted=True,
            offset=5.0,
            scale=2.0,
            speed_limit=100.0,
            require_calibration=True,
            metadata={"model": "SG90"},
        )
        assert config.name == "servo1"
        assert config.actuator_type == ActuatorType.SERVO
        assert config.channel == 5
        assert config.limits.min == 0
        assert config.limits.max == 180
        assert config.unit == "degrees"
        assert config.inverted is True
        assert config.offset == 5.0
        assert config.scale == 2.0
        assert config.speed_limit == 100.0
        assert config.require_calibration is True
        assert config.metadata == {"model": "SG90"}


class TestActuatorStatus:
    """Tests for ActuatorStatus dataclass."""

    def test_default_status(self) -> None:
        """Test ActuatorStatus default values."""
        status = ActuatorStatus()
        assert status.state == ActuatorState.DISABLED
        assert status.position == 0.0
        assert status.target is None
        assert status.is_enabled is False
        assert status.is_calibrated is False
        assert status.error is None

    def test_custom_status(self) -> None:
        """Test ActuatorStatus with custom values."""
        status = ActuatorStatus(
            state=ActuatorState.HOLDING,
            position=45.0,
            target=45.0,
            is_enabled=True,
            is_calibrated=True,
            error=None,
        )
        assert status.state == ActuatorState.HOLDING
        assert status.position == 45.0
        assert status.target == 45.0
        assert status.is_enabled is True
        assert status.is_calibrated is True


class TestSimulatedActuator:
    """Tests for SimulatedActuator."""

    def test_create_actuator(self) -> None:
        """Test creating a simulated actuator."""
        actuator = SimulatedActuator()
        assert actuator.name == "SimulatedActuator"
        assert actuator.state == ActuatorState.DISABLED
        assert actuator.is_enabled is False

    def test_create_actuator_with_name(self) -> None:
        """Test creating actuator with custom name."""
        actuator = SimulatedActuator(name="servo1")
        assert actuator.name == "servo1"

    def test_create_actuator_with_limits(self) -> None:
        """Test creating actuator with limits."""
        limits = Limits(0, 180, 90)
        actuator = SimulatedActuator(name="servo", limits=limits, unit="degrees")
        assert actuator.limits.min == 0
        assert actuator.limits.max == 180
        assert actuator.limits.default == 90
        assert actuator.unit == "degrees"

    def test_create_actuator_with_config(self) -> None:
        """Test creating actuator with full config."""
        config = ActuatorConfig(
            name="configured",
            actuator_type=ActuatorType.SERVO,
            channel=3,
            limits=Limits(0, 180, 90),
        )
        actuator = SimulatedActuator(config=config)
        assert actuator.name == "configured"
        assert actuator.actuator_type == ActuatorType.SERVO
        assert actuator.channel == 3

    def test_enable_disable(self) -> None:
        """Test enabling and disabling actuator."""
        actuator = SimulatedActuator()

        actuator.enable()
        assert actuator.is_enabled is True
        assert actuator.state == ActuatorState.IDLE

        actuator.disable()
        assert actuator.is_enabled is False
        assert actuator.state == ActuatorState.DISABLED

    def test_context_manager(self) -> None:
        """Test using actuator as context manager."""
        actuator = SimulatedActuator()

        with actuator:
            assert actuator.is_enabled is True

        assert actuator.is_enabled is False

    def test_set_value(self) -> None:
        """Test setting actuator value."""
        actuator = SimulatedActuator(limits=Limits(0, 100, 50))
        actuator.enable()

        actuator.set(75)
        assert actuator.get() == 75
        assert actuator.state == ActuatorState.HOLDING

    def test_set_disabled_raises(self) -> None:
        """Test that set raises when disabled."""
        actuator = SimulatedActuator()

        with pytest.raises(DisabledError):
            actuator.set(50)

    def test_set_force_bypasses_disabled(self) -> None:
        """Test that force=True bypasses disabled check."""
        actuator = SimulatedActuator(limits=Limits(0, 100, 50))

        actuator.set(75, force=True)
        assert actuator.get() == 75

    def test_set_outside_limits_raises(self) -> None:
        """Test that set raises for values outside limits."""
        actuator = SimulatedActuator(limits=Limits(0, 100, 50))
        actuator.enable()

        with pytest.raises(LimitsExceededError):
            actuator.set(150)

        with pytest.raises(LimitsExceededError):
            actuator.set(-10)

    def test_go_to_default(self) -> None:
        """Test go_to_default method."""
        actuator = SimulatedActuator(limits=Limits(0, 180, 90))
        actuator.enable()
        actuator.set(45)

        actuator.go_to_default()
        assert actuator.get() == 90

    def test_go_to_min_max(self) -> None:
        """Test go_to_min and go_to_max methods."""
        actuator = SimulatedActuator(limits=Limits(0, 180, 90))
        actuator.enable()

        actuator.go_to_min()
        assert actuator.get() == 0

        actuator.go_to_max()
        assert actuator.get() == 180

    def test_status(self) -> None:
        """Test status method."""
        actuator = SimulatedActuator(limits=Limits(0, 100, 50))
        actuator.enable()
        actuator.set(75)

        status = actuator.status()
        assert status.state == ActuatorState.HOLDING
        assert status.position == 75
        assert status.is_enabled is True
        assert status.is_calibrated is True

    def test_calibration_required(self) -> None:
        """Test that calibration is enforced when required."""
        config = ActuatorConfig(require_calibration=True)
        actuator = SimulatedActuator(config=config)

        with pytest.raises(NotCalibratedError):
            actuator.enable()

        actuator.calibrate()
        assert actuator.is_calibrated is True

        actuator.enable()
        assert actuator.is_enabled is True

    def test_with_driver(self) -> None:
        """Test actuator with driver."""
        driver = SimulatedDriver(channels=16)
        driver.connect()

        actuator = SimulatedActuator(
            name="servo",
            driver=driver,
            channel=0,
            limits=Limits(0, 180, 90),
        )
        actuator.enable()

        actuator.set(90)
        assert actuator.get() == 90

        # Driver channel should have normalized value
        # 90 out of 180 = 0.5
        assert driver.get_channel(0) == 0.5

    def test_repr(self) -> None:
        """Test string representation."""
        actuator = SimulatedActuator(name="test", channel=5)
        repr_str = repr(actuator)
        assert "SimulatedActuator" in repr_str
        assert "test" in repr_str
        assert "5" in repr_str


class TestActuatorTransforms:
    """Tests for value transformation."""

    def test_scale_transform(self) -> None:
        """Test scale transformation."""
        config = ActuatorConfig(
            limits=Limits(0, 100, 50),
            scale=2.0,
        )
        actuator = SimulatedActuator(config=config)
        actuator.enable()

        actuator.set(25)  # Will be scaled to 50
        assert actuator.get() == 50

    def test_offset_transform(self) -> None:
        """Test offset transformation."""
        config = ActuatorConfig(
            limits=Limits(0, 100, 50),
            offset=10.0,
        )
        actuator = SimulatedActuator(config=config)
        actuator.enable()

        actuator.set(40)  # Will add 10 -> 50
        assert actuator.get() == 50

    def test_inverted_transform(self) -> None:
        """Test inverted transformation."""
        config = ActuatorConfig(
            limits=Limits(0, 100, 50),
            inverted=True,
        )
        actuator = SimulatedActuator(config=config)
        actuator.enable()

        actuator.set(25)  # Inverted: 100 - 25 = 75
        assert actuator.get() == 75


class TestActuatorGroup:
    """Tests for ActuatorGroup."""

    def test_create_group(self) -> None:
        """Test creating empty group."""
        group = ActuatorGroup(name="arm")
        assert group.name == "arm"
        assert len(group) == 0

    def test_add_actuator(self) -> None:
        """Test adding actuator to group."""
        group = ActuatorGroup(name="arm")
        actuator = SimulatedActuator(name="shoulder")

        group.add(actuator)

        assert "shoulder" in group
        assert len(group) == 1

    def test_add_actuator_custom_name(self) -> None:
        """Test adding actuator with custom name."""
        group = ActuatorGroup(name="arm")
        actuator = SimulatedActuator(name="shoulder")

        group.add(actuator, name="joint1")

        assert "joint1" in group
        assert "shoulder" not in group

    def test_remove_actuator(self) -> None:
        """Test removing actuator from group."""
        group = ActuatorGroup(name="arm")
        actuator = SimulatedActuator(name="shoulder")
        group.add(actuator)

        removed = group.remove("shoulder")

        assert removed is actuator
        assert "shoulder" not in group

    def test_get_actuator(self) -> None:
        """Test getting actuator by name."""
        group = ActuatorGroup(name="arm")
        actuator = SimulatedActuator(name="shoulder")
        group.add(actuator)

        result = group.get("shoulder")
        assert result is actuator

    def test_get_actuator_not_found(self) -> None:
        """Test getting nonexistent actuator raises error."""
        group = ActuatorGroup(name="arm")

        with pytest.raises(KeyError):
            group.get("nonexistent")

    def test_enable_disable_all(self) -> None:
        """Test enabling/disabling all actuators."""
        group = ActuatorGroup(name="arm")
        act1 = SimulatedActuator(name="a1")
        act2 = SimulatedActuator(name="a2")
        group.add(act1)
        group.add(act2)

        group.enable_all()
        assert act1.is_enabled
        assert act2.is_enabled

        group.disable_all()
        assert not act1.is_enabled
        assert not act2.is_enabled

    def test_set_all(self) -> None:
        """Test setting multiple actuators."""
        group = ActuatorGroup(name="arm")
        act1 = SimulatedActuator(name="a1", limits=Limits(0, 100, 50))
        act2 = SimulatedActuator(name="a2", limits=Limits(0, 100, 50))
        group.add(act1)
        group.add(act2)
        group.enable_all()

        group.set_all({"a1": 25, "a2": 75})

        assert act1.get() == 25
        assert act2.get() == 75

    def test_get_all(self) -> None:
        """Test getting all actuator values."""
        group = ActuatorGroup(name="arm")
        act1 = SimulatedActuator(name="a1", limits=Limits(0, 100, 50))
        act2 = SimulatedActuator(name="a2", limits=Limits(0, 100, 50))
        group.add(act1)
        group.add(act2)
        group.enable_all()
        act1.set(25)
        act2.set(75)

        values = group.get_all()

        assert values == {"a1": 25, "a2": 75}

    def test_go_to_defaults(self) -> None:
        """Test moving all to defaults."""
        group = ActuatorGroup(name="arm")
        act1 = SimulatedActuator(name="a1", limits=Limits(0, 100, 50))
        act2 = SimulatedActuator(name="a2", limits=Limits(0, 180, 90))
        group.add(act1)
        group.add(act2)
        group.enable_all()
        act1.set(25)
        act2.set(45)

        group.go_to_defaults()

        assert act1.get() == 50
        assert act2.get() == 90

    def test_status_all(self) -> None:
        """Test getting status of all actuators."""
        group = ActuatorGroup(name="arm")
        act1 = SimulatedActuator(name="a1")
        act2 = SimulatedActuator(name="a2")
        group.add(act1)
        group.add(act2)

        statuses = group.status_all()

        assert "a1" in statuses
        assert "a2" in statuses
        assert isinstance(statuses["a1"], ActuatorStatus)

    def test_context_manager(self) -> None:
        """Test group as context manager."""
        group = ActuatorGroup(name="arm")
        act1 = SimulatedActuator(name="a1")
        act2 = SimulatedActuator(name="a2")
        group.add(act1)
        group.add(act2)

        with group:
            assert act1.is_enabled
            assert act2.is_enabled

        assert not act1.is_enabled
        assert not act2.is_enabled

    def test_iteration(self) -> None:
        """Test iterating over group."""
        group = ActuatorGroup(name="arm")
        group.add(SimulatedActuator(name="a"))
        group.add(SimulatedActuator(name="b"))

        names = list(group)
        assert "a" in names
        assert "b" in names


class TestFactoryFunctions:
    """Tests for actuator factory functions."""

    def test_create_actuator_default(self) -> None:
        """Test create_actuator with defaults."""
        actuator = create_actuator("test")
        assert actuator.name == "test"
        assert isinstance(actuator, SimulatedActuator)

    def test_create_actuator_with_driver(self) -> None:
        """Test create_actuator with driver."""
        driver = SimulatedDriver()
        actuator = create_actuator("test", driver=driver, channel=5)
        assert actuator.driver is driver
        assert actuator.channel == 5

    def test_create_servo(self) -> None:
        """Test create_servo factory."""
        servo = create_servo("shoulder")
        assert servo.name == "shoulder"
        assert servo.actuator_type == ActuatorType.SERVO
        assert servo.limits.min == 0.0
        assert servo.limits.max == 180.0
        assert servo.limits.default == 90.0
        assert servo.unit == "degrees"

    def test_create_servo_custom(self) -> None:
        """Test create_servo with custom angles."""
        servo = create_servo(
            "custom",
            channel=3,
            min_angle=-90,
            max_angle=90,
            default_angle=0,
        )
        assert servo.channel == 3
        assert servo.limits.min == -90
        assert servo.limits.max == 90
        assert servo.limits.default == 0

    def test_create_motor(self) -> None:
        """Test create_motor factory."""
        motor = create_motor("drive")
        assert motor.name == "drive"
        assert motor.actuator_type == ActuatorType.DC_MOTOR
        assert motor.limits.min == -1.0
        assert motor.limits.max == 1.0
        assert motor.limits.default == 0.0
        assert motor.unit == "speed"


class TestActuatorBaseClass:
    """Tests for Actuator abstract base class behavior."""

    def test_actuator_is_abstract(self) -> None:
        """Test that Actuator base class works via SimulatedActuator."""
        actuator = SimulatedActuator()
        assert isinstance(actuator, Actuator)

    def test_actuator_properties(self) -> None:
        """Test actuator property accessors."""
        actuator = SimulatedActuator(
            name="test",
            channel=5,
            limits=Limits(0, 100, 50),
            unit="mm",
        )

        assert actuator.name == "test"
        assert actuator.channel == 5
        assert actuator.limits.min == 0
        assert actuator.limits.max == 100
        assert actuator.unit == "mm"
        assert actuator.driver is None

    def test_set_driver(self) -> None:
        """Test setting driver after creation."""
        actuator = SimulatedActuator()
        driver = SimulatedDriver()

        actuator.driver = driver

        assert actuator.driver is driver
