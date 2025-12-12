"""Tests for robo_infra.actuators.linear."""

from __future__ import annotations

import pytest

from robo_infra.actuators.dc_motor import SimulatedDCMotor
from robo_infra.actuators.linear import LinearActuator, SimulatedLinearActuator
from robo_infra.actuators.solenoid import SimulatedSolenoid
from robo_infra.core.exceptions import DisabledError, SafetyError
from robo_infra.core.sensor import SimulatedSensor
from robo_infra.core.types import Limits, Unit


class TestLinearActuatorMotorBased:
    def test_extend_retract_requires_enabled(self) -> None:
        a = LinearActuator(motor=SimulatedDCMotor())
        with pytest.raises(DisabledError):
            a.extend()

    def test_extend_retract(self) -> None:
        a = LinearActuator(motor=SimulatedDCMotor())
        a.enable()
        a.extend()
        assert a.position == a.limits.max
        a.retract()
        assert a.position == a.limits.min

    def test_move_to_open_loop(self) -> None:
        a = LinearActuator(motor=SimulatedDCMotor())
        a.enable()
        a.move_to(0.3)
        assert abs(a.position - 0.3) < 1e-6


class TestLinearActuatorSolenoidBased:
    def test_solenoid_extend_retract(self) -> None:
        a = LinearActuator(solenoid=SimulatedSolenoid())
        a.enable()
        a.extend()
        assert a.position == a.limits.max
        a.retract()
        assert a.position == a.limits.min


class TestLinearActuatorFeedback:
    def test_move_to_with_feedback_unreachable_raises(self) -> None:
        # Sensor value stays fixed -> should fail safety check
        sensor = SimulatedSensor(
            name="pos",
            unit=Unit.RAW,
            limits=Limits(min=0, max=1, default=0),
            initial_value=0,
        )
        sensor.enable()

        a = LinearActuator(motor=SimulatedDCMotor(), position_sensor=sensor)
        a.enable()

        with pytest.raises(SafetyError):
            a.move_to(1.0)


class TestSimulatedLinearActuator:
    def test_simulated_factory(self) -> None:
        a = SimulatedLinearActuator()
        a.enable()
        a.move_to(0.7)
        assert abs(a.position - 0.7) < 1e-6
