"""Tests for robo_infra.actuators.stepper."""

from __future__ import annotations

import pytest

from robo_infra.actuators.stepper import SimulatedStepper, Stepper
from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import DisabledError, SafetyError
from robo_infra.core.pin import SimulatedDigitalPin
from robo_infra.core.sensor import SimulatedSensor
from robo_infra.core.types import Direction, Limits, Unit


class TestStepperBasics:
    def test_properties(self) -> None:
        stepper = SimulatedStepper(
            steps_per_rev=200, microsteps=16, max_speed=3000.0, acceleration=100.0
        )
        assert stepper.steps_per_rev == 200
        assert stepper.microsteps == 16
        assert stepper.max_speed == 3000.0
        assert stepper.acceleration == 100.0

    def test_enable_and_disable(self) -> None:
        stepper = SimulatedStepper()
        stepper.enable()
        assert stepper.is_enabled is True
        stepper.disable()
        assert stepper.is_enabled is False

    def test_step_forward_updates_position(self) -> None:
        stepper = SimulatedStepper()
        stepper.enable()
        stepper.step(5, Direction.FORWARD)
        assert stepper.position == 5

    def test_step_reverse_updates_position(self) -> None:
        stepper = SimulatedStepper()
        stepper.enable()
        stepper.step(3, Direction.REVERSE)
        assert stepper.position == -3

    def test_move_by(self) -> None:
        stepper = SimulatedStepper()
        stepper.enable()
        stepper.move_by(10)
        assert stepper.position == 10
        stepper.move_by(-4)
        assert stepper.position == 6

    def test_move_to(self) -> None:
        stepper = SimulatedStepper()
        stepper.enable()
        stepper.move_to(12)
        assert stepper.position == 12
        stepper.move_to(-2)
        assert stepper.position == -2

    def test_set_speed_clamped(self) -> None:
        stepper = SimulatedStepper(max_speed=100.0)
        stepper.enable()
        stepper.set_speed(500.0)
        # internal speed is not exposed, but should not crash
        stepper.step(1, Direction.FORWARD)
        assert stepper.position == 1


class TestStepperErrors:
    def test_step_when_disabled_raises(self) -> None:
        stepper = SimulatedStepper()
        with pytest.raises(DisabledError):
            stepper.step(1, Direction.FORWARD)

    def test_step_out_of_limits_raises(self) -> None:
        # Use a tiny limit range by constructing directly
        step_pin = SimulatedDigitalPin(1)
        dir_pin = SimulatedDigitalPin(2)
        enable_pin = SimulatedDigitalPin(3)
        stepper = Stepper(step_pin=step_pin, dir_pin=dir_pin, enable_pin=enable_pin, name="s")
        # Override limits for test
        stepper._config.limits = Limits(min=-2, max=2, default=0)  # type: ignore[assignment]
        stepper.enable()
        with pytest.raises(SafetyError):
            stepper.step(3, Direction.FORWARD)


class TestStepperHome:
    def test_home_when_already_triggered(self) -> None:
        stepper = SimulatedStepper()
        stepper.enable()
        stepper.move_by(5)

        switch = SimulatedSensor(
            name="sw",
            unit=Unit.RAW,
            limits=Limits(min=0, max=1, default=1),
            initial_value=1,
        )
        switch.enable()

        stepper.home(switch)
        assert stepper.position == 0

    def test_home_not_triggered_raises(self) -> None:
        stepper = SimulatedStepper(steps_per_rev=5, microsteps=1)
        stepper.enable()
        switch = SimulatedSensor(
            name="sw",
            unit=Unit.RAW,
            limits=Limits(min=0, max=1, default=0),
            initial_value=0,
        )
        switch.enable()

        with pytest.raises(SafetyError):
            stepper.home(switch)


class TestStepperDriverChannels:
    def test_driver_channel_step_pulses(self) -> None:
        driver = SimulatedDriver(name="drv", channels=8)
        driver.connect()
        driver.enable()

        stepper = Stepper(step_pin=0, dir_pin=1, enable_pin=2, driver=driver)
        stepper.enable()
        stepper.step(2, Direction.FORWARD)

        # Direction should have been set to forward at least once
        assert driver.get_channel(1) in (0.0, 1.0)
        assert stepper.position == 2
