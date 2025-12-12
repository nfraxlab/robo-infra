"""Tests for robo_infra.actuators.dc_motor."""

from __future__ import annotations

import pytest

from robo_infra.actuators.dc_motor import (
    DCMotor,
    DCMotorConfig,
    DCMotorStatus,
    SimulatedDCMotor,
    create_dc_motor,
)
from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import DisabledError, LimitsExceededError
from robo_infra.core.pin import SimulatedDigitalPin, SimulatedPWMPin
from robo_infra.core.types import Direction


class TestDCMotorConfig:
    def test_defaults(self) -> None:
        cfg = DCMotorConfig()
        assert cfg.name == "DCMotor"
        assert cfg.pin_a is None
        assert cfg.pin_b is None
        assert cfg.enable is None
        assert cfg.inverted is False
        assert cfg.pwm_frequency == 1000


class TestDCMotorStatus:
    def test_defaults(self) -> None:
        status = DCMotorStatus()
        assert status.speed == 0.0
        assert status.direction == Direction.STOP
        assert status.is_running is False


class TestDCMotorPins:
    def test_forward_sets_pins_and_pwm(self) -> None:
        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3)
        motor = DCMotor(pin_a=pin_a, pin_b=pin_b, enable=enable)

        motor.enable()
        motor.forward(0.6)

        assert motor.speed == 0.6
        assert motor.direction == Direction.FORWARD
        assert pin_a.read() is True
        assert pin_b.read() is False
        assert enable.duty_cycle == 0.6

    def test_reverse_sets_pins_and_pwm(self) -> None:
        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3)
        motor = DCMotor(pin_a=pin_a, pin_b=pin_b, enable=enable)

        motor.enable()
        motor.reverse(0.25)

        assert motor.speed == -0.25
        assert motor.direction == Direction.REVERSE
        assert pin_a.read() is False
        assert pin_b.read() is True
        assert enable.duty_cycle == 0.25

    def test_stop_coasts(self) -> None:
        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3)
        motor = DCMotor(pin_a=pin_a, pin_b=pin_b, enable=enable)

        motor.enable()
        motor.forward(0.8)
        motor.stop()

        assert motor.speed == 0.0
        assert motor.direction == Direction.STOP
        assert pin_a.read() is False
        assert pin_b.read() is False
        assert enable.duty_cycle == 0.0

    def test_brake_sets_both_high(self) -> None:
        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3)
        motor = DCMotor(pin_a=pin_a, pin_b=pin_b, enable=enable)

        motor.enable()
        motor.forward(0.7)
        motor.brake()

        assert motor.speed == 0.0
        assert motor.direction == Direction.STOP
        assert pin_a.read() is True
        assert pin_b.read() is True
        assert enable.duty_cycle == 0.0

    def test_coast_sets_both_low(self) -> None:
        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3)
        motor = DCMotor(pin_a=pin_a, pin_b=pin_b, enable=enable)

        motor.enable()
        motor.forward(0.7)
        motor.coast()

        assert motor.speed == 0.0
        assert motor.direction == Direction.STOP
        assert pin_a.read() is False
        assert pin_b.read() is False
        assert enable.duty_cycle == 0.0

    def test_inverted_swaps_direction_pins(self) -> None:
        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3)
        motor = DCMotor(pin_a=pin_a, pin_b=pin_b, enable=enable, inverted=True)

        motor.enable()
        motor.forward(0.5)

        assert motor.direction == Direction.REVERSE
        assert pin_a.read() is False
        assert pin_b.read() is True
        assert enable.duty_cycle == 0.5


class TestDCMotorDriverChannels:
    def test_forward_writes_driver_channels(self) -> None:
        driver = SimulatedDriver(name="drv", channels=8)
        driver.connect()
        driver.enable()

        motor = DCMotor(pin_a=0, pin_b=1, enable=2, driver=driver)
        motor.enable()
        motor.forward(0.9)

        assert driver.get_channel(0) == 1.0
        assert driver.get_channel(1) == 0.0
        assert driver.get_channel(2) == 0.9

    def test_reverse_writes_driver_channels(self) -> None:
        driver = SimulatedDriver(name="drv", channels=8)
        driver.connect()
        driver.enable()

        motor = DCMotor(pin_a=0, pin_b=1, enable=2, driver=driver)
        motor.enable()
        motor.reverse(0.4)

        assert driver.get_channel(0) == 0.0
        assert driver.get_channel(1) == 1.0
        assert driver.get_channel(2) == 0.4


class TestDCMotorErrors:
    def test_set_when_disabled_raises(self) -> None:
        motor = SimulatedDCMotor()
        with pytest.raises(DisabledError):
            motor.set(0.2)

    def test_limits_exceeded_raises(self) -> None:
        motor = SimulatedDCMotor()
        motor.enable()
        with pytest.raises(LimitsExceededError):
            motor.set(1.5)


class TestDCMotorFactory:
    def test_create_simulated(self) -> None:
        motor = create_dc_motor(simulated=True)
        assert isinstance(motor, SimulatedDCMotor)

    def test_create_real(self) -> None:
        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3)

        motor = create_dc_motor(pin_a=pin_a, pin_b=pin_b, enable=enable)
        assert isinstance(motor, DCMotor)
