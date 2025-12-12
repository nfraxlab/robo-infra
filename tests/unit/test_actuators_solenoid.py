"""Tests for robo_infra.actuators.solenoid."""

from __future__ import annotations

import pytest

from robo_infra.actuators.solenoid import Relay, SimulatedSolenoid, Solenoid
from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import DisabledError


class TestSolenoidBasics:
    def test_enable_default_off(self) -> None:
        s = SimulatedSolenoid()
        s.enable()
        assert s.is_active is False

    def test_activate_deactivate(self) -> None:
        s = SimulatedSolenoid()
        s.enable()
        s.activate()
        assert s.is_active is True
        s.deactivate()
        assert s.is_active is False

    def test_toggle(self) -> None:
        s = SimulatedSolenoid()
        s.enable()
        s.toggle()
        assert s.is_active is True
        s.toggle()
        assert s.is_active is False

    def test_pulse_requires_enabled(self) -> None:
        s = SimulatedSolenoid()
        with pytest.raises(DisabledError):
            s.pulse(0)

    def test_pulse(self) -> None:
        s = SimulatedSolenoid()
        s.enable()
        s.pulse(0)
        assert s.is_active is False

    def test_relay_alias(self) -> None:
        r = Relay()
        assert isinstance(r, Solenoid)


class TestSolenoidDriverChannel:
    def test_driver_channel_updates(self) -> None:
        driver = SimulatedDriver(name="drv", channels=2)
        driver.connect()
        driver.enable()

        s = Solenoid(channel=1, driver=driver)
        s.enable()

        s.activate()
        assert driver.get_channel(1) == 1.0

        s.deactivate()
        assert driver.get_channel(1) == 0.0
