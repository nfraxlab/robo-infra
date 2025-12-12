"""Tests for robo_infra.actuators.brushless."""

from __future__ import annotations

import pytest

from robo_infra.actuators.brushless import Brushless, SimulatedBrushless, create_brushless
from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import SafetyError


class TestBrushlessBasics:
    def test_enable_disables_output(self) -> None:
        esc = SimulatedBrushless()
        esc.enable()
        assert esc.throttle == 0.0
        assert esc.armed is False

    def test_arm_allows_throttle(self) -> None:
        esc = SimulatedBrushless()
        esc.enable()
        esc.arm()
        assert esc.armed is True

        esc.set(0.25)
        assert esc.throttle == 0.25

    def test_set_nonzero_unarmed_raises(self) -> None:
        esc = SimulatedBrushless()
        esc.enable()
        with pytest.raises(SafetyError):
            esc.set(0.2)

    def test_disarm_forces_zero(self) -> None:
        esc = SimulatedBrushless()
        esc.enable()
        esc.arm()
        esc.set(0.8)
        esc.disarm()
        assert esc.armed is False
        assert esc.throttle == 0.0

    def test_calibrate_runs_sequence(self) -> None:
        esc = SimulatedBrushless()
        esc.enable()
        esc.calibrate()
        assert esc.throttle == 0.0


class TestBrushlessDriver:
    def test_driver_receives_throttle(self) -> None:
        driver = SimulatedDriver(name="drv", channels=4)
        driver.connect()
        driver.enable()

        esc = Brushless(channel=0, driver=driver)
        esc.enable()
        esc.arm()
        esc.set(0.6)

        assert 0.59 < driver.get_channel(0) < 0.61


class TestBrushlessFactory:
    def test_factory_simulated(self) -> None:
        esc = create_brushless(simulated=True)
        assert isinstance(esc, SimulatedBrushless)

    def test_factory_real(self) -> None:
        driver = SimulatedDriver(name="drv", channels=2)
        esc = create_brushless(driver=driver, channel=1)
        assert isinstance(esc, Brushless)
