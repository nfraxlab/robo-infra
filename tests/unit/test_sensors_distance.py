"""Tests for robo_infra.sensors.distance (Phase 4.1)."""

from __future__ import annotations

import pytest

from robo_infra.core.bus import SimulatedI2CBus
from robo_infra.core.driver import ChannelConfig, SimulatedDriver
from robo_infra.core.exceptions import CommunicationError
from robo_infra.core.pin import SimulatedAnalogPin
from robo_infra.core.types import Unit
from robo_infra.sensors.distance import IRDistance, ToF, Ultrasonic


class TestUltrasonic:
    def test_driver_backed_raw_us_to_cm(self) -> None:
        driver = SimulatedDriver(name="drv", channels=4)
        driver.connect()
        driver.enable()

        # Configure channel to accept microsecond values (0-10000)
        driver.set_channel_config(0, ChannelConfig(min_value=0.0, max_value=10000.0))

        # 58us per cm -> 580us should be ~10cm
        driver.set_channel(0, 580.0)

        s = Ultrasonic(driver=driver, channel=0, unit=Unit.CENTIMETERS)
        s.enable()
        reading = s.read()
        assert reading.unit == Unit.CENTIMETERS
        assert 9.5 < reading.value < 10.5
        assert s.read_raw() == 580


class TestToF:
    def test_i2c_register_word_mm(self) -> None:
        bus = SimulatedI2CBus()
        bus.add_device(0x29, registers={0x00: 0x01, 0x01: 0xF4})  # 0x01F4 = 500mm
        bus.open()

        s = ToF(bus, address=0x29, register=0x00)
        s.enable()
        reading = s.read()
        assert reading.unit == Unit.MILLIMETERS
        assert reading.value == 500.0


class TestIRDistance:
    def test_analog_voltage_conversion(self) -> None:
        pin = SimulatedAnalogPin(0, reference_voltage=3.3, initial_value=2.0)
        s = IRDistance(analog_pin=pin, a=1000.0, b=0.0)
        s.enable()

        reading = s.read()
        assert reading.unit == Unit.MILLIMETERS
        assert reading.value == 500.0

    def test_driver_voltage_channel(self) -> None:
        driver = SimulatedDriver(name="drv", channels=2)
        driver.connect()
        driver.enable()
        driver.set_channel(1, 1.0)

        s = IRDistance(driver=driver, channel=1, a=1000.0, b=0.0)
        s.enable()
        reading = s.read()
        assert reading.value == 1000.0

    def test_requires_source(self) -> None:
        s = IRDistance(a=1000.0)
        s.enable()
        with pytest.raises(CommunicationError):
            _ = s.read()
