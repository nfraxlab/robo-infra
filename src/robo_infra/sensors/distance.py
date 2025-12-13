"""Distance sensor implementations.

Phase 4.1 provides common distance/proximity sensors:
- Ultrasonic (trigger/echo)
- Time-of-Flight (I2C)
- IR distance (analog voltage)

All sensors extend `Sensor` via `DistanceSensor`.

Notes:
- The core `Sensor` abstraction expects `_read_raw() -> int`.
- Each sensor returns a `Reading` whose `.value` is distance in the chosen unit.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.exceptions import CommunicationError
from robo_infra.core.sensor import Sensor
from robo_infra.core.types import Limits, Unit


logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import AnalogPin, DigitalPin


# -----------------------------------------------------------------------------
# Base class
# -----------------------------------------------------------------------------


class DistanceSensor(Sensor):
    """Base class for distance sensors."""

    def read_distance(self) -> float:
        """Convenience: return the latest distance value."""
        return float(self.read().value)


# -----------------------------------------------------------------------------
# Ultrasonic
# -----------------------------------------------------------------------------


class UltrasonicConfig(BaseModel):
    """Configuration for ultrasonic sensors."""

    name: str = "Ultrasonic"
    unit: Unit = Unit.CENTIMETERS

    # Timing
    trigger_pulse_us: int = 10
    timeout_s: float = 0.02

    # Conversion
    # Typical approximation: distance_cm = duration_us / 58
    us_per_cm: float = 58.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class UltrasonicStatus:
    last_duration_us: int | None = None


class Ultrasonic(DistanceSensor):
    """Ultrasonic distance sensor.

    `read()` returns distance (cm or mm) and `read_raw()` returns the pulse duration (μs).

    Two operating styles:
    - Pin-based: `trigger_pin` and `echo_pin` are DigitalPin.
    - Driver-based: provide `driver` + `channel` where the channel yields raw microseconds.
    """

    def __init__(
        self,
        *,
        trigger_pin: DigitalPin | None = None,
        echo_pin: DigitalPin | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        name: str = "Ultrasonic",
        unit: Unit = Unit.CENTIMETERS,
        timeout_s: float = 0.02,
        us_per_cm: float = 58.0,
        config: UltrasonicConfig | None = None,
    ) -> None:
        if config is not None:
            self._cfg = config
            name = config.name
            unit = config.unit
            timeout_s = config.timeout_s
            us_per_cm = config.us_per_cm
        else:
            self._cfg = UltrasonicConfig(
                name=name, unit=unit, timeout_s=timeout_s, us_per_cm=us_per_cm
            )

        self._trigger = trigger_pin
        self._echo = echo_pin
        self._timeout_s = float(timeout_s)
        self._status = UltrasonicStatus(last_duration_us=None)

        # Configure transformation from raw_us -> distance
        if unit == Unit.CENTIMETERS:
            scale = 1.0 / float(us_per_cm)
        elif unit == Unit.MILLIMETERS:
            scale = 10.0 / float(us_per_cm)
        else:
            raise ValueError("Ultrasonic unit must be Unit.CENTIMETERS or Unit.MILLIMETERS")

        super().__init__(
            name=name,
            driver=driver,
            channel=channel,
            unit=unit,
            limits=Limits(min=0.0, max=float("inf")),
            scale=scale,
            offset=0.0,
            inverted=False,
            requires_calibration=False,
        )

    def enable(self) -> None:
        super().enable()
        for pin in (self._trigger, self._echo):
            if pin is not None and not pin.initialized:
                pin.setup()

    def status(self) -> UltrasonicStatus:  # type: ignore[override]
        return self._status

    def _read_raw(self) -> int:
        # Driver-backed: raw microseconds.
        if self._driver is not None:
            raw = int(self._driver.get_channel(self._channel))
            self._status.last_duration_us = raw
            return raw

        if self._trigger is None or self._echo is None:
            raise CommunicationError("Ultrasonic requires trigger+echo pins or a driver channel")

        # Trigger pulse
        self._trigger.write(False)
        self._trigger.write(True)
        time.sleep(self._cfg.trigger_pulse_us / 1_000_000)
        self._trigger.write(False)

        deadline = time.monotonic() + self._timeout_s

        # Wait for echo high
        while not self._echo.read():
            if time.monotonic() > deadline:
                raise CommunicationError("Ultrasonic echo timeout (waiting for HIGH)")

        start = time.perf_counter()

        # Wait for echo low
        while self._echo.read():
            if time.monotonic() > deadline:
                raise CommunicationError("Ultrasonic echo timeout (waiting for LOW)")

        end = time.perf_counter()

        duration_us = int((end - start) * 1_000_000)
        self._status.last_duration_us = duration_us
        return duration_us


class ToFConfig(BaseModel):
    name: str = "ToF"
    address: int = 0x29
    register: int = 0x00
    unit: Unit = Unit.MILLIMETERS
    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class ToFStatus:
    last_mm: int | None = None


class ToF(DistanceSensor):
    """Generic time-of-flight distance sensor over I2C.

    This is a lightweight, register-based reader for a distance value in mm.
    Concrete chip-specific behavior should be implemented in dedicated drivers.

    For simulation/tests, use `SimulatedI2CBus` and set the device register bytes.
    """

    def __init__(
        self,
        bus: I2CBus,
        *,
        address: int = 0x29,
        register: int = 0x00,
        name: str = "ToF",
        unit: Unit = Unit.MILLIMETERS,
        config: ToFConfig | None = None,
    ) -> None:
        if config is not None:
            self._cfg = config
            name = config.name
            address = config.address
            register = config.register
            unit = config.unit
        else:
            self._cfg = ToFConfig(name=name, address=address, register=register, unit=unit)

        if unit != Unit.MILLIMETERS:
            raise ValueError("ToF currently returns mm; use unit=Unit.MILLIMETERS")

        self._bus = bus
        self._address = int(address)
        self._register = int(register)
        self._status = ToFStatus(last_mm=None)

        super().__init__(
            name=name,
            driver=None,
            channel=0,
            unit=unit,
            limits=Limits(min=0.0, max=float("inf")),
            scale=1.0,
            offset=0.0,
            inverted=False,
            requires_calibration=False,
        )

    def enable(self) -> None:
        super().enable()
        try:
            self._bus.open()
        except Exception as e:
            logger.error("Failed to open I2C bus for ToF sensor: %s", e)
            raise

    def disable(self) -> None:
        try:
            self._bus.close()
        except Exception as e:
            # Bus close is non-critical, just log
            logger.debug("Failed to close I2C bus: %s", e)
        super().disable()

    def status(self) -> ToFStatus:  # type: ignore[override]
        return self._status

    def _read_raw(self) -> int:
        try:
            mm = self._bus.read_register_word(self._address, self._register)
        except Exception as e:
            raise CommunicationError(f"ToF I2C read failed: {e}") from e

        self._status.last_mm = int(mm)
        return int(mm)


# -----------------------------------------------------------------------------
# IR distance (analog)
# -----------------------------------------------------------------------------


class IRDistanceConfig(BaseModel):
    name: str = "IRDistance"
    unit: Unit = Unit.MILLIMETERS

    model: str = "inverse_voltage"  # distance_mm = a / max(eps, (voltage - b))
    a: float = 1000.0
    b: float = 0.0
    eps: float = 1e-6

    # Optional clamps
    min_mm: int = 0
    max_mm: int = 5000

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class IRDistanceStatus:
    last_voltage: float | None = None


class IRDistance(DistanceSensor):
    """IR distance sensor reading analog voltage.

    `read()` returns distance in mm by default.

    - Pin-based: provide `analog_pin`.
    - Driver-based: provide `driver` + `channel` representing normalized voltage [0..1] or raw volts.
      (Driver integration is intentionally generic; prefer AnalogPin for real hardware.)
    """

    def __init__(
        self,
        *,
        analog_pin: AnalogPin | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        name: str = "IRDistance",
        a: float = 1000.0,
        b: float = 0.0,
        eps: float = 1e-6,
        min_mm: int = 0,
        max_mm: int = 5000,
        config: IRDistanceConfig | None = None,
    ) -> None:
        if config is not None:
            self._cfg = config
            name = config.name
            a = config.a
            b = config.b
            eps = config.eps
            min_mm = config.min_mm
            max_mm = config.max_mm
        else:
            self._cfg = IRDistanceConfig(
                name=name,
                a=a,
                b=b,
                eps=eps,
                min_mm=min_mm,
                max_mm=max_mm,
            )

        self._analog = analog_pin
        self._a = float(a)
        self._b = float(b)
        self._eps = float(eps)
        self._status = IRDistanceStatus(last_voltage=None)

        super().__init__(
            name=name,
            driver=driver,
            channel=channel,
            unit=Unit.MILLIMETERS,
            limits=Limits(min=float(min_mm), max=float(max_mm)),
            scale=1.0,
            offset=0.0,
            inverted=False,
            requires_calibration=False,
        )

    def enable(self) -> None:
        super().enable()
        if self._analog is not None and not self._analog.initialized:
            self._analog.setup()

    def status(self) -> IRDistanceStatus:  # type: ignore[override]
        return self._status

    def _read_raw(self) -> int:
        # Read voltage
        if self._analog is not None:
            voltage = float(self._analog.read())
        elif self._driver is not None:
            # Treat driver channel as volts.
            voltage = float(self._driver.get_channel(self._channel))
        else:
            raise CommunicationError("IRDistance requires analog_pin or driver channel")

        self._status.last_voltage = voltage

        denom = max(self._eps, voltage - self._b)
        distance_mm = int(self._a / denom)
        distance_mm = max(int(self.limits.min), min(int(self.limits.max), distance_mm))
        return distance_mm


__all__ = [
    "DistanceSensor",
    "IRDistance",
    "IRDistanceConfig",
    "IRDistanceStatus",
    "ToF",
    "ToFConfig",
    "ToFStatus",
    "Ultrasonic",
    "UltrasonicConfig",
    "UltrasonicStatus",
]
