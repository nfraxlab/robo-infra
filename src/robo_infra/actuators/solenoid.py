"""Solenoid / Relay actuator implementation.

This module provides a `Solenoid` actuator for simple on/off outputs like:
- Solenoids
- Relays
- Valves

Control styles:

1) Pin-based control:
   - `pin` is a `DigitalPin` instance

2) Driver-channel control:
   - `channel` is an int and `driver` is a `Driver`

Phase 3.6 requirements:
- `Solenoid` class extending `Actuator`
- methods: activate/on, deactivate/off, toggle, pulse
- `Relay` alias for Solenoid
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator, ActuatorConfig, ActuatorState, ActuatorType
from robo_infra.core.exceptions import DisabledError
from robo_infra.core.types import Limits


logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import DigitalPin


class SolenoidConfig(BaseModel):
    """Configuration for Solenoid/Relay actuators."""

    name: str = "Solenoid"
    channel: int | None = None
    active_high: bool = True
    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class SolenoidStatus:
    """Status information for a solenoid."""

    state: ActuatorState = ActuatorState.DISABLED
    is_enabled: bool = False
    is_active: bool = False
    error: str | None = None


class Solenoid(Actuator):
    """On/off actuator for solenoids and relays."""

    def __init__(
        self,
        pin: DigitalPin | None = None,
        *,
        channel: int | None = None,
        driver: Driver | None = None,
        name: str = "Solenoid",
        active_high: bool = True,
        config: SolenoidConfig | None = None,
    ) -> None:
        if config is not None:
            self._solenoid_config = config
            name = config.name
            channel = config.channel
            active_high = config.active_high
        else:
            self._solenoid_config = SolenoidConfig(
                name=name,
                channel=channel,
                active_high=active_high,
            )

        actuator_config = ActuatorConfig(
            name=self._solenoid_config.name,
            actuator_type=ActuatorType.SOLENOID,
            channel=channel or 0,
            limits=Limits(min=0.0, max=1.0, default=0.0),
            unit="bool",
            require_calibration=False,
        )

        super().__init__(driver=driver, config=actuator_config)

        self._pin = pin
        self._active_high = active_high

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_active(self) -> bool:
        return self._current_value >= 0.5

    def activate(self) -> None:
        self.set(1.0)

    def deactivate(self) -> None:
        self.set(0.0)

    def on(self) -> None:
        self.activate()

    def off(self) -> None:
        self.deactivate()

    def toggle(self) -> None:
        self.set(0.0 if self.is_active else 1.0)

    def pulse(self, duration: float = 0.1) -> None:
        """Activate for a short duration, then deactivate."""
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        self.activate()
        if duration > 0:
            time.sleep(float(duration))
        self.deactivate()

    def status(self) -> SolenoidStatus:  # type: ignore[override]
        return SolenoidStatus(
            state=self._state,
            is_enabled=self._is_enabled,
            is_active=self.is_active,
            error=self._error,
        )

    # ------------------------------------------------------------------
    # Actuator hooks
    # ------------------------------------------------------------------

    def enable(self) -> None:
        super().enable()
        if self._pin is not None and not self._pin.initialized:
            self._pin.setup()
        # default safe state
        try:
            self.deactivate()
        except Exception as e:
            logger.error("Failed to deactivate solenoid on enable: %s", e)

    def disable(self) -> None:
        try:
            self.deactivate()
        except Exception as e:
            logger.error("Failed to deactivate solenoid on disable: %s", e)
        super().disable()

    def _apply_value(self, value: float) -> None:
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        active = float(value) >= 0.5
        pin_value = active if self._active_high else not active

        if self._pin is not None:
            self._pin.write(pin_value)

        if self._driver is not None:
            self._driver.set_channel(self._config.channel, 1.0 if active else 0.0)

        self._current_value = 1.0 if active else 0.0
        self._state = ActuatorState.HOLDING

    def _read_value(self) -> float:
        return self._current_value


Relay = Solenoid


class SimulatedSolenoid(Solenoid):
    """Simulated solenoid using a simulated digital pin."""

    def __init__(self, *, name: str = "SimulatedSolenoid", active_high: bool = True, **kwargs: Any):
        from robo_infra.core.pin import SimulatedDigitalPin

        super().__init__(
            pin=SimulatedDigitalPin(1),
            name=name,
            active_high=active_high,
            driver=None,
            **kwargs,
        )


def create_solenoid(
    *,
    name: str = "Solenoid",
    pin: DigitalPin | None = None,
    channel: int | None = None,
    driver: Driver | None = None,
    active_high: bool = True,
    simulated: bool = False,
    **kwargs: Any,
) -> Solenoid:
    """Factory function for solenoids."""
    if simulated:
        return SimulatedSolenoid(name=name, active_high=active_high, **kwargs)
    return Solenoid(
        pin=pin,
        channel=channel,
        driver=driver,
        name=name,
        active_high=active_high,
        **kwargs,
    )


__all__ = [
    "Relay",
    "SimulatedSolenoid",
    "Solenoid",
    "SolenoidConfig",
    "SolenoidStatus",
    "create_solenoid",
]
