"""Brushless motor (ESC) actuator implementation.

This module provides a `Brushless` actuator for controlling brushless motors
via an ESC (Electronic Speed Controller).

The actuator exposes a simple interface:
- throttle: 0.0 to 1.0
- arm/disarm
- calibrate

Control styles:

1) Pin-based PWM control:
   - `pwm` is a `PWMPin` instance

2) Driver-channel control:
   - `channel` is an int and `driver` is a `Driver`

Protocol is tracked as a string (PWM/OneShot/DShot). For non-PWM protocols,
this class still sends normalized throttle to the driver/channel; hardware-
-specific implementations are expected to live in Drivers (Phase 5).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator, ActuatorConfig, ActuatorState, ActuatorType
from robo_infra.core.exceptions import DisabledError, SafetyError
from robo_infra.core.types import Limits


if TYPE_CHECKING:
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import PWMPin


logger = logging.getLogger(__name__)


class BrushlessConfig(BaseModel):
    """Configuration model for Brushless/ESC actuators."""

    name: str = "Brushless"
    channel: int = 0
    protocol: str = "pwm"  # PWM/OneShot/DShot

    # PWM defaults (common ESC): 1000us-2000us @ 50Hz
    frequency: int = 50
    min_pulse_us: int = 1000
    max_pulse_us: int = 2000

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class BrushlessStatus:
    """Status information for a brushless ESC."""

    state: ActuatorState = ActuatorState.DISABLED
    throttle: float = 0.0
    armed: bool = False
    protocol: str = "pwm"
    is_enabled: bool = False
    error: str | None = None


class Brushless(Actuator):
    """Brushless motor ESC actuator."""

    def __init__(
        self,
        channel: int = 0,
        driver: Driver | None = None,
        *,
        pwm: PWMPin | None = None,
        name: str = "Brushless",
        protocol: str = "pwm",
        frequency: int = 50,
        min_pulse_us: int = 1000,
        max_pulse_us: int = 2000,
        config: BrushlessConfig | None = None,
    ) -> None:
        if config is not None:
            self._esc_config = config
            name = config.name
            channel = config.channel
            protocol = config.protocol
            frequency = config.frequency
            min_pulse_us = config.min_pulse_us
            max_pulse_us = config.max_pulse_us
        else:
            self._esc_config = BrushlessConfig(
                name=name,
                channel=channel,
                protocol=protocol,
                frequency=frequency,
                min_pulse_us=min_pulse_us,
                max_pulse_us=max_pulse_us,
            )

        actuator_config = ActuatorConfig(
            name=self._esc_config.name,
            actuator_type=ActuatorType.GENERIC,  # ESC is a kind of motor; keep generic for now
            channel=channel,
            limits=Limits(min=0.0, max=1.0, default=0.0),
            unit="ratio",
            require_calibration=False,
        )

        super().__init__(driver=driver, config=actuator_config)

        self._pwm = pwm
        self._armed = False
        self._protocol = self._esc_config.protocol
        self._frequency = self._esc_config.frequency
        self._min_pulse_us = self._esc_config.min_pulse_us
        self._max_pulse_us = self._esc_config.max_pulse_us
        self._calibrating = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def throttle(self) -> float:
        return self._current_value

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def protocol(self) -> str:
        return self._protocol

    # ------------------------------------------------------------------
    # Public API (Phase 3.4)
    # ------------------------------------------------------------------

    def arm(self) -> None:
        """Arm the ESC (enables non-zero throttle)."""
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        self._armed = True
        self.set(0.0)

    def disarm(self) -> None:
        """Disarm the ESC and force throttle to 0."""
        # Even if disabled, attempt to stop output.
        self._armed = False
        try:
            self._write_throttle(0.0)
        except Exception as e:
            logger.error("Failed to write zero throttle during disarm: %s", e)
            # Still update state, but log the failure
        self._current_value = 0.0
        self._state = ActuatorState.IDLE if self._is_enabled else ActuatorState.DISABLED

    def calibrate(self) -> None:
        """Run a basic ESC throttle calibration sequence.

        This method performs a simple max->min throttle sequence.
        Hardware-specific timing/confirmation is driver-dependent.
        """
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        self._calibrating = True
        try:
            # Typical sequence: send max then min.
            self._write_throttle(1.0)
            self._write_throttle(0.0)
        finally:
            self._calibrating = False

    def status(self) -> BrushlessStatus:  # type: ignore[override]
        return BrushlessStatus(
            state=self._state,
            throttle=self._current_value,
            armed=self._armed,
            protocol=self._protocol,
            is_enabled=self._is_enabled,
            error=self._error,
        )

    # ------------------------------------------------------------------
    # Actuator hooks
    # ------------------------------------------------------------------

    def enable(self) -> None:
        super().enable()
        if self._pwm is not None:
            if not self._pwm.initialized:
                self._pwm.setup()
            try:
                self._pwm.set_frequency(self._frequency)
            except Exception as e:
                # Non-critical: some platforms don't support dynamic frequency
                logger.debug("PWM frequency change not supported: %s", e)
        # Ensure safe output
        self.disarm()

    def disable(self) -> None:
        try:
            self.disarm()
        except Exception as e:
            logger.error("Failed to disarm during disable: %s", e)
            # Continue to disable even if disarm fails
        super().disable()

    def _apply_value(self, value: float) -> None:
        throttle = float(value)
        if throttle > 0 and not (self._armed or self._calibrating):
            raise SafetyError("ESC not armed", action_taken="refuse throttle")

        self._write_throttle(throttle)

    def _read_value(self) -> float:
        return self._current_value

    # ------------------------------------------------------------------
    # Output helpers
    # ------------------------------------------------------------------

    def _write_throttle(self, throttle: float) -> None:
        throttle = max(0.0, min(1.0, float(throttle)))

        # PWM pin path: map throttle to pulse width and duty cycle
        if self._pwm is not None and self._protocol.lower() in {"pwm", "oneshot", "oneshot125"}:
            pulse = self._min_pulse_us + int(throttle * (self._max_pulse_us - self._min_pulse_us))
            period_us = int(1_000_000 / max(1, self._frequency))
            duty = min(1.0, max(0.0, pulse / period_us))
            self._pwm.set_duty_cycle(duty)

        # Driver path: send normalized throttle
        if self._driver is not None:
            self._driver.set_channel(self._config.channel, throttle)

        self._current_value = throttle
        self._state = ActuatorState.HOLDING


class SimulatedBrushless(Brushless):
    """Simulated brushless ESC using a simulated PWM pin."""

    def __init__(
        self,
        *,
        name: str = "SimulatedBrushless",
        protocol: str = "pwm",
        frequency: int = 50,
        min_pulse_us: int = 1000,
        max_pulse_us: int = 2000,
        **kwargs: Any,
    ) -> None:
        from robo_infra.core.pin import SimulatedPWMPin

        super().__init__(
            pwm=SimulatedPWMPin(1, frequency=frequency),
            name=name,
            protocol=protocol,
            frequency=frequency,
            min_pulse_us=min_pulse_us,
            max_pulse_us=max_pulse_us,
            **kwargs,
        )


def create_brushless(
    *,
    name: str = "Brushless",
    channel: int = 0,
    driver: Driver | None = None,
    pwm: PWMPin | None = None,
    protocol: str = "pwm",
    simulated: bool = False,
    **kwargs: Any,
) -> Brushless:
    """Factory function for brushless ESC actuators."""
    if simulated:
        return SimulatedBrushless(name=name, protocol=protocol, **kwargs)
    return Brushless(
        channel=channel,
        driver=driver,
        pwm=pwm,
        name=name,
        protocol=protocol,
        **kwargs,
    )


__all__ = [
    "Brushless",
    "BrushlessConfig",
    "BrushlessStatus",
    "SimulatedBrushless",
    "create_brushless",
]
