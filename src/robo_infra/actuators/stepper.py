"""Stepper motor actuator implementation.

This module provides a `Stepper` actuator for controlling stepper motors using
common STEP/DIR style drivers.

Supported control styles:

1) Pin-based control:
   - `step_pin` and `dir_pin` are `DigitalPin` instances
   - optional `enable_pin` is a `DigitalPin` instance

2) Driver-channel control:
   - `step_pin`, `dir_pin`, `enable_pin` are integers (driver channel numbers)
   - `driver` is a `Driver` instance

The Stepper is modeled as an open-loop actuator: position is tracked in steps
based on commands (no feedback).

Plan requirements (Phase 3.3):
- properties: position, steps_per_rev, microsteps, max_speed, acceleration
- methods: step, move_to, move_by, set_speed, home, stop, disable
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator, ActuatorConfig, ActuatorState, ActuatorType
from robo_infra.core.exceptions import DisabledError, SafetyError
from robo_infra.core.types import Direction, Limits


if TYPE_CHECKING:
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import DigitalPin
    from robo_infra.core.sensor import Sensor


logger = logging.getLogger(__name__)


class StepperConfig(BaseModel):
    """Configuration model for stepper motors."""

    name: str = "Stepper"

    # IO: either pin objects (passed directly to Stepper) or driver channels here
    step_channel: int | None = None
    dir_channel: int | None = None
    enable_channel: int | None = None

    steps_per_rev: int = Field(default=200, ge=1)
    microsteps: int = Field(default=1, ge=1)

    max_speed: float = Field(default=2000.0, gt=0)  # steps/sec
    acceleration: float = Field(default=0.0, ge=0)  # steps/sec^2 (stored; not enforced)

    # Optional position bounds (in steps)
    min_position: int = -1_000_000
    max_position: int = 1_000_000

    # Enable semantics
    enable_active_high: bool = True

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class StepperStatus:
    """Status information for a stepper motor."""

    state: ActuatorState = ActuatorState.DISABLED
    position: int = 0
    target_position: int | None = None
    direction: Direction = Direction.STOP
    is_enabled: bool = False
    is_running: bool = False
    error: str | None = None


class Stepper(Actuator):
    """Open-loop stepper motor actuator."""

    def __init__(
        self,
        step_pin: DigitalPin | int,
        dir_pin: DigitalPin | int,
        enable_pin: DigitalPin | int | None = None,
        driver: Driver | None = None,
        *,
        name: str = "Stepper",
        steps_per_rev: int = 200,
        microsteps: int = 1,
        max_speed: float = 2000.0,
        acceleration: float = 0.0,
        enable_active_high: bool = True,
        config: StepperConfig | None = None,
    ) -> None:
        if config is not None:
            self._stepper_config = config
            name = config.name
            steps_per_rev = config.steps_per_rev
            microsteps = config.microsteps
            max_speed = config.max_speed
            acceleration = config.acceleration
            enable_active_high = config.enable_active_high
        else:
            self._stepper_config = StepperConfig(
                name=name,
                step_channel=step_pin if isinstance(step_pin, int) else None,
                dir_channel=dir_pin if isinstance(dir_pin, int) else None,
                enable_channel=enable_pin if isinstance(enable_pin, int) else None,
                steps_per_rev=steps_per_rev,
                microsteps=microsteps,
                max_speed=max_speed,
                acceleration=acceleration,
                enable_active_high=enable_active_high,
            )

        limits = Limits(
            min=float(self._stepper_config.min_position),
            max=float(self._stepper_config.max_position),
            default=0.0,
        )

        actuator_config = ActuatorConfig(
            name=self._stepper_config.name,
            actuator_type=ActuatorType.STEPPER,
            channel=0,
            limits=limits,
            unit="count",
            require_calibration=False,
        )

        super().__init__(driver=driver, config=actuator_config)

        # IO references
        self._step_pin_obj: DigitalPin | None = step_pin if not isinstance(step_pin, int) else None
        self._dir_pin_obj: DigitalPin | None = dir_pin if not isinstance(dir_pin, int) else None
        self._enable_pin_obj: DigitalPin | None = (
            enable_pin if (enable_pin is not None and not isinstance(enable_pin, int)) else None
        )

        self._step_ch: int | None = step_pin if isinstance(step_pin, int) else None
        self._dir_ch: int | None = dir_pin if isinstance(dir_pin, int) else None
        self._enable_ch: int | None = enable_pin if isinstance(enable_pin, int) else None

        # Motion state
        self._position_steps: int = 0
        self._target_steps: int | None = None
        self._direction: Direction = Direction.STOP

        # Config properties
        self._steps_per_rev = steps_per_rev
        self._microsteps = microsteps
        self._max_speed = float(max_speed)
        self._acceleration = float(acceleration)
        self._speed = self._max_speed
        self._stop_requested = False
        self._enable_active_high = enable_active_high

    # ---------------------------------------------------------------------
    # Properties
    # ---------------------------------------------------------------------

    @property
    def position(self) -> int:
        """Current step position."""
        return self._position_steps

    @property
    def steps_per_rev(self) -> int:
        """Steps per revolution (full-step count)."""
        return self._steps_per_rev

    @property
    def microsteps(self) -> int:
        """Microstepping divisor."""
        return self._microsteps

    @property
    def max_speed(self) -> float:
        """Maximum speed in steps/sec."""
        return self._max_speed

    @property
    def acceleration(self) -> float:
        """Acceleration in steps/sec^2 (stored, not enforced)."""
        return self._acceleration

    @property
    def is_running(self) -> bool:
        return self._state == ActuatorState.MOVING

    @property
    def direction(self) -> Direction:
        return self._direction

    # ---------------------------------------------------------------------
    # Public methods (Phase 3.3)
    # ---------------------------------------------------------------------

    def set_speed(self, speed: float) -> None:
        """Set stepping speed (steps/sec), clamped to max_speed."""
        self._speed = max(1.0, min(self._max_speed, float(speed)))

    def step(self, steps: int, direction: Direction) -> None:
        """Step the motor a number of steps in a given direction."""
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")
        if steps < 0:
            raise ValueError("steps must be >= 0")
        if steps == 0:
            self._direction = Direction.STOP
            return

        if direction not in (Direction.FORWARD, Direction.REVERSE):
            raise ValueError("direction must be FORWARD or REVERSE")

        self._stop_requested = False
        self._direction = direction
        self._state = ActuatorState.MOVING

        self._write_dir(direction)

        step_delta = steps if direction == Direction.FORWARD else -steps
        target = self._position_steps + step_delta

        # Enforce position limits (integer check)
        if target < int(self.limits.min) or target > int(self.limits.max):
            raise SafetyError(
                "Stepper target exceeds position limits",
                action_taken="refuse movement",
            )

        for _ in range(steps):
            if self._stop_requested:
                break
            self._pulse_step()
            self._position_steps += 1 if direction == Direction.FORWARD else -1

        self._current_value = float(self._position_steps)
        self._state = ActuatorState.HOLDING
        self._direction = Direction.STOP

    def move_by(self, steps: int) -> None:
        """Move relative by N steps (positive forward, negative reverse)."""
        if steps == 0:
            self.stop()
            return
        direction = Direction.FORWARD if steps > 0 else Direction.REVERSE
        self.step(abs(steps), direction)

    def move_to(self, position: int) -> None:
        """Move to an absolute step position."""
        delta = position - self._position_steps
        self._target_steps = position
        self.move_by(delta)
        self._target_steps = None

    def home(self, switch: Sensor) -> None:
        """Home the stepper using a switch sensor.

        This is an open-loop homing routine:
        - If switch already triggered, set position=0.
        - Otherwise, step in REVERSE until triggered or a safety limit reached.
        """
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        def read_switch() -> bool:
            """Read switch state with duck typing support."""
            reading = switch.read()
            try:
                # Try Reading object with value attribute
                return float(reading.value) >= 0.5
            except AttributeError:
                # Fallback for sensors returning floats directly
                return float(reading) >= 0.5  # type: ignore[arg-type]

        # If already triggered, just zero.
        if read_switch():
            self._position_steps = 0
            self._current_value = 0.0
            return

        max_steps = self._steps_per_rev * self._microsteps

        for _ in range(max_steps):
            if self._stop_requested:
                break
            self.step(1, Direction.REVERSE)
            if read_switch():
                self._position_steps = 0
                self._current_value = 0.0
                return

        raise SafetyError("Homing switch not triggered", action_taken="abort homing")

    def stop(self) -> None:
        """Stop motion as soon as possible."""
        self._stop_requested = True
        self._direction = Direction.STOP
        self._state = ActuatorState.IDLE if self._is_enabled else ActuatorState.DISABLED

    def disable(self) -> None:
        """Disable stepper and release holding torque."""
        try:
            self._set_enable(False)
        except Exception as e:
            logger.debug("Failed to set enable pin while disabling %s: %s", self.name, e)
        super().disable()

    # ---------------------------------------------------------------------
    # Overrides / Actuator hooks
    # ---------------------------------------------------------------------

    def enable(self) -> None:
        super().enable()

        for pin in (self._step_pin_obj, self._dir_pin_obj, self._enable_pin_obj):
            if pin is not None and not pin.initialized:
                pin.setup()

        self._set_enable(True)
        self.coast()

    def coast(self) -> None:
        """Idle state: no stepping, direction STOP."""
        self._direction = Direction.STOP
        self._state = ActuatorState.IDLE

    def status(self) -> StepperStatus:  # type: ignore[override]
        return StepperStatus(
            state=self._state,
            position=self._position_steps,
            target_position=self._target_steps,
            direction=self._direction,
            is_enabled=self._is_enabled,
            is_running=self.is_running,
            error=self._error,
        )

    def _apply_value(self, value: float) -> None:
        # Interpret Actuator.set(value) as move_to(step_position)
        self.move_to(round(value))

    def _read_value(self) -> float:
        return float(self._position_steps)

    # ---------------------------------------------------------------------
    # IO helpers
    # ---------------------------------------------------------------------

    def _set_enable(self, enabled: bool) -> None:
        if self._enable_pin_obj is not None:
            pin_value = enabled if self._enable_active_high else not enabled
            self._enable_pin_obj.write(pin_value)
        elif self._driver is not None and self._enable_ch is not None:
            channel_value = 1.0 if (enabled if self._enable_active_high else not enabled) else 0.0
            self._driver.set_channel(self._enable_ch, channel_value)

    def _write_dir(self, direction: Direction) -> None:
        is_forward = direction == Direction.FORWARD
        if self._dir_pin_obj is not None:
            self._dir_pin_obj.write(is_forward)
        elif self._driver is not None and self._dir_ch is not None:
            self._driver.set_channel(self._dir_ch, 1.0 if is_forward else 0.0)

    def _pulse_step(self) -> None:
        if self._step_pin_obj is not None:
            self._step_pin_obj.write(True)
            self._step_pin_obj.write(False)
        elif self._driver is not None and self._step_ch is not None:
            self._driver.set_channel(self._step_ch, 1.0)
            self._driver.set_channel(self._step_ch, 0.0)


class SimulatedStepper(Stepper):
    """Stepper simulation helper using simulated pins."""

    def __init__(
        self,
        *,
        name: str = "SimulatedStepper",
        steps_per_rev: int = 200,
        microsteps: int = 1,
        max_speed: float = 2000.0,
        acceleration: float = 0.0,
        **kwargs: Any,
    ) -> None:
        from robo_infra.core.pin import SimulatedDigitalPin

        super().__init__(
            step_pin=SimulatedDigitalPin(1),
            dir_pin=SimulatedDigitalPin(2),
            enable_pin=SimulatedDigitalPin(3),
            driver=None,
            name=name,
            steps_per_rev=steps_per_rev,
            microsteps=microsteps,
            max_speed=max_speed,
            acceleration=acceleration,
            **kwargs,
        )


def create_stepper(
    *,
    step_pin: DigitalPin | int,
    dir_pin: DigitalPin | int,
    enable_pin: DigitalPin | int | None = None,
    driver: Driver | None = None,
    simulated: bool = False,
    name: str = "Stepper",
    **kwargs: Any,
) -> Stepper:
    """Factory function for steppers."""
    if simulated:
        return SimulatedStepper(name=name, **kwargs)
    return Stepper(
        step_pin=step_pin,
        dir_pin=dir_pin,
        enable_pin=enable_pin,
        driver=driver,
        name=name,
        **kwargs,
    )


__all__ = [
    "SimulatedStepper",
    "Stepper",
    "StepperConfig",
    "StepperStatus",
    "create_stepper",
]
