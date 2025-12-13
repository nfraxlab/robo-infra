"""Linear actuator implementation.

A `LinearActuator` models devices that extend/retract along a line.

This implementation supports two common styles:

1) Motor-based linear actuator
   - A `DCMotor` drives the actuator forward (extend) and reverse (retract)
   - Optionally use a position `Sensor` for feedback

2) Solenoid-based (binary) actuator
   - A `Solenoid`/`Relay` controls extend/retract (on=extend, off=retract)

Phase 3.5 requirements:
- `LinearActuator` class extending `Actuator`
- support for motor-based and solenoid-based
- methods: extend, retract, move_to (if feedback), stop
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.actuators.dc_motor import DCMotor, SimulatedDCMotor
from robo_infra.actuators.solenoid import SimulatedSolenoid, Solenoid
from robo_infra.core.actuator import Actuator, ActuatorConfig, ActuatorState, ActuatorType
from robo_infra.core.exceptions import DisabledError, SafetyError
from robo_infra.core.types import Limits


if TYPE_CHECKING:
    from robo_infra.core.sensor import Sensor


logger = logging.getLogger(__name__)


class LinearActuatorConfig(BaseModel):
    """Configuration model for linear actuators."""

    name: str = "LinearActuator"
    # value semantics: position ratio in [0, 1]
    min_position: float = 0.0
    max_position: float = 1.0
    default_position: float = 0.0

    # Feedback
    has_feedback: bool = False
    tolerance: float = 0.02

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class LinearActuatorStatus:
    """Status information for a linear actuator."""

    state: ActuatorState = ActuatorState.DISABLED
    position: float = 0.0
    target: float | None = None
    is_enabled: bool = False
    is_extended: bool = False
    error: str | None = None


class LinearActuator(Actuator):
    """Linear actuator composed from either a motor or a solenoid."""

    def __init__(
        self,
        *,
        motor: DCMotor | None = None,
        solenoid: Solenoid | None = None,
        position_sensor: Sensor | None = None,
        name: str = "LinearActuator",
        config: LinearActuatorConfig | None = None,
    ) -> None:
        if (motor is None) == (solenoid is None):
            raise ValueError("Provide exactly one of motor= or solenoid=")

        if config is not None:
            self._linear_config = config
            name = config.name
        else:
            self._linear_config = LinearActuatorConfig(name=name)

        limits = Limits(
            min=float(self._linear_config.min_position),
            max=float(self._linear_config.max_position),
            default=float(self._linear_config.default_position),
        )

        actuator_config = ActuatorConfig(
            name=self._linear_config.name,
            actuator_type=ActuatorType.LINEAR,
            channel=0,
            limits=limits,
            unit="ratio",
            require_calibration=False,
        )

        super().__init__(driver=None, config=actuator_config)

        self._motor = motor
        self._solenoid = solenoid
        self._position_sensor = position_sensor
        self._tolerance = float(self._linear_config.tolerance)

        default_position = limits.default if limits.default is not None else limits.min
        self._position_estimate = float(default_position)
        self._target: float | None = None

        if self._position_sensor is not None:
            self._linear_config.has_feedback = True

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def position(self) -> float:
        """Current position ratio in [0, 1]."""
        if self._position_sensor is not None:
            reading = self._position_sensor.read()
            try:
                # Try Reading object with value attribute
                return float(reading.value)
            except AttributeError:
                # Fallback for sensors returning floats directly
                return float(reading)  # type: ignore[arg-type]
        return self._position_estimate

    @property
    def is_extended(self) -> bool:
        return self.position >= (self.limits.max - self._tolerance)

    # ------------------------------------------------------------------
    # Public API (Phase 3.5)
    # ------------------------------------------------------------------

    def extend(self, speed: float = 1.0) -> None:
        """Extend the actuator."""
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        speed = max(0.0, min(1.0, float(speed)))

        if self._motor is not None:
            self._motor.set(speed)
        elif self._solenoid is not None:
            self._solenoid.activate()

        self._position_estimate = float(self.limits.max)
        self._current_value = self._position_estimate
        self._state = ActuatorState.MOVING

    def retract(self, speed: float = 1.0) -> None:
        """Retract the actuator."""
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        speed = max(0.0, min(1.0, float(speed)))

        if self._motor is not None:
            self._motor.set(-speed)
        elif self._solenoid is not None:
            self._solenoid.deactivate()

        self._position_estimate = float(self.limits.min)
        self._current_value = self._position_estimate
        self._state = ActuatorState.MOVING

    def move_to(self, position: float) -> None:
        """Move to an absolute position ratio.

        If a position sensor is configured, this method uses a simple closed-loop
        strategy (bang-bang) with a safety iteration limit.

        Without feedback, we update an internal estimate and command motion.
        """
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        target = max(self.limits.min, min(self.limits.max, float(position)))
        self._target = target
        self._target_value = target

        if self._position_sensor is None:
            # Open-loop: command direction and update estimate immediately.
            if target >= self.position:
                self.extend(speed=1.0)
            else:
                self.retract(speed=1.0)
            self.stop()
            self._position_estimate = target
            self._current_value = target
            self._state = ActuatorState.HOLDING
            self._target = None
            return

        # Feedback mode: iterate a bounded number of steps
        max_iters = 200
        for _ in range(max_iters):
            current = self.position
            error = target - current
            if abs(error) <= self._tolerance:
                break
            if error > 0:
                self.extend(speed=1.0)
            else:
                self.retract(speed=1.0)

        self.stop()

        # Re-check final position and fail if not reached.
        if abs(target - self.position) > (5 * self._tolerance):
            raise SafetyError("Failed to reach target position", action_taken="stop")

        self._position_estimate = target
        self._current_value = target
        self._state = ActuatorState.HOLDING
        self._target = None

    def stop(self) -> None:
        """Stop actuator motion."""
        if self._motor is not None:
            try:
                self._motor.stop()
            except Exception as e:
                logger.error("Failed to stop motor: %s", e)
        if self._solenoid is not None:
            # safest default: deactivate
            try:
                self._solenoid.deactivate()
            except Exception as e:
                logger.error("Failed to deactivate solenoid: %s", e)

        self._state = ActuatorState.IDLE if self._is_enabled else ActuatorState.DISABLED

    def status(self) -> LinearActuatorStatus:  # type: ignore[override]
        return LinearActuatorStatus(
            state=self._state,
            position=self.position,
            target=self._target,
            is_enabled=self._is_enabled,
            is_extended=self.is_extended,
            error=self._error,
        )

    # ------------------------------------------------------------------
    # Actuator hooks
    # ------------------------------------------------------------------

    def enable(self) -> None:
        super().enable()

        if self._motor is not None:
            self._motor.enable()
        if self._solenoid is not None:
            self._solenoid.enable()
        if self._position_sensor is not None:
            self._position_sensor.enable()

        self._state = ActuatorState.IDLE

    def disable(self) -> None:
        # Stop motion first
        try:
            self.stop()
        except Exception as e:
            logger.error("Failed to stop during disable: %s", e)

        # Disable sub-components, tracking failures
        failures: list[str] = []

        if self._motor is not None:
            try:
                self._motor.disable()
            except Exception as e:
                failures.append(f"motor: {e}")
                logger.error("Failed to disable motor: %s", e)

        if self._solenoid is not None:
            try:
                self._solenoid.disable()
            except Exception as e:
                failures.append(f"solenoid: {e}")
                logger.error("Failed to disable solenoid: %s", e)

        if self._position_sensor is not None:
            try:
                self._position_sensor.disable()
            except Exception as e:
                # Sensor disable is non-critical
                logger.debug("Failed to disable position sensor: %s", e)

        super().disable()

        if failures:
            logger.warning(
                "Linear actuator %s disable completed with failures: %s",
                self.name,
                failures,
            )

    def _apply_value(self, value: float) -> None:
        # Interpret Actuator.set(value) as position command
        self.move_to(float(value))

    def _read_value(self) -> float:
        return float(self.position)


class SimulatedLinearActuator(LinearActuator):
    """Simulated linear actuator (motor-based by default)."""

    def __init__(
        self, *, name: str = "SimulatedLinearActuator", solenoid_mode: bool = False, **kwargs: Any
    ):
        if solenoid_mode:
            super().__init__(name=name, solenoid=SimulatedSolenoid(), **kwargs)
        else:
            super().__init__(name=name, motor=SimulatedDCMotor(), **kwargs)


def create_linear_actuator(
    *,
    name: str = "LinearActuator",
    motor: DCMotor | None = None,
    solenoid: Solenoid | None = None,
    position_sensor: Sensor | None = None,
    simulated: bool = False,
    **kwargs: Any,
) -> LinearActuator:
    """Factory function for linear actuators."""
    if simulated:
        return SimulatedLinearActuator(name=name, **kwargs)
    return LinearActuator(
        name=name,
        motor=motor,
        solenoid=solenoid,
        position_sensor=position_sensor,
        **kwargs,
    )


__all__ = [
    "LinearActuator",
    "LinearActuatorConfig",
    "LinearActuatorStatus",
    "SimulatedLinearActuator",
    "create_linear_actuator",
]
