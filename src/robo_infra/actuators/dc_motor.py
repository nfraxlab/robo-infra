"""DC motor actuator implementation.

This module provides a `DCMotor` actuator for controlling a brushed DC motor
via a typical H-bridge interface:

- Two digital direction pins (`pin_a`, `pin_b`)
- One PWM enable pin (`enable`) to control speed

It supports two wiring/control styles:

1) Pin-based control (recommended for real GPIO):
   - `pin_a` / `pin_b` are `DigitalPin` instances
   - `enable` is a `PWMPin` instance

2) Driver-channel control (useful for simulation or driver-backed IO):
   - `pin_a` / `pin_b` / `enable` are integers (driver channel numbers)
   - `driver` is a `Driver` instance

Speed is expressed as a float in [-1.0, 1.0]:
- Positive: forward
- Negative: reverse
- Zero: stop
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator, ActuatorConfig, ActuatorState, ActuatorType
from robo_infra.core.exceptions import DisabledError
from robo_infra.core.types import Direction, Limits


if TYPE_CHECKING:
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import DigitalPin, PWMPin


logger = logging.getLogger(__name__)


class DCMotorConfig(BaseModel):
    """Configuration model for DC motors.

    Attributes:
        name: Human-readable name.
        pin_a: Direction pin A (DigitalPin) or driver channel (int).
        pin_b: Direction pin B (DigitalPin) or driver channel (int).
        enable: Enable/PWM pin (PWMPin) or driver channel (int).
        inverted: If True, swap forward/reverse semantics.
        pwm_frequency: PWM frequency in Hz (only used for pin-based PWM).
        metadata: Additional configuration.
    """

    name: str = "DCMotor"
    pin_a: int | None = None
    pin_b: int | None = None
    enable: int | None = None
    inverted: bool = False
    pwm_frequency: int = 1000
    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class DCMotorStatus:
    """Status information for a DC motor."""

    state: ActuatorState = ActuatorState.DISABLED
    speed: float = 0.0
    direction: Direction = Direction.STOP
    is_enabled: bool = False
    is_running: bool = False
    error: str | None = None


class DCMotor(Actuator):
    """Brushed DC motor actuator.

    Implements a simple H-bridge style interface:
    - Direction pins: A/B
    - PWM enable: controls speed

    Notes:
        - This class is intentionally driver-agnostic.
        - It can operate with GPIO pins (DigitalPin/PWMPin) or with a Driver via channels.
    """

    def __init__(
        self,
        pin_a: DigitalPin | int | None = None,
        pin_b: DigitalPin | int | None = None,
        enable: PWMPin | int | None = None,
        driver: Driver | None = None,
        *,
        name: str = "DCMotor",
        inverted: bool = False,
        pwm_frequency: int = 1000,
        config: DCMotorConfig | None = None,
    ) -> None:
        if config is not None:
            self._motor_config = config
            inverted = config.inverted
            pwm_frequency = config.pwm_frequency
        else:
            self._motor_config = DCMotorConfig(
                name=name,
                pin_a=pin_a if isinstance(pin_a, int) else None,
                pin_b=pin_b if isinstance(pin_b, int) else None,
                enable=enable if isinstance(enable, int) else None,
                inverted=inverted,
                pwm_frequency=pwm_frequency,
            )

        actuator_config = ActuatorConfig(
            name=self._motor_config.name,
            actuator_type=ActuatorType.DC_MOTOR,
            channel=0,
            limits=Limits(min=-1.0, max=1.0, default=0.0),
            unit="ratio",
            inverted=False,  # inversion handled at motor-direction level
            offset=0.0,
            scale=1.0,
            require_calibration=False,
        )

        super().__init__(driver=driver, config=actuator_config)

        self._pin_a_obj: DigitalPin | None = pin_a if not isinstance(pin_a, int) else None
        self._pin_b_obj: DigitalPin | None = pin_b if not isinstance(pin_b, int) else None
        self._enable_obj: PWMPin | None = enable if not isinstance(enable, int) else None

        self._pin_a_ch: int | None = pin_a if isinstance(pin_a, int) else None
        self._pin_b_ch: int | None = pin_b if isinstance(pin_b, int) else None
        self._enable_ch: int | None = enable if isinstance(enable, int) else None

        self._direction = Direction.STOP
        self._pwm_frequency = pwm_frequency
        self._inverted = inverted

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def speed(self) -> float:
        """Current motor speed in [-1.0, 1.0]."""
        return self._current_value

    @property
    def direction(self) -> Direction:
        """Current motor direction."""
        return self._direction

    @property
    def is_running(self) -> bool:
        """Whether the motor is currently running (non-zero speed)."""
        return abs(self._current_value) > 0.0

    @property
    def motor_config(self) -> DCMotorConfig:
        """Full motor configuration."""
        return self._motor_config

    # -------------------------------------------------------------------------
    # Convenience methods
    # -------------------------------------------------------------------------

    def forward(self, speed: float = 1.0) -> None:
        """Run forward at a given speed (0..1)."""
        speed = max(0.0, min(1.0, float(speed)))
        self.set(speed)

    def reverse(self, speed: float = 1.0) -> None:
        """Run reverse at a given speed (0..1)."""
        speed = max(0.0, min(1.0, float(speed)))
        self.set(-speed)

    def stop(self) -> None:
        """Stop the motor (speed=0) and set pins to a safe state."""
        self.set(0.0)

    def brake(self) -> None:
        """Active braking.

        Many H-bridges brake when both inputs are driven HIGH.
        We also set PWM duty to 0.
        """
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        self._direction = Direction.STOP
        self._write_direction(True, True)
        self._write_pwm(0.0)
        self._current_value = 0.0
        self._target_value = 0.0
        self._state = ActuatorState.HOLDING

    def coast(self) -> None:
        """Passive stop (coast).

        Many H-bridges coast when both inputs are driven LOW.
        We also set PWM duty to 0.
        """
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        self._direction = Direction.STOP
        self._write_direction(False, False)
        self._write_pwm(0.0)
        self._current_value = 0.0
        self._target_value = 0.0
        self._state = ActuatorState.HOLDING

    # -------------------------------------------------------------------------
    # Overrides
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        super().enable()

        # Pin-based PWM setup (optional)
        if self._enable_obj is not None:
            if not self._enable_obj.initialized:
                self._enable_obj.setup()
            try:
                # Some platforms may not support dynamic frequency changes.
                self._enable_obj.set_frequency(self._pwm_frequency)
            except Exception as e:
                logger.debug("PWM frequency change not supported: %s", e)

        for pin in (self._pin_a_obj, self._pin_b_obj):
            if pin is not None and not pin.initialized:
                pin.setup()

        # Ensure safe state
        self.coast()

    def disable(self) -> None:
        # Try to stop outputs even if something goes wrong
        try:
            if self._is_enabled:
                self.coast()
        except Exception as e:
            logger.debug("Error while disabling %s: %s", self.name, e)
        super().disable()

    def status(self) -> DCMotorStatus:  # type: ignore[override]
        return DCMotorStatus(
            state=self._state,
            speed=self._current_value,
            direction=self._direction,
            is_enabled=self._is_enabled,
            is_running=self.is_running,
            error=self._error,
        )

    # -------------------------------------------------------------------------
    # Actuator abstract methods
    # -------------------------------------------------------------------------

    def _apply_value(self, value: float) -> None:
        """Apply motor speed to hardware.

        Args:
            value: Speed in [-1.0, 1.0].
        """
        if not self._is_enabled:
            raise DisabledError(f"Actuator {self.name} is disabled")

        speed = float(value)

        if speed == 0.0:
            self._direction = Direction.STOP
            self._write_direction(False, False)
            self._write_pwm(0.0)
            return

        direction = Direction.FORWARD if speed > 0 else Direction.REVERSE
        if self._inverted:
            direction = Direction.REVERSE if direction == Direction.FORWARD else Direction.FORWARD

        self._direction = direction
        duty = abs(speed)

        if direction == Direction.FORWARD:
            self._write_direction(True, False)
        else:
            self._write_direction(False, True)

        self._write_pwm(duty)

    def _read_value(self) -> float:
        """Read motor speed.

        Most DC motor setups don't provide speed feedback, so return the last command.
        """
        return self._current_value

    # -------------------------------------------------------------------------
    # IO helpers
    # -------------------------------------------------------------------------

    def _write_direction(self, a_high: bool, b_high: bool) -> None:
        if self._pin_a_obj is not None:
            self._pin_a_obj.write(a_high)
        elif self._driver is not None and self._pin_a_ch is not None:
            self._driver.set_channel(self._pin_a_ch, 1.0 if a_high else 0.0)

        if self._pin_b_obj is not None:
            self._pin_b_obj.write(b_high)
        elif self._driver is not None and self._pin_b_ch is not None:
            self._driver.set_channel(self._pin_b_ch, 1.0 if b_high else 0.0)

    def _write_pwm(self, duty: float) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        if self._enable_obj is not None:
            self._enable_obj.set_duty_cycle(duty)
        elif self._driver is not None and self._enable_ch is not None:
            self._driver.set_channel(self._enable_ch, duty)


class SimulatedDCMotor(DCMotor):
    """DC motor simulation helper.

    Uses simulated pins by default.
    """

    def __init__(
        self,
        *,
        name: str = "SimulatedDCMotor",
        inverted: bool = False,
        pwm_frequency: int = 1000,
        **kwargs: Any,
    ) -> None:
        from robo_infra.core.pin import SimulatedDigitalPin, SimulatedPWMPin

        pin_a = SimulatedDigitalPin(1)
        pin_b = SimulatedDigitalPin(2)
        enable = SimulatedPWMPin(3, frequency=pwm_frequency)

        super().__init__(
            pin_a=pin_a,
            pin_b=pin_b,
            enable=enable,
            driver=None,
            name=name,
            inverted=inverted,
            pwm_frequency=pwm_frequency,
            **kwargs,
        )


def create_dc_motor(
    *,
    name: str = "DCMotor",
    pin_a: DigitalPin | int | None = None,
    pin_b: DigitalPin | int | None = None,
    enable: PWMPin | int | None = None,
    driver: Driver | None = None,
    simulated: bool = False,
    **kwargs: Any,
) -> DCMotor:
    """Factory function for DC motors."""
    if simulated:
        return SimulatedDCMotor(name=name, **kwargs)

    return DCMotor(
        pin_a=pin_a,
        pin_b=pin_b,
        enable=enable,
        driver=driver,
        name=name,
        **kwargs,
    )


__all__ = [
    "DCMotor",
    "DCMotorConfig",
    "DCMotorStatus",
    "SimulatedDCMotor",
    "create_dc_motor",
]
