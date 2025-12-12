"""Servo motor actuator implementation.

This module provides a Servo actuator that extends the base Actuator class
to provide servo-specific functionality like angle control, pulse width
mapping, and sweep operations.

Example:
    >>> from robo_infra.actuators.servo import Servo, ServoConfig
    >>> from robo_infra.core.driver import SimulatedDriver
    >>>
    >>> # Create a simulated servo
    >>> servo = Servo(name="shoulder", angle_range=(0, 180))
    >>> servo.enable()
    >>> servo.set(90)  # Set to 90 degrees
    >>> print(servo.angle)
    90.0
    >>>
    >>> # With a real driver
    >>> driver = SimulatedDriver(channels=16)
    >>> driver.connect()
    >>> driver.enable()
    >>> servo = Servo(
    ...     name="elbow",
    ...     driver=driver,
    ...     channel=0,
    ...     angle_range=(0, 180),
    ...     pulse_range=(500, 2500),
    ... )
    >>> servo.enable()
    >>> servo.set(45)
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator, ActuatorConfig, ActuatorState, ActuatorType
from robo_infra.core.exceptions import (
    CalibrationError,
    DisabledError,
    LimitsExceededError,
)
from robo_infra.core.types import Limits


if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from robo_infra.core.driver import Driver


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================


# Standard servo pulse widths (microseconds)
STANDARD_PULSE_MIN = 500  # 0.5ms
STANDARD_PULSE_MAX = 2500  # 2.5ms
STANDARD_PULSE_CENTER = 1500  # 1.5ms

# Common servo frequencies
STANDARD_FREQUENCY = 50  # 50Hz is standard for most servos
HIGH_FREQUENCY = 333  # For digital servos


# =============================================================================
# Enums
# =============================================================================


class ServoType(Enum):
    """Types of servo motors."""

    STANDARD = "standard"  # 180 degree range
    CONTINUOUS = "continuous"  # 360 degree continuous rotation
    LINEAR = "linear"  # Linear actuator
    SAIL_WINCH = "sail_winch"  # Multi-turn servo


class ServoRange(Enum):
    """Common servo angle ranges."""

    RANGE_90 = (0, 90)
    RANGE_180 = (0, 180)
    RANGE_270 = (0, 270)
    RANGE_360 = (0, 360)


# =============================================================================
# Configuration Models
# =============================================================================


class ServoConfig(BaseModel):
    """Configuration model for Servo actuators.

    Attributes:
        name: Human-readable servo name.
        channel: Driver channel number.
        angle_range: Min and max angle in degrees.
        pulse_range: Min and max pulse width in microseconds.
        frequency: PWM frequency in Hz.
        servo_type: Type of servo motor.
        inverted: If True, invert the angle.
        offset: Offset angle in degrees.
        speed_deg_per_sec: Maximum speed in degrees per second.
        require_calibration: Whether calibration is required.
        trim: Fine-tune center position (microseconds).
        metadata: Additional servo-specific data.
    """

    name: str = "Servo"
    channel: int = 0
    angle_range: tuple[float, float] = (0.0, 180.0)
    pulse_range: tuple[int, int] = (STANDARD_PULSE_MIN, STANDARD_PULSE_MAX)
    frequency: int = STANDARD_FREQUENCY
    servo_type: ServoType = ServoType.STANDARD
    inverted: bool = False
    offset: float = 0.0
    speed_deg_per_sec: float | None = None
    require_calibration: bool = False
    trim: int = 0
    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}

    @property
    def angle_min(self) -> float:
        """Minimum angle."""
        return self.angle_range[0]

    @property
    def angle_max(self) -> float:
        """Maximum angle."""
        return self.angle_range[1]

    @property
    def pulse_min(self) -> int:
        """Minimum pulse width (microseconds)."""
        return self.pulse_range[0]

    @property
    def pulse_max(self) -> int:
        """Maximum pulse width (microseconds)."""
        return self.pulse_range[1]

    @property
    def angle_span(self) -> float:
        """Total angle range."""
        return self.angle_max - self.angle_min

    @property
    def pulse_span(self) -> int:
        """Total pulse range (microseconds)."""
        return self.pulse_max - self.pulse_min


@dataclass
class ServoStatus:
    """Status information for a servo.

    Attributes:
        state: Current servo state.
        angle: Current angle in degrees.
        target_angle: Target angle if moving.
        pulse_width: Current pulse width in microseconds.
        is_enabled: Whether servo is enabled.
        is_calibrated: Whether servo is calibrated.
        error: Error message if in error state.
    """

    state: ActuatorState = ActuatorState.DISABLED
    angle: float = 0.0
    target_angle: float | None = None
    pulse_width: int = STANDARD_PULSE_CENTER
    is_enabled: bool = False
    is_calibrated: bool = False
    error: str | None = None


# =============================================================================
# Servo Implementation
# =============================================================================


class Servo(Actuator):
    """Servo motor actuator.

    A servo is an actuator that can be commanded to specific angles within
    a defined range. This class handles the conversion between angles and
    PWM pulse widths.

    The servo uses the driver's `set_channel()` method to output PWM signals.
    The value sent to the driver is normalized (0.0 to 1.0) where:
    - 0.0 = minimum pulse width = minimum angle
    - 1.0 = maximum pulse width = maximum angle

    Attributes:
        angle: Current angle in degrees.
        pulse_width: Current pulse width in microseconds.
        frequency: PWM frequency in Hz.
    """

    def __init__(
        self,
        name: str = "Servo",
        driver: Driver | None = None,
        channel: int = 0,
        angle_range: tuple[float, float] = (0.0, 180.0),
        pulse_range: tuple[int, int] = (STANDARD_PULSE_MIN, STANDARD_PULSE_MAX),
        frequency: int = STANDARD_FREQUENCY,
        servo_type: ServoType = ServoType.STANDARD,
        inverted: bool = False,
        offset: float = 0.0,
        speed_deg_per_sec: float | None = None,
        config: ServoConfig | None = None,
    ) -> None:
        """Initialize the servo.

        Args:
            name: Human-readable name.
            driver: Driver to use for PWM output.
            channel: Channel on the driver.
            angle_range: Min and max angle in degrees.
            pulse_range: Min and max pulse width in microseconds.
            frequency: PWM frequency in Hz.
            servo_type: Type of servo motor.
            inverted: If True, invert the angle direction.
            offset: Offset angle in degrees.
            speed_deg_per_sec: Maximum speed (for timed moves).
            config: Full configuration (overrides other args).
        """
        # Build servo config
        if config:
            self._servo_config = config
        else:
            self._servo_config = ServoConfig(
                name=name,
                channel=channel,
                angle_range=angle_range,
                pulse_range=pulse_range,
                frequency=frequency,
                servo_type=servo_type,
                inverted=inverted,
                offset=offset,
                speed_deg_per_sec=speed_deg_per_sec,
            )

        # Create base actuator config
        limits = Limits(
            min=self._servo_config.angle_min,
            max=self._servo_config.angle_max,
            default=(self._servo_config.angle_min + self._servo_config.angle_max) / 2,
        )

        actuator_config = ActuatorConfig(
            name=self._servo_config.name,
            actuator_type=ActuatorType.SERVO,
            channel=self._servo_config.channel,
            limits=limits,
            unit="degrees",
            inverted=self._servo_config.inverted,
            offset=self._servo_config.offset,
            speed_limit=self._servo_config.speed_deg_per_sec,
            require_calibration=self._servo_config.require_calibration,
        )

        super().__init__(
            driver=driver,
            config=actuator_config,
        )

        # Servo-specific state
        self._pulse_width = STANDARD_PULSE_CENTER
        self._calibration_min_pulse = self._servo_config.pulse_min
        self._calibration_max_pulse = self._servo_config.pulse_max
        self._trim = self._servo_config.trim

    # -------------------------------------------------------------------------
    # Servo-Specific Properties
    # -------------------------------------------------------------------------

    @property
    def angle(self) -> float:
        """Current angle in degrees."""
        return self._current_value

    @property
    def target_angle(self) -> float | None:
        """Target angle if moving, else None."""
        return self._target_value

    @property
    def pulse_width(self) -> int:
        """Current pulse width in microseconds."""
        return self._pulse_width

    @property
    def frequency(self) -> int:
        """PWM frequency in Hz."""
        return self._servo_config.frequency

    @property
    def angle_range(self) -> tuple[float, float]:
        """Min and max angle in degrees."""
        return self._servo_config.angle_range

    @property
    def pulse_range(self) -> tuple[int, int]:
        """Current calibrated pulse range in microseconds."""
        return (self._calibration_min_pulse, self._calibration_max_pulse)

    @property
    def servo_type(self) -> ServoType:
        """Type of servo motor."""
        return self._servo_config.servo_type

    @property
    def servo_config(self) -> ServoConfig:
        """Full servo configuration."""
        return self._servo_config

    @property
    def trim(self) -> int:
        """Trim adjustment in microseconds."""
        return self._trim

    @trim.setter
    def trim(self, value: int) -> None:
        """Set trim adjustment."""
        self._trim = value

    # -------------------------------------------------------------------------
    # Angle/Pulse Conversion
    # -------------------------------------------------------------------------

    def angle_to_pulse(self, angle: float) -> int:
        """Convert angle to pulse width.

        Args:
            angle: Angle in degrees.

        Returns:
            Pulse width in microseconds.
        """
        # Normalize angle to 0-1 range
        angle_span = self._servo_config.angle_max - self._servo_config.angle_min
        normalized = (angle - self._servo_config.angle_min) / angle_span if angle_span > 0 else 0

        # Map to pulse range
        pulse_span = self._calibration_max_pulse - self._calibration_min_pulse
        pulse = self._calibration_min_pulse + int(normalized * pulse_span)

        # Apply trim
        pulse += self._trim

        return pulse

    def pulse_to_angle(self, pulse: int) -> float:
        """Convert pulse width to angle.

        Args:
            pulse: Pulse width in microseconds.

        Returns:
            Angle in degrees.
        """
        # Remove trim
        pulse -= self._trim

        # Normalize pulse to 0-1 range
        pulse_span = self._calibration_max_pulse - self._calibration_min_pulse
        normalized = (pulse - self._calibration_min_pulse) / pulse_span if pulse_span > 0 else 0

        # Map to angle range
        angle_span = self._servo_config.angle_max - self._servo_config.angle_min
        angle = self._servo_config.angle_min + (normalized * angle_span)

        return angle

    def _angle_to_normalized(self, angle: float) -> float:
        """Convert angle to normalized 0-1 value for driver.

        Args:
            angle: Angle in degrees.

        Returns:
            Normalized value (0.0 to 1.0).
        """
        angle_span = self._servo_config.angle_max - self._servo_config.angle_min
        if angle_span <= 0:
            return 0.0
        return (angle - self._servo_config.angle_min) / angle_span

    # -------------------------------------------------------------------------
    # Core Servo Operations
    # -------------------------------------------------------------------------

    def set_angle(self, angle: float) -> None:
        """Set servo to a specific angle.

        This is an alias for set() with clearer semantics.

        Args:
            angle: Target angle in degrees.

        Raises:
            DisabledError: If servo is disabled.
            LimitsExceededError: If angle is outside range.
        """
        self.set(angle)

    def set_pulse(self, pulse_us: int) -> None:
        """Set servo directly by pulse width.

        Bypasses angle conversion for precise control.

        Args:
            pulse_us: Pulse width in microseconds.

        Raises:
            DisabledError: If servo is disabled.
            LimitsExceededError: If pulse is outside calibrated range.
        """
        if not self._is_enabled:
            raise DisabledError(f"Servo {self.name} is disabled")

        # Validate pulse range
        if pulse_us < self._calibration_min_pulse or pulse_us > self._calibration_max_pulse:
            raise LimitsExceededError(
                value=pulse_us,
                min_limit=self._calibration_min_pulse,
                max_limit=self._calibration_max_pulse,
                name=f"{self.name} pulse",
            )

        self._pulse_width = pulse_us

        # Convert to angle for state tracking
        angle = self.pulse_to_angle(pulse_us)
        self._current_value = angle
        self._target_value = angle

        # Apply to driver
        if self._driver is not None:
            normalized = self._angle_to_normalized(angle)
            self._driver.set_channel(self._config.channel, normalized)

        logger.debug("Servo %s set to pulse %d µs (%.1f°)", self.name, pulse_us, angle)

    def center(self) -> None:
        """Move servo to center position."""
        center_angle = (self._servo_config.angle_min + self._servo_config.angle_max) / 2
        self.set(center_angle)

    def disable(self) -> None:
        """Disable servo and stop PWM signal.

        This releases the servo, allowing it to be moved freely.
        """
        if self._driver is not None:
            # Set channel to 0 to stop PWM
            try:
                self._driver.set_channel(self._config.channel, 0.0)
            except Exception as e:
                logger.warning("Error disabling servo %s: %s", self.name, e)

        super().disable()

    # -------------------------------------------------------------------------
    # Sweep Operations
    # -------------------------------------------------------------------------

    def sweep(
        self,
        start: float | None = None,
        end: float | None = None,
        step: float = 1.0,
        delay: float = 0.02,
    ) -> None:
        """Sweep servo through a range of angles.

        Args:
            start: Starting angle (default: current angle).
            end: Ending angle (default: max angle).
            step: Step size in degrees.
            delay: Delay between steps in seconds.

        Raises:
            DisabledError: If servo is disabled.
        """
        if not self._is_enabled:
            raise DisabledError(f"Servo {self.name} is disabled")

        start = start if start is not None else self._current_value
        end = end if end is not None else self._servo_config.angle_max

        # Determine direction
        if start < end:
            angles = [start + i * step for i in range(int((end - start) / step) + 1)]
        else:
            angles = [start - i * step for i in range(int((start - end) / step) + 1)]

        # Clamp to range
        angles = [
            max(self._servo_config.angle_min, min(self._servo_config.angle_max, a)) for a in angles
        ]

        self._state = ActuatorState.MOVING

        for angle in angles:
            self.set(angle)
            time.sleep(delay)

        self._state = ActuatorState.HOLDING

    async def sweep_async(
        self,
        start: float | None = None,
        end: float | None = None,
        step: float = 1.0,
        delay: float = 0.02,
    ) -> AsyncIterator[float]:
        """Asynchronously sweep servo through angles.

        Yields each angle as it's set.

        Args:
            start: Starting angle.
            end: Ending angle.
            step: Step size in degrees.
            delay: Delay between steps in seconds.

        Yields:
            Current angle after each step.
        """
        if not self._is_enabled:
            raise DisabledError(f"Servo {self.name} is disabled")

        start = start if start is not None else self._current_value
        end = end if end is not None else self._servo_config.angle_max

        # Determine direction
        if start < end:
            angles = [start + i * step for i in range(int((end - start) / step) + 1)]
        else:
            angles = [start - i * step for i in range(int((start - end) / step) + 1)]

        # Clamp to range
        angles = [
            max(self._servo_config.angle_min, min(self._servo_config.angle_max, a)) for a in angles
        ]

        self._state = ActuatorState.MOVING

        for angle in angles:
            self.set(angle)
            yield angle
            await asyncio.sleep(delay)

        self._state = ActuatorState.HOLDING

    # -------------------------------------------------------------------------
    # Calibration
    # -------------------------------------------------------------------------

    def calibrate(
        self,
        min_pulse: int | None = None,
        max_pulse: int | None = None,
    ) -> None:
        """Calibrate servo pulse widths.

        Allows fine-tuning the pulse range for a specific servo.

        Args:
            min_pulse: Pulse width for minimum angle (microseconds).
            max_pulse: Pulse width for maximum angle (microseconds).

        Raises:
            CalibrationError: If pulse values are invalid.
        """
        if min_pulse is not None and max_pulse is not None and min_pulse >= max_pulse:
            raise CalibrationError(
                f"min_pulse ({min_pulse}) must be less than max_pulse ({max_pulse})"
            )

        if min_pulse is not None:
            self._calibration_min_pulse = min_pulse

        if max_pulse is not None:
            self._calibration_max_pulse = max_pulse

        self._is_calibrated = True
        logger.info(
            "Servo %s calibrated: pulse range %d-%d µs",
            self.name,
            self._calibration_min_pulse,
            self._calibration_max_pulse,
        )

    def _run_calibration(self) -> None:
        """Internal calibration procedure.

        For servos, calibration just marks as calibrated.
        Use calibrate() with pulse values for actual calibration.
        """
        self._is_calibrated = True

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def servo_status(self) -> ServoStatus:
        """Get servo-specific status.

        Returns:
            ServoStatus with current state information.
        """
        return ServoStatus(
            state=self._state,
            angle=self._current_value,
            target_angle=self._target_value,
            pulse_width=self._pulse_width,
            is_enabled=self._is_enabled,
            is_calibrated=self._is_calibrated,
            error=self._error,
        )

    # -------------------------------------------------------------------------
    # Abstract Method Implementations
    # -------------------------------------------------------------------------

    def _apply_value(self, value: float) -> None:
        """Apply angle value to hardware.

        Converts angle to normalized value and sends to driver.

        Args:
            value: Angle in degrees.
        """
        # Update pulse width tracking
        self._pulse_width = self.angle_to_pulse(value)

        if self._driver is not None:
            # Convert to normalized 0-1 for driver
            normalized = self._angle_to_normalized(value)
            self._driver.set_channel(self._config.channel, normalized)

    def _read_value(self) -> float:
        """Read current angle from hardware.

        Most servos don't have position feedback, so this returns
        the last commanded value.

        Returns:
            Current angle in degrees.
        """
        # Most servos don't have feedback, return last known value
        return self._current_value

    # -------------------------------------------------------------------------
    # Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"Servo("
            f"name={self.name!r}, "
            f"angle={self.angle:.1f}°, "
            f"range={self.angle_range}, "
            f"channel={self.channel})"
        )


# =============================================================================
# Simulated Servo
# =============================================================================


class SimulatedServo(Servo):
    """Simulated servo for testing without hardware.

    Behaves exactly like a real servo but doesn't require a driver.
    Useful for unit testing and development.
    """

    def __init__(
        self,
        name: str = "SimulatedServo",
        angle_range: tuple[float, float] = (0.0, 180.0),
        pulse_range: tuple[int, int] = (STANDARD_PULSE_MIN, STANDARD_PULSE_MAX),
        frequency: int = STANDARD_FREQUENCY,
        initial_angle: float | None = None,
        **kwargs: Any,
    ) -> None:
        """Initialize simulated servo.

        Args:
            name: Human-readable name.
            angle_range: Min and max angle in degrees.
            pulse_range: Min and max pulse width in microseconds.
            frequency: PWM frequency in Hz.
            initial_angle: Initial angle (default: center).
            **kwargs: Additional arguments passed to Servo.
        """
        super().__init__(
            name=name,
            angle_range=angle_range,
            pulse_range=pulse_range,
            frequency=frequency,
            **kwargs,
        )

        # Set initial angle
        if initial_angle is not None:
            self._current_value = initial_angle
        else:
            self._current_value = (angle_range[0] + angle_range[1]) / 2

        self._pulse_width = self.angle_to_pulse(self._current_value)

    def _apply_value(self, value: float) -> None:
        """Apply value (simulated - just updates state)."""
        self._pulse_width = self.angle_to_pulse(value)
        logger.debug(
            "SimulatedServo %s: set to %.1f° (pulse: %d µs)",
            self.name,
            value,
            self._pulse_width,
        )

    def _read_value(self) -> float:
        """Read value (simulated - returns last set value)."""
        return self._current_value


# =============================================================================
# Factory Functions
# =============================================================================


def create_servo(
    name: str = "Servo",
    driver: Driver | None = None,
    channel: int = 0,
    angle_range: tuple[float, float] = (0.0, 180.0),
    pulse_range: tuple[int, int] = (STANDARD_PULSE_MIN, STANDARD_PULSE_MAX),
    simulated: bool = False,
    **kwargs: Any,
) -> Servo:
    """Factory function to create a Servo.

    Args:
        name: Human-readable name.
        driver: Driver for hardware control.
        channel: Channel on the driver.
        angle_range: Min and max angle in degrees.
        pulse_range: Min and max pulse width in microseconds.
        simulated: If True, create SimulatedServo.
        **kwargs: Additional servo configuration.

    Returns:
        Servo or SimulatedServo instance.
    """
    if simulated or driver is None:
        return SimulatedServo(
            name=name,
            angle_range=angle_range,
            pulse_range=pulse_range,
            **kwargs,
        )

    return Servo(
        name=name,
        driver=driver,
        channel=channel,
        angle_range=angle_range,
        pulse_range=pulse_range,
        **kwargs,
    )


# =============================================================================
# Exports
# =============================================================================


__all__ = [
    "HIGH_FREQUENCY",
    "STANDARD_FREQUENCY",
    "STANDARD_PULSE_CENTER",
    "STANDARD_PULSE_MAX",
    "STANDARD_PULSE_MIN",
    "Servo",
    "ServoConfig",
    "ServoRange",
    "ServoStatus",
    "ServoType",
    "SimulatedServo",
    "create_servo",
]
