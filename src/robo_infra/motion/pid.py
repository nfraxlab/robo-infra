"""PID controller implementation for robotics control loops.

This module provides a PID (Proportional-Integral-Derivative) controller
with anti-windup, derivative filtering, and output limiting.

Example:
    >>> from robo_infra.motion.pid import PID, PIDConfig
    >>> config = PIDConfig(output_min=-100, output_max=100)
    >>> pid = PID(kp=1.0, ki=0.1, kd=0.05, config=config)
    >>> output = pid.update(setpoint=100.0, measurement=50.0)
"""

from __future__ import annotations

import logging
import time

from pydantic import BaseModel, Field, ValidationInfo, field_validator


logger = logging.getLogger(__name__)


class PIDConfig(BaseModel):
    """Configuration for PID controller.

    Attributes:
        kp: Proportional gain. Higher values increase response speed but may
            cause oscillation.
        ki: Integral gain. Eliminates steady-state error but can cause
            overshoot and windup.
        kd: Derivative gain. Reduces overshoot and oscillation but amplifies
            noise.
        output_min: Minimum output value (for clamping).
        output_max: Maximum output value (for clamping).
        integral_min: Minimum integral term (anti-windup).
        integral_max: Maximum integral term (anti-windup).
        derivative_filter: Low-pass filter coefficient for derivative term.
            Value between 0 and 1. Higher values = more filtering.
        sample_time: Expected time between updates in seconds. Used for
            time-based integration.
    """

    kp: float = Field(default=1.0, description="Proportional gain")
    ki: float = Field(default=0.0, description="Integral gain")
    kd: float = Field(default=0.0, description="Derivative gain")
    output_min: float = Field(default=-1.0, description="Minimum output value")
    output_max: float = Field(default=1.0, description="Maximum output value")
    integral_min: float = Field(default=-10.0, description="Minimum integral term (anti-windup)")
    integral_max: float = Field(default=10.0, description="Maximum integral term (anti-windup)")
    derivative_filter: float = Field(
        default=0.1,
        ge=0.0,
        le=1.0,
        description="Low-pass filter coefficient for derivative (0-1)",
    )
    sample_time: float = Field(
        default=0.01,
        gt=0.0,
        description="Expected sample time in seconds",
    )

    @field_validator("output_max")
    @classmethod
    def validate_output_max(cls, v: float, info: ValidationInfo) -> float:
        """Ensure output_max > output_min."""
        output_min = info.data.get("output_min", -1.0) if info.data else -1.0
        if v <= output_min:
            raise ValueError(f"output_max ({v}) must be greater than output_min ({output_min})")
        return v

    @field_validator("integral_max")
    @classmethod
    def validate_integral_max(cls, v: float, info: ValidationInfo) -> float:
        """Ensure integral_max > integral_min."""
        integral_min = info.data.get("integral_min", -10.0) if info.data else -10.0
        if v <= integral_min:
            raise ValueError(
                f"integral_max ({v}) must be greater than integral_min ({integral_min})"
            )
        return v


class PID:
    """PID controller with anti-windup and derivative filtering.

    This implementation includes:
    - Output limiting (clamping)
    - Integral anti-windup (clamping + conditional integration)
    - Derivative on measurement (not error) to prevent derivative kick
    - Low-pass filter on derivative term to reduce noise
    - Time-based integration for variable update rates

    Example:
        >>> pid = PID(kp=2.0, ki=0.5, kd=0.1)
        >>> # In a control loop:
        >>> while True:
        ...     measurement = sensor.read()
        ...     output = pid.update(setpoint=target, measurement=measurement)
        ...     actuator.set(output)
        ...     time.sleep(0.01)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        *,
        config: PIDConfig | None = None,
    ) -> None:
        """Initialize PID controller.

        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            config: Optional PIDConfig for advanced settings. If provided,
                the kp, ki, kd from config are used unless overridden.
        """
        if config is not None:
            self._config = config
            # Allow explicit gains to override config
            self._kp = kp if kp != 1.0 else config.kp
            self._ki = ki if ki != 0.0 else config.ki
            self._kd = kd if kd != 0.0 else config.kd
        else:
            self._config = PIDConfig(kp=kp, ki=ki, kd=kd)
            self._kp = kp
            self._ki = ki
            self._kd = kd

        # Internal state
        self._integral: float = 0.0
        self._last_error: float = 0.0
        self._last_measurement: float = 0.0
        self._last_time: float | None = None
        self._last_derivative: float = 0.0

        logger.debug(
            "PID initialized: kp=%.3f, ki=%.3f, kd=%.3f, "
            "output=[%.2f, %.2f], integral=[%.2f, %.2f]",
            self._kp,
            self._ki,
            self._kd,
            self._config.output_min,
            self._config.output_max,
            self._config.integral_min,
            self._config.integral_max,
        )

    @property
    def kp(self) -> float:
        """Proportional gain."""
        return self._kp

    @property
    def ki(self) -> float:
        """Integral gain."""
        return self._ki

    @property
    def kd(self) -> float:
        """Derivative gain."""
        return self._kd

    @property
    def config(self) -> PIDConfig:
        """Current PID configuration."""
        return self._config

    @property
    def integral(self) -> float:
        """Current integral term value."""
        return self._integral

    @property
    def last_error(self) -> float:
        """Last error value."""
        return self._last_error

    @property
    def last_derivative(self) -> float:
        """Last filtered derivative value."""
        return self._last_derivative

    def update(self, setpoint: float, measurement: float) -> float:
        """Calculate PID output.

        Args:
            setpoint: Desired target value.
            measurement: Current measured value.

        Returns:
            Control output, clamped to [output_min, output_max].
        """
        # Calculate time delta
        current_time = time.monotonic()
        if self._last_time is None:
            dt = self._config.sample_time
        else:
            dt = current_time - self._last_time
            if dt <= 0:
                dt = self._config.sample_time

        # Calculate error
        error = setpoint - measurement

        # Proportional term
        p_term = self._kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = max(
            self._config.integral_min,
            min(self._config.integral_max, self._integral),
        )
        i_term = self._ki * self._integral

        # Derivative term on measurement (not error) to prevent derivative kick
        # Using derivative on measurement: d/dt(-measurement) = -d(measurement)/dt
        raw_derivative = -(measurement - self._last_measurement) / dt if dt > 0 else 0.0

        # Apply low-pass filter to derivative
        alpha = self._config.derivative_filter
        self._last_derivative = alpha * self._last_derivative + (1 - alpha) * raw_derivative
        d_term = self._kd * self._last_derivative

        # Calculate total output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(self._config.output_min, min(self._config.output_max, output))

        # Conditional integration (additional anti-windup)
        # If output is saturated and integral is making it worse, don't integrate
        if (output >= self._config.output_max and error > 0) or (
            output <= self._config.output_min and error < 0
        ):
            self._integral -= error * dt  # Undo integration

        # Update state for next iteration
        self._last_error = error
        self._last_measurement = measurement
        self._last_time = current_time

        logger.debug(
            "PID update: setpoint=%.3f, measurement=%.3f, error=%.3f, "
            "P=%.3f, I=%.3f, D=%.3f, output=%.3f",
            setpoint,
            measurement,
            error,
            p_term,
            i_term,
            d_term,
            output,
        )

        return output

    def reset(self) -> None:
        """Reset PID controller state.

        Clears integral accumulator and derivative history.
        Call this when starting a new control sequence or after
        a significant discontinuity.
        """
        self._integral = 0.0
        self._last_error = 0.0
        self._last_measurement = 0.0
        self._last_time = None
        self._last_derivative = 0.0
        logger.debug("PID reset")

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update PID gains.

        Args:
            kp: New proportional gain.
            ki: New integral gain.
            kd: New derivative gain.
        """
        self._kp = kp
        self._ki = ki
        self._kd = kd
        logger.debug("PID gains updated: kp=%.3f, ki=%.3f, kd=%.3f", kp, ki, kd)

    def set_output_limits(self, min_val: float, max_val: float) -> None:
        """Update output limits.

        Args:
            min_val: New minimum output value.
            max_val: New maximum output value.

        Raises:
            ValueError: If max_val <= min_val.
        """
        if max_val <= min_val:
            raise ValueError(f"max_val ({max_val}) must be greater than min_val ({min_val})")
        self._config = PIDConfig(
            kp=self._config.kp,
            ki=self._config.ki,
            kd=self._config.kd,
            output_min=min_val,
            output_max=max_val,
            integral_min=self._config.integral_min,
            integral_max=self._config.integral_max,
            derivative_filter=self._config.derivative_filter,
            sample_time=self._config.sample_time,
        )
        logger.debug("PID output limits updated: [%.2f, %.2f]", min_val, max_val)

    def set_sample_time(self, sample_time: float) -> None:
        """Update expected sample time.

        Args:
            sample_time: New sample time in seconds.

        Raises:
            ValueError: If sample_time <= 0.
        """
        if sample_time <= 0:
            raise ValueError(f"sample_time must be positive, got {sample_time}")
        self._config = PIDConfig(
            kp=self._config.kp,
            ki=self._config.ki,
            kd=self._config.kd,
            output_min=self._config.output_min,
            output_max=self._config.output_max,
            integral_min=self._config.integral_min,
            integral_max=self._config.integral_max,
            derivative_filter=self._config.derivative_filter,
            sample_time=sample_time,
        )
        logger.debug("PID sample time updated: %.4f", sample_time)

    def __repr__(self) -> str:
        """Return string representation."""
        return (
            f"PID(kp={self._kp:.3f}, ki={self._ki:.3f}, kd={self._kd:.3f}, "
            f"output=[{self._config.output_min:.2f}, {self._config.output_max:.2f}])"
        )


__all__ = [
    "PID",
    "PIDConfig",
]
