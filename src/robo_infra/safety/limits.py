"""Enhanced limit enforcement for robotics safety.

This module provides active limit enforcement that goes beyond basic clamping
to include velocity limits, acceleration limits, and jerk limits.

The existing Limits class in core/types.py provides basic min/max clamping.
This module adds:
- Velocity limiting (rate of change)
- Acceleration limiting (rate of rate of change)
- Jerk limiting (smoothness)
- Soft limits (warning before hard limit)
- Active enforcement with logging

Example:
    >>> from robo_infra.safety import LimitEnforcer
    >>>
    >>> enforcer = LimitEnforcer(
    ...     position_limits=(0, 180),  # degrees
    ...     velocity_limit=90.0,  # deg/s
    ...     acceleration_limit=180.0,  # deg/s^2
    ... )
    >>>
    >>> # In control loop:
    >>> safe_target = enforcer.enforce(raw_target, current_position)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.types import Limits


if TYPE_CHECKING:
    from collections.abc import Callable


logger = logging.getLogger(__name__)


class LimitViolationType(Enum):
    """Types of limit violations."""

    POSITION_MIN = "position_min"
    POSITION_MAX = "position_max"
    VELOCITY = "velocity"
    ACCELERATION = "acceleration"
    JERK = "jerk"
    SOFT_LIMIT = "soft_limit"


@dataclass
class LimitViolation:
    """Record of a limit violation."""

    violation_type: LimitViolationType
    timestamp: float
    requested: float
    enforced: float
    limit: float
    component: str | None = None


class EnforcerConfig(BaseModel):
    """Configuration for limit enforcer.

    Attributes:
        name: Name of the component being enforced.
        position_min: Minimum position.
        position_max: Maximum position.
        soft_limit_margin: Margin for soft limits (fraction of range).
        velocity_limit: Maximum velocity (units/second).
        acceleration_limit: Maximum acceleration (units/second^2).
        jerk_limit: Maximum jerk (units/second^3).
        clamp_behavior: What to do on violation: "clamp", "reject", "estop".
        log_violations: Log each violation.
    """

    name: str = "Enforcer"
    position_min: float = 0.0
    position_max: float = 1.0
    soft_limit_margin: float = Field(default=0.1, ge=0, le=0.5)
    velocity_limit: float | None = None
    acceleration_limit: float | None = None
    jerk_limit: float | None = None
    clamp_behavior: str = "clamp"  # "clamp", "reject", "estop"
    log_violations: bool = True

    model_config = {"frozen": False, "extra": "allow"}


class LimitEnforcer:
    """Active limit enforcement with velocity/acceleration limiting.

    Goes beyond simple clamping to enforce rate limits, ensuring
    smooth and safe motion even with aggressive commands.
    """

    def __init__(
        self,
        position_limits: tuple[float, float] | Limits | None = None,
        *,
        velocity_limit: float | None = None,
        acceleration_limit: float | None = None,
        jerk_limit: float | None = None,
        config: EnforcerConfig | None = None,
    ) -> None:
        """Initialize limit enforcer.

        Args:
            position_limits: (min, max) position limits.
            velocity_limit: Maximum velocity (units/second).
            acceleration_limit: Maximum acceleration (units/second^2).
            jerk_limit: Maximum jerk (units/second^3).
            config: Full configuration (overrides other args).
        """
        if config:
            self._config = config
        else:
            if isinstance(position_limits, Limits):
                pos_min, pos_max = position_limits.min, position_limits.max
            elif position_limits:
                pos_min, pos_max = position_limits
            else:
                pos_min, pos_max = 0.0, 1.0

            self._config = EnforcerConfig(
                position_min=pos_min,
                position_max=pos_max,
                velocity_limit=velocity_limit,
                acceleration_limit=acceleration_limit,
                jerk_limit=jerk_limit,
            )

        # State for rate limiting
        self._last_position: float | None = None
        self._last_velocity: float = 0.0
        self._last_acceleration: float = 0.0
        self._last_time: float | None = None

        # Violation tracking
        self._violations: list[LimitViolation] = []
        self._violation_count = 0
        self._callbacks: list[Callable[[LimitViolation], None]] = []

    @property
    def position_limits(self) -> Limits:
        """Position limits as a Limits object."""
        return Limits(
            min=self._config.position_min,
            max=self._config.position_max,
        )

    @property
    def soft_limits(self) -> tuple[float, float]:
        """Soft limit positions (warning zone boundaries)."""
        range_val = self._config.position_max - self._config.position_min
        margin = range_val * self._config.soft_limit_margin
        return (
            self._config.position_min + margin,
            self._config.position_max - margin,
        )

    @property
    def violation_count(self) -> int:
        """Total number of violations."""
        return self._violation_count

    @property
    def recent_violations(self) -> list[LimitViolation]:
        """Recent violations (last 100)."""
        return list(self._violations[-100:])

    def register_callback(self, callback: Callable[[LimitViolation], None]) -> None:
        """Register callback for limit violations."""
        self._callbacks.append(callback)

    def reset_state(self) -> None:
        """Reset rate limiting state.

        Call this when position changes discontinuously (e.g., after homing).
        """
        self._last_position = None
        self._last_velocity = 0.0
        self._last_acceleration = 0.0
        self._last_time = None

    def enforce(
        self,
        target: float,
        current_position: float | None = None,
        *,
        dt: float | None = None,
    ) -> float:
        """Enforce limits on a target value.

        Args:
            target: Requested target position.
            current_position: Current actual position (for rate limiting).
            dt: Time since last call (auto-calculated if None).

        Returns:
            Safe target position after limit enforcement.

        Raises:
            LimitsExceededError: If clamp_behavior is "reject".
        """
        now = time.time()

        # Calculate dt
        if dt is None:
            dt = now - self._last_time if self._last_time is not None else 0.02

        # Initialize last position if needed
        if self._last_position is None:
            self._last_position = current_position if current_position is not None else target

        if current_position is not None:
            self._last_position = current_position

        # Apply limits in order
        result = target

        # 1. Position limits (hard)
        result = self._enforce_position_limits(result)

        # 2. Velocity limit
        if self._config.velocity_limit is not None and dt > 0:
            result = self._enforce_velocity_limit(result, dt)

        # 3. Acceleration limit
        if self._config.acceleration_limit is not None and dt > 0:
            result = self._enforce_acceleration_limit(result, dt)

        # 4. Jerk limit
        if self._config.jerk_limit is not None and dt > 0:
            result = self._enforce_jerk_limit(result, dt)

        # Update state - _last_position is guaranteed set above
        assert self._last_position is not None
        if dt > 0:
            new_velocity = (result - self._last_position) / dt
            new_acceleration = (new_velocity - self._last_velocity) / dt
            self._last_velocity = new_velocity
            self._last_acceleration = new_acceleration

        self._last_position = result
        self._last_time = now

        return result

    def _enforce_position_limits(self, target: float) -> float:
        """Enforce position limits."""
        cfg = self._config

        # Check soft limits first
        soft_min, soft_max = self.soft_limits
        if target < soft_min or target > soft_max:
            self._record_violation(
                LimitViolationType.SOFT_LIMIT,
                target,
                target,  # Not enforced, just warning
                soft_min if target < soft_min else soft_max,
            )

        # Enforce hard limits
        if target < cfg.position_min:
            self._record_violation(
                LimitViolationType.POSITION_MIN,
                target,
                cfg.position_min,
                cfg.position_min,
            )
            return cfg.position_min

        if target > cfg.position_max:
            self._record_violation(
                LimitViolationType.POSITION_MAX,
                target,
                cfg.position_max,
                cfg.position_max,
            )
            return cfg.position_max

        return target

    def _enforce_velocity_limit(self, target: float, dt: float) -> float:
        """Enforce velocity limit."""
        # Type assertions - these are guaranteed set before this method is called
        assert self._last_position is not None
        assert self._config.velocity_limit is not None

        max_change = self._config.velocity_limit * dt
        change = target - self._last_position

        if abs(change) > max_change:
            limited_change = max_change if change > 0 else -max_change
            limited_target = self._last_position + limited_change

            self._record_violation(
                LimitViolationType.VELOCITY,
                target,
                limited_target,
                self._config.velocity_limit,
            )
            return limited_target

        return target

    def _enforce_acceleration_limit(self, target: float, dt: float) -> float:
        """Enforce acceleration limit."""
        # Type assertions - these are guaranteed set before this method is called
        assert self._last_position is not None
        assert self._config.acceleration_limit is not None

        # Calculate required velocity to reach target
        required_velocity = (target - self._last_position) / dt
        required_acceleration = (required_velocity - self._last_velocity) / dt

        max_accel = self._config.acceleration_limit
        if abs(required_acceleration) > max_accel:
            limited_accel = max_accel if required_acceleration > 0 else -max_accel
            limited_velocity = self._last_velocity + limited_accel * dt
            limited_target = self._last_position + limited_velocity * dt

            self._record_violation(
                LimitViolationType.ACCELERATION,
                target,
                limited_target,
                max_accel,
            )
            return limited_target

        return target

    def _enforce_jerk_limit(self, target: float, dt: float) -> float:
        """Enforce jerk limit."""
        # Type assertions - these are guaranteed set before this method is called
        assert self._last_position is not None
        assert self._config.jerk_limit is not None

        # Calculate required motion parameters
        required_velocity = (target - self._last_position) / dt
        required_acceleration = (required_velocity - self._last_velocity) / dt
        required_jerk = (required_acceleration - self._last_acceleration) / dt

        max_jerk = self._config.jerk_limit
        if abs(required_jerk) > max_jerk:
            limited_jerk = max_jerk if required_jerk > 0 else -max_jerk
            limited_acceleration = self._last_acceleration + limited_jerk * dt
            limited_velocity = self._last_velocity + limited_acceleration * dt
            limited_target = self._last_position + limited_velocity * dt

            self._record_violation(
                LimitViolationType.JERK,
                target,
                limited_target,
                max_jerk,
            )
            return limited_target

        return target

    def _record_violation(
        self,
        vtype: LimitViolationType,
        requested: float,
        enforced: float,
        limit: float,
    ) -> None:
        """Record a limit violation."""
        violation = LimitViolation(
            violation_type=vtype,
            timestamp=time.time(),
            requested=requested,
            enforced=enforced,
            limit=limit,
            component=self._config.name,
        )

        self._violations.append(violation)
        self._violation_count += 1

        # Keep only recent violations
        if len(self._violations) > 1000:
            self._violations = self._violations[-500:]

        if self._config.log_violations:
            if vtype == LimitViolationType.SOFT_LIMIT:
                logger.warning(
                    "Soft limit: %s approaching limit at %.3f",
                    self._config.name,
                    requested,
                )
            else:
                logger.warning(
                    "Limit enforced: %s %s - requested %.3f, enforced %.3f (limit: %.3f)",
                    self._config.name,
                    vtype.value,
                    requested,
                    enforced,
                    limit,
                )

        # Call callbacks
        for callback in self._callbacks:
            try:
                callback(violation)
            except Exception as e:
                logger.error("Limit enforcer callback failed: %s", e)


# =============================================================================
# Active Limit Guard
# =============================================================================


class LimitGuard:
    """Wraps an actuator with automatic limit enforcement.

    Example:
        >>> servo = Servo(channel=0)
        >>> guard = LimitGuard(
        ...     actuator=servo,
        ...     position_limits=(0, 180),
        ...     velocity_limit=90.0,
        ... )
        >>>
        >>> # Use guard instead of servo directly
        >>> guard.set(target)  # Automatically limited
    """

    def __init__(
        self,
        actuator: Any,
        position_limits: tuple[float, float] | Limits | None = None,
        *,
        velocity_limit: float | None = None,
        acceleration_limit: float | None = None,
        estop: Any | None = None,
    ) -> None:
        """Initialize limit guard.

        Args:
            actuator: Actuator to wrap.
            position_limits: Position limits.
            velocity_limit: Velocity limit.
            acceleration_limit: Acceleration limit.
            estop: E-stop to register with.
        """
        self._actuator = actuator
        self._enforcer = LimitEnforcer(
            position_limits=position_limits,
            velocity_limit=velocity_limit,
            acceleration_limit=acceleration_limit,
            config=EnforcerConfig(name=getattr(actuator, "name", "unknown")),
        )
        self._estop = estop

        # Register with E-stop if provided
        if estop and hasattr(estop, "register_actuator"):
            estop.register_actuator(actuator)

    @property
    def actuator(self) -> Any:
        """Wrapped actuator."""
        return self._actuator

    @property
    def enforcer(self) -> LimitEnforcer:
        """Limit enforcer."""
        return self._enforcer

    def set(self, value: float) -> float:
        """Set actuator value with limit enforcement.

        Args:
            value: Requested value.

        Returns:
            Actual value after enforcement.
        """
        current = self._actuator.get() if hasattr(self._actuator, "get") else None
        safe_value = self._enforcer.enforce(value, current)
        self._actuator.set(safe_value)
        return safe_value

    def get(self) -> float:
        """Get current actuator value."""
        return self._actuator.get()

    def enable(self) -> None:
        """Enable the actuator."""
        self._actuator.enable()

    def disable(self) -> None:
        """Disable the actuator."""
        self._actuator.disable()
        self._enforcer.reset_state()

    @property
    def name(self) -> str:
        """Actuator name."""
        return getattr(self._actuator, "name", "unknown")
