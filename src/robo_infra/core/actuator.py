"""Actuator abstractions for robotics control.

This module provides abstract base classes and utilities for building
actuators like servos, motors, and linear actuators.

Example:
    >>> from robo_infra.core.actuator import Actuator, SimulatedActuator
    >>> from robo_infra.core.types import Limits
    >>>
    >>> # Create a simulated servo
    >>> servo = SimulatedActuator(
    ...     name="shoulder",
    ...     limits=Limits(min_value=0, max_value=180, default_value=90),
    ...     unit="degrees",
    ... )
    >>> servo.enable()
    >>> servo.set(45.0)
    >>> print(servo.get())
    45.0
    >>>
    >>> # With a real driver
    >>> from robo_infra.core.driver import SimulatedDriver
    >>> driver = SimulatedDriver(channels=16)
    >>> driver.connect()
    >>> servo = SimulatedActuator(
    ...     name="elbow",
    ...     driver=driver,
    ...     channel=0,
    ...     limits=Limits(min_value=0, max_value=180, default_value=90),
    ... )
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.exceptions import (
    CalibrationError,
    DisabledError,
    LimitsExceededError,
    NotCalibratedError,
)
from robo_infra.core.types import Limits


if TYPE_CHECKING:
    from collections.abc import Iterator

    from robo_infra.core.driver import Driver

logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class ActuatorState(Enum):
    """States an actuator can be in."""

    DISABLED = "disabled"
    IDLE = "idle"
    MOVING = "moving"
    HOLDING = "holding"
    ERROR = "error"
    CALIBRATING = "calibrating"


class ActuatorType(Enum):
    """Types of actuators."""

    SERVO = "servo"
    DC_MOTOR = "dc_motor"
    STEPPER = "stepper"
    LINEAR = "linear"
    SOLENOID = "solenoid"
    PNEUMATIC = "pneumatic"
    HYDRAULIC = "hydraulic"
    GENERIC = "generic"


# =============================================================================
# Configuration Models
# =============================================================================


class ActuatorConfig(BaseModel):
    """Pydantic configuration model for actuators.

    Attributes:
        name: Human-readable actuator name.
        actuator_type: Type of actuator.
        channel: Driver channel number.
        limits: Position/value limits.
        unit: Unit of measurement (e.g., "degrees", "mm").
        inverted: If True, invert the value.
        offset: Offset to add to values.
        scale: Scale factor for values.
        speed_limit: Maximum speed (unit/second).
        acceleration: Maximum acceleration (unit/second^2).
        require_calibration: Whether calibration is required before use.
        metadata: Additional actuator-specific configuration.
    """

    name: str = "Actuator"
    actuator_type: ActuatorType = ActuatorType.GENERIC
    channel: int = 0
    limits: Limits = Field(default_factory=lambda: Limits(0.0, 1.0, 0.5))
    unit: str = "units"
    inverted: bool = False
    offset: float = 0.0
    scale: float = 1.0
    speed_limit: float | None = None
    acceleration: float | None = None
    require_calibration: bool = False
    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class ActuatorStatus:
    """Status information for an actuator.

    Attributes:
        state: Current actuator state.
        position: Current position/value.
        target: Target position (if moving).
        is_enabled: Whether actuator is enabled.
        is_calibrated: Whether actuator is calibrated.
        error: Error message if in error state.
    """

    state: ActuatorState = ActuatorState.DISABLED
    position: float = 0.0
    target: float | None = None
    is_enabled: bool = False
    is_calibrated: bool = False
    error: str | None = None


# =============================================================================
# Abstract Base Classes
# =============================================================================


class Actuator(ABC):
    """Abstract base class for all actuators.

    An actuator is a device that can be commanded to a position or speed.
    Examples include servos, DC motors, stepper motors, and linear actuators.

    Subclasses must implement:
        - _apply_value(): Send value to hardware
        - _read_value(): Read current value from hardware (if supported)
    """

    def __init__(
        self,
        name: str | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        limits: Limits | None = None,
        unit: str = "units",
        config: ActuatorConfig | None = None,
    ) -> None:
        """Initialize the actuator.

        Args:
            name: Human-readable name.
            driver: Driver to use for hardware control.
            channel: Channel on the driver.
            limits: Value limits.
            unit: Unit of measurement.
            config: Full configuration (overrides other args).
        """
        if config:
            self._config = config
        else:
            self._config = ActuatorConfig(
                name=name or "Actuator",
                channel=channel,
                limits=limits or Limits(0.0, 1.0, 0.5),
                unit=unit,
            )

        self._driver = driver
        self._state = ActuatorState.DISABLED
        default = self._config.limits.default
        self._current_value: float = default if default is not None else self._config.limits.min
        self._target_value: float | None = None
        self._is_enabled = False
        self._is_calibrated = not self._config.require_calibration
        self._error: str | None = None

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def name(self) -> str:
        """Human-readable name of the actuator."""
        return self._config.name

    @property
    def actuator_type(self) -> ActuatorType:
        """Type of this actuator."""
        return self._config.actuator_type

    @property
    def driver(self) -> Driver | None:
        """Driver used for hardware control."""
        return self._driver

    @driver.setter
    def driver(self, value: Driver | None) -> None:
        """Set the driver."""
        self._driver = value

    @property
    def channel(self) -> int:
        """Channel on the driver."""
        return self._config.channel

    @property
    def limits(self) -> Limits:
        """Value limits for this actuator."""
        return self._config.limits

    @property
    def unit(self) -> str:
        """Unit of measurement."""
        return self._config.unit

    @property
    def state(self) -> ActuatorState:
        """Current actuator state."""
        return self._state

    @property
    def is_enabled(self) -> bool:
        """Whether the actuator is enabled."""
        return self._is_enabled

    @property
    def is_calibrated(self) -> bool:
        """Whether the actuator is calibrated."""
        return self._is_calibrated

    @property
    def config(self) -> ActuatorConfig:
        """Full actuator configuration."""
        return self._config

    # -------------------------------------------------------------------------
    # Core Operations
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the actuator.

        Raises:
            NotCalibratedError: If calibration is required but not done.
        """
        if self._config.require_calibration and not self._is_calibrated:
            raise NotCalibratedError(f"Actuator {self.name} requires calibration")

        self._is_enabled = True
        self._state = ActuatorState.IDLE
        logger.debug("Actuator %s enabled", self.name)

    def disable(self) -> None:
        """Disable the actuator.

        When disabled, the actuator will not respond to set() commands.
        """
        self._is_enabled = False
        self._state = ActuatorState.DISABLED
        logger.debug("Actuator %s disabled", self.name)

    def set(self, value: float, *, force: bool = False) -> None:
        """Set the actuator to a value.

        Args:
            value: Target value in actuator units.
            force: If True, bypass enabled check (use with caution).

        Raises:
            DisabledError: If actuator is disabled and force is False.
            LimitsExceededError: If value is outside limits.
        """
        if not self._is_enabled and not force:
            raise DisabledError(f"Actuator {self.name} is disabled")

        # Apply transforms
        transformed = self._transform_value(value)

        # Check limits
        if not self.limits.is_within(transformed):
            raise LimitsExceededError(
                value=transformed,
                min_limit=self.limits.min,
                max_limit=self.limits.max,
                name=self.name,
            )

        self._target_value = transformed
        self._state = ActuatorState.MOVING

        # Apply to hardware
        self._apply_value(transformed)

        self._current_value = transformed
        self._state = ActuatorState.HOLDING

        logger.debug("Actuator %s set to %.4f %s", self.name, transformed, self.unit)

    def get(self) -> float:
        """Get the current actuator value.

        Returns:
            Current value in actuator units.
        """
        if self._driver is not None:
            raw_value = self._read_value()
            self._current_value = self._inverse_transform_value(raw_value)

        return self._current_value

    def go_to_default(self) -> None:
        """Move actuator to its default position."""
        default = self.limits.default if self.limits.default is not None else self.limits.min
        self.set(default)

    def go_to_min(self) -> None:
        """Move actuator to its minimum position."""
        self.set(self.limits.min)

    def go_to_max(self) -> None:
        """Move actuator to its maximum position."""
        self.set(self.limits.max)

    def status(self) -> ActuatorStatus:
        """Get current actuator status.

        Returns:
            ActuatorStatus with current state information.
        """
        return ActuatorStatus(
            state=self._state,
            position=self._current_value,
            target=self._target_value,
            is_enabled=self._is_enabled,
            is_calibrated=self._is_calibrated,
            error=self._error,
        )

    # -------------------------------------------------------------------------
    # Calibration
    # -------------------------------------------------------------------------

    def calibrate(self) -> None:
        """Run calibration procedure.

        Override in subclasses for specific calibration behavior.

        Raises:
            CalibrationError: If calibration fails.
        """
        self._state = ActuatorState.CALIBRATING
        logger.info("Calibrating actuator %s", self.name)

        try:
            self._run_calibration()
            self._is_calibrated = True
            self._state = ActuatorState.IDLE
            logger.info("Actuator %s calibration complete", self.name)
        except Exception as e:
            self._state = ActuatorState.ERROR
            self._error = str(e)
            raise CalibrationError(f"Calibration failed for {self.name}: {e}") from e

    def _run_calibration(self) -> None:  # noqa: B027
        """Internal calibration procedure.

        Override in subclasses for hardware-specific calibration.
        Default implementation just marks as calibrated.
        """

    # -------------------------------------------------------------------------
    # Value Transformation
    # -------------------------------------------------------------------------

    def _transform_value(self, value: float) -> float:
        """Transform input value before applying.

        Applies offset, scale, and inversion.

        Args:
            value: Raw input value.

        Returns:
            Transformed value.
        """
        result = value

        # Apply scale
        result *= self._config.scale

        # Apply offset
        result += self._config.offset

        # Apply inversion
        if self._config.inverted:
            result = self.limits.max - (result - self.limits.min)

        return result

    def _inverse_transform_value(self, value: float) -> float:
        """Inverse transform value when reading.

        Args:
            value: Hardware value.

        Returns:
            User-facing value.
        """
        result = value

        # Reverse inversion
        if self._config.inverted:
            result = self.limits.max - (result - self.limits.min)

        # Reverse offset
        result -= self._config.offset

        # Reverse scale
        if self._config.scale != 0:
            result /= self._config.scale

        return result

    # -------------------------------------------------------------------------
    # Abstract Methods
    # -------------------------------------------------------------------------

    @abstractmethod
    def _apply_value(self, value: float) -> None:
        """Apply a value to the hardware.

        Subclasses must implement this to send commands to hardware.

        Args:
            value: Transformed value to apply.
        """
        ...

    @abstractmethod
    def _read_value(self) -> float:
        """Read current value from hardware.

        Subclasses must implement this for position feedback.

        Returns:
            Current hardware value.
        """
        ...

    # -------------------------------------------------------------------------
    # Context Manager
    # -------------------------------------------------------------------------

    def __enter__(self) -> Actuator:
        """Context manager entry - enable actuator."""
        self.enable()
        return self

    def __exit__(
        self,
        exc_type: type | None,
        exc_val: Exception | None,
        exc_tb: object,
    ) -> None:
        """Context manager exit - disable actuator."""
        self.disable()

    # -------------------------------------------------------------------------
    # Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"{self.__class__.__name__}("
            f"name={self.name!r}, "
            f"channel={self.channel}, "
            f"state={self._state.value})"
        )


# =============================================================================
# Simulated Actuator
# =============================================================================


class SimulatedActuator(Actuator):
    """A simulated actuator for testing without hardware.

    All operations work in memory only.

    Example:
        >>> actuator = SimulatedActuator(
        ...     name="test_servo",
        ...     limits=Limits(0, 180, 90),
        ...     unit="degrees",
        ... )
        >>> actuator.enable()
        >>> actuator.set(45)
        >>> print(actuator.get())
        45.0
    """

    def __init__(
        self,
        name: str | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        limits: Limits | None = None,
        unit: str = "units",
        config: ActuatorConfig | None = None,
    ) -> None:
        """Initialize simulated actuator."""
        super().__init__(
            name=name or "SimulatedActuator",
            driver=driver,
            channel=channel,
            limits=limits,
            unit=unit,
            config=config,
        )

    def _apply_value(self, value: float) -> None:
        """Simulate applying value (store in memory)."""
        if self._driver is not None:
            # Normalize to 0-1 for driver
            normalized = (value - self.limits.min) / (self.limits.max - self.limits.min)
            self._driver.set_channel(self.channel, normalized)

        logger.debug(
            "Simulated actuator %s: set to %.4f %s",
            self.name,
            value,
            self.unit,
        )

    def _read_value(self) -> float:
        """Simulate reading value (return stored value)."""
        if self._driver is not None:
            normalized = self._driver.get_channel(self.channel)
            return self.limits.min + normalized * (self.limits.max - self.limits.min)

        return self._current_value


# =============================================================================
# Actuator Group
# =============================================================================


@dataclass
class ActuatorGroup:
    """A group of actuators that can be controlled together.

    Useful for coordinating multiple actuators as a single unit.

    Example:
        >>> group = ActuatorGroup(name="arm")
        >>> group.add(shoulder_servo)
        >>> group.add(elbow_servo)
        >>> group.add(wrist_servo)
        >>> group.enable_all()
        >>> group.set_all({"shoulder": 45, "elbow": 90, "wrist": 0})
    """

    name: str = "ActuatorGroup"
    actuators: dict[str, Actuator] = field(default_factory=dict)

    def add(self, actuator: Actuator, name: str | None = None) -> None:
        """Add an actuator to the group.

        Args:
            actuator: Actuator to add.
            name: Name to use in group (defaults to actuator.name).
        """
        key = name or actuator.name
        self.actuators[key] = actuator
        logger.debug("Added actuator %s to group %s", key, self.name)

    def remove(self, name: str) -> Actuator | None:
        """Remove an actuator from the group.

        Args:
            name: Name of actuator to remove.

        Returns:
            Removed actuator or None.
        """
        return self.actuators.pop(name, None)

    def get(self, name: str) -> Actuator:
        """Get an actuator by name.

        Args:
            name: Actuator name.

        Returns:
            The actuator.

        Raises:
            KeyError: If actuator not found.
        """
        return self.actuators[name]

    def enable_all(self) -> None:
        """Enable all actuators in the group."""
        for actuator in self.actuators.values():
            actuator.enable()

    def disable_all(self) -> None:
        """Disable all actuators in the group."""
        for actuator in self.actuators.values():
            actuator.disable()

    def set_all(self, values: dict[str, float]) -> None:
        """Set multiple actuators at once.

        Args:
            values: Dictionary mapping actuator names to values.
        """
        for name, value in values.items():
            if name in self.actuators:
                self.actuators[name].set(value)

    def get_all(self) -> dict[str, float]:
        """Get all actuator values.

        Returns:
            Dictionary mapping names to current values.
        """
        return {name: act.get() for name, act in self.actuators.items()}

    def go_to_defaults(self) -> None:
        """Move all actuators to their default positions."""
        for actuator in self.actuators.values():
            actuator.go_to_default()

    def status_all(self) -> dict[str, ActuatorStatus]:
        """Get status of all actuators.

        Returns:
            Dictionary mapping names to status objects.
        """
        return {name: act.status() for name, act in self.actuators.items()}

    def __len__(self) -> int:
        """Number of actuators in group."""
        return len(self.actuators)

    def __iter__(self) -> Iterator[str]:
        """Iterate over actuator names."""
        return iter(self.actuators)

    def __contains__(self, name: str) -> bool:
        """Check if actuator exists in group."""
        return name in self.actuators

    def __enter__(self) -> ActuatorGroup:
        """Context manager - enable all."""
        self.enable_all()
        return self

    def __exit__(
        self,
        exc_type: type | None,
        exc_val: Exception | None,
        exc_tb: object,
    ) -> None:
        """Context manager - disable all."""
        self.disable_all()


# =============================================================================
# Factory Functions
# =============================================================================


def create_actuator(
    name: str,
    actuator_type: ActuatorType = ActuatorType.GENERIC,
    driver: Driver | None = None,
    channel: int = 0,
    limits: Limits | None = None,
    unit: str = "units",
    *,
    simulate: bool = True,
) -> Actuator:
    """Create an actuator instance.

    Factory function to create actuators with common configurations.

    Args:
        name: Actuator name.
        actuator_type: Type of actuator.
        driver: Optional driver for hardware control.
        channel: Driver channel.
        limits: Value limits.
        unit: Unit of measurement.
        simulate: If True, create simulated actuator.

    Returns:
        Actuator instance.

    Example:
        >>> servo = create_actuator(
        ...     "shoulder",
        ...     ActuatorType.SERVO,
        ...     limits=Limits(0, 180, 90),
        ...     unit="degrees",
        ... )
    """
    config = ActuatorConfig(
        name=name,
        actuator_type=actuator_type,
        channel=channel,
        limits=limits or Limits(0.0, 1.0, 0.5),
        unit=unit,
    )

    if simulate or driver is None:
        return SimulatedActuator(config=config, driver=driver)

    # For now, return simulated - real implementations in Phase 3
    return SimulatedActuator(config=config, driver=driver)


def create_servo(
    name: str,
    channel: int = 0,
    min_angle: float = 0.0,
    max_angle: float = 180.0,
    default_angle: float = 90.0,
    driver: Driver | None = None,
) -> Actuator:
    """Create a servo actuator with common defaults.

    Args:
        name: Servo name.
        channel: Driver channel.
        min_angle: Minimum angle in degrees.
        max_angle: Maximum angle in degrees.
        default_angle: Default angle in degrees.
        driver: Optional driver.

    Returns:
        Configured servo actuator.
    """
    return create_actuator(
        name=name,
        actuator_type=ActuatorType.SERVO,
        driver=driver,
        channel=channel,
        limits=Limits(min_angle, max_angle, default_angle),
        unit="degrees",
    )


def create_motor(
    name: str,
    channel: int = 0,
    min_speed: float = -1.0,
    max_speed: float = 1.0,
    driver: Driver | None = None,
) -> Actuator:
    """Create a motor actuator with common defaults.

    Args:
        name: Motor name.
        channel: Driver channel.
        min_speed: Minimum speed (-1 for reverse).
        max_speed: Maximum speed.
        driver: Optional driver.

    Returns:
        Configured motor actuator.
    """
    return create_actuator(
        name=name,
        actuator_type=ActuatorType.DC_MOTOR,
        driver=driver,
        channel=channel,
        limits=Limits(min_speed, max_speed, 0.0),
        unit="speed",
    )
