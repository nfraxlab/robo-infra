"""Degraded mode controller wrapper for graceful degradation.

This module provides a wrapper around controllers that enables
graceful degradation when individual components fail. Instead of
complete failure when a single actuator or sensor fails, the
controller continues operating with reduced capabilities.

Example:
    >>> from robo_infra.utils.degraded import DegradedModeController
    >>>
    >>> controller = SimulatedController(...)
    >>> degraded = DegradedModeController(controller)
    >>>
    >>> # Mark an actuator as degraded
    >>> degraded.mark_actuator_degraded("joint2", "Communication lost")
    >>>
    >>> # Move still works, skipping degraded actuator
    >>> await degraded.move({"joint1": 45.0, "joint2": 90.0})
    >>> # Only joint1 will move, joint2 is skipped with warning
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import datetime
from typing import TYPE_CHECKING, Any


if TYPE_CHECKING:
    from robo_infra.core.controller import Controller

logger = logging.getLogger(__name__)


@dataclass
class DegradedComponent:
    """Information about a degraded component.

    Attributes:
        name: Component name.
        reason: Why it was marked degraded.
        degraded_at: When it was marked degraded.
        last_known_value: Last known good value (if applicable).
    """

    name: str
    reason: str
    degraded_at: datetime = field(default_factory=datetime.now)
    last_known_value: Any = None


@dataclass
class DegradedModeStatus:
    """Status of degraded mode operation.

    Attributes:
        is_degraded: Whether operating in degraded mode.
        degraded_actuators: Set of degraded actuator names.
        degraded_sensors: Set of degraded sensor names.
        skipped_targets: Targets that were skipped in last operation.
        warnings: List of warning messages.
    """

    is_degraded: bool = False
    degraded_actuators: set[str] = field(default_factory=set)
    degraded_sensors: set[str] = field(default_factory=set)
    skipped_targets: dict[str, float] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)


class DegradedModeController:
    """Controller wrapper that enables graceful degradation.

    Wraps a controller to provide graceful degradation when individual
    actuators or sensors fail. Failed components are marked as degraded
    and excluded from operations, allowing the system to continue
    operating with reduced capabilities.

    Args:
        controller: The underlying controller to wrap.
        max_degraded_ratio: Maximum fraction of actuators that can be
            degraded before operations are refused (default 0.5).

    Example:
        >>> controller = SimulatedController(name="arm")
        >>> degraded = DegradedModeController(controller)
        >>>
        >>> # Mark joint as degraded after communication failure
        >>> try:
        ...     await controller.move({"joint2": 90.0})
        ... except CommunicationError:
        ...     degraded.mark_actuator_degraded("joint2", "Lost communication")
        >>>
        >>> # Subsequent moves skip joint2
        >>> await degraded.move({"joint1": 45.0, "joint2": 90.0})
        >>> # Only joint1 moves
    """

    def __init__(
        self,
        controller: Controller,
        *,
        max_degraded_ratio: float = 0.5,
    ):
        self.controller = controller
        self.max_degraded_ratio = max_degraded_ratio

        self._degraded_actuators: dict[str, DegradedComponent] = {}
        self._degraded_sensors: dict[str, DegradedComponent] = {}
        self._last_known_sensor_values: dict[str, Any] = {}

    @property
    def name(self) -> str:
        """Get controller name."""
        return getattr(self.controller, "name", "unknown")

    @property
    def is_degraded(self) -> bool:
        """Check if operating in degraded mode."""
        return bool(self._degraded_actuators or self._degraded_sensors)

    @property
    def degraded_actuators(self) -> set[str]:
        """Get set of degraded actuator names."""
        return set(self._degraded_actuators.keys())

    @property
    def degraded_sensors(self) -> set[str]:
        """Get set of degraded sensor names."""
        return set(self._degraded_sensors.keys())

    def mark_actuator_degraded(
        self,
        actuator: str,
        reason: str,
        *,
        last_position: float | None = None,
    ) -> None:
        """Mark an actuator as degraded.

        Args:
            actuator: Name of the actuator.
            reason: Reason for degradation.
            last_position: Last known position (optional).
        """
        if actuator not in self._degraded_actuators:
            self._degraded_actuators[actuator] = DegradedComponent(
                name=actuator,
                reason=reason,
                last_known_value=last_position,
            )
            logger.warning(
                "Actuator '%s' marked degraded: %s",
                actuator,
                reason,
                extra={
                    "controller": self.name,
                    "actuator": actuator,
                    "reason": reason,
                },
            )

    def mark_sensor_degraded(
        self,
        sensor: str,
        reason: str,
        *,
        last_value: Any = None,
    ) -> None:
        """Mark a sensor as degraded.

        Args:
            sensor: Name of the sensor.
            reason: Reason for degradation.
            last_value: Last known good value (optional).
        """
        if sensor not in self._degraded_sensors:
            self._degraded_sensors[sensor] = DegradedComponent(
                name=sensor,
                reason=reason,
                last_known_value=last_value,
            )
            if last_value is not None:
                self._last_known_sensor_values[sensor] = last_value
            logger.warning(
                "Sensor '%s' marked degraded: %s",
                sensor,
                reason,
                extra={
                    "controller": self.name,
                    "sensor": sensor,
                    "reason": reason,
                },
            )

    def restore_actuator(self, actuator: str) -> bool:
        """Restore an actuator from degraded state.

        Args:
            actuator: Name of the actuator.

        Returns:
            True if actuator was restored, False if not degraded.
        """
        if actuator in self._degraded_actuators:
            del self._degraded_actuators[actuator]
            logger.info(
                "Actuator '%s' restored from degraded state",
                actuator,
                extra={"controller": self.name, "actuator": actuator},
            )
            return True
        return False

    def restore_sensor(self, sensor: str) -> bool:
        """Restore a sensor from degraded state.

        Args:
            sensor: Name of the sensor.

        Returns:
            True if sensor was restored, False if not degraded.
        """
        if sensor in self._degraded_sensors:
            del self._degraded_sensors[sensor]
            logger.info(
                "Sensor '%s' restored from degraded state",
                sensor,
                extra={"controller": self.name, "sensor": sensor},
            )
            return True
        return False

    def restore_all(self) -> int:
        """Restore all degraded components.

        Returns:
            Number of components restored.
        """
        count = len(self._degraded_actuators) + len(self._degraded_sensors)
        self._degraded_actuators.clear()
        self._degraded_sensors.clear()
        if count > 0:
            logger.info(
                "Restored %d components from degraded state",
                count,
                extra={"controller": self.name},
            )
        return count

    def _filter_targets(
        self,
        targets: dict[str, float],
    ) -> tuple[dict[str, float], dict[str, float]]:
        """Filter out degraded actuators from targets.

        Args:
            targets: Target positions by actuator name.

        Returns:
            Tuple of (valid_targets, skipped_targets).
        """
        valid = {}
        skipped = {}

        for actuator, position in targets.items():
            if actuator in self._degraded_actuators:
                skipped[actuator] = position
            else:
                valid[actuator] = position

        return valid, skipped

    def _check_degradation_limit(self) -> None:
        """Check if too many actuators are degraded.

        Raises:
            RuntimeError: If degradation exceeds limit.
        """
        total_actuators = len(getattr(self.controller, "actuators", {}))
        if total_actuators == 0:
            return

        ratio = len(self._degraded_actuators) / total_actuators
        if ratio >= self.max_degraded_ratio:
            raise RuntimeError(
                f"Too many actuators degraded ({len(self._degraded_actuators)}/{total_actuators}). "
                f"Refusing operation for safety."
            )

    async def move(
        self,
        targets: dict[str, float],
        **kwargs: Any,
    ) -> DegradedModeStatus:
        """Move actuators, skipping any that are degraded.

        Args:
            targets: Target positions by actuator name.
            **kwargs: Additional arguments passed to controller.move().

        Returns:
            Status indicating degraded operation details.

        Raises:
            RuntimeError: If too many actuators are degraded.
        """
        self._check_degradation_limit()

        valid_targets, skipped_targets = self._filter_targets(targets)

        status = DegradedModeStatus(
            is_degraded=bool(skipped_targets),
            degraded_actuators=self.degraded_actuators,
            degraded_sensors=self.degraded_sensors,
            skipped_targets=skipped_targets,
        )

        if skipped_targets:
            status.warnings.append(f"Skipping degraded actuators: {list(skipped_targets.keys())}")
            logger.warning(
                "Moving in degraded mode, skipping: %s",
                list(skipped_targets.keys()),
                extra={
                    "controller": self.name,
                    "skipped": list(skipped_targets.keys()),
                    "valid": list(valid_targets.keys()),
                },
            )

        if valid_targets:
            await self.controller.move(valid_targets, **kwargs)

        return status

    async def read_sensor(
        self,
        sensor: str,
        *,
        use_cached_on_failure: bool = True,
    ) -> tuple[Any, bool]:
        """Read a sensor value, with fallback to cached value if degraded.

        Args:
            sensor: Sensor name.
            use_cached_on_failure: If True and sensor is degraded, return
                last known value instead of raising.

        Returns:
            Tuple of (value, is_cached) where is_cached indicates if
            the value is from cache.

        Raises:
            KeyError: If sensor is degraded and no cached value available.
        """
        if sensor in self._degraded_sensors:
            if use_cached_on_failure and sensor in self._last_known_sensor_values:
                logger.debug(
                    "Using cached value for degraded sensor '%s'",
                    sensor,
                    extra={"controller": self.name, "sensor": sensor},
                )
                return self._last_known_sensor_values[sensor], True
            raise KeyError(f"Sensor '{sensor}' is degraded with no cached value")

        # Try to read from controller
        try:
            if hasattr(self.controller, "read_sensor"):
                value = await self.controller.read_sensor(sensor)
            elif hasattr(self.controller, "sensors"):
                sensors_attr = self.controller.sensors
                # Handle both callable and property/dict
                sensors_data = sensors_attr() if callable(sensors_attr) else sensors_attr
                value = sensors_data.get(sensor)
            else:
                raise AttributeError("Controller has no sensor reading capability")

            # Cache the value
            self._last_known_sensor_values[sensor] = value
            return value, False

        except Exception as e:
            # Mark as degraded and try cached value
            self.mark_sensor_degraded(sensor, str(e))
            if use_cached_on_failure and sensor in self._last_known_sensor_values:
                return self._last_known_sensor_values[sensor], True
            raise

    def status(self) -> DegradedModeStatus:
        """Get current degraded mode status.

        Returns:
            Status with degradation information.
        """
        return DegradedModeStatus(
            is_degraded=self.is_degraded,
            degraded_actuators=self.degraded_actuators,
            degraded_sensors=self.degraded_sensors,
        )

    def get_degraded_info(self) -> dict[str, Any]:
        """Get detailed information about degraded components.

        Returns:
            Dictionary with actuator and sensor degradation details.
        """
        return {
            "actuators": {
                name: {
                    "reason": comp.reason,
                    "degraded_at": comp.degraded_at.isoformat(),
                    "last_known_value": comp.last_known_value,
                }
                for name, comp in self._degraded_actuators.items()
            },
            "sensors": {
                name: {
                    "reason": comp.reason,
                    "degraded_at": comp.degraded_at.isoformat(),
                    "last_known_value": comp.last_known_value,
                }
                for name, comp in self._degraded_sensors.items()
            },
        }


__all__ = [
    "DegradedComponent",
    "DegradedModeController",
    "DegradedModeStatus",
]
