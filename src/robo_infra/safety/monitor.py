"""Safety monitoring for current, temperature, and other safety metrics.

This module provides monitoring systems that watch for dangerous conditions
and trigger E-stop when limits are exceeded.

Example:
    >>> from robo_infra.safety import SafetyMonitor, EStop, CurrentLimit
    >>>
    >>> estop = EStop()
    >>> monitor = SafetyMonitor(estop=estop)
    >>>
    >>> # Add current monitoring
    >>> monitor.add_current_limit("motor1", max_current=5.0)
    >>> monitor.add_temperature_limit("motor1", max_temp=80.0)
    >>>
    >>> # Start monitoring
    >>> monitor.start()
    >>>
    >>> # In control loop, update values
    >>> monitor.update_current("motor1", measured_current)
    >>> monitor.update_temperature("motor1", measured_temp)
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel

from robo_infra.core.exceptions import SafetyError


if TYPE_CHECKING:
    from collections.abc import Callable

    from robo_infra.safety.estop import EStop


logger = logging.getLogger(__name__)


class SafetyViolation(SafetyError):
    """Raised when a safety limit is exceeded."""

    def __init__(
        self,
        component: str,
        metric: str,
        value: float,
        limit: float,
        unit: str = "",
    ) -> None:
        self.component = component
        self.metric = metric
        self.value = value
        self.limit = limit
        self.unit = unit
        message = f"Safety violation: {component} {metric} = {value}{unit} (limit: {limit}{unit})"
        super().__init__(message, action_taken="E-stop triggered")


class MonitorState(Enum):
    """Safety monitor states."""

    STOPPED = "stopped"
    MONITORING = "monitoring"
    WARNING = "warning"  # Approaching limits
    VIOLATED = "violated"  # Limit exceeded, E-stop triggered


class SafetyLevel(Enum):
    """Safety limit severity levels."""

    NOTICE = "notice"  # Log only
    WARNING = "warning"  # Log + callback
    CRITICAL = "critical"  # Log + callback + E-stop


class LimitConfig(BaseModel):
    """Configuration for a monitored limit.

    Attributes:
        component: Name of the component being monitored.
        metric: What is being measured (current, temperature, etc.).
        unit: Unit of measurement.
        min_value: Minimum acceptable value (None = no limit).
        max_value: Maximum acceptable value (None = no limit).
        warning_min: Warn if value drops below this.
        warning_max: Warn if value exceeds this.
        level: What action to take on violation.
        hysteresis: Value must return this far within limits to clear.
    """

    component: str
    metric: str
    unit: str = ""
    min_value: float | None = None
    max_value: float | None = None
    warning_min: float | None = None
    warning_max: float | None = None
    level: SafetyLevel = SafetyLevel.CRITICAL
    hysteresis: float = 0.0

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class LimitStatus:
    """Current status of a monitored limit."""

    config: LimitConfig
    current_value: float = 0.0
    last_update: float = 0.0
    is_violated: bool = False
    is_warning: bool = False
    violation_count: int = 0


@dataclass
class MonitorStatus:
    """Overall status of the safety monitor."""

    state: MonitorState = MonitorState.STOPPED
    limits: dict[str, LimitStatus] = field(default_factory=dict)
    total_violations: int = 0
    last_violation_time: float | None = None


class SafetyMonitor:
    """Monitors safety-critical values and triggers E-stop on violations.

    SAFETY CRITICAL:
    - All limit violations trigger E-stop (if level=CRITICAL)
    - Monitoring continues even after violation
    - Thread-safe value updates
    - All events logged for auditing
    """

    def __init__(
        self,
        estop: EStop | None = None,
        *,
        check_interval: float = 0.01,
    ) -> None:
        """Initialize safety monitor.

        Args:
            estop: E-stop to trigger on critical violations.
            check_interval: How often to check limits (seconds).
        """
        self._estop = estop
        self._check_interval = check_interval
        self._state = MonitorState.STOPPED
        self._limits: dict[str, LimitConfig] = {}
        self._values: dict[str, tuple[float, float]] = {}  # (value, timestamp)
        self._statuses: dict[str, LimitStatus] = {}
        self._callbacks: list[Callable[[str, LimitStatus], None]] = []
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._total_violations = 0
        self._last_violation_time: float | None = None

    @property
    def state(self) -> MonitorState:
        """Current monitor state."""
        return self._state

    def status(self) -> MonitorStatus:
        """Get current monitor status."""
        with self._lock:
            return MonitorStatus(
                state=self._state,
                limits=dict(self._statuses),
                total_violations=self._total_violations,
                last_violation_time=self._last_violation_time,
            )

    def add_limit(self, config: LimitConfig) -> None:
        """Add a limit to monitor.

        Args:
            config: Limit configuration.
        """
        key = f"{config.component}:{config.metric}"
        with self._lock:
            self._limits[key] = config
            self._statuses[key] = LimitStatus(config=config)
            logger.info(
                "Added safety limit: %s %s [%s, %s] %s",
                config.component,
                config.metric,
                config.min_value,
                config.max_value,
                config.unit,
            )

    def add_current_limit(
        self,
        component: str,
        max_current: float,
        *,
        min_current: float | None = None,
        warning_threshold: float = 0.8,
        level: SafetyLevel = SafetyLevel.CRITICAL,
    ) -> None:
        """Convenience method to add current monitoring.

        Args:
            component: Name of the component.
            max_current: Maximum allowable current in amps.
            min_current: Minimum current (for detecting open circuits).
            warning_threshold: Warn at this fraction of max.
            level: Safety level for violations.
        """
        self.add_limit(
            LimitConfig(
                component=component,
                metric="current",
                unit="A",
                min_value=min_current,
                max_value=max_current,
                warning_max=max_current * warning_threshold if max_current else None,
                level=level,
            )
        )

    def add_temperature_limit(
        self,
        component: str,
        max_temp: float,
        *,
        min_temp: float | None = None,
        warning_threshold: float = 0.85,
        level: SafetyLevel = SafetyLevel.CRITICAL,
    ) -> None:
        """Convenience method to add temperature monitoring.

        Args:
            component: Name of the component.
            max_temp: Maximum allowable temperature in Celsius.
            min_temp: Minimum temperature (for detecting sensor failure).
            warning_threshold: Warn at this fraction of max.
            level: Safety level for violations.
        """
        self.add_limit(
            LimitConfig(
                component=component,
                metric="temperature",
                unit="°C",
                min_value=min_temp,
                max_value=max_temp,
                warning_max=max_temp * warning_threshold if max_temp else None,
                level=level,
            )
        )

    def add_voltage_limit(
        self,
        component: str,
        max_voltage: float,
        min_voltage: float | None = None,
        *,
        level: SafetyLevel = SafetyLevel.CRITICAL,
    ) -> None:
        """Convenience method to add voltage monitoring.

        Args:
            component: Name of the component.
            max_voltage: Maximum allowable voltage.
            min_voltage: Minimum voltage (for brownout detection).
            level: Safety level for violations.
        """
        self.add_limit(
            LimitConfig(
                component=component,
                metric="voltage",
                unit="V",
                min_value=min_voltage,
                max_value=max_voltage,
                level=level,
            )
        )

    def remove_limit(self, component: str, metric: str) -> None:
        """Remove a monitored limit.

        Args:
            component: Component name.
            metric: Metric name.
        """
        key = f"{component}:{metric}"
        with self._lock:
            self._limits.pop(key, None)
            self._statuses.pop(key, None)
            self._values.pop(key, None)

    def register_callback(
        self,
        callback: Callable[[str, LimitStatus], None],
    ) -> None:
        """Register callback for limit violations.

        Args:
            callback: Function called with (key, status) on violation.
        """
        self._callbacks.append(callback)

    def update_value(self, component: str, metric: str, value: float) -> None:
        """Update a monitored value.

        Call this regularly with sensor readings.

        Args:
            component: Component name.
            metric: Metric name (current, temperature, etc.).
            value: Current measured value.
        """
        key = f"{component}:{metric}"
        with self._lock:
            self._values[key] = (value, time.time())

    def update_current(self, component: str, value: float) -> None:
        """Convenience method to update current reading."""
        self.update_value(component, "current", value)

    def update_temperature(self, component: str, value: float) -> None:
        """Convenience method to update temperature reading."""
        self.update_value(component, "temperature", value)

    def update_voltage(self, component: str, value: float) -> None:
        """Convenience method to update voltage reading."""
        self.update_value(component, "voltage", value)

    def start(self) -> None:
        """Start the safety monitor."""
        if self._running:
            return

        self._running = True
        self._state = MonitorState.MONITORING

        self._thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="SafetyMonitor",
        )
        self._thread.start()
        logger.info("Safety monitor started with %d limits", len(self._limits))

    def stop(self) -> None:
        """Stop the safety monitor."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._state = MonitorState.STOPPED
        logger.info("Safety monitor stopped")

    def _monitor_loop(self) -> None:
        """Main monitoring loop."""
        while self._running:
            self._check_all_limits()
            time.sleep(self._check_interval)

    def _check_all_limits(self) -> None:
        """Check all limits and handle violations."""
        has_warning = False
        has_violation = False

        with self._lock:
            for key, config in self._limits.items():
                value_data = self._values.get(key)
                if value_data is None:
                    continue

                value, timestamp = value_data
                status = self._statuses[key]
                status.current_value = value
                status.last_update = timestamp

                # Check limits
                violation = self._check_limit(value, config)
                warning = self._check_warning(value, config)

                if violation and not status.is_violated:
                    # New violation
                    self._handle_violation(key, config, value, status)
                    has_violation = True
                elif warning and not status.is_warning:
                    # New warning
                    self._handle_warning(key, config, value, status)
                    has_warning = True
                elif not violation and status.is_violated:
                    # Cleared (with hysteresis)
                    if self._check_cleared(value, config):
                        status.is_violated = False
                        status.is_warning = False
                        logger.info("Safety limit cleared: %s", key)

        # Update overall state
        if has_violation:
            self._state = MonitorState.VIOLATED
        elif has_warning:
            self._state = MonitorState.WARNING
        else:
            self._state = MonitorState.MONITORING

    def _check_limit(self, value: float, config: LimitConfig) -> bool:
        """Check if value violates limits."""
        if config.min_value is not None and value < config.min_value:
            return True
        return bool(config.max_value is not None and value > config.max_value)

    def _check_warning(self, value: float, config: LimitConfig) -> bool:
        """Check if value is in warning zone."""
        if config.warning_min is not None and value < config.warning_min:
            return True
        return bool(config.warning_max is not None and value > config.warning_max)

    def _check_cleared(self, value: float, config: LimitConfig) -> bool:
        """Check if value has returned within limits (with hysteresis)."""
        if config.min_value is not None and value < config.min_value + config.hysteresis:
            return False
        return not (config.max_value is not None and value > config.max_value - config.hysteresis)

    def _handle_violation(
        self,
        key: str,
        config: LimitConfig,
        value: float,
        status: LimitStatus,
    ) -> None:
        """Handle a limit violation."""
        status.is_violated = True
        status.is_warning = True
        status.violation_count += 1
        self._total_violations += 1
        self._last_violation_time = time.time()

        logger.critical(
            "🚨 SAFETY VIOLATION: %s %s = %.3f%s (limit: %s to %s%s)",
            config.component,
            config.metric,
            value,
            config.unit,
            config.min_value,
            config.max_value,
            config.unit,
        )

        # Trigger E-stop for critical violations
        if config.level == SafetyLevel.CRITICAL and self._estop:
            try:
                self._estop.trigger(
                    reason=(
                        f"{config.component} {config.metric} = {value}{config.unit} exceeded limit"
                    ),
                    triggered_by="safety_monitor",
                )
            except Exception as e:
                logger.critical("Safety monitor E-stop trigger failed: %s", e)

        # Call callbacks
        for callback in self._callbacks:
            try:
                callback(key, status)
            except Exception as e:
                logger.error("Safety monitor callback failed: %s", e)

    def _handle_warning(
        self,
        key: str,
        config: LimitConfig,
        value: float,
        status: LimitStatus,
    ) -> None:
        """Handle a warning condition."""
        status.is_warning = True

        logger.warning(
            "⚠️ SAFETY WARNING: %s %s = %.3f%s approaching limit",
            config.component,
            config.metric,
            value,
            config.unit,
        )


# =============================================================================
# Collision Detection
# =============================================================================


class CollisionDetector:
    """Detects collisions using force/torque sensors or current monitoring.

    Example:
        >>> detector = CollisionDetector(
        ...     estop=estop,
        ...     force_threshold=10.0,  # Newtons
        ... )
        >>> detector.register_force_sensor(force_sensor)
        >>> detector.start()
    """

    def __init__(
        self,
        estop: EStop | None = None,
        *,
        force_threshold: float = 10.0,
        torque_threshold: float = 1.0,
        current_spike_threshold: float = 2.0,
        check_interval: float = 0.005,
    ) -> None:
        """Initialize collision detector.

        Args:
            estop: E-stop to trigger on collision.
            force_threshold: Force threshold in Newtons.
            torque_threshold: Torque threshold in Nm.
            current_spike_threshold: Current spike multiplier.
            check_interval: Check interval in seconds.
        """
        self._estop = estop
        self._force_threshold = force_threshold
        self._torque_threshold = torque_threshold
        self._current_spike_threshold = current_spike_threshold
        self._check_interval = check_interval
        self._force_sensors: list[Any] = []
        self._current_baselines: dict[str, float] = {}
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._collision_count = 0

    def register_force_sensor(self, sensor: Any) -> None:
        """Register a force/torque sensor for collision detection."""
        self._force_sensors.append(sensor)

    def set_current_baseline(self, actuator_name: str, baseline: float) -> None:
        """Set baseline current for spike detection."""
        with self._lock:
            self._current_baselines[actuator_name] = baseline

    def start(self) -> None:
        """Start collision detection."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(
            target=self._detect_loop,
            daemon=True,
            name="CollisionDetector",
        )
        self._thread.start()
        logger.info("Collision detector started")

    def stop(self) -> None:
        """Stop collision detection."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        logger.info("Collision detector stopped")

    def _detect_loop(self) -> None:
        """Main detection loop."""
        while self._running:
            # Check force sensors
            for sensor in self._force_sensors:
                try:
                    reading = sensor.read()
                    force = reading.value if hasattr(reading, "value") else float(reading)

                    if force > self._force_threshold:
                        self._handle_collision(f"Force sensor: {force:.2f}N")
                except Exception as e:
                    logger.error("Collision detection sensor read failed: %s", e)

            time.sleep(self._check_interval)

    def _handle_collision(self, reason: str) -> None:
        """Handle detected collision."""
        self._collision_count += 1
        logger.critical("💥 COLLISION DETECTED: %s", reason)

        if self._estop:
            try:
                self._estop.trigger(
                    reason=f"Collision detected: {reason}",
                    triggered_by="collision_detector",
                )
            except Exception as e:
                logger.critical("Collision E-stop trigger failed: %s", e)
