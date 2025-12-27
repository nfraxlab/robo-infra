"""Observability integration for robotics controllers.

This module provides Prometheus metrics, health checks, and structured
logging for robo-infra controllers using svc-infra's observability
infrastructure.

Metrics:
    - robo_commands_total: Counter of commands by controller/type/status
    - robo_command_duration_seconds: Histogram of command execution time
    - robo_actuator_position: Gauge of actuator current position
    - robo_safety_triggers_total: Counter of safety events by type

Health Checks:
    - Controller readiness checks
    - Actuator communication checks
    - Safety system status

Example:
    >>> from robo_infra.integrations.observability import (
    ...     track_command,
    ...     record_position,
    ...     record_safety_trigger,
    ...     create_controller_health_check,
    ... )
    >>>
    >>> # Use decorator to instrument commands
    >>> @track_command("move")
    ... async def move(self, target: float):
    ...     ...
    >>>
    >>> # Record position updates
    >>> record_position("arm", "joint1", 45.0)
    >>>
    >>> # Add health check to registry
    >>> from svc_infra.health import HealthRegistry
    >>> registry = HealthRegistry()
    >>> registry.add("arm", create_controller_health_check(arm_controller))
"""

from __future__ import annotations

import functools
import logging
import time
from contextvars import ContextVar
from typing import TYPE_CHECKING, Any, ParamSpec, TypeVar


if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from robo_infra.core.actuator import Actuator
    from robo_infra.core.controller import Controller


logger = logging.getLogger(__name__)

# Type variables for generic decorator
P = ParamSpec("P")
R = TypeVar("R")


# =============================================================================
# Metrics (lazy initialization for optional prometheus-client dependency)
# =============================================================================


class _MetricsRegistry:
    """Singleton registry for Prometheus metrics.

    Uses lazy initialization to avoid requiring prometheus-client
    as a hard dependency.
    """

    _instance: _MetricsRegistry | None = None
    _initialized: bool = False

    # Metrics (initialized lazily)
    commands_total: Any = None
    command_duration_seconds: Any = None
    actuator_position: Any = None
    sensor_value: Any = None
    safety_triggers_total: Any = None

    def __new__(cls) -> _MetricsRegistry:
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def init(self) -> bool:
        """Initialize metrics lazily.

        Returns:
            True if metrics were initialized successfully, False otherwise.
        """
        if self._initialized:
            return True

        try:
            from svc_infra.obs.metrics.base import counter, gauge, histogram

            self.commands_total = counter(
                "robo_commands_total",
                "Total robotics commands executed",
                labels=["controller", "command", "status"],
            )

            self.command_duration_seconds = histogram(
                "robo_command_duration_seconds",
                "Duration of robotics command execution in seconds",
                labels=["controller", "command"],
                buckets=(0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0),
            )

            self.actuator_position = gauge(
                "robo_actuator_position",
                "Current position of robotics actuator",
                labels=["controller", "actuator"],
                multiprocess_mode="livesum",
            )

            self.sensor_value = gauge(
                "robo_sensor_value",
                "Current sensor reading value",
                labels=["controller", "sensor", "unit"],
                multiprocess_mode="livesum",
            )

            self.safety_triggers_total = counter(
                "robo_safety_triggers_total",
                "Total safety events triggered",
                labels=["controller", "trigger_type"],
            )

            self._initialized = True
            logger.debug("Robotics metrics initialized successfully")
            return True

        except ImportError:
            logger.debug(
                "prometheus-client not available, metrics disabled. "
                "Install via 'pip install svc-infra[metrics]'"
            )
            return False
        except Exception as e:
            logger.warning(f"Failed to initialize metrics: {e}")
            return False


# Module-level singleton
_metrics = _MetricsRegistry()


def _init_metrics() -> bool:
    """Initialize metrics lazily.

    Returns:
        True if metrics were initialized successfully, False otherwise.
    """
    return _metrics.init()


# =============================================================================
# Metric Recording Functions
# =============================================================================


def record_command(
    controller: str,
    command: str,
    status: str,
    duration_seconds: float | None = None,
) -> None:
    """Record a command execution metric.

    Args:
        controller: Name of the controller (e.g., "arm", "conveyor").
        command: Type of command (e.g., "move", "home", "stop").
        status: Result status ("success", "error", "timeout").
        duration_seconds: Optional execution duration in seconds.
    """
    if not _init_metrics():
        return

    try:
        _metrics.commands_total.labels(
            controller=controller,
            command=command,
            status=status,
        ).inc()

        if duration_seconds is not None and _metrics.command_duration_seconds:
            _metrics.command_duration_seconds.labels(
                controller=controller,
                command=command,
            ).observe(duration_seconds)
    except Exception as e:
        # Never break the application for metrics failures
        logger.debug(f"Failed to record command metric: {e}")


def record_position(
    controller: str,
    actuator: str,
    position: float,
) -> None:
    """Record current actuator position.

    Args:
        controller: Name of the controller.
        actuator: Name of the actuator/joint.
        position: Current position value.
    """
    if not _init_metrics():
        return

    try:
        _metrics.actuator_position.labels(
            controller=controller,
            actuator=actuator,
        ).set(position)
    except Exception as e:
        logger.debug(f"Failed to record position metric: {e}")


def record_safety_trigger(
    controller: str,
    trigger_type: str,
) -> None:
    """Record a safety event.

    Args:
        controller: Name of the controller.
        trigger_type: Type of safety event (e.g., "estop", "limit_exceeded",
            "watchdog_timeout", "collision_detected").
    """
    if not _init_metrics():
        return

    try:
        _metrics.safety_triggers_total.labels(
            controller=controller,
            trigger_type=trigger_type,
        ).inc()
    except Exception as e:
        logger.debug(f"Failed to record safety trigger metric: {e}")


def record_sensor_value(
    controller: str,
    sensor: str,
    value: float,
    unit: str = "unknown",
) -> None:
    """Record current sensor reading.

    Args:
        controller: Name of the controller.
        sensor: Name of the sensor.
        value: Current sensor value.
        unit: Unit of measurement (e.g., "degrees", "celsius", "meters").
    """
    if not _init_metrics():
        return

    try:
        _metrics.sensor_value.labels(
            controller=controller,
            sensor=sensor,
            unit=unit,
        ).set(value)
    except Exception as e:
        logger.debug(f"Failed to record sensor value metric: {e}")


# =============================================================================
# Safety Trigger Type Constants (5.14.3)
# =============================================================================


class SafetyTriggerType:
    """Constants for safety trigger types.

    Use these with record_safety_trigger() for consistent labeling.

    Example:
        >>> record_safety_trigger("arm", SafetyTriggerType.ESTOP)
    """

    ESTOP = "estop"
    """Emergency stop triggered."""

    LIMIT_EXCEEDED = "limit_exceeded"
    """Joint or actuator limit exceeded."""

    WATCHDOG_TIMEOUT = "watchdog_timeout"
    """Watchdog timer expired without heartbeat."""

    MONITOR_ALERT = "monitor_alert"
    """Safety monitor detected unsafe condition."""

    COLLISION_DETECTED = "collision_detected"
    """Collision detection triggered."""

    COMMUNICATION_LOSS = "communication_loss"
    """Communication with hardware lost."""

    OVERCURRENT = "overcurrent"
    """Motor overcurrent detected."""

    OVERTEMPERATURE = "overtemperature"
    """Temperature limit exceeded."""


# Convenience functions for specific safety events (5.14.3)


def record_estop_triggered(controller: str) -> None:
    """Record emergency stop trigger event.

    Args:
        controller: Name of the controller.
    """
    record_safety_trigger(controller, SafetyTriggerType.ESTOP)
    logger.warning(
        f"E-Stop triggered on {controller}",
        extra={"controller": controller, "trigger_type": SafetyTriggerType.ESTOP},
    )


def record_limit_exceeded(
    controller: str,
    *,
    actuator: str | None = None,
    limit_type: str | None = None,
) -> None:
    """Record limit violation event.

    Args:
        controller: Name of the controller.
        actuator: Optional actuator that exceeded limits.
        limit_type: Optional type of limit ("min", "max", "velocity", "torque").
    """
    record_safety_trigger(controller, SafetyTriggerType.LIMIT_EXCEEDED)
    logger.warning(
        f"Limit exceeded on {controller}" + (f" ({actuator})" if actuator else ""),
        extra={
            "controller": controller,
            "trigger_type": SafetyTriggerType.LIMIT_EXCEEDED,
            "actuator": actuator,
            "limit_type": limit_type,
        },
    )


def record_watchdog_timeout(controller: str) -> None:
    """Record watchdog timeout event.

    Args:
        controller: Name of the controller.
    """
    record_safety_trigger(controller, SafetyTriggerType.WATCHDOG_TIMEOUT)
    logger.warning(
        f"Watchdog timeout on {controller}",
        extra={
            "controller": controller,
            "trigger_type": SafetyTriggerType.WATCHDOG_TIMEOUT,
        },
    )


def record_monitor_alert(
    controller: str,
    *,
    condition: str | None = None,
) -> None:
    """Record safety monitor alert event.

    Args:
        controller: Name of the controller.
        condition: Optional description of the unsafe condition.
    """
    record_safety_trigger(controller, SafetyTriggerType.MONITOR_ALERT)
    logger.warning(
        f"Monitor alert on {controller}" + (f": {condition}" if condition else ""),
        extra={
            "controller": controller,
            "trigger_type": SafetyTriggerType.MONITOR_ALERT,
            "condition": condition,
        },
    )


# =============================================================================
# Command Tracking Decorator
# =============================================================================


def track_command(
    command_name: str,
    *,
    controller_attr: str = "name",
) -> Callable[[Callable[P, Awaitable[R]]], Callable[P, Awaitable[R]]]:
    """Decorator to track command execution with metrics.

    Automatically records:
    - Command count (success/error)
    - Command duration histogram

    Args:
        command_name: Name of the command for metrics labeling.
        controller_attr: Attribute name on self to get controller name.

    Returns:
        Decorated async function with metrics tracking.

    Example:
        >>> class MyController:
        ...     name = "arm"
        ...
        ...     @track_command("move")
        ...     async def move(self, target: float) -> dict:
        ...         # This will be tracked automatically
        ...         ...
    """

    def decorator(fn: Callable[P, Awaitable[R]]) -> Callable[P, Awaitable[R]]:
        @functools.wraps(fn)
        async def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
            # Extract controller name from self
            controller_name = "unknown"
            if args and hasattr(args[0], controller_attr):
                controller_name = getattr(args[0], controller_attr, "unknown")

            start = time.perf_counter()
            status = "success"

            try:
                result = await fn(*args, **kwargs)
                return result
            except Exception as e:
                status = "error"
                # Log with structured context
                logger.warning(
                    f"Command {command_name} failed on {controller_name}: {e}",
                    extra={
                        "controller": controller_name,
                        "command": command_name,
                        "error": str(e),
                        "error_type": type(e).__name__,
                    },
                )
                raise
            finally:
                duration = time.perf_counter() - start
                record_command(
                    controller=controller_name,
                    command=command_name,
                    status=status,
                    duration_seconds=duration,
                )

        return wrapper

    return decorator


# =============================================================================
# Health Check Integration
# =============================================================================


def create_controller_health_check(
    controller: Controller,
    *,
    timeout: float = 5.0,
) -> Callable[[], Awaitable[Any]]:
    """Create a health check function for a controller.

    The health check verifies:
    - Controller is not in error state
    - Controller is enabled (not disabled)
    - Safety systems are not triggered

    Args:
        controller: The controller to check.
        timeout: Timeout for the health check in seconds.

    Returns:
        Async function suitable for svc-infra HealthRegistry.

    Example:
        >>> from svc_infra.health import HealthRegistry
        >>> registry = HealthRegistry()
        >>> registry.add("arm", create_controller_health_check(arm_controller))
    """

    async def health_check() -> Any:
        """Check controller health."""
        from svc_infra.health import HealthCheckResult, HealthStatus

        start = time.perf_counter()
        name = getattr(controller, "name", "controller")

        try:
            # Check if controller has status method
            if hasattr(controller, "status"):
                # Controller.status() returns ControllerStatus (sync)
                status_obj = controller.status()
                # Convert to dict for .get() access
                status_data: dict[str, Any] = {}
                if hasattr(status_obj, "__dict__"):
                    status_data = vars(status_obj)
                elif hasattr(status_obj, "model_dump"):
                    status_data = status_obj.model_dump()
                elif isinstance(status_obj, dict):
                    status_data = status_obj

                # Check for error states
                if status_data.get("error"):
                    return HealthCheckResult(
                        name=name,
                        status=HealthStatus.UNHEALTHY,
                        latency_ms=(time.perf_counter() - start) * 1000,
                        message=f"Controller in error state: {status_data.get('error')}",
                        details=status_data,
                    )

                # Check if disabled
                if not status_data.get("enabled", True):
                    return HealthCheckResult(
                        name=name,
                        status=HealthStatus.DEGRADED,
                        latency_ms=(time.perf_counter() - start) * 1000,
                        message="Controller is disabled",
                        details=status_data,
                    )

                # Check safety status
                safety = status_data.get("safety", {})
                if safety.get("estop_active"):
                    return HealthCheckResult(
                        name=name,
                        status=HealthStatus.UNHEALTHY,
                        latency_ms=(time.perf_counter() - start) * 1000,
                        message="Emergency stop active",
                        details=status_data,
                    )

            # All checks passed
            return HealthCheckResult(
                name=name,
                status=HealthStatus.HEALTHY,
                latency_ms=(time.perf_counter() - start) * 1000,
                message="Controller operational",
            )

        except Exception as e:
            return HealthCheckResult(
                name=name,
                status=HealthStatus.UNHEALTHY,
                latency_ms=(time.perf_counter() - start) * 1000,
                message=f"Health check failed: {e}",
            )

    return health_check


def create_actuator_health_check(
    actuator: Actuator,
    *,
    timeout: float = 5.0,
) -> Callable[[], Awaitable[Any]]:
    """Create a health check function for an actuator.

    The health check verifies:
    - Actuator communication is working
    - Actuator is not in fault state

    Args:
        actuator: The actuator to check.
        timeout: Timeout for the health check in seconds.

    Returns:
        Async function suitable for svc-infra HealthRegistry.

    Example:
        >>> from svc_infra.health import HealthRegistry
        >>> registry = HealthRegistry()
        >>> registry.add("joint1", create_actuator_health_check(joint1))
    """

    async def health_check() -> Any:
        """Check actuator health."""
        from svc_infra.health import HealthCheckResult, HealthStatus

        start = time.perf_counter()
        name = getattr(actuator, "name", "actuator")

        try:
            # Try to get current position to verify communication
            if hasattr(actuator, "current_position"):
                position = actuator.current_position()
                if position is None:
                    return HealthCheckResult(
                        name=name,
                        status=HealthStatus.UNKNOWN,
                        latency_ms=(time.perf_counter() - start) * 1000,
                        message="Position unknown",
                    )

            # Check for fault state
            if hasattr(actuator, "is_fault") and actuator.is_fault:
                return HealthCheckResult(
                    name=name,
                    status=HealthStatus.UNHEALTHY,
                    latency_ms=(time.perf_counter() - start) * 1000,
                    message="Actuator in fault state",
                )

            return HealthCheckResult(
                name=name,
                status=HealthStatus.HEALTHY,
                latency_ms=(time.perf_counter() - start) * 1000,
                message="Actuator operational",
            )

        except Exception as e:
            return HealthCheckResult(
                name=name,
                status=HealthStatus.UNHEALTHY,
                latency_ms=(time.perf_counter() - start) * 1000,
                message=f"Health check failed: {e}",
            )

    return health_check


def register_controller_health_checks(
    registry: Any,  # HealthRegistry
    controllers: list[Controller],
    *,
    timeout: float = 5.0,
    prefix: str = "controller:",
) -> None:
    """Register health checks for multiple controllers.

    Convenience function to add health checks for a list of controllers
    to a svc-infra HealthRegistry.

    Args:
        registry: svc-infra HealthRegistry instance.
        controllers: List of controllers to register.
        timeout: Timeout for each health check in seconds.
        prefix: Prefix for health check names (e.g., "controller:arm").

    Example:
        >>> from svc_infra.health import HealthRegistry
        >>> registry = HealthRegistry()
        >>> register_controller_health_checks(registry, [arm, conveyor])
        >>> # Creates checks: "controller:arm", "controller:conveyor"
    """
    for controller in controllers:
        name = getattr(controller, "name", f"controller_{id(controller)}")
        check_fn = create_controller_health_check(controller, timeout=timeout)
        registry.add(f"{prefix}{name}", check_fn, critical=True, timeout=timeout)
        logger.debug(f"Registered health check for controller: {prefix}{name}")


def add_robotics_health_routes(
    app: Any,  # FastAPI
    controllers: list[Controller],
    *,
    prefix: str = "/_health",
    timeout: float = 5.0,
) -> Any:  # HealthRegistry
    """Add health check routes for robotics controllers to a FastAPI app.

    Creates a HealthRegistry, registers all controller health checks,
    and adds probe endpoints (/_health/live, /_health/ready, etc.).

    Args:
        app: FastAPI application instance.
        controllers: List of controllers to monitor.
        prefix: URL prefix for health routes.
        timeout: Timeout for health checks in seconds.

    Returns:
        The HealthRegistry instance for additional customization.

    Example:
        >>> from fastapi import FastAPI
        >>> app = FastAPI()
        >>> registry = add_robotics_health_routes(app, [arm, conveyor])
        >>> # Adds routes: /_health/live, /_health/ready, /_health/startup
    """
    from svc_infra.health import HealthRegistry, add_health_routes

    registry = HealthRegistry()
    register_controller_health_checks(
        registry,
        controllers,
        timeout=timeout,
    )
    add_health_routes(app, registry, prefix=prefix)

    logger.info(
        f"Robotics health routes added at {prefix}",
        extra={
            "prefix": prefix,
            "controller_count": len(controllers),
        },
    )

    return registry


# =============================================================================
# Structured Logging Setup
# =============================================================================


# Context variable for robotics correlation ID (uses svc-infra when available)
_robotics_correlation_id: ContextVar[str | None] = ContextVar("robotics_request_id", default=None)


def get_robotics_request_id() -> str | None:
    """Get the current request/correlation ID for logging.

    Uses svc-infra's request ID context when available,
    falls back to robotics-specific context variable.

    Returns:
        The current correlation ID, or None if not set.
    """
    try:
        from svc_infra.http.client import get_request_id

        request_id = get_request_id()
        if request_id:
            return request_id
    except ImportError:
        pass

    # Fallback to local context
    return _robotics_correlation_id.get()


def set_robotics_request_id(request_id: str | None) -> None:
    """Set the current request/correlation ID.

    Uses svc-infra's request ID context when available,
    falls back to robotics-specific context variable.

    Args:
        request_id: The correlation ID to set, or None to clear.
    """
    try:
        from svc_infra.http.client import set_request_id

        set_request_id(request_id)
        return
    except ImportError:
        pass

    # Fallback to local context
    _robotics_correlation_id.set(request_id)


def log_with_context(
    level: str,
    message: str,
    *,
    controller: str | None = None,
    actuator: str | None = None,
    command: str | None = None,
    **extra: Any,
) -> None:
    """Log a message with robotics context automatically included.

    Automatically adds:
    - request_id: Current correlation ID (if set)
    - controller: Controller name (if provided)
    - actuator: Actuator name (if provided)
    - command: Command name (if provided)

    Args:
        level: Log level ("debug", "info", "warning", "error").
        message: Log message.
        controller: Optional controller name for context.
        actuator: Optional actuator name for context.
        command: Optional command name for context.
        **extra: Additional context fields to include.

    Example:
        >>> log_with_context(
        ...     "info",
        ...     "Move command completed",
        ...     controller="arm",
        ...     actuator="joint1",
        ...     target=45.0,
        ...     duration_ms=150,
        ... )
    """
    context: dict[str, Any] = {}

    # Add correlation ID
    request_id = get_robotics_request_id()
    if request_id:
        context["request_id"] = request_id

    # Add robotics context
    if controller:
        context["controller"] = controller
    if actuator:
        context["actuator"] = actuator
    if command:
        context["command"] = command

    # Merge additional context
    context.update(extra)

    # Get appropriate logger method
    log_fn = getattr(logger, level.lower(), logger.info)
    log_fn(message, extra=context)


def setup_robotics_logging(
    *,
    level: str = "INFO",
    json_output: bool | None = None,
) -> None:
    """Configure structured logging for robotics applications.

    Uses svc-infra's logging infrastructure when available, with
    fallback to standard Python logging.

    Args:
        level: Log level (DEBUG, INFO, WARNING, ERROR).
        json_output: Force JSON output (True) or plain text (False).
            If None, uses svc-infra's environment-based detection.

    Example:
        >>> setup_robotics_logging(level="DEBUG", json_output=True)
    """
    try:
        from svc_infra.app import setup_logging

        # svc-infra uses 'fmt' parameter, not 'json'
        fmt = "json" if json_output else ("plain" if json_output is False else None)
        setup_logging(level=level, fmt=fmt)
        logger.info(
            "Robotics logging configured via svc-infra",
            extra={"level": level, "fmt": fmt},
        )

    except ImportError:
        # Fallback to standard logging
        logging.basicConfig(
            level=getattr(logging, level.upper(), logging.INFO),
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        )
        logger.info(f"Robotics logging configured (standard), level={level}")


# =============================================================================
# Public API
# =============================================================================

__all__ = [
    "SafetyTriggerType",
    "add_robotics_health_routes",
    "create_actuator_health_check",
    "create_controller_health_check",
    "get_robotics_request_id",
    "log_with_context",
    "record_command",
    "record_estop_triggered",
    "record_limit_exceeded",
    "record_monitor_alert",
    "record_position",
    "record_safety_trigger",
    "record_sensor_value",
    "record_watchdog_timeout",
    "register_controller_health_checks",
    "set_robotics_request_id",
    "setup_robotics_logging",
    "track_command",
]
