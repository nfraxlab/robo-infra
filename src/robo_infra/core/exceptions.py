"""Exceptions for robo-infra.

This module provides exception classes for robotics operations.
All exceptions inherit from RoboInfraError, which follows the
svc-infra pattern with error codes and structured context.

Example:
    >>> from robo_infra.core.exceptions import (
    ...     RoboInfraError,
    ...     SafetyError,
    ...     TimeoutError,
    ...     log_exception,
    ... )
    >>>
    >>> try:
    ...     await controller.move(target)
    ... except SafetyError as e:
    ...     log_exception(logger, "Move failed", e)
    ...     raise
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any


if TYPE_CHECKING:
    import logging


# =============================================================================
# Logging Helper (matches svc-infra pattern)
# =============================================================================


def log_exception(
    logger: logging.Logger,
    msg: str,
    exc: Exception,
    *,
    level: str = "warning",
    include_traceback: bool = True,
) -> None:
    """Log an exception with consistent formatting.

    Use this helper instead of bare `except Exception:` blocks to ensure
    all exceptions are properly logged with context.

    Args:
        logger: The logger instance to use
        msg: Context message describing what operation failed
        exc: The exception that was caught
        level: Log level - "debug", "info", "warning", "error", "critical"
        include_traceback: Whether to include full traceback (exc_info=True)

    Example:
        >>> try:
        ...     result = await controller.move(target)
        ... except CommunicationError as e:
        ...     log_exception(logger, "Failed to move actuator", e)
        ...     # Handle gracefully or re-raise
    """
    log_func = getattr(logger, level.lower(), logger.warning)
    log_func(f"{msg}: {type(exc).__name__}: {exc}", exc_info=include_traceback)


# =============================================================================
# Base Exception (follows svc-infra pattern)
# =============================================================================


class RoboInfraError(Exception):
    """Base exception for all robo-infra errors.

    All robo-infra exceptions can be caught with this single class.
    Each subclass has an error code for categorization and structured
    details for debugging.

    Attributes:
        code: Error code for categorization (e.g., "ROBO_SAFETY").
        message: Human-readable error description.
        details: Additional context as key-value pairs.
    """

    code: str = "ROBO_ERROR"

    def __init__(
        self,
        message: str,
        *,
        details: dict[str, Any] | None = None,
    ):
        self.message = message
        self.details = details or {}
        super().__init__(message)

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.message!r}, code={self.code!r})"


# =============================================================================
# Hardware Exceptions
# =============================================================================


class HardwareNotFoundError(RoboInfraError):
    """Raised when expected hardware is not detected."""

    code: str = "ROBO_HARDWARE_NOT_FOUND"

    def __init__(self, device: str, details: str | None = None) -> None:
        self.device = device
        message = f"Hardware not found: {device}"
        if details:
            message += f" - {details}"
        super().__init__(message, details={"device": device, "info": details})


class CommunicationError(RoboInfraError):
    """Raised when communication with hardware fails."""

    code: str = "ROBO_COMMUNICATION"

    def __init__(
        self, bus: str, address: int | str | None = None, details: str | None = None
    ) -> None:
        self.bus = bus
        self.address = address
        message = f"Communication error on {bus}"
        if address is not None:
            message += f" at address {address}"
        if details:
            message += f": {details}"
        super().__init__(
            message,
            details={"bus": bus, "address": address, "info": details},
        )


class ConnectionLostError(CommunicationError):
    """Raised when hardware connection is lost."""

    code: str = "ROBO_CONNECTION_LOST"

    def __init__(self, device: str, reason: str | None = None) -> None:
        self.device_name = device
        message = f"Connection lost to {device}"
        if reason:
            message += f": {reason}"
        # CommunicationError expects bus, address, details
        super().__init__(bus=device, address=None, details=reason)


# =============================================================================
# Safety Exceptions
# =============================================================================


class SafetyError(RoboInfraError):
    """Raised when a safety limit or condition is triggered."""

    code: str = "ROBO_SAFETY"

    def __init__(self, condition: str, action_taken: str | None = None) -> None:
        self.condition = condition
        self.action_taken = action_taken
        message = f"Safety triggered: {condition}"
        if action_taken:
            message += f" - Action: {action_taken}"
        super().__init__(
            message,
            details={"condition": condition, "action_taken": action_taken},
        )


class LimitsExceededError(SafetyError):
    """Raised when a value exceeds defined limits."""

    code: str = "ROBO_LIMITS_EXCEEDED"

    value: float | None
    min_limit: float | None
    max_limit: float | None
    name: str | None

    def __init__(
        self,
        value_or_message: float | str | None = None,
        min_limit: float | None = None,
        max_limit: float | None = None,
        name: str | None = None,
        *,
        value: float | None = None,
    ) -> None:
        # Support both positional and keyword 'value' argument
        if value is not None:
            # Keyword-arg format: LimitsExceededError(value=10, min_limit=0, ...)
            self.value = value
            self.min_limit = min_limit
            self.max_limit = max_limit
            self.name = name
            name_str = f" for {name}" if name else ""
            message = f"Value {value} exceeds limits [{min_limit}, {max_limit}]{name_str}"
        elif isinstance(value_or_message, str):
            # Simple message format: LimitsExceededError("some message")
            self.value = None
            self.min_limit = min_limit
            self.max_limit = max_limit
            self.name = name
            message = value_or_message
        elif value_or_message is not None:
            # Positional format: LimitsExceededError(10.0, 0.0, 5.0, "name")
            self.value = value_or_message
            self.min_limit = min_limit
            self.max_limit = max_limit
            self.name = name
            name_str = f" for {name}" if name else ""
            message = (
                f"Value {value_or_message} exceeds limits [{min_limit}, {max_limit}]{name_str}"
            )
        else:
            # Fallback
            self.value = None
            self.min_limit = min_limit
            self.max_limit = max_limit
            self.name = name
            message = "Value exceeds limits"

        # Call SafetyError with condition
        condition = f"limits_exceeded: {message}"
        # Override parent __init__ to set our own attributes
        Exception.__init__(self, message)
        self.message = message
        self.condition = condition
        self.action_taken = None
        self.details = {
            "condition": condition,
            "value": self.value,
            "min_limit": self.min_limit,
            "max_limit": self.max_limit,
            "name": self.name,
        }


# =============================================================================
# Operation Exceptions
# =============================================================================


class TimeoutError(RoboInfraError):
    """Raised when an operation times out."""

    code: str = "ROBO_TIMEOUT"

    def __init__(self, operation: str, timeout: float) -> None:
        self.operation = operation
        self.timeout = timeout
        message = f"Operation '{operation}' timed out after {timeout}s"
        super().__init__(message, details={"operation": operation, "timeout": timeout})


class ResourceExhaustedError(RoboInfraError):
    """Raised when a resource is exhausted (buffer full, queue full)."""

    code: str = "ROBO_RESOURCE_EXHAUSTED"

    def __init__(self, resource: str, limit: int | None = None) -> None:
        self.resource = resource
        self.limit = limit
        message = f"Resource exhausted: {resource}"
        if limit is not None:
            message += f" (limit: {limit})"
        super().__init__(message, details={"resource": resource, "limit": limit})


class StateError(RoboInfraError):
    """Raised when an invalid state transition is attempted."""

    code: str = "ROBO_STATE"

    def __init__(
        self,
        message: str,
        *,
        current_state: str | None = None,
        attempted_state: str | None = None,
    ) -> None:
        self.current_state = current_state
        self.attempted_state = attempted_state
        super().__init__(
            message,
            details={
                "current_state": current_state,
                "attempted_state": attempted_state,
            },
        )


class DisabledError(StateError):
    """Raised when trying to use a disabled component."""

    code: str = "ROBO_DISABLED"

    def __init__(self, component: str) -> None:
        self.component = component
        message = f"Component '{component}' is disabled"
        super().__init__(message, current_state="disabled", attempted_state="enabled")


# =============================================================================
# Configuration Exceptions
# =============================================================================


class ConfigurationError(RoboInfraError):
    """Raised when configuration is invalid or missing."""

    code: str = "ROBO_CONFIGURATION"

    def __init__(self, setting: str, reason: str | None = None) -> None:
        self.setting = setting
        self.reason = reason
        message = f"Configuration error for '{setting}'"
        if reason:
            message += f": {reason}"
        super().__init__(message, details={"setting": setting, "reason": reason})


# =============================================================================
# Calibration Exceptions
# =============================================================================


class CalibrationError(RoboInfraError):
    """Raised when calibration fails or is required."""

    code: str = "ROBO_CALIBRATION"

    def __init__(self, component: str, reason: str | None = None) -> None:
        self.component = component
        self.reason = reason
        message = f"Calibration error for {component}"
        if reason:
            message += f": {reason}"
        super().__init__(message, details={"component": component, "reason": reason})


class NotCalibratedError(CalibrationError):
    """Raised when an operation requires calibration that hasn't been done."""

    code: str = "ROBO_NOT_CALIBRATED"

    def __init__(self, component: str) -> None:
        super().__init__(component, "Component requires calibration before use")


# =============================================================================
# Kinematics Exceptions
# =============================================================================


class KinematicsError(RoboInfraError):
    """Raised when kinematic calculations fail (e.g., position unreachable)."""

    code: str = "ROBO_KINEMATICS"

    def __init__(self, message: str, *, target: Any = None) -> None:
        super().__init__(message, details={"target": target})


# =============================================================================
# Public API
# =============================================================================

__all__ = [
    # Calibration
    "CalibrationError",
    # Hardware
    "CommunicationError",
    # Configuration
    "ConfigurationError",
    "ConnectionLostError",
    # Operation
    "DisabledError",
    "HardwareNotFoundError",
    # Kinematics
    "KinematicsError",
    # Safety
    "LimitsExceededError",
    "NotCalibratedError",
    "ResourceExhaustedError",
    # Base
    "RoboInfraError",
    "SafetyError",
    "StateError",
    "TimeoutError",
    # Logging helper
    "log_exception",
]
