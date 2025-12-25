"""Exceptions for robo-infra."""

from __future__ import annotations


class RoboInfraError(Exception):
    """Base exception for all robo-infra errors."""

    pass


class HardwareNotFoundError(RoboInfraError):
    """Raised when expected hardware is not detected."""

    def __init__(self, device: str, details: str | None = None) -> None:
        self.device = device
        self.details = details
        message = f"Hardware not found: {device}"
        if details:
            message += f" - {details}"
        super().__init__(message)


class LimitsExceededError(RoboInfraError):
    """Raised when a value exceeds defined limits."""

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
            message = (
                f"Value {value} exceeds limits "
                f"[{min_limit}, {max_limit}]{name_str}"
            )
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
                f"Value {value_or_message} exceeds limits "
                f"[{min_limit}, {max_limit}]{name_str}"
            )
        else:
            # Fallback
            self.value = None
            self.min_limit = min_limit
            self.max_limit = max_limit
            self.name = name
            message = "Value exceeds limits"
        super().__init__(message)


class CommunicationError(RoboInfraError):
    """Raised when communication with hardware fails."""

    def __init__(
        self, bus: str, address: int | str | None = None, details: str | None = None
    ) -> None:
        self.bus = bus
        self.address = address
        self.details = details
        message = f"Communication error on {bus}"
        if address is not None:
            message += f" at address {address}"
        if details:
            message += f": {details}"
        super().__init__(message)


class CalibrationError(RoboInfraError):
    """Raised when calibration fails or is required."""

    def __init__(self, component: str, reason: str | None = None) -> None:
        self.component = component
        self.reason = reason
        message = f"Calibration error for {component}"
        if reason:
            message += f": {reason}"
        super().__init__(message)


class SafetyError(RoboInfraError):
    """Raised when a safety limit or condition is triggered."""

    def __init__(self, condition: str, action_taken: str | None = None) -> None:
        self.condition = condition
        self.action_taken = action_taken
        message = f"Safety triggered: {condition}"
        if action_taken:
            message += f" - Action: {action_taken}"
        super().__init__(message)


class TimeoutError(RoboInfraError):
    """Raised when an operation times out."""

    def __init__(self, operation: str, timeout: float) -> None:
        self.operation = operation
        self.timeout = timeout
        message = f"Operation '{operation}' timed out after {timeout}s"
        super().__init__(message)


class NotCalibratedError(CalibrationError):
    """Raised when an operation requires calibration that hasn't been done."""

    def __init__(self, component: str) -> None:
        super().__init__(component, "Component requires calibration before use")


class DisabledError(RoboInfraError):
    """Raised when trying to use a disabled component."""

    def __init__(self, component: str) -> None:
        self.component = component
        message = f"Component '{component}' is disabled"
        super().__init__(message)


class ConfigurationError(RoboInfraError):
    """Raised when configuration is invalid or missing."""

    def __init__(self, setting: str, reason: str | None = None) -> None:
        self.setting = setting
        self.reason = reason
        message = f"Configuration error for '{setting}'"
        if reason:
            message += f": {reason}"
        super().__init__(message)


class KinematicsError(RoboInfraError):
    """Raised when kinematic calculations fail (e.g., position unreachable)."""

    def __init__(self, message: str) -> None:
        super().__init__(message)
