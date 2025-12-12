"""Core abstractions for robo-infra."""

from robo_infra.core.exceptions import (
    CalibrationError,
    CommunicationError,
    HardwareNotFoundError,
    LimitsExceededError,
    RoboInfraError,
    SafetyError,
)
from robo_infra.core.types import Angle, Direction, Limits, Position, Range, Speed


__all__ = [
    "Angle",
    "CalibrationError",
    "CommunicationError",
    "Direction",
    "HardwareNotFoundError",
    "Limits",
    "LimitsExceededError",
    "Position",
    "Range",
    "RoboInfraError",
    "SafetyError",
    "Speed",
]
