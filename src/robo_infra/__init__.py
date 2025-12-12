"""
robo-infra: Universal robotics infrastructure package.

Control any robot from servo to rocket with a simple, unified API.
"""

from robo_infra.core.exceptions import (
    CalibrationError,
    CommunicationError,
    HardwareNotFoundError,
    LimitsExceededError,
    RoboInfraError,
    SafetyError,
)
from robo_infra.core.types import Angle, Direction, Limits, Position, Range, Speed


__version__ = "0.1.0"
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
    "__version__",
]
