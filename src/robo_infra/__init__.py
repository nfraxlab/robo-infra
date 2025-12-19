"""
robo-infra: Universal robotics infrastructure package.

Control any robot from servo to rocket with a simple, unified API.
"""

from robo_infra.controllers import (
    DifferentialDrive,
    DifferentialDriveConfig,
    DifferentialDriveState,
    Gripper,
    GripperConfig,
    GripperState,
    JointGroup,
    JointGroupConfig,
    JointGroupState,
    Lock,
    LockConfig,
    LockState,
)
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
    # Types
    "Angle",
    "Direction",
    "Limits",
    "Position",
    "Range",
    "Speed",
    # Exceptions
    "CalibrationError",
    "CommunicationError",
    "HardwareNotFoundError",
    "LimitsExceededError",
    "RoboInfraError",
    "SafetyError",
    # Controllers
    "DifferentialDrive",
    "DifferentialDriveConfig",
    "DifferentialDriveState",
    "Gripper",
    "GripperConfig",
    "GripperState",
    "JointGroup",
    "JointGroupConfig",
    "JointGroupState",
    "Lock",
    "LockConfig",
    "LockState",
    # Version
    "__version__",
]
