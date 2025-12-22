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
from robo_infra.core.can_bus import (
    CANBitrate,
    CANBus,
    CANConfig,
    CANInterface,
    CANMessage,
    CANState,
    SimulatedCANBus,
    get_can,
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
from robo_infra.protocols import (
    CANOpenMaster,
    CANOpenNode,
    ModbusRTU,
    ModbusTCP,
    NMTCommand,
    NMTState,
)


__version__ = "0.1.0"
__all__ = [
    # Types
    "Angle",
    # CAN Bus
    "CANBitrate",
    "CANBus",
    "CANConfig",
    "CANInterface",
    "CANMessage",
    # Protocols
    "CANOpenMaster",
    "CANOpenNode",
    "CANState",
    # Exceptions
    "CalibrationError",
    "CommunicationError",
    # Controllers
    "DifferentialDrive",
    "DifferentialDriveConfig",
    "DifferentialDriveState",
    "Direction",
    "Gripper",
    "GripperConfig",
    "GripperState",
    "HardwareNotFoundError",
    "JointGroup",
    "JointGroupConfig",
    "JointGroupState",
    "Limits",
    "LimitsExceededError",
    "Lock",
    "LockConfig",
    "LockState",
    "ModbusRTU",
    "ModbusTCP",
    "NMTCommand",
    "NMTState",
    "Position",
    "Range",
    "RoboInfraError",
    "SafetyError",
    "SimulatedCANBus",
    "Speed",
    # Version
    "__version__",
    "get_can",
]
