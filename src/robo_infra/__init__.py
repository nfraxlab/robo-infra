"""
robo-infra: Universal robotics infrastructure package.

Control any robot from servo to rocket with a simple, unified API.

Modules:
    actuators: Servo, DC motor, stepper, linear actuator implementations
    controllers: High-level robot controllers (differential drive, gripper, etc.)
    core: Base abstractions (Actuator, Controller, Sensor, Driver, Bus)
    drivers: Hardware driver implementations (PCA9685, L298N, GPIO, etc.)
    integrations: AI and API integration bridges (ai-infra, svc-infra, ROS2)
    motion: Kinematics, trajectory generation, PID control
    platforms: Platform-specific implementations (Raspberry Pi, Jetson, etc.)
    power: Battery monitoring and power management
    protocols: Industrial protocols (CANopen, Modbus)
    safety: Emergency stop, watchdog, safety monitoring
    sensors: Sensor implementations (camera, IMU, GPS, LIDAR, etc.)
    vision: Computer vision utilities (image processing, markers, color)

Example:
    >>> from robo_infra import Servo, DifferentialDrive, EStop
    >>> from robo_infra.actuators import create_servo
    >>> from robo_infra.controllers import Gripper
    >>> from robo_infra.safety import Watchdog
"""

from importlib.metadata import version

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


__version__ = version("robo_infra")

# Public API exports - organized by category, alphabetically sorted within each
__all__ = [
    # Core Types (robo_infra.core.types)
    "Angle",
    # CAN Bus (robo_infra.core.can_bus)
    "CANBitrate",
    "CANBus",
    "CANConfig",
    "CANInterface",
    "CANMessage",
    # Protocols (robo_infra.protocols)
    "CANOpenMaster",
    "CANOpenNode",
    "CANState",
    # Exceptions (robo_infra.core.exceptions)
    "CalibrationError",
    "CommunicationError",
    # Controllers (robo_infra.controllers)
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
