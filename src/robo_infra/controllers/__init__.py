"""High-level controller implementations."""

from robo_infra.controllers.differential import (
    DifferentialDrive,
    DifferentialDriveConfig,
    DifferentialDriveState,
)
from robo_infra.controllers.gripper import (
    Gripper,
    GripperConfig,
    GripperState,
)
from robo_infra.controllers.joint_group import (
    JointGroup,
    JointGroupConfig,
    JointGroupState,
)
from robo_infra.controllers.lock import (
    Lock,
    LockConfig,
    LockState,
)
from robo_infra.controllers.mavlink import (
    AutopilotType,
    FlightModeArduCopter,
    FlightModePX4,
    GPSFixType,
    MAVLinkAttitude,
    MAVLinkBattery,
    MAVLinkConfig,
    MAVLinkController,
    MAVLinkGPS,
    MAVLinkHeartbeat,
    MAVLinkState,
    MAVLinkStatus,
    MAVType,
    create_mavlink_controller,
)
from robo_infra.controllers.quadcopter import (
    Attitude,
    FlightMode,
    FrameType,
    MotorOutputs,
    MotorPosition,
    Position3D,
    Quadcopter,
    QuadcopterConfig,
    QuadcopterState,
    QuadcopterStatus,
    Velocity,
    create_quadcopter,
    mix_motors,
    normalize_motor_outputs,
)


__all__ = [
    # Differential Drive
    "DifferentialDrive",
    "DifferentialDriveConfig",
    "DifferentialDriveState",
    # Gripper
    "Gripper",
    "GripperConfig",
    "GripperState",
    # Joint Group
    "JointGroup",
    "JointGroupConfig",
    "JointGroupState",
    # Lock
    "Lock",
    "LockConfig",
    "LockState",
    # MAVLink Controller
    "AutopilotType",
    "FlightModeArduCopter",
    "FlightModePX4",
    "GPSFixType",
    "MAVLinkAttitude",
    "MAVLinkBattery",
    "MAVLinkConfig",
    "MAVLinkController",
    "MAVLinkGPS",
    "MAVLinkHeartbeat",
    "MAVLinkState",
    "MAVLinkStatus",
    "MAVType",
    "create_mavlink_controller",
    # Quadcopter Controller
    "Attitude",
    "FlightMode",
    "FrameType",
    "MotorOutputs",
    "MotorPosition",
    "Position3D",
    "Quadcopter",
    "QuadcopterConfig",
    "QuadcopterState",
    "QuadcopterStatus",
    "Velocity",
    "create_quadcopter",
    "mix_motors",
    "normalize_motor_outputs",
]
