"""Motion planning and kinematics.

This module provides motion control primitives including:
- PID controller with anti-windup and derivative filtering
- Trajectory generation with multiple profile types
- Kinematics solvers for robot arms and mechanisms
"""

from robo_infra.motion.kinematics import (
    ElbowConfiguration,
    EndEffectorPose,
    JointAngle,
    JointLimitError,
    JointLimits,
    KinematicChain,
    KinematicsError,
    ThreeLinkArm,
    TwoLinkArm,
    UnreachablePositionError,
)
from robo_infra.motion.pid import PID, PIDConfig
from robo_infra.motion.trajectory import (
    LinearInterpolator,
    MultiAxisTrajectoryPoint,
    Trajectory,
    TrajectoryPoint,
    TrapezoidalProfile,
)


__all__ = [
    "PID",
    "ElbowConfiguration",
    "EndEffectorPose",
    "JointAngle",
    "JointLimitError",
    "JointLimits",
    "KinematicChain",
    "KinematicsError",
    "LinearInterpolator",
    "MultiAxisTrajectoryPoint",
    "PIDConfig",
    "ThreeLinkArm",
    "Trajectory",
    "TrajectoryPoint",
    "TrapezoidalProfile",
    "TwoLinkArm",
    "UnreachablePositionError",
]
