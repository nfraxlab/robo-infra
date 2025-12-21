"""Motion planning and kinematics.

This module provides motion control primitives including:
- PID controller with anti-windup and derivative filtering
- Trajectory generation with multiple profile types
- Kinematics solvers for robot arms and mechanisms
- 3D transforms and rotations
- DH parameters for serial arm kinematics
- Inverse kinematics solvers
- Specialized arm types: SCARA, Delta, Stewart platform
- Path planning with obstacle avoidance (RRT, linear, Cartesian)
- Advanced trajectory profiles (S-curve, splines, blending)
"""

from robo_infra.motion.delta import (
    DeltaJoints,
    DeltaLimits,
    DeltaRobot,
    DeltaSingularityType,
    DeltaWorkspace,
    create_delta,
    create_flsun_q5,
    create_kossel_mini,
)
from robo_infra.motion.dh_parameters import (
    DHChain,
    DHConvention,
    DHParameter,
    JointLimit,
    JointType,
    create_planar_3dof,
    create_puma_560,
    create_stanford_arm,
    create_ur5,
)
from robo_infra.motion.ik_solvers import (
    BaseIKSolver,
    CCDIKSolver,
    DampedLeastSquaresIK,
    GradientDescentIK,
    IKConvergenceError,
    IKError,
    IKResult,
    IKResultStatus,
    IKSingularityError,
    IKSolver,
    IKSolverConfig,
    IKUnreachableError,
    JacobianIKSolver,
    create_ik_solver,
    solve_ik,
)
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
from robo_infra.motion.path_planning import (
    BoxObstacle,
    CartesianPathPlanner,
    LinearPathPlanner,
    Obstacle,
    Path,
    PathPlanner,
    PathPoint,
    PathSmoother,
    PathStatus,
    RRTPathPlanner,
    SmoothingMethod,
    SphereObstacle,
)
from robo_infra.motion.pid import PID, PIDConfig
from robo_infra.motion.scara import (
    SCARAArm,
    SCARAConfiguration,
    SCARAJoints,
    SCARALimits,
    create_epson_ls3,
    create_epson_ls6,
    create_scara,
)
from robo_infra.motion.stewart import (
    StewartJoints,
    StewartLimits,
    StewartPlatform,
    StewartPose,
    StewartSingularityType,
    create_flight_simulator,
    create_precision_positioner,
    create_stewart,
)
from robo_infra.motion.trajectory import (
    BlendedTrajectory,
    BlendSegment,
    CubicTrajectory,
    LinearInterpolator,
    MultiAxisTrajectoryPoint,
    QuinticTrajectory,
    SCurveTrajectory,
    SplineTrajectory,
    Trajectory,
    TrajectoryConstraints,
    TrajectoryGenerator,
    TrajectoryPoint,
    TrajectoryProfile,
    TrapezoidalProfile,
)
from robo_infra.motion.transforms import (
    EulerOrder,
    Rotation,
    Transform,
    rotation_x,
    rotation_y,
    rotation_z,
    translation,
)


__all__ = [
    # Kinematics (basic)
    "ElbowConfiguration",
    "EndEffectorPose",
    "JointAngle",
    "JointLimitError",
    "JointLimits",
    "KinematicChain",
    "KinematicsError",
    "ThreeLinkArm",
    "TwoLinkArm",
    "UnreachablePositionError",
    # PID
    "PID",
    "PIDConfig",
    # Trajectory (basic)
    "LinearInterpolator",
    "MultiAxisTrajectoryPoint",
    "Trajectory",
    "TrajectoryPoint",
    "TrapezoidalProfile",
    # Trajectory (advanced)
    "BlendedTrajectory",
    "BlendSegment",
    "CubicTrajectory",
    "QuinticTrajectory",
    "SCurveTrajectory",
    "SplineTrajectory",
    "TrajectoryConstraints",
    "TrajectoryGenerator",
    "TrajectoryProfile",
    # Path Planning
    "BoxObstacle",
    "CartesianPathPlanner",
    "LinearPathPlanner",
    "Obstacle",
    "Path",
    "PathPlanner",
    "PathPoint",
    "PathSmoother",
    "PathStatus",
    "RRTPathPlanner",
    "SmoothingMethod",
    "SphereObstacle",
    # Transforms
    "EulerOrder",
    "Rotation",
    "Transform",
    "rotation_x",
    "rotation_y",
    "rotation_z",
    "translation",
    # DH Parameters
    "DHChain",
    "DHConvention",
    "DHParameter",
    "JointLimit",
    "JointType",
    "create_planar_3dof",
    "create_puma_560",
    "create_stanford_arm",
    "create_ur5",
    # IK Solvers
    "BaseIKSolver",
    "CCDIKSolver",
    "DampedLeastSquaresIK",
    "GradientDescentIK",
    "IKConvergenceError",
    "IKError",
    "IKResult",
    "IKResultStatus",
    "IKSingularityError",
    "IKSolver",
    "IKSolverConfig",
    "IKUnreachableError",
    "JacobianIKSolver",
    "create_ik_solver",
    "solve_ik",
    # SCARA Arm
    "SCARAArm",
    "SCARAConfiguration",
    "SCARAJoints",
    "SCARALimits",
    "create_epson_ls3",
    "create_epson_ls6",
    "create_scara",
    # Delta Robot
    "DeltaJoints",
    "DeltaLimits",
    "DeltaRobot",
    "DeltaSingularityType",
    "DeltaWorkspace",
    "create_delta",
    "create_flsun_q5",
    "create_kossel_mini",
    # Stewart Platform
    "StewartJoints",
    "StewartLimits",
    "StewartPlatform",
    "StewartPose",
    "StewartSingularityType",
    "create_flight_simulator",
    "create_precision_positioner",
    "create_stewart",
]
