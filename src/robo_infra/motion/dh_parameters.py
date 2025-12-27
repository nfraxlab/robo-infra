"""Denavit-Hartenberg parameters for serial robot arms.

This module provides classes for defining robot arm kinematics using
Denavit-Hartenberg (DH) parameters, the standard method for describing
serial link manipulators.

Key classes:
- DHParameter: Single link DH parameter set
- DHChain: Complete kinematic chain
- DHConvention: Standard vs Modified DH convention

Example:
    >>> from robo_infra.motion.dh_parameters import DHParameter, DHChain
    >>>
    >>> # Define a 3-DOF arm using DH parameters
    >>> params = [
    ...     DHParameter(d=0, theta=0, a=100, alpha=0),   # Link 1
    ...     DHParameter(d=0, theta=0, a=100, alpha=0),   # Link 2
    ...     DHParameter(d=0, theta=0, a=50, alpha=0),    # Link 3
    ... ]
    >>> chain = DHChain(params)
    >>>
    >>> # Forward kinematics
    >>> pose = chain.forward([0, 0, 0])  # All joints at 0
    >>> print(f"End effector: {pose.position}")
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

from robo_infra.motion.transforms import Transform


if TYPE_CHECKING:
    from collections.abc import Sequence

    from numpy.typing import NDArray


class DHConvention(Enum):
    """Denavit-Hartenberg parameter convention.

    Two conventions exist:
    - STANDARD: Classic DH convention (Craig's book, many textbooks)
    - MODIFIED: Modified DH convention (used by some manufacturers)

    The difference is in how transformations are applied.
    """

    STANDARD = "standard"
    MODIFIED = "modified"


class JointType(Enum):
    """Type of joint in the kinematic chain.

    - REVOLUTE: Rotates around an axis (theta is variable)
    - PRISMATIC: Slides along an axis (d is variable)
    """

    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"


@dataclass(slots=True)
class JointLimit:
    """Joint position limits.

    Attributes:
        min_val: Minimum joint value (radians for revolute, meters for prismatic).
        max_val: Maximum joint value.
    """

    min_val: float = field(default=-math.pi)
    max_val: float = field(default=math.pi)

    def __post_init__(self) -> None:
        if self.min_val > self.max_val:
            raise ValueError(f"min_val ({self.min_val}) > max_val ({self.max_val})")

    def clamp(self, value: float) -> float:
        """Clamp value to limits."""
        return max(self.min_val, min(self.max_val, value))

    def is_within(self, value: float, tolerance: float = 1e-6) -> bool:
        """Check if value is within limits."""
        return self.min_val - tolerance <= value <= self.max_val + tolerance


@dataclass(slots=True)
class DHParameter:
    """Denavit-Hartenberg parameters for a single link.

    Standard DH parameters describe the transformation from frame i-1 to frame i:

    T = Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)

    Where:
    - d: Link offset along Z axis (distance from X_{i-1} to X_i along Z_{i-1})
    - theta: Joint angle around Z axis (angle from X_{i-1} to X_i around Z_{i-1})
    - a: Link length along X axis (distance from Z_{i-1} to Z_i along X_i)
    - alpha: Link twist around X axis (angle from Z_{i-1} to Z_i around X_i)

    For revolute joints, theta is the joint variable.
    For prismatic joints, d is the joint variable.

    Attributes:
        d: Link offset (meters).
        theta: Joint angle offset (radians). For revolute joints, this is added to joint variable.
        a: Link length (meters).
        alpha: Link twist (radians).
        joint_type: Type of joint (revolute or prismatic).
        limit: Joint position limits.
        name: Optional joint name.
    """

    d: float = 0.0
    theta: float = 0.0
    a: float = 0.0
    alpha: float = 0.0
    joint_type: JointType = field(default=JointType.REVOLUTE)
    limit: JointLimit = field(default_factory=JointLimit)
    name: str = ""

    def transform(
        self,
        joint_value: float,
        convention: DHConvention = DHConvention.STANDARD,
    ) -> Transform:
        """Compute transformation matrix for this link.

        Args:
            joint_value: Joint variable (radians for revolute, meters for prismatic).
            convention: DH convention to use.

        Returns:
            4x4 homogeneous transformation matrix as Transform.
        """
        # Apply joint variable
        if self.joint_type == JointType.REVOLUTE:
            theta = self.theta + joint_value
            d = self.d
        else:  # PRISMATIC
            theta = self.theta
            d = self.d + joint_value

        a = self.a
        alpha = self.alpha

        # Precompute sin/cos
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        if convention == DHConvention.STANDARD:
            # Standard DH: T = Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)
            matrix = np.array(
                [
                    [ct, -st * ca, st * sa, a * ct],
                    [st, ct * ca, -ct * sa, a * st],
                    [0, sa, ca, d],
                    [0, 0, 0, 1],
                ],
                dtype=np.float64,
            )
        else:
            # Modified DH: T = Rx(alpha_{i-1}) @ Tx(a_{i-1}) @ Rz(theta) @ Tz(d)
            matrix = np.array(
                [
                    [ct, -st, 0, a],
                    [st * ca, ct * ca, -sa, -sa * d],
                    [st * sa, ct * sa, ca, ca * d],
                    [0, 0, 0, 1],
                ],
                dtype=np.float64,
            )

        return Transform.from_matrix(matrix)


@dataclass
class DHChain:
    """Kinematic chain defined by DH parameters.

    Represents a serial link manipulator as a sequence of DH parameters.
    Provides forward kinematics and Jacobian computation.

    Attributes:
        parameters: List of DH parameters for each link.
        convention: DH convention to use.
        base_transform: Transform from world to base frame.
        tool_transform: Transform from last link to tool frame.

    Example:
        >>> # 2-DOF planar arm
        >>> params = [
        ...     DHParameter(a=100, alpha=0),  # Link 1
        ...     DHParameter(a=100, alpha=0),  # Link 2
        ... ]
        >>> chain = DHChain(params)
        >>> pose = chain.forward([0.5, -0.3])  # Joint angles in radians
    """

    parameters: list[DHParameter]
    convention: DHConvention = DHConvention.STANDARD
    base_transform: Transform = field(default_factory=Transform.identity)
    tool_transform: Transform = field(default_factory=Transform.identity)

    def __post_init__(self) -> None:
        """Validate chain."""
        if not self.parameters:
            raise ValueError("DHChain requires at least one parameter")

    @property
    def num_joints(self) -> int:
        """Number of joints in the chain."""
        return len(self.parameters)

    @property
    def joint_names(self) -> list[str]:
        """List of joint names."""
        return [p.name or f"joint_{i}" for i, p in enumerate(self.parameters)]

    @property
    def joint_types(self) -> list[JointType]:
        """List of joint types."""
        return [p.joint_type for p in self.parameters]

    @property
    def joint_limits(self) -> list[JointLimit]:
        """List of joint limits."""
        return [p.limit for p in self.parameters]

    def forward(self, joint_values: Sequence[float]) -> Transform:
        """Compute forward kinematics.

        Args:
            joint_values: Joint values for each joint.

        Returns:
            Transform from base to end effector.

        Raises:
            ValueError: If wrong number of joint values provided.
        """
        if len(joint_values) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint values, got {len(joint_values)}")

        # Start with base transform
        T = self.base_transform

        # Chain all link transforms
        for param, q in zip(self.parameters, joint_values, strict=False):
            T = T @ param.transform(q, self.convention)

        # Apply tool transform
        T = T @ self.tool_transform

        return T

    def forward_all(self, joint_values: Sequence[float]) -> list[Transform]:
        """Compute forward kinematics for all frames.

        Args:
            joint_values: Joint values for each joint.

        Returns:
            List of transforms from base to each frame (including end effector).
        """
        if len(joint_values) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint values, got {len(joint_values)}")

        transforms: list[Transform] = []
        T = self.base_transform

        for param, q in zip(self.parameters, joint_values, strict=False):
            T = T @ param.transform(q, self.convention)
            transforms.append(T)

        # Add tool frame
        transforms.append(T @ self.tool_transform)

        return transforms

    def jacobian(
        self,
        joint_values: Sequence[float],
        delta: float = 1e-6,
    ) -> NDArray[np.float64]:
        """Compute the geometric Jacobian matrix.

        The Jacobian relates joint velocities to end effector velocities:
        [v_x, v_y, v_z, w_x, w_y, w_z]^T = J @ [q_dot]^T

        Uses numerical differentiation for generality.

        Args:
            joint_values: Current joint values.
            delta: Perturbation for numerical differentiation.

        Returns:
            6 x n Jacobian matrix where n is number of joints.
        """
        n = self.num_joints
        J = np.zeros((6, n), dtype=np.float64)

        # Current pose
        T0 = self.forward(joint_values)
        p0 = T0.position
        q0 = T0.rotation.as_quaternion(scalar_first=True)

        for i in range(n):
            # Perturb joint i
            q_plus = list(joint_values)
            q_plus[i] += delta

            T_plus = self.forward(q_plus)
            p_plus = T_plus.position
            q_plus_rot = T_plus.rotation.as_quaternion(scalar_first=True)

            # Linear velocity columns (position derivative)
            J[0:3, i] = (p_plus - p0) / delta

            # Angular velocity columns (from quaternion derivative)
            # dq/dt ≈ 0.5 * [w] * q, so w ≈ 2 * dq/dt * q_conj
            dq = (q_plus_rot - q0) / delta

            # Convert quaternion derivative to angular velocity
            # w = 2 * (q_conj @ dq) where we take vector part
            w0, wx0, wy0, wz0 = q0
            dw, dx, dy, dz = dq

            # Angular velocity from quaternion kinematics
            J[3, i] = 2 * (-wx0 * dw + w0 * dx - wz0 * dy + wy0 * dz)
            J[4, i] = 2 * (-wy0 * dw + wz0 * dx + w0 * dy - wx0 * dz)
            J[5, i] = 2 * (-wz0 * dw - wy0 * dx + wx0 * dy + w0 * dz)

        return J

    def jacobian_analytical(
        self,
        joint_values: Sequence[float],
    ) -> NDArray[np.float64]:
        """Compute the analytical Jacobian using geometric approach.

        For revolute joints: J_i = [z_{i-1} x (p_n - p_{i-1}); z_{i-1}]
        For prismatic joints: J_i = [z_{i-1}; 0]

        Args:
            joint_values: Current joint values.

        Returns:
            6 x n Jacobian matrix.
        """
        n = self.num_joints
        J = np.zeros((6, n), dtype=np.float64)

        # Get all frame transforms
        transforms = self.forward_all(joint_values)

        # End effector position
        p_n = transforms[-1].position

        # Base frame z-axis (for first joint)
        T_prev = self.base_transform

        for i in range(n):
            # Z-axis of previous frame
            z_i = T_prev.rotation.apply(np.array([0, 0, 1]))
            p_i = T_prev.position

            if self.parameters[i].joint_type == JointType.REVOLUTE:
                # Linear velocity: z x (p_n - p_i)
                J[0:3, i] = np.cross(z_i, p_n - p_i)
                # Angular velocity: z
                J[3:6, i] = z_i
            else:  # PRISMATIC
                # Linear velocity: z
                J[0:3, i] = z_i
                # Angular velocity: 0
                J[3:6, i] = 0

            # Update to current frame
            T_prev = transforms[i]

        return J

    def manipulability(self, joint_values: Sequence[float]) -> float:
        """Compute Yoshikawa's manipulability measure.

        w = sqrt(det(J @ J^T))

        Higher values indicate better manipulability (further from singularity).

        Args:
            joint_values: Current joint values.

        Returns:
            Manipulability measure (non-negative scalar).
        """
        J = self.jacobian(joint_values)
        JJT = J @ J.T
        det = np.linalg.det(JJT)
        return math.sqrt(max(0.0, det))

    def is_singular(
        self,
        joint_values: Sequence[float],
        threshold: float = 1e-4,
    ) -> bool:
        """Check if configuration is near a singularity.

        Args:
            joint_values: Current joint values.
            threshold: Manipulability threshold for singularity.

        Returns:
            True if near singularity.
        """
        return self.manipulability(joint_values) < threshold

    def check_limits(self, joint_values: Sequence[float]) -> list[bool]:
        """Check which joints are within limits.

        Args:
            joint_values: Joint values to check.

        Returns:
            List of booleans indicating if each joint is within limits.
        """
        return [
            limit.is_within(q) for limit, q in zip(self.joint_limits, joint_values, strict=False)
        ]

    def clamp_to_limits(self, joint_values: Sequence[float]) -> list[float]:
        """Clamp joint values to their limits.

        Args:
            joint_values: Joint values to clamp.

        Returns:
            Clamped joint values.
        """
        return [limit.clamp(q) for limit, q in zip(self.joint_limits, joint_values, strict=False)]


# =============================================================================
# Common Robot Arm Definitions
# =============================================================================


def create_puma_560() -> DHChain:
    """Create DH chain for PUMA 560 robot arm.

    The PUMA 560 is a classic 6-DOF industrial robot arm.
    Uses standard DH parameters from Craig's textbook.

    Returns:
        DHChain for PUMA 560.
    """
    # PUMA 560 DH parameters (in meters and radians)
    # Reference: Craig, Introduction to Robotics
    params = [
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=-math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2.79, 2.79),
            name="waist",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.4318,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-3.93, 0.79),
            name="shoulder",
        ),
        DHParameter(
            d=0.15005,
            theta=0.0,
            a=0.0203,
            alpha=-math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-0.79, 3.93),
            name="elbow",
        ),
        DHParameter(
            d=0.4318,
            theta=0.0,
            a=0.0,
            alpha=math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-5.24, 5.24),
            name="wrist_1",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=-math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2.01, 2.01),
            name="wrist_2",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-5.24, 5.24),
            name="wrist_3",
        ),
    ]
    return DHChain(params, convention=DHConvention.STANDARD)


def create_ur5() -> DHChain:
    """Create DH chain for Universal Robots UR5.

    The UR5 is a popular collaborative robot (cobot) with 6-DOF.
    Uses modified DH parameters.

    Returns:
        DHChain for UR5.
    """
    # UR5 DH parameters (in meters)
    # Reference: UR5 Technical Specifications
    params = [
        DHParameter(
            d=0.089159,
            theta=0.0,
            a=0.0,
            alpha=math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2 * math.pi, 2 * math.pi),
            name="shoulder_pan",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=-0.42500,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2 * math.pi, 2 * math.pi),
            name="shoulder_lift",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=-0.39225,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2 * math.pi, 2 * math.pi),
            name="elbow",
        ),
        DHParameter(
            d=0.10915,
            theta=0.0,
            a=0.0,
            alpha=math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2 * math.pi, 2 * math.pi),
            name="wrist_1",
        ),
        DHParameter(
            d=0.09465,
            theta=0.0,
            a=0.0,
            alpha=-math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2 * math.pi, 2 * math.pi),
            name="wrist_2",
        ),
        DHParameter(
            d=0.0823,
            theta=0.0,
            a=0.0,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2 * math.pi, 2 * math.pi),
            name="wrist_3",
        ),
    ]
    return DHChain(params, convention=DHConvention.MODIFIED)


def create_planar_3dof(l1: float, l2: float, l3: float) -> DHChain:
    """Create DH chain for a 3-DOF planar arm.

    Three revolute joints, all rotating in the same plane.

    Args:
        l1: Length of first link.
        l2: Length of second link.
        l3: Length of third link.

    Returns:
        DHChain for 3-DOF planar arm.
    """
    params = [
        DHParameter(
            d=0.0,
            theta=0.0,
            a=l1,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-math.pi, math.pi),
            name="joint_1",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=l2,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-math.pi, math.pi),
            name="joint_2",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=l3,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-math.pi, math.pi),
            name="joint_3",
        ),
    ]
    return DHChain(params)


def create_stanford_arm() -> DHChain:
    """Create DH chain for Stanford arm (RRP manipulator).

    Classic arm with 2 revolute + 1 prismatic + 3 revolute joints.
    The prismatic joint is useful for studying different joint types.

    Returns:
        DHChain for Stanford arm.
    """
    params = [
        DHParameter(
            d=0.4120,
            theta=0.0,
            a=0.0,
            alpha=-math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2.79, 2.79),
            name="waist",
        ),
        DHParameter(
            d=0.1540,
            theta=0.0,
            a=0.0,
            alpha=math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2.79, 2.79),
            name="shoulder",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=0.0,
            joint_type=JointType.PRISMATIC,  # Prismatic joint!
            limit=JointLimit(0.0, 1.0),
            name="extension",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=-math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2.79, 2.79),
            name="wrist_1",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=math.pi / 2,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-1.57, 1.57),
            name="wrist_2",
        ),
        DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
            limit=JointLimit(-2.79, 2.79),
            name="wrist_3",
        ),
    ]
    return DHChain(params)
