"""SCARA (Selective Compliance Articulated Robot Arm) kinematics.

SCARA robots are 4-DOF manipulators commonly used in pick-and-place and
assembly applications. They have high speed and repeatability in the
horizontal plane, with compliance in vertical direction.

Architecture:
- Joint 1: Rotary (shoulder, around Z)
- Joint 2: Rotary (elbow, around Z)
- Joint 3: Prismatic (vertical, along Z)
- Joint 4: Rotary (wrist, around Z)

Key Features:
- All rotary axes are parallel (vertical)
- High rigidity in horizontal plane
- Compliance in vertical direction
- Closed-form analytical IK solutions

Example:
    >>> from robo_infra.motion.scara import SCARAArm
    >>>
    >>> # Create a SCARA with 200mm and 150mm arm lengths
    >>> scara = SCARAArm(l1=0.2, l2=0.15, z_range=(0.0, 0.1))
    >>>
    >>> # Forward kinematics
    >>> pose = scara.forward(theta1=0.0, theta2=0.0, z=0.05, theta4=0.0)
    >>> print(f"End effector at: {pose.position}")
    >>>
    >>> # Inverse kinematics
    >>> joints = scara.inverse(pose)
    >>> print(f"Joint angles: {joints}")
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

from robo_infra.motion.transforms import Rotation, Transform


if TYPE_CHECKING:
    from numpy.typing import NDArray


class SCARAConfiguration(Enum):
    """SCARA arm configuration (elbow orientation).

    SCARA arms have two possible configurations for any reachable point
    in the horizontal plane (similar to 2-link planar arms).
    """

    LEFT_ARM = "left_arm"  # Elbow to the left (positive theta2)
    RIGHT_ARM = "right_arm"  # Elbow to the right (negative theta2)


@dataclass
class SCARALimits:
    """Joint limits for SCARA arm.

    Attributes:
        theta1_range: Shoulder joint limits in radians.
        theta2_range: Elbow joint limits in radians.
        z_range: Vertical (prismatic) joint limits in meters.
        theta4_range: Wrist joint limits in radians.
    """

    theta1_range: tuple[float, float] = (-math.pi, math.pi)
    theta2_range: tuple[float, float] = (-2.356, 2.356)  # ±135°
    z_range: tuple[float, float] = (0.0, 0.1)
    theta4_range: tuple[float, float] = (-math.pi, math.pi)

    def is_valid(
        self,
        theta1: float,
        theta2: float,
        z: float,
        theta4: float,
    ) -> bool:
        """Check if joint values are within limits."""
        return (
            self.theta1_range[0] <= theta1 <= self.theta1_range[1]
            and self.theta2_range[0] <= theta2 <= self.theta2_range[1]
            and self.z_range[0] <= z <= self.z_range[1]
            and self.theta4_range[0] <= theta4 <= self.theta4_range[1]
        )


@dataclass
class SCARAJoints:
    """Joint values for a SCARA arm.

    Attributes:
        theta1: Shoulder angle in radians.
        theta2: Elbow angle in radians.
        z: Vertical position in meters.
        theta4: Wrist angle in radians.
        configuration: Arm configuration (left/right).
    """

    theta1: float
    theta2: float
    z: float
    theta4: float
    configuration: SCARAConfiguration = SCARAConfiguration.LEFT_ARM

    def as_array(self) -> NDArray[np.float64]:
        """Return joints as numpy array [theta1, theta2, z, theta4]."""
        return np.array([self.theta1, self.theta2, self.z, self.theta4])

    def as_tuple(self) -> tuple[float, float, float, float]:
        """Return joints as tuple."""
        return (self.theta1, self.theta2, self.z, self.theta4)


@dataclass
class SCARAArm:
    """SCARA (Selective Compliance Articulated Robot Arm) 4-DOF robot.

    Common in pick-and-place applications. The SCARA has:
    - 2 rotary joints in horizontal plane (shoulder and elbow)
    - 1 vertical prismatic joint (Z)
    - 1 rotary joint for tool rotation (wrist)

    All rotary joints rotate around vertical (Z) axis, giving the SCARA
    high rigidity in the horizontal plane and compliance in vertical direction.

    Attributes:
        l1: Length of first arm segment (shoulder to elbow) in meters.
        l2: Length of second arm segment (elbow to wrist) in meters.
        z_offset: Base Z offset (height of shoulder joint) in meters.
        limits: Joint limits.

    Example:
        >>> scara = SCARAArm(l1=0.25, l2=0.2, z_range=(0, 0.15))
        >>> pose = scara.forward(0, math.pi/2, 0.1, 0)
        >>> print(f"X: {pose.position[0]:.3f}, Y: {pose.position[1]:.3f}")
        X: 0.250, Y: 0.200
    """

    l1: float  # First arm length
    l2: float  # Second arm length
    z_offset: float = 0.0  # Base height offset
    limits: SCARALimits = field(default_factory=SCARALimits)

    def __post_init__(self) -> None:
        """Validate dimensions."""
        if self.l1 <= 0:
            raise ValueError(f"l1 must be positive, got {self.l1}")
        if self.l2 <= 0:
            raise ValueError(f"l2 must be positive, got {self.l2}")
        # Update z_range in limits if not default
        if hasattr(self, "_z_range_override"):
            self.limits = SCARALimits(z_range=self._z_range_override)

    @classmethod
    def create(
        cls,
        l1: float,
        l2: float,
        z_range: tuple[float, float],
        z_offset: float = 0.0,
    ) -> SCARAArm:
        """Create SCARA arm with specified dimensions.

        Args:
            l1: First arm length in meters.
            l2: Second arm length in meters.
            z_range: Tuple of (min_z, max_z) for prismatic joint.
            z_offset: Height of shoulder joint from base.

        Returns:
            Configured SCARAArm instance.
        """
        limits = SCARALimits(z_range=z_range)
        return cls(l1=l1, l2=l2, z_offset=z_offset, limits=limits)

    @property
    def workspace_radius_max(self) -> float:
        """Maximum reach radius in horizontal plane."""
        return self.l1 + self.l2

    @property
    def workspace_radius_min(self) -> float:
        """Minimum reach radius in horizontal plane."""
        return abs(self.l1 - self.l2)

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """Check if a point is within the workspace.

        Args:
            x: X coordinate.
            y: Y coordinate.
            z: Z coordinate.

        Returns:
            True if point is reachable.
        """
        r = math.sqrt(x * x + y * y)
        z_valid = self.limits.z_range[0] <= z <= self.limits.z_range[1]
        r_valid = self.workspace_radius_min <= r <= self.workspace_radius_max
        return z_valid and r_valid

    def forward(
        self,
        theta1: float,
        theta2: float,
        z: float,
        theta4: float,
    ) -> Transform:
        """Compute forward kinematics.

        Args:
            theta1: Shoulder joint angle in radians.
            theta2: Elbow joint angle in radians.
            z: Vertical position in meters.
            theta4: Wrist joint angle in radians.

        Returns:
            Transform representing end-effector pose.

        Example:
            >>> scara = SCARAArm(l1=0.3, l2=0.2)
            >>> pose = scara.forward(0, 0, 0.05, 0)
            >>> print(f"Position: ({pose.position[0]:.2f}, {pose.position[1]:.2f}, {pose.position[2]:.2f})")
            Position: (0.50, 0.00, 0.05)
        """
        # Position calculation
        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)
        z_pos = z + self.z_offset

        # Orientation: rotation around Z axis
        # Total rotation is theta1 + theta2 + theta4
        total_rotation = theta1 + theta2 + theta4

        # Create rotation matrix (rotation around Z)
        c = math.cos(total_rotation)
        s = math.sin(total_rotation)
        rotation_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        rotation = Rotation(matrix=rotation_matrix)
        return Transform(position=np.array([x, y, z_pos]), rotation=rotation)

    def forward_joints(self, joints: SCARAJoints) -> Transform:
        """Compute forward kinematics from SCARAJoints object."""
        return self.forward(joints.theta1, joints.theta2, joints.z, joints.theta4)

    def inverse(
        self,
        target: Transform,
        configuration: SCARAConfiguration = SCARAConfiguration.LEFT_ARM,
    ) -> SCARAJoints | None:
        """Compute inverse kinematics using analytical solution.

        SCARA IK has closed-form solutions. For a given (x, y, z) position,
        there are up to 2 solutions (left-arm and right-arm configurations).

        Args:
            target: Desired end-effector pose.
            configuration: Desired arm configuration.

        Returns:
            SCARAJoints if solution exists, None otherwise.

        Example:
            >>> scara = SCARAArm(l1=0.3, l2=0.2, z_range=(0, 0.1))
            >>> # Create target pose
            >>> target = scara.forward(0.5, -0.5, 0.05, 0.1)
            >>> # Solve IK
            >>> joints = scara.inverse(target)
            >>> if joints:
            ...     print(f"Theta1: {math.degrees(joints.theta1):.1f}°")
        """
        x, y, z_pos = target.position
        z = z_pos - self.z_offset

        # Check Z is in range
        if not (self.limits.z_range[0] <= z <= self.limits.z_range[1]):
            return None

        # Check if point is reachable in XY plane
        r_sq = x * x + y * y
        r = math.sqrt(r_sq)

        if r > self.workspace_radius_max or r < self.workspace_radius_min:
            return None

        # Solve for theta2 using law of cosines
        # r² = l1² + l2² + 2*l1*l2*cos(theta2)
        cos_theta2 = (r_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)

        # Clamp for numerical stability
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        # Two solutions: elbow up (positive) or down (negative)
        if configuration == SCARAConfiguration.LEFT_ARM:
            theta2 = math.acos(cos_theta2)  # Positive
        else:
            theta2 = -math.acos(cos_theta2)  # Negative

        # Solve for theta1
        # Using atan2 for proper quadrant handling
        k1 = self.l1 + self.l2 * math.cos(theta2)
        k2 = self.l2 * math.sin(theta2)

        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        # Normalize theta1 to [-pi, pi]
        theta1 = math.atan2(math.sin(theta1), math.cos(theta1))

        # Extract desired wrist rotation from target
        # Get the Z rotation from the target transform
        target_rotation = target.rotation.as_euler()[2]  # Get Z rotation

        # theta4 = desired_rotation - theta1 - theta2
        theta4 = target_rotation - theta1 - theta2

        # Normalize theta4 to [-pi, pi]
        theta4 = math.atan2(math.sin(theta4), math.cos(theta4))

        # Check joint limits
        if not self.limits.is_valid(theta1, theta2, z, theta4):
            return None

        return SCARAJoints(
            theta1=theta1,
            theta2=theta2,
            z=z,
            theta4=theta4,
            configuration=configuration,
        )

    def inverse_position(
        self,
        x: float,
        y: float,
        z: float,
        wrist_angle: float = 0.0,
        configuration: SCARAConfiguration = SCARAConfiguration.LEFT_ARM,
    ) -> SCARAJoints | None:
        """Compute IK for a position with specified wrist angle.

        Simpler interface when you just want to reach a point.

        Args:
            x: X coordinate in meters.
            y: Y coordinate in meters.
            z: Z coordinate in meters.
            wrist_angle: Desired tool rotation in radians.
            configuration: Arm configuration.

        Returns:
            SCARAJoints if solution exists.
        """
        # Check Z range
        z_internal = z - self.z_offset
        if not (self.limits.z_range[0] <= z_internal <= self.limits.z_range[1]):
            return None

        # Check XY reachability
        r_sq = x * x + y * y
        r = math.sqrt(r_sq)

        if r > self.workspace_radius_max or r < self.workspace_radius_min:
            return None

        # Solve for theta2
        cos_theta2 = (r_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        if configuration == SCARAConfiguration.LEFT_ARM:
            theta2 = math.acos(cos_theta2)
        else:
            theta2 = -math.acos(cos_theta2)

        # Solve for theta1
        k1 = self.l1 + self.l2 * math.cos(theta2)
        k2 = self.l2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        theta1 = math.atan2(math.sin(theta1), math.cos(theta1))

        # Compute theta4 from desired wrist angle
        theta4 = wrist_angle - theta1 - theta2
        theta4 = math.atan2(math.sin(theta4), math.cos(theta4))

        # Check limits
        if not self.limits.is_valid(theta1, theta2, z_internal, theta4):
            return None

        return SCARAJoints(
            theta1=theta1,
            theta2=theta2,
            z=z_internal,
            theta4=theta4,
            configuration=configuration,
        )

    def jacobian(
        self,
        theta1: float,
        theta2: float,
        z: float,
        theta4: float,
    ) -> NDArray[np.float64]:
        """Compute the Jacobian matrix.

        The Jacobian relates joint velocities to end-effector velocities:
        [ẋ, ẏ, ż, ωz]ᵀ = J · [θ̇1, θ̇2, ż, θ̇4]ᵀ

        Args:
            theta1: Shoulder angle in radians.
            theta2: Elbow angle in radians.
            z: Vertical position (unused for Jacobian).
            theta4: Wrist angle (unused for position Jacobian).

        Returns:
            4x4 Jacobian matrix.
        """
        s1 = math.sin(theta1)
        c1 = math.cos(theta1)
        s12 = math.sin(theta1 + theta2)
        c12 = math.cos(theta1 + theta2)

        # Position Jacobian (3x4)
        # dx/dtheta1, dx/dtheta2, dx/dz, dx/dtheta4
        # dy/dtheta1, dy/dtheta2, dy/dz, dy/dtheta4
        # dz/dtheta1, dz/dtheta2, dz/dz, dz/dtheta4

        # Rotation Jacobian (1x4) - only omega_z matters for SCARA
        # domega_z/dtheta1, domega_z/dtheta2, domega_z/dz, domega_z/dtheta4

        jacobian = np.array(
            [
                [-self.l1 * s1 - self.l2 * s12, -self.l2 * s12, 0, 0],  # dx
                [self.l1 * c1 + self.l2 * c12, self.l2 * c12, 0, 0],  # dy
                [0, 0, 1, 0],  # dz
                [1, 1, 0, 1],  # domega_z
            ]
        )

        return jacobian

    def get_all_solutions(
        self,
        target: Transform,
    ) -> list[SCARAJoints]:
        """Get all valid IK solutions.

        SCARA can have up to 2 solutions (left-arm and right-arm).

        Args:
            target: Desired end-effector pose.

        Returns:
            List of valid joint configurations.
        """
        solutions = []

        for config in SCARAConfiguration:
            joints = self.inverse(target, configuration=config)
            if joints is not None:
                solutions.append(joints)

        return solutions


def create_scara(
    l1: float,
    l2: float,
    z_range: tuple[float, float] = (0.0, 0.1),
    z_offset: float = 0.0,
) -> SCARAArm:
    """Create a SCARA arm with specified dimensions.

    Factory function for easy SCARA creation.

    Args:
        l1: First arm length in meters.
        l2: Second arm length in meters.
        z_range: Z-axis travel range (min, max) in meters.
        z_offset: Height offset of shoulder joint.

    Returns:
        Configured SCARAArm instance.

    Example:
        >>> scara = create_scara(0.25, 0.2, z_range=(0, 0.15))
        >>> print(f"Max reach: {scara.workspace_radius_max:.2f}m")
        Max reach: 0.45m
    """
    return SCARAArm.create(l1=l1, l2=l2, z_range=z_range, z_offset=z_offset)


# Common SCARA configurations
def create_epson_ls3() -> SCARAArm:
    """Create an Epson LS3-B equivalent SCARA.

    Epson LS3-B specifications:
    - Arm lengths: 225mm + 175mm
    - Z stroke: 150mm
    - Payload: 3kg

    Returns:
        SCARAArm configured like Epson LS3-B.
    """
    return create_scara(l1=0.225, l2=0.175, z_range=(0, 0.15))


def create_epson_ls6() -> SCARAArm:
    """Create an Epson LS6-B equivalent SCARA.

    Epson LS6-B specifications:
    - Arm lengths: 300mm + 250mm
    - Z stroke: 200mm
    - Payload: 6kg

    Returns:
        SCARAArm configured like Epson LS6-B.
    """
    return create_scara(l1=0.3, l2=0.25, z_range=(0, 0.2))
