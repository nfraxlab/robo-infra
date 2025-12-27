"""Kinematics solvers for robot arms and mechanisms.

This module provides kinematic calculations for multi-link mechanisms:

- Forward kinematics: Joint angles → End effector position
- Inverse kinematics: End effector position → Joint angles

Supports 2-DOF and 3-DOF planar arms with geometric solutions.

Example:
    >>> from robo_infra.motion import TwoLinkArm, EndEffectorPose
    >>>
    >>> # Create a 2-link arm with 100mm links
    >>> arm = TwoLinkArm(l1=100.0, l2=100.0)
    >>>
    >>> # Forward kinematics: angles to position
    >>> pose = arm.forward([0.0, 0.0])
    >>> print(f"End effector at ({pose.x}, {pose.y})")
    End effector at (200.0, 0.0)
    >>>
    >>> # Inverse kinematics: position to angles
    >>> angles = arm.inverse(EndEffectorPose(x=150.0, y=50.0))
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum


class ElbowConfiguration(Enum):
    """Elbow configuration for inverse kinematics.

    For a 2-DOF arm, there are typically two solutions for a given
    end effector position: elbow up or elbow down.
    """

    UP = "up"
    DOWN = "down"


@dataclass(frozen=True, slots=True)
class JointAngle:
    """A single joint angle with optional name and limits.

    Represents the angle of a single joint in a kinematic chain.
    Angles are in radians by default.

    Attributes:
        angle: Joint angle in radians.
        name: Optional name for the joint (e.g., "shoulder", "elbow").
        min_angle: Minimum allowed angle in radians. Default: -π.
        max_angle: Maximum allowed angle in radians. Default: π.

    Example:
        >>> joint = JointAngle(angle=1.57, name="shoulder")
        >>> print(f"{joint.name}: {math.degrees(joint.angle):.1f}°")
        shoulder: 90.0°
    """

    angle: float
    name: str = ""
    min_angle: float = field(default=-math.pi)
    max_angle: float = field(default=math.pi)

    def __post_init__(self) -> None:
        """Validate joint angle."""
        if self.min_angle > self.max_angle:
            raise ValueError(
                f"min_angle ({self.min_angle}) must be <= max_angle ({self.max_angle})"
            )

    @property
    def degrees(self) -> float:
        """Return the angle in degrees."""
        return math.degrees(self.angle)

    @property
    def is_within_limits(self) -> bool:
        """Check if the angle is within the specified limits."""
        return self.min_angle <= self.angle <= self.max_angle

    def clamped(self) -> JointAngle:
        """Return a new JointAngle clamped to limits."""
        clamped_angle = max(self.min_angle, min(self.max_angle, self.angle))
        return JointAngle(
            angle=clamped_angle,
            name=self.name,
            min_angle=self.min_angle,
            max_angle=self.max_angle,
        )

    def normalized(self) -> JointAngle:
        """Return a new JointAngle with angle normalized to [-π, π]."""
        normalized = math.atan2(math.sin(self.angle), math.cos(self.angle))
        return JointAngle(
            angle=normalized,
            name=self.name,
            min_angle=self.min_angle,
            max_angle=self.max_angle,
        )

    def __repr__(self) -> str:
        """Return a string representation."""
        name_str = f"'{self.name}', " if self.name else ""
        return f"JointAngle({name_str}{self.degrees:.1f}°)"


@dataclass(frozen=True, slots=True)
class EndEffectorPose:
    """Position and orientation of an end effector.

    Represents the Cartesian position of the end effector (tool tip)
    in the robot's coordinate frame. For planar arms, z is typically 0.

    Attributes:
        x: X position in the robot's coordinate frame.
        y: Y position in the robot's coordinate frame.
        z: Z position (default 0 for planar arms).
        orientation: Orientation angle in radians (for arms with wrist).

    Example:
        >>> pose = EndEffectorPose(x=100.0, y=50.0)
        >>> print(f"Position: ({pose.x}, {pose.y})")
        Position: (100.0, 50.0)
    """

    x: float
    y: float
    z: float = 0.0
    orientation: float = 0.0

    @property
    def distance_from_origin(self) -> float:
        """Calculate distance from origin (reach)."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    @property
    def planar_distance(self) -> float:
        """Calculate distance from origin in XY plane."""
        return math.sqrt(self.x**2 + self.y**2)

    @property
    def angle_from_x_axis(self) -> float:
        """Calculate angle from positive X axis in XY plane (radians)."""
        return math.atan2(self.y, self.x)

    def offset(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> EndEffectorPose:
        """Return a new pose offset by the given amounts."""
        return EndEffectorPose(
            x=self.x + dx,
            y=self.y + dy,
            z=self.z + dz,
            orientation=self.orientation,
        )

    def scaled(self, factor: float) -> EndEffectorPose:
        """Return a new pose with position scaled by factor."""
        return EndEffectorPose(
            x=self.x * factor,
            y=self.y * factor,
            z=self.z * factor,
            orientation=self.orientation,
        )

    def rotated_z(self, angle: float) -> EndEffectorPose:
        """Return a new pose rotated around Z axis by angle (radians)."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        new_x = self.x * cos_a - self.y * sin_a
        new_y = self.x * sin_a + self.y * cos_a
        return EndEffectorPose(
            x=new_x,
            y=new_y,
            z=self.z,
            orientation=self.orientation + angle,
        )

    def __repr__(self) -> str:
        """Return a string representation."""
        if self.z == 0.0 and self.orientation == 0.0:
            return f"EndEffectorPose(x={self.x:.3f}, y={self.y:.3f})"
        return (
            f"EndEffectorPose(x={self.x:.3f}, y={self.y:.3f}, "
            f"z={self.z:.3f}, θ={math.degrees(self.orientation):.1f}°)"
        )


class KinematicsError(Exception):
    """Base exception for kinematics errors."""

    pass


class UnreachablePositionError(KinematicsError):
    """Raised when a position is outside the workspace."""

    def __init__(self, pose: EndEffectorPose, max_reach: float, min_reach: float = 0.0) -> None:
        """Initialize with pose and reach information."""
        self.pose = pose
        self.max_reach = max_reach
        self.min_reach = min_reach
        distance = pose.planar_distance
        super().__init__(
            f"Position ({pose.x:.3f}, {pose.y:.3f}) is unreachable. "
            f"Distance {distance:.3f} not in range [{min_reach:.3f}, {max_reach:.3f}]"
        )


class JointLimitError(KinematicsError):
    """Raised when a joint angle exceeds its limits."""

    def __init__(self, joint_index: int, angle: float, min_angle: float, max_angle: float) -> None:
        """Initialize with joint and limit information."""
        self.joint_index = joint_index
        self.angle = angle
        self.min_angle = min_angle
        self.max_angle = max_angle
        super().__init__(
            f"Joint {joint_index} angle {math.degrees(angle):.1f}° "
            f"exceeds limits [{math.degrees(min_angle):.1f}°, {math.degrees(max_angle):.1f}°]"
        )


@dataclass
class JointLimits:
    """Limits for a single joint.

    Attributes:
        min_angle: Minimum allowed angle in radians.
        max_angle: Maximum allowed angle in radians.
        name: Optional name for the joint.
    """

    min_angle: float = -math.pi
    max_angle: float = math.pi
    name: str = ""

    def __post_init__(self) -> None:
        """Validate limits."""
        if self.min_angle > self.max_angle:
            raise ValueError(
                f"min_angle ({self.min_angle}) must be <= max_angle ({self.max_angle})"
            )

    def contains(self, angle: float) -> bool:
        """Check if angle is within limits."""
        return self.min_angle <= angle <= self.max_angle

    def clamp(self, angle: float) -> float:
        """Clamp angle to limits."""
        return max(self.min_angle, min(self.max_angle, angle))

    @property
    def range(self) -> float:
        """Return the range of motion."""
        return self.max_angle - self.min_angle

    @property
    def center(self) -> float:
        """Return the center of the range."""
        return (self.min_angle + self.max_angle) / 2


class KinematicChain(ABC):
    """Abstract base class for kinematic chains.

    A kinematic chain is a series of links connected by joints.
    This class provides the interface for forward and inverse
    kinematics calculations.

    Subclasses must implement:
    - forward(): Calculate end effector pose from joint angles
    - inverse(): Calculate joint angles from end effector pose

    Attributes:
        link_lengths: Length of each link in the chain.
        joint_limits: Limits for each joint.
        num_joints: Number of joints in the chain.
    """

    _link_lengths: list[float]
    _joint_limits: list[JointLimits]

    def __init__(
        self,
        link_lengths: list[float],
        joint_limits: list[JointLimits] | None = None,
    ) -> None:
        """Initialize a kinematic chain.

        Args:
            link_lengths: Length of each link. Must have at least 1 element.
            joint_limits: Optional limits for each joint. If None, defaults
                to [-π, π] for each joint.

        Raises:
            ValueError: If link_lengths is empty or joint_limits length
                doesn't match link_lengths.
        """
        if not link_lengths:
            raise ValueError("link_lengths must have at least 1 element")

        self._link_lengths = list(link_lengths)

        if joint_limits is None:
            self._joint_limits = [JointLimits() for _ in link_lengths]
        else:
            if len(joint_limits) != len(link_lengths):
                raise ValueError(
                    f"joint_limits length ({len(joint_limits)}) must match "
                    f"link_lengths ({len(link_lengths)})"
                )
            self._joint_limits = list(joint_limits)

    @property
    def link_lengths(self) -> list[float]:
        """Return a copy of the link lengths."""
        return list(self._link_lengths)

    @property
    def joint_limits(self) -> list[JointLimits]:
        """Return a copy of the joint limits."""
        return list(self._joint_limits)

    @property
    def num_joints(self) -> int:
        """Return the number of joints."""
        return len(self._link_lengths)

    @property
    def total_length(self) -> float:
        """Return the total length of all links (maximum reach)."""
        return sum(self._link_lengths)

    @property
    def min_reach(self) -> float:
        """Return the minimum reach (when fully folded).

        For a chain with links L1, L2, ..., Ln:
        min_reach = |L1 - L2 - L3 - ... - Ln| or 0 if it can fold to origin.
        """
        if len(self._link_lengths) == 1:
            return self._link_lengths[0]
        # Minimum reach is difference between first link and sum of rest
        first = self._link_lengths[0]
        rest = sum(self._link_lengths[1:])
        return max(0.0, abs(first - rest))

    @property
    def max_reach(self) -> float:
        """Return the maximum reach (when fully extended)."""
        return self.total_length

    def is_reachable(self, pose: EndEffectorPose) -> bool:
        """Check if a pose is within the reachable workspace.

        Args:
            pose: Target end effector pose.

        Returns:
            True if the pose is reachable.
        """
        distance = pose.planar_distance
        return self.min_reach <= distance <= self.max_reach

    def validate_angles(self, angles: list[float], raise_on_error: bool = True) -> bool:
        """Validate that angles are within joint limits.

        Args:
            angles: List of joint angles to validate.
            raise_on_error: If True, raise JointLimitError on violation.

        Returns:
            True if all angles are within limits.

        Raises:
            ValueError: If angles length doesn't match num_joints.
            JointLimitError: If any angle exceeds limits (and raise_on_error=True).
        """
        if len(angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} angles, got {len(angles)}")

        for i, (angle, limits) in enumerate(zip(angles, self._joint_limits, strict=True)):
            if not limits.contains(angle):
                if raise_on_error:
                    raise JointLimitError(i, angle, limits.min_angle, limits.max_angle)
                return False
        return True

    def clamp_angles(self, angles: list[float]) -> list[float]:
        """Clamp angles to their respective joint limits.

        Args:
            angles: List of joint angles to clamp.

        Returns:
            List of clamped angles.

        Raises:
            ValueError: If angles length doesn't match num_joints.
        """
        if len(angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} angles, got {len(angles)}")

        return [
            limits.clamp(angle) for angle, limits in zip(angles, self._joint_limits, strict=True)
        ]

    @abstractmethod
    def forward(self, angles: list[float]) -> EndEffectorPose:
        """Calculate end effector pose from joint angles.

        Args:
            angles: List of joint angles in radians. Length must match num_joints.

        Returns:
            EndEffectorPose representing the end effector position.

        Raises:
            ValueError: If angles length doesn't match num_joints.
        """
        ...

    @abstractmethod
    def inverse(
        self,
        pose: EndEffectorPose,
        configuration: ElbowConfiguration = ElbowConfiguration.UP,
    ) -> list[float]:
        """Calculate joint angles from end effector pose.

        Args:
            pose: Target end effector pose.
            configuration: Elbow configuration (UP or DOWN) for redundant solutions.

        Returns:
            List of joint angles in radians that achieve the pose.

        Raises:
            UnreachablePositionError: If the pose is outside the workspace.
        """
        ...

    def jacobian(self, angles: list[float]) -> list[list[float]]:
        """Calculate the Jacobian matrix at the given joint angles.

        The Jacobian relates joint velocities to end effector velocity.
        Default implementation uses numerical differentiation.

        Args:
            angles: Current joint angles.

        Returns:
            Jacobian matrix as list of lists [dx/dθ1, dx/dθ2, ...], [dy/dθ1, dy/dθ2, ...].
        """
        delta = 1e-6
        jacobian: list[list[float]] = [[], []]

        for i in range(self.num_joints):
            # Perturb angle i
            angles_plus = list(angles)
            angles_plus[i] += delta

            angles_minus = list(angles)
            angles_minus[i] -= delta

            pose_plus = self.forward(angles_plus)
            pose_minus = self.forward(angles_minus)

            # Numerical derivatives
            dx_dtheta = (pose_plus.x - pose_minus.x) / (2 * delta)
            dy_dtheta = (pose_plus.y - pose_minus.y) / (2 * delta)

            jacobian[0].append(dx_dtheta)
            jacobian[1].append(dy_dtheta)

        return jacobian

    def __repr__(self) -> str:
        """Return a string representation."""
        return (
            f"{self.__class__.__name__}(links={self._link_lengths}, max_reach={self.max_reach:.3f})"
        )


class TwoLinkArm(KinematicChain):
    """A 2-DOF planar arm with two rotational joints.

    This is the classic "double pendulum" configuration commonly used
    in robot arms. Both joints rotate in the XY plane.

    The arm consists of:
    - Shoulder joint (θ1): Rotates the entire arm
    - Elbow joint (θ2): Rotates the forearm relative to the upper arm

    Forward kinematics:
        x = L1·cos(θ1) + L2·cos(θ1 + θ2)
        y = L1·sin(θ1) + L2·sin(θ1 + θ2)

    Attributes:
        l1: Length of the first link (upper arm).
        l2: Length of the second link (forearm).

    Example:
        >>> arm = TwoLinkArm(l1=100.0, l2=100.0)
        >>>
        >>> # Forward kinematics
        >>> pose = arm.forward([0.0, 0.0])  # Arm extended along X axis
        >>> print(f"End effector: ({pose.x:.1f}, {pose.y:.1f})")
        End effector: (200.0, 0.0)
        >>>
        >>> # Inverse kinematics
        >>> angles = arm.inverse(EndEffectorPose(x=100.0, y=100.0))
        >>> print(f"Shoulder: {math.degrees(angles[0]):.1f}°")
    """

    def __init__(
        self,
        l1: float,
        l2: float,
        joint_limits: list[JointLimits] | None = None,
    ) -> None:
        """Initialize a 2-link arm.

        Args:
            l1: Length of the first link (shoulder to elbow).
            l2: Length of the second link (elbow to end effector).
            joint_limits: Optional limits for [shoulder, elbow]. Defaults to [-π, π].

        Raises:
            ValueError: If l1 or l2 is not positive.
        """
        if l1 <= 0:
            raise ValueError(f"l1 must be positive, got {l1}")
        if l2 <= 0:
            raise ValueError(f"l2 must be positive, got {l2}")

        super().__init__(
            link_lengths=[l1, l2],
            joint_limits=joint_limits,
        )

    @property
    def l1(self) -> float:
        """Length of the first link (upper arm)."""
        return self._link_lengths[0]

    @property
    def l2(self) -> float:
        """Length of the second link (forearm)."""
        return self._link_lengths[1]

    def forward(self, angles: list[float]) -> EndEffectorPose:
        """Calculate end effector position from joint angles.

        Uses the standard 2-link planar arm forward kinematics:
            x = L1·cos(θ1) + L2·cos(θ1 + θ2)
            y = L1·sin(θ1) + L2·sin(θ1 + θ2)

        Args:
            angles: [θ1, θ2] joint angles in radians.

        Returns:
            EndEffectorPose with x, y position.

        Raises:
            ValueError: If angles doesn't have exactly 2 elements.
        """
        if len(angles) != 2:
            raise ValueError(f"Expected 2 angles, got {len(angles)}")

        theta1, theta2 = angles

        # Position of end effector
        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)

        # End effector orientation (angle of the second link)
        orientation = theta1 + theta2

        return EndEffectorPose(x=x, y=y, z=0.0, orientation=orientation)

    def elbow_position(self, angles: list[float]) -> tuple[float, float]:
        """Calculate the position of the elbow joint.

        Args:
            angles: [θ1, θ2] joint angles in radians.

        Returns:
            (x, y) position of the elbow.

        Raises:
            ValueError: If angles doesn't have exactly 2 elements.
        """
        if len(angles) != 2:
            raise ValueError(f"Expected 2 angles, got {len(angles)}")

        theta1 = angles[0]
        x = self.l1 * math.cos(theta1)
        y = self.l1 * math.sin(theta1)
        return (x, y)

    def inverse(
        self,
        pose: EndEffectorPose,
        configuration: ElbowConfiguration = ElbowConfiguration.UP,
    ) -> list[float]:
        """Calculate joint angles from end effector position.

        Uses the law of cosines to solve the 2-link IK problem.
        There are typically two solutions (elbow up/down) for reachable positions.

        Args:
            pose: Target end effector position.
            configuration: ElbowConfiguration.UP or ElbowConfiguration.DOWN.

        Returns:
            [θ1, θ2] joint angles in radians.

        Raises:
            UnreachablePositionError: If the position is outside the workspace.
        """
        x, y = pose.x, pose.y

        # Distance from origin to target
        distance_sq = x * x + y * y
        distance = math.sqrt(distance_sq)

        # Check if position is reachable
        if distance > self.max_reach:
            raise UnreachablePositionError(pose, self.max_reach, self.min_reach)
        if distance < self.min_reach:
            raise UnreachablePositionError(pose, self.max_reach, self.min_reach)

        # Handle the edge case of very small distances (at origin)
        if distance < 1e-10:
            # At origin, return folded configuration
            if configuration == ElbowConfiguration.UP:
                return [0.0, math.pi]
            else:
                return [0.0, -math.pi]

        # Law of cosines to find θ2
        # cos(θ2) = (x² + y² - L1² - L2²) / (2·L1·L2)
        cos_theta2 = (distance_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)

        # Clamp to [-1, 1] to handle numerical errors at boundaries
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        # θ2 has two solutions: positive (elbow down) or negative (elbow up)
        if configuration == ElbowConfiguration.UP:
            theta2 = -math.acos(cos_theta2)  # Elbow up (negative θ2)
        else:
            theta2 = math.acos(cos_theta2)  # Elbow down (positive θ2)

        # Calculate θ1 using atan2 formula for 2-link IK
        sin_theta2 = math.sin(theta2)
        k1 = self.l1 + self.l2 * cos_theta2
        k2 = self.l2 * sin_theta2

        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return [theta1, theta2]

    def inverse_with_orientation(
        self,
        pose: EndEffectorPose,
    ) -> list[float] | None:
        """Calculate joint angles to achieve position AND orientation.

        For a 2-DOF arm, achieving both position and orientation is only
        possible if the target is on the reachable workspace AND the
        orientation matches what's geometrically possible.

        Args:
            pose: Target pose with position and orientation.

        Returns:
            [θ1, θ2] if achievable, None if not possible.
        """
        # θ1 + θ2 = orientation (end effector angle)
        # So θ1 = orientation - θ2
        target_orientation = pose.orientation
        x, y = pose.x, pose.y

        # Distance from origin
        distance_sq = x * x + y * y
        distance = math.sqrt(distance_sq)

        # Check reachability
        if distance > self.max_reach or distance < self.min_reach:
            return None

        # Solve for θ2 using law of cosines
        cos_theta2 = (distance_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        # Try both elbow configurations
        for sign in [-1, 1]:
            theta2 = sign * math.acos(cos_theta2)
            theta1 = target_orientation - theta2

            # Verify this solution reaches the target position
            check_x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
            check_y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)

            if abs(check_x - x) < 1e-6 and abs(check_y - y) < 1e-6:
                return [theta1, theta2]

        return None

    def analytical_jacobian(self, angles: list[float]) -> list[list[float]]:
        """Calculate the analytical Jacobian matrix.

        For a 2-link arm, the Jacobian is:
            J = [[-L1·sin(θ1) - L2·sin(θ1+θ2), -L2·sin(θ1+θ2)],
                 [ L1·cos(θ1) + L2·cos(θ1+θ2),  L2·cos(θ1+θ2)]]

        Args:
            angles: [θ1, θ2] joint angles.

        Returns:
            2x2 Jacobian matrix as [[dx/dθ1, dx/dθ2], [dy/dθ1, dy/dθ2]].
        """
        if len(angles) != 2:
            raise ValueError(f"Expected 2 angles, got {len(angles)}")

        theta1, theta2 = angles
        theta12 = theta1 + theta2

        # Partial derivatives
        dx_dtheta1 = -self.l1 * math.sin(theta1) - self.l2 * math.sin(theta12)
        dx_dtheta2 = -self.l2 * math.sin(theta12)
        dy_dtheta1 = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta12)
        dy_dtheta2 = self.l2 * math.cos(theta12)

        return [[dx_dtheta1, dx_dtheta2], [dy_dtheta1, dy_dtheta2]]

    def is_singular(self, angles: list[float], tolerance: float = 1e-6) -> bool:
        """Check if the arm is in a singular configuration.

        Singularities occur when the arm is fully extended or fully folded,
        making the Jacobian rank-deficient.

        Args:
            angles: [θ1, θ2] joint angles.
            tolerance: Threshold for considering θ2 as 0 or π.

        Returns:
            True if the configuration is singular.
        """
        if len(angles) != 2:
            raise ValueError(f"Expected 2 angles, got {len(angles)}")

        theta2 = angles[1]
        # Singular when θ2 ≈ 0 (fully extended) or θ2 ≈ ±π (fully folded)
        return abs(math.sin(theta2)) < tolerance

    def workspace_boundary(self, num_points: int = 100) -> list[EndEffectorPose]:
        """Generate points on the workspace boundary.

        The workspace is an annulus (ring) between min_reach and max_reach.

        Args:
            num_points: Number of points to generate on each boundary.

        Returns:
            List of EndEffectorPose on the workspace boundary.
        """
        boundary: list[EndEffectorPose] = []

        # Outer boundary (max reach)
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = self.max_reach * math.cos(angle)
            y = self.max_reach * math.sin(angle)
            boundary.append(EndEffectorPose(x=x, y=y))

        # Inner boundary (min reach) - only if there's a hole
        if self.min_reach > 0:
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                x = self.min_reach * math.cos(angle)
                y = self.min_reach * math.sin(angle)
                boundary.append(EndEffectorPose(x=x, y=y))

        return boundary

    def __repr__(self) -> str:
        """Return a string representation."""
        return f"TwoLinkArm(l1={self.l1:.3f}, l2={self.l2:.3f})"


class ThreeLinkArm(KinematicChain):
    """A 3-DOF planar arm with three rotational joints.

    This arm adds a wrist joint to the 2-link arm, allowing independent
    control of end effector orientation. All joints rotate in the XY plane.

    The arm consists of:
    - Shoulder joint (θ1): Rotates the entire arm
    - Elbow joint (θ2): Rotates the forearm relative to the upper arm
    - Wrist joint (θ3): Rotates the end effector relative to the forearm

    Forward kinematics:
        x = L1·cos(θ1) + L2·cos(θ1 + θ2) + L3·cos(θ1 + θ2 + θ3)
        y = L1·sin(θ1) + L2·sin(θ1 + θ2) + L3·sin(θ1 + θ2 + θ3)
        orientation = θ1 + θ2 + θ3

    For inverse kinematics, this arm is redundant (3 DOF for 2D position + orientation),
    so there are multiple solutions. We use the wrist position to reduce to a 2-link
    problem, then solve for the wrist angle.

    Attributes:
        l1: Length of the first link (upper arm).
        l2: Length of the second link (forearm).
        l3: Length of the third link (wrist/hand).

    Example:
        >>> arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        >>>
        >>> # Forward kinematics
        >>> pose = arm.forward([0.0, 0.0, 0.0])  # Arm extended along X axis
        >>> print(f"End effector: ({pose.x:.1f}, {pose.y:.1f})")
        End effector: (230.0, 0.0)
        >>>
        >>> # Inverse kinematics with target orientation
        >>> angles = arm.inverse(EndEffectorPose(x=150.0, y=50.0, orientation=0.5))
    """

    def __init__(
        self,
        l1: float,
        l2: float,
        l3: float,
        joint_limits: list[JointLimits] | None = None,
    ) -> None:
        """Initialize a 3-link arm.

        Args:
            l1: Length of the first link (shoulder to elbow).
            l2: Length of the second link (elbow to wrist).
            l3: Length of the third link (wrist to end effector).
            joint_limits: Optional limits for [shoulder, elbow, wrist].
                Defaults to [-π, π] for all joints.

        Raises:
            ValueError: If any link length is not positive.
        """
        if l1 <= 0:
            raise ValueError(f"l1 must be positive, got {l1}")
        if l2 <= 0:
            raise ValueError(f"l2 must be positive, got {l2}")
        if l3 <= 0:
            raise ValueError(f"l3 must be positive, got {l3}")

        super().__init__(
            link_lengths=[l1, l2, l3],
            joint_limits=joint_limits,
        )

    @property
    def l1(self) -> float:
        """Length of the first link (upper arm)."""
        return self._link_lengths[0]

    @property
    def l2(self) -> float:
        """Length of the second link (forearm)."""
        return self._link_lengths[1]

    @property
    def l3(self) -> float:
        """Length of the third link (wrist/hand)."""
        return self._link_lengths[2]

    @property
    def min_reach(self) -> float:
        """Return the minimum reach.

        For a 3-link arm, minimum reach depends on the configuration.
        We use a simplified calculation assuming the arm can fold back.
        """
        # If all links can fold, min reach could be 0
        # Otherwise it's the longest link minus sum of others
        lengths = sorted(self._link_lengths, reverse=True)
        diff = lengths[0] - sum(lengths[1:])
        return max(0.0, diff)

    def forward(self, angles: list[float]) -> EndEffectorPose:
        """Calculate end effector position from joint angles.

        Uses the standard 3-link planar arm forward kinematics:
            x = L1·cos(θ1) + L2·cos(θ1+θ2) + L3·cos(θ1+θ2+θ3)
            y = L1·sin(θ1) + L2·sin(θ1+θ2) + L3·sin(θ1+θ2+θ3)
            orientation = θ1 + θ2 + θ3

        Args:
            angles: [θ1, θ2, θ3] joint angles in radians.

        Returns:
            EndEffectorPose with x, y position and orientation.

        Raises:
            ValueError: If angles doesn't have exactly 3 elements.
        """
        if len(angles) != 3:
            raise ValueError(f"Expected 3 angles, got {len(angles)}")

        theta1, theta2, theta3 = angles
        theta12 = theta1 + theta2
        theta123 = theta12 + theta3

        # Position of end effector
        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta12) + self.l3 * math.cos(theta123)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta12) + self.l3 * math.sin(theta123)

        return EndEffectorPose(x=x, y=y, z=0.0, orientation=theta123)

    def elbow_position(self, angles: list[float]) -> tuple[float, float]:
        """Calculate the position of the elbow joint.

        Args:
            angles: [θ1, θ2, θ3] joint angles in radians.

        Returns:
            (x, y) position of the elbow.

        Raises:
            ValueError: If angles doesn't have exactly 3 elements.
        """
        if len(angles) != 3:
            raise ValueError(f"Expected 3 angles, got {len(angles)}")

        theta1 = angles[0]
        x = self.l1 * math.cos(theta1)
        y = self.l1 * math.sin(theta1)
        return (x, y)

    def wrist_position(self, angles: list[float]) -> tuple[float, float]:
        """Calculate the position of the wrist joint.

        Args:
            angles: [θ1, θ2, θ3] joint angles in radians.

        Returns:
            (x, y) position of the wrist.

        Raises:
            ValueError: If angles doesn't have exactly 3 elements.
        """
        if len(angles) != 3:
            raise ValueError(f"Expected 3 angles, got {len(angles)}")

        theta1, theta2, _ = angles
        theta12 = theta1 + theta2

        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta12)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta12)
        return (x, y)

    def inverse(
        self,
        pose: EndEffectorPose,
        configuration: ElbowConfiguration = ElbowConfiguration.UP,
    ) -> list[float]:
        """Calculate joint angles from end effector pose with orientation.

        Uses decoupled approach: first find wrist position, solve 2-link IK
        for shoulder and elbow, then calculate wrist angle.

        Args:
            pose: Target end effector pose (uses x, y, orientation).
            configuration: ElbowConfiguration.UP or ElbowConfiguration.DOWN.

        Returns:
            [θ1, θ2, θ3] joint angles in radians.

        Raises:
            UnreachablePositionError: If the position is outside the workspace.
        """
        x_ee, y_ee = pose.x, pose.y
        target_orientation = pose.orientation

        # Calculate wrist position by subtracting L3 in the target direction
        x_wrist = x_ee - self.l3 * math.cos(target_orientation)
        y_wrist = y_ee - self.l3 * math.sin(target_orientation)

        # Check if wrist position is reachable by the 2-link arm (L1 + L2)
        wrist_distance = math.sqrt(x_wrist**2 + y_wrist**2)
        max_wrist_reach = self.l1 + self.l2
        min_wrist_reach = abs(self.l1 - self.l2)

        if wrist_distance > max_wrist_reach or wrist_distance < min_wrist_reach:
            raise UnreachablePositionError(pose, self.max_reach, self.min_reach)

        # Solve 2-link IK for wrist position
        # Law of cosines for θ2
        wrist_dist_sq = x_wrist**2 + y_wrist**2
        cos_theta2 = (wrist_dist_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        if configuration == ElbowConfiguration.UP:
            theta2 = -math.acos(cos_theta2)
        else:
            theta2 = math.acos(cos_theta2)

        # Calculate θ1
        sin_theta2 = math.sin(theta2)
        k1 = self.l1 + self.l2 * cos_theta2
        k2 = self.l2 * sin_theta2
        theta1 = math.atan2(y_wrist, x_wrist) - math.atan2(k2, k1)

        # Calculate θ3 from the orientation constraint
        # orientation = θ1 + θ2 + θ3, so θ3 = orientation - θ1 - θ2
        theta3 = target_orientation - theta1 - theta2

        return [theta1, theta2, theta3]

    def inverse_position_only(
        self,
        pose: EndEffectorPose,
        configuration: ElbowConfiguration = ElbowConfiguration.UP,
        wrist_angle: float = 0.0,
    ) -> list[float]:
        """Calculate joint angles for position only (ignores orientation).

        This method treats the arm as a 2-link arm (L1, L2+L3) and uses
        the wrist_angle parameter to distribute the end effector.

        Args:
            pose: Target end effector position (orientation ignored).
            configuration: ElbowConfiguration.UP or ElbowConfiguration.DOWN.
            wrist_angle: Fixed wrist angle (θ3) to use.

        Returns:
            [θ1, θ2, θ3] joint angles in radians.

        Raises:
            UnreachablePositionError: If the position is outside the workspace.
        """
        x_ee, y_ee = pose.x, pose.y

        # Distance from origin to target
        distance_sq = x_ee**2 + y_ee**2
        distance = math.sqrt(distance_sq)

        if distance > self.max_reach or distance < self.min_reach:
            raise UnreachablePositionError(pose, self.max_reach, self.min_reach)

        # For position-only IK, we try different wrist positions
        # Search for a solution by varying θ1+θ2 (the wrist angle direction)
        best_solution: list[float] | None = None
        best_error = float("inf")

        # Try multiple orientations to find reachable wrist position
        for i in range(36):  # Try 36 different orientations
            test_orientation = 2 * math.pi * i / 36

            # Wrist position
            x_wrist = x_ee - self.l3 * math.cos(test_orientation)
            y_wrist = y_ee - self.l3 * math.sin(test_orientation)

            wrist_dist = math.sqrt(x_wrist**2 + y_wrist**2)
            max_wrist_reach = self.l1 + self.l2
            min_wrist_reach = abs(self.l1 - self.l2)

            if min_wrist_reach <= wrist_dist <= max_wrist_reach:
                # Solve 2-link IK
                wrist_dist_sq = x_wrist**2 + y_wrist**2
                cos_theta2 = (wrist_dist_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
                cos_theta2 = max(-1.0, min(1.0, cos_theta2))

                if configuration == ElbowConfiguration.UP:
                    theta2 = -math.acos(cos_theta2)
                else:
                    theta2 = math.acos(cos_theta2)

                sin_theta2 = math.sin(theta2)
                k1 = self.l1 + self.l2 * cos_theta2
                k2 = self.l2 * sin_theta2
                theta1 = math.atan2(y_wrist, x_wrist) - math.atan2(k2, k1)

                theta3 = test_orientation - theta1 - theta2

                # Prefer solutions where θ3 is close to wrist_angle
                error = abs(theta3 - wrist_angle)
                if error < best_error:
                    best_error = error
                    best_solution = [theta1, theta2, theta3]

        if best_solution is None:
            raise UnreachablePositionError(pose, self.max_reach, self.min_reach)

        return best_solution

    def analytical_jacobian(self, angles: list[float]) -> list[list[float]]:
        """Calculate the analytical Jacobian matrix.

        For a 3-link arm, the Jacobian is 3x3:
            [[dx/dθ1, dx/dθ2, dx/dθ3],
             [dy/dθ1, dy/dθ2, dy/dθ3],
             [dφ/dθ1, dφ/dθ2, dφ/dθ3]]

        where φ is the end effector orientation.

        Args:
            angles: [θ1, θ2, θ3] joint angles.

        Returns:
            3x3 Jacobian matrix.
        """
        if len(angles) != 3:
            raise ValueError(f"Expected 3 angles, got {len(angles)}")

        theta1, theta2, theta3 = angles
        theta12 = theta1 + theta2
        theta123 = theta12 + theta3

        s1 = math.sin(theta1)
        c1 = math.cos(theta1)
        s12 = math.sin(theta12)
        c12 = math.cos(theta12)
        s123 = math.sin(theta123)
        c123 = math.cos(theta123)

        # Partial derivatives for x
        dx_dtheta1 = -self.l1 * s1 - self.l2 * s12 - self.l3 * s123
        dx_dtheta2 = -self.l2 * s12 - self.l3 * s123
        dx_dtheta3 = -self.l3 * s123

        # Partial derivatives for y
        dy_dtheta1 = self.l1 * c1 + self.l2 * c12 + self.l3 * c123
        dy_dtheta2 = self.l2 * c12 + self.l3 * c123
        dy_dtheta3 = self.l3 * c123

        # Partial derivatives for orientation (all 1s since φ = θ1 + θ2 + θ3)
        dphi_dtheta1 = 1.0
        dphi_dtheta2 = 1.0
        dphi_dtheta3 = 1.0

        return [
            [dx_dtheta1, dx_dtheta2, dx_dtheta3],
            [dy_dtheta1, dy_dtheta2, dy_dtheta3],
            [dphi_dtheta1, dphi_dtheta2, dphi_dtheta3],
        ]

    def is_singular(self, angles: list[float], tolerance: float = 1e-6) -> bool:
        """Check if the arm is in a singular configuration.

        Singularities occur when the Jacobian loses rank. For a 3-link arm,
        this happens when θ2 ≈ 0 or ±π (arm aligned).

        Args:
            angles: [θ1, θ2, θ3] joint angles.
            tolerance: Threshold for considering angles as 0 or π.

        Returns:
            True if the configuration is singular.
        """
        if len(angles) != 3:
            raise ValueError(f"Expected 3 angles, got {len(angles)}")

        theta2 = angles[1]
        # Primary singularity when θ2 ≈ 0 or ±π
        return abs(math.sin(theta2)) < tolerance

    def __repr__(self) -> str:
        """Return a string representation."""
        return f"ThreeLinkArm(l1={self.l1:.3f}, l2={self.l2:.3f}, l3={self.l3:.3f})"
