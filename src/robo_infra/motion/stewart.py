"""Stewart platform (hexapod) kinematics.

The Stewart platform is a 6-DOF parallel manipulator with six linear
actuators connecting a fixed base to a moving platform. It's used in
flight simulators, precision positioning systems, and machine tools.

Architecture:
- 6 linear actuators (legs)
- Fixed base platform with 6 attachment points
- Moving platform with 6 attachment points
- Full 6-DOF motion (3 translations + 3 rotations)

Key Features:
- Very high stiffness (parallel architecture)
- High precision positioning
- High load capacity
- Complex FK (iterative), simple IK (closed-form)
- Limited workspace compared to serial robots

Common Configurations:
- 6-6 platform: 6 joints on base, 6 on platform (general)
- 3-3 platform: 3 pairs of close joints on each platform
- 6-3 platform: 6 joints on base, 3 pairs on platform

Example:
    >>> from robo_infra.motion.stewart import StewartPlatform
    >>>
    >>> # Create a standard hexapod
    >>> stewart = StewartPlatform.create_symmetric(
    ...     base_radius=0.3,
    ...     platform_radius=0.15,
    ...     leg_length_range=(0.4, 0.6),
    ... )
    >>>
    >>> # Inverse kinematics (given pose, find leg lengths)
    >>> pose = Transform.identity()
    >>> leg_lengths = stewart.inverse(pose)
    >>> print(f"Leg lengths: {leg_lengths}")
    >>>
    >>> # Forward kinematics (iterative)
    >>> pose = stewart.forward(leg_lengths)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

from robo_infra.motion.transforms import Rotation, Transform


if TYPE_CHECKING:
    from collections.abc import Sequence

    from numpy.typing import NDArray


class StewartSingularityType(Enum):
    """Types of singularities in Stewart platforms."""

    NONE = "none"
    TRANSLATION = "translation"  # Platform can translate infinitesimally without actuator motion
    ROTATION = "rotation"  # Platform can rotate infinitesimally without actuator motion
    COMBINED = "combined"  # Both types


@dataclass
class StewartLimits:
    """Actuator and workspace limits for Stewart platform.

    Attributes:
        leg_length_range: (min, max) leg lengths in meters.
        position_limits: (min, max) for x, y, z in meters.
        rotation_limits: (min, max) rotation angles in radians.
    """

    leg_length_range: tuple[float, float] = (0.3, 0.5)
    position_limits: tuple[float, float] = (-0.1, 0.1)
    rotation_limits: tuple[float, float] = (-0.3, 0.3)  # ~17 degrees

    def is_valid_leg_length(self, length: float) -> bool:
        """Check if leg length is within limits."""
        return self.leg_length_range[0] <= length <= self.leg_length_range[1]

    def clamp_leg_length(self, length: float) -> float:
        """Clamp leg length to valid range."""
        return max(self.leg_length_range[0], min(self.leg_length_range[1], length))


@dataclass
class StewartPose:
    """Pose of the Stewart platform.

    Attributes:
        position: [x, y, z] position of platform center.
        euler_angles: [roll, pitch, yaw] in radians.
    """

    position: NDArray[np.float64]
    euler_angles: NDArray[np.float64]  # [roll, pitch, yaw]

    @classmethod
    def from_transform(cls, transform: Transform) -> StewartPose:
        """Create from Transform object."""
        position = transform.position.copy()
        euler = transform.rotation.as_euler()
        return cls(position=position, euler_angles=euler)

    def to_transform(self) -> Transform:
        """Convert to Transform object."""
        rotation = Rotation.from_euler(tuple(self.euler_angles))
        return Transform(position=self.position.copy(), rotation=rotation)


@dataclass
class StewartJoints:
    """Joint (leg length) values for a Stewart platform.

    Attributes:
        leg_lengths: Array of 6 leg lengths in meters.
    """

    leg_lengths: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Ensure leg_lengths is correct shape."""
        self.leg_lengths = np.asarray(self.leg_lengths)
        if self.leg_lengths.shape != (6,):
            raise ValueError(f"Expected 6 leg lengths, got {self.leg_lengths.shape}")

    def as_tuple(self) -> tuple[float, ...]:
        """Return as tuple of floats."""
        return tuple(self.leg_lengths)

    def __getitem__(self, idx: int) -> float:
        """Get leg length by index."""
        return float(self.leg_lengths[idx])


@dataclass
class StewartPlatform:
    """6-DOF parallel Stewart platform (hexapod).

    Six linear actuators connect a fixed base to a moving platform,
    providing full 6-DOF motion (3 translations + 3 rotations).

    Geometry:
    - Base has 6 attachment points (base_joints)
    - Platform has 6 attachment points (platform_joints)
    - Actuator i connects base_joints[i] to platform_joints[i]

    Coordinate System:
    - Origin at center of base platform
    - Z axis points up
    - Platform pose is relative to base

    Attributes:
        base_joints: 6 points on the base platform (in base frame).
        platform_joints: 6 points on the moving platform (in platform frame).
        home_height: Height of platform center when all legs are at nominal length.
        limits: Actuator and workspace limits.

    Example:
        >>> # Create symmetric hexapod
        >>> stewart = StewartPlatform.create_symmetric(
        ...     base_radius=0.25,
        ...     platform_radius=0.12,
        ...     leg_length_range=(0.35, 0.55),
        ... )
        >>>
        >>> # Find leg lengths for a pose
        >>> pose = Transform.from_euler((0, 0, 0.4), (0.05, 0, 0))
        >>> legs = stewart.inverse(pose)
    """

    base_joints: NDArray[np.float64]  # Shape: (6, 3)
    platform_joints: NDArray[np.float64]  # Shape: (6, 3)
    home_height: float = 0.4
    limits: StewartLimits = field(default_factory=StewartLimits)

    def __post_init__(self) -> None:
        """Validate joint positions."""
        self.base_joints = np.asarray(self.base_joints, dtype=np.float64)
        self.platform_joints = np.asarray(self.platform_joints, dtype=np.float64)

        if self.base_joints.shape != (6, 3):
            raise ValueError(f"base_joints must be (6, 3), got {self.base_joints.shape}")
        if self.platform_joints.shape != (6, 3):
            raise ValueError(f"platform_joints must be (6, 3), got {self.platform_joints.shape}")

    @classmethod
    def create_symmetric(
        cls,
        base_radius: float,
        platform_radius: float,
        leg_length_range: tuple[float, float] = (0.3, 0.5),
        base_half_angle: float = 10.0,
        platform_half_angle: float = 10.0,
        home_height: float | None = None,
    ) -> StewartPlatform:
        """Create a symmetric Stewart platform.

        Creates a 6-6 platform with joint pairs at 60° intervals.
        Each pair of joints is separated by a small angle.

        Args:
            base_radius: Radius of base platform.
            platform_radius: Radius of moving platform.
            leg_length_range: (min, max) leg lengths.
            base_half_angle: Half-angle between joint pairs on base (degrees).
            platform_half_angle: Half-angle between joint pairs on platform (degrees).
            home_height: Height at home position (computed if None).

        Returns:
            Configured StewartPlatform.
        """
        # Generate joint positions
        base_joints = np.zeros((6, 3))
        platform_joints = np.zeros((6, 3))

        base_half_rad = math.radians(base_half_angle)
        platform_half_rad = math.radians(platform_half_angle)

        for i in range(6):
            # Base angle: pairs at 0°, 120°, 240°
            pair_num = i // 2  # 0, 0, 1, 1, 2, 2
            pair_angle = pair_num * math.radians(120)

            # Alternate within pair
            if i % 2 == 0:
                base_angle = pair_angle - base_half_rad
                plat_angle = pair_angle - platform_half_rad + math.radians(60)
            else:
                base_angle = pair_angle + base_half_rad
                plat_angle = pair_angle + platform_half_rad + math.radians(60)

            base_joints[i, 0] = base_radius * math.cos(base_angle)
            base_joints[i, 1] = base_radius * math.sin(base_angle)
            base_joints[i, 2] = 0

            platform_joints[i, 0] = platform_radius * math.cos(plat_angle)
            platform_joints[i, 1] = platform_radius * math.sin(plat_angle)
            platform_joints[i, 2] = 0

        # Compute home height if not specified
        if home_height is None:
            # Use middle of leg length range
            nominal_leg = (leg_length_range[0] + leg_length_range[1]) / 2
            # Rough estimate - legs are nearly vertical at home
            home_height = nominal_leg * 0.9

        limits = StewartLimits(leg_length_range=leg_length_range)

        return cls(
            base_joints=base_joints,
            platform_joints=platform_joints,
            home_height=home_height,
            limits=limits,
        )

    @classmethod
    def create_from_points(
        cls,
        base_joints: Sequence[tuple[float, float, float]],
        platform_joints: Sequence[tuple[float, float, float]],
        leg_length_range: tuple[float, float],
        home_height: float = 0.4,
    ) -> StewartPlatform:
        """Create Stewart platform from explicit joint positions.

        Args:
            base_joints: 6 points (x, y, z) on base platform.
            platform_joints: 6 points (x, y, z) on platform (in platform frame).
            leg_length_range: (min, max) leg lengths.
            home_height: Height at home position.

        Returns:
            Configured StewartPlatform.
        """
        if len(base_joints) != 6 or len(platform_joints) != 6:
            raise ValueError("Must provide exactly 6 joints for each platform")

        limits = StewartLimits(leg_length_range=leg_length_range)

        return cls(
            base_joints=np.array(base_joints, dtype=np.float64),
            platform_joints=np.array(platform_joints, dtype=np.float64),
            home_height=home_height,
            limits=limits,
        )

    def inverse(self, target: Transform) -> StewartJoints:
        """Compute inverse kinematics (closed-form).

        Given a platform pose, compute the 6 leg lengths.

        Args:
            target: Desired platform pose (position + orientation).

        Returns:
            StewartJoints with 6 leg lengths.

        Raises:
            ValueError: If any leg length is outside limits.

        Example:
            >>> stewart = StewartPlatform.create_symmetric(0.25, 0.12)
            >>> pose = Transform.from_euler((0, 0, 0.4), (0.1, 0, 0))
            >>> legs = stewart.inverse(pose)
            >>> print(f"Leg 0: {legs[0]:.4f}m")
        """
        position = target.position
        rotation_matrix = target.rotation.matrix

        leg_lengths = np.zeros(6)

        for i in range(6):
            # Transform platform joint to world frame
            platform_joint_world = position + rotation_matrix @ self.platform_joints[i]

            # Leg vector from base joint to platform joint
            leg_vector = platform_joint_world - self.base_joints[i]

            # Leg length is magnitude
            leg_lengths[i] = np.linalg.norm(leg_vector)

        # Validate leg lengths
        for i, length in enumerate(leg_lengths):
            if not self.limits.is_valid_leg_length(length):
                raise ValueError(
                    f"Leg {i} length {length:.4f}m outside limits "
                    f"[{self.limits.leg_length_range[0]:.4f}, "
                    f"{self.limits.leg_length_range[1]:.4f}]"
                )

        return StewartJoints(leg_lengths=leg_lengths)

    def inverse_array(self, target: Transform) -> NDArray[np.float64]:
        """Compute inverse kinematics returning numpy array."""
        return self.inverse(target).leg_lengths

    def inverse_safe(self, target: Transform) -> tuple[StewartJoints, bool]:
        """Compute IK with clamping instead of raising errors.

        Returns:
            Tuple of (joints, is_valid) where is_valid indicates if all
            leg lengths are within limits.
        """
        position = target.position
        rotation_matrix = target.rotation.matrix

        leg_lengths = np.zeros(6)
        is_valid = True

        for i in range(6):
            platform_joint_world = position + rotation_matrix @ self.platform_joints[i]
            leg_vector = platform_joint_world - self.base_joints[i]
            length = np.linalg.norm(leg_vector)

            if not self.limits.is_valid_leg_length(length):
                is_valid = False
                length = self.limits.clamp_leg_length(length)

            leg_lengths[i] = length

        return StewartJoints(leg_lengths=leg_lengths), is_valid

    def forward(
        self,
        leg_lengths: NDArray[np.float64] | StewartJoints,
        initial_guess: Transform | None = None,
        max_iterations: int = 100,
        tolerance: float = 1e-6,
    ) -> Transform:
        """Compute forward kinematics (iterative Newton-Raphson).

        Given 6 leg lengths, find the platform pose. This requires
        iterative solution as there's no closed-form FK for Stewart platforms.

        Args:
            leg_lengths: 6 leg lengths in meters.
            initial_guess: Starting pose for iteration (uses home if None).
            max_iterations: Maximum Newton-Raphson iterations.
            tolerance: Convergence tolerance.

        Returns:
            Platform pose as Transform.

        Raises:
            ValueError: If iteration doesn't converge.
        """
        if isinstance(leg_lengths, StewartJoints):
            leg_lengths = leg_lengths.leg_lengths
        else:
            leg_lengths = np.asarray(leg_lengths)

        # Initial guess
        if initial_guess is None:
            # Start at home position
            pose = np.array([0.0, 0.0, self.home_height, 0.0, 0.0, 0.0])
        else:
            euler = initial_guess.rotation.as_euler()
            pose = np.concatenate([initial_guess.position, euler])

        for _iteration in range(max_iterations):
            # Compute current leg lengths and error
            current_transform = Transform(
                position=pose[:3],
                rotation=Rotation.from_euler(tuple(pose[3:6])),
            )

            try:
                current_legs = self.inverse_array(current_transform)
            except ValueError:
                # Outside workspace, adjust and continue
                pose[2] = self.home_height
                continue

            error = leg_lengths - current_legs

            if np.max(np.abs(error)) < tolerance:
                return current_transform

            # Compute Jacobian and update
            jacobian = self._compute_jacobian(pose)

            # Solve J @ delta_pose = error
            try:
                delta_pose = np.linalg.lstsq(jacobian.T, error, rcond=None)[0]
            except np.linalg.LinAlgError:
                # Singular Jacobian, use damped version
                jjt = jacobian.T @ jacobian
                damping = 1e-6 * np.eye(6)
                delta_pose = np.linalg.solve(jjt + damping, jacobian.T @ error)

            # Update pose
            pose += delta_pose

            # Normalize angles to [-pi, pi]
            for i in range(3, 6):
                pose[i] = math.atan2(math.sin(pose[i]), math.cos(pose[i]))

        raise ValueError(f"Forward kinematics did not converge after {max_iterations} iterations")

    def _compute_jacobian(
        self,
        pose: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Compute the Jacobian matrix d(leg_lengths)/d(pose).

        Uses numerical differentiation.

        Returns:
            6x6 Jacobian matrix.
        """
        jacobian = np.zeros((6, 6))
        eps = 1e-8

        transform = Transform(
            position=pose[:3],
            rotation=Rotation.from_euler(tuple(pose[3:6])),
        )

        try:
            base_legs = self.inverse_array(transform)
        except ValueError:
            base_legs = np.ones(6) * self.home_height

        for i in range(6):
            pose_plus = pose.copy()
            pose_plus[i] += eps

            transform_plus = Transform(
                position=pose_plus[:3],
                rotation=Rotation.from_euler(tuple(pose_plus[3:6])),
            )

            try:
                legs_plus = self.inverse_array(transform_plus)
            except ValueError:
                legs_plus = base_legs

            jacobian[:, i] = (legs_plus - base_legs) / eps

        return jacobian

    def is_pose_valid(self, target: Transform) -> bool:
        """Check if a pose is achievable (all legs within limits).

        Args:
            target: Pose to check.

        Returns:
            True if all leg lengths are within limits.
        """
        try:
            self.inverse(target)
            return True
        except ValueError:
            return False

    def detect_singularity(
        self,
        target: Transform,
        threshold: float = 1e-6,
    ) -> StewartSingularityType:
        """Detect if pose is near a singularity.

        Stewart platform singularities occur when:
        - Determinant of Jacobian approaches zero
        - Platform can move infinitesimally without actuator motion

        Args:
            target: Pose to check.
            threshold: Singularity detection threshold.

        Returns:
            Type of singularity detected.
        """
        euler = target.rotation.as_euler()
        pose = np.concatenate([target.position, euler])

        jacobian = self._compute_jacobian(pose)

        try:
            det = np.linalg.det(jacobian)
        except np.linalg.LinAlgError:
            return StewartSingularityType.COMBINED

        if abs(det) < threshold:
            # Analyze which type of singularity
            # Check if translation or rotation submatrix is singular
            trans_jac = jacobian[:, :3]
            rot_jac = jacobian[:, 3:]

            trans_rank = np.linalg.matrix_rank(trans_jac)
            rot_rank = np.linalg.matrix_rank(rot_jac)

            if trans_rank < 3 and rot_rank < 3:
                return StewartSingularityType.COMBINED
            elif trans_rank < 3:
                return StewartSingularityType.TRANSLATION
            elif rot_rank < 3:
                return StewartSingularityType.ROTATION
            else:
                return StewartSingularityType.COMBINED

        return StewartSingularityType.NONE

    def home_pose(self) -> Transform:
        """Get the home pose (platform at home height, level)."""
        return Transform(
            position=np.array([0.0, 0.0, self.home_height]),
            rotation=Rotation.identity(),
        )

    def home_leg_lengths(self) -> StewartJoints:
        """Get leg lengths at home pose."""
        return self.inverse(self.home_pose())

    def jacobian(self, target: Transform) -> NDArray[np.float64]:
        """Compute the Jacobian at a given pose.

        Returns:
            6x6 Jacobian matrix relating pose velocity to leg velocities.
        """
        euler = target.rotation.as_euler()
        pose = np.concatenate([target.position, euler])
        return self._compute_jacobian(pose)


def create_stewart(
    base_radius: float,
    platform_radius: float,
    leg_length_range: tuple[float, float],
    home_height: float | None = None,
) -> StewartPlatform:
    """Create a symmetric Stewart platform.

    Args:
        base_radius: Radius of base platform.
        platform_radius: Radius of moving platform.
        leg_length_range: (min, max) actuator lengths.
        home_height: Height at neutral position.

    Returns:
        Configured StewartPlatform.

    Example:
        >>> stewart = create_stewart(0.25, 0.12, (0.3, 0.5))
        >>> print(f"Home height: {stewart.home_height:.2f}m")
    """
    return StewartPlatform.create_symmetric(
        base_radius=base_radius,
        platform_radius=platform_radius,
        leg_length_range=leg_length_range,
        home_height=home_height,
    )


def create_flight_simulator() -> StewartPlatform:
    """Create a flight simulator style Stewart platform.

    Approximate dimensions for a small motion platform:
    - Base radius: 0.8m
    - Platform radius: 0.4m
    - Leg length range: 0.8m to 1.4m

    Returns:
        StewartPlatform configured for motion simulation.
    """
    return create_stewart(
        base_radius=0.8,
        platform_radius=0.4,
        leg_length_range=(0.8, 1.4),
        home_height=0.9,
    )


def create_precision_positioner() -> StewartPlatform:
    """Create a precision positioning Stewart platform.

    Small, high-precision configuration:
    - Base radius: 100mm
    - Platform radius: 50mm
    - Leg length range: 120mm to 180mm

    Returns:
        StewartPlatform configured for precision positioning.
    """
    return create_stewart(
        base_radius=0.1,
        platform_radius=0.05,
        leg_length_range=(0.12, 0.18),
        home_height=0.14,
    )
