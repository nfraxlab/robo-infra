"""Delta (parallel) robot kinematics.

Delta robots are 3-DOF parallel manipulators commonly used in high-speed
pick-and-place applications. Three arms connect a fixed upper platform
to a moving lower platform (effector).

Architecture:
- 3 rotary actuators on the fixed base (upper platform)
- 3 parallelogram arm assemblies
- Moving platform maintains orientation (always horizontal)

Key Features:
- Very high speed and acceleration
- Low moving mass (motors are fixed)
- Precise positioning
- Limited workspace (dome-shaped)
- Parallel architecture provides high stiffness

Example:
    >>> from robo_infra.motion.delta import DeltaRobot
    >>>
    >>> # Create a delta robot
    >>> delta = DeltaRobot(
    ...     base_radius=0.2,
    ...     effector_radius=0.05,
    ...     upper_arm_length=0.2,
    ...     lower_arm_length=0.4,
    ... )
    >>>
    >>> # Forward kinematics
    >>> x, y, z = delta.forward(0.0, 0.0, 0.0)
    >>> print(f"Effector at: ({x:.3f}, {y:.3f}, {z:.3f})")
    >>>
    >>> # Inverse kinematics
    >>> angles = delta.inverse(0.0, 0.0, -0.3)
    >>> print(f"Motor angles: {angles}")
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


class DeltaSingularityType(Enum):
    """Types of singularities in delta robots."""

    NONE = "none"
    WORKSPACE_BOUNDARY = "workspace_boundary"
    ARM_EXTENSION = "arm_extension"
    ARM_RETRACTION = "arm_retraction"


@dataclass
class DeltaLimits:
    """Joint and workspace limits for delta robot.

    Attributes:
        theta_range: Motor angle limits (min, max) in radians.
        z_min: Minimum Z height (most negative, fully extended).
        z_max: Maximum Z height (least negative, fully retracted).
    """

    theta_range: tuple[float, float] = (-math.pi / 2, math.pi / 2)  # ±90°
    z_min: float = -0.5  # Fully extended down
    z_max: float = -0.05  # Retracted up

    def is_valid_angle(self, theta: float) -> bool:
        """Check if motor angle is within limits."""
        return self.theta_range[0] <= theta <= self.theta_range[1]


@dataclass
class DeltaJoints:
    """Joint values for a delta robot.

    Attributes:
        theta1: Motor 1 angle in radians.
        theta2: Motor 2 angle in radians.
        theta3: Motor 3 angle in radians.
    """

    theta1: float
    theta2: float
    theta3: float

    def as_array(self) -> NDArray[np.float64]:
        """Return joints as numpy array."""
        return np.array([self.theta1, self.theta2, self.theta3])

    def as_tuple(self) -> tuple[float, float, float]:
        """Return joints as tuple."""
        return (self.theta1, self.theta2, self.theta3)


@dataclass
class DeltaWorkspace:
    """Workspace analysis for delta robot.

    Attributes:
        x_range: (min, max) X coordinates.
        y_range: (min, max) Y coordinates.
        z_range: (min, max) Z coordinates.
        volume: Approximate workspace volume in m³.
    """

    x_range: tuple[float, float]
    y_range: tuple[float, float]
    z_range: tuple[float, float]
    volume: float


@dataclass
class DeltaRobot:
    """3-DOF parallel delta robot (like Kossel 3D printers).

    Three arms connect a fixed platform to a moving platform.
    The arms are arranged at 120° intervals around the vertical axis.

    Geometry:
    - Base platform has motors at radius `base_radius` from center
    - Effector platform has arm connections at radius `effector_radius`
    - Upper arms (connected to motors) have length `upper_arm_length`
    - Lower arms (parallelogram) have length `lower_arm_length`

    Coordinate System:
    - Origin at center of base platform
    - Z axis points down (effector is at negative Z)
    - Motor 1 is along the X axis (0°)
    - Motor 2 is at 120°
    - Motor 3 is at 240°

    Attributes:
        base_radius: Distance from center to motor attachment points.
        effector_radius: Distance from center to effector arm connections.
        upper_arm_length: Length of upper arm (motor to elbow).
        lower_arm_length: Length of lower arm (elbow to effector).
        limits: Joint and workspace limits.

    Example:
        >>> delta = DeltaRobot(0.2, 0.05, 0.15, 0.35)
        >>> x, y, z = delta.forward(0, 0, 0)
        >>> print(f"Home position: z = {z:.3f}")
    """

    base_radius: float  # F - base platform radius
    effector_radius: float  # E - effector platform radius
    upper_arm_length: float  # rf - upper arm (bicep) length
    lower_arm_length: float  # re - lower arm (forearm) length
    limits: DeltaLimits = field(default_factory=DeltaLimits)

    # Pre-computed values
    _sqrt3: float = field(init=False, default=math.sqrt(3))
    _sin120: float = field(init=False, default=math.sqrt(3) / 2)
    _cos120: float = field(init=False, default=-0.5)
    _tan60: float = field(init=False, default=math.sqrt(3))
    _tan30: float = field(init=False, default=1 / math.sqrt(3))

    def __post_init__(self) -> None:
        """Validate dimensions."""
        if self.base_radius <= 0:
            raise ValueError(f"base_radius must be positive, got {self.base_radius}")
        if self.effector_radius <= 0:
            raise ValueError(f"effector_radius must be positive, got {self.effector_radius}")
        if self.upper_arm_length <= 0:
            raise ValueError(f"upper_arm_length must be positive, got {self.upper_arm_length}")
        if self.lower_arm_length <= 0:
            raise ValueError(f"lower_arm_length must be positive, got {self.lower_arm_length}")
        if self.effector_radius >= self.base_radius:
            raise ValueError("effector_radius must be smaller than base_radius")

    @property
    def arm_offset(self) -> float:
        """Effective offset between base and effector radii."""
        return self.base_radius - self.effector_radius

    def forward(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
    ) -> tuple[float, float, float]:
        """Compute forward kinematics.

        Given three motor angles, compute the (x, y, z) position of the effector.

        Args:
            theta1: Motor 1 angle in radians (at 0°).
            theta2: Motor 2 angle in radians (at 120°).
            theta3: Motor 3 angle in radians (at 240°).

        Returns:
            Tuple (x, y, z) of effector position in meters.

        Raises:
            ValueError: If the configuration is invalid (no solution).

        Example:
            >>> delta = DeltaRobot(0.15, 0.03, 0.10, 0.30)
            >>> x, y, z = delta.forward(0, 0, 0)
            >>> print(f"Position: ({x:.4f}, {y:.4f}, {z:.4f})")
        """
        # Compute elbow positions for each arm
        # Using the trilateration approach from standard Delta robot kinematics
        # Reference: https://hypertriangle.com/~alex/delta-robot-tutorial/

        t = self.arm_offset  # Effective horizontal offset
        rf = self.upper_arm_length
        re = self.lower_arm_length

        # Motor angles in base plane (measured from -Y, counterclockwise)
        # Arm 1 at 0°, Arm 2 at 120°, Arm 3 at 240°

        # For each arm, the elbow position is:
        # horizontal_distance = t + rf * cos(theta)
        # vertical_distance = -rf * sin(theta)
        # direction = (sin(motor_angle), -cos(motor_angle))

        # Arm 1 (at 0° from -Y, i.e. along -Y direction)
        h1 = t + rf * math.cos(theta1)
        y1 = -h1  # -cos(0) = -1, so y1 = -h1
        z1 = -rf * math.sin(theta1)

        # Arm 2 (at 120° from -Y)
        h2 = t + rf * math.cos(theta2)
        x2 = h2 * self._sin120  # sin(120°) = sqrt(3)/2
        y2 = h2 * 0.5  # -cos(120°) = -(-0.5) = 0.5
        z2 = -rf * math.sin(theta2)

        # Arm 3 (at 240° from -Y)
        h3 = t + rf * math.cos(theta3)
        x3 = -h3 * self._sin120  # sin(240°) = -sqrt(3)/2
        y3 = h3 * 0.5  # -cos(240°) = -(-0.5) = 0.5
        z3 = -rf * math.sin(theta3)

        # Trilateration to find intersection of 3 spheres
        # Sphere centers at elbow positions, radius = lower_arm_length
        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        if abs(dnm) < 1e-10:
            raise ValueError("Singular configuration - arms are coplanar")

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        # x = (a1*z + b1) / dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        # y = (a2*z + b2) / dnm
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # Substitute into sphere 1 equation:
        # (a1*z+b1)²/dnm² + ((a2*z+b2)/dnm - y1)² + (z-z1)² = re²

        # This gives: a*z² + b*z + c = 0
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = b1 * b1 + (b2 - y1 * dnm) * (b2 - y1 * dnm) + z1 * z1 * dnm * dnm - re * re * dnm * dnm

        # Discriminant
        d = b * b - 4.0 * a * c

        if d < 0:
            raise ValueError(
                f"No solution exists for angles ({math.degrees(theta1):.1f}°, "
                f"{math.degrees(theta2):.1f}°, {math.degrees(theta3):.1f}°)"
            )

        # We want the more negative z (lower solution, effector below base)
        z = (-b - math.sqrt(d)) / (2.0 * a)
        x = (a1 * z + b1) / dnm
        y = (a2 * z + b2) / dnm

        return (x, y, z)

    def forward_transform(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
    ) -> Transform:
        """Compute forward kinematics returning a Transform.

        The effector maintains constant orientation (always horizontal).

        Args:
            theta1, theta2, theta3: Motor angles in radians.

        Returns:
            Transform with position and identity rotation.
        """
        x, y, z = self.forward(theta1, theta2, theta3)
        return Transform(
            position=np.array([x, y, z]),
            rotation=Rotation.identity(),
        )

    def inverse(
        self,
        x: float,
        y: float,
        z: float,
    ) -> tuple[float, float, float] | None:
        """Compute inverse kinematics.

        Given an effector position, compute the three motor angles.

        Args:
            x: X coordinate of effector.
            y: Y coordinate of effector.
            z: Z coordinate of effector (should be negative).

        Returns:
            Tuple (theta1, theta2, theta3) in radians, or None if unreachable.

        Example:
            >>> delta = DeltaRobot(0.2, 0.05, 0.15, 0.35)
            >>> angles = delta.inverse(0.0, 0.0, -0.3)
            >>> if angles:
            ...     print(f"Theta1: {math.degrees(angles[0]):.1f}°")
        """
        # Solve for each arm independently
        theta1 = self._inverse_single(x, y, z, 0)
        if theta1 is None:
            return None

        theta2 = self._inverse_single(x, y, z, 120)
        if theta2 is None:
            return None

        theta3 = self._inverse_single(x, y, z, 240)
        if theta3 is None:
            return None

        return (theta1, theta2, theta3)

    def _inverse_single(
        self,
        x0: float,
        y0: float,
        z0: float,
        angle_deg: float,
    ) -> float | None:
        """Calculate inverse kinematics for a single arm.

        Args:
            x0, y0, z0: Effector position.
            angle_deg: Angle of this arm (0, 120, or 240 degrees).

        Returns:
            Motor angle in radians, or None if no solution.
        """
        # Rotate the effector position to align with arm 1 (at 0°)
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # Rotate (x0, y0) by -angle to align with arm 1
        x = x0 * cos_a + y0 * sin_a
        y = -x0 * sin_a + y0 * cos_a
        z = z0

        # Effective offset
        t = self.arm_offset

        # Now solve for theta (same as arm 1 geometry)
        # Elbow joint is at (0, -(t + rf*cos(theta)), -rf*sin(theta))
        # Effector joint is at (x, y, z)
        # Distance between them should be lower_arm_length

        # Using substitution to get quadratic in tan(theta/2)
        # Let s = sin(theta), c = cos(theta)
        # (y + t + rf*c)² + x² + (z + rf*s)² = re²

        # Expand:
        # y² + 2y(t+rf*c) + (t+rf*c)² + x² + z² + 2z*rf*s + rf²*s² = re²
        # y² + 2yt + 2y*rf*c + t² + 2t*rf*c + rf²c² + x² + z² + 2z*rf*s + rf²s² = re²
        # Using c²+s²=1:
        # y² + 2yt + t² + rf² + 2(y+t)*rf*c + 2z*rf*s + x² + z² = re²

        # Rearrange: A*c + B*s + C = 0
        # where:
        y_eff = y + t

        rf = self.upper_arm_length
        re = self.lower_arm_length

        a = 2.0 * rf * y_eff
        b = 2.0 * rf * z
        c = x * x + y_eff * y_eff + z * z + rf * rf - re * re

        # Solve A*cos(theta) + B*sin(theta) + C = 0
        # Using substitution: t = tan(theta/2)
        # cos(theta) = (1-t²)/(1+t²), sin(theta) = 2t/(1+t²)
        # A*(1-t²) + B*2t + C*(1+t²) = 0
        # (C-A)*t² + 2B*t + (C+A) = 0

        d = b * b - c * c + a * a

        if d < 0:
            return None  # No solution for this arm

        sqrt_d = math.sqrt(d)

        # Two solutions, pick the appropriate one
        # theta = 2*atan2(-b ± sqrt(d), c-a)
        raw_theta1 = 2.0 * math.atan2(-b + sqrt_d, c - a)
        raw_theta2 = 2.0 * math.atan2(-b - sqrt_d, c - a)

        # Normalize to [-π, π]
        def normalize(theta: float) -> float:
            while theta > math.pi:
                theta -= 2 * math.pi
            while theta < -math.pi:
                theta += 2 * math.pi
            return theta

        theta1 = normalize(raw_theta1)
        theta2 = normalize(raw_theta2)

        # Prefer the solution within limits
        if self.limits.is_valid_angle(theta1):
            return theta1
        elif self.limits.is_valid_angle(theta2):
            return theta2
        else:
            # Both outside limits
            return None

    def inverse_joints(self, x: float, y: float, z: float) -> DeltaJoints | None:
        """Compute inverse kinematics returning DeltaJoints."""
        result = self.inverse(x, y, z)
        if result is None:
            return None
        return DeltaJoints(theta1=result[0], theta2=result[1], theta3=result[2])

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """Check if a point is within the workspace.

        Args:
            x, y, z: Point to check.

        Returns:
            True if the point is reachable.
        """
        return self.inverse(x, y, z) is not None

    def detect_singularity(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
    ) -> DeltaSingularityType:
        """Detect if the configuration is near a singularity.

        Delta robots have singularities at:
        - Workspace boundary (arms fully extended)
        - Center (arms symmetrically arranged)

        Args:
            theta1, theta2, theta3: Motor angles in radians.

        Returns:
            Type of singularity (NONE if not singular).
        """
        # Check for arm extension singularity
        tolerance = math.radians(5)  # 5 degree tolerance

        for theta in [theta1, theta2, theta3]:
            # Near horizontal (fully extended down)
            if abs(theta - math.pi / 2) < tolerance:
                return DeltaSingularityType.ARM_EXTENSION
            # Near negative horizontal (fully extended up)
            if abs(theta + math.pi / 2) < tolerance:
                return DeltaSingularityType.ARM_RETRACTION

        # Check for workspace boundary
        try:
            x, y, _z = self.forward(theta1, theta2, theta3)
            r = math.sqrt(x * x + y * y)
            max_reach = self.upper_arm_length + self.lower_arm_length
            if r > 0.95 * max_reach:
                return DeltaSingularityType.WORKSPACE_BOUNDARY
        except ValueError:
            return DeltaSingularityType.WORKSPACE_BOUNDARY

        return DeltaSingularityType.NONE

    def jacobian(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
    ) -> NDArray[np.float64]:
        """Compute the Jacobian matrix numerically.

        The Jacobian relates motor velocities to effector velocities:
        [ẋ, ẏ, ż]ᵀ = J · [θ̇1, θ̇2, θ̇3]ᵀ

        Args:
            theta1, theta2, theta3: Motor angles in radians.

        Returns:
            3x3 Jacobian matrix.
        """
        eps = 1e-8
        x0, y0, z0 = self.forward(theta1, theta2, theta3)

        jacobian = np.zeros((3, 3))

        for i, (t1, t2, t3) in enumerate(
            [
                (theta1 + eps, theta2, theta3),
                (theta1, theta2 + eps, theta3),
                (theta1, theta2, theta3 + eps),
            ]
        ):
            x, y, z = self.forward(t1, t2, t3)
            jacobian[0, i] = (x - x0) / eps
            jacobian[1, i] = (y - y0) / eps
            jacobian[2, i] = (z - z0) / eps

        return jacobian

    def compute_workspace(
        self,
        resolution: int = 20,
    ) -> DeltaWorkspace:
        """Compute approximate workspace bounds.

        Samples the joint space to find reachable positions.

        Args:
            resolution: Number of samples per joint dimension.

        Returns:
            DeltaWorkspace with bounds and volume estimate.
        """
        x_min = y_min = z_min = float("inf")
        x_max = y_max = z_max = float("-inf")

        theta_min, theta_max = self.limits.theta_range

        valid_points = 0
        total_points = resolution**3

        for i in range(resolution):
            t1 = theta_min + (theta_max - theta_min) * i / (resolution - 1)
            for j in range(resolution):
                t2 = theta_min + (theta_max - theta_min) * j / (resolution - 1)
                for k in range(resolution):
                    t3 = theta_min + (theta_max - theta_min) * k / (resolution - 1)

                    try:
                        x, y, z = self.forward(t1, t2, t3)
                        valid_points += 1
                        x_min = min(x_min, x)
                        x_max = max(x_max, x)
                        y_min = min(y_min, y)
                        y_max = max(y_max, y)
                        z_min = min(z_min, z)
                        z_max = max(z_max, z)
                    except ValueError:
                        pass

        # Estimate volume (very rough)
        dx = x_max - x_min
        dy = y_max - y_min
        dz = z_max - z_min
        box_volume = dx * dy * dz
        fill_ratio = valid_points / total_points
        volume = box_volume * fill_ratio

        return DeltaWorkspace(
            x_range=(x_min, x_max),
            y_range=(y_min, y_max),
            z_range=(z_min, z_max),
            volume=volume,
        )


def create_delta(
    base_radius: float,
    effector_radius: float,
    upper_arm_length: float,
    lower_arm_length: float,
) -> DeltaRobot:
    """Create a delta robot with specified dimensions.

    Args:
        base_radius: Distance from center to motor attachment.
        effector_radius: Distance from center to effector arm connections.
        upper_arm_length: Length of upper arm (motor to elbow).
        lower_arm_length: Length of lower arm (elbow to effector).

    Returns:
        Configured DeltaRobot.

    Example:
        >>> delta = create_delta(0.15, 0.04, 0.12, 0.3)
        >>> angles = delta.inverse(0, 0, -0.25)
    """
    return DeltaRobot(
        base_radius=base_radius,
        effector_radius=effector_radius,
        upper_arm_length=upper_arm_length,
        lower_arm_length=lower_arm_length,
    )


def create_kossel_mini() -> DeltaRobot:
    """Create a Kossel Mini 3D printer style delta robot.

    Kossel Mini approximate dimensions (adjusted for valid FK):
    - Base radius: ~140mm
    - Effector radius: ~35mm
    - Upper arm (diagonal rod): ~120mm
    - Lower arm (carbon fiber rod): ~300mm

    Geometry constraint: base + upper_arm - effector < lower_arm
    140 + 120 - 35 = 225 < 300 ✓

    Returns:
        DeltaRobot configured like Kossel Mini.
    """
    return create_delta(
        base_radius=0.14,
        effector_radius=0.035,
        upper_arm_length=0.12,
        lower_arm_length=0.30,
    )


def create_flsun_q5() -> DeltaRobot:
    """Create an FLSUN Q5 style delta robot.

    FLSUN Q5 approximate dimensions (adjusted for valid FK):
    - Base radius: ~100mm
    - Effector radius: ~35mm
    - Upper arm: ~100mm
    - Lower arm: ~215mm

    Geometry constraint: base + upper_arm - effector < lower_arm
    100 + 100 - 35 = 165 < 215 ✓

    Returns:
        DeltaRobot configured like FLSUN Q5.
    """
    return create_delta(
        base_radius=0.10,
        effector_radius=0.035,
        upper_arm_length=0.10,
        lower_arm_length=0.215,
    )
