"""3D transformation utilities for robotics.

This module provides classes for representing and manipulating 3D transformations
using homogeneous transformation matrices, quaternions, and Euler angles.

Key classes:
- Transform: 4x4 homogeneous transformation matrix
- Rotation: 3D rotation representation (wrapper around scipy.spatial.transform)

Example:
    >>> from robo_infra.motion.transforms import Transform
    >>>
    >>> # Create from position and Euler angles
    >>> t1 = Transform.from_euler(position=(1.0, 2.0, 3.0), angles=(0, 0, 90), degrees=True)
    >>> print(f"Position: {t1.position}")
    Position: [1. 2. 3.]
    >>>
    >>> # Create from quaternion
    >>> t2 = Transform.from_quaternion(position=(0, 0, 0), quaternion=(0, 0, 0.707, 0.707))
    >>>
    >>> # Compose transformations
    >>> t3 = t1 @ t2
    >>>
    >>> # Get inverse
    >>> t4 = t1.inverse()
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np


if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray


class EulerOrder(Enum):
    """Euler angle rotation order.

    The naming convention follows the axes of rotation in order.
    For example, XYZ means rotate around X, then Y, then Z.

    Intrinsic rotations (rotating frame) vs Extrinsic (fixed frame):
    - Intrinsic XYZ is equivalent to Extrinsic ZYX
    - Use the lowercase version for scipy compatibility
    """

    XYZ = "xyz"  # Roll-Pitch-Yaw (common in aerospace)
    ZYX = "zyx"  # Yaw-Pitch-Roll (common in robotics)
    ZYZ = "zyz"  # Euler angles (common in robot arms)
    XZX = "xzx"
    YXY = "yxy"
    YZY = "yzy"
    ZXZ = "zxz"
    XYX = "xyx"
    XZY = "xzy"
    YXZ = "yxz"
    YZX = "yzx"
    ZXY = "zxy"


@dataclass(slots=True)
class Rotation:
    """3D rotation representation.

    Internally stores rotation as a 3x3 rotation matrix.
    Provides conversions to/from quaternions, Euler angles, and axis-angle.

    Attributes:
        matrix: 3x3 rotation matrix (SO(3)).

    Example:
        >>> rot = Rotation.from_euler(angles=(0, 0, 90), order=EulerOrder.XYZ, degrees=True)
        >>> print(rot.as_euler(degrees=True))
        [ 0.  0. 90.]
    """

    matrix: NDArray[np.float64] = field(default_factory=lambda: np.eye(3))

    def __post_init__(self) -> None:
        """Validate rotation matrix."""
        self.matrix = np.asarray(self.matrix, dtype=np.float64)
        if self.matrix.shape != (3, 3):
            raise ValueError(f"Rotation matrix must be 3x3, got {self.matrix.shape}")

    @classmethod
    def identity(cls) -> Rotation:
        """Create identity rotation (no rotation)."""
        return cls(matrix=np.eye(3))

    @classmethod
    def from_euler(
        cls,
        angles: tuple[float, float, float],
        order: EulerOrder = EulerOrder.XYZ,
        degrees: bool = False,
    ) -> Rotation:
        """Create rotation from Euler angles.

        Args:
            angles: Tuple of three rotation angles.
            order: Order of rotations (e.g., XYZ, ZYX).
            degrees: If True, angles are in degrees; otherwise radians.

        Returns:
            Rotation object.

        Example:
            >>> rot = Rotation.from_euler((0, 0, 90), EulerOrder.XYZ, degrees=True)
        """
        a1, a2, a3 = angles
        if degrees:
            a1, a2, a3 = math.radians(a1), math.radians(a2), math.radians(a3)

        # Build individual rotation matrices
        def rx(a: float) -> NDArray[np.float64]:
            c, s = math.cos(a), math.sin(a)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)

        def ry(a: float) -> NDArray[np.float64]:
            c, s = math.cos(a), math.sin(a)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)

        def rz(a: float) -> NDArray[np.float64]:
            c, s = math.cos(a), math.sin(a)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)

        rot_funcs = {"x": rx, "y": ry, "z": rz}

        # Apply rotations in order (intrinsic: right-to-left multiplication)
        order_str = order.value
        r1 = rot_funcs[order_str[0]](a1)
        r2 = rot_funcs[order_str[1]](a2)
        r3 = rot_funcs[order_str[2]](a3)

        # Intrinsic rotation: R = R1 @ R2 @ R3
        matrix = r1 @ r2 @ r3
        return cls(matrix=matrix)

    @classmethod
    def from_quaternion(
        cls,
        quaternion: tuple[float, float, float, float],
        scalar_first: bool = False,
    ) -> Rotation:
        """Create rotation from quaternion.

        Args:
            quaternion: Quaternion as (x, y, z, w) or (w, x, y, z) if scalar_first.
            scalar_first: If True, quaternion is (w, x, y, z); otherwise (x, y, z, w).

        Returns:
            Rotation object.

        Example:
            >>> rot = Rotation.from_quaternion((0, 0, 0.707, 0.707))  # 90° around Z
        """
        if scalar_first:
            w, x, y, z = quaternion
        else:
            x, y, z, w = quaternion

        # Normalize quaternion
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-10:
            raise ValueError("Quaternion has zero norm")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        # Convert to rotation matrix
        matrix = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

        return cls(matrix=matrix)

    @classmethod
    def from_axis_angle(
        cls,
        axis: tuple[float, float, float],
        angle: float,
        degrees: bool = False,
    ) -> Rotation:
        """Create rotation from axis and angle.

        Args:
            axis: Unit vector defining the rotation axis.
            angle: Rotation angle around the axis.
            degrees: If True, angle is in degrees; otherwise radians.

        Returns:
            Rotation object.

        Example:
            >>> rot = Rotation.from_axis_angle((0, 0, 1), 90, degrees=True)  # 90° around Z
        """
        if degrees:
            angle = math.radians(angle)

        # Normalize axis
        ax, ay, az = axis
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-10:
            raise ValueError("Axis has zero length")
        ax, ay, az = ax / norm, ay / norm, az / norm

        # Rodrigues' rotation formula
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1 - c

        matrix = np.array(
            [
                [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay],
                [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax],
                [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c],
            ],
            dtype=np.float64,
        )

        return cls(matrix=matrix)

    @classmethod
    def from_rotvec(cls, rotvec: tuple[float, float, float]) -> Rotation:
        """Create rotation from rotation vector.

        The rotation vector is axis * angle, where axis is a unit vector
        and angle is in radians.

        Args:
            rotvec: Rotation vector (rx, ry, rz).

        Returns:
            Rotation object.
        """
        rx, ry, rz = rotvec
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)

        if angle < 1e-10:
            return cls.identity()

        axis = (rx / angle, ry / angle, rz / angle)
        return cls.from_axis_angle(axis, angle, degrees=False)

    def as_euler(
        self,
        order: EulerOrder = EulerOrder.XYZ,
        degrees: bool = False,
    ) -> NDArray[np.float64]:
        """Convert to Euler angles.

        Args:
            order: Order of rotations.
            degrees: If True, return angles in degrees.

        Returns:
            Array of three Euler angles.

        Note:
            May have gimbal lock issues near singularities.
        """
        r = self.matrix
        order_str = order.value

        # Handle common cases
        if order_str == "xyz":
            # Roll (X), Pitch (Y), Yaw (Z)
            sy = math.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)
            singular = sy < 1e-6

            if not singular:
                x = math.atan2(r[2, 1], r[2, 2])
                y = math.atan2(-r[2, 0], sy)
                z = math.atan2(r[1, 0], r[0, 0])
            else:
                x = math.atan2(-r[1, 2], r[1, 1])
                y = math.atan2(-r[2, 0], sy)
                z = 0

            angles = np.array([x, y, z])

        elif order_str == "zyx":
            # Yaw (Z), Pitch (Y), Roll (X)
            sy = math.sqrt(r[0, 0] ** 2 + r[0, 1] ** 2)
            singular = sy < 1e-6

            if not singular:
                z = math.atan2(r[1, 0], r[0, 0])
                y = math.atan2(-r[2, 0], sy)
                x = math.atan2(r[2, 1], r[2, 2])
            else:
                z = math.atan2(-r[0, 1], r[1, 1])
                y = math.atan2(-r[2, 0], sy)
                x = 0

            angles = np.array([z, y, x])

        elif order_str == "zyz":
            # ZYZ Euler angles (common for robot arms)
            beta = math.atan2(math.sqrt(r[2, 0] ** 2 + r[2, 1] ** 2), r[2, 2])
            singular = abs(math.sin(beta)) < 1e-6

            if not singular:
                alpha = math.atan2(r[1, 2], r[0, 2])
                gamma = math.atan2(r[2, 1], -r[2, 0])
            else:
                alpha = 0
                gamma = (
                    math.atan2(-r[0, 1], r[1, 1]) if r[2, 2] > 0 else -math.atan2(-r[0, 1], r[1, 1])
                )

            angles = np.array([alpha, beta, gamma])

        else:
            # General case - use iterative solution
            # For now, fallback to XYZ
            return self.as_euler(EulerOrder.XYZ, degrees)

        if degrees:
            angles = np.degrees(angles)

        return angles

    def as_quaternion(self, scalar_first: bool = False) -> NDArray[np.float64]:
        """Convert to quaternion.

        Args:
            scalar_first: If True, return (w, x, y, z); otherwise (x, y, z, w).

        Returns:
            Quaternion as numpy array.
        """
        r = self.matrix
        trace = np.trace(r)

        if trace > 0:
            s = 2.0 * math.sqrt(trace + 1.0)
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s

        if scalar_first:
            return np.array([w, x, y, z], dtype=np.float64)
        else:
            return np.array([x, y, z, w], dtype=np.float64)

    def as_axis_angle(self, degrees: bool = False) -> tuple[NDArray[np.float64], float]:
        """Convert to axis-angle representation.

        Args:
            degrees: If True, return angle in degrees.

        Returns:
            Tuple of (axis, angle) where axis is a unit vector.
        """
        # Use quaternion as intermediate
        quat = self.as_quaternion(scalar_first=True)
        w, x, y, z = quat

        angle = 2.0 * math.acos(np.clip(w, -1.0, 1.0))

        s = math.sqrt(1.0 - w * w)
        if s < 1e-10:
            # No rotation, axis is arbitrary
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = np.array([x / s, y / s, z / s], dtype=np.float64)

        if degrees:
            angle = math.degrees(angle)

        return axis, angle

    def as_rotvec(self) -> NDArray[np.float64]:
        """Convert to rotation vector.

        Returns:
            Rotation vector (axis * angle).
        """
        axis, angle = self.as_axis_angle(degrees=False)
        return axis * angle

    def inverse(self) -> Rotation:
        """Return the inverse rotation.

        For rotation matrices, the inverse is the transpose.
        """
        return Rotation(matrix=self.matrix.T)

    def __matmul__(self, other: Rotation) -> Rotation:
        """Compose two rotations.

        Args:
            other: Another rotation to compose with.

        Returns:
            Combined rotation (self @ other).
        """
        return Rotation(matrix=self.matrix @ other.matrix)

    def apply(self, vectors: ArrayLike) -> NDArray[np.float64]:
        """Apply rotation to vector(s).

        Args:
            vectors: Single vector (3,) or array of vectors (N, 3).

        Returns:
            Rotated vector(s).
        """
        v = np.asarray(vectors, dtype=np.float64)
        if v.ndim == 1:
            return self.matrix @ v
        else:
            return (self.matrix @ v.T).T

    def __repr__(self) -> str:
        """String representation."""
        euler = self.as_euler(degrees=True)
        return f"Rotation(euler_xyz=[{euler[0]:.1f}°, {euler[1]:.1f}°, {euler[2]:.1f}°])"


@dataclass(slots=True, eq=False)
class Transform:
    """4x4 homogeneous transformation matrix.

    Represents a rigid body transformation in 3D space, combining
    rotation and translation.

    Attributes:
        position: Translation vector [x, y, z].
        rotation: Rotation object.

    Example:
        >>> t = Transform.from_euler(position=(1, 2, 3), angles=(0, 0, 90), degrees=True)
        >>> print(t.position)
        [1. 2. 3.]
        >>> t_inv = t.inverse()
        >>> identity = t @ t_inv
    """

    position: NDArray[np.float64] = field(default_factory=lambda: np.zeros(3))
    rotation: Rotation = field(default_factory=Rotation.identity)

    def __post_init__(self) -> None:
        """Validate transform."""
        self.position = np.asarray(self.position, dtype=np.float64)
        if self.position.shape != (3,):
            raise ValueError(f"Position must be (3,), got {self.position.shape}")

    @classmethod
    def identity(cls) -> Transform:
        """Create identity transform (no rotation or translation)."""
        return cls(position=np.zeros(3), rotation=Rotation.identity())

    @classmethod
    def from_matrix(cls, matrix: ArrayLike) -> Transform:
        """Create transform from 4x4 homogeneous matrix.

        Args:
            matrix: 4x4 homogeneous transformation matrix.

        Returns:
            Transform object.
        """
        m = np.asarray(matrix, dtype=np.float64)
        if m.shape != (4, 4):
            raise ValueError(f"Matrix must be 4x4, got {m.shape}")

        position = m[:3, 3]
        rotation = Rotation(matrix=m[:3, :3])
        return cls(position=position, rotation=rotation)

    @classmethod
    def from_euler(
        cls,
        position: tuple[float, float, float],
        angles: tuple[float, float, float],
        order: EulerOrder = EulerOrder.XYZ,
        degrees: bool = False,
    ) -> Transform:
        """Create transform from position and Euler angles.

        Args:
            position: Translation (x, y, z).
            angles: Euler angles (a1, a2, a3).
            order: Order of rotations.
            degrees: If True, angles are in degrees.

        Returns:
            Transform object.
        """
        pos = np.array(position, dtype=np.float64)
        rot = Rotation.from_euler(angles, order, degrees)
        return cls(position=pos, rotation=rot)

    @classmethod
    def from_quaternion(
        cls,
        position: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
        scalar_first: bool = False,
    ) -> Transform:
        """Create transform from position and quaternion.

        Args:
            position: Translation (x, y, z).
            quaternion: Rotation as quaternion.
            scalar_first: If True, quaternion is (w, x, y, z).

        Returns:
            Transform object.
        """
        pos = np.array(position, dtype=np.float64)
        rot = Rotation.from_quaternion(quaternion, scalar_first)
        return cls(position=pos, rotation=rot)

    @classmethod
    def from_axis_angle(
        cls,
        position: tuple[float, float, float],
        axis: tuple[float, float, float],
        angle: float,
        degrees: bool = False,
    ) -> Transform:
        """Create transform from position and axis-angle rotation.

        Args:
            position: Translation (x, y, z).
            axis: Rotation axis (unit vector).
            angle: Rotation angle.
            degrees: If True, angle is in degrees.

        Returns:
            Transform object.
        """
        pos = np.array(position, dtype=np.float64)
        rot = Rotation.from_axis_angle(axis, angle, degrees)
        return cls(position=pos, rotation=rot)

    @classmethod
    def from_translation(cls, x: float, y: float, z: float) -> Transform:
        """Create a pure translation transform.

        Args:
            x, y, z: Translation components.

        Returns:
            Transform with no rotation.
        """
        return cls(position=np.array([x, y, z]), rotation=Rotation.identity())

    @classmethod
    def from_rotation(cls, rotation: Rotation) -> Transform:
        """Create a pure rotation transform.

        Args:
            rotation: Rotation object.

        Returns:
            Transform with no translation.
        """
        return cls(position=np.zeros(3), rotation=rotation)

    @property
    def x(self) -> float:
        """X position component."""
        return float(self.position[0])

    @property
    def y(self) -> float:
        """Y position component."""
        return float(self.position[1])

    @property
    def z(self) -> float:
        """Z position component."""
        return float(self.position[2])

    def as_matrix(self) -> NDArray[np.float64]:
        """Convert to 4x4 homogeneous matrix.

        Returns:
            4x4 transformation matrix.
        """
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self.rotation.matrix
        matrix[:3, 3] = self.position
        return matrix

    def inverse(self) -> Transform:
        """Return the inverse transform.

        For T = [R, p], T^-1 = [R^T, -R^T @ p]
        """
        r_inv = self.rotation.inverse()
        p_inv = -r_inv.apply(self.position)
        return Transform(position=p_inv, rotation=r_inv)

    def __matmul__(self, other: Transform) -> Transform:
        """Compose two transforms.

        For T1 = [R1, p1] and T2 = [R2, p2]:
        T1 @ T2 = [R1 @ R2, R1 @ p2 + p1]

        Args:
            other: Another transform to compose with.

        Returns:
            Combined transform (self @ other).
        """
        new_rotation = self.rotation @ other.rotation
        new_position = self.rotation.apply(other.position) + self.position
        return Transform(position=new_position, rotation=new_rotation)

    def apply(self, points: ArrayLike) -> NDArray[np.float64]:
        """Apply transform to point(s).

        Args:
            points: Single point (3,) or array of points (N, 3).

        Returns:
            Transformed point(s).
        """
        p = np.asarray(points, dtype=np.float64)
        if p.ndim == 1:
            return self.rotation.apply(p) + self.position
        else:
            return self.rotation.apply(p) + self.position

    def distance_to(self, other: Transform) -> float:
        """Euclidean distance between positions.

        Args:
            other: Another transform.

        Returns:
            Distance between positions.
        """
        return float(np.linalg.norm(self.position - other.position))

    def angular_distance_to(self, other: Transform, degrees: bool = False) -> float:
        """Angular distance between rotations.

        Args:
            other: Another transform.
            degrees: If True, return angle in degrees.

        Returns:
            Angle between rotations.
        """
        # Relative rotation
        r_rel = self.rotation.inverse() @ other.rotation
        _, angle = r_rel.as_axis_angle(degrees=degrees)
        return angle

    def interpolate(self, other: Transform, t: float) -> Transform:
        """Linear interpolation between transforms.

        Uses linear interpolation for position and SLERP for rotation.

        Args:
            other: Target transform.
            t: Interpolation parameter [0, 1].

        Returns:
            Interpolated transform.
        """
        t = np.clip(t, 0.0, 1.0)

        # Linear interpolation for position
        pos = (1 - t) * self.position + t * other.position

        # SLERP for rotation
        q1 = self.rotation.as_quaternion(scalar_first=True)
        q2 = other.rotation.as_quaternion(scalar_first=True)

        # Ensure shortest path
        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot

        if dot > 0.9995:
            # Linear interpolation for very close quaternions
            q = q1 + t * (q2 - q1)
            q = q / np.linalg.norm(q)
        else:
            theta = math.acos(np.clip(dot, -1.0, 1.0))
            sin_theta = math.sin(theta)
            q = (math.sin((1 - t) * theta) * q1 + math.sin(t * theta) * q2) / sin_theta

        rot = Rotation.from_quaternion((*tuple(q[1:]), q[0]), scalar_first=False)
        return Transform(position=pos, rotation=rot)

    def __repr__(self) -> str:
        """String representation."""
        euler = self.rotation.as_euler(degrees=True)
        return (
            f"Transform(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"rot=[{euler[0]:.1f}°, {euler[1]:.1f}°, {euler[2]:.1f}°])"
        )

    def __eq__(self, other: object) -> bool:
        """Check equality with tolerance."""
        if not isinstance(other, Transform):
            return NotImplemented
        pos_eq = np.allclose(self.position, other.position, atol=1e-6)
        rot_eq = np.allclose(self.rotation.matrix, other.rotation.matrix, atol=1e-6)
        return pos_eq and rot_eq


def translation(x: float, y: float, z: float) -> Transform:
    """Create a pure translation transform.

    Convenience function for creating translation-only transforms.

    Args:
        x, y, z: Translation components.

    Returns:
        Transform with specified translation, no rotation.
    """
    return Transform.from_translation(x, y, z)


def rotation_x(angle: float, degrees: bool = False) -> Rotation:
    """Create rotation around X axis.

    Args:
        angle: Rotation angle.
        degrees: If True, angle is in degrees.

    Returns:
        Rotation around X axis.
    """
    return Rotation.from_axis_angle((1, 0, 0), angle, degrees)


def rotation_y(angle: float, degrees: bool = False) -> Rotation:
    """Create rotation around Y axis.

    Args:
        angle: Rotation angle.
        degrees: If True, angle is in degrees.

    Returns:
        Rotation around Y axis.
    """
    return Rotation.from_axis_angle((0, 1, 0), angle, degrees)


def rotation_z(angle: float, degrees: bool = False) -> Rotation:
    """Create rotation around Z axis.

    Args:
        angle: Rotation angle.
        degrees: If True, angle is in degrees.

    Returns:
        Rotation around Z axis.
    """
    return Rotation.from_axis_angle((0, 0, 1), angle, degrees)
