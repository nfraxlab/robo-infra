"""Sensor fusion algorithms for orientation estimation.

This module provides implementations of popular sensor fusion algorithms:
- Madgwick filter: Gradient descent-based AHRS algorithm
- Mahony filter: Complementary filter-based AHRS algorithm

Both algorithms fuse accelerometer, gyroscope, and (optionally) magnetometer
data to estimate orientation as a quaternion.

Key Features:
- Real-time orientation estimation from IMU data
- Works with 6-DOF (accel + gyro) or 9-DOF (accel + gyro + mag) sensors
- Configurable gains for tuning response
- Outputs quaternion or Euler angles

References:
- Madgwick, S.O.H., "An efficient orientation filter for inertial and
  inertial/magnetic sensor arrays", 2010
- Mahony, R., Hamel, T., Pflimlin, J.M., "Nonlinear complementary filters
  on the special orthogonal group", 2008
"""

from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

from pydantic import BaseModel, Field

from robo_infra.core.types import Quaternion, Vector3


# =============================================================================
# Constants
# =============================================================================

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class Orientation:
    """Orientation estimation result.

    Attributes:
        quaternion: Orientation as unit quaternion.
        euler: Euler angles (roll, pitch, yaw) in degrees.
        timestamp: Time of estimate.
        dt: Time delta since last update.
    """

    quaternion: Quaternion
    euler: Vector3  # roll, pitch, yaw in degrees
    timestamp: float = field(default_factory=time.time)
    dt: float = 0.0

    @classmethod
    def from_quaternion(cls, q: Quaternion, dt: float = 0.0) -> Orientation:
        """Create Orientation from quaternion.

        Args:
            q: Unit quaternion.
            dt: Time delta.

        Returns:
            Orientation with euler angles computed.
        """
        euler = quaternion_to_euler(q)
        return cls(quaternion=q, euler=euler, dt=dt)


# =============================================================================
# Quaternion Utilities
# =============================================================================


def quaternion_normalize(q: Quaternion) -> Quaternion:
    """Normalize a quaternion to unit length.

    Args:
        q: Input quaternion.

    Returns:
        Normalized unit quaternion.
    """
    norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
    if norm < 1e-10:
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    return Quaternion(w=q.w / norm, x=q.x / norm, y=q.y / norm, z=q.z / norm)


def quaternion_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """Multiply two quaternions.

    Args:
        q1: First quaternion.
        q2: Second quaternion.

    Returns:
        Product quaternion q1 * q2.
    """
    return Quaternion(
        w=q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        x=q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        y=q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        z=q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
    )


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    """Compute quaternion conjugate.

    Args:
        q: Input quaternion.

    Returns:
        Conjugate quaternion.
    """
    return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)


def quaternion_to_euler(q: Quaternion) -> Vector3:
    """Convert quaternion to Euler angles (roll, pitch, yaw).

    Args:
        q: Unit quaternion.

    Returns:
        Vector3 with (roll, pitch, yaw) in degrees.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x**2 + q.y**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Gimbal lock
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return Vector3(
        x=roll * RAD_TO_DEG,
        y=pitch * RAD_TO_DEG,
        z=yaw * RAD_TO_DEG,
    )


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion.

    Args:
        roll: Roll angle in degrees.
        pitch: Pitch angle in degrees.
        yaw: Yaw angle in degrees.

    Returns:
        Unit quaternion.
    """
    roll_rad = roll * DEG_TO_RAD
    pitch_rad = pitch * DEG_TO_RAD
    yaw_rad = yaw * DEG_TO_RAD

    cr = math.cos(roll_rad / 2)
    sr = math.sin(roll_rad / 2)
    cp = math.cos(pitch_rad / 2)
    sp = math.sin(pitch_rad / 2)
    cy = math.cos(yaw_rad / 2)
    sy = math.sin(yaw_rad / 2)

    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


# =============================================================================
# Base Filter Class
# =============================================================================


class OrientationFilter(ABC):
    """Base class for orientation estimation filters.

    Provides common interface for sensor fusion algorithms.
    """

    def __init__(self, sample_period: float = 0.01) -> None:
        """Initialize filter.

        Args:
            sample_period: Expected time between updates in seconds.
        """
        self._sample_period = sample_period
        self._quaternion = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self._last_update = time.time()

    @property
    def quaternion(self) -> Quaternion:
        """Get current orientation quaternion."""
        return self._quaternion

    @property
    def euler(self) -> Vector3:
        """Get current orientation as Euler angles."""
        return quaternion_to_euler(self._quaternion)

    @abstractmethod
    def update(
        self,
        gyro: Vector3,
        accel: Vector3,
        mag: Vector3 | None = None,
        dt: float | None = None,
    ) -> Orientation:
        """Update orientation estimate with new sensor data.

        Args:
            gyro: Angular velocity in °/s (x, y, z).
            accel: Acceleration in m/s² (x, y, z).
            mag: Magnetic field in µT (x, y, z), optional.
            dt: Time delta in seconds, or None to auto-calculate.

        Returns:
            Updated Orientation estimate.
        """
        ...

    def reset(self) -> None:
        """Reset filter to initial state."""
        self._quaternion = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self._last_update = time.time()

    def set_orientation(self, quaternion: Quaternion) -> None:
        """Set current orientation.

        Args:
            quaternion: New orientation quaternion.
        """
        self._quaternion = quaternion_normalize(quaternion)


# =============================================================================
# Madgwick Filter
# =============================================================================


class MadgwickConfig(BaseModel):
    """Configuration for Madgwick filter."""

    model_config = {"frozen": False, "extra": "allow"}

    # Filter gain
    beta: float = Field(
        default=0.1,
        ge=0.0,
        le=1.0,
        description="Algorithm gain (higher = faster convergence, more noise)",
    )

    # Sample period
    sample_period: float = Field(
        default=0.01,
        gt=0.0,
        description="Expected sample period in seconds",
    )

    # Gyroscope units
    gyro_in_degrees: bool = Field(
        default=True,
        description="Whether gyroscope input is in degrees/s (vs radians/s)",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


class MadgwickFilter(OrientationFilter):
    """Madgwick's gradient descent AHRS algorithm.

    Efficient orientation filter using gradient descent optimization.
    Works with 6-DOF (accel + gyro) or 9-DOF (accel + gyro + mag) sensors.

    Based on: "An efficient orientation filter for inertial and
    inertial/magnetic sensor arrays" by Sebastian O.H. Madgwick, 2010.

    Example:
        >>> filter = MadgwickFilter(config=MadgwickConfig(beta=0.1))
        >>> gyro = Vector3(x=0.5, y=-0.3, z=0.1)  # °/s
        >>> accel = Vector3(x=0.0, y=0.0, z=9.81)  # m/s²
        >>> orientation = filter.update(gyro, accel)
        >>> print(f"Yaw: {orientation.euler.z:.1f}°")
    """

    def __init__(self, config: MadgwickConfig | None = None) -> None:
        """Initialize Madgwick filter.

        Args:
            config: Filter configuration.
        """
        config = config or MadgwickConfig()
        super().__init__(sample_period=config.sample_period)

        self._config = config
        self._beta = config.beta

    @property
    def beta(self) -> float:
        """Get filter gain."""
        return self._beta

    @beta.setter
    def beta(self, value: float) -> None:
        """Set filter gain."""
        self._beta = max(0.0, min(1.0, value))

    def update(
        self,
        gyro: Vector3,
        accel: Vector3,
        mag: Vector3 | None = None,
        dt: float | None = None,
    ) -> Orientation:
        """Update orientation using Madgwick algorithm.

        Args:
            gyro: Angular velocity in °/s (or rad/s if configured).
            accel: Acceleration (normalized internally).
            mag: Magnetic field (optional, normalized internally).
            dt: Time delta in seconds.

        Returns:
            Updated Orientation estimate.
        """
        # Calculate time delta
        now = time.time()
        if dt is None:
            dt = now - self._last_update
            if dt <= 0:
                dt = self._sample_period
        self._last_update = now

        # Convert gyroscope to radians/s
        if self._config.gyro_in_degrees:
            gx = gyro.x * DEG_TO_RAD
            gy = gyro.y * DEG_TO_RAD
            gz = gyro.z * DEG_TO_RAD
        else:
            gx, gy, gz = gyro.x, gyro.y, gyro.z

        # Get current quaternion
        q = self._quaternion
        _q0, _q1, _q2, _q3 = q.w, q.x, q.y, q.z

        # Normalize accelerometer
        ax, ay, az = accel.x, accel.y, accel.z
        norm = math.sqrt(ax**2 + ay**2 + az**2)
        if norm < 1e-10:
            # No valid acceleration, use gyro only
            return self._update_gyro_only(gx, gy, gz, dt)
        ax /= norm
        ay /= norm
        az /= norm

        if mag is not None and (mag.x != 0 or mag.y != 0 or mag.z != 0):
            # 9-DOF update with magnetometer
            self._update_9dof(gx, gy, gz, ax, ay, az, mag.x, mag.y, mag.z, dt)
        else:
            # 6-DOF update without magnetometer
            self._update_6dof(gx, gy, gz, ax, ay, az, dt)

        return Orientation.from_quaternion(self._quaternion, dt)

    def _update_6dof(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        dt: float,
    ) -> None:
        """Update using 6-DOF algorithm (accel + gyro only)."""
        q = self._quaternion
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z

        # Estimated direction of gravity
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3

        # Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = (
            _4q1 * q3q3
            - _2q3 * ax
            + 4.0 * q0q0 * q1
            - _2q0 * ay
            - _4q1
            + _8q1 * q1q1
            + _8q1 * q2q2
            + _4q1 * az
        )
        s2 = (
            4.0 * q0q0 * q2
            + _2q0 * ax
            + _4q2 * q3q3
            - _2q3 * ay
            - _4q2
            + _8q2 * q1q1
            + _8q2 * q2q2
            + _4q2 * az
        )
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay

        # Normalize step magnitude
        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm > 1e-10:
            s0 /= norm
            s1 /= norm
            s2 /= norm
            s3 /= norm

        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Apply feedback step
        qDot1 -= self._beta * s0
        qDot2 -= self._beta * s1
        qDot3 -= self._beta * s2
        qDot4 -= self._beta * s3

        # Integrate to yield quaternion
        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt

        # Normalize quaternion
        self._quaternion = quaternion_normalize(Quaternion(w=q0, x=q1, y=q2, z=q3))

    def _update_9dof(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        mx: float,
        my: float,
        mz: float,
        dt: float,
    ) -> None:
        """Update using 9-DOF algorithm (accel + gyro + mag)."""
        q = self._quaternion
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z

        # Normalize magnetometer
        norm = math.sqrt(mx**2 + my**2 + mz**2)
        if norm < 1e-10:
            return self._update_6dof(gx, gy, gz, ax, ay, az, dt)
        mx /= norm
        my /= norm
        mz /= norm

        # Auxiliary variables
        _2q0mx = 2.0 * q0 * mx
        _2q0my = 2.0 * q0 * my
        _2q0mz = 2.0 * q0 * mz
        _2q1mx = 2.0 * q1 * mx
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q0q2 = 2.0 * q0 * q2
        _2q2q3 = 2.0 * q2 * q3
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        # Reference direction of Earth's magnetic field
        hx = (
            mx * q0q0
            - _2q0my * q3
            + _2q0mz * q2
            + mx * q1q1
            + _2q1 * my * q2
            + _2q1 * mz * q3
            - mx * q2q2
            - mx * q3q3
        )
        hy = (
            _2q0mx * q3
            + my * q0q0
            - _2q0mz * q1
            + _2q1mx * q2
            - my * q1q1
            + my * q2q2
            + _2q2 * mz * q3
            - my * q3q3
        )
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = (
            -_2q0mx * q2
            + _2q0my * q1
            + mz * q0q0
            + _2q1mx * q3
            - mz * q1q1
            + _2q2 * my * q3
            - mz * q2q2
            + mz * q3q3
        )
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient descent algorithm corrective step
        s0 = (
            -_2q2 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q1 * (2.0 * q0q1 + _2q2q3 - ay)
            - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )
        s1 = (
            _2q3 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q0 * (2.0 * q0q1 + _2q2q3 - ay)
            - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az)
            + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )
        s2 = (
            -_2q0 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q3 * (2.0 * q0q1 + _2q2q3 - ay)
            - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az)
            + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )
        s3 = (
            _2q1 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q2 * (2.0 * q0q1 + _2q2q3 - ay)
            + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )

        # Normalize step magnitude
        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm > 1e-10:
            s0 /= norm
            s1 /= norm
            s2 /= norm
            s3 /= norm

        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Apply feedback step
        qDot1 -= self._beta * s0
        qDot2 -= self._beta * s1
        qDot3 -= self._beta * s2
        qDot4 -= self._beta * s3

        # Integrate to yield quaternion
        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt

        # Normalize quaternion
        self._quaternion = quaternion_normalize(Quaternion(w=q0, x=q1, y=q2, z=q3))

    def _update_gyro_only(self, gx: float, gy: float, gz: float, dt: float) -> Orientation:
        """Update using gyroscope only (no accelerometer)."""
        q = self._quaternion
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z

        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Integrate
        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt

        self._quaternion = quaternion_normalize(Quaternion(w=q0, x=q1, y=q2, z=q3))
        return Orientation.from_quaternion(self._quaternion, dt)


# =============================================================================
# Mahony Filter
# =============================================================================


class MahonyConfig(BaseModel):
    """Configuration for Mahony filter."""

    model_config = {"frozen": False, "extra": "allow"}

    # Proportional gain
    kp: float = Field(
        default=0.5,
        ge=0.0,
        description="Proportional feedback gain",
    )

    # Integral gain
    ki: float = Field(
        default=0.0,
        ge=0.0,
        description="Integral feedback gain (0 = disabled)",
    )

    # Sample period
    sample_period: float = Field(
        default=0.01,
        gt=0.0,
        description="Expected sample period in seconds",
    )

    # Gyroscope units
    gyro_in_degrees: bool = Field(
        default=True,
        description="Whether gyroscope input is in degrees/s (vs radians/s)",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


class MahonyFilter(OrientationFilter):
    """Mahony's complementary filter AHRS algorithm.

    Complementary filter using proportional-integral feedback.
    Works with 6-DOF (accel + gyro) or 9-DOF (accel + gyro + mag) sensors.

    Based on: "Nonlinear complementary filters on the special orthogonal group"
    by Robert Mahony, Tarek Hamel, and Jean-Michel Pflimlin, 2008.

    Example:
        >>> filter = MahonyFilter(config=MahonyConfig(kp=0.5, ki=0.0))
        >>> gyro = Vector3(x=0.5, y=-0.3, z=0.1)  # °/s
        >>> accel = Vector3(x=0.0, y=0.0, z=9.81)  # m/s²
        >>> orientation = filter.update(gyro, accel)
        >>> print(f"Pitch: {orientation.euler.y:.1f}°")
    """

    def __init__(self, config: MahonyConfig | None = None) -> None:
        """Initialize Mahony filter.

        Args:
            config: Filter configuration.
        """
        config = config or MahonyConfig()
        super().__init__(sample_period=config.sample_period)

        self._config = config
        self._kp = config.kp
        self._ki = config.ki

        # Integral error
        self._integral_fb = Vector3(x=0.0, y=0.0, z=0.0)

    @property
    def kp(self) -> float:
        """Get proportional gain."""
        return self._kp

    @kp.setter
    def kp(self, value: float) -> None:
        """Set proportional gain."""
        self._kp = max(0.0, value)

    @property
    def ki(self) -> float:
        """Get integral gain."""
        return self._ki

    @ki.setter
    def ki(self, value: float) -> None:
        """Set integral gain."""
        self._ki = max(0.0, value)

    def update(
        self,
        gyro: Vector3,
        accel: Vector3,
        mag: Vector3 | None = None,
        dt: float | None = None,
    ) -> Orientation:
        """Update orientation using Mahony algorithm.

        Args:
            gyro: Angular velocity in °/s (or rad/s if configured).
            accel: Acceleration (normalized internally).
            mag: Magnetic field (optional, normalized internally).
            dt: Time delta in seconds.

        Returns:
            Updated Orientation estimate.
        """
        # Calculate time delta
        now = time.time()
        if dt is None:
            dt = now - self._last_update
            if dt <= 0:
                dt = self._sample_period
        self._last_update = now

        # Convert gyroscope to radians/s
        if self._config.gyro_in_degrees:
            gx = gyro.x * DEG_TO_RAD
            gy = gyro.y * DEG_TO_RAD
            gz = gyro.z * DEG_TO_RAD
        else:
            gx, gy, gz = gyro.x, gyro.y, gyro.z

        # Get current quaternion
        q = self._quaternion
        _q0, _q1, _q2, _q3 = q.w, q.x, q.y, q.z

        # Normalize accelerometer
        ax, ay, az = accel.x, accel.y, accel.z
        norm = math.sqrt(ax**2 + ay**2 + az**2)
        if norm < 1e-10:
            # No valid acceleration, use gyro only
            return self._update_gyro_only(gx, gy, gz, dt)
        ax /= norm
        ay /= norm
        az /= norm

        if mag is not None and (mag.x != 0 or mag.y != 0 or mag.z != 0):
            # 9-DOF update with magnetometer
            self._update_9dof(gx, gy, gz, ax, ay, az, mag.x, mag.y, mag.z, dt)
        else:
            # 6-DOF update without magnetometer
            self._update_6dof(gx, gy, gz, ax, ay, az, dt)

        return Orientation.from_quaternion(self._quaternion, dt)

    def _update_6dof(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        dt: float,
    ) -> None:
        """Update using 6-DOF algorithm."""
        q = self._quaternion
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z

        # Estimated direction of gravity
        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        # Error is cross product between estimated and measured direction
        ex = ay * vz - az * vy
        ey = az * vx - ax * vz
        ez = ax * vy - ay * vx

        # Apply integral feedback
        if self._ki > 0:
            self._integral_fb = Vector3(
                x=self._integral_fb.x + ex * dt,
                y=self._integral_fb.y + ey * dt,
                z=self._integral_fb.z + ez * dt,
            )
            gx += self._ki * self._integral_fb.x
            gy += self._ki * self._integral_fb.y
            gz += self._ki * self._integral_fb.z

        # Apply proportional feedback
        gx += self._kp * ex
        gy += self._kp * ey
        gz += self._kp * ez

        # Integrate rate of change
        q0 += (-q1 * gx - q2 * gy - q3 * gz) * 0.5 * dt
        q1 += (q0 * gx + q2 * gz - q3 * gy) * 0.5 * dt
        q2 += (q0 * gy - q1 * gz + q3 * gx) * 0.5 * dt
        q3 += (q0 * gz + q1 * gy - q2 * gx) * 0.5 * dt

        # Normalize quaternion
        self._quaternion = quaternion_normalize(Quaternion(w=q0, x=q1, y=q2, z=q3))

    def _update_9dof(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        mx: float,
        my: float,
        mz: float,
        dt: float,
    ) -> None:
        """Update using 9-DOF algorithm."""
        q = self._quaternion
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z

        # Normalize magnetometer
        norm = math.sqrt(mx**2 + my**2 + mz**2)
        if norm < 1e-10:
            return self._update_6dof(gx, gy, gz, ax, ay, az, dt)
        mx /= norm
        my /= norm
        mz /= norm

        # Auxiliary variables
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        # Reference direction of Earth's magnetic field
        hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
        hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
        bx = math.sqrt(hx * hx + hy * hy)
        bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))

        # Estimated direction of gravity and magnetic field
        vx = 2.0 * (q1q3 - q0q2)
        vy = 2.0 * (q0q1 + q2q3)
        vz = q0q0 - q1q1 - q2q2 + q3q3
        wx = 2.0 * (bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2))
        wy = 2.0 * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3))
        wz = 2.0 * (bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2))

        # Error is sum of cross products
        ex = ay * vz - az * vy + my * wz - mz * wy
        ey = az * vx - ax * vz + mz * wx - mx * wz
        ez = ax * vy - ay * vx + mx * wy - my * wx

        # Apply integral feedback
        if self._ki > 0:
            self._integral_fb = Vector3(
                x=self._integral_fb.x + ex * dt,
                y=self._integral_fb.y + ey * dt,
                z=self._integral_fb.z + ez * dt,
            )
            gx += self._ki * self._integral_fb.x
            gy += self._ki * self._integral_fb.y
            gz += self._ki * self._integral_fb.z

        # Apply proportional feedback
        gx += self._kp * ex
        gy += self._kp * ey
        gz += self._kp * ez

        # Integrate rate of change
        q0 += (-q1 * gx - q2 * gy - q3 * gz) * 0.5 * dt
        q1 += (q0 * gx + q2 * gz - q3 * gy) * 0.5 * dt
        q2 += (q0 * gy - q1 * gz + q3 * gx) * 0.5 * dt
        q3 += (q0 * gz + q1 * gy - q2 * gx) * 0.5 * dt

        # Normalize quaternion
        self._quaternion = quaternion_normalize(Quaternion(w=q0, x=q1, y=q2, z=q3))

    def _update_gyro_only(self, gx: float, gy: float, gz: float, dt: float) -> Orientation:
        """Update using gyroscope only."""
        q = self._quaternion
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z

        # Integrate rate of change
        q0 += (-q1 * gx - q2 * gy - q3 * gz) * 0.5 * dt
        q1 += (q0 * gx + q2 * gz - q3 * gy) * 0.5 * dt
        q2 += (q0 * gy - q1 * gz + q3 * gx) * 0.5 * dt
        q3 += (q0 * gz + q1 * gy - q2 * gx) * 0.5 * dt

        self._quaternion = quaternion_normalize(Quaternion(w=q0, x=q1, y=q2, z=q3))
        return Orientation.from_quaternion(self._quaternion, dt)

    def reset(self) -> None:
        """Reset filter to initial state."""
        super().reset()
        self._integral_fb = Vector3(x=0.0, y=0.0, z=0.0)


# =============================================================================
# Factory Functions
# =============================================================================


def get_orientation_filter(
    algorithm: str = "madgwick",
    **kwargs: Any,
) -> OrientationFilter:
    """Get an orientation filter by algorithm name.

    Args:
        algorithm: Filter algorithm ("madgwick" or "mahony").
        **kwargs: Configuration options.

    Returns:
        Configured OrientationFilter instance.

    Example:
        >>> filter = get_orientation_filter("madgwick", beta=0.1)
        >>> filter = get_orientation_filter("mahony", kp=0.5, ki=0.1)
    """
    algorithm = algorithm.lower()

    if algorithm == "madgwick":
        config = MadgwickConfig(**kwargs)
        return MadgwickFilter(config)
    elif algorithm == "mahony":
        config = MahonyConfig(**kwargs)
        return MahonyFilter(config)
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}. Use 'madgwick' or 'mahony'.")


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Constants
    "DEG_TO_RAD",
    "RAD_TO_DEG",
    # Madgwick
    "MadgwickConfig",
    "MadgwickFilter",
    # Mahony
    "MahonyConfig",
    "MahonyFilter",
    # Data classes
    "Orientation",
    # Base class
    "OrientationFilter",
    "euler_to_quaternion",
    # Factory
    "get_orientation_filter",
    "quaternion_conjugate",
    "quaternion_multiply",
    # Quaternion utilities
    "quaternion_normalize",
    "quaternion_to_euler",
]
