"""Trajectory generation for smooth motion profiles.

This module provides trajectory planning primitives for generating
smooth, physically realizable motion profiles. It supports:

- Point-to-point trajectories
- Linear interpolation
- Trapezoidal velocity profiles
- S-curve (jerk-limited) profiles
- Cubic and quintic polynomial trajectories
- Spline trajectories for multi-waypoint motion
- Blended trajectories for continuous motion

All trajectory classes generate position, velocity, and acceleration
at any time t, enabling smooth and predictable robot motion.

Example:
    >>> from robo_infra.motion import TrajectoryPoint, LinearInterpolator
    >>>
    >>> # Create a linear trajectory from 0 to 100 over 2 seconds
    >>> traj = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
    >>>
    >>> # Get position at t=1.0
    >>> point = traj.sample(1.0)
    >>> print(f"Position: {point.position}, Velocity: {point.velocity}")

    >>> # Use TrajectoryGenerator for different profiles
    >>> from robo_infra.motion import TrajectoryGenerator, TrajectoryProfile
    >>> gen = TrajectoryGenerator(profile=TrajectoryProfile.QUINTIC, max_velocity=10.0)
    >>> traj = gen.generate(start=0.0, end=100.0)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    """A single point in a trajectory.

    Represents the kinematic state at a specific time, including
    position, velocity, acceleration, and optionally jerk.

    This is an immutable, frozen dataclass for thread safety and
    to prevent accidental modification of trajectory data.

    Attributes:
        position: Position at this point (units depend on application).
        velocity: Velocity at this point (position/time).
        acceleration: Acceleration at this point (position/time²).
        time: Time at which this point occurs (seconds from start).
        jerk: Rate of change of acceleration (position/time³). Optional.

    Example:
        >>> point = TrajectoryPoint(
        ...     position=50.0,
        ...     velocity=25.0,
        ...     acceleration=0.0,
        ...     time=1.0
        ... )
        >>> print(f"At t={point.time}s: pos={point.position}")
        At t=1.0s: pos=50.0
    """

    position: float
    velocity: float
    acceleration: float
    time: float
    jerk: float = field(default=0.0)

    def __post_init__(self) -> None:
        """Validate trajectory point values."""
        if self.time < 0:
            raise ValueError(f"time must be >= 0, got {self.time}")

    def __repr__(self) -> str:
        """Return a compact string representation."""
        return (
            f"TrajectoryPoint(t={self.time:.3f}s, "
            f"pos={self.position:.3f}, "
            f"vel={self.velocity:.3f}, "
            f"acc={self.acceleration:.3f})"
        )

    @property
    def is_stationary(self) -> bool:
        """Check if this point represents a stationary state.

        Returns True if velocity and acceleration are both zero
        (or very close to zero, within floating point tolerance).
        """
        return abs(self.velocity) < 1e-9 and abs(self.acceleration) < 1e-9

    @property
    def kinetic_energy_factor(self) -> float:
        """Return velocity squared, proportional to kinetic energy.

        This is useful for energy-based motion planning and
        determining if a trajectory meets energy constraints.
        The actual kinetic energy is 0.5 * m * v², but since
        mass is application-specific, we return just v².
        """
        return self.velocity * self.velocity

    def scaled(self, position_scale: float = 1.0, time_scale: float = 1.0) -> TrajectoryPoint:
        """Return a new point with scaled position and time.

        Useful for unit conversions or trajectory scaling.

        Args:
            position_scale: Factor to multiply position by.
            time_scale: Factor to multiply time by.

        Returns:
            New TrajectoryPoint with scaled values.

        Note:
            When scaling time, velocity scales inversely and
            acceleration scales inversely squared.
        """
        inv_time_scale = 1.0 / time_scale if time_scale != 0 else 1.0
        return TrajectoryPoint(
            position=self.position * position_scale,
            velocity=self.velocity * position_scale * inv_time_scale,
            acceleration=self.acceleration * position_scale * inv_time_scale * inv_time_scale,
            time=self.time * time_scale,
            jerk=self.jerk * position_scale * inv_time_scale * inv_time_scale * inv_time_scale,
        )

    def offset(self, position_offset: float = 0.0, time_offset: float = 0.0) -> TrajectoryPoint:
        """Return a new point with offset position and time.

        Useful for translating trajectories in space or time.

        Args:
            position_offset: Value to add to position.
            time_offset: Value to add to time.

        Returns:
            New TrajectoryPoint with offset values.
        """
        new_time = self.time + time_offset
        if new_time < 0:
            raise ValueError(f"Resulting time would be negative: {new_time}")
        return TrajectoryPoint(
            position=self.position + position_offset,
            velocity=self.velocity,  # Unchanged
            acceleration=self.acceleration,  # Unchanged
            time=new_time,
            jerk=self.jerk,  # Unchanged
        )


@dataclass(frozen=True, slots=True)
class MultiAxisTrajectoryPoint:
    """A trajectory point for multiple axes.

    Represents the kinematic state of multiple axes at a specific time.
    Each axis has its own position, velocity, and acceleration.

    Attributes:
        positions: Position for each axis (dict of axis_name -> position).
        velocities: Velocity for each axis (dict of axis_name -> velocity).
        accelerations: Acceleration for each axis (dict of axis_name -> acceleration).
        time: Time at which this point occurs (seconds from start).

    Example:
        >>> point = MultiAxisTrajectoryPoint(
        ...     positions={"x": 10.0, "y": 20.0, "z": 5.0},
        ...     velocities={"x": 1.0, "y": 2.0, "z": 0.5},
        ...     accelerations={"x": 0.0, "y": 0.0, "z": 0.0},
        ...     time=1.0
        ... )
        >>> print(f"X position: {point.positions['x']}")
        X position: 10.0
    """

    positions: dict[str, float]
    velocities: dict[str, float]
    accelerations: dict[str, float]
    time: float

    def __post_init__(self) -> None:
        """Validate multi-axis trajectory point."""
        if self.time < 0:
            raise ValueError(f"time must be >= 0, got {self.time}")

        # Ensure all dicts have the same keys
        pos_keys = set(self.positions.keys())
        vel_keys = set(self.velocities.keys())
        acc_keys = set(self.accelerations.keys())

        if pos_keys != vel_keys:
            raise ValueError(f"Axis mismatch: positions has {pos_keys}, velocities has {vel_keys}")
        if pos_keys != acc_keys:
            raise ValueError(
                f"Axis mismatch: positions has {pos_keys}, accelerations has {acc_keys}"
            )

    @property
    def axes(self) -> list[str]:
        """Return list of axis names."""
        return list(self.positions.keys())

    @property
    def num_axes(self) -> int:
        """Return number of axes."""
        return len(self.positions)

    def get_axis(self, axis: str) -> TrajectoryPoint:
        """Get a single-axis TrajectoryPoint for the specified axis.

        Args:
            axis: Name of the axis.

        Returns:
            TrajectoryPoint for that axis.

        Raises:
            KeyError: If axis doesn't exist.
        """
        return TrajectoryPoint(
            position=self.positions[axis],
            velocity=self.velocities[axis],
            acceleration=self.accelerations[axis],
            time=self.time,
        )

    def is_stationary(self) -> bool:
        """Check if all axes are stationary."""
        return all(
            abs(v) < 1e-9 and abs(self.accelerations[k]) < 1e-9 for k, v in self.velocities.items()
        )

    def __repr__(self) -> str:
        """Return a compact string representation."""
        axes_str = ", ".join(f"{k}={v:.2f}" for k, v in self.positions.items())
        return f"MultiAxisTrajectoryPoint(t={self.time:.3f}s, {axes_str})"


class LinearInterpolator:
    """Linear interpolation between two positions.

    Generates a trajectory with constant velocity between start and end
    positions over a specified duration. This is the simplest motion
    profile, suitable for non-critical moves or when smoothness is not
    required.

    The velocity is constant throughout the move (except at endpoints),
    and acceleration is zero everywhere except at the start and end
    (where it's theoretically infinite - instantaneous velocity change).

    Attributes:
        start: Starting position.
        end: Ending position.
        duration: Total time for the move (seconds).

    Example:
        >>> interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        >>> interp.position_at(0.0)
        0.0
        >>> interp.position_at(1.0)
        50.0
        >>> interp.position_at(2.0)
        100.0
        >>> interp.velocity_at(1.0)
        50.0
    """

    __slots__ = ("_distance", "_duration", "_end", "_start", "_velocity")

    def __init__(self, start: float, end: float, duration: float) -> None:
        """Initialize a linear interpolator.

        Args:
            start: Starting position.
            end: Ending position.
            duration: Total time for the move (seconds). Must be > 0.

        Raises:
            ValueError: If duration <= 0.
        """
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        self._start = start
        self._end = end
        self._duration = duration
        self._distance = end - start
        self._velocity = self._distance / duration

    @property
    def start(self) -> float:
        """Return the starting position."""
        return self._start

    @property
    def end(self) -> float:
        """Return the ending position."""
        return self._end

    @property
    def duration(self) -> float:
        """Return the total duration."""
        return self._duration

    @property
    def distance(self) -> float:
        """Return the total distance (can be negative)."""
        return self._distance

    @property
    def velocity(self) -> float:
        """Return the constant velocity during the move."""
        return self._velocity

    def position_at(self, t: float) -> float:
        """Get the position at time t.

        Args:
            t: Time from start (seconds). Clamped to [0, duration].

        Returns:
            Position at time t.
        """
        # Clamp time to valid range
        t_clamped = max(0.0, min(t, self._duration))
        return self._start + self._velocity * t_clamped

    def velocity_at(self, t: float) -> float:
        """Get the velocity at time t.

        For linear interpolation, velocity is constant during the move
        and zero before/after.

        Args:
            t: Time from start (seconds).

        Returns:
            Velocity at time t (constant during move, 0 outside).
        """
        if t < 0 or t > self._duration:
            return 0.0
        return self._velocity

    def acceleration_at(self, t: float) -> float:
        """Get the acceleration at time t.

        For linear interpolation, acceleration is always zero
        (velocity is constant).

        Args:
            t: Time from start (seconds).

        Returns:
            Always 0.0 for linear interpolation.
        """
        # Linear interpolation has zero acceleration (constant velocity)
        # In reality, there's infinite acceleration at start/end, but
        # we return 0 for practical purposes
        return 0.0

    def is_complete(self, t: float) -> bool:
        """Check if the trajectory is complete at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            True if t >= duration.
        """
        return t >= self._duration

    def progress(self, t: float) -> float:
        """Get the progress as a fraction [0, 1].

        Args:
            t: Time from start (seconds).

        Returns:
            Progress from 0.0 (start) to 1.0 (end), clamped.
        """
        if self._duration == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._duration))

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time t.

        Returns a complete TrajectoryPoint with position, velocity,
        acceleration, and time.

        Args:
            t: Time from start (seconds). Clamped to [0, duration].

        Returns:
            TrajectoryPoint at time t.
        """
        t_clamped = max(0.0, min(t, self._duration))
        return TrajectoryPoint(
            position=self.position_at(t_clamped),
            velocity=self.velocity_at(t_clamped),
            acceleration=0.0,
            time=t_clamped,
        )

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample the trajectory at n evenly-spaced points.

        Args:
            n: Number of points to sample. Must be >= 2.

        Returns:
            List of n TrajectoryPoints from start to end.

        Raises:
            ValueError: If n < 2.
        """
        if n < 2:
            raise ValueError(f"n must be >= 2, got {n}")

        dt = self._duration / (n - 1)
        return [self.sample(i * dt) for i in range(n)]

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample the trajectory at regular time intervals.

        Args:
            dt: Time interval between samples (seconds). Must be > 0.

        Returns:
            List of TrajectoryPoints from start to end (inclusive).

        Raises:
            ValueError: If dt <= 0.
        """
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")

        points: list[TrajectoryPoint] = []
        t = 0.0
        while t <= self._duration:
            points.append(self.sample(t))
            t += dt

        # Ensure we include the end point
        if points and points[-1].time < self._duration:
            points.append(self.sample(self._duration))

        return points

    def reversed(self) -> LinearInterpolator:
        """Return a reversed interpolator (end to start).

        Returns:
            New LinearInterpolator going from end to start.
        """
        return LinearInterpolator(self._end, self._start, self._duration)

    def scaled(self, time_scale: float) -> LinearInterpolator:
        """Return a time-scaled interpolator.

        Args:
            time_scale: Factor to multiply duration by. Must be > 0.

        Returns:
            New LinearInterpolator with scaled duration.

        Raises:
            ValueError: If time_scale <= 0.
        """
        if time_scale <= 0:
            raise ValueError(f"time_scale must be > 0, got {time_scale}")
        return LinearInterpolator(self._start, self._end, self._duration * time_scale)

    def __repr__(self) -> str:
        """Return a string representation."""
        return (
            f"LinearInterpolator(start={self._start:.3f}, "
            f"end={self._end:.3f}, duration={self._duration:.3f}s)"
        )


class TrapezoidalProfile:
    """Trapezoidal velocity profile for smooth acceleration-limited motion.

    Generates a trajectory with three phases:
    1. Acceleration phase: constant acceleration from rest to max velocity
    2. Cruise phase: constant velocity (may be zero for short moves)
    3. Deceleration phase: constant deceleration from max velocity to rest

    For short moves where max velocity cannot be reached, the profile
    becomes triangular (no cruise phase).

    This is the most common motion profile in industrial robotics as it
    provides a good balance between speed and smoothness while respecting
    acceleration limits.

    Attributes:
        start: Starting position.
        end: Ending position.
        max_velocity: Maximum allowed velocity (positive).
        max_acceleration: Maximum allowed acceleration (positive).

    Example:
        >>> profile = TrapezoidalProfile(
        ...     start=0.0, end=100.0,
        ...     max_velocity=10.0, max_acceleration=5.0
        ... )
        >>> profile.total_time
        12.0  # 2s accel + 8s cruise + 2s decel
        >>> profile.position_at(1.0)  # During acceleration
        2.5
        >>> profile.velocity_at(1.0)
        5.0
    """

    _accel_distance: float
    _accel_time: float
    _cruise_distance: float
    _cruise_time: float
    _decel_time: float
    _direction: float
    _distance: float
    _end: float
    _is_triangular: bool
    _max_acceleration: float
    _max_velocity: float
    _peak_velocity: float
    _start: float
    _total_time: float

    __slots__ = (
        "_accel_distance",
        "_accel_time",
        "_cruise_distance",
        "_cruise_time",
        "_decel_time",
        "_direction",
        "_distance",
        "_end",
        "_is_triangular",
        "_max_acceleration",
        "_max_velocity",
        "_peak_velocity",
        "_start",
        "_total_time",
    )

    def __init__(
        self,
        start: float,
        end: float,
        max_velocity: float,
        max_acceleration: float,
    ) -> None:
        """Initialize a trapezoidal motion profile.

        Args:
            start: Starting position.
            end: Ending position.
            max_velocity: Maximum allowed velocity. Must be > 0.
            max_acceleration: Maximum allowed acceleration. Must be > 0.

        Raises:
            ValueError: If max_velocity <= 0 or max_acceleration <= 0.
        """
        if max_velocity <= 0:
            raise ValueError(f"max_velocity must be > 0, got {max_velocity}")
        if max_acceleration <= 0:
            raise ValueError(f"max_acceleration must be > 0, got {max_acceleration}")

        self._start = start
        self._end = end
        self._max_velocity = max_velocity
        self._max_acceleration = max_acceleration

        # Calculate direction and distance
        self._distance = abs(end - start)
        self._direction = 1.0 if end >= start else -1.0

        # Calculate the profile phases
        self._calculate_profile()

    def _calculate_profile(self) -> None:
        """Calculate the acceleration, cruise, and deceleration phases."""
        # Time to accelerate to max velocity: t = v / a
        time_to_max_vel = self._max_velocity / self._max_acceleration

        # Distance covered during acceleration (and deceleration): d = 0.5 * a * t^2
        accel_distance = 0.5 * self._max_acceleration * time_to_max_vel**2

        # Total distance needed for full accel + decel (no cruise)
        min_distance_for_full_profile = 2 * accel_distance

        if self._distance < min_distance_for_full_profile:
            # Triangular profile: can't reach max velocity
            self._is_triangular = True

            # Peak velocity achieved: v = sqrt(a * d)
            # (derived from: d = 2 * 0.5 * a * t^2, and v = a * t)
            self._peak_velocity = (self._max_acceleration * self._distance) ** 0.5

            # Time for each phase: t = v / a
            self._accel_time = self._peak_velocity / self._max_acceleration
            self._cruise_time = 0.0
            self._decel_time = self._accel_time

            # Distances
            self._accel_distance = 0.5 * self._distance
            self._cruise_distance = 0.0
        else:
            # Full trapezoidal profile
            self._is_triangular = False
            self._peak_velocity = self._max_velocity

            # Acceleration and deceleration times
            self._accel_time = time_to_max_vel
            self._decel_time = time_to_max_vel

            # Acceleration and deceleration distances
            self._accel_distance = accel_distance

            # Cruise distance and time
            self._cruise_distance = self._distance - 2 * accel_distance
            self._cruise_time = self._cruise_distance / self._max_velocity

        # Total time
        self._total_time = self._accel_time + self._cruise_time + self._decel_time

    @property
    def start(self) -> float:
        """Return the starting position."""
        return self._start

    @property
    def end(self) -> float:
        """Return the ending position."""
        return self._end

    @property
    def distance(self) -> float:
        """Return the total distance (absolute value)."""
        return self._distance

    @property
    def max_velocity(self) -> float:
        """Return the maximum velocity constraint."""
        return self._max_velocity

    @property
    def max_acceleration(self) -> float:
        """Return the maximum acceleration constraint."""
        return self._max_acceleration

    @property
    def peak_velocity(self) -> float:
        """Return the actual peak velocity achieved.

        For full trapezoidal profiles, this equals max_velocity.
        For triangular profiles, this is less than max_velocity.
        """
        return self._peak_velocity

    @property
    def total_time(self) -> float:
        """Return the total duration of the motion."""
        return self._total_time

    @property
    def accel_time(self) -> float:
        """Return the duration of the acceleration phase."""
        return self._accel_time

    @property
    def cruise_time(self) -> float:
        """Return the duration of the cruise phase (0 for triangular)."""
        return self._cruise_time

    @property
    def decel_time(self) -> float:
        """Return the duration of the deceleration phase."""
        return self._decel_time

    @property
    def is_triangular(self) -> bool:
        """Return True if this is a triangular profile (no cruise phase)."""
        return self._is_triangular

    def position_at(self, t: float) -> float:
        """Get the position at time t.

        Args:
            t: Time from start (seconds). Clamped to [0, total_time].

        Returns:
            Position at time t.
        """
        # Clamp time
        if t <= 0:
            return self._start
        if t >= self._total_time:
            return self._end

        # Determine which phase we're in
        if t < self._accel_time:
            # Acceleration phase: x = x0 + 0.5 * a * t^2
            distance = 0.5 * self._max_acceleration * t**2
        elif t < self._accel_time + self._cruise_time:
            # Cruise phase: x = x_accel_end + v * (t - t_accel)
            t_cruise = t - self._accel_time
            distance = self._accel_distance + self._peak_velocity * t_cruise
        else:
            # Deceleration phase
            t_decel = t - self._accel_time - self._cruise_time
            # Position at start of decel
            pos_at_decel_start = self._accel_distance + self._cruise_distance
            # Decel equation: pos_decel_start + peak_vel * t - 0.5 * accel * t^2
            distance = (
                pos_at_decel_start
                + self._peak_velocity * t_decel
                - 0.5 * self._max_acceleration * t_decel**2
            )

        return self._start + self._direction * distance

    def velocity_at(self, t: float) -> float:
        """Get the velocity at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            Velocity at time t (signed according to direction).
        """
        # Outside the motion
        if t <= 0 or t >= self._total_time:
            return 0.0

        # Determine which phase we're in
        if t < self._accel_time:
            # Acceleration phase: v = a * t
            velocity = self._max_acceleration * t
        elif t < self._accel_time + self._cruise_time:
            # Cruise phase: constant velocity
            velocity = self._peak_velocity
        else:
            # Deceleration phase: v = v_peak - a * t_decel
            t_decel = t - self._accel_time - self._cruise_time
            velocity = self._peak_velocity - self._max_acceleration * t_decel

        return self._direction * velocity

    def acceleration_at(self, t: float) -> float:
        """Get the acceleration at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            Acceleration at time t (signed according to direction).
        """
        # Outside the motion
        if t <= 0 or t >= self._total_time:
            return 0.0

        # Determine which phase we're in
        if t < self._accel_time:
            # Acceleration phase
            return self._direction * self._max_acceleration
        elif t < self._accel_time + self._cruise_time:
            # Cruise phase: zero acceleration
            return 0.0
        else:
            # Deceleration phase
            return -self._direction * self._max_acceleration

    def is_complete(self, t: float) -> bool:
        """Check if the trajectory is complete at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            True if t >= total_time.
        """
        return t >= self._total_time

    def progress(self, t: float) -> float:
        """Get the progress as a fraction [0, 1].

        Args:
            t: Time from start (seconds).

        Returns:
            Progress from 0.0 (start) to 1.0 (end), clamped.
        """
        if self._total_time == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._total_time))

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time t.

        Returns a complete TrajectoryPoint with position, velocity,
        acceleration, and time.

        Args:
            t: Time from start (seconds). Clamped to [0, total_time].

        Returns:
            TrajectoryPoint at time t.
        """
        t_clamped = max(0.0, min(t, self._total_time))
        return TrajectoryPoint(
            position=self.position_at(t_clamped),
            velocity=self.velocity_at(t_clamped),
            acceleration=self.acceleration_at(t_clamped),
            time=t_clamped,
        )

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample the trajectory at n evenly-spaced points.

        Args:
            n: Number of points to sample. Must be >= 2.

        Returns:
            List of n TrajectoryPoints from start to end.

        Raises:
            ValueError: If n < 2.
        """
        if n < 2:
            raise ValueError(f"n must be >= 2, got {n}")

        dt = self._total_time / (n - 1)
        return [self.sample(i * dt) for i in range(n)]

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample the trajectory at regular time intervals.

        Args:
            dt: Time interval between samples (seconds). Must be > 0.

        Returns:
            List of TrajectoryPoints from start to end (inclusive).

        Raises:
            ValueError: If dt <= 0.
        """
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")

        points: list[TrajectoryPoint] = []
        t = 0.0
        while t <= self._total_time:
            points.append(self.sample(t))
            t += dt

        # Ensure we include the end point
        if points and points[-1].time < self._total_time:
            points.append(self.sample(self._total_time))

        return points

    def reversed(self) -> TrapezoidalProfile:
        """Return a reversed profile (end to start).

        Returns:
            New TrapezoidalProfile going from end to start.
        """
        return TrapezoidalProfile(
            self._end, self._start, self._max_velocity, self._max_acceleration
        )

    def scaled(self, time_scale: float) -> TrapezoidalProfile:
        """Return a time-scaled profile.

        Scaling time affects velocity and acceleration inversely.

        Args:
            time_scale: Factor to multiply duration by. Must be > 0.

        Returns:
            New TrapezoidalProfile with scaled velocity/acceleration.

        Raises:
            ValueError: If time_scale <= 0.
        """
        if time_scale <= 0:
            raise ValueError(f"time_scale must be > 0, got {time_scale}")

        # To scale time by k, we need:
        # - velocity scaled by 1/k
        # - acceleration scaled by 1/k^2
        new_max_vel = self._max_velocity / time_scale
        new_max_accel = self._max_acceleration / (time_scale * time_scale)

        return TrapezoidalProfile(self._start, self._end, new_max_vel, new_max_accel)

    def __repr__(self) -> str:
        """Return a string representation."""
        profile_type = "triangular" if self._is_triangular else "trapezoidal"
        return (
            f"TrapezoidalProfile({profile_type}, "
            f"start={self._start:.3f}, end={self._end:.3f}, "
            f"peak_vel={self._peak_velocity:.3f}, "
            f"total_time={self._total_time:.3f}s)"
        )


class Trajectory:
    """Multi-point trajectory through a sequence of waypoints.

    Generates a smooth trajectory through multiple waypoints using
    linear interpolation between consecutive points. Supports both
    explicit timing (user-specified times) and automatic timing
    (evenly-spaced based on default velocity).

    This is useful for teaching trajectories, playback of recorded
    motions, or defining complex paths through multiple positions.

    Attributes:
        waypoints: List of position values at each waypoint.
        times: List of times at which to reach each waypoint.

    Example:
        >>> # Create trajectory with explicit times
        >>> traj = Trajectory(
        ...     waypoints=[0.0, 50.0, 100.0, 75.0],
        ...     times=[0.0, 1.0, 2.0, 3.0]
        ... )
        >>> traj.position_at(0.5)  # Halfway between 0 and 50
        25.0
        >>> traj.total_time
        3.0

        >>> # Create trajectory with automatic timing
        >>> traj = Trajectory(waypoints=[0.0, 100.0, 50.0])
        >>> traj.add_waypoint(200.0)  # Add another point
    """

    _waypoints: list[float]
    _times: list[float]
    _segments: list[LinearInterpolator]
    _total_time: float
    _default_velocity: float

    __slots__ = (
        "_default_velocity",
        "_segments",
        "_times",
        "_total_time",
        "_waypoints",
    )

    def __init__(
        self,
        waypoints: list[float],
        times: list[float] | None = None,
        default_velocity: float = 1.0,
    ) -> None:
        """Initialize a multi-point trajectory.

        Args:
            waypoints: List of position values. Must have at least 1 point.
            times: Optional list of times for each waypoint. Must have same
                length as waypoints. First time should be 0.0. If None,
                times are calculated based on default_velocity.
            default_velocity: Velocity used to calculate segment durations
                when times are not provided. Must be > 0.

        Raises:
            ValueError: If waypoints is empty, times length doesn't match,
                times are not monotonically increasing, or default_velocity <= 0.
        """
        if not waypoints:
            raise ValueError("waypoints must have at least 1 point")
        if default_velocity <= 0:
            raise ValueError(f"default_velocity must be > 0, got {default_velocity}")

        self._waypoints = list(waypoints)
        self._default_velocity = default_velocity

        if times is not None:
            if len(times) != len(waypoints):
                raise ValueError(
                    f"times length ({len(times)}) must match waypoints length ({len(waypoints)})"
                )
            # Validate monotonically increasing
            for i in range(1, len(times)):
                if times[i] <= times[i - 1]:
                    raise ValueError(
                        f"times must be monotonically increasing, "
                        f"but times[{i}]={times[i]} <= times[{i - 1}]={times[i - 1]}"
                    )
            self._times = list(times)
        else:
            # Calculate times based on default velocity
            self._times = self._calculate_times()

        # Build interpolation segments
        self._segments = self._build_segments()
        self._total_time = self._times[-1] if self._times else 0.0

    def _calculate_times(self) -> list[float]:
        """Calculate times based on distances and default velocity."""
        times = [0.0]
        for i in range(1, len(self._waypoints)):
            distance = abs(self._waypoints[i] - self._waypoints[i - 1])
            duration = distance / self._default_velocity if distance > 0 else 0.1
            times.append(times[-1] + duration)
        return times

    def _build_segments(self) -> list[LinearInterpolator]:
        """Build linear interpolation segments between waypoints."""
        segments: list[LinearInterpolator] = []
        for i in range(len(self._waypoints) - 1):
            duration = self._times[i + 1] - self._times[i]
            if duration > 0:
                segments.append(
                    LinearInterpolator(
                        start=self._waypoints[i],
                        end=self._waypoints[i + 1],
                        duration=duration,
                    )
                )
        return segments

    @property
    def waypoints(self) -> list[float]:
        """Return a copy of the waypoints list."""
        return list(self._waypoints)

    @property
    def times(self) -> list[float]:
        """Return a copy of the times list."""
        return list(self._times)

    @property
    def num_waypoints(self) -> int:
        """Return the number of waypoints."""
        return len(self._waypoints)

    @property
    def num_segments(self) -> int:
        """Return the number of segments (waypoints - 1)."""
        return len(self._segments)

    @property
    def total_time(self) -> float:
        """Return the total duration of the trajectory."""
        return self._total_time

    @property
    def default_velocity(self) -> float:
        """Return the default velocity for automatic timing."""
        return self._default_velocity

    def add_waypoint(
        self, position: float, time: float | None = None, duration: float | None = None
    ) -> None:
        """Add a waypoint to the end of the trajectory.

        Args:
            position: Position value for the new waypoint.
            time: Absolute time for the waypoint. If None, calculated
                from duration or default velocity.
            duration: Duration from previous waypoint. Ignored if time is set.
                If None, calculated from default velocity.

        Raises:
            ValueError: If time is <= the last waypoint time.
        """
        if time is not None:
            if time <= self._times[-1]:
                raise ValueError(f"time ({time}) must be > last waypoint time ({self._times[-1]})")
            new_time = time
        elif duration is not None:
            if duration <= 0:
                raise ValueError(f"duration must be > 0, got {duration}")
            new_time = self._times[-1] + duration
        else:
            # Calculate from default velocity
            distance = abs(position - self._waypoints[-1])
            calc_duration = distance / self._default_velocity if distance > 0 else 0.1
            new_time = self._times[-1] + calc_duration

        # Add the new waypoint
        self._waypoints.append(position)
        self._times.append(new_time)

        # Add new segment
        segment_duration = new_time - self._times[-2]
        if segment_duration > 0:
            self._segments.append(
                LinearInterpolator(
                    start=self._waypoints[-2],
                    end=position,
                    duration=segment_duration,
                )
            )

        self._total_time = new_time

    def _find_segment(self, t: float) -> tuple[int, float]:
        """Find the segment index and local time for a global time t.

        Returns:
            Tuple of (segment_index, local_time_within_segment).
        """
        # Handle edge cases
        if t <= 0:
            return (0, 0.0)
        if t >= self._total_time:
            return (
                max(0, len(self._segments) - 1),
                self._segments[-1].duration if self._segments else 0.0,
            )

        # Find the segment containing t
        for i, _segment in enumerate(self._segments):
            segment_start = self._times[i]
            segment_end = self._times[i + 1]
            if segment_start <= t < segment_end:
                local_t = t - segment_start
                return (i, local_t)

        # Fallback to last segment
        return (len(self._segments) - 1, self._segments[-1].duration if self._segments else 0.0)

    def position_at(self, t: float) -> float:
        """Get the position at time t.

        Args:
            t: Time from start (seconds). Clamped to [0, total_time].

        Returns:
            Position at time t.
        """
        if not self._segments:
            return self._waypoints[0] if self._waypoints else 0.0

        if t <= 0:
            return self._waypoints[0]
        if t >= self._total_time:
            return self._waypoints[-1]

        seg_idx, local_t = self._find_segment(t)
        return self._segments[seg_idx].position_at(local_t)

    def velocity_at(self, t: float) -> float:
        """Get the velocity at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            Velocity at time t.
        """
        if not self._segments:
            return 0.0

        if t <= 0 or t >= self._total_time:
            return 0.0

        seg_idx, local_t = self._find_segment(t)
        return self._segments[seg_idx].velocity_at(local_t)

    def acceleration_at(self, t: float) -> float:
        """Get the acceleration at time t.

        For linear interpolation, acceleration is always zero
        (velocity is constant within each segment).

        Args:
            t: Time from start (seconds).

        Returns:
            Always 0.0 for linear interpolation.
        """
        return 0.0

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time t.

        Args:
            t: Time from start (seconds). Clamped to [0, total_time].

        Returns:
            TrajectoryPoint at time t.
        """
        t_clamped = max(0.0, min(t, self._total_time))
        return TrajectoryPoint(
            position=self.position_at(t_clamped),
            velocity=self.velocity_at(t_clamped),
            acceleration=0.0,
            time=t_clamped,
        )

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample the trajectory at n evenly-spaced points.

        Args:
            n: Number of points to sample. Must be >= 2.

        Returns:
            List of n TrajectoryPoints from start to end.

        Raises:
            ValueError: If n < 2.
        """
        if n < 2:
            raise ValueError(f"n must be >= 2, got {n}")

        if self._total_time == 0:
            return [self.sample(0.0)] * n

        dt = self._total_time / (n - 1)
        return [self.sample(i * dt) for i in range(n)]

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample the trajectory at regular time intervals.

        Args:
            dt: Time interval between samples (seconds). Must be > 0.

        Returns:
            List of TrajectoryPoints from start to end (inclusive).

        Raises:
            ValueError: If dt <= 0.
        """
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")

        points: list[TrajectoryPoint] = []
        t = 0.0
        while t <= self._total_time:
            points.append(self.sample(t))
            t += dt

        # Ensure we include the end point
        if points and points[-1].time < self._total_time:
            points.append(self.sample(self._total_time))

        return points

    def sample_at_waypoints(self) -> list[TrajectoryPoint]:
        """Sample the trajectory at each waypoint time.

        Returns:
            List of TrajectoryPoints, one at each waypoint.
        """
        return [self.sample(t) for t in self._times]

    def is_complete(self, t: float) -> bool:
        """Check if the trajectory is complete at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            True if t >= total_time.
        """
        return t >= self._total_time

    def progress(self, t: float) -> float:
        """Get the progress as a fraction [0, 1].

        Args:
            t: Time from start (seconds).

        Returns:
            Progress from 0.0 (start) to 1.0 (end), clamped.
        """
        if self._total_time == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._total_time))

    def reversed(self) -> Trajectory:
        """Return a reversed trajectory.

        Returns:
            New Trajectory with waypoints in reverse order.
        """
        reversed_waypoints = list(reversed(self._waypoints))
        # Recalculate times for reversed trajectory
        reversed_times = [0.0]
        for i in range(len(self._times) - 1, 0, -1):
            duration = self._times[i] - self._times[i - 1]
            reversed_times.append(reversed_times[-1] + duration)

        return Trajectory(
            waypoints=reversed_waypoints,
            times=reversed_times,
            default_velocity=self._default_velocity,
        )

    def scaled(self, time_scale: float) -> Trajectory:
        """Return a time-scaled trajectory.

        Args:
            time_scale: Factor to multiply all times by. Must be > 0.

        Returns:
            New Trajectory with scaled times.

        Raises:
            ValueError: If time_scale <= 0.
        """
        if time_scale <= 0:
            raise ValueError(f"time_scale must be > 0, got {time_scale}")

        scaled_times = [t * time_scale for t in self._times]
        return Trajectory(
            waypoints=list(self._waypoints),
            times=scaled_times,
            default_velocity=self._default_velocity / time_scale,
        )

    def __repr__(self) -> str:
        """Return a string representation."""
        return f"Trajectory({self.num_waypoints} waypoints, total_time={self._total_time:.3f}s)"


class TrajectoryProfile(Enum):
    """Motion profile type for trajectory generation.

    Different profiles offer tradeoffs between smoothness and speed:

    - TRAPEZOIDAL: Fastest, but has discontinuous acceleration (jerky).
    - S_CURVE: Smooth acceleration, limits jerk for smoother motion.
    - CUBIC: Third-order polynomial, smooth velocity but not acceleration.
    - QUINTIC: Fifth-order polynomial, smooth position, velocity, and acceleration.

    Use TRAPEZOIDAL for speed, S_CURVE or QUINTIC for smooth motion.
    """

    TRAPEZOIDAL = "trapezoidal"
    S_CURVE = "s_curve"
    CUBIC = "cubic"
    QUINTIC = "quintic"


@dataclass(frozen=True, slots=True)
class TrajectoryConstraints:
    """Motion constraints for trajectory generation.

    Attributes:
        max_velocity: Maximum velocity (positive). Required.
        max_acceleration: Maximum acceleration (positive). Required.
        max_jerk: Maximum jerk (positive). Optional, used for S-curve profiles.
    """

    max_velocity: float
    max_acceleration: float
    max_jerk: float | None = None

    def __post_init__(self) -> None:
        """Validate constraints."""
        if self.max_velocity <= 0:
            raise ValueError(f"max_velocity must be > 0, got {self.max_velocity}")
        if self.max_acceleration <= 0:
            raise ValueError(f"max_acceleration must be > 0, got {self.max_acceleration}")
        if self.max_jerk is not None and self.max_jerk <= 0:
            raise ValueError(f"max_jerk must be > 0, got {self.max_jerk}")


class TrajectoryGenerator:
    """Generate time-parameterized trajectories with motion profiles.

    Creates smooth trajectories from start to end positions using
    various motion profiles (trapezoidal, S-curve, polynomial).

    Attributes:
        profile: Motion profile type.
        constraints: Velocity, acceleration, and jerk limits.

    Example:
        >>> gen = TrajectoryGenerator(
        ...     profile=TrajectoryProfile.TRAPEZOIDAL,
        ...     constraints=TrajectoryConstraints(max_velocity=10.0, max_acceleration=5.0)
        ... )
        >>> traj = gen.generate(start=0.0, end=100.0)
        >>> traj.duration
        12.0
    """

    _profile: TrajectoryProfile
    _constraints: TrajectoryConstraints

    __slots__ = ("_constraints", "_profile")

    def __init__(
        self,
        profile: TrajectoryProfile = TrajectoryProfile.TRAPEZOIDAL,
        constraints: TrajectoryConstraints | None = None,
        max_velocity: float = 1.0,
        max_acceleration: float = 1.0,
        max_jerk: float | None = None,
    ) -> None:
        """Initialize the trajectory generator.

        Args:
            profile: Motion profile type.
            constraints: TrajectoryConstraints object. If provided, individual
                max_* parameters are ignored.
            max_velocity: Maximum velocity (used if constraints is None).
            max_acceleration: Maximum acceleration (used if constraints is None).
            max_jerk: Maximum jerk (used if constraints is None).
        """
        self._profile = profile
        if constraints is not None:
            self._constraints = constraints
        else:
            self._constraints = TrajectoryConstraints(
                max_velocity=max_velocity,
                max_acceleration=max_acceleration,
                max_jerk=max_jerk,
            )

    @property
    def profile(self) -> TrajectoryProfile:
        """Return the motion profile type."""
        return self._profile

    @property
    def constraints(self) -> TrajectoryConstraints:
        """Return the motion constraints."""
        return self._constraints

    def generate(
        self,
        start: float,
        end: float,
        duration: float | None = None,
    ) -> TrapezoidalProfile | CubicTrajectory | QuinticTrajectory | SCurveTrajectory:
        """Generate a trajectory from start to end.

        Args:
            start: Starting position.
            end: Ending position.
            duration: Optional fixed duration. If None, calculated from constraints.

        Returns:
            Trajectory object appropriate for the profile type.
        """
        if self._profile == TrajectoryProfile.TRAPEZOIDAL:
            return TrapezoidalProfile(
                start=start,
                end=end,
                max_velocity=self._constraints.max_velocity,
                max_acceleration=self._constraints.max_acceleration,
            )
        elif self._profile == TrajectoryProfile.CUBIC:
            calc_duration = duration if duration else self._calculate_duration(start, end)
            return CubicTrajectory(start=start, end=end, duration=calc_duration)
        elif self._profile == TrajectoryProfile.QUINTIC:
            calc_duration = duration if duration else self._calculate_duration(start, end)
            return QuinticTrajectory(start=start, end=end, duration=calc_duration)
        elif self._profile == TrajectoryProfile.S_CURVE:
            return SCurveTrajectory(
                start=start,
                end=end,
                max_velocity=self._constraints.max_velocity,
                max_acceleration=self._constraints.max_acceleration,
                max_jerk=self._constraints.max_jerk or self._constraints.max_acceleration * 10,
            )
        else:
            raise ValueError(f"Unknown profile: {self._profile}")

    def _calculate_duration(self, start: float, end: float) -> float:
        """Calculate minimum duration based on constraints.

        Uses trapezoidal profile timing as a baseline.
        """
        distance = abs(end - start)
        if distance == 0:
            return 0.1  # Minimum duration for zero distance

        # Time to accelerate to max velocity
        t_accel = self._constraints.max_velocity / self._constraints.max_acceleration
        # Distance during acceleration + deceleration
        d_accel = self._constraints.max_acceleration * t_accel**2  # Both phases

        if distance <= d_accel:
            # Triangular profile
            return 2 * math.sqrt(distance / self._constraints.max_acceleration)
        else:
            # Trapezoidal profile
            t_cruise = (distance - d_accel) / self._constraints.max_velocity
            return 2 * t_accel + t_cruise

    def generate_multi_axis(
        self,
        starts: list[float],
        ends: list[float],
        synchronized: bool = True,
    ) -> list[TrapezoidalProfile | CubicTrajectory | QuinticTrajectory | SCurveTrajectory]:
        """Generate trajectories for multiple axes.

        Args:
            starts: Starting positions for each axis.
            ends: Ending positions for each axis.
            synchronized: If True, all axes complete at the same time.

        Returns:
            List of trajectories, one per axis.

        Raises:
            ValueError: If starts and ends have different lengths.
        """
        if len(starts) != len(ends):
            raise ValueError(f"starts length {len(starts)} must match ends length {len(ends)}")

        if not starts:
            return []

        if synchronized:
            # Calculate the slowest axis duration
            max_duration = 0.0
            for start, end in zip(starts, ends, strict=True):
                traj = self.generate(start, end)
                max_duration = max(
                    max_duration, getattr(traj, "total_time", getattr(traj, "duration", 0.1))
                )

            # Generate all trajectories with the same duration
            return [
                self.generate(s, e, duration=max_duration)
                for s, e in zip(starts, ends, strict=True)
            ]
        else:
            # Independent timing
            return [self.generate(s, e) for s, e in zip(starts, ends, strict=True)]


class CubicTrajectory:
    """Cubic polynomial trajectory for smooth motion.

    Uses a third-degree polynomial to interpolate between start and end
    positions with zero velocity at endpoints.

    Position: p(t) = a0 + a1*t + a2*t^2 + a3*t^3

    Boundary conditions:
    - p(0) = start
    - p(T) = end
    - v(0) = 0
    - v(T) = 0

    Provides smooth velocity but discontinuous acceleration at endpoints.

    Attributes:
        start: Starting position.
        end: Ending position.
        duration: Total duration.
    """

    _start: float
    _end: float
    _duration: float
    _a0: float
    _a1: float
    _a2: float
    _a3: float

    __slots__ = ("_a0", "_a1", "_a2", "_a3", "_duration", "_end", "_start")

    def __init__(
        self,
        start: float,
        end: float,
        duration: float,
        start_velocity: float = 0.0,
        end_velocity: float = 0.0,
    ) -> None:
        """Initialize a cubic trajectory.

        Args:
            start: Starting position.
            end: Ending position.
            duration: Total duration. Must be > 0.
            start_velocity: Initial velocity (default 0).
            end_velocity: Final velocity (default 0).

        Raises:
            ValueError: If duration <= 0.
        """
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        self._start = start
        self._end = end
        self._duration = duration

        # Solve for cubic coefficients
        # p(t) = a0 + a1*t + a2*t^2 + a3*t^3
        # v(t) = a1 + 2*a2*t + 3*a3*t^2
        # Boundary conditions:
        # p(0) = start => a0 = start
        # v(0) = v0 => a1 = v0
        # p(T) = end => a0 + a1*T + a2*T^2 + a3*T^3 = end
        # v(T) = vT => a1 + 2*a2*T + 3*a3*T^2 = vT

        T = duration
        self._a0 = start
        self._a1 = start_velocity

        # Solve for a2, a3
        # From p(T): a2*T^2 + a3*T^3 = end - start - v0*T
        # From v(T): 2*a2*T + 3*a3*T^2 = vT - v0
        # => a2 = (3*(end-start) - (2*v0 + vT)*T) / T^2
        # => a3 = (2*(start-end) + (v0 + vT)*T) / T^3

        self._a2 = (3 * (end - start) - (2 * start_velocity + end_velocity) * T) / (T * T)
        self._a3 = (2 * (start - end) + (start_velocity + end_velocity) * T) / (T * T * T)

    @property
    def start(self) -> float:
        """Return the starting position."""
        return self._start

    @property
    def end(self) -> float:
        """Return the ending position."""
        return self._end

    @property
    def duration(self) -> float:
        """Return the duration (alias for total_time)."""
        return self._duration

    @property
    def total_time(self) -> float:
        """Return the total time."""
        return self._duration

    def position_at(self, t: float) -> float:
        """Get the position at time t.

        Args:
            t: Time from start (seconds). Clamped to [0, duration].

        Returns:
            Position at time t.
        """
        if t <= 0:
            return self._start
        if t >= self._duration:
            return self._end

        return self._a0 + self._a1 * t + self._a2 * t * t + self._a3 * t * t * t

    def velocity_at(self, t: float) -> float:
        """Get the velocity at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            Velocity at time t.
        """
        if t <= 0 or t >= self._duration:
            return 0.0

        return self._a1 + 2 * self._a2 * t + 3 * self._a3 * t * t

    def acceleration_at(self, t: float) -> float:
        """Get the acceleration at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            Acceleration at time t.
        """
        if t < 0 or t > self._duration:
            return 0.0

        return 2 * self._a2 + 6 * self._a3 * t

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time t.

        Args:
            t: Time from start (seconds).

        Returns:
            TrajectoryPoint at time t.
        """
        t_clamped = max(0.0, min(t, self._duration))
        return TrajectoryPoint(
            position=self.position_at(t_clamped),
            velocity=self.velocity_at(t_clamped),
            acceleration=self.acceleration_at(t_clamped),
            time=t_clamped,
        )

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample the trajectory at n evenly-spaced points.

        Args:
            n: Number of points. Must be >= 2.

        Returns:
            List of TrajectoryPoints.
        """
        if n < 2:
            raise ValueError(f"n must be >= 2, got {n}")

        dt = self._duration / (n - 1)
        return [self.sample(i * dt) for i in range(n)]

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample at regular time intervals.

        Args:
            dt: Time interval. Must be > 0.

        Returns:
            List of TrajectoryPoints.
        """
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")

        points: list[TrajectoryPoint] = []
        t = 0.0
        while t <= self._duration:
            points.append(self.sample(t))
            t += dt
        if points[-1].time < self._duration:
            points.append(self.sample(self._duration))
        return points

    def is_complete(self, t: float) -> bool:
        """Check if trajectory is complete at time t."""
        return t >= self._duration

    def progress(self, t: float) -> float:
        """Get progress as fraction [0, 1]."""
        if self._duration == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._duration))

    def __repr__(self) -> str:
        return (
            f"CubicTrajectory(start={self._start:.3f}, "
            f"end={self._end:.3f}, duration={self._duration:.3f}s)"
        )


class QuinticTrajectory:
    """Quintic polynomial trajectory for very smooth motion.

    Uses a fifth-degree polynomial to interpolate between start and end
    positions with zero velocity and acceleration at endpoints.

    Position: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

    Boundary conditions:
    - p(0) = start, p(T) = end
    - v(0) = 0, v(T) = 0
    - a(0) = 0, a(T) = 0

    Provides smooth position, velocity, AND acceleration.
    Ideal for applications requiring minimal jerk.

    Attributes:
        start: Starting position.
        end: Ending position.
        duration: Total duration.
    """

    _start: float
    _end: float
    _duration: float
    _a0: float
    _a1: float
    _a2: float
    _a3: float
    _a4: float
    _a5: float

    __slots__ = ("_a0", "_a1", "_a2", "_a3", "_a4", "_a5", "_duration", "_end", "_start")

    def __init__(
        self,
        start: float,
        end: float,
        duration: float,
        start_velocity: float = 0.0,
        end_velocity: float = 0.0,
        start_acceleration: float = 0.0,
        end_acceleration: float = 0.0,
    ) -> None:
        """Initialize a quintic trajectory.

        Args:
            start: Starting position.
            end: Ending position.
            duration: Total duration. Must be > 0.
            start_velocity: Initial velocity (default 0).
            end_velocity: Final velocity (default 0).
            start_acceleration: Initial acceleration (default 0).
            end_acceleration: Final acceleration (default 0).

        Raises:
            ValueError: If duration <= 0.
        """
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        self._start = start
        self._end = end
        self._duration = duration

        T = duration
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T

        # Coefficients from boundary conditions
        self._a0 = start
        self._a1 = start_velocity
        self._a2 = start_acceleration / 2

        # Solve for a3, a4, a5 using remaining boundary conditions
        # This involves solving a 3x3 system
        h = end - start
        v0, vT = start_velocity, end_velocity
        a0, aT = start_acceleration, end_acceleration

        self._a3 = (20 * h - (8 * vT + 12 * v0) * T - (3 * a0 - aT) * T2) / (2 * T3)
        self._a4 = (-30 * h + (14 * vT + 16 * v0) * T + (3 * a0 - 2 * aT) * T2) / (2 * T4)
        self._a5 = (12 * h - 6 * (vT + v0) * T + (aT - a0) * T2) / (2 * T5)

    @property
    def start(self) -> float:
        """Return the starting position."""
        return self._start

    @property
    def end(self) -> float:
        """Return the ending position."""
        return self._end

    @property
    def duration(self) -> float:
        """Return the duration (alias for total_time)."""
        return self._duration

    @property
    def total_time(self) -> float:
        """Return the total time."""
        return self._duration

    def position_at(self, t: float) -> float:
        """Get the position at time t."""
        if t <= 0:
            return self._start
        if t >= self._duration:
            return self._end

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        return (
            self._a0 + self._a1 * t + self._a2 * t2 + self._a3 * t3 + self._a4 * t4 + self._a5 * t5
        )

    def velocity_at(self, t: float) -> float:
        """Get the velocity at time t."""
        if t <= 0 or t >= self._duration:
            return 0.0

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        return (
            self._a1 + 2 * self._a2 * t + 3 * self._a3 * t2 + 4 * self._a4 * t3 + 5 * self._a5 * t4
        )

    def acceleration_at(self, t: float) -> float:
        """Get the acceleration at time t."""
        if t < 0 or t > self._duration:
            return 0.0

        t2 = t * t
        t3 = t2 * t
        return 2 * self._a2 + 6 * self._a3 * t + 12 * self._a4 * t2 + 20 * self._a5 * t3

    def jerk_at(self, t: float) -> float:
        """Get the jerk (rate of change of acceleration) at time t."""
        if t < 0 or t > self._duration:
            return 0.0

        t2 = t * t
        return 6 * self._a3 + 24 * self._a4 * t + 60 * self._a5 * t2

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time t."""
        t_clamped = max(0.0, min(t, self._duration))
        return TrajectoryPoint(
            position=self.position_at(t_clamped),
            velocity=self.velocity_at(t_clamped),
            acceleration=self.acceleration_at(t_clamped),
            time=t_clamped,
        )

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample at n evenly-spaced points."""
        if n < 2:
            raise ValueError(f"n must be >= 2, got {n}")
        dt = self._duration / (n - 1)
        return [self.sample(i * dt) for i in range(n)]

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample at regular time intervals."""
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
        points: list[TrajectoryPoint] = []
        t = 0.0
        while t <= self._duration:
            points.append(self.sample(t))
            t += dt
        if points[-1].time < self._duration:
            points.append(self.sample(self._duration))
        return points

    def is_complete(self, t: float) -> bool:
        """Check if trajectory is complete."""
        return t >= self._duration

    def progress(self, t: float) -> float:
        """Get progress as fraction [0, 1]."""
        if self._duration == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._duration))

    def __repr__(self) -> str:
        return (
            f"QuinticTrajectory(start={self._start:.3f}, "
            f"end={self._end:.3f}, duration={self._duration:.3f}s)"
        )


class SCurveTrajectory:
    """S-curve (jerk-limited) trajectory for ultra-smooth motion.

    An S-curve profile limits not just velocity and acceleration,
    but also jerk (rate of change of acceleration). This produces
    the smoothest possible motion for high-precision applications.

    The profile has 7 phases:
    1. Jerk-up (positive jerk, increasing acceleration)
    2. Constant acceleration
    3. Jerk-down (negative jerk, decreasing acceleration to zero)
    4. Cruise (constant velocity)
    5. Jerk-down (negative jerk, increasing deceleration)
    6. Constant deceleration
    7. Jerk-up (positive jerk, decreasing deceleration to zero)

    For short moves, some phases may be omitted.

    Attributes:
        start: Starting position.
        end: Ending position.
        max_velocity: Maximum velocity.
        max_acceleration: Maximum acceleration.
        max_jerk: Maximum jerk.
    """

    _start: float
    _end: float
    _max_velocity: float
    _max_acceleration: float
    _max_jerk: float
    _direction: float
    _distance: float
    _total_time: float
    _phases: list[tuple[float, float, float, float]]  # (duration, jerk, accel_start, vel_start)

    __slots__ = (
        "_direction",
        "_distance",
        "_end",
        "_max_acceleration",
        "_max_jerk",
        "_max_velocity",
        "_phases",
        "_start",
        "_total_time",
    )

    def __init__(
        self,
        start: float,
        end: float,
        max_velocity: float,
        max_acceleration: float,
        max_jerk: float,
    ) -> None:
        """Initialize an S-curve trajectory.

        Args:
            start: Starting position.
            end: Ending position.
            max_velocity: Maximum velocity. Must be > 0.
            max_acceleration: Maximum acceleration. Must be > 0.
            max_jerk: Maximum jerk. Must be > 0.

        Raises:
            ValueError: If any constraint <= 0.
        """
        if max_velocity <= 0:
            raise ValueError(f"max_velocity must be > 0, got {max_velocity}")
        if max_acceleration <= 0:
            raise ValueError(f"max_acceleration must be > 0, got {max_acceleration}")
        if max_jerk <= 0:
            raise ValueError(f"max_jerk must be > 0, got {max_jerk}")

        self._start = start
        self._end = end
        self._max_velocity = max_velocity
        self._max_acceleration = max_acceleration
        self._max_jerk = max_jerk

        self._distance = abs(end - start)
        self._direction = 1.0 if end >= start else -1.0

        self._calculate_phases()

    def _calculate_phases(self) -> None:
        """Calculate the 7-phase S-curve profile."""
        # Time to reach max acceleration
        t_j = self._max_acceleration / self._max_jerk

        # Velocity reached during jerk phase
        v_j = 0.5 * self._max_jerk * t_j * t_j

        # Time at constant acceleration to reach max velocity
        if self._max_velocity > 2 * v_j:
            t_a = (self._max_velocity - 2 * v_j) / self._max_acceleration
        else:
            # Can't reach max velocity, triangular acceleration profile
            t_a = 0.0
            # Recalculate t_j for achievable velocity
            t_j = math.sqrt(self._max_velocity / self._max_jerk)
            v_j = 0.5 * self._max_velocity

        # Distance covered during acceleration phase (phases 1-3)
        # d_accel = integral of v(t) over phases 1, 2, 3
        d_j1 = self._max_jerk * t_j**3 / 6  # Phase 1: jerk up
        v_after_j1 = v_j
        a_after_j1 = self._max_acceleration if t_a > 0 else self._max_jerk * t_j

        d_a = v_after_j1 * t_a + 0.5 * a_after_j1 * t_a**2  # Phase 2
        v_after_a = v_after_j1 + a_after_j1 * t_a

        d_j2 = v_after_a * t_j + 0.5 * a_after_j1 * t_j**2 - self._max_jerk * t_j**3 / 6  # Phase 3

        d_accel = d_j1 + d_a + d_j2

        # Cruise distance
        d_cruise = self._distance - 2 * d_accel
        if d_cruise < 0:
            # Need to reduce peak velocity - simplified handling
            # Scale down all phases proportionally
            scale = math.sqrt(self._distance / (2 * d_accel)) if d_accel > 0 else 1.0
            t_j *= scale
            t_a *= scale
            d_cruise = 0.0

        # Cruise time
        v_cruise = self._max_velocity if t_a > 0 else 2 * v_j
        t_cruise = d_cruise / v_cruise if v_cruise > 0 else 0.0

        # Build phases: (duration, jerk, initial_accel, initial_vel)
        self._phases = []
        current_v = 0.0
        current_a = 0.0

        # Phase 1: Jerk up
        self._phases.append((t_j, self._max_jerk, current_a, current_v))
        current_v = v_j
        current_a = self._max_acceleration if t_a > 0 else self._max_jerk * t_j

        # Phase 2: Constant acceleration (may be 0 duration)
        if t_a > 0:
            self._phases.append((t_a, 0.0, current_a, current_v))
            current_v = current_v + current_a * t_a

        # Phase 3: Jerk down
        self._phases.append((t_j, -self._max_jerk, current_a, current_v))
        current_v = v_cruise
        current_a = 0.0

        # Phase 4: Cruise
        if t_cruise > 0:
            self._phases.append((t_cruise, 0.0, 0.0, current_v))

        # Phase 5: Jerk down (negative accel)
        self._phases.append((t_j, -self._max_jerk, 0.0, current_v))
        current_a = -self._max_acceleration if t_a > 0 else -self._max_jerk * t_j
        current_v = current_v + 0.5 * current_a * t_j  # Approx

        # Phase 6: Constant deceleration
        if t_a > 0:
            self._phases.append((t_a, 0.0, current_a, current_v))
            current_v = current_v + current_a * t_a

        # Phase 7: Jerk up (to zero accel)
        self._phases.append((t_j, self._max_jerk, current_a, current_v))

        self._total_time = sum(p[0] for p in self._phases)

    @property
    def start(self) -> float:
        """Return the starting position."""
        return self._start

    @property
    def end(self) -> float:
        """Return the ending position."""
        return self._end

    @property
    def duration(self) -> float:
        """Return the duration."""
        return self._total_time

    @property
    def total_time(self) -> float:
        """Return the total time."""
        return self._total_time

    def _get_phase_state(self, t: float) -> tuple[float, float, float, float]:
        """Get position, velocity, acceleration at time t within phases.

        Returns:
            (position, velocity, acceleration, jerk) at time t.
        """
        if t <= 0:
            return (0.0, 0.0, 0.0, 0.0)

        # Find which phase we're in
        elapsed = 0.0
        pos = 0.0
        vel = 0.0
        accel = 0.0

        for duration, jerk, a0, v0 in self._phases:
            if elapsed + duration >= t:
                # We're in this phase
                dt = t - elapsed
                accel = a0 + jerk * dt
                vel = v0 + a0 * dt + 0.5 * jerk * dt * dt
                pos += v0 * dt + 0.5 * a0 * dt * dt + jerk * dt * dt * dt / 6
                return (pos, vel, accel, jerk)

            # Complete this phase
            dt = duration
            pos += v0 * dt + 0.5 * a0 * dt * dt + jerk * dt * dt * dt / 6
            vel = v0 + a0 * dt + 0.5 * jerk * dt * dt
            accel = a0 + jerk * dt
            elapsed += duration

        return (self._distance, 0.0, 0.0, 0.0)

    def position_at(self, t: float) -> float:
        """Get position at time t."""
        if t <= 0:
            return self._start
        if t >= self._total_time:
            return self._end

        pos, _, _, _ = self._get_phase_state(t)
        return self._start + self._direction * pos

    def velocity_at(self, t: float) -> float:
        """Get velocity at time t."""
        if t <= 0 or t >= self._total_time:
            return 0.0

        _, vel, _, _ = self._get_phase_state(t)
        return self._direction * vel

    def acceleration_at(self, t: float) -> float:
        """Get acceleration at time t."""
        if t < 0 or t > self._total_time:
            return 0.0

        _, _, accel, _ = self._get_phase_state(t)
        return self._direction * accel

    def jerk_at(self, t: float) -> float:
        """Get jerk at time t."""
        if t < 0 or t > self._total_time:
            return 0.0

        _, _, _, jerk = self._get_phase_state(t)
        return self._direction * jerk

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time t."""
        t_clamped = max(0.0, min(t, self._total_time))
        return TrajectoryPoint(
            position=self.position_at(t_clamped),
            velocity=self.velocity_at(t_clamped),
            acceleration=self.acceleration_at(t_clamped),
            time=t_clamped,
        )

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample at n evenly-spaced points."""
        if n < 2:
            raise ValueError(f"n must be >= 2, got {n}")
        dt = self._total_time / (n - 1)
        return [self.sample(i * dt) for i in range(n)]

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample at regular time intervals."""
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
        points: list[TrajectoryPoint] = []
        t = 0.0
        while t <= self._total_time:
            points.append(self.sample(t))
            t += dt
        if points[-1].time < self._total_time:
            points.append(self.sample(self._total_time))
        return points

    def is_complete(self, t: float) -> bool:
        """Check if trajectory is complete."""
        return t >= self._total_time

    def progress(self, t: float) -> float:
        """Get progress as fraction [0, 1]."""
        if self._total_time == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._total_time))

    def __repr__(self) -> str:
        return (
            f"SCurveTrajectory(start={self._start:.3f}, "
            f"end={self._end:.3f}, duration={self._total_time:.3f}s)"
        )


class SplineTrajectory:
    """Smooth trajectory through multiple waypoints using cubic splines.

    Creates a continuous smooth path through multiple waypoints with
    continuous velocity at each waypoint. Uses natural cubic splines
    for interpolation.

    Attributes:
        waypoints: List of waypoint positions.
        times: Times at which to reach each waypoint.
    """

    _waypoints: list[float]
    _times: list[float]
    _coefficients: list[tuple[float, float, float, float]]  # (a, b, c, d) for each segment
    _total_time: float

    __slots__ = ("_coefficients", "_times", "_total_time", "_waypoints")

    def __init__(
        self,
        waypoints: list[float],
        times: list[float] | None = None,
        default_velocity: float = 1.0,
    ) -> None:
        """Initialize a spline trajectory.

        Args:
            waypoints: List of positions. Must have at least 2 points.
            times: Optional times for each waypoint. If None, calculated
                from default_velocity.
            default_velocity: Used to calculate times if not provided.

        Raises:
            ValueError: If waypoints has fewer than 2 points, or times
                don't match waypoints length.
        """
        if len(waypoints) < 2:
            raise ValueError(f"Need at least 2 waypoints, got {len(waypoints)}")

        self._waypoints = list(waypoints)

        if times is not None:
            if len(times) != len(waypoints):
                raise ValueError(f"times length {len(times)} must match waypoints {len(waypoints)}")
            self._times = list(times)
        else:
            # Calculate times from distances
            self._times = [0.0]
            for i in range(1, len(waypoints)):
                dist = abs(waypoints[i] - waypoints[i - 1])
                dt = dist / default_velocity if dist > 0 else 0.1
                self._times.append(self._times[-1] + dt)

        self._total_time = self._times[-1]
        self._coefficients = self._compute_spline_coefficients()

    def _compute_spline_coefficients(self) -> list[tuple[float, float, float, float]]:
        """Compute natural cubic spline coefficients.

        Uses the Thomas algorithm for tridiagonal systems.

        Returns:
            List of (a, b, c, d) coefficients for each segment.
        """
        n = len(self._waypoints)
        if n < 2:
            return []

        # For natural cubic spline, we solve for second derivatives
        # at each point, then compute the polynomial coefficients.

        # Segment lengths (h_i = t_{i+1} - t_i)
        h = [self._times[i + 1] - self._times[i] for i in range(n - 1)]

        # Avoid division by zero
        h = [max(hi, 1e-10) for hi in h]

        if n == 2:
            # Linear interpolation for 2 points
            a = self._waypoints[0]
            d = (self._waypoints[1] - self._waypoints[0]) / h[0]
            return [(a, 0.0, 0.0, d)]

        # Set up tridiagonal system for natural spline
        # Solve for M[i] = second derivatives at each point

        # Diagonal elements
        diag = [1.0] + [2 * (h[i - 1] + h[i]) for i in range(1, n - 1)] + [1.0]
        # Off-diagonal elements
        off_diag = [0.0] + [h[i] for i in range(1, n - 1)]
        sub_diag = [h[i - 1] for i in range(1, n - 1)] + [0.0]

        # Right-hand side
        rhs = [0.0]  # Natural spline: M[0] = 0
        for i in range(1, n - 1):
            rhs.append(
                6
                * (
                    (self._waypoints[i + 1] - self._waypoints[i]) / h[i]
                    - (self._waypoints[i] - self._waypoints[i - 1]) / h[i - 1]
                )
            )
        rhs.append(0.0)  # Natural spline: M[n-1] = 0

        # Thomas algorithm (forward elimination)
        for i in range(1, n):
            if diag[i - 1] == 0:
                continue
            factor = sub_diag[i - 1] / diag[i - 1]
            diag[i] -= factor * off_diag[i - 1]
            rhs[i] -= factor * rhs[i - 1]

        # Back substitution
        M = [0.0] * n
        M[n - 1] = rhs[n - 1] / diag[n - 1] if diag[n - 1] != 0 else 0.0
        for i in range(n - 2, -1, -1):
            if diag[i] != 0:
                M[i] = (rhs[i] - off_diag[i] * M[i + 1]) / diag[i]

        # Compute polynomial coefficients for each segment
        # p_i(t) = a_i + b_i*(t-t_i) + c_i*(t-t_i)^2 + d_i*(t-t_i)^3
        coefficients: list[tuple[float, float, float, float]] = []
        for i in range(n - 1):
            a = self._waypoints[i]
            c = M[i] / 2
            d = (M[i + 1] - M[i]) / (6 * h[i])
            b = (self._waypoints[i + 1] - self._waypoints[i]) / h[i] - h[i] * (
                2 * M[i] + M[i + 1]
            ) / 6
            coefficients.append((a, b, c, d))

        return coefficients

    @property
    def waypoints(self) -> list[float]:
        """Return a copy of waypoints."""
        return list(self._waypoints)

    @property
    def times(self) -> list[float]:
        """Return a copy of times."""
        return list(self._times)

    @property
    def total_time(self) -> float:
        """Return total duration."""
        return self._total_time

    @property
    def duration(self) -> float:
        """Return duration (alias)."""
        return self._total_time

    @property
    def num_segments(self) -> int:
        """Return number of segments."""
        return len(self._coefficients)

    def _find_segment(self, t: float) -> tuple[int, float]:
        """Find segment index and local time for global time t."""
        if t <= 0:
            return (0, 0.0)
        if t >= self._total_time:
            return (len(self._coefficients) - 1, self._times[-1] - self._times[-2])

        for i in range(len(self._times) - 1):
            if self._times[i] <= t < self._times[i + 1]:
                return (i, t - self._times[i])

        return (len(self._coefficients) - 1, self._times[-1] - self._times[-2])

    def position_at(self, t: float) -> float:
        """Get position at time t."""
        if t <= 0:
            return self._waypoints[0]
        if t >= self._total_time:
            return self._waypoints[-1]

        seg_idx, dt = self._find_segment(t)
        a, b, c, d = self._coefficients[seg_idx]
        return a + b * dt + c * dt * dt + d * dt * dt * dt

    def velocity_at(self, t: float) -> float:
        """Get velocity at time t."""
        if t <= 0 or t >= self._total_time:
            return 0.0

        seg_idx, dt = self._find_segment(t)
        _, b, c, d = self._coefficients[seg_idx]
        return b + 2 * c * dt + 3 * d * dt * dt

    def acceleration_at(self, t: float) -> float:
        """Get acceleration at time t."""
        if t < 0 or t > self._total_time:
            return 0.0

        seg_idx, dt = self._find_segment(t)
        _, _, c, d = self._coefficients[seg_idx]
        return 2 * c + 6 * d * dt

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time t."""
        t_clamped = max(0.0, min(t, self._total_time))
        return TrajectoryPoint(
            position=self.position_at(t_clamped),
            velocity=self.velocity_at(t_clamped),
            acceleration=self.acceleration_at(t_clamped),
            time=t_clamped,
        )

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample at n evenly-spaced points."""
        if n < 2:
            raise ValueError(f"n must be >= 2, got {n}")
        dt = self._total_time / (n - 1)
        return [self.sample(i * dt) for i in range(n)]

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample at regular time intervals."""
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
        points: list[TrajectoryPoint] = []
        t = 0.0
        while t <= self._total_time:
            points.append(self.sample(t))
            t += dt
        if points[-1].time < self._total_time:
            points.append(self.sample(self._total_time))
        return points

    def is_complete(self, t: float) -> bool:
        """Check if complete."""
        return t >= self._total_time

    def progress(self, t: float) -> float:
        """Get progress [0, 1]."""
        if self._total_time == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._total_time))

    def __repr__(self) -> str:
        return (
            f"SplineTrajectory({len(self._waypoints)} waypoints, duration={self._total_time:.3f}s)"
        )


@dataclass(slots=True)
class BlendSegment:
    """A segment in a blended trajectory.

    Attributes:
        target: Target position for this segment.
        duration: Duration to reach target.
        blend_radius: Blend radius at end of segment.
    """

    target: float
    duration: float
    blend_radius: float = 0.0


class BlendedTrajectory:
    """Blend between trajectory segments without stopping.

    Creates smooth transitions between segments by blending
    near waypoints instead of coming to a complete stop.
    This allows for faster motion through waypoints.

    Attributes:
        start: Starting position.
        segments: List of segments to execute.
        default_blend_radius: Default blend radius for segments.
    """

    _start: float
    _segments: list[BlendSegment]
    _default_blend_radius: float
    _total_time: float
    _compiled_trajectory: SplineTrajectory | None

    __slots__ = (
        "_compiled_trajectory",
        "_default_blend_radius",
        "_segments",
        "_start",
        "_total_time",
    )

    def __init__(
        self,
        start: float,
        default_blend_radius: float = 0.1,
    ) -> None:
        """Initialize a blended trajectory.

        Args:
            start: Starting position.
            default_blend_radius: Default blend radius for new segments.
        """
        self._start = start
        self._segments = []
        self._default_blend_radius = default_blend_radius
        self._total_time = 0.0
        self._compiled_trajectory = None

    @property
    def start(self) -> float:
        """Return starting position."""
        return self._start

    @property
    def num_segments(self) -> int:
        """Return number of segments."""
        return len(self._segments)

    @property
    def total_time(self) -> float:
        """Return total duration."""
        return self._total_time

    @property
    def duration(self) -> float:
        """Return duration (alias)."""
        return self._total_time

    def add_segment(
        self,
        target: float,
        duration: float,
        blend_radius: float | None = None,
    ) -> BlendedTrajectory:
        """Add a segment to the trajectory.

        Args:
            target: Target position.
            duration: Duration for this segment.
            blend_radius: Blend radius. If None, uses default.

        Returns:
            Self for chaining.
        """
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        br = blend_radius if blend_radius is not None else self._default_blend_radius
        self._segments.append(BlendSegment(target=target, duration=duration, blend_radius=br))
        self._total_time += duration
        self._compiled_trajectory = None  # Invalidate cache
        return self

    def set_blend_radius(self, radius: float) -> BlendedTrajectory:
        """Set the default blend radius for future segments.

        Args:
            radius: New default blend radius.

        Returns:
            Self for chaining.
        """
        if radius < 0:
            raise ValueError(f"radius must be >= 0, got {radius}")
        self._default_blend_radius = radius
        return self

    def _compile(self) -> SplineTrajectory:
        """Compile segments into a spline trajectory."""
        if not self._segments:
            return SplineTrajectory([self._start, self._start], [0.0, 0.1])

        # Build waypoints and times
        waypoints = [self._start]
        times = [0.0]
        current_time = 0.0

        for segment in self._segments:
            current_time += segment.duration
            waypoints.append(segment.target)
            times.append(current_time)

        return SplineTrajectory(waypoints, times)

    def compile(self) -> BlendedTrajectory:
        """Force recompilation of the trajectory.

        Returns:
            Self for chaining.
        """
        self._compiled_trajectory = self._compile()
        return self

    def _get_trajectory(self) -> SplineTrajectory:
        """Get or create the compiled trajectory."""
        if self._compiled_trajectory is None:
            self._compiled_trajectory = self._compile()
        return self._compiled_trajectory

    def position_at(self, t: float) -> float:
        """Get position at time t."""
        return self._get_trajectory().position_at(t)

    def velocity_at(self, t: float) -> float:
        """Get velocity at time t."""
        return self._get_trajectory().velocity_at(t)

    def acceleration_at(self, t: float) -> float:
        """Get acceleration at time t."""
        return self._get_trajectory().acceleration_at(t)

    def sample(self, t: float) -> TrajectoryPoint:
        """Sample at time t."""
        return self._get_trajectory().sample(t)

    def sample_n(self, n: int) -> list[TrajectoryPoint]:
        """Sample at n points."""
        return self._get_trajectory().sample_n(n)

    def sample_dt(self, dt: float) -> list[TrajectoryPoint]:
        """Sample at time intervals."""
        return self._get_trajectory().sample_dt(dt)

    def is_complete(self, t: float) -> bool:
        """Check if complete."""
        return t >= self._total_time

    def progress(self, t: float) -> float:
        """Get progress [0, 1]."""
        if self._total_time == 0:
            return 1.0
        return max(0.0, min(1.0, t / self._total_time))

    def clear(self) -> BlendedTrajectory:
        """Clear all segments.

        Returns:
            Self for chaining.
        """
        self._segments = []
        self._total_time = 0.0
        self._compiled_trajectory = None
        return self

    def __repr__(self) -> str:
        return (
            f"BlendedTrajectory({len(self._segments)} segments, duration={self._total_time:.3f}s)"
        )
