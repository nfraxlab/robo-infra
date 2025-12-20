"""Trajectory generation for smooth motion profiles.

This module provides trajectory planning primitives for generating
smooth, physically realizable motion profiles. It supports:

- Point-to-point trajectories
- Linear interpolation
- Trapezoidal velocity profiles
- S-curve (jerk-limited) profiles

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
"""

from __future__ import annotations

from dataclasses import dataclass, field


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
            raise ValueError(f"Axis mismatch: positions has {pos_keys}, accelerations has {acc_keys}")

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
                    f"times length ({len(times)}) must match "
                    f"waypoints length ({len(waypoints)})"
                )
            # Validate monotonically increasing
            for i in range(1, len(times)):
                if times[i] <= times[i - 1]:
                    raise ValueError(
                        f"times must be monotonically increasing, "
                        f"but times[{i}]={times[i]} <= times[{i-1}]={times[i-1]}"
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
                raise ValueError(
                    f"time ({time}) must be > last waypoint time ({self._times[-1]})"
                )
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
            return (max(0, len(self._segments) - 1), self._segments[-1].duration if self._segments else 0.0)

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
        return (
            f"Trajectory({self.num_waypoints} waypoints, "
            f"total_time={self._total_time:.3f}s)"
        )
