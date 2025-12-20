"""Unit tests for trajectory generation classes.

Tests for TrajectoryPoint, MultiAxisTrajectoryPoint, LinearInterpolator,
TrapezoidalProfile, and Trajectory classes.
"""

from __future__ import annotations

import pytest

from robo_infra.motion.trajectory import (
    LinearInterpolator,
    MultiAxisTrajectoryPoint,
    Trajectory,
    TrajectoryPoint,
    TrapezoidalProfile,
)


# =============================================================================
# TrajectoryPoint Tests
# =============================================================================


class TestTrajectoryPoint:
    """Tests for TrajectoryPoint dataclass."""

    def test_trajectory_point_creation(self) -> None:
        """Test basic TrajectoryPoint creation."""
        point = TrajectoryPoint(position=10.0, velocity=5.0, acceleration=1.0, time=0.5)
        assert point.position == 10.0
        assert point.velocity == 5.0
        assert point.acceleration == 1.0
        assert point.time == 0.5
        assert point.jerk == 0.0  # Default

    def test_trajectory_point_with_jerk(self) -> None:
        """Test TrajectoryPoint with jerk value."""
        point = TrajectoryPoint(
            position=10.0, velocity=5.0, acceleration=1.0, time=0.5, jerk=0.1
        )
        assert point.jerk == 0.1

    def test_trajectory_point_negative_time_raises(self) -> None:
        """Test that negative time raises ValueError."""
        with pytest.raises(ValueError, match="time must be >= 0"):
            TrajectoryPoint(position=0.0, velocity=0.0, acceleration=0.0, time=-1.0)

    def test_trajectory_point_is_stationary(self) -> None:
        """Test is_stationary property."""
        stationary = TrajectoryPoint(position=10.0, velocity=0.0, acceleration=0.0, time=1.0)
        moving = TrajectoryPoint(position=10.0, velocity=5.0, acceleration=0.0, time=1.0)

        assert stationary.is_stationary is True
        assert moving.is_stationary is False

    def test_trajectory_point_kinetic_energy_factor(self) -> None:
        """Test kinetic_energy_factor property."""
        point = TrajectoryPoint(position=10.0, velocity=5.0, acceleration=0.0, time=1.0)
        assert point.kinetic_energy_factor == 25.0  # 5^2

    def test_trajectory_point_scaled(self) -> None:
        """Test scaled() method."""
        point = TrajectoryPoint(position=10.0, velocity=5.0, acceleration=2.0, time=1.0)
        scaled = point.scaled(position_scale=2.0, time_scale=2.0)

        assert scaled.position == 20.0  # 10 * 2
        assert scaled.velocity == 5.0  # 5 * 2 / 2
        assert scaled.acceleration == 1.0  # 2 * 2 / 4
        assert scaled.time == 2.0  # 1 * 2

    def test_trajectory_point_offset(self) -> None:
        """Test offset() method."""
        point = TrajectoryPoint(position=10.0, velocity=5.0, acceleration=2.0, time=1.0)
        offset = point.offset(position_offset=5.0, time_offset=0.5)

        assert offset.position == 15.0
        assert offset.velocity == 5.0  # Unchanged
        assert offset.time == 1.5

    def test_trajectory_point_offset_negative_time_raises(self) -> None:
        """Test offset with negative resulting time raises."""
        point = TrajectoryPoint(position=10.0, velocity=5.0, acceleration=2.0, time=1.0)
        with pytest.raises(ValueError, match="Resulting time would be negative"):
            point.offset(time_offset=-2.0)

    def test_trajectory_point_repr(self) -> None:
        """Test string representation."""
        point = TrajectoryPoint(position=10.0, velocity=5.0, acceleration=2.0, time=1.0)
        repr_str = repr(point)
        assert "TrajectoryPoint" in repr_str
        assert "pos=10.000" in repr_str


# =============================================================================
# MultiAxisTrajectoryPoint Tests
# =============================================================================


class TestMultiAxisTrajectoryPoint:
    """Tests for MultiAxisTrajectoryPoint dataclass."""

    def test_multi_axis_creation(self) -> None:
        """Test basic MultiAxisTrajectoryPoint creation."""
        point = MultiAxisTrajectoryPoint(
            positions={"x": 10.0, "y": 20.0},
            velocities={"x": 1.0, "y": 2.0},
            accelerations={"x": 0.1, "y": 0.2},
            time=1.0,
        )
        assert point.positions["x"] == 10.0
        assert point.velocities["y"] == 2.0
        assert point.time == 1.0

    def test_multi_axis_axes_property(self) -> None:
        """Test axes property."""
        point = MultiAxisTrajectoryPoint(
            positions={"x": 0.0, "y": 0.0, "z": 0.0},
            velocities={"x": 0.0, "y": 0.0, "z": 0.0},
            accelerations={"x": 0.0, "y": 0.0, "z": 0.0},
            time=0.0,
        )
        assert set(point.axes) == {"x", "y", "z"}
        assert point.num_axes == 3

    def test_multi_axis_get_axis(self) -> None:
        """Test get_axis method returns TrajectoryPoint."""
        point = MultiAxisTrajectoryPoint(
            positions={"x": 10.0, "y": 20.0},
            velocities={"x": 1.0, "y": 2.0},
            accelerations={"x": 0.1, "y": 0.2},
            time=1.5,
        )
        x_point = point.get_axis("x")
        assert isinstance(x_point, TrajectoryPoint)
        assert x_point.position == 10.0
        assert x_point.velocity == 1.0
        assert x_point.time == 1.5

    def test_multi_axis_mismatched_keys_raises(self) -> None:
        """Test mismatched axis keys raises ValueError."""
        with pytest.raises(ValueError, match="Axis mismatch"):
            MultiAxisTrajectoryPoint(
                positions={"x": 0.0, "y": 0.0},
                velocities={"x": 0.0},  # Missing y
                accelerations={"x": 0.0, "y": 0.0},
                time=0.0,
            )

    def test_multi_axis_is_stationary(self) -> None:
        """Test is_stationary method."""
        stationary = MultiAxisTrajectoryPoint(
            positions={"x": 10.0, "y": 20.0},
            velocities={"x": 0.0, "y": 0.0},
            accelerations={"x": 0.0, "y": 0.0},
            time=1.0,
        )
        moving = MultiAxisTrajectoryPoint(
            positions={"x": 10.0, "y": 20.0},
            velocities={"x": 1.0, "y": 0.0},
            accelerations={"x": 0.0, "y": 0.0},
            time=1.0,
        )
        assert stationary.is_stationary() is True
        assert moving.is_stationary() is False


# =============================================================================
# LinearInterpolator Tests
# =============================================================================


class TestLinearInterpolator:
    """Tests for LinearInterpolator class."""

    def test_linear_start_position(self) -> None:
        """Test position at t=0 is start."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.position_at(0.0) == 0.0

    def test_linear_end_position(self) -> None:
        """Test position at t=duration is end."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.position_at(2.0) == 100.0

    def test_linear_midpoint(self) -> None:
        """Test position at midpoint is halfway."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.position_at(1.0) == 50.0

    def test_linear_constant_velocity(self) -> None:
        """Test velocity is constant throughout."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.velocity_at(0.5) == 50.0
        assert interp.velocity_at(1.0) == 50.0
        assert interp.velocity_at(1.5) == 50.0

    def test_linear_velocity_property(self) -> None:
        """Test velocity property matches calculated velocity."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.velocity == 50.0

    def test_linear_acceleration_is_zero(self) -> None:
        """Test acceleration is always zero for linear interpolation."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.acceleration_at(0.0) == 0.0
        assert interp.acceleration_at(1.0) == 0.0
        assert interp.acceleration_at(2.0) == 0.0

    def test_linear_is_complete(self) -> None:
        """Test is_complete method."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.is_complete(1.5) is False
        assert interp.is_complete(2.0) is True
        assert interp.is_complete(3.0) is True

    def test_linear_progress(self) -> None:
        """Test progress method."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.progress(0.0) == 0.0
        assert interp.progress(1.0) == 0.5
        assert interp.progress(2.0) == 1.0

    def test_linear_sample(self) -> None:
        """Test sample method returns TrajectoryPoint."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        point = interp.sample(1.0)
        assert isinstance(point, TrajectoryPoint)
        assert point.position == 50.0
        assert point.velocity == 50.0
        assert point.time == 1.0

    def test_linear_sample_n(self) -> None:
        """Test sample_n method."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        points = interp.sample_n(3)
        assert len(points) == 3
        assert points[0].position == 0.0
        assert points[1].position == 50.0
        assert points[2].position == 100.0

    def test_linear_sample_n_less_than_2_raises(self) -> None:
        """Test sample_n with n < 2 raises."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        with pytest.raises(ValueError, match="n must be >= 2"):
            interp.sample_n(1)

    def test_linear_sample_dt(self) -> None:
        """Test sample_dt method."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        points = interp.sample_dt(1.0)
        assert len(points) == 3  # 0, 1, 2
        assert points[0].time == 0.0
        assert points[1].time == 1.0
        assert points[2].time == 2.0

    def test_linear_reversed(self) -> None:
        """Test reversed method."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        rev = interp.reversed()
        assert rev.start == 100.0
        assert rev.end == 0.0
        assert rev.duration == 2.0

    def test_linear_scaled(self) -> None:
        """Test scaled method."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        scaled = interp.scaled(2.0)
        assert scaled.duration == 4.0
        assert scaled.velocity == 25.0  # Half the original

    def test_linear_negative_duration_raises(self) -> None:
        """Test negative duration raises ValueError."""
        with pytest.raises(ValueError, match="duration must be > 0"):
            LinearInterpolator(start=0.0, end=100.0, duration=-1.0)

    def test_linear_zero_duration_raises(self) -> None:
        """Test zero duration raises ValueError."""
        with pytest.raises(ValueError, match="duration must be > 0"):
            LinearInterpolator(start=0.0, end=100.0, duration=0.0)

    def test_linear_negative_direction(self) -> None:
        """Test interpolation in negative direction."""
        interp = LinearInterpolator(start=100.0, end=0.0, duration=2.0)
        assert interp.position_at(1.0) == 50.0
        assert interp.velocity == -50.0

    def test_linear_clamping_before_start(self) -> None:
        """Test position is clamped when t < 0."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.position_at(-1.0) == 0.0

    def test_linear_clamping_after_end(self) -> None:
        """Test position is clamped when t > duration."""
        interp = LinearInterpolator(start=0.0, end=100.0, duration=2.0)
        assert interp.position_at(5.0) == 100.0


# =============================================================================
# TrapezoidalProfile Tests
# =============================================================================


class TestTrapezoidalProfile:
    """Tests for TrapezoidalProfile class."""

    def test_trapezoidal_start_end(self) -> None:
        """Test position at start and end."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        assert profile.position_at(0.0) == 0.0
        assert profile.position_at(profile.total_time) == pytest.approx(100.0, abs=0.01)

    def test_trapezoidal_peak_velocity(self) -> None:
        """Test peak velocity equals max_velocity for full profile."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        # Find the peak velocity during cruise
        mid_time = profile.accel_time + profile.cruise_time / 2
        assert profile.velocity_at(mid_time) == pytest.approx(10.0, abs=0.01)
        assert profile.peak_velocity == 10.0

    def test_trapezoidal_respects_max_accel(self) -> None:
        """Test acceleration respects max_acceleration limit."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        # During acceleration phase
        assert profile.acceleration_at(profile.accel_time / 2) == pytest.approx(5.0, abs=0.01)
        # During cruise phase
        mid_cruise = profile.accel_time + profile.cruise_time / 2
        assert profile.acceleration_at(mid_cruise) == 0.0
        # During deceleration phase
        decel_time = profile.accel_time + profile.cruise_time + profile.decel_time / 2
        assert profile.acceleration_at(decel_time) == pytest.approx(-5.0, abs=0.01)

    def test_trapezoidal_short_move_triangular(self) -> None:
        """Test short move produces triangular profile."""
        # Short distance that can't reach max velocity
        profile = TrapezoidalProfile(
            start=0.0, end=10.0, max_velocity=100.0, max_acceleration=5.0
        )
        assert profile.is_triangular is True
        assert profile.cruise_time == 0.0
        assert profile.peak_velocity < 100.0

    def test_trapezoidal_total_time(self) -> None:
        """Test total_time calculation."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        # Time to accel: v/a = 10/5 = 2s
        # Distance in accel: 0.5 * 5 * 4 = 10
        # Cruise distance: 100 - 20 = 80
        # Cruise time: 80/10 = 8s
        # Total: 2 + 8 + 2 = 12s
        assert profile.total_time == pytest.approx(12.0, abs=0.01)
        assert profile.accel_time == pytest.approx(2.0, abs=0.01)
        assert profile.cruise_time == pytest.approx(8.0, abs=0.01)
        assert profile.decel_time == pytest.approx(2.0, abs=0.01)

    def test_trapezoidal_is_complete(self) -> None:
        """Test is_complete method."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        assert profile.is_complete(profile.total_time - 1) is False
        assert profile.is_complete(profile.total_time) is True
        assert profile.is_complete(profile.total_time + 1) is True

    def test_trapezoidal_sample(self) -> None:
        """Test sample method returns TrajectoryPoint."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        point = profile.sample(profile.accel_time)
        assert isinstance(point, TrajectoryPoint)
        assert point.velocity == pytest.approx(10.0, abs=0.01)

    def test_trapezoidal_sample_n(self) -> None:
        """Test sample_n method."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        points = profile.sample_n(5)
        assert len(points) == 5
        assert points[0].position == 0.0
        assert points[-1].position == pytest.approx(100.0, abs=0.01)

    def test_trapezoidal_reversed(self) -> None:
        """Test reversed method."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        rev = profile.reversed()
        assert rev.start == 100.0
        assert rev.end == 0.0
        assert rev.total_time == pytest.approx(profile.total_time, abs=0.01)

    def test_trapezoidal_negative_direction(self) -> None:
        """Test profile in negative direction."""
        profile = TrapezoidalProfile(
            start=100.0, end=0.0, max_velocity=10.0, max_acceleration=5.0
        )
        assert profile.position_at(0.0) == 100.0
        assert profile.position_at(profile.total_time) == pytest.approx(0.0, abs=0.01)
        # Velocity should be negative
        assert profile.velocity_at(profile.accel_time) < 0

    def test_trapezoidal_invalid_max_velocity_raises(self) -> None:
        """Test invalid max_velocity raises ValueError."""
        with pytest.raises(ValueError, match="max_velocity must be > 0"):
            TrapezoidalProfile(start=0.0, end=100.0, max_velocity=0.0, max_acceleration=5.0)

    def test_trapezoidal_invalid_max_accel_raises(self) -> None:
        """Test invalid max_acceleration raises ValueError."""
        with pytest.raises(ValueError, match="max_acceleration must be > 0"):
            TrapezoidalProfile(start=0.0, end=100.0, max_velocity=10.0, max_acceleration=0.0)

    def test_trapezoidal_velocity_at_boundaries(self) -> None:
        """Test velocity is zero at start and end."""
        profile = TrapezoidalProfile(
            start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0
        )
        assert profile.velocity_at(0.0) == 0.0
        assert profile.velocity_at(profile.total_time) == 0.0
        assert profile.velocity_at(-1.0) == 0.0
        assert profile.velocity_at(profile.total_time + 1.0) == 0.0


# =============================================================================
# Trajectory Tests
# =============================================================================


class TestTrajectory:
    """Tests for Trajectory class."""

    def test_trajectory_waypoints(self) -> None:
        """Test trajectory stores waypoints correctly."""
        traj = Trajectory(
            waypoints=[0.0, 50.0, 100.0],
            times=[0.0, 1.0, 2.0],
        )
        assert traj.waypoints == [0.0, 50.0, 100.0]
        assert traj.times == [0.0, 1.0, 2.0]
        assert traj.num_waypoints == 3
        assert traj.num_segments == 2

    def test_trajectory_sample(self) -> None:
        """Test trajectory sample method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        point = traj.sample(1.0)
        assert isinstance(point, TrajectoryPoint)
        assert point.position == 50.0
        assert point.time == 1.0

    def test_trajectory_add_waypoint(self) -> None:
        """Test adding waypoints dynamically."""
        traj = Trajectory(waypoints=[0.0, 50.0], times=[0.0, 1.0])
        traj.add_waypoint(100.0, time=2.0)

        assert traj.num_waypoints == 3
        assert traj.waypoints[-1] == 100.0
        assert traj.times[-1] == 2.0
        assert traj.total_time == 2.0

    def test_trajectory_add_waypoint_with_duration(self) -> None:
        """Test adding waypoint with duration instead of time."""
        traj = Trajectory(waypoints=[0.0, 50.0], times=[0.0, 1.0])
        traj.add_waypoint(100.0, duration=1.5)

        assert traj.times[-1] == 2.5
        assert traj.total_time == 2.5

    def test_trajectory_add_waypoint_auto_time(self) -> None:
        """Test adding waypoint with automatic time calculation."""
        traj = Trajectory(waypoints=[0.0], default_velocity=10.0)
        traj.add_waypoint(50.0)  # Should take 5 seconds at v=10

        assert traj.times[-1] == pytest.approx(5.0, abs=0.01)

    def test_trajectory_position_at(self) -> None:
        """Test position_at method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0, 50.0],
            times=[0.0, 1.0, 2.0],
        )
        assert traj.position_at(0.0) == 0.0
        assert traj.position_at(0.5) == 50.0  # Midway first segment
        assert traj.position_at(1.0) == 100.0
        assert traj.position_at(1.5) == 75.0  # Midway second segment
        assert traj.position_at(2.0) == 50.0

    def test_trajectory_velocity_at(self) -> None:
        """Test velocity_at method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        assert traj.velocity_at(1.0) == 50.0
        # Velocity at boundaries is 0
        assert traj.velocity_at(0.0) == 0.0
        assert traj.velocity_at(2.0) == 0.0

    def test_trajectory_total_time(self) -> None:
        """Test total_time property."""
        traj = Trajectory(
            waypoints=[0.0, 50.0, 100.0, 75.0],
            times=[0.0, 1.0, 3.0, 5.0],
        )
        assert traj.total_time == 5.0

    def test_trajectory_sample_n(self) -> None:
        """Test sample_n method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        points = traj.sample_n(5)
        assert len(points) == 5
        assert points[0].position == 0.0
        assert points[2].position == 50.0
        assert points[4].position == 100.0

    def test_trajectory_sample_dt(self) -> None:
        """Test sample_dt method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        points = traj.sample_dt(0.5)
        assert len(points) == 5  # 0, 0.5, 1.0, 1.5, 2.0

    def test_trajectory_sample_at_waypoints(self) -> None:
        """Test sample_at_waypoints method."""
        traj = Trajectory(
            waypoints=[0.0, 50.0, 100.0],
            times=[0.0, 1.0, 2.0],
        )
        points = traj.sample_at_waypoints()
        assert len(points) == 3
        assert points[0].position == 0.0
        assert points[1].position == 50.0
        assert points[2].position == 100.0

    def test_trajectory_is_complete(self) -> None:
        """Test is_complete method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        assert traj.is_complete(1.0) is False
        assert traj.is_complete(2.0) is True
        assert traj.is_complete(3.0) is True

    def test_trajectory_progress(self) -> None:
        """Test progress method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        assert traj.progress(0.0) == 0.0
        assert traj.progress(1.0) == 0.5
        assert traj.progress(2.0) == 1.0

    def test_trajectory_reversed(self) -> None:
        """Test reversed method."""
        traj = Trajectory(
            waypoints=[0.0, 50.0, 100.0],
            times=[0.0, 1.0, 3.0],
        )
        rev = traj.reversed()
        assert rev.waypoints == [100.0, 50.0, 0.0]
        assert rev.total_time == 3.0

    def test_trajectory_scaled(self) -> None:
        """Test scaled method."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        scaled = traj.scaled(2.0)
        assert scaled.total_time == 4.0
        assert scaled.position_at(2.0) == 50.0  # Now midpoint

    def test_trajectory_empty_raises(self) -> None:
        """Test empty waypoints raises ValueError."""
        with pytest.raises(ValueError, match="at least 1 point"):
            Trajectory(waypoints=[])

    def test_trajectory_mismatched_times_raises(self) -> None:
        """Test mismatched times length raises ValueError."""
        with pytest.raises(ValueError, match="times length"):
            Trajectory(waypoints=[0.0, 50.0, 100.0], times=[0.0, 1.0])

    def test_trajectory_non_monotonic_times_raises(self) -> None:
        """Test non-monotonic times raises ValueError."""
        with pytest.raises(ValueError, match="monotonically increasing"):
            Trajectory(waypoints=[0.0, 50.0, 100.0], times=[0.0, 2.0, 1.0])

    def test_trajectory_add_waypoint_invalid_time_raises(self) -> None:
        """Test adding waypoint with time <= last time raises."""
        traj = Trajectory(waypoints=[0.0, 50.0], times=[0.0, 2.0])
        with pytest.raises(ValueError, match="must be > last waypoint time"):
            traj.add_waypoint(100.0, time=1.0)

    def test_trajectory_auto_timing(self) -> None:
        """Test automatic timing based on default_velocity."""
        traj = Trajectory(
            waypoints=[0.0, 100.0, 150.0],
            default_velocity=50.0,
        )
        # 0 -> 100 at 50/s = 2s, 100 -> 150 at 50/s = 1s
        assert traj.times[1] == pytest.approx(2.0, abs=0.01)
        assert traj.times[2] == pytest.approx(3.0, abs=0.01)

    def test_trajectory_single_waypoint(self) -> None:
        """Test trajectory with single waypoint."""
        traj = Trajectory(waypoints=[50.0])
        assert traj.num_waypoints == 1
        assert traj.num_segments == 0
        assert traj.position_at(0.0) == 50.0
        assert traj.position_at(10.0) == 50.0

    def test_trajectory_clamping(self) -> None:
        """Test position is clamped at trajectory boundaries."""
        traj = Trajectory(
            waypoints=[0.0, 100.0],
            times=[0.0, 2.0],
        )
        assert traj.position_at(-1.0) == 0.0
        assert traj.position_at(5.0) == 100.0
