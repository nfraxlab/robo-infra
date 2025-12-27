"""Tests for advanced trajectory classes."""

from __future__ import annotations

import pytest

from robo_infra.motion.trajectory import (
    BlendedTrajectory,
    BlendSegment,
    CubicTrajectory,
    QuinticTrajectory,
    SCurveTrajectory,
    SplineTrajectory,
    TrajectoryConstraints,
    TrajectoryGenerator,
    TrajectoryPoint,
    TrajectoryProfile,
)


class TestTrajectoryProfile:
    """Tests for TrajectoryProfile enum."""

    def test_values(self) -> None:
        """Test enum values."""
        assert TrajectoryProfile.TRAPEZOIDAL.value == "trapezoidal"
        assert TrajectoryProfile.S_CURVE.value == "s_curve"
        assert TrajectoryProfile.CUBIC.value == "cubic"
        assert TrajectoryProfile.QUINTIC.value == "quintic"


class TestTrajectoryConstraints:
    """Tests for TrajectoryConstraints dataclass."""

    def test_creation(self) -> None:
        """Test basic creation."""
        constraints = TrajectoryConstraints(
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=100.0,
        )
        assert constraints.max_velocity == 10.0
        assert constraints.max_acceleration == 5.0
        assert constraints.max_jerk == 100.0

    def test_creation_without_jerk(self) -> None:
        """Test creation without jerk."""
        constraints = TrajectoryConstraints(max_velocity=10.0, max_acceleration=5.0)
        assert constraints.max_jerk is None

    def test_invalid_velocity(self) -> None:
        """Test error on invalid velocity."""
        with pytest.raises(ValueError, match="max_velocity must be > 0"):
            TrajectoryConstraints(max_velocity=0.0, max_acceleration=5.0)

    def test_invalid_acceleration(self) -> None:
        """Test error on invalid acceleration."""
        with pytest.raises(ValueError, match="max_acceleration must be > 0"):
            TrajectoryConstraints(max_velocity=10.0, max_acceleration=-1.0)

    def test_invalid_jerk(self) -> None:
        """Test error on invalid jerk."""
        with pytest.raises(ValueError, match="max_jerk must be > 0"):
            TrajectoryConstraints(max_velocity=10.0, max_acceleration=5.0, max_jerk=0.0)


class TestTrajectoryGenerator:
    """Tests for TrajectoryGenerator class."""

    def test_creation_with_constraints(self) -> None:
        """Test creation with TrajectoryConstraints."""
        constraints = TrajectoryConstraints(max_velocity=10.0, max_acceleration=5.0)
        gen = TrajectoryGenerator(profile=TrajectoryProfile.TRAPEZOIDAL, constraints=constraints)
        assert gen.profile == TrajectoryProfile.TRAPEZOIDAL
        assert gen.constraints == constraints

    def test_creation_with_values(self) -> None:
        """Test creation with individual values."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.CUBIC,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        assert gen.constraints.max_velocity == 10.0
        assert gen.constraints.max_acceleration == 5.0

    def test_generate_trapezoidal(self) -> None:
        """Test generating trapezoidal profile."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.TRAPEZOIDAL,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        traj = gen.generate(start=0.0, end=100.0)

        assert traj.start == 0.0
        assert traj.end == 100.0
        assert traj.total_time > 0

    def test_generate_cubic(self) -> None:
        """Test generating cubic profile."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.CUBIC,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        traj = gen.generate(start=0.0, end=100.0)

        assert isinstance(traj, CubicTrajectory)
        assert traj.start == 0.0
        assert traj.end == 100.0

    def test_generate_quintic(self) -> None:
        """Test generating quintic profile."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.QUINTIC,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        traj = gen.generate(start=0.0, end=100.0)

        assert isinstance(traj, QuinticTrajectory)
        assert traj.start == 0.0
        assert traj.end == 100.0

    def test_generate_scurve(self) -> None:
        """Test generating S-curve profile."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.S_CURVE,
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=50.0,
        )
        traj = gen.generate(start=0.0, end=100.0)

        assert isinstance(traj, SCurveTrajectory)

    def test_generate_with_fixed_duration(self) -> None:
        """Test generating with fixed duration."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.CUBIC,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        traj = gen.generate(start=0.0, end=100.0, duration=5.0)

        assert traj.duration == pytest.approx(5.0)

    def test_generate_multi_axis(self) -> None:
        """Test generating multi-axis synchronized trajectories."""
        # Use CUBIC profile which supports duration parameter for synchronization
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.CUBIC,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        trajectories = gen.generate_multi_axis(
            starts=[0.0, 0.0, 0.0],
            ends=[100.0, 50.0, 25.0],
            synchronized=True,
        )

        assert len(trajectories) == 3
        # All should have same duration when synchronized
        durations = [t.duration for t in trajectories]
        assert durations[0] == pytest.approx(durations[1])
        assert durations[0] == pytest.approx(durations[2])

    def test_generate_multi_axis_unsynchronized(self) -> None:
        """Test generating multi-axis unsynchronized trajectories."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.TRAPEZOIDAL,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        trajectories = gen.generate_multi_axis(
            starts=[0.0, 0.0],
            ends=[100.0, 10.0],
            synchronized=False,
        )

        assert len(trajectories) == 2
        # Different distances should have different durations
        assert trajectories[0].total_time > trajectories[1].total_time

    def test_generate_multi_axis_dimension_error(self) -> None:
        """Test error on dimension mismatch."""
        gen = TrajectoryGenerator(
            profile=TrajectoryProfile.TRAPEZOIDAL,
            max_velocity=10.0,
            max_acceleration=5.0,
        )
        with pytest.raises(ValueError, match="must match"):
            gen.generate_multi_axis(starts=[0.0, 0.0], ends=[100.0])


class TestCubicTrajectory:
    """Tests for CubicTrajectory class."""

    def test_creation(self) -> None:
        """Test basic creation."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        assert traj.start == 0.0
        assert traj.end == 100.0
        assert traj.duration == 2.0
        assert traj.total_time == 2.0

    def test_invalid_duration(self) -> None:
        """Test error on invalid duration."""
        with pytest.raises(ValueError, match="duration must be > 0"):
            CubicTrajectory(start=0.0, end=100.0, duration=0.0)

    def test_position_at_endpoints(self) -> None:
        """Test position at start and end."""
        traj = CubicTrajectory(start=10.0, end=50.0, duration=2.0)
        assert traj.position_at(0.0) == pytest.approx(10.0)
        assert traj.position_at(2.0) == pytest.approx(50.0)

    def test_velocity_at_endpoints(self) -> None:
        """Test zero velocity at endpoints."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        assert traj.velocity_at(0.0) == pytest.approx(0.0)
        assert traj.velocity_at(2.0) == pytest.approx(0.0)

    def test_velocity_at_midpoint(self) -> None:
        """Test non-zero velocity at midpoint."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        assert traj.velocity_at(1.0) != 0.0

    def test_sample(self) -> None:
        """Test sampling returns TrajectoryPoint."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        point = traj.sample(1.0)

        assert isinstance(point, TrajectoryPoint)
        assert point.time == pytest.approx(1.0)

    def test_sample_n(self) -> None:
        """Test sampling n points."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        points = traj.sample_n(5)

        assert len(points) == 5
        assert points[0].position == pytest.approx(0.0)
        assert points[-1].position == pytest.approx(100.0)

    def test_sample_n_error(self) -> None:
        """Test error on n < 2."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        with pytest.raises(ValueError, match="must be >= 2"):
            traj.sample_n(1)

    def test_sample_dt(self) -> None:
        """Test sampling at time intervals."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        points = traj.sample_dt(0.5)

        assert len(points) == 5  # 0, 0.5, 1.0, 1.5, 2.0
        assert points[0].time == pytest.approx(0.0)
        assert points[-1].time == pytest.approx(2.0)

    def test_is_complete(self) -> None:
        """Test completion check."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        assert not traj.is_complete(1.0)
        assert traj.is_complete(2.0)
        assert traj.is_complete(3.0)

    def test_progress(self) -> None:
        """Test progress calculation."""
        traj = CubicTrajectory(start=0.0, end=100.0, duration=2.0)
        assert traj.progress(0.0) == pytest.approx(0.0)
        assert traj.progress(1.0) == pytest.approx(0.5)
        assert traj.progress(2.0) == pytest.approx(1.0)

    def test_with_velocities(self) -> None:
        """Test with non-zero boundary velocities."""
        traj = CubicTrajectory(
            start=0.0, end=100.0, duration=2.0, start_velocity=10.0, end_velocity=10.0
        )
        # Should still reach correct endpoints
        assert traj.position_at(0.0) == pytest.approx(0.0)
        assert traj.position_at(2.0) == pytest.approx(100.0)


class TestQuinticTrajectory:
    """Tests for QuinticTrajectory class."""

    def test_creation(self) -> None:
        """Test basic creation."""
        traj = QuinticTrajectory(start=0.0, end=100.0, duration=2.0)
        assert traj.start == 0.0
        assert traj.end == 100.0
        assert traj.duration == 2.0

    def test_invalid_duration(self) -> None:
        """Test error on invalid duration."""
        with pytest.raises(ValueError, match="duration must be > 0"):
            QuinticTrajectory(start=0.0, end=100.0, duration=-1.0)

    def test_zero_boundary_conditions(self) -> None:
        """Test zero velocity and acceleration at endpoints."""
        traj = QuinticTrajectory(start=0.0, end=100.0, duration=2.0)

        # Position endpoints
        assert traj.position_at(0.0) == pytest.approx(0.0)
        assert traj.position_at(2.0) == pytest.approx(100.0)

        # Velocity endpoints (should be zero by default)
        assert traj.velocity_at(0.0) == pytest.approx(0.0)
        assert traj.velocity_at(2.0) == pytest.approx(0.0)

        # Acceleration endpoints (should be zero by default)
        assert traj.acceleration_at(0.0) == pytest.approx(0.0)
        assert traj.acceleration_at(2.0) == pytest.approx(0.0)

    def test_jerk_at(self) -> None:
        """Test jerk calculation."""
        traj = QuinticTrajectory(start=0.0, end=100.0, duration=2.0)
        jerk = traj.jerk_at(1.0)
        assert isinstance(jerk, float)

    def test_smoother_than_cubic(self) -> None:
        """Test quintic is smoother (zero acceleration at ends)."""
        quintic = QuinticTrajectory(start=0.0, end=100.0, duration=2.0)

        # Quintic has zero acceleration at endpoints (unlike cubic)
        assert quintic.acceleration_at(0.0) == pytest.approx(0.0)
        assert quintic.acceleration_at(2.0) == pytest.approx(0.0)

        # Cubic may not have zero acceleration at endpoints
        # (depends on boundary conditions)

    def test_sample_n(self) -> None:
        """Test sampling n points."""
        traj = QuinticTrajectory(start=0.0, end=100.0, duration=2.0)
        points = traj.sample_n(11)

        assert len(points) == 11
        assert points[0].position == pytest.approx(0.0)
        assert points[-1].position == pytest.approx(100.0)


class TestSCurveTrajectory:
    """Tests for SCurveTrajectory class."""

    def test_creation(self) -> None:
        """Test basic creation."""
        traj = SCurveTrajectory(
            start=0.0,
            end=100.0,
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=50.0,
        )
        assert traj.start == 0.0
        assert traj.end == 100.0

    def test_invalid_parameters(self) -> None:
        """Test error on invalid parameters."""
        with pytest.raises(ValueError, match="max_velocity must be > 0"):
            SCurveTrajectory(
                start=0.0, end=100.0, max_velocity=0.0, max_acceleration=5.0, max_jerk=50.0
            )

        with pytest.raises(ValueError, match="max_acceleration must be > 0"):
            SCurveTrajectory(
                start=0.0, end=100.0, max_velocity=10.0, max_acceleration=-1.0, max_jerk=50.0
            )

        with pytest.raises(ValueError, match="max_jerk must be > 0"):
            SCurveTrajectory(
                start=0.0, end=100.0, max_velocity=10.0, max_acceleration=5.0, max_jerk=0.0
            )

    def test_endpoints(self) -> None:
        """Test position at endpoints."""
        traj = SCurveTrajectory(
            start=0.0,
            end=100.0,
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=50.0,
        )
        assert traj.position_at(0.0) == pytest.approx(0.0)
        assert traj.position_at(traj.total_time) == pytest.approx(100.0)

    def test_zero_velocity_at_endpoints(self) -> None:
        """Test zero velocity at start and end."""
        traj = SCurveTrajectory(
            start=0.0,
            end=100.0,
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=50.0,
        )
        assert traj.velocity_at(0.0) == pytest.approx(0.0)
        assert traj.velocity_at(traj.total_time) == pytest.approx(0.0)

    def test_jerk_limited(self) -> None:
        """Test that jerk is limited."""
        traj = SCurveTrajectory(
            start=0.0,
            end=100.0,
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=50.0,
        )
        # Sample jerk throughout trajectory
        for t in [traj.total_time * i / 10 for i in range(11)]:
            jerk = abs(traj.jerk_at(t))
            assert jerk <= 50.0 + 0.1  # Small tolerance

    def test_sample_n(self) -> None:
        """Test sampling."""
        traj = SCurveTrajectory(
            start=0.0,
            end=100.0,
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=50.0,
        )
        points = traj.sample_n(11)

        assert len(points) == 11
        assert points[0].position == pytest.approx(0.0)
        assert points[-1].position == pytest.approx(100.0)

    def test_negative_direction(self) -> None:
        """Test trajectory in negative direction."""
        traj = SCurveTrajectory(
            start=100.0,
            end=0.0,
            max_velocity=10.0,
            max_acceleration=5.0,
            max_jerk=50.0,
        )
        assert traj.position_at(0.0) == pytest.approx(100.0)
        assert traj.position_at(traj.total_time) == pytest.approx(0.0)


class TestSplineTrajectory:
    """Tests for SplineTrajectory class."""

    def test_creation(self) -> None:
        """Test basic creation."""
        traj = SplineTrajectory(waypoints=[0.0, 50.0, 100.0])
        assert len(traj.waypoints) == 3
        assert traj.num_segments == 2

    def test_creation_with_times(self) -> None:
        """Test creation with explicit times."""
        traj = SplineTrajectory(waypoints=[0.0, 50.0, 100.0], times=[0.0, 1.0, 2.0])
        assert traj.times == [0.0, 1.0, 2.0]
        assert traj.total_time == 2.0

    def test_invalid_waypoints(self) -> None:
        """Test error on too few waypoints."""
        with pytest.raises(ValueError, match="at least 2"):
            SplineTrajectory(waypoints=[0.0])

    def test_invalid_times(self) -> None:
        """Test error on times/waypoints mismatch."""
        with pytest.raises(ValueError, match="must match"):
            SplineTrajectory(waypoints=[0.0, 100.0], times=[0.0, 1.0, 2.0])

    def test_position_at_waypoints(self) -> None:
        """Test position at waypoint times."""
        traj = SplineTrajectory(waypoints=[0.0, 50.0, 100.0], times=[0.0, 1.0, 2.0])
        assert traj.position_at(0.0) == pytest.approx(0.0)
        assert traj.position_at(1.0) == pytest.approx(50.0)
        assert traj.position_at(2.0) == pytest.approx(100.0)

    def test_continuous_velocity(self) -> None:
        """Test velocity is continuous (smooth)."""
        traj = SplineTrajectory(waypoints=[0.0, 50.0, 100.0], times=[0.0, 1.0, 2.0])
        # Velocity just before and after waypoint should be similar
        v_before = traj.velocity_at(0.99)
        v_after = traj.velocity_at(1.01)
        # They should be close (C1 continuous)
        assert abs(v_before - v_after) < 5.0  # Allow some tolerance

    def test_sample_n(self) -> None:
        """Test sampling."""
        traj = SplineTrajectory(waypoints=[0.0, 50.0, 100.0])
        points = traj.sample_n(11)

        assert len(points) == 11
        assert points[0].position == pytest.approx(0.0)
        assert points[-1].position == pytest.approx(100.0)

    def test_many_waypoints(self) -> None:
        """Test with many waypoints."""
        waypoints = [float(i * 10) for i in range(10)]
        traj = SplineTrajectory(waypoints=waypoints)

        assert traj.num_segments == 9
        assert traj.position_at(0.0) == pytest.approx(0.0)
        assert traj.position_at(traj.total_time) == pytest.approx(90.0)


class TestBlendedTrajectory:
    """Tests for BlendedTrajectory class."""

    def test_creation(self) -> None:
        """Test basic creation."""
        traj = BlendedTrajectory(start=0.0)
        assert traj.start == 0.0
        assert traj.num_segments == 0
        assert traj.total_time == 0.0

    def test_add_segment(self) -> None:
        """Test adding segments."""
        traj = BlendedTrajectory(start=0.0)
        traj.add_segment(target=50.0, duration=1.0)
        traj.add_segment(target=100.0, duration=1.0)

        assert traj.num_segments == 2
        assert traj.total_time == 2.0

    def test_chaining(self) -> None:
        """Test method chaining."""
        traj = (
            BlendedTrajectory(start=0.0)
            .add_segment(50.0, 1.0)
            .add_segment(100.0, 1.0)
            .set_blend_radius(0.2)
        )
        assert traj.num_segments == 2

    def test_invalid_duration(self) -> None:
        """Test error on invalid duration."""
        traj = BlendedTrajectory(start=0.0)
        with pytest.raises(ValueError, match="duration must be > 0"):
            traj.add_segment(target=50.0, duration=0.0)

    def test_invalid_blend_radius(self) -> None:
        """Test error on invalid blend radius."""
        traj = BlendedTrajectory(start=0.0)
        with pytest.raises(ValueError, match="radius must be >= 0"):
            traj.set_blend_radius(-0.1)

    def test_position_at(self) -> None:
        """Test position sampling."""
        traj = BlendedTrajectory(start=0.0).add_segment(100.0, 2.0)
        assert traj.position_at(0.0) == pytest.approx(0.0)
        assert traj.position_at(2.0) == pytest.approx(100.0)

    def test_multi_segment(self) -> None:
        """Test multi-segment trajectory."""
        traj = (
            BlendedTrajectory(start=0.0)
            .add_segment(50.0, 1.0)
            .add_segment(100.0, 1.0)
            .add_segment(75.0, 1.0)
        )

        assert traj.total_time == 3.0
        assert traj.position_at(0.0) == pytest.approx(0.0)
        assert traj.position_at(3.0) == pytest.approx(75.0)

    def test_sample_n(self) -> None:
        """Test sampling."""
        traj = BlendedTrajectory(start=0.0).add_segment(100.0, 2.0)
        points = traj.sample_n(11)

        assert len(points) == 11
        assert points[0].position == pytest.approx(0.0)
        assert points[-1].position == pytest.approx(100.0)

    def test_clear(self) -> None:
        """Test clearing segments."""
        traj = BlendedTrajectory(start=0.0).add_segment(50.0, 1.0).add_segment(100.0, 1.0).clear()
        assert traj.num_segments == 0
        assert traj.total_time == 0.0

    def test_progress(self) -> None:
        """Test progress calculation."""
        traj = BlendedTrajectory(start=0.0).add_segment(100.0, 2.0)
        assert traj.progress(0.0) == pytest.approx(0.0)
        assert traj.progress(1.0) == pytest.approx(0.5)
        assert traj.progress(2.0) == pytest.approx(1.0)


class TestBlendSegment:
    """Tests for BlendSegment dataclass."""

    def test_creation(self) -> None:
        """Test basic creation."""
        segment = BlendSegment(target=50.0, duration=1.0)
        assert segment.target == 50.0
        assert segment.duration == 1.0
        assert segment.blend_radius == 0.0

    def test_with_blend_radius(self) -> None:
        """Test creation with blend radius."""
        segment = BlendSegment(target=50.0, duration=1.0, blend_radius=0.1)
        assert segment.blend_radius == 0.1
