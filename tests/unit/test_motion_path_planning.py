"""Tests for path planning module."""

from __future__ import annotations

import math

import pytest

from robo_infra.motion.path_planning import (
    BoxObstacle,
    CartesianPathPlanner,
    LinearPathPlanner,
    Path,
    PathPlanner,
    PathPoint,
    PathSmoother,
    PathStatus,
    RRTPathPlanner,
    SmoothingMethod,
    SphereObstacle,
)


class TestPathPoint:
    """Tests for PathPoint dataclass."""

    def test_creation(self) -> None:
        """Test basic creation."""
        point = PathPoint(position=(1.0, 2.0, 3.0))
        assert point.position == (1.0, 2.0, 3.0)
        assert point.cost == 0.0
        assert point.metadata == {}

    def test_with_cost(self) -> None:
        """Test creation with cost."""
        point = PathPoint(position=(1.0, 2.0), cost=5.5)
        assert point.cost == 5.5

    def test_distance_to(self) -> None:
        """Test Euclidean distance calculation."""
        p1 = PathPoint(position=(0.0, 0.0))
        p2 = PathPoint(position=(3.0, 4.0))
        assert p1.distance_to(p2) == pytest.approx(5.0)

    def test_distance_to_3d(self) -> None:
        """Test distance in 3D."""
        p1 = PathPoint(position=(0.0, 0.0, 0.0))
        p2 = PathPoint(position=(1.0, 1.0, 1.0))
        assert p1.distance_to(p2) == pytest.approx(math.sqrt(3))

    def test_distance_dimension_mismatch(self) -> None:
        """Test error on dimension mismatch."""
        p1 = PathPoint(position=(0.0, 0.0))
        p2 = PathPoint(position=(1.0, 1.0, 1.0))
        with pytest.raises(ValueError, match="dimensions don't match"):
            p1.distance_to(p2)

    def test_interpolate(self) -> None:
        """Test linear interpolation."""
        p1 = PathPoint(position=(0.0, 0.0), cost=0.0)
        p2 = PathPoint(position=(10.0, 10.0), cost=10.0)

        mid = p1.interpolate(p2, 0.5)
        assert mid.position == (5.0, 5.0)
        assert mid.cost == pytest.approx(5.0)

    def test_interpolate_clamped(self) -> None:
        """Test interpolation is clamped to [0, 1]."""
        p1 = PathPoint(position=(0.0, 0.0))
        p2 = PathPoint(position=(10.0, 10.0))

        at_start = p1.interpolate(p2, -0.5)
        assert at_start.position == (0.0, 0.0)

        at_end = p1.interpolate(p2, 1.5)
        assert at_end.position == (10.0, 10.0)


class TestPath:
    """Tests for Path dataclass."""

    def test_empty_path(self) -> None:
        """Test empty path."""
        path = Path()
        assert len(path) == 0
        assert path.is_empty
        assert path.start is None
        assert path.goal is None

    def test_path_with_points(self) -> None:
        """Test path with points."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(1.0, 1.0)),
            PathPoint(position=(2.0, 2.0)),
        ]
        path = Path(points=points)
        assert len(path) == 3
        assert not path.is_empty
        assert path.start == points[0]
        assert path.goal == points[2]

    def test_path_status(self) -> None:
        """Test path status."""
        path = Path(status=PathStatus.SUCCESS)
        assert path.is_valid

        path = Path(status=PathStatus.PARTIAL)
        assert path.is_valid

        path = Path(status=PathStatus.FAILURE)
        assert not path.is_valid

    def test_calculate_length(self) -> None:
        """Test path length calculation."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(3.0, 4.0)),  # distance 5
            PathPoint(position=(3.0, 0.0)),  # distance 4
        ]
        path = Path(points=points)
        assert path.calculate_length() == pytest.approx(9.0)

    def test_append(self) -> None:
        """Test appending to path."""
        path = Path()
        path.append(PathPoint(position=(0.0, 0.0)))
        path.append(PathPoint(position=(1.0, 1.0)))
        assert len(path) == 2

    def test_reverse(self) -> None:
        """Test path reversal."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(1.0, 1.0)),
            PathPoint(position=(2.0, 2.0)),
        ]
        path = Path(points=points)
        reversed_path = path.reverse()

        assert reversed_path.start.position == (2.0, 2.0)
        assert reversed_path.goal.position == (0.0, 0.0)

    def test_resample(self) -> None:
        """Test path resampling."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(10.0, 0.0)),
        ]
        path = Path(points=points)
        resampled = path.resample(11)

        assert len(resampled) == 11
        assert resampled[0].position == (0.0, 0.0)
        assert resampled[10].position == (10.0, 0.0)
        assert resampled[5].position[0] == pytest.approx(5.0)

    def test_resample_error(self) -> None:
        """Test resample errors."""
        path = Path(points=[PathPoint(position=(0.0, 0.0))])
        with pytest.raises(ValueError, match="at least 2 points"):
            path.resample(5)

        with pytest.raises(ValueError, match="must be >= 2"):
            Path(
                points=[
                    PathPoint(position=(0.0, 0.0)),
                    PathPoint(position=(1.0, 0.0)),
                ]
            ).resample(1)

    def test_as_positions(self) -> None:
        """Test extracting positions."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(1.0, 1.0)),
        ]
        path = Path(points=points)
        positions = path.as_positions()
        assert positions == [(0.0, 0.0), (1.0, 1.0)]

    def test_as_lists(self) -> None:
        """Test extracting as lists."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(1.0, 1.0)),
        ]
        path = Path(points=points)
        lists = path.as_lists()
        assert lists == [[0.0, 0.0], [1.0, 1.0]]


class TestSphereObstacle:
    """Tests for SphereObstacle."""

    def test_creation(self) -> None:
        """Test basic creation."""
        obs = SphereObstacle(center=(0.0, 0.0), radius=1.0)
        assert obs.center == (0.0, 0.0)
        assert obs.radius == 1.0

    def test_invalid_radius(self) -> None:
        """Test error on invalid radius."""
        with pytest.raises(ValueError, match="radius must be > 0"):
            SphereObstacle(center=(0.0, 0.0), radius=-1.0)

    def test_contains_point_inside(self) -> None:
        """Test point inside sphere."""
        obs = SphereObstacle(center=(0.0, 0.0), radius=1.0)
        assert obs.contains_point((0.0, 0.0))
        assert obs.contains_point((0.5, 0.0))
        assert obs.contains_point((0.0, 0.5))

    def test_contains_point_outside(self) -> None:
        """Test point outside sphere."""
        obs = SphereObstacle(center=(0.0, 0.0), radius=1.0)
        assert not obs.contains_point((2.0, 0.0))
        assert not obs.contains_point((0.0, 2.0))
        assert not obs.contains_point((1.0, 1.0))

    def test_distance_to_point(self) -> None:
        """Test distance calculation."""
        obs = SphereObstacle(center=(0.0, 0.0), radius=1.0)

        # Point outside sphere
        assert obs.distance_to_point((2.0, 0.0)) == pytest.approx(1.0)

        # Point inside sphere - negative distance
        assert obs.distance_to_point((0.0, 0.0)) == pytest.approx(-1.0)

    def test_intersects_segment(self) -> None:
        """Test segment intersection."""
        obs = SphereObstacle(center=(5.0, 0.0), radius=1.0)

        # Intersects
        assert obs.intersects_segment((0.0, 0.0), (10.0, 0.0))

        # Doesn't intersect
        assert not obs.intersects_segment((0.0, 5.0), (10.0, 5.0))


class TestBoxObstacle:
    """Tests for BoxObstacle."""

    def test_creation(self) -> None:
        """Test basic creation."""
        obs = BoxObstacle(min_corner=(0.0, 0.0), max_corner=(1.0, 1.0))
        assert obs.min_corner == (0.0, 0.0)
        assert obs.max_corner == (1.0, 1.0)

    def test_invalid_corners(self) -> None:
        """Test error on invalid corners."""
        with pytest.raises(ValueError, match="must be <="):
            BoxObstacle(min_corner=(1.0, 0.0), max_corner=(0.0, 1.0))

    def test_contains_point_inside(self) -> None:
        """Test point inside box."""
        obs = BoxObstacle(min_corner=(0.0, 0.0), max_corner=(1.0, 1.0))
        assert obs.contains_point((0.5, 0.5))
        assert obs.contains_point((0.0, 0.0))
        assert obs.contains_point((1.0, 1.0))

    def test_contains_point_outside(self) -> None:
        """Test point outside box."""
        obs = BoxObstacle(min_corner=(0.0, 0.0), max_corner=(1.0, 1.0))
        assert not obs.contains_point((2.0, 0.5))
        assert not obs.contains_point((-0.5, 0.5))

    def test_distance_to_point(self) -> None:
        """Test distance calculation."""
        obs = BoxObstacle(min_corner=(0.0, 0.0), max_corner=(1.0, 1.0))

        # Outside
        assert obs.distance_to_point((2.0, 0.5)) == pytest.approx(1.0)

        # Point inside box - negative distance
        assert obs.distance_to_point((0.5, 0.5)) == pytest.approx(-0.5)


class TestLinearPathPlanner:
    """Tests for LinearPathPlanner."""

    def test_creation(self) -> None:
        """Test basic creation."""
        planner = LinearPathPlanner(num_points=50)
        assert planner.num_points == 50

    def test_invalid_num_points(self) -> None:
        """Test error on invalid num_points."""
        with pytest.raises(ValueError, match="must be >= 2"):
            LinearPathPlanner(num_points=1)

    def test_plan_2d(self) -> None:
        """Test planning in 2D."""
        planner = LinearPathPlanner(num_points=11)
        path = planner.plan([0.0, 0.0], [10.0, 10.0])

        assert len(path) == 11
        assert path.status == PathStatus.SUCCESS
        assert path[0].position == (0.0, 0.0)
        assert path[-1].position == (10.0, 10.0)
        assert path[5].position[0] == pytest.approx(5.0)

    def test_plan_3d(self) -> None:
        """Test planning in 3D."""
        planner = LinearPathPlanner(num_points=21)
        path = planner.plan([0.0, 0.0, 0.0], [10.0, 10.0, 10.0])

        assert len(path) == 21
        assert len(path[0].position) == 3

    def test_plan_dimension_mismatch(self) -> None:
        """Test error on dimension mismatch."""
        planner = LinearPathPlanner()
        with pytest.raises(ValueError, match="must match"):
            planner.plan([0.0, 0.0], [1.0, 1.0, 1.0])

    def test_protocol_compliance(self) -> None:
        """Test PathPlanner protocol compliance."""
        planner = LinearPathPlanner()
        assert isinstance(planner, PathPlanner)


class TestCartesianPathPlanner:
    """Tests for CartesianPathPlanner."""

    def test_creation(self) -> None:
        """Test basic creation."""
        planner = CartesianPathPlanner(num_points=50)
        assert planner.num_points == 50

    def test_plan_without_ik(self) -> None:
        """Test planning without IK function."""
        planner = CartesianPathPlanner(num_points=11)
        path = planner.plan([0.0, 0.0, 0.0], [10.0, 10.0, 10.0])

        assert len(path) == 11
        assert path.status == PathStatus.SUCCESS

    def test_plan_with_ik(self) -> None:
        """Test planning with IK function."""

        def simple_ik(cartesian: list[float]) -> list[float]:
            # Simple identity IK for testing
            return [c * 0.1 for c in cartesian]

        planner = CartesianPathPlanner(num_points=11, ik_function=simple_ik)
        path = planner.plan([0.0, 0.0, 0.0], [10.0, 10.0, 10.0])

        assert len(path) == 11
        # Check IK was applied
        assert path[-1].position[0] == pytest.approx(1.0)  # 10 * 0.1

    def test_plan_with_failing_ik(self) -> None:
        """Test handling of IK failure."""

        def failing_ik(cartesian: list[float]) -> list[float]:
            if cartesian[0] > 5:
                raise ValueError("Unreachable")
            return cartesian

        planner = CartesianPathPlanner(num_points=11, ik_function=failing_ik)
        path = planner.plan([0.0, 0.0, 0.0], [10.0, 10.0, 10.0])

        assert path.status == PathStatus.PARTIAL
        assert len(path) < 11


class TestRRTPathPlanner:
    """Tests for RRTPathPlanner."""

    def test_creation(self) -> None:
        """Test basic creation."""
        planner = RRTPathPlanner(max_iterations=1000, step_size=0.1)
        assert isinstance(planner, PathPlanner)

    def test_invalid_parameters(self) -> None:
        """Test error on invalid parameters."""
        with pytest.raises(ValueError, match="max_iterations must be > 0"):
            RRTPathPlanner(max_iterations=0)

        with pytest.raises(ValueError, match="step_size must be > 0"):
            RRTPathPlanner(step_size=0)

        with pytest.raises(ValueError, match="goal_bias must be in"):
            RRTPathPlanner(goal_bias=1.5)

        with pytest.raises(ValueError, match="goal_tolerance must be > 0"):
            RRTPathPlanner(goal_tolerance=0)

    def test_plan_no_obstacles(self) -> None:
        """Test planning without obstacles (direct path)."""
        planner = RRTPathPlanner()
        path = planner.plan([0.0, 0.0], [1.0, 1.0])

        assert path.status == PathStatus.SUCCESS
        assert len(path) > 0

    def test_plan_with_obstacles(self) -> None:
        """Test planning with obstacles."""
        planner = RRTPathPlanner(
            max_iterations=2000,
            step_size=0.2,
            goal_tolerance=0.3,
            goal_bias=0.2,
        )
        # Create a wall obstacle
        obstacles = [BoxObstacle(min_corner=(0.4, -0.5), max_corner=(0.6, 0.5))]

        path = planner.plan([0.0, 0.0], [1.0, 0.0], obstacles=obstacles)

        # Should find a path around the wall (or direct if lucky with sampling)
        assert path.status in (PathStatus.SUCCESS, PathStatus.TIMEOUT)

    def test_plan_start_in_collision(self) -> None:
        """Test planning when start is in collision."""
        planner = RRTPathPlanner()
        obstacles = [SphereObstacle(center=(0.0, 0.0), radius=1.0)]

        path = planner.plan([0.0, 0.0], [5.0, 5.0], obstacles=obstacles)
        assert path.status == PathStatus.COLLISION

    def test_plan_goal_in_collision(self) -> None:
        """Test planning when goal is in collision."""
        planner = RRTPathPlanner()
        obstacles = [SphereObstacle(center=(5.0, 5.0), radius=1.0)]

        path = planner.plan([0.0, 0.0], [5.0, 5.0], obstacles=obstacles)
        assert path.status == PathStatus.COLLISION


class TestPathSmoother:
    """Tests for PathSmoother."""

    def test_creation(self) -> None:
        """Test basic creation."""
        smoother = PathSmoother()
        assert smoother._method == SmoothingMethod.BSPLINE

    def test_invalid_parameters(self) -> None:
        """Test error on invalid parameters."""
        with pytest.raises(ValueError, match="iterations must be >= 1"):
            PathSmoother(iterations=0)

        with pytest.raises(ValueError, match="num_output_points must be >= 2"):
            PathSmoother(num_output_points=1)

    def test_smooth_bspline(self) -> None:
        """Test B-spline smoothing."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(1.0, 2.0)),
            PathPoint(position=(2.0, 0.0)),
            PathPoint(position=(3.0, 2.0)),
            PathPoint(position=(4.0, 0.0)),
        ]
        path = Path(points=points)
        smoother = PathSmoother(method=SmoothingMethod.BSPLINE, num_output_points=50)
        smoothed = smoother.smooth(path)

        assert len(smoothed) == 50
        assert smoothed.is_valid

    def test_smooth_bezier(self) -> None:
        """Test Bezier smoothing."""
        points = [
            PathPoint(position=(0.0, 0.0)),
            PathPoint(position=(1.0, 1.0)),
            PathPoint(position=(2.0, 0.0)),
        ]
        path = Path(points=points)
        smoother = PathSmoother(method=SmoothingMethod.BEZIER, num_output_points=20)
        smoothed = smoother.smooth(path)

        assert len(smoothed) == 20
        # Bezier should start at first and end at last control point
        assert smoothed[0].position[0] == pytest.approx(0.0)
        assert smoothed[-1].position[0] == pytest.approx(2.0)

    def test_smooth_shortcut(self) -> None:
        """Test shortcut smoothing."""
        # Create a zigzag path
        points = [
            PathPoint(position=(float(i), float(i % 2))) for i in range(10)
        ]
        path = Path(points=points)
        smoother = PathSmoother(method=SmoothingMethod.SHORTCUT, iterations=5)
        smoothed = smoother.smooth(path)

        # Shortcut should reduce the number of points
        assert len(smoothed) <= len(path)

    def test_smooth_short_path(self) -> None:
        """Test smoothing a very short path."""
        points = [PathPoint(position=(0.0, 0.0))]
        path = Path(points=points)
        smoother = PathSmoother()
        smoothed = smoother.smooth(path)

        # Should return unchanged
        assert len(smoothed) == 1
