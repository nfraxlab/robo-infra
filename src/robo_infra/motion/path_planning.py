"""Path planning algorithms for motion in joint and Cartesian space.

This module provides path planning capabilities for robotics applications,
including simple linear interpolation, Cartesian space planning, and
obstacle-avoidance algorithms like RRT.

Classes:
    PathPoint: A point along a path with optional metadata.
    Path: A sequence of points forming a path.
    Obstacle: Abstract base for collision detection.
    SphereObstacle: Spherical obstacle for collision detection.
    BoxObstacle: Axis-aligned box obstacle for collision detection.
    PathPlanner: Protocol for path planning algorithms.
    LinearPathPlanner: Simple linear interpolation in joint space.
    CartesianPathPlanner: Linear interpolation in Cartesian space.
    RRTPathPlanner: Rapidly-exploring Random Tree for obstacle avoidance.
    PathSmoother: Path smoothing using B-splines or Bezier curves.

Example:
    >>> from robo_infra.motion.path_planning import LinearPathPlanner, Path
    >>> planner = LinearPathPlanner(num_points=50)
    >>> path = planner.plan(start=[0.0, 0.0], goal=[1.0, 1.0])
    >>> len(path.points)
    50
"""

from __future__ import annotations

import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Protocol, runtime_checkable


if TYPE_CHECKING:
    from collections.abc import Callable


class PathStatus(Enum):
    """Status of a path planning result."""

    SUCCESS = "success"
    FAILURE = "failure"
    PARTIAL = "partial"  # Path found but doesn't reach goal exactly
    TIMEOUT = "timeout"  # Planning timed out
    COLLISION = "collision"  # Path has collision


@dataclass(frozen=True, slots=True)
class PathPoint:
    """A point along a path.

    Attributes:
        position: Position values (joint angles or Cartesian coordinates).
        cost: Optional cost/distance from start.
        metadata: Optional additional data for the point.
    """

    position: tuple[float, ...]
    cost: float = 0.0
    metadata: dict[str, float] = field(default_factory=dict)

    def distance_to(self, other: PathPoint) -> float:
        """Calculate Euclidean distance to another point.

        Args:
            other: Another PathPoint.

        Returns:
            Euclidean distance between points.
        """
        if len(self.position) != len(other.position):
            raise ValueError(
                f"Position dimensions don't match: {len(self.position)} vs {len(other.position)}"
            )
        return math.sqrt(
            sum((a - b) ** 2 for a, b in zip(self.position, other.position, strict=True))
        )

    def interpolate(self, other: PathPoint, t: float) -> PathPoint:
        """Linear interpolation between this point and another.

        Args:
            other: Target point.
            t: Interpolation parameter [0, 1].

        Returns:
            Interpolated PathPoint.
        """
        t = max(0.0, min(1.0, t))
        new_position = tuple(
            a + t * (b - a) for a, b in zip(self.position, other.position, strict=True)
        )
        new_cost = self.cost + t * (other.cost - self.cost)
        return PathPoint(position=new_position, cost=new_cost)


@dataclass(slots=True)
class Path:
    """A sequence of points forming a path.

    Attributes:
        points: List of PathPoints along the path.
        status: Status of the path planning result.
        total_cost: Total cost/length of the path.
    """

    points: list[PathPoint] = field(default_factory=list)
    status: PathStatus = PathStatus.SUCCESS
    total_cost: float = 0.0

    def __len__(self) -> int:
        """Return the number of points in the path."""
        return len(self.points)

    def __iter__(self):
        """Iterate over path points."""
        return iter(self.points)

    def __getitem__(self, index: int) -> PathPoint:
        """Get a path point by index."""
        return self.points[index]

    @property
    def start(self) -> PathPoint | None:
        """Return the start point of the path."""
        return self.points[0] if self.points else None

    @property
    def goal(self) -> PathPoint | None:
        """Return the end point of the path."""
        return self.points[-1] if self.points else None

    @property
    def is_empty(self) -> bool:
        """Check if the path is empty."""
        return len(self.points) == 0

    @property
    def is_valid(self) -> bool:
        """Check if the path is valid (success or partial)."""
        return self.status in (PathStatus.SUCCESS, PathStatus.PARTIAL)

    def calculate_length(self) -> float:
        """Calculate the total length of the path.

        Returns:
            Sum of distances between consecutive points.
        """
        if len(self.points) < 2:
            return 0.0
        return sum(
            self.points[i].distance_to(self.points[i + 1]) for i in range(len(self.points) - 1)
        )

    def append(self, point: PathPoint) -> None:
        """Append a point to the path.

        Args:
            point: PathPoint to add.
        """
        self.points.append(point)

    def reverse(self) -> Path:
        """Return a reversed copy of the path.

        Returns:
            New Path with points in reverse order.
        """
        return Path(
            points=list(reversed(self.points)),
            status=self.status,
            total_cost=self.total_cost,
        )

    def resample(self, num_points: int) -> Path:
        """Resample the path to have a specific number of points.

        Uses linear interpolation to create evenly-spaced points.

        Args:
            num_points: Desired number of points. Must be >= 2.

        Returns:
            New Path with resampled points.

        Raises:
            ValueError: If num_points < 2 or path has < 2 points.
        """
        if num_points < 2:
            raise ValueError(f"num_points must be >= 2, got {num_points}")
        if len(self.points) < 2:
            raise ValueError("Path must have at least 2 points to resample")

        # Calculate total path length and segment lengths
        total_length = self.calculate_length()
        if total_length == 0:
            # All points are the same, just duplicate
            return Path(
                points=[self.points[0]] * num_points,
                status=self.status,
                total_cost=self.total_cost,
            )

        # Calculate cumulative distances
        cumulative = [0.0]
        for i in range(len(self.points) - 1):
            cumulative.append(cumulative[-1] + self.points[i].distance_to(self.points[i + 1]))

        # Generate evenly-spaced points
        new_points: list[PathPoint] = []
        for i in range(num_points):
            target_dist = (i / (num_points - 1)) * total_length

            # Find the segment containing this distance
            for j in range(len(cumulative) - 1):
                if cumulative[j] <= target_dist <= cumulative[j + 1]:
                    segment_length = cumulative[j + 1] - cumulative[j]
                    if segment_length > 0:
                        t = (target_dist - cumulative[j]) / segment_length
                    else:
                        t = 0.0
                    new_points.append(self.points[j].interpolate(self.points[j + 1], t))
                    break
            else:
                # Fallback to last point
                new_points.append(self.points[-1])

        return Path(
            points=new_points,
            status=self.status,
            total_cost=self.total_cost,
        )

    def as_positions(self) -> list[tuple[float, ...]]:
        """Extract positions from all path points.

        Returns:
            List of position tuples.
        """
        return [p.position for p in self.points]

    def as_lists(self) -> list[list[float]]:
        """Extract positions as lists (for compatibility).

        Returns:
            List of position lists.
        """
        return [list(p.position) for p in self.points]


class Obstacle(ABC):
    """Abstract base class for obstacles in path planning.

    Obstacles are used for collision detection during path planning.
    """

    @abstractmethod
    def contains_point(self, point: tuple[float, ...]) -> bool:
        """Check if a point is inside the obstacle.

        Args:
            point: Position to check.

        Returns:
            True if the point is inside the obstacle.
        """
        ...

    @abstractmethod
    def distance_to_point(self, point: tuple[float, ...]) -> float:
        """Calculate the minimum distance from the obstacle to a point.

        Args:
            point: Position to measure from.

        Returns:
            Distance to the obstacle surface (negative if inside).
        """
        ...

    def intersects_segment(
        self, start: tuple[float, ...], end: tuple[float, ...], num_samples: int = 10
    ) -> bool:
        """Check if a line segment intersects the obstacle.

        Uses sampling along the segment for approximate collision detection.

        Args:
            start: Start of the segment.
            end: End of the segment.
            num_samples: Number of samples to check along the segment.

        Returns:
            True if any sample point is inside the obstacle.
        """
        for i in range(num_samples + 1):
            t = i / num_samples
            point = tuple(a + t * (b - a) for a, b in zip(start, end, strict=True))
            if self.contains_point(point):
                return True
        return False


@dataclass(frozen=True, slots=True)
class SphereObstacle(Obstacle):
    """Spherical obstacle for collision detection.

    Works in any number of dimensions.

    Attributes:
        center: Center position of the sphere.
        radius: Radius of the sphere.
    """

    center: tuple[float, ...]
    radius: float

    def __post_init__(self) -> None:
        """Validate obstacle parameters."""
        if self.radius <= 0:
            raise ValueError(f"radius must be > 0, got {self.radius}")

    def contains_point(self, point: tuple[float, ...]) -> bool:
        """Check if a point is inside the sphere."""
        if len(point) != len(self.center):
            raise ValueError(
                f"Point dimension {len(point)} doesn't match obstacle dimension {len(self.center)}"
            )
        dist_sq = sum((a - b) ** 2 for a, b in zip(point, self.center, strict=True))
        return dist_sq <= self.radius**2

    def distance_to_point(self, point: tuple[float, ...]) -> float:
        """Calculate distance from point to sphere surface."""
        if len(point) != len(self.center):
            raise ValueError(
                f"Point dimension {len(point)} doesn't match obstacle dimension {len(self.center)}"
            )
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(point, self.center, strict=True)))
        return dist - self.radius


@dataclass(frozen=True, slots=True)
class BoxObstacle(Obstacle):
    """Axis-aligned box obstacle for collision detection.

    Works in any number of dimensions.

    Attributes:
        min_corner: Minimum corner of the box (lower bounds).
        max_corner: Maximum corner of the box (upper bounds).
    """

    min_corner: tuple[float, ...]
    max_corner: tuple[float, ...]

    def __post_init__(self) -> None:
        """Validate obstacle parameters."""
        if len(self.min_corner) != len(self.max_corner):
            raise ValueError("min_corner and max_corner must have same dimensions")
        for i, (a, b) in enumerate(zip(self.min_corner, self.max_corner, strict=True)):
            if a > b:
                raise ValueError(f"min_corner[{i}]={a} must be <= max_corner[{i}]={b}")

    def contains_point(self, point: tuple[float, ...]) -> bool:
        """Check if a point is inside the box."""
        if len(point) != len(self.min_corner):
            raise ValueError(
                f"Point dimension {len(point)} doesn't match obstacle dimension {len(self.min_corner)}"
            )
        return all(self.min_corner[i] <= point[i] <= self.max_corner[i] for i in range(len(point)))

    def distance_to_point(self, point: tuple[float, ...]) -> float:
        """Calculate distance from point to box surface."""
        if len(point) != len(self.min_corner):
            raise ValueError(
                f"Point dimension {len(point)} doesn't match obstacle dimension {len(self.min_corner)}"
            )

        # Find the closest point on the box to the given point
        closest = tuple(
            max(self.min_corner[i], min(point[i], self.max_corner[i])) for i in range(len(point))
        )

        # If point is inside, distance is negative (distance to nearest edge)
        if self.contains_point(point):
            # Find minimum distance to any edge
            min_dist = float("inf")
            for i in range(len(point)):
                dist_to_min = point[i] - self.min_corner[i]
                dist_to_max = self.max_corner[i] - point[i]
                min_dist = min(min_dist, dist_to_min, dist_to_max)
            return -min_dist

        # Point is outside, return positive distance
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(point, closest, strict=True)))


@runtime_checkable
class PathPlanner(Protocol):
    """Protocol for path planning algorithms.

    All path planners must implement the plan method that takes
    a start and goal position and returns a Path.
    """

    def plan(
        self,
        start: list[float],
        goal: list[float],
        obstacles: list[Obstacle] | None = None,
    ) -> Path:
        """Plan a path from start to goal.

        Args:
            start: Starting position (joint angles or Cartesian coordinates).
            goal: Goal position.
            obstacles: Optional list of obstacles to avoid.

        Returns:
            Path from start to goal.
        """
        ...


class LinearPathPlanner:
    """Simple linear interpolation in joint space.

    Creates a straight-line path between start and goal positions.
    Does not consider obstacles.

    Attributes:
        num_points: Number of points to generate in the path.
    """

    _num_points: int

    __slots__ = ("_num_points",)

    def __init__(self, num_points: int = 50) -> None:
        """Initialize the linear path planner.

        Args:
            num_points: Number of points to generate. Must be >= 2.

        Raises:
            ValueError: If num_points < 2.
        """
        if num_points < 2:
            raise ValueError(f"num_points must be >= 2, got {num_points}")
        self._num_points = num_points

    @property
    def num_points(self) -> int:
        """Return the number of points to generate."""
        return self._num_points

    def plan(
        self,
        start: list[float],
        goal: list[float],
        obstacles: list[Obstacle] | None = None,
    ) -> Path:
        """Plan a linear path from start to goal.

        Note: This planner ignores obstacles. Use RRTPathPlanner for
        obstacle avoidance.

        Args:
            start: Starting position.
            goal: Goal position.
            obstacles: Ignored by this planner.

        Returns:
            Linear path from start to goal.

        Raises:
            ValueError: If start and goal have different dimensions.
        """
        if len(start) != len(goal):
            raise ValueError(f"start dimension {len(start)} must match goal dimension {len(goal)}")

        start_tuple = tuple(start)
        goal_tuple = tuple(goal)

        # Calculate total distance
        total_dist = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(start_tuple, goal_tuple, strict=True))
        )

        # Generate linearly interpolated points
        points: list[PathPoint] = []
        for i in range(self._num_points):
            t = i / (self._num_points - 1)
            position = tuple(a + t * (b - a) for a, b in zip(start_tuple, goal_tuple, strict=True))
            cost = t * total_dist
            points.append(PathPoint(position=position, cost=cost))

        return Path(points=points, status=PathStatus.SUCCESS, total_cost=total_dist)


class CartesianPathPlanner:
    """Linear interpolation in Cartesian space with optional IK.

    Creates a straight-line path in Cartesian (x, y, z) space.
    Can optionally convert to joint space using inverse kinematics.

    Attributes:
        num_points: Number of points to generate.
        ik_function: Optional inverse kinematics function.
    """

    _num_points: int
    _ik_function: Callable[[list[float]], list[float]] | None

    __slots__ = ("_ik_function", "_num_points")

    def __init__(
        self,
        num_points: int = 50,
        ik_function: Callable[[list[float]], list[float]] | None = None,
    ) -> None:
        """Initialize the Cartesian path planner.

        Args:
            num_points: Number of points to generate. Must be >= 2.
            ik_function: Optional function to convert Cartesian to joint space.
                Takes a list of Cartesian coordinates and returns joint angles.

        Raises:
            ValueError: If num_points < 2.
        """
        if num_points < 2:
            raise ValueError(f"num_points must be >= 2, got {num_points}")
        self._num_points = num_points
        self._ik_function = ik_function

    @property
    def num_points(self) -> int:
        """Return the number of points to generate."""
        return self._num_points

    def plan(
        self,
        start: list[float],
        goal: list[float],
        obstacles: list[Obstacle] | None = None,
    ) -> Path:
        """Plan a linear Cartesian path from start to goal.

        If an IK function is provided, the path is converted to joint space.

        Args:
            start: Starting Cartesian position.
            goal: Goal Cartesian position.
            obstacles: Ignored by this planner.

        Returns:
            Path from start to goal (in Cartesian or joint space).

        Raises:
            ValueError: If start and goal have different dimensions.
        """
        if len(start) != len(goal):
            raise ValueError(f"start dimension {len(start)} must match goal dimension {len(goal)}")

        start_tuple = tuple(start)
        goal_tuple = tuple(goal)

        # Calculate total distance
        total_dist = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(start_tuple, goal_tuple, strict=True))
        )

        # Generate linearly interpolated points in Cartesian space
        points: list[PathPoint] = []
        for i in range(self._num_points):
            t = i / (self._num_points - 1)
            cartesian = tuple(a + t * (b - a) for a, b in zip(start_tuple, goal_tuple, strict=True))

            # Convert to joint space if IK function is provided
            if self._ik_function is not None:
                try:
                    joint_pos = tuple(self._ik_function(list(cartesian)))
                    position = joint_pos
                except Exception:
                    # IK failed, return partial path
                    return Path(
                        points=points,
                        status=PathStatus.PARTIAL,
                        total_cost=total_dist,
                    )
            else:
                position = cartesian

            cost = t * total_dist
            points.append(PathPoint(position=position, cost=cost))

        return Path(points=points, status=PathStatus.SUCCESS, total_cost=total_dist)


class RRTPathPlanner:
    """Rapidly-exploring Random Tree path planner.

    An effective algorithm for path planning in high-dimensional
    configuration spaces with obstacles.

    The algorithm works by:
    1. Randomly sampling the configuration space
    2. Finding the nearest existing node
    3. Extending towards the sample
    4. Checking for collisions
    5. Repeating until the goal is reached

    Attributes:
        max_iterations: Maximum iterations before giving up.
        step_size: Maximum distance for each tree extension.
        goal_bias: Probability of sampling the goal directly.
        goal_tolerance: Distance threshold for reaching the goal.
        bounds: Min/max bounds for each dimension.
    """

    _max_iterations: int
    _step_size: float
    _goal_bias: float
    _goal_tolerance: float
    _bounds: list[tuple[float, float]] | None

    __slots__ = (
        "_bounds",
        "_goal_bias",
        "_goal_tolerance",
        "_max_iterations",
        "_step_size",
    )

    def __init__(
        self,
        max_iterations: int = 5000,
        step_size: float = 0.1,
        goal_bias: float = 0.1,
        goal_tolerance: float = 0.05,
        bounds: list[tuple[float, float]] | None = None,
    ) -> None:
        """Initialize the RRT path planner.

        Args:
            max_iterations: Maximum iterations before failing. Must be > 0.
            step_size: Maximum distance for tree extension. Must be > 0.
            goal_bias: Probability of sampling goal [0, 1].
            goal_tolerance: Distance to consider goal reached. Must be > 0.
            bounds: Optional bounds for each dimension [(min, max), ...].

        Raises:
            ValueError: If parameters are invalid.
        """
        if max_iterations <= 0:
            raise ValueError(f"max_iterations must be > 0, got {max_iterations}")
        if step_size <= 0:
            raise ValueError(f"step_size must be > 0, got {step_size}")
        if not 0 <= goal_bias <= 1:
            raise ValueError(f"goal_bias must be in [0, 1], got {goal_bias}")
        if goal_tolerance <= 0:
            raise ValueError(f"goal_tolerance must be > 0, got {goal_tolerance}")

        self._max_iterations = max_iterations
        self._step_size = step_size
        self._goal_bias = goal_bias
        self._goal_tolerance = goal_tolerance
        self._bounds = bounds

    def _random_sample(self, dim: int, goal: tuple[float, ...]) -> tuple[float, ...]:
        """Generate a random sample in the configuration space.

        Args:
            dim: Number of dimensions.
            goal: Goal position (used for goal biasing).

        Returns:
            Random sample point.
        """
        if random.random() < self._goal_bias:
            return goal

        if self._bounds is not None and len(self._bounds) == dim:
            return tuple(random.uniform(self._bounds[i][0], self._bounds[i][1]) for i in range(dim))
        else:
            # Default to [-pi, pi] for joint space
            return tuple(random.uniform(-math.pi, math.pi) for _ in range(dim))

    def _nearest_node(
        self, tree: list[tuple[tuple[float, ...], int]], point: tuple[float, ...]
    ) -> int:
        """Find the index of the nearest node in the tree.

        Args:
            tree: List of (position, parent_index) tuples.
            point: Query point.

        Returns:
            Index of the nearest node.
        """
        min_dist = float("inf")
        nearest_idx = 0
        for i, (node_pos, _) in enumerate(tree):
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(node_pos, point, strict=True)))
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx

    def _steer(
        self, from_point: tuple[float, ...], to_point: tuple[float, ...]
    ) -> tuple[float, ...]:
        """Steer from one point towards another with limited step size.

        Args:
            from_point: Starting point.
            to_point: Target point.

        Returns:
            New point at most step_size away from from_point.
        """
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(from_point, to_point, strict=True)))
        if dist <= self._step_size:
            return to_point

        # Scale to step_size
        ratio = self._step_size / dist
        return tuple(a + ratio * (b - a) for a, b in zip(from_point, to_point, strict=True))

    def _is_collision_free(
        self,
        from_point: tuple[float, ...],
        to_point: tuple[float, ...],
        obstacles: list[Obstacle],
    ) -> bool:
        """Check if a path segment is collision-free.

        Args:
            from_point: Start of segment.
            to_point: End of segment.
            obstacles: List of obstacles to check.

        Returns:
            True if the segment is collision-free.
        """
        return all(not obstacle.intersects_segment(from_point, to_point) for obstacle in obstacles)

    def _extract_path(self, tree: list[tuple[tuple[float, ...], int]], goal_idx: int) -> Path:
        """Extract the path from the tree by backtracking from goal.

        Args:
            tree: The RRT tree.
            goal_idx: Index of the goal node.

        Returns:
            Path from start to goal.
        """
        path_points: list[PathPoint] = []
        current_idx = goal_idx

        while current_idx != -1:
            node_pos, parent_idx = tree[current_idx]
            path_points.append(PathPoint(position=node_pos, cost=0.0))
            current_idx = parent_idx

        # Reverse to get start-to-goal order
        path_points.reverse()

        # Calculate costs
        total_cost = 0.0
        for i, point in enumerate(path_points):
            if i > 0:
                total_cost += path_points[i - 1].distance_to(point)
            # Create new point with updated cost
            path_points[i] = PathPoint(position=point.position, cost=total_cost)

        return Path(points=path_points, status=PathStatus.SUCCESS, total_cost=total_cost)

    def plan(
        self,
        start: list[float],
        goal: list[float],
        obstacles: list[Obstacle] | None = None,
    ) -> Path:
        """Plan a path from start to goal avoiding obstacles.

        Args:
            start: Starting position.
            goal: Goal position.
            obstacles: List of obstacles to avoid.

        Returns:
            Path from start to goal, or failure status if not found.

        Raises:
            ValueError: If start and goal have different dimensions.
        """
        if len(start) != len(goal):
            raise ValueError(f"start dimension {len(start)} must match goal dimension {len(goal)}")

        start_tuple = tuple(start)
        goal_tuple = tuple(goal)
        dim = len(start)
        obstacles = obstacles or []

        # Check if start or goal is in collision
        for obstacle in obstacles:
            if obstacle.contains_point(start_tuple):
                return Path(
                    points=[PathPoint(position=start_tuple, cost=0.0)],
                    status=PathStatus.COLLISION,
                    total_cost=0.0,
                )
            if obstacle.contains_point(goal_tuple):
                return Path(
                    points=[PathPoint(position=start_tuple, cost=0.0)],
                    status=PathStatus.COLLISION,
                    total_cost=0.0,
                )

        # Check if direct path is collision-free
        if not obstacles or self._is_collision_free(start_tuple, goal_tuple, obstacles):
            # Use linear interpolation
            linear_planner = LinearPathPlanner(num_points=50)
            return linear_planner.plan(start, goal)

        # RRT algorithm
        # Tree: list of (position, parent_index)
        tree: list[tuple[tuple[float, ...], int]] = [(start_tuple, -1)]

        for _iteration in range(self._max_iterations):
            # Sample random point (with goal bias)
            random_point = self._random_sample(dim, goal_tuple)

            # Find nearest node in tree
            nearest_idx = self._nearest_node(tree, random_point)
            nearest_pos, _ = tree[nearest_idx]

            # Steer towards random point
            new_point = self._steer(nearest_pos, random_point)

            # Check collision
            if self._is_collision_free(nearest_pos, new_point, obstacles):
                # Add to tree
                tree.append((new_point, nearest_idx))

                # Check if we reached the goal
                goal_dist = math.sqrt(
                    sum((a - b) ** 2 for a, b in zip(new_point, goal_tuple, strict=True))
                )
                if goal_dist <= self._goal_tolerance:
                    # Add goal node and extract path
                    tree.append((goal_tuple, len(tree) - 1))
                    return self._extract_path(tree, len(tree) - 1)

        # Failed to find path
        return Path(
            points=[PathPoint(position=start_tuple, cost=0.0)],
            status=PathStatus.TIMEOUT,
            total_cost=0.0,
        )


class SmoothingMethod(Enum):
    """Method for path smoothing."""

    BSPLINE = "bspline"
    BEZIER = "bezier"
    SHORTCUT = "shortcut"


class PathSmoother:
    """Path smoothing using B-splines, Bezier curves, or shortcuts.

    Smooths a path to reduce jerkiness and improve motion quality.

    Attributes:
        method: Smoothing method to use.
        iterations: Number of smoothing iterations.
    """

    _method: SmoothingMethod
    _iterations: int
    _num_output_points: int

    __slots__ = ("_iterations", "_method", "_num_output_points")

    def __init__(
        self,
        method: SmoothingMethod = SmoothingMethod.BSPLINE,
        iterations: int = 1,
        num_output_points: int = 100,
    ) -> None:
        """Initialize the path smoother.

        Args:
            method: Smoothing method to use.
            iterations: Number of smoothing iterations. Must be >= 1.
            num_output_points: Number of points in smoothed path.

        Raises:
            ValueError: If iterations < 1 or num_output_points < 2.
        """
        if iterations < 1:
            raise ValueError(f"iterations must be >= 1, got {iterations}")
        if num_output_points < 2:
            raise ValueError(f"num_output_points must be >= 2, got {num_output_points}")

        self._method = method
        self._iterations = iterations
        self._num_output_points = num_output_points

    def _bezier_point(self, control_points: list[tuple[float, ...]], t: float) -> tuple[float, ...]:
        """Calculate a point on a Bezier curve using De Casteljau's algorithm.

        Args:
            control_points: Control points of the Bezier curve.
            t: Parameter [0, 1].

        Returns:
            Point on the curve.
        """
        points = list(control_points)
        n = len(points)
        for r in range(1, n):
            for i in range(n - r):
                points[i] = tuple(
                    (1 - t) * points[i][j] + t * points[i + 1][j] for j in range(len(points[i]))
                )
        return points[0]

    def _bspline_point(
        self, control_points: list[tuple[float, ...]], t: float, degree: int = 3
    ) -> tuple[float, ...]:
        """Calculate a point on a B-spline curve.

        Uses a uniform B-spline with the specified degree.

        Args:
            control_points: Control points.
            t: Global parameter [0, 1].
            degree: B-spline degree.

        Returns:
            Point on the curve.
        """
        n = len(control_points)
        if n < degree + 1:
            # Not enough points, fall back to Bezier
            return self._bezier_point(control_points, t)

        # Uniform knot vector
        num_knots = n + degree + 1
        knots = [i / (num_knots - 1) for i in range(num_knots)]

        # De Boor's algorithm
        # Find the span
        span = degree
        for i in range(degree, n):
            if knots[i] <= t < knots[i + 1]:
                span = i
                break
        if t >= 1.0:
            span = n - 1

        # Initialize control points for this segment
        d = [control_points[j] for j in range(span - degree, span + 1)]

        for r in range(1, degree + 1):
            for j in range(degree, r - 1, -1):
                left_knot = knots[span - degree + j]
                right_knot = knots[span + 1 + j - r]
                denom = right_knot - left_knot
                alpha = (t - left_knot) / denom if denom != 0 else 0.0
                d[j] = tuple((1 - alpha) * d[j - 1][k] + alpha * d[j][k] for k in range(len(d[j])))

        return d[degree]

    def _shortcut_smooth(self, path: Path, obstacles: list[Obstacle] | None) -> Path:
        """Smooth path using random shortcutting.

        Tries to connect non-adjacent points with straight lines.

        Args:
            path: Input path.
            obstacles: Obstacles to avoid.

        Returns:
            Smoothed path.
        """
        if len(path.points) < 3:
            return path

        points = list(path.points)
        obstacles = obstacles or []

        for _ in range(self._iterations * len(points)):
            if len(points) < 3:
                break

            # Pick two random non-adjacent points
            i = random.randint(0, len(points) - 3)
            j = random.randint(i + 2, len(points) - 1)

            # Check if shortcut is collision-free
            if not obstacles or self._is_collision_free(
                points[i].position, points[j].position, obstacles
            ):
                # Remove intermediate points
                points = points[: i + 1] + points[j:]

        return Path(
            points=points,
            status=path.status,
            total_cost=sum(points[i].distance_to(points[i + 1]) for i in range(len(points) - 1)),
        )

    def _is_collision_free(
        self,
        from_point: tuple[float, ...],
        to_point: tuple[float, ...],
        obstacles: list[Obstacle],
    ) -> bool:
        """Check if a path segment is collision-free."""
        return all(not obstacle.intersects_segment(from_point, to_point) for obstacle in obstacles)

    def smooth(self, path: Path, obstacles: list[Obstacle] | None = None) -> Path:
        """Smooth a path using the configured method.

        Args:
            path: Input path.
            obstacles: Optional obstacles (used for shortcut method).

        Returns:
            Smoothed path.
        """
        if len(path.points) < 2:
            return path

        if self._method == SmoothingMethod.SHORTCUT:
            return self._shortcut_smooth(path, obstacles)

        # Get control points
        control_points = [p.position for p in path.points]

        # Generate smoothed points
        new_points: list[PathPoint] = []
        for i in range(self._num_output_points):
            t = i / (self._num_output_points - 1)

            if self._method == SmoothingMethod.BEZIER:
                pos = self._bezier_point(control_points, t)
            else:  # BSPLINE
                pos = self._bspline_point(control_points, t)

            new_points.append(PathPoint(position=pos, cost=0.0))

        # Calculate costs
        total_cost = 0.0
        for i in range(len(new_points)):
            if i > 0:
                total_cost += new_points[i - 1].distance_to(new_points[i])
            new_points[i] = PathPoint(position=new_points[i].position, cost=total_cost)

        return Path(
            points=new_points,
            status=path.status,
            total_cost=total_cost,
        )
