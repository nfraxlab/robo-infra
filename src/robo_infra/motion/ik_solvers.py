"""Inverse kinematics solvers for robot arms.

This module provides various IK solver implementations for computing
joint angles from desired end-effector poses.

Key classes:
- IKSolver: Protocol for all IK solvers
- JacobianIKSolver: Pseudo-inverse Jacobian method
- DampedLeastSquaresIK: Levenberg-Marquardt method (singularity robust)
- CCDIKSolver: Cyclic Coordinate Descent (good for long chains)
- GradientDescentIK: Simple gradient-based solver

Example:
    >>> from robo_infra.motion.ik_solvers import DampedLeastSquaresIK
    >>> from robo_infra.motion.dh_parameters import create_planar_3dof
    >>>
    >>> chain = create_planar_3dof(100, 100, 50)
    >>> solver = DampedLeastSquaresIK(chain)
    >>>
    >>> target = Transform.from_translation(150, 50, 0)
    >>> solution = solver.solve(target)
    >>> if solution is not None:
    ...     print(f"Joint angles: {solution}")
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import numpy as np

from robo_infra.motion.dh_parameters import DHChain, JointType


if TYPE_CHECKING:
    from collections.abc import Sequence

    from numpy.typing import NDArray

    from robo_infra.motion.transforms import Transform


class IKError(Exception):
    """Base exception for IK solver errors."""

    pass


class IKConvergenceError(IKError):
    """IK solver failed to converge."""

    pass


class IKUnreachableError(IKError):
    """Target pose is unreachable."""

    pass


class IKSingularityError(IKError):
    """Encountered singularity during solving."""

    pass


class IKResultStatus(Enum):
    """Status of IK solution."""

    SUCCESS = "success"
    CONVERGED = "converged"
    MAX_ITERATIONS = "max_iterations"
    SINGULARITY = "singularity"
    UNREACHABLE = "unreachable"
    JOINT_LIMITS = "joint_limits"


@dataclass
class IKResult:
    """Result of inverse kinematics computation.

    Attributes:
        success: True if a valid solution was found.
        joint_values: Joint angles (None if unsuccessful).
        status: Status of the solution.
        iterations: Number of iterations used.
        error_position: Final position error (meters).
        error_orientation: Final orientation error (radians).
        final_pose: Achieved end-effector pose.
    """

    success: bool
    joint_values: list[float] | None
    status: IKResultStatus
    iterations: int = 0
    error_position: float = 0.0
    error_orientation: float = 0.0
    final_pose: Transform | None = None


@runtime_checkable
class IKSolver(Protocol):
    """Protocol for inverse kinematics solvers.

    All IK solvers must implement this protocol.
    """

    def solve(
        self,
        target: Transform,
        initial_guess: Sequence[float] | None = None,
    ) -> list[float] | None:
        """Solve inverse kinematics for target pose.

        Args:
            target: Desired end-effector transform.
            initial_guess: Starting joint configuration (optional).

        Returns:
            Joint values achieving the target, or None if no solution found.
        """
        ...

    def solve_detailed(
        self,
        target: Transform,
        initial_guess: Sequence[float] | None = None,
    ) -> IKResult:
        """Solve IK with detailed result information.

        Args:
            target: Desired end-effector transform.
            initial_guess: Starting joint configuration (optional).

        Returns:
            IKResult with solution details.
        """
        ...


@dataclass
class IKSolverConfig:
    """Configuration for IK solvers.

    Attributes:
        max_iterations: Maximum iterations for iterative solvers.
        position_tolerance: Position error tolerance (meters).
        orientation_tolerance: Orientation error tolerance (radians).
        step_size: Step size for gradient-based methods.
        damping: Damping factor for DLS solver.
        use_joint_limits: Whether to enforce joint limits.
        random_restarts: Number of random restarts on failure.
    """

    max_iterations: int = 100
    position_tolerance: float = 1e-4
    orientation_tolerance: float = 1e-3
    step_size: float = 0.1
    damping: float = 0.01
    use_joint_limits: bool = True
    random_restarts: int = 5


class BaseIKSolver(ABC):
    """Base class for IK solvers.

    Provides common functionality for all solvers.
    """

    def __init__(
        self,
        chain: DHChain,
        config: IKSolverConfig | None = None,
    ) -> None:
        """Initialize solver.

        Args:
            chain: Kinematic chain to solve for.
            config: Solver configuration.
        """
        self.chain = chain
        self.config = config or IKSolverConfig()

    @abstractmethod
    def _solve_impl(
        self,
        target: Transform,
        initial: list[float],
    ) -> IKResult:
        """Internal solve implementation."""
        ...

    def solve(
        self,
        target: Transform,
        initial_guess: Sequence[float] | None = None,
    ) -> list[float] | None:
        """Solve inverse kinematics.

        Args:
            target: Desired end-effector transform.
            initial_guess: Starting joint configuration.

        Returns:
            Joint values or None if no solution found.
        """
        result = self.solve_detailed(target, initial_guess)
        return result.joint_values if result.success else None

    def solve_detailed(
        self,
        target: Transform,
        initial_guess: Sequence[float] | None = None,
    ) -> IKResult:
        """Solve IK with detailed results.

        Args:
            target: Desired end-effector transform.
            initial_guess: Starting joint configuration.

        Returns:
            IKResult with solution details.
        """
        # Get initial guess
        if initial_guess is not None:
            initial = list(initial_guess)
        else:
            initial = [0.0] * self.chain.num_joints

        # Try solving
        result = self._solve_impl(target, initial)

        # Try random restarts if failed
        if not result.success and self.config.random_restarts > 0:
            for _ in range(self.config.random_restarts):
                random_initial = self._random_configuration()
                restart_result = self._solve_impl(target, random_initial)
                if restart_result.success:
                    return restart_result
                # Keep best result
                if restart_result.error_position < result.error_position:
                    result = restart_result

        return result

    def _random_configuration(self) -> list[float]:
        """Generate random joint configuration within limits."""
        config: list[float] = []
        for param in self.chain.parameters:
            low = param.limit.min_val
            high = param.limit.max_val
            config.append(float(np.random.uniform(low, high)))
        return config

    def _compute_error(
        self,
        current: Transform,
        target: Transform,
    ) -> tuple[NDArray[np.float64], float, float]:
        """Compute pose error between current and target.

        Returns:
            Tuple of (error_vector, position_error, orientation_error).
        """
        # Position error
        pos_error = target.position - current.position
        pos_error_mag = float(np.linalg.norm(pos_error))

        # Orientation error (axis-angle)
        R_error = current.rotation.inverse() @ target.rotation
        axis, angle = R_error.as_axis_angle(degrees=False)
        rot_error = axis * angle
        rot_error_mag = abs(angle)

        # Combined error vector [position; orientation]
        error = np.concatenate([pos_error, rot_error])

        return error, pos_error_mag, rot_error_mag

    def _is_converged(self, pos_error: float, rot_error: float) -> bool:
        """Check if error is within tolerance."""
        return (
            pos_error < self.config.position_tolerance
            and rot_error < self.config.orientation_tolerance
        )

    def _enforce_limits(self, joint_values: list[float]) -> list[float]:
        """Enforce joint limits if configured."""
        if not self.config.use_joint_limits:
            return joint_values
        return self.chain.clamp_to_limits(joint_values)


class JacobianIKSolver(BaseIKSolver):
    """Jacobian pseudo-inverse IK solver.

    Uses the pseudo-inverse of the Jacobian matrix to compute
    joint velocity updates.

    Fast but can be unstable near singularities.
    """

    def _solve_impl(
        self,
        target: Transform,
        initial: list[float],
    ) -> IKResult:
        """Solve using Jacobian pseudo-inverse."""
        q = np.array(initial, dtype=np.float64)

        for i in range(self.config.max_iterations):
            # Current pose
            current = self.chain.forward(q.tolist())

            # Compute error
            error, pos_err, rot_err = self._compute_error(current, target)

            # Check convergence
            if self._is_converged(pos_err, rot_err):
                q_list = self._enforce_limits(q.tolist())
                return IKResult(
                    success=True,
                    joint_values=q_list,
                    status=IKResultStatus.SUCCESS,
                    iterations=i + 1,
                    error_position=pos_err,
                    error_orientation=rot_err,
                    final_pose=self.chain.forward(q_list),
                )

            # Compute Jacobian
            J = self.chain.jacobian(q.tolist())

            # Check for singularity
            if self.chain.is_singular(q.tolist()):
                return IKResult(
                    success=False,
                    joint_values=None,
                    status=IKResultStatus.SINGULARITY,
                    iterations=i + 1,
                    error_position=pos_err,
                    error_orientation=rot_err,
                )

            # Pseudo-inverse
            J_pinv = np.linalg.pinv(J)

            # Update
            dq = self.config.step_size * (J_pinv @ error)
            q = q + dq

            # Enforce limits
            if self.config.use_joint_limits:
                q = np.array(self.chain.clamp_to_limits(q.tolist()))

        # Max iterations reached
        current = self.chain.forward(q.tolist())
        error, pos_err, rot_err = self._compute_error(current, target)

        return IKResult(
            success=False,
            joint_values=q.tolist(),
            status=IKResultStatus.MAX_ITERATIONS,
            iterations=self.config.max_iterations,
            error_position=pos_err,
            error_orientation=rot_err,
            final_pose=current,
        )


class DampedLeastSquaresIK(BaseIKSolver):
    """Damped Least Squares (Levenberg-Marquardt) IK solver.

    More robust than pure pseudo-inverse near singularities.
    Uses damping to prevent large joint velocities.

    J^# = J^T @ (J @ J^T + λ²I)^{-1}

    This is the recommended solver for most applications.
    """

    def _solve_impl(
        self,
        target: Transform,
        initial: list[float],
    ) -> IKResult:
        """Solve using damped least squares."""
        q = np.array(initial, dtype=np.float64)
        damping = self.config.damping

        for i in range(self.config.max_iterations):
            # Current pose
            current = self.chain.forward(q.tolist())

            # Compute error
            error, pos_err, rot_err = self._compute_error(current, target)

            # Check convergence
            if self._is_converged(pos_err, rot_err):
                q_list = self._enforce_limits(q.tolist())
                return IKResult(
                    success=True,
                    joint_values=q_list,
                    status=IKResultStatus.SUCCESS,
                    iterations=i + 1,
                    error_position=pos_err,
                    error_orientation=rot_err,
                    final_pose=self.chain.forward(q_list),
                )

            # Compute Jacobian
            J = self.chain.jacobian(q.tolist())

            # Damped least squares: J^# = J^T @ (J @ J^T + λ²I)^{-1}
            JJT = J @ J.T
            damping_matrix = damping * damping * np.eye(6)

            try:
                J_dls = J.T @ np.linalg.inv(JJT + damping_matrix)
            except np.linalg.LinAlgError:
                # Singular matrix, increase damping
                damping *= 2
                continue

            # Update
            dq = self.config.step_size * (J_dls @ error)
            q = q + dq

            # Enforce limits
            if self.config.use_joint_limits:
                q = np.array(self.chain.clamp_to_limits(q.tolist()))

        # Max iterations reached
        current = self.chain.forward(q.tolist())
        error, pos_err, rot_err = self._compute_error(current, target)

        return IKResult(
            success=False,
            joint_values=q.tolist(),
            status=IKResultStatus.MAX_ITERATIONS,
            iterations=self.config.max_iterations,
            error_position=pos_err,
            error_orientation=rot_err,
            final_pose=current,
        )


class CCDIKSolver(BaseIKSolver):
    """Cyclic Coordinate Descent IK solver.

    Iteratively adjusts one joint at a time to minimize end-effector error.
    Works well for long kinematic chains and real-time applications.

    Does not require computing Jacobian, making it simpler and faster
    for high-DOF systems.
    """

    def __init__(
        self,
        chain: DHChain,
        config: IKSolverConfig | None = None,
        position_only: bool = False,
    ) -> None:
        """Initialize CCD solver.

        Args:
            chain: Kinematic chain.
            config: Solver configuration.
            position_only: If True, only solve for position (ignore orientation).
        """
        super().__init__(chain, config)
        self.position_only = position_only

    def _solve_impl(
        self,
        target: Transform,
        initial: list[float],
    ) -> IKResult:
        """Solve using Cyclic Coordinate Descent."""
        q = list(initial)
        n = self.chain.num_joints
        target_pos = target.position

        for iteration in range(self.config.max_iterations):
            # Check convergence
            current = self.chain.forward(q)
            _, pos_err, rot_err = self._compute_error(current, target)

            if self.position_only:
                # Only check position for position-only mode
                if pos_err < self.config.position_tolerance:
                    return IKResult(
                        success=True,
                        joint_values=self._enforce_limits(q),
                        status=IKResultStatus.SUCCESS,
                        iterations=iteration + 1,
                        error_position=pos_err,
                        error_orientation=rot_err,
                        final_pose=current,
                    )
            elif self._is_converged(pos_err, rot_err):
                return IKResult(
                    success=True,
                    joint_values=self._enforce_limits(q),
                    status=IKResultStatus.SUCCESS,
                    iterations=iteration + 1,
                    error_position=pos_err,
                    error_orientation=rot_err,
                    final_pose=current,
                )

            # Iterate through joints from end to base
            for j in range(n - 1, -1, -1):
                # Skip prismatic joints for now (CCD works best with revolute)
                if self.chain.parameters[j].joint_type != JointType.REVOLUTE:
                    continue

                # Get current end effector position
                transforms = self.chain.forward_all(q)
                end_pos = transforms[-1].position

                # Get joint frame
                joint_transform = transforms[j - 1] if j > 0 else self.chain.base_transform

                joint_pos = joint_transform.position
                joint_axis = joint_transform.rotation.apply(np.array([0, 0, 1]))

                # Vector from joint to end effector
                to_end = end_pos - joint_pos

                # Vector from joint to target
                to_target = target_pos - joint_pos

                # Project onto plane perpendicular to rotation axis
                to_end_proj = to_end - np.dot(to_end, joint_axis) * joint_axis
                to_target_proj = to_target - np.dot(to_target, joint_axis) * joint_axis

                # Normalize
                len_end = np.linalg.norm(to_end_proj)
                len_target = np.linalg.norm(to_target_proj)

                if len_end < 1e-6 or len_target < 1e-6:
                    continue

                to_end_proj = to_end_proj / len_end
                to_target_proj = to_target_proj / len_target

                # Compute rotation angle
                cos_angle = np.clip(np.dot(to_end_proj, to_target_proj), -1.0, 1.0)
                angle = math.acos(cos_angle)

                # Determine sign using cross product
                cross = np.cross(to_end_proj, to_target_proj)
                if np.dot(cross, joint_axis) < 0:
                    angle = -angle

                # Update joint
                q[j] += angle

                # Enforce limits
                if self.config.use_joint_limits:
                    q[j] = self.chain.parameters[j].limit.clamp(q[j])

        # Max iterations reached
        current = self.chain.forward(q)
        _, pos_err, rot_err = self._compute_error(current, target)

        return IKResult(
            success=False,
            joint_values=self._enforce_limits(q),
            status=IKResultStatus.MAX_ITERATIONS,
            iterations=self.config.max_iterations,
            error_position=pos_err,
            error_orientation=rot_err,
            final_pose=current,
        )


class GradientDescentIK(BaseIKSolver):
    """Gradient Descent IK solver.

    Simple gradient-based optimizer that minimizes the pose error.
    Uses numerical gradients.

    Slower than Jacobian methods but conceptually simple.
    """

    def __init__(
        self,
        chain: DHChain,
        config: IKSolverConfig | None = None,
        position_weight: float = 1.0,
        orientation_weight: float = 1.0,
    ) -> None:
        """Initialize gradient descent solver.

        Args:
            chain: Kinematic chain.
            config: Solver configuration.
            position_weight: Weight for position error.
            orientation_weight: Weight for orientation error.
        """
        super().__init__(chain, config)
        self.position_weight = position_weight
        self.orientation_weight = orientation_weight

    def _cost_function(
        self,
        q: list[float],
        target: Transform,
    ) -> float:
        """Compute cost (sum of weighted squared errors)."""
        current = self.chain.forward(q)
        _, pos_err, rot_err = self._compute_error(current, target)

        return (
            self.position_weight * pos_err * pos_err + self.orientation_weight * rot_err * rot_err
        )

    def _compute_gradient(
        self,
        q: list[float],
        target: Transform,
        delta: float = 1e-6,
    ) -> NDArray[np.float64]:
        """Compute numerical gradient of cost function."""
        n = len(q)
        gradient = np.zeros(n, dtype=np.float64)

        cost = self._cost_function(q, target)

        for i in range(n):
            q_plus = q.copy()
            q_plus[i] += delta
            cost_plus = self._cost_function(q_plus, target)
            gradient[i] = (cost_plus - cost) / delta

        return gradient

    def _solve_impl(
        self,
        target: Transform,
        initial: list[float],
    ) -> IKResult:
        """Solve using gradient descent."""
        q = list(initial)

        for i in range(self.config.max_iterations):
            # Check convergence
            current = self.chain.forward(q)
            _, pos_err, rot_err = self._compute_error(current, target)

            if self._is_converged(pos_err, rot_err):
                return IKResult(
                    success=True,
                    joint_values=self._enforce_limits(q),
                    status=IKResultStatus.SUCCESS,
                    iterations=i + 1,
                    error_position=pos_err,
                    error_orientation=rot_err,
                    final_pose=current,
                )

            # Compute gradient
            gradient = self._compute_gradient(q, target)

            # Update (gradient descent step)
            q = [q[j] - self.config.step_size * gradient[j] for j in range(len(q))]

            # Enforce limits
            if self.config.use_joint_limits:
                q = self._enforce_limits(q)

        # Max iterations
        current = self.chain.forward(q)
        _, pos_err, rot_err = self._compute_error(current, target)

        return IKResult(
            success=False,
            joint_values=self._enforce_limits(q),
            status=IKResultStatus.MAX_ITERATIONS,
            iterations=self.config.max_iterations,
            error_position=pos_err,
            error_orientation=rot_err,
            final_pose=current,
        )


# =============================================================================
# Factory Functions
# =============================================================================


def create_ik_solver(
    chain: DHChain,
    method: str = "dls",
    config: IKSolverConfig | None = None,
) -> BaseIKSolver:
    """Create an IK solver for the given chain.

    Args:
        chain: Kinematic chain to solve for.
        method: Solver method ("jacobian", "dls", "ccd", "gradient").
        config: Solver configuration.

    Returns:
        Configured IK solver.

    Example:
        >>> solver = create_ik_solver(chain, method="dls")
        >>> solution = solver.solve(target_pose)
    """
    solvers = {
        "jacobian": JacobianIKSolver,
        "dls": DampedLeastSquaresIK,
        "damped": DampedLeastSquaresIK,
        "ccd": CCDIKSolver,
        "gradient": GradientDescentIK,
    }

    method_lower = method.lower()
    if method_lower not in solvers:
        raise ValueError(f"Unknown IK method: {method}. Available: {list(solvers.keys())}")

    return solvers[method_lower](chain, config)


def solve_ik(
    chain: DHChain,
    target: Transform,
    initial_guess: Sequence[float] | None = None,
    method: str = "dls",
) -> list[float] | None:
    """Convenience function to solve IK.

    Args:
        chain: Kinematic chain.
        target: Desired end-effector pose.
        initial_guess: Starting joint configuration.
        method: Solver method.

    Returns:
        Joint values or None if no solution found.

    Example:
        >>> solution = solve_ik(chain, target_pose, method="dls")
    """
    solver = create_ik_solver(chain, method)
    return solver.solve(target, initial_guess)
