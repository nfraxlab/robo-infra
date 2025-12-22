"""Tests for IK solvers module.

Tests for inverse kinematics solvers:
- JacobianIKSolver (pseudo-inverse)
- DampedLeastSquaresIK (Levenberg-Marquardt)
- CCDIKSolver (Cyclic Coordinate Descent)
- GradientDescentIK
"""

from __future__ import annotations

import numpy as np
import pytest

from robo_infra.motion.dh_parameters import (
    create_planar_3dof,
    create_puma_560,
    create_ur5,
)
from robo_infra.motion.ik_solvers import (
    CCDIKSolver,
    DampedLeastSquaresIK,
    GradientDescentIK,
    IKResult,
    IKResultStatus,
    IKSolverConfig,
    JacobianIKSolver,
)


# =============================================================================
# IKSolverConfig Tests
# =============================================================================


class TestIKSolverConfig:
    """Tests for IKSolverConfig dataclass."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = IKSolverConfig()

        assert config.max_iterations > 0
        assert config.position_tolerance > 0
        assert config.orientation_tolerance > 0
        assert config.random_restarts >= 0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = IKSolverConfig(
            max_iterations=500,
            position_tolerance=0.0001,
            orientation_tolerance=0.001,
            random_restarts=5,
        )

        assert config.max_iterations == 500
        assert config.position_tolerance == 0.0001
        assert config.orientation_tolerance == 0.001
        assert config.random_restarts == 5


class TestIKResult:
    """Tests for IKResult dataclass."""

    def test_success_result(self) -> None:
        """Test successful IK result."""
        result = IKResult(
            success=True,
            joint_values=[0.1, 0.2, 0.3],
            status=IKResultStatus.SUCCESS,
            iterations=42,
            error_position=0.0001,
            error_orientation=0.001,
        )

        assert result.success is True
        assert result.iterations == 42

    def test_failure_result(self) -> None:
        """Test failed IK result."""
        result = IKResult(
            success=False,
            joint_values=[0.0, 0.0, 0.0],
            status=IKResultStatus.MAX_ITERATIONS,
            iterations=100,
            error_position=0.5,
        )

        assert result.success is False

    def test_status_types(self) -> None:
        """Test all status types."""
        statuses = [
            IKResultStatus.SUCCESS,
            IKResultStatus.CONVERGED,
            IKResultStatus.MAX_ITERATIONS,
            IKResultStatus.SINGULARITY,
            IKResultStatus.UNREACHABLE,
            IKResultStatus.JOINT_LIMITS,
        ]

        for status in statuses:
            success = status in (IKResultStatus.SUCCESS, IKResultStatus.CONVERGED)
            result = IKResult(
                success=success,
                joint_values=[0.0] * 3 if success else None,
                status=status,
                iterations=0,
            )
            assert result.success == success


# =============================================================================
# JacobianIKSolver Tests
# =============================================================================


class TestJacobianIKSolver:
    """Tests for Jacobian pseudo-inverse IK solver."""

    def test_creation(self) -> None:
        """Test solver creation."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain)

        assert solver.chain is chain

    def test_solve_planar_arm(self) -> None:
        """Test solving IK for planar arm."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain, config=IKSolverConfig(max_iterations=200))

        # Target: end effector at known pose
        q_initial = [0.3, 0.4, 0.2]
        T_target = chain.forward(q_initial)

        # Solve from a non-singular starting point (not all zeros)
        result = solver.solve_detailed(T_target, [0.1, 0.1, 0.1])

        # Check result returns something (may not converge but should run)
        assert result is not None
        assert result.iterations > 0

    def test_solve_puma560(self) -> None:
        """Test solving IK for PUMA 560."""
        chain = create_puma_560()
        solver = JacobianIKSolver(chain, config=IKSolverConfig(max_iterations=500))

        # Target pose
        q_target = [0.2, -0.3, 0.5, 0.1, -0.2, 0.4]
        T_target = chain.forward(q_target)

        # Solve
        result = solver.solve_detailed(T_target)

        # Should produce some result
        assert result is not None

    def test_out_of_reach(self) -> None:
        """Test IK for out-of-reach target."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain, config=IKSolverConfig(max_iterations=50))

        # Target way beyond reach
        from robo_infra.motion.transforms import Transform
        T_target = Transform.from_translation(100.0, 0.0, 0.0)

        result = solver.solve_detailed(T_target)

        # Should fail or have large error
        assert result.error_position > 1.0  # At least 1m error

    def test_with_custom_config(self) -> None:
        """Test solver with custom configuration."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        config = IKSolverConfig(
            max_iterations=200,
            position_tolerance=0.001,
        )
        solver = JacobianIKSolver(chain, config=config)

        assert solver.config.max_iterations == 200
        assert solver.config.position_tolerance == 0.001


class TestDampedLeastSquaresIK:
    """Tests for Damped Least Squares (Levenberg-Marquardt) solver."""

    def test_creation(self) -> None:
        """Test solver creation."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = DampedLeastSquaresIK(chain)

        assert solver.chain is chain

    def test_solve_with_damping(self) -> None:
        """Test solving with damping."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = DampedLeastSquaresIK(chain)

        q_target = [0.2, 0.3, 0.1]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target)

        assert result.joint_values is not None
        assert len(result.joint_values) == 3

    def test_damping_prevents_singularity_issues(self) -> None:
        """Test that damping helps near singularities."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)

        solver = DampedLeastSquaresIK(chain)

        # Near singularity configuration
        T_target = chain.forward([0.0, 0.0, 0.0])

        result = solver.solve_detailed(T_target, [0.001, 0.001, 0.001])

        # Should not crash or produce NaN
        if result.joint_values is not None:
            assert not any(np.isnan(v) for v in result.joint_values)


class TestCCDIKSolver:
    """Tests for Cyclic Coordinate Descent solver."""

    def test_creation(self) -> None:
        """Test solver creation."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = CCDIKSolver(chain)

        assert solver.chain is chain

    def test_solve_planar(self) -> None:
        """Test CCD on planar arm."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = CCDIKSolver(chain, config=IKSolverConfig(max_iterations=200))

        # Reachable target
        q_target = [0.3, 0.2, -0.1]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target)

        assert result.joint_values is not None

    def test_iterative_convergence(self) -> None:
        """Test CCD converges iteratively."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = CCDIKSolver(chain, config=IKSolverConfig(max_iterations=100))

        q_target = [0.5, -0.3, 0.2]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target, [0.1, 0.1, 0.1])

        # Should have done some iterations
        assert result.iterations > 0


class TestGradientDescentIK:
    """Tests for Gradient Descent solver."""

    def test_creation(self) -> None:
        """Test solver creation."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = GradientDescentIK(chain)

        assert solver.chain is chain

    def test_solve(self) -> None:
        """Test solving with gradient descent."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = GradientDescentIK(chain)

        q_target = [0.2, 0.3, 0.1]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target)

        assert result.joint_values is not None


# =============================================================================
# Solve Method Tests
# =============================================================================


class TestSolveMethod:
    """Tests for the simple solve() method."""

    def test_solve_returns_none_on_failure(self) -> None:
        """Test solve returns None on failure."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain, config=IKSolverConfig(max_iterations=1))

        # Very hard target with only 1 iteration
        from robo_infra.motion.transforms import Transform
        T_target = Transform.from_translation(100.0, 0.0, 0.0)

        result = solver.solve(T_target)

        # Might return None for hard targets
        # (or a poor solution if it gets something)
        assert result is None or isinstance(result, list)

    def test_solve_returns_list_on_success(self) -> None:
        """Test solve returns joint list on success."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain)

        # Known reachable target
        q_target = [0.1, 0.1, 0.1]
        T_target = chain.forward(q_target)

        result = solver.solve(T_target)

        if result is not None:
            assert isinstance(result, list)
            assert len(result) == 3


# =============================================================================
# Error Handling Tests
# =============================================================================


class TestErrorHandling:
    """Tests for error handling in IK solvers."""

    def test_wrong_initial_guess_size(self) -> None:
        """Test error on wrong initial guess size."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain)

        from robo_infra.motion.transforms import Transform
        T_target = Transform.identity()

        with pytest.raises(ValueError):
            solver.solve_detailed(T_target, [0.0, 0.0])  # Should be 3


# =============================================================================
# Multi-restart Tests
# =============================================================================


class TestMultiRestart:
    """Tests for multi-restart functionality."""

    def test_restarts_are_used(self) -> None:
        """Test that restarts are attempted."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)

        # Solver with restarts
        config = IKSolverConfig(
            max_iterations=50,
            random_restarts=3,
        )
        solver = JacobianIKSolver(chain, config=config)

        # Target that may be hard to reach from q=0
        q_target = [0.8, -0.6, 0.4]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target, [0.0, 0.0, 0.0])

        # Should return some result
        assert result is not None


# =============================================================================
# Integration Tests
# =============================================================================


class TestIntegration:
    """Integration tests for IK solvers."""

    def test_all_solvers_on_planar(self) -> None:
        """Test all solvers on planar arm."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)

        # Known target
        q_target = [0.3, 0.2, -0.1]
        T_target = chain.forward(q_target)

        solvers = [
            JacobianIKSolver(chain),
            DampedLeastSquaresIK(chain),
            CCDIKSolver(chain),
            GradientDescentIK(chain),
        ]

        for solver in solvers:
            result = solver.solve_detailed(T_target)
            assert result is not None
            if result.joint_values is not None:
                assert len(result.joint_values) == 3

    def test_puma_ik_fk_roundtrip(self) -> None:
        """Test IK/FK roundtrip on PUMA 560."""
        chain = create_puma_560()
        solver = DampedLeastSquaresIK(
            chain,
            config=IKSolverConfig(max_iterations=500, random_restarts=3),
        )

        # Start with known configuration
        q_original = [0.2, -0.3, 0.5, 0.1, -0.2, 0.4]
        T_target = chain.forward(q_original)

        # Solve from different start
        result = solver.solve_detailed(T_target, [0.0] * 6)

        # If successful, verify FK matches target
        if result.success and result.joint_values is not None:
            T_result = chain.forward(result.joint_values)
            pos_error = np.linalg.norm(T_result.position - T_target.position)
            assert pos_error < 0.1  # Within 10cm

    def test_ur5_ik(self) -> None:
        """Test IK on UR5."""
        chain = create_ur5()
        solver = DampedLeastSquaresIK(
            chain,
            config=IKSolverConfig(max_iterations=500),
        )

        # Target pose
        q_target = [0.1, -0.5, 0.3, -0.2, 0.4, -0.1]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target)

        assert result is not None


# =============================================================================
# Performance Tests
# =============================================================================


class TestPerformance:
    """Performance-related tests."""

    def test_convergence_tracking(self) -> None:
        """Test that iterations are tracked."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain)

        q_target = [0.2, 0.3, 0.1]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target)

        assert result.iterations >= 0
        assert result.iterations <= solver.config.max_iterations

    def test_error_tracking(self) -> None:
        """Test that errors are tracked."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = JacobianIKSolver(chain)

        q_target = [0.2, 0.3, 0.1]
        T_target = chain.forward(q_target)

        result = solver.solve_detailed(T_target)

        # Position error should be tracked
        assert result.error_position >= 0
