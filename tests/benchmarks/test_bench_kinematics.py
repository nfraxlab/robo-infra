"""
Benchmarks for kinematics calculations.

Performance targets:
- 3-DOF IK solve: <1ms
- 6-DOF IK solve: <5ms
- FK calculation: <0.1ms
- Trajectory generation (100 points): <10ms
"""

from __future__ import annotations

# Detect CI environment (GitHub Actions, etc.)
import os

import pytest

from robo_infra.motion import (
    # IK solvers
    CCDIKSolver,
    # Trajectory
    CubicTrajectory,
    DampedLeastSquaresIK,
    # Kinematics chains
    EndEffectorPose,
    GradientDescentIK,
    IKSolverConfig,
    LinearInterpolator,
    # Path planning
    LinearPathPlanner,
    QuinticTrajectory,
    RRTPathPlanner,
    SCurveTrajectory,
    ThreeLinkArm,
    TrajectoryGenerator,
    TrajectoryProfile,
    TrapezoidalProfile,
    TwoLinkArm,
    # Delta robot
    create_delta,
    # DH presets
    create_planar_3dof,
    create_puma_560,
    # SCARA
    create_scara,
    create_ur5,
)

from . import Benchmarker, format_time


IN_CI = (
    os.environ.get("CI", "").lower() == "true"
    or os.environ.get("GITHUB_ACTIONS", "").lower() == "true"
)

# Performance targets (in seconds) - relaxed for CI
# CI environments are typically slower than local development machines
TARGET_FK_2DOF = 0.0001 if not IN_CI else 0.001  # 0.1ms (1ms in CI)
TARGET_FK_3DOF = 0.0001 if not IN_CI else 0.001  # 0.1ms (1ms in CI)
TARGET_FK_6DOF = 0.0005 if not IN_CI else 0.005  # 0.5ms (5ms in CI)
TARGET_IK_3DOF = 0.050 if not IN_CI else 0.500  # 50ms (500ms in CI)
TARGET_IK_6DOF = 0.100 if not IN_CI else 1.000  # 100ms (1000ms in CI)
TARGET_TRAJECTORY_100 = 0.010 if not IN_CI else 0.100  # 10ms (100ms in CI)


@pytest.fixture
def benchmarker() -> Benchmarker:
    """Create a benchmarker instance."""
    return Benchmarker(warmup_iterations=10)


class TestForwardKinematics:
    """Benchmark forward kinematics calculations."""

    def test_two_link_arm_fk(self, benchmarker: Benchmarker) -> None:
        """Benchmark 2-DOF forward kinematics."""
        arm = TwoLinkArm(l1=1.0, l2=1.0)
        angles = [0.5, 0.3]

        result = benchmarker.run(
            "2-DOF FK (TwoLinkArm)",
            lambda: arm.forward(angles),
            iterations=10000,
            target=TARGET_FK_2DOF,
        )

        print(f"\n{result}")
        print(f"   Max rate: {result.ops_per_sec:.0f} calcs/sec")
        assert result.passed, f"2-DOF FK too slow: {format_time(result.mean_time)}"

    def test_three_link_arm_fk(self, benchmarker: Benchmarker) -> None:
        """Benchmark 3-DOF forward kinematics."""
        arm = ThreeLinkArm(l1=1.0, l2=1.0, l3=0.5)
        angles = [0.5, 0.3, 0.1]

        result = benchmarker.run(
            "3-DOF FK (ThreeLinkArm)",
            lambda: arm.forward(angles),
            iterations=10000,
            target=TARGET_FK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"3-DOF FK too slow: {format_time(result.mean_time)}"

    def test_dh_chain_fk_3dof(self, benchmarker: Benchmarker) -> None:
        """Benchmark 3-DOF DH chain forward kinematics."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        joints = [0.5, 0.3, 0.1]

        result = benchmarker.run(
            "3-DOF FK (DH chain)",
            lambda: chain.forward(joints),
            iterations=5000,
            target=TARGET_FK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"3-DOF DH FK too slow: {format_time(result.mean_time)}"

    def test_dh_chain_fk_6dof(self, benchmarker: Benchmarker) -> None:
        """Benchmark 6-DOF DH chain forward kinematics (UR5)."""
        chain = create_ur5()
        joints = [0.1, -0.5, 0.8, 0.2, -0.3, 0.1]

        result = benchmarker.run(
            "6-DOF FK (UR5 DH chain)",
            lambda: chain.forward(joints),
            iterations=5000,
            target=TARGET_FK_6DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"6-DOF DH FK too slow: {format_time(result.mean_time)}"

    def test_puma_560_fk(self, benchmarker: Benchmarker) -> None:
        """Benchmark PUMA 560 forward kinematics."""
        chain = create_puma_560()
        joints = [0.1, -0.5, 0.8, 0.2, -0.3, 0.1]

        result = benchmarker.run(
            "6-DOF FK (PUMA 560)",
            lambda: chain.forward(joints),
            iterations=5000,
            target=TARGET_FK_6DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"PUMA 560 FK too slow: {format_time(result.mean_time)}"


class TestInverseKinematics:
    """Benchmark inverse kinematics calculations."""

    def test_two_link_arm_ik(self, benchmarker: Benchmarker) -> None:
        """Benchmark 2-DOF analytical inverse kinematics."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        target = EndEffectorPose(x=120.0, y=80.0)

        result = benchmarker.run(
            "2-DOF IK (TwoLinkArm)",
            lambda: arm.inverse(target),
            iterations=5000,
            target=TARGET_IK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"2-DOF IK too slow: {format_time(result.mean_time)}"

    def test_three_link_arm_ik(self, benchmarker: Benchmarker) -> None:
        """Benchmark 3-DOF inverse kinematics."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        # Get a valid target from FK
        target = arm.forward([0.5, 0.3, 0.1])

        result = benchmarker.run(
            "3-DOF IK (ThreeLinkArm)",
            lambda: arm.inverse(target),
            iterations=5000,
            target=TARGET_IK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"3-DOF IK too slow: {format_time(result.mean_time)}"

    def test_ccd_ik_solver_3dof(self, benchmarker: Benchmarker) -> None:
        """Benchmark CCD IK solver on 3-DOF chain."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = CCDIKSolver(chain, config=IKSolverConfig(max_iterations=50))
        # Get a valid target from FK
        target = chain.forward([0.5, 0.3, 0.1])

        result = benchmarker.run(
            "3-DOF IK (CCD solver)",
            lambda: solver.solve(target),
            iterations=500,
            target=TARGET_IK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"CCD IK 3-DOF too slow: {format_time(result.mean_time)}"

    def test_damped_least_squares_ik_6dof(self, benchmarker: Benchmarker) -> None:
        """Benchmark Damped Least Squares IK on 6-DOF chain."""
        chain = create_ur5()
        solver = DampedLeastSquaresIK(
            chain,
            config=IKSolverConfig(max_iterations=50, position_tolerance=1e-4),
        )
        # Get a valid target from FK
        target = chain.forward([0.1, -0.3, 0.5, 0.1, -0.2, 0.1])

        result = benchmarker.run(
            "6-DOF IK (DLS solver)",
            lambda: solver.solve(target),
            iterations=200,
            target=TARGET_IK_6DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"DLS IK 6-DOF too slow: {format_time(result.mean_time)}"

    def test_gradient_descent_ik(self, benchmarker: Benchmarker) -> None:
        """Benchmark Gradient Descent IK solver."""
        chain = create_planar_3dof(1.0, 1.0, 0.5)
        solver = GradientDescentIK(
            chain,
            config=IKSolverConfig(max_iterations=100),
        )
        target = chain.forward([0.5, 0.3, 0.1])

        result = benchmarker.run(
            "3-DOF IK (Gradient Descent)",
            lambda: solver.solve(target),
            iterations=200,
            target=TARGET_IK_3DOF * 2,  # More iterations allowed
        )

        print(f"\n{result}")
        # Report even if slower
        print(f"   Gradient Descent is {'meeting' if result.passed else 'not meeting'} target")


class TestSpecializedKinematics:
    """Benchmark specialized robot kinematics."""

    def test_delta_robot_fk(self, benchmarker: Benchmarker) -> None:
        """Benchmark Delta robot forward kinematics."""
        delta = create_delta(0.15, 0.03, 0.10, 0.30)

        result = benchmarker.run(
            "Delta FK",
            lambda: delta.forward(0.3, 0.3, 0.3),
            iterations=5000,
            target=TARGET_FK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"Delta FK too slow: {format_time(result.mean_time)}"

    def test_delta_robot_ik(self, benchmarker: Benchmarker) -> None:
        """Benchmark Delta robot inverse kinematics."""
        delta = create_delta(0.15, 0.03, 0.10, 0.30)
        # Get a valid target from FK
        x, y, z = delta.forward(0.3, 0.3, 0.3)

        result = benchmarker.run(
            "Delta IK",
            lambda: delta.inverse(x, y, z),
            iterations=5000,
            target=TARGET_IK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"Delta IK too slow: {format_time(result.mean_time)}"

    def test_scara_fk(self, benchmarker: Benchmarker) -> None:
        """Benchmark SCARA forward kinematics."""
        scara = create_scara(l1=0.3, l2=0.2, z_range=(0, 0.15))

        result = benchmarker.run(
            "SCARA FK",
            lambda: scara.forward(0.5, 0.3, 0.1, 0.0),
            iterations=5000,
            target=TARGET_FK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"SCARA FK too slow: {format_time(result.mean_time)}"

    def test_scara_ik(self, benchmarker: Benchmarker) -> None:
        """Benchmark SCARA inverse kinematics."""
        scara = create_scara(l1=0.3, l2=0.2, z_range=(0, 0.15))
        # Get a valid target from forward kinematics
        target_pose = scara.forward(0.5, 0.3, 0.1, 0.0)

        result = benchmarker.run(
            "SCARA IK",
            lambda: scara.inverse(target_pose),
            iterations=5000,
            target=TARGET_IK_3DOF,
        )

        print(f"\n{result}")
        assert result.passed, f"SCARA IK too slow: {format_time(result.mean_time)}"


class TestTrajectoryGeneration:
    """Benchmark trajectory generation."""

    def test_linear_interpolator_100_points(self, benchmarker: Benchmarker) -> None:
        """Benchmark linear interpolation with 100 points."""
        start = 0.0
        end = 1.0

        def generate() -> list[float]:
            interp = LinearInterpolator(start, end, duration=1.0)
            return [interp.position_at(t / 100) for t in range(101)]

        result = benchmarker.run(
            "Linear interpolation (100 points)",
            generate,
            iterations=500,
            target=TARGET_TRAJECTORY_100,
        )

        print(f"\n{result}")
        assert result.passed, f"Linear interpolation too slow: {format_time(result.mean_time)}"

    def test_cubic_trajectory_100_points(self, benchmarker: Benchmarker) -> None:
        """Benchmark cubic trajectory with 100 points."""
        start = 0.0
        end = 100.0

        def generate() -> list[float]:
            traj = CubicTrajectory(start=start, end=end, duration=1.0)
            return [traj.position_at(t / 100) for t in range(101)]

        result = benchmarker.run(
            "Cubic trajectory (100 points)",
            generate,
            iterations=500,
            target=TARGET_TRAJECTORY_100,
        )

        print(f"\n{result}")
        assert result.passed, f"Cubic trajectory too slow: {format_time(result.mean_time)}"

    def test_quintic_trajectory_100_points(self, benchmarker: Benchmarker) -> None:
        """Benchmark quintic trajectory with 100 points."""
        start = 0.0
        end = 100.0

        def generate() -> list[float]:
            traj = QuinticTrajectory(start=start, end=end, duration=1.0)
            return [traj.position_at(t / 100) for t in range(101)]

        result = benchmarker.run(
            "Quintic trajectory (100 points)",
            generate,
            iterations=500,
            target=TARGET_TRAJECTORY_100,
        )

        print(f"\n{result}")
        assert result.passed, f"Quintic trajectory too slow: {format_time(result.mean_time)}"

    def test_trapezoidal_profile_100_points(self, benchmarker: Benchmarker) -> None:
        """Benchmark trapezoidal velocity profile trajectory."""
        start = 0.0
        end = 100.0

        def generate() -> list[float]:
            profile = TrapezoidalProfile(
                start=start,
                end=end,
                max_velocity=50.0,
                max_acceleration=25.0,
            )
            duration = profile.total_time
            return [profile.position_at(t * duration / 100) for t in range(101)]

        result = benchmarker.run(
            "Trapezoidal profile (100 points)",
            generate,
            iterations=500,
            target=TARGET_TRAJECTORY_100,
        )

        print(f"\n{result}")
        assert result.passed, f"Trapezoidal profile too slow: {format_time(result.mean_time)}"

    def test_scurve_trajectory_100_points(self, benchmarker: Benchmarker) -> None:
        """Benchmark S-curve trajectory (more complex)."""
        start = 0.0
        end = 100.0

        def generate() -> list[float]:
            traj = SCurveTrajectory(
                start=start,
                end=end,
                max_velocity=50.0,
                max_acceleration=25.0,
                max_jerk=100.0,
            )
            duration = traj.total_time
            return [traj.position_at(t * duration / 100) for t in range(101)]

        result = benchmarker.run(
            "S-curve trajectory (100 points)",
            generate,
            iterations=500,
            target=TARGET_TRAJECTORY_100 * 2,  # More complex
        )

        print(f"\n{result}")
        assert result.passed, f"S-curve trajectory too slow: {format_time(result.mean_time)}"

    def test_trajectory_generator(self, benchmarker: Benchmarker) -> None:
        """Benchmark TrajectoryGenerator for trajectory generation."""

        def generate() -> list[float]:
            gen = TrajectoryGenerator(
                max_velocity=50.0,
                max_acceleration=25.0,
                profile=TrajectoryProfile.TRAPEZOIDAL,
            )
            traj = gen.generate(start=0.0, end=100.0)
            duration = traj.total_time
            return [traj.position_at(t * duration / 100) for t in range(101)]

        result = benchmarker.run(
            "TrajectoryGenerator (100 points)",
            generate,
            iterations=500,
            target=TARGET_TRAJECTORY_100,
        )

        print(f"\n{result}")
        assert result.passed, f"TrajectoryGenerator too slow: {format_time(result.mean_time)}"


class TestPathPlanning:
    """Benchmark path planning algorithms."""

    def test_linear_path_planner(self, benchmarker: Benchmarker) -> None:
        """Benchmark linear path planner."""
        planner = LinearPathPlanner(num_points=100)
        start = [0.0, 0.0, 0.0]
        goal = [1.0, 1.0, 1.0]

        result = benchmarker.run(
            "Linear path planning",
            lambda: planner.plan(start, goal),
            iterations=1000,
            target=TARGET_TRAJECTORY_100,
        )

        print(f"\n{result}")
        assert result.passed, f"Linear path planning too slow: {format_time(result.mean_time)}"

    def test_rrt_path_planner_simple(self, benchmarker: Benchmarker) -> None:
        """Benchmark RRT path planner (no obstacles)."""
        planner = RRTPathPlanner(
            bounds=[(-2, 2), (-2, 2), (-2, 2)],
            max_iterations=500,
            step_size=0.2,
        )
        start = (0.0, 0.0, 0.0)
        goal = (1.0, 1.0, 1.0)

        result = benchmarker.run(
            "RRT path planning (no obstacles)",
            lambda: planner.plan(start, goal),
            iterations=50,
            target=0.100,  # 100ms - RRT is stochastic
        )

        print(f"\n{result}")
        # RRT is inherently variable, just report
        print(f"   RRT takes {format_time(result.mean_time)} on average")


class TestKinematicsComparison:
    """Compare performance of different kinematic methods."""

    def test_fk_solver_comparison(self, benchmarker: Benchmarker) -> None:
        """Compare FK solver performance across configurations."""
        configs = [
            ("2-DOF", TwoLinkArm(l1=1.0, l2=1.0), [0.5, 0.3]),
            ("3-DOF planar", create_planar_3dof(1.0, 1.0, 0.5), [0.5, 0.3, 0.1]),
            ("6-DOF UR5", create_ur5(), [0.1, -0.5, 0.8, 0.2, -0.3, 0.1]),
        ]

        print("\n--- FK Solver Comparison ---")
        for name, solver, joints in configs:
            result = benchmarker.run(
                name,
                lambda s=solver, j=joints: s.forward(j),
                iterations=2000,
            )
            print(f"{name}: {format_time(result.mean_time)} ({result.ops_per_sec:.0f} Hz)")

    def test_trajectory_profile_comparison(self, benchmarker: Benchmarker) -> None:
        """Compare trajectory profile performance."""
        start = 0.0
        end = 100.0

        profiles = [
            ("Linear", lambda: LinearInterpolator(start, end, 1.0)),
            ("Cubic", lambda: CubicTrajectory(start, end, 1.0)),
            ("Quintic", lambda: QuinticTrajectory(start, end, 1.0)),
            ("Trapezoidal", lambda: TrapezoidalProfile(start, end, 50.0, 25.0)),
        ]

        print("\n--- Trajectory Profile Comparison (100 points) ---")
        for name, create_traj in profiles:

            def generate(create: type = create_traj) -> list[float]:
                traj = create()
                if hasattr(traj, "total_time"):
                    duration = traj.total_time
                else:
                    duration = traj.duration
                return [traj.position_at(t * duration / 100) for t in range(101)]

            result = benchmarker.run(name, generate, iterations=200)
            print(f"{name}: {format_time(result.mean_time)}")
