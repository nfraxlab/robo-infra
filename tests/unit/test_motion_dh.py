"""Tests for DH parameters module.

Tests for DHParameter, DHChain, and robot arm factories:
- DH transform computation
- Forward kinematics
- Jacobian calculation
- Robot arm factories (PUMA 560, UR5, planar 3-DOF)
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robo_infra.motion.dh_parameters import (
    DHChain,
    DHConvention,
    DHParameter,
    JointLimit,
    JointType,
    create_planar_3dof,
    create_puma_560,
    create_stanford_arm,
    create_ur5,
)


# =============================================================================
# DHParameter Tests
# =============================================================================


class TestDHParameter:
    """Tests for DHParameter dataclass."""

    def test_creation(self) -> None:
        """Test basic creation."""
        param = DHParameter(
            d=0.1,
            theta=0.0,
            a=0.5,
            alpha=math.pi / 2,
        )
        assert param.d == pytest.approx(0.1)
        assert param.theta == pytest.approx(0.0)
        assert param.a == pytest.approx(0.5)
        assert param.alpha == pytest.approx(math.pi / 2)

    def test_default_values(self) -> None:
        """Test default parameter values."""
        param = DHParameter(d=0.1, theta=0.0, a=0.5, alpha=0.0)
        assert param.joint_type == JointType.REVOLUTE
        # limit is created by default
        assert param.limit is not None

    def test_joint_limit(self) -> None:
        """Test joint limit specification."""
        limit = JointLimit(min_val=-math.pi, max_val=math.pi)
        param = DHParameter(
            d=0.0,
            theta=0.0,
            a=1.0,
            alpha=0.0,
            limit=limit,
        )
        assert param.limit is not None
        assert param.limit.min_val == pytest.approx(-math.pi)
        assert param.limit.max_val == pytest.approx(math.pi)

    def test_revolute_variable_is_theta(self) -> None:
        """Test revolute joint varies theta."""
        param = DHParameter(
            d=0.0,
            theta=0.0,
            a=1.0,
            alpha=0.0,
            joint_type=JointType.REVOLUTE,
        )
        assert param.joint_type == JointType.REVOLUTE

    def test_prismatic_variable_is_d(self) -> None:
        """Test prismatic joint varies d."""
        param = DHParameter(
            d=0.0,
            theta=0.0,
            a=1.0,
            alpha=0.0,
            joint_type=JointType.PRISMATIC,
        )
        assert param.joint_type == JointType.PRISMATIC

    def test_transform_standard_convention(self) -> None:
        """Test transform computation (standard DH)."""
        # Simple revolute joint: a = 1, all else zero
        param = DHParameter(d=0.0, theta=0.0, a=1.0, alpha=0.0)
        T = param.transform(0.0)  # q = 0

        # With theta=0, should translate 1 unit along x
        matrix = T.as_matrix()
        expected = np.array(
            [
                [1, 0, 0, 1],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        np.testing.assert_array_almost_equal(matrix, expected)

    def test_transform_with_theta(self) -> None:
        """Test transform with non-zero theta."""
        param = DHParameter(d=0.0, theta=0.0, a=1.0, alpha=0.0)
        T = param.transform(math.pi / 2)  # 90° rotation

        # Should rotate 90° and translate 1 unit
        matrix = T.as_matrix()
        assert matrix[0, 3] == pytest.approx(0, abs=1e-10)  # x = 0
        assert matrix[1, 3] == pytest.approx(1, abs=1e-10)  # y = 1

    def test_transform_with_d(self) -> None:
        """Test transform with non-zero d (translation along z)."""
        param = DHParameter(d=0.5, theta=0.0, a=0.0, alpha=0.0)
        T = param.transform(0.0)

        # Should translate 0.5 along z
        matrix = T.as_matrix()
        assert matrix[2, 3] == pytest.approx(0.5)

    def test_transform_with_alpha(self) -> None:
        """Test transform with non-zero alpha (rotation around x)."""
        param = DHParameter(d=0.0, theta=0.0, a=0.0, alpha=math.pi / 2)
        T = param.transform(0.0)

        # Should rotate 90° around x-axis
        matrix = T.as_matrix()
        assert matrix[1, 1] == pytest.approx(0, abs=1e-10)
        assert matrix[1, 2] == pytest.approx(-1, abs=1e-10)

    def test_transform_prismatic_joint(self) -> None:
        """Test transform for prismatic joint."""
        param = DHParameter(
            d=0.0,
            theta=0.0,
            a=0.0,
            alpha=0.0,
            joint_type=JointType.PRISMATIC,
        )
        T = param.transform(0.5)  # q = 0.5 (displacement)

        # Should translate 0.5 along z
        matrix = T.as_matrix()
        assert matrix[2, 3] == pytest.approx(0.5)


# =============================================================================
# DHChain Tests
# =============================================================================


class TestDHChain:
    """Tests for DHChain class."""

    def test_creation(self) -> None:
        """Test basic chain creation."""
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        assert chain.num_joints == 2

    def test_num_joints(self) -> None:
        """Test num_joints property."""
        params = [
            DHParameter(d=0, theta=0, a=0.5, alpha=0),
            DHParameter(d=0, theta=0, a=0.5, alpha=0),
            DHParameter(d=0, theta=0, a=0.5, alpha=0),
        ]
        chain = DHChain(params)
        assert chain.num_joints == 3

    def test_forward_identity(self) -> None:
        """Test FK at zero configuration."""
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        q = [0.0, 0.0]
        T = chain.forward(q)

        # Two links of length 1, both at theta=0
        # End effector at (2, 0, 0)
        assert T.position[0] == pytest.approx(2.0)
        assert T.position[1] == pytest.approx(0.0)

    def test_forward_90_degrees(self) -> None:
        """Test FK with 90 degree joint angles."""
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        q = [math.pi / 2, 0.0]  # First joint at 90°
        T = chain.forward(q)

        # First link rotates 90°, second extends from there
        assert T.position[0] == pytest.approx(0, abs=1e-10)
        assert T.position[1] == pytest.approx(2.0)

    def test_forward_all(self) -> None:
        """Test forward_all returns all intermediate transforms."""
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        q = [0.0, 0.0, 0.0]
        transforms = chain.forward_all(q)

        # Should get 4 transforms (one per joint + tool frame)
        assert len(transforms) == 4

        # Each extends further along x
        assert transforms[0].position[0] == pytest.approx(1.0)
        assert transforms[1].position[0] == pytest.approx(2.0)
        assert transforms[2].position[0] == pytest.approx(3.0)

    def test_forward_wrong_dimension(self) -> None:
        """Test FK with wrong number of joint values."""
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        with pytest.raises(ValueError, match="Expected 2"):
            chain.forward([0.0])

    def test_jacobian_shape(self) -> None:
        """Test Jacobian has correct shape."""
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        q = [0.0, 0.0]
        J = chain.jacobian(q)

        # 6x2 Jacobian (6 DOF task space, 2 joints)
        assert J.shape == (6, 2)

    def test_jacobian_planar(self) -> None:
        """Test Jacobian for planar arm."""
        # Two-link planar arm, each link length 1
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        q = [0.0, 0.0]  # Arm extended along x
        J = chain.jacobian(q)

        # Linear velocity part (Jv, first 3 rows)
        # At q=[0,0], end effector at (2,0,0)
        # Rotating joint 1 moves end effector in +y direction
        # Jv for joint 1: cross([0,0,1], [2,0,0]) = [0, 2, 0]
        assert J[1, 0] == pytest.approx(2.0)  # dy/dq1

        # Rotating joint 2 moves end effector in +y direction (by 1)
        # Joint 2 affects only the last link
        assert J[1, 1] == pytest.approx(1.0)  # dy/dq2

    def test_joint_limits(self) -> None:
        """Test joint_limits property."""
        limit = JointLimit(min_val=-1.0, max_val=1.0)
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0, limit=limit),
            DHParameter(d=0, theta=0, a=1.0, alpha=0, limit=limit),
        ]
        chain = DHChain(params)

        limits = chain.joint_limits

        assert len(limits) == 2
        assert limits[0].min_val == pytest.approx(-1.0)
        assert limits[0].max_val == pytest.approx(1.0)


# =============================================================================
# Robot Arm Factory Tests
# =============================================================================


class TestPlanar3DOF:
    """Tests for planar 3-DOF arm factory."""

    def test_creation(self) -> None:
        """Test planar arm creation."""
        arm = create_planar_3dof(1.0, 1.0, 0.5)
        assert arm.num_joints == 3

    def test_forward_kinematics(self) -> None:
        """Test FK for planar arm."""
        arm = create_planar_3dof(1.0, 1.0, 0.5)

        # All joints at zero
        q = [0.0, 0.0, 0.0]
        T = arm.forward(q)

        # End effector at x = 2.5 (sum of link lengths)
        assert T.position[0] == pytest.approx(2.5)
        assert T.position[1] == pytest.approx(0.0, abs=1e-10)

    def test_custom_link_lengths(self) -> None:
        """Test planar arm with custom link lengths."""
        arm = create_planar_3dof(0.5, 0.3, 0.2)

        q = [0.0, 0.0, 0.0]
        T = arm.forward(q)

        assert T.position[0] == pytest.approx(1.0)  # 0.5 + 0.3 + 0.2


class TestPUMA560:
    """Tests for PUMA 560 factory."""

    def test_creation(self) -> None:
        """Test PUMA 560 creation."""
        puma = create_puma_560()
        assert puma.num_joints == 6

    def test_has_limits(self) -> None:
        """Test PUMA 560 has joint limits."""
        puma = create_puma_560()
        limits = puma.joint_limits

        # All joints should have limits
        for limit in limits:
            assert limit.min_val > -float("inf")
            assert limit.max_val < float("inf")

    def test_forward_at_zero(self) -> None:
        """Test FK at zero configuration."""
        puma = create_puma_560()
        q = [0.0] * 6
        T = puma.forward(q)

        # Should produce valid transform
        matrix = T.as_matrix()
        assert matrix.shape == (4, 4)
        assert matrix[3, 3] == pytest.approx(1.0)

    def test_jacobian_shape(self) -> None:
        """Test PUMA 560 Jacobian is 6x6."""
        puma = create_puma_560()
        q = [0.0] * 6
        J = puma.jacobian(q)

        assert J.shape == (6, 6)


class TestUR5:
    """Tests for UR5 factory."""

    def test_creation(self) -> None:
        """Test UR5 creation."""
        ur5 = create_ur5()
        assert ur5.num_joints == 6

    def test_uses_modified_dh(self) -> None:
        """Test UR5 uses modified DH convention."""
        ur5 = create_ur5()
        # Chain should use modified convention
        assert ur5.convention == DHConvention.MODIFIED

    def test_forward_at_zero(self) -> None:
        """Test FK at zero configuration."""
        ur5 = create_ur5()
        q = [0.0] * 6
        T = ur5.forward(q)

        # Valid transform
        matrix = T.as_matrix()
        assert matrix.shape == (4, 4)

    def test_has_limits(self) -> None:
        """Test UR5 has joint limits."""
        ur5 = create_ur5()
        limits = ur5.joint_limits

        # All revolute joints should have ±2π limits
        for limit in limits:
            assert limit.min_val == pytest.approx(-2 * math.pi, abs=0.01)
            assert limit.max_val == pytest.approx(2 * math.pi, abs=0.01)


class TestStanfordArm:
    """Tests for Stanford arm factory."""

    def test_creation(self) -> None:
        """Test Stanford arm creation."""
        stanford = create_stanford_arm()
        assert stanford.num_joints == 6

    def test_has_prismatic_joint(self) -> None:
        """Test Stanford arm has prismatic joint."""
        stanford = create_stanford_arm()

        # Joint 3 (index 2) should be prismatic
        has_prismatic = any(p.joint_type == JointType.PRISMATIC for p in stanford.parameters)
        assert has_prismatic

    def test_forward_with_prismatic(self) -> None:
        """Test FK with prismatic joint motion."""
        stanford = create_stanford_arm()

        # Extend prismatic joint
        q = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # Joint 3 is prismatic
        T = stanford.forward(q)

        # Should produce valid transform
        matrix = T.as_matrix()
        assert matrix.shape == (4, 4)


# =============================================================================
# Integration Tests
# =============================================================================


class TestDHChainIntegration:
    """Integration tests for DH chains."""

    def test_puma_fk_consistency(self) -> None:
        """Test PUMA 560 FK is consistent across calls."""
        puma = create_puma_560()
        q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

        T1 = puma.forward(q)
        T2 = puma.forward(q)

        np.testing.assert_array_almost_equal(T1.as_matrix(), T2.as_matrix())

    def test_jacobian_numerical_gradient(self) -> None:
        """Test Jacobian matches numerical gradient."""
        params = [
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
            DHParameter(d=0, theta=0, a=1.0, alpha=0),
        ]
        chain = DHChain(params)

        q = [0.5, -0.3]
        J = chain.jacobian(q)

        # Numerical gradient
        eps = 1e-6
        J_num = np.zeros_like(J)

        for i in range(2):
            q_plus = list(q)
            q_plus[i] += eps
            q_minus = list(q)
            q_minus[i] -= eps

            T_plus = chain.forward(q_plus)
            T_minus = chain.forward(q_minus)

            # Position gradient
            J_num[:3, i] = (T_plus.position - T_minus.position) / (2 * eps)

        # Compare position Jacobian
        np.testing.assert_array_almost_equal(J[:3, :], J_num[:3, :], decimal=4)

    def test_all_robot_arms_have_valid_fk(self) -> None:
        """Test all factory robots produce valid FK."""
        robots = [
            create_planar_3dof(1.0, 1.0, 0.5),
            create_puma_560(),
            create_ur5(),
            create_stanford_arm(),
        ]

        for robot in robots:
            q = [0.0] * robot.num_joints
            T = robot.forward(q)

            # Check valid transform
            matrix = T.as_matrix()
            assert matrix.shape == (4, 4)
            assert matrix[3, 3] == pytest.approx(1.0)
            np.testing.assert_array_almost_equal(matrix[3, :3], [0, 0, 0])

            # Check rotation is orthonormal
            R = matrix[:3, :3]
            np.testing.assert_array_almost_equal(R @ R.T, np.eye(3), decimal=6)
            assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-6)


# =============================================================================
# JointLimit Tests
# =============================================================================


class TestJointLimit:
    """Tests for JointLimit class."""

    def test_creation(self) -> None:
        """Test basic creation."""
        limit = JointLimit(min_val=-1.0, max_val=1.0)
        assert limit.min_val == pytest.approx(-1.0)
        assert limit.max_val == pytest.approx(1.0)

    def test_clamp(self) -> None:
        """Test clamping values to limits."""
        limit = JointLimit(min_val=-1.0, max_val=1.0)

        assert limit.clamp(0.5) == pytest.approx(0.5)
        assert limit.clamp(-2.0) == pytest.approx(-1.0)
        assert limit.clamp(2.0) == pytest.approx(1.0)

    def test_is_within(self) -> None:
        """Test checking if value is within limits."""
        limit = JointLimit(min_val=-1.0, max_val=1.0)

        assert limit.is_within(0.5) is True
        assert limit.is_within(-0.5) is True
        assert limit.is_within(1.0) is True
        assert limit.is_within(-1.0) is True
        assert limit.is_within(1.5) is False
        assert limit.is_within(-1.5) is False

    def test_invalid_limits(self) -> None:
        """Test invalid limits raise error."""
        with pytest.raises(ValueError):
            JointLimit(min_val=1.0, max_val=-1.0)
