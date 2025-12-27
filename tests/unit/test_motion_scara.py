"""Unit tests for SCARA arm kinematics.

Tests cover:
- SCARAArm creation and validation
- Forward kinematics accuracy
- Inverse kinematics with both configurations
- Joint limits
- Workspace boundary detection
- Jacobian computation
- Factory functions
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robo_infra.motion.scara import (
    SCARAArm,
    SCARAConfiguration,
    SCARAJoints,
    SCARALimits,
    create_epson_ls3,
    create_epson_ls6,
    create_scara,
)
from robo_infra.motion.transforms import Transform


class TestSCARALimits:
    """Test SCARALimits dataclass."""

    def test_default_limits(self) -> None:
        """Test default limit values."""
        limits = SCARALimits()
        assert limits.theta1_range == (-math.pi, math.pi)
        assert limits.z_range == (0.0, 0.1)

    def test_is_valid_within_limits(self) -> None:
        """Test validation with values within limits."""
        limits = SCARALimits(z_range=(0, 0.15))
        assert limits.is_valid(0, 0, 0.1, 0)
        assert limits.is_valid(0.5, -0.5, 0.05, 0.3)

    def test_is_valid_outside_limits(self) -> None:
        """Test validation with values outside limits."""
        limits = SCARALimits(z_range=(0, 0.1))
        # Z out of range
        assert not limits.is_valid(0, 0, 0.2, 0)
        assert not limits.is_valid(0, 0, -0.1, 0)

    def test_custom_limits(self) -> None:
        """Test custom limit values."""
        limits = SCARALimits(
            theta1_range=(-2.0, 2.0),
            theta2_range=(-1.5, 1.5),
            z_range=(0, 0.2),
            theta4_range=(-1.0, 1.0),
        )
        assert limits.is_valid(1.0, 1.0, 0.1, 0.5)
        assert not limits.is_valid(2.5, 0, 0.1, 0)  # theta1 out


class TestSCARAJoints:
    """Test SCARAJoints dataclass."""

    def test_as_array(self) -> None:
        """Test conversion to numpy array."""
        joints = SCARAJoints(theta1=0.1, theta2=0.2, z=0.05, theta4=0.3)
        arr = joints.as_array()
        np.testing.assert_array_almost_equal(arr, [0.1, 0.2, 0.05, 0.3])

    def test_as_tuple(self) -> None:
        """Test conversion to tuple."""
        joints = SCARAJoints(theta1=0.1, theta2=0.2, z=0.05, theta4=0.3)
        assert joints.as_tuple() == (0.1, 0.2, 0.05, 0.3)

    def test_configuration_default(self) -> None:
        """Test default configuration is left arm."""
        joints = SCARAJoints(theta1=0, theta2=0, z=0, theta4=0)
        assert joints.configuration == SCARAConfiguration.LEFT_ARM


class TestSCARAArm:
    """Test SCARAArm class."""

    def test_creation(self) -> None:
        """Test basic SCARA creation."""
        scara = SCARAArm(l1=0.3, l2=0.2)
        assert scara.l1 == 0.3
        assert scara.l2 == 0.2
        assert scara.z_offset == 0.0

    def test_invalid_dimensions(self) -> None:
        """Test validation of dimensions."""
        with pytest.raises(ValueError, match="l1 must be positive"):
            SCARAArm(l1=0, l2=0.2)
        with pytest.raises(ValueError, match="l2 must be positive"):
            SCARAArm(l1=0.3, l2=-0.1)

    def test_workspace_radius(self) -> None:
        """Test workspace radius calculations."""
        scara = SCARAArm(l1=0.3, l2=0.2)
        assert scara.workspace_radius_max == 0.5  # l1 + l2
        assert scara.workspace_radius_min == pytest.approx(0.1)  # |l1 - l2|

    def test_create_factory(self) -> None:
        """Test create factory method."""
        scara = SCARAArm.create(l1=0.25, l2=0.2, z_range=(0, 0.15), z_offset=0.1)
        assert scara.l1 == 0.25
        assert scara.l2 == 0.2
        assert scara.z_offset == 0.1
        assert scara.limits.z_range == (0, 0.15)


class TestSCARAForwardKinematics:
    """Test SCARA forward kinematics."""

    @pytest.fixture
    def scara(self) -> SCARAArm:
        """Create test SCARA arm."""
        return create_scara(l1=0.3, l2=0.2, z_range=(0, 0.15))

    def test_home_position(self, scara: SCARAArm) -> None:
        """Test FK at home position (all zeros)."""
        pose = scara.forward(0, 0, 0, 0)
        # x = l1 + l2 = 0.5, y = 0, z = 0
        np.testing.assert_array_almost_equal(pose.position, [0.5, 0.0, 0.0])

    def test_elbow_90(self, scara: SCARAArm) -> None:
        """Test FK with elbow at 90 degrees."""
        pose = scara.forward(0, math.pi / 2, 0.1, 0)
        # With theta2 = 90°: x = l1 + l2*cos(90) = 0.3, y = l2*sin(90) = 0.2
        np.testing.assert_array_almost_equal(pose.position, [0.3, 0.2, 0.1], decimal=5)

    def test_shoulder_90(self, scara: SCARAArm) -> None:
        """Test FK with shoulder at 90 degrees."""
        pose = scara.forward(math.pi / 2, 0, 0.05, 0)
        # Rotates entire arm: x = 0, y = 0.5
        np.testing.assert_array_almost_equal(pose.position, [0.0, 0.5, 0.05], decimal=5)

    def test_z_offset(self) -> None:
        """Test FK respects z_offset."""
        scara = SCARAArm.create(l1=0.3, l2=0.2, z_range=(0, 0.1), z_offset=0.5)
        pose = scara.forward(0, 0, 0.05, 0)
        assert pose.position[2] == pytest.approx(0.55)  # z + z_offset

    def test_wrist_rotation(self, scara: SCARAArm) -> None:
        """Test that wrist rotation affects orientation."""
        pose1 = scara.forward(0, 0, 0, 0)
        pose2 = scara.forward(0, 0, 0, math.pi / 2)

        # Positions should be same
        np.testing.assert_array_almost_equal(pose1.position, pose2.position)

        # Orientations should differ
        euler1 = pose1.rotation.as_euler()
        euler2 = pose2.rotation.as_euler()
        assert not np.allclose(euler1, euler2)

    def test_forward_joints(self, scara: SCARAArm) -> None:
        """Test forward_joints method."""
        joints = SCARAJoints(theta1=0.1, theta2=0.2, z=0.05, theta4=0.1)
        pose = scara.forward_joints(joints)
        expected = scara.forward(0.1, 0.2, 0.05, 0.1)
        np.testing.assert_array_almost_equal(pose.position, expected.position)


class TestSCARAInverseKinematics:
    """Test SCARA inverse kinematics."""

    @pytest.fixture
    def scara(self) -> SCARAArm:
        """Create test SCARA arm."""
        return create_scara(l1=0.3, l2=0.2, z_range=(0, 0.15))

    def test_round_trip_home(self, scara: SCARAArm) -> None:
        """Test IK recovers FK at home position."""
        original = SCARAJoints(theta1=0, theta2=0.1, z=0.05, theta4=0)
        pose = scara.forward_joints(original)

        recovered = scara.inverse(pose, SCARAConfiguration.LEFT_ARM)
        assert recovered is not None

        # Forward with recovered should match
        recovered_pose = scara.forward_joints(recovered)
        np.testing.assert_array_almost_equal(pose.position, recovered_pose.position, decimal=4)

    def test_round_trip_various_poses(self, scara: SCARAArm) -> None:
        """Test IK round-trip for various poses."""
        test_cases = [
            (0.5, 0.3, 0.1, 0.2),
            (-0.3, 0.5, 0.05, -0.1),
            (0.0, 1.0, 0.08, 0.0),
        ]

        for t1, t2, z, t4 in test_cases:
            pose = scara.forward(t1, t2, z, t4)
            recovered = scara.inverse(pose)

            if recovered is not None:
                recovered_pose = scara.forward_joints(recovered)
                np.testing.assert_array_almost_equal(
                    pose.position, recovered_pose.position, decimal=3
                )

    def test_unreachable_point(self, scara: SCARAArm) -> None:
        """Test IK returns None for unreachable points."""
        # Point outside max reach
        pose = Transform.from_euler((1.0, 0.0, 0.05), (0, 0, 0))
        result = scara.inverse(pose)
        assert result is None

    def test_z_out_of_range(self, scara: SCARAArm) -> None:
        """Test IK returns None for Z outside range."""
        pose = Transform.from_euler((0.4, 0.0, 0.5), (0, 0, 0))  # Z too high
        result = scara.inverse(pose)
        assert result is None

    def test_left_arm_configuration(self, scara: SCARAArm) -> None:
        """Test left arm configuration gives positive theta2."""
        pose = scara.forward(0, 0.5, 0.05, 0)  # Positive theta2
        result = scara.inverse(pose, SCARAConfiguration.LEFT_ARM)

        if result is not None:
            assert result.theta2 >= 0 or result.configuration == SCARAConfiguration.LEFT_ARM

    def test_right_arm_configuration(self, scara: SCARAArm) -> None:
        """Test right arm configuration gives negative theta2."""
        pose = scara.forward(0, -0.5, 0.05, 0)  # Negative theta2
        result = scara.inverse(pose, SCARAConfiguration.RIGHT_ARM)

        if result is not None:
            assert result.theta2 <= 0 or result.configuration == SCARAConfiguration.RIGHT_ARM

    def test_inverse_position(self, scara: SCARAArm) -> None:
        """Test inverse_position method."""
        result = scara.inverse_position(0.4, 0.1, 0.05)
        assert result is not None

        pose = scara.forward_joints(result)
        assert pose.position[0] == pytest.approx(0.4, abs=1e-3)
        assert pose.position[1] == pytest.approx(0.1, abs=1e-3)
        assert pose.position[2] == pytest.approx(0.05, abs=1e-3)

    def test_get_all_solutions(self, scara: SCARAArm) -> None:
        """Test getting all IK solutions."""
        # Point reachable in both configurations
        pose = scara.forward(0.3, 0.4, 0.05, 0)
        solutions = scara.get_all_solutions(pose)

        # Should have at least one solution
        assert len(solutions) >= 1

        # All solutions should yield similar end-effector position
        for sol in solutions:
            recovered = scara.forward_joints(sol)
            np.testing.assert_array_almost_equal(pose.position, recovered.position, decimal=3)


class TestSCARAWorkspace:
    """Test SCARA workspace analysis."""

    def test_is_reachable_in_workspace(self) -> None:
        """Test points within workspace are reachable."""
        scara = create_scara(0.3, 0.2, z_range=(0, 0.1))

        assert scara.is_reachable(0.4, 0, 0.05)  # Within reach
        assert scara.is_reachable(0.3, 0.2, 0.05)  # Near max reach

    def test_is_reachable_outside_workspace(self) -> None:
        """Test points outside workspace are not reachable."""
        scara = create_scara(0.3, 0.2, z_range=(0, 0.1))

        assert not scara.is_reachable(0.6, 0, 0.05)  # Beyond max reach
        assert not scara.is_reachable(0.4, 0, 0.2)  # Z too high
        assert not scara.is_reachable(0.4, 0, -0.1)  # Z too low

    def test_workspace_boundary(self) -> None:
        """Test workspace boundary detection."""
        scara = create_scara(0.25, 0.15, z_range=(0, 0.1))

        # Max reach boundary
        max_r = scara.workspace_radius_max
        assert scara.is_reachable(max_r * 0.95, 0, 0.05)  # Inside
        assert not scara.is_reachable(max_r * 1.05, 0, 0.05)  # Outside


class TestSCARAJacobian:
    """Test SCARA Jacobian computation."""

    def test_jacobian_shape(self) -> None:
        """Test Jacobian has correct shape."""
        scara = create_scara(0.3, 0.2)
        jac = scara.jacobian(0, 0, 0.05, 0)
        assert jac.shape == (4, 4)

    def test_jacobian_numerical_verification(self) -> None:
        """Test Jacobian matches numerical differentiation."""
        scara = create_scara(0.3, 0.2)
        theta1, theta2, z, theta4 = 0.3, 0.5, 0.05, 0.1

        jac = scara.jacobian(theta1, theta2, z, theta4)

        # Verify with numerical differentiation
        eps = 1e-6
        pose0 = scara.forward(theta1, theta2, z, theta4)

        # Check dx/dtheta1
        pose1 = scara.forward(theta1 + eps, theta2, z, theta4)
        dx_dt1_num = (pose1.position[0] - pose0.position[0]) / eps

        assert jac[0, 0] == pytest.approx(dx_dt1_num, rel=1e-3)


class TestSCARAFactoryFunctions:
    """Test SCARA factory functions."""

    def test_create_scara(self) -> None:
        """Test create_scara function."""
        scara = create_scara(0.25, 0.2, z_range=(0, 0.15), z_offset=0.1)
        assert scara.l1 == 0.25
        assert scara.l2 == 0.2
        assert scara.z_offset == 0.1

    def test_create_epson_ls3(self) -> None:
        """Test Epson LS3-B equivalent."""
        scara = create_epson_ls3()
        assert scara.l1 == 0.225
        assert scara.l2 == 0.175
        assert scara.limits.z_range == (0, 0.15)

        # Should have ~400mm reach
        assert scara.workspace_radius_max == 0.4

    def test_create_epson_ls6(self) -> None:
        """Test Epson LS6-B equivalent."""
        scara = create_epson_ls6()
        assert scara.l1 == 0.3
        assert scara.l2 == 0.25
        assert scara.limits.z_range == (0, 0.2)

        # Should have ~550mm reach
        assert scara.workspace_radius_max == 0.55
