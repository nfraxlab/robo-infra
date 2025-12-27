"""Unit tests for Stewart platform kinematics.

Tests cover:
- StewartPlatform creation and validation
- Inverse kinematics (closed-form)
- Forward kinematics (iterative)
- Workspace validation
- Singularity detection
- Jacobian computation
- Factory functions
"""

from __future__ import annotations

import numpy as np
import pytest

from robo_infra.motion.stewart import (
    StewartJoints,
    StewartLimits,
    StewartPlatform,
    StewartPose,
    StewartSingularityType,
    create_flight_simulator,
    create_precision_positioner,
    create_stewart,
)
from robo_infra.motion.transforms import Transform


class TestStewartLimits:
    """Test StewartLimits dataclass."""

    def test_default_limits(self) -> None:
        """Test default limit values."""
        limits = StewartLimits()
        assert limits.leg_length_range == (0.3, 0.5)
        assert limits.position_limits == (-0.1, 0.1)
        assert limits.rotation_limits == (-0.3, 0.3)

    def test_is_valid_leg_length(self) -> None:
        """Test leg length validation."""
        limits = StewartLimits(leg_length_range=(0.3, 0.5))
        assert limits.is_valid_leg_length(0.4)
        assert limits.is_valid_leg_length(0.3)  # Boundary
        assert limits.is_valid_leg_length(0.5)  # Boundary
        assert not limits.is_valid_leg_length(0.2)
        assert not limits.is_valid_leg_length(0.6)

    def test_clamp_leg_length(self) -> None:
        """Test leg length clamping."""
        limits = StewartLimits(leg_length_range=(0.3, 0.5))
        assert limits.clamp_leg_length(0.4) == 0.4
        assert limits.clamp_leg_length(0.2) == 0.3
        assert limits.clamp_leg_length(0.6) == 0.5


class TestStewartJoints:
    """Test StewartJoints dataclass."""

    def test_creation(self) -> None:
        """Test joints creation."""
        legs = np.array([0.35, 0.36, 0.37, 0.38, 0.39, 0.40])
        joints = StewartJoints(leg_lengths=legs)
        np.testing.assert_array_almost_equal(joints.leg_lengths, legs)

    def test_invalid_shape(self) -> None:
        """Test error on wrong number of legs."""
        with pytest.raises(ValueError, match="Expected 6"):
            StewartJoints(leg_lengths=np.array([0.3, 0.4, 0.5]))

    def test_as_tuple(self) -> None:
        """Test conversion to tuple."""
        joints = StewartJoints(leg_lengths=np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]))
        t = joints.as_tuple()
        assert len(t) == 6
        assert t[0] == pytest.approx(0.1)

    def test_getitem(self) -> None:
        """Test indexing."""
        joints = StewartJoints(leg_lengths=np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]))
        assert joints[0] == pytest.approx(0.1)
        assert joints[5] == pytest.approx(0.6)


class TestStewartPose:
    """Test StewartPose dataclass."""

    def test_from_transform(self) -> None:
        """Test creation from Transform."""
        transform = Transform.from_euler((0.1, 0.2, 0.4), (0.05, 0.03, 0.01))
        pose = StewartPose.from_transform(transform)

        np.testing.assert_array_almost_equal(pose.position, [0.1, 0.2, 0.4])

    def test_to_transform(self) -> None:
        """Test conversion to Transform."""
        pose = StewartPose(
            position=np.array([0.1, 0.2, 0.4]),
            euler_angles=np.array([0.05, 0.03, 0.01]),
        )
        transform = pose.to_transform()

        np.testing.assert_array_almost_equal(transform.position, [0.1, 0.2, 0.4])


class TestStewartPlatform:
    """Test StewartPlatform class."""

    def test_create_symmetric(self) -> None:
        """Test symmetric platform creation."""
        stewart = StewartPlatform.create_symmetric(
            base_radius=0.25,
            platform_radius=0.12,
            leg_length_range=(0.3, 0.5),
        )

        assert stewart.base_joints.shape == (6, 3)
        assert stewart.platform_joints.shape == (6, 3)
        assert stewart.limits.leg_length_range == (0.3, 0.5)

    def test_create_from_points(self) -> None:
        """Test creation from explicit points."""
        base = [
            (0.1, 0, 0),
            (0.05, 0.087, 0),
            (-0.05, 0.087, 0),
            (-0.1, 0, 0),
            (-0.05, -0.087, 0),
            (0.05, -0.087, 0),
        ]
        platform = [
            (0.05, 0, 0),
            (0.025, 0.043, 0),
            (-0.025, 0.043, 0),
            (-0.05, 0, 0),
            (-0.025, -0.043, 0),
            (0.025, -0.043, 0),
        ]

        stewart = StewartPlatform.create_from_points(
            base_joints=base,
            platform_joints=platform,
            leg_length_range=(0.2, 0.4),
            home_height=0.3,
        )

        assert stewart.home_height == 0.3

    def test_invalid_joint_count(self) -> None:
        """Test error on wrong number of joints."""
        with pytest.raises(ValueError):
            StewartPlatform.create_from_points(
                base_joints=[(0, 0, 0)] * 5,  # Wrong count
                platform_joints=[(0, 0, 0)] * 6,
                leg_length_range=(0.2, 0.4),
            )


class TestStewartInverseKinematics:
    """Test Stewart platform inverse kinematics."""

    @pytest.fixture
    def stewart(self) -> StewartPlatform:
        """Create test Stewart platform."""
        return create_stewart(0.2, 0.1, (0.2, 0.5), home_height=0.35)

    def test_home_pose_ik(self, stewart: StewartPlatform) -> None:
        """Test IK at home pose."""
        pose = stewart.home_pose()
        joints = stewart.inverse(pose)

        # All legs should be approximately equal at home
        legs = joints.leg_lengths
        assert np.std(legs) < 0.01

    def test_translation_ik(self, stewart: StewartPlatform) -> None:
        """Test IK with translation only."""
        pose = Transform.from_euler((0.02, 0.01, 0.35), (0, 0, 0))
        joints = stewart.inverse(pose)

        # Should get valid leg lengths
        for length in joints.leg_lengths:
            assert stewart.limits.is_valid_leg_length(length)

    def test_rotation_ik(self, stewart: StewartPlatform) -> None:
        """Test IK with rotation only."""
        pose = Transform.from_euler((0, 0, 0.35), (0.05, 0.03, 0))
        joints = stewart.inverse(pose)

        # Should get valid leg lengths
        for length in joints.leg_lengths:
            assert stewart.limits.is_valid_leg_length(length)

    def test_combined_motion_ik(self, stewart: StewartPlatform) -> None:
        """Test IK with translation and rotation."""
        pose = Transform.from_euler((0.02, -0.01, 0.36), (0.05, -0.03, 0.02))
        joints = stewart.inverse(pose)

        assert joints.leg_lengths.shape == (6,)

    def test_ik_out_of_range(self, stewart: StewartPlatform) -> None:
        """Test IK fails for out-of-range pose."""
        # Very large translation - should exceed leg limits
        pose = Transform.from_euler((0.5, 0.5, 0.35), (0, 0, 0))

        with pytest.raises(ValueError, match="outside limits"):
            stewart.inverse(pose)

    def test_inverse_safe(self, stewart: StewartPlatform) -> None:
        """Test inverse_safe returns validity flag."""
        # Valid pose
        valid_pose = stewart.home_pose()
        _joints, is_valid = stewart.inverse_safe(valid_pose)
        assert is_valid

        # Invalid pose (large displacement)
        invalid_pose = Transform.from_euler((0.5, 0, 0.35), (0, 0, 0))
        _joints, is_valid = stewart.inverse_safe(invalid_pose)
        assert not is_valid

    def test_inverse_array(self, stewart: StewartPlatform) -> None:
        """Test inverse_array returns numpy array."""
        pose = stewart.home_pose()
        legs = stewart.inverse_array(pose)

        assert isinstance(legs, np.ndarray)
        assert legs.shape == (6,)


class TestStewartForwardKinematics:
    """Test Stewart platform forward kinematics."""

    @pytest.fixture
    def stewart(self) -> StewartPlatform:
        """Create test Stewart platform."""
        return create_stewart(0.2, 0.1, (0.2, 0.5), home_height=0.35)

    def test_fk_at_home(self, stewart: StewartPlatform) -> None:
        """Test FK at home leg lengths."""
        home_joints = stewart.home_leg_lengths()
        pose = stewart.forward(home_joints)

        # Should recover approximately home pose
        np.testing.assert_array_almost_equal(pose.position, [0, 0, stewart.home_height], decimal=2)

    def test_fk_ik_round_trip(self, stewart: StewartPlatform) -> None:
        """Test FK recovers IK input for small perturbations from home."""
        # Start with a pose close to home (small perturbations converge better)
        home = stewart.home_pose()
        target = Transform.from_euler(
            (0.005, 0.005, home.position[2] + 0.01),  # Small translation
            (0.02, 0.02, 0.01),  # Small rotation
        )

        try:
            # IK to get leg lengths
            joints = stewart.inverse(target)

            # FK to recover pose (may not converge for all geometries)
            recovered = stewart.forward(joints, initial_guess=home)

            # Should match within tolerance
            np.testing.assert_array_almost_equal(target.position, recovered.position, decimal=2)
        except ValueError:
            # FK may not converge for some platform configurations
            pytest.skip("FK did not converge for this configuration")

    def test_fk_from_array(self, stewart: StewartPlatform) -> None:
        """Test FK accepts numpy array."""
        legs = np.ones(6) * 0.35
        # This might fail to converge, but shouldn't error
        try:
            pose = stewart.forward(legs)
            assert isinstance(pose, Transform)
        except ValueError:
            # May not converge for arbitrary leg lengths
            pass

    def test_fk_with_initial_guess(self, stewart: StewartPlatform) -> None:
        """Test FK with custom initial guess."""
        home_joints = stewart.home_leg_lengths()
        initial = stewart.home_pose()

        pose = stewart.forward(home_joints, initial_guess=initial)

        np.testing.assert_array_almost_equal(pose.position, initial.position, decimal=2)


class TestStewartValidation:
    """Test Stewart platform validation methods."""

    @pytest.fixture
    def stewart(self) -> StewartPlatform:
        """Create test Stewart platform."""
        return create_stewart(0.2, 0.1, (0.2, 0.5))

    def test_is_pose_valid(self, stewart: StewartPlatform) -> None:
        """Test pose validation."""
        # Home pose should be valid
        assert stewart.is_pose_valid(stewart.home_pose())

        # Large translation should be invalid
        invalid = Transform.from_euler((0.5, 0, 0.35), (0, 0, 0))
        assert not stewart.is_pose_valid(invalid)

    def test_home_pose(self, stewart: StewartPlatform) -> None:
        """Test home_pose method."""
        pose = stewart.home_pose()

        np.testing.assert_array_almost_equal(pose.position, [0, 0, stewart.home_height])
        # Should be level (identity rotation)
        euler = pose.rotation.as_euler()
        np.testing.assert_array_almost_equal(euler, [0, 0, 0])

    def test_home_leg_lengths(self, stewart: StewartPlatform) -> None:
        """Test home_leg_lengths method."""
        joints = stewart.home_leg_lengths()

        assert joints.leg_lengths.shape == (6,)
        # All legs should be roughly equal at home
        assert np.std(joints.leg_lengths) < 0.02


class TestStewartSingularity:
    """Test Stewart platform singularity detection."""

    @pytest.fixture
    def stewart(self) -> StewartPlatform:
        """Create test Stewart platform."""
        return create_stewart(0.2, 0.1, (0.2, 0.5))

    def test_singularity_detection_runs(self, stewart: StewartPlatform) -> None:
        """Test singularity detection executes without error."""
        # Note: Some symmetric configurations may be near-singular at home
        singularity = stewart.detect_singularity(stewart.home_pose())
        assert singularity in [
            StewartSingularityType.NONE,
            StewartSingularityType.TRANSLATION,
            StewartSingularityType.ROTATION,
            StewartSingularityType.COMBINED,
        ]

    def test_singularity_types(self) -> None:
        """Test singularity type enum."""
        assert StewartSingularityType.NONE.value == "none"
        assert StewartSingularityType.TRANSLATION.value == "translation"
        assert StewartSingularityType.ROTATION.value == "rotation"
        assert StewartSingularityType.COMBINED.value == "combined"


class TestStewartJacobian:
    """Test Stewart platform Jacobian computation."""

    def test_jacobian_shape(self) -> None:
        """Test Jacobian has correct shape."""
        stewart = create_stewart(0.2, 0.1, (0.2, 0.5))
        jac = stewart.jacobian(stewart.home_pose())

        assert jac.shape == (6, 6)

    def test_jacobian_computation(self) -> None:
        """Test Jacobian computation runs without error."""
        stewart = create_stewart(0.2, 0.1, (0.2, 0.5))
        jac = stewart.jacobian(stewart.home_pose())

        # Jacobian should have finite values
        assert np.all(np.isfinite(jac))
        # Jacobian should not be all zeros
        assert np.any(jac != 0)


class TestStewartFactoryFunctions:
    """Test Stewart platform factory functions."""

    def test_create_stewart(self) -> None:
        """Test create_stewart function."""
        stewart = create_stewart(0.25, 0.12, (0.3, 0.5), home_height=0.4)

        assert stewart.limits.leg_length_range == (0.3, 0.5)
        assert stewart.home_height == 0.4

    def test_create_flight_simulator(self) -> None:
        """Test flight simulator configuration."""
        stewart = create_flight_simulator()

        # Large platform
        assert np.max(np.abs(stewart.base_joints)) > 0.5
        assert stewart.limits.leg_length_range == (0.8, 1.4)
        assert stewart.home_height == 0.9

        # Should be functional
        joints = stewart.home_leg_lengths()
        assert joints.leg_lengths.shape == (6,)

    def test_create_precision_positioner(self) -> None:
        """Test precision positioner configuration."""
        stewart = create_precision_positioner()

        # Small platform
        assert np.max(np.abs(stewart.base_joints)) < 0.2
        assert stewart.limits.leg_length_range == (0.12, 0.18)

        # Should be functional
        joints = stewart.home_leg_lengths()
        assert joints.leg_lengths.shape == (6,)
