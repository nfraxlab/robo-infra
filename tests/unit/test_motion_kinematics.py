"""Unit tests for kinematics module.

Tests cover:
- JointAngle and EndEffectorPose dataclasses
- JointLimits validation and clamping
- KinematicChain base class utilities
- TwoLinkArm forward and inverse kinematics
- ThreeLinkArm forward and inverse kinematics
- Roundtrip (FK -> IK -> FK) consistency
- Edge cases and error handling
"""

from __future__ import annotations

import math

import pytest

from robo_infra.motion import (
    ElbowConfiguration,
    EndEffectorPose,
    JointAngle,
    JointLimitError,
    JointLimits,
    KinematicsError,
    ThreeLinkArm,
    TwoLinkArm,
    UnreachablePositionError,
)


# =============================================================================
# JointAngle Tests
# =============================================================================


class TestJointAngle:
    """Tests for JointAngle dataclass."""

    def test_joint_angle_creation(self) -> None:
        """Test basic JointAngle creation."""
        joint = JointAngle(angle=1.57, name="shoulder")
        assert joint.angle == 1.57
        assert joint.name == "shoulder"
        assert joint.min_angle == -math.pi
        assert joint.max_angle == math.pi

    def test_joint_angle_degrees(self) -> None:
        """Test degrees property."""
        joint = JointAngle(angle=math.pi / 2)
        assert abs(joint.degrees - 90.0) < 0.001

    def test_joint_angle_is_within_limits(self) -> None:
        """Test is_within_limits property."""
        joint = JointAngle(angle=1.0, min_angle=-2.0, max_angle=2.0)
        assert joint.is_within_limits is True

        joint_out = JointAngle(angle=3.0, min_angle=-2.0, max_angle=2.0)
        assert joint_out.is_within_limits is False

    def test_joint_angle_clamped(self) -> None:
        """Test clamped method."""
        joint = JointAngle(angle=5.0, min_angle=-2.0, max_angle=2.0)
        clamped = joint.clamped()
        assert clamped.angle == 2.0
        assert clamped.name == joint.name

    def test_joint_angle_normalized(self) -> None:
        """Test normalized method."""
        joint = JointAngle(angle=3 * math.pi)
        normalized = joint.normalized()
        assert abs(normalized.angle - math.pi) < 0.001

    def test_joint_angle_invalid_limits_raises(self) -> None:
        """Test that invalid limits raise ValueError."""
        with pytest.raises(ValueError, match="min_angle.*must be <= max_angle"):
            JointAngle(angle=0.0, min_angle=2.0, max_angle=1.0)

    def test_joint_angle_repr(self) -> None:
        """Test string representation."""
        joint = JointAngle(angle=math.pi / 2, name="elbow")
        assert "elbow" in repr(joint)
        assert "90" in repr(joint)


# =============================================================================
# EndEffectorPose Tests
# =============================================================================


class TestEndEffectorPose:
    """Tests for EndEffectorPose dataclass."""

    def test_pose_creation(self) -> None:
        """Test basic pose creation."""
        pose = EndEffectorPose(x=100.0, y=50.0)
        assert pose.x == 100.0
        assert pose.y == 50.0
        assert pose.z == 0.0
        assert pose.orientation == 0.0

    def test_pose_with_z_and_orientation(self) -> None:
        """Test pose with z and orientation."""
        pose = EndEffectorPose(x=100.0, y=50.0, z=25.0, orientation=1.0)
        assert pose.z == 25.0
        assert pose.orientation == 1.0

    def test_distance_from_origin(self) -> None:
        """Test distance_from_origin property."""
        pose = EndEffectorPose(x=3.0, y=4.0)
        assert abs(pose.distance_from_origin - 5.0) < 0.001

    def test_planar_distance(self) -> None:
        """Test planar_distance property."""
        pose = EndEffectorPose(x=3.0, y=4.0, z=10.0)
        assert abs(pose.planar_distance - 5.0) < 0.001

    def test_angle_from_x_axis(self) -> None:
        """Test angle_from_x_axis property."""
        pose = EndEffectorPose(x=1.0, y=1.0)
        assert abs(pose.angle_from_x_axis - math.pi / 4) < 0.001

    def test_offset(self) -> None:
        """Test offset method."""
        pose = EndEffectorPose(x=100.0, y=50.0)
        offset_pose = pose.offset(dx=10.0, dy=-5.0)
        assert offset_pose.x == 110.0
        assert offset_pose.y == 45.0

    def test_scaled(self) -> None:
        """Test scaled method."""
        pose = EndEffectorPose(x=100.0, y=50.0, z=25.0)
        scaled = pose.scaled(2.0)
        assert scaled.x == 200.0
        assert scaled.y == 100.0
        assert scaled.z == 50.0

    def test_rotated_z(self) -> None:
        """Test rotated_z method."""
        pose = EndEffectorPose(x=1.0, y=0.0)
        rotated = pose.rotated_z(math.pi / 2)
        assert abs(rotated.x) < 0.001
        assert abs(rotated.y - 1.0) < 0.001

    def test_pose_repr_simple(self) -> None:
        """Test simple repr."""
        pose = EndEffectorPose(x=100.0, y=50.0)
        assert "100" in repr(pose)
        assert "50" in repr(pose)


# =============================================================================
# JointLimits Tests
# =============================================================================


class TestJointLimits:
    """Tests for JointLimits dataclass."""

    def test_joint_limits_creation(self) -> None:
        """Test basic creation."""
        limits = JointLimits(min_angle=-1.0, max_angle=1.0, name="shoulder")
        assert limits.min_angle == -1.0
        assert limits.max_angle == 1.0
        assert limits.name == "shoulder"

    def test_joint_limits_defaults(self) -> None:
        """Test default limits."""
        limits = JointLimits()
        assert limits.min_angle == -math.pi
        assert limits.max_angle == math.pi

    def test_contains(self) -> None:
        """Test contains method."""
        limits = JointLimits(min_angle=-1.0, max_angle=1.0)
        assert limits.contains(0.5) is True
        assert limits.contains(-0.5) is True
        assert limits.contains(1.5) is False
        assert limits.contains(-1.5) is False

    def test_clamp(self) -> None:
        """Test clamp method."""
        limits = JointLimits(min_angle=-1.0, max_angle=1.0)
        assert limits.clamp(0.5) == 0.5
        assert limits.clamp(2.0) == 1.0
        assert limits.clamp(-2.0) == -1.0

    def test_range(self) -> None:
        """Test range property."""
        limits = JointLimits(min_angle=-1.0, max_angle=1.0)
        assert limits.range == 2.0

    def test_center(self) -> None:
        """Test center property."""
        limits = JointLimits(min_angle=-1.0, max_angle=1.0)
        assert limits.center == 0.0

    def test_invalid_limits_raises(self) -> None:
        """Test that invalid limits raise ValueError."""
        with pytest.raises(ValueError, match="min_angle.*must be <= max_angle"):
            JointLimits(min_angle=2.0, max_angle=1.0)


# =============================================================================
# TwoLinkArm Forward Kinematics Tests
# =============================================================================


class TestTwoLinkArmFK:
    """Forward kinematics tests for TwoLinkArm."""

    def test_2dof_fk_zero_angles(self) -> None:
        """Test FK with zero angles (arm extended along X axis)."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        pose = arm.forward([0.0, 0.0])
        assert abs(pose.x - 200.0) < 0.001
        assert abs(pose.y) < 0.001

    def test_2dof_fk_90_degree_angles(self) -> None:
        """Test FK with both joints at 90 degrees."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        pose = arm.forward([math.pi / 2, math.pi / 2])
        # First link points up (0, 100)
        # Second link points left from there (-100, 100)
        assert abs(pose.x - (-100.0)) < 0.001
        assert abs(pose.y - 100.0) < 0.001

    def test_2dof_fk_extended(self) -> None:
        """Test FK at maximum extension."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        pose = arm.forward([0.0, 0.0])
        assert abs(pose.x - 180.0) < 0.001
        assert abs(pose.y) < 0.001

    def test_2dof_fk_folded(self) -> None:
        """Test FK when arm is folded back."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        pose = arm.forward([0.0, math.pi])
        # Second link folds back, so x = 100 - 80 = 20
        assert abs(pose.x - 20.0) < 0.001
        assert abs(pose.y) < 0.001

    def test_2dof_fk_elbow_position(self) -> None:
        """Test elbow position calculation."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        elbow = arm.elbow_position([math.pi / 4, 0.0])
        expected_x = 100.0 * math.cos(math.pi / 4)
        expected_y = 100.0 * math.sin(math.pi / 4)
        assert abs(elbow[0] - expected_x) < 0.001
        assert abs(elbow[1] - expected_y) < 0.001

    def test_2dof_fk_wrong_angles_count_raises(self) -> None:
        """Test that wrong number of angles raises ValueError."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        with pytest.raises(ValueError, match="Expected 2 angles"):
            arm.forward([0.0, 0.0, 0.0])

    def test_2dof_fk_orientation(self) -> None:
        """Test that orientation is set correctly."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        pose = arm.forward([0.5, 0.3])
        assert abs(pose.orientation - 0.8) < 0.001


# =============================================================================
# TwoLinkArm Inverse Kinematics Tests
# =============================================================================


class TestTwoLinkArmIK:
    """Inverse kinematics tests for TwoLinkArm."""

    def test_2dof_ik_reachable_point(self) -> None:
        """Test IK for a reachable point."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        target = EndEffectorPose(x=150.0, y=50.0)
        angles = arm.inverse(target)
        assert len(angles) == 2

        # Verify by forward kinematics
        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.001
        assert abs(result.y - target.y) < 0.001

    def test_2dof_ik_elbow_up(self) -> None:
        """Test IK with elbow up configuration."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        target = EndEffectorPose(x=100.0, y=100.0)
        angles = arm.inverse(target, configuration=ElbowConfiguration.UP)

        # Elbow up means θ2 should be negative
        assert angles[1] < 0

        # Verify position
        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.001
        assert abs(result.y - target.y) < 0.001

    def test_2dof_ik_elbow_down(self) -> None:
        """Test IK with elbow down configuration."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        target = EndEffectorPose(x=100.0, y=100.0)
        angles = arm.inverse(target, configuration=ElbowConfiguration.DOWN)

        # Elbow down means θ2 should be positive
        assert angles[1] > 0

        # Verify position
        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.001
        assert abs(result.y - target.y) < 0.001

    def test_2dof_ik_unreachable_too_far_raises(self) -> None:
        """Test IK raises for unreachable point (too far)."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        target = EndEffectorPose(x=300.0, y=0.0)  # Max reach is 200

        with pytest.raises(UnreachablePositionError):
            arm.inverse(target)

    def test_2dof_ik_unreachable_too_close_raises(self) -> None:
        """Test IK raises for unreachable point (too close)."""
        arm = TwoLinkArm(l1=100.0, l2=50.0)  # Min reach is 50
        target = EndEffectorPose(x=10.0, y=0.0)

        with pytest.raises(UnreachablePositionError):
            arm.inverse(target)

    def test_2dof_ik_on_boundary_outer(self) -> None:
        """Test IK on outer boundary (max reach)."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        target = EndEffectorPose(x=200.0, y=0.0)  # Exactly at max reach
        angles = arm.inverse(target)

        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.001
        assert abs(result.y - target.y) < 0.001

    def test_2dof_ik_on_boundary_inner(self) -> None:
        """Test IK on inner boundary (min reach)."""
        arm = TwoLinkArm(l1=100.0, l2=50.0)  # Min reach is 50
        target = EndEffectorPose(x=50.0, y=0.0)
        angles = arm.inverse(target)

        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.01
        assert abs(result.y - target.y) < 0.01

    def test_2dof_ik_negative_quadrant(self) -> None:
        """Test IK in negative Y quadrant."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        target = EndEffectorPose(x=100.0, y=-100.0)
        angles = arm.inverse(target)

        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.001
        assert abs(result.y - target.y) < 0.001


# =============================================================================
# TwoLinkArm Additional Tests
# =============================================================================


class TestTwoLinkArmProperties:
    """Tests for TwoLinkArm properties and utilities."""

    def test_link_lengths(self) -> None:
        """Test l1 and l2 properties."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        assert arm.l1 == 100.0
        assert arm.l2 == 80.0

    def test_max_reach(self) -> None:
        """Test max_reach property."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        assert arm.max_reach == 180.0

    def test_min_reach(self) -> None:
        """Test min_reach property."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        assert arm.min_reach == 20.0

    def test_min_reach_equal_links(self) -> None:
        """Test min_reach with equal length links."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        assert arm.min_reach == 0.0

    def test_is_reachable(self) -> None:
        """Test is_reachable method."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        assert arm.is_reachable(EndEffectorPose(x=100.0, y=50.0)) is True
        assert arm.is_reachable(EndEffectorPose(x=200.0, y=0.0)) is False
        assert arm.is_reachable(EndEffectorPose(x=10.0, y=0.0)) is False

    def test_analytical_jacobian(self) -> None:
        """Test analytical Jacobian."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        jacobian = arm.analytical_jacobian([0.0, 0.0])

        # At [0, 0]: dx/dθ1 = 0, dx/dθ2 = 0, dy/dθ1 = 180, dy/dθ2 = 80
        assert abs(jacobian[0][0]) < 0.001
        assert abs(jacobian[0][1]) < 0.001
        assert abs(jacobian[1][0] - 180.0) < 0.001
        assert abs(jacobian[1][1] - 80.0) < 0.001

    def test_is_singular_extended(self) -> None:
        """Test singularity detection when extended."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        assert arm.is_singular([0.0, 0.0]) is True

    def test_is_singular_folded(self) -> None:
        """Test singularity detection when folded."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        assert arm.is_singular([0.0, math.pi]) is True

    def test_is_singular_normal(self) -> None:
        """Test singularity detection in normal position."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        assert arm.is_singular([0.0, math.pi / 2]) is False

    def test_workspace_boundary(self) -> None:
        """Test workspace boundary generation."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        boundary = arm.workspace_boundary(num_points=10)

        # Should have points for outer and inner boundary
        assert len(boundary) == 20

        # Check outer boundary points are at max reach
        for pose in boundary[:10]:
            dist = pose.planar_distance
            assert abs(dist - arm.max_reach) < 0.001 or abs(dist - arm.min_reach) < 0.001

    def test_invalid_link_length_raises(self) -> None:
        """Test that non-positive link lengths raise ValueError."""
        with pytest.raises(ValueError, match="l1 must be positive"):
            TwoLinkArm(l1=0.0, l2=100.0)

        with pytest.raises(ValueError, match="l2 must be positive"):
            TwoLinkArm(l1=100.0, l2=-10.0)

    def test_num_joints(self) -> None:
        """Test num_joints property."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        assert arm.num_joints == 2

    def test_repr(self) -> None:
        """Test string representation."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        assert "TwoLinkArm" in repr(arm)
        assert "100" in repr(arm)
        assert "80" in repr(arm)


# =============================================================================
# ThreeLinkArm Forward Kinematics Tests
# =============================================================================


class TestThreeLinkArmFK:
    """Forward kinematics tests for ThreeLinkArm."""

    def test_3dof_fk_zero_angles(self) -> None:
        """Test FK with zero angles (arm extended along X axis)."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        pose = arm.forward([0.0, 0.0, 0.0])
        assert abs(pose.x - 230.0) < 0.001
        assert abs(pose.y) < 0.001
        assert abs(pose.orientation) < 0.001

    def test_3dof_fk_90_degree_first_joint(self) -> None:
        """Test FK with first joint at 90 degrees."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        pose = arm.forward([math.pi / 2, 0.0, 0.0])
        assert abs(pose.x) < 0.001
        assert abs(pose.y - 230.0) < 0.001

    def test_3dof_fk_orientation(self) -> None:
        """Test that orientation equals sum of angles."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        angles = [0.3, 0.4, 0.5]
        pose = arm.forward(angles)
        assert abs(pose.orientation - 1.2) < 0.001

    def test_3dof_fk_elbow_position(self) -> None:
        """Test elbow position calculation."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        elbow = arm.elbow_position([math.pi / 4, 0.0, 0.0])
        expected_x = 100.0 * math.cos(math.pi / 4)
        expected_y = 100.0 * math.sin(math.pi / 4)
        assert abs(elbow[0] - expected_x) < 0.001
        assert abs(elbow[1] - expected_y) < 0.001

    def test_3dof_fk_wrist_position(self) -> None:
        """Test wrist position calculation."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        wrist = arm.wrist_position([0.0, 0.0, 0.0])
        assert abs(wrist[0] - 180.0) < 0.001
        assert abs(wrist[1]) < 0.001

    def test_3dof_fk_wrong_angles_count_raises(self) -> None:
        """Test that wrong number of angles raises ValueError."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        with pytest.raises(ValueError, match="Expected 3 angles"):
            arm.forward([0.0, 0.0])


# =============================================================================
# ThreeLinkArm Inverse Kinematics Tests
# =============================================================================


class TestThreeLinkArmIK:
    """Inverse kinematics tests for ThreeLinkArm."""

    def test_3dof_ik_with_orientation(self) -> None:
        """Test IK with specified orientation."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        target = EndEffectorPose(x=150.0, y=50.0, orientation=0.5)
        angles = arm.inverse(target)

        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.01
        assert abs(result.y - target.y) < 0.01
        assert abs(result.orientation - target.orientation) < 0.01

    def test_3dof_ik_elbow_configurations(self) -> None:
        """Test that different elbow configurations give different solutions."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        target = EndEffectorPose(x=150.0, y=50.0, orientation=0.3)

        angles_up = arm.inverse(target, configuration=ElbowConfiguration.UP)
        angles_down = arm.inverse(target, configuration=ElbowConfiguration.DOWN)

        # θ2 should have opposite signs
        assert angles_up[1] * angles_down[1] < 0

        # Both should reach the target
        result_up = arm.forward(angles_up)
        result_down = arm.forward(angles_down)
        assert abs(result_up.x - target.x) < 0.01
        assert abs(result_down.x - target.x) < 0.01

    def test_3dof_ik_unreachable_raises(self) -> None:
        """Test IK raises for unreachable position."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        target = EndEffectorPose(x=400.0, y=0.0)

        with pytest.raises(UnreachablePositionError):
            arm.inverse(target)

    def test_3dof_ik_position_only(self) -> None:
        """Test position-only IK."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        target = EndEffectorPose(x=150.0, y=80.0)
        angles = arm.inverse_position_only(target)

        result = arm.forward(angles)
        assert abs(result.x - target.x) < 0.1
        assert abs(result.y - target.y) < 0.1


# =============================================================================
# ThreeLinkArm Additional Tests
# =============================================================================


class TestThreeLinkArmProperties:
    """Tests for ThreeLinkArm properties and utilities."""

    def test_link_lengths(self) -> None:
        """Test l1, l2, l3 properties."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        assert arm.l1 == 100.0
        assert arm.l2 == 80.0
        assert arm.l3 == 50.0

    def test_max_reach(self) -> None:
        """Test max_reach property."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        assert arm.max_reach == 230.0

    def test_min_reach(self) -> None:
        """Test min_reach property."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        # Min reach should be 0 since 80+50 > 100
        assert arm.min_reach == 0.0

    def test_analytical_jacobian(self) -> None:
        """Test analytical Jacobian is 3x3."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        jacobian = arm.analytical_jacobian([0.0, 0.0, 0.0])

        assert len(jacobian) == 3
        assert len(jacobian[0]) == 3

        # Orientation row should be all 1s
        assert jacobian[2] == [1.0, 1.0, 1.0]

    def test_is_singular(self) -> None:
        """Test singularity detection."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        assert arm.is_singular([0.0, 0.0, 0.0]) is True
        assert arm.is_singular([0.0, math.pi / 2, 0.0]) is False

    def test_invalid_link_length_raises(self) -> None:
        """Test that non-positive link lengths raise ValueError."""
        with pytest.raises(ValueError, match="l1 must be positive"):
            ThreeLinkArm(l1=0.0, l2=80.0, l3=50.0)

        with pytest.raises(ValueError, match="l2 must be positive"):
            ThreeLinkArm(l1=100.0, l2=-10.0, l3=50.0)

        with pytest.raises(ValueError, match="l3 must be positive"):
            ThreeLinkArm(l1=100.0, l2=80.0, l3=0.0)

    def test_num_joints(self) -> None:
        """Test num_joints property."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        assert arm.num_joints == 3

    def test_repr(self) -> None:
        """Test string representation."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        assert "ThreeLinkArm" in repr(arm)
        assert "100" in repr(arm)
        assert "80" in repr(arm)
        assert "50" in repr(arm)


# =============================================================================
# Roundtrip Tests (FK -> IK -> FK)
# =============================================================================


class TestRoundtrip:
    """Roundtrip consistency tests."""

    def test_fk_ik_roundtrip_2dof(self) -> None:
        """Test FK -> IK -> FK roundtrip for 2-DOF arm."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)

        # Start with known angles
        original_angles = [0.5, 0.3]
        pose = arm.forward(original_angles)

        # Solve IK
        recovered_angles = arm.inverse(pose)

        # Forward again
        recovered_pose = arm.forward(recovered_angles)

        # Positions should match
        assert abs(pose.x - recovered_pose.x) < 0.001
        assert abs(pose.y - recovered_pose.y) < 0.001

    def test_fk_ik_roundtrip_2dof_multiple_poses(self) -> None:
        """Test roundtrip for multiple poses."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)

        test_angles = [
            [0.0, 0.0],
            [0.5, -0.5],
            [1.0, 0.5],
            [-0.5, 1.0],
            [math.pi / 4, math.pi / 3],
        ]

        for angles in test_angles:
            pose = arm.forward(angles)
            recovered = arm.inverse(pose)
            result = arm.forward(recovered)

            assert abs(pose.x - result.x) < 0.001
            assert abs(pose.y - result.y) < 0.001

    def test_fk_ik_roundtrip_3dof(self) -> None:
        """Test FK -> IK -> FK roundtrip for 3-DOF arm."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)

        # Start with known angles
        original_angles = [0.3, 0.4, 0.2]
        pose = arm.forward(original_angles)

        # Solve IK (with orientation)
        recovered_angles = arm.inverse(pose)

        # Forward again
        recovered_pose = arm.forward(recovered_angles)

        # Position and orientation should match
        assert abs(pose.x - recovered_pose.x) < 0.01
        assert abs(pose.y - recovered_pose.y) < 0.01
        assert abs(pose.orientation - recovered_pose.orientation) < 0.01

    def test_fk_ik_roundtrip_3dof_multiple_poses(self) -> None:
        """Test roundtrip for multiple 3-DOF poses."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)

        test_angles = [
            [0.0, 0.0, 0.0],
            [0.3, 0.4, 0.2],
            [0.5, -0.3, 0.1],
            [-0.2, 0.5, -0.3],
        ]

        for angles in test_angles:
            pose = arm.forward(angles)
            recovered = arm.inverse(pose)
            result = arm.forward(recovered)

            assert abs(pose.x - result.x) < 0.01
            assert abs(pose.y - result.y) < 0.01
            assert abs(pose.orientation - result.orientation) < 0.01


# =============================================================================
# KinematicChain Base Class Tests
# =============================================================================


class TestKinematicChainBase:
    """Tests for KinematicChain base class utilities."""

    def test_validate_angles(self) -> None:
        """Test validate_angles method."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        arm._joint_limits = [
            JointLimits(min_angle=-1.0, max_angle=1.0),
            JointLimits(min_angle=-1.0, max_angle=1.0),
        ]

        assert arm.validate_angles([0.5, 0.5]) is True

        with pytest.raises(JointLimitError):
            arm.validate_angles([2.0, 0.5])

    def test_validate_angles_no_raise(self) -> None:
        """Test validate_angles with raise_on_error=False."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        arm._joint_limits = [
            JointLimits(min_angle=-1.0, max_angle=1.0),
            JointLimits(min_angle=-1.0, max_angle=1.0),
        ]

        assert arm.validate_angles([2.0, 0.5], raise_on_error=False) is False

    def test_clamp_angles(self) -> None:
        """Test clamp_angles method."""
        arm = TwoLinkArm(l1=100.0, l2=100.0)
        arm._joint_limits = [
            JointLimits(min_angle=-1.0, max_angle=1.0),
            JointLimits(min_angle=-0.5, max_angle=0.5),
        ]

        clamped = arm.clamp_angles([2.0, -1.0])
        assert clamped[0] == 1.0
        assert clamped[1] == -0.5

    def test_jacobian_numerical(self) -> None:
        """Test numerical Jacobian calculation."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)

        # Compare numerical to analytical
        angles = [0.3, 0.4]
        numerical = arm.jacobian(angles)
        analytical = arm.analytical_jacobian(angles)

        for i in range(2):
            for j in range(2):
                assert abs(numerical[i][j] - analytical[i][j]) < 0.01

    def test_link_lengths_property(self) -> None:
        """Test link_lengths returns a copy."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        lengths = arm.link_lengths
        lengths[0] = 999.0
        assert arm.link_lengths[0] == 100.0

    def test_joint_limits_property(self) -> None:
        """Test joint_limits returns a copy."""
        arm = TwoLinkArm(l1=100.0, l2=80.0)
        limits = arm.joint_limits
        assert len(limits) == 2

    def test_total_length(self) -> None:
        """Test total_length property."""
        arm = ThreeLinkArm(l1=100.0, l2=80.0, l3=50.0)
        assert arm.total_length == 230.0


# =============================================================================
# Error Classes Tests
# =============================================================================


class TestErrors:
    """Tests for error classes."""

    def test_unreachable_position_error(self) -> None:
        """Test UnreachablePositionError message."""
        pose = EndEffectorPose(x=300.0, y=0.0)
        error = UnreachablePositionError(pose, max_reach=200.0, min_reach=20.0)

        assert "300" in str(error)
        assert "unreachable" in str(error).lower()
        assert error.pose == pose
        assert error.max_reach == 200.0
        assert error.min_reach == 20.0

    def test_joint_limit_error(self) -> None:
        """Test JointLimitError message."""
        error = JointLimitError(
            joint_index=1,
            angle=2.0,
            min_angle=-1.0,
            max_angle=1.0,
        )

        assert "Joint 1" in str(error)
        assert "exceeds limits" in str(error).lower()
        assert error.joint_index == 1
        assert error.angle == 2.0

    def test_kinematics_error_base(self) -> None:
        """Test KinematicsError is base exception."""
        error = KinematicsError("Test error")
        assert str(error) == "Test error"
        assert isinstance(error, Exception)
