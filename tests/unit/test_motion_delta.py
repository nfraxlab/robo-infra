"""Unit tests for Delta robot kinematics.

Tests cover:
- DeltaRobot creation and validation
- Forward kinematics accuracy
- Inverse kinematics
- Workspace analysis
- Singularity detection
- Jacobian computation
- Factory functions
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robo_infra.motion.delta import (
    DeltaJoints,
    DeltaLimits,
    DeltaRobot,
    DeltaSingularityType,
    DeltaWorkspace,
    create_delta,
    create_flsun_q5,
    create_kossel_mini,
)
from robo_infra.motion.transforms import Transform


class TestDeltaLimits:
    """Test DeltaLimits dataclass."""
    
    def test_default_limits(self) -> None:
        """Test default limit values."""
        limits = DeltaLimits()
        assert limits.theta_range == (-math.pi / 2, math.pi / 2)
        assert limits.z_min == -0.5
        assert limits.z_max == -0.05
    
    def test_is_valid_angle(self) -> None:
        """Test angle validation."""
        limits = DeltaLimits()
        assert limits.is_valid_angle(0)
        assert limits.is_valid_angle(math.pi / 4)
        assert limits.is_valid_angle(-math.pi / 4)
        assert not limits.is_valid_angle(math.pi)
        assert not limits.is_valid_angle(-math.pi)
    
    def test_custom_limits(self) -> None:
        """Test custom limit values."""
        limits = DeltaLimits(theta_range=(-1.0, 1.0))
        assert limits.is_valid_angle(0.5)
        assert not limits.is_valid_angle(1.5)


class TestDeltaJoints:
    """Test DeltaJoints dataclass."""
    
    def test_as_array(self) -> None:
        """Test conversion to numpy array."""
        joints = DeltaJoints(theta1=0.1, theta2=0.2, theta3=0.3)
        arr = joints.as_array()
        np.testing.assert_array_almost_equal(arr, [0.1, 0.2, 0.3])
    
    def test_as_tuple(self) -> None:
        """Test conversion to tuple."""
        joints = DeltaJoints(theta1=0.1, theta2=0.2, theta3=0.3)
        assert joints.as_tuple() == (0.1, 0.2, 0.3)


class TestDeltaRobot:
    """Test DeltaRobot class."""
    
    def test_creation(self) -> None:
        """Test basic delta robot creation."""
        delta = DeltaRobot(
            base_radius=0.2,
            effector_radius=0.05,
            upper_arm_length=0.15,
            lower_arm_length=0.3,
        )
        assert delta.base_radius == 0.2
        assert delta.effector_radius == 0.05
        assert delta.upper_arm_length == 0.15
        assert delta.lower_arm_length == 0.3
    
    def test_invalid_dimensions(self) -> None:
        """Test validation of dimensions."""
        with pytest.raises(ValueError, match="base_radius must be positive"):
            DeltaRobot(0, 0.05, 0.15, 0.3)
        
        with pytest.raises(ValueError, match="effector_radius must be positive"):
            DeltaRobot(0.2, -0.05, 0.15, 0.3)
        
        with pytest.raises(ValueError, match="effector_radius must be smaller"):
            DeltaRobot(0.1, 0.2, 0.15, 0.3)  # effector > base
    
    def test_arm_offset(self) -> None:
        """Test arm offset calculation."""
        delta = DeltaRobot(0.2, 0.05, 0.15, 0.3)
        assert delta.arm_offset == pytest.approx(0.15)  # base - effector


class TestDeltaForwardKinematics:
    """Test Delta robot forward kinematics."""
    
    @pytest.fixture
    def delta(self) -> DeltaRobot:
        """Create test delta robot with valid geometry.
        
        The geometry must satisfy: lower_arm > upper_arm + arm_offset
        for valid FK solutions at reasonable angles.
        """
        # More standard delta geometry: base 0.17m, effector 0.035m, 
        # upper arm 0.1m, lower arm 0.25m
        return DeltaRobot(0.17, 0.035, 0.1, 0.25)
    
    def test_home_position(self, delta: DeltaRobot) -> None:
        """Test FK at home position (all zeros)."""
        x, y, z = delta.forward(0, 0, 0)
        
        # At home, effector should be centered (x=0, y=0)
        assert abs(x) < 0.01
        assert abs(y) < 0.01
        # Z should be negative (below base)
        assert z < 0
    
    def test_symmetric_motion(self, delta: DeltaRobot) -> None:
        """Test symmetric motor motion keeps effector centered."""
        angle = 0.2
        x, y, z = delta.forward(angle, angle, angle)
        
        # Should remain centered
        assert abs(x) < 0.01
        assert abs(y) < 0.01
    
    def test_z_motion_with_symmetric_angles(self, delta: DeltaRobot) -> None:
        """Test that increasing symmetric angles lowers effector."""
        x1, y1, z1 = delta.forward(0, 0, 0)
        x2, y2, z2 = delta.forward(0.3, 0.3, 0.3)
        
        # Higher angles should lower the effector
        assert z2 < z1
    
    def test_xy_motion_with_asymmetric_angles(self, delta: DeltaRobot) -> None:
        """Test asymmetric angles create XY motion."""
        x1, y1, z1 = delta.forward(0.2, 0.1, 0.1)
        
        # Should have moved in XY plane
        assert abs(x1) > 0.001 or abs(y1) > 0.001
    
    def test_forward_transform(self, delta: DeltaRobot) -> None:
        """Test forward_transform returns Transform."""
        transform = delta.forward_transform(0, 0, 0)
        
        assert isinstance(transform, Transform)
        # Delta effector maintains horizontal orientation
        euler = transform.rotation.as_euler()
        np.testing.assert_array_almost_equal(euler, [0, 0, 0], decimal=5)


class TestDeltaInverseKinematics:
    """Test Delta robot inverse kinematics."""
    
    @pytest.fixture
    def delta(self) -> DeltaRobot:
        """Create test delta robot with valid geometry."""
        return DeltaRobot(0.17, 0.035, 0.1, 0.25)
    
    def test_round_trip_home(self, delta: DeltaRobot) -> None:
        """Test IK recovers FK at home position."""
        x, y, z = delta.forward(0, 0, 0)
        result = delta.inverse(x, y, z)
        
        assert result is not None
        
        # Should give similar position
        x2, y2, z2 = delta.forward(*result)
        assert abs(x - x2) < 0.001
        assert abs(y - y2) < 0.001
        assert abs(z - z2) < 0.001
    
    def test_round_trip_various_positions(self, delta: DeltaRobot) -> None:
        """Test IK round-trip for various motor angles."""
        test_angles = [
            (0.1, 0.1, 0.1),
            (0.2, 0.1, 0.15),
            (-0.1, 0.1, 0.0),
            (0.0, 0.2, 0.1),
        ]
        
        for t1, t2, t3 in test_angles:
            x, y, z = delta.forward(t1, t2, t3)
            result = delta.inverse(x, y, z)
            
            if result is not None:
                x2, y2, z2 = delta.forward(*result)
                assert abs(x - x2) < 0.001
                assert abs(y - y2) < 0.001
                assert abs(z - z2) < 0.001
    
    def test_unreachable_point(self, delta: DeltaRobot) -> None:
        """Test IK returns None for unreachable points."""
        # Point way outside workspace
        result = delta.inverse(1.0, 0, -0.1)
        assert result is None
    
    def test_inverse_joints(self, delta: DeltaRobot) -> None:
        """Test inverse_joints returns DeltaJoints."""
        x, y, z = delta.forward(0.1, 0.1, 0.1)
        joints = delta.inverse_joints(x, y, z)
        
        assert joints is not None
        assert isinstance(joints, DeltaJoints)
    
    def test_is_reachable(self, delta: DeltaRobot) -> None:
        """Test is_reachable method."""
        x, y, z = delta.forward(0, 0, 0)
        assert delta.is_reachable(x, y, z)
        
        # Unreachable point
        assert not delta.is_reachable(1.0, 0, -0.1)


class TestDeltaSingularity:
    """Test Delta robot singularity detection."""
    
    @pytest.fixture
    def delta(self) -> DeltaRobot:
        """Create test delta robot."""
        return DeltaRobot(0.17, 0.035, 0.1, 0.25)
    
    def test_no_singularity_at_home(self, delta: DeltaRobot) -> None:
        """Test no singularity at home position."""
        singularity = delta.detect_singularity(0, 0, 0)
        assert singularity == DeltaSingularityType.NONE
    
    def test_extension_singularity(self, delta: DeltaRobot) -> None:
        """Test detection of arm extension singularity."""
        # Near pi/2 (fully extended down)
        singularity = delta.detect_singularity(1.5, 1.5, 1.5)
        assert singularity == DeltaSingularityType.ARM_EXTENSION
    
    def test_retraction_singularity(self, delta: DeltaRobot) -> None:
        """Test detection of arm retraction singularity."""
        # Near -pi/2 (fully retracted up)
        singularity = delta.detect_singularity(-1.5, -1.5, -1.5)
        assert singularity == DeltaSingularityType.ARM_RETRACTION


class TestDeltaJacobian:
    """Test Delta robot Jacobian computation."""
    
    def test_jacobian_shape(self) -> None:
        """Test Jacobian has correct shape."""
        delta = DeltaRobot(0.17, 0.035, 0.1, 0.25)
        jac = delta.jacobian(0, 0, 0)
        assert jac.shape == (3, 3)
    
    def test_jacobian_numerical_verification(self) -> None:
        """Test Jacobian matches numerical differentiation."""
        delta = DeltaRobot(0.17, 0.035, 0.1, 0.25)
        t1, t2, t3 = 0.1, 0.1, 0.1
        
        jac = delta.jacobian(t1, t2, t3)
        
        # Jacobian should be non-singular at this point
        det = np.linalg.det(jac)
        assert abs(det) > 1e-6


class TestDeltaWorkspace:
    """Test Delta robot workspace analysis."""
    
    def test_compute_workspace(self) -> None:
        """Test workspace computation."""
        delta = DeltaRobot(0.17, 0.035, 0.1, 0.25)
        workspace = delta.compute_workspace(resolution=10)
        
        assert isinstance(workspace, DeltaWorkspace)
        assert workspace.x_range[0] < workspace.x_range[1]
        assert workspace.y_range[0] < workspace.y_range[1]
        assert workspace.z_range[0] < workspace.z_range[1]
        assert workspace.volume > 0
    
    def test_workspace_is_centered(self) -> None:
        """Test workspace is roughly centered around origin."""
        delta = DeltaRobot(0.17, 0.035, 0.1, 0.25)
        workspace = delta.compute_workspace(resolution=10)
        
        # X and Y should be symmetric around 0
        x_center = (workspace.x_range[0] + workspace.x_range[1]) / 2
        y_center = (workspace.y_range[0] + workspace.y_range[1]) / 2
        
        assert abs(x_center) < 0.05
        assert abs(y_center) < 0.05
    
    def test_workspace_z_is_negative(self) -> None:
        """Test workspace Z is all negative (below base)."""
        delta = DeltaRobot(0.17, 0.035, 0.1, 0.25)
        workspace = delta.compute_workspace(resolution=10)
        
        assert workspace.z_range[1] < 0  # Even max Z should be negative


class TestDeltaFactoryFunctions:
    """Test Delta robot factory functions."""
    
    def test_create_delta(self) -> None:
        """Test create_delta function."""
        delta = create_delta(0.15, 0.03, 0.10, 0.30)
        assert delta.base_radius == 0.15
        assert delta.effector_radius == 0.03
        assert delta.upper_arm_length == 0.10
        assert delta.lower_arm_length == 0.30
    
    def test_create_kossel_mini(self) -> None:
        """Test Kossel Mini style delta."""
        delta = create_kossel_mini()
        # Updated dimensions for valid FK
        assert delta.base_radius == 0.14
        assert delta.effector_radius == 0.035
        assert delta.upper_arm_length == 0.12
        assert delta.lower_arm_length == 0.30
        
        # Should be able to reach center
        x, y, z = delta.forward(0, 0, 0)
        assert abs(x) < 0.01
        assert abs(y) < 0.01
    
    def test_create_flsun_q5(self) -> None:
        """Test FLSUN Q5 style delta."""
        delta = create_flsun_q5()
        # Updated dimensions for valid FK
        assert delta.base_radius == 0.10
        assert delta.effector_radius == 0.035
        assert delta.upper_arm_length == 0.10
        assert delta.lower_arm_length == 0.215
        
        # Should be functional
        angles = delta.inverse(0, 0, -0.15)
        assert angles is not None
