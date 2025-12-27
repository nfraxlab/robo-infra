"""Tests for motion transforms module.

Tests for Transform and Rotation classes including:
- Euler angle conversions
- Quaternion conversions
- Axis-angle conversions
- Transform composition
- Transform interpolation
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robo_infra.motion.transforms import (
    EulerOrder,
    Rotation,
    Transform,
    rotation_x,
    rotation_y,
    rotation_z,
    translation,
)


# =============================================================================
# Rotation Tests
# =============================================================================


class TestRotationIdentity:
    """Tests for identity rotation."""

    def test_identity_matrix(self) -> None:
        """Test identity rotation has identity matrix."""
        rot = Rotation.identity()
        np.testing.assert_array_almost_equal(rot.matrix, np.eye(3))

    def test_identity_euler(self) -> None:
        """Test identity has zero Euler angles."""
        rot = Rotation.identity()
        euler = rot.as_euler(degrees=True)
        np.testing.assert_array_almost_equal(euler, [0, 0, 0], decimal=5)

    def test_identity_quaternion(self) -> None:
        """Test identity has [0, 0, 0, 1] quaternion."""
        rot = Rotation.identity()
        quat = rot.as_quaternion()  # [x, y, z, w]
        np.testing.assert_array_almost_equal(quat, [0, 0, 0, 1], decimal=5)

    def test_identity_axis_angle(self) -> None:
        """Test identity has zero rotation angle."""
        rot = Rotation.identity()
        _axis, angle = rot.as_axis_angle()
        assert angle == pytest.approx(0.0, abs=1e-6)


class TestRotationFromEuler:
    """Tests for Euler angle construction."""

    def test_euler_xyz_90_z(self) -> None:
        """Test 90 degree rotation around Z."""
        rot = Rotation.from_euler((0, 0, 90), EulerOrder.XYZ, degrees=True)

        # Apply to [1, 0, 0] should give [0, 1, 0]
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_euler_xyz_90_y(self) -> None:
        """Test 90 degree rotation around Y."""
        rot = Rotation.from_euler((0, 90, 0), EulerOrder.XYZ, degrees=True)

        # Apply to [0, 0, 1] should give [1, 0, 0]
        v = rot.apply(np.array([0, 0, 1]))
        np.testing.assert_array_almost_equal(v, [1, 0, 0], decimal=5)

    def test_euler_xyz_90_x(self) -> None:
        """Test 90 degree rotation around X."""
        rot = Rotation.from_euler((90, 0, 0), EulerOrder.XYZ, degrees=True)

        # Apply to [0, 1, 0] should give [0, 0, 1]
        v = rot.apply(np.array([0, 1, 0]))
        np.testing.assert_array_almost_equal(v, [0, 0, 1], decimal=5)

    def test_euler_radians(self) -> None:
        """Test Euler angles in radians."""
        rot = Rotation.from_euler((0, 0, math.pi / 2), EulerOrder.XYZ, degrees=False)
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_euler_zyx_order(self) -> None:
        """Test ZYX Euler order."""
        rot = Rotation.from_euler((90, 0, 0), EulerOrder.ZYX, degrees=True)
        # First rotation is around Z
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)


class TestRotationFromQuaternion:
    """Tests for quaternion construction."""

    def test_quaternion_identity(self) -> None:
        """Test identity quaternion."""
        rot = Rotation.from_quaternion((0, 0, 0, 1))
        np.testing.assert_array_almost_equal(rot.matrix, np.eye(3))

    def test_quaternion_90_z(self) -> None:
        """Test 90 degree rotation around Z from quaternion."""
        # 90° around Z: w=cos(45°), z=sin(45°)
        w = math.cos(math.radians(45))
        z = math.sin(math.radians(45))
        rot = Rotation.from_quaternion((0, 0, z, w))

        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_quaternion_scalar_first(self) -> None:
        """Test scalar-first quaternion convention."""
        w = math.cos(math.radians(45))
        z = math.sin(math.radians(45))
        rot = Rotation.from_quaternion((w, 0, 0, z), scalar_first=True)

        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_quaternion_roundtrip(self) -> None:
        """Test quaternion roundtrip conversion."""
        original = (0.1, 0.2, 0.3, 0.9)
        original_normalized = np.array(original) / np.linalg.norm(original)

        rot = Rotation.from_quaternion(tuple(original_normalized))
        result = rot.as_quaternion()

        # Quaternions q and -q represent same rotation
        dot = abs(np.dot(original_normalized, result))
        assert dot == pytest.approx(1.0, abs=1e-5)

    def test_quaternion_normalization(self) -> None:
        """Test quaternion is normalized."""
        rot = Rotation.from_quaternion((0.1, 0.2, 0.3, 0.9))  # Not normalized
        quat = rot.as_quaternion()
        norm = np.linalg.norm(quat)
        assert norm == pytest.approx(1.0, abs=1e-6)

    def test_quaternion_zero_norm_raises(self) -> None:
        """Test zero-norm quaternion raises error."""
        with pytest.raises(ValueError, match="zero norm"):
            Rotation.from_quaternion((0, 0, 0, 0))


class TestRotationFromAxisAngle:
    """Tests for axis-angle construction."""

    def test_axis_angle_z(self) -> None:
        """Test rotation around Z axis."""
        rot = Rotation.from_axis_angle((0, 0, 1), 90, degrees=True)
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_axis_angle_arbitrary(self) -> None:
        """Test rotation around arbitrary axis."""
        axis = (1, 1, 1)  # Will be normalized
        rot = Rotation.from_axis_angle(axis, 120, degrees=True)

        # 120° around (1,1,1) cycles [1,0,0] -> [0,1,0] -> [0,0,1] -> [1,0,0]
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_axis_angle_roundtrip(self) -> None:
        """Test axis-angle roundtrip conversion."""
        axis = np.array([1, 0, 0])
        angle = 60.0

        rot = Rotation.from_axis_angle(tuple(axis), angle, degrees=True)
        result_axis, result_angle = rot.as_axis_angle(degrees=True)

        # Check angle matches
        assert result_angle == pytest.approx(angle, abs=1e-4)
        # Check axis (might be flipped with angle)
        dot = abs(np.dot(axis, result_axis))
        assert dot == pytest.approx(1.0, abs=1e-5)

    def test_axis_angle_zero_length_raises(self) -> None:
        """Test zero-length axis raises error."""
        with pytest.raises(ValueError, match="zero length"):
            Rotation.from_axis_angle((0, 0, 0), 90, degrees=True)


class TestRotationFromRotvec:
    """Tests for rotation vector construction."""

    def test_rotvec_z(self) -> None:
        """Test rotation vector around Z."""
        rotvec = (0, 0, math.pi / 2)  # 90° around Z
        rot = Rotation.from_rotvec(rotvec)

        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_rotvec_zero(self) -> None:
        """Test zero rotation vector gives identity."""
        rot = Rotation.from_rotvec((0, 0, 0))
        np.testing.assert_array_almost_equal(rot.matrix, np.eye(3))

    def test_rotvec_roundtrip(self) -> None:
        """Test rotation vector roundtrip."""
        original = np.array([0.5, 0.3, 0.4])
        rot = Rotation.from_rotvec(tuple(original))
        result = rot.as_rotvec()
        np.testing.assert_array_almost_equal(result, original, decimal=5)


class TestRotationOperations:
    """Tests for rotation operations."""

    def test_inverse(self) -> None:
        """Test rotation inverse."""
        rot = Rotation.from_euler((30, 45, 60), EulerOrder.XYZ, degrees=True)
        rot_inv = rot.inverse()

        # R @ R^-1 = I
        combined = rot @ rot_inv
        np.testing.assert_array_almost_equal(combined.matrix, np.eye(3), decimal=5)

    def test_composition(self) -> None:
        """Test rotation composition."""
        r1 = Rotation.from_euler((90, 0, 0), EulerOrder.XYZ, degrees=True)  # 90° around X
        r2 = Rotation.from_euler((0, 90, 0), EulerOrder.XYZ, degrees=True)  # 90° around Y

        combined = r1 @ r2

        # Apply both rotations to a vector
        v = np.array([0, 0, 1])
        v_combined = combined.apply(v)
        v_sequential = r1.apply(r2.apply(v))

        np.testing.assert_array_almost_equal(v_combined, v_sequential, decimal=5)

    def test_apply_single_vector(self) -> None:
        """Test applying rotation to single vector."""
        rot = Rotation.from_euler((0, 0, 90), EulerOrder.XYZ, degrees=True)
        v = np.array([1, 0, 0])
        result = rot.apply(v)

        assert result.shape == (3,)
        np.testing.assert_array_almost_equal(result, [0, 1, 0], decimal=5)

    def test_apply_multiple_vectors(self) -> None:
        """Test applying rotation to multiple vectors."""
        rot = Rotation.from_euler((0, 0, 90), EulerOrder.XYZ, degrees=True)
        vectors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        result = rot.apply(vectors)

        assert result.shape == (3, 3)
        np.testing.assert_array_almost_equal(result[0], [0, 1, 0], decimal=5)
        np.testing.assert_array_almost_equal(result[1], [-1, 0, 0], decimal=5)
        np.testing.assert_array_almost_equal(result[2], [0, 0, 1], decimal=5)

    def test_repr(self) -> None:
        """Test string representation."""
        rot = Rotation.from_euler((30, 45, 60), EulerOrder.XYZ, degrees=True)
        repr_str = repr(rot)
        assert "Rotation" in repr_str


# =============================================================================
# Transform Tests
# =============================================================================


class TestTransformIdentity:
    """Tests for identity transform."""

    def test_identity(self) -> None:
        """Test identity transform."""
        t = Transform.identity()
        np.testing.assert_array_almost_equal(t.position, [0, 0, 0])
        np.testing.assert_array_almost_equal(t.rotation.matrix, np.eye(3))

    def test_identity_matrix(self) -> None:
        """Test identity as 4x4 matrix."""
        t = Transform.identity()
        np.testing.assert_array_almost_equal(t.as_matrix(), np.eye(4))


class TestTransformConstruction:
    """Tests for transform construction."""

    def test_from_euler(self) -> None:
        """Test construction from Euler angles."""
        t = Transform.from_euler(
            position=(1, 2, 3),
            angles=(0, 0, 90),
            degrees=True,
        )
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

        v = t.rotation.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)

    def test_from_quaternion(self) -> None:
        """Test construction from quaternion."""
        w = math.cos(math.radians(45))
        z = math.sin(math.radians(45))
        t = Transform.from_quaternion(
            position=(1, 2, 3),
            quaternion=(0, 0, z, w),
        )
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    def test_from_axis_angle(self) -> None:
        """Test construction from axis-angle."""
        t = Transform.from_axis_angle(
            position=(1, 2, 3),
            axis=(0, 0, 1),
            angle=90,
            degrees=True,
        )
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    def test_from_translation(self) -> None:
        """Test pure translation construction."""
        t = Transform.from_translation(1, 2, 3)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        np.testing.assert_array_almost_equal(t.rotation.matrix, np.eye(3))

    def test_from_rotation(self) -> None:
        """Test pure rotation construction."""
        rot = Rotation.from_euler((0, 0, 90), EulerOrder.XYZ, degrees=True)
        t = Transform.from_rotation(rot)
        np.testing.assert_array_almost_equal(t.position, [0, 0, 0])

    def test_from_matrix(self) -> None:
        """Test construction from 4x4 matrix."""
        matrix = np.array(
            [
                [0, -1, 0, 1],
                [1, 0, 0, 2],
                [0, 0, 1, 3],
                [0, 0, 0, 1],
            ],
            dtype=np.float64,
        )

        t = Transform.from_matrix(matrix)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    def test_from_matrix_invalid_shape(self) -> None:
        """Test invalid matrix shape raises error."""
        with pytest.raises(ValueError, match="4x4"):
            Transform.from_matrix(np.eye(3))


class TestTransformProperties:
    """Tests for transform properties."""

    def test_xyz_properties(self) -> None:
        """Test x, y, z property access."""
        t = Transform.from_translation(1.5, 2.5, 3.5)
        assert t.x == pytest.approx(1.5)
        assert t.y == pytest.approx(2.5)
        assert t.z == pytest.approx(3.5)

    def test_as_matrix_roundtrip(self) -> None:
        """Test matrix conversion roundtrip."""
        t1 = Transform.from_euler(
            position=(1, 2, 3),
            angles=(30, 45, 60),
            degrees=True,
        )
        matrix = t1.as_matrix()
        t2 = Transform.from_matrix(matrix)

        np.testing.assert_array_almost_equal(t1.position, t2.position)
        np.testing.assert_array_almost_equal(t1.rotation.matrix, t2.rotation.matrix)


class TestTransformOperations:
    """Tests for transform operations."""

    def test_inverse(self) -> None:
        """Test transform inverse."""
        t = Transform.from_euler(
            position=(1, 2, 3),
            angles=(30, 45, 60),
            degrees=True,
        )
        t_inv = t.inverse()

        # T @ T^-1 = I
        combined = t @ t_inv
        np.testing.assert_array_almost_equal(combined.position, [0, 0, 0], decimal=5)
        np.testing.assert_array_almost_equal(combined.rotation.matrix, np.eye(3), decimal=5)

    def test_composition(self) -> None:
        """Test transform composition."""
        t1 = Transform.from_translation(1, 0, 0)
        t2 = Transform.from_euler(
            position=(0, 0, 0),
            angles=(0, 0, 90),
            degrees=True,
        )

        # Rotate then translate
        combined = t2 @ t1

        # Point at origin should end up at rotated position
        p = combined.apply(np.array([0, 0, 0]))
        np.testing.assert_array_almost_equal(p, [0, 1, 0], decimal=5)

    def test_apply_single_point(self) -> None:
        """Test applying transform to single point."""
        t = Transform.from_euler(
            position=(1, 0, 0),
            angles=(0, 0, 90),
            degrees=True,
        )
        p = np.array([1, 0, 0])
        result = t.apply(p)

        # Rotate [1,0,0] -> [0,1,0], then translate by [1,0,0]
        np.testing.assert_array_almost_equal(result, [1, 1, 0], decimal=5)

    def test_distance_to(self) -> None:
        """Test distance between transforms."""
        t1 = Transform.from_translation(0, 0, 0)
        t2 = Transform.from_translation(3, 4, 0)

        assert t1.distance_to(t2) == pytest.approx(5.0)

    def test_angular_distance_to(self) -> None:
        """Test angular distance between transforms."""
        t1 = Transform.identity()
        t2 = Transform.from_euler(
            position=(0, 0, 0),
            angles=(0, 0, 90),
            degrees=True,
        )

        angle = t1.angular_distance_to(t2, degrees=True)
        assert angle == pytest.approx(90, abs=0.1)

    def test_interpolate_position(self) -> None:
        """Test position interpolation."""
        t1 = Transform.from_translation(0, 0, 0)
        t2 = Transform.from_translation(10, 10, 10)

        t_mid = t1.interpolate(t2, 0.5)
        np.testing.assert_array_almost_equal(t_mid.position, [5, 5, 5], decimal=5)

    def test_interpolate_rotation(self) -> None:
        """Test rotation interpolation (SLERP)."""
        t1 = Transform.identity()
        t2 = Transform.from_euler(
            position=(0, 0, 0),
            angles=(0, 0, 90),
            degrees=True,
        )

        t_mid = t1.interpolate(t2, 0.5)
        euler = t_mid.rotation.as_euler(degrees=True)

        # Middle should be 45 degrees
        assert euler[2] == pytest.approx(45, abs=1)

    def test_interpolate_endpoints(self) -> None:
        """Test interpolation at endpoints."""
        t1 = Transform.from_translation(1, 2, 3)
        t2 = Transform.from_translation(4, 5, 6)

        t_start = t1.interpolate(t2, 0.0)
        t_end = t1.interpolate(t2, 1.0)

        np.testing.assert_array_almost_equal(t_start.position, t1.position)
        np.testing.assert_array_almost_equal(t_end.position, t2.position)

    def test_equality(self) -> None:
        """Test transform equality."""
        t1 = Transform.from_translation(1, 2, 3)
        t2 = Transform.from_translation(1, 2, 3)
        t3 = Transform.from_translation(1, 2, 4)

        assert t1 == t2
        assert t1 != t3

    def test_repr(self) -> None:
        """Test string representation."""
        t = Transform.from_translation(1, 2, 3)
        repr_str = repr(t)
        assert "Transform" in repr_str
        assert "1.000" in repr_str


# =============================================================================
# Convenience Function Tests
# =============================================================================


class TestConvenienceFunctions:
    """Tests for convenience functions."""

    def test_translation_function(self) -> None:
        """Test translation() convenience function."""
        t = translation(1, 2, 3)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        np.testing.assert_array_almost_equal(t.rotation.matrix, np.eye(3))

    def test_rotation_x_function(self) -> None:
        """Test rotation_x() convenience function."""
        rot = rotation_x(90, degrees=True)
        v = rot.apply(np.array([0, 1, 0]))
        np.testing.assert_array_almost_equal(v, [0, 0, 1], decimal=5)

    def test_rotation_y_function(self) -> None:
        """Test rotation_y() convenience function."""
        rot = rotation_y(90, degrees=True)
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 0, -1], decimal=5)

    def test_rotation_z_function(self) -> None:
        """Test rotation_z() convenience function."""
        rot = rotation_z(90, degrees=True)
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, 1, 0], decimal=5)


# =============================================================================
# Edge Cases
# =============================================================================


class TestEdgeCases:
    """Tests for edge cases."""

    def test_very_small_rotation(self) -> None:
        """Test very small rotation angle."""
        rot = Rotation.from_axis_angle((0, 0, 1), 0.0001, degrees=True)
        _, angle = rot.as_axis_angle(degrees=True)
        assert angle == pytest.approx(0.0001, abs=1e-5)

    def test_180_degree_rotation(self) -> None:
        """Test 180 degree rotation."""
        rot = Rotation.from_axis_angle((0, 0, 1), 180, degrees=True)
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [-1, 0, 0], decimal=5)

    def test_large_euler_angles(self) -> None:
        """Test large Euler angles."""
        rot = Rotation.from_euler((720, 360, 180), EulerOrder.XYZ, degrees=True)
        # 720° around X = 0° rotation, 360° around Y = 0°, 180° around Z = 180°
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [-1, 0, 0], decimal=4)

    def test_negative_angles(self) -> None:
        """Test negative rotation angles."""
        rot = Rotation.from_euler((0, 0, -90), EulerOrder.XYZ, degrees=True)
        v = rot.apply(np.array([1, 0, 0]))
        np.testing.assert_array_almost_equal(v, [0, -1, 0], decimal=5)

    def test_position_validation(self) -> None:
        """Test position shape validation."""
        with pytest.raises(ValueError, match="must be"):
            Transform(position=np.array([1, 2]))

    def test_rotation_matrix_validation(self) -> None:
        """Test rotation matrix shape validation."""
        with pytest.raises(ValueError, match="3x3"):
            Rotation(matrix=np.eye(4))
