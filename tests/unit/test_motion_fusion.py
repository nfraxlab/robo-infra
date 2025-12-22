"""Unit tests for sensor fusion algorithms.

Tests cover:
- Orientation data class
- Quaternion utilities
- Madgwick filter
- Mahony filter
- Factory function
"""

from __future__ import annotations

import math

import pytest

from robo_infra.motion.fusion import (
    DEG_TO_RAD,
    RAD_TO_DEG,
    MadgwickConfig,
    MadgwickFilter,
    MahonyConfig,
    MahonyFilter,
    Orientation,
    OrientationFilter,
    euler_to_quaternion,
    get_orientation_filter,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_normalize,
    quaternion_to_euler,
)
from robo_infra.core.types import Quaternion, Vector3


# =============================================================================
# Constants Tests
# =============================================================================


class TestConstants:
    """Tests for module constants."""

    def test_deg_to_rad(self) -> None:
        """Test degree to radian conversion constant."""
        assert abs(DEG_TO_RAD - math.pi / 180.0) < 1e-10

    def test_rad_to_deg(self) -> None:
        """Test radian to degree conversion constant."""
        assert abs(RAD_TO_DEG - 180.0 / math.pi) < 1e-10


# =============================================================================
# Orientation Tests
# =============================================================================


class TestOrientation:
    """Tests for Orientation dataclass."""

    def test_create_orientation(self) -> None:
        """Test creating an orientation."""
        q = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        euler = Vector3(x=0.0, y=0.0, z=0.0)
        orientation = Orientation(
            quaternion=q,
            euler=euler,
            timestamp=1234567890.0,
            dt=0.01,
        )

        assert orientation.quaternion.w == 1.0
        assert orientation.euler.x == 0.0
        assert orientation.dt == 0.01

    def test_from_quaternion(self) -> None:
        """Test creating orientation from quaternion."""
        q = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        orientation = Orientation.from_quaternion(q, dt=0.02)

        assert orientation.quaternion.w == 1.0
        assert orientation.dt == 0.02


# =============================================================================
# Quaternion Utility Tests
# =============================================================================


class TestQuaternionNormalize:
    """Tests for quaternion_normalize function."""

    def test_normalize_identity(self) -> None:
        """Test normalizing identity quaternion."""
        q = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        result = quaternion_normalize(q)

        assert abs(result.w - 1.0) < 1e-10
        assert abs(result.x) < 1e-10

    def test_normalize_scaled(self) -> None:
        """Test normalizing scaled quaternion."""
        q = Quaternion(w=2.0, x=0.0, y=0.0, z=0.0)
        result = quaternion_normalize(q)

        assert abs(result.w - 1.0) < 1e-10

    def test_normalize_zero(self) -> None:
        """Test normalizing zero quaternion returns identity."""
        q = Quaternion(w=0.0, x=0.0, y=0.0, z=0.0)
        result = quaternion_normalize(q)

        assert result.w == 1.0


class TestQuaternionMultiply:
    """Tests for quaternion_multiply function."""

    def test_multiply_identity(self) -> None:
        """Test multiplying by identity."""
        q1 = Quaternion(w=0.707, x=0.707, y=0.0, z=0.0)
        identity = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        result = quaternion_multiply(q1, identity)

        assert abs(result.w - q1.w) < 1e-10
        assert abs(result.x - q1.x) < 1e-10


class TestQuaternionConjugate:
    """Tests for quaternion_conjugate function."""

    def test_conjugate(self) -> None:
        """Test quaternion conjugate."""
        q = Quaternion(w=0.707, x=0.5, y=0.3, z=0.1)
        result = quaternion_conjugate(q)

        assert result.w == q.w
        assert result.x == -q.x
        assert result.y == -q.y
        assert result.z == -q.z


class TestQuaternionToEuler:
    """Tests for quaternion_to_euler function."""

    def test_identity_to_euler(self) -> None:
        """Test identity quaternion to euler."""
        q = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        euler = quaternion_to_euler(q)

        assert abs(euler.x) < 1e-6  # roll
        assert abs(euler.y) < 1e-6  # pitch
        assert abs(euler.z) < 1e-6  # yaw


class TestEulerToQuaternion:
    """Tests for euler_to_quaternion function."""

    def test_zero_angles(self) -> None:
        """Test zero euler angles."""
        q = euler_to_quaternion(0.0, 0.0, 0.0)

        assert abs(q.w - 1.0) < 1e-10
        assert abs(q.x) < 1e-10
        assert abs(q.y) < 1e-10
        assert abs(q.z) < 1e-10

    def test_roundtrip(self) -> None:
        """Test euler -> quaternion -> euler roundtrip."""
        roll, pitch, yaw = 10.0, 20.0, 30.0
        q = euler_to_quaternion(roll, pitch, yaw)
        euler = quaternion_to_euler(q)

        assert abs(euler.x - roll) < 0.1
        assert abs(euler.y - pitch) < 0.1
        assert abs(euler.z - yaw) < 0.1


# =============================================================================
# Madgwick Filter Tests
# =============================================================================


class TestMadgwickConfig:
    """Tests for MadgwickConfig."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = MadgwickConfig()

        assert config.beta == 0.1
        assert config.sample_period == 0.01
        assert config.gyro_in_degrees is True

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = MadgwickConfig(beta=0.2, sample_period=0.02)

        assert config.beta == 0.2
        assert config.sample_period == 0.02


class TestMadgwickFilter:
    """Tests for MadgwickFilter."""

    def test_create_filter(self) -> None:
        """Test creating a filter."""
        config = MadgwickConfig()
        filt = MadgwickFilter(config)

        assert filt is not None
        assert filt.quaternion.w == 1.0

    def test_filter_properties(self) -> None:
        """Test filter properties."""
        config = MadgwickConfig()
        filt = MadgwickFilter(config)

        assert isinstance(filt.quaternion, Quaternion)
        assert isinstance(filt.euler, Vector3)

    def test_filter_reset(self) -> None:
        """Test filter reset."""
        config = MadgwickConfig()
        filt = MadgwickFilter(config)
        filt.reset()

        assert filt.quaternion.w == 1.0

    def test_filter_update(self) -> None:
        """Test filter update."""
        config = MadgwickConfig()
        filt = MadgwickFilter(config)

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        orientation = filt.update(gyro=gyro, accel=accel, dt=0.01)

        assert isinstance(orientation, Orientation)


# =============================================================================
# Mahony Filter Tests
# =============================================================================


class TestMahonyConfig:
    """Tests for MahonyConfig."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = MahonyConfig()

        assert config.kp == 0.5
        assert config.ki == 0.0


class TestMahonyFilter:
    """Tests for MahonyFilter."""

    def test_create_filter(self) -> None:
        """Test creating a filter."""
        config = MahonyConfig()
        filt = MahonyFilter(config)

        assert filt is not None

    def test_filter_update(self) -> None:
        """Test filter update."""
        config = MahonyConfig()
        filt = MahonyFilter(config)

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        orientation = filt.update(gyro=gyro, accel=accel, dt=0.01)

        assert isinstance(orientation, Orientation)


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestGetOrientationFilter:
    """Tests for get_orientation_filter factory function."""

    def test_get_madgwick(self) -> None:
        """Test getting Madgwick filter."""
        filt = get_orientation_filter("madgwick")

        assert isinstance(filt, MadgwickFilter)

    def test_get_mahony(self) -> None:
        """Test getting Mahony filter."""
        filt = get_orientation_filter("mahony")

        assert isinstance(filt, MahonyFilter)

    def test_get_with_config(self) -> None:
        """Test getting filter with config."""
        filt = get_orientation_filter("madgwick", beta=0.2)

        assert isinstance(filt, MadgwickFilter)

    def test_unknown_algorithm(self) -> None:
        """Test unknown algorithm raises error."""
        with pytest.raises(ValueError, match="Unknown algorithm"):
            get_orientation_filter("unknown")

    def test_case_insensitive(self) -> None:
        """Test algorithm name is case insensitive."""
        filt = get_orientation_filter("MADGWICK")
        assert isinstance(filt, MadgwickFilter)

        filt = get_orientation_filter("Mahony")
        assert isinstance(filt, MahonyFilter)
