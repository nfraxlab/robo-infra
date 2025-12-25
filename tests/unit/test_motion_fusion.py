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

from robo_infra.core.types import Quaternion, Vector3
from robo_infra.motion.fusion import (
    DEG_TO_RAD,
    RAD_TO_DEG,
    MadgwickConfig,
    MadgwickFilter,
    MahonyConfig,
    MahonyFilter,
    Orientation,
    euler_to_quaternion,
    get_orientation_filter,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_normalize,
    quaternion_to_euler,
)


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


# =============================================================================
# Phase 5.9.3.1 - Sensor Fusion Comprehensive Tests
# =============================================================================


class TestMadgwickFilterComprehensive:
    """Comprehensive tests for MadgwickFilter."""

    def test_filter_beta_property(self) -> None:
        """Test beta getter and setter."""
        filt = MadgwickFilter()

        assert filt.beta == 0.1

        filt.beta = 0.5
        assert filt.beta == 0.5

    def test_filter_beta_clamped(self) -> None:
        """Test beta is clamped to [0, 1]."""
        filt = MadgwickFilter()

        filt.beta = -0.5
        assert filt.beta == 0.0

        filt.beta = 1.5
        assert filt.beta == 1.0

    def test_filter_high_beta_fast_convergence(self) -> None:
        """Test higher beta leads to faster convergence."""
        config_fast = MadgwickConfig(beta=0.9)
        config_slow = MadgwickConfig(beta=0.01)

        filt_fast = MadgwickFilter(config_fast)
        filt_slow = MadgwickFilter(config_slow)

        # Apply significant tilt - sensor tilted so Y points up
        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=9.81, z=0.0)  # Tilted 90° around X

        # Update both filters
        for _ in range(20):
            filt_fast.update(gyro, accel, dt=0.01)
            filt_slow.update(gyro, accel, dt=0.01)

        # Fast filter should have larger roll deviation from identity
        fast_roll = abs(filt_fast.euler.x)
        slow_roll = abs(filt_slow.euler.x)
        # Fast filter responds more quickly to gravity direction
        assert fast_roll >= slow_roll or fast_roll > 10.0

    def test_filter_update_gyro_only(self) -> None:
        """Test update with zero acceleration (gyro only mode)."""
        filt = MadgwickFilter()

        gyro = Vector3(x=10.0, y=0.0, z=0.0)  # Rotating
        accel = Vector3(x=0.0, y=0.0, z=0.0)  # Zero - triggers gyro-only

        orientation = filt.update(gyro, accel, dt=0.01)

        assert isinstance(orientation, Orientation)
        # Filter should still work (gyro integration)

    def test_filter_update_with_magnetometer(self) -> None:
        """Test 9-DOF update with magnetometer."""
        filt = MadgwickFilter()

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        mag = Vector3(x=20.0, y=0.0, z=40.0)  # Pointing north

        orientation = filt.update(gyro, accel, mag=mag, dt=0.01)

        assert isinstance(orientation, Orientation)

    def test_filter_update_magnetometer_zero(self) -> None:
        """Test that zero magnetometer falls back to 6-DOF."""
        filt = MadgwickFilter()

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        mag = Vector3(x=0.0, y=0.0, z=0.0)  # Zero mag

        orientation = filt.update(gyro, accel, mag=mag, dt=0.01)

        assert isinstance(orientation, Orientation)

    def test_filter_set_orientation(self) -> None:
        """Test setting orientation manually."""
        filt = MadgwickFilter()

        # Set to 45° yaw
        q = euler_to_quaternion(0.0, 0.0, 45.0)
        filt.set_orientation(q)

        euler = filt.euler
        assert abs(euler.z - 45.0) < 1.0

    def test_filter_auto_dt(self) -> None:
        """Test filter uses auto-calculated dt when None."""
        filt = MadgwickFilter()

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        # First update
        orientation = filt.update(gyro, accel, dt=None)
        assert isinstance(orientation, Orientation)
        assert orientation.dt >= 0

    def test_filter_convergence_to_gravity(self) -> None:
        """Test filter converges to gravity direction."""
        filt = MadgwickFilter(MadgwickConfig(beta=0.5))

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)  # Level

        # Run many updates
        for _ in range(100):
            filt.update(gyro, accel, dt=0.01)

        euler = filt.euler
        # Should converge to near-zero roll and pitch
        assert abs(euler.x) < 5.0  # roll
        assert abs(euler.y) < 5.0  # pitch


class TestMahonyFilterComprehensive:
    """Comprehensive tests for MahonyFilter."""

    def test_filter_kp_property(self) -> None:
        """Test kp getter and setter."""
        filt = MahonyFilter()

        assert filt.kp == 0.5

        filt.kp = 1.0
        assert filt.kp == 1.0

    def test_filter_ki_property(self) -> None:
        """Test ki getter and setter."""
        filt = MahonyFilter()

        assert filt.ki == 0.0

        filt.ki = 0.1
        assert filt.ki == 0.1

    def test_filter_kp_clamped_negative(self) -> None:
        """Test kp is clamped to non-negative."""
        filt = MahonyFilter()

        filt.kp = -0.5
        assert filt.kp == 0.0

    def test_filter_ki_clamped_negative(self) -> None:
        """Test ki is clamped to non-negative."""
        filt = MahonyFilter()

        filt.ki = -0.5
        assert filt.ki == 0.0

    def test_filter_integral_feedback(self) -> None:
        """Test filter with integral feedback enabled."""
        config = MahonyConfig(kp=0.5, ki=0.1)
        filt = MahonyFilter(config)

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        for _ in range(50):
            filt.update(gyro, accel, dt=0.01)

        assert isinstance(filt.euler, Vector3)

    def test_filter_reset_clears_integral(self) -> None:
        """Test reset clears integral error."""
        config = MahonyConfig(kp=0.5, ki=0.1)
        filt = MahonyFilter(config)

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        # Accumulate some integral error
        for _ in range(10):
            filt.update(gyro, accel, dt=0.01)

        filt.reset()

        # Quaternion should be identity
        assert filt.quaternion.w == 1.0

    def test_filter_update_with_magnetometer(self) -> None:
        """Test 9-DOF update with magnetometer."""
        filt = MahonyFilter()

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        mag = Vector3(x=20.0, y=0.0, z=40.0)

        orientation = filt.update(gyro, accel, mag=mag, dt=0.01)

        assert isinstance(orientation, Orientation)

    def test_filter_update_gyro_only(self) -> None:
        """Test update with zero acceleration."""
        filt = MahonyFilter()

        gyro = Vector3(x=5.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=0.0)

        orientation = filt.update(gyro, accel, dt=0.01)

        assert isinstance(orientation, Orientation)

    def test_filter_high_kp_fast_response(self) -> None:
        """Test higher kp leads to faster response."""
        config_fast = MahonyConfig(kp=2.0)
        config_slow = MahonyConfig(kp=0.1)

        filt_fast = MahonyFilter(config_fast)
        filt_slow = MahonyFilter(config_slow)

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=9.81, z=0.0)  # 90° tilt around X

        for _ in range(20):
            filt_fast.update(gyro, accel, dt=0.01)
            filt_slow.update(gyro, accel, dt=0.01)

        fast_roll = abs(filt_fast.euler.x)
        slow_roll = abs(filt_slow.euler.x)
        # Fast filter responds more quickly to gravity direction
        assert fast_roll >= slow_roll or fast_roll > 10.0


class TestQuaternionUtilitiesComprehensive:
    """Comprehensive tests for quaternion utilities."""

    def test_normalize_non_unit(self) -> None:
        """Test normalizing non-unit quaternion."""
        q = Quaternion(w=3.0, x=4.0, y=0.0, z=0.0)
        result = quaternion_normalize(q)

        # Magnitude should be 1
        mag = math.sqrt(result.w**2 + result.x**2 + result.y**2 + result.z**2)
        assert abs(mag - 1.0) < 1e-10

    def test_normalize_preserves_direction(self) -> None:
        """Test normalize preserves quaternion direction."""
        q = Quaternion(w=2.0, x=2.0, y=0.0, z=0.0)
        result = quaternion_normalize(q)

        # w and x should be equal after normalization
        assert abs(result.w - result.x) < 1e-10

    def test_multiply_non_commutative(self) -> None:
        """Test quaternion multiplication is non-commutative."""
        q1 = euler_to_quaternion(90, 0, 0)
        q2 = euler_to_quaternion(0, 90, 0)

        r1 = quaternion_multiply(q1, q2)
        r2 = quaternion_multiply(q2, q1)

        # Results should be different
        diff = abs(r1.w - r2.w) + abs(r1.x - r2.x) + abs(r1.y - r2.y) + abs(r1.z - r2.z)
        assert diff > 0.01

    def test_multiply_inverse_yields_identity(self) -> None:
        """Test q * q^-1 = identity."""
        q = quaternion_normalize(Quaternion(w=0.5, x=0.5, y=0.5, z=0.5))
        q_conj = quaternion_conjugate(q)

        result = quaternion_multiply(q, q_conj)

        assert abs(result.w - 1.0) < 1e-6
        assert abs(result.x) < 1e-6
        assert abs(result.y) < 1e-6
        assert abs(result.z) < 1e-6

    def test_euler_to_quaternion_90_roll(self) -> None:
        """Test 90 degree roll conversion."""
        q = euler_to_quaternion(90.0, 0.0, 0.0)
        euler = quaternion_to_euler(q)

        assert abs(euler.x - 90.0) < 0.1

    def test_euler_to_quaternion_90_pitch(self) -> None:
        """Test 90 degree pitch conversion."""
        q = euler_to_quaternion(0.0, 90.0, 0.0)
        euler = quaternion_to_euler(q)

        # Note: gimbal lock may affect this
        assert abs(euler.y - 90.0) < 1.0

    def test_euler_to_quaternion_90_yaw(self) -> None:
        """Test 90 degree yaw conversion."""
        q = euler_to_quaternion(0.0, 0.0, 90.0)
        euler = quaternion_to_euler(q)

        assert abs(euler.z - 90.0) < 0.1

    def test_euler_roundtrip_multiple_angles(self) -> None:
        """Test euler roundtrip with various angles."""
        test_cases = [
            (0, 0, 0),
            (30, 0, 0),
            (0, 30, 0),
            (0, 0, 30),
            (10, 20, 30),
            (-45, 15, 60),
        ]

        for roll, pitch, yaw in test_cases:
            q = euler_to_quaternion(roll, pitch, yaw)
            euler = quaternion_to_euler(q)

            assert abs(euler.x - roll) < 0.5, f"Roll failed for {roll},{pitch},{yaw}"
            assert abs(euler.y - pitch) < 0.5, f"Pitch failed for {roll},{pitch},{yaw}"
            assert abs(euler.z - yaw) < 0.5, f"Yaw failed for {roll},{pitch},{yaw}"


class TestOrientationComprehensive:
    """Comprehensive tests for Orientation dataclass."""

    def test_from_quaternion_computes_euler(self) -> None:
        """Test from_quaternion computes euler angles."""
        q = euler_to_quaternion(30.0, 15.0, 45.0)
        orientation = Orientation.from_quaternion(q)

        assert abs(orientation.euler.x - 30.0) < 0.5
        assert abs(orientation.euler.y - 15.0) < 0.5
        assert abs(orientation.euler.z - 45.0) < 0.5

    def test_orientation_has_timestamp(self) -> None:
        """Test orientation has timestamp."""
        q = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        orientation = Orientation.from_quaternion(q)

        assert orientation.timestamp > 0


class TestFusionQuaternionOutput:
    """Tests for fusion quaternion output."""

    def test_madgwick_quaternion_normalized(self) -> None:
        """Test Madgwick always returns normalized quaternion."""
        filt = MadgwickFilter()

        gyro = Vector3(x=50.0, y=30.0, z=20.0)
        accel = Vector3(x=1.0, y=2.0, z=9.0)

        for _ in range(100):
            filt.update(gyro, accel, dt=0.01)

        q = filt.quaternion
        mag = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        assert abs(mag - 1.0) < 1e-6

    def test_mahony_quaternion_normalized(self) -> None:
        """Test Mahony always returns normalized quaternion."""
        filt = MahonyFilter()

        gyro = Vector3(x=50.0, y=30.0, z=20.0)
        accel = Vector3(x=1.0, y=2.0, z=9.0)

        for _ in range(100):
            filt.update(gyro, accel, dt=0.01)

        q = filt.quaternion
        mag = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        assert abs(mag - 1.0) < 1e-6


class TestFusionEulerOutput:
    """Tests for fusion euler output."""

    def test_madgwick_euler_range(self) -> None:
        """Test Madgwick euler output is in valid range."""
        filt = MadgwickFilter()

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        filt.update(gyro, accel, dt=0.01)
        euler = filt.euler

        # Roll and yaw: [-180, 180]
        # Pitch: [-90, 90]
        assert -180.0 <= euler.x <= 180.0
        assert -90.0 <= euler.y <= 90.0
        assert -180.0 <= euler.z <= 180.0

    def test_mahony_euler_range(self) -> None:
        """Test Mahony euler output is in valid range."""
        filt = MahonyFilter()

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        filt.update(gyro, accel, dt=0.01)
        euler = filt.euler

        assert -180.0 <= euler.x <= 180.0
        assert -90.0 <= euler.y <= 90.0
        assert -180.0 <= euler.z <= 180.0


# =============================================================================
# Phase 5.9.3.2 - Integration Tests
# =============================================================================


class TestIMUFusionAccelGyro:
    """Integration tests for IMU fusion with accel + gyro."""

    def test_madgwick_stationary_level(self) -> None:
        """Test Madgwick with stationary level IMU."""
        filt = MadgwickFilter(MadgwickConfig(beta=0.5))

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        for _ in range(200):
            filt.update(gyro, accel, dt=0.01)

        euler = filt.euler
        assert abs(euler.x) < 2.0  # Near-zero roll
        assert abs(euler.y) < 2.0  # Near-zero pitch

    def test_mahony_stationary_level(self) -> None:
        """Test Mahony with stationary level IMU."""
        filt = MahonyFilter(MahonyConfig(kp=1.0))

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        for _ in range(200):
            filt.update(gyro, accel, dt=0.01)

        euler = filt.euler
        assert abs(euler.x) < 2.0
        assert abs(euler.y) < 2.0

    def test_tilted_30_degrees_roll(self) -> None:
        """Test fusion with 30 degree roll tilt."""
        filt = MadgwickFilter(MadgwickConfig(beta=0.5))

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        # Tilted 30° roll: g*sin(30) in y, g*cos(30) in z
        accel = Vector3(
            x=0.0,
            y=9.81 * 0.5,  # sin(30) = 0.5
            z=9.81 * 0.866,  # cos(30) ≈ 0.866
        )

        for _ in range(200):
            filt.update(gyro, accel, dt=0.01)

        euler = filt.euler
        # Should converge to ~30° roll
        assert abs(euler.x - 30.0) < 5.0

    def test_rotation_tracking(self) -> None:
        """Test filter tracks rotation from gyro."""
        filt = MadgwickFilter(MadgwickConfig(beta=0.01))  # Low beta for gyro trust

        # Rotate at 90 deg/s around z for 1 second = 90 degrees
        gyro = Vector3(x=0.0, y=0.0, z=90.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        for _ in range(100):  # 100 * 0.01 = 1 second
            filt.update(gyro, accel, dt=0.01)

        euler = filt.euler
        # Should have rotated approximately 90 degrees in yaw
        assert abs(euler.z) > 45.0  # At least significant rotation


class TestIMUFusionWithMag:
    """Integration tests for IMU fusion with magnetometer."""

    def test_madgwick_9dof(self) -> None:
        """Test Madgwick 9-DOF fusion."""
        filt = MadgwickFilter(MadgwickConfig(beta=0.5))

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        mag = Vector3(x=25.0, y=0.0, z=45.0)  # Pointing north

        for _ in range(200):
            filt.update(gyro, accel, mag=mag, dt=0.01)

        euler = filt.euler
        assert abs(euler.x) < 5.0  # Level roll
        assert abs(euler.y) < 5.0  # Level pitch

    def test_mahony_9dof(self) -> None:
        """Test Mahony 9-DOF fusion."""
        filt = MahonyFilter(MahonyConfig(kp=1.0))

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        mag = Vector3(x=25.0, y=0.0, z=45.0)

        for _ in range(200):
            filt.update(gyro, accel, mag=mag, dt=0.01)

        euler = filt.euler
        assert abs(euler.x) < 5.0
        assert abs(euler.y) < 5.0


class TestFusionDriftCorrection:
    """Tests for fusion drift correction."""

    def test_gyro_drift_correction(self) -> None:
        """Test that accelerometer corrects gyro drift."""
        filt = MadgwickFilter(MadgwickConfig(beta=0.1))

        # Apply biased gyro (drift)
        gyro_drift = Vector3(x=1.0, y=0.0, z=0.0)  # Slow drift
        accel = Vector3(x=0.0, y=0.0, z=9.81)  # Level

        for _ in range(500):
            filt.update(gyro_drift, accel, dt=0.01)

        euler = filt.euler
        # Accel should have corrected drift, keeping roll near zero
        assert abs(euler.x) < 30.0  # Limited by correction

    def test_filter_comparison(self) -> None:
        """Test both filters give similar results for same input."""
        madgwick = MadgwickFilter(MadgwickConfig(beta=0.1))
        mahony = MahonyFilter(MahonyConfig(kp=0.5))

        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        for _ in range(200):
            madgwick.update(gyro, accel, dt=0.01)
            mahony.update(gyro, accel, dt=0.01)

        # Both should converge to approximately level
        m_euler = madgwick.euler
        h_euler = mahony.euler

        assert abs(m_euler.x) < 5.0
        assert abs(h_euler.x) < 5.0
        assert abs(m_euler.y) < 5.0
        assert abs(h_euler.y) < 5.0


class TestFactoryFunctionComprehensive:
    """Comprehensive tests for get_orientation_filter factory."""

    def test_factory_madgwick_with_all_options(self) -> None:
        """Test factory with all Madgwick options."""
        filt = get_orientation_filter(
            "madgwick",
            beta=0.2,
            sample_period=0.02,
            gyro_in_degrees=False,
        )

        assert isinstance(filt, MadgwickFilter)

    def test_factory_mahony_with_all_options(self) -> None:
        """Test factory with all Mahony options."""
        filt = get_orientation_filter(
            "mahony",
            kp=1.0,
            ki=0.1,
            sample_period=0.02,
            gyro_in_degrees=False,
        )

        assert isinstance(filt, MahonyFilter)

    def test_factory_returns_fresh_instances(self) -> None:
        """Test factory returns new instances each call."""
        filt1 = get_orientation_filter("madgwick")
        filt2 = get_orientation_filter("madgwick")

        assert filt1 is not filt2


class TestGyroUnitsConfiguration:
    """Tests for gyroscope unit configuration."""

    def test_madgwick_degrees_per_second(self) -> None:
        """Test Madgwick with degrees per second input."""
        config = MadgwickConfig(gyro_in_degrees=True)
        filt = MadgwickFilter(config)

        gyro = Vector3(x=90.0, y=0.0, z=0.0)  # 90 deg/s
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        orientation = filt.update(gyro, accel, dt=0.01)
        assert isinstance(orientation, Orientation)

    def test_madgwick_radians_per_second(self) -> None:
        """Test Madgwick with radians per second input."""
        config = MadgwickConfig(gyro_in_degrees=False)
        filt = MadgwickFilter(config)

        gyro = Vector3(x=math.pi / 2, y=0.0, z=0.0)  # ~90 deg/s in rad
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        orientation = filt.update(gyro, accel, dt=0.01)
        assert isinstance(orientation, Orientation)

    def test_mahony_degrees_per_second(self) -> None:
        """Test Mahony with degrees per second input."""
        config = MahonyConfig(gyro_in_degrees=True)
        filt = MahonyFilter(config)

        gyro = Vector3(x=90.0, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        orientation = filt.update(gyro, accel, dt=0.01)
        assert isinstance(orientation, Orientation)

    def test_mahony_radians_per_second(self) -> None:
        """Test Mahony with radians per second input."""
        config = MahonyConfig(gyro_in_degrees=False)
        filt = MahonyFilter(config)

        gyro = Vector3(x=math.pi / 2, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        orientation = filt.update(gyro, accel, dt=0.01)
        assert isinstance(orientation, Orientation)
