"""Integration tests for sensor fusion end-to-end scenarios.

These tests verify the complete sensor fusion pipeline:
1. IMU sensor data collection (simulated)
2. Sensor fusion filter processing (Madgwick/Mahony)
3. Orientation estimation output
4. Integration with controllers for feedback control

End-to-end tests without real hardware.
"""

from __future__ import annotations

import math
import time

import pytest

from robo_infra.core.sensor import SimulatedSensor, Unit
from robo_infra.core.types import Limits, Quaternion, Vector3
from robo_infra.motion.fusion import (
    MadgwickConfig,
    MadgwickFilter,
    MahonyConfig,
    MahonyFilter,
    Orientation,
    quaternion_to_euler,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def accelerometer() -> SimulatedSensor:
    """Create a simulated 3-axis accelerometer."""
    return SimulatedSensor(
        name="accelerometer",
        unit=Unit.METERS_PER_SECOND,  # Use m/s as proxy for m/s^2
        limits=Limits(min=-16.0, max=16.0, default=0.0),
        noise=0.1,
    )


@pytest.fixture
def gyroscope() -> SimulatedSensor:
    """Create a simulated 3-axis gyroscope."""
    return SimulatedSensor(
        name="gyroscope",
        unit=Unit.RADIANS_PER_SECOND,
        limits=Limits(min=-2000.0, max=2000.0, default=0.0),
        noise=0.01,
    )


@pytest.fixture
def magnetometer() -> SimulatedSensor:
    """Create a simulated 3-axis magnetometer."""
    return SimulatedSensor(
        name="magnetometer",
        unit=Unit.RAW,  # Use raw for magnetic field
        limits=Limits(min=-100.0, max=100.0, default=0.0),
        noise=0.5,
    )


@pytest.fixture
def madgwick_filter() -> MadgwickFilter:
    """Create a Madgwick sensor fusion filter."""
    return MadgwickFilter(config=MadgwickConfig(beta=0.1))


@pytest.fixture
def mahony_filter() -> MahonyFilter:
    """Create a Mahony sensor fusion filter."""
    return MahonyFilter(config=MahonyConfig(kp=0.5, ki=0.01))


# =============================================================================
# Test: IMU Data Through Fusion to Orientation
# =============================================================================


class TestIMUDataToOrientation:
    """Test complete IMU data flow through sensor fusion."""

    def test_create_imu_sensors(
        self,
        accelerometer: SimulatedSensor,
        gyroscope: SimulatedSensor,
    ) -> None:
        """Test creating accelerometer and gyroscope sensors."""
        assert accelerometer.name == "accelerometer"
        assert gyroscope.name == "gyroscope"
        assert accelerometer.unit == Unit.METERS_PER_SECOND
        assert gyroscope.unit == Unit.RADIANS_PER_SECOND

    def test_create_fusion_filter(self, madgwick_filter: MadgwickFilter) -> None:
        """Test creating a Madgwick fusion filter."""
        assert madgwick_filter is not None
        assert madgwick_filter.beta == 0.1

    def test_initial_orientation_identity(self, madgwick_filter: MadgwickFilter) -> None:
        """Test that initial orientation is identity quaternion."""
        q = madgwick_filter.quaternion
        # Identity quaternion: (1, 0, 0, 0)
        assert q.w == pytest.approx(1.0, abs=0.01)
        assert q.x == pytest.approx(0.0, abs=0.01)
        assert q.y == pytest.approx(0.0, abs=0.01)
        assert q.z == pytest.approx(0.0, abs=0.01)

    def test_update_with_static_imu_data(self, madgwick_filter: MadgwickFilter) -> None:
        """Test updating filter with static (gravity-only) IMU data."""
        # Static sensor: accelerometer measures gravity, gyro is zero
        accel = Vector3(x=0.0, y=0.0, z=9.81)  # Gravity pointing down
        gyro = Vector3(x=0.0, y=0.0, z=0.0)  # No rotation

        dt = 0.01  # 100 Hz
        for _ in range(100):
            madgwick_filter.update(gyro, accel, dt=dt)

        # Orientation should remain near identity (flat, level)
        euler = madgwick_filter.euler
        assert abs(euler.x) < 10  # Roll near 0
        assert abs(euler.y) < 10  # Pitch near 0

    def test_update_with_rotation(self, madgwick_filter: MadgwickFilter) -> None:
        """Test updating filter with gyroscope rotation."""
        # Simulate rotation around Z axis (yaw)
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        gyro = Vector3(x=0.0, y=0.0, z=45.0)  # 45 deg/s yaw rate (default is degrees)

        dt = 0.01
        for _ in range(100):  # 1 second of rotation
            madgwick_filter.update(gyro, accel, dt=dt)

        # Quaternion z component should have changed (rotation around z axis)
        q = madgwick_filter.quaternion
        assert abs(q.z) > 0.01  # Z rotation should be present

    def test_9dof_update_with_magnetometer(self, madgwick_filter: MadgwickFilter) -> None:
        """Test 9-DOF update with magnetometer data."""
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        gyro = Vector3(x=0.0, y=0.0, z=0.0)
        mag = Vector3(x=25.0, y=0.0, z=45.0)  # Earth's magnetic field

        dt = 0.01
        for _ in range(100):
            madgwick_filter.update(gyro, accel, mag=mag, dt=dt)

        # Should produce valid orientation
        q = madgwick_filter.quaternion
        norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        assert norm == pytest.approx(1.0, abs=0.01)

    def test_orientation_output_format(self, madgwick_filter: MadgwickFilter) -> None:
        """Test orientation output is properly formatted."""
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        gyro = Vector3(x=0.0, y=0.0, z=0.0)

        madgwick_filter.update(gyro, accel, dt=0.01)

        # Get orientation as Orientation object
        orientation = Orientation.from_quaternion(madgwick_filter.quaternion, dt=0.01)

        assert isinstance(orientation.quaternion, Quaternion)
        assert isinstance(orientation.euler, Vector3)
        assert orientation.dt == 0.01


class TestMahonyFilter:
    """Test Mahony filter for sensor fusion."""

    def test_create_mahony_filter(self, mahony_filter: MahonyFilter) -> None:
        """Test creating a Mahony filter."""
        assert mahony_filter.kp == 0.5
        assert mahony_filter.ki == 0.01

    def test_mahony_update(self, mahony_filter: MahonyFilter) -> None:
        """Test Mahony filter update."""
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        gyro = Vector3(x=0.0, y=0.0, z=0.0)

        mahony_filter.update(gyro, accel, dt=0.01)

        q = mahony_filter.quaternion
        norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        assert norm == pytest.approx(1.0, abs=0.01)

    def test_mahony_vs_madgwick_consistency(
        self, madgwick_filter: MadgwickFilter, mahony_filter: MahonyFilter
    ) -> None:
        """Test that both filters produce similar results for same input."""
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        gyro = Vector3(x=0.0, y=0.0, z=0.1)

        dt = 0.01
        for _ in range(200):
            madgwick_filter.update(gyro, accel, dt=dt)
            mahony_filter.update(gyro, accel, dt=dt)

        # Both should produce similar euler angles (within 20 degrees)
        mad_euler = madgwick_filter.euler
        mah_euler = mahony_filter.euler

        assert abs(mad_euler.x - mah_euler.x) < 20
        assert abs(mad_euler.y - mah_euler.y) < 20


class TestFusionPerformance:
    """Test sensor fusion performance characteristics."""

    def test_fusion_update_rate(self, madgwick_filter: MadgwickFilter) -> None:
        """Test that fusion can run at high update rates."""
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        gyro = Vector3(x=0.0, y=0.0, z=0.0)

        num_updates = 1000
        start = time.perf_counter()
        for _ in range(num_updates):
            madgwick_filter.update(gyro, accel, dt=0.001)
        elapsed = time.perf_counter() - start

        updates_per_sec = num_updates / elapsed
        print(f"\nFusion update rate: {updates_per_sec:.0f} Hz")

        # Should achieve at least 10kHz on modern hardware
        assert updates_per_sec > 5000

    def test_quaternion_normalization(self, madgwick_filter: MadgwickFilter) -> None:
        """Test that quaternion stays normalized after many updates."""
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        gyro = Vector3(x=0.1, y=0.2, z=0.3)  # Constant rotation

        dt = 0.01
        for _ in range(10000):
            madgwick_filter.update(gyro, accel, dt=dt)

        q = madgwick_filter.quaternion
        norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        assert norm == pytest.approx(1.0, abs=0.001)


class TestEulerAngleConversion:
    """Test quaternion to Euler angle conversion."""

    def test_identity_quaternion_to_euler(self) -> None:
        """Test identity quaternion produces zero Euler angles."""
        q = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        euler = quaternion_to_euler(q)

        assert euler.x == pytest.approx(0.0, abs=0.1)
        assert euler.y == pytest.approx(0.0, abs=0.1)
        assert euler.z == pytest.approx(0.0, abs=0.1)

    def test_90_degree_yaw(self) -> None:
        """Test 90-degree yaw rotation."""
        # Quaternion for 90-degree yaw: (cos(45°), 0, 0, sin(45°))
        angle = math.radians(90)
        q = Quaternion(
            w=math.cos(angle / 2),
            x=0.0,
            y=0.0,
            z=math.sin(angle / 2),
        )
        euler = quaternion_to_euler(q)

        assert euler.z == pytest.approx(90.0, abs=1.0)

    def test_euler_angles_range(self, madgwick_filter: MadgwickFilter) -> None:
        """Test that Euler angles stay within expected ranges."""
        # Apply various rotations
        rotations = [
            Vector3(x=0.5, y=0.0, z=0.0),
            Vector3(x=0.0, y=0.5, z=0.0),
            Vector3(x=0.0, y=0.0, z=0.5),
            Vector3(x=0.3, y=0.3, z=0.3),
        ]

        accel = Vector3(x=0.0, y=0.0, z=9.81)
        dt = 0.01

        for gyro in rotations:
            for _ in range(100):
                madgwick_filter.update(gyro, accel, dt=dt)

            euler = madgwick_filter.euler
            # Roll and yaw: -180 to 180
            # Pitch: -90 to 90
            assert -180 <= euler.x <= 180
            assert -90 <= euler.y <= 90
            assert -180 <= euler.z <= 180
