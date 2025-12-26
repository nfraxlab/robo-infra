"""
Benchmarks for sensor read operations.

Performance targets:
- Sensor read (simulated): <1ms
- IMU fusion update: >500 Hz (2ms target)
- Buffer operations: <1ms
"""

from __future__ import annotations

import pytest

from robo_infra.core import Limits, Reading, ReadingBuffer, SimulatedSensor, Vector3
from robo_infra.motion import MadgwickFilter, MahonyFilter
from robo_infra.motion.fusion import MadgwickConfig, MahonyConfig

from . import Benchmarker, format_time

# Performance targets (in seconds)
TARGET_SENSOR_READ = 0.001  # 1ms
TARGET_IMU_UPDATE = 0.002  # 2ms (500 Hz)
TARGET_BUFFER_OP = 0.001  # 1ms


@pytest.fixture
def benchmarker() -> Benchmarker:
    """Create a benchmarker instance."""
    return Benchmarker(warmup_iterations=10)


class TestSensorReadLatency:
    """Benchmark sensor read latency."""

    def test_simulated_sensor_read(self, benchmarker: Benchmarker) -> None:
        """Benchmark simulated sensor read latency."""
        sensor = SimulatedSensor(name="bench_sensor", limits=Limits(min=0, max=100))
        sensor.enable()

        result = benchmarker.run(
            "SimulatedSensor read",
            lambda: sensor.read(),
            iterations=10000,
            target=TARGET_SENSOR_READ,
        )

        print(f"\n{result}")
        print(f"   Max rate: {result.ops_per_sec:.0f} reads/sec")
        assert result.passed, f"Sensor read too slow: {format_time(result.mean_time)}"

    def test_multiple_sensor_reads(self, benchmarker: Benchmarker) -> None:
        """Benchmark reading from multiple sensors."""
        sensors = [
            SimulatedSensor(name=f"sensor_{i}", limits=Limits(min=0, max=100))
            for i in range(5)
        ]
        for s in sensors:
            s.enable()

        def read_all() -> list[float]:
            return [s.read() for s in sensors]

        result = benchmarker.run(
            "5 sensor reads",
            read_all,
            iterations=2000,
            target=TARGET_SENSOR_READ * 5,
        )

        print(f"\n{result}")
        assert result.passed, f"Multi-sensor read too slow: {format_time(result.mean_time)}"


class TestBufferOperations:
    """Benchmark sensor buffer operations."""

    def test_buffer_add(self, benchmarker: Benchmarker) -> None:
        """Benchmark buffer add operation."""
        buffer = ReadingBuffer(max_size=100)
        reading = Reading(value=42.0)

        result = benchmarker.run(
            "ReadingBuffer add",
            lambda: buffer.add(reading),
            iterations=10000,
            target=TARGET_BUFFER_OP,
        )

        print(f"\n{result}")
        assert result.passed, f"Buffer add too slow: {format_time(result.mean_time)}"

    def test_buffer_mean(self, benchmarker: Benchmarker) -> None:
        """Benchmark buffer mean calculation."""
        buffer = ReadingBuffer(max_size=100)
        for i in range(100):
            buffer.add(Reading(value=float(i)))

        result = benchmarker.run(
            "ReadingBuffer mean (100 values)",
            lambda: buffer.mean(),
            iterations=5000,
            target=TARGET_BUFFER_OP,
        )

        print(f"\n{result}")
        assert result.passed, f"Buffer mean too slow: {format_time(result.mean_time)}"

    def test_buffer_std_dev(self, benchmarker: Benchmarker) -> None:
        """Benchmark buffer standard deviation."""
        buffer = ReadingBuffer(max_size=100)
        for i in range(100):
            buffer.add(Reading(value=float(i)))

        result = benchmarker.run(
            "ReadingBuffer std_dev (100 values)",
            lambda: buffer.std_dev(),
            iterations=5000,
            target=TARGET_BUFFER_OP,
        )

        print(f"\n{result}")
        assert result.passed, f"Buffer std_dev too slow: {format_time(result.mean_time)}"


class TestIMUFusion:
    """Benchmark IMU fusion filter updates."""

    def test_madgwick_filter_update(self, benchmarker: Benchmarker) -> None:
        """Benchmark Madgwick filter update (gyro only)."""
        config = MadgwickConfig(sample_period=0.01, beta=0.1)  # 100 Hz
        filter_ = MadgwickFilter(config=config)
        gyro = Vector3(x=0.1, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        result = benchmarker.run(
            "Madgwick filter update",
            lambda: filter_.update(gyro, accel),
            iterations=5000,
            target=TARGET_IMU_UPDATE,
        )

        print(f"\n{result}")
        rate = result.ops_per_sec
        print(f"   Update rate: {rate:.0f} Hz")
        assert rate >= 500, f"Madgwick filter too slow: {rate:.0f} Hz < 500 Hz target"

    def test_madgwick_with_magnetometer(self, benchmarker: Benchmarker) -> None:
        """Benchmark Madgwick filter with magnetometer."""
        config = MadgwickConfig(sample_period=0.01, beta=0.1)  # 100 Hz
        filter_ = MadgwickFilter(config=config)
        gyro = Vector3(x=0.1, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)
        mag = Vector3(x=0.5, y=0.0, z=0.0)

        result = benchmarker.run(
            "Madgwick filter with mag",
            lambda: filter_.update(gyro, accel, mag),
            iterations=5000,
            target=TARGET_IMU_UPDATE,
        )

        print(f"\n{result}")
        rate = result.ops_per_sec
        print(f"   Update rate: {rate:.0f} Hz")
        assert rate >= 500, f"Madgwick+mag too slow: {rate:.0f} Hz < 500 Hz target"

    def test_mahony_filter_update(self, benchmarker: Benchmarker) -> None:
        """Benchmark Mahony filter update."""
        config = MahonyConfig(sample_period=0.01, kp=1.0, ki=0.0)  # 100 Hz
        filter_ = MahonyFilter(config=config)
        gyro = Vector3(x=0.1, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        result = benchmarker.run(
            "Mahony filter update",
            lambda: filter_.update(gyro, accel),
            iterations=5000,
            target=TARGET_IMU_UPDATE,
        )

        print(f"\n{result}")
        rate = result.ops_per_sec
        print(f"   Update rate: {rate:.0f} Hz")
        assert rate >= 500, f"Mahony filter too slow: {rate:.0f} Hz < 500 Hz target"


class TestFilterComparison:
    """Compare IMU filter performance."""

    def test_filter_comparison(self, benchmarker: Benchmarker) -> None:
        """Compare Madgwick and Mahony filter performance."""
        madgwick = MadgwickFilter(config=MadgwickConfig(sample_period=0.01, beta=0.1))
        mahony = MahonyFilter(config=MahonyConfig(sample_period=0.01, kp=1.0, ki=0.0))
        gyro = Vector3(x=0.1, y=0.0, z=0.0)
        accel = Vector3(x=0.0, y=0.0, z=9.81)

        print("\n--- IMU Filter Comparison ---")

        result_madgwick = benchmarker.run(
            "Madgwick",
            lambda: madgwick.update(gyro, accel),
            iterations=2000,
        )
        print(f"Madgwick: {format_time(result_madgwick.mean_time)} ({result_madgwick.ops_per_sec:.0f} Hz)")

        result_mahony = benchmarker.run(
            "Mahony",
            lambda: mahony.update(gyro, accel),
            iterations=2000,
        )
        print(f"Mahony: {format_time(result_mahony.mean_time)} ({result_mahony.ops_per_sec:.0f} Hz)")
