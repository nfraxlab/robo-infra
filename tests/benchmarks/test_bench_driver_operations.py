"""
Benchmarks for driver operations.

Performance targets:
- Driver creation: <10ms
- I2C byte read: <1ms (simulated)
- SPI transfer: <1ms (simulated)
"""

from __future__ import annotations

import pytest

from robo_infra.core import (
    Limits,
    SimulatedActuator,
    SimulatedController,
    SimulatedI2CBus,
    SimulatedSensor,
    SimulatedSPIBus,
)

from . import Benchmarker, format_time

# Performance targets (in seconds)
TARGET_DRIVER_CREATE = 0.010  # 10ms
TARGET_BUS_OP = 0.001  # 1ms
TARGET_ENABLE_DISABLE = 0.005  # 5ms


@pytest.fixture
def benchmarker() -> Benchmarker:
    """Create a benchmarker instance."""
    return Benchmarker(warmup_iterations=10)


class TestDriverCreation:
    """Benchmark driver creation performance."""

    def test_controller_creation(self, benchmarker: Benchmarker) -> None:
        """Benchmark controller creation."""
        result = benchmarker.run(
            "SimulatedController creation",
            lambda: SimulatedController(name="bench_controller"),
            iterations=1000,
            target=TARGET_DRIVER_CREATE,
        )

        print(f"\n{result}")
        assert result.passed, f"Controller creation too slow: {format_time(result.mean_time)}"

    def test_actuator_creation(self, benchmarker: Benchmarker) -> None:
        """Benchmark actuator creation."""
        result = benchmarker.run(
            "SimulatedActuator creation",
            lambda: SimulatedActuator(name="bench_actuator", limits=Limits(min=0, max=180)),
            iterations=1000,
            target=TARGET_DRIVER_CREATE,
        )

        print(f"\n{result}")
        assert result.passed, f"Actuator creation too slow: {format_time(result.mean_time)}"

    def test_sensor_creation(self, benchmarker: Benchmarker) -> None:
        """Benchmark sensor creation."""
        result = benchmarker.run(
            "SimulatedSensor creation",
            lambda: SimulatedSensor(name="bench_sensor", limits=Limits(min=0, max=100)),
            iterations=1000,
            target=TARGET_DRIVER_CREATE,
        )

        print(f"\n{result}")
        assert result.passed, f"Sensor creation too slow: {format_time(result.mean_time)}"


class TestI2COperations:
    """Benchmark I2C bus operations."""

    def test_i2c_bus_creation(self, benchmarker: Benchmarker) -> None:
        """Benchmark I2C bus creation."""
        result = benchmarker.run(
            "SimulatedI2CBus creation",
            lambda: SimulatedI2CBus(),
            iterations=1000,
            target=TARGET_DRIVER_CREATE,
        )

        print(f"\n{result}")
        assert result.passed, f"I2C bus creation too slow: {format_time(result.mean_time)}"

    def test_i2c_read_byte(self, benchmarker: Benchmarker) -> None:
        """Benchmark I2C byte read."""
        bus = SimulatedI2CBus()
        bus.open()

        result = benchmarker.run(
            "I2C read_byte",
            lambda: bus.read_byte(0x50),
            iterations=5000,
            target=TARGET_BUS_OP,
        )

        bus.close()
        print(f"\n{result}")
        assert result.passed, f"I2C read_byte too slow: {format_time(result.mean_time)}"

    def test_i2c_write_byte(self, benchmarker: Benchmarker) -> None:
        """Benchmark I2C byte write."""
        bus = SimulatedI2CBus()
        bus.open()

        result = benchmarker.run(
            "I2C write_byte",
            lambda: bus.write_byte(0x50, 0xAB),
            iterations=5000,
            target=TARGET_BUS_OP,
        )

        bus.close()
        print(f"\n{result}")
        assert result.passed, f"I2C write_byte too slow: {format_time(result.mean_time)}"


class TestSPIOperations:
    """Benchmark SPI operations."""

    def test_spi_bus_creation(self, benchmarker: Benchmarker) -> None:
        """Benchmark SPI bus creation."""
        result = benchmarker.run(
            "SimulatedSPIBus creation",
            lambda: SimulatedSPIBus(),
            iterations=1000,
            target=TARGET_DRIVER_CREATE,
        )

        print(f"\n{result}")
        assert result.passed, f"SPI bus creation too slow: {format_time(result.mean_time)}"

    def test_spi_transfer(self, benchmarker: Benchmarker) -> None:
        """Benchmark SPI transfer."""
        bus = SimulatedSPIBus()
        bus.open()
        data = bytes([0x01, 0x02, 0x03, 0x04])

        result = benchmarker.run(
            "SPI transfer (4 bytes)",
            lambda: bus.transfer(data),
            iterations=5000,
            target=TARGET_BUS_OP,
        )

        bus.close()
        print(f"\n{result}")
        assert result.passed, f"SPI transfer too slow: {format_time(result.mean_time)}"


class TestControllerOperations:
    """Benchmark controller operations."""

    def test_controller_enable_disable(self, benchmarker: Benchmarker) -> None:
        """Benchmark controller enable/disable cycle."""
        controller = SimulatedController(name="bench")
        actuator = SimulatedActuator(name="a1", limits=Limits(min=0, max=180))
        controller.add_actuator("a1", actuator)

        def enable_disable() -> None:
            controller.enable()
            controller.disable()

        result = benchmarker.run(
            "Controller enable/disable cycle",
            enable_disable,
            iterations=1000,
            target=TARGET_ENABLE_DISABLE,
        )

        print(f"\n{result}")
        assert result.passed, f"Enable/disable too slow: {format_time(result.mean_time)}"

    def test_actuator_set_position(self, benchmarker: Benchmarker) -> None:
        """Benchmark actuator position setting."""
        actuator = SimulatedActuator(name="a1", limits=Limits(min=0, max=180))
        actuator.enable()

        result = benchmarker.run(
            "Actuator set",
            lambda: actuator.set(90.0),
            iterations=5000,
            target=TARGET_BUS_OP,
        )

        print(f"\n{result}")
        assert result.passed, f"set too slow: {format_time(result.mean_time)}"

    def test_sensor_read(self, benchmarker: Benchmarker) -> None:
        """Benchmark sensor reading."""
        sensor = SimulatedSensor(name="s1", limits=Limits(min=0, max=100))
        sensor.enable()

        result = benchmarker.run(
            "Sensor read",
            lambda: sensor.read(),
            iterations=5000,
            target=TARGET_BUS_OP,
        )

        print(f"\n{result}")
        assert result.passed, f"Sensor read too slow: {format_time(result.mean_time)}"


class TestDriverComparison:
    """Compare driver performance."""

    def test_driver_type_comparison(self, benchmarker: Benchmarker) -> None:
        """Compare creation times for different driver types."""
        drivers = [
            ("SimulatedController", lambda: SimulatedController(name="c")),
            ("SimulatedActuator", lambda: SimulatedActuator(name="a", limits=Limits(min=0, max=180))),
            ("SimulatedSensor", lambda: SimulatedSensor(name="s", limits=Limits(min=0, max=100))),
            ("SimulatedI2CBus", lambda: SimulatedI2CBus()),
            ("SimulatedSPIBus", lambda: SimulatedSPIBus()),
        ]

        print("\n--- Driver Creation Comparison ---")
        for name, create in drivers:
            result = benchmarker.run(name, create, iterations=500)
            print(f"{name}: {format_time(result.mean_time)}")
