"""
Benchmarks for controller operations.

Performance targets:
- Controller creation: <10ms
- Actuator set: <1ms
- Sensor read: <1ms
- Enable/disable cycle: <5ms
"""

from __future__ import annotations

import pytest

from robo_infra.core import (
    Limits,
    SimulatedActuator,
    SimulatedController,
    SimulatedSensor,
)

from . import Benchmarker, format_time


# Performance targets (in seconds)
TARGET_CREATE = 0.010  # 10ms
TARGET_OPERATION = 0.001  # 1ms
TARGET_ENABLE_DISABLE = 0.005  # 5ms


@pytest.fixture
def benchmarker() -> Benchmarker:
    """Create a benchmarker instance."""
    return Benchmarker(warmup_iterations=10)


class TestControllerCreation:
    """Benchmark controller creation performance."""

    def test_controller_creation(self, benchmarker: Benchmarker) -> None:
        """Benchmark SimulatedController creation."""
        result = benchmarker.run(
            "SimulatedController creation",
            lambda: SimulatedController(name="bench"),
            iterations=1000,
            target=TARGET_CREATE,
        )

        print(f"\n{result}")
        assert result.passed, f"Controller creation too slow: {format_time(result.mean_time)}"

    def test_controller_with_actuators(self, benchmarker: Benchmarker) -> None:
        """Benchmark controller creation with multiple actuators."""

        def create_with_actuators() -> SimulatedController:
            controller = SimulatedController(name="bench")
            for i in range(6):
                actuator = SimulatedActuator(
                    name=f"joint_{i}",
                    limits=Limits(min=-180, max=180),
                )
                controller.add_actuator(f"joint_{i}", actuator)
            return controller

        result = benchmarker.run(
            "Controller with 6 actuators",
            create_with_actuators,
            iterations=500,
            target=TARGET_CREATE * 2,  # Allow more time for 6 actuators
        )

        print(f"\n{result}")
        assert result.passed, (
            f"Controller+actuators creation too slow: {format_time(result.mean_time)}"
        )

    def test_controller_with_sensors(self, benchmarker: Benchmarker) -> None:
        """Benchmark controller creation with multiple sensors."""

        def create_with_sensors() -> SimulatedController:
            controller = SimulatedController(name="bench")
            for i in range(4):
                sensor = SimulatedSensor(
                    name=f"sensor_{i}",
                    limits=Limits(min=0, max=100),
                )
                controller.add_sensor(f"sensor_{i}", sensor)
            return controller

        result = benchmarker.run(
            "Controller with 4 sensors",
            create_with_sensors,
            iterations=500,
            target=TARGET_CREATE * 2,
        )

        print(f"\n{result}")
        assert result.passed, (
            f"Controller+sensors creation too slow: {format_time(result.mean_time)}"
        )


class TestControllerEnableDisable:
    """Benchmark controller enable/disable operations."""

    def test_enable_disable_cycle(self, benchmarker: Benchmarker) -> None:
        """Benchmark controller enable/disable cycle."""
        controller = SimulatedController(name="bench")
        actuator = SimulatedActuator(name="a1", limits=Limits(min=0, max=180))
        controller.add_actuator("a1", actuator)

        def enable_disable() -> None:
            controller.enable()
            controller.disable()

        result = benchmarker.run(
            "Enable/disable cycle",
            enable_disable,
            iterations=1000,
            target=TARGET_ENABLE_DISABLE * 2,
        )

        print(f"\n{result}")
        assert result.passed, f"Enable/disable too slow: {format_time(result.mean_time)}"

    def test_actuator_enable_disable(self, benchmarker: Benchmarker) -> None:
        """Benchmark actuator enable/disable cycle."""
        actuator = SimulatedActuator(name="a1", limits=Limits(min=0, max=180))

        def enable_disable() -> None:
            actuator.enable()
            actuator.disable()

        result = benchmarker.run(
            "Actuator enable/disable",
            enable_disable,
            iterations=2000,
            target=TARGET_ENABLE_DISABLE,
        )

        print(f"\n{result}")
        assert result.passed, f"Actuator enable/disable too slow: {format_time(result.mean_time)}"


class TestActuatorOperations:
    """Benchmark actuator operations."""

    def test_actuator_set(self, benchmarker: Benchmarker) -> None:
        """Benchmark actuator set operation."""
        actuator = SimulatedActuator(name="a1", limits=Limits(min=0, max=180))
        actuator.enable()

        positions = [0.0, 45.0, 90.0, 135.0, 180.0]
        idx = 0

        def set_position() -> None:
            nonlocal idx
            actuator.set(positions[idx % len(positions)])
            idx += 1

        result = benchmarker.run(
            "Actuator set",
            set_position,
            iterations=5000,
            target=TARGET_OPERATION,
        )

        print(f"\n{result}")
        assert result.passed, f"Actuator set too slow: {format_time(result.mean_time)}"

    def test_actuator_get(self, benchmarker: Benchmarker) -> None:
        """Benchmark actuator get operation."""
        actuator = SimulatedActuator(name="a1", limits=Limits(min=0, max=180))
        actuator.enable()
        actuator.set(90.0)

        result = benchmarker.run(
            "Actuator get",
            lambda: actuator.get(),
            iterations=5000,
            target=TARGET_OPERATION,
        )

        print(f"\n{result}")
        assert result.passed, f"Actuator get too slow: {format_time(result.mean_time)}"

    def test_multi_actuator_set(self, benchmarker: Benchmarker) -> None:
        """Benchmark setting multiple actuators."""
        actuators = [
            SimulatedActuator(name=f"a{i}", limits=Limits(min=0, max=180)) for i in range(6)
        ]
        for a in actuators:
            a.enable()

        positions = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0]

        def set_all() -> None:
            for a, p in zip(actuators, positions, strict=True):
                a.set(p)

        result = benchmarker.run(
            "Set 6 actuators",
            set_all,
            iterations=1000,
            target=TARGET_OPERATION * 6,
        )

        print(f"\n{result}")
        assert result.passed, f"Multi-actuator set too slow: {format_time(result.mean_time)}"


class TestSensorOperations:
    """Benchmark sensor operations."""

    def test_sensor_read(self, benchmarker: Benchmarker) -> None:
        """Benchmark sensor read operation."""
        sensor = SimulatedSensor(name="s1", limits=Limits(min=0, max=100))
        sensor.enable()

        result = benchmarker.run(
            "Sensor read",
            lambda: sensor.read(),
            iterations=5000,
            target=TARGET_OPERATION,
        )

        print(f"\n{result}")
        print(f"   Max rate: {result.ops_per_sec:.0f} reads/sec")
        assert result.passed, f"Sensor read too slow: {format_time(result.mean_time)}"

    def test_multi_sensor_read(self, benchmarker: Benchmarker) -> None:
        """Benchmark reading multiple sensors."""
        sensors = [SimulatedSensor(name=f"s{i}", limits=Limits(min=0, max=100)) for i in range(4)]
        for s in sensors:
            s.enable()

        def read_all() -> list[float]:
            return [s.read() for s in sensors]

        result = benchmarker.run(
            "Read 4 sensors",
            read_all,
            iterations=1000,
            target=TARGET_OPERATION * 4,
        )

        print(f"\n{result}")
        assert result.passed, f"Multi-sensor read too slow: {format_time(result.mean_time)}"


class TestControllerLifecycle:
    """Benchmark full controller lifecycle."""

    def test_full_lifecycle(self, benchmarker: Benchmarker) -> None:
        """Benchmark complete controller lifecycle."""

        def lifecycle() -> None:
            # Create
            controller = SimulatedController(name="bench")
            actuator = SimulatedActuator(name="a1", limits=Limits(min=0, max=180))
            sensor = SimulatedSensor(name="s1", limits=Limits(min=0, max=100))
            controller.add_actuator("a1", actuator)
            controller.add_sensor("s1", sensor)

            # Enable
            controller.enable()

            # Operate
            actuator.set(90.0)
            _ = sensor.read()
            _ = actuator.get()

            # Disable
            controller.disable()

        result = benchmarker.run(
            "Full lifecycle (create/enable/operate/disable)",
            lifecycle,
            iterations=200,
            target=0.050,  # 50ms for full cycle
        )

        print(f"\n{result}")
        assert result.passed, f"Full lifecycle too slow: {format_time(result.mean_time)}"


class TestComponentComparison:
    """Compare component creation performance."""

    def test_component_comparison(self, benchmarker: Benchmarker) -> None:
        """Compare creation times for different component types."""
        components = [
            ("SimulatedController", lambda: SimulatedController(name="c")),
            (
                "SimulatedActuator",
                lambda: SimulatedActuator(name="a", limits=Limits(min=0, max=180)),
            ),
            ("SimulatedSensor", lambda: SimulatedSensor(name="s", limits=Limits(min=0, max=100))),
        ]

        print("\n--- Component Creation Comparison ---")
        for name, create in components:
            result = benchmarker.run(name, create, iterations=1000)
            print(f"{name}: {format_time(result.mean_time)}")
