"""
Benchmark suite for robo-infra.

This package contains performance benchmarks for:
- Controller operations (creation, enable, disable, move)
- Sensor reads (latency, throughput)
- Driver operations (I2C, SPI, serial)
- Kinematics calculations (IK, FK, trajectory)

Usage:
    # Run all benchmarks
    pytest tests/benchmarks/ -v

    # Run specific benchmark
    pytest tests/benchmarks/bench_controller_operations.py -v

    # Run with timing output
    pytest tests/benchmarks/ -v --tb=short

Note:
    Benchmarks use simulation mode and don't require actual hardware.
    Performance targets are documented in .github/PLAN.md Phase 5.19.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from collections.abc import Callable


__all__ = [
    "BenchmarkResult",
    "Benchmarker",
    "benchmark",
    "format_time",
]


@dataclass
class BenchmarkResult:
    """Result of a benchmark run.

    Attributes:
        name: Name of the benchmark.
        iterations: Number of iterations run.
        total_time: Total time for all iterations in seconds.
        mean_time: Mean time per iteration in seconds.
        median_time: Median time per iteration in seconds.
        min_time: Minimum time per iteration in seconds.
        max_time: Maximum time per iteration in seconds.
        std_dev: Standard deviation of times in seconds.
        target: Optional target time in seconds.
        passed: Whether the benchmark met its target.
    """

    name: str
    iterations: int
    total_time: float
    mean_time: float
    median_time: float
    min_time: float
    max_time: float
    std_dev: float
    target: float | None = None
    passed: bool = True
    times: list[float] = field(default_factory=list, repr=False)

    @property
    def ops_per_sec(self) -> float:
        """Operations per second based on mean time."""
        return 1.0 / self.mean_time if self.mean_time > 0 else float("inf")

    def __str__(self) -> str:
        status = "[OK]" if self.passed else "[X]"
        target_str = f" (target: {format_time(self.target)})" if self.target else ""
        return (
            f"{status} {self.name}: {format_time(self.mean_time)} "
            f"(min={format_time(self.min_time)}, max={format_time(self.max_time)}, "
            f"std={format_time(self.std_dev)}){target_str}"
        )


def format_time(seconds: float | None) -> str:
    """Format a time value for display.

    Args:
        seconds: Time in seconds.

    Returns:
        Human-readable time string.
    """
    if seconds is None:
        return "N/A"
    if seconds >= 1.0:
        return f"{seconds:.3f}s"
    if seconds >= 0.001:
        return f"{seconds * 1000:.3f}ms"
    if seconds >= 0.000001:
        return f"{seconds * 1_000_000:.3f}µs"
    return f"{seconds * 1_000_000_000:.3f}ns"


class Benchmarker:
    """Simple benchmarking utility.

    Example:
        >>> benchmarker = Benchmarker()
        >>> result = benchmarker.run(
        ...     "my_operation",
        ...     lambda: my_function(),
        ...     iterations=1000,
        ...     target=0.001,  # 1ms target
        ... )
        >>> print(result)
    """

    def __init__(self, warmup_iterations: int = 10) -> None:
        """Initialize the benchmarker.

        Args:
            warmup_iterations: Number of warmup iterations before timing.
        """
        self.warmup_iterations = warmup_iterations
        self.results: list[BenchmarkResult] = []

    def run(
        self,
        name: str,
        func: Callable[[], object],
        *,
        iterations: int = 100,
        target: float | None = None,
        setup: Callable[[], object] | None = None,
        teardown: Callable[[], object] | None = None,
    ) -> BenchmarkResult:
        """Run a benchmark.

        Args:
            name: Name of the benchmark.
            func: Function to benchmark (called without arguments).
            iterations: Number of iterations to run.
            target: Optional target time in seconds.
            setup: Optional setup function called before each iteration.
            teardown: Optional teardown function called after each iteration.

        Returns:
            BenchmarkResult with timing statistics.
        """
        # Warmup
        for _ in range(self.warmup_iterations):
            if setup:
                setup()
            func()
            if teardown:
                teardown()

        # Actual benchmark
        times: list[float] = []
        for _ in range(iterations):
            if setup:
                setup()

            start = time.perf_counter()
            func()
            end = time.perf_counter()

            times.append(end - start)

            if teardown:
                teardown()

        # Calculate statistics
        total_time = sum(times)
        mean_time = statistics.mean(times)
        median_time = statistics.median(times)
        min_time = min(times)
        max_time = max(times)
        std_dev = statistics.stdev(times) if len(times) > 1 else 0.0

        # Check target
        passed = target is None or mean_time <= target

        result = BenchmarkResult(
            name=name,
            iterations=iterations,
            total_time=total_time,
            mean_time=mean_time,
            median_time=median_time,
            min_time=min_time,
            max_time=max_time,
            std_dev=std_dev,
            target=target,
            passed=passed,
            times=times,
        )

        self.results.append(result)
        return result

    async def run_async(
        self,
        name: str,
        func: Callable[[], object],
        *,
        iterations: int = 100,
        target: float | None = None,
    ) -> BenchmarkResult:
        """Run an async benchmark.

        Args:
            name: Name of the benchmark.
            func: Async function to benchmark.
            iterations: Number of iterations to run.
            target: Optional target time in seconds.

        Returns:
            BenchmarkResult with timing statistics.
        """
        import asyncio

        # Warmup
        for _ in range(self.warmup_iterations):
            result = func()
            if asyncio.iscoroutine(result):
                await result

        # Actual benchmark
        times: list[float] = []
        for _ in range(iterations):
            start = time.perf_counter()
            result = func()
            if asyncio.iscoroutine(result):
                await result
            end = time.perf_counter()
            times.append(end - start)

        # Calculate statistics
        total_time = sum(times)
        mean_time = statistics.mean(times)
        median_time = statistics.median(times)
        min_time = min(times)
        max_time = max(times)
        std_dev = statistics.stdev(times) if len(times) > 1 else 0.0

        passed = target is None or mean_time <= target

        result = BenchmarkResult(
            name=name,
            iterations=iterations,
            total_time=total_time,
            mean_time=mean_time,
            median_time=median_time,
            min_time=min_time,
            max_time=max_time,
            std_dev=std_dev,
            target=target,
            passed=passed,
            times=times,
        )

        self.results.append(result)
        return result

    def report(self) -> str:
        """Generate a report of all benchmark results.

        Returns:
            Formatted report string.
        """
        lines = ["=" * 60, "Benchmark Results", "=" * 60]

        passed = sum(1 for r in self.results if r.passed)
        failed = len(self.results) - passed

        for result in self.results:
            lines.append(str(result))

        lines.append("=" * 60)
        lines.append(f"Total: {len(self.results)} benchmarks, {passed} passed, {failed} failed")
        lines.append("=" * 60)

        return "\n".join(lines)


def benchmark(
    name: str,
    *,
    iterations: int = 100,
    target: float | None = None,
    warmup: int = 10,
) -> Callable[[Callable[[], object]], BenchmarkResult]:
    """Decorator to benchmark a function.

    Args:
        name: Name of the benchmark.
        iterations: Number of iterations.
        target: Optional target time in seconds.
        warmup: Number of warmup iterations.

    Returns:
        Decorator that runs the benchmark and returns the result.

    Example:
        >>> @benchmark("my_operation", iterations=1000, target=0.001)
        ... def bench_my_operation():
        ...     my_function()
    """

    def decorator(func: Callable[[], object]) -> BenchmarkResult:
        benchmarker = Benchmarker(warmup_iterations=warmup)
        return benchmarker.run(name, func, iterations=iterations, target=target)

    return decorator
