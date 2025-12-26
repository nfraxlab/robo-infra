"""Tests for resilience utilities (Phase 5.15.4-5.15.5).

Tests for retry, circuit breaker, and timeout utilities.
"""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock

import pytest

from robo_infra.core.exceptions import CommunicationError
from robo_infra.core.exceptions import TimeoutError as RoboTimeoutError
from robo_infra.utils.resilience import (
    CircuitBreaker,
    CircuitBreakerError,
    CircuitState,
    RetryConfig,
    RetryExhaustedError,
    create_driver_circuit_breaker,
    create_sensor_circuit_breaker,
    run_with_timeout,
    with_retry,
    with_timeout,
)


# =============================================================================
# Retry Tests
# =============================================================================


class TestRetryConfig:
    """Tests for RetryConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = RetryConfig()
        assert config.max_attempts == 3
        assert config.base_delay == 0.1
        assert config.max_delay == 60.0
        assert config.exponential_base == 2.0
        assert config.jitter == 0.1

    def test_calculate_delay_exponential(self) -> None:
        """Test exponential delay calculation."""
        config = RetryConfig(base_delay=1.0, jitter=0.0)

        assert config.calculate_delay(1) == 1.0  # 1.0 * 2^0
        assert config.calculate_delay(2) == 2.0  # 1.0 * 2^1
        assert config.calculate_delay(3) == 4.0  # 1.0 * 2^2
        assert config.calculate_delay(4) == 8.0  # 1.0 * 2^3

    def test_calculate_delay_max_cap(self) -> None:
        """Test delay is capped at max_delay."""
        config = RetryConfig(base_delay=1.0, max_delay=5.0, jitter=0.0)

        assert config.calculate_delay(10) == 5.0  # Would be 512, capped at 5


class TestWithRetry:
    """Tests for with_retry decorator."""

    @pytest.mark.asyncio
    async def test_retry_success_first_attempt(self) -> None:
        """Test successful function on first attempt."""
        call_count = 0

        @with_retry(max_attempts=3)
        async def success_fn() -> str:
            nonlocal call_count
            call_count += 1
            return "success"

        result = await success_fn()
        assert result == "success"
        assert call_count == 1

    @pytest.mark.asyncio
    async def test_retry_success_after_failures(self) -> None:
        """Test successful function after some failures."""
        call_count = 0

        @with_retry(max_attempts=3, base_delay=0.001)
        async def eventually_succeeds() -> str:
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise ValueError("Not yet")
            return "success"

        result = await eventually_succeeds()
        assert result == "success"
        assert call_count == 3

    @pytest.mark.asyncio
    async def test_retry_exhausted(self) -> None:
        """Test RetryExhaustedError when all attempts fail."""

        @with_retry(max_attempts=3, base_delay=0.001)
        async def always_fails() -> None:
            raise ValueError("Always fails")

        with pytest.raises(RetryExhaustedError) as exc_info:
            await always_fails()

        assert exc_info.value.attempts == 3
        assert isinstance(exc_info.value.last_exception, ValueError)

    @pytest.mark.asyncio
    async def test_retry_only_on_specified_exceptions(self) -> None:
        """Test retry only happens for specified exception types."""
        call_count = 0

        @with_retry(max_attempts=3, base_delay=0.001, retry_on=(ValueError,))
        async def fails_with_type_error() -> None:
            nonlocal call_count
            call_count += 1
            raise TypeError("Not retryable")

        with pytest.raises(TypeError):
            await fails_with_type_error()

        # Should only be called once since TypeError is not in retry_on
        assert call_count == 1

    @pytest.mark.asyncio
    async def test_retry_callback(self) -> None:
        """Test on_retry callback is called."""
        retries: list[tuple[int, Exception]] = []

        def on_retry(attempt: int, exc: Exception) -> None:
            retries.append((attempt, exc))

        @with_retry(max_attempts=3, base_delay=0.001, on_retry=on_retry)
        async def fails_twice() -> str:
            if len(retries) < 2:
                raise ValueError("Retry me")
            return "success"

        result = await fails_twice()
        assert result == "success"
        assert len(retries) == 2
        assert retries[0][0] == 1
        assert retries[1][0] == 2


# =============================================================================
# Circuit Breaker Tests
# =============================================================================


class TestCircuitBreaker:
    """Tests for CircuitBreaker."""

    @pytest.mark.asyncio
    async def test_circuit_starts_closed(self) -> None:
        """Test circuit starts in closed state."""
        breaker = CircuitBreaker("test")
        assert breaker.state == CircuitState.CLOSED

    @pytest.mark.asyncio
    async def test_circuit_opens_on_failures(self) -> None:
        """Test circuit opens after failure threshold."""
        breaker = CircuitBreaker("test", failure_threshold=3)

        for _ in range(3):
            try:
                async with breaker:
                    raise ValueError("Failure")
            except ValueError:
                pass

        assert breaker.state == CircuitState.OPEN

    @pytest.mark.asyncio
    async def test_circuit_rejects_when_open(self) -> None:
        """Test open circuit rejects calls."""
        breaker = CircuitBreaker("test", failure_threshold=1)

        # Trigger opening
        try:
            async with breaker:
                raise ValueError("Failure")
        except ValueError:
            pass

        assert breaker.state == CircuitState.OPEN

        with pytest.raises(CircuitBreakerError) as exc_info:
            async with breaker:
                pass

        assert exc_info.value.state == CircuitState.OPEN

    @pytest.mark.asyncio
    async def test_circuit_closes_on_success(self) -> None:
        """Test circuit closes after successful calls in half-open."""
        breaker = CircuitBreaker(
            "test",
            failure_threshold=1,
            recovery_timeout=0.01,
            success_threshold=1,
        )

        # Trigger opening
        try:
            async with breaker:
                raise ValueError("Failure")
        except ValueError:
            pass

        assert breaker.state == CircuitState.OPEN

        # Wait for recovery timeout
        await asyncio.sleep(0.02)

        # Successful call should close circuit
        async with breaker:
            pass

        assert breaker.state == CircuitState.CLOSED

    @pytest.mark.asyncio
    async def test_protect_decorator(self) -> None:
        """Test protect decorator works."""
        breaker = CircuitBreaker("test")
        mock_fn = AsyncMock(return_value="result")

        protected = breaker.protect(mock_fn)
        result = await protected()

        assert result == "result"
        mock_fn.assert_called_once()

    def test_reset(self) -> None:
        """Test manual reset."""
        breaker = CircuitBreaker("test")
        breaker._state = CircuitState.OPEN
        breaker._failure_count = 10

        breaker.reset()

        assert breaker.state == CircuitState.CLOSED
        assert breaker._failure_count == 0


class TestDriverCircuitBreaker:
    """Tests for create_driver_circuit_breaker."""

    def test_creates_breaker_with_correct_settings(self) -> None:
        """Test driver circuit breaker has appropriate settings."""
        breaker = create_driver_circuit_breaker("motor")

        assert breaker.name == "motor"
        assert breaker.failure_threshold == 5
        assert breaker.recovery_timeout == 30.0
        assert CommunicationError in breaker.failure_exceptions


class TestSensorCircuitBreaker:
    """Tests for create_sensor_circuit_breaker."""

    def test_creates_breaker_with_correct_settings(self) -> None:
        """Test sensor circuit breaker has appropriate settings."""
        breaker = create_sensor_circuit_breaker("temperature")

        assert breaker.name == "temperature"
        assert breaker.failure_threshold == 10
        assert breaker.recovery_timeout == 10.0


# =============================================================================
# Timeout Tests
# =============================================================================


class TestWithTimeout:
    """Tests for with_timeout context manager."""

    @pytest.mark.asyncio
    async def test_completes_within_timeout(self) -> None:
        """Test operation completes within timeout."""
        async with with_timeout(1.0, "fast_op"):
            await asyncio.sleep(0.01)

    @pytest.mark.asyncio
    async def test_raises_robo_timeout_error(self) -> None:
        """Test RoboTimeoutError is raised on timeout."""
        with pytest.raises(RoboTimeoutError) as exc_info:
            async with with_timeout(0.01, "slow_op"):
                await asyncio.sleep(1.0)

        assert exc_info.value.operation == "slow_op"
        assert exc_info.value.timeout == 0.01


class TestRunWithTimeout:
    """Tests for run_with_timeout function."""

    @pytest.mark.asyncio
    async def test_returns_result(self) -> None:
        """Test returns coroutine result."""

        async def get_value() -> str:
            return "result"

        result = await run_with_timeout(get_value(), 1.0, "get_value")
        assert result == "result"

    @pytest.mark.asyncio
    async def test_raises_robo_timeout_error(self) -> None:
        """Test RoboTimeoutError on timeout."""

        async def slow() -> None:
            await asyncio.sleep(1.0)

        with pytest.raises(RoboTimeoutError) as exc_info:
            await run_with_timeout(slow(), 0.01, "slow")

        assert exc_info.value.operation == "slow"


# =============================================================================
# Export Tests
# =============================================================================


class TestExports:
    """Tests for module exports."""

    def test_all_exports_importable(self) -> None:
        """Test all exports are importable from utils."""
        from robo_infra.utils import (
            create_driver_circuit_breaker,
            create_sensor_circuit_breaker,
            run_with_timeout,
            with_retry,
        )

        assert callable(with_retry)
        assert callable(create_driver_circuit_breaker)
        assert callable(create_sensor_circuit_breaker)
        assert callable(run_with_timeout)
