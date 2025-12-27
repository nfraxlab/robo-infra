"""Resilience utilities for robo-infra.

This module provides retry, circuit breaker, and timeout utilities
for building resilient robotics applications. It wraps svc-infra's
resilience utilities and adds robotics-specific patterns.

Example:
    >>> from robo_infra.utils.resilience import (
    ...     with_retry,
    ...     CircuitBreaker,
    ...     with_timeout,
    ... )
    >>>
    >>> # Retry on communication errors
    >>> @with_retry(max_attempts=3, retry_on=(CommunicationError,))
    ... async def read_sensor():
    ...     return await driver.read()
    >>>
    >>> # Circuit breaker for flaky hardware
    >>> breaker = CircuitBreaker("motor-driver", failure_threshold=5)
    >>> async with breaker:
    ...     await driver.send_command(cmd)
    >>>
    >>> # Timeout for operations
    >>> async with with_timeout(5.0, "move"):
    ...     await actuator.move(target)
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING


# Re-export from svc-infra (optional dependency)
try:
    from svc_infra.resilience import (
        CircuitBreaker,
        CircuitBreakerError,
        CircuitBreakerStats,
        CircuitState,
        RetryConfig,
        RetryExhaustedError,
        retry_sync,
        with_retry,
    )

    _SVC_INFRA_AVAILABLE = True
except ImportError:
    _SVC_INFRA_AVAILABLE = False
    # Create placeholders for type checking
    CircuitBreaker = None  # type: ignore[assignment, misc]
    CircuitBreakerError = None  # type: ignore[assignment, misc]
    CircuitBreakerStats = None  # type: ignore[assignment, misc]
    CircuitState = None  # type: ignore[assignment, misc]
    RetryConfig = None  # type: ignore[assignment, misc]
    RetryExhaustedError = None  # type: ignore[assignment, misc]
    retry_sync = None  # type: ignore[assignment]
    with_retry = None  # type: ignore[assignment]

from robo_infra.core.exceptions import TimeoutError as RoboTimeoutError


def _check_svc_infra_resilience() -> None:
    """Raise ImportError if svc-infra resilience is not available."""
    if not _SVC_INFRA_AVAILABLE:
        raise ImportError(
            "svc-infra is required for resilience utilities. "
            "Install with: pip install robo-infra[api]"
        )


if TYPE_CHECKING:
    from collections.abc import AsyncGenerator


# =============================================================================
# Timeout Context Manager
# =============================================================================


@asynccontextmanager
async def with_timeout(
    timeout: float,
    operation: str,
) -> AsyncGenerator[None, None]:
    """Async context manager for enforcing operation timeouts.

    Raises RoboTimeoutError (not asyncio.TimeoutError) to provide
    consistent exception handling across robo-infra.

    Args:
        timeout: Timeout in seconds.
        operation: Name of the operation (for error message).

    Raises:
        TimeoutError: If the operation exceeds the timeout.

    Example:
        >>> async with with_timeout(5.0, "move"):
        ...     await actuator.move(target)
        >>>
        >>> async with with_timeout(1.0, "sensor_read"):
        ...     value = await sensor.read()
    """
    try:
        async with asyncio.timeout(timeout):
            yield
    except TimeoutError as e:
        raise RoboTimeoutError(operation, timeout) from e


async def run_with_timeout(
    coro,  # Coroutine to run
    timeout: float,
    operation: str,
):
    """Run a coroutine with a timeout.

    Args:
        coro: The coroutine to execute.
        timeout: Timeout in seconds.
        operation: Name of the operation (for error message).

    Returns:
        The result of the coroutine.

    Raises:
        TimeoutError: If the operation exceeds the timeout.

    Example:
        >>> result = await run_with_timeout(
        ...     sensor.read(),
        ...     timeout=1.0,
        ...     operation="sensor_read",
        ... )
    """
    try:
        return await asyncio.wait_for(coro, timeout=timeout)
    except TimeoutError as e:
        raise RoboTimeoutError(operation, timeout) from e


# =============================================================================
# Robotics-Specific Circuit Breakers
# =============================================================================


def create_driver_circuit_breaker(
    name: str,
    *,
    failure_threshold: int = 5,
    recovery_timeout: float = 30.0,
) -> CircuitBreaker:
    """Create a circuit breaker configured for hardware driver communication.

    Uses conservative settings appropriate for hardware communication:
    - Lower failure threshold (hardware failures are more serious)
    - Longer recovery timeout (hardware may need time to reset)

    Args:
        name: Name for the circuit breaker (e.g., "i2c-motor").
        failure_threshold: Number of failures before opening.
        recovery_timeout: Seconds to wait before testing recovery.

    Returns:
        Configured CircuitBreaker instance.

    Example:
        >>> from robo_infra.utils.resilience import create_driver_circuit_breaker
        >>>
        >>> class I2CMotorDriver:
        ...     def __init__(self):
        ...         self._circuit = create_driver_circuit_breaker("i2c-motor")
        ...
        ...     async def send_command(self, cmd: bytes) -> bytes:
        ...         async with self._circuit:
        ...             return await self._i2c.transfer(cmd)
    """
    from robo_infra.core.exceptions import CommunicationError

    return CircuitBreaker(
        name=name,
        failure_threshold=failure_threshold,
        recovery_timeout=recovery_timeout,
        half_open_max_calls=2,  # Fewer test calls for hardware
        success_threshold=2,
        failure_exceptions=(CommunicationError, OSError, ConnectionError),
    )


def create_sensor_circuit_breaker(
    name: str,
    *,
    failure_threshold: int = 10,
    recovery_timeout: float = 10.0,
) -> CircuitBreaker:
    """Create a circuit breaker configured for sensor reading.

    Uses more tolerant settings since sensor glitches are common
    and non-critical sensors can be skipped temporarily.

    Args:
        name: Name for the circuit breaker (e.g., "temperature-sensor").
        failure_threshold: Number of failures before opening.
        recovery_timeout: Seconds to wait before testing recovery.

    Returns:
        Configured CircuitBreaker instance.

    Example:
        >>> circuit = create_sensor_circuit_breaker("temp-sensor")
        >>> async with circuit:
        ...     temp = await sensor.read()
    """
    from robo_infra.core.exceptions import CommunicationError

    return CircuitBreaker(
        name=name,
        failure_threshold=failure_threshold,
        recovery_timeout=recovery_timeout,
        half_open_max_calls=3,
        success_threshold=1,  # One success is enough to trust sensor again
        failure_exceptions=(CommunicationError, OSError, asyncio.TimeoutError),
    )


# =============================================================================
# Public API
# =============================================================================

__all__ = [
    # From svc-infra
    "CircuitBreaker",
    "CircuitBreakerError",
    "CircuitBreakerStats",
    "CircuitState",
    "RetryConfig",
    "RetryExhaustedError",
    # Robo-infra additions
    "create_driver_circuit_breaker",
    "create_sensor_circuit_breaker",
    "retry_sync",
    "run_with_timeout",
    "with_retry",
    "with_timeout",
]
