"""Utility functions and helpers.

This module provides internal utilities for robo-infra including:
- Resilience patterns (retry, circuit breaker, timeout)
- Degraded mode operation for graceful degradation

Example:
    >>> from robo_infra.utils.resilience import with_retry, CircuitBreaker
    >>> from robo_infra.utils.degraded import DegradedModeController
    >>>
    >>> @with_retry(max_attempts=3)
    ... async def read_sensor():
    ...     return await driver.read()
    >>>
    >>> degraded = DegradedModeController(controller)
"""

from robo_infra.utils.degraded import (
    DegradedComponent,
    DegradedModeController,
    DegradedModeStatus,
)
from robo_infra.utils.resilience import (
    CircuitBreaker,
    CircuitBreakerError,
    CircuitBreakerStats,
    CircuitState,
    RetryConfig,
    RetryExhaustedError,
    create_driver_circuit_breaker,
    create_sensor_circuit_breaker,
    retry_sync,
    run_with_timeout,
    with_retry,
    with_timeout,
)


__all__ = [
    # Resilience
    "CircuitBreaker",
    "CircuitBreakerError",
    "CircuitBreakerStats",
    "CircuitState",
    # Degraded mode
    "DegradedComponent",
    "DegradedModeController",
    "DegradedModeStatus",
    "RetryConfig",
    "RetryExhaustedError",
    "create_driver_circuit_breaker",
    "create_sensor_circuit_breaker",
    "retry_sync",
    "run_with_timeout",
    "with_retry",
    "with_timeout",
]
