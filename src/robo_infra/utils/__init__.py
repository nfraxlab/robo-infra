"""Utility functions and helpers.

This module provides internal utilities for robo-infra including:
- Resilience patterns (retry, circuit breaker, timeout)
- Degraded mode operation for graceful degradation
- Resource management (connection pooling, cleanup handlers)
- Hardware abstraction (simulation, detection, health monitoring)

Example:
    >>> from robo_infra.utils.resilience import with_retry, CircuitBreaker
    >>> from robo_infra.utils.degraded import DegradedModeController
    >>> from robo_infra.utils.resources import ConnectionPool, register_cleanup
    >>> from robo_infra.utils.hardware import HardwareProbe, DriverHealth
    >>>
    >>> @with_retry(max_attempts=3)
    ... async def read_sensor():
    ...     return await driver.read()
    >>>
    >>> degraded = DegradedModeController(controller)
    >>>
    >>> # Register cleanup for graceful shutdown
    >>> register_cleanup(controller.emergency_stop)
    >>>
    >>> # Check hardware access
    >>> probe = HardwareProbe()
    >>> result = probe.check_gpio_access()
"""

from robo_infra.utils.degraded import (
    DegradedComponent,
    DegradedModeController,
    DegradedModeStatus,
)
from robo_infra.utils.hardware import (
    DriverHealth,
    DriverReconnector,
    FailureMode,
    HardwareProbe,
    HealthCheck,
    HealthStatus,
    JetsonOptimizer,
    MovementState,
    PlatformOptimizer,
    ProbeResult,
    RaspberryPiOptimizer,
    ReconnectConfig,
    ReconnectStrategy,
    SimulationConfig,
    check_hardware_access,
    get_platform_optimizer,
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
from robo_infra.utils.resources import (
    AsyncContextManager,
    ConnectionPool,
    LimitedBuffer,
    ManagedResource,
    PoolConfig,
    ResourceManager,
    register_cleanup,
    register_cleanup_async,
)
from robo_infra.utils.security import (
    AddressRange,
    HardwareAccess,
    InputValidator,
    JointLimits,
    PrivilegeError,
    SpeedLimits,
    ValidationError,
    check_all_hardware_access,
    check_can_access,
    check_gpio_access,
    check_i2c_access,
    check_serial_access,
    check_spi_access,
    get_required_groups,
    sanitize_name,
    sanitize_serial_command,
    validate_acceleration,
    validate_can_id,
    validate_i2c_address,
    validate_joint_angle,
    validate_joint_angles,
    validate_port_name,
    validate_speed,
)


__all__ = [
    # Security - Input Validation
    "AddressRange",
    # Resources
    "AsyncContextManager",
    # Resilience
    "CircuitBreaker",
    "CircuitBreakerError",
    "CircuitBreakerStats",
    "CircuitState",
    "ConnectionPool",
    # Degraded Mode
    "DegradedComponent",
    "DegradedModeController",
    "DegradedModeStatus",
    # Hardware
    "DriverHealth",
    "DriverReconnector",
    "FailureMode",
    # Security - Privilege Checking
    "HardwareAccess",
    "HardwareProbe",
    "HealthCheck",
    "HealthStatus",
    "InputValidator",
    "JetsonOptimizer",
    "JointLimits",
    "LimitedBuffer",
    "ManagedResource",
    "MovementState",
    "PlatformOptimizer",
    "PoolConfig",
    "PrivilegeError",
    "ProbeResult",
    "RaspberryPiOptimizer",
    "ReconnectConfig",
    "ReconnectStrategy",
    "ResourceManager",
    "RetryConfig",
    "RetryExhaustedError",
    "SimulationConfig",
    "SpeedLimits",
    "ValidationError",
    "check_all_hardware_access",
    "check_can_access",
    "check_gpio_access",
    "check_hardware_access",
    "check_i2c_access",
    "check_serial_access",
    "check_spi_access",
    "create_driver_circuit_breaker",
    "create_sensor_circuit_breaker",
    "get_platform_optimizer",
    "get_required_groups",
    "register_cleanup",
    "register_cleanup_async",
    "retry_sync",
    "run_with_timeout",
    "sanitize_name",
    "sanitize_serial_command",
    "validate_acceleration",
    "validate_can_id",
    "validate_i2c_address",
    "validate_joint_angle",
    "validate_joint_angles",
    "validate_port_name",
    "validate_speed",
    "with_retry",
    "with_timeout",
]
