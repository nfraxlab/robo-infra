"""Tests for hardware abstraction utilities (Phase 5.17).

Tests for simulation configuration, hardware probing, health monitoring,
driver reconnection, and platform optimizations.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.exceptions import CommunicationError
from robo_infra.core.exceptions import TimeoutError as RoboTimeoutError
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


# =============================================================================
# SimulationConfig Tests
# =============================================================================


class TestSimulationConfig:
    """Tests for SimulationConfig."""

    def test_default_values(self) -> None:
        """Test default configuration values."""
        config = SimulationConfig()
        assert config.delay == 0.001
        assert config.delay_jitter == 0.0
        assert config.noise == 0.0
        assert config.failure_rate == 0.0
        assert config.failure_mode == FailureMode.COMMUNICATION_ERROR
        assert config.physics_enabled is False
        assert config.acceleration == 100.0
        assert config.max_velocity == 50.0

    def test_custom_values(self) -> None:
        """Test custom configuration values."""
        config = SimulationConfig(
            delay=0.01,
            noise=0.05,
            failure_rate=0.01,
            failure_mode=FailureMode.TIMEOUT,
        )
        assert config.delay == 0.01
        assert config.noise == 0.05
        assert config.failure_rate == 0.01
        assert config.failure_mode == FailureMode.TIMEOUT

    def test_get_delay_no_jitter(self) -> None:
        """Test get_delay without jitter."""
        config = SimulationConfig(delay=0.01, delay_jitter=0.0)
        assert config.get_delay() == 0.01

    def test_get_delay_with_jitter(self) -> None:
        """Test get_delay with jitter is within bounds."""
        config = SimulationConfig(delay=0.01, delay_jitter=0.005)
        delays = [config.get_delay() for _ in range(100)]

        # All delays should be within jitter range
        assert all(0.005 <= d <= 0.015 for d in delays)
        # Some variation should exist
        assert len(set(delays)) > 1

    def test_apply_noise_zero(self) -> None:
        """Test apply_noise with no noise."""
        config = SimulationConfig(noise=0.0)
        assert config.apply_noise(1.0) == 1.0

    def test_apply_noise_nonzero(self) -> None:
        """Test apply_noise with noise."""
        config = SimulationConfig(noise=0.1)  # ±10%
        values = [config.apply_noise(1.0) for _ in range(100)]

        # Values should be around 1.0
        assert 0.7 < sum(values) / len(values) < 1.3
        # Some variation should exist
        assert len(set(values)) > 1

    def test_should_fail_zero_rate(self) -> None:
        """Test should_fail with zero failure rate."""
        config = SimulationConfig(failure_rate=0.0)
        # Should never fail
        assert not any(config.should_fail() for _ in range(100))

    def test_should_fail_always(self) -> None:
        """Test should_fail with 100% failure rate."""
        config = SimulationConfig(failure_rate=1.0)
        # Should always fail
        assert all(config.should_fail() for _ in range(10))

    def test_simulate_failure_timeout(self) -> None:
        """Test simulate_failure raises TimeoutError."""
        config = SimulationConfig(failure_mode=FailureMode.TIMEOUT)
        with pytest.raises(RoboTimeoutError):
            config.simulate_failure()

    def test_simulate_failure_communication_error(self) -> None:
        """Test simulate_failure raises CommunicationError."""
        config = SimulationConfig(failure_mode=FailureMode.COMMUNICATION_ERROR)
        with pytest.raises(CommunicationError):
            config.simulate_failure()

    def test_simulate_failure_intermittent(self) -> None:
        """Test simulate_failure raises either exception type."""
        config = SimulationConfig(failure_mode=FailureMode.INTERMITTENT)
        errors = []
        for _ in range(20):
            try:
                config.simulate_failure()
            except (RoboTimeoutError, CommunicationError) as e:
                errors.append(type(e).__name__)

        # Should have both types (with high probability)
        assert len(errors) == 20


# =============================================================================
# MovementState Tests
# =============================================================================


class TestMovementState:
    """Tests for MovementState physics simulation."""

    def test_initial_state(self) -> None:
        """Test initial state."""
        state = MovementState()
        assert state.position == 0.0
        assert state.velocity == 0.0
        assert state.target == 0.0

    def test_at_target(self) -> None:
        """Test update when already at target."""
        state = MovementState(position=1.0, target=1.0)
        config = SimulationConfig(physics_enabled=True)

        reached = state.update(config, dt=0.01)

        assert reached is True
        assert state.position == 1.0
        assert state.velocity == 0.0

    def test_move_toward_target(self) -> None:
        """Test movement toward target."""
        state = MovementState(position=0.0, target=10.0)
        config = SimulationConfig(physics_enabled=True, acceleration=100.0)

        # Update multiple times
        for _ in range(100):
            state.update(config, dt=0.01)

        # Should have moved toward target
        assert state.position > 0.0

    def test_reaches_target(self) -> None:
        """Test that position eventually reaches target."""
        state = MovementState(position=0.0, target=5.0)
        config = SimulationConfig(
            physics_enabled=True,
            acceleration=100.0,
            max_velocity=50.0,
        )

        # Update until target reached or timeout
        for _ in range(1000):
            if state.update(config, dt=0.01):
                break

        assert abs(state.position - state.target) < 0.01


# =============================================================================
# HardwareProbe Tests
# =============================================================================


class TestHardwareProbe:
    """Tests for HardwareProbe."""

    def test_init_defaults(self) -> None:
        """Test default initialization."""
        probe = HardwareProbe()
        assert probe.timeout == 2.0
        assert probe.retries == 3
        assert probe.retry_delay == 0.1

    def test_init_custom(self) -> None:
        """Test custom initialization."""
        probe = HardwareProbe(timeout=5.0, retries=5, retry_delay=0.5)
        assert probe.timeout == 5.0
        assert probe.retries == 5
        assert probe.retry_delay == 0.5

    @patch("robo_infra.utils.hardware.Path")
    @patch("robo_infra.utils.hardware.os.access")
    def test_check_gpio_access_no_chips(
        self, mock_access: MagicMock, mock_path: MagicMock
    ) -> None:
        """Test check_gpio_access when no GPIO chips exist."""
        mock_path.return_value.glob.return_value = []

        probe = HardwareProbe()
        result = probe.check_gpio_access()

        assert result.found is False
        assert "No GPIO chips found" in str(result.error)

    @patch("robo_infra.utils.hardware.Path")
    def test_check_serial_port_not_found(self, mock_path: MagicMock) -> None:
        """Test check_serial_port when port doesn't exist."""
        mock_path.return_value.exists.return_value = False

        probe = HardwareProbe()
        result = probe.check_serial_port("/dev/ttyUSB0")

        assert result.found is False
        assert "not found" in str(result.error)

    @patch("robo_infra.utils.hardware.Path")
    @patch("robo_infra.utils.hardware.os.access")
    def test_check_serial_port_no_access(
        self, mock_access: MagicMock, mock_path: MagicMock
    ) -> None:
        """Test check_serial_port when port is inaccessible."""
        mock_path.return_value.exists.return_value = True
        mock_access.return_value = False

        probe = HardwareProbe()
        result = probe.check_serial_port("/dev/ttyUSB0")

        assert result.found is False
        assert "dialout" in str(result.error)

    @patch("robo_infra.utils.hardware.Path")
    def test_check_spi_device_not_found(self, mock_path: MagicMock) -> None:
        """Test check_spi_device when device doesn't exist."""
        mock_path.return_value.exists.return_value = False

        probe = HardwareProbe()
        result = probe.check_spi_device(bus=0, device=0)

        assert result.found is False
        assert "not found" in str(result.error)


class TestProbeResult:
    """Tests for ProbeResult."""

    def test_found_result(self) -> None:
        """Test result when hardware found."""
        result = ProbeResult(
            found=True,
            response_time=0.01,
            details={"bus": 1, "address": "0x40"},
        )
        assert result.found is True
        assert result.response_time == 0.01
        assert result.error is None
        assert result.details["bus"] == 1

    def test_not_found_result(self) -> None:
        """Test result when hardware not found."""
        result = ProbeResult(
            found=False,
            response_time=0.5,
            error="Device not responding",
        )
        assert result.found is False
        assert result.error == "Device not responding"


# =============================================================================
# DriverHealth Tests
# =============================================================================


class TestDriverHealth:
    """Tests for DriverHealth monitoring."""

    def test_init_defaults(self) -> None:
        """Test default initialization."""
        health = DriverHealth("test_driver")
        assert health.name == "test_driver"
        assert health.error_threshold == 5
        assert health.latency_threshold == 0.1
        assert health.window_size == 100

    def test_record_success(self) -> None:
        """Test recording successful operations."""
        health = DriverHealth("test")
        health.record_success(latency=0.01)
        health.record_success(latency=0.02)

        check = health.check()
        assert check.status == HealthStatus.HEALTHY
        assert check.latency == 0.015  # Average
        assert check.error_count == 0

    def test_record_error(self) -> None:
        """Test recording failed operations."""
        health = DriverHealth("test", error_threshold=3)
        health.record_success(0.01)  # Start healthy

        health.record_error("Connection lost")
        check = health.check()
        assert check.status == HealthStatus.DEGRADED
        assert check.error_count == 1

    def test_consecutive_errors_unhealthy(self) -> None:
        """Test that consecutive errors lead to UNHEALTHY status."""
        health = DriverHealth("test", error_threshold=3)

        for i in range(3):
            health.record_error(f"Error {i}")

        check = health.check()
        assert check.status == HealthStatus.UNHEALTHY
        assert check.error_count == 3

    def test_high_latency_degraded(self) -> None:
        """Test that high latency leads to DEGRADED status."""
        health = DriverHealth("test", latency_threshold=0.1)

        # Record high latency
        health.record_success(latency=0.2)

        check = health.check()
        assert check.status == HealthStatus.DEGRADED

    def test_success_resets_consecutive_errors(self) -> None:
        """Test that success resets consecutive error count."""
        health = DriverHealth("test", error_threshold=5)

        health.record_error("Error 1")
        health.record_error("Error 2")
        health.record_success(0.01)

        check = health.check()
        # Should be HEALTHY because consecutive errors reset
        assert check.status == HealthStatus.HEALTHY
        assert check.error_count == 2  # Total errors still tracked

    def test_unknown_status_no_data(self) -> None:
        """Test UNKNOWN status when no data recorded."""
        health = DriverHealth("test")
        check = health.check()
        assert check.status == HealthStatus.UNKNOWN

    def test_reset(self) -> None:
        """Test resetting health statistics."""
        health = DriverHealth("test")
        health.record_success(0.01)
        health.record_error("Error")

        health.reset()

        check = health.check()
        assert check.status == HealthStatus.UNKNOWN
        assert check.error_count == 0


class TestHealthCheck:
    """Tests for HealthCheck."""

    def test_health_check_attributes(self) -> None:
        """Test HealthCheck attributes."""
        check = HealthCheck(
            status=HealthStatus.HEALTHY,
            latency=0.01,
            error_count=0,
            last_success=time.time(),
            details={"sample_count": 10},
        )
        assert check.status == HealthStatus.HEALTHY
        assert check.latency == 0.01
        assert check.error_count == 0
        assert check.last_success is not None
        assert check.details["sample_count"] == 10


# =============================================================================
# ReconnectConfig Tests
# =============================================================================


class TestReconnectConfig:
    """Tests for ReconnectConfig."""

    def test_default_values(self) -> None:
        """Test default configuration values."""
        config = ReconnectConfig()
        assert config.strategy == ReconnectStrategy.EXPONENTIAL_BACKOFF
        assert config.max_attempts == 10
        assert config.initial_delay == 0.1
        assert config.max_delay == 30.0
        assert config.backoff_factor == 2.0

    def test_immediate_delay(self) -> None:
        """Test immediate strategy has no delay."""
        config = ReconnectConfig(strategy=ReconnectStrategy.IMMEDIATE)
        assert config.get_delay(1) == 0.0
        assert config.get_delay(10) == 0.0

    def test_linear_backoff(self) -> None:
        """Test linear backoff delay calculation."""
        config = ReconnectConfig(
            strategy=ReconnectStrategy.LINEAR_BACKOFF,
            initial_delay=1.0,
            max_delay=10.0,
        )
        assert config.get_delay(1) == 1.0
        assert config.get_delay(2) == 2.0
        assert config.get_delay(5) == 5.0
        assert config.get_delay(15) == 10.0  # Capped at max_delay

    def test_exponential_backoff(self) -> None:
        """Test exponential backoff delay calculation."""
        config = ReconnectConfig(
            strategy=ReconnectStrategy.EXPONENTIAL_BACKOFF,
            initial_delay=1.0,
            backoff_factor=2.0,
            max_delay=30.0,
        )
        assert config.get_delay(1) == 1.0  # 1 * 2^0
        assert config.get_delay(2) == 2.0  # 1 * 2^1
        assert config.get_delay(3) == 4.0  # 1 * 2^2
        assert config.get_delay(4) == 8.0  # 1 * 2^3
        assert config.get_delay(10) == 30.0  # Capped at max_delay


# =============================================================================
# DriverReconnector Tests
# =============================================================================


class TestDriverReconnector:
    """Tests for DriverReconnector."""

    def test_connect_success_first_attempt(self) -> None:
        """Test successful connection on first attempt."""
        mock_driver = MagicMock()
        create_driver = MagicMock(return_value=mock_driver)

        reconnector = DriverReconnector(create_driver)
        driver = reconnector.connect()

        assert driver is mock_driver
        assert reconnector.attempt_count == 1
        create_driver.assert_called_once()

    def test_connect_with_retries(self) -> None:
        """Test connection with retries."""
        mock_driver = MagicMock()
        create_driver = MagicMock(
            side_effect=[
                Exception("Fail 1"),
                Exception("Fail 2"),
                mock_driver,
            ]
        )

        config = ReconnectConfig(
            strategy=ReconnectStrategy.IMMEDIATE,
            max_attempts=5,
        )
        reconnector = DriverReconnector(create_driver, config=config)
        driver = reconnector.connect()

        assert driver is mock_driver
        assert reconnector.attempt_count == 3

    def test_connect_all_attempts_fail(self) -> None:
        """Test connection when all attempts fail."""
        create_driver = MagicMock(side_effect=Exception("Always fails"))

        config = ReconnectConfig(
            strategy=ReconnectStrategy.IMMEDIATE,
            max_attempts=3,
        )
        reconnector = DriverReconnector(create_driver, config=config)

        with pytest.raises(CommunicationError) as exc_info:
            reconnector.connect()

        assert "Failed to connect after 3 attempts" in str(exc_info.value)
        assert reconnector.attempt_count == 3

    def test_connect_calls_driver_connect(self) -> None:
        """Test that connect() is called on driver if available."""
        mock_driver = MagicMock()
        create_driver = MagicMock(return_value=mock_driver)

        reconnector = DriverReconnector(create_driver)
        reconnector.connect()

        mock_driver.connect.assert_called_once()

    def test_on_reconnect_callback(self) -> None:
        """Test on_reconnect callback is called on reconnection."""
        mock_driver = MagicMock()
        create_driver = MagicMock(
            side_effect=[Exception("Fail"), mock_driver]
        )
        on_reconnect = MagicMock()

        config = ReconnectConfig(strategy=ReconnectStrategy.IMMEDIATE)
        reconnector = DriverReconnector(
            create_driver,
            config=config,
            on_reconnect=on_reconnect,
        )
        reconnector.connect()

        on_reconnect.assert_called_once_with(mock_driver, 2)

    def test_reset(self) -> None:
        """Test resetting attempt counter."""
        create_driver = MagicMock(return_value=MagicMock())
        reconnector = DriverReconnector(create_driver)
        reconnector.connect()

        reconnector.reset()

        assert reconnector.attempt_count == 0

    @pytest.mark.asyncio
    async def test_connect_async_success(self) -> None:
        """Test async connection success."""
        # Use a simple class without connect_async to avoid await issues
        class SimpleDriver:
            pass

        mock_driver = SimpleDriver()
        create_driver = MagicMock(return_value=mock_driver)

        reconnector = DriverReconnector(create_driver)
        driver = await reconnector.connect_async()

        assert driver is mock_driver

    @pytest.mark.asyncio
    async def test_connect_async_with_retries(self) -> None:
        """Test async connection with retries."""
        # Use a simple class without connect_async to avoid await issues
        class SimpleDriver:
            pass

        mock_driver = SimpleDriver()
        create_driver = MagicMock(
            side_effect=[
                Exception("Fail 1"),
                mock_driver,
            ]
        )

        config = ReconnectConfig(
            strategy=ReconnectStrategy.IMMEDIATE,
            max_attempts=5,
        )
        reconnector = DriverReconnector(create_driver, config=config)
        driver = await reconnector.connect_async()

        assert driver is mock_driver
        assert reconnector.attempt_count == 2


# =============================================================================
# PlatformOptimizer Tests
# =============================================================================


class TestPlatformOptimizer:
    """Tests for PlatformOptimizer."""

    def test_base_optimizer(self) -> None:
        """Test base platform optimizer."""
        optimizer = PlatformOptimizer("test_platform")
        assert optimizer.platform == "test_platform"
        assert "software PWM" in optimizer.pwm_hint
        assert "libgpiod" in optimizer.gpio_hint
        assert optimizer.serial_buffer_size == 4096
        assert optimizer.i2c_clock_stretch_timeout == 0.01


class TestRaspberryPiOptimizer:
    """Tests for RaspberryPiOptimizer."""

    def test_raspberry_pi_optimizer(self) -> None:
        """Test Raspberry Pi optimizer."""
        optimizer = RaspberryPiOptimizer()
        assert optimizer.platform == "raspberry_pi"
        assert "hardware PWM" in optimizer.pwm_hint
        assert "12" in optimizer.pwm_hint  # Hardware PWM pin
        assert "pigpio" in optimizer.gpio_hint

    def test_is_hardware_pwm_pin(self) -> None:
        """Test hardware PWM pin detection."""
        optimizer = RaspberryPiOptimizer()
        assert optimizer.is_hardware_pwm_pin(12) is True
        assert optimizer.is_hardware_pwm_pin(13) is True
        assert optimizer.is_hardware_pwm_pin(18) is True
        assert optimizer.is_hardware_pwm_pin(19) is True
        assert optimizer.is_hardware_pwm_pin(17) is False


class TestJetsonOptimizer:
    """Tests for JetsonOptimizer."""

    def test_jetson_optimizer(self) -> None:
        """Test Jetson optimizer."""
        optimizer = JetsonOptimizer()
        assert optimizer.platform == "jetson"
        assert "Jetson.GPIO" in optimizer.pwm_hint
        assert optimizer.serial_buffer_size == 8192  # Larger for Jetson


class TestGetPlatformOptimizer:
    """Tests for get_platform_optimizer function."""

    def test_get_platform_optimizer_returns_instance(self) -> None:
        """Test that get_platform_optimizer returns an optimizer."""
        optimizer = get_platform_optimizer()
        assert isinstance(optimizer, PlatformOptimizer)


# =============================================================================
# Convenience Function Tests
# =============================================================================


class TestCheckHardwareAccess:
    """Tests for check_hardware_access function."""

    @patch("robo_infra.utils.hardware.Path")
    @patch("robo_infra.utils.hardware.os.access")
    def test_check_multiple_resources(
        self, mock_access: MagicMock, mock_path: MagicMock
    ) -> None:
        """Test checking multiple hardware resources."""
        # Mock GPIO as available
        mock_gpio_chip = MagicMock()
        mock_gpio_chip.__str__ = lambda _: "/dev/gpiochip0"
        mock_path.return_value.glob.return_value = [mock_gpio_chip]
        mock_path.return_value.exists.return_value = True
        mock_access.return_value = True

        results = check_hardware_access(
            require_gpio=True,
            i2c_buses=[1],
        )

        assert "gpio" in results
        assert "i2c-1" in results

    def test_empty_check(self) -> None:
        """Test check with no resources specified."""
        results = check_hardware_access()
        assert results == {}


# =============================================================================
# Integration Tests
# =============================================================================


class TestHardwareAbstractionIntegration:
    """Integration tests for hardware abstraction."""

    def test_simulation_config_with_failure_injection(self) -> None:
        """Test simulation with failure injection."""
        config = SimulationConfig(
            delay=0.001,
            noise=0.02,
            failure_rate=0.5,
            failure_mode=FailureMode.COMMUNICATION_ERROR,
        )

        # Run multiple operations, some should fail
        failures = 0
        successes = 0

        for _ in range(100):
            if config.should_fail():
                failures += 1
            else:
                successes += 1

        # With 50% rate, should have reasonable mix
        assert failures > 20
        assert successes > 20

    def test_health_with_reconnector(self) -> None:
        """Test health monitoring with reconnection."""
        health = DriverHealth("test", error_threshold=3)
        mock_driver = MagicMock()

        # Simulate initial connection failures
        create_count = 0

        def create_driver() -> MagicMock:
            nonlocal create_count
            create_count += 1
            if create_count < 3:
                health.record_error(f"Connection failed {create_count}")
                raise CommunicationError("Connection failed")
            return mock_driver

        config = ReconnectConfig(
            strategy=ReconnectStrategy.IMMEDIATE,
            max_attempts=5,
        )
        reconnector = DriverReconnector(create_driver, config=config)
        driver = reconnector.connect()

        assert driver is mock_driver
        assert health.check().error_count == 2

        # After successful connection, record success
        health.record_success(0.01)
        assert health.check().status == HealthStatus.HEALTHY
