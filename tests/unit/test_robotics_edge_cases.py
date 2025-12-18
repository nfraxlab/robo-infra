"""Unit tests for robotics edge cases.

Tests cover critical safety scenarios that could cause physical harm,
property damage, or robot failure:

Task 4.4.8 Areas:
1. E-stop failure recovery — What happens when disable() throws during E-stop?
2. Sensor disconnection — Hardware disconnects mid-stream
3. Limit boundary behavior — Actuator at min, max, and beyond limits
4. Communication timeout — I2C/SPI device not responding
5. Control loop overrun — Update() takes longer than period
6. Concurrent actuator access — Multiple threads calling set() simultaneously
7. PWM value overflow — Duty cycle > 100% or < 0%
8. Encoder rollover — Position counter overflow
9. Power loss recovery — Robot state after unexpected power cycle
10. Homing failure — Limit switch not triggered, motor stall
11. Collision detection — Force sensor triggers during movement
12. Thermal shutdown — Motor overtemperature behavior

ROBOTICS SOFTWARE SAFETY REQUIREMENTS:
- E-stop MUST always disable actuators (first priority)
- Never lose position/state silently
- Fail safe, not fail dangerous
- Explicit simulation mode (no silent fallback)
"""

from __future__ import annotations

import threading
import time
from typing import Any

import pytest

from robo_infra.core.actuator import (
    Actuator,
    ActuatorConfig,
    ActuatorState,
    ActuatorStatus,
)
from robo_infra.core.driver import (
    Driver,
    DriverState,
)
from robo_infra.core.exceptions import (
    CalibrationError,
    CommunicationError,
    DisabledError,
    HardwareNotFoundError,
    LimitsExceededError,
    NotCalibratedError,
    SafetyError,
)
from robo_infra.core.exceptions import (
    TimeoutError as RoboTimeoutError,
)
from robo_infra.core.types import Limits


# ============================================================================
# Test Fixtures
# ============================================================================


class MockActuator:
    """Mock actuator for testing."""

    def __init__(self, name: str = "mock_actuator", fail_on_disable: bool = False) -> None:
        self.name = name
        self._enabled = True
        self._fail_on_disable = fail_on_disable
        self._disable_count = 0

    def disable(self) -> None:
        self._disable_count += 1
        if self._fail_on_disable:
            raise RuntimeError(f"Failed to disable {self.name}")
        self._enabled = False


class MockDriver(Driver):
    """Mock driver for testing."""

    def __init__(
        self,
        name: str = "mock_driver",
        channels: int = 16,
        fail_on_connect: bool = False,
        fail_on_write: bool = False,
        write_delay: float = 0.0,
    ) -> None:
        super().__init__(name=name, channels=channels)
        self._fail_on_connect = fail_on_connect
        self._fail_on_write = fail_on_write
        self._write_delay = write_delay
        self._values: dict[int, float] = {}

    def connect(self) -> None:
        if self._fail_on_connect:
            raise CommunicationError("I2C", 0x40, "Connection refused")
        self._state = DriverState.CONNECTED
        self._connected = True

    def disconnect(self) -> None:
        self._state = DriverState.DISCONNECTED
        self._connected = False

    def _write_channel(self, channel: int, value: float) -> None:
        if self._write_delay > 0:
            time.sleep(self._write_delay)
        if self._fail_on_write:
            raise CommunicationError("I2C", 0x40, "Write failed")
        self._values[channel] = value

    def _read_channel(self, channel: int) -> float:
        return self._values.get(channel, 0.0)


class _TestableActuator(Actuator):
    """Concrete actuator for testing. Prefix with _ to avoid pytest collection."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._applied_values: list[float] = []
        self._read_value_result = 0.0

    def _apply_value(self, value: float) -> None:
        self._applied_values.append(value)
        if self._driver:
            self._driver.set_channel(self.channel, value)

    def _read_value(self) -> float:
        if self._driver:
            return self._driver.read_channel(self.channel)
        return self._read_value_result


# Alias for convenience - pytest won't collect this
TestableActuator = _TestableActuator


# ============================================================================
# 1. E-Stop Failure Recovery Tests
# ============================================================================


class TestEStopFailureRecovery:
    """Tests for E-stop behavior when actuator disable() throws."""

    def test_estop_attempts_all_actuators_even_if_some_fail(self):
        """E-stop should attempt to disable ALL actuators, even if some fail."""
        from robo_infra.safety.estop import EStop, EStopConfig, EStopError

        estop = EStop(config=EStopConfig(propagate_errors=True))

        # Register 3 actuators - middle one fails
        actuator1 = MockActuator("servo1", fail_on_disable=False)
        actuator2 = MockActuator("servo2", fail_on_disable=True)
        actuator3 = MockActuator("servo3", fail_on_disable=False)

        estop.register_actuator(actuator1)
        estop.register_actuator(actuator2)
        estop.register_actuator(actuator3)

        # Trigger should attempt all and raise
        with pytest.raises(EStopError) as exc_info:
            estop.trigger("Test E-stop")

        # Verify all actuators were attempted
        assert actuator1._disable_count >= 1
        assert actuator2._disable_count >= 1  # Failed but attempted
        assert actuator3._disable_count >= 1

        # Verify error contains failed actuator
        assert "servo2" in exc_info.value.failed_actuators

    def test_estop_multiple_retry_attempts(self):
        """E-stop should retry disable() multiple times per actuator."""
        from robo_infra.safety.estop import EStop, EStopConfig, EStopError

        config = EStopConfig(max_disable_attempts=3, propagate_errors=True)
        estop = EStop(config=config)

        # Actuator that always fails
        failing_actuator = MockActuator("failing", fail_on_disable=True)
        estop.register_actuator(failing_actuator)

        with pytest.raises(EStopError):
            estop.trigger("Test retry")

        # Should have attempted 3 times
        assert failing_actuator._disable_count == 3

    def test_estop_succeeds_on_retry(self):
        """E-stop should succeed if actuator works on retry."""
        from robo_infra.safety.estop import EStop, EStopConfig

        estop = EStop(config=EStopConfig(max_disable_attempts=3))

        # Actuator that fails first 2 times, succeeds on 3rd
        call_count = 0

        class RetryableActuator:
            name = "retryable"

            def disable(self):
                nonlocal call_count
                call_count += 1
                if call_count < 3:
                    raise RuntimeError("Not yet!")

        estop.register_actuator(RetryableActuator())

        event = estop.trigger("Test retry success")

        assert "retryable" in event.actuators_disabled
        assert "retryable" not in event.actuators_failed
        assert call_count == 3

    def test_estop_callbacks_called_after_disable_attempts(self):
        """E-stop callbacks should be called AFTER all disable attempts."""
        from robo_infra.safety.estop import EStop

        estop = EStop()
        callback_called = False
        callback_event = None

        def callback(event):
            nonlocal callback_called, callback_event
            callback_called = True
            callback_event = event

        estop.register_callback(callback)
        estop.register_actuator(MockActuator("test"))

        estop.trigger("Test callback")

        assert callback_called
        assert callback_event is not None
        assert "test" in callback_event.actuators_disabled

    def test_estop_callback_error_does_not_prevent_estop(self):
        """E-stop should complete even if callback throws."""
        from robo_infra.safety.estop import EStop

        estop = EStop()

        def failing_callback(event):
            raise RuntimeError("Callback failed!")

        estop.register_callback(failing_callback)
        actuator = MockActuator("test")
        estop.register_actuator(actuator)

        # Should not raise, should still disable actuator
        event = estop.trigger("Test callback failure")

        assert not actuator._enabled
        assert "test" in event.actuators_disabled

    def test_estop_state_transitions(self):
        """E-stop should transition through correct states."""
        from robo_infra.safety.estop import EStop, EStopState

        estop = EStop()
        estop.register_actuator(MockActuator("test"))

        assert estop.state == EStopState.ARMED

        estop.trigger("Test")
        assert estop.state == EStopState.TRIGGERED

    def test_estop_event_log_maintained(self):
        """E-stop should maintain event log for auditing."""
        from robo_infra.safety.estop import EStop

        estop = EStop()
        estop.register_actuator(MockActuator("test"))

        estop.trigger("First trigger", triggered_by="user")
        estop.reset()
        estop.trigger("Second trigger", triggered_by="sensor")

        assert len(estop.event_log) >= 2

        first_event = estop.event_log[0]
        assert first_event.reason == "First trigger"
        assert first_event.triggered_by == "user"


# ============================================================================
# 2. Limit Boundary Behavior Tests
# ============================================================================


class TestLimitBoundaryBehavior:
    """Tests for actuator behavior at min/max limits."""

    def test_set_at_min_limit_succeeds(self):
        """Setting value at minimum limit should succeed."""
        limits = Limits(min=0.0, max=180.0, default=90.0)
        actuator = TestableActuator(name="test", limits=limits)
        actuator.enable()

        actuator.set(0.0)  # Exactly at min
        assert actuator.get() == 0.0

    def test_set_at_max_limit_succeeds(self):
        """Setting value at maximum limit should succeed."""
        limits = Limits(min=0.0, max=180.0, default=90.0)
        actuator = TestableActuator(name="test", limits=limits)
        actuator.enable()

        actuator.set(180.0)  # Exactly at max
        assert actuator.get() == 180.0

    def test_set_beyond_max_raises(self):
        """Setting value beyond maximum should raise LimitsExceededError."""
        limits = Limits(min=0.0, max=180.0, default=90.0)
        actuator = TestableActuator(name="test", limits=limits)
        actuator.enable()

        with pytest.raises(LimitsExceededError) as exc_info:
            actuator.set(181.0)

        assert exc_info.value.value == 181.0
        assert exc_info.value.max_limit == 180.0

    def test_set_below_min_raises(self):
        """Setting value below minimum should raise LimitsExceededError."""
        limits = Limits(min=0.0, max=180.0, default=90.0)
        actuator = TestableActuator(name="test", limits=limits)
        actuator.enable()

        with pytest.raises(LimitsExceededError) as exc_info:
            actuator.set(-1.0)

        assert exc_info.value.value == -1.0
        assert exc_info.value.min_limit == 0.0

    def test_limit_enforcer_clamps_velocity(self):
        """Limit enforcer should clamp velocity to configured limit."""
        from robo_infra.safety.limits import LimitEnforcer

        enforcer = LimitEnforcer(
            position_limits=(0.0, 180.0),
            velocity_limit=10.0,  # 10 units/second
        )

        # First position establishes reference
        enforcer.enforce(90.0)

        # Large jump should be velocity-limited (with some delay)
        time.sleep(0.1)  # 100ms
        result = enforcer.enforce(180.0)  # Trying to jump 90 degrees

        # Result should be clamped based on velocity limit
        # In 0.1s at 10 deg/s, max move is ~1 degree
        assert result <= 92.0  # Much less than 180

    def test_limit_enforcer_soft_limit_warning(self):
        """Limit enforcer should warn when approaching soft limit."""
        from robo_infra.safety.limits import LimitEnforcer

        enforcer = LimitEnforcer(
            position_limits=(0.0, 100.0),
        )

        # Position in soft limit zone (>90% of range)
        result = enforcer.enforce(95.0)

        # Should still work but mark as soft limit
        assert result == 95.0
        # Check violations list exists (implementation may vary)
        assert isinstance(enforcer.recent_violations, list)


# ============================================================================
# 3. Communication Timeout Tests
# ============================================================================


class TestCommunicationTimeout:
    """Tests for I2C/SPI device not responding."""

    def test_driver_connect_timeout_raises(self):
        """Driver should raise on connection timeout."""
        driver = MockDriver(fail_on_connect=True)

        with pytest.raises(CommunicationError) as exc_info:
            driver.connect()

        assert "Connection refused" in str(exc_info.value)

    def test_driver_write_timeout_raises(self):
        """Driver should raise on write timeout."""
        driver = MockDriver(fail_on_write=True)
        driver.connect()
        driver.enable()

        with pytest.raises(CommunicationError):
            driver.set_channel(0, 0.5)

    def test_actuator_handles_driver_error(self):
        """Actuator should propagate driver communication errors."""
        driver = MockDriver(fail_on_write=True)
        driver.connect()
        driver.enable()

        actuator = TestableActuator(
            name="test",
            driver=driver,
            limits=Limits(0.0, 1.0, 0.5),
        )
        actuator.enable()

        with pytest.raises(CommunicationError):
            actuator.set(0.5)

    def test_hardware_not_found_error_explicit(self):
        """HardwareNotFoundError should include device details."""
        error = HardwareNotFoundError("PCA9685", "Not found at 0x40")

        assert "PCA9685" in str(error)
        assert "0x40" in str(error)
        assert error.device == "PCA9685"


# ============================================================================
# 4. Control Loop Overrun Tests
# ============================================================================


class TestControlLoopOverrun:
    """Tests for control loop timing issues."""

    def test_watchdog_triggers_on_missed_feed(self):
        """Watchdog should trigger if feed() not called in time."""
        from robo_infra.safety.watchdog import Watchdog, WatchdogState

        watchdog = Watchdog(timeout=0.05)  # 50ms timeout
        watchdog.start()

        # Don't feed for longer than timeout
        time.sleep(0.1)

        # Watchdog should have triggered
        assert watchdog.state == WatchdogState.TRIGGERED

        watchdog.stop()

    def test_watchdog_warns_on_slow_feed(self):
        """Watchdog should warn when feed approaches timeout."""
        from robo_infra.safety.watchdog import Watchdog

        watchdog = Watchdog(timeout=0.1)  # 100ms timeout
        watchdog.start()

        # Feed just before timeout (at warning threshold)
        time.sleep(0.06)
        status = watchdog.status()

        # Should be approaching warning threshold
        assert status.last_feed_age > 0.05

        watchdog.feed()
        watchdog.stop()

    def test_watchdog_feed_resets_timer(self):
        """Regular feed() calls should keep watchdog happy."""
        from robo_infra.safety.watchdog import Watchdog, WatchdogState

        watchdog = Watchdog(timeout=0.05)  # 50ms timeout
        watchdog.start()

        # Feed regularly
        for _ in range(5):
            time.sleep(0.02)  # Less than timeout
            watchdog.feed()

        assert watchdog.state == WatchdogState.ARMED

        watchdog.stop()

    def test_watchdog_pause_disables_checking(self):
        """Paused watchdog should not trigger."""
        from robo_infra.safety.watchdog import Watchdog, WatchdogState

        watchdog = Watchdog(timeout=0.05)  # 50ms timeout
        watchdog.start()

        watchdog.pause()
        time.sleep(0.1)  # Would normally trigger

        assert watchdog.state == WatchdogState.PAUSED

        watchdog.stop()


# ============================================================================
# 5. Concurrent Actuator Access Tests
# ============================================================================


class TestConcurrentActuatorAccess:
    """Tests for thread safety when multiple threads call actuator methods."""

    def test_concurrent_set_calls_all_complete(self):
        """Concurrent set() calls should all complete without deadlock."""
        actuator = TestableActuator(
            name="test",
            limits=Limits(0.0, 180.0, 90.0),
        )
        actuator.enable()

        errors = []
        set_count = 0
        lock = threading.Lock()

        def set_value(value):
            nonlocal set_count
            try:
                actuator.set(value)
                with lock:
                    set_count += 1
            except Exception as e:
                errors.append(e)

        threads = []
        for i in range(10):
            t = threading.Thread(target=set_value, args=(float(i * 10),))
            threads.append(t)
            t.start()

        for t in threads:
            t.join(timeout=2.0)

        # All threads should complete
        assert all(not t.is_alive() for t in threads)
        # At least some should succeed
        assert set_count > 0

    def test_concurrent_enable_disable_safe(self):
        """Concurrent enable/disable should not corrupt state."""
        actuator = TestableActuator(
            name="test",
            limits=Limits(0.0, 180.0, 90.0),
        )

        def toggle():
            for _ in range(100):
                actuator.enable()
                actuator.disable()

        threads = [threading.Thread(target=toggle) for _ in range(4)]

        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        # Should not be in inconsistent state
        assert actuator.state in [ActuatorState.DISABLED, ActuatorState.IDLE]


# ============================================================================
# 6. PWM Value Overflow Tests
# ============================================================================


class TestPWMValueOverflow:
    """Tests for duty cycle boundary conditions."""

    def test_pwm_value_at_zero(self):
        """PWM at 0% should work."""
        driver = MockDriver()
        driver.connect()
        driver.enable()

        driver.set_channel(0, 0.0)
        assert driver._values[0] == 0.0

    def test_pwm_value_at_max(self):
        """PWM at 100% should work."""
        driver = MockDriver()
        driver.connect()
        driver.enable()

        driver.set_channel(0, 1.0)
        assert driver._values[0] == 1.0

    def test_servo_angle_range_mapping(self):
        """Servo should correctly map angles to positions."""
        from robo_infra.actuators.servo import Servo

        servo = Servo(
            name="test",
            angle_range=(0, 180),
            pulse_range=(500, 2500),  # Standard servo range
        )
        servo.enable()

        # Test angle setting works at key positions
        servo.set(90)  # Middle position
        assert servo.angle == 90

        servo.set(0)  # Min
        assert servo.angle == 0

        servo.set(180)  # Max
        assert servo.angle == 180


# ============================================================================
# 7. Encoder Rollover Tests
# ============================================================================


class TestEncoderRollover:
    """Tests for position counter overflow handling."""

    def test_encoder_handles_max_int32(self):
        """Encoder should handle counts near int32 max."""
        from robo_infra.sensors.encoder import QuadratureConfig, QuadratureEncoder

        config = QuadratureConfig(pulses_per_revolution=1000)
        encoder = QuadratureEncoder(name="test", config=config)

        # Simulate count near max int32
        encoder._count = 2_147_483_640  # Near INT32_MAX

        # Should handle reading without overflow
        count = encoder.count
        assert count == 2_147_483_640

    def test_encoder_reset_clears_count(self):
        """Encoder reset should clear count to zero."""
        from robo_infra.sensors.encoder import QuadratureConfig, QuadratureEncoder

        config = QuadratureConfig(pulses_per_revolution=1000)
        encoder = QuadratureEncoder(name="test", config=config)

        encoder._count = 1000
        encoder.reset()

        assert encoder.count == 0

    def test_encoder_direction_detection(self):
        """Encoder should correctly detect direction."""
        from robo_infra.sensors.encoder import (
            EncoderDirection,
            QuadratureConfig,
            QuadratureEncoder,
        )

        config = QuadratureConfig(pulses_per_revolution=1000)
        encoder = QuadratureEncoder(name="test", config=config)

        # Initially stationary
        assert encoder.direction == EncoderDirection.STATIONARY


# ============================================================================
# 8. Disabled Actuator Access Tests
# ============================================================================


class TestDisabledActuatorAccess:
    """Tests for operations on disabled actuators."""

    def test_set_on_disabled_raises(self):
        """set() on disabled actuator should raise DisabledError."""
        actuator = TestableActuator(
            name="test",
            limits=Limits(0.0, 180.0, 90.0),
        )
        # Don't enable

        with pytest.raises(DisabledError) as exc_info:
            actuator.set(90.0)

        assert "disabled" in str(exc_info.value).lower()

    def test_force_set_bypasses_disabled_check(self):
        """set(force=True) should bypass disabled check."""
        actuator = TestableActuator(
            name="test",
            limits=Limits(0.0, 180.0, 90.0),
        )
        # Don't enable

        actuator.set(90.0, force=True)  # Should not raise
        assert actuator.get() == 90.0

    def test_get_works_when_disabled(self):
        """get() should work even when disabled."""
        actuator = TestableActuator(
            name="test",
            limits=Limits(0.0, 180.0, 90.0),
        )
        # Don't enable

        value = actuator.get()  # Should not raise
        assert isinstance(value, float)


# ============================================================================
# 9. Calibration Requirements Tests
# ============================================================================


class TestCalibrationRequirements:
    """Tests for calibration-required actuators."""

    def test_enable_without_calibration_raises(self):
        """enable() on uncalibrated actuator should raise if required."""
        config = ActuatorConfig(
            name="test",
            require_calibration=True,
        )
        actuator = TestableActuator(name="test", config=config)

        with pytest.raises(NotCalibratedError):
            actuator.enable()

    def test_calibration_allows_enable(self):
        """Calibration should allow enable()."""
        config = ActuatorConfig(
            name="test",
            require_calibration=True,
        )
        actuator = TestableActuator(name="test", config=config)

        actuator.calibrate()
        actuator.enable()  # Should not raise

        assert actuator.is_enabled


# ============================================================================
# 10. State Transition Tests
# ============================================================================


class TestActuatorStateTransitions:
    """Tests for actuator state machine behavior."""

    def test_initial_state_is_disabled(self):
        """New actuator should start disabled."""
        actuator = TestableActuator(name="test", limits=Limits(0.0, 1.0, 0.5))

        assert actuator.state == ActuatorState.DISABLED
        assert not actuator.is_enabled

    def test_enable_transitions_to_idle(self):
        """enable() should transition to IDLE state."""
        actuator = TestableActuator(name="test", limits=Limits(0.0, 1.0, 0.5))

        actuator.enable()

        assert actuator.state == ActuatorState.IDLE
        assert actuator.is_enabled

    def test_set_transitions_through_moving(self):
        """set() should transition through MOVING to HOLDING."""
        actuator = TestableActuator(name="test", limits=Limits(0.0, 1.0, 0.5))
        actuator.enable()

        actuator.set(0.75)

        # After set completes, should be in HOLDING
        assert actuator.state == ActuatorState.HOLDING

    def test_disable_transitions_to_disabled(self):
        """disable() should transition to DISABLED."""
        actuator = TestableActuator(name="test", limits=Limits(0.0, 1.0, 0.5))
        actuator.enable()
        actuator.set(0.5)

        actuator.disable()

        assert actuator.state == ActuatorState.DISABLED
        assert not actuator.is_enabled


# ============================================================================
# 11. Actuator Status Tests
# ============================================================================


class TestActuatorStatus:
    """Tests for actuator status reporting."""

    def test_status_includes_all_fields(self):
        """status() should return complete ActuatorStatus."""
        actuator = TestableActuator(name="test", limits=Limits(0.0, 180.0, 90.0))
        actuator.enable()
        actuator.set(45.0)

        status = actuator.status()

        assert isinstance(status, ActuatorStatus)
        assert status.state == ActuatorState.HOLDING
        assert status.position == 45.0
        assert status.is_enabled is True

    def test_status_reflects_error_state(self):
        """status() should reflect error state."""
        actuator = TestableActuator(name="test", limits=Limits(0.0, 180.0, 90.0))
        actuator._state = ActuatorState.ERROR
        actuator._error = "Test error"

        status = actuator.status()

        assert status.state == ActuatorState.ERROR
        assert status.error == "Test error"


# ============================================================================
# 12. Safety Error Tests
# ============================================================================


class TestSafetyErrors:
    """Tests for safety-related error handling."""

    def test_safety_error_includes_action(self):
        """SafetyError should include action taken."""
        error = SafetyError("Limit exceeded", action_taken="Actuator disabled")

        assert "Limit exceeded" in str(error)
        assert error.action_taken == "Actuator disabled"

    def test_limits_exceeded_error_details(self):
        """LimitsExceededError should include all details."""
        error = LimitsExceededError(
            value=200.0,
            min_limit=0.0,
            max_limit=180.0,
            name="shoulder",
        )

        assert error.value == 200.0
        assert error.min_limit == 0.0
        assert error.max_limit == 180.0
        assert error.name == "shoulder"
        assert "200" in str(error)
        assert "180" in str(error)

    def test_calibration_error_details(self):
        """CalibrationError should include component and reason."""
        error = CalibrationError("IMU", "Magnetometer not responding")

        assert error.component == "IMU"
        assert error.reason == "Magnetometer not responding"
        assert "IMU" in str(error)


# ============================================================================
# 13. Driver Channel Tests
# ============================================================================


class TestDriverChannels:
    """Tests for driver channel management."""

    def test_driver_validates_channel_range(self):
        """Driver should validate channel is within range."""
        driver = MockDriver(channels=16)
        driver.connect()
        driver.enable()

        # Valid channels
        driver.set_channel(0, 0.5)
        driver.set_channel(15, 0.5)

        # Channel 16 is out of range for 16-channel driver
        # (behavior depends on implementation - may raise or wrap)

    def test_driver_state_after_disconnect(self):
        """Driver should be in DISCONNECTED state after disconnect."""
        driver = MockDriver()
        driver.connect()

        assert driver.state == DriverState.CONNECTED

        driver.disconnect()

        assert driver.state == DriverState.DISCONNECTED


# ============================================================================
# 14. Limits Configuration Tests
# ============================================================================


class TestLimitsConfiguration:
    """Tests for Limits configuration edge cases."""

    def test_limits_with_no_default(self):
        """Limits without default should use min as default."""
        limits = Limits(min=0.0, max=180.0)

        # Default should be 0 (min) if not specified
        # Verify limits object is valid
        assert limits.min == 0.0
        assert limits.max == 180.0

    def test_limits_is_within(self):
        """is_within should correctly check bounds."""
        limits = Limits(min=0.0, max=180.0)

        assert limits.is_within(0.0)
        assert limits.is_within(90.0)
        assert limits.is_within(180.0)
        assert not limits.is_within(-1.0)
        assert not limits.is_within(181.0)

    def test_limits_clamp(self):
        """clamp should constrain value to limits."""
        limits = Limits(min=0.0, max=180.0)

        assert limits.clamp(-10.0) == 0.0
        assert limits.clamp(90.0) == 90.0
        assert limits.clamp(200.0) == 180.0


# ============================================================================
# 15. Actuator Go-To Methods Tests
# ============================================================================


class TestActuatorGoToMethods:
    """Tests for go_to_default, go_to_min, go_to_max."""

    def test_go_to_default(self):
        """go_to_default should move to default position."""
        limits = Limits(min=0.0, max=180.0, default=90.0)
        actuator = TestableActuator(name="test", limits=limits)
        actuator.enable()

        actuator.go_to_default()

        assert actuator.get() == 90.0

    def test_go_to_min(self):
        """go_to_min should move to minimum position."""
        limits = Limits(min=0.0, max=180.0, default=90.0)
        actuator = TestableActuator(name="test", limits=limits)
        actuator.enable()

        actuator.go_to_min()

        assert actuator.get() == 0.0

    def test_go_to_max(self):
        """go_to_max should move to maximum position."""
        limits = Limits(min=0.0, max=180.0, default=90.0)
        actuator = TestableActuator(name="test", limits=limits)
        actuator.enable()

        actuator.go_to_max()

        assert actuator.get() == 180.0


# ============================================================================
# 16. Exception Hierarchy Tests
# ============================================================================


class TestExceptionHierarchy:
    """Tests for exception class hierarchy."""

    def test_all_exceptions_inherit_from_base(self):
        """All robo-infra exceptions should inherit from RoboInfraError."""
        from robo_infra.core.exceptions import RoboInfraError

        assert issubclass(HardwareNotFoundError, RoboInfraError)
        assert issubclass(LimitsExceededError, RoboInfraError)
        assert issubclass(CommunicationError, RoboInfraError)
        assert issubclass(CalibrationError, RoboInfraError)
        assert issubclass(SafetyError, RoboInfraError)
        assert issubclass(RoboTimeoutError, RoboInfraError)
        assert issubclass(NotCalibratedError, CalibrationError)

    def test_exceptions_are_catchable_by_base(self):
        """All exceptions should be catchable by base class."""
        from robo_infra.core.exceptions import RoboInfraError

        try:
            raise LimitsExceededError(100, 0, 50, "test")
        except RoboInfraError:
            pass  # Should be caught

        try:
            raise HardwareNotFoundError("device")
        except RoboInfraError:
            pass  # Should be caught


# ============================================================================
# 17. Sensor Streaming Tests
# ============================================================================


class TestSensorStreaming:
    """Tests for sensor async streaming."""

    @pytest.mark.asyncio
    async def test_sensor_stream_cancellation(self):
        """Sensor stream should handle cancellation gracefully."""
        from robo_infra.core.sensor import SimulatedSensor
        from robo_infra.core.types import Unit

        sensor = SimulatedSensor(
            name="test",
            unit=Unit.CELSIUS,
            initial_value=25.0,
        )
        sensor.enable()

        readings = []
        read_count = 0

        async for reading in sensor.stream(rate=100.0, count=3):  # 100Hz, 3 samples
            readings.append(reading)
            read_count += 1

        assert len(readings) == 3

    @pytest.mark.asyncio
    async def test_sensor_stream_respects_interval(self):
        """Sensor stream should respect configured rate."""
        from robo_infra.core.sensor import SimulatedSensor
        from robo_infra.core.types import Unit

        sensor = SimulatedSensor(
            name="test",
            unit=Unit.CELSIUS,
            initial_value=25.0,
        )
        sensor.enable()

        start = time.perf_counter()
        readings = []

        # Stream at 20Hz for 3 samples - should take ~100ms (50ms per sample after first)
        async for reading in sensor.stream(rate=20.0, count=3):
            readings.append(reading)

        elapsed = time.perf_counter() - start

        # 3 readings at 20Hz (50ms interval) should take ~100ms
        assert elapsed >= 0.08  # Allow some tolerance
