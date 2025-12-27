"""Unit tests for safety monitoring system (Phase 7.3).

Tests for SafetyMonitor, LimitConfig, CollisionDetector and related safety infrastructure.
Target coverage: 28% → 80%+
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.safety.monitor import (
    CollisionDetector,
    LimitConfig,
    LimitStatus,
    MonitorState,
    MonitorStatus,
    SafetyLevel,
    SafetyMonitor,
    SafetyViolation,
)


# =============================================================================
# SafetyViolation Exception Tests
# =============================================================================


class TestSafetyViolation:
    """Tests for SafetyViolation exception."""

    def test_safety_violation_message(self) -> None:
        """Test SafetyViolation exception message format."""
        violation = SafetyViolation(
            component="motor1",
            metric="current",
            value=7.5,
            limit=5.0,
            unit="A",
        )
        assert "motor1" in str(violation)
        assert "current" in str(violation)
        assert "7.5" in str(violation)
        assert "5.0" in str(violation)
        assert "A" in str(violation)

    def test_safety_violation_attributes(self) -> None:
        """Test SafetyViolation stores all attributes."""
        violation = SafetyViolation(
            component="joint1",
            metric="temperature",
            value=95.0,
            limit=80.0,
            unit="°C",
        )
        assert violation.component == "joint1"
        assert violation.metric == "temperature"
        assert violation.value == 95.0
        assert violation.limit == 80.0
        assert violation.unit == "°C"


# =============================================================================
# LimitConfig Tests
# =============================================================================


class TestLimitConfig:
    """Tests for LimitConfig model."""

    def test_limit_config_defaults(self) -> None:
        """Test LimitConfig default values."""
        config = LimitConfig(component="motor1", metric="current")
        assert config.component == "motor1"
        assert config.metric == "current"
        assert config.unit == ""
        assert config.min_value is None
        assert config.max_value is None
        assert config.level == SafetyLevel.CRITICAL
        assert config.hysteresis == 0.0

    def test_limit_config_with_values(self) -> None:
        """Test LimitConfig with all values set."""
        config = LimitConfig(
            component="motor1",
            metric="current",
            unit="A",
            min_value=0.1,
            max_value=5.0,
            warning_min=0.2,
            warning_max=4.0,
            level=SafetyLevel.WARNING,
            hysteresis=0.1,
        )
        assert config.min_value == 0.1
        assert config.max_value == 5.0
        assert config.warning_min == 0.2
        assert config.warning_max == 4.0
        assert config.level == SafetyLevel.WARNING
        assert config.hysteresis == 0.1


class TestLimitStatus:
    """Tests for LimitStatus dataclass."""

    def test_limit_status_defaults(self) -> None:
        """Test LimitStatus default values."""
        config = LimitConfig(component="test", metric="current")
        status = LimitStatus(config=config)
        assert status.current_value == 0.0
        assert status.last_update == 0.0
        assert status.is_violated is False
        assert status.is_warning is False
        assert status.violation_count == 0


# =============================================================================
# Phase 5.6.1.2 - Initialization Tests
# =============================================================================


class TestSafetyMonitorInit:
    """Tests for SafetyMonitor initialization (5.6.1.2)."""

    def test_monitor_init_default(self) -> None:
        """Test SafetyMonitor default initialization."""
        monitor = SafetyMonitor()
        assert monitor.state == MonitorState.STOPPED
        assert isinstance(monitor.status(), MonitorStatus)

    def test_monitor_init_with_estop(self) -> None:
        """Test SafetyMonitor initialization with E-stop."""
        mock_estop = MagicMock()
        monitor = SafetyMonitor(estop=mock_estop)
        assert monitor._estop is mock_estop

    def test_monitor_init_with_interval(self) -> None:
        """Test SafetyMonitor initialization with custom interval."""
        monitor = SafetyMonitor(check_interval=0.05)
        assert monitor._check_interval == 0.05

    def test_monitor_add_limit(self) -> None:
        """Test adding a limit to monitor."""
        monitor = SafetyMonitor()
        config = LimitConfig(
            component="motor1",
            metric="current",
            max_value=5.0,
            unit="A",
        )
        monitor.add_limit(config)
        assert "motor1:current" in monitor._limits

    def test_monitor_add_sensor(self) -> None:
        """Test adding sensor limits via convenience methods."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.add_temperature_limit("motor1", max_temp=80.0)
        monitor.add_voltage_limit("power", max_voltage=25.0, min_voltage=10.0)

        assert "motor1:current" in monitor._limits
        assert "motor1:temperature" in monitor._limits
        assert "power:voltage" in monitor._limits

    def test_monitor_remove_limit(self) -> None:
        """Test removing a limit."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        assert "motor1:current" in monitor._limits

        monitor.remove_limit("motor1", "current")
        assert "motor1:current" not in monitor._limits

    def test_monitor_get_sensors(self) -> None:
        """Test getting all monitored sensors."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.add_temperature_limit("motor2", max_temp=80.0)

        status = monitor.status()
        assert len(status.limits) == 2


class TestMonitorThresholds:
    """Tests for monitor threshold configuration (5.6.1.2)."""

    def test_monitor_init_with_thresholds(self) -> None:
        """Test monitor with specific thresholds."""
        monitor = SafetyMonitor()
        monitor.add_current_limit(
            "motor1",
            max_current=10.0,
            min_current=0.1,
            warning_threshold=0.8,
        )

        config = monitor._limits["motor1:current"]
        assert config.max_value == 10.0
        assert config.min_value == 0.1
        assert config.warning_max == pytest.approx(8.0, abs=0.01)

    def test_monitor_temperature_threshold(self) -> None:
        """Test temperature threshold configuration."""
        monitor = SafetyMonitor()
        monitor.add_temperature_limit(
            "motor1",
            max_temp=100.0,
            warning_threshold=0.85,
        )

        config = monitor._limits["motor1:temperature"]
        assert config.max_value == 100.0
        assert config.warning_max == pytest.approx(85.0, abs=0.01)


class TestMonitorCallback:
    """Tests for monitor callback registration (5.6.1.2)."""

    def test_monitor_init_with_callback(self) -> None:
        """Test monitor with callback registration."""
        callback_called = []

        def my_callback(key: str, status: LimitStatus) -> None:
            callback_called.append((key, status))

        monitor = SafetyMonitor()
        monitor.register_callback(my_callback)
        assert len(monitor._callbacks) == 1


# =============================================================================
# Phase 5.6.1.3 - Threshold Monitoring Tests
# =============================================================================


class TestCurrentMonitoring:
    """Tests for current monitoring (5.6.1.3)."""

    def test_current_below_threshold_ok(self) -> None:
        """Test current below threshold is OK."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.update_current("motor1", 3.0)

        # Manually check limit
        config = monitor._limits["motor1:current"]
        is_violated = monitor._check_limit(3.0, config)
        assert is_violated is False

    def test_current_at_threshold_ok(self) -> None:
        """Test current exactly at threshold is OK."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)

        config = monitor._limits["motor1:current"]
        is_violated = monitor._check_limit(5.0, config)
        assert is_violated is False  # At limit, not above

    def test_current_above_threshold_alerts(self) -> None:
        """Test current above threshold triggers alert."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)

        config = monitor._limits["motor1:current"]
        is_violated = monitor._check_limit(5.1, config)
        assert is_violated is True

    def test_current_warning_threshold(self) -> None:
        """Test current warning threshold detection."""
        monitor = SafetyMonitor()
        monitor.add_current_limit(
            "motor1",
            max_current=10.0,
            warning_threshold=0.8,
        )

        config = monitor._limits["motor1:current"]
        # 8.5A is above warning (8.0) but below max (10.0)
        is_warning = monitor._check_warning(8.5, config)
        is_violated = monitor._check_limit(8.5, config)
        assert is_warning is True
        assert is_violated is False

    def test_current_critical_threshold(self) -> None:
        """Test current critical threshold triggers violation."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)

        config = monitor._limits["motor1:current"]
        # 6.0A is above max (5.0)
        is_violated = monitor._check_limit(6.0, config)
        assert is_violated is True


class TestTemperatureMonitoring:
    """Tests for temperature monitoring (5.6.1.3)."""

    def test_temperature_below_threshold_ok(self) -> None:
        """Test temperature below threshold is OK."""
        monitor = SafetyMonitor()
        monitor.add_temperature_limit("motor1", max_temp=80.0)

        config = monitor._limits["motor1:temperature"]
        is_violated = monitor._check_limit(50.0, config)
        assert is_violated is False

    def test_temperature_above_threshold_alerts(self) -> None:
        """Test temperature above threshold triggers alert."""
        monitor = SafetyMonitor()
        monitor.add_temperature_limit("motor1", max_temp=80.0)

        config = monitor._limits["motor1:temperature"]
        is_violated = monitor._check_limit(85.0, config)
        assert is_violated is True

    def test_temperature_hysteresis(self) -> None:
        """Test temperature hysteresis prevents rapid toggling."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="temperature",
                max_value=80.0,
                hysteresis=5.0,  # Must drop to 75°C to clear
            )
        )

        config = monitor._limits["motor1:temperature"]
        # After violation, need to drop below 75°C to clear
        is_cleared = monitor._check_cleared(78.0, config)
        assert is_cleared is False  # Still in hysteresis zone

        is_cleared = monitor._check_cleared(74.0, config)
        assert is_cleared is True  # Below hysteresis threshold


class TestVoltageMonitoring:
    """Tests for voltage monitoring (5.6.1.3)."""

    def test_voltage_low_alerts(self) -> None:
        """Test low voltage triggers alert."""
        monitor = SafetyMonitor()
        monitor.add_voltage_limit("power", max_voltage=25.0, min_voltage=10.0)

        config = monitor._limits["power:voltage"]
        is_violated = monitor._check_limit(8.0, config)
        assert is_violated is True  # Below min

    def test_voltage_high_alerts(self) -> None:
        """Test high voltage triggers alert."""
        monitor = SafetyMonitor()
        monitor.add_voltage_limit("power", max_voltage=25.0, min_voltage=10.0)

        config = monitor._limits["power:voltage"]
        is_violated = monitor._check_limit(28.0, config)
        assert is_violated is True  # Above max

    def test_voltage_normal_ok(self) -> None:
        """Test normal voltage is OK."""
        monitor = SafetyMonitor()
        monitor.add_voltage_limit("power", max_voltage=25.0, min_voltage=10.0)

        config = monitor._limits["power:voltage"]
        is_violated = monitor._check_limit(15.0, config)
        assert is_violated is False


# =============================================================================
# Phase 5.6.1.4 - Alert Callback Tests
# =============================================================================


class TestAlertCallbacks:
    """Tests for alert callbacks (5.6.1.4)."""

    def test_alert_callback_called_on_threshold(self) -> None:
        """Test callback is called when threshold exceeded."""
        callback_data = []

        def callback(key: str, status: LimitStatus) -> None:
            callback_data.append({"key": key, "value": status.current_value})

        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.register_callback(callback)

        # Simulate violation
        status = monitor._statuses["motor1:current"]
        status.current_value = 6.0
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.0, status)

        assert len(callback_data) == 1
        assert callback_data[0]["key"] == "motor1:current"
        assert callback_data[0]["value"] == 6.0

    def test_alert_callback_receives_sensor_info(self) -> None:
        """Test callback receives correct sensor information."""
        received_status = []

        def callback(key: str, status: LimitStatus) -> None:
            received_status.append(status)

        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.register_callback(callback)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.5, status)

        assert len(received_status) == 1
        assert received_status[0].config.component == "motor1"
        assert received_status[0].config.metric == "current"

    def test_alert_callback_receives_value_and_threshold(self) -> None:
        """Test callback receives value and threshold info."""
        received_data = []

        def callback(key: str, status: LimitStatus) -> None:
            received_data.append(
                {
                    "value": status.current_value,
                    "max": status.config.max_value,
                }
            )

        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.register_callback(callback)

        status = monitor._statuses["motor1:current"]
        status.current_value = 7.0
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 7.0, status)

        assert received_data[0]["value"] == 7.0
        assert received_data[0]["max"] == 5.0

    def test_alert_callback_receives_alert_type(self) -> None:
        """Test callback can determine alert type from status."""
        received_status = []

        def callback(key: str, status: LimitStatus) -> None:
            received_status.append(status)

        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.register_callback(callback)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.0, status)

        assert received_status[0].is_violated is True

    def test_multiple_alert_callbacks(self) -> None:
        """Test multiple callbacks are all called."""
        callback1_called = []
        callback2_called = []

        def callback1(key: str, status: LimitStatus) -> None:
            callback1_called.append(key)

        def callback2(key: str, status: LimitStatus) -> None:
            callback2_called.append(key)

        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.register_callback(callback1)
        monitor.register_callback(callback2)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.0, status)

        assert len(callback1_called) == 1
        assert len(callback2_called) == 1

    def test_alert_callback_exception_handled_gracefully(self) -> None:
        """Test callback exception doesn't crash monitor."""
        good_callback_called = []

        def bad_callback(key: str, status: LimitStatus) -> None:
            raise RuntimeError("Callback error")

        def good_callback(key: str, status: LimitStatus) -> None:
            good_callback_called.append(key)

        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.register_callback(bad_callback)
        monitor.register_callback(good_callback)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        # Should not raise, should continue to good callback
        monitor._handle_violation("motor1:current", config, 6.0, status)

        assert len(good_callback_called) == 1


# =============================================================================
# Phase 5.6.1.5 - Monitoring Loop Tests
# =============================================================================


class TestMonitoringLoop:
    """Tests for monitoring loop (5.6.1.5)."""

    def test_start_monitoring_begins_loop(self) -> None:
        """Test start() begins monitoring loop."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0)

        assert monitor.state == MonitorState.STOPPED
        monitor.start()
        assert monitor.state == MonitorState.MONITORING

        # Give thread time to start
        time.sleep(0.05)
        assert monitor._running is True

        monitor.stop()

    def test_stop_monitoring_ends_loop(self) -> None:
        """Test stop() ends monitoring loop."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.start()
        time.sleep(0.02)

        monitor.stop()
        assert monitor.state == MonitorState.STOPPED
        assert monitor._running is False

    def test_monitoring_interval_respected(self) -> None:
        """Test monitoring respects check interval."""
        monitor = SafetyMonitor(check_interval=0.05)
        monitor.add_current_limit("motor1", max_current=5.0)

        check_times = []
        original_check = monitor._check_all_limits

        def tracking_check() -> None:
            check_times.append(time.time())
            original_check()

        monitor._check_all_limits = tracking_check  # type: ignore[method-assign]
        monitor.start()
        time.sleep(0.15)
        monitor.stop()

        # Should have ~3 checks in 0.15s with 0.05s interval
        assert len(check_times) >= 2

    def test_monitoring_checks_all_sensors(self) -> None:
        """Test monitoring checks all registered sensors."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.add_temperature_limit("motor1", max_temp=80.0)
        monitor.add_voltage_limit("power", max_voltage=25.0)

        # Update all values
        monitor.update_current("motor1", 3.0)
        monitor.update_temperature("motor1", 50.0)
        monitor.update_voltage("power", 12.0)

        monitor.start()
        time.sleep(0.03)
        monitor.stop()

        # All should have values updated
        assert monitor._values.get("motor1:current") is not None
        assert monitor._values.get("motor1:temperature") is not None
        assert monitor._values.get("power:voltage") is not None

    def test_monitoring_with_simulated_sensors(self) -> None:
        """Test monitoring with simulated sensor updates."""
        callback_violations = []

        def callback(key: str, status: LimitStatus) -> None:
            callback_violations.append(key)

        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.register_callback(callback)
        monitor.start()

        # Simulate normal operation
        monitor.update_current("motor1", 3.0)
        time.sleep(0.03)

        # Simulate violation
        monitor.update_current("motor1", 6.0)
        time.sleep(0.03)

        monitor.stop()

        # Should have detected the violation
        assert "motor1:current" in callback_violations


# =============================================================================
# Phase 5.6.1.6 - Integration Tests
# =============================================================================


class TestMonitorIntegration:
    """Integration tests for SafetyMonitor (5.6.1.6)."""

    def test_monitor_triggers_estop_on_critical(self) -> None:
        """Test monitor triggers E-stop on critical violation."""
        mock_estop = MagicMock()
        monitor = SafetyMonitor(estop=mock_estop, check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0, level=SafetyLevel.CRITICAL)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 7.0, status)

        mock_estop.trigger.assert_called_once()
        call_args = mock_estop.trigger.call_args
        assert "motor1" in call_args.kwargs.get("reason", "")
        assert "current" in call_args.kwargs.get("reason", "")

    def test_monitor_does_not_estop_on_warning(self) -> None:
        """Test monitor does NOT trigger E-stop on warning level."""
        mock_estop = MagicMock()
        monitor = SafetyMonitor(estop=mock_estop, check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0, level=SafetyLevel.WARNING)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.0, status)

        # E-stop should NOT be called for WARNING level
        mock_estop.trigger.assert_not_called()

    def test_monitor_logs_warnings(self) -> None:
        """Test monitor logs warnings appropriately."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=10.0, warning_threshold=0.8)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        with patch("robo_infra.safety.monitor.logger") as mock_logger:
            monitor._handle_warning("motor1:current", config, 8.5, status)
            mock_logger.warning.assert_called()

    def test_monitor_status_report(self) -> None:
        """Test monitor status report."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.add_temperature_limit("motor2", max_temp=80.0)

        monitor.update_current("motor1", 3.0)
        monitor.update_temperature("motor2", 60.0)

        status = monitor.status()
        assert status.state == MonitorState.STOPPED
        assert len(status.limits) == 2
        assert status.total_violations == 0

    def test_monitor_tracks_violations(self) -> None:
        """Test monitor tracks total violations."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.add_current_limit("motor2", max_current=5.0)

        # Trigger violations
        status1 = monitor._statuses["motor1:current"]
        config1 = monitor._limits["motor1:current"]
        monitor._handle_violation("motor1:current", config1, 6.0, status1)

        status2 = monitor._statuses["motor2:current"]
        config2 = monitor._limits["motor2:current"]
        monitor._handle_violation("motor2:current", config2, 7.0, status2)

        full_status = monitor.status()
        assert full_status.total_violations == 2
        assert full_status.last_violation_time is not None


# =============================================================================
# CollisionDetector Tests
# =============================================================================


class TestCollisionDetector:
    """Tests for CollisionDetector."""

    def test_collision_detector_init(self) -> None:
        """Test CollisionDetector initialization."""
        detector = CollisionDetector()
        assert detector._force_threshold == 10.0
        assert detector._running is False

    def test_collision_detector_with_estop(self) -> None:
        """Test CollisionDetector with E-stop."""
        mock_estop = MagicMock()
        detector = CollisionDetector(estop=mock_estop, force_threshold=5.0)
        assert detector._force_threshold == 5.0
        assert detector._estop is mock_estop

    def test_register_force_sensor(self) -> None:
        """Test registering force sensor."""
        detector = CollisionDetector()
        mock_sensor = MagicMock()
        detector.register_force_sensor(mock_sensor)
        assert mock_sensor in detector._force_sensors

    def test_set_current_baseline(self) -> None:
        """Test setting current baseline for spike detection."""
        detector = CollisionDetector()
        detector.set_current_baseline("motor1", 2.5)
        assert detector._current_baselines["motor1"] == 2.5

    def test_collision_triggers_estop(self) -> None:
        """Test collision detection triggers E-stop."""
        mock_estop = MagicMock()
        detector = CollisionDetector(estop=mock_estop)

        detector._handle_collision("Force exceeded: 15.0N")

        mock_estop.trigger.assert_called_once()
        assert "Collision" in mock_estop.trigger.call_args.kwargs.get("reason", "")

    def test_collision_count_tracking(self) -> None:
        """Test collision count is tracked."""
        detector = CollisionDetector()

        assert detector._collision_count == 0
        detector._handle_collision("Test collision 1")
        detector._handle_collision("Test collision 2")
        assert detector._collision_count == 2

    def test_start_stop_detector(self) -> None:
        """Test starting and stopping collision detector."""
        detector = CollisionDetector(check_interval=0.01)

        assert detector._running is False
        detector.start()
        assert detector._running is True
        time.sleep(0.02)

        detector.stop()
        assert detector._running is False


# =============================================================================
# MonitorState and SafetyLevel Enum Tests
# =============================================================================


class TestEnums:
    """Tests for safety enums."""

    def test_monitor_state_values(self) -> None:
        """Test MonitorState enum values."""
        assert MonitorState.STOPPED.value == "stopped"
        assert MonitorState.MONITORING.value == "monitoring"
        assert MonitorState.WARNING.value == "warning"
        assert MonitorState.VIOLATED.value == "violated"

    def test_safety_level_values(self) -> None:
        """Test SafetyLevel enum values."""
        assert SafetyLevel.NOTICE.value == "notice"
        assert SafetyLevel.WARNING.value == "warning"
        assert SafetyLevel.CRITICAL.value == "critical"


# =============================================================================
# Edge Case Tests
# =============================================================================


class TestMonitorEdgeCases:
    """Edge case tests for SafetyMonitor."""

    def test_update_nonexistent_limit(self) -> None:
        """Test updating value for non-existent limit doesn't crash."""
        monitor = SafetyMonitor()
        # Should not raise
        monitor.update_current("nonexistent", 5.0)

    def test_remove_nonexistent_limit(self) -> None:
        """Test removing non-existent limit doesn't crash."""
        monitor = SafetyMonitor()
        # Should not raise
        monitor.remove_limit("nonexistent", "current")

    def test_start_twice(self) -> None:
        """Test starting twice doesn't create multiple threads."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.start()
        thread1 = monitor._thread

        monitor.start()  # Second start
        thread2 = monitor._thread

        assert thread1 is thread2  # Same thread

        monitor.stop()

    def test_stop_without_start(self) -> None:
        """Test stopping without starting doesn't crash."""
        monitor = SafetyMonitor()
        # Should not raise
        monitor.stop()

    def test_limit_with_no_values(self) -> None:
        """Test checking limits when no values provided."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0)
        # Don't update any values

        # Check should not crash
        monitor._check_all_limits()

    def test_estop_exception_handled(self) -> None:
        """Test E-stop exception is handled gracefully."""
        mock_estop = MagicMock()
        mock_estop.trigger.side_effect = RuntimeError("E-stop failed")

        monitor = SafetyMonitor(estop=mock_estop)
        monitor.add_current_limit("motor1", max_current=5.0)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        # Should not raise
        monitor._handle_violation("motor1:current", config, 7.0, status)

    def test_min_value_violation(self) -> None:
        """Test violation on min value."""
        monitor = SafetyMonitor()
        monitor.add_voltage_limit("power", max_voltage=25.0, min_voltage=10.0)

        config = monitor._limits["power:voltage"]
        is_violated = monitor._check_limit(5.0, config)
        assert is_violated is True  # Below min

    def test_warning_on_min_value(self) -> None:
        """Test warning near min value."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="power",
                metric="voltage",
                min_value=10.0,
                max_value=25.0,
                warning_min=12.0,
            )
        )

        config = monitor._limits["power:voltage"]
        is_warning = monitor._check_warning(11.0, config)
        assert is_warning is True  # Below warning_min


# =============================================================================
# Phase 7.3: Additional Test Classes for Coverage Improvement
# =============================================================================


class TestMonitorStateTransitions:
    """Tests for monitor state transitions."""

    def test_state_stopped_to_monitoring(self) -> None:
        """State transitions from STOPPED to MONITORING on start."""
        monitor = SafetyMonitor(check_interval=0.01)

        assert monitor.state == MonitorState.STOPPED
        monitor.start()
        assert monitor.state == MonitorState.MONITORING

        monitor.stop()
        assert monitor.state == MonitorState.STOPPED

    def test_warning_status_set_on_approach(self) -> None:
        """Warning status is set when approaching limit."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="current",
                max_value=10.0,
                warning_max=8.0,
            )
        )
        monitor.start()

        # Update with value in warning zone
        monitor.update_current("motor1", 9.0)
        time.sleep(0.05)

        status = monitor._statuses["motor1:current"]
        assert status.is_warning is True

        monitor.stop()

    def test_violated_status_set_on_exceed(self) -> None:
        """Violated status is set when limit exceeded."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.start()

        # Update with violating value
        monitor.update_current("motor1", 6.0)
        time.sleep(0.05)

        status = monitor._statuses["motor1:current"]
        assert status.is_violated is True

        monitor.stop()


class TestMonitorStatusDataclass:
    """Tests for MonitorStatus dataclass."""

    def test_monitor_status_default_values(self) -> None:
        """MonitorStatus has expected default values."""
        status = MonitorStatus()

        assert status.state == MonitorState.STOPPED
        assert status.limits == {}
        assert status.total_violations == 0
        assert status.last_violation_time is None

    def test_monitor_status_from_monitor(self) -> None:
        """MonitorStatus correctly reflects monitor state."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.add_temperature_limit("motor2", max_temp=80.0)

        status = monitor.status()

        assert len(status.limits) == 2
        assert "motor1:current" in status.limits
        assert "motor2:temperature" in status.limits


class TestLimitStatusDataclass:
    """Tests for LimitStatus dataclass."""

    def test_limit_status_update_tracking(self) -> None:
        """LimitStatus tracks current value and update time."""
        config = LimitConfig(component="test", metric="current", max_value=5.0)
        status = LimitStatus(config=config)

        # Initial values
        assert status.current_value == 0.0
        assert status.last_update == 0.0

        # Update values
        status.current_value = 3.5
        status.last_update = time.time()

        assert status.current_value == 3.5
        assert status.last_update > 0

    def test_limit_status_violation_count(self) -> None:
        """LimitStatus tracks violation count."""
        config = LimitConfig(component="test", metric="current", max_value=5.0)
        status = LimitStatus(config=config)

        assert status.violation_count == 0

        status.violation_count += 1
        assert status.violation_count == 1


class TestLimitConfigExtras:
    """Tests for LimitConfig model extras."""

    def test_config_allows_extra_fields(self) -> None:
        """LimitConfig accepts extra fields due to extra='allow'."""
        config = LimitConfig(
            component="test",
            metric="current",
            custom_field="custom_value",  # type: ignore[call-arg]
        )

        assert config.component == "test"
        assert config.custom_field == "custom_value"  # type: ignore[attr-defined]

    def test_config_is_mutable(self) -> None:
        """LimitConfig can be mutated due to frozen=False."""
        config = LimitConfig(component="original", metric="current")

        config.component = "modified"

        assert config.component == "modified"


class TestHysteresisExtended:
    """Extended tests for hysteresis behavior."""

    def test_hysteresis_on_min_value(self) -> None:
        """Hysteresis works for min value violations."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="power",
                metric="voltage",
                min_value=10.0,
                max_value=25.0,
                hysteresis=2.0,
            )
        )

        config = monitor._limits["power:voltage"]

        # After violation at min, need to rise above 12.0 to clear
        is_cleared = monitor._check_cleared(11.0, config)
        assert is_cleared is False  # Still in hysteresis zone

        is_cleared = monitor._check_cleared(13.0, config)
        assert is_cleared is True  # Above min + hysteresis

    def test_hysteresis_zero_clears_immediately(self) -> None:
        """Zero hysteresis allows immediate clearing."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="current",
                max_value=5.0,
                hysteresis=0.0,
            )
        )

        config = monitor._limits["motor1:current"]

        # Exactly at limit should clear
        is_cleared = monitor._check_cleared(5.0, config)
        assert is_cleared is True

    def test_hysteresis_large_value(self) -> None:
        """Large hysteresis requires significant return within limits."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="temperature",
                max_value=80.0,
                hysteresis=20.0,  # Must drop to 60°C
            )
        )

        config = monitor._limits["motor1:temperature"]

        # At 70°C, still in hysteresis zone
        is_cleared = monitor._check_cleared(70.0, config)
        assert is_cleared is False

        # At 55°C, cleared
        is_cleared = monitor._check_cleared(55.0, config)
        assert is_cleared is True


class TestViolationTracking:
    """Tests for detailed violation tracking."""

    def test_per_limit_violation_count(self) -> None:
        """Each limit tracks its own violation count."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.add_current_limit("motor2", max_current=5.0)

        # Trigger violation on motor1 twice
        status1 = monitor._statuses["motor1:current"]
        config1 = monitor._limits["motor1:current"]
        monitor._handle_violation("motor1:current", config1, 6.0, status1)
        status1.is_violated = False  # Reset for second violation
        monitor._handle_violation("motor1:current", config1, 7.0, status1)

        # Trigger violation on motor2 once
        status2 = monitor._statuses["motor2:current"]
        config2 = monitor._limits["motor2:current"]
        monitor._handle_violation("motor2:current", config2, 6.0, status2)

        assert status1.violation_count == 2
        assert status2.violation_count == 1
        assert monitor._total_violations == 3

    def test_last_violation_time_updated(self) -> None:
        """Last violation time is updated on each violation."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)

        before = time.time()
        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]
        monitor._handle_violation("motor1:current", config, 6.0, status)
        after = time.time()

        assert monitor._last_violation_time is not None
        assert before <= monitor._last_violation_time <= after


class TestCollisionDetectorExtended:
    """Extended tests for CollisionDetector."""

    def test_detector_custom_thresholds(self) -> None:
        """CollisionDetector accepts custom thresholds."""
        detector = CollisionDetector(
            force_threshold=5.0,
            torque_threshold=0.5,
            current_spike_threshold=3.0,
            check_interval=0.002,
        )

        assert detector._force_threshold == 5.0
        assert detector._torque_threshold == 0.5
        assert detector._current_spike_threshold == 3.0
        assert detector._check_interval == 0.002

    def test_detector_multiple_sensors(self) -> None:
        """CollisionDetector handles multiple sensors."""
        detector = CollisionDetector()

        sensor1 = MagicMock()
        sensor2 = MagicMock()
        sensor3 = MagicMock()

        detector.register_force_sensor(sensor1)
        detector.register_force_sensor(sensor2)
        detector.register_force_sensor(sensor3)

        assert len(detector._force_sensors) == 3

    def test_detector_start_twice_no_duplicate_threads(self) -> None:
        """Starting detector twice doesn't create duplicate threads."""
        detector = CollisionDetector(check_interval=0.01)

        detector.start()
        thread1 = detector._thread

        detector.start()
        thread2 = detector._thread

        assert thread1 is thread2

        detector.stop()

    def test_detector_stop_without_start(self) -> None:
        """Stopping detector without starting doesn't crash."""
        detector = CollisionDetector()

        # Should not raise
        detector.stop()

        assert detector._running is False

    def test_detector_sensor_reading_with_value_attribute(self) -> None:
        """Detector handles sensor reading with .value attribute."""
        detector = CollisionDetector(force_threshold=5.0, check_interval=0.01)
        mock_estop = MagicMock()
        detector._estop = mock_estop

        # Sensor that returns object with .value
        mock_sensor = MagicMock()
        mock_reading = MagicMock()
        mock_reading.value = 15.0  # Above threshold
        mock_sensor.read.return_value = mock_reading

        detector.register_force_sensor(mock_sensor)
        detector.start()
        time.sleep(0.03)
        detector.stop()

        # Should have triggered E-stop
        assert mock_estop.trigger.called

    def test_detector_sensor_reading_float(self) -> None:
        """Detector handles sensor returning raw float."""
        detector = CollisionDetector(force_threshold=5.0, check_interval=0.01)
        mock_estop = MagicMock()
        detector._estop = mock_estop

        # Sensor that returns raw float
        mock_sensor = MagicMock()
        mock_sensor.read.return_value = 15.0  # Above threshold

        detector.register_force_sensor(mock_sensor)
        detector.start()
        time.sleep(0.03)
        detector.stop()

        # Should have triggered E-stop
        assert mock_estop.trigger.called

    def test_detector_sensor_exception_handled(self) -> None:
        """Detector handles sensor read exceptions gracefully."""
        detector = CollisionDetector(check_interval=0.01)

        # Sensor that raises exception
        mock_sensor = MagicMock()
        mock_sensor.read.side_effect = RuntimeError("Sensor failure")

        detector.register_force_sensor(mock_sensor)
        detector.start()
        time.sleep(0.03)
        detector.stop()

        # Should not crash, just log error


class TestValueUpdateMethods:
    """Tests for value update convenience methods."""

    def test_update_current_stores_value(self) -> None:
        """update_current stores value in _values."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)

        before = time.time()
        monitor.update_current("motor1", 3.5)
        after = time.time()

        value, timestamp = monitor._values["motor1:current"]
        assert value == 3.5
        assert before <= timestamp <= after

    def test_update_temperature_stores_value(self) -> None:
        """update_temperature stores value in _values."""
        monitor = SafetyMonitor()
        monitor.add_temperature_limit("motor1", max_temp=80.0)

        monitor.update_temperature("motor1", 65.0)

        value, _ = monitor._values["motor1:temperature"]
        assert value == 65.0

    def test_update_voltage_stores_value(self) -> None:
        """update_voltage stores value in _values."""
        monitor = SafetyMonitor()
        monitor.add_voltage_limit("power", max_voltage=25.0)

        monitor.update_voltage("power", 12.5)

        value, _ = monitor._values["power:voltage"]
        assert value == 12.5

    def test_update_value_generic(self) -> None:
        """update_value works with any metric."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="joint1",
                metric="torque",
                max_value=5.0,
                unit="Nm",
            )
        )

        monitor.update_value("joint1", "torque", 2.5)

        value, _ = monitor._values["joint1:torque"]
        assert value == 2.5


class TestMonitoringLoopDetails:
    """Detailed tests for monitoring loop behavior."""

    def test_monitoring_updates_status_values(self) -> None:
        """Monitoring loop updates status current_value."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_current_limit("motor1", max_current=5.0)

        monitor.update_current("motor1", 3.5)
        monitor.start()
        time.sleep(0.03)
        monitor.stop()

        status = monitor._statuses["motor1:current"]
        assert status.current_value == 3.5

    def test_monitoring_clears_violations_with_good_values(self) -> None:
        """Monitoring clears violations when values return to normal."""
        monitor = SafetyMonitor(check_interval=0.01)
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="current",
                max_value=5.0,
                hysteresis=0.5,
            )
        )

        monitor.start()

        # Trigger violation
        monitor.update_current("motor1", 6.0)
        time.sleep(0.03)

        status = monitor._statuses["motor1:current"]
        assert status.is_violated is True

        # Return to safe value (below max - hysteresis)
        monitor.update_current("motor1", 4.0)
        time.sleep(0.03)

        # Should be cleared
        assert status.is_violated is False

        monitor.stop()


class TestSafetyLevelBehavior:
    """Tests for different safety level behaviors."""

    def test_notice_level_no_estop(self) -> None:
        """NOTICE level does not trigger E-stop."""
        mock_estop = MagicMock()
        monitor = SafetyMonitor(estop=mock_estop)
        monitor.add_current_limit("motor1", max_current=5.0, level=SafetyLevel.NOTICE)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.0, status)

        mock_estop.trigger.assert_not_called()

    def test_warning_level_no_estop(self) -> None:
        """WARNING level does not trigger E-stop."""
        mock_estop = MagicMock()
        monitor = SafetyMonitor(estop=mock_estop)
        monitor.add_current_limit("motor1", max_current=5.0, level=SafetyLevel.WARNING)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.0, status)

        mock_estop.trigger.assert_not_called()

    def test_critical_level_triggers_estop(self) -> None:
        """CRITICAL level triggers E-stop."""
        mock_estop = MagicMock()
        monitor = SafetyMonitor(estop=mock_estop)
        monitor.add_current_limit("motor1", max_current=5.0, level=SafetyLevel.CRITICAL)

        status = monitor._statuses["motor1:current"]
        config = monitor._limits["motor1:current"]

        monitor._handle_violation("motor1:current", config, 6.0, status)

        mock_estop.trigger.assert_called_once()


class TestWarningConditions:
    """Tests for warning condition detection."""

    def test_no_warning_in_safe_zone(self) -> None:
        """No warning when value is well within limits."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="current",
                max_value=10.0,
                warning_max=8.0,
            )
        )

        config = monitor._limits["motor1:current"]
        is_warning = monitor._check_warning(5.0, config)
        assert is_warning is False

    def test_warning_above_warning_max(self) -> None:
        """Warning when value exceeds warning_max."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="current",
                max_value=10.0,
                warning_max=8.0,
            )
        )

        config = monitor._limits["motor1:current"]
        is_warning = monitor._check_warning(9.0, config)
        assert is_warning is True

    def test_warning_below_warning_min(self) -> None:
        """Warning when value falls below warning_min."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="power",
                metric="voltage",
                min_value=10.0,
                max_value=25.0,
                warning_min=12.0,
            )
        )

        config = monitor._limits["power:voltage"]
        is_warning = monitor._check_warning(11.0, config)
        assert is_warning is True

    def test_no_warning_when_not_configured(self) -> None:
        """No warning when warning thresholds not set."""
        monitor = SafetyMonitor()
        monitor.add_limit(
            LimitConfig(
                component="motor1",
                metric="current",
                max_value=10.0,
                # No warning_max set
            )
        )

        config = monitor._limits["motor1:current"]
        is_warning = monitor._check_warning(9.0, config)
        assert is_warning is False


class TestLimitRemoval:
    """Tests for limit removal behavior."""

    def test_remove_limit_clears_status(self) -> None:
        """Removing limit also removes status."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)

        assert "motor1:current" in monitor._statuses
        assert "motor1:current" in monitor._limits

        monitor.remove_limit("motor1", "current")

        assert "motor1:current" not in monitor._statuses
        assert "motor1:current" not in monitor._limits

    def test_remove_limit_clears_values(self) -> None:
        """Removing limit also removes stored values."""
        monitor = SafetyMonitor()
        monitor.add_current_limit("motor1", max_current=5.0)
        monitor.update_current("motor1", 3.0)

        assert "motor1:current" in monitor._values

        monitor.remove_limit("motor1", "current")

        assert "motor1:current" not in monitor._values
