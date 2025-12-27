"""Unit tests for robo_infra.safety.watchdog module.

Phase 5.6.2: Watchdog Tests
Target coverage: 59% → 85%+

Tests cover:
- WatchdogError exception
- WatchdogState enum
- WatchdogConfig model
- WatchdogStatus dataclass
- Watchdog initialization and lifecycle
- Heartbeat (feed) operations
- Timeout handling
- Start/Stop/Pause/Resume
- Recovery after timeout
- ControlLoopTimer
"""

from __future__ import annotations

import logging
import time
from unittest.mock import MagicMock

import pytest

from robo_infra.safety.watchdog import (
    ControlLoopTimer,
    Watchdog,
    WatchdogConfig,
    WatchdogError,
    WatchdogState,
    WatchdogStatus,
)


# =============================================================================
# WatchdogError Tests
# =============================================================================


class TestWatchdogError:
    """Tests for WatchdogError exception."""

    def test_error_has_required_attributes(self) -> None:
        """Error stores name, timeout, and last_feed_age."""
        error = WatchdogError(name="test", timeout=0.1, last_feed_age=0.15)

        assert error.name == "test"
        assert error.timeout == 0.1
        assert error.last_feed_age == 0.15

    def test_error_message_format(self) -> None:
        """Error message includes timing details."""
        error = WatchdogError(name="control_loop", timeout=0.1, last_feed_age=0.153)

        assert "control_loop" in str(error)
        assert "0.153s" in str(error)
        assert "0.1s" in str(error)

    def test_error_inherits_safety_error(self) -> None:
        """WatchdogError inherits from SafetyError."""
        from robo_infra.core.exceptions import SafetyError

        error = WatchdogError(name="test", timeout=0.1, last_feed_age=0.2)
        assert isinstance(error, SafetyError)


# =============================================================================
# WatchdogState Tests
# =============================================================================


class TestWatchdogState:
    """Tests for WatchdogState enum."""

    def test_all_states_exist(self) -> None:
        """All expected states are defined."""
        assert WatchdogState.STOPPED.value == "stopped"
        assert WatchdogState.ARMED.value == "armed"
        assert WatchdogState.TRIGGERED.value == "triggered"
        assert WatchdogState.PAUSED.value == "paused"

    def test_state_count(self) -> None:
        """Only expected states exist."""
        assert len(WatchdogState) == 4


# =============================================================================
# WatchdogConfig Tests
# =============================================================================


class TestWatchdogConfig:
    """Tests for WatchdogConfig model."""

    def test_default_config(self) -> None:
        """Default configuration values."""
        config = WatchdogConfig()

        assert config.name == "Watchdog"
        assert config.timeout == 0.1
        assert config.trigger_estop is True
        assert config.auto_start is False
        assert config.warn_threshold == 0.8

    def test_custom_config(self) -> None:
        """Custom configuration values."""
        config = WatchdogConfig(
            name="ControlLoop",
            timeout=0.5,
            trigger_estop=False,
            auto_start=True,
            warn_threshold=0.9,
        )

        assert config.name == "ControlLoop"
        assert config.timeout == 0.5
        assert config.trigger_estop is False
        assert config.auto_start is True
        assert config.warn_threshold == 0.9

    def test_timeout_must_be_positive(self) -> None:
        """Timeout must be greater than zero."""
        with pytest.raises(ValueError):
            WatchdogConfig(timeout=0)

        with pytest.raises(ValueError):
            WatchdogConfig(timeout=-0.1)

    def test_warn_threshold_range(self) -> None:
        """Warn threshold must be between 0 and 1."""
        # Valid edge cases
        WatchdogConfig(warn_threshold=0.0)
        WatchdogConfig(warn_threshold=1.0)

        # Invalid values
        with pytest.raises(ValueError):
            WatchdogConfig(warn_threshold=-0.1)

        with pytest.raises(ValueError):
            WatchdogConfig(warn_threshold=1.1)


# =============================================================================
# WatchdogStatus Tests
# =============================================================================


class TestWatchdogStatus:
    """Tests for WatchdogStatus dataclass."""

    def test_default_status(self) -> None:
        """Default status values."""
        status = WatchdogStatus()

        assert status.state == WatchdogState.STOPPED
        assert status.timeout == 0.1
        assert status.last_feed_time == 0.0
        assert status.feed_count == 0
        assert status.timeout_count == 0
        assert status.last_feed_age == 0.0

    def test_custom_status(self) -> None:
        """Custom status values."""
        status = WatchdogStatus(
            state=WatchdogState.ARMED,
            timeout=0.5,
            last_feed_time=1234567890.0,
            feed_count=100,
            timeout_count=2,
            last_feed_age=0.05,
        )

        assert status.state == WatchdogState.ARMED
        assert status.timeout == 0.5
        assert status.feed_count == 100
        assert status.timeout_count == 2


# =============================================================================
# Watchdog Initialization Tests
# =============================================================================


class TestWatchdogInit:
    """Tests for Watchdog initialization."""

    def test_init_default_timeout(self) -> None:
        """Default timeout is 0.1 seconds."""
        watchdog = Watchdog()

        assert watchdog.timeout == 0.1
        assert watchdog.state == WatchdogState.STOPPED
        assert watchdog.is_armed is False

    def test_init_custom_timeout(self) -> None:
        """Custom timeout via constructor."""
        watchdog = Watchdog(timeout=0.5)

        assert watchdog.timeout == 0.5

    def test_init_with_config(self) -> None:
        """Initialize with full config object."""
        config = WatchdogConfig(name="TestWatchdog", timeout=0.2)
        watchdog = Watchdog(config=config)

        assert watchdog.timeout == 0.2

    def test_config_overrides_timeout(self) -> None:
        """Config object overrides timeout argument."""
        config = WatchdogConfig(timeout=0.3)
        watchdog = Watchdog(timeout=0.1, config=config)

        # Config should take precedence
        assert watchdog.timeout == 0.3

    def test_init_with_estop(self) -> None:
        """Initialize with E-stop reference."""
        mock_estop = MagicMock()
        watchdog = Watchdog(estop=mock_estop)

        # E-stop is stored internally
        assert watchdog._estop is mock_estop

    def test_auto_start_disabled_by_default(self) -> None:
        """Watchdog does not auto-start by default."""
        watchdog = Watchdog()

        assert watchdog.state == WatchdogState.STOPPED
        assert not watchdog.is_armed

    def test_auto_start_enabled(self) -> None:
        """Watchdog auto-starts when configured."""
        config = WatchdogConfig(auto_start=True, timeout=0.5)
        watchdog = Watchdog(config=config)

        try:
            assert watchdog.state == WatchdogState.ARMED
            assert watchdog.is_armed
        finally:
            watchdog.stop()


# =============================================================================
# Heartbeat (Feed) Tests
# =============================================================================


class TestHeartbeat:
    """Tests for watchdog heartbeat (feed) operation."""

    def test_feed_increments_count(self) -> None:
        """Each feed increments the feed count."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            initial_status = watchdog.status()
            assert initial_status.feed_count == 0

            watchdog.feed()
            assert watchdog.status().feed_count == 1

            watchdog.feed()
            watchdog.feed()
            assert watchdog.status().feed_count == 3
        finally:
            watchdog.stop()

    def test_feed_updates_last_feed_time(self) -> None:
        """Feed updates the last feed time."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            before = time.time()
            watchdog.feed()
            after = time.time()

            status = watchdog.status()
            assert before <= status.last_feed_time <= after
        finally:
            watchdog.stop()

    def test_feed_resets_age(self) -> None:
        """Feed resets the last feed age."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            # Let some time pass
            time.sleep(0.05)
            age_before = watchdog.status().last_feed_age
            assert age_before > 0.04

            # Feed should reset age
            watchdog.feed()
            age_after = watchdog.status().last_feed_age
            assert age_after < 0.01
        finally:
            watchdog.stop()

    def test_multiple_rapid_feeds(self) -> None:
        """Multiple rapid feeds are all counted."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            for _ in range(100):
                watchdog.feed()

            assert watchdog.status().feed_count == 100
        finally:
            watchdog.stop()

    def test_feed_warns_when_late(self, caplog: pytest.LogCaptureFixture) -> None:
        """Feed logs warning when close to timeout."""
        config = WatchdogConfig(timeout=0.1, warn_threshold=0.5)
        watchdog = Watchdog(config=config)
        watchdog.start()

        try:
            # Wait past warn threshold (50% of 0.1s = 0.05s)
            time.sleep(0.06)

            with caplog.at_level(logging.WARNING):
                watchdog.feed()

            assert "feed late" in caplog.text.lower()
        finally:
            watchdog.stop()


# =============================================================================
# Timeout Tests
# =============================================================================


class TestTimeout:
    """Tests for watchdog timeout behavior."""

    def test_timeout_changes_state(self) -> None:
        """Timeout changes state to TRIGGERED."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            # Don't feed, let timeout occur
            time.sleep(0.15)

            assert watchdog.state == WatchdogState.TRIGGERED
        finally:
            watchdog.stop()

    def test_timeout_increments_count(self) -> None:
        """Timeout increments timeout count."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            assert watchdog.status().timeout_count == 0

            # Let timeout occur
            time.sleep(0.15)

            assert watchdog.status().timeout_count == 1
        finally:
            watchdog.stop()

    def test_timeout_triggers_estop(self) -> None:
        """Timeout triggers E-stop when configured."""
        mock_estop = MagicMock()
        watchdog = Watchdog(timeout=0.05, estop=mock_estop)
        watchdog.start()

        try:
            time.sleep(0.15)

            mock_estop.trigger.assert_called_once()
            call_kwargs = mock_estop.trigger.call_args.kwargs
            assert "reason" in call_kwargs
            assert "watchdog" in call_kwargs["triggered_by"].lower()
        finally:
            watchdog.stop()

    def test_timeout_does_not_trigger_estop_when_disabled(self) -> None:
        """Timeout does not trigger E-stop when disabled in config."""
        mock_estop = MagicMock()
        config = WatchdogConfig(timeout=0.05, trigger_estop=False)
        watchdog = Watchdog(estop=mock_estop, config=config)
        watchdog.start()

        try:
            time.sleep(0.15)

            mock_estop.trigger.assert_not_called()
        finally:
            watchdog.stop()

    def test_timeout_calls_callbacks(self) -> None:
        """Timeout calls registered callbacks."""
        watchdog = Watchdog(timeout=0.05)
        callback = MagicMock()
        watchdog.register_callback(callback)
        watchdog.start()

        try:
            time.sleep(0.15)

            callback.assert_called_once()
            # Callback receives WatchdogStatus
            status_arg = callback.call_args[0][0]
            assert isinstance(status_arg, WatchdogStatus)
        finally:
            watchdog.stop()

    def test_timeout_logs_critical(self, caplog: pytest.LogCaptureFixture) -> None:
        """Timeout logs critical message."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            with caplog.at_level(logging.CRITICAL):
                time.sleep(0.15)

            assert "watchdog" in caplog.text.lower()
            assert "timeout" in caplog.text.lower()
        finally:
            watchdog.stop()

    def test_timeout_only_triggers_once(self) -> None:
        """Timeout only triggers once per timeout event."""
        mock_estop = MagicMock()
        callback = MagicMock()
        watchdog = Watchdog(timeout=0.05, estop=mock_estop)
        watchdog.register_callback(callback)
        watchdog.start()

        try:
            # Let timeout occur and wait more
            time.sleep(0.2)

            # Should only be called once (state changes to TRIGGERED, stops monitoring)
            assert mock_estop.trigger.call_count == 1
            assert callback.call_count == 1
        finally:
            watchdog.stop()

    def test_estop_trigger_failure_doesnt_break_watchdog(
        self, caplog: pytest.LogCaptureFixture
    ) -> None:
        """E-stop trigger failure is handled gracefully."""
        mock_estop = MagicMock()
        mock_estop.trigger.side_effect = Exception("E-stop broken")
        watchdog = Watchdog(timeout=0.05, estop=mock_estop)
        watchdog.start()

        try:
            with caplog.at_level(logging.CRITICAL):
                time.sleep(0.15)

            # E-stop failure logged but watchdog still triggered
            assert watchdog.state == WatchdogState.TRIGGERED
            assert "E-stop" in caplog.text or "e-stop" in caplog.text.lower()
        finally:
            watchdog.stop()

    def test_callback_exception_doesnt_break_others(self) -> None:
        """Callback exception doesn't prevent other callbacks."""
        watchdog = Watchdog(timeout=0.05)

        bad_callback = MagicMock(side_effect=Exception("callback error"))
        good_callback = MagicMock()

        watchdog.register_callback(bad_callback)
        watchdog.register_callback(good_callback)
        watchdog.start()

        try:
            time.sleep(0.15)

            # Both callbacks called despite first one failing
            bad_callback.assert_called_once()
            good_callback.assert_called_once()
        finally:
            watchdog.stop()


# =============================================================================
# Start/Stop/Pause/Resume Tests
# =============================================================================


class TestStartStop:
    """Tests for watchdog start/stop lifecycle."""

    def test_start_arms_watchdog(self) -> None:
        """Start transitions to ARMED state."""
        watchdog = Watchdog(timeout=1.0)

        assert watchdog.state == WatchdogState.STOPPED

        watchdog.start()
        try:
            assert watchdog.state == WatchdogState.ARMED
            assert watchdog.is_armed
        finally:
            watchdog.stop()

    def test_stop_stops_watchdog(self) -> None:
        """Stop transitions to STOPPED state."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()
        watchdog.stop()

        assert watchdog.state == WatchdogState.STOPPED
        assert not watchdog.is_armed

    def test_double_start_ignored(self) -> None:
        """Starting an already-started watchdog is no-op."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            thread1 = watchdog._thread
            watchdog.start()  # Second start
            thread2 = watchdog._thread

            # Same thread, no new one created
            assert thread1 is thread2
        finally:
            watchdog.stop()

    def test_stop_before_start_safe(self) -> None:
        """Stop before start is safe (no-op)."""
        watchdog = Watchdog(timeout=1.0)

        # Should not raise
        watchdog.stop()

        assert watchdog.state == WatchdogState.STOPPED

    def test_restart_works(self) -> None:
        """Stop then start works correctly."""
        watchdog = Watchdog(timeout=1.0)

        watchdog.start()
        assert watchdog.is_armed

        watchdog.stop()
        assert not watchdog.is_armed

        watchdog.start()
        try:
            assert watchdog.is_armed
        finally:
            watchdog.stop()

    def test_pause_stops_monitoring(self) -> None:
        """Pause changes state to PAUSED."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            watchdog.pause()
            assert watchdog.state == WatchdogState.PAUSED
            assert not watchdog.is_armed
        finally:
            watchdog.stop()

    def test_resume_restarts_monitoring(self) -> None:
        """Resume changes state back to ARMED."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            watchdog.pause()
            watchdog.resume()

            assert watchdog.state == WatchdogState.ARMED
            assert watchdog.is_armed
        finally:
            watchdog.stop()

    def test_pause_only_works_when_armed(self) -> None:
        """Pause only works from ARMED state."""
        watchdog = Watchdog(timeout=1.0)

        # When stopped, pause does nothing
        watchdog.pause()
        assert watchdog.state == WatchdogState.STOPPED

    def test_resume_only_works_when_paused(self) -> None:
        """Resume only works from PAUSED state."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            # When armed, resume does nothing
            watchdog.resume()
            assert watchdog.state == WatchdogState.ARMED
        finally:
            watchdog.stop()

    def test_resume_resets_feed_time(self) -> None:
        """Resume resets the last feed time."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            time.sleep(0.1)
            watchdog.pause()

            # Record age during pause
            time.sleep(0.1)

            before_resume = time.time()
            watchdog.resume()
            after_resume = time.time()

            status = watchdog.status()
            assert before_resume <= status.last_feed_time <= after_resume
        finally:
            watchdog.stop()

    def test_paused_watchdog_does_not_timeout(self) -> None:
        """Watchdog does not timeout while paused."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            watchdog.pause()
            time.sleep(0.15)  # Would timeout if armed

            # Still paused, not triggered
            assert watchdog.state == WatchdogState.PAUSED
        finally:
            watchdog.stop()


# =============================================================================
# Recovery Tests
# =============================================================================


class TestRecovery:
    """Tests for watchdog recovery after timeout."""

    def test_reset_clears_triggered_state(self) -> None:
        """Reset clears triggered state and rearms."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            # Trigger timeout
            time.sleep(0.15)
            assert watchdog.state == WatchdogState.TRIGGERED

            # Reset
            watchdog.reset()
            assert watchdog.state == WatchdogState.ARMED
        finally:
            watchdog.stop()

    def test_reset_resets_feed_time(self) -> None:
        """Reset resets the last feed time."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            time.sleep(0.15)

            before = time.time()
            watchdog.reset()
            after = time.time()

            status = watchdog.status()
            assert before <= status.last_feed_time <= after
        finally:
            watchdog.stop()

    def test_reset_only_works_when_triggered(self) -> None:
        """Reset only works from TRIGGERED state."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            # When armed, reset does nothing
            watchdog.reset()
            assert watchdog.state == WatchdogState.ARMED
        finally:
            watchdog.stop()

    def test_can_timeout_again_after_reset(self) -> None:
        """Watchdog can timeout again after reset."""
        callback = MagicMock()
        watchdog = Watchdog(timeout=0.05)
        watchdog.register_callback(callback)
        watchdog.start()

        try:
            # First timeout
            time.sleep(0.15)
            assert callback.call_count == 1

            # Reset
            watchdog.reset()

            # Second timeout
            time.sleep(0.15)
            assert callback.call_count == 2
        finally:
            watchdog.stop()


# =============================================================================
# Status Tests
# =============================================================================


class TestStatus:
    """Tests for watchdog status reporting."""

    def test_status_returns_watchdog_status(self) -> None:
        """Status method returns WatchdogStatus."""
        watchdog = Watchdog(timeout=0.5)

        status = watchdog.status()
        assert isinstance(status, WatchdogStatus)

    def test_status_reflects_current_state(self) -> None:
        """Status reflects current watchdog state."""
        watchdog = Watchdog(timeout=1.0)

        assert watchdog.status().state == WatchdogState.STOPPED

        watchdog.start()
        try:
            assert watchdog.status().state == WatchdogState.ARMED
        finally:
            watchdog.stop()

    def test_status_reflects_feed_count(self) -> None:
        """Status shows correct feed count."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            for i in range(5):
                watchdog.feed()
                assert watchdog.status().feed_count == i + 1
        finally:
            watchdog.stop()

    def test_status_reflects_timeout_config(self) -> None:
        """Status shows configured timeout."""
        watchdog = Watchdog(timeout=0.42)

        status = watchdog.status()
        assert status.timeout == 0.42


# =============================================================================
# Callback Registration Tests
# =============================================================================


class TestCallbackRegistration:
    """Tests for callback registration."""

    def test_register_single_callback(self) -> None:
        """Single callback can be registered."""
        watchdog = Watchdog(timeout=0.05)
        callback = MagicMock()
        watchdog.register_callback(callback)
        watchdog.start()

        try:
            time.sleep(0.15)
            callback.assert_called_once()
        finally:
            watchdog.stop()

    def test_register_multiple_callbacks(self) -> None:
        """Multiple callbacks can be registered."""
        watchdog = Watchdog(timeout=0.05)
        callbacks = [MagicMock() for _ in range(3)]

        for cb in callbacks:
            watchdog.register_callback(cb)

        watchdog.start()

        try:
            time.sleep(0.15)

            for cb in callbacks:
                cb.assert_called_once()
        finally:
            watchdog.stop()


# =============================================================================
# ControlLoopTimer Tests
# =============================================================================


class TestControlLoopTimer:
    """Tests for ControlLoopTimer."""

    def test_init_default_frequency(self) -> None:
        """Default frequency is 50Hz."""
        timer = ControlLoopTimer()

        assert timer.frequency == 50.0
        assert timer.period == pytest.approx(0.02, rel=0.01)

    def test_init_custom_frequency(self) -> None:
        """Custom frequency is accepted."""
        timer = ControlLoopTimer(frequency=100.0)

        assert timer.frequency == 100.0
        assert timer.period == 0.01

    def test_init_with_watchdog(self) -> None:
        """Timer can be initialized with watchdog."""
        watchdog = Watchdog(timeout=1.0)
        timer = ControlLoopTimer(frequency=50.0, watchdog=watchdog)

        assert timer._watchdog is watchdog

    def test_start_stop(self) -> None:
        """Timer can be started and stopped."""
        timer = ControlLoopTimer(frequency=100.0)

        timer.start()
        timer.stop()

        # No exception = success

    def test_cycle_count_increments(self) -> None:
        """Cycle count increments after each cycle."""
        timer = ControlLoopTimer(frequency=100.0)
        timer.start()

        for i in range(5):
            timer.begin_cycle()
            timer.end_cycle()
            assert timer.cycle_count == i + 1

        timer.stop()

    def test_end_cycle_returns_elapsed_time(self) -> None:
        """End cycle returns elapsed time."""
        timer = ControlLoopTimer(frequency=100.0)
        timer.start()

        timer.begin_cycle()
        time.sleep(0.005)  # 5ms
        elapsed = timer.end_cycle()

        assert elapsed >= 0.004  # At least 4ms (timing tolerance)

        timer.stop()

    def test_overrun_detection(self) -> None:
        """Overrun is detected when cycle exceeds period."""
        timer = ControlLoopTimer(frequency=100.0, warn_on_overrun=True)  # 10ms period
        timer.start()

        assert timer.overrun_count == 0

        timer.begin_cycle()
        time.sleep(0.015)  # 15ms > 10ms period
        timer.end_cycle()

        assert timer.overrun_count == 1

        timer.stop()

    def test_overrun_ratio(self) -> None:
        """Overrun ratio is calculated correctly."""
        timer = ControlLoopTimer(frequency=100.0)
        timer.start()

        # 2 normal cycles
        for _ in range(2):
            timer.begin_cycle()
            timer.end_cycle()

        # 1 overrun cycle
        timer.begin_cycle()
        time.sleep(0.015)
        timer.end_cycle()

        # 1/3 cycles overran
        assert timer.overrun_ratio == pytest.approx(1 / 3, rel=0.01)

        timer.stop()

    def test_reset_stats(self) -> None:
        """Stats can be reset."""
        timer = ControlLoopTimer(frequency=100.0)
        timer.start()

        for _ in range(5):
            timer.begin_cycle()
            timer.end_cycle()

        assert timer.cycle_count == 5

        timer.reset_stats()

        assert timer.cycle_count == 0
        assert timer.overrun_count == 0

        timer.stop()

    def test_feeds_watchdog_on_cycle(self) -> None:
        """Timer feeds watchdog on each cycle."""
        watchdog = Watchdog(timeout=1.0)
        timer = ControlLoopTimer(frequency=100.0, watchdog=watchdog)
        watchdog.start()
        timer.start()

        try:
            initial = watchdog.status().feed_count

            timer.begin_cycle()
            timer.end_cycle()

            assert watchdog.status().feed_count == initial + 1
        finally:
            timer.stop()
            watchdog.stop()

    def test_overrun_warning_logged(self, caplog: pytest.LogCaptureFixture) -> None:
        """Overrun logs warning when enabled."""
        timer = ControlLoopTimer(frequency=100.0, warn_on_overrun=True)
        timer.start()

        with caplog.at_level(logging.WARNING):
            timer.begin_cycle()
            time.sleep(0.015)
            timer.end_cycle()

        assert "overrun" in caplog.text.lower()

        timer.stop()

    def test_severe_overrun_logged_as_error(self, caplog: pytest.LogCaptureFixture) -> None:
        """Severe overrun (2x target) logs error."""
        timer = ControlLoopTimer(frequency=100.0, warn_on_overrun=True, max_overrun_ratio=2.0)
        timer.start()

        with caplog.at_level(logging.ERROR):
            timer.begin_cycle()
            time.sleep(0.025)  # 2.5x target
            timer.end_cycle()

        assert "severe" in caplog.text.lower()

        timer.stop()

    def test_empty_cycle_count_has_zero_overrun_ratio(self) -> None:
        """Zero cycles means zero overrun ratio (no division by zero)."""
        timer = ControlLoopTimer()

        assert timer.overrun_ratio == 0.0


# =============================================================================
# Thread Safety Tests
# =============================================================================


class TestThreadSafety:
    """Tests for thread safety."""

    def test_concurrent_feeds(self) -> None:
        """Multiple threads can feed concurrently."""
        import threading

        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:

            def feed_loop() -> None:
                for _ in range(100):
                    watchdog.feed()
                    time.sleep(0.001)

            threads = [threading.Thread(target=feed_loop) for _ in range(5)]

            for t in threads:
                t.start()
            for t in threads:
                t.join()

            # Should have 500 feeds
            assert watchdog.status().feed_count == 500
        finally:
            watchdog.stop()

    def test_status_thread_safe(self) -> None:
        """Status can be read while feeding."""
        import threading

        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        stop_event = threading.Event()
        errors: list[Exception] = []

        try:

            def feed_loop() -> None:
                while not stop_event.is_set():
                    watchdog.feed()
                    time.sleep(0.001)

            def status_loop() -> None:
                try:
                    while not stop_event.is_set():
                        status = watchdog.status()
                        # Verify status is valid
                        assert isinstance(status.feed_count, int)
                        assert status.feed_count >= 0
                except Exception as e:
                    errors.append(e)

            feed_thread = threading.Thread(target=feed_loop)
            status_thread = threading.Thread(target=status_loop)

            feed_thread.start()
            status_thread.start()

            time.sleep(0.1)
            stop_event.set()

            feed_thread.join()
            status_thread.join()

            assert len(errors) == 0
        finally:
            watchdog.stop()


# =============================================================================
# Edge Case Tests
# =============================================================================


class TestEdgeCases:
    """Tests for edge cases and error conditions."""

    def test_very_short_timeout(self) -> None:
        """Very short timeout works correctly."""
        watchdog = Watchdog(timeout=0.01)  # 10ms
        watchdog.start()

        try:
            # Feed rapidly
            for _ in range(20):
                watchdog.feed()
                time.sleep(0.003)  # 3ms < 10ms timeout

            assert watchdog.state == WatchdogState.ARMED
        finally:
            watchdog.stop()

    def test_very_long_timeout(self) -> None:
        """Long timeout works correctly."""
        watchdog = Watchdog(timeout=10.0)  # 10 seconds
        watchdog.start()

        try:
            # Short sleep shouldn't trigger
            time.sleep(0.1)
            assert watchdog.state == WatchdogState.ARMED
        finally:
            watchdog.stop()

    def test_zero_warn_threshold(self) -> None:
        """Zero warn threshold means always warn."""
        config = WatchdogConfig(timeout=0.1, warn_threshold=0.0)
        watchdog = Watchdog(config=config)
        watchdog.start()

        try:
            # Any delay should warn
            time.sleep(0.001)
            # Not testing log output, just that it doesn't crash
            watchdog.feed()
        finally:
            watchdog.stop()

    def test_one_warn_threshold(self) -> None:
        """Warn threshold of 1.0 means warn at timeout."""
        config = WatchdogConfig(timeout=0.1, warn_threshold=1.0)
        watchdog = Watchdog(config=config)
        watchdog.start()

        try:
            # Normal feed, no warning until timeout
            watchdog.feed()
        finally:
            watchdog.stop()

    def test_stop_during_timeout_handling(self) -> None:
        """Stop during timeout handling is safe."""
        mock_estop = MagicMock()

        def slow_trigger(*args: object, **kwargs: object) -> None:
            time.sleep(0.05)

        mock_estop.trigger.side_effect = slow_trigger
        watchdog = Watchdog(timeout=0.02, estop=mock_estop)
        watchdog.start()

        # Let timeout start
        time.sleep(0.08)

        # Stop during potential trigger
        watchdog.stop()

        assert watchdog.state == WatchdogState.STOPPED


# =============================================================================
# Phase 7.4: Additional Tests for Coverage
# =============================================================================


class TestWatchdogErrorExtended:
    """Extended tests for WatchdogError exception."""

    def test_error_action_taken_attribute(self) -> None:
        """WatchdogError has action_taken attribute."""
        error = WatchdogError(name="test", timeout=0.1, last_feed_age=0.2)

        assert hasattr(error, "action_taken")
        assert error.action_taken == "E-stop triggered"

    def test_error_with_zero_last_feed_age(self) -> None:
        """WatchdogError handles zero last_feed_age."""
        error = WatchdogError(name="test", timeout=0.1, last_feed_age=0.0)

        assert error.last_feed_age == 0.0
        assert "0.000s" in str(error)

    def test_error_with_large_values(self) -> None:
        """WatchdogError handles large timeout values."""
        error = WatchdogError(name="test", timeout=60.0, last_feed_age=120.5)

        assert error.timeout == 60.0
        assert error.last_feed_age == 120.5
        assert "60" in str(error)


class TestWatchdogConfigExtended:
    """Extended tests for WatchdogConfig model."""

    def test_config_allows_extra_attributes(self) -> None:
        """Config accepts extra attributes due to extra='allow'."""
        config = WatchdogConfig(name="test", custom_field="value")

        assert config.custom_field == "value"

    def test_config_model_config_not_frozen(self) -> None:
        """Config is not frozen (can be modified)."""
        config = WatchdogConfig()

        config.timeout = 0.5
        assert config.timeout == 0.5

    def test_config_with_very_small_timeout(self) -> None:
        """Config accepts very small timeout values."""
        config = WatchdogConfig(timeout=0.001)

        assert config.timeout == 0.001

    def test_config_with_large_timeout(self) -> None:
        """Config accepts large timeout values."""
        config = WatchdogConfig(timeout=3600.0)

        assert config.timeout == 3600.0


class TestWatchdogStatusExtended:
    """Extended tests for WatchdogStatus dataclass."""

    def test_status_all_states(self) -> None:
        """Status can have any WatchdogState."""
        for state in WatchdogState:
            status = WatchdogStatus(state=state)
            assert status.state == state

    def test_status_timeout_count_tracking(self) -> None:
        """Status tracks multiple timeouts."""
        status = WatchdogStatus(timeout_count=5)

        assert status.timeout_count == 5

    def test_status_large_feed_count(self) -> None:
        """Status handles large feed counts."""
        status = WatchdogStatus(feed_count=1_000_000)

        assert status.feed_count == 1_000_000


class TestWatchdogInitExtended:
    """Extended initialization tests for Watchdog."""

    def test_init_with_none_timeout_uses_default(self) -> None:
        """None timeout uses default 0.1s."""
        watchdog = Watchdog(timeout=None)

        assert watchdog.timeout == 0.1

    def test_init_config_name_propagates(self) -> None:
        """Config name is used in watchdog."""
        config = WatchdogConfig(name="CustomWatchdog")
        watchdog = Watchdog(config=config)

        assert watchdog._config.name == "CustomWatchdog"

    def test_initial_feed_count_zero(self) -> None:
        """Initial feed count is zero."""
        watchdog = Watchdog()

        assert watchdog.status().feed_count == 0

    def test_initial_timeout_count_zero(self) -> None:
        """Initial timeout count is zero."""
        watchdog = Watchdog()

        assert watchdog.status().timeout_count == 0


class TestHeartbeatExtended:
    """Extended heartbeat (feed) tests."""

    def test_feed_before_start(self) -> None:
        """Feed before start still updates internal state."""
        watchdog = Watchdog(timeout=1.0)

        # Feed count updates even when stopped
        watchdog.feed()

        assert watchdog.status().feed_count == 1

    def test_feed_when_paused(self) -> None:
        """Feed when paused still updates count."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            watchdog.pause()
            watchdog.feed()

            assert watchdog.status().feed_count == 1
        finally:
            watchdog.stop()

    def test_feed_when_triggered(self) -> None:
        """Feed when triggered still updates count."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            time.sleep(0.15)
            assert watchdog.state == WatchdogState.TRIGGERED

            watchdog.feed()

            assert watchdog.status().feed_count == 1
        finally:
            watchdog.stop()


class TestTimeoutExtended:
    """Extended timeout tests."""

    def test_timeout_without_estop(self) -> None:
        """Timeout works when no E-stop is configured."""
        callback = MagicMock()
        watchdog = Watchdog(timeout=0.05)
        watchdog.register_callback(callback)
        watchdog.start()

        try:
            time.sleep(0.15)

            assert watchdog.state == WatchdogState.TRIGGERED
            callback.assert_called_once()
        finally:
            watchdog.stop()

    def test_timeout_reason_includes_name(self) -> None:
        """Timeout E-stop reason includes watchdog name."""
        mock_estop = MagicMock()
        config = WatchdogConfig(name="MyWatchdog", timeout=0.05)
        watchdog = Watchdog(estop=mock_estop, config=config)
        watchdog.start()

        try:
            time.sleep(0.15)

            call_kwargs = mock_estop.trigger.call_args.kwargs
            assert "MyWatchdog" in call_kwargs["reason"]
        finally:
            watchdog.stop()

    def test_timeout_count_persists_across_resets(self) -> None:
        """Timeout count accumulates across resets."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            # First timeout
            time.sleep(0.15)
            assert watchdog.status().timeout_count == 1

            # Reset and second timeout
            watchdog.reset()
            time.sleep(0.15)
            assert watchdog.status().timeout_count == 2
        finally:
            watchdog.stop()


class TestStartStopExtended:
    """Extended start/stop lifecycle tests."""

    def test_start_logs_info(self, caplog: pytest.LogCaptureFixture) -> None:
        """Start logs info message."""
        watchdog = Watchdog(timeout=0.5)

        with caplog.at_level(logging.INFO):
            watchdog.start()

        try:
            assert "started" in caplog.text.lower()
        finally:
            watchdog.stop()

    def test_stop_logs_info(self, caplog: pytest.LogCaptureFixture) -> None:
        """Stop logs info message."""
        watchdog = Watchdog(timeout=0.5)
        watchdog.start()

        with caplog.at_level(logging.INFO):
            watchdog.stop()

        assert "stopped" in caplog.text.lower()

    def test_stop_joins_thread(self) -> None:
        """Stop waits for thread to join."""
        watchdog = Watchdog(timeout=0.5)
        watchdog.start()

        thread = watchdog._thread
        assert thread is not None
        assert thread.is_alive()

        watchdog.stop()

        assert watchdog._thread is None


class TestPauseResumeExtended:
    """Extended pause/resume tests."""

    def test_pause_logs_debug(self, caplog: pytest.LogCaptureFixture) -> None:
        """Pause logs debug message."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            with caplog.at_level(logging.DEBUG):
                watchdog.pause()

            assert "paused" in caplog.text.lower()
        finally:
            watchdog.stop()

    def test_resume_logs_debug(self, caplog: pytest.LogCaptureFixture) -> None:
        """Resume logs debug message."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            watchdog.pause()
            with caplog.at_level(logging.DEBUG):
                watchdog.resume()

            assert "resumed" in caplog.text.lower()
        finally:
            watchdog.stop()

    def test_multiple_pause_resume_cycles(self) -> None:
        """Multiple pause/resume cycles work correctly."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            for _ in range(5):
                assert watchdog.state == WatchdogState.ARMED
                watchdog.pause()
                assert watchdog.state == WatchdogState.PAUSED
                watchdog.resume()
                assert watchdog.state == WatchdogState.ARMED
        finally:
            watchdog.stop()


class TestResetExtended:
    """Extended reset tests."""

    def test_reset_logs_info(self, caplog: pytest.LogCaptureFixture) -> None:
        """Reset logs info message."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            time.sleep(0.15)

            with caplog.at_level(logging.INFO):
                watchdog.reset()

            assert "reset" in caplog.text.lower()
        finally:
            watchdog.stop()

    def test_reset_when_stopped(self) -> None:
        """Reset when stopped is no-op."""
        watchdog = Watchdog(timeout=1.0)

        assert watchdog.state == WatchdogState.STOPPED

        watchdog.reset()

        assert watchdog.state == WatchdogState.STOPPED

    def test_reset_when_paused(self) -> None:
        """Reset when paused is no-op."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            watchdog.pause()
            assert watchdog.state == WatchdogState.PAUSED

            watchdog.reset()

            assert watchdog.state == WatchdogState.PAUSED
        finally:
            watchdog.stop()


class TestControlLoopTimerExtended:
    """Extended ControlLoopTimer tests."""

    def test_no_warning_when_disabled(self, caplog: pytest.LogCaptureFixture) -> None:
        """No warning logged when warn_on_overrun is False."""
        timer = ControlLoopTimer(frequency=100.0, warn_on_overrun=False)
        timer.start()

        with caplog.at_level(logging.WARNING):
            timer.begin_cycle()
            time.sleep(0.015)  # Overrun
            timer.end_cycle()

        # Should not have "overrun" in log when disabled
        # Note: severe overrun still logs error regardless
        timer.stop()

    def test_normal_cycle_sleeps_to_maintain_frequency(self) -> None:
        """Normal cycle sleeps to maintain target frequency."""
        timer = ControlLoopTimer(frequency=50.0)  # 20ms period
        timer.start()

        start = time.perf_counter()
        timer.begin_cycle()
        # Very fast cycle (< 1ms)
        elapsed = timer.end_cycle()
        actual = time.perf_counter() - start

        # Should have slept to reach ~20ms
        assert actual >= 0.018  # At least 18ms

        timer.stop()

    def test_watchdog_fed_multiple_cycles(self) -> None:
        """Watchdog is fed on each cycle."""
        watchdog = Watchdog(timeout=1.0)
        timer = ControlLoopTimer(frequency=100.0, watchdog=watchdog)
        watchdog.start()
        timer.start()

        try:
            for i in range(10):
                timer.begin_cycle()
                timer.end_cycle()

            assert watchdog.status().feed_count == 10
        finally:
            timer.stop()
            watchdog.stop()

    def test_timer_without_watchdog(self) -> None:
        """Timer works without watchdog."""
        timer = ControlLoopTimer(frequency=100.0, watchdog=None)
        timer.start()

        timer.begin_cycle()
        elapsed = timer.end_cycle()

        assert elapsed > 0
        timer.stop()


class TestCallbackBehavior:
    """Tests for callback behavior."""

    def test_callback_receives_triggered_state(self) -> None:
        """Callback receives status with TRIGGERED state."""
        watchdog = Watchdog(timeout=0.05)
        received_status: list[WatchdogStatus] = []

        def capture_callback(status: WatchdogStatus) -> None:
            received_status.append(status)

        watchdog.register_callback(capture_callback)
        watchdog.start()

        try:
            time.sleep(0.15)

            assert len(received_status) == 1
            assert received_status[0].state == WatchdogState.TRIGGERED
        finally:
            watchdog.stop()

    def test_callback_order_preserved(self) -> None:
        """Callbacks are called in registration order."""
        watchdog = Watchdog(timeout=0.05)
        call_order: list[int] = []

        watchdog.register_callback(lambda _: call_order.append(1))
        watchdog.register_callback(lambda _: call_order.append(2))
        watchdog.register_callback(lambda _: call_order.append(3))

        watchdog.start()

        try:
            time.sleep(0.15)

            assert call_order == [1, 2, 3]
        finally:
            watchdog.stop()


class TestWatchdogProperties:
    """Tests for Watchdog properties."""

    def test_is_armed_false_when_stopped(self) -> None:
        """is_armed is False when stopped."""
        watchdog = Watchdog()

        assert watchdog.is_armed is False

    def test_is_armed_true_when_armed(self) -> None:
        """is_armed is True when armed."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            assert watchdog.is_armed is True
        finally:
            watchdog.stop()

    def test_is_armed_false_when_paused(self) -> None:
        """is_armed is False when paused."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            watchdog.pause()
            assert watchdog.is_armed is False
        finally:
            watchdog.stop()

    def test_is_armed_false_when_triggered(self) -> None:
        """is_armed is False when triggered."""
        watchdog = Watchdog(timeout=0.05)
        watchdog.start()

        try:
            time.sleep(0.15)
            assert watchdog.is_armed is False
        finally:
            watchdog.stop()


class TestWatchdogThreadDetails:
    """Tests for watchdog thread behavior."""

    def test_thread_is_daemon(self) -> None:
        """Watchdog thread is daemon thread."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            assert watchdog._thread is not None
            assert watchdog._thread.daemon is True
        finally:
            watchdog.stop()

    def test_thread_name_includes_watchdog_name(self) -> None:
        """Thread name includes watchdog name."""
        config = WatchdogConfig(name="ControlLoop", timeout=1.0)
        watchdog = Watchdog(config=config)
        watchdog.start()

        try:
            assert watchdog._thread is not None
            assert "ControlLoop" in watchdog._thread.name
        finally:
            watchdog.stop()


class TestStatusAgeCalculation:
    """Tests for status age calculation."""

    def test_status_age_zero_before_feed(self) -> None:
        """Status age is zero before any feed."""
        watchdog = Watchdog(timeout=1.0)

        status = watchdog.status()
        assert status.last_feed_age == 0.0

    def test_status_age_increases_over_time(self) -> None:
        """Status age increases as time passes."""
        watchdog = Watchdog(timeout=1.0)
        watchdog.start()

        try:
            watchdog.feed()
            time.sleep(0.1)

            status = watchdog.status()
            assert status.last_feed_age >= 0.09
        finally:
            watchdog.stop()
