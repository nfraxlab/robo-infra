"""Watchdog timer for robotics control loop safety.

A watchdog ensures the control loop is running and responsive.
If the watchdog is not "fed" (reset) within the timeout period,
it triggers an E-stop to prevent runaway or frozen robots.

Example:
    >>> from robo_infra.safety import Watchdog, EStop
    >>>
    >>> estop = EStop()
    >>> watchdog = Watchdog(timeout=0.1, estop=estop)  # 100ms timeout
    >>> watchdog.start()
    >>>
    >>> # In control loop:
    >>> while running:
    ...     do_control_update()
    ...     watchdog.feed()  # Must call every 100ms
    >>>
    >>> watchdog.stop()
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from pydantic import BaseModel, Field

from robo_infra.core.exceptions import SafetyError


if TYPE_CHECKING:
    from collections.abc import Callable

    from robo_infra.safety.estop import EStop


logger = logging.getLogger(__name__)


class WatchdogError(SafetyError):
    """Raised when watchdog times out."""

    def __init__(self, name: str, timeout: float, last_feed_age: float) -> None:
        self.name = name
        self.timeout = timeout
        self.last_feed_age = last_feed_age
        message = (
            f"Watchdog '{name}' timed out: no feed for {last_feed_age:.3f}s (timeout: {timeout}s)"
        )
        super().__init__(message, action_taken="E-stop triggered")


class WatchdogState(Enum):
    """Watchdog states."""

    STOPPED = "stopped"  # Watchdog is not running
    ARMED = "armed"  # Running and watching
    TRIGGERED = "triggered"  # Timeout occurred
    PAUSED = "paused"  # Temporarily paused (e.g., during homing)


class WatchdogConfig(BaseModel):
    """Configuration for watchdog timer.

    Attributes:
        name: Name of this watchdog instance.
        timeout: Time in seconds before watchdog triggers.
        trigger_estop: If True, trigger E-stop on timeout.
        auto_start: If True, start monitoring when created.
        warn_threshold: Warn if feed age exceeds this fraction of timeout.
    """

    name: str = "Watchdog"
    timeout: float = Field(default=0.1, gt=0, description="Timeout in seconds")
    trigger_estop: bool = True
    auto_start: bool = False
    warn_threshold: float = Field(default=0.8, ge=0, le=1.0)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class WatchdogStatus:
    """Current status of watchdog."""

    state: WatchdogState = WatchdogState.STOPPED
    timeout: float = 0.1
    last_feed_time: float = 0.0
    feed_count: int = 0
    timeout_count: int = 0
    last_feed_age: float = 0.0


class Watchdog:
    """Control loop watchdog timer.

    The watchdog runs in a separate thread and monitors for regular "feeds"
    from the main control loop. If no feed is received within the timeout
    period, the watchdog triggers (optionally triggering E-stop).

    SAFETY CRITICAL:
    - Watchdog thread is high-priority
    - Timeout always triggers E-stop (if configured)
    - Errors in callback don't prevent E-stop
    """

    def __init__(
        self,
        timeout: float | None = None,
        estop: EStop | None = None,
        config: WatchdogConfig | None = None,
    ) -> None:
        """Initialize watchdog.

        Args:
            timeout: Timeout in seconds (shorthand for config.timeout).
            estop: E-stop to trigger on timeout.
            config: Full configuration (overrides timeout arg).
        """
        if config:
            self._config = config
        else:
            self._config = WatchdogConfig(timeout=timeout if timeout is not None else 0.1)

        self._estop = estop
        self._state = WatchdogState.STOPPED
        self._last_feed_time: float = 0.0
        self._feed_count: int = 0
        self._timeout_count: int = 0
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._callbacks: list[Callable[[WatchdogStatus], None]] = []

        if self._config.auto_start:
            self.start()

    @property
    def state(self) -> WatchdogState:
        """Current watchdog state."""
        return self._state

    @property
    def is_armed(self) -> bool:
        """True if watchdog is actively monitoring."""
        return self._state == WatchdogState.ARMED

    @property
    def timeout(self) -> float:
        """Current timeout in seconds."""
        return self._config.timeout

    def status(self) -> WatchdogStatus:
        """Get current watchdog status."""
        with self._lock:
            age = time.time() - self._last_feed_time if self._last_feed_time else 0.0
            return WatchdogStatus(
                state=self._state,
                timeout=self._config.timeout,
                last_feed_time=self._last_feed_time,
                feed_count=self._feed_count,
                timeout_count=self._timeout_count,
                last_feed_age=age,
            )

    def register_callback(self, callback: Callable[[WatchdogStatus], None]) -> None:
        """Register callback for timeout events.

        Callbacks are called AFTER E-stop is triggered.
        """
        self._callbacks.append(callback)

    def start(self) -> None:
        """Start the watchdog timer.

        Must call feed() regularly after starting.
        """
        if self._running:
            return

        self._running = True
        self._last_feed_time = time.time()
        self._state = WatchdogState.ARMED

        self._thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name=f"Watchdog-{self._config.name}",
        )
        self._thread.start()
        logger.info(
            "Watchdog '%s' started (timeout: %.3fs)",
            self._config.name,
            self._config.timeout,
        )

    def stop(self) -> None:
        """Stop the watchdog timer."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=self._config.timeout * 2)
            self._thread = None
        self._state = WatchdogState.STOPPED
        logger.info("Watchdog '%s' stopped", self._config.name)

    def pause(self) -> None:
        """Temporarily pause the watchdog (e.g., during homing)."""
        if self._state == WatchdogState.ARMED:
            self._state = WatchdogState.PAUSED
            logger.debug("Watchdog '%s' paused", self._config.name)

    def resume(self) -> None:
        """Resume the watchdog after pausing."""
        if self._state == WatchdogState.PAUSED:
            self._last_feed_time = time.time()  # Reset timer on resume
            self._state = WatchdogState.ARMED
            logger.debug("Watchdog '%s' resumed", self._config.name)

    def feed(self) -> None:
        """Feed the watchdog to prevent timeout.

        Call this regularly from your control loop.
        Should be called at least once per timeout period.
        """
        with self._lock:
            now = time.time()
            age = now - self._last_feed_time if self._last_feed_time else 0.0

            # Warn if getting close to timeout
            if age > self._config.timeout * self._config.warn_threshold:
                logger.warning(
                    "Watchdog '%s' feed late: %.3fs (timeout: %.3fs)",
                    self._config.name,
                    age,
                    self._config.timeout,
                )

            self._last_feed_time = now
            self._feed_count += 1

    def _monitor_loop(self) -> None:
        """Main watchdog monitoring loop."""
        check_interval = self._config.timeout / 4  # Check 4x per timeout

        while self._running:
            time.sleep(check_interval)

            if self._state != WatchdogState.ARMED:
                continue

            with self._lock:
                age = time.time() - self._last_feed_time

            if age >= self._config.timeout:
                self._handle_timeout(age)

    def _handle_timeout(self, age: float) -> None:
        """Handle watchdog timeout."""
        self._state = WatchdogState.TRIGGERED
        self._timeout_count += 1

        logger.critical(
            "🐕 WATCHDOG TIMEOUT: '%s' - no feed for %.3fs (timeout: %.3fs)",
            self._config.name,
            age,
            self._config.timeout,
        )

        # Trigger E-stop FIRST
        if self._config.trigger_estop and self._estop:
            try:
                self._estop.trigger(
                    reason=f"Watchdog '{self._config.name}' timeout: {age:.3f}s",
                    triggered_by="watchdog",
                )
            except Exception as e:
                logger.critical("Watchdog E-stop trigger failed: %s", e)

        # Call callbacks
        status = self.status()
        for callback in self._callbacks:
            try:
                callback(status)
            except Exception as e:
                logger.error("Watchdog callback failed: %s", e)

    def reset(self) -> None:
        """Reset watchdog after timeout.

        Clears triggered state and rearms the watchdog.
        """
        if self._state == WatchdogState.TRIGGERED:
            self._last_feed_time = time.time()
            self._state = WatchdogState.ARMED
            logger.info("Watchdog '%s' reset and rearmed", self._config.name)


# =============================================================================
# Control Loop Timing
# =============================================================================


class ControlLoopTimer:
    """Timer for enforcing control loop frequency with overrun detection.

    Ensures control loops run at a consistent frequency and detects
    when updates take longer than the target period.

    Example:
        >>> timer = ControlLoopTimer(frequency=50.0)  # 50Hz
        >>> timer.start()
        >>>
        >>> while running:
        ...     timer.begin_cycle()
        ...     do_control_update()
        ...     timer.end_cycle()  # Sleeps to maintain frequency
    """

    def __init__(
        self,
        frequency: float = 50.0,
        *,
        warn_on_overrun: bool = True,
        max_overrun_ratio: float = 2.0,
        watchdog: Watchdog | None = None,
    ) -> None:
        """Initialize control loop timer.

        Args:
            frequency: Target loop frequency in Hz.
            warn_on_overrun: Log warning on overrun.
            max_overrun_ratio: Trigger watchdog if overrun exceeds this ratio.
            watchdog: Watchdog to feed on each cycle.
        """
        self._frequency = frequency
        self._period = 1.0 / frequency
        self._warn_on_overrun = warn_on_overrun
        self._max_overrun_ratio = max_overrun_ratio
        self._watchdog = watchdog

        self._cycle_start: float = 0.0
        self._cycle_count: int = 0
        self._overrun_count: int = 0
        self._total_overrun_time: float = 0.0
        self._running = False

    @property
    def frequency(self) -> float:
        """Target frequency in Hz."""
        return self._frequency

    @property
    def period(self) -> float:
        """Target period in seconds."""
        return self._period

    @property
    def cycle_count(self) -> int:
        """Number of cycles completed."""
        return self._cycle_count

    @property
    def overrun_count(self) -> int:
        """Number of cycles that exceeded the target period."""
        return self._overrun_count

    @property
    def overrun_ratio(self) -> float:
        """Fraction of cycles that overran."""
        if self._cycle_count == 0:
            return 0.0
        return self._overrun_count / self._cycle_count

    def start(self) -> None:
        """Start the timer."""
        self._running = True
        self._cycle_start = time.perf_counter()
        logger.debug(
            "Control loop timer started: %.1fHz (period: %.3fms)",
            self._frequency,
            self._period * 1000,
        )

    def stop(self) -> None:
        """Stop the timer."""
        self._running = False
        logger.debug(
            "Control loop timer stopped after %d cycles (%d overruns, %.1f%%)",
            self._cycle_count,
            self._overrun_count,
            self.overrun_ratio * 100,
        )

    def begin_cycle(self) -> None:
        """Mark the beginning of a control cycle."""
        self._cycle_start = time.perf_counter()

    def end_cycle(self) -> float:
        """Mark the end of a control cycle and sleep if needed.

        Returns:
            Actual cycle time in seconds.
        """
        now = time.perf_counter()
        elapsed = now - self._cycle_start
        self._cycle_count += 1

        # Feed watchdog
        if self._watchdog:
            self._watchdog.feed()

        # Check for overrun
        if elapsed > self._period:
            overrun = elapsed - self._period
            self._overrun_count += 1
            self._total_overrun_time += overrun

            if self._warn_on_overrun:
                logger.warning(
                    "Control loop overrun: %.3fms > %.3fms (cycle %d)",
                    elapsed * 1000,
                    self._period * 1000,
                    self._cycle_count,
                )

            # Check for severe overrun
            if elapsed > self._period * self._max_overrun_ratio:
                logger.error(
                    "Severe control loop overrun: %.3fms (%.1fx target)",
                    elapsed * 1000,
                    elapsed / self._period,
                )
        else:
            # Sleep for remaining time
            remaining = self._period - elapsed
            time.sleep(remaining)

        return elapsed

    def reset_stats(self) -> None:
        """Reset overrun statistics."""
        self._cycle_count = 0
        self._overrun_count = 0
        self._total_overrun_time = 0.0
