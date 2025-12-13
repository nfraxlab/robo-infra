"""Emergency stop system for robotics safety.

The E-stop is the most critical safety system. When triggered, it MUST:
1. Disable ALL actuators immediately (no exceptions)
2. Propagate errors (never suppress)
3. Require explicit reset before resuming

Example:
    >>> from robo_infra.safety import EStop, EStopState
    >>>
    >>> estop = EStop()
    >>> estop.register_actuator(servo1)
    >>> estop.register_actuator(motor1)
    >>>
    >>> # In emergency:
    >>> estop.trigger("User pressed E-stop button")
    >>> # All actuators are now disabled
    >>>
    >>> # After resolving the issue:
    >>> estop.reset()
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from pydantic import BaseModel

from robo_infra.core.exceptions import SafetyError


if TYPE_CHECKING:
    from collections.abc import Callable


logger = logging.getLogger(__name__)


class EStopError(SafetyError):
    """Raised when E-stop fails to disable an actuator.

    CRITICAL: This error means the robot may still be moving.
    Physical intervention may be required.
    """

    def __init__(
        self,
        failed_actuators: list[str],
        successful_actuators: list[str],
        errors: dict[str, Exception],
    ) -> None:
        self.failed_actuators = failed_actuators
        self.successful_actuators = successful_actuators
        self.errors = errors
        message = (
            f"E-STOP FAILED: {len(failed_actuators)} actuators could not be disabled: "
            f"{failed_actuators}. Errors: {errors}"
        )
        super().__init__(message, action_taken="Partial disable attempted")


class EStopState(Enum):
    """E-stop system states."""

    ARMED = "armed"  # Normal operation, ready to trigger
    TRIGGERED = "triggered"  # E-stop active, all actuators disabled
    RESET_PENDING = "reset_pending"  # Waiting for conditions to clear
    DISABLED = "disabled"  # E-stop system itself is disabled (DANGEROUS)


class EStopConfig(BaseModel):
    """Configuration for E-stop system.

    Attributes:
        name: Name of this E-stop instance.
        require_reset_confirmation: If True, reset() must be called twice.
        log_all_triggers: If True, log every trigger with full context.
        propagate_errors: If True, raise EStopError on partial failure.
    """

    name: str = "EStop"
    require_reset_confirmation: bool = True
    log_all_triggers: bool = True
    propagate_errors: bool = True  # MUST be True in production
    max_disable_attempts: int = 3

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class EStopEvent:
    """Record of an E-stop event."""

    timestamp: float
    reason: str
    triggered_by: str | None = None
    actuators_disabled: list[str] = field(default_factory=list)
    actuators_failed: list[str] = field(default_factory=list)
    errors: dict[str, str] = field(default_factory=dict)


@runtime_checkable
class Disableable(Protocol):
    """Protocol for anything that can be disabled (actuators, controllers)."""

    @property
    def name(self) -> str:
        """Name of the component."""
        ...

    def disable(self) -> None:
        """Disable the component. MUST stop all motion."""
        ...


class EStop:
    """Emergency stop system that disables all registered actuators.

    SAFETY CRITICAL:
    - E-stop ALWAYS attempts to disable ALL actuators
    - Errors are NEVER suppressed - they propagate after all attempts
    - Multiple disable attempts are made for each actuator
    - All events are logged for safety auditing
    """

    def __init__(self, config: EStopConfig | None = None) -> None:
        """Initialize E-stop system.

        Args:
            config: E-stop configuration. Uses safe defaults if None.
        """
        self._config = config or EStopConfig()
        self._state = EStopState.ARMED
        self._actuators: dict[str, Disableable] = {}
        self._callbacks: list[Callable[[EStopEvent], None]] = []
        self._event_log: list[EStopEvent] = []
        self._lock = threading.RLock()
        self._reset_confirmation_pending = False
        self._last_trigger_reason: str | None = None

    @property
    def state(self) -> EStopState:
        """Current E-stop state."""
        return self._state

    @property
    def is_triggered(self) -> bool:
        """True if E-stop is currently triggered."""
        return self._state == EStopState.TRIGGERED

    @property
    def is_armed(self) -> bool:
        """True if E-stop is armed and ready."""
        return self._state == EStopState.ARMED

    @property
    def event_log(self) -> list[EStopEvent]:
        """History of E-stop events (read-only copy)."""
        return list(self._event_log)

    def register_actuator(self, actuator: Disableable) -> None:
        """Register an actuator to be disabled on E-stop.

        Args:
            actuator: Any object with name property and disable() method.
        """
        with self._lock:
            self._actuators[actuator.name] = actuator
            logger.debug("Registered actuator '%s' with E-stop", actuator.name)

    def unregister_actuator(self, name: str) -> None:
        """Remove an actuator from E-stop control.

        Args:
            name: Name of actuator to remove.
        """
        with self._lock:
            if name in self._actuators:
                del self._actuators[name]
                logger.debug("Unregistered actuator '%s' from E-stop", name)

    def register_callback(self, callback: Callable[[EStopEvent], None]) -> None:
        """Register a callback to be called on E-stop trigger.

        Callbacks are called AFTER all actuators are disabled.
        Callback errors are logged but do not prevent E-stop.

        Args:
            callback: Function that takes an EStopEvent.
        """
        self._callbacks.append(callback)

    def trigger(
        self,
        reason: str,
        *,
        triggered_by: str | None = None,
    ) -> EStopEvent:
        """Trigger emergency stop - DISABLE ALL ACTUATORS IMMEDIATELY.

        This method:
        1. Attempts to disable every registered actuator
        2. Makes multiple attempts for each actuator
        3. Logs all successes and failures
        4. Calls registered callbacks
        5. Raises EStopError if any actuator failed to disable

        Args:
            reason: Why E-stop was triggered (for logging/audit).
            triggered_by: Who/what triggered it (user, sensor, etc.).

        Returns:
            EStopEvent with full details of what happened.

        Raises:
            EStopError: If any actuator failed to disable (propagate_errors=True).
        """
        with self._lock:
            logger.critical(
                "🛑 E-STOP TRIGGERED: %s (by: %s)",
                reason,
                triggered_by or "unknown",
            )

            self._state = EStopState.TRIGGERED
            self._last_trigger_reason = reason

            # Track results
            successful: list[str] = []
            failed: list[str] = []
            errors: dict[str, Exception] = {}

            # Attempt to disable ALL actuators - NO EXCEPTIONS SWALLOWED
            for name, actuator in self._actuators.items():
                disabled = False
                last_error: Exception | None = None

                # Multiple attempts per actuator
                for attempt in range(self._config.max_disable_attempts):
                    try:
                        actuator.disable()
                        disabled = True
                        logger.info("E-stop disabled actuator '%s' (attempt %d)", name, attempt + 1)
                        break
                    except Exception as e:
                        last_error = e
                        logger.error(
                            "E-stop failed to disable '%s' (attempt %d/%d): %s",
                            name,
                            attempt + 1,
                            self._config.max_disable_attempts,
                            e,
                        )

                if disabled:
                    successful.append(name)
                else:
                    failed.append(name)
                    if last_error:
                        errors[name] = last_error

            # Create event record
            event = EStopEvent(
                timestamp=time.time(),
                reason=reason,
                triggered_by=triggered_by,
                actuators_disabled=successful,
                actuators_failed=failed,
                errors={k: str(v) for k, v in errors.items()},
            )
            self._event_log.append(event)

            # Call callbacks (errors here don't prevent E-stop)
            for callback in self._callbacks:
                try:
                    callback(event)
                except Exception as e:
                    logger.error("E-stop callback failed: %s", e)

            # Log final status
            if failed:
                logger.critical(
                    "🚨 E-STOP INCOMPLETE: %d/%d actuators failed to disable: %s",
                    len(failed),
                    len(self._actuators),
                    failed,
                )
            else:
                logger.info(
                    "✅ E-STOP COMPLETE: All %d actuators disabled",
                    len(successful),
                )

            # CRITICAL: Raise if any actuator failed to disable
            if failed and self._config.propagate_errors:
                raise EStopError(
                    failed_actuators=failed,
                    successful_actuators=successful,
                    errors=errors,
                )

            return event

    def reset(self, *, confirm: bool = False) -> bool:
        """Reset E-stop to armed state.

        After reset, actuators must be explicitly re-enabled.
        Homing may be required depending on actuator type.

        Args:
            confirm: Required if require_reset_confirmation is True.

        Returns:
            True if reset successful, False if confirmation needed.
        """
        with self._lock:
            if self._state != EStopState.TRIGGERED:
                logger.warning("E-stop reset called but not in triggered state")
                return True

            # Two-step confirmation if configured
            if self._config.require_reset_confirmation:
                if not self._reset_confirmation_pending:
                    self._reset_confirmation_pending = True
                    self._state = EStopState.RESET_PENDING
                    logger.warning("E-stop reset pending - call reset(confirm=True) to confirm")
                    return False

                if not confirm:
                    logger.warning("E-stop reset requires confirm=True")
                    return False

            self._reset_confirmation_pending = False
            self._state = EStopState.ARMED
            logger.info("✅ E-STOP RESET - System armed (actuators still disabled)")

            return True

    def force_arm(self) -> None:
        """Force E-stop to armed state without normal reset.

        ⚠️ DANGEROUS: Only use when you are certain the system is safe.
        This bypasses reset confirmation and all safety checks.
        """
        with self._lock:
            logger.warning("⚠️ E-STOP FORCE ARMED - Safety checks bypassed!")
            self._state = EStopState.ARMED
            self._reset_confirmation_pending = False

    def disable_system(self) -> None:
        """Disable the E-stop system itself.

        ⚠️ EXTREMELY DANGEROUS: Only use for maintenance.
        E-stop will not function while disabled.
        """
        with self._lock:
            logger.critical("🚨 E-STOP SYSTEM DISABLED - NO SAFETY PROTECTION!")
            self._state = EStopState.DISABLED

    def enable_system(self) -> None:
        """Re-enable the E-stop system after disable_system()."""
        with self._lock:
            self._state = EStopState.ARMED
            logger.info("E-stop system re-enabled")


# =============================================================================
# Hardware E-Stop Pin
# =============================================================================


class HardwareEStop:
    """Hardware E-stop button connected via GPIO pin.

    Monitors a physical E-stop button and triggers the software E-stop.

    Example:
        >>> from robo_infra.core.pin import GPIOPin
        >>> from robo_infra.safety import EStop, HardwareEStop
        >>>
        >>> estop = EStop()
        >>> hw_estop = HardwareEStop(
        ...     pin=GPIOPin(17),
        ...     software_estop=estop,
        ...     normally_closed=True,  # NC button (safer)
        ... )
        >>> hw_estop.start_monitoring()
    """

    def __init__(
        self,
        pin: Any,  # Pin protocol
        software_estop: EStop,
        *,
        normally_closed: bool = True,
        poll_interval: float = 0.01,
        debounce_ms: float = 50.0,
    ) -> None:
        """Initialize hardware E-stop monitor.

        Args:
            pin: GPIO pin connected to E-stop button.
            software_estop: EStop instance to trigger.
            normally_closed: If True, button is NC (safer design).
            poll_interval: How often to check pin (seconds).
            debounce_ms: Debounce time in milliseconds.
        """
        self._pin = pin
        self._estop = software_estop
        self._normally_closed = normally_closed
        self._poll_interval = poll_interval
        self._debounce_ms = debounce_ms
        self._monitoring = False
        self._thread: threading.Thread | None = None

    def start_monitoring(self) -> None:
        """Start monitoring the hardware E-stop button."""
        if self._monitoring:
            return

        self._monitoring = True
        self._thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="HardwareEStop-Monitor",
        )
        self._thread.start()
        logger.info("Hardware E-stop monitoring started")

    def stop_monitoring(self) -> None:
        """Stop monitoring the hardware E-stop button."""
        self._monitoring = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        logger.info("Hardware E-stop monitoring stopped")

    def _monitor_loop(self) -> None:
        """Main monitoring loop."""
        last_state = self._read_button()
        debounce_start: float | None = None

        while self._monitoring:
            current_state = self._read_button()

            # Debounce logic
            if current_state != last_state:
                if debounce_start is None:
                    debounce_start = time.time()
                elif (time.time() - debounce_start) * 1000 >= self._debounce_ms:
                    # State changed and debounce passed
                    if self._is_triggered_state(current_state):
                        self._estop.trigger(
                            reason="Hardware E-stop button pressed",
                            triggered_by="hardware_button",
                        )
                    last_state = current_state
                    debounce_start = None
            else:
                debounce_start = None

            time.sleep(self._poll_interval)

    def _read_button(self) -> bool:
        """Read the current button state."""
        try:
            return bool(self._pin.read())
        except Exception as e:
            logger.error("Failed to read E-stop button: %s", e)
            # If we can't read the button, assume triggered (fail safe)
            return not self._normally_closed

    def _is_triggered_state(self, pin_state: bool) -> bool:
        """Check if the pin state indicates E-stop should trigger."""
        if self._normally_closed:
            # NC: circuit breaks when button pressed -> pin goes LOW
            return not pin_state
        else:
            # NO: circuit closes when button pressed -> pin goes HIGH
            return pin_state
