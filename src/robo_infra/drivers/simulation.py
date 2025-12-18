"""Simulation driver for testing without hardware.

This module provides enhanced simulation drivers with features for testing:
- Configurable response delays
- Optional operation logging
- Value change callbacks for testing
- Channel history tracking

Example:
    >>> from robo_infra.drivers.simulation import SimulationDriver
    >>>
    >>> # Basic usage
    >>> driver = SimulationDriver(channels=16)
    >>> driver.connect()
    >>> driver.set_channel(0, 0.5)
    >>> print(driver.get_channel(0))
    0.5
    >>>
    >>> # With callbacks for testing
    >>> changes = []
    >>> driver.on_channel_change(lambda ch, val: changes.append((ch, val)))
    >>> driver.set_channel(1, 0.75)
    >>> print(changes)
    [(1, 0.75)]
    >>>
    >>> # With simulated delays
    >>> driver = SimulationDriver(channels=8, delay=0.01)  # 10ms delay
"""

from __future__ import annotations

import asyncio
import logging
import os
import time
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum

from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)


logger = logging.getLogger(__name__)


# =============================================================================
# Types
# =============================================================================


class OperationType(Enum):
    """Types of operations that can be logged."""

    CONNECT = "connect"
    DISCONNECT = "disconnect"
    ENABLE = "enable"
    DISABLE = "disable"
    READ = "read"
    WRITE = "write"
    FREQUENCY = "frequency"


@dataclass
class OperationRecord:
    """Record of a driver operation.

    Attributes:
        operation: Type of operation performed.
        channel: Channel number (for read/write operations).
        value: Value involved in the operation.
        timestamp: Unix timestamp of the operation.
        duration: Time taken for the operation in seconds.
    """

    operation: OperationType
    channel: int | None = None
    value: float | None = None
    timestamp: float = field(default_factory=time.time)
    duration: float = 0.0


@dataclass
class ChannelHistory:
    """History of values for a single channel.

    Attributes:
        channel: Channel number.
        values: List of (timestamp, value) tuples.
        max_history: Maximum number of values to retain.
    """

    channel: int
    values: list[tuple[float, float]] = field(default_factory=list)
    max_history: int = 1000

    def add(self, value: float, timestamp: float | None = None) -> None:
        """Add a value to history.

        Args:
            value: Value to add.
            timestamp: Optional timestamp (defaults to current time).
        """
        ts = timestamp if timestamp is not None else time.time()
        self.values.append((ts, value))

        # Trim to max history
        if len(self.values) > self.max_history:
            self.values = self.values[-self.max_history :]

    def clear(self) -> None:
        """Clear all history."""
        self.values.clear()

    @property
    def latest(self) -> float | None:
        """Get the latest value, or None if no history."""
        return self.values[-1][1] if self.values else None

    def __len__(self) -> int:
        """Number of values in history."""
        return len(self.values)


# =============================================================================
# Callback Types
# =============================================================================

ChannelChangeCallback = Callable[[int, float], None]
OperationCallback = Callable[[OperationRecord], None]


# =============================================================================
# Simulation Driver
# =============================================================================


@register_driver("simulation")
class SimulationDriver(Driver):
    """Enhanced simulation driver for testing without hardware.

    Features:
    - All channel operations work in memory
    - Configurable response delays (simulates hardware latency)
    - Optional operation logging with full history
    - Value change callbacks for testing
    - Channel history tracking

    Example:
        >>> driver = SimulationDriver(channels=16)
        >>> driver.connect()
        >>>
        >>> # Basic operations
        >>> driver.set_channel(0, 0.5)
        >>> print(driver.get_channel(0))
        0.5
        >>>
        >>> # Track changes with callback
        >>> values = []
        >>> driver.on_channel_change(lambda ch, v: values.append((ch, v)))
        >>> driver.set_channel(1, 0.75)
        >>> driver.set_channel(2, 1.0)
        >>> print(values)
        [(1, 0.75), (2, 1.0)]
        >>>
        >>> # Get operation history
        >>> for record in driver.operation_history:
        ...     print(f"{record.operation.value}: ch={record.channel} val={record.value}")
    """

    def __init__(
        self,
        name: str | None = None,
        channels: int = 16,
        config: DriverConfig | None = None,
        *,
        delay: float = 0.0,
        log_operations: bool = True,
        max_history: int = 1000,
    ) -> None:
        """Initialize simulation driver.

        Args:
            name: Driver name (defaults to "SimulationDriver").
            channels: Number of channels to simulate.
            config: Optional driver configuration.
            delay: Simulated delay in seconds for each operation.
            log_operations: Whether to log all operations to history.
            max_history: Maximum number of operations to keep in history.
        """
        super().__init__(name or "SimulationDriver", channels, config)

        # Simulation settings
        self._delay = delay
        self._log_operations = log_operations
        self._max_history = max_history

        # Callbacks
        self._channel_change_callbacks: list[ChannelChangeCallback] = []
        self._operation_callbacks: list[OperationCallback] = []

        # History tracking
        self._operation_history: list[OperationRecord] = []
        self._channel_histories: dict[int, ChannelHistory] = defaultdict(
            lambda: ChannelHistory(channel=0, max_history=max_history)
        )
        # Initialize channel histories with correct channel numbers
        for ch in range(channels):
            self._channel_histories[ch] = ChannelHistory(channel=ch, max_history=max_history)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def delay(self) -> float:
        """Simulated delay in seconds per operation."""
        return self._delay

    @delay.setter
    def delay(self, value: float) -> None:
        """Set the simulated delay."""
        if value < 0:
            raise ValueError("Delay cannot be negative")
        self._delay = value

    @property
    def log_operations(self) -> bool:
        """Whether operations are being logged."""
        return self._log_operations

    @log_operations.setter
    def log_operations(self, value: bool) -> None:
        """Enable or disable operation logging."""
        self._log_operations = value

    @property
    def operation_history(self) -> list[OperationRecord]:
        """Get the operation history (read-only copy)."""
        return list(self._operation_history)

    # -------------------------------------------------------------------------
    # Lifecycle Methods
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Simulate connecting to hardware.

        Emits a warning if ROBO_SIMULATION is not set to remind
        the user that no real hardware is connected.
        """
        # Warn about simulation mode unless explicitly enabled
        if not os.getenv("ROBO_SIMULATION"):
            logger.warning(
                "⚠️ SIMULATION MODE — SimulationDriver '%s' has no real hardware. "
                "Set ROBO_SIMULATION=true to suppress this warning.",
                self._name,
            )

        start = time.time()
        self._simulate_delay()

        self._state = DriverState.CONNECTED
        duration = time.time() - start

        self._record_operation(OperationType.CONNECT, duration=duration)
        logger.debug("SimulationDriver %s connected", self._name)

    def disconnect(self) -> None:
        """Simulate disconnecting from hardware."""
        start = time.time()
        self._simulate_delay()

        self._state = DriverState.DISCONNECTED
        duration = time.time() - start

        self._record_operation(OperationType.DISCONNECT, duration=duration)
        logger.debug("SimulationDriver %s disconnected", self._name)

    def enable(self) -> None:
        """Enable the driver with logging."""
        super().enable()
        self._record_operation(OperationType.ENABLE)

    def disable(self) -> None:
        """Disable the driver with logging."""
        super().disable()
        self._record_operation(OperationType.DISABLE)

    # -------------------------------------------------------------------------
    # Channel Operations
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Simulate writing to a channel.

        Args:
            channel: Channel number.
            value: Value to write.
        """
        start = time.time()
        self._simulate_delay()

        # Store in history
        timestamp = time.time()
        self._channel_histories[channel].add(value, timestamp)

        duration = time.time() - start

        # Record operation
        self._record_operation(
            OperationType.WRITE,
            channel=channel,
            value=value,
            duration=duration,
        )

        # Fire callbacks
        self._fire_channel_change_callbacks(channel, value)

        logger.debug(
            "SimulationDriver %s write: channel=%d value=%.4f",
            self._name,
            channel,
            value,
        )

    def _read_channel(self, channel: int) -> float:
        """Simulate reading from a channel.

        Args:
            channel: Channel number.

        Returns:
            Current channel value.
        """
        start = time.time()
        self._simulate_delay()

        value = self._channel_values.get(channel, 0.0)
        duration = time.time() - start

        self._record_operation(
            OperationType.READ,
            channel=channel,
            value=value,
            duration=duration,
        )

        logger.debug(
            "SimulationDriver %s read: channel=%d value=%.4f",
            self._name,
            channel,
            value,
        )

        return value

    def _apply_frequency(self, frequency: int) -> None:
        """Simulate applying frequency.

        Args:
            frequency: Frequency in Hz.
        """
        self._simulate_delay()
        self._record_operation(OperationType.FREQUENCY, value=float(frequency))
        logger.debug(
            "SimulationDriver %s frequency set to %d Hz",
            self._name,
            frequency,
        )

    # -------------------------------------------------------------------------
    # Callback Registration
    # -------------------------------------------------------------------------

    def on_channel_change(self, callback: ChannelChangeCallback) -> None:
        """Register a callback for channel value changes.

        The callback is invoked with (channel, value) whenever a channel
        value is written.

        Args:
            callback: Function taking (channel: int, value: float).

        Example:
            >>> changes = []
            >>> driver.on_channel_change(lambda ch, v: changes.append((ch, v)))
            >>> driver.set_channel(0, 0.5)
            >>> print(changes)
            [(0, 0.5)]
        """
        self._channel_change_callbacks.append(callback)

    def remove_channel_change_callback(self, callback: ChannelChangeCallback) -> bool:
        """Remove a channel change callback.

        Args:
            callback: The callback to remove.

        Returns:
            True if callback was found and removed.
        """
        try:
            self._channel_change_callbacks.remove(callback)
            return True
        except ValueError:
            return False

    def on_operation(self, callback: OperationCallback) -> None:
        """Register a callback for all operations.

        The callback is invoked with an OperationRecord for every
        operation performed on the driver.

        Args:
            callback: Function taking an OperationRecord.
        """
        self._operation_callbacks.append(callback)

    def remove_operation_callback(self, callback: OperationCallback) -> bool:
        """Remove an operation callback.

        Args:
            callback: The callback to remove.

        Returns:
            True if callback was found and removed.
        """
        try:
            self._operation_callbacks.remove(callback)
            return True
        except ValueError:
            return False

    def clear_callbacks(self) -> None:
        """Remove all callbacks."""
        self._channel_change_callbacks.clear()
        self._operation_callbacks.clear()

    # -------------------------------------------------------------------------
    # History Access
    # -------------------------------------------------------------------------

    def get_channel_history(self, channel: int) -> list[tuple[float, float]]:
        """Get value history for a channel.

        Args:
            channel: Channel number.

        Returns:
            List of (timestamp, value) tuples.
        """
        self._validate_channel(channel)
        return list(self._channel_histories[channel].values)

    def clear_channel_history(self, channel: int | None = None) -> None:
        """Clear channel history.

        Args:
            channel: Channel to clear, or None to clear all.
        """
        if channel is not None:
            self._validate_channel(channel)
            self._channel_histories[channel].clear()
        else:
            for history in self._channel_histories.values():
                history.clear()

    def clear_operation_history(self) -> None:
        """Clear the operation history."""
        self._operation_history.clear()

    def get_write_count(self, channel: int | None = None) -> int:
        """Get the number of write operations.

        Args:
            channel: Specific channel, or None for all channels.

        Returns:
            Number of write operations.
        """
        writes = [op for op in self._operation_history if op.operation == OperationType.WRITE]
        if channel is not None:
            writes = [op for op in writes if op.channel == channel]
        return len(writes)

    def get_read_count(self, channel: int | None = None) -> int:
        """Get the number of read operations.

        Args:
            channel: Specific channel, or None for all channels.

        Returns:
            Number of read operations.
        """
        reads = [op for op in self._operation_history if op.operation == OperationType.READ]
        if channel is not None:
            reads = [op for op in reads if op.channel == channel]
        return len(reads)

    # -------------------------------------------------------------------------
    # Simulation Helpers
    # -------------------------------------------------------------------------

    def _simulate_delay(self) -> None:
        """Apply simulated delay if configured."""
        if self._delay > 0:
            time.sleep(self._delay)

    async def _simulate_delay_async(self) -> None:
        """Apply simulated delay asynchronously."""
        if self._delay > 0:
            await asyncio.sleep(self._delay)

    def _record_operation(
        self,
        operation: OperationType,
        channel: int | None = None,
        value: float | None = None,
        duration: float = 0.0,
    ) -> None:
        """Record an operation in history and fire callbacks.

        Args:
            operation: Type of operation.
            channel: Optional channel number.
            value: Optional value.
            duration: Operation duration in seconds.
        """
        record = OperationRecord(
            operation=operation,
            channel=channel,
            value=value,
            timestamp=time.time(),
            duration=duration,
        )

        # Log to history if enabled
        if self._log_operations:
            self._operation_history.append(record)

            # Trim to max history
            if len(self._operation_history) > self._max_history:
                self._operation_history = self._operation_history[-self._max_history :]

        # Fire operation callbacks
        for callback in self._operation_callbacks:
            try:
                callback(record)
            except Exception as e:
                logger.warning("Operation callback failed: %s", e)

    def _fire_channel_change_callbacks(self, channel: int, value: float) -> None:
        """Fire all channel change callbacks.

        Args:
            channel: Channel that changed.
            value: New value.
        """
        for callback in self._channel_change_callbacks:
            try:
                callback(channel, value)
            except Exception as e:
                logger.warning("Channel change callback failed: %s", e)

    # -------------------------------------------------------------------------
    # Testing Helpers
    # -------------------------------------------------------------------------

    def set_simulated_value(self, channel: int, value: float) -> None:
        """Set a channel value directly (bypassing callbacks).

        Useful for simulating external changes to channel values,
        like a sensor reading changing.

        Args:
            channel: Channel number.
            value: Value to set.
        """
        self._validate_channel(channel)
        self._channel_values[channel] = value

    def reset(self) -> None:
        """Reset the driver to initial state.

        Clears all channel values, history, and callbacks.
        """
        # Reset channel values to defaults
        for ch in range(self._channels):
            config = self._channel_configs.get(ch)
            default = config.default_value if config else 0.0
            self._channel_values[ch] = default

        # Clear histories
        self.clear_channel_history()
        self.clear_operation_history()

        # Note: callbacks are NOT cleared - user must do that explicitly
        logger.debug("SimulationDriver %s reset", self._name)

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"SimulationDriver(name={self._name!r}, "
            f"channels={self._channels}, "
            f"delay={self._delay}, "
            f"state={self._state.value})"
        )
