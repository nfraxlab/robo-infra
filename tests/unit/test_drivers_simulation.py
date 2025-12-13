"""Tests for SimulationDriver.

Tests cover:
- Basic channel operations (read/write)
- Lifecycle (connect/disconnect/enable/disable)
- Configurable delays
- Operation logging and history
- Value change callbacks
- Channel history tracking
- Testing helpers
"""

from __future__ import annotations

import time

import pytest

from robo_infra.core.driver import DriverState
from robo_infra.drivers.simulation import (
    ChannelHistory,
    OperationRecord,
    OperationType,
    SimulationDriver,
)


# =============================================================================
# Test Fixtures
# =============================================================================


@pytest.fixture
def driver() -> SimulationDriver:
    """Create a connected SimulationDriver."""
    d = SimulationDriver(name="TestDriver", channels=8)
    d.connect()
    return d


@pytest.fixture
def disconnected_driver() -> SimulationDriver:
    """Create a disconnected SimulationDriver."""
    return SimulationDriver(name="TestDriver", channels=8)


# =============================================================================
# Test ChannelHistory
# =============================================================================


class TestChannelHistory:
    """Tests for ChannelHistory dataclass."""

    def test_channel_history_add(self) -> None:
        """Test adding values to history."""
        history = ChannelHistory(channel=0)
        history.add(0.5, timestamp=1000.0)
        history.add(0.75, timestamp=1001.0)

        assert len(history) == 2
        assert history.values == [(1000.0, 0.5), (1001.0, 0.75)]

    def test_channel_history_latest(self) -> None:
        """Test getting latest value."""
        history = ChannelHistory(channel=0)
        assert history.latest is None

        history.add(0.5)
        assert history.latest == 0.5

        history.add(1.0)
        assert history.latest == 1.0

    def test_channel_history_clear(self) -> None:
        """Test clearing history."""
        history = ChannelHistory(channel=0)
        history.add(0.5)
        history.add(0.75)
        assert len(history) == 2

        history.clear()
        assert len(history) == 0
        assert history.latest is None

    def test_channel_history_max_history(self) -> None:
        """Test that history is trimmed to max_history."""
        history = ChannelHistory(channel=0, max_history=3)

        for i in range(5):
            history.add(float(i))

        assert len(history) == 3
        # Should have last 3 values: 2.0, 3.0, 4.0
        assert [v for _, v in history.values] == [2.0, 3.0, 4.0]


# =============================================================================
# Test OperationRecord
# =============================================================================


class TestOperationRecord:
    """Tests for OperationRecord dataclass."""

    def test_operation_record_creation(self) -> None:
        """Test creating an operation record."""
        record = OperationRecord(
            operation=OperationType.WRITE,
            channel=0,
            value=0.5,
            duration=0.001,
        )

        assert record.operation == OperationType.WRITE
        assert record.channel == 0
        assert record.value == 0.5
        assert record.duration == 0.001
        assert record.timestamp > 0

    def test_operation_types(self) -> None:
        """Test all operation types exist."""
        assert OperationType.CONNECT.value == "connect"
        assert OperationType.DISCONNECT.value == "disconnect"
        assert OperationType.ENABLE.value == "enable"
        assert OperationType.DISABLE.value == "disable"
        assert OperationType.READ.value == "read"
        assert OperationType.WRITE.value == "write"
        assert OperationType.FREQUENCY.value == "frequency"


# =============================================================================
# Test SimulationDriver Initialization
# =============================================================================


class TestSimulationDriverInit:
    """Tests for SimulationDriver initialization."""

    def test_default_initialization(self) -> None:
        """Test default initialization."""
        driver = SimulationDriver()

        assert driver.name == "SimulationDriver"
        assert driver.channels == 16
        assert driver.delay == 0.0
        assert driver.log_operations is True

    def test_custom_initialization(self) -> None:
        """Test custom initialization."""
        driver = SimulationDriver(
            name="CustomDriver",
            channels=4,
            delay=0.01,
            log_operations=False,
            max_history=100,
        )

        assert driver.name == "CustomDriver"
        assert driver.channels == 4
        assert driver.delay == 0.01
        assert driver.log_operations is False

    def test_repr(self) -> None:
        """Test string representation."""
        driver = SimulationDriver(name="Test", channels=4, delay=0.005)

        repr_str = repr(driver)
        assert "SimulationDriver" in repr_str
        assert "Test" in repr_str
        assert "channels=4" in repr_str
        assert "delay=0.005" in repr_str


# =============================================================================
# Test Lifecycle
# =============================================================================


class TestSimulationDriverLifecycle:
    """Tests for driver lifecycle methods."""

    def test_connect(self, disconnected_driver: SimulationDriver) -> None:
        """Test connecting driver."""
        assert disconnected_driver.state == DriverState.DISCONNECTED

        disconnected_driver.connect()

        assert disconnected_driver.state == DriverState.CONNECTED
        assert disconnected_driver.is_connected is True

    def test_disconnect(self, driver: SimulationDriver) -> None:
        """Test disconnecting driver."""
        assert driver.state == DriverState.CONNECTED

        driver.disconnect()

        assert driver.state == DriverState.DISCONNECTED
        assert driver.is_connected is False

    def test_enable_disable(self, driver: SimulationDriver) -> None:
        """Test enable/disable."""
        assert driver.is_enabled is True

        driver.disable()
        assert driver.is_enabled is False

        driver.enable()
        assert driver.is_enabled is True

    def test_context_manager(self) -> None:
        """Test context manager usage."""
        driver = SimulationDriver(name="Context")

        with driver:
            assert driver.is_connected is True
            driver.set_channel(0, 0.5)

        assert driver.is_connected is False


# =============================================================================
# Test Channel Operations
# =============================================================================


class TestSimulationDriverChannelOps:
    """Tests for channel read/write operations."""

    def test_set_channel(self, driver: SimulationDriver) -> None:
        """Test setting channel value."""
        driver.set_channel(0, 0.5)
        assert driver.get_channel(0) == 0.5

    def test_set_multiple_channels(self, driver: SimulationDriver) -> None:
        """Test setting multiple channels."""
        for ch in range(8):
            driver.set_channel(ch, ch / 10.0)

        for ch in range(8):
            assert driver.get_channel(ch) == ch / 10.0

    def test_get_all_channels(self, driver: SimulationDriver) -> None:
        """Test getting all channels."""
        driver.set_channel(0, 0.1)
        driver.set_channel(1, 0.2)
        driver.set_channel(2, 0.3)

        all_channels = driver.get_all_channels()

        assert all_channels[0] == 0.1
        assert all_channels[1] == 0.2
        assert all_channels[2] == 0.3

    def test_set_all_channels(self, driver: SimulationDriver) -> None:
        """Test setting all channels to same value."""
        driver.set_all_channels(0.5)

        for ch in range(8):
            assert driver.get_channel(ch) == 0.5

    def test_channel_validation(self, driver: SimulationDriver) -> None:
        """Test channel number validation."""
        with pytest.raises(ValueError, match="out of range"):
            driver.set_channel(99, 0.5)

        with pytest.raises(ValueError, match="out of range"):
            driver.get_channel(-1)

    def test_set_simulated_value(self, driver: SimulationDriver) -> None:
        """Test setting simulated value directly."""
        driver.set_simulated_value(0, 0.123)

        # Value should be set but no callback should have fired
        assert driver.get_channel(0) == 0.123


# =============================================================================
# Test Delay
# =============================================================================


class TestSimulationDriverDelay:
    """Tests for simulated delays."""

    def test_no_delay_by_default(self, driver: SimulationDriver) -> None:
        """Test that operations are fast with no delay."""
        start = time.time()
        driver.set_channel(0, 0.5)
        elapsed = time.time() - start

        assert elapsed < 0.01  # Should be nearly instant

    def test_configurable_delay(self) -> None:
        """Test operations are delayed when configured."""
        driver = SimulationDriver(channels=4, delay=0.05)  # 50ms delay
        driver.connect()

        start = time.time()
        driver.set_channel(0, 0.5)
        elapsed = time.time() - start

        assert elapsed >= 0.05  # Should be at least 50ms

    def test_delay_property(self, driver: SimulationDriver) -> None:
        """Test delay property get/set."""
        assert driver.delay == 0.0

        driver.delay = 0.1
        assert driver.delay == 0.1

    def test_negative_delay_rejected(self, driver: SimulationDriver) -> None:
        """Test that negative delay raises error."""
        with pytest.raises(ValueError, match="Delay cannot be negative"):
            driver.delay = -0.1


# =============================================================================
# Test Operation Logging
# =============================================================================


class TestSimulationDriverOperationLogging:
    """Tests for operation logging and history."""

    def test_operations_logged_by_default(self) -> None:
        """Test that operations are logged by default."""
        driver = SimulationDriver(channels=4)
        driver.connect()
        driver.set_channel(0, 0.5)
        driver.get_channel(0)

        history = driver.operation_history
        assert len(history) >= 3  # connect + write + read

    def test_connect_logged(self) -> None:
        """Test connect operation is logged."""
        driver = SimulationDriver(channels=4)
        driver.connect()

        history = driver.operation_history
        connect_ops = [h for h in history if h.operation == OperationType.CONNECT]
        assert len(connect_ops) == 1

    def test_disconnect_logged(self, driver: SimulationDriver) -> None:
        """Test disconnect operation is logged."""
        driver.disconnect()

        history = driver.operation_history
        disconnect_ops = [
            h for h in history if h.operation == OperationType.DISCONNECT
        ]
        assert len(disconnect_ops) == 1

    def test_write_logged(self, driver: SimulationDriver) -> None:
        """Test write operations are logged."""
        driver.set_channel(0, 0.5)

        history = driver.operation_history
        write_ops = [h for h in history if h.operation == OperationType.WRITE]
        assert len(write_ops) == 1
        assert write_ops[0].channel == 0
        assert write_ops[0].value == 0.5

    def test_read_logged(self, driver: SimulationDriver) -> None:
        """Test read operations are logged."""
        driver.get_channel(0)

        history = driver.operation_history
        read_ops = [h for h in history if h.operation == OperationType.READ]
        assert len(read_ops) == 1
        assert read_ops[0].channel == 0

    def test_enable_disable_logged(self, driver: SimulationDriver) -> None:
        """Test enable/disable operations are logged."""
        driver.disable()
        driver.enable()

        history = driver.operation_history
        enable_ops = [h for h in history if h.operation == OperationType.ENABLE]
        disable_ops = [h for h in history if h.operation == OperationType.DISABLE]

        assert len(enable_ops) == 1
        assert len(disable_ops) == 1

    def test_frequency_logged(self, driver: SimulationDriver) -> None:
        """Test frequency change is logged."""
        driver.set_frequency(100)

        history = driver.operation_history
        freq_ops = [h for h in history if h.operation == OperationType.FREQUENCY]
        assert len(freq_ops) == 1
        assert freq_ops[0].value == 100.0

    def test_logging_disabled(self) -> None:
        """Test that logging can be disabled."""
        driver = SimulationDriver(channels=4, log_operations=False)
        driver.connect()
        driver.set_channel(0, 0.5)

        assert len(driver.operation_history) == 0

    def test_logging_property(self, driver: SimulationDriver) -> None:
        """Test log_operations property."""
        assert driver.log_operations is True

        driver.log_operations = False
        assert driver.log_operations is False

    def test_clear_operation_history(self, driver: SimulationDriver) -> None:
        """Test clearing operation history."""
        driver.set_channel(0, 0.5)
        assert len(driver.operation_history) > 0

        driver.clear_operation_history()
        assert len(driver.operation_history) == 0

    def test_get_write_count(self, driver: SimulationDriver) -> None:
        """Test getting write count."""
        driver.set_channel(0, 0.1)
        driver.set_channel(0, 0.2)
        driver.set_channel(1, 0.3)

        assert driver.get_write_count() == 3
        assert driver.get_write_count(channel=0) == 2
        assert driver.get_write_count(channel=1) == 1

    def test_get_read_count(self, driver: SimulationDriver) -> None:
        """Test getting read count."""
        driver.get_channel(0)
        driver.get_channel(0)
        driver.get_channel(1)

        assert driver.get_read_count() == 3
        assert driver.get_read_count(channel=0) == 2
        assert driver.get_read_count(channel=1) == 1

    def test_max_history_limit(self) -> None:
        """Test that history is limited to max_history."""
        driver = SimulationDriver(channels=4, max_history=5)
        driver.connect()

        for i in range(10):
            driver.set_channel(0, float(i))

        # Max history is 5, so we should have at most 5 records
        assert len(driver.operation_history) <= 5


# =============================================================================
# Test Value Change Callbacks
# =============================================================================


class TestSimulationDriverCallbacks:
    """Tests for value change callbacks."""

    def test_channel_change_callback(self, driver: SimulationDriver) -> None:
        """Test channel change callback is called."""
        changes: list[tuple[int, float]] = []

        def on_change(channel: int, value: float) -> None:
            changes.append((channel, value))

        driver.on_channel_change(on_change)
        driver.set_channel(0, 0.5)
        driver.set_channel(1, 0.75)

        assert changes == [(0, 0.5), (1, 0.75)]

    def test_multiple_callbacks(self, driver: SimulationDriver) -> None:
        """Test multiple callbacks are called."""
        changes1: list[tuple[int, float]] = []
        changes2: list[tuple[int, float]] = []

        driver.on_channel_change(lambda c, v: changes1.append((c, v)))
        driver.on_channel_change(lambda c, v: changes2.append((c, v)))

        driver.set_channel(0, 0.5)

        assert changes1 == [(0, 0.5)]
        assert changes2 == [(0, 0.5)]

    def test_remove_callback(self, driver: SimulationDriver) -> None:
        """Test removing a callback."""
        changes: list[tuple[int, float]] = []

        def on_change(channel: int, value: float) -> None:
            changes.append((channel, value))

        driver.on_channel_change(on_change)
        driver.set_channel(0, 0.5)
        assert len(changes) == 1

        removed = driver.remove_channel_change_callback(on_change)
        assert removed is True

        driver.set_channel(0, 0.75)
        assert len(changes) == 1  # No new change

    def test_remove_nonexistent_callback(self, driver: SimulationDriver) -> None:
        """Test removing a callback that doesn't exist."""

        def some_callback(c: int, v: float) -> None:
            pass

        removed = driver.remove_channel_change_callback(some_callback)
        assert removed is False

    def test_operation_callback(self, driver: SimulationDriver) -> None:
        """Test operation callback is called."""
        operations: list[OperationRecord] = []

        driver.on_operation(lambda op: operations.append(op))

        driver.set_channel(0, 0.5)
        driver.get_channel(0)

        assert len(operations) == 2
        assert operations[0].operation == OperationType.WRITE
        assert operations[1].operation == OperationType.READ

    def test_remove_operation_callback(self, driver: SimulationDriver) -> None:
        """Test removing operation callback."""
        operations: list[OperationRecord] = []

        def on_op(op: OperationRecord) -> None:
            operations.append(op)

        driver.on_operation(on_op)
        driver.set_channel(0, 0.5)
        assert len(operations) == 1

        removed = driver.remove_operation_callback(on_op)
        assert removed is True

        driver.set_channel(0, 0.75)
        assert len(operations) == 1

    def test_clear_callbacks(self, driver: SimulationDriver) -> None:
        """Test clearing all callbacks."""
        changes: list[tuple[int, float]] = []
        operations: list[OperationRecord] = []

        driver.on_channel_change(lambda c, v: changes.append((c, v)))
        driver.on_operation(lambda op: operations.append(op))

        driver.set_channel(0, 0.5)
        assert len(changes) == 1
        assert len(operations) == 1

        driver.clear_callbacks()

        driver.set_channel(0, 0.75)
        assert len(changes) == 1  # No new changes
        assert len(operations) == 1

    def test_callback_exception_handling(self, driver: SimulationDriver) -> None:
        """Test that callback exceptions don't break driver."""

        def bad_callback(channel: int, value: float) -> None:
            raise RuntimeError("Callback error")

        driver.on_channel_change(bad_callback)

        # Should not raise
        driver.set_channel(0, 0.5)
        assert driver.get_channel(0) == 0.5


# =============================================================================
# Test Channel History
# =============================================================================


class TestSimulationDriverChannelHistory:
    """Tests for channel value history."""

    def test_get_channel_history(self, driver: SimulationDriver) -> None:
        """Test getting channel history."""
        driver.set_channel(0, 0.1)
        driver.set_channel(0, 0.2)
        driver.set_channel(0, 0.3)

        history = driver.get_channel_history(0)
        values = [v for _, v in history]

        assert values == [0.1, 0.2, 0.3]

    def test_channel_history_timestamps(self, driver: SimulationDriver) -> None:
        """Test that channel history has timestamps."""
        driver.set_channel(0, 0.5)

        history = driver.get_channel_history(0)
        assert len(history) == 1

        timestamp, value = history[0]
        assert timestamp > 0
        assert value == 0.5

    def test_clear_single_channel_history(self, driver: SimulationDriver) -> None:
        """Test clearing history for single channel."""
        driver.set_channel(0, 0.5)
        driver.set_channel(1, 0.5)

        driver.clear_channel_history(0)

        assert len(driver.get_channel_history(0)) == 0
        assert len(driver.get_channel_history(1)) == 1

    def test_clear_all_channel_history(self, driver: SimulationDriver) -> None:
        """Test clearing history for all channels."""
        driver.set_channel(0, 0.5)
        driver.set_channel(1, 0.5)

        driver.clear_channel_history(None)

        assert len(driver.get_channel_history(0)) == 0
        assert len(driver.get_channel_history(1)) == 0

    def test_channel_history_validation(self, driver: SimulationDriver) -> None:
        """Test channel history validates channel number."""
        with pytest.raises(ValueError, match="out of range"):
            driver.get_channel_history(99)

        with pytest.raises(ValueError, match="out of range"):
            driver.clear_channel_history(99)


# =============================================================================
# Test Reset
# =============================================================================


class TestSimulationDriverReset:
    """Tests for driver reset functionality."""

    def test_reset(self, driver: SimulationDriver) -> None:
        """Test driver reset."""
        driver.set_channel(0, 0.5)
        driver.set_channel(1, 0.75)

        driver.reset()

        # History should be cleared (check before reading)
        assert len(driver.get_channel_history(0)) == 0
        assert len(driver.operation_history) == 0

        # Values should be back to defaults
        # (reading after history check is OK, adds new ops to empty history)
        assert driver.get_channel(0) == 0.0
        assert driver.get_channel(1) == 0.0

    def test_reset_preserves_callbacks(self, driver: SimulationDriver) -> None:
        """Test that reset does not clear callbacks."""
        changes: list[tuple[int, float]] = []
        driver.on_channel_change(lambda c, v: changes.append((c, v)))

        driver.reset()
        driver.set_channel(0, 0.5)

        # Callback should still fire
        assert len(changes) == 1


# =============================================================================
# Test Driver Registration
# =============================================================================


class TestSimulationDriverRegistration:
    """Tests for driver registration."""

    def test_simulation_driver_registered(self) -> None:
        """Test that SimulationDriver is registered."""
        from robo_infra.core.driver import get_driver, register_driver

        # Re-register since test_core_driver.py clears the registry
        register_driver("simulation")(SimulationDriver)

        driver_cls = get_driver("simulation")
        assert driver_cls is SimulationDriver

    def test_create_from_registry(self) -> None:
        """Test creating driver from registry."""
        from robo_infra.core.driver import get_driver, register_driver

        # Re-register since test_core_driver.py clears the registry
        register_driver("simulation")(SimulationDriver)

        driver_cls = get_driver("simulation")
        driver = driver_cls(name="FromRegistry", channels=4)

        assert driver.name == "FromRegistry"
        assert driver.channels == 4
