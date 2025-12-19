"""Tests for robo_infra.controllers.lock module."""

from __future__ import annotations

import pytest

from robo_infra.actuators.servo import Servo
from robo_infra.controllers.lock import (
    Lock,
    LockConfig,
    LockState,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def lock_servo() -> Servo:
    """Create a simulated lock servo."""
    return Servo(name="lock_servo", angle_range=(0, 90))


@pytest.fixture
def lock_config() -> LockConfig:
    """Create default lock configuration."""
    return LockConfig(
        name="test_lock",
        locked_position=0.0,
        unlocked_position=90.0,
        position_tolerance=5.0,
        start_locked=True,
    )


@pytest.fixture
def lock_config_starts_unlocked() -> LockConfig:
    """Create lock configuration that starts unlocked."""
    return LockConfig(
        name="test_lock",
        locked_position=0.0,
        unlocked_position=90.0,
        start_locked=False,
    )


@pytest.fixture
def lock(lock_servo: Servo, lock_config: LockConfig) -> Lock:
    """Create a Lock controller with default config."""
    return Lock(
        name="test_lock",
        actuator=lock_servo,
        config=lock_config,
    )


@pytest.fixture
def enabled_lock(lock: Lock) -> Lock:
    """Create and enable a Lock controller."""
    lock.enable()
    return lock


@pytest.fixture
def lock_starts_unlocked(lock_servo: Servo, lock_config_starts_unlocked: LockConfig) -> Lock:
    """Create a Lock controller that starts unlocked."""
    return Lock(
        name="test_lock",
        actuator=lock_servo,
        config=lock_config_starts_unlocked,
    )


# =============================================================================
# Initialization Tests
# =============================================================================


class TestLockInit:
    """Tests for Lock initialization."""

    def test_lock_init(self, lock_servo: Servo) -> None:
        """Test basic initialization."""
        lock = Lock(
            name="my_lock",
            actuator=lock_servo,
        )

        assert lock.name == "my_lock"
        assert lock.actuator is lock_servo
        assert lock.lock_state == LockState.DISABLED
        assert not lock.is_enabled

    def test_lock_init_with_config(self, lock_servo: Servo, lock_config: LockConfig) -> None:
        """Test initialization with custom config."""
        lock = Lock(
            name="custom_lock",
            actuator=lock_servo,
            config=lock_config,
        )

        assert lock.name == "custom_lock"
        assert lock.lock_config.locked_position == 0.0
        assert lock.lock_config.unlocked_position == 90.0
        assert lock.lock_config.start_locked is True

    def test_lock_init_starts_locked(self, enabled_lock: Lock) -> None:
        """Test that lock starts in locked state when configured."""
        assert enabled_lock.is_locked
        assert enabled_lock.lock_state == LockState.LOCKED
        assert enabled_lock.state_str == "locked"

    def test_lock_init_starts_unlocked(self, lock_starts_unlocked: Lock) -> None:
        """Test that lock starts in unlocked state when configured."""
        lock_starts_unlocked.enable()
        assert lock_starts_unlocked.is_unlocked
        assert lock_starts_unlocked.lock_state == LockState.UNLOCKED
        assert lock_starts_unlocked.state_str == "unlocked"

    def test_lock_init_without_actuator_raises(self) -> None:
        """Test that initialization without actuator raises ValueError."""
        with pytest.raises(ValueError, match="actuator is required"):
            Lock(name="no_actuator", actuator=None)  # type: ignore[arg-type]


# =============================================================================
# Lock/Unlock Tests
# =============================================================================


class TestLockUnlock:
    """Tests for Lock lock/unlock operations."""

    def test_lock_moves_to_locked_position(self, enabled_lock: Lock) -> None:
        """Test that lock() moves to locked position."""
        # Start by unlocking
        enabled_lock.unlock()
        assert enabled_lock.is_unlocked

        # Lock
        enabled_lock.lock()
        assert enabled_lock.is_locked
        assert enabled_lock.lock_state == LockState.LOCKED

    def test_unlock_moves_to_unlocked_position(self, enabled_lock: Lock) -> None:
        """Test that unlock() moves to unlocked position."""
        # Ensure we start locked
        assert enabled_lock.is_locked

        # Unlock
        enabled_lock.unlock()
        assert enabled_lock.is_unlocked
        assert enabled_lock.lock_state == LockState.UNLOCKED

    def test_toggle_from_locked(self, enabled_lock: Lock) -> None:
        """Test toggle from locked state."""
        assert enabled_lock.is_locked

        enabled_lock.toggle()
        assert enabled_lock.is_unlocked

    def test_toggle_from_unlocked(self, enabled_lock: Lock) -> None:
        """Test toggle from unlocked state."""
        enabled_lock.unlock()
        assert enabled_lock.is_unlocked

        enabled_lock.toggle()
        assert enabled_lock.is_locked

    def test_lock_when_disabled_raises(self, lock: Lock) -> None:
        """Test that lock() raises when lock is disabled."""
        assert not lock.is_enabled
        with pytest.raises(RuntimeError, match="not enabled"):
            lock.lock()

    def test_unlock_when_disabled_raises(self, lock: Lock) -> None:
        """Test that unlock() raises when lock is disabled."""
        assert not lock.is_enabled
        with pytest.raises(RuntimeError, match="not enabled"):
            lock.unlock()


# =============================================================================
# State Tests
# =============================================================================


class TestLockState:
    """Tests for Lock state management."""

    def test_is_locked(self, enabled_lock: Lock) -> None:
        """Test is_locked returns True at locked position."""
        enabled_lock.lock()
        assert enabled_lock.is_locked
        assert not enabled_lock.is_unlocked
        assert enabled_lock.state_str == "locked"

    def test_is_unlocked(self, enabled_lock: Lock) -> None:
        """Test is_unlocked returns True at unlocked position."""
        enabled_lock.unlock()
        assert enabled_lock.is_unlocked
        assert not enabled_lock.is_locked
        assert enabled_lock.state_str == "unlocked"

    def test_state_property(self, enabled_lock: Lock) -> None:
        """Test state_str property returns correct string values."""
        # Locked state
        enabled_lock.lock()
        assert enabled_lock.state_str == "locked"

        # Unlocked state
        enabled_lock.unlock()
        assert enabled_lock.state_str == "unlocked"


# =============================================================================
# Async Tests
# =============================================================================


class TestLockAsync:
    """Tests for Lock async operations."""

    @pytest.mark.asyncio
    async def test_alock(self, enabled_lock: Lock) -> None:
        """Test async lock operation."""
        enabled_lock.unlock()
        assert enabled_lock.is_unlocked

        await enabled_lock.alock()
        assert enabled_lock.is_locked
        assert enabled_lock.lock_state == LockState.LOCKED

    @pytest.mark.asyncio
    async def test_aunlock(self, enabled_lock: Lock) -> None:
        """Test async unlock operation."""
        assert enabled_lock.is_locked

        await enabled_lock.aunlock()
        assert enabled_lock.is_unlocked
        assert enabled_lock.lock_state == LockState.UNLOCKED

    @pytest.mark.asyncio
    async def test_alock_when_disabled_raises(self, lock: Lock) -> None:
        """Test that alock() raises when lock is disabled."""
        with pytest.raises(RuntimeError, match="not enabled"):
            await lock.alock()

    @pytest.mark.asyncio
    async def test_aunlock_when_disabled_raises(self, lock: Lock) -> None:
        """Test that aunlock() raises when lock is disabled."""
        with pytest.raises(RuntimeError, match="not enabled"):
            await lock.aunlock()


# =============================================================================
# Enable/Disable Tests
# =============================================================================


class TestLockEnableDisable:
    """Tests for Lock enable/disable."""

    def test_enable(self, lock: Lock) -> None:
        """Test enabling lock."""
        assert not lock.is_enabled
        lock.enable()
        assert lock.is_enabled
        assert lock.lock_state != LockState.DISABLED

    def test_disable(self, enabled_lock: Lock) -> None:
        """Test disabling lock."""
        assert enabled_lock.is_enabled
        enabled_lock.disable()
        assert not enabled_lock.is_enabled
        assert enabled_lock.lock_state == LockState.DISABLED

    def test_enable_sets_initial_state_locked(self, lock: Lock) -> None:
        """Test that enable sets initial locked state."""
        assert lock.lock_config.start_locked is True
        lock.enable()
        assert lock.is_locked

    def test_enable_sets_initial_state_unlocked(self, lock_starts_unlocked: Lock) -> None:
        """Test that enable sets initial unlocked state."""
        assert lock_starts_unlocked.lock_config.start_locked is False
        lock_starts_unlocked.enable()
        assert lock_starts_unlocked.is_unlocked


# =============================================================================
# Config Tests
# =============================================================================


class TestLockConfig:
    """Tests for LockConfig."""

    def test_config_defaults(self) -> None:
        """Test config default values."""
        config = LockConfig(name="test")
        assert config.locked_position == 0.0
        assert config.unlocked_position == 90.0
        assert config.position_tolerance == 1.0
        assert config.transition_time == 0.5
        assert config.require_confirmation is False
        assert config.auto_lock_timeout is None
        assert config.start_locked is True

    def test_config_range_property(self) -> None:
        """Test config range calculation."""
        config = LockConfig(
            name="test",
            locked_position=10.0,
            unlocked_position=80.0,
        )
        assert config.range == 70.0

    def test_config_is_inverted_property(self) -> None:
        """Test config inverted detection."""
        # Normal config: locked < unlocked
        normal = LockConfig(name="normal", locked_position=0.0, unlocked_position=90.0)
        assert not normal.is_inverted

        # Inverted config: locked > unlocked
        inverted = LockConfig(name="inverted", locked_position=90.0, unlocked_position=0.0)
        assert inverted.is_inverted


# =============================================================================
# Additional Method Tests
# =============================================================================


class TestLockMethods:
    """Tests for additional Lock methods."""

    def test_stop(self, enabled_lock: Lock) -> None:
        """Test stop method."""
        enabled_lock.unlock()
        enabled_lock.stop()
        # State should be updated based on position
        assert enabled_lock.lock_state == LockState.UNLOCKED

    def test_repr(self, lock: Lock) -> None:
        """Test string representation."""
        repr_str = repr(lock)
        assert "Lock" in repr_str
        assert "test_lock" in repr_str

    def test_toggle_when_disabled_raises(self, lock: Lock) -> None:
        """Test that toggle() raises when lock is disabled."""
        with pytest.raises(RuntimeError, match="not enabled"):
            lock.toggle()

    def test_position_property(self, enabled_lock: Lock) -> None:
        """Test position property."""
        enabled_lock.lock()
        pos = enabled_lock.position
        assert isinstance(pos, float)


__all__ = [
    "TestLockAsync",
    "TestLockConfig",
    "TestLockEnableDisable",
    "TestLockInit",
    "TestLockMethods",
    "TestLockState",
    "TestLockUnlock",
]
