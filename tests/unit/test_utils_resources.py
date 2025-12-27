"""Tests for resource management utilities (Phase 5.16).

Tests for context managers, connection pooling, cleanup handlers, and memory management.
"""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock

import pytest

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


# =============================================================================
# Test Fixtures
# =============================================================================


class MockAsyncResource(AsyncContextManager):
    """Mock resource with async context manager support."""

    def __init__(self) -> None:
        self.connected = False
        self.connect_count = 0
        self.disconnect_count = 0

    async def connect_async(self) -> None:
        """Async connect."""
        self.connect_count += 1
        self.connected = True

    async def disconnect_async(self) -> None:
        """Async disconnect."""
        self.disconnect_count += 1
        self.connected = False


class MockManagedResource(ManagedResource):
    """Mock managed resource for testing."""

    def __init__(self, name: str | None = None) -> None:
        super().__init__(name)
        self.open_count = 0
        self.close_count = 0

    def _open(self) -> None:
        """Open the resource."""
        self.open_count += 1

    def _close(self) -> None:
        """Close the resource."""
        self.close_count += 1


class MockAsyncManagedResource(ManagedResource):
    """Mock managed resource with async support."""

    def __init__(self, name: str | None = None) -> None:
        super().__init__(name)
        self.open_count = 0
        self.close_count = 0
        self.async_open_count = 0
        self.async_close_count = 0

    def _open(self) -> None:
        """Open the resource."""
        self.open_count += 1

    def _close(self) -> None:
        """Close the resource."""
        self.close_count += 1

    async def _open_async(self) -> None:
        """Open the resource asynchronously."""
        self.async_open_count += 1

    async def _close_async(self) -> None:
        """Close the resource asynchronously."""
        self.async_close_count += 1


@pytest.fixture(autouse=True)
def reset_resource_manager() -> None:
    """Reset ResourceManager state before each test."""
    ResourceManager.reset()
    ConnectionPool._instances.clear()


# =============================================================================
# AsyncContextManager Tests
# =============================================================================


class TestAsyncContextManager:
    """Tests for AsyncContextManager mixin."""

    @pytest.mark.asyncio
    async def test_aenter_calls_connect_async(self) -> None:
        """Test __aenter__ calls connect_async."""
        resource = MockAsyncResource()
        assert not resource.connected

        result = await resource.__aenter__()

        assert result is resource
        assert resource.connected
        assert resource.connect_count == 1

    @pytest.mark.asyncio
    async def test_aexit_calls_disconnect_async(self) -> None:
        """Test __aexit__ calls disconnect_async."""
        resource = MockAsyncResource()
        await resource.__aenter__()
        assert resource.connected

        await resource.__aexit__(None, None, None)

        assert not resource.connected
        assert resource.disconnect_count == 1

    @pytest.mark.asyncio
    async def test_async_with_statement(self) -> None:
        """Test async context manager in with statement."""
        resource = MockAsyncResource()

        async with resource as r:
            assert r is resource
            assert resource.connected
            assert resource.connect_count == 1

        assert not resource.connected
        assert resource.disconnect_count == 1

    @pytest.mark.asyncio
    async def test_aexit_on_exception(self) -> None:
        """Test __aexit__ is called even on exception."""
        resource = MockAsyncResource()

        with pytest.raises(ValueError):
            async with resource:
                raise ValueError("test error")

        assert not resource.connected
        assert resource.disconnect_count == 1


# =============================================================================
# ManagedResource Tests
# =============================================================================


class TestManagedResource:
    """Tests for ManagedResource base class."""

    def test_init_sets_name(self) -> None:
        """Test resource name is set correctly."""
        resource = MockManagedResource(name="test_resource")
        assert resource._resource_name == "test_resource"

    def test_init_default_name(self) -> None:
        """Test default resource name is class name."""
        resource = MockManagedResource()
        assert resource._resource_name == "MockManagedResource"

    def test_is_open_initially_false(self) -> None:
        """Test resource is initially closed."""
        resource = MockManagedResource()
        assert not resource.is_open

    def test_open_sets_is_open(self) -> None:
        """Test open() sets is_open flag."""
        resource = MockManagedResource()
        resource.open()
        assert resource.is_open
        assert resource.open_count == 1

    def test_open_idempotent(self) -> None:
        """Test open() only opens once."""
        resource = MockManagedResource()
        resource.open()
        resource.open()
        assert resource.open_count == 1

    def test_close_clears_is_open(self) -> None:
        """Test close() clears is_open flag."""
        resource = MockManagedResource()
        resource.open()
        resource.close()
        assert not resource.is_open
        assert resource.close_count == 1

    def test_close_idempotent(self) -> None:
        """Test close() only closes once."""
        resource = MockManagedResource()
        resource.open()
        resource.close()
        resource.close()
        assert resource.close_count == 1

    def test_sync_context_manager(self) -> None:
        """Test sync context manager support."""
        resource = MockManagedResource()

        with resource as r:
            assert r is resource
            assert resource.is_open

        assert not resource.is_open
        assert resource.open_count == 1
        assert resource.close_count == 1

    @pytest.mark.asyncio
    async def test_async_context_manager(self) -> None:
        """Test async context manager support."""
        resource = MockAsyncManagedResource()

        async with resource as r:
            assert r is resource
            assert resource.is_open
            assert resource.async_open_count == 1

        assert not resource.is_open
        assert resource.async_close_count == 1

    @pytest.mark.asyncio
    async def test_open_async_default(self) -> None:
        """Test default async open calls sync version."""
        resource = MockManagedResource()
        await resource.open_async()
        assert resource.is_open
        assert resource.open_count == 1

    @pytest.mark.asyncio
    async def test_close_async_default(self) -> None:
        """Test default async close calls sync version."""
        resource = MockManagedResource()
        resource.open()
        await resource.close_async()
        assert not resource.is_open
        assert resource.close_count == 1


# =============================================================================
# PoolConfig Tests
# =============================================================================


class TestPoolConfig:
    """Tests for PoolConfig."""

    def test_default_values(self) -> None:
        """Test default configuration values."""
        config = PoolConfig()
        assert config.max_size == 10
        assert config.min_size == 1
        assert config.acquire_timeout == 5.0
        assert config.idle_timeout == 300.0

    def test_custom_values(self) -> None:
        """Test custom configuration values."""
        config = PoolConfig(
            max_size=5,
            min_size=2,
            acquire_timeout=10.0,
            idle_timeout=600.0,
        )
        assert config.max_size == 5
        assert config.min_size == 2
        assert config.acquire_timeout == 10.0
        assert config.idle_timeout == 600.0


# =============================================================================
# ConnectionPool Tests
# =============================================================================


class TestConnectionPool:
    """Tests for ConnectionPool."""

    def test_get_or_create_creates_new(self) -> None:
        """Test get_or_create creates new pool."""
        pool = ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: MagicMock(),
        )
        assert pool is not None
        assert pool._name == "test_pool"

    def test_get_or_create_returns_existing(self) -> None:
        """Test get_or_create returns existing pool."""
        pool1 = ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: MagicMock(),
        )
        pool2 = ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: MagicMock(),
        )
        assert pool1 is pool2

    def test_get_existing_pool(self) -> None:
        """Test get returns existing pool."""
        ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: MagicMock(),
        )
        pool = ConnectionPool.get("test_pool")
        assert pool is not None

    def test_get_nonexistent_pool(self) -> None:
        """Test get returns None for nonexistent pool."""
        pool = ConnectionPool.get("nonexistent")
        assert pool is None

    @pytest.mark.asyncio
    async def test_acquire_creates_connection(self) -> None:
        """Test acquire creates connection from factory."""
        factory = MagicMock(return_value="connection")
        pool = ConnectionPool.get_or_create("test_pool", factory=factory)

        async with pool.acquire() as conn:
            assert conn == "connection"
            factory.assert_called_once()

    @pytest.mark.asyncio
    async def test_acquire_reuses_connection(self) -> None:
        """Test acquire reuses returned connections."""
        factory = MagicMock(return_value="connection")
        pool = ConnectionPool.get_or_create("test_pool", factory=factory)

        async with pool.acquire() as conn1:
            assert conn1 == "connection"

        async with pool.acquire() as conn2:
            assert conn2 == "connection"

        # Factory should only be called once
        factory.assert_called_once()

    @pytest.mark.asyncio
    async def test_acquire_timeout(self) -> None:
        """Test acquire raises timeout when pool exhausted."""
        config = PoolConfig(max_size=1, acquire_timeout=0.1)
        pool = ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: MagicMock(),
            config=config,
        )

        async with pool.acquire():
            # Try to acquire second connection
            with pytest.raises(asyncio.TimeoutError):
                async with pool.acquire():
                    pass

    def test_acquire_sync(self) -> None:
        """Test synchronous acquire."""
        factory = MagicMock(return_value="connection")
        pool = ConnectionPool.get_or_create("test_pool", factory=factory)

        with pool.acquire_sync() as conn:
            assert conn == "connection"

    def test_size_and_in_use(self) -> None:
        """Test size and in_use properties."""
        pool = ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: MagicMock(),
        )

        assert pool.size == 0
        assert pool.in_use == 0

        with pool.acquire_sync():
            assert pool.in_use == 1

        assert pool.size == 1
        assert pool.in_use == 0

    def test_close_calls_closer(self) -> None:
        """Test close calls closer function."""
        conn = MagicMock()
        closer = MagicMock()
        pool = ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: conn,
            closer=closer,
        )

        with pool.acquire_sync():
            pass

        pool.close()
        closer.assert_called_once_with(conn)

    def test_close_all(self) -> None:
        """Test close_all closes all pools."""
        ConnectionPool.get_or_create("pool1", factory=lambda: MagicMock())
        ConnectionPool.get_or_create("pool2", factory=lambda: MagicMock())

        ConnectionPool.close_all()

        assert len(ConnectionPool._instances) == 0


# =============================================================================
# ResourceManager Tests
# =============================================================================


class TestResourceManager:
    """Tests for ResourceManager."""

    def test_register_resource(self) -> None:
        """Test registering a resource."""
        resource = MockManagedResource()
        # Resource is auto-registered in __init__
        assert resource in ResourceManager._resources

    def test_on_cleanup_registers_callback(self) -> None:
        """Test registering cleanup callback."""
        callback = MagicMock()
        ResourceManager.on_cleanup(callback)
        assert callback in ResourceManager._callbacks

    def test_on_cleanup_async_registers_callback(self) -> None:
        """Test registering async cleanup callback."""
        callback = AsyncMock()
        ResourceManager.on_cleanup_async(callback)
        assert callback in ResourceManager._async_callbacks

    def test_cleanup_calls_callbacks(self) -> None:
        """Test cleanup calls all callbacks."""
        callback1 = MagicMock()
        callback2 = MagicMock()
        ResourceManager.on_cleanup(callback1)
        ResourceManager.on_cleanup(callback2)

        ResourceManager.cleanup()

        callback1.assert_called_once()
        callback2.assert_called_once()

    def test_cleanup_closes_resources(self) -> None:
        """Test cleanup closes all resources."""
        resource = MockManagedResource()
        resource.open()

        ResourceManager.cleanup()

        assert resource.close_count == 1

    def test_cleanup_closes_pools(self) -> None:
        """Test cleanup closes connection pools."""
        ConnectionPool.get_or_create("test_pool", factory=lambda: MagicMock())

        ResourceManager.cleanup()

        assert len(ConnectionPool._instances) == 0

    def test_cleanup_handles_callback_errors(self) -> None:
        """Test cleanup continues on callback errors."""
        callback1 = MagicMock(side_effect=Exception("error"))
        callback2 = MagicMock()
        ResourceManager.on_cleanup(callback1)
        ResourceManager.on_cleanup(callback2)

        ResourceManager.cleanup()

        callback1.assert_called_once()
        callback2.assert_called_once()

    @pytest.mark.asyncio
    async def test_cleanup_async(self) -> None:
        """Test async cleanup."""
        sync_callback = MagicMock()
        async_callback = AsyncMock()
        ResourceManager.on_cleanup(sync_callback)
        ResourceManager.on_cleanup_async(async_callback)

        await ResourceManager.cleanup_async()

        sync_callback.assert_called_once()
        async_callback.assert_called_once()

    @pytest.mark.asyncio
    async def test_cleanup_async_closes_resources(self) -> None:
        """Test async cleanup closes resources."""
        resource = MockAsyncManagedResource()
        resource.open()

        await ResourceManager.cleanup_async()

        assert resource.async_close_count == 1

    def test_reset(self) -> None:
        """Test reset clears all state."""
        MockManagedResource()
        ResourceManager.on_cleanup(MagicMock())
        ResourceManager.on_cleanup_async(AsyncMock())

        ResourceManager.reset()

        assert len(ResourceManager._resources) == 0
        assert len(ResourceManager._callbacks) == 0
        assert len(ResourceManager._async_callbacks) == 0


# =============================================================================
# LimitedBuffer Tests
# =============================================================================


class TestLimitedBuffer:
    """Tests for LimitedBuffer."""

    def test_default_max_size(self) -> None:
        """Test default max size."""
        buffer: LimitedBuffer[int] = LimitedBuffer()
        assert buffer.max_size == 1000

    def test_custom_max_size(self) -> None:
        """Test custom max size."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=100)
        assert buffer.max_size == 100

    def test_append(self) -> None:
        """Test appending items."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        buffer.append(1)
        buffer.append(2)
        buffer.append(3)

        assert len(buffer) == 3
        assert list(buffer) == [1, 2, 3]

    def test_append_evicts_old(self) -> None:
        """Test appending evicts old items when full."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=3)
        buffer.append(1)
        buffer.append(2)
        buffer.append(3)
        buffer.append(4)

        assert len(buffer) == 3
        assert list(buffer) == [2, 3, 4]

    def test_extend(self) -> None:
        """Test extending with multiple items."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        buffer.extend([1, 2, 3])

        assert len(buffer) == 3
        assert list(buffer) == [1, 2, 3]

    def test_clear(self) -> None:
        """Test clearing buffer."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        buffer.extend([1, 2, 3])
        buffer.clear()

        assert len(buffer) == 0

    def test_last(self) -> None:
        """Test getting last n items."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        buffer.extend([1, 2, 3, 4, 5])

        assert buffer.last(3) == [3, 4, 5]
        assert buffer.last(1) == [5]
        assert buffer.last(10) == [1, 2, 3, 4, 5]

    def test_first(self) -> None:
        """Test getting first n items."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        buffer.extend([1, 2, 3, 4, 5])

        assert buffer.first(3) == [1, 2, 3]
        assert buffer.first(1) == [1]
        assert buffer.first(10) == [1, 2, 3, 4, 5]

    def test_len(self) -> None:
        """Test __len__."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        assert len(buffer) == 0

        buffer.extend([1, 2, 3])
        assert len(buffer) == 3

    def test_iter(self) -> None:
        """Test __iter__."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        buffer.extend([1, 2, 3])

        items = list(buffer)
        assert items == [1, 2, 3]

    def test_bool_empty(self) -> None:
        """Test __bool__ for empty buffer."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        assert not buffer

    def test_bool_nonempty(self) -> None:
        """Test __bool__ for non-empty buffer."""
        buffer: LimitedBuffer[int] = LimitedBuffer(max_size=10)
        buffer.append(1)
        assert buffer


# =============================================================================
# Convenience Function Tests
# =============================================================================


class TestConvenienceFunctions:
    """Tests for convenience functions."""

    def test_register_cleanup(self) -> None:
        """Test register_cleanup function."""
        callback = MagicMock()
        register_cleanup(callback)
        assert callback in ResourceManager._callbacks

    def test_register_cleanup_async(self) -> None:
        """Test register_cleanup_async function."""
        callback = AsyncMock()
        register_cleanup_async(callback)
        assert callback in ResourceManager._async_callbacks


# =============================================================================
# Integration Tests
# =============================================================================


class TestResourceIntegration:
    """Integration tests for resource management."""

    @pytest.mark.asyncio
    async def test_full_lifecycle(self) -> None:
        """Test full resource lifecycle."""
        # Create resources
        resource1 = MockManagedResource(name="resource1")
        resource2 = MockAsyncManagedResource(name="resource2")

        # Create pool
        pool = ConnectionPool.get_or_create(
            "test_pool",
            factory=lambda: MagicMock(),
        )

        # Register callbacks
        callback = MagicMock()
        ResourceManager.on_cleanup(callback)

        # Use resources
        resource1.open()
        resource2.open()
        with pool.acquire_sync():
            pass

        # Async cleanup
        await ResourceManager.cleanup_async()

        # Verify cleanup
        assert resource1.close_count == 1
        assert resource2.async_close_count == 1
        callback.assert_called_once()
        assert len(ConnectionPool._instances) == 0

    @pytest.mark.asyncio
    async def test_concurrent_pool_access(self) -> None:
        """Test concurrent access to connection pool."""
        access_count = 0

        def factory() -> int:
            nonlocal access_count
            access_count += 1
            return access_count

        pool = ConnectionPool.get_or_create(
            "test_pool",
            factory=factory,
            config=PoolConfig(max_size=3),
        )

        async def use_pool() -> int:
            async with pool.acquire() as conn:
                await asyncio.sleep(0.01)
                return conn

        # Run 5 concurrent tasks with max 3 connections
        tasks = [use_pool() for _ in range(5)]
        results = await asyncio.gather(*tasks)

        # Should have created at most 3 connections
        assert access_count <= 3
        # All tasks should complete
        assert len(results) == 5
