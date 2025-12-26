"""Resource management utilities for robo-infra.

This module provides utilities for managing hardware resources:
- Async context manager protocols
- Connection pooling for shared buses
- Cleanup handlers for graceful shutdown
- Memory management for sensor buffers

Example:
    >>> from robo_infra.utils.resources import (
    ...     ResourceManager,
    ...     ConnectionPool,
    ...     register_cleanup,
    ... )
    >>>
    >>> # Register cleanup handlers
    >>> register_cleanup(controller.emergency_stop)
    >>>
    >>> # Use connection pool for shared I2C bus
    >>> pool = ConnectionPool.get("i2c", bus_number=1)
    >>> async with pool.acquire() as bus:
    ...     await bus.read(address, register)
"""

from __future__ import annotations

import asyncio
import atexit
import logging
import signal
import weakref
from abc import ABC, abstractmethod
from collections import deque
from contextlib import asynccontextmanager, contextmanager
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, ClassVar, Generic, Self, TypeVar


if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable, Iterator
    from types import FrameType


logger = logging.getLogger(__name__)


# =============================================================================
# Type Variables
# =============================================================================

T = TypeVar("T")
ResourceT = TypeVar("ResourceT", bound="ManagedResource")


# =============================================================================
# Async Context Manager Protocol
# =============================================================================


class AsyncContextManager(ABC):
    """Mixin providing async context manager support.

    Add this to classes that manage hardware resources to enable
    async with statement support alongside sync context managers.

    Example:
        >>> class MyDriver(Driver, AsyncContextManager):
        ...     async def connect_async(self) -> None:
        ...         # Async initialization
        ...         pass
        ...
        ...     async def disconnect_async(self) -> None:
        ...         # Async cleanup
        ...         pass
        >>>
        >>> async with MyDriver() as driver:
        ...     await driver.do_something()
    """

    @abstractmethod
    async def connect_async(self) -> None:
        """Async connect to the resource."""
        ...

    @abstractmethod
    async def disconnect_async(self) -> None:
        """Async disconnect from the resource."""
        ...

    async def __aenter__(self) -> Self:
        """Async context manager entry."""
        await self.connect_async()
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: Any,
    ) -> bool:
        """Async context manager exit."""
        await self.disconnect_async()
        return False


# =============================================================================
# Managed Resource Base
# =============================================================================


class ManagedResource(ABC):
    """Base class for resources that need lifecycle management.

    Provides both sync and async context manager support, and
    automatically registers with the global ResourceManager.

    Subclasses must implement:
        - _open(): Open/connect the resource
        - _close(): Close/disconnect the resource

    Optionally implement for async support:
        - _open_async(): Async version of open
        - _close_async(): Async version of close
    """

    def __init__(self, name: str | None = None) -> None:
        """Initialize managed resource.

        Args:
            name: Optional name for logging/debugging.
        """
        self._resource_name = name or self.__class__.__name__
        self._is_open = False
        # Register with global manager for cleanup
        ResourceManager.register(self)

    @property
    def is_open(self) -> bool:
        """Whether the resource is currently open."""
        return self._is_open

    @abstractmethod
    def _open(self) -> None:
        """Open the resource (sync)."""
        ...

    @abstractmethod
    def _close(self) -> None:
        """Close the resource (sync)."""
        ...

    async def _open_async(self) -> None:
        """Open the resource (async). Default runs sync version."""
        self._open()

    async def _close_async(self) -> None:
        """Close the resource (async). Default runs sync version."""
        self._close()

    def open(self) -> None:
        """Open the resource."""
        if not self._is_open:
            self._open()
            self._is_open = True
            logger.debug("Opened resource: %s", self._resource_name)

    def close(self) -> None:
        """Close the resource."""
        if self._is_open:
            self._close()
            self._is_open = False
            logger.debug("Closed resource: %s", self._resource_name)

    async def open_async(self) -> None:
        """Open the resource asynchronously."""
        if not self._is_open:
            await self._open_async()
            self._is_open = True
            logger.debug("Opened resource (async): %s", self._resource_name)

    async def close_async(self) -> None:
        """Close the resource asynchronously."""
        if self._is_open:
            await self._close_async()
            self._is_open = False
            logger.debug("Closed resource (async): %s", self._resource_name)

    def __enter__(self) -> Self:
        """Sync context manager entry."""
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: Any,
    ) -> None:
        """Sync context manager exit."""
        self.close()

    async def __aenter__(self) -> Self:
        """Async context manager entry."""
        await self.open_async()
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: Any,
    ) -> bool:
        """Async context manager exit."""
        await self.close_async()
        return False


# =============================================================================
# Connection Pool
# =============================================================================


@dataclass
class PoolConfig:
    """Configuration for connection pools.

    Attributes:
        max_size: Maximum number of connections.
        min_size: Minimum connections to keep open.
        acquire_timeout: Timeout for acquiring a connection.
        idle_timeout: Time before idle connections are closed.
    """

    max_size: int = 10
    min_size: int = 1
    acquire_timeout: float = 5.0
    idle_timeout: float = 300.0  # 5 minutes


class ConnectionPool(Generic[T]):
    """Generic connection pool for shared resources.

    Manages a pool of connections (e.g., I2C buses, serial ports)
    that can be shared across multiple components.

    Example:
        >>> # Create a pool for I2C bus 1
        >>> pool = ConnectionPool.get_or_create(
        ...     "i2c_1",
        ...     factory=lambda: I2CBus(1),
        ...     closer=lambda bus: bus.close(),
        ... )
        >>>
        >>> async with pool.acquire() as bus:
        ...     data = await bus.read(0x40, 0x00, 2)
    """

    _instances: ClassVar[dict[str, ConnectionPool[Any]]] = {}
    _lock: ClassVar[asyncio.Lock | None] = None

    def __init__(
        self,
        name: str,
        factory: Callable[[], T],
        closer: Callable[[T], None] | None = None,
        config: PoolConfig | None = None,
    ) -> None:
        """Initialize connection pool.

        Args:
            name: Unique name for this pool.
            factory: Callable that creates new connections.
            closer: Optional callable to close connections.
            config: Pool configuration.
        """
        self._name = name
        self._factory = factory
        self._closer = closer
        self._config = config or PoolConfig()

        self._pool: deque[T] = deque()
        self._in_use: set[int] = set()  # Track by id()
        self._semaphore = asyncio.Semaphore(self._config.max_size)
        self._pool_lock = asyncio.Lock()

    @classmethod
    def get_or_create(
        cls,
        name: str,
        factory: Callable[[], T],
        closer: Callable[[T], None] | None = None,
        config: PoolConfig | None = None,
    ) -> ConnectionPool[T]:
        """Get existing pool or create new one.

        Args:
            name: Unique pool name.
            factory: Connection factory.
            closer: Connection closer.
            config: Pool configuration.

        Returns:
            Connection pool instance.
        """
        if name not in cls._instances:
            cls._instances[name] = ConnectionPool(name, factory, closer, config)
            logger.debug("Created connection pool: %s", name)
        return cls._instances[name]

    @classmethod
    def get(cls, name: str) -> ConnectionPool[T] | None:
        """Get existing pool by name.

        Args:
            name: Pool name.

        Returns:
            Pool if exists, None otherwise.
        """
        return cls._instances.get(name)

    @classmethod
    def close_all(cls) -> None:
        """Close all connection pools."""
        for _name, pool in list(cls._instances.items()):
            pool.close()
        cls._instances.clear()
        logger.info("Closed all connection pools")

    @asynccontextmanager
    async def acquire(self) -> AsyncIterator[T]:
        """Acquire a connection from the pool.

        Yields:
            Connection from the pool.

        Example:
            >>> async with pool.acquire() as conn:
            ...     await conn.execute(...)
        """
        await asyncio.wait_for(
            self._semaphore.acquire(),
            timeout=self._config.acquire_timeout,
        )

        try:
            async with self._pool_lock:
                if self._pool:
                    conn = self._pool.popleft()
                else:
                    conn = self._factory()

                self._in_use.add(id(conn))

            try:
                yield conn
            finally:
                async with self._pool_lock:
                    self._in_use.discard(id(conn))
                    self._pool.append(conn)
        finally:
            self._semaphore.release()

    @contextmanager
    def acquire_sync(self) -> Iterator[T]:
        """Acquire a connection synchronously.

        Yields:
            Connection from the pool.
        """
        # Simple sync version - no semaphore needed for sync
        if self._pool:
            conn = self._pool.popleft()
        else:
            conn = self._factory()

        self._in_use.add(id(conn))

        try:
            yield conn
        finally:
            self._in_use.discard(id(conn))
            self._pool.append(conn)

    @property
    def size(self) -> int:
        """Current number of connections in pool."""
        return len(self._pool)

    @property
    def in_use(self) -> int:
        """Number of connections currently in use."""
        return len(self._in_use)

    def close(self) -> None:
        """Close all connections in the pool."""
        while self._pool:
            conn = self._pool.popleft()
            if self._closer:
                try:
                    self._closer(conn)
                except Exception as e:
                    logger.warning("Error closing connection: %s", e)
        logger.debug("Closed connection pool: %s", self._name)


# =============================================================================
# Resource Manager (Global Cleanup)
# =============================================================================


class ResourceManager:
    """Global manager for tracking and cleaning up resources.

    Automatically registers resources for cleanup on process termination.
    Handles SIGTERM/SIGINT for graceful shutdown.

    Example:
        >>> # Resources auto-register when created
        >>> driver = MyDriver()  # Automatically tracked
        >>>
        >>> # Manual registration
        >>> ResourceManager.register(my_resource)
        >>>
        >>> # Add cleanup callback
        >>> ResourceManager.on_cleanup(controller.emergency_stop)
        >>>
        >>> # Trigger cleanup (automatic on exit/signal)
        >>> ResourceManager.cleanup()
    """

    _resources: ClassVar[weakref.WeakSet[ManagedResource]] = weakref.WeakSet()
    _callbacks: ClassVar[list[Callable[[], None]]] = []
    _async_callbacks: ClassVar[list[Callable[[], Awaitable[None]]]] = []
    _initialized: ClassVar[bool] = False
    _cleaning_up: ClassVar[bool] = False

    @classmethod
    def register(cls, resource: ManagedResource) -> None:
        """Register a resource for cleanup tracking.

        Args:
            resource: Resource to track.
        """
        cls._resources.add(resource)
        cls._ensure_initialized()

    @classmethod
    def on_cleanup(cls, callback: Callable[[], None]) -> None:
        """Register a callback to run on cleanup.

        Args:
            callback: Callable to invoke during cleanup.
        """
        cls._callbacks.append(callback)
        cls._ensure_initialized()

    @classmethod
    def on_cleanup_async(cls, callback: Callable[[], Awaitable[None]]) -> None:
        """Register an async callback to run on cleanup.

        Args:
            callback: Async callable to invoke during cleanup.
        """
        cls._async_callbacks.append(callback)
        cls._ensure_initialized()

    @classmethod
    def cleanup(cls) -> None:
        """Perform cleanup of all registered resources.

        Safe to call multiple times - will only run once.
        """
        if cls._cleaning_up:
            return
        cls._cleaning_up = True

        logger.info("Starting resource cleanup...")

        # Run registered callbacks
        for callback in cls._callbacks:
            try:
                callback()
            except Exception as e:
                logger.error("Cleanup callback error: %s", e)

        # Close all tracked resources
        for resource in list(cls._resources):
            try:
                resource.close()
            except Exception as e:
                logger.error("Error closing resource %s: %s", resource, e)

        # Close connection pools
        ConnectionPool.close_all()

        logger.info("Resource cleanup complete")
        cls._cleaning_up = False

    @classmethod
    async def cleanup_async(cls) -> None:
        """Perform async cleanup of all registered resources."""
        if cls._cleaning_up:
            return
        cls._cleaning_up = True

        logger.info("Starting async resource cleanup...")

        # Run sync callbacks first
        for callback in cls._callbacks:
            try:
                callback()
            except Exception as e:
                logger.error("Cleanup callback error: %s", e)

        # Run async callbacks
        for async_callback in cls._async_callbacks:
            try:
                awaitable = async_callback()
                await awaitable
            except Exception as e:
                logger.error("Async cleanup callback error: %s", e)

        # Close all tracked resources
        for resource in list(cls._resources):
            try:
                await resource.close_async()
            except Exception as e:
                logger.error("Error closing resource %s: %s", resource, e)

        # Close connection pools
        ConnectionPool.close_all()

        logger.info("Async resource cleanup complete")
        cls._cleaning_up = False

    @classmethod
    def _ensure_initialized(cls) -> None:
        """Ensure cleanup handlers are registered."""
        if cls._initialized:
            return

        # Register atexit handler
        atexit.register(cls.cleanup)

        # Register signal handlers
        def _signal_handler(signum: int, frame: FrameType | None) -> None:
            logger.warning("Received signal %d, cleaning up...", signum)
            cls.cleanup()
            # Re-raise to allow default handling
            signal.default_int_handler(signum, frame)

        # Only register if not already handled (e.g., in pytest)
        try:
            signal.signal(signal.SIGTERM, _signal_handler)
            signal.signal(signal.SIGINT, _signal_handler)
        except ValueError:
            # Signal handling only works in main thread
            logger.debug("Cannot register signal handlers (not main thread)")

        cls._initialized = True
        logger.debug("Resource manager initialized")

    @classmethod
    def reset(cls) -> None:
        """Reset the manager (mainly for testing)."""
        cls._resources = weakref.WeakSet()
        cls._callbacks = []
        cls._async_callbacks = []
        cls._initialized = False
        cls._cleaning_up = False


# =============================================================================
# Buffer with Size Limit
# =============================================================================


@dataclass
class LimitedBuffer(Generic[T]):
    """Buffer with a maximum size limit for memory management.

    Automatically evicts oldest items when limit is reached.
    Useful for sensor history buffers.

    Attributes:
        max_size: Maximum number of items to keep.
        items: The buffered items (deque).

    Example:
        >>> buffer = LimitedBuffer[float](max_size=100)
        >>> for reading in sensor_stream:
        ...     buffer.append(reading)  # Auto-evicts old readings
        >>> recent = buffer.last(10)  # Get last 10 readings
    """

    max_size: int = 1000
    items: deque[T] = field(default_factory=deque)

    def __post_init__(self) -> None:
        """Initialize with correct maxlen."""
        if not isinstance(self.items, deque):
            self.items = deque(self.items, maxlen=self.max_size)
        else:
            # Re-create with maxlen
            self.items = deque(self.items, maxlen=self.max_size)

    def append(self, item: T) -> None:
        """Append an item, evicting oldest if at capacity."""
        self.items.append(item)

    def extend(self, items: list[T]) -> None:
        """Extend with multiple items."""
        self.items.extend(items)

    def clear(self) -> None:
        """Clear all items."""
        self.items.clear()

    def last(self, n: int = 1) -> list[T]:
        """Get the last n items.

        Args:
            n: Number of items to get.

        Returns:
            List of the last n items.
        """
        return list(self.items)[-n:]

    def first(self, n: int = 1) -> list[T]:
        """Get the first n items.

        Args:
            n: Number of items to get.

        Returns:
            List of the first n items.
        """
        return list(self.items)[:n]

    def __len__(self) -> int:
        """Number of items in buffer."""
        return len(self.items)

    def __iter__(self) -> Iterator[T]:
        """Iterate over items."""
        return iter(self.items)

    def __bool__(self) -> bool:
        """True if buffer has items."""
        return bool(self.items)


# =============================================================================
# Convenience Functions
# =============================================================================


def register_cleanup(callback: Callable[[], None]) -> None:
    """Register a cleanup callback.

    Args:
        callback: Function to call on cleanup.

    Example:
        >>> register_cleanup(controller.emergency_stop)
    """
    ResourceManager.on_cleanup(callback)


def register_cleanup_async(callback: Callable[[], Awaitable[None]]) -> None:
    """Register an async cleanup callback.

    Args:
        callback: Async function to call on cleanup.

    Example:
        >>> register_cleanup_async(controller.safe_shutdown)
    """
    ResourceManager.on_cleanup_async(callback)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "AsyncContextManager",
    "ConnectionPool",
    "LimitedBuffer",
    "ManagedResource",
    "PoolConfig",
    "ResourceManager",
    "register_cleanup",
    "register_cleanup_async",
]
