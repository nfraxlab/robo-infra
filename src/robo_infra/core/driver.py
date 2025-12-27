"""Driver abstractions for hardware control.

This module provides abstract base classes and utilities for building
hardware drivers that control actuators and read sensors.

Example:
    >>> from robo_infra.core.driver import Driver, register_driver, get_driver
    >>>
    >>> @register_driver("my_pwm")
    >>> class MyPWMDriver(Driver):
    ...     def __init__(self, bus, address: int = 0x40):
    ...         super().__init__(name="MyPWM", channels=16)
    ...         self.bus = bus
    ...         self.address = address
    ...
    ...     def connect(self) -> None:
    ...         self.bus.open()
    ...         self._connected = True
    ...
    ...     def disconnect(self) -> None:
    ...         self.bus.close()
    ...         self._connected = False
    ...
    ...     def _write_channel(self, channel: int, value: float) -> None:
    ...         # Convert 0-1 to PWM register value
    ...         pwm_value = int(value * 4095)
    ...         self.bus.write_register(self.address, channel * 4, pwm_value)
    ...
    ...     def _read_channel(self, channel: int) -> float:
    ...         data = self.bus.read_register(self.address, channel * 4, 2)
    ...         return int.from_bytes(data, 'big') / 4095
    >>>
    >>> # Later, get the driver by name
    >>> driver_cls = get_driver("my_pwm")
    >>> driver = driver_cls(i2c_bus)
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, TypeVar


if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

from robo_infra.core.exceptions import (
    CommunicationError,
    DisabledError,
    HardwareNotFoundError,
)


logger = logging.getLogger(__name__)


# =============================================================================
# Type Variables
# =============================================================================

T = TypeVar("T", bound="Driver")


# =============================================================================
# Enums
# =============================================================================


class DriverState(Enum):
    """States a driver can be in."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"
    DISABLED = "disabled"


class ChannelMode(Enum):
    """Modes for driver channels."""

    OUTPUT = "output"
    INPUT = "input"
    PWM = "pwm"
    ANALOG = "analog"
    DISABLED = "disabled"


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class ChannelConfig:
    """Configuration for a single driver channel.

    Attributes:
        mode: Channel operating mode.
        min_value: Minimum allowed value.
        max_value: Maximum allowed value.
        default_value: Default value on initialization.
        inverted: If True, invert the value (1-value).
        name: Optional human-readable name.
        metadata: Additional channel-specific metadata.
    """

    mode: ChannelMode = ChannelMode.OUTPUT
    min_value: float = 0.0
    max_value: float = 1.0
    default_value: float = 0.0
    inverted: bool = False
    name: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class DriverConfig:
    """Configuration for a driver.

    Attributes:
        name: Human-readable driver name.
        channels: Number of channels.
        frequency: Operating frequency in Hz (for PWM drivers).
        channel_configs: Per-channel configurations.
        auto_connect: Whether to connect automatically on init.
        metadata: Additional driver-specific metadata.
    """

    name: str = "Driver"
    channels: int = 1
    frequency: int | None = None
    channel_configs: dict[int, ChannelConfig] = field(default_factory=dict)
    auto_connect: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)


# =============================================================================
# Driver Registry
# =============================================================================

_driver_registry: dict[str, type[Driver]] = {}


def register_driver(name: str) -> Callable[[type[T]], type[T]]:
    """Decorator to register a driver class.

    Args:
        name: Unique name for the driver.

    Returns:
        Decorator function.

    Example:
        >>> @register_driver("pca9685")
        >>> class PCA9685Driver(Driver):
        ...     pass
    """

    def decorator(cls: type[T]) -> type[T]:
        if name in _driver_registry:
            logger.warning("Overwriting existing driver registration: %s", name)
        _driver_registry[name] = cls
        logger.debug("Registered driver: %s -> %s", name, cls.__name__)
        return cls

    return decorator


def get_driver(name: str) -> type[Driver]:
    """Get a registered driver class by name.

    Args:
        name: Name of the registered driver.

    Returns:
        The driver class.

    Raises:
        HardwareNotFoundError: If driver is not registered.
    """
    if name not in _driver_registry:
        available = list(_driver_registry.keys())
        raise HardwareNotFoundError(f"Driver '{name}' not found. Available: {available}")
    return _driver_registry[name]


def list_drivers() -> list[str]:
    """List all registered driver names.

    Returns:
        List of registered driver names.
    """
    return list(_driver_registry.keys())


def clear_driver_registry() -> None:
    """Clear the driver registry. Mainly for testing."""
    _driver_registry.clear()


# =============================================================================
# Abstract Base Classes
# =============================================================================


class Driver(ABC):
    """Abstract base class for all hardware drivers.

    A driver provides a uniform interface for controlling hardware
    that has multiple channels (e.g., PWM controller, motor driver).

    Subclasses must implement:
        - connect(): Establish connection to hardware
        - disconnect(): Close connection
        - _write_channel(): Write value to a channel
        - _read_channel(): Read value from a channel
    """

    def __init__(
        self,
        name: str | None = None,
        channels: int = 1,
        config: DriverConfig | None = None,
    ) -> None:
        """Initialize the driver.

        Args:
            name: Human-readable name for the driver.
            channels: Number of channels this driver supports.
            config: Optional driver configuration.
        """
        self._config = config or DriverConfig(
            name=name or self.__class__.__name__,
            channels=channels,
        )
        self._name = self._config.name
        self._channels = self._config.channels
        self._state = DriverState.DISCONNECTED
        self._enabled = True
        self._frequency: int | None = self._config.frequency

        # Channel state
        self._channel_values: dict[int, float] = {}
        self._channel_configs: dict[int, ChannelConfig] = {}

        # Initialize channel configs
        for ch in range(self._channels):
            self._channel_configs[ch] = self._config.channel_configs.get(ch, ChannelConfig())
            self._channel_values[ch] = self._channel_configs[ch].default_value

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def name(self) -> str:
        """Human-readable name of the driver."""
        return self._name

    @property
    def channels(self) -> int:
        """Number of channels this driver supports."""
        return self._channels

    @property
    def state(self) -> DriverState:
        """Current driver state."""
        return self._state

    @property
    def is_connected(self) -> bool:
        """Whether the driver is connected to hardware."""
        return self._state == DriverState.CONNECTED

    @property
    def is_enabled(self) -> bool:
        """Whether the driver is enabled."""
        return self._enabled

    @property
    def frequency(self) -> int | None:
        """Operating frequency in Hz (for PWM drivers)."""
        return self._frequency

    @property
    def config(self) -> DriverConfig:
        """Driver configuration."""
        return self._config

    # -------------------------------------------------------------------------
    # Lifecycle Methods
    # -------------------------------------------------------------------------

    @abstractmethod
    def connect(self) -> None:
        """Connect to the hardware.

        Subclasses should:
        1. Initialize hardware communication
        2. Set self._state = DriverState.CONNECTED on success
        3. Raise CommunicationError on failure

        Raises:
            CommunicationError: If connection fails.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the hardware.

        Subclasses should:
        1. Clean up hardware resources
        2. Set self._state = DriverState.DISCONNECTED
        """
        ...

    def enable(self) -> None:
        """Enable the driver.

        When enabled, channel writes will be sent to hardware.
        """
        self._enabled = True
        if self._state == DriverState.DISABLED:
            self._state = DriverState.CONNECTED
        logger.debug("Driver %s enabled", self._name)

    def disable(self) -> None:
        """Disable the driver.

        When disabled, channel writes are ignored.
        Useful for emergency stops or safe mode.
        """
        self._enabled = False
        if self._state == DriverState.CONNECTED:
            self._state = DriverState.DISABLED
        logger.debug("Driver %s disabled", self._name)

    def __enter__(self) -> Driver:
        """Context manager entry - connect to hardware."""
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type | None,
        exc_val: Exception | None,
        exc_tb: object,
    ) -> None:
        """Context manager exit - disconnect from hardware."""
        self.disconnect()

    async def __aenter__(self) -> Driver:
        """Async context manager entry - connect to hardware.

        For drivers with async connect, override connect_async().
        Default implementation calls sync connect().
        """
        await self.connect_async()
        return self

    async def __aexit__(
        self,
        exc_type: type | None,
        exc_val: Exception | None,
        exc_tb: object,
    ) -> None:
        """Async context manager exit - disconnect from hardware."""
        await self.disconnect_async()

    async def connect_async(self) -> None:
        """Async version of connect.

        Override this for drivers that need async initialization.
        Default implementation calls sync connect().
        """
        self.connect()

    async def disconnect_async(self) -> None:
        """Async version of disconnect.

        Override this for drivers that need async cleanup.
        Default implementation calls sync disconnect().
        """
        self.disconnect()

    # -------------------------------------------------------------------------
    # Channel Operations
    # -------------------------------------------------------------------------

    def set_channel(self, channel: int, value: float) -> None:
        """Set a channel to a value.

        Args:
            channel: Channel number (0-indexed).
            value: Value to set (typically 0.0 to 1.0).

        Raises:
            DisabledError: If driver is disabled.
            ValueError: If channel is out of range.
            CommunicationError: If write fails.
        """
        self._validate_channel(channel)

        if not self._enabled:
            raise DisabledError(f"Driver {self._name} is disabled")

        # Apply channel config
        config = self._channel_configs.get(channel, ChannelConfig())
        clamped = max(config.min_value, min(config.max_value, value))

        if config.inverted:
            clamped = config.max_value - (clamped - config.min_value)

        # Store and write
        self._channel_values[channel] = clamped

        if self.is_connected:
            self._write_channel(channel, clamped)
            logger.debug(
                "Driver %s channel %d set to %.4f",
                self._name,
                channel,
                clamped,
            )

    def get_channel(self, channel: int) -> float:
        """Get the current value of a channel.

        Args:
            channel: Channel number (0-indexed).

        Returns:
            Current channel value.

        Raises:
            ValueError: If channel is out of range.
            CommunicationError: If read fails.
        """
        self._validate_channel(channel)

        if self.is_connected:
            value = self._read_channel(channel)
            self._channel_values[channel] = value
            return value

        return self._channel_values.get(channel, 0.0)

    def set_all_channels(self, value: float) -> None:
        """Set all channels to the same value.

        Args:
            value: Value to set on all channels.
        """
        for ch in range(self._channels):
            self.set_channel(ch, value)

    def get_all_channels(self) -> dict[int, float]:
        """Get values of all channels.

        Returns:
            Dictionary mapping channel numbers to values.
        """
        return {ch: self.get_channel(ch) for ch in range(self._channels)}

    def get_channel_config(self, channel: int) -> ChannelConfig:
        """Get configuration for a channel.

        Args:
            channel: Channel number.

        Returns:
            Channel configuration.
        """
        self._validate_channel(channel)
        return self._channel_configs.get(channel, ChannelConfig())

    def set_channel_config(self, channel: int, config: ChannelConfig) -> None:
        """Set configuration for a channel.

        Args:
            channel: Channel number.
            config: New channel configuration.
        """
        self._validate_channel(channel)
        self._channel_configs[channel] = config

    # -------------------------------------------------------------------------
    # Frequency Control
    # -------------------------------------------------------------------------

    def set_frequency(self, frequency: int) -> None:
        """Set the operating frequency.

        For PWM drivers, this sets the PWM frequency.

        Args:
            frequency: Frequency in Hz.
        """
        self._frequency = frequency
        if self.is_connected:
            self._apply_frequency(frequency)
        logger.debug("Driver %s frequency set to %d Hz", self._name, frequency)

    def _apply_frequency(self, frequency: int) -> None:  # noqa: B027
        """Apply frequency to hardware.

        Override in subclasses that support frequency control.

        Args:
            frequency: Frequency in Hz.
        """

    # -------------------------------------------------------------------------
    # Abstract Methods for Subclasses
    # -------------------------------------------------------------------------

    @abstractmethod
    def _write_channel(self, channel: int, value: float) -> None:
        """Write a value to a hardware channel.

        Subclasses must implement this to send values to hardware.

        Args:
            channel: Channel number (already validated).
            value: Value to write (already clamped/inverted).
        """
        ...

    @abstractmethod
    def _read_channel(self, channel: int) -> float:
        """Read a value from a hardware channel.

        Subclasses must implement this to read from hardware.

        Args:
            channel: Channel number (already validated).

        Returns:
            Current channel value.
        """
        ...

    # -------------------------------------------------------------------------
    # Validation Helpers
    # -------------------------------------------------------------------------

    def _validate_channel(self, channel: int) -> None:
        """Validate that a channel number is in range.

        Args:
            channel: Channel number to validate.

        Raises:
            ValueError: If channel is out of range.
        """
        if not 0 <= channel < self._channels:
            raise ValueError(
                f"Channel {channel} out of range for {self._name} (0-{self._channels - 1})"
            )


# =============================================================================
# Simulated Driver
# =============================================================================


class SimulatedDriver(Driver):
    """A simulated driver for testing without hardware.

    All channel operations work in memory only.

    Example:
        >>> driver = SimulatedDriver(name="TestDriver", channels=8)
        >>> driver.connect()
        >>> driver.set_channel(0, 0.5)
        >>> print(driver.get_channel(0))
        0.5
    """

    def __init__(
        self,
        name: str | None = None,
        channels: int = 16,
        config: DriverConfig | None = None,
    ) -> None:
        """Initialize simulated driver.

        Args:
            name: Driver name.
            channels: Number of channels.
            config: Driver configuration.
        """
        super().__init__(name or "SimulatedDriver", channels, config)

    def connect(self) -> None:
        """Simulate connecting to hardware."""
        self._state = DriverState.CONNECTED
        logger.debug("Simulated driver %s connected", self._name)

    def disconnect(self) -> None:
        """Simulate disconnecting from hardware."""
        self._state = DriverState.DISCONNECTED
        logger.debug("Simulated driver %s disconnected", self._name)

    def _write_channel(self, channel: int, value: float) -> None:
        """Simulate writing to channel (stores in memory)."""
        logger.debug(
            "Simulated write: %s channel %d = %.4f",
            self._name,
            channel,
            value,
        )

    def _read_channel(self, channel: int) -> float:
        """Simulate reading from channel (returns stored value)."""
        return self._channel_values.get(channel, 0.0)

    def _apply_frequency(self, frequency: int) -> None:
        """Simulate applying frequency."""
        logger.debug("Simulated frequency: %s = %d Hz", self._name, frequency)


# =============================================================================
# Multi-Driver Manager
# =============================================================================


class DriverManager:
    """Manages multiple drivers as a group.

    Useful for systems with multiple driver boards that need
    coordinated control.

    Example:
        >>> manager = DriverManager()
        >>> manager.add_driver("pwm1", driver1)
        >>> manager.add_driver("pwm2", driver2)
        >>> manager.connect_all()
        >>> manager.set_channel("pwm1", 0, 0.5)
    """

    def __init__(self) -> None:
        """Initialize the driver manager."""
        self._drivers: dict[str, Driver] = {}

    def add_driver(self, name: str, driver: Driver) -> None:
        """Add a driver to the manager.

        Args:
            name: Unique name for the driver.
            driver: Driver instance.
        """
        if name in self._drivers:
            logger.warning("Replacing existing driver: %s", name)
        self._drivers[name] = driver
        logger.debug("Added driver %s to manager", name)

    def remove_driver(self, name: str) -> Driver | None:
        """Remove a driver from the manager.

        Args:
            name: Driver name.

        Returns:
            The removed driver, or None if not found.
        """
        return self._drivers.pop(name, None)

    def get_driver(self, name: str) -> Driver:
        """Get a driver by name.

        Args:
            name: Driver name.

        Returns:
            The driver.

        Raises:
            HardwareNotFoundError: If driver not found.
        """
        if name not in self._drivers:
            raise HardwareNotFoundError(f"Driver '{name}' not in manager")
        return self._drivers[name]

    def list_drivers(self) -> list[str]:
        """List all driver names.

        Returns:
            List of driver names.
        """
        return list(self._drivers.keys())

    def connect_all(self) -> None:
        """Connect all drivers."""
        for name, driver in self._drivers.items():
            try:
                driver.connect()
                logger.info("Connected driver: %s", name)
            except CommunicationError as e:
                logger.error("Failed to connect driver %s: %s", name, e)
                raise

    def disconnect_all(self) -> None:
        """Disconnect all drivers."""
        for name, driver in self._drivers.items():
            try:
                driver.disconnect()
                logger.info("Disconnected driver: %s", name)
            except Exception as e:
                logger.error("Error disconnecting driver %s: %s", name, e)

    def enable_all(self) -> None:
        """Enable all drivers."""
        for driver in self._drivers.values():
            driver.enable()

    def disable_all(self) -> None:
        """Disable all drivers (emergency stop)."""
        for driver in self._drivers.values():
            driver.disable()

    def set_channel(self, driver_name: str, channel: int, value: float) -> None:
        """Set a channel on a specific driver.

        Args:
            driver_name: Name of the driver.
            channel: Channel number.
            value: Value to set.
        """
        driver = self.get_driver(driver_name)
        driver.set_channel(channel, value)

    def get_channel(self, driver_name: str, channel: int) -> float:
        """Get a channel value from a specific driver.

        Args:
            driver_name: Name of the driver.
            channel: Channel number.

        Returns:
            Channel value.
        """
        driver = self.get_driver(driver_name)
        return driver.get_channel(channel)

    def __enter__(self) -> DriverManager:
        """Context manager entry - connect all drivers."""
        self.connect_all()
        return self

    def __exit__(
        self,
        exc_type: type | None,
        exc_val: Exception | None,
        exc_tb: object,
    ) -> None:
        """Context manager exit - disconnect all drivers."""
        self.disconnect_all()

    def __len__(self) -> int:
        """Number of drivers in manager."""
        return len(self._drivers)

    def __iter__(self) -> Iterator[str]:
        """Iterate over driver names."""
        return iter(self._drivers)

    def __contains__(self, name: str) -> bool:
        """Check if driver exists."""
        return name in self._drivers
