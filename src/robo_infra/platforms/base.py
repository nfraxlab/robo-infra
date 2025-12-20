"""Platform abstraction layer for hardware-agnostic robotics.

This module provides the core abstractions for platform-specific implementations.
Each platform (Raspberry Pi, Jetson, Arduino, etc.) implements the Platform
protocol to provide hardware access in a consistent way.

Example:
    >>> from robo_infra.platforms import get_platform, PlatformRegistry
    >>>
    >>> # Auto-detect current platform
    >>> platform = get_platform()
    >>> print(f"Running on: {platform.name}")
    >>>
    >>> # Get a GPIO pin
    >>> pin = platform.get_pin(17)
    >>> pin.setup()
    >>> pin.high()
    >>>
    >>> # Get an I2C bus
    >>> i2c = platform.get_bus("i2c", bus=1)
    >>> devices = i2c.scan()
    >>>
    >>> # Override platform via environment variable
    >>> # ROBO_PLATFORM=simulation python my_robot.py
"""

from __future__ import annotations

import logging
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from pydantic import BaseModel

from robo_infra.core.exceptions import HardwareNotFoundError


if TYPE_CHECKING:
    from robo_infra.core.bus import Bus
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import Pin


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class PlatformType(Enum):
    """Known platform types."""

    # Single Board Computers
    RASPBERRY_PI = "raspberry_pi"
    JETSON = "jetson"
    BEAGLEBONE = "beaglebone"
    ORANGE_PI = "orange_pi"
    ROCK_PI = "rock_pi"
    PINE64 = "pine64"

    # Microcontrollers (via USB/Serial/WiFi)
    ARDUINO = "arduino"
    ESP32 = "esp32"
    ESP8266 = "esp8266"
    MICROBIT = "microbit"
    PICO = "pico"  # Raspberry Pi Pico

    # Generic/Fallback
    LINUX_GENERIC = "linux_generic"  # Any Linux with /dev/gpiochip*
    SIMULATION = "simulation"  # No hardware, pure simulation
    UNKNOWN = "unknown"


class PlatformCapability(Enum):
    """Capabilities a platform may support."""

    GPIO = "gpio"
    PWM = "pwm"
    HARDWARE_PWM = "hardware_pwm"
    I2C = "i2c"
    SPI = "spi"
    UART = "uart"
    CAN = "can"
    ADC = "adc"
    DAC = "dac"
    ONEWIRE = "onewire"
    CAMERA_CSI = "camera_csi"
    CAMERA_USB = "camera_usb"


# =============================================================================
# Configuration
# =============================================================================


class PlatformConfig(BaseModel):
    """Configuration for a platform.

    Attributes:
        name: Human-readable platform name.
        platform_type: Type of platform.
        auto_detect: Whether to auto-detect hardware.
        simulation_fallback: If True, fall back to simulation when hardware not found.
        pin_numbering: Pin numbering scheme ("BCM", "BOARD", "GPIO", etc.).
    """

    name: str = "Platform"
    platform_type: PlatformType = PlatformType.UNKNOWN
    auto_detect: bool = True
    simulation_fallback: bool = True
    pin_numbering: str = "BCM"  # BCM, BOARD, GPIO, TEGRA_SOC, etc.

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class PlatformInfo:
    """Information about a detected platform.

    Attributes:
        platform_type: Type of platform detected.
        model: Specific model (e.g., "Raspberry Pi 4 Model B").
        revision: Hardware revision code.
        serial: Hardware serial number (if available).
        capabilities: Set of supported capabilities.
        gpio_chips: List of available GPIO chips.
        i2c_buses: List of available I2C bus numbers.
        spi_buses: List of available SPI bus numbers.
        uart_ports: List of available UART port paths.
    """

    platform_type: PlatformType
    model: str = "Unknown"
    revision: str = ""
    serial: str = ""
    capabilities: set[PlatformCapability] = field(default_factory=set)
    gpio_chips: list[str] = field(default_factory=list)
    i2c_buses: list[int] = field(default_factory=list)
    spi_buses: list[tuple[int, int]] = field(default_factory=list)
    uart_ports: list[str] = field(default_factory=list)


# =============================================================================
# Platform Protocol
# =============================================================================


@runtime_checkable
class Platform(Protocol):
    """Protocol defining the interface for all platforms.

    Every platform implementation must provide these properties and methods
    to enable hardware-agnostic robot control.
    """

    @property
    def name(self) -> str:
        """Human-readable name of the platform."""
        ...

    @property
    def platform_type(self) -> PlatformType:
        """Type of this platform."""
        ...

    @property
    def is_available(self) -> bool:
        """Check if this platform is available (hardware detected)."""
        ...

    @property
    def info(self) -> PlatformInfo:
        """Get detailed platform information."""
        ...

    @property
    def capabilities(self) -> set[PlatformCapability]:
        """Get supported capabilities."""
        ...

    def get_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Get a GPIO pin by ID.

        Args:
            pin_id: Pin number or name (platform-specific).
            **kwargs: Additional pin configuration.

        Returns:
            Pin instance for the specified GPIO.

        Raises:
            HardwareNotFoundError: If pin is not available.
        """
        ...

    def get_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Get a communication bus.

        Args:
            bus_type: Type of bus ("i2c", "spi", "uart", "can").
            **kwargs: Bus-specific configuration.

        Returns:
            Bus instance for communication.

        Raises:
            HardwareNotFoundError: If bus is not available.
        """
        ...

    def get_driver(self, driver_type: str, **kwargs: Any) -> Driver:
        """Get a hardware driver.

        Args:
            driver_type: Type of driver to create.
            **kwargs: Driver-specific configuration.

        Returns:
            Driver instance.

        Raises:
            HardwareNotFoundError: If driver cannot be created.
        """
        ...

    def cleanup(self) -> None:
        """Release all hardware resources.

        Should be called when shutting down to ensure clean hardware state.
        """
        ...


# =============================================================================
# Abstract Base Platform
# =============================================================================


class BasePlatform(ABC):
    """Abstract base class implementing common platform functionality.

    Provides default implementations for common operations while requiring
    subclasses to implement platform-specific hardware access.
    """

    def __init__(self, config: PlatformConfig | None = None) -> None:
        """Initialize the platform.

        Args:
            config: Platform configuration. Uses defaults if None.
        """
        self._config = config or PlatformConfig()
        self._pins: dict[int | str, Pin] = {}
        self._buses: dict[str, Bus] = {}
        self._drivers: dict[str, Driver] = {}
        self._info: PlatformInfo | None = None
        self._initialized = False

        logger.debug("Initializing platform: %s", self._config.name)

    @property
    def name(self) -> str:
        """Human-readable name of the platform."""
        return self._config.name

    @property
    def platform_type(self) -> PlatformType:
        """Type of this platform."""
        return self._config.platform_type

    @property
    @abstractmethod
    def is_available(self) -> bool:
        """Check if this platform is available (hardware detected)."""
        ...

    @property
    def info(self) -> PlatformInfo:
        """Get detailed platform information."""
        if self._info is None:
            self._info = self._detect_info()
        return self._info

    @property
    def capabilities(self) -> set[PlatformCapability]:
        """Get supported capabilities."""
        return self.info.capabilities

    @abstractmethod
    def _detect_info(self) -> PlatformInfo:
        """Detect platform information.

        Subclasses must implement to detect hardware capabilities.
        """
        ...

    @abstractmethod
    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create a platform-specific pin.

        Subclasses must implement to create actual hardware pins.
        """
        ...

    @abstractmethod
    def _create_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Create a platform-specific bus.

        Subclasses must implement to create actual hardware buses.
        """
        ...

    def get_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Get a GPIO pin by ID.

        Caches pins to prevent duplicate hardware access.
        """
        # Check cache first
        cache_key = pin_id
        if cache_key in self._pins:
            return self._pins[cache_key]

        # Create new pin
        pin = self._create_pin(pin_id, **kwargs)
        self._pins[cache_key] = pin

        logger.debug("Created pin %s on %s", pin_id, self.name)
        return pin

    def get_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Get a communication bus.

        Caches buses to prevent duplicate hardware access.
        """
        # Create cache key from bus type and config
        cache_key = f"{bus_type}_{hash(frozenset(kwargs.items()))}"
        if cache_key in self._buses:
            return self._buses[cache_key]

        # Create new bus
        bus = self._create_bus(bus_type, **kwargs)
        self._buses[cache_key] = bus

        logger.debug("Created %s bus on %s", bus_type, self.name)
        return bus

    def get_driver(self, driver_type: str, **kwargs: Any) -> Driver:
        """Get a hardware driver.

        Uses the driver registry to create platform-appropriate drivers.
        """
        from robo_infra.core.driver import get_driver

        # Get driver class from registry
        driver_cls = get_driver(driver_type)

        # Create driver instance
        driver = driver_cls(**kwargs)
        self._drivers[driver_type] = driver

        logger.debug("Created %s driver on %s", driver_type, self.name)
        return driver

    def cleanup(self) -> None:
        """Release all hardware resources."""
        logger.info("Cleaning up platform: %s", self.name)

        # Cleanup drivers
        for name, driver in self._drivers.items():
            try:
                if hasattr(driver, "disconnect"):
                    driver.disconnect()
            except Exception as e:
                logger.warning("Error cleaning up driver %s: %s", name, e)

        # Cleanup buses
        for name, bus in self._buses.items():
            try:
                if hasattr(bus, "close"):
                    bus.close()
            except Exception as e:
                logger.warning("Error cleaning up bus %s: %s", name, e)

        # Cleanup pins
        for pin_id, pin in self._pins.items():
            try:
                pin.cleanup()
            except Exception as e:
                logger.warning("Error cleaning up pin %s: %s", pin_id, e)

        self._drivers.clear()
        self._buses.clear()
        self._pins.clear()
        self._initialized = False

    def __enter__(self) -> BasePlatform:
        """Context manager entry."""
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Context manager exit - cleanup resources."""
        self.cleanup()

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name={self.name!r}, type={self.platform_type.value})"


# =============================================================================
# Simulation Platform
# =============================================================================


class SimulationPlatform(BasePlatform):
    """Simulation platform for testing without hardware.

    Provides simulated pins, buses, and drivers for development and testing.
    This is the fallback when no hardware is detected or ROBO_SIMULATION=true.
    """

    def __init__(self, config: PlatformConfig | None = None) -> None:
        """Initialize simulation platform."""
        if config is None:
            config = PlatformConfig(
                name="Simulation",
                platform_type=PlatformType.SIMULATION,
            )
        super().__init__(config)
        logger.info("⚠️ SIMULATION MODE - No real hardware")

    @property
    def is_available(self) -> bool:
        """Simulation is always available."""
        return True

    def _detect_info(self) -> PlatformInfo:
        """Return simulated platform info."""
        return PlatformInfo(
            platform_type=PlatformType.SIMULATION,
            model="Simulated Platform",
            capabilities={
                PlatformCapability.GPIO,
                PlatformCapability.PWM,
                PlatformCapability.HARDWARE_PWM,
                PlatformCapability.I2C,
                PlatformCapability.SPI,
                PlatformCapability.UART,
                PlatformCapability.ADC,
                PlatformCapability.DAC,
            },
            gpio_chips=["gpiochip0"],
            i2c_buses=[0, 1],
            spi_buses=[(0, 0), (0, 1)],
            uart_ports=["/dev/ttyS0"],
        )

    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create a simulated pin."""
        from robo_infra.core.pin import PinMode, SimulatedDigitalPin, SimulatedPWMPin

        mode = kwargs.get("mode", PinMode.OUTPUT)
        pin_num = int(pin_id) if isinstance(pin_id, int | str) else 0

        # Use PWM pin if PWM mode requested, otherwise digital
        if mode == PinMode.PWM:
            return SimulatedPWMPin(
                number=pin_num,
                frequency=kwargs.get("frequency", 50),
                name=kwargs.get("name"),
            )
        else:
            return SimulatedDigitalPin(
                number=pin_num,
                mode=mode,
                name=kwargs.get("name"),
                inverted=kwargs.get("inverted", False),
            )

    def _create_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Create a simulated bus."""
        from robo_infra.core.bus import (
            I2CConfig,
            SerialConfig,
            SimulatedI2CBus,
            SimulatedSerialBus,
            SimulatedSPIBus,
            SPIConfig,
        )

        bus_type_lower = bus_type.lower()

        if bus_type_lower == "i2c":
            bus_num = kwargs.get("bus", 1)
            i2c_config = I2CConfig(bus_number=bus_num)
            return SimulatedI2CBus(config=i2c_config)
        elif bus_type_lower == "spi":
            bus_num = kwargs.get("bus", 0)
            device = kwargs.get("device", 0)
            spi_config = SPIConfig(bus=bus_num, device=device)
            return SimulatedSPIBus(config=spi_config)
        elif bus_type_lower in ("uart", "serial"):
            port = kwargs.get("port", "/dev/ttyS0")
            baudrate = kwargs.get("baudrate", 9600)
            serial_config = SerialConfig(port=port, baudrate=baudrate)
            return SimulatedSerialBus(config=serial_config)
        else:
            raise HardwareNotFoundError(
                device=f"Bus type: {bus_type}",
                details="Supported types: i2c, spi, uart, serial",
            )


# =============================================================================
# Platform Registry
# =============================================================================


class PlatformRegistry:
    """Registry for platform implementations.

    Maintains a list of available platforms and provides auto-detection.

    Example:
        >>> registry = PlatformRegistry()
        >>> registry.register(RaspberryPiPlatform)
        >>> registry.register(JetsonPlatform)
        >>>
        >>> # Auto-detect available platform
        >>> platform = registry.detect()
        >>> print(f"Detected: {platform.name}")
    """

    _instance: PlatformRegistry | None = None

    def __init__(self) -> None:
        """Initialize the registry."""
        self._platforms: dict[PlatformType, type[BasePlatform]] = {}
        self._detection_order: list[PlatformType] = []

        # Always register simulation platform
        self.register(SimulationPlatform, PlatformType.SIMULATION)

    @classmethod
    def get_instance(cls) -> PlatformRegistry:
        """Get the singleton registry instance."""
        if cls._instance is None:
            cls._instance = PlatformRegistry()
        return cls._instance

    def register(
        self,
        platform_class: type[BasePlatform],
        platform_type: PlatformType,
        *,
        priority: int | None = None,
    ) -> None:
        """Register a platform implementation.

        Args:
            platform_class: The platform class to register.
            platform_type: Type identifier for the platform.
            priority: Detection priority (lower = checked first).
        """
        self._platforms[platform_type] = platform_class

        if platform_type not in self._detection_order:
            if priority is not None:
                self._detection_order.insert(priority, platform_type)
            # Add before simulation (which should be last)
            elif PlatformType.SIMULATION in self._detection_order:
                idx = self._detection_order.index(PlatformType.SIMULATION)
                self._detection_order.insert(idx, platform_type)
            else:
                self._detection_order.append(platform_type)

        logger.debug(
            "Registered platform %s (%s)",
            platform_type.value,
            platform_class.__name__,
        )

    def unregister(self, platform_type: PlatformType) -> None:
        """Unregister a platform implementation."""
        if platform_type in self._platforms:
            del self._platforms[platform_type]
        if platform_type in self._detection_order:
            self._detection_order.remove(platform_type)

    def get(self, platform_type: PlatformType) -> type[BasePlatform] | None:
        """Get a platform class by type."""
        return self._platforms.get(platform_type)

    def detect(self, *, force_type: PlatformType | None = None) -> BasePlatform:
        """Detect and return the appropriate platform.

        Args:
            force_type: Force a specific platform type (ignores detection).

        Returns:
            Platform instance for the detected/forced platform.

        Raises:
            HardwareNotFoundError: If no suitable platform found.
        """
        # Check environment variable override
        env_platform = os.environ.get("ROBO_PLATFORM", "").lower()
        if env_platform:
            # Handle common aliases
            platform_map = {
                "sim": PlatformType.SIMULATION,
                "simulation": PlatformType.SIMULATION,
                "rpi": PlatformType.RASPBERRY_PI,
                "raspberry_pi": PlatformType.RASPBERRY_PI,
                "raspberrypi": PlatformType.RASPBERRY_PI,
                "jetson": PlatformType.JETSON,
                "arduino": PlatformType.ARDUINO,
                "esp32": PlatformType.ESP32,
                "beaglebone": PlatformType.BEAGLEBONE,
                "linux": PlatformType.LINUX_GENERIC,
            }
            force_type = platform_map.get(env_platform)
            if force_type is None:
                # Try to parse as PlatformType value
                try:
                    force_type = PlatformType(env_platform)
                except ValueError:
                    logger.warning(
                        "Unknown ROBO_PLATFORM value: %s, using auto-detection",
                        env_platform,
                    )

        # Force simulation mode check
        if os.environ.get("ROBO_SIMULATION", "").lower() in ("1", "true", "yes"):
            force_type = PlatformType.SIMULATION

        # If forced, return that platform
        if force_type is not None:
            platform_cls = self._platforms.get(force_type)
            if platform_cls is not None:
                logger.info("Using forced platform: %s", force_type.value)
                return platform_cls()
            else:
                logger.warning(
                    "Forced platform %s not registered, falling back to detection",
                    force_type.value,
                )

        # Auto-detect by trying each platform in order
        for platform_type in self._detection_order:
            if platform_type == PlatformType.SIMULATION:
                continue  # Skip simulation in detection (it's the fallback)

            platform_cls = self._platforms.get(platform_type)
            if platform_cls is None:
                continue

            try:
                platform = platform_cls()
                if platform.is_available:
                    logger.info("Detected platform: %s", platform.name)
                    return platform
            except Exception as e:
                logger.debug(
                    "Platform %s not available: %s",
                    platform_type.value,
                    e,
                )

        # Fall back to simulation
        logger.info("No hardware detected, using simulation platform")
        return SimulationPlatform()

    def list_platforms(self) -> list[PlatformType]:
        """List all registered platform types."""
        return list(self._platforms.keys())

    def list_available(self) -> list[tuple[PlatformType, str]]:
        """List all platforms and their availability.

        Returns:
            List of (platform_type, status) tuples.
        """
        result = []
        for platform_type in self._detection_order:
            platform_cls = self._platforms.get(platform_type)
            if platform_cls is None:
                result.append((platform_type, "not registered"))
                continue

            try:
                platform = platform_cls()
                status = "available" if platform.is_available else "not available"
            except Exception as e:
                status = f"error: {e}"

            result.append((platform_type, status))

        return result


# =============================================================================
# Module-level functions
# =============================================================================

# Singleton platform instance
_current_platform: BasePlatform | None = None


def get_platform(*, force_type: PlatformType | None = None) -> BasePlatform:
    """Get the current platform instance.

    Auto-detects the platform on first call and caches the result.
    Subsequent calls return the same platform instance.

    Args:
        force_type: Force a specific platform type.

    Returns:
        Platform instance.

    Example:
        >>> platform = get_platform()
        >>> print(f"Running on: {platform.name}")
        >>> pin = platform.get_pin(17)
    """
    global _current_platform  # noqa: PLW0603

    if _current_platform is None or force_type is not None:
        registry = PlatformRegistry.get_instance()
        _current_platform = registry.detect(force_type=force_type)

    return _current_platform


def reset_platform() -> None:
    """Reset the current platform.

    Cleans up resources and clears the cached platform.
    Next call to get_platform() will re-detect.
    """
    global _current_platform  # noqa: PLW0603

    if _current_platform is not None:
        _current_platform.cleanup()
        _current_platform = None


def register_platform(
    platform_class: type[BasePlatform],
    platform_type: PlatformType,
    *,
    priority: int | None = None,
) -> None:
    """Register a platform with the global registry.

    Args:
        platform_class: The platform class to register.
        platform_type: Type identifier for the platform.
        priority: Detection priority (lower = checked first).
    """
    registry = PlatformRegistry.get_instance()
    registry.register(platform_class, platform_type, priority=priority)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "BasePlatform",
    # Protocol and base classes
    "Platform",
    "PlatformCapability",
    # Configuration
    "PlatformConfig",
    "PlatformInfo",
    # Registry
    "PlatformRegistry",
    # Enums
    "PlatformType",
    "SimulationPlatform",
    # Functions
    "get_platform",
    "register_platform",
    "reset_platform",
]
