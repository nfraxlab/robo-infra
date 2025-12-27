"""Platform factory for cross-platform hardware access.

This module provides high-level factory functions that automatically
detect the current platform and return appropriate hardware implementations.

Example:
    >>> from robo_infra.platforms import get_gpio, get_i2c, get_spi
    >>>
    >>> # Get a GPIO pin - works on any platform
    >>> pin = get_gpio(17)
    >>> pin.setup()
    >>> pin.high()
    >>>
    >>> # Get an I2C bus
    >>> i2c = get_i2c(bus=1)
    >>> devices = i2c.scan()
    >>>
    >>> # Get an SPI bus
    >>> spi = get_spi(bus=0, device=0)
    >>> spi.transfer([0x01, 0x02])
"""

from __future__ import annotations

import logging
from typing import Any

from robo_infra.core.bus import Bus, I2CBus, SPIBus
from robo_infra.core.pin import Pin, PinMode
from robo_infra.platforms.base import (
    PlatformCapability,
    PlatformType,
    get_platform,
)
from robo_infra.platforms.detection import detect_platform


logger = logging.getLogger(__name__)


# =============================================================================
# GPIO Factory
# =============================================================================


def get_gpio(
    pin_id: int | str,
    *,
    mode: PinMode = PinMode.OUTPUT,
    pull: str | None = None,
    inverted: bool = False,
    name: str | None = None,
    **kwargs: Any,
) -> Pin:
    """Get a GPIO pin for the current platform.

    Automatically detects the platform and returns an appropriate GPIO pin.
    This is the recommended way to get GPIO pins in cross-platform code.

    Args:
        pin_id: Pin number or name (platform-specific).
            - Raspberry Pi: BCM pin number (e.g., 17, 18)
            - Jetson: GPIO number (e.g., "GPIO17")
            - Linux Generic: gpiochip line number
            - Arduino: Digital pin number (e.g., 13)
        mode: Pin mode (INPUT, OUTPUT, PWM).
        pull: Pull-up/down configuration ("up", "down", None).
        inverted: If True, logic is inverted (HIGH=0, LOW=1).
        name: Optional name for debugging.
        **kwargs: Platform-specific options.

    Returns:
        Pin instance appropriate for the detected platform.

    Raises:
        HardwareNotFoundError: If GPIO is not available on this platform.

    Example:
        >>> # Simple LED control
        >>> led = get_gpio(17, mode=PinMode.OUTPUT)
        >>> led.setup()
        >>> led.high()
        >>>
        >>> # Button with pull-up
        >>> button = get_gpio(18, mode=PinMode.INPUT, pull="up")
        >>> button.setup()
        >>> if button.read() == 0:  # Pressed (inverted logic with pull-up)
        ...     print("Button pressed!")
        >>>
        >>> # PWM for servo
        >>> servo = get_gpio(12, mode=PinMode.PWM, frequency=50)
        >>> servo.setup()
        >>> servo.duty_cycle = 7.5  # Center position
    """
    platform = get_platform()
    platform_type = detect_platform()

    logger.debug(
        "get_gpio(pin=%s, mode=%s) on platform %s",
        pin_id,
        mode,
        platform_type.value,
    )

    # Check platform supports GPIO
    if PlatformCapability.GPIO not in platform.capabilities:
        from robo_infra.core.exceptions import HardwareNotFoundError

        raise HardwareNotFoundError(
            device=f"GPIO pin {pin_id}",
            details=f"Platform {platform.name} does not support GPIO",
        )

    # Collect kwargs for pin creation
    pin_kwargs = {
        "mode": mode,
        "inverted": inverted,
        "name": name,
        **kwargs,
    }

    if pull is not None:
        pin_kwargs["pull"] = pull

    return platform.get_pin(pin_id, **pin_kwargs)


# =============================================================================
# I2C Factory
# =============================================================================


def get_i2c(
    bus: int = 1,
    *,
    frequency: int | None = None,
    name: str | None = None,
    **kwargs: Any,
) -> I2CBus:
    """Get an I2C bus for the current platform.

    Automatically detects the platform and returns an appropriate I2C bus.
    This is the recommended way to get I2C buses in cross-platform code.

    Args:
        bus: I2C bus number (default 1 for most platforms).
            - Raspberry Pi: 1 is the main user I2C bus
            - Jetson: 0, 1, or 7 depending on model
            - BeagleBone: 1 or 2
        frequency: Bus frequency in Hz (optional, uses default if not set).
        name: Optional name for debugging.
        **kwargs: Platform-specific options.

    Returns:
        I2CBus instance appropriate for the detected platform.

    Raises:
        HardwareNotFoundError: If I2C is not available on this platform.

    Example:
        >>> # Scan for devices
        >>> i2c = get_i2c(bus=1)
        >>> devices = i2c.scan()
        >>> print(f"Found devices at: {[hex(d) for d in devices]}")
        >>>
        >>> # Read from sensor
        >>> i2c.write_byte(0x48, 0x00)  # Set register
        >>> temp = i2c.read_word(0x48)
        >>> print(f"Temperature: {temp / 100:.1f}°C")
    """
    platform = get_platform()
    platform_type = detect_platform()

    logger.debug(
        "get_i2c(bus=%s) on platform %s",
        bus,
        platform_type.value,
    )

    # Check platform supports I2C
    if PlatformCapability.I2C not in platform.capabilities:
        from robo_infra.core.exceptions import HardwareNotFoundError

        raise HardwareNotFoundError(
            device=f"I2C bus {bus}",
            details=f"Platform {platform.name} does not support I2C",
        )

    # Check bus is available
    if platform.info.i2c_buses and bus not in platform.info.i2c_buses:
        logger.warning(
            "I2C bus %d not in detected buses %s, proceeding anyway",
            bus,
            platform.info.i2c_buses,
        )

    # Collect kwargs for bus creation
    bus_kwargs = {
        "bus": bus,
        **kwargs,
    }

    if frequency is not None:
        bus_kwargs["frequency"] = frequency
    if name is not None:
        bus_kwargs["name"] = name

    i2c_bus = platform.get_bus("i2c", **bus_kwargs)

    # Type check - should return I2CBus
    if not isinstance(i2c_bus, I2CBus):
        logger.warning(
            "Platform returned non-I2CBus type: %s",
            type(i2c_bus).__name__,
        )

    return i2c_bus  # type: ignore[return-value]


# =============================================================================
# SPI Factory
# =============================================================================


def get_spi(
    bus: int = 0,
    device: int = 0,
    *,
    speed_hz: int | None = None,
    mode: int = 0,
    bits_per_word: int = 8,
    name: str | None = None,
    **kwargs: Any,
) -> SPIBus:
    """Get an SPI bus for the current platform.

    Automatically detects the platform and returns an appropriate SPI bus.
    This is the recommended way to get SPI buses in cross-platform code.

    Args:
        bus: SPI bus number (default 0).
            - Raspberry Pi: 0 is the main SPI bus (CE0=GPIO8, CE1=GPIO7)
            - Jetson: 0 or 1 depending on model
        device: SPI device (chip select) number (default 0).
        speed_hz: SPI clock speed in Hz (optional, uses default if not set).
        mode: SPI mode (0-3, default 0).
            - Mode 0: CPOL=0, CPHA=0
            - Mode 1: CPOL=0, CPHA=1
            - Mode 2: CPOL=1, CPHA=0
            - Mode 3: CPOL=1, CPHA=1
        bits_per_word: Bits per word (default 8).
        name: Optional name for debugging.
        **kwargs: Platform-specific options.

    Returns:
        SPIBus instance appropriate for the detected platform.

    Raises:
        HardwareNotFoundError: If SPI is not available on this platform.

    Example:
        >>> # Basic SPI transfer
        >>> spi = get_spi(bus=0, device=0, speed_hz=1_000_000)
        >>> response = spi.transfer([0x01, 0x02, 0x03])
        >>>
        >>> # Read from SPI sensor
        >>> spi = get_spi(bus=0, device=0, mode=1)
        >>> spi.write_byte(0x80)  # Read command
        >>> data = spi.read(4)
        >>> print(f"Received: {data.hex()}")
    """
    platform = get_platform()
    platform_type = detect_platform()

    logger.debug(
        "get_spi(bus=%s, device=%s) on platform %s",
        bus,
        device,
        platform_type.value,
    )

    # Check platform supports SPI
    if PlatformCapability.SPI not in platform.capabilities:
        from robo_infra.core.exceptions import HardwareNotFoundError

        raise HardwareNotFoundError(
            device=f"SPI bus {bus}:{device}",
            details=f"Platform {platform.name} does not support SPI",
        )

    # Check bus:device is available
    if platform.info.spi_buses and (bus, device) not in platform.info.spi_buses:
        logger.warning(
            "SPI bus %d:%d not in detected buses %s, proceeding anyway",
            bus,
            device,
            platform.info.spi_buses,
        )

    # Collect kwargs for bus creation
    bus_kwargs = {
        "bus": bus,
        "device": device,
        "mode": mode,
        "bits_per_word": bits_per_word,
        **kwargs,
    }

    if speed_hz is not None:
        bus_kwargs["speed_hz"] = speed_hz
    if name is not None:
        bus_kwargs["name"] = name

    spi_bus = platform.get_bus("spi", **bus_kwargs)

    # Type check - should return SPIBus
    if not isinstance(spi_bus, SPIBus):
        logger.warning(
            "Platform returned non-SPIBus type: %s",
            type(spi_bus).__name__,
        )

    return spi_bus  # type: ignore[return-value]


# =============================================================================
# UART Factory (bonus - commonly needed)
# =============================================================================


def get_uart(
    port: str | None = None,
    *,
    baudrate: int = 9600,
    timeout: float | None = 1.0,
    name: str | None = None,
    **kwargs: Any,
) -> Bus:
    """Get a UART/Serial bus for the current platform.

    Automatically detects the platform and returns an appropriate UART bus.

    Args:
        port: Serial port path (auto-detected if None).
            - Raspberry Pi: "/dev/ttyAMA0" or "/dev/serial0"
            - Jetson: "/dev/ttyTHS1"
            - Linux: "/dev/ttyUSB0", "/dev/ttyACM0"
        baudrate: Baud rate (default 9600).
        timeout: Read timeout in seconds (default 1.0).
        name: Optional name for debugging.
        **kwargs: Platform-specific options.

    Returns:
        Bus instance for UART communication.

    Raises:
        HardwareNotFoundError: If UART is not available on this platform.

    Example:
        >>> # Connect to GPS module
        >>> uart = get_uart(port="/dev/ttyAMA0", baudrate=9600)
        >>> line = uart.readline()
        >>> print(f"GPS: {line}")
    """
    platform = get_platform()
    platform_type = detect_platform()

    # Auto-detect port if not specified
    if port is None:
        if platform.info.uart_ports:
            port = platform.info.uart_ports[0]
        else:
            # Platform-specific defaults
            default_ports = {
                PlatformType.RASPBERRY_PI: "/dev/serial0",
                PlatformType.JETSON: "/dev/ttyTHS1",
                PlatformType.BEAGLEBONE: "/dev/ttyO1",
                PlatformType.LINUX_GENERIC: "/dev/ttyUSB0",
                PlatformType.SIMULATION: "/dev/ttyS0",
            }
            port = default_ports.get(platform_type, "/dev/ttyS0")

    logger.debug(
        "get_uart(port=%s, baudrate=%s) on platform %s",
        port,
        baudrate,
        platform_type.value,
    )

    # Check platform supports UART
    if PlatformCapability.UART not in platform.capabilities:
        from robo_infra.core.exceptions import HardwareNotFoundError

        raise HardwareNotFoundError(
            device=f"UART port {port}",
            details=f"Platform {platform.name} does not support UART",
        )

    # Collect kwargs for bus creation
    bus_kwargs = {
        "port": port,
        "baudrate": baudrate,
        **kwargs,
    }

    if timeout is not None:
        bus_kwargs["timeout"] = timeout
    if name is not None:
        bus_kwargs["name"] = name

    return platform.get_bus("uart", **bus_kwargs)


# =============================================================================
# Convenience Functions
# =============================================================================


def list_available_gpio() -> list[int | str]:
    """List available GPIO pins on the current platform.

    Returns:
        List of available GPIO pin identifiers.

    Example:
        >>> pins = list_available_gpio()
        >>> print(f"Available pins: {pins}")
    """
    platform = get_platform()

    if PlatformCapability.GPIO not in platform.capabilities:
        return []

    # Return GPIO chips as available pins (platform-specific details)
    return platform.info.gpio_chips  # type: ignore[return-value]


def list_available_i2c() -> list[int]:
    """List available I2C buses on the current platform.

    Returns:
        List of available I2C bus numbers.

    Example:
        >>> buses = list_available_i2c()
        >>> for bus_num in buses:
        ...     i2c = get_i2c(bus=bus_num)
        ...     print(f"Bus {bus_num}: {i2c.scan()}")
    """
    platform = get_platform()

    if PlatformCapability.I2C not in platform.capabilities:
        return []

    return platform.info.i2c_buses


def list_available_spi() -> list[tuple[int, int]]:
    """List available SPI bus:device pairs on the current platform.

    Returns:
        List of (bus, device) tuples.

    Example:
        >>> buses = list_available_spi()
        >>> for bus, device in buses:
        ...     spi = get_spi(bus=bus, device=device)
        ...     print(f"SPI {bus}:{device} ready")
    """
    platform = get_platform()

    if PlatformCapability.SPI not in platform.capabilities:
        return []

    return platform.info.spi_buses


def list_available_uart() -> list[str]:
    """List available UART ports on the current platform.

    Returns:
        List of available UART port paths.

    Example:
        >>> ports = list_available_uart()
        >>> for port in ports:
        ...     print(f"Available: {port}")
    """
    platform = get_platform()

    if PlatformCapability.UART not in platform.capabilities:
        return []

    return platform.info.uart_ports


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Main factory functions
    "get_gpio",
    "get_i2c",
    "get_spi",
    "get_uart",
    # Convenience functions
    "list_available_gpio",
    "list_available_i2c",
    "list_available_spi",
    "list_available_uart",
]
