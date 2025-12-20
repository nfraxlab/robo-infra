"""Platform-specific implementations.

This package provides platform abstractions and auto-detection for
hardware-agnostic robotics control.

Example:
    >>> from robo_infra.platforms import get_platform
    >>>
    >>> # Auto-detect current platform
    >>> platform = get_platform()
    >>> print(f"Running on: {platform.name}")
    >>>
    >>> # Get hardware resources
    >>> pin = platform.get_pin(17)
    >>> i2c = platform.get_bus("i2c", bus=1)
"""

from robo_infra.platforms.base import (
    BasePlatform,
    Platform,
    PlatformCapability,
    PlatformConfig,
    PlatformInfo,
    PlatformRegistry,
    PlatformType,
    SimulationPlatform,
    get_platform,
    register_platform,
    reset_platform,
)
from robo_infra.platforms.detection import (
    detect_arduino,
    detect_beaglebone,
    detect_esp32,
    detect_jetson,
    detect_linux_generic,
    detect_macos,
    detect_microbit,
    detect_orange_pi,
    detect_pico,
    detect_pine64,
    detect_platform,
    detect_raspberry_pi,
    detect_rock_pi,
    detect_windows,
    get_platform_info,
)
from robo_infra.platforms.raspberry_pi import (
    HARDWARE_PWM_PINS_PI5,
    HARDWARE_PWM_PINS_STANDARD,
    PI_MODELS,
    GPIOBackend,
    PinNumbering,
    RaspberryPiDigitalPin,
    RaspberryPiPlatform,
    RaspberryPiPWMPin,
)


__all__ = [
    "HARDWARE_PWM_PINS_PI5",
    "HARDWARE_PWM_PINS_STANDARD",
    "PI_MODELS",
    # Base classes and protocols
    "BasePlatform",
    # Raspberry Pi specific
    "GPIOBackend",
    "PinNumbering",
    "Platform",
    "PlatformCapability",
    "PlatformConfig",
    "PlatformInfo",
    "PlatformRegistry",
    "PlatformType",
    "RaspberryPiDigitalPin",
    "RaspberryPiPWMPin",
    "RaspberryPiPlatform",
    "SimulationPlatform",
    # Detection functions
    "detect_arduino",
    "detect_beaglebone",
    "detect_esp32",
    "detect_jetson",
    "detect_linux_generic",
    "detect_macos",
    "detect_microbit",
    "detect_orange_pi",
    "detect_pico",
    "detect_pine64",
    "detect_platform",
    "detect_raspberry_pi",
    "detect_rock_pi",
    "detect_windows",
    # Module-level functions
    "get_platform",
    "get_platform_info",
    "register_platform",
    "reset_platform",
]
