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

from robo_infra.platforms.arduino import (
    ARDUINO_USB_IDS,
    BOARD_CAPABILITIES,
    ArduinoAnalogPin,
    ArduinoBoard,
    ArduinoDigitalPin,
    ArduinoPlatform,
    ArduinoPWMPin,
    ArduinoServoPin,
    FirmataCommand,
)
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
from robo_infra.platforms.beaglebone import (
    ADC_PINS as BB_ADC_PINS,
)
from robo_infra.platforms.beaglebone import (
    ADC_RESOLUTION as BB_ADC_RESOLUTION,
)
from robo_infra.platforms.beaglebone import (
    ADC_VREF as BB_ADC_VREF,
)
from robo_infra.platforms.beaglebone import (
    BB_CAPABILITIES,
    P8_GPIO_MAP,
    P9_GPIO_MAP,
    BBIOBackend,
    BeagleBoneADCPin,
    BeagleBoneCapabilities,
    BeagleBoneDigitalPin,
    BeagleBoneModel,
    BeagleBonePlatform,
    BeagleBonePWMPin,
    DeviceTreeOverlayManager,
    PRUInterface,
    PRUState,
)
from robo_infra.platforms.beaglebone import (
    PWM_PINS as BB_PWM_PINS,
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
from robo_infra.platforms.esp32 import (
    ESP32_CAPABILITIES,
    ESP32AnalogPin,
    ESP32Chip,
    ESP32ConnectionMode,
    ESP32DACPin,
    ESP32DigitalPin,
    ESP32HallSensor,
    ESP32Platform,
    ESP32PWMPin,
    ESP32TouchPin,
    MicroPythonREPL,
)
from robo_infra.platforms.jetson import (
    HARDWARE_PWM_PINS as JETSON_HARDWARE_PWM_PINS,
)
from robo_infra.platforms.jetson import (
    JETSON_MODELS,
    JetsonDigitalPin,
    JetsonModel,
    JetsonPinNumbering,
    JetsonPlatform,
    JetsonPowerMode,
    JetsonPWMPin,
)
from robo_infra.platforms.linux_generic import (
    GPIOBackend as LinuxGPIOBackend,
)
from robo_infra.platforms.linux_generic import (
    GPIOChipInfo,
    GPIOEdge,
    GPIOLineInfo,
    LinuxDigitalPin,
    LinuxGenericPlatform,
    LinuxPWMPin,
    LinuxSBCCapabilities,
    LinuxSBCType,
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
    # Arduino constants
    "ARDUINO_USB_IDS",
    # BeagleBone constants
    "BB_ADC_PINS",
    "BB_ADC_RESOLUTION",
    "BB_ADC_VREF",
    "BB_CAPABILITIES",
    "BB_PWM_PINS",
    "BOARD_CAPABILITIES",
    # ESP32 constants
    "ESP32_CAPABILITIES",
    # Raspberry Pi constants
    "HARDWARE_PWM_PINS_PI5",
    "HARDWARE_PWM_PINS_STANDARD",
    # Jetson constants
    "JETSON_HARDWARE_PWM_PINS",
    "JETSON_MODELS",
    "P8_GPIO_MAP",
    "P9_GPIO_MAP",
    "PI_MODELS",
    # Arduino specific
    "ArduinoAnalogPin",
    "ArduinoBoard",
    "ArduinoDigitalPin",
    "ArduinoPWMPin",
    "ArduinoPlatform",
    "ArduinoServoPin",
    # BeagleBone specific
    "BBIOBackend",
    # Base classes and protocols
    "BasePlatform",
    "BeagleBoneADCPin",
    "BeagleBoneCapabilities",
    "BeagleBoneDigitalPin",
    "BeagleBoneModel",
    "BeagleBonePWMPin",
    "BeagleBonePlatform",
    "DeviceTreeOverlayManager",
    # ESP32 specific
    "ESP32AnalogPin",
    "ESP32Chip",
    "ESP32ConnectionMode",
    "ESP32DACPin",
    "ESP32DigitalPin",
    "ESP32HallSensor",
    "ESP32PWMPin",
    "ESP32Platform",
    "ESP32TouchPin",
    "FirmataCommand",
    # Raspberry Pi specific
    "GPIOBackend",
    # Linux Generic specific
    "GPIOChipInfo",
    "GPIOEdge",
    "GPIOLineInfo",
    # Jetson specific
    "JetsonDigitalPin",
    "JetsonModel",
    "JetsonPWMPin",
    "JetsonPinNumbering",
    "JetsonPlatform",
    "JetsonPowerMode",
    "LinuxDigitalPin",
    "LinuxGPIOBackend",
    "LinuxGenericPlatform",
    "LinuxPWMPin",
    "LinuxSBCCapabilities",
    "LinuxSBCType",
    "MicroPythonREPL",
    "PRUInterface",
    "PRUState",
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
