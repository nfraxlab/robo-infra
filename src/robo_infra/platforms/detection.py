"""Platform detection utilities.

This module provides functions to detect the current hardware platform
by examining system files, device trees, USB devices, and other indicators.

Example:
    >>> from robo_infra.platforms.detection import detect_platform, get_platform_info
    >>>
    >>> # Detect the current platform
    >>> platform_type = detect_platform()
    >>> print(f"Detected: {platform_type}")
    >>>
    >>> # Get detailed platform info
    >>> info = get_platform_info()
    >>> print(f"Model: {info.model}")
    >>> print(f"Capabilities: {info.capabilities}")
"""

from __future__ import annotations

import logging
import os
import platform
import re
import sys
from pathlib import Path

from robo_infra.platforms.base import (
    PlatformCapability,
    PlatformInfo,
    PlatformType,
)


logger = logging.getLogger(__name__)


# =============================================================================
# USB Vendor/Product IDs
# =============================================================================

# Arduino VID/PID pairs (Vendor ID: Product IDs)
ARDUINO_VID_PIDS = {
    0x2341: [  # Arduino SA
        0x0001,  # Uno
        0x0010,  # Mega 2560
        0x003B,  # Serial Adapter
        0x003D,  # Due Programming Port
        0x003E,  # Due
        0x0036,  # Leonardo
        0x0037,  # Micro
        0x0042,  # Mega 2560 R3
        0x0043,  # Uno R3
        0x0044,  # Mega ADK R3
        0x0045,  # Serial R3
        0x0243,  # Leonardo
        0x8036,  # Leonardo Bootloader
        0x8037,  # Micro Bootloader
    ],
    0x2A03: [  # Arduino.org
        0x0001,  # Uno
        0x003D,  # Due Programming Port
        0x0042,  # Mega 2560 R3
        0x0043,  # Uno R3
    ],
    0x1A86: [  # QinHeng (CH340/CH341)
        0x7523,  # CH340
        0x5523,  # CH341
    ],
    0x10C4: [  # Silicon Labs (CP210x)
        0xEA60,  # CP2102
        0xEA70,  # CP2105
        0xEA71,  # CP2108
    ],
    0x0403: [  # FTDI
        0x6001,  # FT232
        0x6010,  # FT2232
        0x6011,  # FT4232
        0x6014,  # FT232H
        0x6015,  # FT-X
    ],
}

# ESP32/ESP8266 VID/PID pairs
ESP_VID_PIDS = {
    0x10C4: [  # Silicon Labs (CP210x - common on ESP dev boards)
        0xEA60,  # CP2102
    ],
    0x1A86: [  # QinHeng (CH340 - common on cheap ESP boards)
        0x7523,  # CH340
    ],
    0x303A: [  # Espressif
        0x0002,  # ESP32-S2
        0x1001,  # ESP32-S3
        0x80D1,  # ESP32-C3
    ],
}

# Micro:bit VID/PID
MICROBIT_VID_PIDS = {
    0x0D28: [  # ARM Ltd
        0x0204,  # micro:bit
    ],
}

# Raspberry Pi Pico VID/PID
PICO_VID_PIDS = {
    0x2E8A: [  # Raspberry Pi Foundation
        0x0005,  # Pico
        0x000A,  # Pico W
    ],
}


# =============================================================================
# Detection Functions
# =============================================================================


def detect_raspberry_pi() -> tuple[bool, str]:
    """Detect if running on a Raspberry Pi.

    Checks:
    - /proc/cpuinfo for BCM2xxx SoC
    - /sys/firmware/devicetree/base/model for Pi model
    - /proc/device-tree/compatible for raspberry

    Returns:
        Tuple of (is_detected, model_string).
    """
    # Check device tree model (most reliable)
    model_path = Path("/sys/firmware/devicetree/base/model")
    if model_path.exists():
        try:
            model = model_path.read_text().strip().rstrip("\x00")
            if "raspberry pi" in model.lower():
                logger.debug("Detected Raspberry Pi from device tree: %s", model)
                return True, model
        except (OSError, PermissionError) as e:
            logger.debug("Could not read device tree model: %s", e)

    # Check /proc/device-tree/compatible
    compatible_path = Path("/proc/device-tree/compatible")
    if compatible_path.exists():
        try:
            compatible = compatible_path.read_text()
            if "raspberrypi" in compatible.lower():
                logger.debug("Detected Raspberry Pi from compatible")
                return True, "Raspberry Pi"
        except (OSError, PermissionError):
            pass

    # Check /proc/cpuinfo for BCM SoC
    cpuinfo_path = Path("/proc/cpuinfo")
    if cpuinfo_path.exists():
        try:
            cpuinfo = cpuinfo_path.read_text()
            if "BCM" in cpuinfo:
                # Extract model from cpuinfo
                model_match = re.search(r"Model\s*:\s*(.+)", cpuinfo)
                model = model_match.group(1) if model_match else "Raspberry Pi"
                logger.debug("Detected Raspberry Pi from cpuinfo: %s", model)
                return True, model
        except (OSError, PermissionError):
            pass

    return False, ""


def detect_jetson() -> tuple[bool, str]:
    """Detect if running on an NVIDIA Jetson.

    Checks:
    - /etc/nv_tegra_release for Tegra version
    - /proc/device-tree/model for Jetson model

    Returns:
        Tuple of (is_detected, model_string).
    """
    # Check for Tegra release file (most reliable)
    tegra_path = Path("/etc/nv_tegra_release")
    if tegra_path.exists():
        try:
            # File exists, definitely a Jetson
            model_path = Path("/proc/device-tree/model")
            if model_path.exists():
                model = model_path.read_text().strip().rstrip("\x00")
                logger.debug("Detected Jetson from tegra release: %s", model)
                return True, model
            return True, "NVIDIA Jetson"
        except (OSError, PermissionError):
            return True, "NVIDIA Jetson"

    # Check device tree for NVIDIA
    model_path = Path("/proc/device-tree/model")
    if model_path.exists():
        try:
            model = model_path.read_text().strip().rstrip("\x00")
            if "jetson" in model.lower() or "nvidia" in model.lower():
                logger.debug("Detected Jetson from device tree: %s", model)
                return True, model
        except (OSError, PermissionError):
            pass

    return False, ""


def detect_beaglebone() -> tuple[bool, str]:
    """Detect if running on a BeagleBone.

    Checks:
    - /proc/device-tree/model for BeagleBone model

    Returns:
        Tuple of (is_detected, model_string).
    """
    model_path = Path("/proc/device-tree/model")
    if model_path.exists():
        try:
            model = model_path.read_text().strip().rstrip("\x00")
            if "beagle" in model.lower():
                logger.debug("Detected BeagleBone: %s", model)
                return True, model
        except (OSError, PermissionError):
            pass

    return False, ""


def detect_orange_pi() -> tuple[bool, str]:
    """Detect if running on an Orange Pi.

    Checks:
    - /etc/orangepi-release
    - /proc/device-tree/model

    Returns:
        Tuple of (is_detected, model_string).
    """
    # Check for Orange Pi release file
    release_path = Path("/etc/orangepi-release")
    if release_path.exists():
        try:
            release = release_path.read_text()
            board_match = re.search(r"BOARD=(\S+)", release)
            model = board_match.group(1) if board_match else "Orange Pi"
            logger.debug("Detected Orange Pi from release: %s", model)
            return True, f"Orange Pi {model}"
        except (OSError, PermissionError):
            return True, "Orange Pi"

    # Check device tree
    model_path = Path("/proc/device-tree/model")
    if model_path.exists():
        try:
            model = model_path.read_text().strip().rstrip("\x00")
            if "orange" in model.lower():
                logger.debug("Detected Orange Pi from device tree: %s", model)
                return True, model
        except (OSError, PermissionError):
            pass

    return False, ""


def detect_rock_pi() -> tuple[bool, str]:
    """Detect if running on a Rock Pi / Radxa board.

    Returns:
        Tuple of (is_detected, model_string).
    """
    model_path = Path("/proc/device-tree/model")
    if model_path.exists():
        try:
            model = model_path.read_text().strip().rstrip("\x00")
            if "rock" in model.lower() or "radxa" in model.lower():
                logger.debug("Detected Rock Pi: %s", model)
                return True, model
        except (OSError, PermissionError):
            pass

    return False, ""


def detect_pine64() -> tuple[bool, str]:
    """Detect if running on a Pine64 board.

    Returns:
        Tuple of (is_detected, model_string).
    """
    model_path = Path("/proc/device-tree/model")
    if model_path.exists():
        try:
            model = model_path.read_text().strip().rstrip("\x00")
            if "pine" in model.lower():
                logger.debug("Detected Pine64: %s", model)
                return True, model
        except (OSError, PermissionError):
            pass

    return False, ""


def _list_usb_devices() -> list[tuple[int, int]]:
    """List USB devices as (vendor_id, product_id) tuples.

    Works on Linux, macOS, and Windows (with pyusb installed).
    """
    devices = []

    # Try Linux sysfs first
    usb_path = Path("/sys/bus/usb/devices")
    if usb_path.exists():
        for device_dir in usb_path.iterdir():
            vid_path = device_dir / "idVendor"
            pid_path = device_dir / "idProduct"
            if vid_path.exists() and pid_path.exists():
                try:
                    vid = int(vid_path.read_text().strip(), 16)
                    pid = int(pid_path.read_text().strip(), 16)
                    devices.append((vid, pid))
                except (ValueError, OSError):
                    pass
        return devices

    # Try pyusb as fallback
    try:
        import usb.core  # type: ignore[import-not-found]

        for dev in usb.core.find(find_all=True):
            devices.append((dev.idVendor, dev.idProduct))
    except ImportError:
        pass
    except Exception as e:
        logger.debug("pyusb enumeration failed: %s", e)

    return devices


def _check_usb_for_vid_pids(vid_pids: dict[int, list[int]]) -> bool:
    """Check if any USB device matches the given VID/PID pairs."""
    devices = _list_usb_devices()
    return any(vid in vid_pids and pid in vid_pids[vid] for vid, pid in devices)


def detect_arduino() -> tuple[bool, str]:
    """Detect if an Arduino is connected via USB.

    Returns:
        Tuple of (is_detected, description).
    """
    if _check_usb_for_vid_pids(ARDUINO_VID_PIDS):
        logger.debug("Detected Arduino via USB")
        return True, "Arduino (USB)"

    # Also check for /dev/ttyACM* or /dev/ttyUSB* on Linux
    if sys.platform.startswith("linux"):
        tty_paths = list(Path("/dev").glob("ttyACM*")) + list(Path("/dev").glob("ttyUSB*"))
        if tty_paths:
            # Could be Arduino, but we can't be certain
            logger.debug("Found serial devices that could be Arduino: %s", tty_paths)
            return True, f"Arduino (Serial: {tty_paths[0].name})"

    return False, ""


def detect_esp32() -> tuple[bool, str]:
    """Detect if an ESP32/ESP8266 is connected via USB.

    Returns:
        Tuple of (is_detected, description).
    """
    if _check_usb_for_vid_pids(ESP_VID_PIDS):
        logger.debug("Detected ESP32/ESP8266 via USB")
        return True, "ESP32 (USB)"

    return False, ""


def detect_microbit() -> tuple[bool, str]:
    """Detect if a micro:bit is connected via USB.

    Returns:
        Tuple of (is_detected, description).
    """
    if _check_usb_for_vid_pids(MICROBIT_VID_PIDS):
        logger.debug("Detected micro:bit via USB")
        return True, "BBC micro:bit"

    return False, ""


def detect_pico() -> tuple[bool, str]:
    """Detect if a Raspberry Pi Pico is connected via USB.

    Returns:
        Tuple of (is_detected, description).
    """
    if _check_usb_for_vid_pids(PICO_VID_PIDS):
        logger.debug("Detected Raspberry Pi Pico via USB")
        return True, "Raspberry Pi Pico"

    return False, ""


def detect_linux_generic() -> tuple[bool, str]:
    """Detect if running on a generic Linux system with GPIO.

    Checks for /dev/gpiochip* devices (libgpiod support).

    Returns:
        Tuple of (is_detected, description).
    """
    if not sys.platform.startswith("linux"):
        return False, ""

    gpio_chips = list(Path("/dev").glob("gpiochip*"))
    if gpio_chips:
        logger.debug("Detected Linux with GPIO chips: %s", gpio_chips)
        return True, f"Linux (GPIO: {len(gpio_chips)} chips)"

    return False, ""


def detect_macos() -> bool:
    """Detect if running on macOS.

    macOS can only use simulation mode for GPIO (no native GPIO support),
    but can communicate with Arduinos and other USB devices.
    """
    return sys.platform == "darwin"


def detect_windows() -> bool:
    """Detect if running on Windows.

    Windows can only use simulation mode for GPIO (no native GPIO support),
    but can communicate with Arduinos and other USB devices.
    """
    return sys.platform == "win32"


# =============================================================================
# Convenience Functions
# =============================================================================


def is_simulation_mode() -> bool:
    """Check if running in simulation mode.

    Simulation mode is activated when:
    - ROBO_SIMULATION env var is set to "1", "true", or "yes"
    - ROBO_PLATFORM env var is set to "sim" or "simulation"
    - Platform is detected as macOS or Windows (no native GPIO)

    Returns:
        True if in simulation mode.

    Example:
        >>> if is_simulation_mode():
        ...     print("Using simulated hardware")
        ... else:
        ...     print("Using real hardware")
    """
    # Check explicit simulation env var
    if os.environ.get("ROBO_SIMULATION", "").lower() in ("1", "true", "yes"):
        return True

    # Check platform env override
    env_platform = os.environ.get("ROBO_PLATFORM", "").lower()
    if env_platform in ("sim", "simulation"):
        return True

    # macOS and Windows are always simulation mode for GPIO
    return bool(detect_macos() or detect_windows())


def is_raspberry_pi() -> bool:
    """Check if running on a Raspberry Pi.

    Returns:
        True if running on any Raspberry Pi model.

    Example:
        >>> if is_raspberry_pi():
        ...     from robo_infra.platforms.raspberry_pi import RaspberryPiGPIO
        ...     gpio = RaspberryPiGPIO()
    """
    is_detected, _ = detect_raspberry_pi()
    return is_detected


def is_jetson() -> bool:
    """Check if running on an NVIDIA Jetson.

    Returns:
        True if running on any Jetson model.

    Example:
        >>> if is_jetson():
        ...     from robo_infra.platforms.jetson import JetsonGPIO
        ...     gpio = JetsonGPIO()
    """
    is_detected, _ = detect_jetson()
    return is_detected


def is_beaglebone() -> bool:
    """Check if running on a BeagleBone.

    Returns:
        True if running on any BeagleBone model.
    """
    is_detected, _ = detect_beaglebone()
    return is_detected


def is_arduino_connected() -> bool:
    """Check if an Arduino is connected via USB.

    Returns:
        True if an Arduino is detected on USB.
    """
    is_detected, _ = detect_arduino()
    return is_detected


def is_esp32_connected() -> bool:
    """Check if an ESP32/ESP8266 is connected via USB.

    Returns:
        True if an ESP32/ESP8266 is detected on USB.
    """
    is_detected, _ = detect_esp32()
    return is_detected


# =============================================================================
# Platform Info Detection
# =============================================================================


def _detect_gpio_chips() -> list[str]:
    """Detect available GPIO chips on the system."""
    chips = []
    dev_path = Path("/dev")
    if dev_path.exists():
        chips = [p.name for p in dev_path.glob("gpiochip*")]
    return sorted(chips)


def _detect_i2c_buses() -> list[int]:
    """Detect available I2C buses on the system."""
    buses = []
    dev_path = Path("/dev")
    if dev_path.exists():
        for path in dev_path.glob("i2c-*"):
            try:
                bus_num = int(path.name.split("-")[1])
                buses.append(bus_num)
            except (ValueError, IndexError):
                pass
    return sorted(buses)


def _detect_spi_buses() -> list[tuple[int, int]]:
    """Detect available SPI buses on the system."""
    buses = []
    dev_path = Path("/dev")
    if dev_path.exists():
        for path in dev_path.glob("spidev*.*"):
            try:
                parts = path.name.replace("spidev", "").split(".")
                bus = int(parts[0])
                device = int(parts[1])
                buses.append((bus, device))
            except (ValueError, IndexError):
                pass
    return sorted(buses)


def _detect_uart_ports() -> list[str]:
    """Detect available UART/serial ports on the system."""
    ports = []
    dev_path = Path("/dev")

    if dev_path.exists():
        # Linux serial ports
        patterns = ["ttyAMA*", "ttyUSB*", "ttyACM*", "ttyS*", "serial*"]
        for pattern in patterns:
            for path in dev_path.glob(pattern):
                ports.append(str(path))

    # Also check for common macOS serial ports
    if sys.platform == "darwin":
        for path in dev_path.glob("cu.*"):
            ports.append(str(path))
        for path in dev_path.glob("tty.*"):
            if "Bluetooth" not in path.name:
                ports.append(str(path))

    return sorted(set(ports))


def _get_rpi_capabilities() -> set[PlatformCapability]:
    """Get capabilities for Raspberry Pi."""
    caps = {
        PlatformCapability.GPIO,
        PlatformCapability.PWM,
        PlatformCapability.I2C,
        PlatformCapability.SPI,
        PlatformCapability.UART,
        PlatformCapability.ONEWIRE,
        PlatformCapability.CAMERA_CSI,
        PlatformCapability.CAMERA_USB,
    }

    # Pi 4/5 have hardware PWM on GPIO 12, 13, 18, 19
    caps.add(PlatformCapability.HARDWARE_PWM)

    # Check for ADC (MCP3008 is common, but not built-in)
    # Not adding ADC by default since Pi doesn't have built-in ADC

    return caps


def _get_jetson_capabilities() -> set[PlatformCapability]:
    """Get capabilities for NVIDIA Jetson."""
    return {
        PlatformCapability.GPIO,
        PlatformCapability.PWM,
        PlatformCapability.HARDWARE_PWM,
        PlatformCapability.I2C,
        PlatformCapability.SPI,
        PlatformCapability.UART,
        PlatformCapability.CAN,  # Jetson AGX has CAN
        PlatformCapability.CAMERA_CSI,
        PlatformCapability.CAMERA_USB,
    }


def _get_beaglebone_capabilities() -> set[PlatformCapability]:
    """Get capabilities for BeagleBone."""
    return {
        PlatformCapability.GPIO,
        PlatformCapability.PWM,
        PlatformCapability.HARDWARE_PWM,
        PlatformCapability.I2C,
        PlatformCapability.SPI,
        PlatformCapability.UART,
        PlatformCapability.ADC,  # BeagleBone has built-in ADC
        PlatformCapability.CAN,  # BBB has CAN
        PlatformCapability.CAMERA_USB,
    }


def _get_arduino_capabilities() -> set[PlatformCapability]:
    """Get capabilities for Arduino (via USB)."""
    return {
        PlatformCapability.GPIO,
        PlatformCapability.PWM,
        PlatformCapability.ADC,
        # Note: I2C/SPI available but we control via Firmata, not directly
    }


def _get_esp32_capabilities() -> set[PlatformCapability]:
    """Get capabilities for ESP32 (via USB/WiFi)."""
    return {
        PlatformCapability.GPIO,
        PlatformCapability.PWM,
        PlatformCapability.ADC,
        PlatformCapability.DAC,  # ESP32 has 2 DAC channels
        PlatformCapability.I2C,
        PlatformCapability.SPI,
        PlatformCapability.UART,
    }


def _get_linux_capabilities() -> set[PlatformCapability]:
    """Get capabilities for generic Linux."""
    caps = {PlatformCapability.GPIO}

    # Check for other capabilities based on available devices
    if _detect_i2c_buses():
        caps.add(PlatformCapability.I2C)
    if _detect_spi_buses():
        caps.add(PlatformCapability.SPI)
    if _detect_uart_ports():
        caps.add(PlatformCapability.UART)
    if list(Path("/sys/class/pwm").glob("pwmchip*")):
        caps.add(PlatformCapability.PWM)

    return caps


# =============================================================================
# Main Detection Functions
# =============================================================================


def detect_platform() -> PlatformType:
    """Detect the current platform type.

    Runs through all detection functions and returns the first match.

    Returns:
        PlatformType for the detected platform.
    """
    # Check environment override first
    env_platform = os.environ.get("ROBO_PLATFORM", "").lower()
    if env_platform:
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
        if env_platform in platform_map:
            return platform_map[env_platform]

    # Force simulation mode
    if os.environ.get("ROBO_SIMULATION", "").lower() in ("1", "true", "yes"):
        return PlatformType.SIMULATION

    # SBC detection (running on the device)
    detections = [
        (detect_raspberry_pi, PlatformType.RASPBERRY_PI),
        (detect_jetson, PlatformType.JETSON),
        (detect_beaglebone, PlatformType.BEAGLEBONE),
        (detect_orange_pi, PlatformType.ORANGE_PI),
        (detect_rock_pi, PlatformType.ROCK_PI),
        (detect_pine64, PlatformType.PINE64),
    ]

    for detect_func, platform_type in detections:
        is_detected, _ = detect_func()
        if is_detected:
            return platform_type

    # Microcontroller detection (connected via USB)
    usb_detections = [
        (detect_arduino, PlatformType.ARDUINO),
        (detect_esp32, PlatformType.ESP32),
        (detect_microbit, PlatformType.MICROBIT),
        (detect_pico, PlatformType.PICO),
    ]

    for detect_func, platform_type in usb_detections:
        is_detected, _ = detect_func()
        if is_detected:
            return platform_type

    # Generic Linux with GPIO
    is_linux, _ = detect_linux_generic()
    if is_linux:
        return PlatformType.LINUX_GENERIC

    # Fallback to simulation on non-Linux platforms
    if detect_macos() or detect_windows():
        logger.info(
            "Running on %s - using simulation mode (no native GPIO)",
            platform.system(),
        )
        return PlatformType.SIMULATION

    return PlatformType.UNKNOWN


def get_platform_info() -> PlatformInfo:
    """Get detailed information about the current platform.

    Returns:
        PlatformInfo with detected capabilities and hardware info.
    """
    platform_type = detect_platform()

    # Default values
    model = "Unknown"
    revision = ""
    serial = ""
    capabilities: set[PlatformCapability] = set()

    # Platform-specific info gathering
    if platform_type == PlatformType.RASPBERRY_PI:
        _, model = detect_raspberry_pi()
        capabilities = _get_rpi_capabilities()

        # Get revision and serial from /proc/cpuinfo
        cpuinfo_path = Path("/proc/cpuinfo")
        if cpuinfo_path.exists():
            try:
                cpuinfo = cpuinfo_path.read_text()
                rev_match = re.search(r"Revision\s*:\s*(\S+)", cpuinfo)
                if rev_match:
                    revision = rev_match.group(1)
                serial_match = re.search(r"Serial\s*:\s*(\S+)", cpuinfo)
                if serial_match:
                    serial = serial_match.group(1)
            except (OSError, PermissionError):
                pass

    elif platform_type == PlatformType.JETSON:
        _, model = detect_jetson()
        capabilities = _get_jetson_capabilities()

    elif platform_type == PlatformType.BEAGLEBONE:
        _, model = detect_beaglebone()
        capabilities = _get_beaglebone_capabilities()

    elif platform_type == PlatformType.ORANGE_PI:
        _, model = detect_orange_pi()
        capabilities = _get_linux_capabilities()

    elif platform_type == PlatformType.ROCK_PI:
        _, model = detect_rock_pi()
        capabilities = _get_linux_capabilities()

    elif platform_type == PlatformType.PINE64:
        _, model = detect_pine64()
        capabilities = _get_linux_capabilities()

    elif platform_type == PlatformType.ARDUINO:
        _, model = detect_arduino()
        capabilities = _get_arduino_capabilities()

    elif platform_type == PlatformType.ESP32:
        _, model = detect_esp32()
        capabilities = _get_esp32_capabilities()

    elif platform_type == PlatformType.LINUX_GENERIC:
        model = f"Linux ({platform.machine()})"
        capabilities = _get_linux_capabilities()

    elif platform_type == PlatformType.SIMULATION:
        model = f"Simulation ({platform.system()} {platform.machine()})"
        capabilities = {
            PlatformCapability.GPIO,
            PlatformCapability.PWM,
            PlatformCapability.HARDWARE_PWM,
            PlatformCapability.I2C,
            PlatformCapability.SPI,
            PlatformCapability.UART,
            PlatformCapability.ADC,
            PlatformCapability.DAC,
        }

    return PlatformInfo(
        platform_type=platform_type,
        model=model,
        revision=revision,
        serial=serial,
        capabilities=capabilities,
        gpio_chips=_detect_gpio_chips(),
        i2c_buses=_detect_i2c_buses(),
        spi_buses=_detect_spi_buses(),
        uart_ports=_detect_uart_ports(),
    )


# =============================================================================
# Exports
# =============================================================================

__all__ = [
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
    # Detection functions
    "detect_platform",
    # Individual detectors
    "detect_raspberry_pi",
    "detect_rock_pi",
    "detect_windows",
    "get_platform_info",
    # Convenience functions
    "is_arduino_connected",
    "is_beaglebone",
    "is_esp32_connected",
    "is_jetson",
    "is_raspberry_pi",
    "is_simulation_mode",
]
