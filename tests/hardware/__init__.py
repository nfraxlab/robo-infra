"""
Hardware test framework for robo-infra.

This package contains tests that require real hardware to run.
These tests are skipped by default and only run on appropriate hardware
platforms with the proper configuration.

Usage:
    # Run hardware tests on a Raspberry Pi
    ROBO_HARDWARE_TESTS=1 pytest tests/hardware/ -v

    # Run specific hardware category
    ROBO_HARDWARE_TESTS=1 pytest tests/hardware/test_servo_control.py -v

Configuration:
    Set ROBO_HARDWARE_TESTS=1 to enable hardware tests.
    Without this, all hardware tests are skipped for safety.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from enum import Enum

import pytest


class HardwareCategory(Enum):
    """Categories of hardware tests."""

    GPIO = "gpio"
    I2C = "i2c"
    SPI = "spi"
    PWM = "pwm"
    SERVO = "servo"
    MOTOR = "motor"
    SENSOR = "sensor"
    CAMERA = "camera"


@dataclass
class HardwareRequirement:
    """Defines what hardware is required for a test."""

    category: HardwareCategory
    devices: list[str] = field(default_factory=list)
    pins: list[int] = field(default_factory=list)
    i2c_addresses: list[int] = field(default_factory=list)
    spi_devices: list[str] = field(default_factory=list)
    description: str = ""


def hardware_tests_enabled() -> bool:
    """Check if hardware tests are enabled via environment variable."""
    return os.environ.get("ROBO_HARDWARE_TESTS", "").lower() in ("1", "true", "yes")


def is_simulation_mode() -> bool:
    """Check if we're in simulation mode."""
    return os.environ.get("ROBO_SIMULATION", "").lower() in ("1", "true", "yes")


def skip_unless_hardware(reason: str = "Requires real hardware"):
    """Decorator to skip test unless hardware tests are enabled."""
    return pytest.mark.skipif(
        not hardware_tests_enabled(),
        reason=f"{reason} (set ROBO_HARDWARE_TESTS=1 to enable)",
    )


def requires_platform(*platform_types: str):
    """Mark test as requiring specific platform(s).

    Args:
        platform_types: Platform type names (e.g., "raspberry_pi", "jetson")
    """
    from robo_infra.platforms import detect_platform

    def check() -> bool:
        if not hardware_tests_enabled():
            return False
        current = detect_platform()
        return current.value in platform_types

    return pytest.mark.skipif(
        not check(),
        reason=f"Requires platform: {', '.join(platform_types)}",
    )


def requires_device(device_path: str):
    """Mark test as requiring a specific device file.

    Args:
        device_path: Path to device file (e.g., "/dev/i2c-1")
    """
    return pytest.mark.skipif(
        not (hardware_tests_enabled() and os.path.exists(device_path)),
        reason=f"Requires device: {device_path}",
    )


def requires_i2c_address(bus: int, address: int):
    """Mark test as requiring an I2C device at specific address.

    Args:
        bus: I2C bus number (0 or 1 typically)
        address: 7-bit I2C address
    """
    return pytest.mark.skipif(
        not hardware_tests_enabled(),
        reason=f"Requires I2C device at 0x{address:02X} on bus {bus}",
    )


# Common hardware markers
hardware = pytest.mark.hardware
raspberry_pi_only = requires_platform("raspberry_pi")
jetson_only = requires_platform("jetson")
gpio_required = requires_device("/dev/gpiochip0")
i2c_required = requires_device("/dev/i2c-1")
spi_required = requires_device("/dev/spidev0.0")


__all__ = [
    "HardwareCategory",
    "HardwareRequirement",
    "gpio_required",
    "hardware",
    "hardware_tests_enabled",
    "i2c_required",
    "is_simulation_mode",
    "jetson_only",
    "raspberry_pi_only",
    "requires_device",
    "requires_i2c_address",
    "requires_platform",
    "skip_unless_hardware",
    "spi_required",
]
