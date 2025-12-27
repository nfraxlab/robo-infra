"""
Pytest configuration for hardware tests.

Hardware tests require:
1. ROBO_HARDWARE_TESTS=1 environment variable
2. Actual hardware platform (Raspberry Pi, Jetson, etc.)
3. Connected peripherals (servos, sensors, etc.)

See docs/hardware-testing.md for setup instructions.
"""

from __future__ import annotations

import logging
import os
from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    from _pytest.config import Config
    from _pytest.nodes import Item

logger = logging.getLogger(__name__)


def pytest_configure(config: Config) -> None:
    """Register custom markers for hardware tests."""
    config.addinivalue_line(
        "markers",
        "hardware: marks tests as requiring real hardware (skip without ROBO_HARDWARE_TESTS=1)",
    )
    config.addinivalue_line(
        "markers",
        "raspberry_pi: marks tests as requiring Raspberry Pi",
    )
    config.addinivalue_line(
        "markers",
        "jetson: marks tests as requiring NVIDIA Jetson",
    )
    config.addinivalue_line(
        "markers",
        "gpio: marks tests as requiring GPIO access",
    )
    config.addinivalue_line(
        "markers",
        "i2c: marks tests as requiring I2C bus",
    )
    config.addinivalue_line(
        "markers",
        "spi: marks tests as requiring SPI bus",
    )
    config.addinivalue_line(
        "markers",
        "pwm: marks tests as requiring PWM hardware",
    )
    config.addinivalue_line(
        "markers",
        "servo: marks tests as requiring connected servo",
    )
    config.addinivalue_line(
        "markers",
        "motor: marks tests as requiring connected motor",
    )
    config.addinivalue_line(
        "markers",
        "sensor: marks tests as requiring connected sensor",
    )


def pytest_collection_modifyitems(config: Config, items: list[Item]) -> None:
    """Skip hardware tests if ROBO_HARDWARE_TESTS is not set."""
    hardware_enabled = os.environ.get("ROBO_HARDWARE_TESTS", "").lower() in (
        "1",
        "true",
        "yes",
    )

    skip_hardware = pytest.mark.skip(
        reason="Hardware tests disabled (set ROBO_HARDWARE_TESTS=1 to enable)"
    )

    for item in items:
        # Skip any test marked with @pytest.mark.hardware
        if "hardware" in item.keywords and not hardware_enabled:
            item.add_marker(skip_hardware)


@pytest.fixture
def hardware_enabled() -> bool:
    """Check if hardware tests are enabled."""
    return os.environ.get("ROBO_HARDWARE_TESTS", "").lower() in ("1", "true", "yes")


@pytest.fixture
def require_hardware() -> None:
    """Skip test if hardware tests are not enabled."""
    if os.environ.get("ROBO_HARDWARE_TESTS", "").lower() not in ("1", "true", "yes"):
        pytest.skip("Hardware tests not enabled (set ROBO_HARDWARE_TESTS=1)")


@pytest.fixture
def require_raspberry_pi() -> None:
    """Skip test if not running on Raspberry Pi."""
    from robo_infra.platforms import PlatformType, detect_platform

    if detect_platform() != PlatformType.RASPBERRY_PI:
        pytest.skip("Requires Raspberry Pi hardware")


@pytest.fixture
def require_jetson() -> None:
    """Skip test if not running on NVIDIA Jetson."""
    from robo_infra.platforms import PlatformType, detect_platform

    if detect_platform() != PlatformType.JETSON:
        pytest.skip("Requires NVIDIA Jetson hardware")


@pytest.fixture
def require_gpio() -> None:
    """Skip test if GPIO is not available."""
    if not os.path.exists("/dev/gpiochip0"):
        pytest.skip("Requires GPIO access (/dev/gpiochip0)")


@pytest.fixture
def require_i2c() -> None:
    """Skip test if I2C bus is not available."""
    if not os.path.exists("/dev/i2c-1"):
        pytest.skip("Requires I2C bus (/dev/i2c-1)")


@pytest.fixture
def require_spi() -> None:
    """Skip test if SPI is not available."""
    if not os.path.exists("/dev/spidev0.0"):
        pytest.skip("Requires SPI device (/dev/spidev0.0)")


@pytest.fixture
def platform_info() -> dict:
    """Get information about the current platform."""
    from robo_infra.platforms import (
        PlatformType,
        detect_platform,
        get_platform_capabilities,
    )

    platform_type = detect_platform()

    return {
        "type": platform_type,
        "name": platform_type.value,
        "capabilities": get_platform_capabilities(platform_type),
        "has_gpio": os.path.exists("/dev/gpiochip0"),
        "has_i2c": os.path.exists("/dev/i2c-1"),
        "has_spi": os.path.exists("/dev/spidev0.0"),
    }


@pytest.fixture
def test_led_pin() -> int:
    """GPIO pin for test LED (configurable via env var)."""
    return int(os.environ.get("ROBO_TEST_LED_PIN", "17"))


@pytest.fixture
def test_servo_channel() -> int:
    """PCA9685 channel for test servo (configurable via env var)."""
    return int(os.environ.get("ROBO_TEST_SERVO_CHANNEL", "0"))


@pytest.fixture
def test_i2c_bus() -> int:
    """I2C bus number for tests (configurable via env var)."""
    return int(os.environ.get("ROBO_TEST_I2C_BUS", "1"))


@pytest.fixture
def pca9685_address() -> int:
    """I2C address for PCA9685 (configurable via env var)."""
    return int(os.environ.get("ROBO_TEST_PCA9685_ADDR", "0x40"), 16)


@pytest.fixture
def mpu6050_address() -> int:
    """I2C address for MPU6050 IMU (configurable via env var)."""
    return int(os.environ.get("ROBO_TEST_MPU6050_ADDR", "0x68"), 16)
