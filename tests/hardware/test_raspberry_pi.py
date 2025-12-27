"""
Raspberry Pi-specific hardware tests.

These tests validate robo-infra functionality on Raspberry Pi hardware.
Tests are skipped unless running on a Raspberry Pi with ROBO_HARDWARE_TESTS=1.

Required Hardware:
- Raspberry Pi (any model with GPIO header)
- Optional: LED on GPIO 17 for GPIO output test
- Optional: Button on GPIO 27 for GPIO input test

Environment Variables:
- ROBO_HARDWARE_TESTS=1: Enable hardware tests
- ROBO_TEST_LED_PIN: GPIO pin for LED (default: 17)
- ROBO_TEST_BUTTON_PIN: GPIO pin for button (default: 27)
"""

from __future__ import annotations

import os
import time

import pytest


pytestmark = [
    pytest.mark.hardware,
    pytest.mark.raspberry_pi,
]


class TestRaspberryPiDetection:
    """Tests for Raspberry Pi platform detection."""

    def test_platform_detection(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
    ) -> None:
        """Test that platform is correctly detected as Raspberry Pi."""
        from robo_infra.platforms import PlatformType, detect_platform

        platform = detect_platform()
        assert platform == PlatformType.RASPBERRY_PI

    def test_platform_capabilities(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
    ) -> None:
        """Test that Raspberry Pi capabilities are detected."""
        from robo_infra.platforms import (
            PlatformCapability,
            PlatformType,
            get_platform_capabilities,
        )

        caps = get_platform_capabilities(PlatformType.RASPBERRY_PI)

        # Raspberry Pi should have these capabilities
        assert PlatformCapability.GPIO in caps
        assert PlatformCapability.I2C in caps
        assert PlatformCapability.SPI in caps
        assert PlatformCapability.PWM in caps

    def test_platform_info(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
        platform_info: dict,
    ) -> None:
        """Test platform info fixture provides correct data."""
        from robo_infra.platforms import PlatformType

        assert platform_info["type"] == PlatformType.RASPBERRY_PI
        assert platform_info["name"] == "raspberry_pi"
        assert platform_info["has_gpio"] is True


class TestRaspberryPiGPIO:
    """Tests for GPIO functionality on Raspberry Pi."""

    def test_gpio_device_exists(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
        require_gpio: None,
    ) -> None:
        """Test that GPIO device file exists."""
        assert os.path.exists("/dev/gpiochip0")

    def test_gpio_output_led_blink(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
        require_gpio: None,
        test_led_pin: int,
    ) -> None:
        """Test GPIO output by blinking an LED.

        Requires: LED connected to GPIO pin (default: 17)
        """
        from robo_infra.drivers import GPIODriver, PinMode

        driver = GPIODriver()
        driver.setup_pin(test_led_pin, PinMode.OUTPUT)

        # Blink pattern: on-off-on-off
        for _ in range(2):
            driver.write_pin(test_led_pin, True)
            time.sleep(0.1)
            driver.write_pin(test_led_pin, False)
            time.sleep(0.1)

        # Clean up
        driver.cleanup()

    def test_gpio_input_read(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
        require_gpio: None,
    ) -> None:
        """Test GPIO input reading.

        This test reads a pin state without requiring specific hardware.
        """
        from robo_infra.drivers import GPIODriver, PinMode

        button_pin = int(os.environ.get("ROBO_TEST_BUTTON_PIN", "27"))

        driver = GPIODriver()
        driver.setup_pin(button_pin, PinMode.INPUT_PULLUP)

        # Just verify we can read without error
        state = driver.read_pin(button_pin)
        assert isinstance(state, bool)

        driver.cleanup()


class TestRaspberryPiI2C:
    """Tests for I2C functionality on Raspberry Pi."""

    def test_i2c_bus_exists(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
        require_i2c: None,
    ) -> None:
        """Test that I2C bus device exists."""
        assert os.path.exists("/dev/i2c-1")

    def test_i2c_scan(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
        require_i2c: None,
        test_i2c_bus: int,
    ) -> None:
        """Test I2C bus scanning.

        Lists all detected I2C devices on the bus.
        """
        from robo_infra.core import I2CBus

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        devices = bus.scan()
        print(f"\nDetected I2C devices: {[hex(addr) for addr in devices]}")

        bus.close()

        # At minimum, the scan should complete without error
        assert isinstance(devices, list)


class TestRaspberryPiSPI:
    """Tests for SPI functionality on Raspberry Pi."""

    def test_spi_device_exists(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
        require_spi: None,
    ) -> None:
        """Test that SPI device file exists."""
        assert os.path.exists("/dev/spidev0.0")


class TestRaspberryPiPWM:
    """Tests for PWM functionality on Raspberry Pi."""

    def test_hardware_pwm_pins(
        self,
        require_hardware: None,
        require_raspberry_pi: None,
    ) -> None:
        """Test that hardware PWM pins are correctly identified."""
        from robo_infra.utils.hardware import PlatformOptimizer

        optimizer = PlatformOptimizer()

        # Raspberry Pi hardware PWM pins: GPIO 12, 13, 18, 19
        assert optimizer.is_hardware_pwm_pin(12)
        assert optimizer.is_hardware_pwm_pin(13)
        assert optimizer.is_hardware_pwm_pin(18)
        assert optimizer.is_hardware_pwm_pin(19)

        # Non-PWM pins
        assert not optimizer.is_hardware_pwm_pin(17)
        assert not optimizer.is_hardware_pwm_pin(27)
