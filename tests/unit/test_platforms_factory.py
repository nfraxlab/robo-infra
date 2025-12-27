"""Tests for platform factory functions.

Tests the cross-platform hardware access factory: get_gpio, get_i2c, get_spi, get_uart.
"""

from __future__ import annotations

import os
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.bus import Bus, I2CBus, SPIBus
from robo_infra.core.exceptions import HardwareNotFoundError
from robo_infra.core.pin import Pin, PinMode
from robo_infra.platforms.base import (
    PlatformCapability,
    PlatformInfo,
    PlatformType,
    reset_platform,
)
from robo_infra.platforms.factory import (
    get_gpio,
    get_i2c,
    get_spi,
    get_uart,
    list_available_gpio,
    list_available_i2c,
    list_available_spi,
    list_available_uart,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture(autouse=True)
def reset_platform_state() -> None:
    """Reset platform state before each test."""
    reset_platform()
    # Clear environment variables
    for var in ("ROBO_PLATFORM", "ROBO_SIMULATION"):
        if var in os.environ:
            del os.environ[var]
    yield
    reset_platform()


@pytest.fixture
def simulation_env() -> None:
    """Set up simulation environment."""
    os.environ["ROBO_SIMULATION"] = "true"
    reset_platform()


@pytest.fixture
def mock_platform() -> MagicMock:
    """Create a mock platform with all capabilities."""
    platform = MagicMock()
    platform.name = "MockPlatform"
    platform.platform_type = PlatformType.SIMULATION
    platform.capabilities = {
        PlatformCapability.GPIO,
        PlatformCapability.I2C,
        PlatformCapability.SPI,
        PlatformCapability.UART,
        PlatformCapability.PWM,
    }
    platform.info = PlatformInfo(
        platform_type=PlatformType.SIMULATION,
        model="Mock Platform",
        capabilities=platform.capabilities,
        gpio_chips=["gpiochip0", "gpiochip1"],
        i2c_buses=[0, 1],
        spi_buses=[(0, 0), (0, 1), (1, 0)],
        uart_ports=["/dev/ttyS0", "/dev/ttyUSB0"],
    )
    return platform


@pytest.fixture
def mock_limited_platform() -> MagicMock:
    """Create a mock platform with limited capabilities."""
    platform = MagicMock()
    platform.name = "LimitedPlatform"
    platform.platform_type = PlatformType.LINUX_GENERIC
    platform.capabilities = {PlatformCapability.GPIO}  # Only GPIO
    platform.info = PlatformInfo(
        platform_type=PlatformType.LINUX_GENERIC,
        model="Limited Platform",
        capabilities=platform.capabilities,
        gpio_chips=["gpiochip0"],
        i2c_buses=[],
        spi_buses=[],
        uart_ports=[],
    )
    return platform


# =============================================================================
# get_gpio Tests
# =============================================================================


class TestGetGpio:
    """Tests for get_gpio factory function."""

    def test_get_gpio_returns_pin(self, mock_platform: MagicMock) -> None:
        """Test that get_gpio returns a Pin instance."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            pin = get_gpio(17)

        assert pin is mock_pin
        mock_platform.get_pin.assert_called_once()

    def test_get_gpio_passes_pin_id(self, mock_platform: MagicMock) -> None:
        """Test that pin_id is passed correctly."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(17)

        call_args = mock_platform.get_pin.call_args
        assert call_args[0][0] == 17

    def test_get_gpio_with_string_pin_id(self, mock_platform: MagicMock) -> None:
        """Test that string pin_id works."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio("GPIO17")

        call_args = mock_platform.get_pin.call_args
        assert call_args[0][0] == "GPIO17"

    def test_get_gpio_passes_mode(self, mock_platform: MagicMock) -> None:
        """Test that mode parameter is passed."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(17, mode=PinMode.INPUT)

        call_args = mock_platform.get_pin.call_args
        assert call_args[1]["mode"] == PinMode.INPUT

    def test_get_gpio_default_mode_is_output(self, mock_platform: MagicMock) -> None:
        """Test that default mode is OUTPUT."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(17)

        call_args = mock_platform.get_pin.call_args
        assert call_args[1]["mode"] == PinMode.OUTPUT

    def test_get_gpio_passes_pull(self, mock_platform: MagicMock) -> None:
        """Test that pull parameter is passed."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(17, pull="up")

        call_args = mock_platform.get_pin.call_args
        assert call_args[1]["pull"] == "up"

    def test_get_gpio_passes_inverted(self, mock_platform: MagicMock) -> None:
        """Test that inverted parameter is passed."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(17, inverted=True)

        call_args = mock_platform.get_pin.call_args
        assert call_args[1]["inverted"] is True

    def test_get_gpio_passes_name(self, mock_platform: MagicMock) -> None:
        """Test that name parameter is passed."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(17, name="LED")

        call_args = mock_platform.get_pin.call_args
        assert call_args[1]["name"] == "LED"

    def test_get_gpio_passes_kwargs(self, mock_platform: MagicMock) -> None:
        """Test that extra kwargs are passed."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(17, frequency=50, duty_cycle=7.5)

        call_args = mock_platform.get_pin.call_args
        assert call_args[1]["frequency"] == 50
        assert call_args[1]["duty_cycle"] == 7.5

    def test_get_gpio_raises_when_gpio_not_supported(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that HardwareNotFoundError is raised when GPIO not supported."""
        # Remove GPIO capability
        mock_limited_platform.capabilities = set()

        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ), pytest.raises(HardwareNotFoundError) as exc_info:
            get_gpio(17)

        assert "GPIO" in str(exc_info.value)

    def test_get_gpio_pwm_mode(self, mock_platform: MagicMock) -> None:
        """Test getting GPIO in PWM mode."""
        mock_pin = MagicMock(spec=Pin)
        mock_platform.get_pin.return_value = mock_pin

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_gpio(12, mode=PinMode.PWM, frequency=50)

        call_args = mock_platform.get_pin.call_args
        assert call_args[1]["mode"] == PinMode.PWM
        assert call_args[1]["frequency"] == 50


class TestGetGpioSimulation:
    """Tests for get_gpio with simulation platform."""

    def test_get_gpio_simulation_mode(self, simulation_env: None) -> None:
        """Test get_gpio works in simulation mode."""
        pin = get_gpio(17, mode=PinMode.OUTPUT)
        assert pin is not None
        assert hasattr(pin, "high")
        assert hasattr(pin, "low")

    def test_get_gpio_simulation_input_mode(self, simulation_env: None) -> None:
        """Test get_gpio works in INPUT mode in simulation."""
        pin = get_gpio(17, mode=PinMode.INPUT)
        assert pin is not None

    def test_get_gpio_simulation_pwm_mode(self, simulation_env: None) -> None:
        """Test get_gpio works in PWM mode in simulation."""
        pin = get_gpio(12, mode=PinMode.PWM, frequency=50)
        assert pin is not None


# =============================================================================
# get_i2c Tests
# =============================================================================


class TestGetI2c:
    """Tests for get_i2c factory function."""

    def test_get_i2c_returns_bus(self, mock_platform: MagicMock) -> None:
        """Test that get_i2c returns an I2CBus instance."""
        mock_bus = MagicMock(spec=I2CBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            bus = get_i2c(bus=1)

        assert bus is mock_bus
        mock_platform.get_bus.assert_called_once()

    def test_get_i2c_default_bus_is_1(self, mock_platform: MagicMock) -> None:
        """Test that default bus is 1."""
        mock_bus = MagicMock(spec=I2CBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_i2c()

        call_args = mock_platform.get_bus.call_args
        assert call_args[0][0] == "i2c"
        assert call_args[1]["bus"] == 1

    def test_get_i2c_passes_bus_number(self, mock_platform: MagicMock) -> None:
        """Test that bus number is passed correctly."""
        mock_bus = MagicMock(spec=I2CBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_i2c(bus=0)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["bus"] == 0

    def test_get_i2c_passes_frequency(self, mock_platform: MagicMock) -> None:
        """Test that frequency is passed correctly."""
        mock_bus = MagicMock(spec=I2CBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_i2c(bus=1, frequency=400000)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["frequency"] == 400000

    def test_get_i2c_passes_name(self, mock_platform: MagicMock) -> None:
        """Test that name is passed correctly."""
        mock_bus = MagicMock(spec=I2CBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_i2c(bus=1, name="sensor_bus")

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["name"] == "sensor_bus"

    def test_get_i2c_raises_when_i2c_not_supported(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that HardwareNotFoundError is raised when I2C not supported."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ), pytest.raises(HardwareNotFoundError) as exc_info:
            get_i2c(bus=1)

        assert "I2C" in str(exc_info.value)

    def test_get_i2c_warns_for_unavailable_bus(
        self, mock_platform: MagicMock, caplog: pytest.LogCaptureFixture
    ) -> None:
        """Test that warning is logged for unavailable bus number."""
        mock_bus = MagicMock(spec=I2CBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            import logging

            with caplog.at_level(logging.WARNING):
                get_i2c(bus=99)  # Not in [0, 1]

        assert "not in detected buses" in caplog.text


class TestGetI2cSimulation:
    """Tests for get_i2c with simulation platform."""

    def test_get_i2c_simulation_mode(self, simulation_env: None) -> None:
        """Test get_i2c works in simulation mode."""
        bus = get_i2c(bus=1)
        assert bus is not None
        assert hasattr(bus, "scan")

    def test_get_i2c_simulation_scan(self, simulation_env: None) -> None:
        """Test I2C scan works in simulation."""
        bus = get_i2c(bus=1)
        devices = bus.scan()
        assert isinstance(devices, list)


# =============================================================================
# get_spi Tests
# =============================================================================


class TestGetSpi:
    """Tests for get_spi factory function."""

    def test_get_spi_returns_bus(self, mock_platform: MagicMock) -> None:
        """Test that get_spi returns an SPIBus instance."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            bus = get_spi(bus=0, device=0)

        assert bus is mock_bus
        mock_platform.get_bus.assert_called_once()

    def test_get_spi_default_bus_and_device(self, mock_platform: MagicMock) -> None:
        """Test that default bus=0, device=0."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_spi()

        call_args = mock_platform.get_bus.call_args
        assert call_args[0][0] == "spi"
        assert call_args[1]["bus"] == 0
        assert call_args[1]["device"] == 0

    def test_get_spi_passes_bus_and_device(self, mock_platform: MagicMock) -> None:
        """Test that bus and device are passed correctly."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_spi(bus=1, device=2)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["bus"] == 1
        assert call_args[1]["device"] == 2

    def test_get_spi_passes_speed_hz(self, mock_platform: MagicMock) -> None:
        """Test that speed_hz is passed correctly."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_spi(bus=0, device=0, speed_hz=1_000_000)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["speed_hz"] == 1_000_000

    def test_get_spi_passes_mode(self, mock_platform: MagicMock) -> None:
        """Test that SPI mode is passed correctly."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_spi(bus=0, device=0, mode=3)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["mode"] == 3

    def test_get_spi_passes_bits_per_word(self, mock_platform: MagicMock) -> None:
        """Test that bits_per_word is passed correctly."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_spi(bus=0, device=0, bits_per_word=16)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["bits_per_word"] == 16

    def test_get_spi_default_mode_is_0(self, mock_platform: MagicMock) -> None:
        """Test that default SPI mode is 0."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_spi()

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["mode"] == 0

    def test_get_spi_default_bits_per_word_is_8(self, mock_platform: MagicMock) -> None:
        """Test that default bits_per_word is 8."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_spi()

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["bits_per_word"] == 8

    def test_get_spi_raises_when_spi_not_supported(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that HardwareNotFoundError is raised when SPI not supported."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ), pytest.raises(HardwareNotFoundError) as exc_info:
            get_spi(bus=0, device=0)

        assert "SPI" in str(exc_info.value)

    def test_get_spi_warns_for_unavailable_bus(
        self, mock_platform: MagicMock, caplog: pytest.LogCaptureFixture
    ) -> None:
        """Test that warning is logged for unavailable bus:device."""
        mock_bus = MagicMock(spec=SPIBus)
        mock_platform.get_bus.return_value = mock_bus

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            import logging

            with caplog.at_level(logging.WARNING):
                get_spi(bus=99, device=99)  # Not in [(0,0), (0,1), (1,0)]

        assert "not in detected buses" in caplog.text


class TestGetSpiSimulation:
    """Tests for get_spi with simulation platform."""

    def test_get_spi_simulation_mode(self, simulation_env: None) -> None:
        """Test get_spi works in simulation mode."""
        bus = get_spi(bus=0, device=0)
        assert bus is not None
        assert hasattr(bus, "transfer")

    def test_get_spi_simulation_transfer(self, simulation_env: None) -> None:
        """Test SPI transfer works in simulation."""
        bus = get_spi(bus=0, device=0)
        result = bus.transfer([0x01, 0x02, 0x03])
        assert isinstance(result, (list, bytes, bytearray))


# =============================================================================
# get_uart Tests
# =============================================================================


class TestGetUart:
    """Tests for get_uart factory function."""

    def test_get_uart_returns_bus(self, mock_platform: MagicMock) -> None:
        """Test that get_uart returns a Bus instance."""
        mock_bus = MagicMock(spec=Bus)
        mock_platform.get_bus.return_value = mock_bus
        mock_platform.capabilities.add(PlatformCapability.UART)

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            bus = get_uart(port="/dev/ttyS0")

        assert bus is mock_bus
        mock_platform.get_bus.assert_called_once()

    def test_get_uart_auto_detects_port(self, mock_platform: MagicMock) -> None:
        """Test that port is auto-detected if not specified."""
        mock_bus = MagicMock(spec=Bus)
        mock_platform.get_bus.return_value = mock_bus
        mock_platform.capabilities.add(PlatformCapability.UART)

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_uart()  # No port specified

        call_args = mock_platform.get_bus.call_args
        # Should use first available port from info
        assert call_args[1]["port"] == "/dev/ttyS0"

    def test_get_uart_passes_baudrate(self, mock_platform: MagicMock) -> None:
        """Test that baudrate is passed correctly."""
        mock_bus = MagicMock(spec=Bus)
        mock_platform.get_bus.return_value = mock_bus
        mock_platform.capabilities.add(PlatformCapability.UART)

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_uart(port="/dev/ttyS0", baudrate=115200)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["baudrate"] == 115200

    def test_get_uart_default_baudrate_is_9600(self, mock_platform: MagicMock) -> None:
        """Test that default baudrate is 9600."""
        mock_bus = MagicMock(spec=Bus)
        mock_platform.get_bus.return_value = mock_bus
        mock_platform.capabilities.add(PlatformCapability.UART)

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_uart(port="/dev/ttyS0")

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["baudrate"] == 9600

    def test_get_uart_passes_timeout(self, mock_platform: MagicMock) -> None:
        """Test that timeout is passed correctly."""
        mock_bus = MagicMock(spec=Bus)
        mock_platform.get_bus.return_value = mock_bus
        mock_platform.capabilities.add(PlatformCapability.UART)

        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            get_uart(port="/dev/ttyS0", timeout=5.0)

        call_args = mock_platform.get_bus.call_args
        assert call_args[1]["timeout"] == 5.0

    def test_get_uart_raises_when_uart_not_supported(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that HardwareNotFoundError is raised when UART not supported."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ), pytest.raises(HardwareNotFoundError) as exc_info:
            get_uart(port="/dev/ttyS0")

        assert "UART" in str(exc_info.value)


class TestGetUartSimulation:
    """Tests for get_uart with simulation platform."""

    def test_get_uart_simulation_mode(self, simulation_env: None) -> None:
        """Test get_uart works in simulation mode."""
        bus = get_uart(port="/dev/ttyS0")
        assert bus is not None


# =============================================================================
# list_available_* Tests
# =============================================================================


class TestListAvailableGpio:
    """Tests for list_available_gpio function."""

    def test_list_available_gpio_returns_chips(self, mock_platform: MagicMock) -> None:
        """Test that GPIO chips are returned."""
        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            chips = list_available_gpio()

        assert chips == ["gpiochip0", "gpiochip1"]

    def test_list_available_gpio_empty_when_no_gpio(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that empty list is returned when GPIO not supported."""
        mock_limited_platform.capabilities = set()

        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ):
            chips = list_available_gpio()

        assert chips == []


class TestListAvailableI2c:
    """Tests for list_available_i2c function."""

    def test_list_available_i2c_returns_buses(self, mock_platform: MagicMock) -> None:
        """Test that I2C buses are returned."""
        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            buses = list_available_i2c()

        assert buses == [0, 1]

    def test_list_available_i2c_empty_when_no_i2c(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that empty list is returned when I2C not supported."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ):
            buses = list_available_i2c()

        assert buses == []


class TestListAvailableSpi:
    """Tests for list_available_spi function."""

    def test_list_available_spi_returns_buses(self, mock_platform: MagicMock) -> None:
        """Test that SPI bus:device pairs are returned."""
        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            buses = list_available_spi()

        assert buses == [(0, 0), (0, 1), (1, 0)]

    def test_list_available_spi_empty_when_no_spi(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that empty list is returned when SPI not supported."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ):
            buses = list_available_spi()

        assert buses == []


class TestListAvailableUart:
    """Tests for list_available_uart function."""

    def test_list_available_uart_returns_ports(self, mock_platform: MagicMock) -> None:
        """Test that UART ports are returned."""
        with patch("robo_infra.platforms.factory.get_platform", return_value=mock_platform):
            ports = list_available_uart()

        assert ports == ["/dev/ttyS0", "/dev/ttyUSB0"]

    def test_list_available_uart_empty_when_no_uart(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that empty list is returned when UART not supported."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ):
            ports = list_available_uart()

        assert ports == []


# =============================================================================
# Integration Tests
# =============================================================================


class TestFactoryIntegration:
    """Integration tests for factory functions."""

    def test_factory_functions_use_same_platform_instance(
        self, simulation_env: None
    ) -> None:
        """Test that all factory functions use the same platform."""
        # Call multiple factory functions
        gpio_pin = get_gpio(17)
        i2c_bus = get_i2c(bus=1)
        spi_bus = get_spi(bus=0, device=0)

        # They should all work (same simulation platform)
        assert gpio_pin is not None
        assert i2c_bus is not None
        assert spi_bus is not None

    def test_factory_respects_simulation_env(self) -> None:
        """Test that ROBO_SIMULATION environment variable works."""
        os.environ["ROBO_SIMULATION"] = "true"
        reset_platform()

        pin = get_gpio(17)

        # Should be a simulated pin
        assert pin is not None
        # Verify it's from simulation platform
        from robo_infra.platforms.base import get_platform

        platform = get_platform()
        assert platform.platform_type == PlatformType.SIMULATION

    def test_factory_respects_platform_env(self) -> None:
        """Test that ROBO_PLATFORM environment variable works."""
        os.environ["ROBO_PLATFORM"] = "simulation"
        reset_platform()

        pin = get_gpio(17)
        assert pin is not None

        from robo_infra.platforms.base import get_platform

        platform = get_platform()
        assert platform.platform_type == PlatformType.SIMULATION


# =============================================================================
# Error Handling Tests
# =============================================================================


class TestFactoryErrorHandling:
    """Tests for factory error handling."""

    def test_get_gpio_error_includes_pin_info(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that GPIO error includes pin information."""
        mock_limited_platform.capabilities = set()

        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ), pytest.raises(HardwareNotFoundError) as exc_info:
            get_gpio(17)

        assert "17" in str(exc_info.value) or "GPIO" in str(exc_info.value)

    def test_get_i2c_error_includes_bus_info(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that I2C error includes bus information."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ), pytest.raises(HardwareNotFoundError) as exc_info:
            get_i2c(bus=5)

        assert "5" in str(exc_info.value) or "I2C" in str(exc_info.value)

    def test_get_spi_error_includes_bus_device_info(
        self, mock_limited_platform: MagicMock
    ) -> None:
        """Test that SPI error includes bus:device information."""
        with patch(
            "robo_infra.platforms.factory.get_platform", return_value=mock_limited_platform
        ), pytest.raises(HardwareNotFoundError) as exc_info:
            get_spi(bus=1, device=2)

        assert "SPI" in str(exc_info.value)


# =============================================================================
# Module Exports Tests
# =============================================================================


class TestModuleExports:
    """Tests for module exports."""

    def test_factory_module_exports_get_gpio(self) -> None:
        """Test that get_gpio is exported from factory module."""
        from robo_infra.platforms.factory import get_gpio as exported_get_gpio

        assert exported_get_gpio is not None
        assert callable(exported_get_gpio)

    def test_factory_module_exports_get_i2c(self) -> None:
        """Test that get_i2c is exported from factory module."""
        from robo_infra.platforms.factory import get_i2c as exported_get_i2c

        assert exported_get_i2c is not None
        assert callable(exported_get_i2c)

    def test_factory_module_exports_get_spi(self) -> None:
        """Test that get_spi is exported from factory module."""
        from robo_infra.platforms.factory import get_spi as exported_get_spi

        assert exported_get_spi is not None
        assert callable(exported_get_spi)

    def test_factory_module_exports_get_uart(self) -> None:
        """Test that get_uart is exported from factory module."""
        from robo_infra.platforms.factory import get_uart as exported_get_uart

        assert exported_get_uart is not None
        assert callable(exported_get_uart)

    def test_platforms_init_exports_factory_functions(self) -> None:
        """Test that factory functions are exported from platforms __init__."""
        from robo_infra.platforms import (
            get_gpio,
            get_i2c,
            get_spi,
            get_uart,
            list_available_gpio,
            list_available_i2c,
            list_available_spi,
            list_available_uart,
        )

        assert all(
            callable(f)
            for f in [
                get_gpio,
                get_i2c,
                get_spi,
                get_uart,
                list_available_gpio,
                list_available_i2c,
                list_available_spi,
                list_available_uart,
            ]
        )

    def test_factory_all_exports(self) -> None:
        """Test __all__ exports are correct."""
        from robo_infra.platforms import factory

        expected = [
            "get_gpio",
            "get_i2c",
            "get_spi",
            "get_uart",
            "list_available_gpio",
            "list_available_i2c",
            "list_available_spi",
            "list_available_uart",
        ]

        for name in expected:
            assert name in factory.__all__, f"{name} not in __all__"
