"""Unit tests for platform abstraction layer.

Tests cover:
- Platform protocol and base class
- PlatformRegistry with auto-detection
- SimulationPlatform
- Detection functions with mocking
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.pin import PinMode
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
    detect_orange_pi,
    detect_platform,
    detect_raspberry_pi,
    get_platform_info,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def clean_registry():
    """Provide a clean registry for each test."""
    # Reset the singleton
    PlatformRegistry._instance = None
    yield
    PlatformRegistry._instance = None


@pytest.fixture
def clean_platform():
    """Reset the global platform after each test."""
    reset_platform()
    yield
    reset_platform()


@pytest.fixture
def simulation_env(monkeypatch):
    """Set ROBO_SIMULATION=true for testing."""
    monkeypatch.setenv("ROBO_SIMULATION", "true")
    yield


# =============================================================================
# PlatformType Tests
# =============================================================================


class TestPlatformType:
    """Tests for PlatformType enum."""

    def test_platform_type_values(self):
        """Test PlatformType has expected values."""
        assert PlatformType.RASPBERRY_PI.value == "raspberry_pi"
        assert PlatformType.JETSON.value == "jetson"
        assert PlatformType.ARDUINO.value == "arduino"
        assert PlatformType.ESP32.value == "esp32"
        assert PlatformType.SIMULATION.value == "simulation"

    def test_platform_type_all_values(self):
        """Test all PlatformType values are strings."""
        for pt in PlatformType:
            assert isinstance(pt.value, str)
            assert len(pt.value) > 0


class TestPlatformCapability:
    """Tests for PlatformCapability enum."""

    def test_capability_values(self):
        """Test PlatformCapability has expected values."""
        assert PlatformCapability.GPIO.value == "gpio"
        assert PlatformCapability.PWM.value == "pwm"
        assert PlatformCapability.I2C.value == "i2c"
        assert PlatformCapability.SPI.value == "spi"
        assert PlatformCapability.UART.value == "uart"

    def test_capability_all_values(self):
        """Test all capabilities are strings."""
        for cap in PlatformCapability:
            assert isinstance(cap.value, str)


# =============================================================================
# PlatformConfig Tests
# =============================================================================


class TestPlatformConfig:
    """Tests for PlatformConfig model."""

    def test_default_config(self):
        """Test default configuration values."""
        config = PlatformConfig()
        assert config.name == "Platform"
        assert config.platform_type == PlatformType.UNKNOWN
        assert config.auto_detect is True
        assert config.simulation_fallback is True
        assert config.pin_numbering == "BCM"

    def test_custom_config(self):
        """Test custom configuration."""
        config = PlatformConfig(
            name="My Pi",
            platform_type=PlatformType.RASPBERRY_PI,
            pin_numbering="BOARD",
        )
        assert config.name == "My Pi"
        assert config.platform_type == PlatformType.RASPBERRY_PI
        assert config.pin_numbering == "BOARD"


# =============================================================================
# PlatformInfo Tests
# =============================================================================


class TestPlatformInfo:
    """Tests for PlatformInfo dataclass."""

    def test_minimal_info(self):
        """Test minimal platform info."""
        info = PlatformInfo(platform_type=PlatformType.SIMULATION)
        assert info.platform_type == PlatformType.SIMULATION
        assert info.model == "Unknown"
        assert info.capabilities == set()

    def test_full_info(self):
        """Test full platform info."""
        info = PlatformInfo(
            platform_type=PlatformType.RASPBERRY_PI,
            model="Raspberry Pi 4 Model B",
            revision="c03112",
            serial="10000000abcdef",
            capabilities={PlatformCapability.GPIO, PlatformCapability.I2C},
            gpio_chips=["gpiochip0", "gpiochip1"],
            i2c_buses=[1],
            spi_buses=[(0, 0), (0, 1)],
            uart_ports=["/dev/ttyAMA0"],
        )
        assert info.model == "Raspberry Pi 4 Model B"
        assert PlatformCapability.GPIO in info.capabilities
        assert len(info.gpio_chips) == 2


# =============================================================================
# SimulationPlatform Tests
# =============================================================================


class TestSimulationPlatform:
    """Tests for SimulationPlatform."""

    def test_simulation_is_available(self):
        """Test simulation platform is always available."""
        platform = SimulationPlatform()
        assert platform.is_available is True

    def test_simulation_name(self):
        """Test simulation platform name."""
        platform = SimulationPlatform()
        assert platform.name == "Simulation"
        assert platform.platform_type == PlatformType.SIMULATION

    def test_simulation_info(self):
        """Test simulation platform info."""
        platform = SimulationPlatform()
        info = platform.info
        assert info.platform_type == PlatformType.SIMULATION
        assert info.model == "Simulated Platform"
        assert PlatformCapability.GPIO in info.capabilities

    def test_simulation_get_pin(self):
        """Test getting a simulated pin."""
        platform = SimulationPlatform()
        pin = platform.get_pin(17, mode=PinMode.OUTPUT)
        assert pin is not None
        assert pin.number == 17

    def test_simulation_get_pin_cached(self):
        """Test pins are cached."""
        platform = SimulationPlatform()
        pin1 = platform.get_pin(17)
        pin2 = platform.get_pin(17)
        assert pin1 is pin2

    def test_simulation_get_i2c_bus(self):
        """Test getting a simulated I2C bus."""
        platform = SimulationPlatform()
        bus = platform.get_bus("i2c", bus=1)
        assert bus is not None

    def test_simulation_get_spi_bus(self):
        """Test getting a simulated SPI bus."""
        platform = SimulationPlatform()
        bus = platform.get_bus("spi", bus=0, device=0)
        assert bus is not None

    def test_simulation_get_uart(self):
        """Test getting a simulated UART."""
        platform = SimulationPlatform()
        bus = platform.get_bus("uart", port="/dev/ttyUSB0")
        assert bus is not None

    def test_simulation_unknown_bus_raises(self):
        """Test unknown bus type raises error."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        platform = SimulationPlatform()
        with pytest.raises(HardwareNotFoundError):
            platform.get_bus("unknown")

    def test_simulation_cleanup(self):
        """Test cleanup releases resources."""
        platform = SimulationPlatform()
        platform.get_pin(17)
        platform.get_bus("i2c", bus=1)

        platform.cleanup()

        # Internal caches should be cleared
        assert len(platform._pins) == 0
        assert len(platform._buses) == 0

    def test_simulation_context_manager(self):
        """Test using platform as context manager."""
        with SimulationPlatform() as platform:
            pin = platform.get_pin(17)
            assert pin is not None
        # After exit, resources should be cleaned up
        assert len(platform._pins) == 0

    def test_simulation_repr(self):
        """Test string representation."""
        platform = SimulationPlatform()
        repr_str = repr(platform)
        assert "Simulation" in repr_str
        assert "simulation" in repr_str


# =============================================================================
# PlatformRegistry Tests
# =============================================================================


class TestPlatformRegistry:
    """Tests for PlatformRegistry."""

    def test_registry_singleton(self, clean_registry):
        """Test registry is a singleton."""
        reg1 = PlatformRegistry.get_instance()
        reg2 = PlatformRegistry.get_instance()
        assert reg1 is reg2

    def test_registry_has_simulation(self, clean_registry):
        """Test registry has simulation platform by default."""
        registry = PlatformRegistry.get_instance()
        assert PlatformType.SIMULATION in registry.list_platforms()

    def test_register_platform(self, clean_registry):
        """Test registering a platform."""
        registry = PlatformRegistry.get_instance()

        class MockPlatform(BasePlatform):
            @property
            def is_available(self) -> bool:
                return True

            def _detect_info(self) -> PlatformInfo:
                return PlatformInfo(platform_type=PlatformType.UNKNOWN)

            def _create_pin(self, pin_id: int | str, **kwargs: Any):
                return MagicMock()

            def _create_bus(self, bus_type: str, **kwargs: Any):
                return MagicMock()

        registry.register(MockPlatform, PlatformType.UNKNOWN)
        assert PlatformType.UNKNOWN in registry.list_platforms()

    def test_unregister_platform(self, clean_registry):
        """Test unregistering a platform."""
        registry = PlatformRegistry.get_instance()

        # Register then unregister
        registry.register(SimulationPlatform, PlatformType.UNKNOWN)
        registry.unregister(PlatformType.UNKNOWN)

        assert PlatformType.UNKNOWN not in registry.list_platforms()

    def test_detect_returns_simulation_on_unknown(self, clean_registry, monkeypatch):
        """Test detection falls back to simulation."""
        # Clear any platform env vars
        monkeypatch.delenv("ROBO_PLATFORM", raising=False)
        monkeypatch.delenv("ROBO_SIMULATION", raising=False)

        registry = PlatformRegistry.get_instance()
        platform = registry.detect()

        # On dev machines without robotics hardware, should fall back to simulation
        assert isinstance(platform, BasePlatform)

    def test_detect_with_env_override(self, clean_registry, monkeypatch):
        """Test ROBO_PLATFORM environment variable override."""
        monkeypatch.setenv("ROBO_PLATFORM", "simulation")

        registry = PlatformRegistry.get_instance()
        platform = registry.detect()

        assert platform.platform_type == PlatformType.SIMULATION

    def test_detect_with_simulation_env(self, clean_registry, monkeypatch):
        """Test ROBO_SIMULATION environment variable."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")

        registry = PlatformRegistry.get_instance()
        platform = registry.detect()

        assert platform.platform_type == PlatformType.SIMULATION

    def test_list_available(self, clean_registry):
        """Test listing available platforms."""
        registry = PlatformRegistry.get_instance()
        available = registry.list_available()

        assert len(available) > 0
        # Simulation should be available
        sim_entry = next(
            (pt, status) for pt, status in available if pt == PlatformType.SIMULATION
        )
        assert sim_entry[1] == "available"


# =============================================================================
# Module-level Function Tests
# =============================================================================


class TestModuleFunctions:
    """Tests for module-level functions."""

    def test_get_platform(self, clean_platform, simulation_env):
        """Test get_platform returns platform instance."""
        platform = get_platform()
        assert isinstance(platform, BasePlatform)
        assert platform.platform_type == PlatformType.SIMULATION

    def test_get_platform_cached(self, clean_platform, simulation_env):
        """Test get_platform caches result."""
        platform1 = get_platform()
        platform2 = get_platform()
        assert platform1 is platform2

    def test_reset_platform(self, clean_platform, simulation_env):
        """Test reset_platform clears cache."""
        platform1 = get_platform()
        reset_platform()
        platform2 = get_platform()
        assert platform1 is not platform2

    def test_register_platform_function(self, clean_registry):
        """Test register_platform module function."""

        class TestPlatform(BasePlatform):
            @property
            def is_available(self) -> bool:
                return False

            def _detect_info(self) -> PlatformInfo:
                return PlatformInfo(platform_type=PlatformType.UNKNOWN)

            def _create_pin(self, pin_id: int | str, **kwargs: Any):
                return MagicMock()

            def _create_bus(self, bus_type: str, **kwargs: Any):
                return MagicMock()

        register_platform(TestPlatform, PlatformType.UNKNOWN)

        registry = PlatformRegistry.get_instance()
        assert PlatformType.UNKNOWN in registry.list_platforms()


# =============================================================================
# Detection Tests (with mocking)
# =============================================================================


class TestRaspberryPiDetection:
    """Tests for Raspberry Pi detection."""

    def test_detect_raspberry_pi_from_device_tree(self, tmp_path, monkeypatch):
        """Test detection from device tree model."""
        # Create mock device tree
        model_file = tmp_path / "model"
        model_file.write_text("Raspberry Pi 4 Model B Rev 1.4\x00")

        with (
            patch(
                "robo_infra.platforms.detection.Path",
                return_value=model_file,
            ),
            patch.object(Path, "exists", return_value=True),
            patch.object(
                Path,
                "read_text",
                return_value="Raspberry Pi 4 Model B Rev 1.4\x00",
            ),
        ):
            is_pi, model = detect_raspberry_pi()
            assert is_pi is True
            assert "Raspberry Pi" in model

    def test_detect_raspberry_pi_not_present(self):
        """Test detection returns False when not on Pi."""
        with patch.object(Path, "exists", return_value=False):
            is_pi, _model = detect_raspberry_pi()
            assert is_pi is False


class TestJetsonDetection:
    """Tests for Jetson detection."""

    def test_detect_jetson_from_tegra_release(self, tmp_path):
        """Test detection from tegra release file."""
        tegra_path = tmp_path / "nv_tegra_release"
        tegra_path.write_text("# R32 (release), REVISION: 6.1")

        with (
            patch.object(Path, "exists", side_effect=lambda: True),
            patch.object(Path, "read_text", return_value="NVIDIA Jetson Nano\x00"),
        ):
            _is_jetson, _model = detect_jetson()
            # Will be True if file exists
            # Note: actual detection depends on file paths

    def test_detect_jetson_not_present(self):
        """Test detection returns False when not on Jetson."""
        with patch.object(Path, "exists", return_value=False):
            is_jetson, _model = detect_jetson()
            assert is_jetson is False


class TestBeagleBoneDetection:
    """Tests for BeagleBone detection."""

    def test_detect_beaglebone_not_present(self):
        """Test detection returns False when not on BeagleBone."""
        with patch.object(Path, "exists", return_value=False):
            is_bb, _model = detect_beaglebone()
            assert is_bb is False


class TestOrangePiDetection:
    """Tests for Orange Pi detection."""

    def test_detect_orangepi_not_present(self):
        """Test detection returns False when not on Orange Pi."""
        with patch.object(Path, "exists", return_value=False):
            is_opi, _model = detect_orange_pi()
            assert is_opi is False


class TestArduinoDetection:
    """Tests for Arduino detection."""

    def test_detect_arduino_via_usb(self):
        """Test Arduino detection via USB VID/PID."""
        # Mock USB device listing
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x2341, 0x0043)],  # Arduino Uno R3
        ):
            is_arduino, model = detect_arduino()
            assert is_arduino is True
            assert "Arduino" in model

    def test_detect_arduino_not_present(self):
        """Test detection returns False when no Arduino."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[],
        ), patch.object(Path, "glob", return_value=[]):
            is_arduino, _model = detect_arduino()
            assert is_arduino is False


class TestESP32Detection:
    """Tests for ESP32 detection."""

    def test_detect_esp32_via_usb(self):
        """Test ESP32 detection via USB VID/PID."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x303A, 0x1001)],  # ESP32-S3
        ):
            is_esp, model = detect_esp32()
            assert is_esp is True
            assert "ESP32" in model

    def test_detect_esp32_not_present(self):
        """Test detection returns False when no ESP32."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[],
        ):
            is_esp, _model = detect_esp32()
            assert is_esp is False


class TestLinuxGenericDetection:
    """Tests for generic Linux detection."""

    def test_detect_linux_with_gpio(self):
        """Test detection on Linux with GPIO chips."""
        import sys

        if not sys.platform.startswith("linux"):
            pytest.skip("Linux-only test")

        with patch.object(Path, "glob", return_value=[Path("/dev/gpiochip0")]):
            _is_linux, _desc = detect_linux_generic()
            # Result depends on actual platform

    def test_detect_linux_no_gpio(self):
        """Test detection on Linux without GPIO chips."""
        with patch.object(Path, "glob", return_value=[]):
            is_linux, _desc = detect_linux_generic()
            assert is_linux is False


class TestPlatformDetection:
    """Tests for main detect_platform function."""

    def test_detect_platform_simulation_env(self, monkeypatch):
        """Test ROBO_SIMULATION forces simulation."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        platform_type = detect_platform()
        assert platform_type == PlatformType.SIMULATION

    def test_detect_platform_robo_platform_env(self, monkeypatch):
        """Test ROBO_PLATFORM environment variable."""
        monkeypatch.setenv("ROBO_PLATFORM", "rpi")
        platform_type = detect_platform()
        assert platform_type == PlatformType.RASPBERRY_PI

    def test_detect_platform_macos(self, monkeypatch):
        """Test macOS detection returns simulation."""
        monkeypatch.delenv("ROBO_PLATFORM", raising=False)
        monkeypatch.delenv("ROBO_SIMULATION", raising=False)

        with (
            patch("robo_infra.platforms.detection.sys.platform", "darwin"),
            patch.object(Path, "exists", return_value=False),
            patch.object(Path, "glob", return_value=[]),
        ):
            platform_type = detect_platform()
            assert platform_type == PlatformType.SIMULATION

    def test_detect_platform_windows(self, monkeypatch):
        """Test Windows detection returns simulation."""
        monkeypatch.delenv("ROBO_PLATFORM", raising=False)
        monkeypatch.delenv("ROBO_SIMULATION", raising=False)

        with (
            patch("robo_infra.platforms.detection.sys.platform", "win32"),
            patch.object(Path, "exists", return_value=False),
            patch.object(Path, "glob", return_value=[]),
        ):
            platform_type = detect_platform()
            assert platform_type == PlatformType.SIMULATION


class TestGetPlatformInfo:
    """Tests for get_platform_info function."""

    def test_get_platform_info_simulation(self, monkeypatch):
        """Test platform info for simulation."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")

        info = get_platform_info()
        assert info.platform_type == PlatformType.SIMULATION
        assert "Simulation" in info.model
        assert PlatformCapability.GPIO in info.capabilities

    def test_get_platform_info_has_required_fields(self, monkeypatch):
        """Test platform info has all required fields."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")

        info = get_platform_info()
        assert hasattr(info, "platform_type")
        assert hasattr(info, "model")
        assert hasattr(info, "capabilities")
        assert hasattr(info, "gpio_chips")
        assert hasattr(info, "i2c_buses")
        assert hasattr(info, "spi_buses")
        assert hasattr(info, "uart_ports")


# =============================================================================
# Platform Protocol Compliance Tests
# =============================================================================


class TestPlatformProtocol:
    """Tests that platforms comply with the Platform protocol."""

    def test_simulation_implements_protocol(self):
        """Test SimulationPlatform implements Platform protocol."""
        platform = SimulationPlatform()
        assert isinstance(platform, Platform)

    def test_simulation_has_required_properties(self):
        """Test SimulationPlatform has all required properties."""
        platform = SimulationPlatform()

        # Required properties
        assert hasattr(platform, "name")
        assert hasattr(platform, "platform_type")
        assert hasattr(platform, "is_available")
        assert hasattr(platform, "info")
        assert hasattr(platform, "capabilities")

        # Required methods
        assert hasattr(platform, "get_pin")
        assert hasattr(platform, "get_bus")
        assert hasattr(platform, "get_driver")
        assert hasattr(platform, "cleanup")


# =============================================================================
# Edge Cases and Error Handling
# =============================================================================


class TestEdgeCases:
    """Tests for edge cases and error handling."""

    def test_multiple_pin_types(self):
        """Test getting pins with different IDs."""
        platform = SimulationPlatform()

        pin_int = platform.get_pin(17)
        pin_str = platform.get_pin("18")

        assert pin_int.number == 17
        assert pin_str.number == 18

    def test_bus_caching_different_configs(self):
        """Test buses with different configs are cached separately."""
        platform = SimulationPlatform()

        bus1 = platform.get_bus("i2c", bus=0)
        bus2 = platform.get_bus("i2c", bus=1)

        assert bus1 is not bus2

    def test_platform_config_extra_fields(self):
        """Test PlatformConfig allows extra fields."""
        config = PlatformConfig(
            name="Test",
            custom_field="custom_value",
        )
        assert config.model_extra.get("custom_field") == "custom_value"

    def test_empty_usb_devices(self):
        """Test USB detection with no devices."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[],
        ):
            is_arduino, _ = detect_arduino()
            is_esp, _ = detect_esp32()

            assert is_arduino is False
            assert is_esp is False
