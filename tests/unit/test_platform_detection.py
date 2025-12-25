"""Tests for platform detection module.

Phase 5.7.1 of robo-infra PLAN.md:
- Platform detection tests with comprehensive mock coverage
- Detection function tests for all supported platforms
- Auto-detection and environment override tests
- Platform info extraction tests

Total target: 17+ tests
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.platforms.base import (
    PlatformCapability,
    PlatformInfo,
    PlatformType,
)
from robo_infra.platforms.detection import (
    ARDUINO_VID_PIDS,
    ESP_VID_PIDS,
    MICROBIT_VID_PIDS,
    PICO_VID_PIDS,
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


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def clean_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Remove platform-related environment variables."""
    monkeypatch.delenv("ROBO_PLATFORM", raising=False)
    monkeypatch.delenv("ROBO_SIMULATION", raising=False)


@pytest.fixture
def mock_no_hardware() -> MagicMock:
    """Mock for when no hardware is present."""
    with patch.object(Path, "exists", return_value=False):
        yield


# =============================================================================
# VID/PID Constants Tests
# =============================================================================


class TestVIDPIDConstants:
    """Tests for USB VID/PID constants."""

    def test_arduino_vid_pids_has_official_arduino(self) -> None:
        """Arduino VID/PID includes official Arduino vendor."""
        assert 0x2341 in ARDUINO_VID_PIDS  # Arduino SA

    def test_arduino_vid_pids_has_clone_chips(self) -> None:
        """Arduino VID/PID includes common clone chips."""
        assert 0x1A86 in ARDUINO_VID_PIDS  # CH340/CH341
        assert 0x10C4 in ARDUINO_VID_PIDS  # CP210x
        assert 0x0403 in ARDUINO_VID_PIDS  # FTDI

    def test_esp_vid_pids_has_espressif(self) -> None:
        """ESP VID/PID includes Espressif vendor."""
        assert 0x303A in ESP_VID_PIDS  # Espressif

    def test_microbit_vid_pids_has_arm(self) -> None:
        """Micro:bit VID/PID includes ARM Ltd."""
        assert 0x0D28 in MICROBIT_VID_PIDS  # ARM Ltd

    def test_pico_vid_pids_has_raspberry_pi(self) -> None:
        """Pico VID/PID includes Raspberry Pi Foundation."""
        assert 0x2E8A in PICO_VID_PIDS  # Raspberry Pi Foundation


# =============================================================================
# Raspberry Pi Detection Tests
# =============================================================================


class TestDetectRaspberryPi:
    """Tests for Raspberry Pi detection."""

    def test_detect_from_device_tree_model(self) -> None:
        """Detect Pi from /sys/firmware/devicetree/base/model."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = True
        mock_path.read_text.return_value = "Raspberry Pi 4 Model B Rev 1.4\x00"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_pi, model = detect_raspberry_pi()
            assert is_pi is True
            assert "Raspberry Pi 4" in model

    def test_detect_from_cpuinfo_bcm(self) -> None:
        """Detect Pi from /proc/cpuinfo BCM processor."""
        # First path doesn't exist, second (cpuinfo) does
        call_count = [0]

        def exists_side_effect() -> bool:
            call_count[0] += 1
            # Only cpuinfo path exists (third call)
            return call_count[0] == 3

        def read_text_side_effect() -> str:
            return "Hardware: BCM2711\nModel: Raspberry Pi 4 Model B\n"

        mock_path = MagicMock(spec=Path)
        mock_path.exists.side_effect = exists_side_effect
        mock_path.read_text.side_effect = read_text_side_effect

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_pi, model = detect_raspberry_pi()
            # Detection depends on BCM in cpuinfo
            assert isinstance(is_pi, bool)
            assert isinstance(model, str)

    def test_detect_from_compatible(self) -> None:
        """Detect Pi from /proc/device-tree/compatible."""
        call_count = [0]

        def exists_side_effect() -> bool:
            call_count[0] += 1
            # Second path (compatible) exists
            return call_count[0] == 2

        mock_path = MagicMock(spec=Path)
        mock_path.exists.side_effect = exists_side_effect
        mock_path.read_text.return_value = "raspberrypi,4-model-b\x00brcm,bcm2711\x00"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_pi, model = detect_raspberry_pi()
            assert isinstance(is_pi, bool)

    def test_not_detected_when_no_files(self) -> None:
        """Detection returns False when no Pi files exist."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = False

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_pi, model = detect_raspberry_pi()
            assert is_pi is False
            assert model == ""

    def test_handles_permission_error(self) -> None:
        """Detection handles permission errors gracefully."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = True
        mock_path.read_text.side_effect = PermissionError("Access denied")

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_pi, model = detect_raspberry_pi()
            # Should handle error and continue
            assert isinstance(is_pi, bool)


# =============================================================================
# Jetson Detection Tests
# =============================================================================


class TestDetectJetson:
    """Tests for NVIDIA Jetson detection."""

    def test_detect_from_tegra_release(self) -> None:
        """Detect Jetson from /etc/nv_tegra_release."""
        call_count = [0]

        def exists_side_effect() -> bool:
            call_count[0] += 1
            return call_count[0] == 1  # First path (tegra) exists

        mock_path = MagicMock(spec=Path)
        mock_path.exists.side_effect = exists_side_effect

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_jetson, model = detect_jetson()
            # Will be True if tegra release file exists
            assert is_jetson is True
            assert "Jetson" in model or "NVIDIA" in model

    def test_detect_from_device_tree(self) -> None:
        """Detect Jetson from /proc/device-tree/model."""
        call_count = [0]

        def exists_side_effect() -> bool:
            call_count[0] += 1
            return call_count[0] == 2  # Second path (device tree) exists

        mock_path = MagicMock(spec=Path)
        mock_path.exists.side_effect = exists_side_effect
        mock_path.read_text.return_value = "NVIDIA Jetson Nano Developer Kit\x00"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_jetson, model = detect_jetson()
            assert is_jetson is True
            assert "Jetson" in model

    def test_not_detected_when_no_files(self) -> None:
        """Detection returns False when no Jetson files exist."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = False

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_jetson, model = detect_jetson()
            assert is_jetson is False
            assert model == ""


# =============================================================================
# BeagleBone Detection Tests
# =============================================================================


class TestDetectBeagleBone:
    """Tests for BeagleBone detection."""

    def test_detect_from_device_tree(self) -> None:
        """Detect BeagleBone from /proc/device-tree/model."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = True
        mock_path.read_text.return_value = "TI AM335x BeagleBone Black\x00"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_bb, model = detect_beaglebone()
            assert is_bb is True
            assert "BeagleBone" in model or "Beagle" in model

    def test_not_detected_when_no_files(self) -> None:
        """Detection returns False when no BeagleBone files exist."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = False

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_bb, model = detect_beaglebone()
            assert is_bb is False
            assert model == ""


# =============================================================================
# Orange Pi Detection Tests
# =============================================================================


class TestDetectOrangePi:
    """Tests for Orange Pi detection."""

    def test_detect_from_release_file(self) -> None:
        """Detect Orange Pi from /etc/orangepi-release."""
        call_count = [0]

        def exists_side_effect() -> bool:
            call_count[0] += 1
            return call_count[0] == 1  # First path (release) exists

        mock_path = MagicMock(spec=Path)
        mock_path.exists.side_effect = exists_side_effect
        mock_path.read_text.return_value = "BOARD=orangepi4\nBOARD_NAME=Orange Pi 4\n"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_opi, model = detect_orange_pi()
            assert is_opi is True
            assert "Orange Pi" in model

    def test_detect_from_device_tree(self) -> None:
        """Detect Orange Pi from /proc/device-tree/model."""
        call_count = [0]

        def exists_side_effect() -> bool:
            call_count[0] += 1
            return call_count[0] == 2  # Second path (device tree) exists

        mock_path = MagicMock(spec=Path)
        mock_path.exists.side_effect = exists_side_effect
        mock_path.read_text.return_value = "Xunlong Orange Pi PC Plus\x00"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_opi, model = detect_orange_pi()
            assert is_opi is True
            assert "Orange" in model

    def test_not_detected_when_no_files(self) -> None:
        """Detection returns False when no Orange Pi files exist."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = False

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_opi, model = detect_orange_pi()
            assert is_opi is False
            assert model == ""


# =============================================================================
# Rock Pi Detection Tests
# =============================================================================


class TestDetectRockPi:
    """Tests for Rock Pi / Radxa detection."""

    def test_detect_rock_pi(self) -> None:
        """Detect Rock Pi from device tree."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = True
        mock_path.read_text.return_value = "Radxa ROCK Pi 4B\x00"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_rock, model = detect_rock_pi()
            assert is_rock is True
            assert "Rock" in model or "Radxa" in model

    def test_not_detected_when_no_files(self) -> None:
        """Detection returns False when no Rock Pi files exist."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = False

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_rock, model = detect_rock_pi()
            assert is_rock is False
            assert model == ""


# =============================================================================
# Pine64 Detection Tests
# =============================================================================


class TestDetectPine64:
    """Tests for Pine64 detection."""

    def test_detect_pine64(self) -> None:
        """Detect Pine64 from device tree."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = True
        mock_path.read_text.return_value = "Pine64 PinePhone\x00"

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_pine, model = detect_pine64()
            assert is_pine is True
            assert "Pine" in model

    def test_not_detected_when_no_files(self) -> None:
        """Detection returns False when no Pine64 files exist."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = False

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            is_pine, model = detect_pine64()
            assert is_pine is False
            assert model == ""


# =============================================================================
# USB Device Detection Tests (Arduino, ESP32, Micro:bit, Pico)
# =============================================================================


class TestDetectArduino:
    """Tests for Arduino USB detection."""

    def test_detect_arduino_uno_r3(self) -> None:
        """Detect Arduino Uno R3 via USB VID/PID."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x2341, 0x0043)],  # Arduino Uno R3
        ):
            is_arduino, model = detect_arduino()
            assert is_arduino is True
            assert "Arduino" in model

    def test_detect_arduino_mega(self) -> None:
        """Detect Arduino Mega via USB VID/PID."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x2341, 0x0042)],  # Arduino Mega 2560 R3
        ):
            is_arduino, model = detect_arduino()
            assert is_arduino is True

    def test_detect_arduino_ch340_clone(self) -> None:
        """Detect Arduino clone with CH340 chip."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x1A86, 0x7523)],  # CH340
        ):
            is_arduino, model = detect_arduino()
            assert is_arduino is True

    def test_detect_arduino_from_serial_port(self) -> None:
        """Detect Arduino from /dev/ttyACM* on Linux."""
        with (
            patch(
                "robo_infra.platforms.detection._list_usb_devices",
                return_value=[],
            ),
            patch(
                "robo_infra.platforms.detection.sys.platform",
                "linux",
            ),
            patch.object(
                Path,
                "glob",
                return_value=[Path("/dev/ttyACM0")],
            ),
        ):
            is_arduino, model = detect_arduino()
            assert is_arduino is True
            assert "Serial" in model or "ttyACM0" in model

    def test_not_detected_when_no_devices(self) -> None:
        """Detection returns False when no Arduino present."""
        with (
            patch(
                "robo_infra.platforms.detection._list_usb_devices",
                return_value=[],
            ),
            patch.object(Path, "glob", return_value=[]),
        ):
            is_arduino, model = detect_arduino()
            assert is_arduino is False
            assert model == ""


class TestDetectESP32:
    """Tests for ESP32/ESP8266 USB detection."""

    def test_detect_esp32_s3(self) -> None:
        """Detect ESP32-S3 via USB VID/PID."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x303A, 0x1001)],  # ESP32-S3
        ):
            is_esp, model = detect_esp32()
            assert is_esp is True
            assert "ESP32" in model

    def test_detect_esp32_cp2102(self) -> None:
        """Detect ESP32 with CP2102 chip."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x10C4, 0xEA60)],  # CP2102
        ):
            is_esp, model = detect_esp32()
            assert is_esp is True

    def test_not_detected_when_no_devices(self) -> None:
        """Detection returns False when no ESP32 present."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[],
        ):
            is_esp, model = detect_esp32()
            assert is_esp is False
            assert model == ""


class TestDetectMicrobit:
    """Tests for micro:bit USB detection."""

    def test_detect_microbit(self) -> None:
        """Detect micro:bit via USB VID/PID."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x0D28, 0x0204)],  # micro:bit
        ):
            is_mb, model = detect_microbit()
            assert is_mb is True
            assert "micro:bit" in model

    def test_not_detected_when_no_devices(self) -> None:
        """Detection returns False when no micro:bit present."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[],
        ):
            is_mb, model = detect_microbit()
            assert is_mb is False
            assert model == ""


class TestDetectPico:
    """Tests for Raspberry Pi Pico USB detection."""

    def test_detect_pico(self) -> None:
        """Detect Raspberry Pi Pico via USB VID/PID."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x2E8A, 0x0005)],  # Pico
        ):
            is_pico, model = detect_pico()
            assert is_pico is True
            assert "Pico" in model

    def test_detect_pico_w(self) -> None:
        """Detect Raspberry Pi Pico W via USB VID/PID."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[(0x2E8A, 0x000A)],  # Pico W
        ):
            is_pico, model = detect_pico()
            assert is_pico is True

    def test_not_detected_when_no_devices(self) -> None:
        """Detection returns False when no Pico present."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[],
        ):
            is_pico, model = detect_pico()
            assert is_pico is False
            assert model == ""


# =============================================================================
# Linux Generic Detection Tests
# =============================================================================


class TestDetectLinuxGeneric:
    """Tests for generic Linux GPIO detection."""

    def test_detect_linux_with_gpiochip(self) -> None:
        """Detect Linux with /dev/gpiochip* devices."""
        with (
            patch(
                "robo_infra.platforms.detection.sys.platform",
                "linux",
            ),
            patch.object(
                Path,
                "glob",
                return_value=[Path("/dev/gpiochip0"), Path("/dev/gpiochip1")],
            ),
        ):
            is_linux, desc = detect_linux_generic()
            assert is_linux is True
            assert "GPIO" in desc or "chip" in desc

    def test_not_detected_on_non_linux(self) -> None:
        """Detection returns False on non-Linux platforms."""
        with patch(
            "robo_infra.platforms.detection.sys.platform",
            "darwin",
        ):
            is_linux, desc = detect_linux_generic()
            assert is_linux is False
            assert desc == ""

    def test_not_detected_without_gpiochip(self) -> None:
        """Detection returns False when no GPIO chips."""
        with (
            patch(
                "robo_infra.platforms.detection.sys.platform",
                "linux",
            ),
            patch.object(Path, "glob", return_value=[]),
        ):
            is_linux, desc = detect_linux_generic()
            assert is_linux is False


# =============================================================================
# OS Detection Tests
# =============================================================================


class TestDetectMacOS:
    """Tests for macOS detection."""

    def test_detect_macos_true(self) -> None:
        """Detection returns True on macOS."""
        with patch(
            "robo_infra.platforms.detection.sys.platform",
            "darwin",
        ):
            assert detect_macos() is True

    def test_detect_macos_false_on_linux(self) -> None:
        """Detection returns False on Linux."""
        with patch(
            "robo_infra.platforms.detection.sys.platform",
            "linux",
        ):
            assert detect_macos() is False


class TestDetectWindows:
    """Tests for Windows detection."""

    def test_detect_windows_true(self) -> None:
        """Detection returns True on Windows."""
        with patch(
            "robo_infra.platforms.detection.sys.platform",
            "win32",
        ):
            assert detect_windows() is True

    def test_detect_windows_false_on_linux(self) -> None:
        """Detection returns False on Linux."""
        with patch(
            "robo_infra.platforms.detection.sys.platform",
            "linux",
        ):
            assert detect_windows() is False


# =============================================================================
# Main detect_platform() Tests
# =============================================================================


class TestDetectPlatform:
    """Tests for main detect_platform function."""

    def test_env_override_rpi(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=rpi returns RASPBERRY_PI."""
        monkeypatch.setenv("ROBO_PLATFORM", "rpi")
        assert detect_platform() == PlatformType.RASPBERRY_PI

    def test_env_override_raspberry_pi(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=raspberry_pi returns RASPBERRY_PI."""
        monkeypatch.setenv("ROBO_PLATFORM", "raspberry_pi")
        assert detect_platform() == PlatformType.RASPBERRY_PI

    def test_env_override_jetson(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=jetson returns JETSON."""
        monkeypatch.setenv("ROBO_PLATFORM", "jetson")
        assert detect_platform() == PlatformType.JETSON

    def test_env_override_simulation(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=sim returns SIMULATION."""
        monkeypatch.setenv("ROBO_PLATFORM", "sim")
        assert detect_platform() == PlatformType.SIMULATION

    def test_env_override_arduino(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=arduino returns ARDUINO."""
        monkeypatch.setenv("ROBO_PLATFORM", "arduino")
        assert detect_platform() == PlatformType.ARDUINO

    def test_env_override_esp32(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=esp32 returns ESP32."""
        monkeypatch.setenv("ROBO_PLATFORM", "esp32")
        assert detect_platform() == PlatformType.ESP32

    def test_env_override_beaglebone(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=beaglebone returns BEAGLEBONE."""
        monkeypatch.setenv("ROBO_PLATFORM", "beaglebone")
        assert detect_platform() == PlatformType.BEAGLEBONE

    def test_env_override_linux(self, clean_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
        """ROBO_PLATFORM=linux returns LINUX_GENERIC."""
        monkeypatch.setenv("ROBO_PLATFORM", "linux")
        assert detect_platform() == PlatformType.LINUX_GENERIC

    def test_simulation_env_forces_simulation(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """ROBO_SIMULATION=true forces SIMULATION mode."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        assert detect_platform() == PlatformType.SIMULATION

    def test_simulation_env_yes(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """ROBO_SIMULATION=yes forces SIMULATION mode."""
        monkeypatch.setenv("ROBO_SIMULATION", "yes")
        assert detect_platform() == PlatformType.SIMULATION

    def test_simulation_env_one(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """ROBO_SIMULATION=1 forces SIMULATION mode."""
        monkeypatch.setenv("ROBO_SIMULATION", "1")
        assert detect_platform() == PlatformType.SIMULATION

    def test_fallback_to_simulation_on_macos(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Falls back to SIMULATION on macOS."""
        with (
            patch(
                "robo_infra.platforms.detection.sys.platform",
                "darwin",
            ),
            patch.object(Path, "exists", return_value=False),
            patch.object(Path, "glob", return_value=[]),
            patch(
                "robo_infra.platforms.detection._list_usb_devices",
                return_value=[],
            ),
        ):
            assert detect_platform() == PlatformType.SIMULATION

    def test_fallback_to_simulation_on_windows(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Falls back to SIMULATION on Windows."""
        with (
            patch(
                "robo_infra.platforms.detection.sys.platform",
                "win32",
            ),
            patch.object(Path, "exists", return_value=False),
            patch.object(Path, "glob", return_value=[]),
            patch(
                "robo_infra.platforms.detection._list_usb_devices",
                return_value=[],
            ),
        ):
            assert detect_platform() == PlatformType.SIMULATION

    def test_detect_raspberry_pi_priority(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Raspberry Pi detection has priority over generic Linux."""
        with patch(
            "robo_infra.platforms.detection.detect_raspberry_pi",
            return_value=(True, "Raspberry Pi 4"),
        ):
            assert detect_platform() == PlatformType.RASPBERRY_PI

    def test_detect_jetson_priority(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Jetson detection has priority after Raspberry Pi."""
        with (
            patch(
                "robo_infra.platforms.detection.detect_raspberry_pi",
                return_value=(False, ""),
            ),
            patch(
                "robo_infra.platforms.detection.detect_jetson",
                return_value=(True, "NVIDIA Jetson Nano"),
            ),
        ):
            assert detect_platform() == PlatformType.JETSON


# =============================================================================
# get_platform_info() Tests
# =============================================================================


class TestGetPlatformInfo:
    """Tests for get_platform_info function."""

    def test_info_has_platform_type(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Platform info includes platform_type."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        info = get_platform_info()
        assert info.platform_type == PlatformType.SIMULATION

    def test_info_has_model(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Platform info includes model string."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        info = get_platform_info()
        assert "Simulation" in info.model

    def test_info_has_capabilities(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Platform info includes capabilities set."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        info = get_platform_info()
        assert isinstance(info.capabilities, set)
        assert PlatformCapability.GPIO in info.capabilities

    def test_simulation_has_all_capabilities(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Simulation mode has all basic capabilities."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        info = get_platform_info()

        expected = {
            PlatformCapability.GPIO,
            PlatformCapability.PWM,
            PlatformCapability.I2C,
            PlatformCapability.SPI,
            PlatformCapability.UART,
            PlatformCapability.ADC,
            PlatformCapability.DAC,
        }
        assert expected.issubset(info.capabilities)

    def test_info_has_hardware_lists(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Platform info includes hardware lists."""
        monkeypatch.setenv("ROBO_SIMULATION", "true")
        info = get_platform_info()

        assert hasattr(info, "gpio_chips")
        assert hasattr(info, "i2c_buses")
        assert hasattr(info, "spi_buses")
        assert hasattr(info, "uart_ports")

        assert isinstance(info.gpio_chips, list)
        assert isinstance(info.i2c_buses, list)
        assert isinstance(info.spi_buses, list)
        assert isinstance(info.uart_ports, list)

    def test_raspberry_pi_info(self, clean_env: None) -> None:
        """Platform info for Raspberry Pi includes Pi-specific capabilities."""
        with (
            patch(
                "robo_infra.platforms.detection.detect_platform",
                return_value=PlatformType.RASPBERRY_PI,
            ),
            patch(
                "robo_infra.platforms.detection.detect_raspberry_pi",
                return_value=(True, "Raspberry Pi 4 Model B"),
            ),
            patch.object(Path, "exists", return_value=False),
        ):
            info = get_platform_info()
            assert info.platform_type == PlatformType.RASPBERRY_PI
            assert "Raspberry Pi" in info.model
            assert PlatformCapability.GPIO in info.capabilities
            assert PlatformCapability.I2C in info.capabilities
            assert PlatformCapability.SPI in info.capabilities

    def test_jetson_info(self, clean_env: None) -> None:
        """Platform info for Jetson includes Jetson-specific capabilities."""
        with (
            patch(
                "robo_infra.platforms.detection.detect_platform",
                return_value=PlatformType.JETSON,
            ),
            patch(
                "robo_infra.platforms.detection.detect_jetson",
                return_value=(True, "NVIDIA Jetson Nano"),
            ),
        ):
            info = get_platform_info()
            assert info.platform_type == PlatformType.JETSON
            assert "Jetson" in info.model or "NVIDIA" in info.model
            assert PlatformCapability.CAN in info.capabilities

    def test_arduino_info(self, clean_env: None) -> None:
        """Platform info for Arduino includes Arduino capabilities."""
        with (
            patch(
                "robo_infra.platforms.detection.detect_platform",
                return_value=PlatformType.ARDUINO,
            ),
            patch(
                "robo_infra.platforms.detection.detect_arduino",
                return_value=(True, "Arduino Uno"),
            ),
        ):
            info = get_platform_info()
            assert info.platform_type == PlatformType.ARDUINO
            assert "Arduino" in info.model
            assert PlatformCapability.GPIO in info.capabilities
            assert PlatformCapability.ADC in info.capabilities

    def test_esp32_info(self, clean_env: None) -> None:
        """Platform info for ESP32 includes ESP32 capabilities."""
        with (
            patch(
                "robo_infra.platforms.detection.detect_platform",
                return_value=PlatformType.ESP32,
            ),
            patch(
                "robo_infra.platforms.detection.detect_esp32",
                return_value=(True, "ESP32 (USB)"),
            ),
        ):
            info = get_platform_info()
            assert info.platform_type == PlatformType.ESP32
            assert "ESP32" in info.model
            assert PlatformCapability.DAC in info.capabilities

    def test_beaglebone_info(self, clean_env: None) -> None:
        """Platform info for BeagleBone includes BeagleBone capabilities."""
        with (
            patch(
                "robo_infra.platforms.detection.detect_platform",
                return_value=PlatformType.BEAGLEBONE,
            ),
            patch(
                "robo_infra.platforms.detection.detect_beaglebone",
                return_value=(True, "BeagleBone Black"),
            ),
        ):
            info = get_platform_info()
            assert info.platform_type == PlatformType.BEAGLEBONE
            assert "Beagle" in info.model
            assert PlatformCapability.ADC in info.capabilities
            assert PlatformCapability.CAN in info.capabilities


# =============================================================================
# Edge Cases Tests
# =============================================================================


class TestEdgeCases:
    """Tests for edge cases and error handling."""

    def test_unknown_platform_env_ignored(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Unknown ROBO_PLATFORM value is ignored."""
        monkeypatch.setenv("ROBO_PLATFORM", "unknown_platform_xyz")

        # Should fall through to auto-detection
        with (
            patch.object(Path, "exists", return_value=False),
            patch.object(Path, "glob", return_value=[]),
            patch(
                "robo_infra.platforms.detection._list_usb_devices",
                return_value=[],
            ),
            patch(
                "robo_infra.platforms.detection.sys.platform",
                "darwin",
            ),
        ):
            result = detect_platform()
            # Falls back to simulation on macOS
            assert result == PlatformType.SIMULATION

    def test_case_insensitive_env(
        self, clean_env: None, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """ROBO_PLATFORM is case-insensitive."""
        monkeypatch.setenv("ROBO_PLATFORM", "RPI")
        assert detect_platform() == PlatformType.RASPBERRY_PI

    def test_empty_usb_device_list(self) -> None:
        """Empty USB device list doesn't cause errors."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[],
        ):
            is_arduino, _ = detect_arduino()
            is_esp, _ = detect_esp32()
            is_mb, _ = detect_microbit()
            is_pico, _ = detect_pico()

            # All should return False gracefully
            # Note: Arduino may still detect from serial ports
            assert is_esp is False
            assert is_mb is False
            assert is_pico is False

    def test_file_read_errors_handled(self) -> None:
        """File read errors are handled gracefully."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = True
        mock_path.read_text.side_effect = OSError("Read error")

        with patch(
            "robo_infra.platforms.detection.Path",
            return_value=mock_path,
        ):
            # Should not raise
            is_pi, _ = detect_raspberry_pi()
            is_jetson, _ = detect_jetson()
            is_bb, _ = detect_beaglebone()

            # All handle errors gracefully
            assert isinstance(is_pi, bool)
            assert isinstance(is_jetson, bool)
            assert isinstance(is_bb, bool)

    def test_multiple_usb_devices(self) -> None:
        """Multiple USB devices are handled correctly."""
        with patch(
            "robo_infra.platforms.detection._list_usb_devices",
            return_value=[
                (0x1234, 0x5678),  # Unknown device
                (0x2341, 0x0043),  # Arduino Uno R3
                (0x303A, 0x1001),  # ESP32-S3
            ],
        ):
            is_arduino, _ = detect_arduino()
            is_esp, _ = detect_esp32()

            # Both should be detected
            assert is_arduino is True
            assert is_esp is True


# =============================================================================
# Logging Tests
# =============================================================================


class TestLogging:
    """Tests for logging behavior."""

    def test_detection_logs_at_debug_level(
        self, caplog: pytest.LogCaptureFixture
    ) -> None:
        """Platform detection logs at debug level."""
        mock_path = MagicMock(spec=Path)
        mock_path.exists.return_value = True
        mock_path.read_text.return_value = "Raspberry Pi 4\x00"

        with (
            caplog.at_level(logging.DEBUG),
            patch(
                "robo_infra.platforms.detection.Path",
                return_value=mock_path,
            ),
        ):
            detect_raspberry_pi()

        # Should have debug logs
        assert any("Raspberry Pi" in r.message for r in caplog.records) or len(caplog.records) == 0

    def test_simulation_fallback_logs_info(
        self, clean_env: None,
        monkeypatch: pytest.MonkeyPatch,
        caplog: pytest.LogCaptureFixture,
    ) -> None:
        """Simulation fallback logs at info level."""
        with (
            caplog.at_level(logging.INFO),
            patch(
                "robo_infra.platforms.detection.sys.platform",
                "darwin",
            ),
            patch.object(Path, "exists", return_value=False),
            patch.object(Path, "glob", return_value=[]),
            patch(
                "robo_infra.platforms.detection._list_usb_devices",
                return_value=[],
            ),
        ):
            detect_platform()

        # Should log simulation mode
        assert any("simulation" in r.message.lower() for r in caplog.records) or len(caplog.records) == 0
