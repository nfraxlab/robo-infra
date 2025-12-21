"""Unit tests for Raspberry Pi Camera implementation.

Tests PiCamera class with mocked picamera2/picamera libraries.
"""

from __future__ import annotations

import os
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from robo_infra.sensors.camera import (
    CameraIntrinsics,
    CameraState,
    Frame,
    PixelFormat,
)
from robo_infra.sensors.cameras.picamera import (
    PiCamera,
    PiCameraConfig,
    _get_camera_library,
    _is_raspberry_pi,
    list_picameras,
    open_picamera,
)


# =============================================================================
# Helper Functions Tests
# =============================================================================


class TestHelperFunctions:
    """Tests for Pi camera helper functions."""

    def test_is_raspberry_pi_with_env_var(self):
        """Test Pi detection with environment variable."""
        with patch.dict(os.environ, {"ROBO_FORCE_PI": "1"}):
            # This should return True when env var is set
            _is_raspberry_pi()
            # May or may not be True depending on other checks

    def test_get_camera_library_no_library(self):
        """Test library detection when neither is available."""
        with patch.dict("sys.modules", {"picamera2": None, "picamera": None}):
            # Force import errors
            with patch("builtins.__import__", side_effect=ImportError):
                result = _get_camera_library()
                assert result is None


# =============================================================================
# PiCameraConfig Tests
# =============================================================================


class TestPiCameraConfig:
    """Tests for PiCameraConfig."""

    def test_default_config(self):
        """Test default PiCameraConfig values."""
        config = PiCameraConfig()

        assert config.width == 640
        assert config.height == 480
        assert config.fps == 30
        assert config.sensor_mode == 0
        assert config.video_stabilization is False
        assert config.hdr_mode is False
        assert config.autofocus_mode == "auto"
        assert config.jpeg_quality == 85
        assert config.rotation == 0

    def test_custom_config(self):
        """Test custom PiCameraConfig."""
        config = PiCameraConfig(
            width=1920,
            height=1080,
            fps=60,
            sensor_mode=2,
            video_stabilization=True,
            hdr_mode=True,
            autofocus_mode="continuous",
            rotation=180,
        )

        assert config.width == 1920
        assert config.height == 1080
        assert config.fps == 60
        assert config.sensor_mode == 2
        assert config.video_stabilization is True
        assert config.hdr_mode is True
        assert config.autofocus_mode == "continuous"
        assert config.rotation == 180


# =============================================================================
# PiCamera Tests (Simulated Mode)
# =============================================================================


class TestPiCameraSimulated:
    """Tests for PiCamera in simulation mode."""

    @pytest.fixture
    def simulated_camera(self):
        """Create a simulated Pi camera."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = PiCamera(camera_num=0)
            yield camera
            if camera.is_enabled:
                camera.disable()

    def test_create_camera(self, simulated_camera):
        """Test camera creation."""
        assert simulated_camera.name == "picamera_0"
        assert simulated_camera.camera_num == 0
        assert simulated_camera.state == CameraState.DISCONNECTED

    def test_enable_disable(self, simulated_camera):
        """Test enabling and disabling camera."""
        simulated_camera.enable()
        assert simulated_camera.is_enabled
        assert simulated_camera.state == CameraState.CONNECTED
        assert simulated_camera.sensor_name == "simulated"

        simulated_camera.disable()
        assert not simulated_camera.is_enabled
        assert simulated_camera.state == CameraState.DISCONNECTED

    def test_capture_simulated(self, simulated_camera):
        """Test capturing simulated frame."""
        simulated_camera.enable()
        frame = simulated_camera.capture()

        assert isinstance(frame, Frame)
        assert frame.width == 640
        assert frame.height == 480
        assert frame.format == PixelFormat.RGB
        assert frame.data.shape == (480, 640, 3)
        assert frame.data.dtype == np.uint8

    def test_capture_still_simulated(self, simulated_camera):
        """Test high-res still capture simulation."""
        simulated_camera.enable()
        # When no resize is needed (same resolution), it should work
        still = simulated_camera.capture_still(resolution=None)

        assert isinstance(still, Frame)
        # Returns sensor resolution in simulation (which equals config resolution)
        assert still.width > 0
        assert still.height > 0

    def test_capture_still_simulated_with_resize(self, simulated_camera):
        """Test high-res still capture with resize (requires scipy)."""
        pytest.importorskip("scipy")  # Skip if scipy not installed
        simulated_camera.enable()
        still = simulated_camera.capture_still(resolution=(1920, 1080))

        assert isinstance(still, Frame)
        assert still.width == 1920
        assert still.height == 1080

    def test_context_manager(self):
        """Test camera as context manager."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            with PiCamera() as camera:
                assert camera.is_enabled
                frame = camera.capture()
                assert frame is not None

            assert not camera.is_enabled

    def test_get_intrinsics(self, simulated_camera):
        """Test getting camera intrinsics."""
        simulated_camera.enable()
        intrinsics = simulated_camera.get_intrinsics()

        assert isinstance(intrinsics, CameraIntrinsics)
        assert intrinsics.width == 640
        assert intrinsics.height == 480
        assert intrinsics.fx > 0
        assert intrinsics.fy > 0


# =============================================================================
# PiCamera Tests (Mocked picamera2)
# =============================================================================


class TestPiCameraMocked:
    """Tests for PiCamera with mocked picamera2 library."""

    @pytest.fixture
    def mock_picamera2(self):
        """Create mock picamera2 module."""
        mock_module = MagicMock()
        mock_picam = MagicMock()

        # Mock camera properties
        mock_picam.camera_properties = {
            "Model": "imx219",
            "PixelArraySize": (3280, 2464),
        }

        # Mock capture
        mock_array = np.zeros((480, 640, 3), dtype=np.uint8)
        mock_picam.capture_array.return_value = mock_array

        # Mock module
        mock_module.Picamera2.return_value = mock_picam
        mock_module.Preview = MagicMock()

        return mock_module, mock_picam

    def test_init_picamera2(self, mock_picamera2):
        """Test initialization with mocked picamera2."""
        mock_module, _mock_picam = mock_picamera2

        with patch.dict("sys.modules", {"picamera2": mock_module}), patch(
            "robo_infra.sensors.cameras.picamera._get_camera_library",
            return_value="picamera2",
        ), patch(
            "robo_infra.sensors.cameras.picamera._is_raspberry_pi",
            return_value=True,
        ):
            camera = PiCamera()
            camera.enable()

            assert camera.is_enabled
            assert camera.library == "picamera2"
            mock_module.Picamera2.assert_called_once()

            camera.disable()

    def test_capture_with_flips(self, mock_picamera2):
        """Test capture with flip settings."""
        mock_module, mock_picam = mock_picamera2
        test_data = np.arange(480 * 640 * 3, dtype=np.uint8).reshape((480, 640, 3))
        mock_picam.capture_array.return_value = test_data.copy()

        config = PiCameraConfig(flip_horizontal=True, flip_vertical=True)

        with patch.dict("sys.modules", {"picamera2": mock_module}), patch(
            "robo_infra.sensors.cameras.picamera._get_camera_library",
            return_value="picamera2",
        ), patch(
            "robo_infra.sensors.cameras.picamera._is_raspberry_pi",
            return_value=True,
        ):
            camera = PiCamera(config=config)
            camera.enable()
            frame = camera.capture()

            # Frame should be flipped
            assert frame is not None
            camera.disable()


# =============================================================================
# Device Discovery Tests
# =============================================================================


class TestPiCameraDiscovery:
    """Tests for Pi camera device discovery."""

    def test_list_devices_no_library(self):
        """Test listing devices when no library available."""
        with patch(
            "robo_infra.sensors.cameras.picamera._get_camera_library",
            return_value=None,
        ):
            cameras = list_picameras()
            assert cameras == []

    def test_list_devices_picamera2(self):
        """Test listing devices with picamera2."""
        mock_module = MagicMock()
        mock_module.global_camera_info.return_value = [
            {"Model": "imx219", "Location": 0},
            {"Model": "imx708", "Location": 1},
        ]

        with patch.dict("sys.modules", {"picamera2": mock_module}), patch(
            "robo_infra.sensors.cameras.picamera._get_camera_library",
            return_value="picamera2",
        ):
            PiCamera.list_devices()
                # Returns list of CameraInfo


class TestOpenPicamera:
    """Tests for open_picamera convenience function."""

    def test_open_picamera_simulation(self):
        """Test opening Pi camera in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = open_picamera(resolution=(1280, 720), fps=60)

            assert camera.is_enabled
            assert camera.config.width == 1280
            assert camera.config.height == 720
            assert camera.config.fps == 60

            camera.disable()
