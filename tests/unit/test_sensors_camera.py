"""Unit tests for camera sensor abstractions.

Tests the camera protocol, Frame, DepthFrame, CameraIntrinsics,
SimulatedCamera, and USBCamera (with mocked OpenCV).
"""

from __future__ import annotations

import time
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from robo_infra.sensors.camera import (
    CameraConfig,
    CameraInfo,
    CameraIntrinsics,
    CameraState,
    DepthFrame,
    Frame,
    PixelFormat,
    SimulatedCamera,
)


# =============================================================================
# Frame Tests
# =============================================================================


class TestFrame:
    """Tests for Frame dataclass."""

    def test_creation(self) -> None:
        """Test frame creation with valid data."""
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
        )
        assert frame.width == 640
        assert frame.height == 480
        assert frame.format == PixelFormat.RGB
        assert frame.channels == 3

    def test_creation_grayscale(self) -> None:
        """Test frame creation with grayscale data."""
        data = np.zeros((480, 640), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.GRAY,
        )
        assert frame.channels == 1

    def test_invalid_dimensions(self) -> None:
        """Test frame creation with mismatched dimensions."""
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        with pytest.raises(ValueError, match="height"):
            Frame(
                data=data,
                timestamp=time.monotonic(),
                width=640,
                height=240,  # Wrong height
                format=PixelFormat.RGB,
            )

    def test_invalid_width(self) -> None:
        """Test frame creation with mismatched width."""
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        with pytest.raises(ValueError, match="width"):
            Frame(
                data=data,
                timestamp=time.monotonic(),
                width=320,  # Wrong width
                height=480,
                format=PixelFormat.RGB,
            )

    def test_size_property(self) -> None:
        """Test size property."""
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
        )
        assert frame.size == (640, 480)

    def test_shape_property(self) -> None:
        """Test shape property."""
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
        )
        assert frame.shape == (480, 640, 3)

    def test_to_rgb_from_rgb(self) -> None:
        """Test RGB to RGB conversion (no-op)."""
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
        )
        rgb_frame = frame.to_rgb()
        assert rgb_frame is frame  # Same object

    def test_to_rgb_from_bgr(self) -> None:
        """Test BGR to RGB conversion."""
        # Create BGR frame with known values
        data = np.zeros((2, 2, 3), dtype=np.uint8)
        data[0, 0] = [255, 0, 0]  # Blue in BGR
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            format=PixelFormat.BGR,
        )
        rgb_frame = frame.to_rgb()
        assert rgb_frame.format == PixelFormat.RGB
        # Blue (255,0,0) in BGR becomes (0,0,255) in RGB
        assert rgb_frame.data[0, 0, 0] == 0
        assert rgb_frame.data[0, 0, 2] == 255

    def test_to_rgb_from_gray(self) -> None:
        """Test grayscale to RGB conversion."""
        data = np.full((2, 2), 128, dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            format=PixelFormat.GRAY,
        )
        rgb_frame = frame.to_rgb()
        assert rgb_frame.format == PixelFormat.RGB
        assert rgb_frame.channels == 3
        assert rgb_frame.data[0, 0, 0] == 128
        assert rgb_frame.data[0, 0, 1] == 128
        assert rgb_frame.data[0, 0, 2] == 128

    def test_to_bgr_from_bgr(self) -> None:
        """Test BGR to BGR conversion (no-op)."""
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.BGR,
        )
        bgr_frame = frame.to_bgr()
        assert bgr_frame is frame

    def test_to_bgr_from_rgb(self) -> None:
        """Test RGB to BGR conversion."""
        data = np.zeros((2, 2, 3), dtype=np.uint8)
        data[0, 0] = [255, 0, 0]  # Red in RGB
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            format=PixelFormat.RGB,
        )
        bgr_frame = frame.to_bgr()
        assert bgr_frame.format == PixelFormat.BGR
        # Red (255,0,0) in RGB becomes (0,0,255) in BGR
        assert bgr_frame.data[0, 0, 0] == 0
        assert bgr_frame.data[0, 0, 2] == 255

    def test_to_grayscale(self) -> None:
        """Test RGB to grayscale conversion."""
        data = np.zeros((2, 2, 3), dtype=np.uint8)
        data[0, 0] = [255, 255, 255]  # White
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            format=PixelFormat.RGB,
        )
        gray_frame = frame.to_grayscale()
        assert gray_frame.format == PixelFormat.GRAY
        assert gray_frame.channels == 1
        assert gray_frame.data[0, 0] == 255  # White -> 255

    def test_to_grayscale_from_gray(self) -> None:
        """Test grayscale to grayscale (no-op)."""
        data = np.full((2, 2), 128, dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            format=PixelFormat.GRAY,
        )
        gray_frame = frame.to_grayscale()
        assert gray_frame is frame

    def test_crop(self) -> None:
        """Test frame cropping."""
        data = np.arange(100).reshape(10, 10).astype(np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=10,
            height=10,
            format=PixelFormat.GRAY,
        )
        cropped = frame.crop(2, 3, 5, 4)
        assert cropped.width == 5
        assert cropped.height == 4
        assert cropped.data[0, 0] == data[3, 2]

    def test_crop_out_of_bounds(self) -> None:
        """Test crop with out-of-bounds region."""
        data = np.zeros((10, 10), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=10,
            height=10,
            format=PixelFormat.GRAY,
        )
        with pytest.raises(ValueError, match="exceeds"):
            frame.crop(5, 5, 10, 10)

    def test_crop_negative_coords(self) -> None:
        """Test crop with negative coordinates."""
        data = np.zeros((10, 10), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=10,
            height=10,
            format=PixelFormat.GRAY,
        )
        with pytest.raises(ValueError, match="non-negative"):
            frame.crop(-1, 0, 5, 5)

    def test_copy(self) -> None:
        """Test frame copy."""
        data = np.zeros((10, 10, 3), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=1.0,
            width=10,
            height=10,
            format=PixelFormat.RGB,
            frame_number=5,
        )
        copied = frame.copy()
        assert copied.data is not frame.data
        assert copied.frame_number == frame.frame_number
        # Modify original, copy should be unchanged
        frame.data[0, 0, 0] = 255
        assert copied.data[0, 0, 0] == 0


# =============================================================================
# DepthFrame Tests
# =============================================================================


class TestDepthFrame:
    """Tests for DepthFrame dataclass."""

    def test_creation(self) -> None:
        """Test depth frame creation."""
        data = np.zeros((480, 640), dtype=np.uint16)
        depth = DepthFrame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            depth_scale=0.001,
            min_depth=0.3,
            max_depth=5.0,
        )
        assert depth.width == 640
        assert depth.height == 480
        assert depth.depth_scale == 0.001

    def test_to_meters(self) -> None:
        """Test conversion to meters."""
        data = np.array([[1000, 2000], [3000, 4000]], dtype=np.uint16)
        depth = DepthFrame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            depth_scale=0.001,  # mm to m
        )
        meters = depth.to_meters()
        assert meters[0, 0] == pytest.approx(1.0)
        assert meters[0, 1] == pytest.approx(2.0)
        assert meters[1, 1] == pytest.approx(4.0)

    def test_mask_valid(self) -> None:
        """Test valid depth mask."""
        data = np.array([[500, 0], [5000, 20000]], dtype=np.uint16)
        depth = DepthFrame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            depth_scale=0.001,
            min_depth=0.3,
            max_depth=10.0,
        )
        mask = depth.mask_valid()
        assert mask[0, 0]  # 0.5m is valid
        assert not mask[0, 1]  # 0m is invalid
        assert mask[1, 0]  # 5.0m is valid
        assert not mask[1, 1]  # 20m exceeds max

    def test_to_colormap(self) -> None:
        """Test depth visualization."""
        data = np.array([[1000, 5000], [3000, 10000]], dtype=np.uint16)
        depth = DepthFrame(
            data=data,
            timestamp=time.monotonic(),
            width=2,
            height=2,
            depth_scale=0.001,
            min_depth=0.0,
            max_depth=10.0,
        )
        colorized = depth.to_colormap()
        assert colorized.format == PixelFormat.RGB
        assert colorized.channels == 3
        assert colorized.width == 2
        assert colorized.height == 2


# =============================================================================
# CameraIntrinsics Tests
# =============================================================================


class TestCameraIntrinsics:
    """Tests for CameraIntrinsics dataclass."""

    def test_creation(self) -> None:
        """Test intrinsics creation."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        assert intrinsics.fx == 600.0
        assert intrinsics.fy == 600.0

    def test_invalid_focal_length(self) -> None:
        """Test creation with invalid focal length."""
        with pytest.raises(ValueError, match="positive"):
            CameraIntrinsics(
                fx=-100.0,
                fy=600.0,
                cx=320.0,
                cy=240.0,
                width=640,
                height=480,
            )

    def test_invalid_dimensions(self) -> None:
        """Test creation with invalid dimensions."""
        with pytest.raises(ValueError, match="positive"):
            CameraIntrinsics(
                fx=600.0,
                fy=600.0,
                cx=320.0,
                cy=240.0,
                width=0,
                height=480,
            )

    def test_camera_matrix(self) -> None:
        """Test camera matrix property."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        K = intrinsics.camera_matrix
        assert K.shape == (3, 3)
        assert K[0, 0] == 600.0  # fx
        assert K[1, 1] == 600.0  # fy
        assert K[0, 2] == 320.0  # cx
        assert K[1, 2] == 240.0  # cy
        assert K[2, 2] == 1.0

    def test_project(self) -> None:
        """Test 3D to 2D projection."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        # Project point at (0, 0, 1) - should map to principal point
        u, v = intrinsics.project((0.0, 0.0, 1.0))
        assert u == pytest.approx(320.0)
        assert v == pytest.approx(240.0)

    def test_project_offset(self) -> None:
        """Test projection with offset."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        # Point 1m to the right at 1m depth
        u, v = intrinsics.project((1.0, 0.0, 1.0))
        assert u == pytest.approx(920.0)  # 320 + 600*1
        assert v == pytest.approx(240.0)

    def test_project_behind_camera(self) -> None:
        """Test projection of point behind camera."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        with pytest.raises(ValueError, match="front"):
            intrinsics.project((0.0, 0.0, -1.0))

    def test_unproject(self) -> None:
        """Test 2D to 3D unprojection."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        # Unproject principal point at depth 2m
        x, y, z = intrinsics.unproject((320.0, 240.0), 2.0)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(2.0)

    def test_project_unproject_roundtrip(self) -> None:
        """Test project then unproject returns original point."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        original = (0.5, 0.3, 2.0)
        u, v = intrinsics.project(original)
        recovered = intrinsics.unproject((u, v), original[2])
        assert recovered[0] == pytest.approx(original[0])
        assert recovered[1] == pytest.approx(original[1])
        assert recovered[2] == pytest.approx(original[2])

    def test_pixel_to_ray(self) -> None:
        """Test pixel to ray conversion."""
        intrinsics = CameraIntrinsics(
            fx=600.0,
            fy=600.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        origin, direction = intrinsics.pixel_to_ray((320.0, 240.0))
        assert origin == (0.0, 0.0, 0.0)
        assert direction[2] == pytest.approx(1.0, abs=0.01)  # Points forward

    def test_default(self) -> None:
        """Test default intrinsics creation."""
        intrinsics = CameraIntrinsics.default(640, 480)
        assert intrinsics.width == 640
        assert intrinsics.height == 480
        assert intrinsics.cx == pytest.approx(320.0)
        assert intrinsics.cy == pytest.approx(240.0)
        assert intrinsics.fx > 0
        assert intrinsics.fy > 0


# =============================================================================
# CameraConfig Tests
# =============================================================================


class TestCameraConfig:
    """Tests for CameraConfig Pydantic model."""

    def test_defaults(self) -> None:
        """Test default configuration values."""
        config = CameraConfig()
        assert config.width == 640
        assert config.height == 480
        assert config.fps == 30
        assert config.format == PixelFormat.RGB
        assert config.auto_exposure is True

    def test_custom_values(self) -> None:
        """Test custom configuration values."""
        config = CameraConfig(
            width=1920,
            height=1080,
            fps=60,
            format=PixelFormat.BGR,
        )
        assert config.width == 1920
        assert config.height == 1080
        assert config.fps == 60
        assert config.format == PixelFormat.BGR

    def test_resolution_property(self) -> None:
        """Test resolution property."""
        config = CameraConfig(width=1280, height=720)
        assert config.resolution == (1280, 720)

    def test_invalid_fps(self) -> None:
        """Test invalid FPS value."""
        with pytest.raises(ValueError):
            CameraConfig(fps=0)

    def test_invalid_dimensions(self) -> None:
        """Test invalid dimensions."""
        with pytest.raises(ValueError):
            CameraConfig(width=0)


# =============================================================================
# SimulatedCamera Tests
# =============================================================================


class TestSimulatedCamera:
    """Tests for SimulatedCamera."""

    def test_creation(self) -> None:
        """Test simulated camera creation."""
        camera = SimulatedCamera()
        assert camera.name == "simulated_camera"
        assert not camera.is_enabled
        assert camera.state == CameraState.DISCONNECTED

    def test_enable_disable(self) -> None:
        """Test enable and disable."""
        camera = SimulatedCamera()
        camera.enable()
        assert camera.is_enabled
        assert camera.state == CameraState.CONNECTED

        camera.disable()
        assert not camera.is_enabled
        assert camera.state == CameraState.DISCONNECTED

    def test_context_manager(self) -> None:
        """Test context manager usage."""
        with SimulatedCamera() as camera:
            assert camera.is_enabled
        assert not camera.is_enabled

    def test_capture_requires_enable(self) -> None:
        """Test capture fails if not enabled."""
        camera = SimulatedCamera()
        with pytest.raises(RuntimeError, match="enabled"):
            camera.capture()

    def test_capture(self) -> None:
        """Test frame capture."""
        camera = SimulatedCamera()
        camera.enable()
        frame = camera.capture()

        assert frame.width == 640
        assert frame.height == 480
        assert frame.format == PixelFormat.RGB
        assert frame.frame_number == 1

        # Second capture increments frame number
        frame2 = camera.capture()
        assert frame2.frame_number == 2

        camera.disable()

    def test_capture_with_config(self) -> None:
        """Test capture with custom config."""
        config = CameraConfig(width=320, height=240)
        camera = SimulatedCamera(config=config)
        camera.enable()
        frame = camera.capture()

        assert frame.width == 320
        assert frame.height == 240

        camera.disable()

    def test_pattern_solid(self) -> None:
        """Test solid color pattern."""
        camera = SimulatedCamera(pattern="solid", color=(255, 0, 0))
        camera.enable()
        frame = camera.capture()

        # Check most pixels are the solid color
        # (bottom bar may be different)
        assert frame.data[0, 0, 0] == 255
        assert frame.data[0, 0, 1] == 0
        assert frame.data[0, 0, 2] == 0

        camera.disable()

    def test_pattern_gradient(self) -> None:
        """Test gradient pattern."""
        camera = SimulatedCamera(pattern="gradient")
        camera.enable()
        frame = camera.capture()

        # Left side should be darker than right
        left_val = frame.data[0, 0, 0]
        right_val = frame.data[0, -1, 0]
        assert left_val < right_val

        camera.disable()

    def test_pattern_checkerboard(self) -> None:
        """Test checkerboard pattern."""
        camera = SimulatedCamera(pattern="checkerboard")
        camera.enable()
        frame = camera.capture()

        # Should have distinct values (0 and 255)
        unique_values = np.unique(frame.data[:-5])  # Exclude timestamp bar
        assert len(unique_values) >= 2

        camera.disable()

    def test_pattern_noise(self) -> None:
        """Test noise pattern."""
        camera = SimulatedCamera(pattern="noise")
        camera.enable()
        frame1 = camera.capture()
        frame2 = camera.capture()

        # Frames should be different (random noise)
        assert not np.array_equal(frame1.data, frame2.data)

        camera.disable()

    def test_list_devices(self) -> None:
        """Test device listing."""
        devices = SimulatedCamera.list_devices()
        assert len(devices) == 1
        assert devices[0].name == "Simulated Camera"

    def test_get_intrinsics(self) -> None:
        """Test getting intrinsics."""
        camera = SimulatedCamera()
        intrinsics = camera.get_intrinsics()
        assert intrinsics.width == 640
        assert intrinsics.height == 480

    def test_set_intrinsics(self) -> None:
        """Test setting custom intrinsics."""
        camera = SimulatedCamera()
        custom = CameraIntrinsics(
            fx=500.0,
            fy=500.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
        )
        camera.set_intrinsics(custom)
        assert camera.get_intrinsics().fx == 500.0

    def test_repr(self) -> None:
        """Test string representation."""
        camera = SimulatedCamera()
        repr_str = repr(camera)
        assert "SimulatedCamera" in repr_str
        assert "640x480" in repr_str

    @pytest.mark.asyncio
    async def test_capture_async(self) -> None:
        """Test async capture."""
        camera = SimulatedCamera()
        camera.enable()
        frame = await camera.capture_async()
        assert frame.width == 640
        camera.disable()

    @pytest.mark.asyncio
    async def test_stream(self) -> None:
        """Test frame streaming."""
        camera = SimulatedCamera()
        camera.enable()

        frames = []
        async for frame in camera.stream(max_frames=3):
            frames.append(frame)

        assert len(frames) == 3
        assert frames[0].frame_number == 1
        assert frames[2].frame_number == 3

        camera.disable()

    @pytest.mark.asyncio
    async def test_stream_timeout(self) -> None:
        """Test stream with timeout."""
        config = CameraConfig(fps=100)  # Fast FPS
        camera = SimulatedCamera(config=config)
        camera.enable()

        frames = []
        async for frame in camera.stream(timeout=0.1):
            frames.append(frame)

        assert len(frames) > 0
        camera.disable()

    def test_configure(self) -> None:
        """Test reconfiguring camera."""
        camera = SimulatedCamera()
        camera.enable()

        new_config = CameraConfig(width=320, height=240)
        camera.configure(new_config)

        frame = camera.capture()
        assert frame.width == 320
        assert frame.height == 240

        camera.disable()


# =============================================================================
# CameraInfo Tests
# =============================================================================


class TestCameraInfo:
    """Tests for CameraInfo dataclass."""

    def test_creation(self) -> None:
        """Test camera info creation."""
        info = CameraInfo(
            device_id=0,
            name="Test Camera",
            driver="test",
        )
        assert info.device_id == 0
        assert info.name == "Test Camera"
        assert info.is_available is True

    def test_with_details(self) -> None:
        """Test camera info with full details."""
        info = CameraInfo(
            device_id="/dev/video0",
            name="USB Camera",
            driver="v4l2",
            supported_resolutions=[(640, 480), (1280, 720)],
            supported_formats=[PixelFormat.RGB, PixelFormat.BGR],
        )
        assert len(info.supported_resolutions) == 2
        assert len(info.supported_formats) == 2


# =============================================================================
# PixelFormat Tests
# =============================================================================


class TestPixelFormat:
    """Tests for PixelFormat enum."""

    def test_values(self) -> None:
        """Test enum values."""
        assert PixelFormat.RGB.value == "RGB"
        assert PixelFormat.BGR.value == "BGR"
        assert PixelFormat.GRAY.value == "GRAY"
        assert PixelFormat.JPEG.value == "JPEG"


# =============================================================================
# USB Camera Tests (with mocked OpenCV)
# =============================================================================


class TestUSBCamera:
    """Tests for USBCamera with mocked OpenCV."""

    @pytest.fixture
    def mock_cv2(self) -> Any:
        """Create mock cv2 module."""
        mock = MagicMock()
        mock.VideoCapture.return_value = MagicMock()
        mock.VideoCapture.return_value.isOpened.return_value = True
        mock.VideoCapture.return_value.get.return_value = 640
        mock.CAP_PROP_FRAME_WIDTH = 3
        mock.CAP_PROP_FRAME_HEIGHT = 4
        mock.CAP_PROP_FPS = 5
        mock.CAP_PROP_BUFFERSIZE = 38
        mock.CAP_PROP_BACKEND = 200
        mock.CAP_PROP_AUTO_EXPOSURE = 21
        mock.CAP_PROP_EXPOSURE = 15
        mock.CAP_PROP_GAIN = 14
        mock.CAP_V4L2 = 200
        mock.CAP_DSHOW = 700
        mock.CAP_AVFOUNDATION = 1200
        mock.COLOR_BGR2RGB = 4
        mock.COLOR_BGR2GRAY = 6
        mock.videoio_registry.getBackendName.return_value = "V4L2"
        return mock

    def test_creation(self, mock_cv2: Any) -> None:
        """Test USB camera creation."""
        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera(device_id=0)
            assert camera.device_id == 0
            assert not camera.is_enabled

    def test_enable(self, mock_cv2: Any) -> None:
        """Test camera enable."""
        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera(device_id=0)
            camera.enable()

            assert camera.is_enabled
            mock_cv2.VideoCapture.assert_called_once()

    def test_disable(self, mock_cv2: Any) -> None:
        """Test camera disable."""
        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera(device_id=0)
            camera.enable()
            camera.disable()

            assert not camera.is_enabled
            mock_cv2.VideoCapture.return_value.release.assert_called_once()

    def test_capture(self, mock_cv2: Any) -> None:
        """Test frame capture."""
        mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mock_cv2.VideoCapture.return_value.read.return_value = (True, mock_frame)
        mock_cv2.cvtColor.return_value = mock_frame

        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera(device_id=0)
            camera.enable()
            frame = camera.capture()

            assert frame.width == 640
            assert frame.height == 480

    def test_capture_failed(self, mock_cv2: Any) -> None:
        """Test capture failure handling."""
        mock_cv2.VideoCapture.return_value.read.return_value = (False, None)

        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera(device_id=0)
            camera.enable()

            with pytest.raises(RuntimeError, match="Failed to capture"):
                camera.capture()

    def test_open_failed(self, mock_cv2: Any) -> None:
        """Test connection failure handling."""
        mock_cv2.VideoCapture.return_value.isOpened.return_value = False

        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera(device_id=99)

            with pytest.raises(RuntimeError, match="Failed to open"):
                camera.enable()

    def test_list_devices(self, mock_cv2: Any) -> None:
        """Test device listing."""
        # First call succeeds, second fails
        mock_cv2.VideoCapture.return_value.isOpened.side_effect = [True, False]

        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            devices = USBCamera.list_devices(max_devices=2)
            assert len(devices) == 1

    def test_auto_detect(self, mock_cv2: Any) -> None:
        """Test auto-detection."""
        # Reset the side_effect for this test
        mock_cv2.VideoCapture.return_value.isOpened.reset_mock()
        mock_cv2.VideoCapture.return_value.isOpened.side_effect = [True, False, False, False, False]
        mock_cv2.VideoCapture.return_value.get.return_value = 640

        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera.auto_detect()
            assert camera is not None
            assert camera.device_id == 0

    def test_auto_detect_no_camera(self, mock_cv2: Any) -> None:
        """Test auto-detection with no cameras."""
        mock_cv2.VideoCapture.return_value.isOpened.return_value = False

        with patch.dict("sys.modules", {"cv2": mock_cv2}):
            from robo_infra.sensors.cameras.usb import USBCamera

            camera = USBCamera.auto_detect()
            assert camera is None
