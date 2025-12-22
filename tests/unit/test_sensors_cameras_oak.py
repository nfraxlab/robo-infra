"""Unit tests for Luxonis OAK camera implementation.

Tests OAKCamera class with mocked depthai library.
"""

from __future__ import annotations

import os
from unittest.mock import patch

import pytest

from robo_infra.sensors.camera import (
    CameraIntrinsics,
    CameraState,
    DepthFrame,
    Frame,
    PixelFormat,
)
from robo_infra.sensors.cameras.oak import (
    Detection,
    OAKCamera,
    OAKConfig,
    TrackedObject,
    list_oak_cameras,
    open_oak,
)


# =============================================================================
# Detection Tests
# =============================================================================


class TestDetection:
    """Tests for Detection dataclass."""

    @pytest.fixture
    def sample_detection(self):
        """Create a sample detection."""
        return Detection(
            label=0,
            label_name="person",
            confidence=0.95,
            bbox=(0.1, 0.2, 0.4, 0.6),
            depth=2.5,
            spatial_coords=(0.5, -0.3, 2.5),
        )

    def test_create_detection(self, sample_detection):
        """Test creating a detection."""
        assert sample_detection.label == 0
        assert sample_detection.label_name == "person"
        assert sample_detection.confidence == 0.95
        assert sample_detection.bbox == (0.1, 0.2, 0.4, 0.6)
        assert sample_detection.depth == 2.5
        assert sample_detection.spatial_coords == (0.5, -0.3, 2.5)

    def test_detection_center(self, sample_detection):
        """Test detection center calculation."""
        center = sample_detection.center
        assert center == (0.25, 0.4)  # ((0.1+0.4)/2, (0.2+0.6)/2)

    def test_detection_width(self, sample_detection):
        """Test detection width calculation."""
        assert sample_detection.width == pytest.approx(0.3, rel=1e-5)

    def test_detection_height(self, sample_detection):
        """Test detection height calculation."""
        assert sample_detection.height == pytest.approx(0.4, rel=1e-5)

    def test_detection_area(self, sample_detection):
        """Test detection area calculation."""
        assert sample_detection.area == pytest.approx(0.12, rel=1e-5)  # 0.3 * 0.4

    def test_to_pixels(self, sample_detection):
        """Test converting to pixel coordinates."""
        x_min, y_min, x_max, y_max = sample_detection.to_pixels(640, 480)

        assert x_min == int(0.1 * 640)  # 64
        assert y_min == int(0.2 * 480)  # 96
        assert x_max == int(0.4 * 640)  # 256
        assert y_max == int(0.6 * 480)  # 288

    def test_detection_no_spatial(self):
        """Test detection without spatial coordinates."""
        det = Detection(
            label=1,
            confidence=0.8,
            bbox=(0.2, 0.3, 0.5, 0.7),
        )
        assert det.depth is None
        assert det.spatial_coords is None
        assert det.label_name == ""


# =============================================================================
# TrackedObject Tests
# =============================================================================


class TestTrackedObject:
    """Tests for TrackedObject dataclass."""

    def test_create_tracked_object(self):
        """Test creating a tracked object."""
        obj = TrackedObject(
            label=0,
            label_name="person",
            confidence=0.9,
            bbox=(0.1, 0.2, 0.3, 0.4),
            track_id=42,
            age=10,
            status="tracked",
            velocity=(0.1, 0.0, 0.05),
        )

        assert obj.track_id == 42
        assert obj.age == 10
        assert obj.status == "tracked"
        assert obj.velocity == (0.1, 0.0, 0.05)

    def test_tracked_object_defaults(self):
        """Test tracked object default values."""
        obj = TrackedObject(
            label=1,
            confidence=0.85,
            bbox=(0.0, 0.0, 0.5, 0.5),
        )

        assert obj.track_id == -1
        assert obj.age == 0
        assert obj.status == "new"
        assert obj.velocity is None


# =============================================================================
# OAKConfig Tests
# =============================================================================


class TestOAKConfig:
    """Tests for OAKConfig."""

    def test_default_config(self):
        """Test default OAKConfig values."""
        config = OAKConfig()

        assert config.width == 640
        assert config.height == 480
        assert config.fps == 30
        assert config.enable_depth is True
        assert config.depth_median_filter == "kernel_7x7"
        assert config.depth_lrc_check is True
        assert config.nn_confidence_threshold == 0.5
        assert config.nn_spatial_detection is True
        assert config.usb_speed == "usb3"

    def test_custom_config(self):
        """Test custom OAKConfig."""
        config = OAKConfig(
            width=1280,
            height=720,
            fps=60,
            enable_depth=False,
            depth_median_filter="kernel_5x5",
            enable_ir_projector=True,
            ir_projector_brightness=0.5,
            nn_blob_path="/path/to/model.blob",
            nn_confidence_threshold=0.7,
        )

        assert config.width == 1280
        assert config.height == 720
        assert config.fps == 60
        assert config.enable_depth is False
        assert config.depth_median_filter == "kernel_5x5"
        assert config.enable_ir_projector is True
        assert config.ir_projector_brightness == 0.5
        assert config.nn_blob_path == "/path/to/model.blob"
        assert config.nn_confidence_threshold == 0.7


# =============================================================================
# OAKCamera Tests (Simulated Mode)
# =============================================================================


class TestOAKCameraSimulated:
    """Tests for OAKCamera in simulation mode."""

    @pytest.fixture
    def simulated_camera(self):
        """Create a simulated OAK camera."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = OAKCamera(device_id=None)
            yield camera
            if camera.is_enabled:
                camera.disable()

    def test_create_camera(self, simulated_camera):
        """Test camera creation."""
        assert "oak" in simulated_camera.name
        assert simulated_camera.state == CameraState.DISCONNECTED

    def test_enable_disable(self, simulated_camera):
        """Test enabling and disabling camera."""
        simulated_camera.enable()
        assert simulated_camera.is_enabled
        assert simulated_camera.state == CameraState.CONNECTED
        assert simulated_camera.device_name == "Simulated OAK-D"
        assert simulated_camera.has_depth is True

        simulated_camera.disable()
        assert not simulated_camera.is_enabled
        assert simulated_camera.state == CameraState.DISCONNECTED

    def test_capture_rgb(self, simulated_camera):
        """Test capturing RGB frame."""
        simulated_camera.enable()
        frame = simulated_camera.capture()

        assert isinstance(frame, Frame)
        assert frame.width == 640
        assert frame.height == 480
        assert frame.format == PixelFormat.RGB
        assert frame.data.shape == (480, 640, 3)

    def test_capture_stereo(self, simulated_camera):
        """Test capturing stereo frames."""
        simulated_camera.enable()
        left, right, depth = simulated_camera.capture_stereo()

        assert isinstance(left, Frame)
        assert isinstance(right, Frame)
        assert isinstance(depth, DepthFrame)

    def test_context_manager(self):
        """Test camera as context manager."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            with OAKCamera() as camera:
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
# Neural Network Tests (Simulated)
# =============================================================================


class TestOAKNeuralNetworkSimulated:
    """Tests for OAK neural network inference in simulation mode."""

    @pytest.fixture
    def simulated_camera_with_nn(self):
        """Create a simulated OAK camera with NN loaded."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = OAKCamera()
            camera.enable()
            camera._nn_loaded = True  # Simulate loaded network
            yield camera
            camera.disable()

    def test_detect_simulated(self, simulated_camera_with_nn):
        """Test simulated detection."""
        detections = simulated_camera_with_nn.detect()

        assert isinstance(detections, list)
        # Simulated may return 0-3 detections
        for det in detections:
            assert isinstance(det, Detection)
            assert 0 <= det.confidence <= 1
            assert det.label >= 0
            assert len(det.bbox) == 4

    def test_run_neural_network_no_model(self):
        """Test error when no model loaded."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = OAKCamera()
            camera.enable()

            with pytest.raises(RuntimeError, match="No neural network loaded"):
                camera.run_neural_network()

            camera.disable()

    def test_load_neural_network_file_not_found(self):
        """Test error when blob file doesn't exist."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = OAKCamera()
            camera.enable()

            with pytest.raises(FileNotFoundError, match="Blob file not found"):
                camera.load_neural_network("/nonexistent/model.blob")

            camera.disable()

    def test_detection_with_spatial_coords(self, simulated_camera_with_nn):
        """Test that simulated detections have spatial coordinates."""
        # Run multiple times to get some detections
        all_detections = []
        for _ in range(10):
            detections = simulated_camera_with_nn.detect()
            all_detections.extend(detections)

        # Check that at least some detections have spatial coords
        spatial_detections = [d for d in all_detections if d.spatial_coords is not None]
        assert len(spatial_detections) > 0

        for det in spatial_detections:
            assert len(det.spatial_coords) == 3
            assert det.depth is not None


# =============================================================================
# COCO Classes Tests
# =============================================================================


class TestCOCOClasses:
    """Tests for COCO class labels."""

    def test_coco_classes_count(self):
        """Test COCO classes list has correct count."""
        assert len(OAKCamera.COCO_CLASSES) == 80

    def test_coco_classes_contents(self):
        """Test COCO classes contains expected labels."""
        classes = OAKCamera.COCO_CLASSES
        assert "person" in classes
        assert "car" in classes
        assert "dog" in classes
        assert "cat" in classes
        assert "laptop" in classes


# =============================================================================
# Device Discovery Tests
# =============================================================================


class TestOAKDiscovery:
    """Tests for OAK device discovery."""

    def test_list_devices_no_library(self):
        """Test listing devices when library not installed."""
        with (
            patch.dict("sys.modules", {"depthai": None}),
            patch("builtins.__import__", side_effect=ImportError),
        ):
            OAKCamera.list_devices()
            # Should return empty list, not raise

    def test_is_available_no_library(self):
        """Test availability check when library not installed."""
        with (
            patch.dict("sys.modules", {"depthai": None}),
            patch("builtins.__import__", side_effect=ImportError),
        ):
            result = OAKCamera.is_available()
            assert result is False


# =============================================================================
# Convenience Function Tests
# =============================================================================


class TestOpenOak:
    """Tests for open_oak convenience function."""

    def test_open_oak_simulation(self):
        """Test opening OAK in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = open_oak(
                resolution=(1280, 720),
                fps=60,
                enable_depth=True,
            )

            assert camera.is_enabled
            assert camera.config.width == 1280
            assert camera.config.height == 720
            assert camera.config.fps == 60
            assert isinstance(camera.config, OAKConfig)
            assert camera.config.enable_depth is True

            camera.disable()

    def test_open_oak_no_depth(self):
        """Test opening OAK without depth."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = open_oak(enable_depth=False)

            assert camera.is_enabled
            assert camera.config.enable_depth is False

            camera.disable()


# =============================================================================
# list_oak_cameras Tests
# =============================================================================


class TestListOakCameras:
    """Tests for list_oak_cameras function."""

    def test_list_oak_cameras_empty(self):
        """Test listing cameras returns list."""
        with patch.object(OAKCamera, "list_devices", return_value=[]):
            cameras = list_oak_cameras()
            assert cameras == []
