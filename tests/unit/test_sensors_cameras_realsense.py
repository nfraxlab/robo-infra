"""Unit tests for Intel RealSense camera implementation.

Tests RealSenseCamera class with mocked pyrealsense2 library.
"""

from __future__ import annotations

import os
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from robo_infra.sensors.camera import (
    CameraIntrinsics,
    CameraState,
    DepthFrame,
    Frame,
    PixelFormat,
)
from robo_infra.sensors.cameras.realsense import (
    PointCloud,
    RealSenseCamera,
    RealSenseConfig,
    list_realsense_cameras,
    open_realsense,
)


# =============================================================================
# PointCloud Tests
# =============================================================================


class TestPointCloud:
    """Tests for PointCloud dataclass."""

    @pytest.fixture
    def sample_pointcloud(self):
        """Create a sample point cloud."""
        points = np.random.randn(1000, 3).astype(np.float32)
        colors = np.random.randint(0, 255, (1000, 3), dtype=np.uint8)
        return PointCloud(
            points=points,
            colors=colors,
            timestamp=time.monotonic(),
            frame_number=1,
        )

    def test_create_pointcloud(self, sample_pointcloud):
        """Test creating a point cloud."""
        assert sample_pointcloud.num_points == 1000
        assert sample_pointcloud.points.shape == (1000, 3)
        assert sample_pointcloud.colors.shape == (1000, 3)

    def test_pointcloud_no_colors(self):
        """Test point cloud without colors."""
        points = np.random.randn(500, 3).astype(np.float32)
        pc = PointCloud(
            points=points,
            colors=None,
            timestamp=time.monotonic(),
        )
        assert pc.num_points == 500
        assert pc.colors is None

    def test_filter_by_distance(self, sample_pointcloud):
        """Test filtering point cloud by distance."""
        # Some points will be within range, some won't
        filtered = sample_pointcloud.filter_by_distance(
            min_distance=0.0, max_distance=2.0
        )

        assert filtered.num_points <= sample_pointcloud.num_points

        # Check all remaining points are within range
        distances = np.linalg.norm(filtered.points, axis=1)
        assert np.all(distances <= 2.0)

    def test_downsample(self, sample_pointcloud):
        """Test voxel downsampling."""
        downsampled = sample_pointcloud.downsample(voxel_size=0.5)

        # Should have fewer or equal points
        assert downsampled.num_points <= sample_pointcloud.num_points

    def test_to_open3d_import_error(self, sample_pointcloud):
        """Test Open3D conversion when not installed."""
        with (
            patch.dict("sys.modules", {"open3d": None}),
            pytest.raises(ImportError, match="Open3D required"),
        ):
            sample_pointcloud.to_open3d()


# =============================================================================
# RealSenseConfig Tests
# =============================================================================


class TestRealSenseConfig:
    """Tests for RealSenseConfig."""

    def test_default_config(self):
        """Test default RealSenseConfig values."""
        config = RealSenseConfig()

        assert config.width == 640
        assert config.height == 480
        assert config.fps == 30
        assert config.depth_width == 640
        assert config.depth_height == 480
        assert config.depth_fps == 30
        assert config.depth_preset == "high_accuracy"
        assert config.laser_power == 150
        assert config.emitter_enabled is True
        assert config.align_depth_to_color is True
        assert config.spatial_filter is True
        assert config.temporal_filter is True

    def test_custom_config(self):
        """Test custom RealSenseConfig."""
        config = RealSenseConfig(
            width=1280,
            height=720,
            depth_width=848,
            depth_height=480,
            depth_preset="high_density",
            laser_power=200,
            decimation_filter=True,
            hole_filling=True,
        )

        assert config.width == 1280
        assert config.height == 720
        assert config.depth_width == 848
        assert config.depth_preset == "high_density"
        assert config.laser_power == 200
        assert config.decimation_filter is True
        assert config.hole_filling is True


# =============================================================================
# RealSenseCamera Tests (Simulated Mode)
# =============================================================================


class TestRealSenseCameraSimulated:
    """Tests for RealSenseCamera in simulation mode."""

    @pytest.fixture
    def simulated_camera(self):
        """Create a simulated RealSense camera."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = RealSenseCamera(serial_number=None)
            yield camera
            if camera.is_enabled:
                camera.disable()

    def test_create_camera(self, simulated_camera):
        """Test camera creation."""
        assert "realsense" in simulated_camera.name
        assert simulated_camera.state == CameraState.DISCONNECTED

    def test_enable_disable(self, simulated_camera):
        """Test enabling and disabling camera."""
        simulated_camera.enable()
        assert simulated_camera.is_enabled
        assert simulated_camera.state == CameraState.CONNECTED
        assert simulated_camera.device_name == "Simulated RealSense D435"

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

    def test_capture_depth(self, simulated_camera):
        """Test capturing depth frame."""
        simulated_camera.enable()
        depth = simulated_camera.capture_depth()

        assert isinstance(depth, DepthFrame)
        assert depth.width == 640
        assert depth.height == 480
        assert depth.depth_scale == 0.001
        assert depth.data.dtype == np.uint16

    def test_capture_rgbd(self, simulated_camera):
        """Test capturing aligned RGB-D."""
        simulated_camera.enable()
        rgb, depth = simulated_camera.capture_rgbd()

        assert isinstance(rgb, Frame)
        assert isinstance(depth, DepthFrame)
        assert rgb.format == PixelFormat.RGB
        assert depth.depth_scale == 0.001

    def test_get_pointcloud(self, simulated_camera):
        """Test generating point cloud."""
        simulated_camera.enable()
        pc = simulated_camera.get_pointcloud()

        assert isinstance(pc, PointCloud)
        assert pc.num_points > 0
        assert pc.points.shape[1] == 3  # XYZ

    def test_context_manager(self):
        """Test camera as context manager."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            with RealSenseCamera() as camera:
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

    def test_depth_intrinsics(self, simulated_camera):
        """Test getting depth camera intrinsics."""
        simulated_camera.enable()
        intrinsics = simulated_camera.get_depth_intrinsics()

        assert isinstance(intrinsics, CameraIntrinsics)
        assert intrinsics.fx > 0


# =============================================================================
# RealSenseCamera Tests (Mocked pyrealsense2)
# =============================================================================


class TestRealSenseCameraMocked:
    """Tests for RealSenseCamera with mocked pyrealsense2 library."""

    @pytest.fixture
    def mock_rs(self):
        """Create mock pyrealsense2 module."""
        mock_module = MagicMock()

        # Mock camera info enum
        mock_module.camera_info = MagicMock()
        mock_module.camera_info.name = "name"
        mock_module.camera_info.serial_number = "serial"
        mock_module.camera_info.firmware_version = "firmware"

        # Mock stream types
        mock_module.stream = MagicMock()
        mock_module.stream.color = 1
        mock_module.stream.depth = 2

        # Mock format
        mock_module.format = MagicMock()
        mock_module.format.rgb8 = 1
        mock_module.format.z16 = 2

        # Mock option
        mock_module.option = MagicMock()

        # Mock device
        mock_device = MagicMock()
        mock_device.get_info.side_effect = lambda x: {
            "name": "Intel RealSense D435",
            "serial": "123456789",
            "firmware": "5.12.0",
        }.get(str(x), "unknown")

        # Mock depth sensor
        mock_depth_sensor = MagicMock()
        mock_depth_sensor.get_depth_scale.return_value = 0.001
        mock_depth_sensor.supports.return_value = True
        mock_device.first_depth_sensor.return_value = mock_depth_sensor

        # Mock pipeline
        mock_pipeline = MagicMock()
        mock_profile = MagicMock()
        mock_profile.get_device.return_value = mock_device
        mock_profile.get_streams.return_value = []
        mock_pipeline.start.return_value = mock_profile
        mock_module.pipeline.return_value = mock_pipeline

        # Mock config
        mock_module.config.return_value = MagicMock()

        # Mock frames
        mock_frameset = MagicMock()
        mock_color_frame = MagicMock()
        mock_color_frame.get_data.return_value = np.zeros(
            (480, 640, 3), dtype=np.uint8
        )
        mock_depth_frame = MagicMock()
        mock_depth_frame.get_data.return_value = np.zeros(
            (480, 640), dtype=np.uint16
        )
        mock_frameset.get_color_frame.return_value = mock_color_frame
        mock_frameset.get_depth_frame.return_value = mock_depth_frame
        mock_pipeline.wait_for_frames.return_value = mock_frameset

        # Mock align
        mock_align = MagicMock()
        mock_align.process.return_value = mock_frameset
        mock_module.align.return_value = mock_align

        # Mock pointcloud
        mock_pc = MagicMock()
        mock_points = MagicMock()
        mock_points.get_vertices.return_value = np.zeros(
            (1000, 3), dtype=np.float32
        ).tobytes()
        mock_points.get_texture_coordinates.return_value = np.zeros(
            (1000, 2), dtype=np.float32
        ).tobytes()
        mock_pc.calculate.return_value = mock_points
        mock_module.pointcloud.return_value = mock_pc

        # Mock context for device listing
        mock_ctx = MagicMock()
        mock_ctx.query_devices.return_value = []
        mock_module.context.return_value = mock_ctx

        return mock_module, mock_pipeline

    def test_list_devices_empty(self):
        """Test listing devices when none available."""
        with patch(
            "robo_infra.sensors.cameras.realsense.RealSenseCamera.list_devices",
            return_value=[],
        ):
            cameras = list_realsense_cameras()
            assert cameras == []


# =============================================================================
# Device Discovery Tests
# =============================================================================


class TestRealSenseDiscovery:
    """Tests for RealSense device discovery."""

    def test_is_available_no_library(self):
        """Test availability check when library not installed."""
        with (
            patch.dict("sys.modules", {"pyrealsense2": None}),
            patch("builtins.__import__", side_effect=ImportError),
        ):
            result = RealSenseCamera.is_available()
            assert result is False


# =============================================================================
# Convenience Function Tests
# =============================================================================


class TestOpenRealsense:
    """Tests for open_realsense convenience function."""

    def test_open_realsense_simulation(self):
        """Test opening RealSense in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            camera = open_realsense(
                color_resolution=(1280, 720),
                depth_resolution=(640, 480),
                fps=30,
            )

            assert camera.is_enabled
            assert camera.config.width == 1280
            assert camera.config.height == 720
            assert isinstance(camera.config, RealSenseConfig)
            assert camera.config.depth_width == 640
            assert camera.config.depth_height == 480

            camera.disable()


# =============================================================================
# Depth Processing Tests
# =============================================================================


class TestDepthProcessing:
    """Tests for depth frame processing."""

    @pytest.fixture
    def depth_frame(self):
        """Create a sample depth frame."""
        data = np.random.randint(300, 10000, (480, 640), dtype=np.uint16)
        return DepthFrame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            depth_scale=0.001,
            min_depth=0.3,
            max_depth=10.0,
        )

    def test_to_meters(self, depth_frame):
        """Test depth to meters conversion."""
        meters = depth_frame.to_meters()

        assert meters.dtype == np.float32
        assert meters.shape == (480, 640)
        # Values should be in reasonable range
        assert np.min(meters) >= 0.0
        assert np.max(meters) <= 65.535  # 65535 * 0.001

    def test_mask_valid(self, depth_frame):
        """Test valid depth masking."""
        mask = depth_frame.mask_valid()

        assert mask.dtype == np.bool_
        assert mask.shape == (480, 640)

    def test_to_colormap(self, depth_frame):
        """Test depth to colormap conversion."""
        colorized = depth_frame.to_colormap()

        assert isinstance(colorized, Frame)
        assert colorized.format == PixelFormat.RGB
        assert colorized.data.shape == (480, 640, 3)
        assert colorized.data.dtype == np.uint8
