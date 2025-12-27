"""Intel RealSense depth camera implementation.

This module provides support for Intel RealSense depth cameras using
the pyrealsense2 library.

Supported Hardware:
    - D415, D435, D435i (stereo depth)
    - D455, D455f (wide FOV depth)
    - L515 (LiDAR depth)
    - D405 (short-range depth)
    - T265 (tracking camera)

Requirements:
    - Intel RealSense SDK 2.0
    - pyrealsense2 library

Example:
    >>> from robo_infra.sensors.cameras.realsense import RealSenseCamera
    >>>
    >>> # Create camera with default settings
    >>> camera = RealSenseCamera()
    >>>
    >>> with camera:
    ...     # Capture RGB frame
    ...     frame = camera.capture()
    ...
    ...     # Capture depth frame
    ...     depth = camera.capture_depth()
    ...
    ...     # Capture aligned RGB-D
    ...     rgb, depth = camera.capture_rgbd()
    ...
    ...     # Get point cloud
    ...     points = camera.get_pointcloud()
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, ClassVar

import numpy as np

from robo_infra.sensors.camera import (
    Camera,
    CameraConfig,
    CameraInfo,
    CameraIntrinsics,
    CameraState,
    DepthFrame,
    Frame,
    PixelFormat,
)


if TYPE_CHECKING:
    from numpy.typing import NDArray


logger = logging.getLogger(__name__)


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class PointCloud:
    """3D point cloud from depth camera.

    Attributes:
        points: Nx3 array of XYZ coordinates in meters.
        colors: Nx3 array of RGB colors (0-255) or None.
        timestamp: Capture timestamp.
        frame_number: Sequential frame number.
    """

    points: NDArray[np.float32]
    colors: NDArray[np.uint8] | None
    timestamp: float
    frame_number: int = 0

    @property
    def num_points(self) -> int:
        """Number of points in the cloud."""
        return self.points.shape[0]

    def filter_by_distance(
        self, min_distance: float = 0.0, max_distance: float = 10.0
    ) -> PointCloud:
        """Filter points by distance from camera.

        Args:
            min_distance: Minimum distance in meters.
            max_distance: Maximum distance in meters.

        Returns:
            Filtered point cloud.
        """
        distances = np.linalg.norm(self.points, axis=1)
        mask = (distances >= min_distance) & (distances <= max_distance)

        return PointCloud(
            points=self.points[mask],
            colors=self.colors[mask] if self.colors is not None else None,
            timestamp=self.timestamp,
            frame_number=self.frame_number,
        )

    def downsample(self, voxel_size: float = 0.01) -> PointCloud:
        """Downsample point cloud using voxel grid.

        Args:
            voxel_size: Voxel size in meters.

        Returns:
            Downsampled point cloud.
        """
        # Simple voxel downsampling
        voxel_indices = np.floor(self.points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)

        return PointCloud(
            points=self.points[unique_indices],
            colors=self.colors[unique_indices] if self.colors is not None else None,
            timestamp=self.timestamp,
            frame_number=self.frame_number,
        )

    def to_open3d(self) -> Any:
        """Convert to Open3D point cloud.

        Returns:
            open3d.geometry.PointCloud object.

        Raises:
            ImportError: If Open3D is not installed.
        """
        try:
            import open3d as o3d
        except ImportError as e:
            raise ImportError(
                "Open3D required for point cloud conversion:\n  pip install open3d"
            ) from e

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        if self.colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(self.colors / 255.0)
        return pcd


# =============================================================================
# RealSense Configuration
# =============================================================================


class RealSenseConfig(CameraConfig):
    """Extended configuration for Intel RealSense cameras.

    Adds depth-specific and RealSense-specific settings.
    """

    # Depth stream settings
    depth_width: int = 640
    depth_height: int = 480
    depth_fps: int = 30

    # Depth processing
    depth_preset: str = "high_accuracy"  # high_accuracy, high_density, default
    laser_power: int = 150  # 0-360 for D400, 0-100 for L515
    emitter_enabled: bool = True

    # Filters
    decimation_filter: bool = False
    decimation_magnitude: int = 2
    spatial_filter: bool = True
    temporal_filter: bool = True
    hole_filling: bool = False

    # Alignment
    align_depth_to_color: bool = True

    # IMU (for cameras with IMU)
    enable_imu: bool = False
    imu_fps: int = 200


# =============================================================================
# RealSense Camera Implementation
# =============================================================================


class RealSenseCamera(Camera):
    """Intel RealSense depth camera.

    Supports D400 series and L500 series depth cameras. Provides RGB
    streaming, depth streaming, RGB-D capture, and point cloud generation.

    Features:
        - RGB and depth streaming
        - Aligned RGB-D capture
        - Point cloud generation
        - Depth post-processing filters
        - IMU data (on supported models)
        - Auto-exposure and laser control

    Args:
        serial_number: Camera serial number (None for first available).
        config: Camera configuration.
        intrinsics: Override camera intrinsics.

    Example:
        >>> camera = RealSenseCamera()
        >>> camera.enable()
        >>> rgb, depth = camera.capture_rgbd()
        >>> points = camera.get_pointcloud()
        >>> camera.disable()
    """

    camera_type: ClassVar[str] = "realsense"

    # Device-specific depth scales (meters per unit)
    _DEPTH_SCALES: ClassVar[dict[str, float]] = {
        "D415": 0.001,
        "D435": 0.001,
        "D435i": 0.001,
        "D455": 0.001,
        "D405": 0.001,
        "L515": 0.00025,  # 0.25mm per unit
    }

    def __init__(
        self,
        serial_number: str | None = None,
        *,
        config: CameraConfig | RealSenseConfig | None = None,
        intrinsics: CameraIntrinsics | None = None,
    ) -> None:
        """Initialize RealSense camera.

        Args:
            serial_number: Camera serial number (None = first available).
            config: Camera configuration.
            intrinsics: Camera intrinsics (auto-detected from camera).
        """
        super().__init__(
            name=f"realsense_{serial_number or 'default'}",
            config=config or RealSenseConfig(),
            intrinsics=intrinsics,
        )

        self._serial_number = serial_number
        self._rs_config: Any = None
        self._pipeline: Any = None
        self._profile: Any = None
        self._align: Any = None
        self._pc: Any = None  # Point cloud calculator
        self._filters: list[Any] = []
        self._device_info: dict[str, str] = {}
        self._depth_scale: float = 0.001
        self._color_intrinsics: Any = None
        self._depth_intrinsics: Any = None

    @property
    def serial_number(self) -> str | None:
        """Camera serial number."""
        return self._serial_number

    @property
    def device_name(self) -> str:
        """Device product name."""
        return self._device_info.get("name", "Unknown RealSense")

    @property
    def firmware_version(self) -> str:
        """Device firmware version."""
        return self._device_info.get("firmware", "Unknown")

    @property
    def depth_scale(self) -> float:
        """Depth scale (meters per unit)."""
        return self._depth_scale

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def _connect(self) -> None:
        """Connect to RealSense camera."""
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            if os.getenv("ROBO_SIMULATION"):
                logger.warning("⚠️ SIMULATION MODE - pyrealsense2 not available")
                self._init_simulation()
                return
            raise ImportError(
                "pyrealsense2 required for RealSense cameras:\n  pip install robo-infra[realsense]"
            ) from e

        self._rs = rs

        # Create pipeline
        self._pipeline = rs.pipeline()
        self._rs_config = rs.config()

        # Select device by serial number if provided
        if self._serial_number:
            self._rs_config.enable_device(self._serial_number)

        # Configure color stream
        self._rs_config.enable_stream(
            rs.stream.color,
            self._config.width,
            self._config.height,
            rs.format.rgb8,
            self._config.fps,
        )

        # Configure depth stream
        if isinstance(self._config, RealSenseConfig):
            depth_w = self._config.depth_width
            depth_h = self._config.depth_height
            depth_fps = self._config.depth_fps
        else:
            depth_w = self._config.width
            depth_h = self._config.height
            depth_fps = self._config.fps

        self._rs_config.enable_stream(
            rs.stream.depth,
            depth_w,
            depth_h,
            rs.format.z16,
            depth_fps,
        )

        # Start pipeline
        self._profile = self._pipeline.start(self._rs_config)

        # Get device info
        device = self._profile.get_device()
        self._device_info = {
            "name": device.get_info(rs.camera_info.name),
            "serial": device.get_info(rs.camera_info.serial_number),
            "firmware": device.get_info(rs.camera_info.firmware_version),
        }
        self._serial_number = self._device_info["serial"]

        # Get depth scale
        depth_sensor = device.first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()

        # Configure depth sensor
        self._configure_depth_sensor(depth_sensor)

        # Get intrinsics
        streams = self._profile.get_streams()
        for stream in streams:
            if stream.stream_type() == rs.stream.color:
                self._color_intrinsics = stream.as_video_stream_profile().get_intrinsics()
            elif stream.stream_type() == rs.stream.depth:
                self._depth_intrinsics = stream.as_video_stream_profile().get_intrinsics()

        # Create aligner (depth to color)
        if isinstance(self._config, RealSenseConfig) and self._config.align_depth_to_color:
            self._align = rs.align(rs.stream.color)

        # Create point cloud calculator
        self._pc = rs.pointcloud()

        # Setup filters
        self._setup_filters()

        logger.info(
            f"Connected to {self._device_info['name']} "
            f"(SN: {self._serial_number}, FW: {self._device_info['firmware']})"
        )

    def _configure_depth_sensor(self, depth_sensor: Any) -> None:
        """Configure depth sensor settings."""
        rs = self._rs

        # Laser power
        if isinstance(self._config, RealSenseConfig):
            if depth_sensor.supports(rs.option.laser_power):
                depth_sensor.set_option(rs.option.laser_power, self._config.laser_power)
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(
                    rs.option.emitter_enabled,
                    1.0 if self._config.emitter_enabled else 0.0,
                )

            # Preset
            if depth_sensor.supports(rs.option.visual_preset):
                presets = {
                    "high_accuracy": rs.rs400_visual_preset.high_accuracy,
                    "high_density": rs.rs400_visual_preset.high_density,
                    "default": rs.rs400_visual_preset.default,
                }
                preset = presets.get(self._config.depth_preset, rs.rs400_visual_preset.default)
                depth_sensor.set_option(rs.option.visual_preset, preset.value)

    def _setup_filters(self) -> None:
        """Setup depth post-processing filters."""
        rs = self._rs

        if not isinstance(self._config, RealSenseConfig):
            return

        self._filters = []

        # Decimation filter (reduces resolution for faster processing)
        if self._config.decimation_filter:
            decimation = rs.decimation_filter()
            decimation.set_option(rs.option.filter_magnitude, self._config.decimation_magnitude)
            self._filters.append(decimation)

        # Spatial filter (edge-preserving smoothing)
        if self._config.spatial_filter:
            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude, 2)
            spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
            spatial.set_option(rs.option.filter_smooth_delta, 20)
            self._filters.append(spatial)

        # Temporal filter (reduces noise over time)
        if self._config.temporal_filter:
            temporal = rs.temporal_filter()
            temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
            temporal.set_option(rs.option.filter_smooth_delta, 20)
            self._filters.append(temporal)

        # Hole filling
        if self._config.hole_filling:
            hole_fill = rs.hole_filling_filter()
            self._filters.append(hole_fill)

    def _init_simulation(self) -> None:
        """Initialize simulated camera for testing."""
        self._device_info = {
            "name": "Simulated RealSense D435",
            "serial": "SIMULATION",
            "firmware": "0.0.0",
        }
        self._depth_scale = 0.001
        logger.info("Initialized simulated RealSense camera")

    def _disconnect(self) -> None:
        """Disconnect from RealSense camera."""
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as e:
                logger.warning(f"Error stopping pipeline: {e}")
            finally:
                self._pipeline = None
                self._profile = None
                self._align = None
                self._pc = None
                self._filters = []

    # -------------------------------------------------------------------------
    # Frame Capture
    # -------------------------------------------------------------------------

    def capture(self) -> Frame:
        """Capture a single RGB frame.

        Returns:
            RGB frame.

        Raises:
            RuntimeError: If camera is not enabled.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        if self._pipeline is None:
            # Simulation mode
            return self._capture_simulated_rgb()

        self._state = CameraState.CAPTURING
        timestamp = time.monotonic()

        try:
            # Wait for frames
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)

            # Get color frame
            color_frame = frames.get_color_frame()
            if not color_frame:
                raise RuntimeError("Failed to get color frame")

            # Convert to numpy
            color_data = np.asanyarray(color_frame.get_data())

            self._frame_count += 1
            self._state = CameraState.CONNECTED

            return Frame(
                data=color_data,
                timestamp=timestamp,
                width=color_data.shape[1],
                height=color_data.shape[0],
                format=PixelFormat.RGB,
                frame_number=self._frame_count,
            )

        except Exception as e:
            self._state = CameraState.ERROR
            raise RuntimeError(f"Failed to capture frame: {e}") from e

    def capture_depth(self) -> DepthFrame:
        """Capture a depth frame.

        Returns:
            Depth frame with depth values.

        Raises:
            RuntimeError: If camera is not enabled.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        if self._pipeline is None:
            return self._capture_simulated_depth()

        self._state = CameraState.CAPTURING
        timestamp = time.monotonic()

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)

            # Apply alignment if configured
            if self._align is not None:
                frames = self._align.process(frames)

            # Get depth frame
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                raise RuntimeError("Failed to get depth frame")

            # Apply filters
            for f in self._filters:
                depth_frame = f.process(depth_frame)

            # Convert to numpy
            depth_data = np.asanyarray(depth_frame.get_data())

            self._frame_count += 1
            self._state = CameraState.CONNECTED

            return DepthFrame(
                data=depth_data,
                timestamp=timestamp,
                width=depth_data.shape[1],
                height=depth_data.shape[0],
                depth_scale=self._depth_scale,
                min_depth=0.3,  # D400 typical min
                max_depth=10.0,  # D400 typical max
                frame_number=self._frame_count,
            )

        except Exception as e:
            self._state = CameraState.ERROR
            raise RuntimeError(f"Failed to capture depth: {e}") from e

    def capture_rgbd(self) -> tuple[Frame, DepthFrame]:
        """Capture aligned RGB and depth frames.

        Returns:
            Tuple of (RGB frame, aligned depth frame).

        Raises:
            RuntimeError: If camera is not enabled.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        if self._pipeline is None:
            return (self._capture_simulated_rgb(), self._capture_simulated_depth())

        self._state = CameraState.CAPTURING
        timestamp = time.monotonic()

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)

            # Apply alignment
            if self._align is not None:
                frames = self._align.process(frames)

            # Get color frame
            color_frame = frames.get_color_frame()
            if not color_frame:
                raise RuntimeError("Failed to get color frame")

            # Get depth frame
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                raise RuntimeError("Failed to get depth frame")

            # Apply filters to depth
            for f in self._filters:
                depth_frame = f.process(depth_frame)

            # Convert to numpy
            color_data = np.asanyarray(color_frame.get_data())
            depth_data = np.asanyarray(depth_frame.get_data())

            self._frame_count += 1
            self._state = CameraState.CONNECTED

            rgb = Frame(
                data=color_data,
                timestamp=timestamp,
                width=color_data.shape[1],
                height=color_data.shape[0],
                format=PixelFormat.RGB,
                frame_number=self._frame_count,
            )

            depth = DepthFrame(
                data=depth_data,
                timestamp=timestamp,
                width=depth_data.shape[1],
                height=depth_data.shape[0],
                depth_scale=self._depth_scale,
                min_depth=0.3,
                max_depth=10.0,
                frame_number=self._frame_count,
            )

            return (rgb, depth)

        except Exception as e:
            self._state = CameraState.ERROR
            raise RuntimeError(f"Failed to capture RGB-D: {e}") from e

    def get_pointcloud(
        self,
        colored: bool = True,
        *,
        max_distance: float = 10.0,
    ) -> PointCloud:
        """Generate point cloud from current frame.

        Args:
            colored: Include RGB colors.
            max_distance: Maximum depth in meters.

        Returns:
            3D point cloud.

        Raises:
            RuntimeError: If camera is not enabled.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        if self._pipeline is None:
            return self._generate_simulated_pointcloud()

        self._state = CameraState.CAPTURING
        timestamp = time.monotonic()

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)

            # Align for colored point cloud
            if self._align is not None and colored:
                frames = self._align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame() if colored else None

            if not depth_frame:
                raise RuntimeError("Failed to get depth frame")

            # Apply filters
            for f in self._filters:
                depth_frame = f.process(depth_frame)

            # Calculate point cloud
            if colored and color_frame:
                self._pc.map_to(color_frame)
            points = self._pc.calculate(depth_frame)

            # Get vertices
            vertices = np.asanyarray(points.get_vertices()).view(np.float32)
            vertices = vertices.reshape(-1, 3)

            # Get colors if available
            colors = None
            if colored and color_frame:
                tex_coords = np.asanyarray(points.get_texture_coordinates()).view(np.float32)
                tex_coords = tex_coords.reshape(-1, 2)

                # Sample colors from texture
                color_data = np.asanyarray(color_frame.get_data())
                h, w = color_data.shape[:2]
                u = (tex_coords[:, 0] * w).astype(np.int32)
                v = (tex_coords[:, 1] * h).astype(np.int32)
                u = np.clip(u, 0, w - 1)
                v = np.clip(v, 0, h - 1)
                colors = color_data[v, u]

            # Filter by distance
            distances = np.linalg.norm(vertices, axis=1)
            valid_mask = (distances > 0) & (distances < max_distance)
            vertices = vertices[valid_mask]
            if colors is not None:
                colors = colors[valid_mask]

            self._frame_count += 1
            self._state = CameraState.CONNECTED

            return PointCloud(
                points=vertices,
                colors=colors,
                timestamp=timestamp,
                frame_number=self._frame_count,
            )

        except Exception as e:
            self._state = CameraState.ERROR
            raise RuntimeError(f"Failed to get point cloud: {e}") from e

    # -------------------------------------------------------------------------
    # Simulation Methods
    # -------------------------------------------------------------------------

    def _capture_simulated_rgb(self) -> Frame:
        """Generate simulated RGB frame."""
        height, width = self._config.height, self._config.width
        frame_data = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
        return Frame(
            data=frame_data,
            timestamp=time.monotonic(),
            width=width,
            height=height,
            format=PixelFormat.RGB,
            frame_number=self._frame_count,
        )

    def _capture_simulated_depth(self) -> DepthFrame:
        """Generate simulated depth frame."""
        if isinstance(self._config, RealSenseConfig):
            height = self._config.depth_height
            width = self._config.depth_width
        else:
            height, width = self._config.height, self._config.width

        # Generate gradient depth (closer at top, farther at bottom)
        y = np.linspace(500, 3000, height, dtype=np.uint16)  # 0.5m to 3m
        depth_data = np.tile(y[:, np.newaxis], (1, width))
        # Add some noise
        depth_data = depth_data + np.random.randint(-50, 50, depth_data.shape, dtype=np.int16)
        depth_data = np.clip(depth_data, 0, 65535).astype(np.uint16)

        return DepthFrame(
            data=depth_data,
            timestamp=time.monotonic(),
            width=width,
            height=height,
            depth_scale=self._depth_scale,
            min_depth=0.3,
            max_depth=10.0,
            frame_number=self._frame_count,
        )

    def _generate_simulated_pointcloud(self) -> PointCloud:
        """Generate simulated point cloud."""
        # Generate a simple plane
        n_points = 1000
        x = np.random.uniform(-1, 1, n_points).astype(np.float32)
        y = np.random.uniform(-1, 1, n_points).astype(np.float32)
        z = np.random.uniform(0.5, 2.0, n_points).astype(np.float32)
        points = np.column_stack([x, y, z])

        colors = np.random.randint(0, 255, (n_points, 3), dtype=np.uint8)

        return PointCloud(
            points=points,
            colors=colors,
            timestamp=time.monotonic(),
            frame_number=self._frame_count,
        )

    # -------------------------------------------------------------------------
    # Intrinsics
    # -------------------------------------------------------------------------

    def get_intrinsics(self) -> CameraIntrinsics:
        """Get camera intrinsic parameters.

        Returns:
            Color camera intrinsic parameters.
        """
        if self._intrinsics is not None:
            return self._intrinsics

        if self._color_intrinsics is not None:
            intr = self._color_intrinsics
            return CameraIntrinsics(
                fx=intr.fx,
                fy=intr.fy,
                cx=intr.ppx,
                cy=intr.ppy,
                width=intr.width,
                height=intr.height,
                distortion=np.array(intr.coeffs),
            )

        # Return default intrinsics
        return CameraIntrinsics.default(self._config.width, self._config.height)

    def get_depth_intrinsics(self) -> CameraIntrinsics:
        """Get depth camera intrinsic parameters.

        Returns:
            Depth camera intrinsic parameters.
        """
        if self._depth_intrinsics is not None:
            intr = self._depth_intrinsics
            return CameraIntrinsics(
                fx=intr.fx,
                fy=intr.fy,
                cx=intr.ppx,
                cy=intr.ppy,
                width=intr.width,
                height=intr.height,
                distortion=np.array(intr.coeffs),
            )

        # Return default intrinsics
        if isinstance(self._config, RealSenseConfig):
            return CameraIntrinsics.default(self._config.depth_width, self._config.depth_height)
        return CameraIntrinsics.default(self._config.width, self._config.height)

    # -------------------------------------------------------------------------
    # Device Discovery
    # -------------------------------------------------------------------------

    @classmethod
    def list_devices(cls) -> list[CameraInfo]:
        """List available RealSense devices.

        Returns:
            List of available camera information.
        """
        cameras = []

        try:
            import pyrealsense2 as rs

            ctx = rs.context()
            for device in ctx.query_devices():
                name = device.get_info(rs.camera_info.name)
                serial = device.get_info(rs.camera_info.serial_number)

                cameras.append(
                    CameraInfo(
                        device_id=serial,
                        name=name,
                        driver="pyrealsense2",
                        supported_resolutions=[
                            (640, 480),
                            (848, 480),
                            (1280, 720),
                            (1920, 1080),
                        ],
                        supported_formats=[PixelFormat.RGB, PixelFormat.BGR],
                        is_available=True,
                    )
                )
        except ImportError:
            logger.debug("pyrealsense2 not available")
        except Exception as e:
            logger.debug(f"Error listing RealSense devices: {e}")

        return cameras

    @classmethod
    def is_available(cls) -> bool:
        """Check if any RealSense camera is available.

        Returns:
            True if at least one RealSense is connected.
        """
        try:
            import pyrealsense2 as rs

            ctx = rs.context()
            return len(ctx.query_devices()) > 0
        except ImportError:
            return False


# =============================================================================
# Convenience Functions
# =============================================================================


def list_realsense_cameras() -> list[CameraInfo]:
    """List available Intel RealSense cameras.

    Returns:
        List of available camera devices.
    """
    return RealSenseCamera.list_devices()


def open_realsense(
    serial_number: str | None = None,
    color_resolution: tuple[int, int] = (640, 480),
    depth_resolution: tuple[int, int] = (640, 480),
    fps: int = 30,
) -> RealSenseCamera:
    """Open a RealSense camera with common settings.

    Args:
        serial_number: Camera serial number (None = first available).
        color_resolution: RGB resolution (width, height).
        depth_resolution: Depth resolution (width, height).
        fps: Frames per second.

    Returns:
        Configured and enabled RealSenseCamera instance.

    Example:
        >>> camera = open_realsense(color_resolution=(1280, 720))
        >>> rgb, depth = camera.capture_rgbd()
        >>> camera.disable()
    """
    config = RealSenseConfig(
        width=color_resolution[0],
        height=color_resolution[1],
        fps=fps,
        depth_width=depth_resolution[0],
        depth_height=depth_resolution[1],
        depth_fps=fps,
    )
    camera = RealSenseCamera(serial_number=serial_number, config=config)
    camera.enable()
    return camera
