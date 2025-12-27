"""Camera sensor abstractions for computer vision.

This module provides abstract base classes and data types for camera sensors,
including USB webcams, CSI cameras, and depth cameras.

Example:
    >>> from robo_infra.sensors.camera import Frame, CameraConfig, CameraIntrinsics
    >>> from robo_infra.sensors.cameras.usb import USBCamera
    >>>
    >>> # Create a USB camera
    >>> config = CameraConfig(width=1280, height=720, fps=30)
    >>> camera = USBCamera(device_id=0, config=config)
    >>> camera.enable()
    >>>
    >>> # Capture a single frame
    >>> frame = camera.capture()
    >>> print(f"Frame: {frame.width}x{frame.height}, format={frame.format}")
    >>>
    >>> # Stream frames
    >>> async for frame in camera.stream():
    ...     process_frame(frame)
    ...     if should_stop():
    ...         break
    >>>
    >>> camera.disable()
"""

from __future__ import annotations

import asyncio
import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, ClassVar

import numpy as np
from pydantic import BaseModel, Field


if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from numpy.typing import NDArray

logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class PixelFormat(Enum):
    """Pixel format for camera frames."""

    RGB = "RGB"  # 3-channel RGB (0-255)
    BGR = "BGR"  # 3-channel BGR (OpenCV default)
    RGBA = "RGBA"  # 4-channel with alpha
    BGRA = "BGRA"  # 4-channel BGR with alpha
    GRAY = "GRAY"  # Single channel grayscale
    GRAY16 = "GRAY16"  # 16-bit grayscale (depth)
    YUV = "YUV"  # YUV color space
    JPEG = "JPEG"  # JPEG compressed
    RAW = "RAW"  # Raw Bayer pattern


class CameraState(Enum):
    """Camera operational state."""

    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    STREAMING = "streaming"
    CAPTURING = "capturing"
    ERROR = "error"


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class Frame:
    """Single camera frame.

    Attributes:
        data: Image data as numpy array (height, width, channels) or (height, width).
        timestamp: Frame capture time in seconds (monotonic clock).
        width: Frame width in pixels.
        height: Frame height in pixels.
        format: Pixel format (RGB, BGR, GRAY, etc.).
        frame_number: Sequential frame number (optional).
        exposure_time: Exposure time in seconds (optional).
        metadata: Additional frame metadata.

    Example:
        >>> frame = Frame(
        ...     data=np.zeros((480, 640, 3), dtype=np.uint8),
        ...     timestamp=time.monotonic(),
        ...     width=640,
        ...     height=480,
        ...     format=PixelFormat.RGB,
        ... )
        >>> print(f"Frame shape: {frame.data.shape}")
    """

    data: NDArray[np.uint8]
    timestamp: float
    width: int
    height: int
    format: PixelFormat
    frame_number: int = 0
    exposure_time: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate frame data."""
        if self.data.shape[0] != self.height:
            raise ValueError(f"Data height {self.data.shape[0]} doesn't match height {self.height}")
        if self.data.shape[1] != self.width:
            raise ValueError(f"Data width {self.data.shape[1]} doesn't match width {self.width}")

    @property
    def channels(self) -> int:
        """Number of color channels."""
        if len(self.data.shape) == 2:
            return 1
        return self.data.shape[2]

    @property
    def size(self) -> tuple[int, int]:
        """Frame size as (width, height)."""
        return (self.width, self.height)

    @property
    def shape(self) -> tuple[int, ...]:
        """Frame data shape."""
        return self.data.shape

    def to_rgb(self) -> Frame:
        """Convert frame to RGB format.

        Returns:
            New frame in RGB format.

        Raises:
            ValueError: If conversion is not supported.
        """
        if self.format == PixelFormat.RGB:
            return self

        if self.format == PixelFormat.BGR:
            # BGR to RGB: reverse channel order
            rgb_data = self.data[:, :, ::-1].copy()
            return Frame(
                data=rgb_data,
                timestamp=self.timestamp,
                width=self.width,
                height=self.height,
                format=PixelFormat.RGB,
                frame_number=self.frame_number,
                exposure_time=self.exposure_time,
                metadata=self.metadata.copy(),
            )

        if self.format == PixelFormat.GRAY:
            # Grayscale to RGB: repeat channel
            rgb_data = np.stack([self.data] * 3, axis=-1)
            return Frame(
                data=rgb_data,
                timestamp=self.timestamp,
                width=self.width,
                height=self.height,
                format=PixelFormat.RGB,
                frame_number=self.frame_number,
                exposure_time=self.exposure_time,
                metadata=self.metadata.copy(),
            )

        raise ValueError(f"Cannot convert {self.format} to RGB")

    def to_bgr(self) -> Frame:
        """Convert frame to BGR format (OpenCV compatible).

        Returns:
            New frame in BGR format.
        """
        if self.format == PixelFormat.BGR:
            return self

        if self.format == PixelFormat.RGB:
            bgr_data = self.data[:, :, ::-1].copy()
            return Frame(
                data=bgr_data,
                timestamp=self.timestamp,
                width=self.width,
                height=self.height,
                format=PixelFormat.BGR,
                frame_number=self.frame_number,
                exposure_time=self.exposure_time,
                metadata=self.metadata.copy(),
            )

        # Convert to RGB first, then to BGR
        rgb_frame = self.to_rgb()
        return rgb_frame.to_bgr()

    def to_grayscale(self) -> Frame:
        """Convert frame to grayscale.

        Returns:
            New frame in grayscale format.
        """
        if self.format == PixelFormat.GRAY:
            return self

        if self.format in (PixelFormat.RGB, PixelFormat.BGR):
            # Standard luminance formula
            if self.format == PixelFormat.RGB:
                r, g, b = self.data[:, :, 0], self.data[:, :, 1], self.data[:, :, 2]
            else:
                b, g, r = self.data[:, :, 0], self.data[:, :, 1], self.data[:, :, 2]

            gray_data = (0.299 * r + 0.587 * g + 0.114 * b).astype(np.uint8)
            return Frame(
                data=gray_data,
                timestamp=self.timestamp,
                width=self.width,
                height=self.height,
                format=PixelFormat.GRAY,
                frame_number=self.frame_number,
                exposure_time=self.exposure_time,
                metadata=self.metadata.copy(),
            )

        raise ValueError(f"Cannot convert {self.format} to grayscale")

    def resize(self, width: int, height: int) -> Frame:
        """Resize frame using bilinear interpolation.

        Args:
            width: New width.
            height: New height.

        Returns:
            Resized frame.
        """
        # Simple resize without OpenCV dependency
        # Use numpy-based resize for basic functionality
        from scipy.ndimage import zoom

        if len(self.data.shape) == 2:
            zoom_factors = (height / self.height, width / self.width)
        else:
            zoom_factors = (height / self.height, width / self.width, 1)

        resized = zoom(self.data, zoom_factors, order=1)
        return Frame(
            data=resized.astype(np.uint8),
            timestamp=self.timestamp,
            width=width,
            height=height,
            format=self.format,
            frame_number=self.frame_number,
            exposure_time=self.exposure_time,
            metadata=self.metadata.copy(),
        )

    def crop(self, x: int, y: int, width: int, height: int) -> Frame:
        """Crop a region from the frame.

        Args:
            x: Left edge of crop region.
            y: Top edge of crop region.
            width: Width of crop region.
            height: Height of crop region.

        Returns:
            Cropped frame.

        Raises:
            ValueError: If crop region is out of bounds.
        """
        if x < 0 or y < 0:
            raise ValueError("Crop coordinates must be non-negative")
        if x + width > self.width or y + height > self.height:
            raise ValueError("Crop region exceeds frame bounds")

        if len(self.data.shape) == 2:
            cropped = self.data[y : y + height, x : x + width].copy()
        else:
            cropped = self.data[y : y + height, x : x + width, :].copy()

        return Frame(
            data=cropped,
            timestamp=self.timestamp,
            width=width,
            height=height,
            format=self.format,
            frame_number=self.frame_number,
            exposure_time=self.exposure_time,
            metadata=self.metadata.copy(),
        )

    def copy(self) -> Frame:
        """Create a deep copy of the frame."""
        return Frame(
            data=self.data.copy(),
            timestamp=self.timestamp,
            width=self.width,
            height=self.height,
            format=self.format,
            frame_number=self.frame_number,
            exposure_time=self.exposure_time,
            metadata=self.metadata.copy(),
        )


@dataclass(slots=True)
class DepthFrame:
    """Depth image frame from depth cameras.

    Attributes:
        data: Depth data as numpy array (height, width) in millimeters or meters.
        timestamp: Frame capture time.
        width: Frame width in pixels.
        height: Frame height in pixels.
        depth_scale: Scale factor to convert to meters (e.g., 0.001 for mm).
        min_depth: Minimum valid depth.
        max_depth: Maximum valid depth.
        frame_number: Sequential frame number.

    Example:
        >>> depth = DepthFrame(
        ...     data=np.zeros((480, 640), dtype=np.uint16),
        ...     timestamp=time.monotonic(),
        ...     width=640,
        ...     height=480,
        ...     depth_scale=0.001,  # Values in mm, scale to meters
        ...     min_depth=0.3,
        ...     max_depth=10.0,
        ... )
    """

    data: NDArray[np.uint16] | NDArray[np.float32]
    timestamp: float
    width: int
    height: int
    depth_scale: float = 0.001  # Default: mm to meters
    min_depth: float = 0.1
    max_depth: float = 10.0
    frame_number: int = 0

    def to_meters(self) -> NDArray[np.float32]:
        """Convert depth data to meters.

        Returns:
            Depth values in meters as float32 array.
        """
        return (self.data * self.depth_scale).astype(np.float32)

    def to_colormap(self, colormap: str = "jet") -> Frame:
        """Convert depth to colorized visualization.

        Args:
            colormap: Matplotlib colormap name.

        Returns:
            RGB frame with depth visualization.
        """
        # Normalize depth to 0-255
        depth_m = self.to_meters()
        normalized = np.clip((depth_m - self.min_depth) / (self.max_depth - self.min_depth), 0, 1)
        # Work with float to avoid uint8 overflow
        norm_float = normalized * 255.0

        # Apply colormap (simple jet-like mapping without matplotlib)
        # Blue (near) -> Green -> Red (far)
        r = np.clip(norm_float * 4 - 510, 0, 255).astype(np.uint8)
        g = np.where(
            norm_float < 128,
            np.clip(norm_float * 2, 0, 255),
            np.clip(510 - norm_float * 2, 0, 255),
        ).astype(np.uint8)
        b = np.clip(255 - norm_float * 4, 0, 255).astype(np.uint8)

        rgb_data = np.stack([r, g, b], axis=-1)
        return Frame(
            data=rgb_data,
            timestamp=self.timestamp,
            width=self.width,
            height=self.height,
            format=PixelFormat.RGB,
            frame_number=self.frame_number,
        )

    def mask_valid(self) -> NDArray[np.bool_]:
        """Get mask of valid depth pixels.

        Returns:
            Boolean mask where True indicates valid depth.
        """
        depth_m = self.to_meters()
        return (depth_m >= self.min_depth) & (depth_m <= self.max_depth) & (depth_m > 0)


@dataclass(slots=True)
class CameraIntrinsics:
    """Camera intrinsic parameters for 3D vision.

    These parameters describe the camera's internal geometry and are needed
    for converting between 2D pixel coordinates and 3D world coordinates.

    Attributes:
        fx: Focal length in x direction (pixels).
        fy: Focal length in y direction (pixels).
        cx: Principal point x coordinate (pixels).
        cy: Principal point y coordinate (pixels).
        width: Image width.
        height: Image height.
        distortion: Distortion coefficients [k1, k2, p1, p2, k3] or None.

    Example:
        >>> intrinsics = CameraIntrinsics(
        ...     fx=600.0, fy=600.0,
        ...     cx=320.0, cy=240.0,
        ...     width=640, height=480,
        ... )
        >>> # Project 3D point to 2D pixel
        >>> u, v = intrinsics.project([0.5, 0.3, 1.0])
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: NDArray[np.float64] | None = None

    def __post_init__(self) -> None:
        """Validate parameters."""
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("Focal lengths must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Image dimensions must be positive")

    @property
    def camera_matrix(self) -> NDArray[np.float64]:
        """Get 3x3 camera matrix (K).

        Returns:
            Camera intrinsic matrix K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
        """
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def project(self, point_3d: tuple[float, float, float]) -> tuple[float, float]:
        """Project a 3D point to 2D pixel coordinates.

        Uses pinhole camera model without distortion.

        Args:
            point_3d: 3D point (x, y, z) in camera coordinates.

        Returns:
            2D pixel coordinates (u, v).

        Raises:
            ValueError: If z <= 0 (point behind camera).
        """
        x, y, z = point_3d
        if z <= 0:
            raise ValueError("Point must be in front of camera (z > 0)")

        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return (u, v)

    def unproject(self, pixel: tuple[float, float], depth: float) -> tuple[float, float, float]:
        """Unproject a 2D pixel to 3D point.

        Args:
            pixel: Pixel coordinates (u, v).
            depth: Depth value (z coordinate).

        Returns:
            3D point (x, y, z) in camera coordinates.
        """
        u, v = pixel
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        return (x, y, depth)

    def pixel_to_ray(
        self, pixel: tuple[float, float]
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Get ray direction for a pixel.

        Args:
            pixel: Pixel coordinates (u, v).

        Returns:
            Tuple of (origin, direction) where origin is (0, 0, 0)
            and direction is a unit vector.
        """
        x, y, z = self.unproject(pixel, 1.0)
        length = np.sqrt(x * x + y * y + z * z)
        direction = (x / length, y / length, z / length)
        return ((0.0, 0.0, 0.0), direction)

    @classmethod
    def default(cls, width: int = 640, height: int = 480) -> CameraIntrinsics:
        """Create default intrinsics for a given resolution.

        Assumes a ~60 degree horizontal FOV.

        Args:
            width: Image width.
            height: Image height.

        Returns:
            Default camera intrinsics.
        """
        # Approximate focal length for ~60 degree HFOV
        fx = width / (2 * np.tan(np.radians(30)))
        fy = fx  # Square pixels
        cx = width / 2
        cy = height / 2
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)


@dataclass(slots=True)
class CameraInfo:
    """Information about an available camera device.

    Attributes:
        device_id: Device identifier (index, path, or serial number).
        name: Human-readable camera name.
        driver: Camera driver/backend name.
        supported_resolutions: List of supported (width, height) tuples.
        supported_formats: List of supported pixel formats.
        is_available: Whether the camera is currently available.
    """

    device_id: int | str
    name: str
    driver: str = ""
    supported_resolutions: list[tuple[int, int]] = field(default_factory=list)
    supported_formats: list[PixelFormat] = field(default_factory=list)
    is_available: bool = True


# =============================================================================
# Pydantic Config Model
# =============================================================================


class CameraConfig(BaseModel):
    """Pydantic configuration for cameras.

    Example:
        >>> config = CameraConfig(width=1920, height=1080, fps=60)
        >>> camera = USBCamera(device_id=0, config=config)
    """

    model_config = {"arbitrary_types_allowed": True}

    width: int = Field(default=640, ge=1, description="Frame width in pixels")
    height: int = Field(default=480, ge=1, description="Frame height in pixels")
    fps: int = Field(default=30, ge=1, le=240, description="Frames per second")
    format: PixelFormat = Field(default=PixelFormat.RGB, description="Pixel format")

    # Auto-exposure and gain
    auto_exposure: bool = Field(default=True, description="Enable auto exposure")
    exposure_time: float | None = Field(
        default=None, ge=0.0001, le=10.0, description="Manual exposure time (seconds)"
    )
    auto_gain: bool = Field(default=True, description="Enable auto gain")
    gain: float | None = Field(default=None, ge=0, le=100, description="Manual gain value")

    # White balance
    auto_white_balance: bool = Field(default=True, description="Enable auto WB")

    # Buffer settings
    buffer_size: int = Field(default=2, ge=1, le=10, description="Frame buffer size")

    # Flip/rotate
    flip_horizontal: bool = Field(default=False, description="Mirror horizontally")
    flip_vertical: bool = Field(default=False, description="Flip vertically")

    @property
    def resolution(self) -> tuple[int, int]:
        """Get resolution as (width, height) tuple."""
        return (self.width, self.height)


# =============================================================================
# Abstract Base Class
# =============================================================================


class Camera(ABC):
    """Abstract base class for camera sensors.

    Cameras are sensors that capture images. They support:
    - Single frame capture
    - Continuous streaming (async)
    - Configuration (resolution, FPS, format)
    - Intrinsic parameter access (for 3D vision)

    Subclasses must implement:
    - `capture()`: Capture a single frame
    - `_connect()`: Connect to camera hardware
    - `_disconnect()`: Disconnect from camera hardware

    Example:
        >>> class MyCamera(Camera):
        ...     def capture(self) -> Frame:
        ...         # Read from hardware
        ...         return Frame(...)
    """

    # Class-level type for device discovery
    camera_type: ClassVar[str] = "generic"

    def __init__(
        self,
        name: str = "camera",
        *,
        config: CameraConfig | None = None,
        intrinsics: CameraIntrinsics | None = None,
    ) -> None:
        """Initialize camera.

        Args:
            name: Unique camera identifier.
            config: Camera configuration.
            intrinsics: Camera intrinsic parameters.
        """
        self._name = name
        self._config = config or CameraConfig()
        self._intrinsics = intrinsics
        self._state = CameraState.DISCONNECTED
        self._is_enabled = False
        self._frame_count = 0
        self._last_frame_time: float | None = None
        self._streaming = False
        self._stream_task: asyncio.Task[None] | None = None

    @property
    def name(self) -> str:
        """Camera name."""
        return self._name

    @property
    def config(self) -> CameraConfig:
        """Camera configuration."""
        return self._config

    @property
    def state(self) -> CameraState:
        """Current camera state."""
        return self._state

    @property
    def is_enabled(self) -> bool:
        """Whether camera is enabled."""
        return self._is_enabled

    @property
    def is_connected(self) -> bool:
        """Whether camera is connected."""
        return self._state in (
            CameraState.CONNECTED,
            CameraState.STREAMING,
            CameraState.CAPTURING,
        )

    @property
    def is_streaming(self) -> bool:
        """Whether camera is currently streaming."""
        return self._streaming

    @property
    def frame_count(self) -> int:
        """Number of frames captured."""
        return self._frame_count

    @property
    def fps_actual(self) -> float:
        """Actual FPS based on frame timing."""
        # This would need to track frame times; return configured for now
        return float(self._config.fps)

    def enable(self) -> None:
        """Enable the camera and connect to hardware."""
        if self._is_enabled:
            return

        logger.debug(f"Enabling camera: {self._name}")
        self._connect()
        self._is_enabled = True
        self._state = CameraState.CONNECTED
        logger.info(f"Camera enabled: {self._name}")

    def disable(self) -> None:
        """Disable the camera and release resources."""
        if not self._is_enabled:
            return

        logger.debug(f"Disabling camera: {self._name}")

        # Stop streaming if active
        if self._streaming:
            self._streaming = False
            if self._stream_task:
                self._stream_task.cancel()
                self._stream_task = None

        self._disconnect()
        self._is_enabled = False
        self._state = CameraState.DISCONNECTED
        logger.info(f"Camera disabled: {self._name}")

    @abstractmethod
    def capture(self) -> Frame:
        """Capture a single frame.

        Returns:
            Captured frame.

        Raises:
            RuntimeError: If camera is not enabled.
        """

    async def capture_async(self) -> Frame:
        """Capture a single frame asynchronously.

        Default implementation runs capture() in thread pool.

        Returns:
            Captured frame.
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.capture)

    async def stream(
        self,
        max_frames: int | None = None,
        timeout: float | None = None,
    ) -> AsyncIterator[Frame]:
        """Stream frames continuously.

        Args:
            max_frames: Maximum number of frames to stream (None = unlimited).
            timeout: Maximum time to stream in seconds (None = unlimited).

        Yields:
            Captured frames.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before streaming")

        self._streaming = True
        self._state = CameraState.STREAMING
        frame_count = 0
        start_time = time.monotonic()
        frame_interval = 1.0 / self._config.fps

        try:
            while self._streaming:
                # Check limits
                if max_frames is not None and frame_count >= max_frames:
                    break
                if timeout is not None and (time.monotonic() - start_time) >= timeout:
                    break

                # Capture frame
                frame = await self.capture_async()
                frame_count += 1
                yield frame

                # Rate limiting
                elapsed = time.monotonic() - start_time
                expected_time = frame_count * frame_interval
                if elapsed < expected_time:
                    await asyncio.sleep(expected_time - elapsed)

        finally:
            self._streaming = False
            self._state = CameraState.CONNECTED

    def stop_streaming(self) -> None:
        """Stop the current stream."""
        self._streaming = False

    def get_intrinsics(self) -> CameraIntrinsics:
        """Get camera intrinsic parameters.

        Returns:
            Camera intrinsics (uses defaults if not calibrated).
        """
        if self._intrinsics is not None:
            return self._intrinsics
        return CameraIntrinsics.default(self._config.width, self._config.height)

    def set_intrinsics(self, intrinsics: CameraIntrinsics) -> None:
        """Set camera intrinsic parameters.

        Args:
            intrinsics: Camera intrinsics from calibration.
        """
        self._intrinsics = intrinsics

    @abstractmethod
    def _connect(self) -> None:
        """Connect to camera hardware.

        Subclasses implement hardware-specific connection.
        """

    @abstractmethod
    def _disconnect(self) -> None:
        """Disconnect from camera hardware.

        Subclasses implement hardware-specific disconnection.
        """

    def configure(self, config: CameraConfig) -> None:
        """Update camera configuration.

        Args:
            config: New configuration.
        """
        was_enabled = self._is_enabled
        if was_enabled:
            self.disable()

        self._config = config

        if was_enabled:
            self.enable()

    @classmethod
    def list_devices(cls) -> list[CameraInfo]:
        """List available camera devices.

        Override in subclasses for platform-specific device discovery.

        Returns:
            List of available cameras.
        """
        return []

    def __enter__(self) -> Camera:
        """Context manager entry."""
        self.enable()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Context manager exit."""
        self.disable()

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"{self.__class__.__name__}("
            f"name='{self._name}', "
            f"resolution={self._config.width}x{self._config.height}, "
            f"state={self._state.value})"
        )


# =============================================================================
# Simulated Camera
# =============================================================================


class SimulatedCamera(Camera):
    """Simulated camera for testing without hardware.

    Generates synthetic frames (solid colors, patterns, noise).

    Example:
        >>> camera = SimulatedCamera(pattern="gradient")
        >>> camera.enable()
        >>> frame = camera.capture()
        >>> print(frame.shape)
        (480, 640, 3)
    """

    camera_type: ClassVar[str] = "simulated"

    def __init__(
        self,
        name: str = "simulated_camera",
        *,
        config: CameraConfig | None = None,
        pattern: str = "gradient",
        color: tuple[int, int, int] = (128, 128, 128),
    ) -> None:
        """Initialize simulated camera.

        Args:
            name: Camera name.
            config: Camera configuration.
            pattern: Pattern type ("solid", "gradient", "checkerboard", "noise").
            color: Base color for solid pattern (RGB).
        """
        super().__init__(name=name, config=config)
        self._pattern = pattern
        self._color = color
        self._start_time = time.monotonic()

    def _connect(self) -> None:
        """Simulated connect (no-op)."""
        self._start_time = time.monotonic()
        logger.debug(f"Simulated camera connected: {self._name}")

    def _disconnect(self) -> None:
        """Simulated disconnect (no-op)."""
        logger.debug(f"Simulated camera disconnected: {self._name}")

    def capture(self) -> Frame:
        """Capture a simulated frame.

        Returns:
            Synthetic frame based on configured pattern.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        self._state = CameraState.CAPTURING
        width = self._config.width
        height = self._config.height

        # Generate pattern
        if self._pattern == "solid":
            data = np.full((height, width, 3), self._color, dtype=np.uint8)
        elif self._pattern == "gradient":
            # Horizontal gradient
            x = np.linspace(0, 255, width, dtype=np.uint8)
            data = np.tile(x, (height, 1))
            data = np.stack([data, data, data], axis=-1)
        elif self._pattern == "checkerboard":
            # 8x8 checkerboard
            block_h = height // 8
            block_w = width // 8
            y, x = np.mgrid[0:height, 0:width]
            checker = ((x // block_w) + (y // block_h)) % 2
            data = np.where(checker[:, :, np.newaxis], 255, 0).astype(np.uint8).repeat(3, axis=2)
        elif self._pattern == "noise":
            data = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
        else:
            # Default: dark gray
            data = np.full((height, width, 3), 64, dtype=np.uint8)

        # Add timestamp indicator (white bar at bottom)
        bar_height = max(2, height // 100)
        bar_width = int((time.monotonic() % 1.0) * width)
        data[-bar_height:, :bar_width] = 255

        self._frame_count += 1
        self._last_frame_time = time.monotonic()
        self._state = CameraState.CONNECTED

        return Frame(
            data=data,
            timestamp=self._last_frame_time,
            width=width,
            height=height,
            format=PixelFormat.RGB,
            frame_number=self._frame_count,
        )

    @classmethod
    def list_devices(cls) -> list[CameraInfo]:
        """List simulated devices."""
        return [
            CameraInfo(
                device_id=0,
                name="Simulated Camera",
                driver="simulated",
                supported_resolutions=[
                    (640, 480),
                    (1280, 720),
                    (1920, 1080),
                ],
                supported_formats=[PixelFormat.RGB, PixelFormat.GRAY],
            )
        ]
