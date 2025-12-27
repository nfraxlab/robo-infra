"""USB webcam implementation using OpenCV.

This module provides USB camera support through OpenCV's VideoCapture,
with cross-platform support for V4L2 (Linux), DirectShow (Windows),
and AVFoundation (macOS).

Example:
    >>> from robo_infra.sensors.cameras.usb import USBCamera
    >>> from robo_infra.sensors.camera import CameraConfig
    >>>
    >>> # Auto-detect camera
    >>> camera = USBCamera()
    >>> camera.enable()
    >>> frame = camera.capture()
    >>>
    >>> # With explicit configuration
    >>> config = CameraConfig(width=1280, height=720, fps=30)
    >>> camera = USBCamera(device_id=0, config=config)
    >>> with camera:
    ...     for i in range(10):
    ...         frame = camera.capture()
    ...         print(f"Frame {i}: {frame.width}x{frame.height}")
"""

from __future__ import annotations

import logging
import platform
import time
from typing import Any, ClassVar

from robo_infra.sensors.camera import (
    Camera,
    CameraConfig,
    CameraInfo,
    CameraIntrinsics,
    CameraState,
    Frame,
    PixelFormat,
)


logger = logging.getLogger(__name__)


def _get_cv2() -> Any:
    """Lazy import of OpenCV.

    Returns:
        cv2 module.

    Raises:
        ImportError: If OpenCV is not installed.
    """
    try:
        import cv2

        return cv2
    except ImportError as e:
        raise ImportError(
            "OpenCV (cv2) is required for USB camera support. "
            "Install with: pip install robo-infra[vision]"
        ) from e


class USBCamera(Camera):
    """USB webcam using OpenCV VideoCapture.

    Supports:
    - USB webcams (UVC class)
    - Built-in laptop cameras
    - Virtual cameras (OBS, v4l2loopback)

    Platform support:
    - Linux: V4L2 backend
    - Windows: DirectShow / MSMF backend
    - macOS: AVFoundation backend

    Example:
        >>> camera = USBCamera(device_id=0)
        >>> camera.enable()
        >>> frame = camera.capture()
        >>> print(f"Captured {frame.width}x{frame.height} frame")
        >>> camera.disable()

    Attributes:
        device_id: Camera device index or path.
        backend: OpenCV backend (auto-detected if None).
    """

    camera_type: ClassVar[str] = "usb"

    # Backend preferences by platform
    _BACKENDS: ClassVar[dict[str, list[int]]] = {
        "Linux": [],  # Will be populated with CAP_V4L2
        "Windows": [],  # Will be populated with CAP_DSHOW, CAP_MSMF
        "Darwin": [],  # Will be populated with CAP_AVFOUNDATION
    }

    def __init__(
        self,
        device_id: int | str = 0,
        *,
        config: CameraConfig | None = None,
        backend: int | None = None,
        intrinsics: CameraIntrinsics | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize USB camera.

        Args:
            device_id: Camera device index (0, 1, ...) or path ("/dev/video0").
            config: Camera configuration.
            backend: OpenCV backend constant (e.g., cv2.CAP_V4L2).
            intrinsics: Camera intrinsic parameters.
            name: Camera name (auto-generated if None).
        """
        camera_name = name or f"usb_camera_{device_id}"
        super().__init__(name=camera_name, config=config, intrinsics=intrinsics)

        self._device_id = device_id
        self._backend = backend
        self._cap: Any = None  # cv2.VideoCapture

    @property
    def device_id(self) -> int | str:
        """Camera device ID."""
        return self._device_id

    @property
    def backend(self) -> int | None:
        """OpenCV backend being used."""
        return self._backend

    def _connect(self) -> None:
        """Connect to USB camera via OpenCV."""
        cv2 = _get_cv2()

        # Determine backend
        backend = self._backend
        if backend is None:
            backend = self._get_default_backend(cv2)

        # Open camera
        logger.debug(f"Opening camera {self._device_id} with backend {backend}")

        if backend is not None:
            self._cap = cv2.VideoCapture(self._device_id, backend)
        else:
            self._cap = cv2.VideoCapture(self._device_id)

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Failed to open camera {self._device_id}. "
                "Check that the camera is connected and not in use."
            )

        # Apply configuration
        self._apply_config(cv2)

        logger.info(
            f"Camera connected: {self._name} "
            f"({self._config.width}x{self._config.height} @ {self._config.fps}fps)"
        )

    def _disconnect(self) -> None:
        """Disconnect from USB camera."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.debug(f"Camera disconnected: {self._name}")

    def _get_default_backend(self, cv2: Any) -> int | None:
        """Get default backend for current platform.

        Args:
            cv2: OpenCV module.

        Returns:
            Backend constant or None for auto.
        """
        system = platform.system()

        if system == "Linux":
            return getattr(cv2, "CAP_V4L2", None)
        elif system == "Windows":
            # Prefer DirectShow for better compatibility
            return getattr(cv2, "CAP_DSHOW", None)
        elif system == "Darwin":
            return getattr(cv2, "CAP_AVFOUNDATION", None)
        return None

    def _apply_config(self, cv2: Any) -> None:
        """Apply configuration to camera.

        Args:
            cv2: OpenCV module.
        """
        if self._cap is None:
            return

        # Set resolution
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._config.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._config.height)

        # Set FPS
        self._cap.set(cv2.CAP_PROP_FPS, self._config.fps)

        # Set buffer size (reduces latency)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, self._config.buffer_size)

        # Exposure
        if not self._config.auto_exposure and self._config.exposure_time is not None:
            self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual
            self._cap.set(cv2.CAP_PROP_EXPOSURE, self._config.exposure_time)
        else:
            self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto

        # Gain
        if not self._config.auto_gain and self._config.gain is not None:
            self._cap.set(cv2.CAP_PROP_GAIN, self._config.gain)

        # White balance
        if hasattr(cv2, "CAP_PROP_AUTO_WB"):
            self._cap.set(
                cv2.CAP_PROP_AUTO_WB,
                1 if self._config.auto_white_balance else 0,
            )

        # Verify actual resolution
        actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if actual_width != self._config.width or actual_height != self._config.height:
            logger.warning(
                f"Requested {self._config.width}x{self._config.height}, "
                f"got {actual_width}x{actual_height}"
            )
            # Update config to reflect actual values
            self._config = CameraConfig(
                width=actual_width,
                height=actual_height,
                fps=self._config.fps,
                format=self._config.format,
                auto_exposure=self._config.auto_exposure,
                exposure_time=self._config.exposure_time,
                auto_gain=self._config.auto_gain,
                gain=self._config.gain,
                auto_white_balance=self._config.auto_white_balance,
                buffer_size=self._config.buffer_size,
                flip_horizontal=self._config.flip_horizontal,
                flip_vertical=self._config.flip_vertical,
            )

    def capture(self) -> Frame:
        """Capture a single frame from the camera.

        Returns:
            Captured frame.

        Raises:
            RuntimeError: If camera is not enabled or capture fails.
        """
        if not self._is_enabled or self._cap is None:
            raise RuntimeError("Camera must be enabled before capture")

        cv2 = _get_cv2()
        self._state = CameraState.CAPTURING

        # Read frame
        ret, frame_data = self._cap.read()

        if not ret or frame_data is None:
            self._state = CameraState.ERROR
            raise RuntimeError("Failed to capture frame")

        # OpenCV returns BGR by default
        height, width = frame_data.shape[:2]

        # Apply flips if configured
        if self._config.flip_horizontal:
            frame_data = cv2.flip(frame_data, 1)
        if self._config.flip_vertical:
            frame_data = cv2.flip(frame_data, 0)

        # Convert format if needed
        if self._config.format == PixelFormat.RGB:
            frame_data = cv2.cvtColor(frame_data, cv2.COLOR_BGR2RGB)
            pixel_format = PixelFormat.RGB
        elif self._config.format == PixelFormat.GRAY:
            frame_data = cv2.cvtColor(frame_data, cv2.COLOR_BGR2GRAY)
            pixel_format = PixelFormat.GRAY
        else:
            pixel_format = PixelFormat.BGR

        self._frame_count += 1
        self._last_frame_time = time.monotonic()
        self._state = CameraState.CONNECTED

        return Frame(
            data=frame_data,
            timestamp=self._last_frame_time,
            width=width,
            height=height,
            format=pixel_format,
            frame_number=self._frame_count,
        )

    def get_property(self, prop_id: int) -> float:
        """Get a camera property.

        Args:
            prop_id: OpenCV property ID (e.g., cv2.CAP_PROP_BRIGHTNESS).

        Returns:
            Property value.
        """
        if self._cap is None:
            raise RuntimeError("Camera not connected")
        return self._cap.get(prop_id)

    def set_property(self, prop_id: int, value: float) -> bool:
        """Set a camera property.

        Args:
            prop_id: OpenCV property ID.
            value: Property value.

        Returns:
            True if successful.
        """
        if self._cap is None:
            raise RuntimeError("Camera not connected")
        return self._cap.set(prop_id, value)

    @classmethod
    def list_devices(cls, max_devices: int = 10) -> list[CameraInfo]:
        """List available USB cameras.

        Probes device indices 0 to max_devices-1.

        Args:
            max_devices: Maximum number of devices to probe.

        Returns:
            List of available cameras.
        """
        try:
            cv2 = _get_cv2()
        except ImportError:
            return []

        devices: list[CameraInfo] = []

        for i in range(max_devices):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                # Try to get camera name (platform-specific)
                name = f"USB Camera {i}"
                backend = int(cap.get(cv2.CAP_PROP_BACKEND))
                backend_name = cv2.videoio_registry.getBackendName(backend)

                devices.append(
                    CameraInfo(
                        device_id=i,
                        name=name,
                        driver=backend_name,
                        supported_resolutions=[(width, height)],
                        supported_formats=[PixelFormat.BGR, PixelFormat.RGB, PixelFormat.GRAY],
                        is_available=True,
                    )
                )
                cap.release()

        return devices

    @classmethod
    def auto_detect(cls, config: CameraConfig | None = None) -> USBCamera | None:
        """Auto-detect and create a camera from the first available device.

        Args:
            config: Optional camera configuration.

        Returns:
            USBCamera instance or None if no camera found.
        """
        devices = cls.list_devices(max_devices=5)
        if devices:
            device_id = devices[0].device_id
            if isinstance(device_id, int):
                return cls(device_id=device_id, config=config)
        return None


# =============================================================================
# Convenience Functions
# =============================================================================


def list_usb_cameras() -> list[CameraInfo]:
    """List all available USB cameras.

    Returns:
        List of CameraInfo for each detected camera.
    """
    return USBCamera.list_devices()


def open_camera(
    device_id: int = 0,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
) -> USBCamera:
    """Open a USB camera with common settings.

    Convenience function for quick camera access.

    Args:
        device_id: Camera device index.
        width: Frame width.
        height: Frame height.
        fps: Frames per second.

    Returns:
        Enabled USBCamera instance.

    Example:
        >>> with open_camera(0, 1280, 720, 30) as camera:
        ...     frame = camera.capture()
    """
    config = CameraConfig(width=width, height=height, fps=fps)
    camera = USBCamera(device_id=device_id, config=config)
    camera.enable()
    return camera
