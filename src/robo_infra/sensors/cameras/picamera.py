"""Raspberry Pi Camera Module implementation.

This module provides support for Raspberry Pi Camera Modules (CSI cameras)
using the picamera2 library (Pi 5 / Bookworm) or picamera (older Pi OS).

Supported Hardware:
    - Pi Camera Module v1 (OV5647, 5MP)
    - Pi Camera Module v2 (IMX219, 8MP)
    - Pi Camera Module v3 (IMX708, 12MP, autofocus)
    - Pi HQ Camera (IMX477, 12.3MP)
    - Pi Global Shutter Camera (IMX296, 1.6MP)
    - Third-party CSI cameras (ArduCam, etc.)

Requirements:
    - Raspberry Pi with camera interface enabled
    - picamera2 (Pi 5 / Bullseye+) or picamera (legacy)
    - libcamera (usually pre-installed on Pi OS)

Example:
    >>> from robo_infra.sensors.cameras.picamera import PiCamera
    >>>
    >>> # Create camera with default settings
    >>> camera = PiCamera()
    >>>
    >>> # Or with custom configuration
    >>> from robo_infra.sensors.camera import CameraConfig
    >>> config = CameraConfig(width=1920, height=1080, fps=30)
    >>> camera = PiCamera(config=config)
    >>>
    >>> with camera:
    ...     frame = camera.capture()
    ...     print(f"Captured: {frame.width}x{frame.height}")
    ...
    ...     # High-res still capture
    ...     still = camera.capture_still(resolution=(4056, 3040))
    ...     print(f"Still: {still.width}x{still.height}")
"""

from __future__ import annotations

import logging
import os
import time
from typing import TYPE_CHECKING, Any, ClassVar

import numpy as np

from robo_infra.sensors.camera import (
    Camera,
    CameraConfig,
    CameraInfo,
    CameraIntrinsics,
    CameraState,
    Frame,
    PixelFormat,
)


if TYPE_CHECKING:
    from numpy.typing import NDArray


logger = logging.getLogger(__name__)


# =============================================================================
# Library Detection
# =============================================================================


def _is_raspberry_pi() -> bool:
    """Check if running on a Raspberry Pi."""
    # Check for /proc/device-tree/model
    try:
        with open("/proc/device-tree/model") as f:
            model = f.read().lower()
            return "raspberry pi" in model
    except (FileNotFoundError, PermissionError):
        pass

    # Check for Pi-specific paths
    if os.path.exists("/opt/vc/lib"):
        return True

    # Check environment variable (for forcing in tests)
    return bool(os.getenv("ROBO_FORCE_PI"))


def _get_camera_library() -> str | None:
    """Detect which camera library is available.

    Returns:
        'picamera2' for modern Pi OS, 'picamera' for legacy, None if unavailable.
    """
    # Try picamera2 first (modern, Pi 5 / Bookworm+)
    try:
        import picamera2  # noqa: F401

        return "picamera2"
    except ImportError:
        pass

    # Try legacy picamera
    try:
        import picamera  # noqa: F401

        return "picamera"
    except ImportError:
        pass

    return None


# =============================================================================
# Pi Camera Configuration
# =============================================================================


class PiCameraConfig(CameraConfig):
    """Extended configuration for Raspberry Pi Camera.

    Adds Pi-specific settings like sensor mode, HDR, and video stabilization.
    """

    # Sensor mode (0 = auto)
    sensor_mode: int = 0

    # Video stabilization
    video_stabilization: bool = False

    # HDR mode (Pi Camera v3)
    hdr_mode: bool = False

    # Autofocus mode (Pi Camera v3)
    autofocus_mode: str = "auto"  # auto, manual, continuous

    # Image quality for JPEG (1-100)
    jpeg_quality: int = 85

    # Rotation in degrees: 0, 90, 180, or 270
    rotation: int = 0

    # Preview settings
    preview_enabled: bool = False
    preview_alpha: int = 255  # 0-255 transparency


# =============================================================================
# Pi Camera Implementation
# =============================================================================


class PiCamera(Camera):
    """Raspberry Pi Camera Module (CSI).

    Supports Pi Camera v1, v2, v3, HQ Camera, and compatible CSI cameras.
    Automatically detects and uses picamera2 (modern) or picamera (legacy).

    Features:
        - Video streaming and still capture
        - Configurable resolution, FPS, exposure
        - Autofocus support (v3)
        - HDR mode (v3)
        - Video stabilization
        - Hardware-accelerated encoding

    Args:
        camera_num: Camera number (0 for first camera, 1 for second).
        config: Camera configuration.
        intrinsics: Camera intrinsic parameters.
        force_library: Force specific library ('picamera2' or 'picamera').

    Example:
        >>> camera = PiCamera()
        >>> camera.enable()
        >>> frame = camera.capture()
        >>> camera.disable()
    """

    camera_type: ClassVar[str] = "picamera"

    # Default intrinsics for common sensors (approximate)
    _SENSOR_INTRINSICS: ClassVar[dict[str, dict[str, float]]] = {
        "imx219": {  # Pi Camera v2
            "fx_factor": 0.95,  # Focal length as fraction of width
            "fy_factor": 0.95,
        },
        "imx477": {  # Pi HQ Camera
            "fx_factor": 0.92,
            "fy_factor": 0.92,
        },
        "imx708": {  # Pi Camera v3
            "fx_factor": 0.93,
            "fy_factor": 0.93,
        },
        "ov5647": {  # Pi Camera v1
            "fx_factor": 0.90,
            "fy_factor": 0.90,
        },
    }

    def __init__(
        self,
        camera_num: int = 0,
        *,
        config: CameraConfig | PiCameraConfig | None = None,
        intrinsics: CameraIntrinsics | None = None,
        force_library: str | None = None,
    ) -> None:
        """Initialize Pi Camera.

        Args:
            camera_num: Camera index (0 or 1 for Pi Compute Module).
            config: Camera configuration.
            intrinsics: Camera intrinsics (auto-detected if None).
            force_library: Force 'picamera2' or 'picamera' library.
        """
        super().__init__(
            name=f"picamera_{camera_num}",
            config=config or PiCameraConfig(),
            intrinsics=intrinsics,
        )

        self._camera_num = camera_num
        self._force_library = force_library
        self._library: str | None = None
        self._picam: Any = None  # picamera2.Picamera2 or picamera.PiCamera
        self._sensor_name: str = "unknown"
        self._sensor_resolution: tuple[int, int] = (0, 0)
        self._preview_active = False

        # Validate we're on a Pi (unless simulating)
        if not _is_raspberry_pi() and not os.getenv("ROBO_SIMULATION"):
            logger.warning("Not running on Raspberry Pi. Set ROBO_SIMULATION=true to simulate.")

    @property
    def camera_num(self) -> int:
        """Camera number (0 or 1)."""
        return self._camera_num

    @property
    def sensor_name(self) -> str:
        """Detected sensor name (e.g., 'imx219', 'imx708')."""
        return self._sensor_name

    @property
    def sensor_resolution(self) -> tuple[int, int]:
        """Native sensor resolution (width, height)."""
        return self._sensor_resolution

    @property
    def library(self) -> str | None:
        """Camera library in use ('picamera2' or 'picamera')."""
        return self._library

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def _connect(self) -> None:
        """Connect to Pi Camera hardware."""
        # Determine which library to use
        if self._force_library:
            self._library = self._force_library
        else:
            self._library = _get_camera_library()

        if self._library is None:
            if os.getenv("ROBO_SIMULATION"):
                logger.warning("⚠️ SIMULATION MODE - No Pi camera library available")
                self._init_simulation()
                return
            raise ImportError(
                "No camera library available. Install picamera2:\n"
                "  pip install robo-infra[picamera]\n"
                "  or: sudo apt install python3-picamera2"
            )

        if self._library == "picamera2":
            self._init_picamera2()
        else:
            self._init_picamera_legacy()

    def _disconnect(self) -> None:
        """Disconnect from Pi Camera hardware."""
        if self._picam is None:
            return

        try:
            if self._library == "picamera2":
                self._picam.stop()
                self._picam.close()
            else:
                self._picam.close()
        except Exception as e:
            logger.warning(f"Error closing camera: {e}")
        finally:
            self._picam = None

    def _init_simulation(self) -> None:
        """Initialize simulated camera for testing."""
        self._sensor_name = "simulated"
        self._sensor_resolution = (self._config.width, self._config.height)
        logger.info("Initialized simulated Pi Camera")

    def _init_picamera2(self) -> None:
        """Initialize using picamera2 library."""
        from picamera2 import Picamera2

        self._picam = Picamera2(camera_num=self._camera_num)

        # Get sensor info
        props = self._picam.camera_properties
        self._sensor_name = props.get("Model", "unknown").lower()
        self._sensor_resolution = props.get(
            "PixelArraySize", (self._config.width, self._config.height)
        )

        # Configure camera
        config_dict = self._build_picamera2_config()
        self._picam.configure(config_dict)

        # Apply additional settings
        self._apply_picamera2_settings()

        # Start the camera
        self._picam.start()

        logger.info(
            f"Initialized Pi Camera: {self._sensor_name} "
            f"({self._sensor_resolution[0]}x{self._sensor_resolution[1]})"
        )

    def _build_picamera2_config(self) -> dict[str, Any]:
        """Build picamera2 configuration dictionary."""

        # Use video configuration for streaming, still for single capture
        config = self._picam.create_video_configuration(
            main={"size": (self._config.width, self._config.height), "format": "RGB888"}
        )

        # Adjust buffer count
        config["buffer_count"] = self._config.buffer_size

        # Set sensor mode if specified
        if isinstance(self._config, PiCameraConfig) and self._config.sensor_mode > 0:
            modes = self._picam.sensor_modes
            if self._config.sensor_mode < len(modes):
                config["sensor"] = {"mode": self._config.sensor_mode}

        return config

    def _apply_picamera2_settings(self) -> None:
        """Apply additional picamera2 settings."""
        controls = {}

        # Exposure
        if not self._config.auto_exposure and self._config.exposure_time is not None:
            # Convert seconds to microseconds
            controls["ExposureTime"] = int(self._config.exposure_time * 1_000_000)
            controls["AeEnable"] = False
        else:
            controls["AeEnable"] = True

        # Gain
        if not self._config.auto_gain and self._config.gain is not None:
            controls["AnalogueGain"] = self._config.gain

        # White balance
        if not self._config.auto_white_balance:
            controls["AwbEnable"] = False

        # Pi Camera v3 specific settings
        if isinstance(self._config, PiCameraConfig):
            # Autofocus
            if self._sensor_name == "imx708":
                if self._config.autofocus_mode == "continuous":
                    controls["AfMode"] = 2  # AfModeContinuous
                elif self._config.autofocus_mode == "manual":
                    controls["AfMode"] = 0  # AfModeManual
                else:
                    controls["AfMode"] = 1  # AfModeAuto

            # HDR (requires sensor support)
            if self._config.hdr_mode:
                controls["HdrMode"] = 1  # If supported

        if controls:
            self._picam.set_controls(controls)

    def _init_picamera_legacy(self) -> None:
        """Initialize using legacy picamera library."""
        import picamera

        self._picam = picamera.PiCamera(camera_num=self._camera_num)

        # Get sensor info
        self._sensor_name = self._picam.revision.lower()
        self._sensor_resolution = self._picam.MAX_RESOLUTION

        # Configure resolution and framerate
        self._picam.resolution = (self._config.width, self._config.height)
        self._picam.framerate = self._config.fps

        # Rotation
        if isinstance(self._config, PiCameraConfig):
            self._picam.rotation = self._config.rotation

        # Video stabilization
        if isinstance(self._config, PiCameraConfig):
            self._picam.video_stabilization = self._config.video_stabilization

        # Exposure
        if not self._config.auto_exposure:
            self._picam.exposure_mode = "off"
            if self._config.exposure_time is not None:
                self._picam.shutter_speed = int(self._config.exposure_time * 1_000_000)
        else:
            self._picam.exposure_mode = "auto"

        # Allow camera to warm up
        time.sleep(0.1)

        logger.info(
            f"Initialized Pi Camera (legacy): {self._sensor_name} "
            f"({self._sensor_resolution[0]}x{self._sensor_resolution[1]})"
        )

    # -------------------------------------------------------------------------
    # Frame Capture
    # -------------------------------------------------------------------------

    def capture(self) -> Frame:
        """Capture a single frame.

        Returns:
            Captured frame in RGB format.

        Raises:
            RuntimeError: If camera is not enabled.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        self._state = CameraState.CAPTURING
        timestamp = time.monotonic()

        try:
            if self._library == "picamera2":
                data = self._capture_picamera2()
            elif self._library == "picamera":
                data = self._capture_picamera_legacy()
            else:
                # Simulation mode
                data = self._capture_simulated()

            self._frame_count += 1
            self._last_frame_time = timestamp
            self._state = CameraState.CONNECTED

            return Frame(
                data=data,
                timestamp=timestamp,
                width=self._config.width,
                height=self._config.height,
                format=PixelFormat.RGB,
                frame_number=self._frame_count,
                exposure_time=self._config.exposure_time,
            )

        except Exception as e:
            self._state = CameraState.ERROR
            raise RuntimeError(f"Failed to capture frame: {e}") from e

    def _capture_picamera2(self) -> NDArray[np.uint8]:
        """Capture frame using picamera2."""
        # Capture as numpy array
        array = self._picam.capture_array("main")

        # Apply flips if configured
        if self._config.flip_horizontal:
            array = np.fliplr(array)
        if self._config.flip_vertical:
            array = np.flipud(array)

        return array

    def _capture_picamera_legacy(self) -> NDArray[np.uint8]:
        """Capture frame using legacy picamera."""

        # Capture to numpy array
        output = np.empty((self._config.height, self._config.width, 3), dtype=np.uint8)
        self._picam.capture(output, "rgb")

        # Apply flips
        if self._config.flip_horizontal:
            output = np.fliplr(output)
        if self._config.flip_vertical:
            output = np.flipud(output)

        return output

    def _capture_simulated(self) -> NDArray[np.uint8]:
        """Generate simulated frame for testing."""
        # Generate a test pattern
        height, width = self._config.height, self._config.width
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Create gradient pattern
        x = np.linspace(0, 255, width, dtype=np.uint8)
        y = np.linspace(0, 255, height, dtype=np.uint8)
        xx, yy = np.meshgrid(x, y)

        frame[:, :, 0] = xx  # Red gradient
        frame[:, :, 1] = yy  # Green gradient
        frame[:, :, 2] = 128  # Blue constant

        return frame

    # -------------------------------------------------------------------------
    # Still Capture
    # -------------------------------------------------------------------------

    def capture_still(
        self,
        resolution: tuple[int, int] | None = None,
        *,
        jpeg_quality: int | None = None,
    ) -> Frame:
        """Capture a high-resolution still image.

        Temporarily switches to still capture mode for higher quality.

        Args:
            resolution: Still resolution (width, height). Uses sensor max if None.
            jpeg_quality: JPEG quality if saving (1-100).

        Returns:
            High-resolution frame.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        # Use sensor resolution if not specified
        if resolution is None:
            resolution = self._sensor_resolution

        if self._library == "picamera2":
            return self._capture_still_picamera2(resolution)
        elif self._library == "picamera":
            return self._capture_still_picamera_legacy(resolution)
        else:
            # Simulation: just return regular capture at requested size
            frame = self.capture()
            if resolution != (frame.width, frame.height):
                return frame.resize(resolution[0], resolution[1])
            return frame

    def _capture_still_picamera2(self, resolution: tuple[int, int]) -> Frame:
        """Capture still using picamera2."""
        # Switch to still configuration
        still_config = self._picam.create_still_configuration(
            main={"size": resolution, "format": "RGB888"}
        )
        self._picam.switch_mode_and_capture_file(still_config, "/dev/null")

        # Actually capture
        timestamp = time.monotonic()
        self._picam.switch_mode(still_config)
        array = self._picam.capture_array("main")

        # Switch back to video mode
        video_config = self._build_picamera2_config()
        self._picam.switch_mode(video_config)

        return Frame(
            data=array,
            timestamp=timestamp,
            width=resolution[0],
            height=resolution[1],
            format=PixelFormat.RGB,
            frame_number=self._frame_count,
        )

    def _capture_still_picamera_legacy(self, resolution: tuple[int, int]) -> Frame:
        """Capture still using legacy picamera."""
        # Temporarily change resolution
        old_resolution = self._picam.resolution
        self._picam.resolution = resolution

        # Capture
        timestamp = time.monotonic()
        output = np.empty((resolution[1], resolution[0], 3), dtype=np.uint8)
        self._picam.capture(output, "rgb")

        # Restore resolution
        self._picam.resolution = old_resolution

        return Frame(
            data=output,
            timestamp=timestamp,
            width=resolution[0],
            height=resolution[1],
            format=PixelFormat.RGB,
            frame_number=self._frame_count,
        )

    # -------------------------------------------------------------------------
    # Preview
    # -------------------------------------------------------------------------

    def start_preview(self, fullscreen: bool = True) -> None:
        """Start camera preview on the display.

        Args:
            fullscreen: Whether to show fullscreen preview.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before preview")

        if self._preview_active:
            return

        if self._library == "picamera2":
            from picamera2 import Preview

            self._picam.start_preview(Preview.DRM if fullscreen else Preview.QTGL)
        elif self._library == "picamera":
            self._picam.start_preview()
            if fullscreen:
                self._picam.preview_fullscreen = True

        self._preview_active = True
        logger.info("Started camera preview")

    def stop_preview(self) -> None:
        """Stop camera preview."""
        if not self._preview_active:
            return

        if self._library in {"picamera2", "picamera"}:
            self._picam.stop_preview()

        self._preview_active = False
        logger.info("Stopped camera preview")

    # -------------------------------------------------------------------------
    # Intrinsics
    # -------------------------------------------------------------------------

    def get_intrinsics(self) -> CameraIntrinsics:
        """Get camera intrinsic parameters.

        Returns estimated intrinsics based on sensor type if not provided.

        Returns:
            Camera intrinsic parameters.
        """
        if self._intrinsics is not None:
            return self._intrinsics

        # Estimate intrinsics based on sensor
        sensor_params = self._SENSOR_INTRINSICS.get(
            self._sensor_name, {"fx_factor": 0.9, "fy_factor": 0.9}
        )

        width = self._config.width
        height = self._config.height

        return CameraIntrinsics(
            fx=width * sensor_params["fx_factor"],
            fy=height * sensor_params["fy_factor"],
            cx=width / 2,
            cy=height / 2,
            width=width,
            height=height,
        )

    # -------------------------------------------------------------------------
    # Autofocus (Pi Camera v3)
    # -------------------------------------------------------------------------

    def trigger_autofocus(self, wait: bool = True) -> bool:
        """Trigger autofocus (Pi Camera v3 only).

        Args:
            wait: Whether to wait for focus to complete.

        Returns:
            True if focus succeeded, False otherwise.
        """
        if self._sensor_name != "imx708":
            logger.warning("Autofocus only supported on Pi Camera v3 (IMX708)")
            return False

        if self._library == "picamera2":
            try:
                from picamera2 import Controls

                self._picam.set_controls({"AfTrigger": Controls.AfTriggerStart})

                if wait:
                    # Wait for focus to complete (max 3 seconds)
                    import time

                    for _ in range(30):
                        time.sleep(0.1)
                        metadata = self._picam.capture_metadata()
                        if metadata.get("AfState") == 2:  # Focused
                            return True

                return True
            except Exception as e:
                logger.error(f"Autofocus failed: {e}")
                return False

        return False

    def set_focus_distance(self, distance_m: float) -> None:
        """Set manual focus distance (Pi Camera v3 only).

        Args:
            distance_m: Focus distance in meters (0.1 to infinity).
        """
        if self._sensor_name != "imx708":
            logger.warning("Manual focus only supported on Pi Camera v3 (IMX708)")
            return

        if self._library == "picamera2":
            # Convert distance to lens position (approximate: 1 / distance)
            lens_pos = max(0.0, min(32.0, 1.0 / distance_m))
            self._picam.set_controls({"LensPosition": lens_pos})

    # -------------------------------------------------------------------------
    # Device Discovery
    # -------------------------------------------------------------------------

    @classmethod
    def list_devices(cls) -> list[CameraInfo]:
        """List available Pi Camera devices.

        Returns:
            List of available camera information.
        """
        cameras = []

        library = _get_camera_library()
        if library == "picamera2":
            try:
                from picamera2 import Picamera2

                global_info = Picamera2.global_camera_info()
                for i, info in enumerate(global_info):
                    cameras.append(
                        CameraInfo(
                            device_id=i,
                            name=info.get("Model", f"Pi Camera {i}"),
                            driver="picamera2",
                            supported_resolutions=[
                                (640, 480),
                                (1280, 720),
                                (1920, 1080),
                            ],
                            supported_formats=[PixelFormat.RGB, PixelFormat.BGR],
                            is_available=True,
                        )
                    )
            except Exception as e:
                logger.debug(f"Error listing picamera2 devices: {e}")

        elif library == "picamera":
            # Legacy picamera doesn't have good enumeration
            try:
                import picamera

                cam = picamera.PiCamera()
                cameras.append(
                    CameraInfo(
                        device_id=0,
                        name=f"Pi Camera ({cam.revision})",
                        driver="picamera",
                        supported_resolutions=[
                            (640, 480),
                            (1280, 720),
                            (1920, 1080),
                        ],
                        supported_formats=[PixelFormat.RGB],
                        is_available=True,
                    )
                )
                cam.close()
            except Exception as e:
                logger.debug(f"Error detecting picamera: {e}")

        return cameras

    @classmethod
    def is_available(cls) -> bool:
        """Check if Pi Camera is available on this system.

        Returns:
            True if running on Pi with camera library.
        """
        return _is_raspberry_pi() and _get_camera_library() is not None


# =============================================================================
# Convenience Functions
# =============================================================================


def list_picameras() -> list[CameraInfo]:
    """List available Raspberry Pi cameras.

    Returns:
        List of available Pi Camera devices.
    """
    return PiCamera.list_devices()


def open_picamera(
    camera_num: int = 0,
    resolution: tuple[int, int] = (1280, 720),
    fps: int = 30,
) -> PiCamera:
    """Open a Pi Camera with common settings.

    Args:
        camera_num: Camera number (0 or 1).
        resolution: Resolution (width, height).
        fps: Frames per second.

    Returns:
        Configured and enabled PiCamera instance.

    Example:
        >>> camera = open_picamera(resolution=(1920, 1080))
        >>> frame = camera.capture()
        >>> camera.disable()
    """
    config = PiCameraConfig(
        width=resolution[0],
        height=resolution[1],
        fps=fps,
    )
    camera = PiCamera(camera_num=camera_num, config=config)
    camera.enable()
    return camera
