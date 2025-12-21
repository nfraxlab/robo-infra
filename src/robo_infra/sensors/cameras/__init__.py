"""Camera implementations for robo-infra.

This package contains implementations of the Camera protocol for
various camera types and platforms.

Available Cameras:
    - USBCamera: USB webcams via OpenCV (cross-platform)
    - SimulatedCamera: Simulated camera for testing
    - PiCamera: Raspberry Pi Camera Module (CSI)
    - RealSenseCamera: Intel RealSense depth cameras (D400/L500 series)
    - OAKCamera: Luxonis OAK-D cameras with on-device AI

Example:
    >>> from robo_infra.sensors.cameras import USBCamera, list_cameras
    >>>
    >>> # List available cameras
    >>> for camera_info in list_cameras():
    ...     print(f"{camera_info.name} at {camera_info.device_id}")
    >>>
    >>> # Open first available camera
    >>> camera = USBCamera(device_id=0)
    >>> with camera:
    ...     frame = camera.capture()
    ...     print(f"Captured {frame.width}x{frame.height}")
    >>>
    >>> # Use RealSense depth camera
    >>> from robo_infra.sensors.cameras import RealSenseCamera
    >>> camera = RealSenseCamera()
    >>> with camera:
    ...     rgb, depth = camera.capture_rgbd()
    ...     points = camera.get_pointcloud()
    >>>
    >>> # Use OAK-D with neural network
    >>> from robo_infra.sensors.cameras import OAKCamera
    >>> camera = OAKCamera()
    >>> with camera:
    ...     camera.load_neural_network("yolov5.blob")
    ...     detections = camera.detect()
"""

from __future__ import annotations

from robo_infra.sensors.camera import (
    Camera,
    CameraConfig,
    CameraInfo,
    CameraIntrinsics,
    CameraState,
    DepthFrame,
    Frame,
    PixelFormat,
    SimulatedCamera,
)
from robo_infra.sensors.cameras.oak import (
    Detection,
    OAKCamera,
    OAKConfig,
    TrackedObject,
    list_oak_cameras,
    open_oak,
)

# Platform-specific cameras (lazy imports for optional dependencies)
from robo_infra.sensors.cameras.picamera import (
    PiCamera,
    PiCameraConfig,
    list_picameras,
    open_picamera,
)
from robo_infra.sensors.cameras.realsense import (
    PointCloud,
    RealSenseCamera,
    RealSenseConfig,
    list_realsense_cameras,
    open_realsense,
)
from robo_infra.sensors.cameras.usb import (
    USBCamera,
    list_usb_cameras,
    open_camera,
)


__all__ = [
    # Base classes and protocols
    "Camera",
    "CameraConfig",
    "CameraInfo",
    "CameraIntrinsics",
    "CameraState",
    "DepthFrame",
    "Detection",
    "Frame",
    # Luxonis OAK
    "OAKCamera",
    "OAKConfig",
    # Raspberry Pi Camera
    "PiCamera",
    "PiCameraConfig",
    "PixelFormat",
    "PointCloud",
    # Intel RealSense
    "RealSenseCamera",
    "RealSenseConfig",
    # USB cameras
    "SimulatedCamera",
    "TrackedObject",
    "USBCamera",
    # Convenience functions
    "list_cameras",
    "list_oak_cameras",
    "list_picameras",
    "list_realsense_cameras",
    "list_usb_cameras",
    "open_camera",
    "open_oak",
    "open_picamera",
    "open_realsense",
]


def list_cameras() -> list[CameraInfo]:
    """List all available cameras from all backends.

    Combines results from USB cameras and any platform-specific cameras.

    Returns:
        List of all available cameras.
    """
    cameras: list[CameraInfo] = []

    # Add USB cameras
    cameras.extend(list_usb_cameras())

    # Add simulated camera
    cameras.extend(SimulatedCamera.list_devices())

    # Add Raspberry Pi cameras
    try:
        cameras.extend(list_picameras())
    except Exception:
        pass  # picamera not available

    # Add Intel RealSense cameras
    try:
        cameras.extend(list_realsense_cameras())
    except Exception:
        pass  # pyrealsense2 not available

    # Add Luxonis OAK cameras
    try:
        cameras.extend(list_oak_cameras())
    except Exception:
        pass  # depthai not available

    return cameras
