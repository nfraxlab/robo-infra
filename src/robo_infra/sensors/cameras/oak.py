"""Luxonis OAK (DepthAI) camera implementation.

This module provides support for Luxonis OAK-D and OAK-1 cameras using
the DepthAI library. These cameras feature on-device AI processing
for real-time object detection, tracking, and depth perception.

Supported Hardware:
    - OAK-1 (RGB only, on-device AI)
    - OAK-D (RGB + stereo depth, on-device AI)
    - OAK-D Lite (compact RGB + depth)
    - OAK-D Pro (RGB + depth + IR projector)
    - OAK-D S2 / OAK-D W (various form factors)
    - OAK-FFC (flexible camera module)

Requirements:
    - depthai library (pip install depthai)
    - OpenCV (for image decoding)

Example:
    >>> from robo_infra.sensors.cameras.oak import OAKCamera
    >>>
    >>> # Create camera with default settings
    >>> camera = OAKCamera()
    >>>
    >>> with camera:
    ...     # Capture RGB frame
    ...     frame = camera.capture()
    ...
    ...     # Capture stereo (left, right, depth)
    ...     left, right, depth = camera.capture_stereo()
    ...
    ...     # Run neural network for object detection
    ...     detections = camera.run_neural_network("yolov5n_coco.blob")
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

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


logger = logging.getLogger(__name__)


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class Detection:
    """Object detection result from neural network.

    Attributes:
        label: Class label/ID.
        label_name: Human-readable class name (if available).
        confidence: Detection confidence (0-1).
        bbox: Bounding box (x_min, y_min, x_max, y_max) normalized 0-1.
        depth: Estimated depth in meters (if available).
        spatial_coords: 3D position (x, y, z) in meters (if available).
    """

    label: int
    confidence: float
    bbox: tuple[float, float, float, float]  # x_min, y_min, x_max, y_max (normalized)
    label_name: str = ""
    depth: float | None = None
    spatial_coords: tuple[float, float, float] | None = None

    @property
    def center(self) -> tuple[float, float]:
        """Center point of bounding box (normalized)."""
        x_min, y_min, x_max, y_max = self.bbox
        return ((x_min + x_max) / 2, (y_min + y_max) / 2)

    @property
    def width(self) -> float:
        """Bounding box width (normalized)."""
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> float:
        """Bounding box height (normalized)."""
        return self.bbox[3] - self.bbox[1]

    @property
    def area(self) -> float:
        """Bounding box area (normalized)."""
        return self.width * self.height

    def to_pixels(self, width: int, height: int) -> tuple[int, int, int, int]:
        """Convert normalized bbox to pixel coordinates.

        Args:
            width: Image width.
            height: Image height.

        Returns:
            Bounding box as (x_min, y_min, x_max, y_max) in pixels.
        """
        x_min, y_min, x_max, y_max = self.bbox
        return (
            int(x_min * width),
            int(y_min * height),
            int(x_max * width),
            int(y_max * height),
        )


@dataclass(slots=True)
class TrackedObject(Detection):
    """Tracked object with ID and trajectory.

    Extends Detection with tracking information.
    """

    track_id: int = -1
    age: int = 0  # Number of frames tracked
    status: str = "new"  # new, tracked, lost
    velocity: tuple[float, float, float] | None = None


# =============================================================================
# OAK Configuration
# =============================================================================


class OAKConfig(CameraConfig):
    """Extended configuration for Luxonis OAK cameras.

    Adds depth, stereo, and neural network settings.
    """

    # Stereo depth settings
    enable_depth: bool = True
    depth_median_filter: str = "kernel_7x7"  # off, kernel_3x3, kernel_5x5, kernel_7x7
    depth_lrc_check: bool = True  # Left-right consistency check
    depth_extended_disparity: bool = False  # Double disparity range
    depth_subpixel: bool = True  # Sub-pixel accuracy

    # IR projector (OAK-D Pro)
    enable_ir_projector: bool = True
    ir_projector_brightness: float = 0.0  # 0-1

    # Neural network settings
    nn_blob_path: str | None = None  # Path to .blob model file
    nn_confidence_threshold: float = 0.5
    nn_spatial_detection: bool = True  # Calculate 3D position

    # Preview settings
    preview_size: tuple[int, int] = (300, 300)

    # USB settings
    usb_speed: str = "usb3"  # usb2, usb3


# =============================================================================
# OAK Camera Implementation
# =============================================================================


class OAKCamera(Camera):
    """Luxonis OAK-D camera with on-device AI.

    Supports OAK-1, OAK-D, OAK-D Lite, OAK-D Pro and variants. Provides
    RGB streaming, stereo depth, and on-device neural network inference.

    Features:
        - 4K RGB camera (12MP sensor)
        - Stereo depth from dual cameras
        - On-device neural network inference
        - Object detection and tracking
        - Spatial AI (3D object positions)
        - IR projector for low-light depth (Pro models)

    Args:
        device_id: Device MxID or None for first available.
        config: Camera configuration.
        intrinsics: Override camera intrinsics.

    Example:
        >>> camera = OAKCamera()
        >>> camera.enable()
        >>>
        >>> # Basic capture
        >>> frame = camera.capture()
        >>>
        >>> # Stereo depth
        >>> left, right, depth = camera.capture_stereo()
        >>>
        >>> # Object detection
        >>> camera.load_neural_network("yolov5.blob")
        >>> detections = camera.detect()
        >>>
        >>> camera.disable()
    """

    camera_type: ClassVar[str] = "oak"

    # Common COCO class names for detection models
    COCO_CLASSES: ClassVar[list[str]] = [
        "person",
        "bicycle",
        "car",
        "motorcycle",
        "airplane",
        "bus",
        "train",
        "truck",
        "boat",
        "traffic light",
        "fire hydrant",
        "stop sign",
        "parking meter",
        "bench",
        "bird",
        "cat",
        "dog",
        "horse",
        "sheep",
        "cow",
        "elephant",
        "bear",
        "zebra",
        "giraffe",
        "backpack",
        "umbrella",
        "handbag",
        "tie",
        "suitcase",
        "frisbee",
        "skis",
        "snowboard",
        "sports ball",
        "kite",
        "baseball bat",
        "baseball glove",
        "skateboard",
        "surfboard",
        "tennis racket",
        "bottle",
        "wine glass",
        "cup",
        "fork",
        "knife",
        "spoon",
        "bowl",
        "banana",
        "apple",
        "sandwich",
        "orange",
        "broccoli",
        "carrot",
        "hot dog",
        "pizza",
        "donut",
        "cake",
        "chair",
        "couch",
        "potted plant",
        "bed",
        "dining table",
        "toilet",
        "tv",
        "laptop",
        "mouse",
        "remote",
        "keyboard",
        "cell phone",
        "microwave",
        "oven",
        "toaster",
        "sink",
        "refrigerator",
        "book",
        "clock",
        "vase",
        "scissors",
        "teddy bear",
        "hair drier",
        "toothbrush",
    ]

    def __init__(
        self,
        device_id: str | None = None,
        *,
        config: CameraConfig | OAKConfig | None = None,
        intrinsics: CameraIntrinsics | None = None,
    ) -> None:
        """Initialize OAK camera.

        Args:
            device_id: Device MxID (None = first available).
            config: Camera configuration.
            intrinsics: Camera intrinsics (auto-detected from camera).
        """
        super().__init__(
            name=f"oak_{device_id or 'default'}",
            config=config or OAKConfig(),
            intrinsics=intrinsics,
        )

        self._device_id = device_id
        self._device: Any = None
        self._pipeline: Any = None
        self._dai: Any = None  # depthai module reference

        # Output queues
        self._color_queue: Any = None
        self._left_queue: Any = None
        self._right_queue: Any = None
        self._depth_queue: Any = None
        self._nn_queue: Any = None
        self._tracker_queue: Any = None

        # Device info
        self._device_info: dict[str, str] = {}
        self._calibration: Any = None
        self._nn_loaded = False
        self._class_names: list[str] = []

    @property
    def device_id(self) -> str | None:
        """Device MxID."""
        return self._device_id

    @property
    def device_name(self) -> str:
        """Device product name."""
        return self._device_info.get("name", "Unknown OAK")

    @property
    def has_depth(self) -> bool:
        """Whether device supports depth (OAK-D variants)."""
        return self._device_info.get("has_depth", False)

    @property
    def has_ir_projector(self) -> bool:
        """Whether device has IR projector (Pro models)."""
        return self._device_info.get("has_ir", False)

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def _connect(self) -> None:
        """Connect to OAK camera."""
        try:
            import depthai as dai
        except ImportError as e:
            if os.getenv("ROBO_SIMULATION"):
                logger.warning("⚠️ SIMULATION MODE - depthai not available")
                self._init_simulation()
                return
            raise ImportError(
                "depthai required for OAK cameras:\n  pip install robo-infra[oak]"
            ) from e

        self._dai = dai

        # Build pipeline
        self._pipeline = self._build_pipeline(dai)

        # Connect to device
        if self._device_id:
            device_info = dai.DeviceInfo(self._device_id)
            self._device = dai.Device(self._pipeline, device_info)
        else:
            self._device = dai.Device(self._pipeline)

        # Get device info
        self._device_info = {
            "name": self._device.getDeviceName(),
            "mxid": self._device.getMxId(),
            "has_depth": self._device.getIrDrivers() is not None
            or "OAK-D" in self._device.getDeviceName(),
            "has_ir": len(self._device.getIrDrivers() or []) > 0,
        }
        self._device_id = self._device_info["mxid"]

        # Get calibration
        self._calibration = self._device.readCalibration()

        # Setup output queues
        self._color_queue = self._device.getOutputQueue(name="color", maxSize=4, blocking=False)

        if isinstance(self._config, OAKConfig) and self._config.enable_depth:
            self._left_queue = self._device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self._right_queue = self._device.getOutputQueue(name="right", maxSize=4, blocking=False)
            self._depth_queue = self._device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        logger.info(f"Connected to {self._device_info['name']} (MxID: {self._device_id})")

    def _build_pipeline(self, dai: Any) -> Any:
        """Build DepthAI pipeline.

        Args:
            dai: depthai module.

        Returns:
            Configured pipeline.
        """
        pipeline = dai.Pipeline()

        # Color camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self._config.width, self._config.height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam_rgb.setFps(self._config.fps)

        # Color output
        xout_color = pipeline.create(dai.node.XLinkOut)
        xout_color.setStreamName("color")
        cam_rgb.preview.link(xout_color.input)

        # Stereo depth (for OAK-D models)
        if isinstance(self._config, OAKConfig) and self._config.enable_depth:
            # Mono cameras (left and right)
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_left.setCamera("left")
            mono_right.setCamera("right")
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_left.setFps(self._config.fps)
            mono_right.setFps(self._config.fps)

            # Stereo depth
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
            stereo.setLeftRightCheck(self._config.depth_lrc_check)
            stereo.setExtendedDisparity(self._config.depth_extended_disparity)
            stereo.setSubpixel(self._config.depth_subpixel)

            # Median filter
            median_map = {
                "off": dai.MedianFilter.MEDIAN_OFF,
                "kernel_3x3": dai.MedianFilter.KERNEL_3x3,
                "kernel_5x5": dai.MedianFilter.KERNEL_5x5,
                "kernel_7x7": dai.MedianFilter.KERNEL_7x7,
            }
            stereo.setMedianFilter(
                median_map.get(
                    self._config.depth_median_filter,
                    dai.MedianFilter.KERNEL_7x7,
                )
            )

            # Link mono cameras to stereo
            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            # Outputs for left, right, depth
            xout_left = pipeline.create(dai.node.XLinkOut)
            xout_left.setStreamName("left")
            mono_left.out.link(xout_left.input)

            xout_right = pipeline.create(dai.node.XLinkOut)
            xout_right.setStreamName("right")
            mono_right.out.link(xout_right.input)

            xout_depth = pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName("depth")
            stereo.depth.link(xout_depth.input)

        return pipeline

    def _init_simulation(self) -> None:
        """Initialize simulated camera for testing."""
        self._device_info = {
            "name": "Simulated OAK-D",
            "mxid": "SIMULATION",
            "has_depth": True,
            "has_ir": False,
        }
        logger.info("Initialized simulated OAK camera")

    def _disconnect(self) -> None:
        """Disconnect from OAK camera."""
        if self._device is not None:
            try:
                self._device.close()
            except Exception as e:
                logger.warning(f"Error closing device: {e}")
            finally:
                self._device = None
                self._pipeline = None
                self._color_queue = None
                self._left_queue = None
                self._right_queue = None
                self._depth_queue = None
                self._nn_queue = None

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

        if self._device is None:
            return self._capture_simulated_rgb()

        self._state = CameraState.CAPTURING
        timestamp = time.monotonic()

        try:
            # Get frame from queue (timeout 5 seconds)
            in_frame = self._color_queue.get()
            if in_frame is None:
                raise RuntimeError("Timeout waiting for frame")

            # Get numpy array
            frame_data = in_frame.getCvFrame()

            # Convert BGR to RGB if needed
            if len(frame_data.shape) == 3 and frame_data.shape[2] == 3:
                frame_data = frame_data[:, :, ::-1]  # BGR to RGB

            self._frame_count += 1
            self._state = CameraState.CONNECTED

            return Frame(
                data=frame_data,
                timestamp=timestamp,
                width=frame_data.shape[1],
                height=frame_data.shape[0],
                format=PixelFormat.RGB,
                frame_number=self._frame_count,
            )

        except Exception as e:
            self._state = CameraState.ERROR
            raise RuntimeError(f"Failed to capture frame: {e}") from e

    def capture_stereo(self) -> tuple[Frame, Frame, DepthFrame]:
        """Capture stereo frames (left, right, and depth).

        Returns:
            Tuple of (left frame, right frame, depth frame).

        Raises:
            RuntimeError: If camera doesn't support depth or is not enabled.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before capture")

        if not self.has_depth:
            raise RuntimeError("Device does not support stereo depth")

        if self._device is None:
            return (
                self._capture_simulated_rgb(),
                self._capture_simulated_rgb(),
                self._capture_simulated_depth(),
            )

        self._state = CameraState.CAPTURING
        timestamp = time.monotonic()

        try:
            # Get frames from queues
            left_frame = self._left_queue.get()
            right_frame = self._right_queue.get()
            depth_frame = self._depth_queue.get()

            if not all([left_frame, right_frame, depth_frame]):
                raise RuntimeError("Timeout waiting for stereo frames")

            # Convert to numpy arrays
            left_data = left_frame.getCvFrame()
            right_data = right_frame.getCvFrame()
            depth_data = depth_frame.getCvFrame()

            self._frame_count += 1
            self._state = CameraState.CONNECTED

            left = Frame(
                data=left_data,
                timestamp=timestamp,
                width=left_data.shape[1],
                height=left_data.shape[0],
                format=PixelFormat.GRAY,
                frame_number=self._frame_count,
            )

            right = Frame(
                data=right_data,
                timestamp=timestamp,
                width=right_data.shape[1],
                height=right_data.shape[0],
                format=PixelFormat.GRAY,
                frame_number=self._frame_count,
            )

            depth = DepthFrame(
                data=depth_data,
                timestamp=timestamp,
                width=depth_data.shape[1],
                height=depth_data.shape[0],
                depth_scale=0.001,  # OAK outputs mm
                min_depth=0.2,
                max_depth=20.0,
                frame_number=self._frame_count,
            )

            return (left, right, depth)

        except Exception as e:
            self._state = CameraState.ERROR
            raise RuntimeError(f"Failed to capture stereo: {e}") from e

    # -------------------------------------------------------------------------
    # Neural Network Inference
    # -------------------------------------------------------------------------

    def load_neural_network(
        self,
        blob_path: str | Path,
        *,
        class_names: list[str] | None = None,
        input_size: tuple[int, int] = (416, 416),
        spatial: bool = True,
    ) -> None:
        """Load a neural network model for on-device inference.

        The neural network will run on the device's VPU for real-time
        inference without loading the host CPU.

        Args:
            blob_path: Path to .blob model file.
            class_names: List of class names for labels.
            input_size: Network input size (width, height).
            spatial: Enable spatial detection (3D positions).

        Raises:
            FileNotFoundError: If blob file doesn't exist.
            RuntimeError: If camera is not connected.
        """
        if not self._is_enabled:
            raise RuntimeError("Camera must be enabled before loading network")

        blob_path = Path(blob_path)
        if not blob_path.exists():
            raise FileNotFoundError(f"Blob file not found: {blob_path}")

        self._class_names = class_names or self.COCO_CLASSES

        if self._device is None:
            # Simulation mode
            self._nn_loaded = True
            logger.info(f"Loaded neural network (simulated): {blob_path.name}")
            return

        # Rebuild pipeline with neural network
        dai = self._dai
        pipeline = dai.Pipeline()

        # Color camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(input_size[0], input_size[1])
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(self._config.fps)

        # Detection network
        if spatial and self.has_depth:
            # Spatial detection network (with depth)
            detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

            # Mono cameras for depth
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_left.setCamera("left")
            mono_right.setCamera("right")
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)
            stereo.depth.link(detection_nn.inputDepth)

            detection_nn.setBoundingBoxScaleFactor(0.5)
            detection_nn.setDepthLowerThreshold(100)
            detection_nn.setDepthUpperThreshold(10000)
        else:
            # Standard detection network
            detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)

        detection_nn.setBlobPath(str(blob_path))
        detection_nn.setConfidenceThreshold(
            self._config.nn_confidence_threshold if isinstance(self._config, OAKConfig) else 0.5
        )
        detection_nn.setNumClasses(len(self._class_names))
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
        detection_nn.setAnchorMasks({"side26": [0, 1, 2], "side13": [3, 4, 5]})
        detection_nn.setIouThreshold(0.5)
        detection_nn.setNumInferenceThreads(2)
        detection_nn.input.setBlocking(False)

        cam_rgb.preview.link(detection_nn.input)

        # Detection output
        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("detections")
        detection_nn.out.link(xout_nn.input)

        # Color passthrough
        xout_color = pipeline.create(dai.node.XLinkOut)
        xout_color.setStreamName("color")
        detection_nn.passthrough.link(xout_color.input)

        # Stop current device and restart with new pipeline
        self._device.close()
        if self._device_id:
            device_info = dai.DeviceInfo(self._device_id)
            self._device = dai.Device(pipeline, device_info)
        else:
            self._device = dai.Device(pipeline)

        # Setup queues
        self._color_queue = self._device.getOutputQueue(name="color", maxSize=4, blocking=False)
        self._nn_queue = self._device.getOutputQueue(name="detections", maxSize=4, blocking=False)

        self._nn_loaded = True
        logger.info(f"Loaded neural network: {blob_path.name}")

    def run_neural_network(
        self,
        blob_path: str | Path | None = None,
        *,
        class_names: list[str] | None = None,
    ) -> list[Detection]:
        """Run neural network inference and return detections.

        If blob_path is provided and different from currently loaded,
        will load the new network first.

        Args:
            blob_path: Path to .blob model file (optional if already loaded).
            class_names: Class names for labels.

        Returns:
            List of detections.

        Raises:
            RuntimeError: If no network is loaded and blob_path not provided.
        """
        if blob_path is not None:
            self.load_neural_network(blob_path, class_names=class_names)
        elif not self._nn_loaded:
            raise RuntimeError(
                "No neural network loaded. Provide blob_path or call load_neural_network()"
            )

        if self._device is None:
            # Simulation mode - return fake detections
            return self._simulate_detections()

        try:
            # Get detections from queue
            in_dets = self._nn_queue.tryGet()
            if in_dets is None:
                return []

            detections = []
            for det in in_dets.detections:
                detection = Detection(
                    label=det.label,
                    label_name=(
                        self._class_names[det.label]
                        if det.label < len(self._class_names)
                        else f"class_{det.label}"
                    ),
                    confidence=det.confidence,
                    bbox=(det.xmin, det.ymin, det.xmax, det.ymax),
                )

                # Add spatial coordinates if available
                if hasattr(det, "spatialCoordinates"):
                    sc = det.spatialCoordinates
                    detection = Detection(
                        label=det.label,
                        label_name=detection.label_name,
                        confidence=det.confidence,
                        bbox=(det.xmin, det.ymin, det.xmax, det.ymax),
                        depth=sc.z / 1000.0,  # mm to meters
                        spatial_coords=(
                            sc.x / 1000.0,
                            sc.y / 1000.0,
                            sc.z / 1000.0,
                        ),
                    )

                detections.append(detection)

            return detections

        except Exception as e:
            logger.error(f"Neural network inference failed: {e}")
            return []

    def detect(self) -> list[Detection]:
        """Shorthand for run_neural_network().

        Returns:
            List of detections from currently loaded network.
        """
        return self.run_neural_network()

    def _simulate_detections(self) -> list[Detection]:
        """Generate simulated detections for testing."""
        import random

        num_detections = random.randint(0, 3)
        detections = []

        for _ in range(num_detections):
            label = random.randint(0, 79)
            x = random.uniform(0.1, 0.7)
            y = random.uniform(0.1, 0.7)
            w = random.uniform(0.1, 0.3)
            h = random.uniform(0.1, 0.3)

            detections.append(
                Detection(
                    label=label,
                    label_name=self._class_names[label]
                    if label < len(self._class_names)
                    else f"class_{label}",
                    confidence=random.uniform(0.5, 0.99),
                    bbox=(x, y, x + w, y + h),
                    depth=random.uniform(0.5, 5.0),
                    spatial_coords=(
                        random.uniform(-1, 1),
                        random.uniform(-1, 1),
                        random.uniform(0.5, 5.0),
                    ),
                )
            )

        return detections

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
        height, width = 400, 640  # Standard OAK-D mono resolution

        # Generate gradient depth
        y = np.linspace(200, 5000, height, dtype=np.uint16)
        depth_data = np.tile(y[:, np.newaxis], (1, width))
        depth_data = depth_data + np.random.randint(-100, 100, depth_data.shape, dtype=np.int16)
        depth_data = np.clip(depth_data, 0, 65535).astype(np.uint16)

        return DepthFrame(
            data=depth_data,
            timestamp=time.monotonic(),
            width=width,
            height=height,
            depth_scale=0.001,
            min_depth=0.2,
            max_depth=20.0,
            frame_number=self._frame_count,
        )

    # -------------------------------------------------------------------------
    # Intrinsics
    # -------------------------------------------------------------------------

    def get_intrinsics(self) -> CameraIntrinsics:
        """Get camera intrinsic parameters.

        Returns:
            RGB camera intrinsic parameters.
        """
        if self._intrinsics is not None:
            return self._intrinsics

        if self._calibration is not None:
            try:
                dai = self._dai
                intr = self._calibration.getCameraIntrinsics(
                    dai.CameraBoardSocket.RGB,
                    self._config.width,
                    self._config.height,
                )
                return CameraIntrinsics(
                    fx=intr[0][0],
                    fy=intr[1][1],
                    cx=intr[0][2],
                    cy=intr[1][2],
                    width=self._config.width,
                    height=self._config.height,
                )
            except Exception as e:
                logger.debug(f"Could not get calibration intrinsics: {e}")

        # Return default intrinsics
        return CameraIntrinsics.default(self._config.width, self._config.height)

    # -------------------------------------------------------------------------
    # Device Discovery
    # -------------------------------------------------------------------------

    @classmethod
    def list_devices(cls) -> list[CameraInfo]:
        """List available OAK devices.

        Returns:
            List of available camera information.
        """
        cameras = []

        try:
            import depthai as dai

            for device in dai.Device.getAllAvailableDevices():
                cameras.append(
                    CameraInfo(
                        device_id=device.getMxId(),
                        name=device.name or "OAK Camera",
                        driver="depthai",
                        supported_resolutions=[
                            (640, 480),
                            (1280, 720),
                            (1920, 1080),
                        ],
                        supported_formats=[PixelFormat.RGB, PixelFormat.BGR],
                        is_available=device.state == dai.XLinkDeviceState.X_LINK_UNBOOTED,
                    )
                )
        except ImportError:
            logger.debug("depthai not available")
        except Exception as e:
            logger.debug(f"Error listing OAK devices: {e}")

        return cameras

    @classmethod
    def is_available(cls) -> bool:
        """Check if any OAK camera is available.

        Returns:
            True if at least one OAK is connected.
        """
        try:
            import depthai as dai

            return len(dai.Device.getAllAvailableDevices()) > 0
        except ImportError:
            return False


# =============================================================================
# Convenience Functions
# =============================================================================


def list_oak_cameras() -> list[CameraInfo]:
    """List available Luxonis OAK cameras.

    Returns:
        List of available camera devices.
    """
    return OAKCamera.list_devices()


def open_oak(
    device_id: str | None = None,
    resolution: tuple[int, int] = (640, 480),
    fps: int = 30,
    enable_depth: bool = True,
) -> OAKCamera:
    """Open an OAK camera with common settings.

    Args:
        device_id: Device MxID (None = first available).
        resolution: RGB resolution (width, height).
        fps: Frames per second.
        enable_depth: Enable stereo depth.

    Returns:
        Configured and enabled OAKCamera instance.

    Example:
        >>> camera = open_oak(enable_depth=True)
        >>> left, right, depth = camera.capture_stereo()
        >>> camera.disable()
    """
    config = OAKConfig(
        width=resolution[0],
        height=resolution[1],
        fps=fps,
        enable_depth=enable_depth,
    )
    camera = OAKCamera(device_id=device_id, config=config)
    camera.enable()
    return camera
