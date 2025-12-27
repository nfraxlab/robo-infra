"""Fiducial marker detection for robot localization and tracking.

This module provides ArUco and AprilTag marker detection for robotics
applications including robot localization, object tracking, and camera
calibration.

Markers are planar fiducial patterns that can be detected with high
accuracy and provide both identity (ID) and 6-DOF pose estimation.

Example:
    >>> from robo_infra.vision.markers import ArUcoDetector, AprilTagDetector
    >>> from robo_infra.sensors.cameras import USBCamera
    >>>
    >>> # ArUco marker detection
    >>> detector = ArUcoDetector(dictionary="DICT_4X4_50")
    >>> with USBCamera() as camera:
    ...     frame = camera.capture()
    ...     markers = detector.detect(frame)
    ...     for marker in markers:
    ...         print(f"Marker {marker.id} at {marker.center}")
    >>>
    >>> # Pose estimation with camera intrinsics
    >>> intrinsics = camera.get_intrinsics()
    >>> for marker in markers:
    ...     pose = detector.estimate_pose(marker, intrinsics, marker_size=0.05)
    ...     print(f"Marker {marker.id}: translation={pose.translation}")
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, ClassVar

import numpy as np


if TYPE_CHECKING:
    from numpy.typing import NDArray

    from robo_infra.sensors.camera import CameraIntrinsics, Frame

logger = logging.getLogger(__name__)

# Simulation mode
SIMULATION = os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes")


# =============================================================================
# OpenCV Import Helper
# =============================================================================


def _get_cv2():
    """Get cv2 module, raising helpful error if not available."""
    try:
        import cv2

        return cv2
    except ImportError as e:
        raise ImportError(
            "OpenCV (cv2) is required for marker detection. "
            "Install with: pip install robo-infra[vision]"
        ) from e


def _has_cv2() -> bool:
    """Check if OpenCV is available."""
    try:
        import cv2  # noqa: F401

        return True
    except ImportError:
        return False


def _get_aruco():
    """Get cv2.aruco module."""
    cv2 = _get_cv2()
    if not hasattr(cv2, "aruco"):
        raise ImportError(
            "OpenCV ArUco module not available. "
            "Install with: pip install robo-infra[vision] or pip install opencv-contrib-python"
        )
    return cv2.aruco


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class MarkerPose:
    """6-DOF pose of a detected marker.

    Attributes:
        translation: Translation vector (x, y, z) in meters.
        rotation: Rotation vector (Rodrigues format) or rotation matrix.
        rotation_matrix: 3x3 rotation matrix.
        marker_id: ID of the marker this pose belongs to.
        reprojection_error: Average reprojection error in pixels.

    Properties:
        euler_angles: Rotation as (roll, pitch, yaw) in radians.
        quaternion: Rotation as quaternion (w, x, y, z).
    """

    translation: tuple[float, float, float]
    rotation: NDArray[np.float64]  # Rodrigues vector (3,) or matrix (3,3)
    marker_id: int
    reprojection_error: float = 0.0
    _rotation_matrix: NDArray[np.float64] | None = field(default=None, repr=False)

    @property
    def rotation_matrix(self) -> NDArray[np.float64]:
        """Get 3x3 rotation matrix."""
        if self._rotation_matrix is not None:
            return self._rotation_matrix

        if self.rotation.shape == (3, 3):
            return self.rotation

        # Convert Rodrigues vector to matrix
        cv2 = _get_cv2()
        R, _ = cv2.Rodrigues(self.rotation)
        return R

    @property
    def euler_angles(self) -> tuple[float, float, float]:
        """Get rotation as Euler angles (roll, pitch, yaw) in radians."""
        R = self.rotation_matrix

        # Extract Euler angles (ZYX convention)
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0

        return (float(roll), float(pitch), float(yaw))

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        """Get rotation as quaternion (w, x, y, z)."""
        R = self.rotation_matrix

        # Convert rotation matrix to quaternion
        trace = R[0, 0] + R[1, 1] + R[2, 2]

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return (float(w), float(x), float(y), float(z))

    @property
    def distance(self) -> float:
        """Distance from camera to marker in meters."""
        x, y, z = self.translation
        return float(np.sqrt(x**2 + y**2 + z**2))

    def transform_point(self, point: tuple[float, float, float]) -> tuple[float, float, float]:
        """Transform a point from marker frame to camera frame.

        Args:
            point: Point (x, y, z) in marker coordinate frame.

        Returns:
            Point in camera coordinate frame.
        """
        p = np.array(point)
        R = self.rotation_matrix
        t = np.array(self.translation)
        transformed = R @ p + t
        return (float(transformed[0]), float(transformed[1]), float(transformed[2]))


@dataclass(slots=True)
class ArUcoMarker:
    """Detected ArUco marker.

    Attributes:
        id: Unique marker ID from the dictionary.
        corners: 4x2 array of corner pixel coordinates (clockwise from top-left).
        center: Center point (x, y) in pixels.
        area: Marker area in pixels squared.
        timestamp: Detection timestamp.
    """

    id: int
    corners: NDArray[np.float32]  # Shape: (4, 2)
    center: tuple[float, float]
    area: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        """Calculate area if not provided."""
        if self.area == 0.0 and self.corners is not None:
            # Shoelace formula for polygon area
            x = self.corners[:, 0]
            y = self.corners[:, 1]
            self.area = 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    @property
    def size(self) -> float:
        """Approximate marker size in pixels (average side length)."""
        if self.corners is None:
            return 0.0

        # Calculate average side length
        sides = []
        for i in range(4):
            j = (i + 1) % 4
            dx = self.corners[j, 0] - self.corners[i, 0]
            dy = self.corners[j, 1] - self.corners[i, 1]
            sides.append(np.sqrt(dx**2 + dy**2))
        return float(np.mean(sides))

    @property
    def bounding_box(self) -> tuple[int, int, int, int]:
        """Get bounding box (x, y, width, height)."""
        if self.corners is None:
            return (0, 0, 0, 0)

        x_min = int(np.floor(np.min(self.corners[:, 0])))
        y_min = int(np.floor(np.min(self.corners[:, 1])))
        x_max = int(np.ceil(np.max(self.corners[:, 0])))
        y_max = int(np.ceil(np.max(self.corners[:, 1])))
        return (x_min, y_min, x_max - x_min, y_max - y_min)


@dataclass(slots=True)
class AprilTag:
    """Detected AprilTag marker.

    Attributes:
        id: Tag ID.
        family: Tag family name (e.g., "tag36h11").
        corners: 4x2 array of corner pixel coordinates.
        center: Center point (x, y) in pixels.
        hamming: Hamming distance (number of bit errors).
        decision_margin: Confidence of detection.
        timestamp: Detection timestamp.
    """

    id: int
    family: str
    corners: NDArray[np.float32]  # Shape: (4, 2)
    center: tuple[float, float]
    hamming: int = 0
    decision_margin: float = 0.0
    timestamp: float = 0.0

    @property
    def size(self) -> float:
        """Approximate tag size in pixels."""
        if self.corners is None:
            return 0.0

        sides = []
        for i in range(4):
            j = (i + 1) % 4
            dx = self.corners[j, 0] - self.corners[i, 0]
            dy = self.corners[j, 1] - self.corners[i, 1]
            sides.append(np.sqrt(dx**2 + dy**2))
        return float(np.mean(sides))

    @property
    def bounding_box(self) -> tuple[int, int, int, int]:
        """Get bounding box (x, y, width, height)."""
        if self.corners is None:
            return (0, 0, 0, 0)

        x_min = int(np.floor(np.min(self.corners[:, 0])))
        y_min = int(np.floor(np.min(self.corners[:, 1])))
        x_max = int(np.ceil(np.max(self.corners[:, 0])))
        y_max = int(np.ceil(np.max(self.corners[:, 1])))
        return (x_min, y_min, x_max - x_min, y_max - y_min)


# =============================================================================
# ArUco Dictionary Names
# =============================================================================


class ArUcoDictionary(str, Enum):
    """Available ArUco marker dictionaries."""

    # 4x4 dictionaries (16 bits)
    DICT_4X4_50 = "DICT_4X4_50"
    DICT_4X4_100 = "DICT_4X4_100"
    DICT_4X4_250 = "DICT_4X4_250"
    DICT_4X4_1000 = "DICT_4X4_1000"

    # 5x5 dictionaries (25 bits)
    DICT_5X5_50 = "DICT_5X5_50"
    DICT_5X5_100 = "DICT_5X5_100"
    DICT_5X5_250 = "DICT_5X5_250"
    DICT_5X5_1000 = "DICT_5X5_1000"

    # 6x6 dictionaries (36 bits)
    DICT_6X6_50 = "DICT_6X6_50"
    DICT_6X6_100 = "DICT_6X6_100"
    DICT_6X6_250 = "DICT_6X6_250"
    DICT_6X6_1000 = "DICT_6X6_1000"

    # 7x7 dictionaries (49 bits)
    DICT_7X7_50 = "DICT_7X7_50"
    DICT_7X7_100 = "DICT_7X7_100"
    DICT_7X7_250 = "DICT_7X7_250"
    DICT_7X7_1000 = "DICT_7X7_1000"

    # Original ArUco dictionary
    DICT_ARUCO_ORIGINAL = "DICT_ARUCO_ORIGINAL"

    # AprilTag compatible
    DICT_APRILTAG_16h5 = "DICT_APRILTAG_16h5"
    DICT_APRILTAG_25h9 = "DICT_APRILTAG_25h9"
    DICT_APRILTAG_36h10 = "DICT_APRILTAG_36h10"
    DICT_APRILTAG_36h11 = "DICT_APRILTAG_36h11"


def _get_aruco_dictionary(name: str | ArUcoDictionary):
    """Get OpenCV ArUco dictionary by name."""
    aruco = _get_aruco()
    cv2 = _get_cv2()

    if isinstance(name, ArUcoDictionary):
        name = name.value

    # Map dictionary names to OpenCV constants
    dict_map = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }

    if name not in dict_map:
        raise ValueError(f"Unknown dictionary '{name}'. Available: {list(dict_map.keys())}")

    return aruco.getPredefinedDictionary(dict_map[name])


# =============================================================================
# ArUco Detector
# =============================================================================


class ArUcoDetector:
    """ArUco marker detector for robot localization.

    ArUco markers are black and white square fiducial markers that can be
    reliably detected and identified. They're commonly used for robot
    localization, camera calibration, and augmented reality.

    Attributes:
        dictionary: ArUco dictionary being used.
        parameters: Detection parameters.

    Example:
        >>> detector = ArUcoDetector(dictionary="DICT_4X4_50")
        >>> markers = detector.detect(frame)
        >>> for marker in markers:
        ...     print(f"Found marker {marker.id} at {marker.center}")
        >>>
        >>> # Estimate 3D pose
        >>> pose = detector.estimate_pose(markers[0], camera_intrinsics, marker_size=0.1)
        >>> print(f"Distance: {pose.distance:.2f}m")
    """

    # Supported dictionaries
    DICTIONARIES: ClassVar[list[str]] = [d.value for d in ArUcoDictionary]

    def __init__(
        self,
        dictionary: str | ArUcoDictionary = "DICT_4X4_50",
        *,
        adaptive_threshold_win_size: int = 23,
        adaptive_threshold_constant: float = 7.0,
        min_marker_perimeter_rate: float = 0.03,
        max_marker_perimeter_rate: float = 4.0,
        corner_refinement_method: str = "subpix",
        corner_refinement_win_size: int = 5,
        corner_refinement_max_iterations: int = 30,
        simulation: bool | None = None,
    ) -> None:
        """Initialize ArUco detector.

        Args:
            dictionary: ArUco dictionary to use. Common choices:
                - "DICT_4X4_50": Small dictionary, fast detection (default)
                - "DICT_6X6_250": Medium dictionary, good balance
                - "DICT_APRILTAG_36h11": AprilTag compatible, high robustness
            adaptive_threshold_win_size: Window size for adaptive thresholding.
            adaptive_threshold_constant: Constant for adaptive threshold.
            min_marker_perimeter_rate: Minimum perimeter relative to image size.
            max_marker_perimeter_rate: Maximum perimeter relative to image size.
            corner_refinement_method: Corner refinement method:
                - "none": No refinement
                - "subpix": Sub-pixel refinement (default)
                - "contour": Contour-based refinement
                - "apriltag": AprilTag refinement (for AprilTag dictionaries)
            corner_refinement_win_size: Window size for corner refinement.
            corner_refinement_max_iterations: Max iterations for corner refinement.
            simulation: Override simulation mode (default: from environment).
        """
        self._simulation = simulation if simulation is not None else SIMULATION
        self._dictionary_name = dictionary if isinstance(dictionary, str) else dictionary.value

        # Store parameters for lazy initialization
        self._params = {
            "adaptive_threshold_win_size": adaptive_threshold_win_size,
            "adaptive_threshold_constant": adaptive_threshold_constant,
            "min_marker_perimeter_rate": min_marker_perimeter_rate,
            "max_marker_perimeter_rate": max_marker_perimeter_rate,
            "corner_refinement_method": corner_refinement_method,
            "corner_refinement_win_size": corner_refinement_win_size,
            "corner_refinement_max_iterations": corner_refinement_max_iterations,
        }

        # Lazy initialization
        self._dictionary = None
        self._detector = None

    def _init_detector(self) -> None:
        """Lazily initialize OpenCV detector."""
        if self._detector is not None:
            return

        if self._simulation:
            logger.info("ArUcoDetector: running in simulation mode")
            return

        aruco = _get_aruco()
        cv2 = _get_cv2()

        # Get dictionary
        self._dictionary = _get_aruco_dictionary(self._dictionary_name)

        # Create detector parameters
        params = aruco.DetectorParameters()

        # Apply custom parameters
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = self._params["adaptive_threshold_win_size"]
        params.adaptiveThreshWinSizeStep = 10
        params.adaptiveThreshConstant = self._params["adaptive_threshold_constant"]
        params.minMarkerPerimeterRate = self._params["min_marker_perimeter_rate"]
        params.maxMarkerPerimeterRate = self._params["max_marker_perimeter_rate"]

        # Corner refinement
        refinement_map = {
            "none": cv2.aruco.CORNER_REFINE_NONE,
            "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
            "contour": cv2.aruco.CORNER_REFINE_CONTOUR,
            "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG,
        }
        method = self._params["corner_refinement_method"]
        if method not in refinement_map:
            raise ValueError(f"Unknown corner_refinement_method: {method}")

        params.cornerRefinementMethod = refinement_map[method]
        params.cornerRefinementWinSize = self._params["corner_refinement_win_size"]
        params.cornerRefinementMaxIterations = self._params["corner_refinement_max_iterations"]

        # Create detector (OpenCV 4.7+ API)
        self._detector = aruco.ArucoDetector(self._dictionary, params)

    @property
    def dictionary_name(self) -> str:
        """Get dictionary name."""
        return self._dictionary_name

    def detect(self, frame: Frame) -> list[ArUcoMarker]:
        """Detect ArUco markers in a frame.

        Args:
            frame: Input frame (color or grayscale).

        Returns:
            List of detected ArUco markers.

        Example:
            >>> markers = detector.detect(frame)
            >>> for marker in markers:
            ...     print(f"ID={marker.id}, center={marker.center}")
        """
        from robo_infra.sensors.camera import PixelFormat

        timestamp = time.monotonic()

        if self._simulation:
            # Return simulated markers for testing
            logger.debug("ArUcoDetector.detect: simulation mode, returning empty list")
            return []

        self._init_detector()

        # Convert to grayscale if needed
        if frame.format == PixelFormat.GRAY:
            gray = frame.data
        elif frame.format == PixelFormat.BGR:
            cv2 = _get_cv2()
            gray = cv2.cvtColor(frame.data, cv2.COLOR_BGR2GRAY)
        elif frame.format == PixelFormat.RGB:
            cv2 = _get_cv2()
            gray = cv2.cvtColor(frame.data, cv2.COLOR_RGB2GRAY)
        else:
            # Try to convert via Frame's method
            gray_frame = frame.to_grayscale()
            gray = gray_frame.data

        # Detect markers
        corners, ids, _rejected = self._detector.detectMarkers(gray)

        markers = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]  # Shape: (4, 2)

                # Calculate center
                center_x = float(np.mean(marker_corners[:, 0]))
                center_y = float(np.mean(marker_corners[:, 1]))

                marker = ArUcoMarker(
                    id=int(marker_id),
                    corners=marker_corners.astype(np.float32),
                    center=(center_x, center_y),
                    timestamp=timestamp,
                )
                markers.append(marker)

        logger.debug(f"ArUcoDetector.detect: found {len(markers)} markers")
        return markers

    def estimate_pose(
        self,
        marker: ArUcoMarker,
        intrinsics: CameraIntrinsics,
        marker_size: float,
    ) -> MarkerPose:
        """Estimate 6-DOF pose of a detected marker.

        Args:
            marker: Detected ArUco marker.
            intrinsics: Camera intrinsic parameters.
            marker_size: Physical size of marker side in meters.

        Returns:
            6-DOF pose of the marker relative to the camera.

        Example:
            >>> pose = detector.estimate_pose(marker, intrinsics, marker_size=0.1)
            >>> print(f"Translation: {pose.translation}")
            >>> print(f"Distance: {pose.distance:.2f}m")
        """
        if self._simulation:
            # Return simulated pose
            return MarkerPose(
                translation=(0.5, 0.0, 1.0),
                rotation=np.array([0.0, 0.0, 0.0]),
                marker_id=marker.id,
                reprojection_error=0.0,
            )

        cv2 = _get_cv2()

        # Prepare object points (marker corners in marker coordinate system)
        half_size = marker_size / 2
        obj_points = np.array(
            [
                [-half_size, half_size, 0],  # Top-left
                [half_size, half_size, 0],  # Top-right
                [half_size, -half_size, 0],  # Bottom-right
                [-half_size, -half_size, 0],  # Bottom-left
            ],
            dtype=np.float32,
        )

        # Prepare image points
        img_points = marker.corners.reshape(-1, 2).astype(np.float32)

        # Camera matrix and distortion
        camera_matrix = intrinsics.camera_matrix
        dist_coeffs = intrinsics.distortion if intrinsics.distortion is not None else np.zeros(5)

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            raise RuntimeError(f"Failed to estimate pose for marker {marker.id}")

        # Calculate reprojection error
        projected, _ = cv2.projectPoints(obj_points, rvec, tvec, camera_matrix, dist_coeffs)
        error = np.mean(np.linalg.norm(projected.reshape(-1, 2) - img_points, axis=1))

        return MarkerPose(
            translation=(float(tvec[0, 0]), float(tvec[1, 0]), float(tvec[2, 0])),
            rotation=rvec.flatten(),
            marker_id=marker.id,
            reprojection_error=float(error),
        )

    def draw_markers(
        self,
        frame: Frame,
        markers: list[ArUcoMarker],
        *,
        color: tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
        show_id: bool = True,
    ) -> Frame:
        """Draw detected markers on a frame.

        Args:
            frame: Input frame.
            markers: List of detected markers to draw.
            color: Drawing color (BGR).
            thickness: Line thickness.
            show_id: Whether to show marker IDs.

        Returns:
            Frame with markers drawn.
        """
        from robo_infra.sensors.camera import Frame as FrameClass
        from robo_infra.sensors.camera import PixelFormat

        if self._simulation:
            return frame.copy()

        cv2 = _get_cv2()

        # Ensure we have a color image
        if frame.format == PixelFormat.GRAY:
            output = cv2.cvtColor(frame.data, cv2.COLOR_GRAY2BGR)
            output_format = PixelFormat.BGR
        elif frame.format == PixelFormat.RGB:
            output = frame.data.copy()
            output_format = PixelFormat.RGB
        else:
            output = frame.data.copy()
            output_format = frame.format

        for marker in markers:
            # Draw polygon
            pts = marker.corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(output, [pts], True, color, thickness)

            # Draw ID
            if show_id:
                center = (int(marker.center[0]), int(marker.center[1]))
                cv2.putText(
                    output,
                    str(marker.id),
                    center,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

        return FrameClass(
            data=output,
            timestamp=frame.timestamp,
            width=frame.width,
            height=frame.height,
            format=output_format,
            frame_number=frame.frame_number,
            exposure_time=frame.exposure_time,
            metadata=frame.metadata.copy(),
        )

    @staticmethod
    def generate_marker(
        dictionary: str | ArUcoDictionary,
        marker_id: int,
        size: int = 200,
        border_bits: int = 1,
    ) -> Frame:
        """Generate an ArUco marker image.

        Args:
            dictionary: ArUco dictionary to use.
            marker_id: Marker ID to generate.
            size: Output image size in pixels (square).
            border_bits: Width of black border in bits.

        Returns:
            Frame containing the marker image.

        Example:
            >>> marker_img = ArUcoDetector.generate_marker("DICT_4X4_50", 42, size=300)
            >>> # Save or display marker_img.data
        """
        from robo_infra.sensors.camera import Frame as FrameClass
        from robo_infra.sensors.camera import PixelFormat

        aruco = _get_aruco()

        aruco_dict = _get_aruco_dictionary(dictionary)
        marker_img = aruco.generateImageMarker(aruco_dict, marker_id, size, borderBits=border_bits)

        return FrameClass(
            data=marker_img,
            timestamp=time.monotonic(),
            width=size,
            height=size,
            format=PixelFormat.GRAY,
            frame_number=0,
        )


# =============================================================================
# AprilTag Detector
# =============================================================================


class AprilTagDetector:
    """AprilTag marker detector.

    AprilTag is a visual fiducial system similar to ArUco but with different
    encoding and typically better robustness to occlusion and lighting.

    This implementation uses OpenCV's AprilTag-compatible ArUco dictionaries
    for detection. For native AprilTag support with all features, consider
    using the apriltag Python package directly.

    Attributes:
        family: AprilTag family being used.

    Example:
        >>> detector = AprilTagDetector(family="tag36h11")
        >>> tags = detector.detect(frame)
        >>> for tag in tags:
        ...     print(f"Tag {tag.id} at {tag.center}")
    """

    # Supported AprilTag families (via OpenCV)
    FAMILIES: ClassVar[list[str]] = [
        "tag16h5",
        "tag25h9",
        "tag36h10",
        "tag36h11",
    ]

    def __init__(
        self,
        family: str = "tag36h11",
        *,
        simulation: bool | None = None,
    ) -> None:
        """Initialize AprilTag detector.

        Args:
            family: AprilTag family to detect:
                - "tag16h5": 16-bit, 30 unique tags
                - "tag25h9": 25-bit, 35 unique tags
                - "tag36h10": 36-bit, 2320 unique tags
                - "tag36h11": 36-bit, 587 unique tags (default, most robust)
            simulation: Override simulation mode.
        """
        self._simulation = simulation if simulation is not None else SIMULATION
        self._family = family.lower()

        # Map family name to OpenCV dictionary
        self._dict_map = {
            "tag16h5": "DICT_APRILTAG_16h5",
            "tag25h9": "DICT_APRILTAG_25h9",
            "tag36h10": "DICT_APRILTAG_36h10",
            "tag36h11": "DICT_APRILTAG_36h11",
        }

        if self._family not in self._dict_map:
            raise ValueError(
                f"Unknown AprilTag family '{family}'. Supported: {list(self._dict_map.keys())}"
            )

        # Use ArUcoDetector internally
        self._aruco_detector = ArUcoDetector(
            dictionary=self._dict_map[self._family],
            corner_refinement_method="apriltag",
            simulation=self._simulation,
        )

    @property
    def family(self) -> str:
        """Get AprilTag family name."""
        return self._family

    def detect(self, frame: Frame) -> list[AprilTag]:
        """Detect AprilTag markers in a frame.

        Args:
            frame: Input frame.

        Returns:
            List of detected AprilTag markers.
        """
        timestamp = time.monotonic()

        if self._simulation:
            logger.debug("AprilTagDetector.detect: simulation mode")
            return []

        # Use ArUco detector
        aruco_markers = self._aruco_detector.detect(frame)

        # Convert to AprilTag format
        tags = []
        for marker in aruco_markers:
            tag = AprilTag(
                id=marker.id,
                family=self._family,
                corners=marker.corners,
                center=marker.center,
                hamming=0,  # Not available via OpenCV
                decision_margin=1.0,  # Not available via OpenCV
                timestamp=timestamp,
            )
            tags.append(tag)

        return tags

    def estimate_pose(
        self,
        tag: AprilTag,
        intrinsics: CameraIntrinsics,
        tag_size: float,
    ) -> MarkerPose:
        """Estimate 6-DOF pose of a detected tag.

        Args:
            tag: Detected AprilTag.
            intrinsics: Camera intrinsic parameters.
            tag_size: Physical size of tag in meters.

        Returns:
            6-DOF pose of the tag.
        """
        # Convert to ArUco marker for pose estimation
        aruco_marker = ArUcoMarker(
            id=tag.id,
            corners=tag.corners,
            center=tag.center,
            timestamp=tag.timestamp,
        )

        return self._aruco_detector.estimate_pose(aruco_marker, intrinsics, tag_size)

    def draw_tags(
        self,
        frame: Frame,
        tags: list[AprilTag],
        *,
        color: tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
        show_id: bool = True,
    ) -> Frame:
        """Draw detected tags on a frame.

        Args:
            frame: Input frame.
            tags: List of detected tags.
            color: Drawing color (BGR).
            thickness: Line thickness.
            show_id: Whether to show tag IDs.

        Returns:
            Frame with tags drawn.
        """
        # Convert to ArUco markers
        aruco_markers = [
            ArUcoMarker(
                id=tag.id,
                corners=tag.corners,
                center=tag.center,
                timestamp=tag.timestamp,
            )
            for tag in tags
        ]

        return self._aruco_detector.draw_markers(
            frame,
            aruco_markers,
            color=color,
            thickness=thickness,
            show_id=show_id,
        )


# =============================================================================
# Utility Functions
# =============================================================================


def list_aruco_dictionaries() -> list[str]:
    """List available ArUco dictionaries.

    Returns:
        List of dictionary names.
    """
    return ArUcoDetector.DICTIONARIES


def list_apriltag_families() -> list[str]:
    """List available AprilTag families.

    Returns:
        List of family names.
    """
    return AprilTagDetector.FAMILIES


def create_marker_board(
    dictionary: str | ArUcoDictionary,
    rows: int,
    cols: int,
    marker_size: int = 100,
    marker_separation: int = 20,
    start_id: int = 0,
) -> Frame:
    """Create a board of ArUco markers for printing.

    Args:
        dictionary: ArUco dictionary to use.
        rows: Number of rows.
        cols: Number of columns.
        marker_size: Size of each marker in pixels.
        marker_separation: Separation between markers in pixels.
        start_id: Starting marker ID.

    Returns:
        Frame containing the marker board.

    Example:
        >>> board = create_marker_board("DICT_4X4_50", 4, 4, marker_size=150)
        >>> # Save board.data as PNG for printing
    """
    from robo_infra.sensors.camera import Frame as FrameClass
    from robo_infra.sensors.camera import PixelFormat

    # Calculate board size
    board_width = cols * marker_size + (cols - 1) * marker_separation
    board_height = rows * marker_size + (rows - 1) * marker_separation

    # Create white background
    board = np.ones((board_height, board_width), dtype=np.uint8) * 255

    # Place markers
    marker_id = start_id
    for row in range(rows):
        for col in range(cols):
            # Generate marker
            marker_frame = ArUcoDetector.generate_marker(dictionary, marker_id, marker_size)

            # Calculate position
            x = col * (marker_size + marker_separation)
            y = row * (marker_size + marker_separation)

            # Place marker on board
            board[y : y + marker_size, x : x + marker_size] = marker_frame.data

            marker_id += 1

    return FrameClass(
        data=board,
        timestamp=time.monotonic(),
        width=board_width,
        height=board_height,
        format=PixelFormat.GRAY,
        frame_number=0,
    )
