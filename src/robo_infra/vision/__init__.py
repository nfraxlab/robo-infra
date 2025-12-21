"""Vision utilities for computer vision processing.

This package provides image processing, marker detection, and color
detection utilities for robotics applications.

Modules:
    processing: Image processing functions (resize, crop, blur, edge_detect)
    markers: Fiducial marker detection (ArUco, AprilTag)
    color: Color-based object detection (HSV thresholding)

Example:
    >>> from robo_infra.vision import (
    ...     resize, crop, blur, edge_detect, to_grayscale,
    ...     ArUcoDetector, AprilTagDetector,
    ...     ColorDetector, ColorBlob,
    ... )
    >>>
    >>> # Image processing
    >>> resized = resize(frame, (320, 240))
    >>> edges = edge_detect(frame, method="canny")
    >>>
    >>> # Marker detection
    >>> detector = ArUcoDetector()
    >>> markers = detector.detect(frame)
    >>> for marker in markers:
    ...     print(f"Marker {marker.id} at {marker.center}")
    >>>
    >>> # Color detection
    >>> red_detector = ColorDetector.red()
    >>> blobs = red_detector.detect(frame)
"""

from __future__ import annotations

from robo_infra.vision.color import (
    ColorBlob,
    ColorDetector,
    ColorRange,
)
from robo_infra.vision.markers import (
    AprilTag,
    AprilTagDetector,
    ArUcoDetector,
    ArUcoMarker,
    MarkerPose,
)
from robo_infra.vision.processing import (
    bilateral_filter,
    blur,
    canny_edge,
    crop,
    dilate,
    edge_detect,
    erode,
    histogram_equalize,
    laplacian_edge,
    morphology,
    resize,
    rotate,
    sobel_edge,
    threshold,
    to_grayscale,
)


__all__ = [
    "AprilTag",
    "AprilTagDetector",
    "ArUcoDetector",
    "ArUcoMarker",
    "ColorBlob",
    "ColorDetector",
    "ColorRange",
    "MarkerPose",
    "bilateral_filter",
    "blur",
    "canny_edge",
    "crop",
    "dilate",
    "edge_detect",
    "erode",
    "histogram_equalize",
    "laplacian_edge",
    "morphology",
    "resize",
    "rotate",
    "sobel_edge",
    "threshold",
    "to_grayscale",
]
