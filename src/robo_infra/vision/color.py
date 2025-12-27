"""Color-based object detection using HSV thresholding.

This module provides color detection utilities for finding objects based
on their color. It uses HSV (Hue, Saturation, Value) color space for more
robust detection under varying lighting conditions.

Example:
    >>> from robo_infra.vision.color import ColorDetector, ColorBlob
    >>> from robo_infra.sensors.cameras import USBCamera
    >>>
    >>> # Detect red objects
    >>> red_detector = ColorDetector.red()
    >>> with USBCamera() as camera:
    ...     frame = camera.capture()
    ...     blobs = red_detector.detect(frame)
    ...     for blob in blobs:
    ...         print(f"Red blob at {blob.center}, area={blob.area}")
    >>>
    >>> # Custom color range
    >>> orange_detector = ColorDetector(
    ...     lower_hsv=(5, 100, 100),
    ...     upper_hsv=(25, 255, 255),
    ...     name="orange",
    ... )
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar

import numpy as np


if TYPE_CHECKING:
    from numpy.typing import NDArray

    from robo_infra.sensors.camera import Frame

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
            "OpenCV (cv2) is required for color detection. "
            "Install with: pip install robo-infra[vision]"
        ) from e


def _has_cv2() -> bool:
    """Check if OpenCV is available."""
    try:
        import cv2  # noqa: F401

        return True
    except ImportError:
        return False


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class ColorRange:
    """HSV color range definition.

    Attributes:
        lower: Lower HSV bounds (h, s, v).
        upper: Upper HSV bounds (h, s, v).
        name: Optional color name.
    """

    lower: tuple[int, int, int]
    upper: tuple[int, int, int]
    name: str = ""

    def __post_init__(self) -> None:
        """Validate HSV values."""
        # OpenCV uses H: 0-179, S: 0-255, V: 0-255
        for i, (lower_val, upper_val) in enumerate(zip(self.lower, self.upper, strict=True)):
            max_val = 179 if i == 0 else 255
            if not (0 <= lower_val <= max_val) or not (0 <= upper_val <= max_val):
                channel = ["H", "S", "V"][i]
                raise ValueError(
                    f"Invalid {channel} range: ({lower_val}, {upper_val}). Must be 0-{max_val}"
                )

    def contains(self, hsv: tuple[int, int, int]) -> bool:
        """Check if an HSV color is within this range.

        Handles hue wraparound for red colors.

        Args:
            hsv: HSV color value.

        Returns:
            True if color is within range.
        """
        h, s, v = hsv

        # Check saturation and value (straightforward)
        if not (self.lower[1] <= s <= self.upper[1]):
            return False
        if not (self.lower[2] <= v <= self.upper[2]):
            return False

        # Handle hue wraparound
        if self.lower[0] <= self.upper[0]:
            # Normal case (e.g., green: 35-85)
            return self.lower[0] <= h <= self.upper[0]
        else:
            # Wraparound case (e.g., red: 170-10)
            return h >= self.lower[0] or h <= self.upper[0]

    def expand(
        self,
        h_delta: int = 0,
        s_delta: int = 0,
        v_delta: int = 0,
    ) -> ColorRange:
        """Create expanded color range.

        Args:
            h_delta: Amount to expand hue range.
            s_delta: Amount to expand saturation range.
            v_delta: Amount to expand value range.

        Returns:
            New expanded ColorRange.
        """
        return ColorRange(
            lower=(
                max(0, self.lower[0] - h_delta),
                max(0, self.lower[1] - s_delta),
                max(0, self.lower[2] - v_delta),
            ),
            upper=(
                min(179, self.upper[0] + h_delta),
                min(255, self.upper[1] + s_delta),
                min(255, self.upper[2] + v_delta),
            ),
            name=self.name,
        )


@dataclass(slots=True)
class ColorBlob:
    """Detected color blob/region.

    Attributes:
        center: Center point (x, y) in pixels.
        area: Blob area in pixels squared.
        bounding_box: Bounding box (x, y, width, height).
        contour: Contour points as Nx2 array.
        circularity: How circular the blob is (0-1, 1=perfect circle).
        aspect_ratio: Width/height ratio of bounding box.
        solidity: Contour area / convex hull area (0-1).
        color_name: Name of detected color.
        mean_hsv: Mean HSV values in the blob region.
        timestamp: Detection timestamp.
    """

    center: tuple[float, float]
    area: float
    bounding_box: tuple[int, int, int, int]
    contour: NDArray[np.int32]
    circularity: float = 0.0
    aspect_ratio: float = 1.0
    solidity: float = 1.0
    color_name: str = ""
    mean_hsv: tuple[float, float, float] | None = None
    timestamp: float = 0.0

    @property
    def width(self) -> int:
        """Bounding box width."""
        return self.bounding_box[2]

    @property
    def height(self) -> int:
        """Bounding box height."""
        return self.bounding_box[3]

    @property
    def x(self) -> int:
        """Bounding box left edge."""
        return self.bounding_box[0]

    @property
    def y(self) -> int:
        """Bounding box top edge."""
        return self.bounding_box[1]

    @property
    def radius(self) -> float:
        """Equivalent circle radius based on area."""
        return np.sqrt(self.area / np.pi)


# =============================================================================
# Predefined Color Ranges
# =============================================================================


# Standard color ranges (HSV, OpenCV format: H=0-179)
PREDEFINED_COLORS: dict[str, ColorRange | tuple[ColorRange, ColorRange]] = {
    # Red wraps around 0/180, so we need two ranges
    "red": (
        ColorRange((0, 100, 100), (10, 255, 255), "red"),
        ColorRange((160, 100, 100), (179, 255, 255), "red"),
    ),
    "orange": ColorRange((11, 100, 100), (25, 255, 255), "orange"),
    "yellow": ColorRange((26, 100, 100), (35, 255, 255), "yellow"),
    "green": ColorRange((36, 100, 100), (85, 255, 255), "green"),
    "cyan": ColorRange((86, 100, 100), (100, 255, 255), "cyan"),
    "blue": ColorRange((101, 100, 100), (130, 255, 255), "blue"),
    "purple": ColorRange((131, 100, 100), (145, 255, 255), "purple"),
    "pink": ColorRange((146, 50, 100), (165, 255, 255), "pink"),
    "white": ColorRange((0, 0, 200), (179, 30, 255), "white"),
    "black": ColorRange((0, 0, 0), (179, 255, 50), "black"),
    "gray": ColorRange((0, 0, 50), (179, 30, 200), "gray"),
}


# =============================================================================
# Color Detector
# =============================================================================


class ColorDetector:
    """Detect objects by color using HSV thresholding.

    ColorDetector finds contiguous regions (blobs) that match a specified
    HSV color range. This is useful for tracking colored objects, line
    following, or any task requiring color-based segmentation.

    Attributes:
        color_ranges: List of HSV color ranges to detect.
        name: Name of the color being detected.

    Example:
        >>> # Use predefined color
        >>> red_detector = ColorDetector.red()
        >>> blobs = red_detector.detect(frame)
        >>>
        >>> # Custom color range
        >>> detector = ColorDetector(
        ...     lower_hsv=(35, 100, 100),
        ...     upper_hsv=(85, 255, 255),
        ...     name="green",
        ... )
    """

    # Predefined color names
    COLORS: ClassVar[list[str]] = list(PREDEFINED_COLORS.keys())

    def __init__(
        self,
        lower_hsv: tuple[int, int, int] | None = None,
        upper_hsv: tuple[int, int, int] | None = None,
        *,
        color_ranges: list[ColorRange] | None = None,
        name: str = "custom",
        min_area: float = 100.0,
        max_area: float = float("inf"),
        min_circularity: float = 0.0,
        max_blobs: int = 100,
        blur_kernel: int = 5,
        morph_kernel: int = 5,
        simulation: bool | None = None,
    ) -> None:
        """Initialize color detector.

        Args:
            lower_hsv: Lower HSV bounds (h, s, v). H: 0-179, S/V: 0-255.
            upper_hsv: Upper HSV bounds (h, s, v).
            color_ranges: List of ColorRange objects (alternative to lower/upper).
            name: Name of the color being detected.
            min_area: Minimum blob area in pixels (filters noise).
            max_area: Maximum blob area in pixels.
            min_circularity: Minimum circularity (0-1) for blobs.
            max_blobs: Maximum number of blobs to return.
            blur_kernel: Gaussian blur kernel size (0 to disable).
            morph_kernel: Morphological operation kernel size (0 to disable).
            simulation: Override simulation mode.
        """
        self._simulation = simulation if simulation is not None else SIMULATION
        self._name = name
        self._min_area = min_area
        self._max_area = max_area
        self._min_circularity = min_circularity
        self._max_blobs = max_blobs
        self._blur_kernel = blur_kernel
        self._morph_kernel = morph_kernel

        # Build color ranges
        if color_ranges is not None:
            self._color_ranges = color_ranges
        elif lower_hsv is not None and upper_hsv is not None:
            self._color_ranges = [ColorRange(lower_hsv, upper_hsv, name)]
        else:
            raise ValueError("Either provide lower_hsv/upper_hsv or color_ranges")

    @property
    def name(self) -> str:
        """Get detector name."""
        return self._name

    @property
    def color_ranges(self) -> list[ColorRange]:
        """Get color ranges."""
        return self._color_ranges.copy()

    @classmethod
    def from_color(
        cls,
        color: str,
        *,
        min_area: float = 100.0,
        max_area: float = float("inf"),
        simulation: bool | None = None,
    ) -> ColorDetector:
        """Create detector for a predefined color.

        Args:
            color: Color name (red, green, blue, yellow, etc.).
            min_area: Minimum blob area.
            max_area: Maximum blob area.
            simulation: Override simulation mode.

        Returns:
            ColorDetector for the specified color.
        """
        color = color.lower()
        if color not in PREDEFINED_COLORS:
            raise ValueError(
                f"Unknown color '{color}'. Available: {list(PREDEFINED_COLORS.keys())}"
            )

        color_def = PREDEFINED_COLORS[color]
        color_ranges = list(color_def) if isinstance(color_def, tuple) else [color_def]

        return cls(
            color_ranges=color_ranges,
            name=color,
            min_area=min_area,
            max_area=max_area,
            simulation=simulation,
        )

    # Convenience class methods for common colors
    @classmethod
    def red(cls, **kwargs) -> ColorDetector:
        """Create red color detector."""
        return cls.from_color("red", **kwargs)

    @classmethod
    def green(cls, **kwargs) -> ColorDetector:
        """Create green color detector."""
        return cls.from_color("green", **kwargs)

    @classmethod
    def blue(cls, **kwargs) -> ColorDetector:
        """Create blue color detector."""
        return cls.from_color("blue", **kwargs)

    @classmethod
    def yellow(cls, **kwargs) -> ColorDetector:
        """Create yellow color detector."""
        return cls.from_color("yellow", **kwargs)

    @classmethod
    def orange(cls, **kwargs) -> ColorDetector:
        """Create orange color detector."""
        return cls.from_color("orange", **kwargs)

    @classmethod
    def purple(cls, **kwargs) -> ColorDetector:
        """Create purple color detector."""
        return cls.from_color("purple", **kwargs)

    @classmethod
    def pink(cls, **kwargs) -> ColorDetector:
        """Create pink color detector."""
        return cls.from_color("pink", **kwargs)

    @classmethod
    def cyan(cls, **kwargs) -> ColorDetector:
        """Create cyan color detector."""
        return cls.from_color("cyan", **kwargs)

    @classmethod
    def white(cls, **kwargs) -> ColorDetector:
        """Create white color detector."""
        return cls.from_color("white", **kwargs)

    @classmethod
    def black(cls, **kwargs) -> ColorDetector:
        """Create black color detector."""
        return cls.from_color("black", **kwargs)

    def detect(self, frame: Frame) -> list[ColorBlob]:
        """Detect color blobs in a frame.

        Args:
            frame: Input frame (RGB or BGR).

        Returns:
            List of detected color blobs, sorted by area (largest first).

        Example:
            >>> blobs = detector.detect(frame)
            >>> if blobs:
            ...     largest = blobs[0]
            ...     print(f"Largest blob at {largest.center}")
        """
        from robo_infra.sensors.camera import PixelFormat

        timestamp = time.monotonic()

        if self._simulation:
            logger.debug("ColorDetector.detect: simulation mode")
            return []

        cv2 = _get_cv2()

        # Convert to BGR if needed
        if frame.format == PixelFormat.RGB:
            bgr = cv2.cvtColor(frame.data, cv2.COLOR_RGB2BGR)
        elif frame.format == PixelFormat.BGR:
            bgr = frame.data.copy()
        elif frame.format == PixelFormat.GRAY:
            # Can't detect colors in grayscale
            logger.warning("ColorDetector.detect: frame is grayscale")
            return []
        else:
            # Try converting via RGB
            rgb_frame = frame.to_rgb()
            bgr = cv2.cvtColor(rgb_frame.data, cv2.COLOR_RGB2BGR)

        # Apply blur for noise reduction
        if self._blur_kernel > 0:
            kernel_size = self._blur_kernel | 1  # Ensure odd
            bgr = cv2.GaussianBlur(bgr, (kernel_size, kernel_size), 0)

        # Convert to HSV
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Create combined mask from all color ranges
        combined_mask = np.zeros((frame.height, frame.width), dtype=np.uint8)

        for color_range in self._color_ranges:
            lower = np.array(color_range.lower, dtype=np.uint8)
            upper = np.array(color_range.upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Apply morphological operations to clean up mask
        if self._morph_kernel > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self._morph_kernel, self._morph_kernel)
            )
            # Opening removes small noise
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            # Closing fills small holes
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process contours into blobs
        blobs = []
        for contour in contours:
            area = cv2.contourArea(contour)

            # Filter by area
            if area < self._min_area or area > self._max_area:
                continue

            # Calculate properties
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0

            # Filter by circularity
            if circularity < self._min_circularity:
                continue

            # Bounding box
            x, y, w, h = cv2.boundingRect(contour)

            # Moments for centroid
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
            else:
                cx = x + w / 2
                cy = y + h / 2

            # Aspect ratio
            aspect_ratio = w / h if h > 0 else 1.0

            # Solidity
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0.0

            # Mean HSV in blob region
            mask_blob = np.zeros((frame.height, frame.width), dtype=np.uint8)
            cv2.drawContours(mask_blob, [contour], -1, 255, -1)
            mean_hsv_val = cv2.mean(hsv, mask=mask_blob)[:3]

            blob = ColorBlob(
                center=(float(cx), float(cy)),
                area=float(area),
                bounding_box=(x, y, w, h),
                contour=contour.reshape(-1, 2),
                circularity=float(circularity),
                aspect_ratio=float(aspect_ratio),
                solidity=float(solidity),
                color_name=self._name,
                mean_hsv=(
                    float(mean_hsv_val[0]),
                    float(mean_hsv_val[1]),
                    float(mean_hsv_val[2]),
                ),
                timestamp=timestamp,
            )
            blobs.append(blob)

        # Sort by area (largest first) and limit
        blobs.sort(key=lambda b: b.area, reverse=True)
        blobs = blobs[: self._max_blobs]

        logger.debug(f"ColorDetector.detect: found {len(blobs)} {self._name} blobs")
        return blobs

    def get_mask(self, frame: Frame) -> Frame:
        """Get the binary mask for this color in a frame.

        Useful for visualization or custom processing.

        Args:
            frame: Input frame.

        Returns:
            Binary mask frame (white where color is detected).
        """
        from robo_infra.sensors.camera import Frame as FrameClass
        from robo_infra.sensors.camera import PixelFormat

        if self._simulation:
            return FrameClass(
                data=np.zeros((frame.height, frame.width), dtype=np.uint8),
                timestamp=frame.timestamp,
                width=frame.width,
                height=frame.height,
                format=PixelFormat.GRAY,
                frame_number=frame.frame_number,
            )

        cv2 = _get_cv2()

        # Convert to BGR if needed
        if frame.format == PixelFormat.RGB:
            bgr = cv2.cvtColor(frame.data, cv2.COLOR_RGB2BGR)
        elif frame.format == PixelFormat.BGR:
            bgr = frame.data
        else:
            rgb_frame = frame.to_rgb()
            bgr = cv2.cvtColor(rgb_frame.data, cv2.COLOR_RGB2BGR)

        # Convert to HSV
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Create combined mask
        combined_mask = np.zeros((frame.height, frame.width), dtype=np.uint8)

        for color_range in self._color_ranges:
            lower = np.array(color_range.lower, dtype=np.uint8)
            upper = np.array(color_range.upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        return FrameClass(
            data=combined_mask,
            timestamp=frame.timestamp,
            width=frame.width,
            height=frame.height,
            format=PixelFormat.GRAY,
            frame_number=frame.frame_number,
        )

    def draw_blobs(
        self,
        frame: Frame,
        blobs: list[ColorBlob],
        *,
        draw_contours: bool = True,
        draw_bounding_box: bool = False,
        draw_center: bool = True,
        draw_label: bool = True,
        contour_color: tuple[int, int, int] = (0, 255, 0),
        box_color: tuple[int, int, int] = (255, 0, 0),
        center_color: tuple[int, int, int] = (0, 0, 255),
        thickness: int = 2,
    ) -> Frame:
        """Draw detected blobs on a frame.

        Args:
            frame: Input frame.
            blobs: List of detected blobs.
            draw_contours: Whether to draw blob contours.
            draw_bounding_box: Whether to draw bounding boxes.
            draw_center: Whether to draw center points.
            draw_label: Whether to draw labels with area.
            contour_color: Color for contours (BGR).
            box_color: Color for bounding boxes (BGR).
            center_color: Color for center points (BGR).
            thickness: Line thickness.

        Returns:
            Frame with blobs drawn.
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
            # Convert colors from BGR to RGB
            contour_color = contour_color[::-1]
            box_color = box_color[::-1]
            center_color = center_color[::-1]
        else:
            output = frame.data.copy()
            output_format = frame.format

        for blob in blobs:
            # Draw contour
            if draw_contours:
                contour = blob.contour.reshape(-1, 1, 2).astype(np.int32)
                cv2.drawContours(output, [contour], -1, contour_color, thickness)

            # Draw bounding box
            if draw_bounding_box:
                x, y, w, h = blob.bounding_box
                cv2.rectangle(output, (x, y), (x + w, y + h), box_color, thickness)

            # Draw center
            if draw_center:
                center = (int(blob.center[0]), int(blob.center[1]))
                cv2.circle(output, center, 5, center_color, -1)

            # Draw label
            if draw_label:
                label = f"{blob.color_name}: {int(blob.area)}"
                pos = (int(blob.center[0]) - 20, int(blob.center[1]) - 10)
                cv2.putText(
                    output,
                    label,
                    pos,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    contour_color,
                    1,
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


# =============================================================================
# Color Space Utilities
# =============================================================================


def rgb_to_hsv(r: int, g: int, b: int) -> tuple[int, int, int]:
    """Convert RGB to HSV (OpenCV format).

    Args:
        r: Red value (0-255).
        g: Green value (0-255).
        b: Blue value (0-255).

    Returns:
        HSV tuple (h: 0-179, s: 0-255, v: 0-255).
    """
    cv2 = _get_cv2()

    # Create a 1x1 BGR pixel
    bgr = np.array([[[b, g, r]]], dtype=np.uint8)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    return (int(hsv[0, 0, 0]), int(hsv[0, 0, 1]), int(hsv[0, 0, 2]))


def hsv_to_rgb(h: int, s: int, v: int) -> tuple[int, int, int]:
    """Convert HSV (OpenCV format) to RGB.

    Args:
        h: Hue value (0-179).
        s: Saturation value (0-255).
        v: Value (0-255).

    Returns:
        RGB tuple (r, g, b) each 0-255.
    """
    cv2 = _get_cv2()

    # Create a 1x1 HSV pixel
    hsv = np.array([[[h, s, v]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    return (int(bgr[0, 0, 2]), int(bgr[0, 0, 1]), int(bgr[0, 0, 0]))


def get_dominant_color(
    frame: Frame,
    k: int = 3,
    ignore_white: bool = True,
    ignore_black: bool = True,
) -> tuple[int, int, int]:
    """Get the dominant color in a frame using k-means clustering.

    Args:
        frame: Input frame (RGB or BGR).
        k: Number of color clusters to use.
        ignore_white: Ignore very bright pixels.
        ignore_black: Ignore very dark pixels.

    Returns:
        Dominant color as RGB tuple.

    Example:
        >>> dominant = get_dominant_color(frame)
        >>> print(f"Dominant color: RGB{dominant}")
    """
    from robo_infra.sensors.camera import PixelFormat

    cv2 = _get_cv2()

    # Convert to RGB
    if frame.format == PixelFormat.RGB:
        rgb = frame.data
    elif frame.format == PixelFormat.BGR:
        rgb = cv2.cvtColor(frame.data, cv2.COLOR_BGR2RGB)
    else:
        rgb_frame = frame.to_rgb()
        rgb = rgb_frame.data

    # Reshape to list of pixels
    pixels = rgb.reshape(-1, 3).astype(np.float32)

    # Filter out white and black
    if ignore_white or ignore_black:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        hsv_pixels = hsv.reshape(-1, 3)

        mask = np.ones(len(pixels), dtype=bool)
        if ignore_white:
            mask &= hsv_pixels[:, 1] > 30  # Minimum saturation
        if ignore_black:
            mask &= hsv_pixels[:, 2] > 50  # Minimum value

        pixels = pixels[mask]

    if len(pixels) < k:
        # Not enough pixels after filtering
        return (128, 128, 128)

    # K-means clustering
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    _, labels, centers = cv2.kmeans(pixels, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    # Find most common cluster
    unique, counts = np.unique(labels, return_counts=True)
    dominant_idx = unique[np.argmax(counts)]
    dominant_color = centers[dominant_idx]

    return (int(dominant_color[0]), int(dominant_color[1]), int(dominant_color[2]))


def sample_color_at_point(
    frame: Frame,
    point: tuple[int, int],
    radius: int = 3,
) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    """Sample color at a point with averaging over a region.

    Args:
        frame: Input frame.
        point: Point (x, y) to sample.
        radius: Radius around point to average.

    Returns:
        Tuple of (RGB, HSV) color values.
    """
    from robo_infra.sensors.camera import PixelFormat

    cv2 = _get_cv2()

    x, y = point

    # Ensure within bounds
    x1 = max(0, x - radius)
    y1 = max(0, y - radius)
    x2 = min(frame.width, x + radius + 1)
    y2 = min(frame.height, y + radius + 1)

    # Extract region
    if frame.format == PixelFormat.RGB:
        rgb_region = frame.data[y1:y2, x1:x2]
    elif frame.format == PixelFormat.BGR:
        bgr_region = frame.data[y1:y2, x1:x2]
        rgb_region = cv2.cvtColor(bgr_region, cv2.COLOR_BGR2RGB)
    else:
        rgb_frame = frame.to_rgb()
        rgb_region = rgb_frame.data[y1:y2, x1:x2]

    # Average color
    mean_rgb = np.mean(rgb_region, axis=(0, 1))
    r, g, b = int(mean_rgb[0]), int(mean_rgb[1]), int(mean_rgb[2])

    # Convert to HSV
    h, s, v = rgb_to_hsv(r, g, b)

    return ((r, g, b), (h, s, v))


def auto_color_range(
    frame: Frame,
    point: tuple[int, int],
    radius: int = 10,
    h_tolerance: int = 10,
    s_tolerance: int = 50,
    v_tolerance: int = 50,
) -> ColorRange:
    """Automatically determine color range from a sample point.

    Useful for interactive color selection.

    Args:
        frame: Input frame.
        point: Sample point (x, y).
        radius: Radius around point to sample.
        h_tolerance: Hue tolerance.
        s_tolerance: Saturation tolerance.
        v_tolerance: Value tolerance.

    Returns:
        ColorRange centered on the sampled color.

    Example:
        >>> # User clicks at (320, 240)
        >>> color_range = auto_color_range(frame, (320, 240))
        >>> detector = ColorDetector(color_ranges=[color_range])
    """
    _, (h, s, v) = sample_color_at_point(frame, point, radius)

    lower = (
        max(0, h - h_tolerance),
        max(0, s - s_tolerance),
        max(0, v - v_tolerance),
    )
    upper = (
        min(179, h + h_tolerance),
        min(255, s + s_tolerance),
        min(255, v + v_tolerance),
    )

    return ColorRange(lower, upper, "sampled")


def list_available_colors() -> list[str]:
    """List available predefined colors.

    Returns:
        List of color names.
    """
    return list(PREDEFINED_COLORS.keys())
