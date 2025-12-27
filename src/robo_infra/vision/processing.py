"""Image processing utilities for computer vision.

This module provides common image processing functions that wrap OpenCV
operations while maintaining a consistent interface with robo-infra Frames.

Functions are designed to be lightweight wrappers around cv2, not duplicating
functionality but providing a more ergonomic API for robotics applications.

Example:
    >>> from robo_infra.vision.processing import (
    ...     resize, crop, rotate, blur, edge_detect, to_grayscale
    ... )
    >>> from robo_infra.sensors.camera import Frame
    >>>
    >>> # Resize frame
    >>> small = resize(frame, (320, 240))
    >>>
    >>> # Crop region of interest
    >>> roi = crop(frame, (100, 100, 200, 200))  # x, y, width, height
    >>>
    >>> # Rotate frame
    >>> rotated = rotate(frame, 45.0)
    >>>
    >>> # Apply blur
    >>> blurred = blur(frame, kernel_size=5)
    >>>
    >>> # Edge detection
    >>> edges = edge_detect(frame, method="canny")
"""

from __future__ import annotations

import logging
import os
import time
from typing import Literal

import numpy as np

from robo_infra.sensors.camera import Frame, PixelFormat


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
            "OpenCV (cv2) is required for vision processing. "
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
# Basic Transformations
# =============================================================================


def resize(
    frame: Frame,
    size: tuple[int, int],
    *,
    interpolation: str = "linear",
) -> Frame:
    """Resize a frame to a new size.

    Args:
        frame: Input frame to resize.
        size: Target size as (width, height).
        interpolation: Interpolation method:
            - "nearest": Nearest neighbor (fast, blocky)
            - "linear": Bilinear interpolation (default)
            - "cubic": Bicubic interpolation (smooth, slower)
            - "area": Area averaging (good for downscaling)
            - "lanczos": Lanczos interpolation (highest quality)

    Returns:
        Resized frame.

    Example:
        >>> small = resize(frame, (320, 240))
        >>> upscaled = resize(frame, (1280, 720), interpolation="lanczos")
    """
    new_width, new_height = size

    if new_width <= 0 or new_height <= 0:
        raise ValueError("Size dimensions must be positive")

    if new_width == frame.width and new_height == frame.height:
        return frame.copy()

    cv2 = _get_cv2()

    # Map interpolation names to cv2 constants
    interp_map = {
        "nearest": cv2.INTER_NEAREST,
        "linear": cv2.INTER_LINEAR,
        "cubic": cv2.INTER_CUBIC,
        "area": cv2.INTER_AREA,
        "lanczos": cv2.INTER_LANCZOS4,
    }

    if interpolation not in interp_map:
        raise ValueError(
            f"Unknown interpolation '{interpolation}'. Choose from: {list(interp_map.keys())}"
        )

    interp_flag = interp_map[interpolation]
    resized_data = cv2.resize(frame.data, (new_width, new_height), interpolation=interp_flag)

    return Frame(
        data=resized_data,
        timestamp=frame.timestamp,
        width=new_width,
        height=new_height,
        format=frame.format,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def crop(
    frame: Frame,
    roi: tuple[int, int, int, int],
) -> Frame:
    """Crop a region of interest from a frame.

    Args:
        frame: Input frame to crop.
        roi: Region of interest as (x, y, width, height).
            - x: Left edge of crop region
            - y: Top edge of crop region
            - width: Width of crop region
            - height: Height of crop region

    Returns:
        Cropped frame.

    Raises:
        ValueError: If crop region is invalid or out of bounds.

    Example:
        >>> roi = crop(frame, (100, 100, 200, 200))
    """
    x, y, width, height = roi

    if x < 0 or y < 0:
        raise ValueError("Crop coordinates must be non-negative")
    if width <= 0 or height <= 0:
        raise ValueError("Crop dimensions must be positive")
    if x + width > frame.width or y + height > frame.height:
        raise ValueError(
            f"Crop region ({x}, {y}, {width}, {height}) exceeds frame bounds "
            f"({frame.width}, {frame.height})"
        )

    # Use Frame's built-in crop method
    return frame.crop(x, y, width, height)


def rotate(
    frame: Frame,
    angle: float,
    *,
    center: tuple[float, float] | None = None,
    scale: float = 1.0,
    border_mode: str = "constant",
    border_value: tuple[int, int, int] = (0, 0, 0),
) -> Frame:
    """Rotate a frame by a given angle.

    Args:
        frame: Input frame to rotate.
        angle: Rotation angle in degrees (positive = counter-clockwise).
        center: Center of rotation (x, y). Defaults to frame center.
        scale: Optional scale factor during rotation.
        border_mode: How to handle pixels outside the original image:
            - "constant": Fill with border_value (default)
            - "replicate": Replicate edge pixels
            - "reflect": Reflect across edge
            - "wrap": Wrap around
        border_value: RGB color value for constant border mode.

    Returns:
        Rotated frame.

    Example:
        >>> rotated = rotate(frame, 45.0)
        >>> rotated = rotate(frame, 90.0, center=(320, 240))
    """
    cv2 = _get_cv2()

    # Default to center of frame
    if center is None:
        center = (frame.width / 2, frame.height / 2)

    # Map border mode
    border_map = {
        "constant": cv2.BORDER_CONSTANT,
        "replicate": cv2.BORDER_REPLICATE,
        "reflect": cv2.BORDER_REFLECT,
        "wrap": cv2.BORDER_WRAP,
    }

    if border_mode not in border_map:
        raise ValueError(
            f"Unknown border_mode '{border_mode}'. Choose from: {list(border_map.keys())}"
        )

    border_flag = border_map[border_mode]

    # Create rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, scale)

    # Apply rotation
    rotated_data = cv2.warpAffine(
        frame.data,
        rotation_matrix,
        (frame.width, frame.height),
        borderMode=border_flag,
        borderValue=border_value,
    )

    return Frame(
        data=rotated_data,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=frame.format,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def to_grayscale(frame: Frame) -> Frame:
    """Convert a frame to grayscale.

    Uses the Frame's built-in conversion if available, otherwise
    uses OpenCV for format-specific conversions.

    Args:
        frame: Input frame in any format.

    Returns:
        Grayscale frame with single channel.

    Example:
        >>> gray = to_grayscale(frame)
        >>> print(gray.channels)  # 1
    """
    if frame.format == PixelFormat.GRAY:
        return frame.copy()

    # Use Frame's built-in method if it supports the format
    if frame.format in (PixelFormat.RGB, PixelFormat.BGR):
        return frame.to_grayscale()

    # Use OpenCV for other formats
    cv2 = _get_cv2()

    if frame.format == PixelFormat.RGBA:
        gray_data = cv2.cvtColor(frame.data, cv2.COLOR_RGBA2GRAY)
    elif frame.format == PixelFormat.BGRA:
        gray_data = cv2.cvtColor(frame.data, cv2.COLOR_BGRA2GRAY)
    elif frame.format == PixelFormat.YUV:
        # Y channel is already grayscale
        gray_data = frame.data[:, :, 0].copy()
    else:
        # Try RGB conversion as fallback
        rgb_frame = frame.to_rgb()
        return to_grayscale(rgb_frame)

    return Frame(
        data=gray_data,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=PixelFormat.GRAY,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


# =============================================================================
# Thresholding
# =============================================================================


def threshold(
    frame: Frame,
    value: int,
    *,
    thresh_type: Literal[
        "binary",
        "binary_inv",
        "trunc",
        "tozero",
        "tozero_inv",
        "otsu",
        "adaptive_mean",
        "adaptive_gaussian",
    ] = "binary",
    max_value: int = 255,
    block_size: int = 11,
    c: int = 2,
) -> Frame:
    """Apply thresholding to a frame.

    For adaptive thresholding methods, the frame is converted to grayscale first.

    Args:
        frame: Input frame (color or grayscale).
        value: Threshold value (ignored for otsu and adaptive methods).
        thresh_type: Type of thresholding:
            - "binary": dst = max_value if src > value else 0
            - "binary_inv": dst = 0 if src > value else max_value
            - "trunc": dst = value if src > value else src
            - "tozero": dst = src if src > value else 0
            - "tozero_inv": dst = 0 if src > value else src
            - "otsu": Automatic threshold using Otsu's method
            - "adaptive_mean": Local threshold based on mean
            - "adaptive_gaussian": Local threshold based on weighted sum
        max_value: Maximum value for binary thresholding (default 255).
        block_size: Size of pixel neighborhood for adaptive methods (odd number).
        c: Constant subtracted from mean in adaptive methods.

    Returns:
        Thresholded frame in grayscale.

    Example:
        >>> binary = threshold(gray_frame, 128)
        >>> otsu = threshold(gray_frame, 0, thresh_type="otsu")
        >>> adaptive = threshold(frame, 0, thresh_type="adaptive_gaussian")
    """
    cv2 = _get_cv2()

    # Convert to grayscale if needed
    gray_frame = to_grayscale(frame) if frame.format != PixelFormat.GRAY else frame

    # Standard threshold types
    type_map = {
        "binary": cv2.THRESH_BINARY,
        "binary_inv": cv2.THRESH_BINARY_INV,
        "trunc": cv2.THRESH_TRUNC,
        "tozero": cv2.THRESH_TOZERO,
        "tozero_inv": cv2.THRESH_TOZERO_INV,
    }

    if thresh_type == "otsu":
        _, result = cv2.threshold(
            gray_frame.data,
            0,
            max_value,
            cv2.THRESH_BINARY | cv2.THRESH_OTSU,
        )
    elif thresh_type == "adaptive_mean":
        result = cv2.adaptiveThreshold(
            gray_frame.data,
            max_value,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            block_size,
            c,
        )
    elif thresh_type == "adaptive_gaussian":
        result = cv2.adaptiveThreshold(
            gray_frame.data,
            max_value,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block_size,
            c,
        )
    elif thresh_type in type_map:
        _, result = cv2.threshold(
            gray_frame.data,
            value,
            max_value,
            type_map[thresh_type],
        )
    else:
        all_types = [*type_map.keys(), "otsu", "adaptive_mean", "adaptive_gaussian"]
        raise ValueError(f"Unknown thresh_type '{thresh_type}'. Choose from: {all_types}")

    return Frame(
        data=result,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=PixelFormat.GRAY,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


# =============================================================================
# Blurring and Filtering
# =============================================================================


def blur(
    frame: Frame,
    kernel_size: int = 5,
    *,
    method: Literal["average", "gaussian", "median"] = "gaussian",
    sigma: float = 0.0,
) -> Frame:
    """Apply blur to a frame.

    Args:
        frame: Input frame.
        kernel_size: Size of the blur kernel (must be odd for gaussian/median).
        method: Blur method:
            - "average": Simple box filter (fast)
            - "gaussian": Gaussian blur (default, smoothest)
            - "median": Median filter (good for salt-and-pepper noise)
        sigma: Gaussian standard deviation (0 = auto-calculate from kernel size).

    Returns:
        Blurred frame.

    Example:
        >>> blurred = blur(frame, kernel_size=5)
        >>> denoised = blur(frame, kernel_size=5, method="median")
    """
    cv2 = _get_cv2()

    if kernel_size <= 0:
        raise ValueError("Kernel size must be positive")

    # Ensure odd kernel size for gaussian and median
    if method in ("gaussian", "median") and kernel_size % 2 == 0:
        kernel_size += 1

    if method == "average":
        blurred = cv2.blur(frame.data, (kernel_size, kernel_size))
    elif method == "gaussian":
        blurred = cv2.GaussianBlur(frame.data, (kernel_size, kernel_size), sigma)
    elif method == "median":
        blurred = cv2.medianBlur(frame.data, kernel_size)
    else:
        raise ValueError(f"Unknown blur method '{method}'. Choose from: average, gaussian, median")

    return Frame(
        data=blurred,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=frame.format,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def bilateral_filter(
    frame: Frame,
    d: int = 9,
    sigma_color: float = 75.0,
    sigma_space: float = 75.0,
) -> Frame:
    """Apply bilateral filter for edge-preserving smoothing.

    Bilateral filter smooths the image while keeping edges sharp.
    Good for denoising while preserving features.

    Args:
        frame: Input frame.
        d: Diameter of each pixel neighborhood (use -1 for auto).
        sigma_color: Filter sigma in the color space.
        sigma_space: Filter sigma in the coordinate space.

    Returns:
        Filtered frame.

    Example:
        >>> smooth = bilateral_filter(frame, d=9, sigma_color=75)
    """
    cv2 = _get_cv2()

    filtered = cv2.bilateralFilter(frame.data, d, sigma_color, sigma_space)

    return Frame(
        data=filtered,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=frame.format,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


# =============================================================================
# Edge Detection
# =============================================================================


def edge_detect(
    frame: Frame,
    *,
    method: Literal["canny", "sobel", "laplacian", "scharr"] = "canny",
    threshold1: float = 100.0,
    threshold2: float = 200.0,
    ksize: int = 3,
) -> Frame:
    """Detect edges in a frame.

    Args:
        frame: Input frame.
        method: Edge detection method:
            - "canny": Canny edge detector (default, multi-stage)
            - "sobel": Sobel gradient (magnitude of x and y)
            - "laplacian": Laplacian of Gaussian
            - "scharr": Scharr gradient (more accurate than Sobel)
        threshold1: Lower threshold for Canny hysteresis.
        threshold2: Upper threshold for Canny hysteresis.
        ksize: Kernel size for Sobel/Laplacian (1, 3, 5, or 7).

    Returns:
        Edge-detected frame in grayscale.

    Example:
        >>> edges = edge_detect(frame, method="canny")
        >>> sobel = edge_detect(frame, method="sobel")
    """
    cv2 = _get_cv2()

    # Convert to grayscale first
    gray = to_grayscale(frame) if frame.format != PixelFormat.GRAY else frame

    if method == "canny":
        edges = cv2.Canny(
            gray.data,
            threshold1,
            threshold2,
            apertureSize=ksize,
        )
    elif method == "sobel":
        grad_x = cv2.Sobel(gray.data, cv2.CV_64F, 1, 0, ksize=ksize)
        grad_y = cv2.Sobel(gray.data, cv2.CV_64F, 0, 1, ksize=ksize)
        magnitude = np.sqrt(grad_x**2 + grad_y**2)
        edges = np.clip(magnitude, 0, 255).astype(np.uint8)
    elif method == "laplacian":
        laplacian = cv2.Laplacian(gray.data, cv2.CV_64F, ksize=ksize)
        edges = np.clip(np.abs(laplacian), 0, 255).astype(np.uint8)
    elif method == "scharr":
        grad_x = cv2.Scharr(gray.data, cv2.CV_64F, 1, 0)
        grad_y = cv2.Scharr(gray.data, cv2.CV_64F, 0, 1)
        magnitude = np.sqrt(grad_x**2 + grad_y**2)
        edges = np.clip(magnitude, 0, 255).astype(np.uint8)
    else:
        raise ValueError(
            f"Unknown edge detection method '{method}'. "
            f"Choose from: canny, sobel, laplacian, scharr"
        )

    return Frame(
        data=edges,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=PixelFormat.GRAY,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def canny_edge(
    frame: Frame,
    threshold1: float = 100.0,
    threshold2: float = 200.0,
    *,
    aperture_size: int = 3,
    l2_gradient: bool = False,
) -> Frame:
    """Apply Canny edge detection with full parameter control.

    Args:
        frame: Input frame.
        threshold1: Lower threshold for hysteresis.
        threshold2: Upper threshold for hysteresis.
        aperture_size: Aperture size for Sobel operator (3, 5, or 7).
        l2_gradient: Use L2 norm for gradient magnitude (slower but more accurate).

    Returns:
        Binary edge image.

    Example:
        >>> edges = canny_edge(frame, 50, 150)
    """
    cv2 = _get_cv2()

    gray = to_grayscale(frame) if frame.format != PixelFormat.GRAY else frame

    edges = cv2.Canny(
        gray.data,
        threshold1,
        threshold2,
        apertureSize=aperture_size,
        L2gradient=l2_gradient,
    )

    return Frame(
        data=edges,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=PixelFormat.GRAY,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def sobel_edge(
    frame: Frame,
    *,
    dx: int = 1,
    dy: int = 1,
    ksize: int = 3,
    scale: float = 1.0,
) -> Frame:
    """Apply Sobel edge detection.

    Args:
        frame: Input frame.
        dx: Order of derivative in x direction.
        dy: Order of derivative in y direction.
        ksize: Size of extended Sobel kernel (1, 3, 5, or 7).
        scale: Scale factor for computed derivative values.

    Returns:
        Gradient magnitude image.
    """
    cv2 = _get_cv2()

    gray = to_grayscale(frame) if frame.format != PixelFormat.GRAY else frame

    if dx > 0:
        grad_x = cv2.Sobel(gray.data, cv2.CV_64F, dx, 0, ksize=ksize, scale=scale)
    else:
        grad_x = np.zeros_like(gray.data, dtype=np.float64)

    if dy > 0:
        grad_y = cv2.Sobel(gray.data, cv2.CV_64F, 0, dy, ksize=ksize, scale=scale)
    else:
        grad_y = np.zeros_like(gray.data, dtype=np.float64)

    magnitude = np.sqrt(grad_x**2 + grad_y**2)
    edges = np.clip(magnitude, 0, 255).astype(np.uint8)

    return Frame(
        data=edges,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=PixelFormat.GRAY,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def laplacian_edge(
    frame: Frame,
    *,
    ksize: int = 3,
    scale: float = 1.0,
) -> Frame:
    """Apply Laplacian edge detection.

    Args:
        frame: Input frame.
        ksize: Aperture size for computing second-derivative filters.
        scale: Scale factor for computed Laplacian values.

    Returns:
        Edge-detected frame.
    """
    cv2 = _get_cv2()

    gray = to_grayscale(frame) if frame.format != PixelFormat.GRAY else frame

    laplacian = cv2.Laplacian(gray.data, cv2.CV_64F, ksize=ksize, scale=scale)
    edges = np.clip(np.abs(laplacian), 0, 255).astype(np.uint8)

    return Frame(
        data=edges,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=PixelFormat.GRAY,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


# =============================================================================
# Morphological Operations
# =============================================================================


def erode(
    frame: Frame,
    kernel_size: int = 3,
    *,
    iterations: int = 1,
    kernel_shape: Literal["rect", "ellipse", "cross"] = "rect",
) -> Frame:
    """Erode the foreground (shrink white regions).

    Args:
        frame: Input frame (binary or grayscale recommended).
        kernel_size: Size of the structuring element.
        iterations: Number of erosion iterations.
        kernel_shape: Shape of structuring element.

    Returns:
        Eroded frame.

    Example:
        >>> eroded = erode(binary_frame, kernel_size=3, iterations=2)
    """
    cv2 = _get_cv2()

    shape_map = {
        "rect": cv2.MORPH_RECT,
        "ellipse": cv2.MORPH_ELLIPSE,
        "cross": cv2.MORPH_CROSS,
    }

    if kernel_shape not in shape_map:
        raise ValueError(f"Unknown kernel_shape '{kernel_shape}'")

    kernel = cv2.getStructuringElement(shape_map[kernel_shape], (kernel_size, kernel_size))
    eroded = cv2.erode(frame.data, kernel, iterations=iterations)

    return Frame(
        data=eroded,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=frame.format,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def dilate(
    frame: Frame,
    kernel_size: int = 3,
    *,
    iterations: int = 1,
    kernel_shape: Literal["rect", "ellipse", "cross"] = "rect",
) -> Frame:
    """Dilate the foreground (grow white regions).

    Args:
        frame: Input frame (binary or grayscale recommended).
        kernel_size: Size of the structuring element.
        iterations: Number of dilation iterations.
        kernel_shape: Shape of structuring element.

    Returns:
        Dilated frame.

    Example:
        >>> dilated = dilate(binary_frame, kernel_size=3)
    """
    cv2 = _get_cv2()

    shape_map = {
        "rect": cv2.MORPH_RECT,
        "ellipse": cv2.MORPH_ELLIPSE,
        "cross": cv2.MORPH_CROSS,
    }

    if kernel_shape not in shape_map:
        raise ValueError(f"Unknown kernel_shape '{kernel_shape}'")

    kernel = cv2.getStructuringElement(shape_map[kernel_shape], (kernel_size, kernel_size))
    dilated = cv2.dilate(frame.data, kernel, iterations=iterations)

    return Frame(
        data=dilated,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=frame.format,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


def morphology(
    frame: Frame,
    operation: Literal["open", "close", "gradient", "tophat", "blackhat"],
    kernel_size: int = 5,
    *,
    iterations: int = 1,
    kernel_shape: Literal["rect", "ellipse", "cross"] = "rect",
) -> Frame:
    """Apply morphological operation.

    Args:
        frame: Input frame.
        operation: Morphological operation:
            - "open": Erosion followed by dilation (remove small noise)
            - "close": Dilation followed by erosion (fill small holes)
            - "gradient": Difference between dilation and erosion (outline)
            - "tophat": Difference between image and opening (bright spots)
            - "blackhat": Difference between closing and image (dark spots)
        kernel_size: Size of structuring element.
        iterations: Number of iterations (for open/close).
        kernel_shape: Shape of structuring element.

    Returns:
        Processed frame.

    Example:
        >>> cleaned = morphology(binary, "open", kernel_size=3)
        >>> filled = morphology(binary, "close", kernel_size=5)
    """
    cv2 = _get_cv2()

    op_map = {
        "open": cv2.MORPH_OPEN,
        "close": cv2.MORPH_CLOSE,
        "gradient": cv2.MORPH_GRADIENT,
        "tophat": cv2.MORPH_TOPHAT,
        "blackhat": cv2.MORPH_BLACKHAT,
    }

    shape_map = {
        "rect": cv2.MORPH_RECT,
        "ellipse": cv2.MORPH_ELLIPSE,
        "cross": cv2.MORPH_CROSS,
    }

    if operation not in op_map:
        raise ValueError(f"Unknown operation '{operation}'")
    if kernel_shape not in shape_map:
        raise ValueError(f"Unknown kernel_shape '{kernel_shape}'")

    kernel = cv2.getStructuringElement(shape_map[kernel_shape], (kernel_size, kernel_size))
    result = cv2.morphologyEx(frame.data, op_map[operation], kernel, iterations=iterations)

    return Frame(
        data=result,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=frame.format,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


# =============================================================================
# Histogram Operations
# =============================================================================


def histogram_equalize(frame: Frame) -> Frame:
    """Equalize histogram to improve contrast.

    For color images, converts to YUV, equalizes the Y channel,
    and converts back.

    Args:
        frame: Input frame.

    Returns:
        Contrast-enhanced frame.

    Example:
        >>> enhanced = histogram_equalize(low_contrast_frame)
    """
    cv2 = _get_cv2()

    if frame.format == PixelFormat.GRAY:
        equalized = cv2.equalizeHist(frame.data)
        return Frame(
            data=equalized,
            timestamp=frame.timestamp,
            width=frame.width,
            height=frame.height,
            format=frame.format,
            frame_number=frame.frame_number,
            exposure_time=frame.exposure_time,
            metadata=frame.metadata.copy(),
        )

    # For color images, convert to YUV and equalize Y channel
    if frame.format == PixelFormat.RGB:
        yuv = cv2.cvtColor(frame.data, cv2.COLOR_RGB2YUV)
    elif frame.format == PixelFormat.BGR:
        yuv = cv2.cvtColor(frame.data, cv2.COLOR_BGR2YUV)
    else:
        # Convert to RGB first
        rgb_frame = frame.to_rgb()
        yuv = cv2.cvtColor(rgb_frame.data, cv2.COLOR_RGB2YUV)

    # Equalize Y channel
    yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])

    # Convert back
    if frame.format == PixelFormat.RGB:
        equalized = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)
    elif frame.format == PixelFormat.BGR:
        equalized = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    else:
        equalized = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)

    return Frame(
        data=equalized,
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
        format=frame.format
        if frame.format in (PixelFormat.RGB, PixelFormat.BGR)
        else PixelFormat.RGB,
        frame_number=frame.frame_number,
        exposure_time=frame.exposure_time,
        metadata=frame.metadata.copy(),
    )


# =============================================================================
# Utility Functions
# =============================================================================


def create_test_frame(
    width: int = 640,
    height: int = 480,
    format: PixelFormat = PixelFormat.RGB,
    pattern: Literal["solid", "gradient", "checkerboard", "noise"] = "gradient",
    color: tuple[int, int, int] = (128, 128, 128),
) -> Frame:
    """Create a test frame for development and testing.

    Args:
        width: Frame width.
        height: Frame height.
        format: Pixel format.
        pattern: Test pattern type:
            - "solid": Solid color fill
            - "gradient": Horizontal gradient
            - "checkerboard": Black and white checkerboard
            - "noise": Random noise
        color: Color for solid pattern (RGB values 0-255).

    Returns:
        Test frame.

    Example:
        >>> test = create_test_frame(640, 480, pattern="gradient")
    """
    if pattern == "solid":
        if format == PixelFormat.GRAY:
            data = np.full((height, width), color[0], dtype=np.uint8)
        else:
            data = np.full((height, width, 3), color, dtype=np.uint8)
    elif pattern == "gradient":
        gradient = np.linspace(0, 255, width, dtype=np.uint8)
        if format == PixelFormat.GRAY:
            data = np.tile(gradient, (height, 1))
        else:
            gray = np.tile(gradient, (height, 1))
            data = np.stack([gray, gray, gray], axis=-1)
    elif pattern == "checkerboard":
        # Create checkerboard pattern
        block_size = 32
        x = np.arange(width) // block_size
        y = np.arange(height) // block_size
        xx, yy = np.meshgrid(x, y)
        checker = ((xx + yy) % 2 * 255).astype(np.uint8)
        if format == PixelFormat.GRAY:
            data = checker
        else:
            data = np.stack([checker, checker, checker], axis=-1)
    elif pattern == "noise":
        if format == PixelFormat.GRAY:
            data = np.random.randint(0, 256, (height, width), dtype=np.uint8)
        else:
            data = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
    else:
        raise ValueError(f"Unknown pattern '{pattern}'")

    # Handle BGR format
    if format == PixelFormat.BGR and len(data.shape) == 3:
        data = data[:, :, ::-1].copy()

    return Frame(
        data=data,
        timestamp=time.monotonic(),
        width=width,
        height=height,
        format=format,
        frame_number=0,
    )
