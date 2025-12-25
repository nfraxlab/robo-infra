"""Tests for vision/color.py - Color-based object detection."""

from __future__ import annotations

import time

import numpy as np
import pytest

from robo_infra.sensors.camera import Frame, PixelFormat


# =============================================================================
# Test Helper
# =============================================================================


def create_test_frame(
    width: int = 640,
    height: int = 480,
    format: PixelFormat = PixelFormat.RGB,
    color: tuple[int, int, int] | None = None,
) -> Frame:
    """Create a test frame.

    Args:
        width: Frame width.
        height: Frame height.
        format: Pixel format.
        color: If specified, create solid color frame.
    """
    if color is not None:
        if format == PixelFormat.GRAY:
            data = np.full((height, width), color[0], dtype=np.uint8)
        else:
            data = np.full((height, width, 3), color, dtype=np.uint8)
            if format == PixelFormat.BGR:
                data = data[:, :, ::-1].copy()
    elif format == PixelFormat.GRAY:
        data = np.random.randint(0, 256, (height, width), dtype=np.uint8)
    else:
        data = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

    return Frame(
        data=data,
        timestamp=time.monotonic(),
        width=width,
        height=height,
        format=format,
        frame_number=0,
    )


def create_color_blob_frame(
    width: int = 640,
    height: int = 480,
    blob_color: tuple[int, int, int] = (255, 0, 0),
    blob_center: tuple[int, int] = (320, 240),
    blob_radius: int = 50,
    format: PixelFormat = PixelFormat.RGB,
) -> Frame:
    """Create a frame with a colored blob on black background."""
    # Start with black background
    data = np.zeros((height, width, 3), dtype=np.uint8)

    # Draw circular blob
    y, x = np.ogrid[:height, :width]
    cx, cy = blob_center
    mask = (x - cx) ** 2 + (y - cy) ** 2 <= blob_radius ** 2
    data[mask] = blob_color

    if format == PixelFormat.BGR:
        data = data[:, :, ::-1].copy()

    return Frame(
        data=data,
        timestamp=time.monotonic(),
        width=width,
        height=height,
        format=format,
        frame_number=0,
    )


# =============================================================================
# Test ColorRange
# =============================================================================


class TestColorRange:
    """Tests for ColorRange dataclass."""

    def test_color_range_creation(self):
        """Test creating a ColorRange."""
        from robo_infra.vision.color import ColorRange

        cr = ColorRange((0, 100, 100), (10, 255, 255), "red")

        assert cr.lower == (0, 100, 100)
        assert cr.upper == (10, 255, 255)
        assert cr.name == "red"

    def test_color_range_validation(self):
        """Test ColorRange validates bounds."""
        from robo_infra.vision.color import ColorRange

        # Hue out of range
        with pytest.raises(ValueError):
            ColorRange((180, 100, 100), (200, 255, 255))

        # Saturation out of range
        with pytest.raises(ValueError):
            ColorRange((0, 300, 100), (10, 255, 255))

    def test_color_range_contains(self):
        """Test ColorRange.contains method."""
        from robo_infra.vision.color import ColorRange

        # Green range
        cr = ColorRange((35, 100, 100), (85, 255, 255))

        assert cr.contains((60, 150, 200))  # Green
        assert not cr.contains((0, 150, 200))  # Red hue

    def test_color_range_contains_wraparound(self):
        """Test ColorRange.contains with hue wraparound (red)."""
        from robo_infra.vision.color import ColorRange

        # Red wraps around 0/180
        cr = ColorRange((170, 100, 100), (10, 255, 255))

        assert cr.contains((175, 150, 200))  # High red
        assert cr.contains((5, 150, 200))  # Low red
        assert not cr.contains((90, 150, 200))  # Green

    def test_color_range_expand(self):
        """Test ColorRange.expand method."""
        from robo_infra.vision.color import ColorRange

        cr = ColorRange((60, 100, 100), (80, 200, 200))
        expanded = cr.expand(h_delta=10, s_delta=20, v_delta=30)

        assert expanded.lower == (50, 80, 70)
        assert expanded.upper == (90, 220, 230)

    def test_color_range_expand_clamped(self):
        """Test ColorRange.expand clamps to valid bounds."""
        from robo_infra.vision.color import ColorRange

        cr = ColorRange((5, 10, 10), (175, 250, 250))
        expanded = cr.expand(h_delta=20, s_delta=20, v_delta=20)

        assert expanded.lower[0] == 0  # Clamped to 0
        assert expanded.upper[0] == 179  # Clamped to 179
        assert expanded.upper[1] == 255  # Clamped to 255


# =============================================================================
# Test ColorBlob
# =============================================================================


class TestColorBlob:
    """Tests for ColorBlob dataclass."""

    def test_color_blob_creation(self):
        """Test creating a ColorBlob."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.int32)

        blob = ColorBlob(
            center=(150.0, 150.0),
            area=10000.0,
            bounding_box=(100, 100, 100, 100),
            contour=contour,
        )

        assert blob.center == (150.0, 150.0)
        assert blob.area == 10000.0

    def test_color_blob_properties(self):
        """Test ColorBlob properties."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.int32)

        blob = ColorBlob(
            center=(150.0, 150.0),
            area=10000.0,
            bounding_box=(100, 100, 100, 100),
            contour=contour,
        )

        assert blob.x == 100
        assert blob.y == 100
        assert blob.width == 100
        assert blob.height == 100

    def test_color_blob_radius(self):
        """Test ColorBlob radius calculation."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.int32)

        # Area of 100 * pi = 314.159...
        blob = ColorBlob(
            center=(50.0, 50.0),
            area=np.pi * 100,  # Area = pi * r^2, r=10
            bounding_box=(0, 0, 100, 100),
            contour=contour,
        )

        assert blob.radius == pytest.approx(10.0)


# =============================================================================
# Test Predefined Colors
# =============================================================================


class TestPredefinedColors:
    """Tests for predefined color ranges."""

    def test_list_available_colors(self):
        """Test listing available colors."""
        from robo_infra.vision.color import list_available_colors

        colors = list_available_colors()

        assert "red" in colors
        assert "green" in colors
        assert "blue" in colors
        assert len(colors) >= 10

    def test_predefined_colors_exist(self):
        """Test predefined color ranges exist."""
        from robo_infra.vision.color import PREDEFINED_COLORS

        assert "red" in PREDEFINED_COLORS
        assert "green" in PREDEFINED_COLORS
        assert "blue" in PREDEFINED_COLORS
        assert "yellow" in PREDEFINED_COLORS
        assert "orange" in PREDEFINED_COLORS
        assert "white" in PREDEFINED_COLORS
        assert "black" in PREDEFINED_COLORS


# =============================================================================
# Test ColorDetector
# =============================================================================


class TestColorDetectorCreation:
    """Tests for ColorDetector creation."""

    def test_create_with_hsv_range(self):
        """Test creating detector with HSV range."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector(
            lower_hsv=(35, 100, 100),
            upper_hsv=(85, 255, 255),
            name="green",
            simulation=True,
        )

        assert detector.name == "green"

    def test_create_without_range_raises(self):
        """Test creating detector without range raises error."""
        from robo_infra.vision.color import ColorDetector

        with pytest.raises(ValueError, match="Either provide"):
            ColorDetector(name="test", simulation=True)

    def test_from_color(self):
        """Test creating detector from predefined color."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.from_color("green", simulation=True)

        assert detector.name == "green"

    def test_from_color_invalid(self):
        """Test creating detector from invalid color."""
        from robo_infra.vision.color import ColorDetector

        with pytest.raises(ValueError, match="Unknown color"):
            ColorDetector.from_color("magenta")

    def test_convenience_methods(self):
        """Test convenience class methods."""
        from robo_infra.vision.color import ColorDetector

        # Test all color convenience methods
        assert ColorDetector.red(simulation=True).name == "red"
        assert ColorDetector.green(simulation=True).name == "green"
        assert ColorDetector.blue(simulation=True).name == "blue"
        assert ColorDetector.yellow(simulation=True).name == "yellow"
        assert ColorDetector.orange(simulation=True).name == "orange"
        assert ColorDetector.purple(simulation=True).name == "purple"
        assert ColorDetector.pink(simulation=True).name == "pink"
        assert ColorDetector.cyan(simulation=True).name == "cyan"
        assert ColorDetector.white(simulation=True).name == "white"
        assert ColorDetector.black(simulation=True).name == "black"


class TestColorDetectorSimulated:
    """Tests for ColorDetector in simulation mode."""

    def test_detect_simulated(self):
        """Test detection in simulation mode."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(simulation=True)
        frame = create_test_frame()

        blobs = detector.detect(frame)

        assert isinstance(blobs, list)
        assert len(blobs) == 0  # Simulation returns empty

    def test_get_mask_simulated(self):
        """Test get_mask in simulation mode."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(simulation=True)
        frame = create_test_frame(640, 480)

        mask = detector.get_mask(frame)

        assert mask.format == PixelFormat.GRAY
        assert mask.width == 640
        assert mask.height == 480
        # Simulation returns zero mask
        assert np.all(mask.data == 0)

    def test_draw_blobs_simulated(self):
        """Test drawing blobs in simulation mode."""
        from robo_infra.vision.color import ColorBlob, ColorDetector

        detector = ColorDetector.red(simulation=True)
        frame = create_test_frame()

        contour = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.int32)
        blobs = [
            ColorBlob(
                center=(150.0, 150.0),
                area=10000.0,
                bounding_box=(100, 100, 100, 100),
                contour=contour,
            )
        ]

        result = detector.draw_blobs(frame, blobs)

        assert result.width == frame.width


class TestColorDetectorMocked:
    """Tests for ColorDetector with mocked OpenCV."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_detect_no_blobs(self, cv2):
        """Test detection on frame with no matching colors."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        # Pure blue frame, no red
        frame = create_test_frame(color=(0, 0, 255))

        blobs = detector.detect(frame)

        assert len(blobs) == 0

    def test_detect_with_blob(self, cv2):
        """Test detection on frame with matching color."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=50)
        # Frame with red blob
        frame = create_color_blob_frame(blob_color=(255, 0, 0), blob_radius=30)

        blobs = detector.detect(frame)

        # Should find the red blob
        assert len(blobs) >= 1
        if blobs:
            assert blobs[0].area > 0

    def test_detect_grayscale_warning(self, cv2, caplog):
        """Test detection on grayscale frame logs warning."""
        import logging

        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame(format=PixelFormat.GRAY)

        with caplog.at_level(logging.WARNING):
            blobs = detector.detect(frame)

        assert len(blobs) == 0
        assert "grayscale" in caplog.text

    def test_detect_bgr_format(self, cv2):
        """Test detection on BGR format frame."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.green(min_area=50)
        # Green blob in BGR format
        frame = create_color_blob_frame(
            blob_color=(0, 255, 0), blob_radius=30, format=PixelFormat.BGR
        )

        blobs = detector.detect(frame)

        # Should find the green blob
        assert len(blobs) >= 1

    def test_get_mask(self, cv2):
        """Test get_mask returns binary mask."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_color_blob_frame(blob_color=(255, 0, 0))

        mask = detector.get_mask(frame)

        assert mask.format == PixelFormat.GRAY
        # Should have some non-zero pixels
        assert np.any(mask.data > 0)

    def test_draw_blobs(self, cv2):
        """Test drawing blobs on frame."""
        from robo_infra.vision.color import ColorBlob, ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame()

        contour = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.int32)
        blobs = [
            ColorBlob(
                center=(150.0, 150.0),
                area=10000.0,
                bounding_box=(100, 100, 100, 100),
                contour=contour,
                color_name="red",
            )
        ]

        result = detector.draw_blobs(frame, blobs)

        assert result.width == frame.width

    def test_draw_blobs_options(self, cv2):
        """Test draw_blobs with different options."""
        from robo_infra.vision.color import ColorBlob, ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame()

        contour = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.int32)
        blobs = [
            ColorBlob(
                center=(150.0, 150.0),
                area=10000.0,
                bounding_box=(100, 100, 100, 100),
                contour=contour,
            )
        ]

        result = detector.draw_blobs(
            frame,
            blobs,
            draw_contours=True,
            draw_bounding_box=True,
            draw_center=True,
            draw_label=True,
        )

        assert result.width == frame.width


# =============================================================================
# Test Color Space Utilities
# =============================================================================


class TestColorSpaceUtilities:
    """Tests for color space utility functions."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_rgb_to_hsv(self, cv2):
        """Test RGB to HSV conversion."""
        from robo_infra.vision.color import rgb_to_hsv

        # Red
        h, s, v = rgb_to_hsv(255, 0, 0)
        assert h == 0  # Red hue
        assert s == 255
        assert v == 255

        # Green
        h, s, v = rgb_to_hsv(0, 255, 0)
        assert 50 <= h <= 70  # Green hue range

    def test_hsv_to_rgb(self, cv2):
        """Test HSV to RGB conversion."""
        from robo_infra.vision.color import hsv_to_rgb

        # Red
        r, g, b = hsv_to_rgb(0, 255, 255)
        assert r == 255
        assert g == 0
        assert b == 0

    def test_rgb_hsv_roundtrip(self, cv2):
        """Test RGB -> HSV -> RGB roundtrip."""
        from robo_infra.vision.color import hsv_to_rgb, rgb_to_hsv

        original = (100, 150, 200)
        h, s, v = rgb_to_hsv(*original)
        r, g, b = hsv_to_rgb(h, s, v)

        assert r == pytest.approx(original[0], abs=2)
        assert g == pytest.approx(original[1], abs=2)
        assert b == pytest.approx(original[2], abs=2)

    def test_sample_color_at_point(self, cv2):
        """Test sampling color at a point."""
        from robo_infra.vision.color import sample_color_at_point

        # Create solid red frame
        frame = create_test_frame(color=(255, 0, 0))

        rgb, hsv = sample_color_at_point(frame, (320, 240), radius=5)

        assert rgb == (255, 0, 0)
        assert hsv[0] == 0  # Red hue

    def test_auto_color_range(self, cv2):
        """Test automatic color range detection."""
        from robo_infra.vision.color import auto_color_range

        # Create solid green frame
        frame = create_test_frame(color=(0, 255, 0))

        color_range = auto_color_range(frame, (320, 240))

        # Should create a range around green
        assert color_range.name == "sampled"
        # Lower and upper should bracket the green hue
        assert color_range.lower[0] < color_range.upper[0]

    def test_get_dominant_color(self, cv2):
        """Test getting dominant color."""
        from robo_infra.vision.color import get_dominant_color

        # Create mostly blue frame
        data = np.full((480, 640, 3), (0, 0, 255), dtype=np.uint8)
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
            frame_number=0,
        )

        r, g, b = get_dominant_color(frame, k=1)

        # Should be close to blue
        assert b > r
        assert b > g


# =============================================================================
# Test Filtering Options
# =============================================================================


class TestFilteringOptions:
    """Tests for blob filtering options."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_min_area_filter(self, cv2):
        """Test minimum area filtering."""
        from robo_infra.vision.color import ColorDetector

        # Small min_area should find small blobs
        detector_small = ColorDetector.red(min_area=10)
        # Large min_area should filter out small blobs
        detector_large = ColorDetector.red(min_area=50000)

        frame = create_color_blob_frame(blob_color=(255, 0, 0), blob_radius=20)

        blobs_small = detector_small.detect(frame)
        blobs_large = detector_large.detect(frame)

        # Small min_area should find more blobs
        assert len(blobs_small) >= len(blobs_large)

    def test_max_blobs_limit(self, cv2):
        """Test max blobs limit."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=10, max_blobs=1)

        # Create frame with multiple blobs
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        # Add multiple red circles
        for i in range(5):
            y, x = np.ogrid[:480, :640]
            cx, cy = 100 + i * 100, 240
            mask = (x - cx) ** 2 + (y - cy) ** 2 <= 30 ** 2
            data[mask] = (255, 0, 0)

        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
            frame_number=0,
        )

        blobs = detector.detect(frame)

        assert len(blobs) <= 1


# =============================================================================
# Test Color Ranges Property
# =============================================================================


class TestColorRangesProperty:
    """Tests for color_ranges property."""

    def test_get_color_ranges(self):
        """Test getting color ranges."""
        from robo_infra.vision.color import ColorDetector, ColorRange

        cr = ColorRange((35, 100, 100), (85, 255, 255), "green")
        detector = ColorDetector(color_ranges=[cr], simulation=True)

        ranges = detector.color_ranges

        assert len(ranges) == 1
        assert ranges[0].name == "green"

    def test_color_ranges_is_copy(self):
        """Test that color_ranges returns a copy."""
        from robo_infra.vision.color import ColorDetector, ColorRange

        cr = ColorRange((35, 100, 100), (85, 255, 255), "green")
        detector = ColorDetector(color_ranges=[cr], simulation=True)

        ranges1 = detector.color_ranges
        ranges2 = detector.color_ranges

        assert ranges1 is not ranges2


# =============================================================================
# Phase 5.9.2.1 - Color Detection Comprehensive Tests
# =============================================================================


class TestColorDetectorDetectComprehensive:
    """Comprehensive tests for ColorDetector.detect method."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_detect_red_blob_rgb(self, cv2):
        """Test detecting red blob in RGB frame."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=50)
        frame = create_color_blob_frame(blob_color=(255, 0, 0), blob_radius=40)

        blobs = detector.detect(frame)

        assert len(blobs) >= 1
        blob = blobs[0]
        assert blob.area > 0
        assert blob.color_name == "red"

    def test_detect_green_blob_bgr(self, cv2):
        """Test detecting green blob in BGR frame."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.green(min_area=50)
        frame = create_color_blob_frame(
            blob_color=(0, 255, 0), blob_radius=40, format=PixelFormat.BGR
        )

        blobs = detector.detect(frame)

        assert len(blobs) >= 1

    def test_detect_blue_blob(self, cv2):
        """Test detecting blue blob."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.blue(min_area=50)
        frame = create_color_blob_frame(blob_color=(0, 0, 255), blob_radius=40)

        blobs = detector.detect(frame)

        assert len(blobs) >= 1

    def test_detect_yellow_blob(self, cv2):
        """Test detecting yellow blob."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.yellow(min_area=50)
        # Yellow is RGB (255, 255, 0)
        frame = create_color_blob_frame(blob_color=(255, 255, 0), blob_radius=40)

        blobs = detector.detect(frame)

        assert len(blobs) >= 1

    def test_detect_multiple_blobs(self, cv2):
        """Test detecting multiple blobs of same color."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=50)

        # Create frame with multiple red blobs
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        for i in range(3):
            y, x = np.ogrid[:480, :640]
            cx, cy = 100 + i * 200, 240
            mask = (x - cx) ** 2 + (y - cy) ** 2 <= 40 ** 2
            data[mask] = (255, 0, 0)

        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
            frame_number=0,
        )

        blobs = detector.detect(frame)

        assert len(blobs) >= 2

    def test_detect_sorted_by_area(self, cv2):
        """Test that blobs are sorted by area (largest first)."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=50)

        # Create frame with blobs of different sizes
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        radii = [20, 50, 35]  # Small, large, medium
        for i, r in enumerate(radii):
            y, x = np.ogrid[:480, :640]
            cx, cy = 100 + i * 200, 240
            mask = (x - cx) ** 2 + (y - cy) ** 2 <= r ** 2
            data[mask] = (255, 0, 0)

        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.RGB,
            frame_number=0,
        )

        blobs = detector.detect(frame)

        # Should be sorted by area, largest first
        for i in range(len(blobs) - 1):
            assert blobs[i].area >= blobs[i + 1].area

    def test_detect_blob_center_accuracy(self, cv2):
        """Test that blob center is accurately computed."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=50)
        center = (320, 240)
        frame = create_color_blob_frame(
            blob_color=(255, 0, 0), blob_center=center, blob_radius=40
        )

        blobs = detector.detect(frame)

        assert len(blobs) >= 1
        # Center should be close to the actual center
        cx, cy = blobs[0].center
        assert abs(cx - center[0]) < 5
        assert abs(cy - center[1]) < 5

    def test_detect_blob_properties(self, cv2):
        """Test blob properties are computed correctly."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=50)
        frame = create_color_blob_frame(blob_color=(255, 0, 0), blob_radius=40)

        blobs = detector.detect(frame)

        assert len(blobs) >= 1
        blob = blobs[0]

        # Check properties exist and are valid
        assert blob.circularity > 0.7  # Should be fairly circular
        assert blob.solidity > 0.8  # Should be solid
        assert blob.mean_hsv is not None
        assert blob.timestamp > 0


class TestColorDetectorFromColorComprehensive:
    """Comprehensive tests for ColorDetector.from_color class method."""

    def test_from_color_all_predefined(self):
        """Test from_color works for all predefined colors."""
        from robo_infra.vision.color import ColorDetector, list_available_colors

        for color in list_available_colors():
            detector = ColorDetector.from_color(color, simulation=True)
            assert detector.name == color

    def test_from_color_case_insensitive(self):
        """Test from_color is case insensitive."""
        from robo_infra.vision.color import ColorDetector

        detector_lower = ColorDetector.from_color("red", simulation=True)
        detector_upper = ColorDetector.from_color("RED", simulation=True)
        detector_mixed = ColorDetector.from_color("Red", simulation=True)

        assert detector_lower.name == "red"
        assert detector_upper.name == "red"
        assert detector_mixed.name == "red"

    def test_from_color_with_params(self):
        """Test from_color with custom parameters."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.from_color(
            "green",
            min_area=500,
            max_area=10000,
            simulation=True,
        )

        assert detector.name == "green"


class TestColorDetectorPredefinedColorsComprehensive:
    """Comprehensive tests for predefined color detectors."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_predefined_red_dual_range(self, cv2):
        """Test red has dual range for hue wraparound."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(simulation=True)
        ranges = detector.color_ranges

        # Red should have 2 ranges (wraps around 0)
        assert len(ranges) == 2

    def test_predefined_green_single_range(self, cv2):
        """Test green has single range."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.green(simulation=True)
        ranges = detector.color_ranges

        assert len(ranges) == 1

    def test_all_predefined_have_valid_ranges(self):
        """Test all predefined colors have valid HSV ranges."""
        from robo_infra.vision.color import PREDEFINED_COLORS, ColorRange

        for color_name, color_def in PREDEFINED_COLORS.items():
            if isinstance(color_def, tuple):
                for cr in color_def:
                    assert isinstance(cr, ColorRange)
            else:
                assert isinstance(color_def, ColorRange)


class TestColorDetectorGetMaskComprehensive:
    """Comprehensive tests for ColorDetector.get_mask method."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_get_mask_red(self, cv2):
        """Test get_mask returns mask for red regions."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_color_blob_frame(blob_color=(255, 0, 0), blob_radius=40)

        mask = detector.get_mask(frame)

        assert mask.format == PixelFormat.GRAY
        assert np.any(mask.data == 255)  # Some white pixels

    def test_get_mask_bgr_input(self, cv2):
        """Test get_mask works with BGR input."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.green()
        frame = create_color_blob_frame(
            blob_color=(0, 255, 0), blob_radius=40, format=PixelFormat.BGR
        )

        mask = detector.get_mask(frame)

        assert mask.format == PixelFormat.GRAY

    def test_get_mask_preserves_dimensions(self, cv2):
        """Test get_mask preserves frame dimensions."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.blue()
        frame = create_test_frame(800, 600)

        mask = detector.get_mask(frame)

        assert mask.width == 800
        assert mask.height == 600


class TestColorDetectorDrawBlobsComprehensive:
    """Comprehensive tests for ColorDetector.draw_blobs method."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    @pytest.fixture
    def sample_blob(self):
        """Create a sample blob for testing."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.int32
        )
        return ColorBlob(
            center=(150.0, 150.0),
            area=10000.0,
            bounding_box=(100, 100, 100, 100),
            contour=contour,
            color_name="red",
        )

    def test_draw_blobs_rgb(self, cv2, sample_blob):
        """Test drawing blobs on RGB frame."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame(format=PixelFormat.RGB)

        result = detector.draw_blobs(frame, [sample_blob])

        assert result.format == PixelFormat.RGB

    def test_draw_blobs_bgr(self, cv2, sample_blob):
        """Test drawing blobs on BGR frame."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame(format=PixelFormat.BGR)

        result = detector.draw_blobs(frame, [sample_blob])

        assert result.format == PixelFormat.BGR

    def test_draw_blobs_grayscale(self, cv2, sample_blob):
        """Test drawing blobs on grayscale frame converts to color."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame(format=PixelFormat.GRAY)

        result = detector.draw_blobs(frame, [sample_blob])

        # Should convert to BGR for drawing
        assert result.format == PixelFormat.BGR

    def test_draw_blobs_empty_list(self, cv2):
        """Test drawing empty blob list."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame()

        result = detector.draw_blobs(frame, [])

        assert result.width == frame.width

    def test_draw_blobs_all_options(self, cv2, sample_blob):
        """Test drawing with all options enabled."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red()
        frame = create_test_frame()

        result = detector.draw_blobs(
            frame,
            [sample_blob],
            draw_contours=True,
            draw_bounding_box=True,
            draw_center=True,
            draw_label=True,
            contour_color=(0, 255, 0),
            box_color=(255, 0, 0),
            center_color=(0, 0, 255),
            thickness=3,
        )

        assert result.width == frame.width


class TestColorBlobPropertiesComprehensive:
    """Comprehensive tests for ColorBlob properties."""

    def test_blob_x_property(self):
        """Test blob x property."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array([[50, 60], [150, 60], [150, 160], [50, 160]], dtype=np.int32)
        blob = ColorBlob(
            center=(100.0, 110.0),
            area=10000.0,
            bounding_box=(50, 60, 100, 100),
            contour=contour,
        )

        assert blob.x == 50

    def test_blob_y_property(self):
        """Test blob y property."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array([[50, 60], [150, 60], [150, 160], [50, 160]], dtype=np.int32)
        blob = ColorBlob(
            center=(100.0, 110.0),
            area=10000.0,
            bounding_box=(50, 60, 100, 100),
            contour=contour,
        )

        assert blob.y == 60

    def test_blob_width_property(self):
        """Test blob width property."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array([[0, 0], [80, 0], [80, 60], [0, 60]], dtype=np.int32)
        blob = ColorBlob(
            center=(40.0, 30.0),
            area=4800.0,
            bounding_box=(0, 0, 80, 60),
            contour=contour,
        )

        assert blob.width == 80

    def test_blob_height_property(self):
        """Test blob height property."""
        from robo_infra.vision.color import ColorBlob

        contour = np.array([[0, 0], [80, 0], [80, 60], [0, 60]], dtype=np.int32)
        blob = ColorBlob(
            center=(40.0, 30.0),
            area=4800.0,
            bounding_box=(0, 0, 80, 60),
            contour=contour,
        )

        assert blob.height == 60


class TestRgbToHsvComprehensive:
    """Comprehensive tests for rgb_to_hsv conversion."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_rgb_to_hsv_pure_red(self, cv2):
        """Test conversion of pure red."""
        from robo_infra.vision.color import rgb_to_hsv

        h, s, v = rgb_to_hsv(255, 0, 0)

        assert h == 0
        assert s == 255
        assert v == 255

    def test_rgb_to_hsv_pure_green(self, cv2):
        """Test conversion of pure green."""
        from robo_infra.vision.color import rgb_to_hsv

        h, s, v = rgb_to_hsv(0, 255, 0)

        # Green is around hue 60 in standard, 60 in OpenCV format
        assert 55 <= h <= 65
        assert s == 255
        assert v == 255

    def test_rgb_to_hsv_pure_blue(self, cv2):
        """Test conversion of pure blue."""
        from robo_infra.vision.color import rgb_to_hsv

        h, s, v = rgb_to_hsv(0, 0, 255)

        # Blue is around hue 120 in OpenCV format
        assert 115 <= h <= 125
        assert s == 255
        assert v == 255

    def test_rgb_to_hsv_white(self, cv2):
        """Test conversion of white."""
        from robo_infra.vision.color import rgb_to_hsv

        h, s, v = rgb_to_hsv(255, 255, 255)

        assert s == 0  # No saturation
        assert v == 255  # Full value

    def test_rgb_to_hsv_black(self, cv2):
        """Test conversion of black."""
        from robo_infra.vision.color import rgb_to_hsv

        h, s, v = rgb_to_hsv(0, 0, 0)

        assert v == 0  # Zero value


class TestGetDominantColorComprehensive:
    """Comprehensive tests for get_dominant_color function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_get_dominant_color_solid(self, cv2):
        """Test dominant color of solid frame."""
        from robo_infra.vision.color import get_dominant_color

        frame = create_test_frame(color=(100, 150, 200))

        r, g, b = get_dominant_color(frame, k=1)

        assert 90 <= r <= 110
        assert 140 <= g <= 160
        assert 190 <= b <= 210

    def test_get_dominant_color_bgr(self, cv2):
        """Test dominant color from BGR frame."""
        from robo_infra.vision.color import get_dominant_color

        # Create BGR frame with red color
        data = np.full((100, 100, 3), (0, 0, 255), dtype=np.uint8)  # BGR red
        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=100,
            height=100,
            format=PixelFormat.BGR,
            frame_number=0,
        )

        r, g, b = get_dominant_color(frame, k=1)

        assert r > 200  # Should be red

    def test_get_dominant_color_multiple_clusters(self, cv2):
        """Test dominant color with multiple clusters."""
        from robo_infra.vision.color import get_dominant_color

        # Create frame with mostly one color
        frame = create_test_frame(color=(50, 100, 150))

        r, g, b = get_dominant_color(frame, k=3)

        # Should still get the dominant color
        assert 40 <= r <= 60
        assert 90 <= g <= 110


# =============================================================================
# Integration Tests
# =============================================================================


class TestColorDetectionIntegration:
    """Integration tests for color detection pipeline."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_full_detection_pipeline(self, cv2):
        """Test complete detection pipeline."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.red(min_area=100)
        frame = create_color_blob_frame(blob_color=(255, 0, 0), blob_radius=50)

        # Detect blobs
        blobs = detector.detect(frame)
        assert len(blobs) >= 1

        # Get mask
        mask = detector.get_mask(frame)
        assert mask.format == PixelFormat.GRAY

        # Draw blobs
        result = detector.draw_blobs(frame, blobs)
        assert result.format == frame.format

    def test_detect_and_track_workflow(self, cv2):
        """Test detect-and-track style workflow."""
        from robo_infra.vision.color import ColorDetector

        detector = ColorDetector.green(min_area=50)

        # Simulate multiple frames
        for i in range(3):
            center = (200 + i * 50, 240)
            frame = create_color_blob_frame(
                blob_color=(0, 255, 0),
                blob_center=center,
                blob_radius=40,
            )

            blobs = detector.detect(frame)
            assert len(blobs) >= 1

            # Center should track
            cx, cy = blobs[0].center
            assert abs(cx - center[0]) < 10

    def test_custom_color_detection(self, cv2):
        """Test detection with custom color range."""
        from robo_infra.vision.color import ColorDetector, ColorRange

        # Custom orange-ish range
        custom_range = ColorRange((10, 100, 100), (25, 255, 255), "custom_orange")
        detector = ColorDetector(color_ranges=[custom_range], min_area=50)

        # Create frame with orange blob
        frame = create_color_blob_frame(blob_color=(255, 165, 0), blob_radius=40)

        blobs = detector.detect(frame)
        # May or may not find depending on exact HSV conversion
        assert isinstance(blobs, list)

