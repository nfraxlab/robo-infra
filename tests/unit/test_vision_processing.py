"""Tests for vision/processing.py - Image processing utilities."""

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
    pattern: str = "gradient",
) -> Frame:
    """Create a test frame for testing."""
    if pattern == "solid":
        if format == PixelFormat.GRAY:
            data = np.full((height, width), 128, dtype=np.uint8)
        else:
            data = np.full((height, width, 3), (128, 128, 128), dtype=np.uint8)
    elif pattern == "gradient":
        gradient = np.linspace(0, 255, width, dtype=np.uint8)
        if format == PixelFormat.GRAY:
            data = np.tile(gradient, (height, 1))
        else:
            gray = np.tile(gradient, (height, 1))
            data = np.stack([gray, gray, gray], axis=-1)
    elif pattern == "checkerboard":
        block_size = 32
        x = np.arange(width) // block_size
        y = np.arange(height) // block_size
        xx, yy = np.meshgrid(x, y)
        checker = ((xx + yy) % 2 * 255).astype(np.uint8)
        if format == PixelFormat.GRAY:
            data = checker
        else:
            data = np.stack([checker, checker, checker], axis=-1)
    elif pattern == "edges":
        # Create a frame with clear edges for edge detection testing
        data = np.zeros((height, width, 3), dtype=np.uint8)
        # Vertical line
        data[:, width // 2 - 5 : width // 2 + 5] = 255
        # Horizontal line
        data[height // 2 - 5 : height // 2 + 5, :] = 255
        if format == PixelFormat.GRAY:
            data = data[:, :, 0]
    else:
        data = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
        if format == PixelFormat.GRAY:
            data = data[:, :, 0]

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


# =============================================================================
# Test Resize
# =============================================================================


class TestResize:
    """Tests for resize function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_resize_downscale(self, cv2):
        """Test downscaling a frame."""
        from robo_infra.vision.processing import resize

        frame = create_test_frame(640, 480)
        result = resize(frame, (320, 240))

        assert result.width == 320
        assert result.height == 240
        assert result.format == frame.format
        assert result.data.shape == (240, 320, 3)

    def test_resize_upscale(self, cv2):
        """Test upscaling a frame."""
        from robo_infra.vision.processing import resize

        frame = create_test_frame(320, 240)
        result = resize(frame, (640, 480))

        assert result.width == 640
        assert result.height == 480

    def test_resize_same_size_returns_copy(self, cv2):
        """Test resize with same size returns a copy."""
        from robo_infra.vision.processing import resize

        frame = create_test_frame(640, 480)
        result = resize(frame, (640, 480))

        assert result.width == frame.width
        assert result.height == frame.height
        # Verify it's a copy, not the same object
        assert result is not frame
        assert result.data is not frame.data

    def test_resize_interpolation_methods(self, cv2):
        """Test different interpolation methods."""
        from robo_infra.vision.processing import resize

        frame = create_test_frame(640, 480)

        for method in ["nearest", "linear", "cubic", "area", "lanczos"]:
            result = resize(frame, (320, 240), interpolation=method)
            assert result.width == 320
            assert result.height == 240

    def test_resize_grayscale(self, cv2):
        """Test resizing grayscale frames."""
        from robo_infra.vision.processing import resize

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY)
        result = resize(frame, (320, 240))

        assert result.format == PixelFormat.GRAY
        assert result.data.shape == (240, 320)

    def test_resize_invalid_size(self, cv2):
        """Test resize with invalid size raises error."""
        from robo_infra.vision.processing import resize

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError):
            resize(frame, (0, 240))

        with pytest.raises(ValueError):
            resize(frame, (320, -1))

    def test_resize_invalid_interpolation(self, cv2):
        """Test resize with invalid interpolation raises error."""
        from robo_infra.vision.processing import resize

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError, match="Unknown interpolation"):
            resize(frame, (320, 240), interpolation="invalid")


# =============================================================================
# Test Crop
# =============================================================================


class TestCrop:
    """Tests for crop function."""

    def test_crop_basic(self):
        """Test basic cropping."""
        from robo_infra.vision.processing import crop

        frame = create_test_frame(640, 480)
        result = crop(frame, (100, 100, 200, 200))

        assert result.width == 200
        assert result.height == 200
        assert result.data.shape == (200, 200, 3)

    def test_crop_full_frame(self):
        """Test cropping full frame."""
        from robo_infra.vision.processing import crop

        frame = create_test_frame(640, 480)
        result = crop(frame, (0, 0, 640, 480))

        assert result.width == 640
        assert result.height == 480

    def test_crop_out_of_bounds(self):
        """Test crop with out of bounds raises error."""
        from robo_infra.vision.processing import crop

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError):
            crop(frame, (600, 100, 100, 100))  # Exceeds width

        with pytest.raises(ValueError):
            crop(frame, (100, 450, 100, 100))  # Exceeds height

    def test_crop_negative_coords(self):
        """Test crop with negative coordinates raises error."""
        from robo_infra.vision.processing import crop

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError):
            crop(frame, (-10, 100, 100, 100))

    def test_crop_grayscale(self):
        """Test cropping grayscale frames."""
        from robo_infra.vision.processing import crop

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY)
        result = crop(frame, (100, 100, 200, 200))

        assert result.format == PixelFormat.GRAY
        assert result.data.shape == (200, 200)


# =============================================================================
# Test Rotate
# =============================================================================


class TestRotate:
    """Tests for rotate function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_rotate_90_degrees(self, cv2):
        """Test 90 degree rotation."""
        from robo_infra.vision.processing import rotate

        frame = create_test_frame(640, 480)
        result = rotate(frame, 90.0)

        assert result.width == 640
        assert result.height == 480
        assert result.format == frame.format

    def test_rotate_45_degrees(self, cv2):
        """Test 45 degree rotation."""
        from robo_infra.vision.processing import rotate

        frame = create_test_frame(640, 480)
        result = rotate(frame, 45.0)

        assert result.width == 640
        assert result.height == 480

    def test_rotate_with_center(self, cv2):
        """Test rotation with custom center."""
        from robo_infra.vision.processing import rotate

        frame = create_test_frame(640, 480)
        result = rotate(frame, 45.0, center=(100, 100))

        assert result.width == 640
        assert result.height == 480

    def test_rotate_with_scale(self, cv2):
        """Test rotation with scaling."""
        from robo_infra.vision.processing import rotate

        frame = create_test_frame(640, 480)
        result = rotate(frame, 0.0, scale=0.5)

        assert result.width == 640
        assert result.height == 480

    def test_rotate_border_modes(self, cv2):
        """Test rotation with different border modes."""
        from robo_infra.vision.processing import rotate

        frame = create_test_frame(640, 480)

        for mode in ["constant", "replicate", "reflect", "wrap"]:
            result = rotate(frame, 45.0, border_mode=mode)
            assert result.width == 640

    def test_rotate_invalid_border_mode(self, cv2):
        """Test rotate with invalid border mode raises error."""
        from robo_infra.vision.processing import rotate

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError, match="Unknown border_mode"):
            rotate(frame, 45.0, border_mode="invalid")


# =============================================================================
# Test Grayscale
# =============================================================================


class TestToGrayscale:
    """Tests for to_grayscale function."""

    def test_rgb_to_grayscale(self):
        """Test RGB to grayscale conversion."""
        from robo_infra.vision.processing import to_grayscale

        frame = create_test_frame(640, 480, format=PixelFormat.RGB)
        result = to_grayscale(frame)

        assert result.format == PixelFormat.GRAY
        assert result.channels == 1
        assert result.data.shape == (480, 640)

    def test_bgr_to_grayscale(self):
        """Test BGR to grayscale conversion."""
        from robo_infra.vision.processing import to_grayscale

        frame = create_test_frame(640, 480, format=PixelFormat.BGR)
        result = to_grayscale(frame)

        assert result.format == PixelFormat.GRAY

    def test_grayscale_passthrough(self):
        """Test grayscale frame returns copy."""
        from robo_infra.vision.processing import to_grayscale

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY)
        result = to_grayscale(frame)

        assert result.format == PixelFormat.GRAY
        assert result is not frame  # Should be a copy


# =============================================================================
# Test Threshold
# =============================================================================


class TestThreshold:
    """Tests for threshold function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_binary_threshold(self, cv2):
        """Test binary thresholding."""
        from robo_infra.vision.processing import threshold

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY, pattern="gradient")
        result = threshold(frame, 128)

        assert result.format == PixelFormat.GRAY
        # Binary should have only 0 or 255
        unique_values = np.unique(result.data)
        assert len(unique_values) == 2
        assert 0 in unique_values
        assert 255 in unique_values

    def test_threshold_types(self, cv2):
        """Test different threshold types."""
        from robo_infra.vision.processing import threshold

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY)

        for thresh_type in ["binary", "binary_inv", "trunc", "tozero", "tozero_inv"]:
            result = threshold(frame, 128, thresh_type=thresh_type)
            assert result.format == PixelFormat.GRAY

    def test_otsu_threshold(self, cv2):
        """Test Otsu's thresholding."""
        from robo_infra.vision.processing import threshold

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY)
        result = threshold(frame, 0, thresh_type="otsu")

        assert result.format == PixelFormat.GRAY

    def test_adaptive_threshold(self, cv2):
        """Test adaptive thresholding."""
        from robo_infra.vision.processing import threshold

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY)

        for method in ["adaptive_mean", "adaptive_gaussian"]:
            result = threshold(frame, 0, thresh_type=method)
            assert result.format == PixelFormat.GRAY

    def test_threshold_color_frame(self, cv2):
        """Test threshold converts color to grayscale first."""
        from robo_infra.vision.processing import threshold

        frame = create_test_frame(640, 480, format=PixelFormat.RGB)
        result = threshold(frame, 128)

        assert result.format == PixelFormat.GRAY


# =============================================================================
# Test Blur
# =============================================================================


class TestBlur:
    """Tests for blur function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_gaussian_blur(self, cv2):
        """Test Gaussian blur."""
        from robo_infra.vision.processing import blur

        frame = create_test_frame(640, 480)
        result = blur(frame, kernel_size=5)

        assert result.width == frame.width
        assert result.height == frame.height
        assert result.format == frame.format

    def test_average_blur(self, cv2):
        """Test average blur."""
        from robo_infra.vision.processing import blur

        frame = create_test_frame(640, 480)
        result = blur(frame, kernel_size=5, method="average")

        assert result.width == frame.width

    def test_median_blur(self, cv2):
        """Test median blur."""
        from robo_infra.vision.processing import blur

        frame = create_test_frame(640, 480)
        result = blur(frame, kernel_size=5, method="median")

        assert result.width == frame.width

    def test_blur_adjusts_even_kernel(self, cv2):
        """Test blur adjusts even kernel size to odd."""
        from robo_infra.vision.processing import blur

        frame = create_test_frame(640, 480)
        # Even kernel should be adjusted to odd
        result = blur(frame, kernel_size=4)

        assert result.width == frame.width

    def test_blur_invalid_method(self, cv2):
        """Test blur with invalid method raises error."""
        from robo_infra.vision.processing import blur

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError, match="Unknown blur method"):
            blur(frame, kernel_size=5, method="invalid")


# =============================================================================
# Test Bilateral Filter
# =============================================================================


class TestBilateralFilter:
    """Tests for bilateral_filter function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_bilateral_filter(self, cv2):
        """Test bilateral filter."""
        from robo_infra.vision.processing import bilateral_filter

        frame = create_test_frame(640, 480)
        result = bilateral_filter(frame)

        assert result.width == frame.width
        assert result.height == frame.height


# =============================================================================
# Test Edge Detection
# =============================================================================


class TestEdgeDetect:
    """Tests for edge detection functions."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_canny_edge(self, cv2):
        """Test Canny edge detection."""
        from robo_infra.vision.processing import edge_detect

        frame = create_test_frame(640, 480, pattern="edges")
        result = edge_detect(frame, method="canny")

        assert result.format == PixelFormat.GRAY

    def test_sobel_edge(self, cv2):
        """Test Sobel edge detection."""
        from robo_infra.vision.processing import edge_detect

        frame = create_test_frame(640, 480, pattern="edges")
        result = edge_detect(frame, method="sobel")

        assert result.format == PixelFormat.GRAY

    def test_laplacian_edge(self, cv2):
        """Test Laplacian edge detection."""
        from robo_infra.vision.processing import edge_detect

        frame = create_test_frame(640, 480, pattern="edges")
        result = edge_detect(frame, method="laplacian")

        assert result.format == PixelFormat.GRAY

    def test_scharr_edge(self, cv2):
        """Test Scharr edge detection."""
        from robo_infra.vision.processing import edge_detect

        frame = create_test_frame(640, 480, pattern="edges")
        result = edge_detect(frame, method="scharr")

        assert result.format == PixelFormat.GRAY

    def test_edge_detect_color_frame(self, cv2):
        """Test edge detection on color frame."""
        from robo_infra.vision.processing import edge_detect

        frame = create_test_frame(640, 480, format=PixelFormat.RGB, pattern="edges")
        result = edge_detect(frame, method="canny")

        assert result.format == PixelFormat.GRAY

    def test_edge_detect_invalid_method(self, cv2):
        """Test edge detection with invalid method raises error."""
        from robo_infra.vision.processing import edge_detect

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError, match="Unknown edge detection method"):
            edge_detect(frame, method="invalid")


class TestCannyEdge:
    """Tests for canny_edge function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_canny_edge_custom_thresholds(self, cv2):
        """Test Canny edge with custom thresholds."""
        from robo_infra.vision.processing import canny_edge

        frame = create_test_frame(640, 480, pattern="edges")
        result = canny_edge(frame, threshold1=50, threshold2=150)

        assert result.format == PixelFormat.GRAY


class TestSobelEdge:
    """Tests for sobel_edge function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_sobel_edge_x_only(self, cv2):
        """Test Sobel edge detection in x direction only."""
        from robo_infra.vision.processing import sobel_edge

        frame = create_test_frame(640, 480, pattern="edges")
        result = sobel_edge(frame, dx=1, dy=0)

        assert result.format == PixelFormat.GRAY


class TestLaplacianEdge:
    """Tests for laplacian_edge function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_laplacian_edge_custom_ksize(self, cv2):
        """Test Laplacian edge with custom kernel size."""
        from robo_infra.vision.processing import laplacian_edge

        frame = create_test_frame(640, 480, pattern="edges")
        result = laplacian_edge(frame, ksize=5)

        assert result.format == PixelFormat.GRAY


# =============================================================================
# Test Morphological Operations
# =============================================================================


class TestMorphology:
    """Tests for morphological operations."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_erode(self, cv2):
        """Test erosion."""
        from robo_infra.vision.processing import erode

        frame = create_test_frame(640, 480, pattern="checkerboard")
        result = erode(frame, kernel_size=3)

        assert result.width == frame.width

    def test_dilate(self, cv2):
        """Test dilation."""
        from robo_infra.vision.processing import dilate

        frame = create_test_frame(640, 480, pattern="checkerboard")
        result = dilate(frame, kernel_size=3)

        assert result.width == frame.width

    def test_morphology_open(self, cv2):
        """Test opening operation."""
        from robo_infra.vision.processing import morphology

        frame = create_test_frame(640, 480, pattern="checkerboard")
        result = morphology(frame, "open", kernel_size=3)

        assert result.width == frame.width

    def test_morphology_close(self, cv2):
        """Test closing operation."""
        from robo_infra.vision.processing import morphology

        frame = create_test_frame(640, 480, pattern="checkerboard")
        result = morphology(frame, "close", kernel_size=3)

        assert result.width == frame.width

    def test_morphology_gradient(self, cv2):
        """Test morphological gradient."""
        from robo_infra.vision.processing import morphology

        frame = create_test_frame(640, 480, pattern="checkerboard")
        result = morphology(frame, "gradient", kernel_size=3)

        assert result.width == frame.width

    def test_morphology_invalid_operation(self, cv2):
        """Test morphology with invalid operation."""
        from robo_infra.vision.processing import morphology

        frame = create_test_frame(640, 480)

        with pytest.raises(ValueError, match="Unknown operation"):
            morphology(frame, "invalid", kernel_size=3)


# =============================================================================
# Test Histogram Equalization
# =============================================================================


class TestHistogramEqualize:
    """Tests for histogram_equalize function."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_histogram_equalize_grayscale(self, cv2):
        """Test histogram equalization on grayscale."""
        from robo_infra.vision.processing import histogram_equalize

        frame = create_test_frame(640, 480, format=PixelFormat.GRAY)
        result = histogram_equalize(frame)

        assert result.format == PixelFormat.GRAY

    def test_histogram_equalize_color(self, cv2):
        """Test histogram equalization on color."""
        from robo_infra.vision.processing import histogram_equalize

        frame = create_test_frame(640, 480, format=PixelFormat.RGB)
        result = histogram_equalize(frame)

        assert result.channels == 3


# =============================================================================
# Test Create Test Frame
# =============================================================================


class TestCreateTestFrame:
    """Tests for create_test_frame utility."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_create_solid(self, cv2):
        """Test creating solid pattern."""
        from robo_infra.vision.processing import create_test_frame

        frame = create_test_frame(pattern="solid", color=(255, 0, 0))

        assert frame.width == 640
        assert frame.height == 480
        # All pixels should have same color
        assert np.all(frame.data == (255, 0, 0))

    def test_create_gradient(self, cv2):
        """Test creating gradient pattern."""
        from robo_infra.vision.processing import create_test_frame

        frame = create_test_frame(pattern="gradient")

        assert frame.width == 640
        # Left edge should be darker than right
        assert frame.data[0, 0, 0] < frame.data[0, -1, 0]

    def test_create_checkerboard(self, cv2):
        """Test creating checkerboard pattern."""
        from robo_infra.vision.processing import create_test_frame

        frame = create_test_frame(pattern="checkerboard")

        # Should have alternating values
        unique = np.unique(frame.data)
        assert len(unique) == 2

    def test_create_noise(self, cv2):
        """Test creating noise pattern."""
        from robo_infra.vision.processing import create_test_frame

        frame = create_test_frame(pattern="noise")

        # Should have many unique values
        unique = np.unique(frame.data)
        assert len(unique) > 100

    def test_create_grayscale(self, cv2):
        """Test creating grayscale frame."""
        from robo_infra.vision.processing import create_test_frame

        frame = create_test_frame(format=PixelFormat.GRAY)

        assert frame.format == PixelFormat.GRAY
        assert frame.channels == 1
