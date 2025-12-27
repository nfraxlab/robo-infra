"""Tests for vision/markers.py - Fiducial marker detection."""

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
) -> Frame:
    """Create a test frame."""
    if format == PixelFormat.GRAY:
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


# =============================================================================
# Test MarkerPose
# =============================================================================


class TestMarkerPose:
    """Tests for MarkerPose dataclass."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_marker_pose_creation(self, cv2):
        """Test creating a MarkerPose."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.5, 0.1, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=42,
        )

        assert pose.translation == (0.5, 0.1, 1.0)
        assert pose.marker_id == 42

    def test_marker_pose_distance(self, cv2):
        """Test MarkerPose distance calculation."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.0, 0.0, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        assert pose.distance == pytest.approx(1.0)

    def test_marker_pose_distance_3d(self, cv2):
        """Test MarkerPose distance with 3D translation."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(3.0, 4.0, 0.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        assert pose.distance == pytest.approx(5.0)  # 3-4-5 triangle

    def test_marker_pose_rotation_matrix(self, cv2):
        """Test MarkerPose rotation matrix."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.0, 0.0, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        R = pose.rotation_matrix
        assert R.shape == (3, 3)
        # Identity rotation for zero Rodrigues vector
        np.testing.assert_array_almost_equal(R, np.eye(3))

    def test_marker_pose_euler_angles(self, cv2):
        """Test MarkerPose euler angles."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.0, 0.0, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        roll, pitch, yaw = pose.euler_angles
        assert roll == pytest.approx(0.0, abs=1e-6)
        assert pitch == pytest.approx(0.0, abs=1e-6)
        assert yaw == pytest.approx(0.0, abs=1e-6)

    def test_marker_pose_quaternion(self, cv2):
        """Test MarkerPose quaternion."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.0, 0.0, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        w, x, y, z = pose.quaternion
        # Identity quaternion
        assert w == pytest.approx(1.0, abs=1e-3)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)
        assert z == pytest.approx(0.0, abs=1e-6)

    def test_marker_pose_transform_point(self, cv2):
        """Test MarkerPose transform_point."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(1.0, 2.0, 3.0),
            rotation=np.array([0.0, 0.0, 0.0]),  # No rotation
            marker_id=1,
        )

        # Point at origin in marker frame should be at translation in camera frame
        result = pose.transform_point((0.0, 0.0, 0.0))
        assert result[0] == pytest.approx(1.0)
        assert result[1] == pytest.approx(2.0)
        assert result[2] == pytest.approx(3.0)


# =============================================================================
# Test ArUcoMarker
# =============================================================================


class TestArUcoMarker:
    """Tests for ArUcoMarker dataclass."""

    def test_aruco_marker_creation(self):
        """Test creating an ArUcoMarker."""
        from robo_infra.vision.markers import ArUcoMarker

        corners = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32
        )

        marker = ArUcoMarker(
            id=42,
            corners=corners,
            center=(150.0, 150.0),
        )

        assert marker.id == 42
        assert marker.center == (150.0, 150.0)

    def test_aruco_marker_area(self):
        """Test ArUcoMarker area calculation."""
        from robo_infra.vision.markers import ArUcoMarker

        # 100x100 square
        corners = np.array(
            [[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.float32
        )

        marker = ArUcoMarker(
            id=1,
            corners=corners,
            center=(50.0, 50.0),
        )

        assert marker.area == pytest.approx(10000.0)

    def test_aruco_marker_size(self):
        """Test ArUcoMarker size calculation."""
        from robo_infra.vision.markers import ArUcoMarker

        # 100x100 square
        corners = np.array(
            [[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.float32
        )

        marker = ArUcoMarker(
            id=1,
            corners=corners,
            center=(50.0, 50.0),
        )

        assert marker.size == pytest.approx(100.0)

    def test_aruco_marker_bounding_box(self):
        """Test ArUcoMarker bounding box."""
        from robo_infra.vision.markers import ArUcoMarker

        corners = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32
        )

        marker = ArUcoMarker(
            id=1,
            corners=corners,
            center=(150.0, 150.0),
        )

        x, y, w, h = marker.bounding_box
        assert x == 100
        assert y == 100
        assert w == 100
        assert h == 100


# =============================================================================
# Test AprilTag
# =============================================================================


class TestAprilTag:
    """Tests for AprilTag dataclass."""

    def test_apriltag_creation(self):
        """Test creating an AprilTag."""
        from robo_infra.vision.markers import AprilTag

        corners = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32
        )

        tag = AprilTag(
            id=5,
            family="tag36h11",
            corners=corners,
            center=(150.0, 150.0),
        )

        assert tag.id == 5
        assert tag.family == "tag36h11"

    def test_apriltag_size(self):
        """Test AprilTag size calculation."""
        from robo_infra.vision.markers import AprilTag

        corners = np.array(
            [[0, 0], [50, 0], [50, 50], [0, 50]], dtype=np.float32
        )

        tag = AprilTag(
            id=1,
            family="tag36h11",
            corners=corners,
            center=(25.0, 25.0),
        )

        assert tag.size == pytest.approx(50.0)


# =============================================================================
# Test ArUcoDictionary Enum
# =============================================================================


class TestArUcoDictionary:
    """Tests for ArUcoDictionary enum."""

    def test_dictionary_values(self):
        """Test dictionary enum values."""
        from robo_infra.vision.markers import ArUcoDictionary

        assert ArUcoDictionary.DICT_4X4_50.value == "DICT_4X4_50"
        assert ArUcoDictionary.DICT_6X6_250.value == "DICT_6X6_250"
        assert ArUcoDictionary.DICT_APRILTAG_36h11.value == "DICT_APRILTAG_36h11"

    def test_list_dictionaries(self):
        """Test listing dictionaries."""
        from robo_infra.vision.markers import list_aruco_dictionaries

        dicts = list_aruco_dictionaries()

        assert "DICT_4X4_50" in dicts
        assert "DICT_6X6_250" in dicts
        assert len(dicts) >= 17  # At least 17 dictionaries


# =============================================================================
# Test ArUcoDetector
# =============================================================================


class TestArUcoDetectorSimulated:
    """Tests for ArUcoDetector in simulation mode."""

    def test_create_detector(self):
        """Test creating an ArUco detector."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector(dictionary="DICT_4X4_50", simulation=True)

        assert detector.dictionary_name == "DICT_4X4_50"

    def test_create_detector_with_enum(self):
        """Test creating detector with enum."""
        from robo_infra.vision.markers import ArUcoDetector, ArUcoDictionary

        detector = ArUcoDetector(
            dictionary=ArUcoDictionary.DICT_6X6_250, simulation=True
        )

        assert detector.dictionary_name == "DICT_6X6_250"

    def test_detect_simulated(self):
        """Test detection in simulation mode."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector(simulation=True)
        frame = create_test_frame()

        markers = detector.detect(frame)

        assert isinstance(markers, list)
        assert len(markers) == 0  # Simulation returns empty

    def test_estimate_pose_simulated(self):
        """Test pose estimation in simulation mode."""
        from robo_infra.sensors.camera import CameraIntrinsics
        from robo_infra.vision.markers import ArUcoDetector, ArUcoMarker

        detector = ArUcoDetector(simulation=True)

        # Create mock marker
        corners = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32
        )
        marker = ArUcoMarker(id=1, corners=corners, center=(150, 150))

        # Create mock intrinsics
        intrinsics = CameraIntrinsics(
            fx=600, fy=600, cx=320, cy=240, width=640, height=480
        )

        pose = detector.estimate_pose(marker, intrinsics, marker_size=0.1)

        assert pose.marker_id == 1
        assert pose.translation == (0.5, 0.0, 1.0)  # Simulated values

    def test_draw_markers_simulated(self):
        """Test drawing markers in simulation mode."""
        from robo_infra.vision.markers import ArUcoDetector, ArUcoMarker

        detector = ArUcoDetector(simulation=True)
        frame = create_test_frame()

        corners = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32
        )
        markers = [ArUcoMarker(id=1, corners=corners, center=(150, 150))]

        result = detector.draw_markers(frame, markers)

        assert result.width == frame.width


class TestArUcoDetectorMocked:
    """Tests for ArUcoDetector with mocked OpenCV."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    @pytest.fixture
    def cv2_aruco(self, cv2):
        """Get cv2.aruco module, skip if not available."""
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_detector_init(self, cv2_aruco):
        """Test detector initialization with real OpenCV."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector(dictionary="DICT_4X4_50")

        assert detector.dictionary_name == "DICT_4X4_50"

    def test_detect_no_markers(self, cv2_aruco):
        """Test detection on frame with no markers."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame()  # Random noise, no markers

        markers = detector.detect(frame)

        # Random noise shouldn't produce markers
        assert isinstance(markers, list)

    def test_generate_marker(self, cv2_aruco):
        """Test generating a marker image."""
        from robo_infra.vision.markers import ArUcoDetector

        marker = ArUcoDetector.generate_marker("DICT_4X4_50", 42, size=200)

        assert marker.width == 200
        assert marker.height == 200
        assert marker.format == PixelFormat.GRAY


# =============================================================================
# Test AprilTagDetector
# =============================================================================


class TestAprilTagDetector:
    """Tests for AprilTagDetector."""

    def test_create_detector(self):
        """Test creating an AprilTag detector."""
        from robo_infra.vision.markers import AprilTagDetector

        detector = AprilTagDetector(family="tag36h11", simulation=True)

        assert detector.family == "tag36h11"

    def test_list_families(self):
        """Test listing AprilTag families."""
        from robo_infra.vision.markers import list_apriltag_families

        families = list_apriltag_families()

        assert "tag36h11" in families
        assert "tag16h5" in families
        assert len(families) == 4

    def test_invalid_family(self):
        """Test creating detector with invalid family."""
        from robo_infra.vision.markers import AprilTagDetector

        with pytest.raises(ValueError, match="Unknown AprilTag family"):
            AprilTagDetector(family="invalid")

    def test_detect_simulated(self):
        """Test detection in simulation mode."""
        from robo_infra.vision.markers import AprilTagDetector

        detector = AprilTagDetector(simulation=True)
        frame = create_test_frame()

        tags = detector.detect(frame)

        assert isinstance(tags, list)
        assert len(tags) == 0  # Simulation returns empty


# =============================================================================
# Test Utility Functions
# =============================================================================


class TestMarkerUtilities:
    """Tests for marker utility functions."""

    @pytest.fixture
    def cv2_aruco(self):
        """Get cv2.aruco module, skip if not available."""
        cv2 = pytest.importorskip("cv2")
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_create_marker_board(self, cv2_aruco):
        """Test creating a marker board."""
        from robo_infra.vision.markers import create_marker_board

        board = create_marker_board(
            "DICT_4X4_50",
            rows=3,
            cols=4,
            marker_size=100,
            marker_separation=20,
        )

        # Width: 4 markers * 100 + 3 separations * 20 = 460
        # Height: 3 markers * 100 + 2 separations * 20 = 340
        assert board.width == 460
        assert board.height == 340
        assert board.format == PixelFormat.GRAY


# =============================================================================
# Test Corner Refinement Methods
# =============================================================================


class TestCornerRefinement:
    """Tests for corner refinement methods."""

    @pytest.fixture
    def cv2_aruco(self):
        """Get cv2.aruco module, skip if not available."""
        cv2 = pytest.importorskip("cv2")
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_corner_refinement_subpix(self, cv2_aruco):
        """Test sub-pixel corner refinement."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector(corner_refinement_method="subpix")
        assert detector is not None

    def test_corner_refinement_contour(self, cv2_aruco):
        """Test contour corner refinement."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector(corner_refinement_method="contour")
        assert detector is not None

    def test_corner_refinement_none(self, cv2_aruco):
        """Test no corner refinement."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector(corner_refinement_method="none")
        assert detector is not None

    def test_corner_refinement_invalid(self, cv2_aruco):
        """Test invalid corner refinement method."""
        from robo_infra.vision.markers import ArUcoDetector

        # Invalid method should raise on first detection (lazy init)
        detector = ArUcoDetector(corner_refinement_method="invalid")
        frame = create_test_frame()

        with pytest.raises(ValueError, match="Unknown corner_refinement_method"):
            detector.detect(frame)


# =============================================================================
# Test Detection with Different Formats
# =============================================================================


class TestDetectionFormats:
    """Tests for detection with different frame formats."""

    @pytest.fixture
    def cv2_aruco(self):
        """Get cv2.aruco module, skip if not available."""
        cv2 = pytest.importorskip("cv2")
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_detect_grayscale(self, cv2_aruco):
        """Test detection on grayscale frame."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame(format=PixelFormat.GRAY)

        markers = detector.detect(frame)
        assert isinstance(markers, list)

    def test_detect_rgb(self, cv2_aruco):
        """Test detection on RGB frame."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame(format=PixelFormat.RGB)

        markers = detector.detect(frame)
        assert isinstance(markers, list)

    def test_detect_bgr(self, cv2_aruco):
        """Test detection on BGR frame."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame(format=PixelFormat.BGR)

        markers = detector.detect(frame)
        assert isinstance(markers, list)


# =============================================================================
# Phase 5.9.2.2 - Marker Detection Comprehensive Tests
# =============================================================================


class TestArUcoDetectorDetectComprehensive:
    """Comprehensive tests for ArUcoDetector.detect method."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    @pytest.fixture
    def cv2_aruco(self, cv2):
        """Get cv2.aruco module, skip if not available."""
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_detect_generated_marker(self, cv2_aruco):
        """Test detecting a generated marker."""
        from robo_infra.vision.markers import ArUcoDetector

        # Generate a marker
        marker_frame = ArUcoDetector.generate_marker("DICT_4X4_50", 7, size=200)

        # Embed it in a larger frame
        data = np.full((480, 640), 255, dtype=np.uint8)
        # Place marker in center
        x, y = 220, 140
        data[y : y + 200, x : x + 200] = marker_frame.data

        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.GRAY,
            frame_number=0,
        )

        detector = ArUcoDetector(dictionary="DICT_4X4_50")
        markers = detector.detect(frame)

        assert len(markers) >= 1
        assert any(m.id == 7 for m in markers)

    def test_detect_multiple_markers(self, cv2_aruco):
        """Test detecting multiple markers."""
        from robo_infra.vision.markers import ArUcoDetector

        # Generate multiple markers
        data = np.full((480, 640), 255, dtype=np.uint8)

        marker_ids = [1, 5, 10]
        positions = [(50, 50), (250, 50), (450, 50)]

        for mid, (px, py) in zip(marker_ids, positions, strict=True):
            marker_frame = ArUcoDetector.generate_marker("DICT_4X4_50", mid, size=100)
            data[py : py + 100, px : px + 100] = marker_frame.data

        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.GRAY,
            frame_number=0,
        )

        detector = ArUcoDetector(dictionary="DICT_4X4_50")
        markers = detector.detect(frame)

        detected_ids = {m.id for m in markers}
        assert len(detected_ids) >= 2  # Should detect at least 2

    def test_detect_rgb_format(self, cv2_aruco):
        """Test detection converts RGB to grayscale."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame(format=PixelFormat.RGB)

        markers = detector.detect(frame)
        assert isinstance(markers, list)

    def test_detect_bgr_format(self, cv2_aruco):
        """Test detection converts BGR to grayscale."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame(format=PixelFormat.BGR)

        markers = detector.detect(frame)
        assert isinstance(markers, list)


class TestArUcoMarkerPropertiesComprehensive:
    """Comprehensive tests for ArUcoMarker properties."""

    def test_marker_size_calculation(self):
        """Test marker size is average side length."""
        from robo_infra.vision.markers import ArUcoMarker

        # 100x100 square
        corners = np.array(
            [[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.float32
        )

        marker = ArUcoMarker(id=1, corners=corners, center=(50.0, 50.0))

        assert marker.size == pytest.approx(100.0)

    def test_marker_size_non_square(self):
        """Test marker size with non-square marker."""
        from robo_infra.vision.markers import ArUcoMarker

        # Rectangle: 100 wide, 50 tall
        corners = np.array(
            [[0, 0], [100, 0], [100, 50], [0, 50]], dtype=np.float32
        )

        marker = ArUcoMarker(id=1, corners=corners, center=(50.0, 25.0))

        # Average of 100, 50, 100, 50 = 75
        assert marker.size == pytest.approx(75.0)

    def test_marker_bounding_box_offset(self):
        """Test bounding box with offset marker."""
        from robo_infra.vision.markers import ArUcoMarker

        corners = np.array(
            [[200, 150], [300, 150], [300, 250], [200, 250]], dtype=np.float32
        )

        marker = ArUcoMarker(id=1, corners=corners, center=(250.0, 200.0))

        x, y, w, h = marker.bounding_box
        assert x == 200
        assert y == 150
        assert w == 100
        assert h == 100

    def test_marker_area_auto_calculation(self):
        """Test that area is auto-calculated in __post_init__."""
        from robo_infra.vision.markers import ArUcoMarker

        corners = np.array(
            [[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.float32
        )

        marker = ArUcoMarker(id=1, corners=corners, center=(50.0, 50.0))

        # Area should be calculated
        assert marker.area == pytest.approx(10000.0)


class TestArUcoPoseEstimationComprehensive:
    """Comprehensive tests for ArUco pose estimation."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    @pytest.fixture
    def cv2_aruco(self, cv2):
        """Get cv2.aruco module, skip if not available."""
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    @pytest.fixture
    def intrinsics(self):
        """Create camera intrinsics."""
        from robo_infra.sensors.camera import CameraIntrinsics

        return CameraIntrinsics(
            fx=600, fy=600, cx=320, cy=240, width=640, height=480
        )

    def test_pose_estimation_returns_marker_id(self, cv2_aruco, intrinsics):
        """Test pose estimation preserves marker ID."""
        from robo_infra.vision.markers import ArUcoDetector, ArUcoMarker

        detector = ArUcoDetector()

        corners = np.array(
            [[280, 200], [360, 200], [360, 280], [280, 280]], dtype=np.float32
        )
        marker = ArUcoMarker(id=42, corners=corners, center=(320, 240))

        pose = detector.estimate_pose(marker, intrinsics, marker_size=0.1)

        assert pose.marker_id == 42

    def test_pose_distance_positive(self, cv2_aruco, intrinsics):
        """Test pose distance is always positive."""
        from robo_infra.vision.markers import ArUcoDetector, ArUcoMarker

        detector = ArUcoDetector()

        corners = np.array(
            [[280, 200], [360, 200], [360, 280], [280, 280]], dtype=np.float32
        )
        marker = ArUcoMarker(id=1, corners=corners, center=(320, 240))

        pose = detector.estimate_pose(marker, intrinsics, marker_size=0.1)

        assert pose.distance > 0

    def test_pose_translation_has_z(self, cv2_aruco, intrinsics):
        """Test pose translation has positive Z (in front of camera)."""
        from robo_infra.vision.markers import ArUcoDetector, ArUcoMarker

        detector = ArUcoDetector()

        corners = np.array(
            [[280, 200], [360, 200], [360, 280], [280, 280]], dtype=np.float32
        )
        marker = ArUcoMarker(id=1, corners=corners, center=(320, 240))

        pose = detector.estimate_pose(marker, intrinsics, marker_size=0.1)

        # Z should be positive (marker is in front of camera)
        assert pose.translation[2] > 0


class TestArUcoDrawMarkersComprehensive:
    """Comprehensive tests for ArUcoDetector.draw_markers method."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    @pytest.fixture
    def cv2_aruco(self, cv2):
        """Get cv2.aruco module, skip if not available."""
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    @pytest.fixture
    def sample_marker(self):
        """Create a sample marker."""
        from robo_infra.vision.markers import ArUcoMarker

        corners = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32
        )
        return ArUcoMarker(id=5, corners=corners, center=(150.0, 150.0))

    def test_draw_markers_rgb(self, cv2_aruco, sample_marker):
        """Test drawing on RGB frame."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame(format=PixelFormat.RGB)

        result = detector.draw_markers(frame, [sample_marker])

        assert result.format == PixelFormat.RGB

    def test_draw_markers_grayscale(self, cv2_aruco, sample_marker):
        """Test drawing on grayscale frame converts to color."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame(format=PixelFormat.GRAY)

        result = detector.draw_markers(frame, [sample_marker])

        assert result.format == PixelFormat.BGR

    def test_draw_markers_custom_color(self, cv2_aruco, sample_marker):
        """Test drawing with custom color."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame()

        result = detector.draw_markers(
            frame, [sample_marker], color=(255, 0, 0), thickness=3
        )

        assert result.width == frame.width

    def test_draw_markers_hide_id(self, cv2_aruco, sample_marker):
        """Test drawing without showing ID."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame()

        result = detector.draw_markers(frame, [sample_marker], show_id=False)

        assert result.width == frame.width

    def test_draw_markers_empty_list(self, cv2_aruco):
        """Test drawing empty marker list."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame()

        result = detector.draw_markers(frame, [])

        assert result.width == frame.width


class TestArUcoGenerateMarkerComprehensive:
    """Comprehensive tests for ArUcoDetector.generate_marker method."""

    @pytest.fixture
    def cv2_aruco(self):
        """Get cv2.aruco module, skip if not available."""
        cv2 = pytest.importorskip("cv2")
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_generate_marker_4x4(self, cv2_aruco):
        """Test generating 4x4 marker."""
        from robo_infra.vision.markers import ArUcoDetector

        marker = ArUcoDetector.generate_marker("DICT_4X4_50", 0, size=100)

        assert marker.width == 100
        assert marker.height == 100

    def test_generate_marker_6x6(self, cv2_aruco):
        """Test generating 6x6 marker."""
        from robo_infra.vision.markers import ArUcoDetector

        marker = ArUcoDetector.generate_marker("DICT_6X6_250", 42, size=150)

        assert marker.width == 150
        assert marker.height == 150

    def test_generate_marker_different_sizes(self, cv2_aruco):
        """Test generating markers of different sizes."""
        from robo_infra.vision.markers import ArUcoDetector

        for size in [50, 100, 200, 500]:
            marker = ArUcoDetector.generate_marker("DICT_4X4_50", 1, size=size)
            assert marker.width == size
            assert marker.height == size

    def test_generate_marker_border_bits(self, cv2_aruco):
        """Test generating marker with custom border."""
        from robo_infra.vision.markers import ArUcoDetector

        marker1 = ArUcoDetector.generate_marker(
            "DICT_4X4_50", 1, size=100, border_bits=1
        )
        marker2 = ArUcoDetector.generate_marker(
            "DICT_4X4_50", 1, size=100, border_bits=2
        )

        # Different border sizes produce different images
        assert not np.array_equal(marker1.data, marker2.data)

    def test_generate_marker_is_grayscale(self, cv2_aruco):
        """Test generated marker is grayscale."""
        from robo_infra.vision.markers import ArUcoDetector

        marker = ArUcoDetector.generate_marker("DICT_4X4_50", 1, size=100)

        assert marker.format == PixelFormat.GRAY
        assert len(marker.data.shape) == 2


class TestAprilTagDetectorComprehensive:
    """Comprehensive tests for AprilTagDetector."""

    @pytest.fixture
    def cv2_aruco(self):
        """Get cv2.aruco module, skip if not available."""
        cv2 = pytest.importorskip("cv2")
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_apriltag_all_families(self, cv2_aruco):
        """Test creating detector for all families."""
        from robo_infra.vision.markers import AprilTagDetector, list_apriltag_families

        for family in list_apriltag_families():
            detector = AprilTagDetector(family=family)
            assert detector.family == family

    def test_apriltag_detect_empty(self, cv2_aruco):
        """Test AprilTag detection on empty frame."""
        from robo_infra.vision.markers import AprilTagDetector

        detector = AprilTagDetector()
        frame = create_test_frame()

        tags = detector.detect(frame)

        assert isinstance(tags, list)

    def test_apriltag_pose_estimation(self, cv2_aruco):
        """Test AprilTag pose estimation."""
        from robo_infra.sensors.camera import CameraIntrinsics
        from robo_infra.vision.markers import AprilTag, AprilTagDetector

        detector = AprilTagDetector()

        corners = np.array(
            [[280, 200], [360, 200], [360, 280], [280, 280]], dtype=np.float32
        )
        tag = AprilTag(
            id=5,
            family="tag36h11",
            corners=corners,
            center=(320.0, 240.0),
        )

        intrinsics = CameraIntrinsics(
            fx=600, fy=600, cx=320, cy=240, width=640, height=480
        )

        pose = detector.estimate_pose(tag, intrinsics, tag_size=0.1)

        assert pose.marker_id == 5
        assert pose.distance > 0

    def test_apriltag_draw_tags(self, cv2_aruco):
        """Test AprilTag drawing."""
        from robo_infra.vision.markers import AprilTag, AprilTagDetector

        detector = AprilTagDetector()
        frame = create_test_frame()

        corners = np.array(
            [[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32
        )
        tags = [
            AprilTag(
                id=1,
                family="tag36h11",
                corners=corners,
                center=(150.0, 150.0),
            )
        ]

        result = detector.draw_tags(frame, tags)

        assert result.width == frame.width


class TestMarkerPoseComprehensive:
    """Comprehensive tests for MarkerPose properties."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    def test_pose_distance_3d_pythagorean(self, cv2):
        """Test distance calculation with 3D translation."""
        from robo_infra.vision.markers import MarkerPose

        # 3-4-5 right triangle in x-z, then extended
        pose = MarkerPose(
            translation=(3.0, 4.0, 0.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        assert pose.distance == pytest.approx(5.0)

    def test_pose_rotation_matrix_identity(self, cv2):
        """Test rotation matrix is identity for zero rotation."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.0, 0.0, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        R = pose.rotation_matrix
        np.testing.assert_array_almost_equal(R, np.eye(3))

    def test_pose_euler_angles_zero(self, cv2):
        """Test euler angles are zero for identity rotation."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.0, 0.0, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        roll, pitch, yaw = pose.euler_angles
        assert roll == pytest.approx(0.0, abs=1e-6)
        assert pitch == pytest.approx(0.0, abs=1e-6)
        assert yaw == pytest.approx(0.0, abs=1e-6)

    def test_pose_quaternion_identity(self, cv2):
        """Test quaternion is identity for zero rotation."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(0.0, 0.0, 1.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        w, x, y, z = pose.quaternion
        assert w == pytest.approx(1.0, abs=1e-3)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)
        assert z == pytest.approx(0.0, abs=1e-6)

    def test_pose_transform_point_translation_only(self, cv2):
        """Test transform_point with translation only."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(1.0, 2.0, 3.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        result = pose.transform_point((0.0, 0.0, 0.0))

        assert result[0] == pytest.approx(1.0)
        assert result[1] == pytest.approx(2.0)
        assert result[2] == pytest.approx(3.0)

    def test_pose_transform_point_with_offset(self, cv2):
        """Test transform_point with offset point."""
        from robo_infra.vision.markers import MarkerPose

        pose = MarkerPose(
            translation=(1.0, 2.0, 3.0),
            rotation=np.array([0.0, 0.0, 0.0]),
            marker_id=1,
        )

        result = pose.transform_point((1.0, 1.0, 1.0))

        assert result[0] == pytest.approx(2.0)
        assert result[1] == pytest.approx(3.0)
        assert result[2] == pytest.approx(4.0)


class TestCreateMarkerBoardComprehensive:
    """Comprehensive tests for create_marker_board utility."""

    @pytest.fixture
    def cv2_aruco(self):
        """Get cv2.aruco module, skip if not available."""
        cv2 = pytest.importorskip("cv2")
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_create_board_dimensions(self, cv2_aruco):
        """Test board has correct dimensions."""
        from robo_infra.vision.markers import create_marker_board

        board = create_marker_board(
            "DICT_4X4_50",
            rows=2,
            cols=3,
            marker_size=100,
            marker_separation=20,
        )

        # Width: 3 * 100 + 2 * 20 = 340
        # Height: 2 * 100 + 1 * 20 = 220
        assert board.width == 340
        assert board.height == 220

    def test_create_board_single_marker(self, cv2_aruco):
        """Test creating board with single marker."""
        from robo_infra.vision.markers import create_marker_board

        board = create_marker_board(
            "DICT_4X4_50",
            rows=1,
            cols=1,
            marker_size=200,
            marker_separation=0,
        )

        assert board.width == 200
        assert board.height == 200

    def test_create_board_custom_start_id(self, cv2_aruco):
        """Test creating board with custom start ID."""
        from robo_infra.vision.markers import create_marker_board

        board = create_marker_board(
            "DICT_4X4_50",
            rows=2,
            cols=2,
            marker_size=100,
            marker_separation=10,
            start_id=10,
        )

        # Should create board successfully
        assert board.format == PixelFormat.GRAY


# =============================================================================
# Integration Tests
# =============================================================================


class TestMarkerDetectionIntegration:
    """Integration tests for marker detection pipeline."""

    @pytest.fixture
    def cv2(self):
        """Get cv2 module, skip if not available."""
        return pytest.importorskip("cv2")

    @pytest.fixture
    def cv2_aruco(self, cv2):
        """Get cv2.aruco module, skip if not available."""
        if not hasattr(cv2, "aruco"):
            pytest.skip("cv2.aruco not available")
        return cv2.aruco

    def test_generate_detect_roundtrip(self, cv2_aruco):
        """Test generating a marker and detecting it."""
        from robo_infra.vision.markers import ArUcoDetector

        # Generate marker
        marker_id = 15
        marker_frame = ArUcoDetector.generate_marker("DICT_4X4_50", marker_id, size=200)

        # Create larger frame with marker
        data = np.full((480, 640), 255, dtype=np.uint8)
        data[140:340, 220:420] = marker_frame.data

        frame = Frame(
            data=data,
            timestamp=time.monotonic(),
            width=640,
            height=480,
            format=PixelFormat.GRAY,
            frame_number=0,
        )

        # Detect
        detector = ArUcoDetector(dictionary="DICT_4X4_50")
        markers = detector.detect(frame)

        assert len(markers) >= 1
        assert markers[0].id == marker_id

    def test_detect_and_draw_pipeline(self, cv2_aruco):
        """Test detect and draw pipeline."""
        from robo_infra.vision.markers import ArUcoDetector

        detector = ArUcoDetector()
        frame = create_test_frame()

        markers = detector.detect(frame)
        result = detector.draw_markers(frame, markers)

        assert result.width == frame.width

    def test_apriltag_full_pipeline(self, cv2_aruco):
        """Test AprilTag detection pipeline."""
        from robo_infra.vision.markers import AprilTagDetector

        detector = AprilTagDetector(family="tag36h11")
        frame = create_test_frame()

        tags = detector.detect(frame)
        result = detector.draw_tags(frame, tags)

        assert result.width == frame.width
