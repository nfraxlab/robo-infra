"""Tests for vision/markers.py - Fiducial marker detection."""

from __future__ import annotations

import os
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
        from robo_infra.vision.markers import ArUcoDictionary, ArUcoDetector

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
        from robo_infra.vision.markers import ArUcoDetector, ArUcoMarker
        from robo_infra.sensors.camera import CameraIntrinsics

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
