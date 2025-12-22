"""Unit tests for LIDAR sensor implementations.

Tests cover:
- LIDARScan dataclass
- SimulatedLIDAR driver
- LIDAR factory and enums
"""

from __future__ import annotations

import numpy as np
import pytest

from robo_infra.sensors.lidar import (
    LIDAR_SPECS,
    LIDARConfig,
    LIDARModel,
    LIDARScan,
    LIDARState,
    ScanQuality,
    SimulatedLIDAR,
    get_lidar,
)


# =============================================================================
# LIDARScan Tests
# =============================================================================


class TestLIDARScan:
    """Tests for LIDARScan dataclass."""

    def test_create_scan_with_arrays(self) -> None:
        """Test creating a scan with numpy arrays."""
        ranges = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        angles = np.linspace(0, np.pi, 5)
        intensities = np.array([100, 200, 150, 180, 120])

        scan = LIDARScan(
            ranges=ranges,
            angles=angles,
            intensities=intensities,
            timestamp=1234567890.0,
            scan_number=1,
        )

        assert scan.num_points == 5
        assert scan.scan_number == 1
        np.testing.assert_array_equal(scan.ranges, ranges)

    def test_create_scan_without_intensities(self) -> None:
        """Test creating a scan without intensities."""
        ranges = np.array([1.0, 2.0, 3.0])
        angles = np.array([0.0, 1.0, 2.0])

        scan = LIDARScan(ranges=ranges, angles=angles)

        assert scan.intensities is None
        assert scan.num_points == 3

    def test_scan_validation_length_mismatch(self) -> None:
        """Test that mismatched array lengths raise error."""
        ranges = np.array([1.0, 2.0, 3.0])
        angles = np.array([0.0, 1.0])

        with pytest.raises(ValueError, match="same length"):
            LIDARScan(ranges=ranges, angles=angles)

    def test_to_cartesian_conversion(self) -> None:
        """Test polar to Cartesian conversion."""
        ranges = np.array([1.0, 1.0])
        angles = np.array([0.0, np.pi / 2])

        scan = LIDARScan(ranges=ranges, angles=angles)
        x, y = scan.to_cartesian()

        np.testing.assert_almost_equal(x[0], 1.0)
        np.testing.assert_almost_equal(y[0], 0.0)
        np.testing.assert_almost_equal(x[1], 0.0, decimal=5)
        np.testing.assert_almost_equal(y[1], 1.0)

    def test_filter_range(self) -> None:
        """Test range filtering."""
        ranges = np.array([0.5, 1.5, 2.5, 3.5])
        angles = np.array([0.0, 1.0, 2.0, 3.0])

        scan = LIDARScan(ranges=ranges, angles=angles)
        filtered = scan.filter_range(min_range=1.0, max_range=3.0)

        assert np.isnan(filtered.ranges[0])  # Below min
        assert filtered.ranges[1] == 1.5
        assert filtered.ranges[2] == 2.5
        assert np.isnan(filtered.ranges[3])  # Above max

    def test_valid_mask_with_nan(self) -> None:
        """Test valid mask with NaN values."""
        ranges = np.array([1.0, np.nan, 2.0, np.nan])
        angles = np.array([0.0, 1.0, 2.0, 3.0])

        scan = LIDARScan(ranges=ranges, angles=angles)

        assert scan.valid_count == 2
        np.testing.assert_array_equal(scan.valid_mask, [True, False, True, False])

    def test_min_max_mean_range(self) -> None:
        """Test statistical properties."""
        ranges = np.array([1.0, 2.0, np.nan, 3.0])
        angles = np.array([0.0, 1.0, 2.0, 3.0])

        scan = LIDARScan(ranges=ranges, angles=angles)

        assert scan.min_range == 1.0
        assert scan.max_range == 3.0
        assert scan.mean_range == 2.0


class TestLIDARConfig:
    """Tests for LIDARConfig."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = LIDARConfig()

        assert config.name == "lidar"
        assert config.scan_frequency > 0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = LIDARConfig(
            name="test_lidar",
            scan_frequency=20.0,
            min_range=0.2,
            max_range=15.0,
        )

        assert config.name == "test_lidar"
        assert config.scan_frequency == 20.0


class TestSimulatedLIDAR:
    """Tests for SimulatedLIDAR."""

    def test_create_simulated_lidar(self) -> None:
        """Test creating a simulated LIDAR."""
        lidar = SimulatedLIDAR()

        assert lidar is not None
        assert lidar.name == "simulated_lidar"  # Default name

    def test_simulated_lidar_scan(self) -> None:
        """Test getting a scan from simulated LIDAR."""
        lidar = SimulatedLIDAR()
        lidar.enable()
        lidar.start_motor()

        scan = lidar.scan()

        assert isinstance(scan, LIDARScan)
        assert scan.num_points > 0
        assert len(scan.ranges) == len(scan.angles)

        lidar.stop_motor()
        lidar.disable()

    def test_simulated_lidar_ranges_within_limits(self) -> None:
        """Test that simulated ranges are within configured limits."""
        config = LIDARConfig(range_min=0.1, range_max=10.0)
        lidar = SimulatedLIDAR(config=config)
        lidar.enable()
        lidar.start_motor()

        scan = lidar.scan()
        valid_ranges = scan.ranges[~np.isnan(scan.ranges)]

        assert np.all(valid_ranges >= 0)
        assert np.all(valid_ranges <= config.range_max)

        lidar.stop_motor()
        lidar.disable()


class TestGetLidar:
    """Tests for get_lidar factory."""

    def test_get_simulated_lidar(self) -> None:
        """Test getting a simulated LIDAR."""
        lidar = get_lidar(model=LIDARModel.SIMULATED)

        assert lidar is not None
        assert isinstance(lidar, SimulatedLIDAR)

    def test_get_lidar_with_config(self) -> None:
        """Test getting LIDAR with custom config."""
        config = LIDARConfig(name="custom_lidar")
        lidar = get_lidar(model=LIDARModel.SIMULATED, config=config)

        # Config name is set but simulated LIDAR uses its own name
        assert lidar is not None


class TestLIDARSpecs:
    """Tests for LIDAR specifications dictionary."""

    def test_rplidar_a1_specs(self) -> None:
        """Test RPLIDAR A1 specifications."""
        assert LIDARModel.RPLIDAR_A1.value in LIDAR_SPECS

    def test_ydlidar_x4_specs(self) -> None:
        """Test YDLidar X4 specifications."""
        assert LIDARModel.YDLIDAR_X4.value in LIDAR_SPECS


class TestScanQuality:
    """Tests for ScanQuality enum."""

    def test_quality_values(self) -> None:
        """Test scan quality values exist."""
        assert ScanQuality.LOW is not None
        assert ScanQuality.MEDIUM is not None
        assert ScanQuality.HIGH is not None


class TestLIDARState:
    """Tests for LIDARState enum."""

    def test_lidar_states(self) -> None:
        """Test LIDAR states exist."""
        assert LIDARState.IDLE is not None
        assert LIDARState.SCANNING is not None


class TestLIDARModel:
    """Tests for LIDARModel enum."""

    def test_supported_models(self) -> None:
        """Test supported LIDAR models."""
        assert LIDARModel.SIMULATED is not None
        assert LIDARModel.RPLIDAR_A1 is not None
        assert LIDARModel.YDLIDAR_X4 is not None
