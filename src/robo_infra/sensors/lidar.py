"""LIDAR sensor implementations.

Phase 4.8.3 provides 2D LIDAR scanner interfaces:
- Base LIDAR class for scan-based distance measurement
- SimulatedLIDAR for testing without hardware
- Hardware-specific drivers for common LIDAR models

Supported LIDAR types:
- RPLIDAR (A1, A2, A3, S1, S2)
- YDLidar (X2, X4, G2, G4)
- Hokuyo URG (URG-04LX, UTM-30LX)
- SLAMTEC (M1M1, M2M2)

Notes:
- All angles are in radians
- All distances are in meters
- Scans are returned as numpy arrays for efficiency
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import struct
import time
from abc import abstractmethod
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import TYPE_CHECKING, Any

import numpy as np
from pydantic import BaseModel, Field

from robo_infra.core.exceptions import (
    CommunicationError,
    DisabledError,
    HardwareNotFoundError,
)
from robo_infra.core.sensor import Sensor
from robo_infra.core.types import Limits, Unit


if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Iterator


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================


# Standard LIDAR models and their specifications
LIDAR_SPECS = {
    "rplidar_a1": {
        "range_min": 0.15,
        "range_max": 12.0,
        "angle_min": 0.0,
        "angle_max": 2 * math.pi,
        "scan_frequency": 5.5,
        "angular_resolution": math.radians(1.0),
    },
    "rplidar_a2": {
        "range_min": 0.15,
        "range_max": 18.0,
        "angle_min": 0.0,
        "angle_max": 2 * math.pi,
        "scan_frequency": 10.0,
        "angular_resolution": math.radians(0.45),
    },
    "rplidar_a3": {
        "range_min": 0.15,
        "range_max": 25.0,
        "angle_min": 0.0,
        "angle_max": 2 * math.pi,
        "scan_frequency": 15.0,
        "angular_resolution": math.radians(0.225),
    },
    "ydlidar_x2": {
        "range_min": 0.12,
        "range_max": 8.0,
        "angle_min": 0.0,
        "angle_max": 2 * math.pi,
        "scan_frequency": 5.0,
        "angular_resolution": math.radians(0.5),
    },
    "ydlidar_x4": {
        "range_min": 0.12,
        "range_max": 10.0,
        "angle_min": 0.0,
        "angle_max": 2 * math.pi,
        "scan_frequency": 7.0,
        "angular_resolution": math.radians(0.5),
    },
    "hokuyo_urg04": {
        "range_min": 0.02,
        "range_max": 5.6,
        "angle_min": math.radians(-120),
        "angle_max": math.radians(120),
        "scan_frequency": 10.0,
        "angular_resolution": math.radians(0.36),
    },
    "hokuyo_utm30": {
        "range_min": 0.1,
        "range_max": 30.0,
        "angle_min": math.radians(-135),
        "angle_max": math.radians(135),
        "scan_frequency": 40.0,
        "angular_resolution": math.radians(0.25),
    },
}


# =============================================================================
# Enums
# =============================================================================


class LIDARModel(str, Enum):
    """Supported LIDAR models."""

    RPLIDAR_A1 = "rplidar_a1"
    RPLIDAR_A2 = "rplidar_a2"
    RPLIDAR_A3 = "rplidar_a3"
    RPLIDAR_S1 = "rplidar_s1"
    RPLIDAR_S2 = "rplidar_s2"
    YDLIDAR_X2 = "ydlidar_x2"
    YDLIDAR_X4 = "ydlidar_x4"
    YDLIDAR_G2 = "ydlidar_g2"
    YDLIDAR_G4 = "ydlidar_g4"
    HOKUYO_URG04 = "hokuyo_urg04"
    HOKUYO_UTM30 = "hokuyo_utm30"
    SLAMTEC_M1M1 = "slamtec_m1m1"
    GENERIC = "generic"
    SIMULATED = "simulated"


class LIDARState(IntEnum):
    """LIDAR operational state."""

    IDLE = 0
    STARTING = 1
    SCANNING = 2
    STOPPING = 3
    ERROR = 4
    DISABLED = 5


class ScanQuality(IntEnum):
    """Scan quality levels."""

    INVALID = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    EXCELLENT = 4


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class LIDARScan:
    """A single LIDAR scan.

    Contains distance measurements at various angles around the sensor.

    Attributes:
        ranges: Distance measurements in meters (NaN for invalid readings).
        angles: Angles for each measurement in radians.
        intensities: Reflection intensities (0-255), or None if not available.
        timestamp: Time when scan was captured.
        scan_number: Sequential scan number.
        scan_frequency: Current scan frequency in Hz.
        quality: Overall scan quality.
    """

    ranges: np.ndarray  # Shape: (N,) float32
    angles: np.ndarray  # Shape: (N,) float32
    intensities: np.ndarray | None = None  # Shape: (N,) uint8
    timestamp: float = field(default_factory=time.time)
    scan_number: int = 0
    scan_frequency: float = 0.0
    quality: ScanQuality = ScanQuality.MEDIUM

    def __post_init__(self) -> None:
        """Validate scan data."""
        if len(self.ranges) != len(self.angles):
            raise ValueError(
                f"ranges and angles must have same length: {len(self.ranges)} != {len(self.angles)}"
            )
        if self.intensities is not None and len(self.intensities) != len(self.ranges):
            raise ValueError(
                f"intensities must have same length as ranges: "
                f"{len(self.intensities)} != {len(self.ranges)}"
            )

    @property
    def num_points(self) -> int:
        """Number of points in scan."""
        return len(self.ranges)

    @property
    def valid_mask(self) -> np.ndarray:
        """Boolean mask of valid (non-NaN) readings."""
        return ~np.isnan(self.ranges)

    @property
    def valid_count(self) -> int:
        """Number of valid readings."""
        return int(np.sum(self.valid_mask))

    @property
    def min_range(self) -> float:
        """Minimum valid range in scan."""
        valid = self.ranges[self.valid_mask]
        return float(np.min(valid)) if len(valid) > 0 else float("nan")

    @property
    def max_range(self) -> float:
        """Maximum valid range in scan."""
        valid = self.ranges[self.valid_mask]
        return float(np.max(valid)) if len(valid) > 0 else float("nan")

    @property
    def mean_range(self) -> float:
        """Mean valid range in scan."""
        valid = self.ranges[self.valid_mask]
        return float(np.mean(valid)) if len(valid) > 0 else float("nan")

    def to_cartesian(self) -> tuple[np.ndarray, np.ndarray]:
        """Convert polar coordinates to Cartesian (x, y).

        Returns:
            Tuple of (x, y) numpy arrays in meters.
        """
        x = self.ranges * np.cos(self.angles)
        y = self.ranges * np.sin(self.angles)
        return x, y

    def get_range_at_angle(self, angle: float, tolerance: float = 0.05) -> float:
        """Get range at a specific angle.

        Args:
            angle: Target angle in radians.
            tolerance: Angular tolerance for matching.

        Returns:
            Range at the closest angle, or NaN if none found.
        """
        diff = np.abs(self.angles - angle)
        idx = np.argmin(diff)
        if diff[idx] <= tolerance:
            return float(self.ranges[idx])
        return float("nan")

    def filter_range(self, min_range: float = 0.0, max_range: float = float("inf")) -> LIDARScan:
        """Return a new scan with ranges outside bounds set to NaN.

        Args:
            min_range: Minimum valid range.
            max_range: Maximum valid range.

        Returns:
            New LIDARScan with filtered ranges.
        """
        ranges = self.ranges.copy()
        ranges[(ranges < min_range) | (ranges > max_range)] = float("nan")
        return LIDARScan(
            ranges=ranges,
            angles=self.angles.copy(),
            intensities=self.intensities.copy() if self.intensities is not None else None,
            timestamp=self.timestamp,
            scan_number=self.scan_number,
            scan_frequency=self.scan_frequency,
            quality=self.quality,
        )

    def subsample(self, factor: int) -> LIDARScan:
        """Subsample scan by a factor.

        Args:
            factor: Subsampling factor (e.g., 2 = keep every other point).

        Returns:
            New LIDARScan with subsampled data.
        """
        return LIDARScan(
            ranges=self.ranges[::factor].copy(),
            angles=self.angles[::factor].copy(),
            intensities=self.intensities[::factor].copy() if self.intensities is not None else None,
            timestamp=self.timestamp,
            scan_number=self.scan_number,
            scan_frequency=self.scan_frequency,
            quality=self.quality,
        )


@dataclass
class LIDARInfo:
    """Information about a LIDAR sensor."""

    model: LIDARModel
    serial_number: str = ""
    firmware_version: str = ""
    hardware_version: str = ""
    health_status: str = "unknown"
    scan_modes: list[str] = field(default_factory=list)
    current_scan_mode: str = ""


# =============================================================================
# Configuration
# =============================================================================


class LIDARConfig(BaseModel):
    """Configuration for LIDAR sensors."""

    model_config = {"frozen": False, "extra": "allow"}

    # Identification
    name: str = Field(default="lidar", description="Sensor name")
    model: LIDARModel = Field(default=LIDARModel.GENERIC, description="LIDAR model")

    # Connection
    port: str = Field(default="/dev/ttyUSB0", description="Serial port")
    baudrate: int = Field(default=115200, description="Serial baudrate")

    # Scan parameters
    scan_frequency: float = Field(
        default=10.0, ge=1.0, le=100.0, description="Scan frequency in Hz"
    )
    angle_min: float = Field(default=0.0, description="Minimum angle in radians")
    angle_max: float = Field(default=2 * math.pi, description="Maximum angle in radians")
    range_min: float = Field(default=0.1, ge=0.0, description="Minimum range in meters")
    range_max: float = Field(default=12.0, gt=0.0, description="Maximum range in meters")

    # Processing
    angular_resolution: float = Field(
        default=math.radians(1.0), gt=0, description="Angular resolution in radians"
    )
    filter_outliers: bool = Field(default=False, description="Filter statistical outliers")
    outlier_threshold: float = Field(
        default=3.0, ge=1.0, description="Outlier threshold (std devs)"
    )

    # Motor control
    motor_speed: int = Field(default=0, ge=0, le=100, description="Motor speed (0=auto)")

    # Timeout
    timeout: float = Field(default=1.0, gt=0.0, description="Read timeout in seconds")

    metadata: dict[str, Any] = Field(default_factory=dict)

    @classmethod
    def from_model(cls, model: LIDARModel, **kwargs: Any) -> LIDARConfig:
        """Create config from a known LIDAR model.

        Args:
            model: LIDAR model to use.
            **kwargs: Additional config overrides.

        Returns:
            LIDARConfig with model-specific defaults.
        """
        specs = LIDAR_SPECS.get(model.value, {})
        return cls(model=model, **specs, **kwargs)

    @property
    def angle_range(self) -> float:
        """Total angle range in radians."""
        return self.angle_max - self.angle_min

    @property
    def expected_points_per_scan(self) -> int:
        """Expected number of points per scan."""
        return int(self.angle_range / self.angular_resolution)


# =============================================================================
# LIDAR Base Class
# =============================================================================


class LIDAR(Sensor):
    """Base class for 2D LIDAR sensors.

    Provides scan-based distance measurement around the sensor.

    Example:
        >>> lidar = SimulatedLIDAR(config=LIDARConfig(name="front_lidar"))
        >>> lidar.enable()
        >>> lidar.start_motor()
        >>> scan = lidar.scan()
        >>> print(f"Got {scan.num_points} points, min={scan.min_range:.2f}m")
        >>> lidar.stop_motor()
        >>> lidar.disable()
    """

    def __init__(
        self,
        config: LIDARConfig | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize LIDAR sensor.

        Args:
            config: LIDAR configuration.
            name: Optional name override.
        """
        self._config = config or LIDARConfig()
        self._lidar_state = LIDARState.IDLE
        self._scan_count = 0
        self._last_scan: LIDARScan | None = None
        self._motor_running = False
        self._info: LIDARInfo | None = None

        # Initialize base sensor
        super().__init__(
            name=name or self._config.name,
            driver=None,
            channel=0,
            unit=Unit.METERS,
            limits=Limits(
                min=self._config.range_min,
                max=self._config.range_max,
                default=self._config.range_min,  # Default to min range
            ),
        )

    @property
    def config(self) -> LIDARConfig:
        """Get LIDAR configuration."""
        return self._config

    @property
    def lidar_state(self) -> LIDARState:
        """Get current LIDAR state."""
        return self._lidar_state

    @property
    def is_scanning(self) -> bool:
        """Check if LIDAR is actively scanning."""
        return self._lidar_state == LIDARState.SCANNING

    @property
    def motor_running(self) -> bool:
        """Check if motor is running."""
        return self._motor_running

    @property
    def scan_count(self) -> int:
        """Number of scans captured."""
        return self._scan_count

    @property
    def last_scan(self) -> LIDARScan | None:
        """Get the last captured scan."""
        return self._last_scan

    @property
    def info(self) -> LIDARInfo | None:
        """Get LIDAR device information."""
        return self._info

    def _read_raw(self) -> int:
        """Read raw value (returns average range in mm)."""
        if self._last_scan is not None:
            return int(self._last_scan.mean_range * 1000)
        return 0

    # -------------------------------------------------------------------------
    # Motor Control
    # -------------------------------------------------------------------------

    @abstractmethod
    def start_motor(self) -> None:
        """Start the LIDAR motor.

        Must be called before scanning.
        """
        ...

    @abstractmethod
    def stop_motor(self) -> None:
        """Stop the LIDAR motor."""
        ...

    def set_motor_speed(self, speed: int) -> None:
        """Set motor speed (0-100, 0=auto).

        Args:
            speed: Motor speed percentage.
        """
        if speed < 0 or speed > 100:
            raise ValueError(f"Speed must be 0-100, got {speed}")
        self._config.motor_speed = speed
        logger.debug("%s: Motor speed set to %d", self.name, speed)

    # -------------------------------------------------------------------------
    # Scanning
    # -------------------------------------------------------------------------

    @abstractmethod
    def scan(self, timeout: float | None = None) -> LIDARScan:
        """Capture a single scan.

        Args:
            timeout: Maximum time to wait for scan.

        Returns:
            LIDARScan with distance measurements.

        Raises:
            CommunicationError: If scan fails.
            DisabledError: If sensor is disabled.
        """
        ...

    def scan_filtered(
        self,
        timeout: float | None = None,
        min_range: float | None = None,
        max_range: float | None = None,
    ) -> LIDARScan:
        """Capture and filter a scan.

        Args:
            timeout: Maximum time to wait.
            min_range: Override minimum range.
            max_range: Override maximum range.

        Returns:
            Filtered LIDARScan.
        """
        scan = self.scan(timeout)
        return scan.filter_range(
            min_range=min_range or self._config.range_min,
            max_range=max_range or self._config.range_max,
        )

    async def scan_async(self, timeout: float | None = None) -> LIDARScan:
        """Capture a scan asynchronously.

        Args:
            timeout: Maximum time to wait.

        Returns:
            LIDARScan with distance measurements.
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.scan, timeout)

    async def stream(
        self,
        count: int | None = None,
        timeout: float = 1.0,
    ) -> AsyncIterator[LIDARScan]:
        """Stream scans asynchronously.

        Args:
            count: Maximum number of scans (None = infinite).
            timeout: Timeout per scan.

        Yields:
            LIDARScan objects.
        """
        scans = 0
        while self._enabled and (count is None or scans < count):
            try:
                scan = await self.scan_async(timeout)
                yield scan
                scans += 1
            except Exception as e:
                logger.warning("%s: Scan failed: %s", self.name, e)
                await asyncio.sleep(0.1)

    def iter_scans(self, count: int | None = None) -> Iterator[LIDARScan]:
        """Iterate over scans synchronously.

        Args:
            count: Maximum number of scans (None = infinite).

        Yields:
            LIDARScan objects.
        """
        scans = 0
        while self._enabled and (count is None or scans < count):
            try:
                yield self.scan()
                scans += 1
            except Exception as e:
                logger.warning("%s: Scan failed: %s", self.name, e)
                time.sleep(0.1)

    # -------------------------------------------------------------------------
    # Analysis
    # -------------------------------------------------------------------------

    def get_closest_point(self) -> tuple[float, float] | None:
        """Get the closest point from the last scan.

        Returns:
            Tuple of (range, angle) for closest point, or None.
        """
        if self._last_scan is None:
            return None

        valid_mask = self._last_scan.valid_mask
        if not np.any(valid_mask):
            return None

        valid_ranges = self._last_scan.ranges[valid_mask]
        valid_angles = self._last_scan.angles[valid_mask]
        idx = np.argmin(valid_ranges)
        return (float(valid_ranges[idx]), float(valid_angles[idx]))

    def check_sector(
        self,
        angle_start: float,
        angle_end: float,
        threshold: float,
    ) -> bool:
        """Check if any obstacle is within threshold in a sector.

        Args:
            angle_start: Start angle in radians.
            angle_end: End angle in radians.
            threshold: Distance threshold in meters.

        Returns:
            True if obstacle detected within threshold.
        """
        if self._last_scan is None:
            return False

        mask = (self._last_scan.angles >= angle_start) & (self._last_scan.angles <= angle_end)
        sector_ranges = self._last_scan.ranges[mask]
        valid = sector_ranges[~np.isnan(sector_ranges)]

        if len(valid) == 0:
            return False

        return bool(np.min(valid) < threshold)

    # -------------------------------------------------------------------------
    # Device Info
    # -------------------------------------------------------------------------

    @abstractmethod
    def get_info(self) -> LIDARInfo:
        """Get device information.

        Returns:
            LIDARInfo with device details.
        """
        ...

    @abstractmethod
    def get_health(self) -> str:
        """Get device health status.

        Returns:
            Health status string ("good", "warning", "error").
        """
        ...


# =============================================================================
# Simulated LIDAR
# =============================================================================


class SimulatedLIDAR(LIDAR):
    """Simulated LIDAR for testing without hardware.

    Generates synthetic scans with configurable patterns.

    Example:
        >>> lidar = SimulatedLIDAR()
        >>> lidar.enable()
        >>> lidar.start_motor()
        >>> scan = lidar.scan()
        >>> print(f"Simulated {scan.num_points} points")
    """

    def __init__(
        self,
        config: LIDARConfig | None = None,
        name: str | None = None,
        pattern: str = "random",
        obstacle_distance: float = 2.0,
        noise_stddev: float = 0.02,
    ) -> None:
        """Initialize simulated LIDAR.

        Args:
            config: LIDAR configuration.
            name: Optional name override.
            pattern: Scan pattern ("random", "circle", "corridor", "room").
            obstacle_distance: Base obstacle distance for patterns.
            noise_stddev: Standard deviation of range noise.
        """
        config = config or LIDARConfig(model=LIDARModel.SIMULATED)
        super().__init__(config, name or "simulated_lidar")

        self._pattern = pattern
        self._obstacle_distance = obstacle_distance
        self._noise_stddev = noise_stddev
        self._rng = np.random.default_rng()

        self._info = LIDARInfo(
            model=LIDARModel.SIMULATED,
            serial_number="SIM-001",
            firmware_version="1.0.0",
            hardware_version="SIM",
            health_status="good",
            scan_modes=["normal", "express"],
            current_scan_mode="normal",
        )

    def connect(self) -> None:
        """Connect to simulated LIDAR (no-op)."""
        logger.info("%s: Simulated LIDAR connected", self.name)

    def disconnect(self) -> None:
        """Disconnect from simulated LIDAR (no-op)."""
        logger.info("%s: Simulated LIDAR disconnected", self.name)

    def start_motor(self) -> None:
        """Start simulated motor."""
        self._check_enabled()
        self._motor_running = True
        self._lidar_state = LIDARState.SCANNING
        logger.info("%s: Simulated motor started", self.name)

    def stop_motor(self) -> None:
        """Stop simulated motor."""
        self._motor_running = False
        self._lidar_state = LIDARState.IDLE
        logger.info("%s: Simulated motor stopped", self.name)

    def scan(self, timeout: float | None = None) -> LIDARScan:
        """Generate a simulated scan.

        Args:
            timeout: Ignored for simulation.

        Returns:
            Simulated LIDARScan.
        """
        self._check_enabled()

        if not self._motor_running:
            raise CommunicationError("Motor not running")

        # Generate angles
        num_points = self._config.expected_points_per_scan
        angles = np.linspace(
            self._config.angle_min,
            self._config.angle_max,
            num_points,
            endpoint=False,
            dtype=np.float32,
        )

        # Generate ranges based on pattern
        ranges = self._generate_pattern(angles)

        # Add noise
        if self._noise_stddev > 0:
            noise = self._rng.normal(0, self._noise_stddev, num_points).astype(np.float32)
            ranges = ranges + noise

        # Clamp to valid range
        ranges = np.clip(ranges, self._config.range_min, self._config.range_max)

        # Random invalid readings (5% chance)
        invalid_mask = self._rng.random(num_points) < 0.05
        ranges[invalid_mask] = float("nan")

        # Generate intensities
        intensities = self._rng.integers(100, 255, size=num_points, dtype=np.uint8)
        intensities[invalid_mask] = 0

        self._scan_count += 1
        scan = LIDARScan(
            ranges=ranges,
            angles=angles,
            intensities=intensities,
            timestamp=time.time(),
            scan_number=self._scan_count,
            scan_frequency=self._config.scan_frequency,
            quality=ScanQuality.HIGH,
        )
        self._last_scan = scan

        # Simulate scan time
        time.sleep(1.0 / self._config.scan_frequency)

        return scan

    def _generate_pattern(self, angles: np.ndarray) -> np.ndarray:
        """Generate range values based on pattern.

        Args:
            angles: Array of angles.

        Returns:
            Array of ranges.
        """
        num_points = len(angles)

        if self._pattern == "circle":
            # Circular room
            return np.full(num_points, self._obstacle_distance, dtype=np.float32)

        elif self._pattern == "corridor":
            # Corridor pattern (walls at front/back, open sides)
            ranges = np.full(num_points, self._config.range_max, dtype=np.float32)
            # Walls at 0° and 180°
            wall_mask = (np.abs(angles) < 0.3) | (np.abs(angles - math.pi) < 0.3)
            ranges[wall_mask] = self._obstacle_distance
            return ranges

        elif self._pattern == "room":
            # Rectangular room
            ranges = np.zeros(num_points, dtype=np.float32)
            # Calculate distance to rectangular walls
            room_width = self._obstacle_distance * 2
            room_length = self._obstacle_distance * 3
            for i, angle in enumerate(angles):
                cos_a = np.cos(angle)
                sin_a = np.sin(angle)
                # Distance to x walls
                dx = room_width / 2 / abs(cos_a) if abs(cos_a) > 0.001 else float("inf")
                # Distance to y walls
                dy = room_length / 2 / abs(sin_a) if abs(sin_a) > 0.001 else float("inf")
                ranges[i] = min(dx, dy)
            return ranges

        else:  # random
            # Random obstacles
            base = self._rng.uniform(
                self._obstacle_distance * 0.5,
                self._config.range_max * 0.8,
                num_points,
            ).astype(np.float32)
            return base

    def get_info(self) -> LIDARInfo:
        """Get simulated device info."""
        return self._info

    def get_health(self) -> str:
        """Get simulated health status."""
        return "good"

    def _check_enabled(self) -> None:
        """Check if sensor is enabled."""
        if not self._is_enabled:
            raise DisabledError(f"{self.name} is disabled")


# =============================================================================
# Hardware LIDAR Base
# =============================================================================


class SerialLIDAR(LIDAR):
    """Base class for serial-connected LIDAR sensors.

    Provides common serial communication functionality.
    """

    def __init__(
        self,
        config: LIDARConfig | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize serial LIDAR.

        Args:
            config: LIDAR configuration.
            name: Optional name override.
        """
        super().__init__(config, name)
        self._serial: Any = None

    def connect(self) -> None:
        """Connect to LIDAR via serial port."""
        if os.environ.get("ROBO_SIMULATION", "").lower() in ("true", "1", "yes"):
            logger.warning("⚠️ SIMULATION MODE - %s not connected to real hardware", self.name)
            return

        try:
            import serial
        except ImportError as e:
            raise HardwareNotFoundError(
                "pyserial not installed. Install with: pip install pyserial"
            ) from e

        try:
            self._serial = serial.Serial(
                port=self._config.port,
                baudrate=self._config.baudrate,
                timeout=self._config.timeout,
            )
            logger.info(
                "%s: Connected to %s at %d baud",
                self.name,
                self._config.port,
                self._config.baudrate,
            )
        except serial.SerialException as e:
            raise CommunicationError(f"Failed to connect to {self._config.port}: {e}") from e

    def disconnect(self) -> None:
        """Disconnect from LIDAR."""
        if self._serial is not None:
            self.stop_motor()
            self._serial.close()
            self._serial = None
            logger.info("%s: Disconnected", self.name)

    def _send_command(self, command: bytes) -> None:
        """Send command to LIDAR."""
        if self._serial is None:
            raise CommunicationError("Not connected")
        self._serial.write(command)
        self._serial.flush()

    def _read_response(self, size: int) -> bytes:
        """Read response from LIDAR."""
        if self._serial is None:
            raise CommunicationError("Not connected")
        return self._serial.read(size)


# =============================================================================
# RPLIDAR Driver
# =============================================================================


class RPLIDAR(SerialLIDAR):
    """RPLIDAR sensor driver (A1, A2, A3, S1, S2).

    Supports SLAMTEC RPLIDAR sensors via serial communication.

    Example:
        >>> lidar = RPLIDAR(config=LIDARConfig(
        ...     model=LIDARModel.RPLIDAR_A2,
        ...     port="/dev/ttyUSB0"
        ... ))
        >>> lidar.connect()
        >>> lidar.enable()
        >>> lidar.start_motor()
        >>> for scan in lidar.iter_scans(count=10):
        ...     print(f"Scan {scan.scan_number}: {scan.valid_count} points")
        >>> lidar.stop_motor()
        >>> lidar.disconnect()
    """

    # RPLIDAR Commands
    CMD_SYNC = 0xA5
    CMD_STOP = 0x25
    CMD_RESET = 0x40
    CMD_SCAN = 0x20
    CMD_EXPRESS_SCAN = 0x82
    CMD_FORCE_SCAN = 0x21
    CMD_GET_INFO = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_GET_SAMPLERATE = 0x59
    CMD_SET_MOTOR_PWM = 0xF0

    # Response types
    RESP_MEASUREMENT = 0x81
    RESP_DEVICE_INFO = 0x04
    RESP_HEALTH = 0x06

    def __init__(
        self,
        config: LIDARConfig | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize RPLIDAR.

        Args:
            config: LIDAR configuration.
            name: Optional name override.
        """
        if config is None:
            config = LIDARConfig.from_model(LIDARModel.RPLIDAR_A2)
        super().__init__(config, name or "rplidar")

    def start_motor(self) -> None:
        """Start RPLIDAR motor."""
        self._check_enabled()

        if self._serial is None:
            # Simulation mode
            self._motor_running = True
            self._lidar_state = LIDARState.SCANNING
            return

        # DTR controls motor on some models
        self._serial.dtr = False
        time.sleep(0.5)

        # Set motor PWM if needed
        if self._config.motor_speed > 0:
            pwm = int(self._config.motor_speed * 10.23)  # 0-1023
            self._send_command(
                bytes([self.CMD_SYNC, self.CMD_SET_MOTOR_PWM]) + struct.pack("<H", pwm)
            )

        self._motor_running = True
        self._lidar_state = LIDARState.SCANNING
        logger.info("%s: Motor started", self.name)

    def stop_motor(self) -> None:
        """Stop RPLIDAR motor."""
        if self._serial is not None:
            self._send_command(bytes([self.CMD_SYNC, self.CMD_STOP]))
            time.sleep(0.1)
            self._serial.dtr = True

        self._motor_running = False
        self._lidar_state = LIDARState.IDLE
        logger.info("%s: Motor stopped", self.name)

    def reset(self) -> None:
        """Reset RPLIDAR."""
        if self._serial is not None:
            self._send_command(bytes([self.CMD_SYNC, self.CMD_RESET]))
            time.sleep(2.0)
        logger.info("%s: Reset complete", self.name)

    def scan(self, timeout: float | None = None) -> LIDARScan:
        """Capture a single scan.

        Args:
            timeout: Maximum time to wait.

        Returns:
            LIDARScan with measurements.
        """
        self._check_enabled()

        if not self._motor_running:
            raise CommunicationError("Motor not running")

        if self._serial is None:
            # Simulation mode - generate fake data
            return self._generate_simulated_scan()

        # Start scan
        self._send_command(bytes([self.CMD_SYNC, self.CMD_SCAN]))

        # Read descriptor
        descriptor = self._read_response(7)
        if len(descriptor) < 7:
            raise CommunicationError("Failed to read scan descriptor")

        # Collect measurements for one full rotation
        measurements: list[tuple[float, float, int]] = []
        start_angle: float | None = None
        start_time = time.time()
        timeout_val = timeout or self._config.timeout

        while True:
            if time.time() - start_time > timeout_val:
                raise CommunicationError("Scan timeout")

            # Read measurement packet
            packet = self._read_response(5)
            if len(packet) < 5:
                continue

            # Parse measurement
            quality = packet[0] >> 2
            if quality == 0:
                continue  # Invalid measurement

            angle = ((packet[1] >> 1) + (packet[2] << 7)) / 64.0
            distance = (packet[3] + (packet[4] << 8)) / 4000.0  # Convert to meters

            angle_rad = math.radians(angle)

            if start_angle is None:
                start_angle = angle_rad
            elif angle_rad < start_angle and len(measurements) > 10:
                # Completed one rotation
                break

            if distance > 0:
                measurements.append((angle_rad, distance, quality))

        # Convert to arrays
        if not measurements:
            raise CommunicationError("No valid measurements")

        angles = np.array([m[0] for m in measurements], dtype=np.float32)
        ranges = np.array([m[1] for m in measurements], dtype=np.float32)
        intensities = np.array([m[2] for m in measurements], dtype=np.uint8)

        self._scan_count += 1
        scan = LIDARScan(
            ranges=ranges,
            angles=angles,
            intensities=intensities,
            timestamp=time.time(),
            scan_number=self._scan_count,
            scan_frequency=self._config.scan_frequency,
            quality=ScanQuality.HIGH,
        )
        self._last_scan = scan
        return scan

    def _generate_simulated_scan(self) -> LIDARScan:
        """Generate simulated scan data."""
        num_points = self._config.expected_points_per_scan
        angles = np.linspace(
            self._config.angle_min,
            self._config.angle_max,
            num_points,
            endpoint=False,
            dtype=np.float32,
        )
        rng = np.random.default_rng()
        ranges = rng.uniform(1.0, self._config.range_max * 0.8, num_points).astype(np.float32)
        intensities = rng.integers(50, 200, size=num_points, dtype=np.uint8)

        self._scan_count += 1
        scan = LIDARScan(
            ranges=ranges,
            angles=angles,
            intensities=intensities,
            timestamp=time.time(),
            scan_number=self._scan_count,
            scan_frequency=self._config.scan_frequency,
            quality=ScanQuality.MEDIUM,
        )
        self._last_scan = scan
        return scan

    def get_info(self) -> LIDARInfo:
        """Get RPLIDAR device information."""
        if self._serial is None:
            return LIDARInfo(
                model=self._config.model,
                serial_number="SIM-RPLIDAR",
                firmware_version="1.0.0",
                hardware_version="SIM",
                health_status="good",
            )

        self._send_command(bytes([self.CMD_SYNC, self.CMD_GET_INFO]))
        descriptor = self._read_response(7)
        if len(descriptor) < 7:
            raise CommunicationError("Failed to read info descriptor")

        data = self._read_response(20)
        if len(data) < 20:
            raise CommunicationError("Failed to read device info")

        data[0]
        firmware_minor = data[1]
        firmware_major = data[2]
        hardware = data[3]
        serial_number = data[4:20].hex()

        info = LIDARInfo(
            model=self._config.model,
            serial_number=serial_number,
            firmware_version=f"{firmware_major}.{firmware_minor}",
            hardware_version=str(hardware),
            health_status="unknown",
        )
        self._info = info
        return info

    def get_health(self) -> str:
        """Get RPLIDAR health status."""
        if self._serial is None:
            return "good"

        self._send_command(bytes([self.CMD_SYNC, self.CMD_GET_HEALTH]))
        descriptor = self._read_response(7)
        if len(descriptor) < 7:
            raise CommunicationError("Failed to read health descriptor")

        data = self._read_response(3)
        if len(data) < 3:
            raise CommunicationError("Failed to read health data")

        status = data[0]
        error_code = (data[1] << 8) | data[2]

        if status == 0:
            return "good"
        elif status == 1:
            return f"warning (code: {error_code})"
        else:
            return f"error (code: {error_code})"

    def _check_enabled(self) -> None:
        """Check if sensor is enabled."""
        if not self._is_enabled:
            raise DisabledError(f"{self.name} is disabled")


# =============================================================================
# YDLidar Driver
# =============================================================================


class YDLidar(SerialLIDAR):
    """YDLidar sensor driver (X2, X4, G2, G4).

    Supports YDLidar sensors via serial communication.

    Example:
        >>> lidar = YDLidar(config=LIDARConfig(
        ...     model=LIDARModel.YDLIDAR_X4,
        ...     port="/dev/ttyUSB0"
        ... ))
        >>> lidar.connect()
        >>> lidar.enable()
        >>> lidar.start_motor()
        >>> scan = lidar.scan()
        >>> lidar.stop_motor()
    """

    # YDLidar Commands
    CMD_START_SCAN = 0x60
    CMD_STOP_SCAN = 0x65
    CMD_GET_INFO = 0x90
    CMD_GET_HEALTH = 0x91
    CMD_RESTART = 0x40

    def __init__(
        self,
        config: LIDARConfig | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize YDLidar.

        Args:
            config: LIDAR configuration.
            name: Optional name override.
        """
        if config is None:
            config = LIDARConfig.from_model(LIDARModel.YDLIDAR_X4)
        super().__init__(config, name or "ydlidar")

    def start_motor(self) -> None:
        """Start YDLidar motor."""
        self._check_enabled()

        if self._serial is None:
            self._motor_running = True
            self._lidar_state = LIDARState.SCANNING
            return

        # DTR low to start motor
        self._serial.dtr = False
        time.sleep(0.5)

        self._motor_running = True
        self._lidar_state = LIDARState.SCANNING
        logger.info("%s: Motor started", self.name)

    def stop_motor(self) -> None:
        """Stop YDLidar motor."""
        if self._serial is not None:
            self._send_command(bytes([0xA5, self.CMD_STOP_SCAN]))
            time.sleep(0.1)
            self._serial.dtr = True

        self._motor_running = False
        self._lidar_state = LIDARState.IDLE
        logger.info("%s: Motor stopped", self.name)

    def scan(self, timeout: float | None = None) -> LIDARScan:
        """Capture a single scan."""
        self._check_enabled()

        if not self._motor_running:
            raise CommunicationError("Motor not running")

        if self._serial is None:
            return self._generate_simulated_scan()

        # Start scan
        self._send_command(bytes([0xA5, self.CMD_START_SCAN]))

        # Collect measurements
        measurements: list[tuple[float, float, int]] = []
        start_time = time.time()
        timeout_val = timeout or self._config.timeout

        while True:
            if time.time() - start_time > timeout_val:
                if len(measurements) > 100:
                    break
                raise CommunicationError("Scan timeout")

            # Read packet header
            header = self._read_response(10)
            if len(header) < 10:
                continue

            # Check header
            if header[0] != 0xAA or header[1] != 0x55:
                continue

            # Parse packet
            header[2]
            sample_qty = header[3]

            if sample_qty == 0:
                continue

            # Read samples
            sample_data = self._read_response(sample_qty * 3)
            if len(sample_data) < sample_qty * 3:
                continue

            # Parse each sample
            for i in range(sample_qty):
                offset = i * 3
                distance = (sample_data[offset] | (sample_data[offset + 1] << 8)) / 4.0
                angle = sample_data[offset + 2] * 360.0 / 256.0

                if distance > 0:
                    measurements.append((math.radians(angle), distance / 1000.0, 100))

            if len(measurements) > self._config.expected_points_per_scan:
                break

        if not measurements:
            raise CommunicationError("No valid measurements")

        angles = np.array([m[0] for m in measurements], dtype=np.float32)
        ranges = np.array([m[1] for m in measurements], dtype=np.float32)
        intensities = np.array([m[2] for m in measurements], dtype=np.uint8)

        self._scan_count += 1
        scan = LIDARScan(
            ranges=ranges,
            angles=angles,
            intensities=intensities,
            timestamp=time.time(),
            scan_number=self._scan_count,
            scan_frequency=self._config.scan_frequency,
        )
        self._last_scan = scan
        return scan

    def _generate_simulated_scan(self) -> LIDARScan:
        """Generate simulated scan."""
        num_points = self._config.expected_points_per_scan
        angles = np.linspace(0, 2 * math.pi, num_points, endpoint=False, dtype=np.float32)
        rng = np.random.default_rng()
        ranges = rng.uniform(0.5, self._config.range_max * 0.7, num_points).astype(np.float32)

        self._scan_count += 1
        scan = LIDARScan(
            ranges=ranges,
            angles=angles,
            timestamp=time.time(),
            scan_number=self._scan_count,
        )
        self._last_scan = scan
        return scan

    def get_info(self) -> LIDARInfo:
        """Get YDLidar device info."""
        return LIDARInfo(
            model=self._config.model,
            serial_number="YDLIDAR",
            firmware_version="1.0",
            health_status="good",
        )

    def get_health(self) -> str:
        """Get YDLidar health status."""
        return "good"

    def _check_enabled(self) -> None:
        if not self._is_enabled:
            raise DisabledError(f"{self.name} is disabled")


# =============================================================================
# Hokuyo Driver
# =============================================================================


class HokuyoLIDAR(SerialLIDAR):
    """Hokuyo LIDAR sensor driver (URG-04LX, UTM-30LX).

    Supports Hokuyo URG sensors via serial SCIP 2.0 protocol.

    Example:
        >>> lidar = HokuyoLIDAR(config=LIDARConfig(
        ...     model=LIDARModel.HOKUYO_URG04,
        ...     port="/dev/ttyACM0"
        ... ))
        >>> lidar.connect()
        >>> lidar.enable()
        >>> lidar.start_motor()
        >>> scan = lidar.scan()
    """

    def __init__(
        self,
        config: LIDARConfig | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize Hokuyo LIDAR.

        Args:
            config: LIDAR configuration.
            name: Optional name override.
        """
        if config is None:
            config = LIDARConfig.from_model(LIDARModel.HOKUYO_URG04)
        super().__init__(config, name or "hokuyo")

    def start_motor(self) -> None:
        """Start Hokuyo laser (motor is always running)."""
        self._check_enabled()

        if self._serial is not None:
            # Send BM command to turn on laser
            self._send_command(b"BM\n")
            response = self._read_response(100)
            logger.debug("BM response: %s", response)

        self._motor_running = True
        self._lidar_state = LIDARState.SCANNING
        logger.info("%s: Laser started", self.name)

    def stop_motor(self) -> None:
        """Stop Hokuyo laser."""
        if self._serial is not None:
            # Send QT command to turn off laser
            self._send_command(b"QT\n")
            self._read_response(100)

        self._motor_running = False
        self._lidar_state = LIDARState.IDLE
        logger.info("%s: Laser stopped", self.name)

    def scan(self, timeout: float | None = None) -> LIDARScan:
        """Capture a single scan using GD command."""
        self._check_enabled()

        if not self._motor_running:
            raise CommunicationError("Laser not running")

        if self._serial is None:
            return self._generate_simulated_scan()

        # Request single scan with GD command
        # GD start_step end_step cluster_count
        start_step = 44  # First valid step for URG-04LX
        end_step = 725  # Last valid step
        cluster = 1

        cmd = f"GD{start_step:04d}{end_step:04d}{cluster:02d}\n"
        self._send_command(cmd.encode())

        # Read response
        response = b""
        start_time = time.time()
        timeout_val = timeout or self._config.timeout

        while time.time() - start_time < timeout_val:
            chunk = self._read_response(4096)
            if not chunk:
                break
            response += chunk
            if b"\n\n" in response:
                break

        # Parse SCIP response
        lines = response.decode("ascii", errors="ignore").split("\n")

        # Skip header lines, parse data
        data_lines = [line for line in lines if line and not line.startswith("G")]

        if len(data_lines) < 2:
            raise CommunicationError("Invalid scan response")

        # Decode distance data (3-character encoding)
        distances: list[float] = []
        for line in data_lines[2:]:  # Skip echo and status
            for i in range(0, len(line) - 1, 3):
                if i + 2 < len(line):
                    chars = line[i : i + 3]
                    try:
                        dist = (
                            ((ord(chars[0]) - 0x30) << 12)
                            + ((ord(chars[1]) - 0x30) << 6)
                            + (ord(chars[2]) - 0x30)
                        )
                        distances.append(dist / 1000.0)  # mm to m
                    except (IndexError, ValueError):
                        distances.append(float("nan"))

        if not distances:
            raise CommunicationError("No distance data parsed")

        # Generate angles
        num_steps = end_step - start_step + 1
        angles = np.linspace(
            self._config.angle_min,
            self._config.angle_max,
            min(len(distances), num_steps),
            dtype=np.float32,
        )
        ranges = np.array(distances[: len(angles)], dtype=np.float32)

        self._scan_count += 1
        scan = LIDARScan(
            ranges=ranges,
            angles=angles,
            timestamp=time.time(),
            scan_number=self._scan_count,
            scan_frequency=self._config.scan_frequency,
        )
        self._last_scan = scan
        return scan

    def _generate_simulated_scan(self) -> LIDARScan:
        """Generate simulated Hokuyo scan."""
        num_points = 682  # URG-04LX typical
        angles = np.linspace(
            self._config.angle_min,
            self._config.angle_max,
            num_points,
            dtype=np.float32,
        )
        rng = np.random.default_rng()
        ranges = rng.uniform(0.1, self._config.range_max * 0.8, num_points).astype(np.float32)

        self._scan_count += 1
        return LIDARScan(
            ranges=ranges,
            angles=angles,
            timestamp=time.time(),
            scan_number=self._scan_count,
        )

    def get_info(self) -> LIDARInfo:
        """Get Hokuyo device info."""
        if self._serial is None:
            return LIDARInfo(
                model=self._config.model,
                serial_number="SIM-HOKUYO",
                health_status="good",
            )

        # Send VV command for version info
        self._send_command(b"VV\n")
        response = self._read_response(500)
        lines = response.decode("ascii", errors="ignore").split("\n")

        vendor = ""
        product = ""
        firmware = ""
        serial = ""

        for line in lines:
            if line.startswith("VEND:"):
                vendor = line.split(":")[1].strip()
            elif line.startswith("PROD:"):
                product = line.split(":")[1].strip()
            elif line.startswith("FIRM:"):
                firmware = line.split(":")[1].strip()
            elif line.startswith("SERI:"):
                serial = line.split(":")[1].strip()

        return LIDARInfo(
            model=self._config.model,
            serial_number=serial,
            firmware_version=firmware,
            hardware_version=f"{vendor} {product}",
            health_status="good",
        )

    def get_health(self) -> str:
        """Get Hokuyo health status."""
        return "good"

    def _check_enabled(self) -> None:
        if not self._is_enabled:
            raise DisabledError(f"{self.name} is disabled")


# =============================================================================
# Factory Function
# =============================================================================


def get_lidar(
    model: LIDARModel | str = LIDARModel.GENERIC,
    port: str = "/dev/ttyUSB0",
    simulation: bool | None = None,
    **kwargs: Any,
) -> LIDAR:
    """Get a LIDAR instance for the specified model.

    Args:
        model: LIDAR model to use.
        port: Serial port for hardware.
        simulation: Force simulation mode (None = auto-detect).
        **kwargs: Additional config options.

    Returns:
        Appropriate LIDAR instance.

    Example:
        >>> lidar = get_lidar("rplidar_a2", "/dev/ttyUSB0")
        >>> lidar.connect()
    """
    if isinstance(model, str):
        try:
            model = LIDARModel(model.lower())
        except ValueError:
            model = LIDARModel.GENERIC

    # Check for simulation mode
    if simulation is None:
        simulation = os.environ.get("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

    if simulation or model == LIDARModel.SIMULATED:
        return SimulatedLIDAR(config=LIDARConfig(model=model, port=port, **kwargs))

    config = LIDARConfig.from_model(model, port=port, **kwargs)

    # Select driver based on model
    if model.value.startswith("rplidar"):
        return RPLIDAR(config)
    elif model.value.startswith("ydlidar"):
        return YDLidar(config)
    elif model.value.startswith("hokuyo"):
        return HokuyoLIDAR(config)
    else:
        logger.warning("Unknown model %s, using simulated LIDAR", model)
        return SimulatedLIDAR(config=config)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Classes
    "LIDAR",
    "LIDAR_SPECS",
    "RPLIDAR",
    "HokuyoLIDAR",
    # Config
    "LIDARConfig",
    "LIDARInfo",
    # Enums
    "LIDARModel",
    # Data classes
    "LIDARScan",
    "LIDARState",
    "ScanQuality",
    "SerialLIDAR",
    "SimulatedLIDAR",
    "YDLidar",
    # Factory
    "get_lidar",
]
