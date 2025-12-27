"""GPS/GNSS sensor implementations.

Phase 4.8.3 provides GPS/GNSS sensor interfaces:
- Base GPS class for position, velocity, and time data
- SimulatedGPS for testing without hardware
- NMEA sentence parsing for standard GPS receivers
- Support for u-blox, Adafruit, and generic GPS modules

NMEA Sentences Supported:
- GGA: Global Positioning System Fix Data
- RMC: Recommended Minimum Navigation Information
- VTG: Course Over Ground and Ground Speed
- GSA: GPS DOP and Active Satellites
- GSV: Satellites in View

Notes:
- All coordinates are in decimal degrees (WGS84)
- Altitude is in meters above mean sea level
- Speed is in meters per second
- Heading is in degrees (0-360, true north)
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import re
import time
from abc import abstractmethod
from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import Enum, IntEnum
from typing import TYPE_CHECKING, Any

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

# Earth radius in meters (WGS84)
EARTH_RADIUS = 6371000.0

# Conversion factors
KNOTS_TO_MPS = 0.514444
KMPH_TO_MPS = 0.277778


# =============================================================================
# Enums
# =============================================================================


class FixQuality(IntEnum):
    """GPS fix quality indicators."""

    NO_FIX = 0
    GPS_FIX = 1
    DGPS_FIX = 2
    PPS_FIX = 3
    RTK_FIXED = 4
    RTK_FLOAT = 5
    DEAD_RECKONING = 6
    MANUAL_INPUT = 7
    SIMULATION = 8


class FixMode(IntEnum):
    """GPS fix mode."""

    NO_FIX = 1
    FIX_2D = 2
    FIX_3D = 3


class GPSModel(str, Enum):
    """Supported GPS module types."""

    GENERIC = "generic"
    UBLOX = "ublox"
    ADAFRUIT_ULTIMATE = "adafruit_ultimate"
    ADAFRUIT_MINI = "adafruit_mini"
    BEITIAN = "beitian"
    QUECTEL = "quectel"
    SIMULATED = "simulated"


class GPSState(IntEnum):
    """GPS operational state."""

    IDLE = 0
    SEARCHING = 1
    ACQUIRING = 2
    TRACKING = 3
    ERROR = 4
    DISABLED = 5


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class GPSReading:
    """A GPS position reading.

    Attributes:
        latitude: Latitude in decimal degrees (-90 to 90).
        longitude: Longitude in decimal degrees (-180 to 180).
        altitude: Altitude above mean sea level in meters.
        speed: Ground speed in meters per second.
        heading: Course over ground in degrees (0-360, true north).
        fix_quality: Quality of GPS fix.
        fix_mode: 2D or 3D fix mode.
        satellites: Number of satellites used in fix.
        hdop: Horizontal dilution of precision.
        vdop: Vertical dilution of precision.
        pdop: Position dilution of precision.
        timestamp: UTC timestamp of fix.
        age: Age of differential correction (seconds), or None.
    """

    latitude: float
    longitude: float
    altitude: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    fix_quality: FixQuality = FixQuality.NO_FIX
    fix_mode: FixMode = FixMode.NO_FIX
    satellites: int = 0
    hdop: float = 99.99
    vdop: float = 99.99
    pdop: float = 99.99
    timestamp: float = field(default_factory=time.time)
    age: float | None = None

    @property
    def has_fix(self) -> bool:
        """Check if GPS has a valid fix."""
        return self.fix_quality != FixQuality.NO_FIX and self.fix_mode != FixMode.NO_FIX

    @property
    def is_accurate(self) -> bool:
        """Check if fix is accurate (HDOP < 2.0)."""
        return self.has_fix and self.hdop < 2.0

    @property
    def is_3d(self) -> bool:
        """Check if this is a 3D fix."""
        return self.fix_mode == FixMode.FIX_3D

    def distance_to(self, other: GPSReading) -> float:
        """Calculate distance to another point (Haversine formula).

        Args:
            other: Another GPS reading.

        Returns:
            Distance in meters.
        """
        lat1 = math.radians(self.latitude)
        lat2 = math.radians(other.latitude)
        dlat = math.radians(other.latitude - self.latitude)
        dlon = math.radians(other.longitude - self.longitude)

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return EARTH_RADIUS * c

    def bearing_to(self, other: GPSReading) -> float:
        """Calculate bearing to another point.

        Args:
            other: Another GPS reading.

        Returns:
            Bearing in degrees (0-360).
        """
        lat1 = math.radians(self.latitude)
        lat2 = math.radians(other.latitude)
        dlon = math.radians(other.longitude - self.longitude)

        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)

        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360


@dataclass
class SatelliteInfo:
    """Information about a visible satellite."""

    prn: int  # Satellite PRN number
    elevation: float  # Elevation angle in degrees
    azimuth: float  # Azimuth angle in degrees
    snr: float  # Signal-to-noise ratio in dB
    in_use: bool = False  # Whether satellite is used in fix


@dataclass
class GPSInfo:
    """GPS device information."""

    model: GPSModel
    firmware_version: str = ""
    serial_number: str = ""
    receiver_mode: str = ""
    satellites_visible: int = 0
    satellites_in_use: int = 0
    antenna_status: str = "unknown"


# =============================================================================
# NMEA Parser
# =============================================================================


class NMEAParser:
    """Parser for NMEA 0183 sentences.

    Supports GGA, RMC, VTG, GSA, and GSV sentences.

    Example:
        >>> parser = NMEAParser()
        >>> result = parser.parse("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47")
        >>> if result:
        ...     print(f"Lat: {result.get('latitude')}")
    """

    # NMEA sentence patterns
    SENTENCE_PATTERN = re.compile(r"^\$([A-Z]{2})([A-Z]{3}),(.*)(\*[0-9A-F]{2})?$")

    def __init__(self) -> None:
        """Initialize NMEA parser."""
        self._last_gga: dict[str, Any] | None = None
        self._last_rmc: dict[str, Any] | None = None
        self._last_vtg: dict[str, Any] | None = None
        self._last_gsa: dict[str, Any] | None = None
        self._satellites: dict[int, SatelliteInfo] = {}

    def parse(self, sentence: str) -> dict[str, Any] | None:
        """Parse an NMEA sentence.

        Args:
            sentence: NMEA sentence string.

        Returns:
            Parsed data dictionary, or None if invalid.
        """
        sentence = sentence.strip()

        # Validate checksum
        if "*" in sentence:
            data_part = sentence[1 : sentence.index("*")]
            checksum_str = sentence[sentence.index("*") + 1 :]
            try:
                expected_checksum = int(checksum_str, 16)
                actual_checksum = 0
                for char in data_part:
                    actual_checksum ^= ord(char)
                if actual_checksum != expected_checksum:
                    logger.debug("NMEA checksum mismatch: %s", sentence)
                    return None
            except ValueError:
                pass

        # Match sentence pattern
        match = self.SENTENCE_PATTERN.match(sentence)
        if not match:
            return None

        match.group(1)  # GP, GL, GN, etc.
        sentence_type = match.group(2)  # GGA, RMC, etc.
        fields = match.group(3).split(",")

        # Parse based on sentence type
        if sentence_type == "GGA":
            return self._parse_gga(fields)
        elif sentence_type == "RMC":
            return self._parse_rmc(fields)
        elif sentence_type == "VTG":
            return self._parse_vtg(fields)
        elif sentence_type == "GSA":
            return self._parse_gsa(fields)
        elif sentence_type == "GSV":
            return self._parse_gsv(fields)
        else:
            return {"type": sentence_type, "raw": fields}

    def _parse_gga(self, fields: list[str]) -> dict[str, Any]:
        """Parse GGA (Global Positioning System Fix Data)."""
        try:
            result = {
                "type": "GGA",
                "time": self._parse_time(fields[0]) if fields[0] else None,
                "latitude": self._parse_latitude(fields[1], fields[2])
                if fields[1] and fields[2]
                else None,
                "longitude": self._parse_longitude(fields[3], fields[4])
                if fields[3] and fields[4]
                else None,
                "fix_quality": FixQuality(int(fields[5])) if fields[5] else FixQuality.NO_FIX,
                "satellites": int(fields[6]) if fields[6] else 0,
                "hdop": float(fields[7]) if fields[7] else 99.99,
                "altitude": float(fields[8]) if fields[8] else 0.0,
                "altitude_unit": fields[9] if len(fields) > 9 else "M",
                "geoid_separation": float(fields[10]) if len(fields) > 10 and fields[10] else 0.0,
                "dgps_age": float(fields[12]) if len(fields) > 12 and fields[12] else None,
            }
            self._last_gga = result
            return result
        except (ValueError, IndexError) as e:
            logger.debug("GGA parse error: %s", e)
            return {"type": "GGA", "error": str(e)}

    def _parse_rmc(self, fields: list[str]) -> dict[str, Any]:
        """Parse RMC (Recommended Minimum Navigation Information)."""
        try:
            result = {
                "type": "RMC",
                "time": self._parse_time(fields[0]) if fields[0] else None,
                "status": fields[1],  # A=active, V=void
                "latitude": self._parse_latitude(fields[2], fields[3])
                if fields[2] and fields[3]
                else None,
                "longitude": self._parse_longitude(fields[4], fields[5])
                if fields[4] and fields[5]
                else None,
                "speed_knots": float(fields[6]) if fields[6] else 0.0,
                "speed_mps": float(fields[6]) * KNOTS_TO_MPS if fields[6] else 0.0,
                "heading": float(fields[7]) if fields[7] else 0.0,
                "date": self._parse_date(fields[8]) if len(fields) > 8 and fields[8] else None,
                "magnetic_variation": float(fields[9]) if len(fields) > 9 and fields[9] else None,
            }
            self._last_rmc = result
            return result
        except (ValueError, IndexError) as e:
            logger.debug("RMC parse error: %s", e)
            return {"type": "RMC", "error": str(e)}

    def _parse_vtg(self, fields: list[str]) -> dict[str, Any]:
        """Parse VTG (Course Over Ground and Ground Speed)."""
        try:
            result = {
                "type": "VTG",
                "heading_true": float(fields[0]) if fields[0] else 0.0,
                "heading_magnetic": float(fields[2]) if len(fields) > 2 and fields[2] else 0.0,
                "speed_knots": float(fields[4]) if len(fields) > 4 and fields[4] else 0.0,
                "speed_kmph": float(fields[6]) if len(fields) > 6 and fields[6] else 0.0,
                "speed_mps": float(fields[6]) * KMPH_TO_MPS
                if len(fields) > 6 and fields[6]
                else 0.0,
            }
            self._last_vtg = result
            return result
        except (ValueError, IndexError) as e:
            logger.debug("VTG parse error: %s", e)
            return {"type": "VTG", "error": str(e)}

    def _parse_gsa(self, fields: list[str]) -> dict[str, Any]:
        """Parse GSA (GPS DOP and Active Satellites)."""
        try:
            # Extract satellite PRNs (fields 2-13)
            satellites = []
            for i in range(2, min(14, len(fields))):
                if fields[i]:
                    satellites.append(int(fields[i]))

            result = {
                "type": "GSA",
                "mode": fields[0],  # A=auto, M=manual
                "fix_mode": FixMode(int(fields[1])) if fields[1] else FixMode.NO_FIX,
                "satellites": satellites,
                "pdop": float(fields[14]) if len(fields) > 14 and fields[14] else 99.99,
                "hdop": float(fields[15]) if len(fields) > 15 and fields[15] else 99.99,
                "vdop": float(fields[16]) if len(fields) > 16 and fields[16] else 99.99,
            }
            self._last_gsa = result
            return result
        except (ValueError, IndexError) as e:
            logger.debug("GSA parse error: %s", e)
            return {"type": "GSA", "error": str(e)}

    def _parse_gsv(self, fields: list[str]) -> dict[str, Any]:
        """Parse GSV (Satellites in View)."""
        try:
            total_messages = int(fields[0]) if fields[0] else 1
            message_num = int(fields[1]) if fields[1] else 1
            total_satellites = int(fields[2]) if fields[2] else 0

            # Parse satellite info (4 satellites per message)
            satellites = []
            for i in range(4):
                idx = 3 + i * 4
                if idx + 3 < len(fields) and fields[idx]:
                    try:
                        sat = SatelliteInfo(
                            prn=int(fields[idx]),
                            elevation=float(fields[idx + 1]) if fields[idx + 1] else 0.0,
                            azimuth=float(fields[idx + 2]) if fields[idx + 2] else 0.0,
                            snr=float(fields[idx + 3]) if fields[idx + 3] else 0.0,
                        )
                        satellites.append(sat)
                        self._satellites[sat.prn] = sat
                    except (ValueError, IndexError):
                        pass

            return {
                "type": "GSV",
                "total_messages": total_messages,
                "message_num": message_num,
                "total_satellites": total_satellites,
                "satellites": satellites,
            }
        except (ValueError, IndexError) as e:
            logger.debug("GSV parse error: %s", e)
            return {"type": "GSV", "error": str(e)}

    def _parse_latitude(self, value: str, direction: str) -> float:
        """Parse NMEA latitude to decimal degrees.

        NMEA format is DDMM.MMMMM (degrees and decimal minutes).
        """
        degrees = float(value[:2])
        minutes = float(value[2:])
        decimal = degrees + minutes / 60.0
        if direction == "S":
            decimal = -decimal
        return decimal

    def _parse_longitude(self, value: str, direction: str) -> float:
        """Parse NMEA longitude to decimal degrees.

        NMEA format is DDDMM.MMMMM (degrees and decimal minutes).
        """
        degrees = float(value[:3])
        minutes = float(value[3:])
        decimal = degrees + minutes / 60.0
        if direction == "W":
            decimal = -decimal
        return decimal

    def _parse_time(self, value: str) -> datetime | None:
        """Parse NMEA time to datetime."""
        if len(value) < 6:
            return None
        try:
            hour = int(value[0:2])
            minute = int(value[2:4])
            second = float(value[4:])
            return datetime.now(UTC).replace(
                hour=hour,
                minute=minute,
                second=int(second),
                microsecond=int((second % 1) * 1000000),
            )
        except ValueError:
            return None

    def _parse_date(self, value: str) -> datetime | None:
        """Parse NMEA date to datetime."""
        if len(value) < 6:
            return None
        try:
            day = int(value[0:2])
            month = int(value[2:4])
            year = int(value[4:6]) + 2000
            return datetime(year, month, day, tzinfo=UTC)
        except ValueError:
            return None

    def get_reading(self) -> GPSReading | None:
        """Combine parsed data into a GPS reading.

        Returns:
            GPSReading if sufficient data is available, else None.
        """
        if self._last_gga is None or self._last_gga.get("latitude") is None:
            return None

        gga = self._last_gga
        rmc = self._last_rmc or {}
        gsa = self._last_gsa or {}

        return GPSReading(
            latitude=gga["latitude"],
            longitude=gga["longitude"],
            altitude=gga.get("altitude", 0.0),
            speed=rmc.get("speed_mps", 0.0),
            heading=rmc.get("heading", 0.0),
            fix_quality=gga.get("fix_quality", FixQuality.NO_FIX),
            fix_mode=gsa.get("fix_mode", FixMode.NO_FIX),
            satellites=gga.get("satellites", 0),
            hdop=gga.get("hdop", 99.99),
            vdop=gsa.get("vdop", 99.99),
            pdop=gsa.get("pdop", 99.99),
            timestamp=time.time(),
            age=gga.get("dgps_age"),
        )

    def get_satellites(self) -> list[SatelliteInfo]:
        """Get list of visible satellites."""
        return list(self._satellites.values())


# =============================================================================
# Configuration
# =============================================================================


class GPSConfig(BaseModel):
    """Configuration for GPS sensors."""

    model_config = {"frozen": False, "extra": "allow"}

    # Identification
    name: str = Field(default="gps", description="Sensor name")
    model: GPSModel = Field(default=GPSModel.GENERIC, description="GPS module type")

    # Connection
    port: str = Field(default="/dev/ttyUSB0", description="Serial port")
    baudrate: int = Field(default=9600, description="Serial baudrate")

    # Update rate
    update_rate: float = Field(default=1.0, ge=0.1, le=20.0, description="Update rate in Hz")

    # Filtering
    filter_invalid: bool = Field(default=True, description="Filter out invalid readings")
    min_satellites: int = Field(default=3, ge=0, description="Minimum satellites for valid fix")
    max_hdop: float = Field(default=10.0, gt=0.0, description="Maximum acceptable HDOP")

    # Timeout
    timeout: float = Field(default=2.0, gt=0.0, description="Read timeout in seconds")

    # u-blox specific
    dynamic_model: str = Field(
        default="portable",
        description="u-blox dynamic model (portable, stationary, pedestrian, automotive, sea, airborne)",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# GPS Base Class
# =============================================================================


class GPS(Sensor):
    """Base class for GPS/GNSS sensors.

    Provides position, velocity, and time data from GPS receivers.

    Example:
        >>> gps = SimulatedGPS(config=GPSConfig(name="main_gps"))
        >>> gps.enable()
        >>> gps.connect()
        >>> reading = gps.read_position()
        >>> if reading.has_fix:
        ...     print(f"Position: {reading.latitude:.6f}, {reading.longitude:.6f}")
    """

    def __init__(
        self,
        config: GPSConfig | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize GPS sensor.

        Args:
            config: GPS configuration.
            name: Optional name override.
        """
        self._config = config or GPSConfig()
        self._gps_state = GPSState.IDLE
        self._last_reading: GPSReading | None = None
        self._reading_count = 0
        self._parser = NMEAParser()
        self._info: GPSInfo | None = None

        # Initialize base sensor
        super().__init__(
            name=name or self._config.name,
            driver=None,
            channel=0,
            unit=Unit.RAW,
            limits=Limits(min=-180, max=180, default=0),
        )

    @property
    def config(self) -> GPSConfig:
        """Get GPS configuration."""
        return self._config

    @property
    def gps_state(self) -> GPSState:
        """Get current GPS state."""
        return self._gps_state

    @property
    def has_fix(self) -> bool:
        """Check if GPS has a valid fix."""
        return self._last_reading is not None and self._last_reading.has_fix

    @property
    def last_reading(self) -> GPSReading | None:
        """Get the last GPS reading."""
        return self._last_reading

    @property
    def reading_count(self) -> int:
        """Number of readings captured."""
        return self._reading_count

    @property
    def info(self) -> GPSInfo | None:
        """Get GPS device information."""
        return self._info

    def _read_raw(self) -> int:
        """Read raw value (returns fix quality)."""
        if self._last_reading is not None:
            return int(self._last_reading.fix_quality)
        return 0

    # -------------------------------------------------------------------------
    # Connection
    # -------------------------------------------------------------------------

    @abstractmethod
    def connect(self) -> None:
        """Connect to GPS receiver."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from GPS receiver."""
        ...

    # -------------------------------------------------------------------------
    # Reading
    # -------------------------------------------------------------------------

    @abstractmethod
    def read_position(self, timeout: float | None = None) -> GPSReading:
        """Read current GPS position.

        Args:
            timeout: Maximum time to wait for fix.

        Returns:
            GPSReading with position data.

        Raises:
            CommunicationError: If read fails.
            DisabledError: If sensor is disabled.
        """
        ...

    def read_position_filtered(
        self,
        timeout: float | None = None,
        require_fix: bool = True,
        min_satellites: int | None = None,
        max_hdop: float | None = None,
    ) -> GPSReading | None:
        """Read position with quality filtering.

        Args:
            timeout: Maximum time to wait.
            require_fix: Require valid fix.
            min_satellites: Minimum satellite count.
            max_hdop: Maximum acceptable HDOP.

        Returns:
            GPSReading if quality requirements met, else None.
        """
        reading = self.read_position(timeout)

        if require_fix and not reading.has_fix:
            return None

        min_sats = min_satellites or self._config.min_satellites
        if reading.satellites < min_sats:
            return None

        max_hdop_val = max_hdop or self._config.max_hdop
        if reading.hdop > max_hdop_val:
            return None

        return reading

    async def read_position_async(self, timeout: float | None = None) -> GPSReading:
        """Read position asynchronously.

        Args:
            timeout: Maximum time to wait.

        Returns:
            GPSReading with position data.
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.read_position, timeout)

    async def stream(
        self,
        count: int | None = None,
        timeout: float = 2.0,
    ) -> AsyncIterator[GPSReading]:
        """Stream GPS readings asynchronously.

        Args:
            count: Maximum number of readings (None = infinite).
            timeout: Timeout per reading.

        Yields:
            GPSReading objects.
        """
        readings = 0
        while self._enabled and (count is None or readings < count):
            try:
                reading = await self.read_position_async(timeout)
                yield reading
                readings += 1
            except Exception as e:
                logger.warning("%s: Read failed: %s", self.name, e)
                await asyncio.sleep(0.5)

    def iter_readings(self, count: int | None = None) -> Iterator[GPSReading]:
        """Iterate over GPS readings synchronously.

        Args:
            count: Maximum number of readings (None = infinite).

        Yields:
            GPSReading objects.
        """
        readings = 0
        while self._enabled and (count is None or readings < count):
            try:
                yield self.read_position()
                readings += 1
            except Exception as e:
                logger.warning("%s: Read failed: %s", self.name, e)
                time.sleep(0.5)

    # -------------------------------------------------------------------------
    # Utilities
    # -------------------------------------------------------------------------

    def get_satellites(self) -> list[SatelliteInfo]:
        """Get list of visible satellites.

        Returns:
            List of SatelliteInfo objects.
        """
        return self._parser.get_satellites()

    def distance_traveled(self, start: GPSReading, end: GPSReading) -> float:
        """Calculate distance between two readings.

        Args:
            start: Start position.
            end: End position.

        Returns:
            Distance in meters.
        """
        return start.distance_to(end)

    @abstractmethod
    def get_info(self) -> GPSInfo:
        """Get device information.

        Returns:
            GPSInfo with device details.
        """
        ...


# =============================================================================
# Simulated GPS
# =============================================================================


class SimulatedGPS(GPS):
    """Simulated GPS for testing without hardware.

    Generates synthetic GPS readings with configurable behavior.

    Example:
        >>> gps = SimulatedGPS(start_lat=37.7749, start_lon=-122.4194)
        >>> gps.enable()
        >>> gps.connect()
        >>> for reading in gps.iter_readings(count=5):
        ...     print(f"Lat: {reading.latitude:.6f}")
    """

    def __init__(
        self,
        config: GPSConfig | None = None,
        name: str | None = None,
        start_lat: float = 37.7749,
        start_lon: float = -122.4194,
        start_alt: float = 10.0,
        speed: float = 0.0,
        heading: float = 0.0,
        simulate_movement: bool = False,
        noise_stddev: float = 0.00001,
    ) -> None:
        """Initialize simulated GPS.

        Args:
            config: GPS configuration.
            name: Optional name override.
            start_lat: Starting latitude.
            start_lon: Starting longitude.
            start_alt: Starting altitude.
            speed: Simulated speed in m/s.
            heading: Simulated heading in degrees.
            simulate_movement: Whether to simulate movement.
            noise_stddev: Standard deviation for position noise.
        """
        config = config or GPSConfig(model=GPSModel.SIMULATED)
        super().__init__(config, name or "simulated_gps")

        self._lat = start_lat
        self._lon = start_lon
        self._alt = start_alt
        self._speed = speed
        self._heading = heading
        self._simulate_movement = simulate_movement
        self._noise_stddev = noise_stddev
        self._last_update = time.time()
        self._connected = False

        import random

        self._rng = random.Random()

        self._info = GPSInfo(
            model=GPSModel.SIMULATED,
            firmware_version="1.0.0",
            serial_number="SIM-GPS-001",
            satellites_visible=12,
            satellites_in_use=8,
        )

    def connect(self) -> None:
        """Connect to simulated GPS."""
        self._connected = True
        self._gps_state = GPSState.TRACKING
        logger.info("%s: Simulated GPS connected", self.name)

    def disconnect(self) -> None:
        """Disconnect from simulated GPS."""
        self._connected = False
        self._gps_state = GPSState.IDLE
        logger.info("%s: Simulated GPS disconnected", self.name)

    def read_position(self, timeout: float | None = None) -> GPSReading:
        """Generate a simulated GPS reading.

        Args:
            timeout: Ignored for simulation.

        Returns:
            Simulated GPSReading.
        """
        self._check_enabled()

        if not self._connected:
            raise CommunicationError("GPS not connected")

        # Update position if simulating movement
        if self._simulate_movement and self._speed > 0:
            now = time.time()
            dt = now - self._last_update
            self._last_update = now

            # Calculate movement
            distance = self._speed * dt
            bearing = math.radians(self._heading)

            # Update position (simplified flat-earth calculation)
            dlat = (distance * math.cos(bearing)) / 111320.0
            dlon = (distance * math.sin(bearing)) / (111320.0 * math.cos(math.radians(self._lat)))

            self._lat += dlat
            self._lon += dlon

        # Add noise
        lat_noise = self._rng.gauss(0, self._noise_stddev)
        lon_noise = self._rng.gauss(0, self._noise_stddev)

        self._reading_count += 1
        reading = GPSReading(
            latitude=self._lat + lat_noise,
            longitude=self._lon + lon_noise,
            altitude=self._alt + self._rng.gauss(0, 0.5),
            speed=self._speed + self._rng.gauss(0, 0.1) if self._speed > 0 else 0.0,
            heading=self._heading,
            fix_quality=FixQuality.GPS_FIX,
            fix_mode=FixMode.FIX_3D,
            satellites=8 + self._rng.randint(-2, 4),
            hdop=1.0 + self._rng.random() * 0.5,
            vdop=1.5 + self._rng.random() * 0.5,
            pdop=1.2 + self._rng.random() * 0.5,
            timestamp=time.time(),
        )
        self._last_reading = reading

        # Simulate update delay
        time.sleep(1.0 / self._config.update_rate)

        return reading

    def set_position(self, lat: float, lon: float, alt: float = 10.0) -> None:
        """Set simulated position.

        Args:
            lat: Latitude.
            lon: Longitude.
            alt: Altitude.
        """
        self._lat = lat
        self._lon = lon
        self._alt = alt

    def set_velocity(self, speed: float, heading: float) -> None:
        """Set simulated velocity.

        Args:
            speed: Speed in m/s.
            heading: Heading in degrees.
        """
        self._speed = speed
        self._heading = heading
        self._simulate_movement = speed > 0

    def get_info(self) -> GPSInfo:
        """Get simulated device info."""
        return self._info

    def _check_enabled(self) -> None:
        """Check if sensor is enabled."""
        if not self._is_enabled:
            raise DisabledError(f"{self.name} is disabled")


# =============================================================================
# Serial GPS
# =============================================================================


class SerialGPS(GPS):
    """GPS receiver connected via serial port.

    Supports standard NMEA GPS receivers.

    Example:
        >>> gps = SerialGPS(config=GPSConfig(
        ...     port="/dev/ttyUSB0",
        ...     baudrate=9600
        ... ))
        >>> gps.connect()
        >>> gps.enable()
        >>> reading = gps.read_position()
    """

    def __init__(
        self,
        config: GPSConfig | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize serial GPS.

        Args:
            config: GPS configuration.
            name: Optional name override.
        """
        super().__init__(config, name or "serial_gps")
        self._serial: Any = None

    def connect(self) -> None:
        """Connect to GPS via serial port."""
        if os.environ.get("ROBO_SIMULATION", "").lower() in ("true", "1", "yes"):
            logger.warning("⚠️ SIMULATION MODE - %s not connected to real hardware", self.name)
            self._gps_state = GPSState.TRACKING
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
            self._gps_state = GPSState.SEARCHING
            logger.info(
                "%s: Connected to %s at %d baud",
                self.name,
                self._config.port,
                self._config.baudrate,
            )
        except serial.SerialException as e:
            raise CommunicationError(f"Failed to connect to {self._config.port}: {e}") from e

    def disconnect(self) -> None:
        """Disconnect from GPS."""
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        self._gps_state = GPSState.IDLE
        logger.info("%s: Disconnected", self.name)

    def read_position(self, timeout: float | None = None) -> GPSReading:
        """Read GPS position from NMEA stream.

        Args:
            timeout: Maximum time to wait.

        Returns:
            GPSReading with position data.
        """
        self._check_enabled()

        if self._serial is None:
            # Simulation mode
            return self._generate_simulated_reading()

        timeout_val = timeout or self._config.timeout
        start_time = time.time()

        while time.time() - start_time < timeout_val:
            line = self._serial.readline()
            if not line:
                continue

            try:
                sentence = line.decode("ascii", errors="ignore").strip()
                if sentence.startswith("$"):
                    self._parser.parse(sentence)
            except Exception as e:
                logger.debug("NMEA parse error: %s", e)
                continue

            # Check if we have a complete reading
            reading = self._parser.get_reading()
            if reading is not None:
                self._reading_count += 1
                self._last_reading = reading

                if reading.has_fix:
                    self._gps_state = GPSState.TRACKING
                else:
                    self._gps_state = GPSState.SEARCHING

                return reading

        raise CommunicationError("GPS read timeout")

    def _generate_simulated_reading(self) -> GPSReading:
        """Generate simulated reading for simulation mode."""
        import random

        self._reading_count += 1
        reading = GPSReading(
            latitude=37.7749 + random.gauss(0, 0.0001),
            longitude=-122.4194 + random.gauss(0, 0.0001),
            altitude=10.0 + random.gauss(0, 1.0),
            speed=0.0,
            heading=0.0,
            fix_quality=FixQuality.GPS_FIX,
            fix_mode=FixMode.FIX_3D,
            satellites=8,
            hdop=1.2,
            timestamp=time.time(),
        )
        self._last_reading = reading
        return reading

    def send_command(self, command: str) -> None:
        """Send a command to the GPS receiver.

        Args:
            command: Command string (without $ or checksum).
        """
        if self._serial is None:
            return

        # Calculate checksum
        checksum = 0
        for char in command:
            checksum ^= ord(char)

        sentence = f"${command}*{checksum:02X}\r\n"
        self._serial.write(sentence.encode())
        self._serial.flush()

    def get_info(self) -> GPSInfo:
        """Get GPS device info."""
        satellites = self._parser.get_satellites()
        in_use = sum(1 for s in satellites if s.in_use)

        return GPSInfo(
            model=self._config.model,
            satellites_visible=len(satellites),
            satellites_in_use=in_use,
        )

    def _check_enabled(self) -> None:
        if not self._is_enabled:
            raise DisabledError(f"{self.name} is disabled")


# =============================================================================
# Factory Function
# =============================================================================


def get_gps(
    model: GPSModel | str = GPSModel.GENERIC,
    port: str = "/dev/ttyUSB0",
    baudrate: int = 9600,
    simulation: bool | None = None,
    **kwargs: Any,
) -> GPS:
    """Get a GPS instance for the specified configuration.

    Args:
        model: GPS module type.
        port: Serial port.
        baudrate: Serial baudrate.
        simulation: Force simulation mode (None = auto-detect).
        **kwargs: Additional config options.

    Returns:
        Appropriate GPS instance.

    Example:
        >>> gps = get_gps(port="/dev/ttyUSB0")
        >>> gps.connect()
    """
    if isinstance(model, str):
        try:
            model = GPSModel(model.lower())
        except ValueError:
            model = GPSModel.GENERIC

    # Check for simulation mode
    if simulation is None:
        simulation = os.environ.get("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

    if simulation or model == GPSModel.SIMULATED:
        return SimulatedGPS(config=GPSConfig(model=model, port=port, baudrate=baudrate, **kwargs))

    config = GPSConfig(model=model, port=port, baudrate=baudrate, **kwargs)
    return SerialGPS(config)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Constants
    "EARTH_RADIUS",
    # Classes
    "GPS",
    "KMPH_TO_MPS",
    "KNOTS_TO_MPS",
    "FixMode",
    # Enums
    "FixQuality",
    # Config
    "GPSConfig",
    "GPSInfo",
    "GPSModel",
    # Data classes
    "GPSReading",
    "GPSState",
    # Parser
    "NMEAParser",
    "SatelliteInfo",
    "SerialGPS",
    "SimulatedGPS",
    # Factory
    "get_gps",
]
