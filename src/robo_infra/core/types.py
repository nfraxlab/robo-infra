"""Common types for robo-infra."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Annotated

from pydantic import Field


# Type aliases
Range = tuple[float, float]
Speed = Annotated[float, Field(ge=0.0, le=1.0)]
Angle = Annotated[float, Field(ge=-360.0, le=360.0)]


class Direction(Enum):
    """Movement direction for actuators."""

    FORWARD = "forward"
    REVERSE = "reverse"
    STOP = "stop"


class Unit(Enum):
    """Measurement units."""

    # Angle
    DEGREES = "deg"
    RADIANS = "rad"

    # Distance
    MILLIMETERS = "mm"
    CENTIMETERS = "cm"
    METERS = "m"
    INCHES = "in"

    # Time
    SECONDS = "s"
    MILLISECONDS = "ms"
    MICROSECONDS = "us"

    # Speed
    RPM = "rpm"
    DEGREES_PER_SECOND = "deg/s"
    RADIANS_PER_SECOND = "rad/s"
    METERS_PER_SECOND = "m/s"

    # Force/Torque
    NEWTONS = "N"
    NEWTON_METERS = "Nm"
    KILOGRAMS = "kg"
    GRAMS = "g"

    # Electrical
    VOLTS = "V"
    AMPS = "A"
    WATTS = "W"
    OHMS = "Ω"

    # Temperature
    CELSIUS = "°C"
    FAHRENHEIT = "°F"
    KELVIN = "K"

    # Pressure
    PASCALS = "Pa"
    HECTOPASCALS = "hPa"
    KILOPASCALS = "kPa"
    BAR = "bar"
    PSI = "psi"
    ATM = "atm"

    # Light
    LUX = "lx"
    LUMENS = "lm"

    # Dimensionless
    PERCENT = "%"
    RATIO = "ratio"
    COUNT = "count"
    RAW = "raw"


@dataclass(frozen=True, slots=True)
class Limits:
    """Value limits for an actuator or sensor."""

    min: float
    max: float
    default: float | None = None

    def __post_init__(self) -> None:
        """Validate limits."""
        if self.min > self.max:
            raise ValueError(f"min ({self.min}) cannot be greater than max ({self.max})")
        if self.default is not None and not (self.min <= self.default <= self.max):
            raise ValueError(
                f"default ({self.default}) must be between min ({self.min}) and max ({self.max})"
            )

    def clamp(self, value: float) -> float:
        """Clamp a value to the limits."""
        return max(self.min, min(self.max, value))

    def is_within(self, value: float) -> bool:
        """Check if a value is within the limits."""
        return self.min <= value <= self.max

    def normalize(self, value: float) -> float:
        """Normalize a value to 0-1 range within limits."""
        if self.max == self.min:
            return 0.0
        return (value - self.min) / (self.max - self.min)

    def denormalize(self, normalized: float) -> float:
        """Convert a 0-1 normalized value back to actual value."""
        return self.min + normalized * (self.max - self.min)


@dataclass(slots=True)
class Position:
    """3D position with optional orientation (6-DOF)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0  # Rotation around X axis
    pitch: float = 0.0  # Rotation around Y axis
    yaw: float = 0.0  # Rotation around Z axis

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float) -> Position:
        """Create position from XYZ coordinates only."""
        return cls(x=x, y=y, z=z)

    @classmethod
    def from_tuple(cls, coords: tuple[float, ...]) -> Position:
        """Create position from tuple (x, y, z) or (x, y, z, roll, pitch, yaw)."""
        if len(coords) == 3:
            return cls(x=coords[0], y=coords[1], z=coords[2])
        elif len(coords) == 6:
            return cls(
                x=coords[0],
                y=coords[1],
                z=coords[2],
                roll=coords[3],
                pitch=coords[4],
                yaw=coords[5],
            )
        else:
            raise ValueError(f"Expected 3 or 6 coordinates, got {len(coords)}")

    def to_tuple(self, include_orientation: bool = True) -> tuple[float, ...]:
        """Convert to tuple."""
        if include_orientation:
            return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)
        return (self.x, self.y, self.z)

    def distance_to(self, other: Position) -> float:
        """Calculate Euclidean distance to another position."""
        return math.sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )


@dataclass(slots=True)
class Vector3:
    """3D vector for accelerations, angular velocities, etc."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def magnitude(self) -> float:
        """Calculate the magnitude of the vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Vector3:
        """Return a unit vector in the same direction."""
        mag = self.magnitude()
        if mag == 0:
            return Vector3(0, 0, 0)
        return Vector3(self.x / mag, self.y / mag, self.z / mag)

    def to_tuple(self) -> tuple[float, float, float]:
        """Convert to tuple."""
        return (self.x, self.y, self.z)


@dataclass(slots=True)
class Quaternion:
    """Quaternion for 3D orientation representation.

    Uses Hamilton convention (w, x, y, z) where:
    - w is the scalar (real) part
    - (x, y, z) is the vector (imaginary) part
    """

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def magnitude(self) -> float:
        """Calculate the magnitude (norm) of the quaternion."""
        return math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Quaternion:
        """Return a unit quaternion in the same direction."""
        mag = self.magnitude()
        if mag == 0:
            return Quaternion(1.0, 0.0, 0.0, 0.0)
        return Quaternion(self.w / mag, self.x / mag, self.y / mag, self.z / mag)

    def conjugate(self) -> Quaternion:
        """Return the conjugate of this quaternion."""
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def to_tuple(self) -> tuple[float, float, float, float]:
        """Convert to tuple (w, x, y, z)."""
        return (self.w, self.x, self.y, self.z)

    @classmethod
    def identity(cls) -> Quaternion:
        """Return identity quaternion (no rotation)."""
        return cls(1.0, 0.0, 0.0, 0.0)


@dataclass(slots=True)
class Reading:
    """A sensor reading with metadata."""

    value: float
    unit: Unit = Unit.RAW
    timestamp: float = field(default_factory=lambda: 0.0)
    raw: int | None = None

    def __post_init__(self) -> None:
        """Set timestamp if not provided."""
        if self.timestamp == 0.0:
            import time

            self.timestamp = time.time()


# Unit conversion utilities
def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


def normalize_angle(angle: float, min_angle: float = -180.0, max_angle: float = 180.0) -> float:
    """Normalize an angle to a given range.

    Args:
        angle: The angle to normalize
        min_angle: Minimum of the target range (default -180)
        max_angle: Maximum of the target range (default 180)

    Returns:
        The angle normalized to [min_angle, max_angle)
    """
    range_size = max_angle - min_angle
    return ((angle - min_angle) % range_size) + min_angle


def wrap_angle_180(angle: float) -> float:
    """Wrap angle to [-180, 180) range."""
    return normalize_angle(angle, -180.0, 180.0)


def wrap_angle_360(angle: float) -> float:
    """Wrap angle to [0, 360) range."""
    return normalize_angle(angle, 0.0, 360.0)


def map_range(
    value: float,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
) -> float:
    """Map a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Distance conversions
def mm_to_m(mm: float) -> float:
    """Convert millimeters to meters."""
    return mm / 1000.0


def m_to_mm(m: float) -> float:
    """Convert meters to millimeters."""
    return m * 1000.0


def inches_to_mm(inches: float) -> float:
    """Convert inches to millimeters."""
    return inches * 25.4


def mm_to_inches(mm: float) -> float:
    """Convert millimeters to inches."""
    return mm / 25.4


# Temperature conversions
def celsius_to_fahrenheit(celsius: float) -> float:
    """Convert Celsius to Fahrenheit."""
    return celsius * 9 / 5 + 32


def fahrenheit_to_celsius(fahrenheit: float) -> float:
    """Convert Fahrenheit to Celsius."""
    return (fahrenheit - 32) * 5 / 9


def celsius_to_kelvin(celsius: float) -> float:
    """Convert Celsius to Kelvin."""
    return celsius + 273.15


def kelvin_to_celsius(kelvin: float) -> float:
    """Convert Kelvin to Celsius."""
    return kelvin - 273.15


# Speed conversions
def rpm_to_rad_per_sec(rpm: float) -> float:
    """Convert RPM to radians per second."""
    return rpm * 2 * math.pi / 60


def rad_per_sec_to_rpm(rad_per_sec: float) -> float:
    """Convert radians per second to RPM."""
    return rad_per_sec * 60 / (2 * math.pi)


def deg_per_sec_to_rad_per_sec(deg_per_sec: float) -> float:
    """Convert degrees per second to radians per second."""
    return degrees_to_radians(deg_per_sec)


def rad_per_sec_to_deg_per_sec(rad_per_sec: float) -> float:
    """Convert radians per second to degrees per second."""
    return radians_to_degrees(rad_per_sec)
