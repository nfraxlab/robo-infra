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
    """Normalize an angle to a given range."""
    range_size = max_angle - min_angle
    return ((angle - min_angle) % range_size) + min_angle


def map_range(
    value: float,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
) -> float:
    """Map a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
