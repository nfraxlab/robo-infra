"""Unit tests for core types."""

from robo_infra.core.types import (
    Direction,
    Limits,
    Position,
    Reading,
    Unit,
    Vector3,
)


class TestLimits:
    """Test Limits dataclass."""

    def test_creation(self) -> None:
        """Test basic limits creation."""
        limits = Limits(min=0.0, max=180.0)
        assert limits.min == 0.0
        assert limits.max == 180.0

    def test_clamp_within_range(self) -> None:
        """Test clamping value within range."""
        limits = Limits(min=0.0, max=100.0)
        assert limits.clamp(50.0) == 50.0

    def test_clamp_below_min(self) -> None:
        """Test clamping value below minimum."""
        limits = Limits(min=0.0, max=100.0)
        assert limits.clamp(-10.0) == 0.0

    def test_clamp_above_max(self) -> None:
        """Test clamping value above maximum."""
        limits = Limits(min=0.0, max=100.0)
        assert limits.clamp(150.0) == 100.0

    def test_normalize(self) -> None:
        """Test normalizing value to 0-1 range."""
        limits = Limits(min=0.0, max=100.0)
        assert limits.normalize(50.0) == 0.5
        assert limits.normalize(0.0) == 0.0
        assert limits.normalize(100.0) == 1.0

    def test_denormalize(self) -> None:
        """Test denormalizing value from 0-1 range."""
        limits = Limits(min=0.0, max=100.0)
        assert limits.denormalize(0.5) == 50.0
        assert limits.denormalize(0.0) == 0.0
        assert limits.denormalize(1.0) == 100.0

    def test_is_within(self) -> None:
        """Test is_within check."""
        limits = Limits(min=0.0, max=100.0)
        assert limits.is_within(50.0) is True
        assert limits.is_within(0.0) is True
        assert limits.is_within(100.0) is True
        assert limits.is_within(-1.0) is False
        assert limits.is_within(101.0) is False

    def test_range(self) -> None:
        """Test range calculation."""
        limits = Limits(min=10.0, max=60.0)
        assert (limits.max - limits.min) == 50.0


class TestPosition:
    """Test Position dataclass."""

    def test_creation_default(self) -> None:
        """Test default position creation."""
        pos = Position()
        assert pos.x == 0.0
        assert pos.y == 0.0
        assert pos.z == 0.0
        assert pos.roll == 0.0
        assert pos.pitch == 0.0
        assert pos.yaw == 0.0

    def test_creation_with_values(self) -> None:
        """Test position creation with values."""
        pos = Position(x=1.0, y=2.0, z=3.0, roll=10.0, pitch=20.0, yaw=30.0)
        assert pos.x == 1.0
        assert pos.y == 2.0
        assert pos.z == 3.0
        assert pos.roll == 10.0
        assert pos.pitch == 20.0
        assert pos.yaw == 30.0

    def test_from_xyz(self) -> None:
        """Test from_xyz factory method."""
        pos = Position.from_xyz(1.0, 2.0, 3.0)
        assert pos.x == 1.0
        assert pos.y == 2.0
        assert pos.z == 3.0
        assert pos.roll == 0.0

    def test_to_tuple(self) -> None:
        """Test conversion to tuple."""
        pos = Position(x=1.0, y=2.0, z=3.0)
        t = pos.to_tuple(include_orientation=False)
        assert t == (1.0, 2.0, 3.0)

    def test_to_tuple_with_orientation(self) -> None:
        """Test conversion to tuple with orientation."""
        pos = Position(x=1.0, y=2.0, z=3.0, roll=10.0, pitch=20.0, yaw=30.0)
        t = pos.to_tuple(include_orientation=True)
        assert t == (1.0, 2.0, 3.0, 10.0, 20.0, 30.0)


class TestVector3:
    """Test Vector3 dataclass."""

    def test_creation(self) -> None:
        """Test vector creation."""
        v = Vector3(x=1.0, y=2.0, z=3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_magnitude(self) -> None:
        """Test vector magnitude calculation."""
        v = Vector3(x=3.0, y=4.0, z=0.0)
        # magnitude is a method, not a property
        assert v.magnitude() == 5.0

    def test_normalized(self) -> None:
        """Test vector normalization."""
        v = Vector3(x=3.0, y=4.0, z=0.0)
        n = v.normalized()
        assert abs(n.x - 0.6) < 0.001
        assert abs(n.y - 0.8) < 0.001
        assert abs(n.z) < 0.001

    def test_zero_magnitude_normalization(self) -> None:
        """Test normalizing zero vector returns zero."""
        v = Vector3(x=0.0, y=0.0, z=0.0)
        n = v.normalized()
        assert n.x == 0.0
        assert n.y == 0.0
        assert n.z == 0.0


class TestReading:
    """Test Reading dataclass."""

    def test_creation(self) -> None:
        """Test reading creation."""
        reading = Reading(value=25.5, unit=Unit.CELSIUS)
        assert reading.value == 25.5
        assert reading.unit == Unit.CELSIUS
        assert reading.timestamp is not None


class TestDirection:
    """Test Direction enum."""

    def test_directions_exist(self) -> None:
        """Test all directions exist."""
        assert Direction.FORWARD
        assert Direction.REVERSE
        assert Direction.STOP


class TestUnit:
    """Test Unit enum."""

    def test_common_units_exist(self) -> None:
        """Test common units exist."""
        # Angle
        assert Unit.DEGREES
        assert Unit.RADIANS
        # Distance
        assert Unit.METERS
        assert Unit.MILLIMETERS
        # Temperature
        assert Unit.CELSIUS
        assert Unit.FAHRENHEIT
        # Time
        assert Unit.SECONDS
        assert Unit.MILLISECONDS
        # Force
        assert Unit.NEWTONS
        # Electrical
        assert Unit.VOLTS
        assert Unit.AMPS
