"""Encoder sensor implementations.

Phase 4.3 provides encoder sensors:
- QuadratureEncoder (incremental, A/B channels with direction)
- AbsoluteEncoder (absolute position, single or multi-turn)

All encoders extend `Sensor` via a shared `Encoder` base class.

Notes:
- The core `Sensor` abstraction expects `_read_raw() -> int`.
- Encoders return count/position data.
- QuadratureEncoder tracks relative position with direction detection.
- AbsoluteEncoder provides absolute position within a revolution.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.exceptions import CommunicationError
from robo_infra.core.sensor import Sensor
from robo_infra.core.types import Limits, Unit


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus, SPIBus
    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import DigitalPin


# -----------------------------------------------------------------------------
# Enums
# -----------------------------------------------------------------------------


class EncoderDirection(Enum):
    """Direction of encoder rotation."""

    FORWARD = "forward"
    REVERSE = "reverse"
    STATIONARY = "stationary"


# -----------------------------------------------------------------------------
# Base class
# -----------------------------------------------------------------------------


class Encoder(Sensor):
    """Base class for encoder sensors.

    Encoders measure rotational or linear position by counting pulses
    or reading absolute position values.

    Provides:
    - `read_count()`: Current count/position as integer
    - `read_position()`: Position in configured units (degrees, mm, etc.)
    - `reset()`: Zero the counter
    """

    def __init__(
        self,
        name: str,
        *,
        driver: Driver | None = None,
        channel: int = 0,
        unit: Unit = Unit.COUNT,
        limits: Limits | None = None,
        counts_per_revolution: int = 0,
        **kwargs: Any,
    ) -> None:
        """Initialize encoder.

        Args:
            name: Sensor name
            driver: Optional driver for hardware communication
            channel: Driver channel number
            unit: Unit of measurement (COUNT, DEGREES, etc.)
            limits: Value limits
            counts_per_revolution: Pulses per full revolution (for angular conversion)
            **kwargs: Additional Sensor arguments
        """
        super().__init__(
            name=name,
            driver=driver,
            channel=channel,
            unit=unit,
            limits=limits or Limits(min=float("-inf"), max=float("inf")),
            **kwargs,
        )
        self._counts_per_revolution = counts_per_revolution
        self._count: int = 0
        self._direction = EncoderDirection.STATIONARY

    @property
    def count(self) -> int:
        """Current encoder count."""
        return self._count

    @property
    def direction(self) -> EncoderDirection:
        """Current direction of rotation."""
        return self._direction

    @property
    def counts_per_revolution(self) -> int:
        """Pulses per revolution (PPR)."""
        return self._counts_per_revolution

    def read_count(self) -> int:
        """Read current count value.

        Returns:
            Current encoder count (may be negative for reverse rotation).
        """
        _ = self.read()  # Trigger read cycle
        return self._count

    def read_position(self) -> float:
        """Read position in configured units.

        For angular encoders, converts count to degrees.
        For linear encoders, converts count to distance units.

        Returns:
            Position in the encoder's configured unit.
        """
        reading = self.read()
        return reading.value

    def reset(self) -> None:
        """Reset encoder count to zero."""
        self._count = 0
        self._direction = EncoderDirection.STATIONARY


# -----------------------------------------------------------------------------
# Quadrature Encoder
# -----------------------------------------------------------------------------


class QuadratureConfig(BaseModel):
    """Configuration for quadrature encoders."""

    name: str = "QuadratureEncoder"
    unit: Unit = Unit.COUNT

    # Resolution
    pulses_per_revolution: int = Field(default=100, ge=1, description="Pulses per revolution (PPR)")
    counts_per_pulse: int = Field(
        default=4, ge=1, le=4, description="Counts per pulse (1, 2, or 4 for edges)"
    )

    # Direction
    invert_direction: bool = False

    # Speed calculation
    speed_sample_period_s: float = Field(
        default=0.1, gt=0, description="Period for speed calculation"
    )

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}

    @property
    def counts_per_revolution(self) -> int:
        """Total counts per revolution (PPR x edges)."""
        return self.pulses_per_revolution * self.counts_per_pulse


@dataclass
class QuadratureStatus:
    """Status of a quadrature encoder."""

    count: int = 0
    direction: EncoderDirection = EncoderDirection.STATIONARY
    speed_cps: float = 0.0  # counts per second
    last_a: bool = False
    last_b: bool = False


class QuadratureEncoder(Encoder):
    """Quadrature (incremental) encoder.

    Uses two phase-shifted signals (A and B) to detect:
    - Position (count)
    - Direction (which phase leads)
    - Speed (counts per second)

    Supports:
    - Pin-based: Direct GPIO pins for A and B channels
    - Driver-based: Hardware counter via driver channel

    Example:
        >>> from robo_infra.core.pin import SimulatedDigitalPin, PinMode
        >>> pin_a = SimulatedDigitalPin(0, mode=PinMode.INPUT)
        >>> pin_b = SimulatedDigitalPin(1, mode=PinMode.INPUT)
        >>> encoder = QuadratureEncoder(pin_a=pin_a, pin_b=pin_b, ppr=100)
        >>> encoder.enable()
        >>> count = encoder.read_count()
    """

    def __init__(
        self,
        *,
        pin_a: DigitalPin | None = None,
        pin_b: DigitalPin | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        ppr: int = 100,
        name: str = "QuadratureEncoder",
        config: QuadratureConfig | None = None,
    ) -> None:
        """Initialize quadrature encoder.

        Args:
            pin_a: Channel A digital input pin
            pin_b: Channel B digital input pin
            driver: Alternative driver (hardware counter)
            channel: Driver channel number
            ppr: Pulses per revolution
            name: Encoder name
            config: Full configuration object
        """
        if config is not None:
            self._config = config
            name = config.name
            ppr = config.pulses_per_revolution
        else:
            self._config = QuadratureConfig(
                name=name,
                pulses_per_revolution=ppr,
            )

        self._pin_a = pin_a
        self._pin_b = pin_b
        self._status = QuadratureStatus()

        # Speed tracking
        self._last_count: int = 0
        self._last_speed_time: float = 0.0
        self._speed_cps: float = 0.0

        super().__init__(
            name=name,
            driver=driver,
            channel=channel,
            unit=self._config.unit,
            counts_per_revolution=self._config.counts_per_revolution,
        )

    def enable(self) -> None:
        """Enable the encoder and set up pins."""
        super().enable()
        if self._pin_a is not None and not self._pin_a.initialized:
            self._pin_a.setup()
        if self._pin_b is not None and not self._pin_b.initialized:
            self._pin_b.setup()
        self._last_speed_time = time.time()

    def _read_raw(self) -> int:
        """Read raw encoder count."""
        if self._driver is not None:
            # Driver provides hardware counter value
            try:
                raw = int(self._driver.get_channel(self._channel))
                delta = raw - self._count
                self._count = raw

                # Determine direction from delta
                if delta > 0:
                    self._direction = EncoderDirection.FORWARD
                elif delta < 0:
                    self._direction = EncoderDirection.REVERSE
                else:
                    self._direction = EncoderDirection.STATIONARY

                self._update_speed()
                return raw
            except Exception as e:
                raise CommunicationError(f"Failed to read encoder: {e}") from e

        elif self._pin_a is not None and self._pin_b is not None:
            # Software decoding of quadrature signals
            return self._decode_quadrature()

        # Simulated: return current count
        return self._count

    def _decode_quadrature(self) -> int:
        """Decode quadrature signals and update count.

        Uses a state machine to decode A/B transitions.
        This is a simplified polling-based decoder.
        """
        if self._pin_a is None or self._pin_b is None:
            return self._count

        a = self._pin_a.read()
        b = self._pin_b.read()

        last_a = self._status.last_a
        last_b = self._status.last_b

        # Quadrature state machine
        # State transitions determine direction
        if a != last_a or b != last_b:
            # Determine direction from state transition
            if a != last_a:
                # A changed
                if a == b:
                    self._count -= 1
                    self._direction = EncoderDirection.REVERSE
                else:
                    self._count += 1
                    self._direction = EncoderDirection.FORWARD
            elif a == b:
                # B changed, signals equal
                self._count += 1
                self._direction = EncoderDirection.FORWARD
            else:
                # B changed, signals differ
                self._count -= 1
                self._direction = EncoderDirection.REVERSE

            if self._config.invert_direction:
                self._count = -self._count
                if self._direction == EncoderDirection.FORWARD:
                    self._direction = EncoderDirection.REVERSE
                elif self._direction == EncoderDirection.REVERSE:
                    self._direction = EncoderDirection.FORWARD

        self._status.last_a = a
        self._status.last_b = b
        self._status.count = self._count
        self._status.direction = self._direction

        self._update_speed()
        return self._count

    def _update_speed(self) -> None:
        """Update speed calculation."""
        now = time.time()
        dt = now - self._last_speed_time

        if dt >= self._config.speed_sample_period_s:
            delta_count = self._count - self._last_count
            self._speed_cps = delta_count / dt
            self._last_count = self._count
            self._last_speed_time = now
            self._status.speed_cps = self._speed_cps

    def read_speed(self) -> float:
        """Read current speed in counts per second.

        Returns:
            Speed in counts/second (positive for forward, negative for reverse).
        """
        _ = self.read()  # Trigger read and speed update
        return self._speed_cps

    def read_speed_rpm(self) -> float:
        """Read current speed in revolutions per minute.

        Returns:
            Speed in RPM.
        """
        cps = self.read_speed()
        if self._counts_per_revolution <= 0:
            return 0.0
        rps = cps / self._counts_per_revolution
        return rps * 60.0

    def reset(self) -> None:
        """Reset encoder count and speed tracking."""
        super().reset()
        self._last_count = 0
        self._speed_cps = 0.0
        self._last_speed_time = time.time()
        self._status = QuadratureStatus()

    def status(self) -> QuadratureStatus:  # type: ignore[override]
        """Get encoder status."""
        self._status.count = self._count
        self._status.direction = self._direction
        self._status.speed_cps = self._speed_cps
        return self._status

    def simulate_pulses(self, pulses: int) -> None:
        """Simulate encoder pulses for testing.

        Args:
            pulses: Number of pulses (positive for forward, negative for reverse).
        """
        self._count += pulses * self._config.counts_per_pulse
        if pulses > 0:
            self._direction = EncoderDirection.FORWARD
        elif pulses < 0:
            self._direction = EncoderDirection.REVERSE
        self._update_speed()


# -----------------------------------------------------------------------------
# Absolute Encoder
# -----------------------------------------------------------------------------


class AbsoluteConfig(BaseModel):
    """Configuration for absolute encoders."""

    name: str = "AbsoluteEncoder"
    unit: Unit = Unit.DEGREES

    # I2C/SPI settings
    i2c_address: int = Field(default=0x36, description="I2C address")
    spi_cs_pin: int = Field(default=0, description="SPI chip select pin")
    data_register: int = Field(default=0x00, description="Data register address")

    # Resolution
    resolution_bits: int = Field(default=12, description="ADC resolution (bits)")

    # Multi-turn
    multi_turn: bool = Field(default=False, description="Enable multi-turn tracking")
    max_turns: int = Field(default=256, description="Maximum turns to track")

    # Offset
    zero_offset: float = Field(default=0.0, description="Zero position offset (degrees)")

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}

    @property
    def max_count(self) -> int:
        """Maximum count value based on resolution."""
        return (1 << self.resolution_bits) - 1

    @property
    def degrees_per_count(self) -> float:
        """Degrees per count value."""
        return 360.0 / (self.max_count + 1)


@dataclass
class AbsoluteStatus:
    """Status of an absolute encoder."""

    raw_position: int = 0
    position_degrees: float = 0.0
    turns: int = 0
    total_degrees: float = 0.0


class AbsoluteEncoder(Encoder):
    """Absolute position encoder.

    Provides absolute position within a revolution (or multiple revolutions
    with multi-turn support). Position is maintained even after power loss.

    Supports:
    - I2C bus: Common for magnetic encoders (AS5600, AS5048)
    - SPI bus: High-speed interfaces
    - Driver: Generic ADC or dedicated encoder interface

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.open()
        >>> encoder = AbsoluteEncoder(bus=bus, config=AbsoluteConfig())
        >>> encoder.enable()
        >>> degrees = encoder.read_position()
    """

    def __init__(
        self,
        *,
        bus: I2CBus | None = None,
        spi_bus: SPIBus | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        name: str = "AbsoluteEncoder",
        config: AbsoluteConfig | None = None,
    ) -> None:
        """Initialize absolute encoder.

        Args:
            bus: I2C bus for communication
            spi_bus: SPI bus for communication
            driver: Alternative driver
            channel: Driver channel number
            name: Encoder name
            config: Full configuration object
        """
        self._config = config or AbsoluteConfig(name=name)
        self._bus = bus
        self._spi_bus = spi_bus
        self._status = AbsoluteStatus()

        # Multi-turn tracking (initialized before super but reset values set after)
        self._turns: int = 0
        self._total_degrees: float = 0.0
        self._first_multi_turn_read: bool = True

        # Calculate scale for degrees
        scale = self._config.degrees_per_count

        super().__init__(
            name=self._config.name,
            driver=driver,
            channel=channel,
            unit=self._config.unit,
            counts_per_revolution=self._config.max_count + 1,
            scale=scale,
            offset=-self._config.zero_offset,
        )

        # Override _last_raw from base class to 0 for multi-turn tracking
        self._last_raw = 0

    def _read_raw(self) -> int:
        """Read raw position from encoder."""
        raw: int = 0

        if self._bus is not None:
            # Read from I2C
            try:
                data = self._bus.read_register(
                    self._config.i2c_address,
                    self._config.data_register,
                    2,  # 2 bytes for 12-16 bit resolution
                )
                raw = (data[0] << 8) | data[1]
                # Mask to resolution
                raw &= self._config.max_count
            except Exception as e:
                raise CommunicationError(f"Failed to read encoder: {e}") from e

        elif self._spi_bus is not None:
            # Read from SPI
            try:
                data = self._spi_bus.transfer(bytes([0x00, 0x00]))
                raw = (data[0] << 8) | data[1]
                raw &= self._config.max_count
            except Exception as e:
                raise CommunicationError(f"Failed to read encoder: {e}") from e

        elif self._driver is not None:
            # Read from driver
            try:
                raw = int(self._driver.get_channel(self._channel))
                raw &= self._config.max_count
            except Exception as e:
                raise CommunicationError(f"Failed to read encoder: {e}") from e

        # Update multi-turn tracking
        if self._config.multi_turn:
            self._update_multi_turn(raw)

        self._count = raw
        self._status.raw_position = raw
        self._status.position_degrees = raw * self._config.degrees_per_count
        self._last_raw = raw

        return raw

    def _update_multi_turn(self, raw: int) -> None:
        """Update multi-turn tracking based on position wraparound.

        Detects when the encoder crosses from max to 0 or vice versa
        to track full revolutions.
        """
        # Skip first read - just establish baseline
        if self._first_multi_turn_read:
            self._first_multi_turn_read = False
            # Update total degrees without turn detection
            single_rev = raw * self._config.degrees_per_count
            self._total_degrees = single_rev
            self._status.turns = 0
            self._status.total_degrees = self._total_degrees
            return

        max_count = self._config.max_count
        half_count = max_count // 2

        # Handle potential None from base class
        last_raw = self._last_raw if self._last_raw is not None else 0
        delta = raw - last_raw

        # Detect wraparound
        if delta > half_count:
            # Wrapped from low to high (reverse direction)
            self._turns -= 1
        elif delta < -half_count:
            # Wrapped from high to low (forward direction)
            self._turns += 1

        # Clamp turns
        self._turns = max(-self._config.max_turns, min(self._config.max_turns, self._turns))

        # Calculate total degrees
        single_rev = raw * self._config.degrees_per_count
        self._total_degrees = (self._turns * 360.0) + single_rev

        self._status.turns = self._turns
        self._status.total_degrees = self._total_degrees

    def read_degrees(self) -> float:
        """Read position in degrees (0-360).

        Returns:
            Position in degrees within single revolution.
        """
        _ = self.read()
        return self._status.position_degrees - self._config.zero_offset

    def read_total_degrees(self) -> float:
        """Read total degrees including multi-turn.

        Only meaningful if multi_turn is enabled.

        Returns:
            Total degrees including full revolutions.
        """
        _ = self.read()
        return self._total_degrees - self._config.zero_offset

    def read_turns(self) -> int:
        """Read number of full turns.

        Only meaningful if multi_turn is enabled.

        Returns:
            Number of complete revolutions (positive or negative).
        """
        _ = self.read()
        return self._turns

    def set_zero(self) -> None:
        """Set current position as zero.

        Updates the zero_offset in config to make current position = 0.
        """
        current = self._status.position_degrees
        self._config.zero_offset = current
        self._offset = -current

    def reset(self) -> None:
        """Reset multi-turn tracking."""
        super().reset()
        self._turns = 0
        self._total_degrees = 0.0
        self._first_multi_turn_read = True
        self._status = AbsoluteStatus()

    def status(self) -> AbsoluteStatus:  # type: ignore[override]
        """Get encoder status."""
        return self._status
