"""Unit tests for encoder sensors (Phase 4.3)."""

from __future__ import annotations

import pytest

from robo_infra.core.bus import SimulatedI2CBus
from robo_infra.core.pin import PinMode, SimulatedDigitalPin
from robo_infra.core.types import Unit
from robo_infra.sensors.encoder import (
    AbsoluteConfig,
    AbsoluteEncoder,
    AbsoluteStatus,
    EncoderDirection,
    QuadratureConfig,
    QuadratureEncoder,
    QuadratureStatus,
)


# =============================================================================
# Encoder Base Class Tests
# =============================================================================


class TestEncoderBase:
    """Tests for Encoder base class (via QuadratureEncoder)."""

    def test_encoder_initialization(self) -> None:
        """Test encoder initialization with defaults."""
        encoder = QuadratureEncoder(name="TestEncoder")

        assert encoder.name == "TestEncoder"
        assert encoder.count == 0
        assert encoder.direction == EncoderDirection.STATIONARY

    def test_encoder_with_cpr(self) -> None:
        """Test encoder with counts per revolution."""
        encoder = QuadratureEncoder(ppr=100, name="RotaryEncoder")

        # 100 PPR * 4 edges = 400 CPR
        assert encoder.counts_per_revolution == 400

    def test_encoder_reset(self) -> None:
        """Test encoder reset."""
        encoder = QuadratureEncoder(name="TestEncoder")
        encoder._count = 100
        encoder._direction = EncoderDirection.FORWARD

        encoder.reset()

        assert encoder.count == 0
        assert encoder.direction == EncoderDirection.STATIONARY


# =============================================================================
# Quadrature Encoder Config Tests
# =============================================================================


class TestQuadratureConfig:
    """Tests for QuadratureConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = QuadratureConfig()

        assert config.name == "QuadratureEncoder"
        assert config.unit == Unit.COUNT
        assert config.pulses_per_revolution == 100
        assert config.counts_per_pulse == 4
        assert config.invert_direction is False
        assert config.speed_sample_period_s == 0.1

    def test_counts_per_revolution_calculated(self) -> None:
        """Test CPR calculation."""
        config = QuadratureConfig(pulses_per_revolution=200, counts_per_pulse=4)

        assert config.counts_per_revolution == 800

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = QuadratureConfig(
            name="HighResEncoder",
            pulses_per_revolution=2048,
            counts_per_pulse=4,
            invert_direction=True,
        )

        assert config.name == "HighResEncoder"
        assert config.pulses_per_revolution == 2048
        assert config.counts_per_revolution == 8192
        assert config.invert_direction is True


# =============================================================================
# Quadrature Encoder Tests
# =============================================================================


class TestQuadratureEncoder:
    """Tests for QuadratureEncoder."""

    def test_encoder_initialization_default(self) -> None:
        """Test default encoder initialization."""
        encoder = QuadratureEncoder()

        assert encoder.name == "QuadratureEncoder"
        assert encoder.count == 0
        assert encoder.direction == EncoderDirection.STATIONARY

    def test_encoder_with_ppr(self) -> None:
        """Test encoder with PPR parameter."""
        encoder = QuadratureEncoder(ppr=200, name="Motor1Encoder")

        assert encoder.name == "Motor1Encoder"
        assert encoder.counts_per_revolution == 800  # 200 * 4 edges

    def test_encoder_with_config(self) -> None:
        """Test encoder with config object."""
        config = QuadratureConfig(
            name="ConfiguredEncoder",
            pulses_per_revolution=500,
        )
        encoder = QuadratureEncoder(config=config)

        assert encoder.name == "ConfiguredEncoder"
        assert encoder.counts_per_revolution == 2000

    def test_encoder_with_pins(self) -> None:
        """Test encoder with digital pins."""
        pin_a = SimulatedDigitalPin(0, mode=PinMode.INPUT)
        pin_b = SimulatedDigitalPin(1, mode=PinMode.INPUT)

        encoder = QuadratureEncoder(pin_a=pin_a, pin_b=pin_b, ppr=100)
        encoder.enable()

        assert encoder._pin_a is pin_a
        assert encoder._pin_b is pin_b
        assert pin_a.initialized
        assert pin_b.initialized

    def test_encoder_read_count(self) -> None:
        """Test reading encoder count."""
        encoder = QuadratureEncoder(ppr=100)
        encoder.enable()

        count = encoder.read_count()

        assert count == 0

    def test_encoder_simulate_pulses_forward(self) -> None:
        """Test simulating forward pulses."""
        encoder = QuadratureEncoder(ppr=100)
        encoder.enable()

        encoder.simulate_pulses(10)

        assert encoder.count == 40  # 10 pulses * 4 counts/pulse
        assert encoder.direction == EncoderDirection.FORWARD

    def test_encoder_simulate_pulses_reverse(self) -> None:
        """Test simulating reverse pulses."""
        encoder = QuadratureEncoder(ppr=100)
        encoder.enable()

        encoder.simulate_pulses(-5)

        assert encoder.count == -20  # -5 pulses * 4 counts/pulse
        assert encoder.direction == EncoderDirection.REVERSE

    def test_encoder_reset(self) -> None:
        """Test encoder reset."""
        encoder = QuadratureEncoder(ppr=100)
        encoder.enable()
        encoder.simulate_pulses(50)

        encoder.reset()

        assert encoder.count == 0
        assert encoder.direction == EncoderDirection.STATIONARY
        status = encoder.status()
        assert status.count == 0
        assert status.speed_cps == 0.0

    def test_encoder_read_speed(self) -> None:
        """Test reading speed."""
        encoder = QuadratureEncoder(ppr=100)
        encoder.enable()

        speed = encoder.read_speed()

        assert speed == 0.0  # No motion

    def test_encoder_read_speed_rpm(self) -> None:
        """Test reading speed in RPM."""
        encoder = QuadratureEncoder(ppr=100)
        encoder.enable()

        rpm = encoder.read_speed_rpm()

        assert rpm == 0.0

    def test_encoder_status(self) -> None:
        """Test encoder status."""
        encoder = QuadratureEncoder(ppr=100)
        encoder.enable()
        encoder.simulate_pulses(25)

        status = encoder.status()

        assert isinstance(status, QuadratureStatus)
        assert status.count == 100  # 25 * 4
        assert status.direction == EncoderDirection.FORWARD


class TestQuadratureEncoderPinDecoding:
    """Tests for quadrature signal decoding with pins."""

    def test_decode_a_rising_b_low_forward(self) -> None:
        """Test A rising with B low = forward."""
        pin_a = SimulatedDigitalPin(0, mode=PinMode.INPUT)
        pin_b = SimulatedDigitalPin(1, mode=PinMode.INPUT)

        encoder = QuadratureEncoder(pin_a=pin_a, pin_b=pin_b, ppr=100)
        encoder.enable()

        # Initial state: A=low, B=low
        pin_a.write(False)
        pin_b.write(False)
        _ = encoder.read_count()

        # A goes high while B is low -> forward
        pin_a.write(True)
        count = encoder.read_count()

        assert count == 1
        assert encoder.direction == EncoderDirection.FORWARD

    def test_decode_a_rising_b_high_reverse(self) -> None:
        """Test A rising with B high = reverse."""
        pin_a = SimulatedDigitalPin(0, mode=PinMode.INPUT)
        pin_b = SimulatedDigitalPin(1, mode=PinMode.INPUT)

        encoder = QuadratureEncoder(pin_a=pin_a, pin_b=pin_b, ppr=100)
        encoder.enable()

        # Initial state: A=low, B=high
        pin_a.write(False)
        pin_b.write(True)
        _ = encoder.read_count()
        initial = encoder.count

        # A goes high while B is high -> reverse
        pin_a.write(True)
        _ = encoder.read_count()

        # Count should decrease (reverse direction)
        assert encoder.count < initial
        assert encoder.direction == EncoderDirection.REVERSE

    def test_decode_sequence_forward(self) -> None:
        """Test forward rotation sequence."""
        pin_a = SimulatedDigitalPin(0, mode=PinMode.INPUT)
        pin_b = SimulatedDigitalPin(1, mode=PinMode.INPUT)

        encoder = QuadratureEncoder(pin_a=pin_a, pin_b=pin_b, ppr=100)
        encoder.enable()

        # Full quadrature cycle forward: 00 -> 10 -> 11 -> 01 -> 00
        # Initial state
        pin_a.write(False)
        pin_b.write(False)
        _ = encoder.read_count()

        # 00 -> 10 (A rises, B low)
        pin_a.write(True)
        _ = encoder.read_count()

        # 10 -> 11 (B rises, A high) -> forward since A != B before
        pin_b.write(True)
        _ = encoder.read_count()

        # 11 -> 01 (A falls, B high)
        pin_a.write(False)
        count = encoder.read_count()

        # Count should have incremented
        assert count != 0

    def test_no_change_no_count(self) -> None:
        """Test no state change = no count change."""
        pin_a = SimulatedDigitalPin(0, mode=PinMode.INPUT)
        pin_b = SimulatedDigitalPin(1, mode=PinMode.INPUT)

        encoder = QuadratureEncoder(pin_a=pin_a, pin_b=pin_b, ppr=100)
        encoder.enable()

        pin_a.write(True)
        pin_b.write(False)
        initial = encoder.read_count()

        # Same state
        pin_a.write(True)
        pin_b.write(False)
        same = encoder.read_count()

        assert same == initial


# =============================================================================
# Absolute Encoder Config Tests
# =============================================================================


class TestAbsoluteConfig:
    """Tests for AbsoluteConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = AbsoluteConfig()

        assert config.name == "AbsoluteEncoder"
        assert config.unit == Unit.DEGREES
        assert config.i2c_address == 0x36
        assert config.resolution_bits == 12
        assert config.multi_turn is False
        assert config.zero_offset == 0.0

    def test_max_count_calculation(self) -> None:
        """Test max count based on resolution."""
        config = AbsoluteConfig(resolution_bits=12)

        assert config.max_count == 4095

    def test_degrees_per_count(self) -> None:
        """Test degrees per count calculation."""
        config = AbsoluteConfig(resolution_bits=12)

        # 360 / 4096 = 0.087890625
        assert config.degrees_per_count == pytest.approx(0.087890625)

    def test_high_resolution_config(self) -> None:
        """Test high resolution configuration."""
        config = AbsoluteConfig(resolution_bits=14)

        assert config.max_count == 16383
        assert config.degrees_per_count == pytest.approx(360.0 / 16384)


# =============================================================================
# Absolute Encoder Tests
# =============================================================================


class TestAbsoluteEncoder:
    """Tests for AbsoluteEncoder."""

    def test_encoder_initialization(self) -> None:
        """Test default encoder initialization."""
        encoder = AbsoluteEncoder()

        assert encoder.name == "AbsoluteEncoder"
        assert encoder.unit == Unit.DEGREES
        assert encoder.count == 0

    def test_encoder_with_config(self) -> None:
        """Test encoder with config object."""
        config = AbsoluteConfig(
            name="JointEncoder",
            resolution_bits=14,
            multi_turn=True,
        )
        encoder = AbsoluteEncoder(config=config)

        assert encoder.name == "JointEncoder"
        assert encoder._config.multi_turn is True

    def test_encoder_with_i2c_bus(self) -> None:
        """Test encoder with I2C bus."""
        bus = SimulatedI2CBus()
        bus.open()

        # Register device data: 2048 raw counts = 0x0800
        # Register 0x00 = 0x08, Register 0x01 = 0x00
        bus.add_device(0x36, registers={0x00: 0x08, 0x01: 0x00})

        config = AbsoluteConfig(i2c_address=0x36, data_register=0x00)
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        _ = encoder.read()

        # Raw position should be 2048 (0x0800)
        status = encoder.status()
        assert status.raw_position == 2048

    def test_encoder_read_degrees(self) -> None:
        """Test reading position in degrees."""
        bus = SimulatedI2CBus()
        bus.open()

        # Position at 180 degrees (2048 out of 4096)
        # 2048 = 0x0800
        bus.add_device(0x36, registers={0x00: 0x08, 0x01: 0x00})

        config = AbsoluteConfig(i2c_address=0x36, data_register=0x00)
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        degrees = encoder.read_degrees()

        assert degrees == pytest.approx(180.0, abs=0.1)

    def test_encoder_with_zero_offset(self) -> None:
        """Test encoder with zero offset."""
        bus = SimulatedI2CBus()
        bus.open()

        # 2048 = 0x0800 = 180 degrees
        bus.add_device(0x36, registers={0x00: 0x08, 0x01: 0x00})

        config = AbsoluteConfig(
            i2c_address=0x36,
            data_register=0x00,
            zero_offset=180.0,
        )
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        degrees = encoder.read_degrees()

        assert degrees == pytest.approx(0.0, abs=0.1)

    def test_encoder_set_zero(self) -> None:
        """Test setting current position as zero."""
        bus = SimulatedI2CBus()
        bus.open()

        # 1024 = 0x0400 = 90 degrees
        bus.add_device(0x36, registers={0x00: 0x04, 0x01: 0x00})

        config = AbsoluteConfig(i2c_address=0x36, data_register=0x00)
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        # Read initial position
        degrees = encoder.read_degrees()
        assert degrees == pytest.approx(90.0, abs=0.1)

        # Set zero
        encoder.set_zero()

        # Now should read as zero (same raw position, offset applied)
        degrees_after = encoder.read_degrees()
        assert degrees_after == pytest.approx(0.0, abs=0.1)

    def test_encoder_status(self) -> None:
        """Test encoder status."""
        encoder = AbsoluteEncoder()
        encoder.enable()

        status = encoder.status()

        assert isinstance(status, AbsoluteStatus)
        assert status.raw_position == 0
        assert status.position_degrees == 0.0
        assert status.turns == 0

    def test_encoder_reset(self) -> None:
        """Test encoder reset."""
        bus = SimulatedI2CBus()
        bus.open()

        # Max position: 0x0FFF = 4095
        bus.add_device(0x36, registers={0x00: 0x0F, 0x01: 0xFF})

        config = AbsoluteConfig(
            i2c_address=0x36,
            data_register=0x00,
            multi_turn=True,
        )
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        # Trigger some readings
        _ = encoder.read()

        encoder.reset()

        assert encoder.count == 0
        assert encoder.read_turns() == 0


class TestAbsoluteEncoderMultiTurn:
    """Tests for absolute encoder multi-turn tracking."""

    def test_multi_turn_forward_wrap(self) -> None:
        """Test turn increment on forward wraparound."""
        bus = SimulatedI2CBus()
        bus.open()

        config = AbsoluteConfig(
            i2c_address=0x36,
            data_register=0x00,
            multi_turn=True,
        )
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        # Start near max: 0x0FF0 = 4080
        device = bus.add_device(0x36, registers={0x00: 0x0F, 0x01: 0xF0})
        _ = encoder.read()

        # Wrap to near zero (forward direction): 0x0010 = 16
        device.registers[0x00] = 0x00
        device.registers[0x01] = 0x10
        _ = encoder.read()

        turns = encoder.read_turns()
        assert turns == 1

    def test_multi_turn_reverse_wrap(self) -> None:
        """Test turn decrement on reverse wraparound."""
        bus = SimulatedI2CBus()
        bus.open()

        config = AbsoluteConfig(
            i2c_address=0x36,
            data_register=0x00,
            multi_turn=True,
        )
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        # Start near zero: 0x0010 = 16
        device = bus.add_device(0x36, registers={0x00: 0x00, 0x01: 0x10})
        _ = encoder.read()

        # Wrap to near max (reverse direction): 0x0FF0 = 4080
        device.registers[0x00] = 0x0F
        device.registers[0x01] = 0xF0
        _ = encoder.read()

        turns = encoder.read_turns()
        assert turns == -1

    def test_multi_turn_total_degrees(self) -> None:
        """Test total degrees calculation with turns."""
        bus = SimulatedI2CBus()
        bus.open()

        config = AbsoluteConfig(
            i2c_address=0x36,
            data_register=0x00,
            multi_turn=True,
        )
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        # Simulate one full turn + 90 degrees
        # First, establish position near max: 0x0FF0 = 4080
        device = bus.add_device(0x36, registers={0x00: 0x0F, 0x01: 0xF0})
        _ = encoder.read()

        # Wrap forward: 0x0010 = 16
        device.registers[0x00] = 0x00
        device.registers[0x01] = 0x10
        _ = encoder.read()

        # Move to ~90 degrees (1024 counts): 0x0400
        device.registers[0x00] = 0x04
        device.registers[0x01] = 0x00
        _ = encoder.read()

        total = encoder.read_total_degrees()

        # Should be ~450 degrees (360 + 90)
        assert total == pytest.approx(450.0, abs=1.0)

    def test_multi_turn_disabled(self) -> None:
        """Test that multi-turn is not tracked when disabled."""
        bus = SimulatedI2CBus()
        bus.open()

        config = AbsoluteConfig(
            i2c_address=0x36,
            data_register=0x00,
            multi_turn=False,  # Disabled
        )
        encoder = AbsoluteEncoder(bus=bus, config=config)
        encoder.enable()

        # Start near max: 0x0FF0 = 4080
        device = bus.add_device(0x36, registers={0x00: 0x0F, 0x01: 0xF0})
        _ = encoder.read()

        # Wrap forward: 0x0010 = 16
        device.registers[0x00] = 0x00
        device.registers[0x01] = 0x10
        _ = encoder.read()

        turns = encoder.read_turns()
        assert turns == 0  # No tracking


# =============================================================================
# Direction Enum Tests
# =============================================================================


class TestEncoderDirection:
    """Tests for EncoderDirection enum."""

    def test_direction_values(self) -> None:
        """Test direction enum values."""
        assert EncoderDirection.FORWARD.value == "forward"
        assert EncoderDirection.REVERSE.value == "reverse"
        assert EncoderDirection.STATIONARY.value == "stationary"

    def test_all_directions_accessible(self) -> None:
        """Test all directions are accessible."""
        directions = list(EncoderDirection)
        assert len(directions) == 3
