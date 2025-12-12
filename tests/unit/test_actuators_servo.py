"""Tests for robo_infra.actuators.servo module."""

from __future__ import annotations

import pytest

from robo_infra.actuators.servo import (
    HIGH_FREQUENCY,
    STANDARD_FREQUENCY,
    STANDARD_PULSE_CENTER,
    STANDARD_PULSE_MAX,
    STANDARD_PULSE_MIN,
    Servo,
    ServoConfig,
    ServoRange,
    ServoStatus,
    ServoType,
    SimulatedServo,
    create_servo,
)
from robo_infra.core.actuator import ActuatorState, ActuatorType
from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import (
    CalibrationError,
    DisabledError,
    LimitsExceededError,
)


# =============================================================================
# Constants Tests
# =============================================================================


class TestServoConstants:
    """Tests for servo constants."""

    def test_standard_pulse_values(self) -> None:
        """Test standard pulse width constants."""
        assert STANDARD_PULSE_MIN == 500  # 0.5ms
        assert STANDARD_PULSE_MAX == 2500  # 2.5ms
        assert STANDARD_PULSE_CENTER == 1500  # 1.5ms

    def test_frequency_constants(self) -> None:
        """Test frequency constants."""
        assert STANDARD_FREQUENCY == 50
        assert HIGH_FREQUENCY == 333


# =============================================================================
# Enum Tests
# =============================================================================


class TestServoEnums:
    """Tests for servo enums."""

    def test_servo_type_values(self) -> None:
        """Test ServoType enum values."""
        assert ServoType.STANDARD.value == "standard"
        assert ServoType.CONTINUOUS.value == "continuous"
        assert ServoType.LINEAR.value == "linear"
        assert ServoType.SAIL_WINCH.value == "sail_winch"

    def test_servo_range_values(self) -> None:
        """Test ServoRange enum values."""
        assert ServoRange.RANGE_90.value == (0, 90)
        assert ServoRange.RANGE_180.value == (0, 180)
        assert ServoRange.RANGE_270.value == (0, 270)
        assert ServoRange.RANGE_360.value == (0, 360)


# =============================================================================
# ServoConfig Tests
# =============================================================================


class TestServoConfig:
    """Tests for ServoConfig model."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = ServoConfig()

        assert config.name == "Servo"
        assert config.channel == 0
        assert config.angle_range == (0.0, 180.0)
        assert config.pulse_range == (STANDARD_PULSE_MIN, STANDARD_PULSE_MAX)
        assert config.frequency == STANDARD_FREQUENCY
        assert config.servo_type == ServoType.STANDARD
        assert config.inverted is False
        assert config.offset == 0.0
        assert config.speed_deg_per_sec is None
        assert config.require_calibration is False
        assert config.trim == 0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = ServoConfig(
            name="Shoulder",
            channel=3,
            angle_range=(0.0, 270.0),
            pulse_range=(600, 2400),
            frequency=100,
            servo_type=ServoType.CONTINUOUS,
            inverted=True,
            offset=5.0,
            speed_deg_per_sec=120.0,
            trim=10,
        )

        assert config.name == "Shoulder"
        assert config.channel == 3
        assert config.angle_range == (0.0, 270.0)
        assert config.pulse_range == (600, 2400)
        assert config.frequency == 100
        assert config.servo_type == ServoType.CONTINUOUS
        assert config.inverted is True
        assert config.offset == 5.0
        assert config.speed_deg_per_sec == 120.0
        assert config.trim == 10

    def test_config_computed_properties(self) -> None:
        """Test computed properties."""
        config = ServoConfig(
            angle_range=(10.0, 170.0),
            pulse_range=(600, 2400),
        )

        assert config.angle_min == 10.0
        assert config.angle_max == 170.0
        assert config.pulse_min == 600
        assert config.pulse_max == 2400
        assert config.angle_span == 160.0
        assert config.pulse_span == 1800


# =============================================================================
# ServoStatus Tests
# =============================================================================


class TestServoStatus:
    """Tests for ServoStatus dataclass."""

    def test_default_status(self) -> None:
        """Test default status values."""
        status = ServoStatus()

        assert status.state == ActuatorState.DISABLED
        assert status.angle == 0.0
        assert status.target_angle is None
        assert status.pulse_width == STANDARD_PULSE_CENTER
        assert status.is_enabled is False
        assert status.is_calibrated is False
        assert status.error is None

    def test_custom_status(self) -> None:
        """Test custom status values."""
        status = ServoStatus(
            state=ActuatorState.MOVING,
            angle=90.0,
            target_angle=180.0,
            pulse_width=1500,
            is_enabled=True,
            is_calibrated=True,
            error=None,
        )

        assert status.state == ActuatorState.MOVING
        assert status.angle == 90.0
        assert status.target_angle == 180.0
        assert status.pulse_width == 1500
        assert status.is_enabled is True


# =============================================================================
# Servo Tests
# =============================================================================


class TestServoInit:
    """Tests for Servo initialization."""

    def test_default_init(self) -> None:
        """Test default initialization."""
        servo = Servo()

        assert servo.name == "Servo"
        assert servo.channel == 0
        assert servo.angle_range == (0.0, 180.0)
        assert servo.pulse_range == (STANDARD_PULSE_MIN, STANDARD_PULSE_MAX)
        assert servo.frequency == STANDARD_FREQUENCY
        assert servo.servo_type == ServoType.STANDARD
        assert servo.actuator_type == ActuatorType.SERVO

    def test_custom_init(self) -> None:
        """Test custom initialization."""
        servo = Servo(
            name="TestServo",
            channel=5,
            angle_range=(0.0, 270.0),
            pulse_range=(600, 2400),
            frequency=100,
            inverted=True,
        )

        assert servo.name == "TestServo"
        assert servo.channel == 5
        assert servo.angle_range == (0.0, 270.0)
        assert servo.pulse_range == (600, 2400)
        assert servo.frequency == 100

    def test_init_with_config(self) -> None:
        """Test initialization with ServoConfig."""
        config = ServoConfig(
            name="ConfigServo",
            channel=2,
            angle_range=(0.0, 90.0),
        )
        servo = Servo(config=config)

        assert servo.name == "ConfigServo"
        assert servo.channel == 2
        assert servo.angle_range == (0.0, 90.0)

    def test_init_with_driver(self) -> None:
        """Test initialization with driver."""
        driver = SimulatedDriver(name="test", channels=16)
        servo = Servo(name="TestServo", driver=driver, channel=3)

        assert servo.driver is driver
        assert servo.channel == 3


class TestServoAnglePulseConversion:
    """Tests for angle to pulse conversion."""

    def test_angle_to_pulse_min(self) -> None:
        """Test conversion at minimum angle."""
        servo = SimulatedServo(angle_range=(0.0, 180.0), pulse_range=(500, 2500))

        pulse = servo.angle_to_pulse(0.0)
        assert pulse == 500

    def test_angle_to_pulse_max(self) -> None:
        """Test conversion at maximum angle."""
        servo = SimulatedServo(angle_range=(0.0, 180.0), pulse_range=(500, 2500))

        pulse = servo.angle_to_pulse(180.0)
        assert pulse == 2500

    def test_angle_to_pulse_center(self) -> None:
        """Test conversion at center angle."""
        servo = SimulatedServo(angle_range=(0.0, 180.0), pulse_range=(500, 2500))

        pulse = servo.angle_to_pulse(90.0)
        assert pulse == 1500

    def test_pulse_to_angle_min(self) -> None:
        """Test conversion at minimum pulse."""
        servo = SimulatedServo(angle_range=(0.0, 180.0), pulse_range=(500, 2500))

        angle = servo.pulse_to_angle(500)
        assert angle == 0.0

    def test_pulse_to_angle_max(self) -> None:
        """Test conversion at maximum pulse."""
        servo = SimulatedServo(angle_range=(0.0, 180.0), pulse_range=(500, 2500))

        angle = servo.pulse_to_angle(2500)
        assert angle == 180.0

    def test_pulse_to_angle_center(self) -> None:
        """Test conversion at center pulse."""
        servo = SimulatedServo(angle_range=(0.0, 180.0), pulse_range=(500, 2500))

        angle = servo.pulse_to_angle(1500)
        assert angle == 90.0

    def test_conversion_with_trim(self) -> None:
        """Test conversion with trim adjustment."""
        servo = SimulatedServo()
        servo.trim = 50

        # Trim should be added to pulse
        pulse = servo.angle_to_pulse(90.0)
        assert pulse == 1550  # 1500 + 50


class TestServoOperations:
    """Tests for servo operations."""

    def test_set_angle_when_enabled(self) -> None:
        """Test setting angle when enabled."""
        servo = SimulatedServo()
        servo.enable()

        servo.set(90.0)

        assert servo.angle == 90.0
        assert servo.state == ActuatorState.HOLDING

    def test_set_angle_when_disabled_raises(self) -> None:
        """Test that setting angle when disabled raises error."""
        servo = SimulatedServo()

        with pytest.raises(DisabledError):
            servo.set(90.0)

    def test_set_angle_out_of_range_raises(self) -> None:
        """Test that out of range angle raises error."""
        servo = SimulatedServo(angle_range=(0.0, 180.0))
        servo.enable()

        with pytest.raises(LimitsExceededError):
            servo.set(200.0)

    def test_set_angle_method(self) -> None:
        """Test set_angle convenience method."""
        servo = SimulatedServo()
        servo.enable()

        servo.set_angle(45.0)

        assert servo.angle == 45.0

    def test_set_pulse(self) -> None:
        """Test set_pulse method."""
        servo = SimulatedServo(pulse_range=(500, 2500))
        servo.enable()

        servo.set_pulse(1000)

        assert servo.pulse_width == 1000
        # Angle should be ~45 degrees (quarter of range)
        assert 44.0 < servo.angle < 46.0

    def test_set_pulse_out_of_range_raises(self) -> None:
        """Test that out of range pulse raises error."""
        servo = SimulatedServo(pulse_range=(500, 2500))
        servo.enable()

        with pytest.raises(LimitsExceededError):
            servo.set_pulse(3000)

    def test_center(self) -> None:
        """Test center method."""
        servo = SimulatedServo(angle_range=(0.0, 180.0))
        servo.enable()

        servo.center()

        assert servo.angle == 90.0

    def test_go_to_default(self) -> None:
        """Test go_to_default method."""
        servo = SimulatedServo(angle_range=(0.0, 180.0))
        servo.enable()
        servo.set(30.0)  # Move somewhere

        servo.go_to_default()

        # Default should be center
        assert servo.angle == 90.0

    def test_go_to_min(self) -> None:
        """Test go_to_min method."""
        servo = SimulatedServo(angle_range=(10.0, 170.0))
        servo.enable()

        servo.go_to_min()

        assert servo.angle == 10.0

    def test_go_to_max(self) -> None:
        """Test go_to_max method."""
        servo = SimulatedServo(angle_range=(10.0, 170.0))
        servo.enable()

        servo.go_to_max()

        assert servo.angle == 170.0


class TestServoSweep:
    """Tests for servo sweep operations."""

    def test_sweep_default(self) -> None:
        """Test basic sweep."""
        servo = SimulatedServo(angle_range=(0.0, 180.0))
        servo.enable()
        servo.set(0.0)

        # Sweep with minimal delay for testing
        servo.sweep(start=0.0, end=10.0, step=5.0, delay=0.001)

        # Should end at 10 degrees
        assert servo.angle == 10.0

    def test_sweep_reverse(self) -> None:
        """Test reverse sweep."""
        servo = SimulatedServo(angle_range=(0.0, 180.0))
        servo.enable()
        servo.set(10.0)

        servo.sweep(start=10.0, end=0.0, step=5.0, delay=0.001)

        assert servo.angle == 0.0

    def test_sweep_when_disabled_raises(self) -> None:
        """Test that sweep when disabled raises error."""
        servo = SimulatedServo()

        with pytest.raises(DisabledError):
            servo.sweep()

    @pytest.mark.asyncio
    async def test_sweep_async(self) -> None:
        """Test async sweep."""
        servo = SimulatedServo(angle_range=(0.0, 180.0))
        servo.enable()
        servo.set(0.0)

        angles: list[float] = []
        async for angle in servo.sweep_async(start=0.0, end=10.0, step=5.0, delay=0.001):
            angles.append(angle)

        assert len(angles) == 3  # 0, 5, 10
        assert angles[-1] == 10.0


class TestServoCalibration:
    """Tests for servo calibration."""

    def test_calibrate_pulse_range(self) -> None:
        """Test calibrating pulse range."""
        servo = SimulatedServo()

        servo.calibrate(min_pulse=600, max_pulse=2400)

        assert servo.pulse_range == (600, 2400)
        assert servo.is_calibrated is True

    def test_calibrate_min_only(self) -> None:
        """Test calibrating only min pulse."""
        servo = SimulatedServo(pulse_range=(500, 2500))

        servo.calibrate(min_pulse=600)

        assert servo.pulse_range == (600, 2500)

    def test_calibrate_max_only(self) -> None:
        """Test calibrating only max pulse."""
        servo = SimulatedServo(pulse_range=(500, 2500))

        servo.calibrate(max_pulse=2400)

        assert servo.pulse_range == (500, 2400)

    def test_calibrate_invalid_raises(self) -> None:
        """Test that invalid calibration raises error."""
        servo = SimulatedServo()

        with pytest.raises(CalibrationError):
            servo.calibrate(min_pulse=2500, max_pulse=500)  # Min > Max


class TestServoStatusMethods:
    """Tests for servo status methods."""

    def test_servo_status(self) -> None:
        """Test servo_status method."""
        servo = SimulatedServo()
        servo.enable()
        servo.set(90.0)

        status = servo.servo_status()

        assert status.state == ActuatorState.HOLDING
        assert status.angle == 90.0
        assert status.is_enabled is True
        assert status.is_calibrated is True

    def test_status(self) -> None:
        """Test base status method."""
        servo = SimulatedServo()
        servo.enable()
        servo.set(45.0)

        status = servo.status()

        assert status.state == ActuatorState.HOLDING
        assert status.position == 45.0
        assert status.is_enabled is True


class TestServoContextManager:
    """Tests for servo context manager."""

    def test_context_manager(self) -> None:
        """Test context manager enables and disables."""
        servo = SimulatedServo()

        with servo:
            assert servo.is_enabled is True
            servo.set(90.0)

        assert servo.is_enabled is False


class TestServoWithDriver:
    """Tests for servo with a driver."""

    def test_servo_with_simulated_driver(self) -> None:
        """Test servo with SimulatedDriver."""
        driver = SimulatedDriver(name="test", channels=16)
        driver.connect()
        driver.enable()

        servo = Servo(
            name="TestServo",
            driver=driver,
            channel=0,
            angle_range=(0.0, 180.0),
        )
        servo.enable()

        servo.set(90.0)

        # Check driver received the command
        assert 0.49 < driver.get_channel(0) < 0.51  # Should be ~0.5 (center)

    def test_servo_disable_stops_pwm(self) -> None:
        """Test that disabling servo stops PWM."""
        driver = SimulatedDriver(name="test", channels=16)
        driver.connect()
        driver.enable()

        servo = Servo(
            name="TestServo",
            driver=driver,
            channel=0,
        )
        servo.enable()
        servo.set(90.0)
        servo.disable()

        # Channel should be 0 after disable
        assert driver.get_channel(0) == 0.0


# =============================================================================
# SimulatedServo Tests
# =============================================================================


class TestSimulatedServo:
    """Tests for SimulatedServo."""

    def test_simulated_default_values(self) -> None:
        """Test simulated servo default values."""
        servo = SimulatedServo()

        # Should start at center
        assert servo.angle == 90.0

    def test_simulated_initial_angle(self) -> None:
        """Test simulated servo with initial angle."""
        servo = SimulatedServo(initial_angle=45.0)

        assert servo.angle == 45.0

    def test_simulated_works_without_driver(self) -> None:
        """Test that simulated servo works without a driver."""
        servo = SimulatedServo()
        servo.enable()

        servo.set(90.0)
        assert servo.angle == 90.0

        servo.set(0.0)
        assert servo.angle == 0.0


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreateServo:
    """Tests for create_servo factory function."""

    def test_create_simulated_explicit(self) -> None:
        """Test creating simulated servo explicitly."""
        servo = create_servo(name="Test", simulated=True)

        assert isinstance(servo, SimulatedServo)
        assert servo.name == "Test"

    def test_create_simulated_no_driver(self) -> None:
        """Test creating simulated servo when no driver provided."""
        servo = create_servo(name="Test")

        assert isinstance(servo, SimulatedServo)

    def test_create_with_driver(self) -> None:
        """Test creating servo with driver."""
        driver = SimulatedDriver(name="test", channels=16)
        servo = create_servo(name="Test", driver=driver, channel=0)

        assert isinstance(servo, Servo)
        assert not isinstance(servo, SimulatedServo)
        assert servo.driver is driver

    def test_create_with_custom_range(self) -> None:
        """Test creating servo with custom angle range."""
        servo = create_servo(
            name="Test",
            angle_range=(0.0, 270.0),
            pulse_range=(600, 2400),
        )

        assert servo.angle_range == (0.0, 270.0)
        assert servo.pulse_range == (600, 2400)


class TestServoEdgeCases:
    """Tests for edge cases."""

    def test_zero_angle_span(self) -> None:
        """Test servo with zero angle span."""
        servo = SimulatedServo(angle_range=(90.0, 90.0))
        servo.enable()

        servo.set(90.0)
        assert servo.angle == 90.0

    def test_negative_angle_range(self) -> None:
        """Test servo with negative angles."""
        servo = SimulatedServo(angle_range=(-90.0, 90.0))
        servo.enable()

        servo.set(-45.0)
        assert servo.angle == -45.0

        servo.set(45.0)
        assert servo.angle == 45.0

    def test_repr(self) -> None:
        """Test string representation."""
        servo = SimulatedServo(name="TestServo")
        servo.enable()
        servo.set(45.0)

        repr_str = repr(servo)

        assert "TestServo" in repr_str
        assert "45.0" in repr_str
