"""Tests for PCA9685 16-channel PWM driver.

Tests cover:
- Initialization and configuration
- Lifecycle (connect/disconnect)
- PWM channel operations (set_pwm, get_pwm)
- Normalized channel operations (set_channel, get_channel)
- Frequency control
- Sleep/wake modes
- All-call and configuration options
- Simulation mode
- I2C communication with mock bus
"""

from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.driver import DriverState, get_driver, register_driver
from robo_infra.drivers.pca9685 import (
    PCA9685,
    PCA9685_DEFAULT_ADDRESS,
    PCA9685_MAX_FREQUENCY,
    PCA9685_MAX_PWM,
    PCA9685_MIN_FREQUENCY,
    PCA9685_RESOLUTION,
    PCA9685Config,
    PCA9685Mode1,
    PCA9685Mode2,
    PCA9685Register,
)


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus


# =============================================================================
# Test Fixtures
# =============================================================================


@pytest.fixture
def mock_bus() -> MagicMock:
    """Create a mock I2C bus."""
    bus = MagicMock()
    bus.is_open = True
    bus.scan.return_value = [0x40]  # PCA9685 found

    # Default register values
    registers: dict[int, int] = {}

    def read_register_byte(address: int, register: int) -> int:
        return registers.get(register, 0)

    def write_register_byte(address: int, register: int, value: int) -> None:
        registers[register] = value

    def read_register(address: int, register: int, length: int) -> bytes:
        return bytes([registers.get(register + i, 0) for i in range(length)])

    def write_register(
        address: int, register: int, data: bytes | list[int]
    ) -> int:
        data_bytes = bytes(data) if isinstance(data, list) else data
        for i, byte in enumerate(data_bytes):
            registers[register + i] = byte
        return len(data_bytes)

    bus.read_register_byte = MagicMock(side_effect=read_register_byte)
    bus.write_register_byte = MagicMock(side_effect=write_register_byte)
    bus.read_register = MagicMock(side_effect=read_register)
    bus.write_register = MagicMock(side_effect=write_register)

    return bus


@pytest.fixture
def driver_simulation() -> PCA9685:
    """Create a PCA9685 in simulation mode (no bus)."""
    driver = PCA9685()
    driver.connect()
    return driver


@pytest.fixture
def driver_with_bus(mock_bus: MagicMock) -> PCA9685:
    """Create a PCA9685 with mock I2C bus."""
    driver = PCA9685(bus=mock_bus)
    driver.connect()
    return driver


# =============================================================================
# Test Register Definitions
# =============================================================================


class TestPCA9685Registers:
    """Tests for PCA9685 register definitions."""

    def test_mode_registers(self) -> None:
        """Test MODE register addresses."""
        assert PCA9685Register.MODE1 == 0x00
        assert PCA9685Register.MODE2 == 0x01

    def test_led_registers(self) -> None:
        """Test LED register addresses."""
        assert PCA9685Register.LED0_ON_L == 0x06
        assert PCA9685Register.LED0_ON_H == 0x07
        assert PCA9685Register.LED0_OFF_L == 0x08
        assert PCA9685Register.LED0_OFF_H == 0x09
        assert PCA9685Register.LED15_OFF_H == 0x45

    def test_all_led_registers(self) -> None:
        """Test ALL_LED register addresses."""
        assert PCA9685Register.ALL_LED_ON_L == 0xFA
        assert PCA9685Register.ALL_LED_OFF_H == 0xFD

    def test_prescale_register(self) -> None:
        """Test prescale register address."""
        assert PCA9685Register.PRE_SCALE == 0xFE


class TestPCA9685ModeBits:
    """Tests for MODE register bit definitions."""

    def test_mode1_bits(self) -> None:
        """Test MODE1 bit definitions."""
        assert PCA9685Mode1.ALLCALL == 0x01
        assert PCA9685Mode1.SLEEP == 0x10
        assert PCA9685Mode1.AI == 0x20
        assert PCA9685Mode1.RESTART == 0x80

    def test_mode2_bits(self) -> None:
        """Test MODE2 bit definitions."""
        assert PCA9685Mode2.OUTDRV == 0x04
        assert PCA9685Mode2.INVRT == 0x10


# =============================================================================
# Test PCA9685Config
# =============================================================================


class TestPCA9685Config:
    """Tests for PCA9685Config dataclass."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = PCA9685Config()

        assert config.address == PCA9685_DEFAULT_ADDRESS
        assert config.frequency == 50
        assert config.auto_increment is True
        assert config.invert is False
        assert config.totem_pole is True
        assert config.all_call is True

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = PCA9685Config(
            address=0x41,
            frequency=100,
            auto_increment=False,
            invert=True,
            totem_pole=False,
            all_call=False,
        )

        assert config.address == 0x41
        assert config.frequency == 100
        assert config.auto_increment is False
        assert config.invert is True
        assert config.totem_pole is False
        assert config.all_call is False


# =============================================================================
# Test Initialization
# =============================================================================


class TestPCA9685Init:
    """Tests for PCA9685 initialization."""

    def test_default_initialization(self) -> None:
        """Test default initialization (simulation mode)."""
        driver = PCA9685()

        assert driver.address == PCA9685_DEFAULT_ADDRESS
        assert driver.channels == 16
        assert driver.simulation_mode is True
        assert driver.state == DriverState.DISCONNECTED

    def test_custom_address(self) -> None:
        """Test initialization with custom address."""
        driver = PCA9685(address=0x41)

        assert driver.address == 0x41

    def test_with_config(self) -> None:
        """Test initialization with config."""
        config = PCA9685Config(address=0x42, frequency=100)
        driver = PCA9685(config=config)

        assert driver.address == 0x42

    def test_with_bus(self, mock_bus: MagicMock) -> None:
        """Test initialization with I2C bus."""
        driver = PCA9685(bus=mock_bus)

        assert driver.simulation_mode is False
        assert driver.address == PCA9685_DEFAULT_ADDRESS

    def test_repr(self, driver_simulation: PCA9685) -> None:
        """Test string representation."""
        repr_str = repr(driver_simulation)

        assert "PCA9685" in repr_str
        assert "0x40" in repr_str
        assert "simulation" in repr_str


# =============================================================================
# Test Lifecycle
# =============================================================================


class TestPCA9685Lifecycle:
    """Tests for driver lifecycle methods."""

    def test_connect_simulation(self) -> None:
        """Test connecting in simulation mode."""
        driver = PCA9685()
        assert driver.state == DriverState.DISCONNECTED

        driver.connect()

        assert driver.state == DriverState.CONNECTED
        assert driver.is_sleeping is False

    def test_disconnect_simulation(self, driver_simulation: PCA9685) -> None:
        """Test disconnecting in simulation mode."""
        assert driver_simulation.state == DriverState.CONNECTED

        driver_simulation.disconnect()

        assert driver_simulation.state == DriverState.DISCONNECTED

    def test_connect_with_bus(self, mock_bus: MagicMock) -> None:
        """Test connecting with I2C bus."""
        driver = PCA9685(bus=mock_bus)
        driver.connect()

        assert driver.state == DriverState.CONNECTED
        assert mock_bus.scan.called

    def test_connect_device_not_found(self, mock_bus: MagicMock) -> None:
        """Test connecting when device is not found."""
        mock_bus.scan.return_value = []  # No devices found

        driver = PCA9685(bus=mock_bus)

        with pytest.raises(Exception, match="not found"):
            driver.connect()

    def test_context_manager(self) -> None:
        """Test context manager usage."""
        with PCA9685() as driver:
            assert driver.state == DriverState.CONNECTED
            driver.set_channel(0, 0.5)

        assert driver.state == DriverState.DISCONNECTED


# =============================================================================
# Test PWM Channel Operations
# =============================================================================


class TestPCA9685PWMOperations:
    """Tests for raw PWM operations."""

    def test_set_pwm_simulation(self, driver_simulation: PCA9685) -> None:
        """Test setting PWM in simulation mode."""
        driver_simulation.set_pwm(0, on=0, off=2048)

        # Check duty cycle is approximately 50%
        assert abs(driver_simulation.get_channel(0) - 0.5) < 0.01

    def test_get_pwm_simulation(self, driver_simulation: PCA9685) -> None:
        """Test getting PWM in simulation mode."""
        driver_simulation.set_pwm(0, on=0, off=1024)

        on, off = driver_simulation.get_pwm(0)

        assert on == 0
        assert off == 1024

    def test_set_pwm_full_on(self, driver_simulation: PCA9685) -> None:
        """Test setting PWM to fully on."""
        driver_simulation.set_pwm(0, on=PCA9685_RESOLUTION, off=0)

        on, off = driver_simulation.get_pwm(0)
        assert on == PCA9685_RESOLUTION
        assert off == 0
        assert driver_simulation.get_channel(0) == 1.0

    def test_set_pwm_full_off(self, driver_simulation: PCA9685) -> None:
        """Test setting PWM to fully off."""
        driver_simulation.set_pwm(0, on=0, off=PCA9685_RESOLUTION)

        on, off = driver_simulation.get_pwm(0)
        assert on == 0
        assert off == PCA9685_RESOLUTION
        assert driver_simulation.get_channel(0) == 0.0

    def test_set_pwm_clamping(self, driver_simulation: PCA9685) -> None:
        """Test that PWM values are clamped."""
        driver_simulation.set_pwm(0, on=5000, off=5000)

        on, off = driver_simulation.get_pwm(0)
        assert on <= PCA9685_RESOLUTION
        assert off <= PCA9685_RESOLUTION

    def test_set_pwm_with_bus(
        self, driver_with_bus: PCA9685, mock_bus: MagicMock
    ) -> None:
        """Test setting PWM with I2C bus."""
        driver_with_bus.set_pwm(0, on=0, off=2048)

        # Verify write_register was called with correct register
        expected_reg = PCA9685Register.LED0_ON_L
        mock_bus.write_register.assert_called()
        call_args = mock_bus.write_register.call_args
        assert call_args[0][1] == expected_reg  # Register address

    def test_set_all_pwm(self, driver_simulation: PCA9685) -> None:
        """Test setting all PWM channels at once."""
        driver_simulation.set_all_pwm(on=0, off=2048)

        for ch in range(16):
            assert abs(driver_simulation.get_channel(ch) - 0.5) < 0.01

    def test_channel_validation(self, driver_simulation: PCA9685) -> None:
        """Test channel validation."""
        with pytest.raises(ValueError, match="out of range"):
            driver_simulation.set_pwm(16, on=0, off=1000)

        with pytest.raises(ValueError, match="out of range"):
            driver_simulation.get_pwm(-1)


# =============================================================================
# Test Normalized Channel Operations
# =============================================================================


class TestPCA9685NormalizedOperations:
    """Tests for normalized (0-1) channel operations."""

    def test_set_channel_zero(self, driver_simulation: PCA9685) -> None:
        """Test setting channel to 0 (off)."""
        driver_simulation.set_channel(0, 0.0)

        assert driver_simulation.get_channel(0) == 0.0

    def test_set_channel_one(self, driver_simulation: PCA9685) -> None:
        """Test setting channel to 1 (full on)."""
        driver_simulation.set_channel(0, 1.0)

        assert driver_simulation.get_channel(0) == 1.0

    def test_set_channel_half(self, driver_simulation: PCA9685) -> None:
        """Test setting channel to 0.5 (half)."""
        driver_simulation.set_channel(0, 0.5)

        value = driver_simulation.get_channel(0)
        assert abs(value - 0.5) < 0.01

    def test_set_channel_various(self, driver_simulation: PCA9685) -> None:
        """Test various channel values."""
        test_values = [0.0, 0.25, 0.5, 0.75, 1.0]

        for i, val in enumerate(test_values):
            driver_simulation.set_channel(i, val)

        for i, expected in enumerate(test_values):
            actual = driver_simulation.get_channel(i)
            assert abs(actual - expected) < 0.01

    def test_set_all_channels(self, driver_simulation: PCA9685) -> None:
        """Test setting all channels to same value."""
        driver_simulation.set_all_channels(0.75)

        for ch in range(16):
            value = driver_simulation.get_channel(ch)
            assert abs(value - 0.75) < 0.01

    def test_get_all_channels(self, driver_simulation: PCA9685) -> None:
        """Test getting all channel values."""
        for ch in range(16):
            driver_simulation.set_channel(ch, ch / 15.0)

        all_values = driver_simulation.get_all_channels()

        assert len(all_values) == 16
        for ch in range(16):
            expected = ch / 15.0
            assert abs(all_values[ch] - expected) < 0.01


# =============================================================================
# Test Frequency Control
# =============================================================================


class TestPCA9685Frequency:
    """Tests for frequency control."""

    def test_default_frequency(self, driver_simulation: PCA9685) -> None:
        """Test default frequency is 50Hz (for servos)."""
        assert driver_simulation.current_frequency == 50

    def test_set_frequency_simulation(self, driver_simulation: PCA9685) -> None:
        """Test setting frequency in simulation mode."""
        driver_simulation.set_frequency(100)

        # Frequency won't be exact due to prescaler
        assert driver_simulation.current_frequency > 0

    def test_set_frequency_min(self, driver_simulation: PCA9685) -> None:
        """Test setting minimum frequency."""
        driver_simulation.set_frequency(PCA9685_MIN_FREQUENCY)

        assert driver_simulation.current_frequency >= PCA9685_MIN_FREQUENCY - 5

    def test_set_frequency_max(self, driver_simulation: PCA9685) -> None:
        """Test setting maximum frequency."""
        driver_simulation.set_frequency(PCA9685_MAX_FREQUENCY)

        assert driver_simulation.current_frequency <= PCA9685_MAX_FREQUENCY + 50

    def test_set_frequency_out_of_range_low(
        self, driver_simulation: PCA9685
    ) -> None:
        """Test setting frequency below minimum."""
        with pytest.raises(ValueError, match="Frequency must be"):
            driver_simulation.set_frequency(10)

    def test_set_frequency_out_of_range_high(
        self, driver_simulation: PCA9685
    ) -> None:
        """Test setting frequency above maximum."""
        with pytest.raises(ValueError, match="Frequency must be"):
            driver_simulation.set_frequency(2000)

    def test_set_frequency_with_bus(
        self, driver_with_bus: PCA9685, mock_bus: MagicMock
    ) -> None:
        """Test setting frequency with I2C bus."""
        initial_calls = mock_bus.write_register_byte.call_count

        driver_with_bus.set_frequency(100)

        # Should have written to PRE_SCALE register
        assert mock_bus.write_register_byte.call_count > initial_calls


# =============================================================================
# Test Sleep/Wake
# =============================================================================


class TestPCA9685SleepWake:
    """Tests for sleep/wake functionality."""

    def test_sleep_simulation(self, driver_simulation: PCA9685) -> None:
        """Test sleep in simulation mode."""
        assert driver_simulation.is_sleeping is False

        driver_simulation.sleep()

        assert driver_simulation.is_sleeping is True

    def test_wake_simulation(self, driver_simulation: PCA9685) -> None:
        """Test wake in simulation mode."""
        driver_simulation.sleep()
        assert driver_simulation.is_sleeping is True

        driver_simulation.wake()

        assert driver_simulation.is_sleeping is False

    def test_sleep_wake_cycle(self, driver_simulation: PCA9685) -> None:
        """Test sleep/wake cycle."""
        driver_simulation.set_channel(0, 0.5)

        driver_simulation.sleep()
        assert driver_simulation.is_sleeping is True

        driver_simulation.wake()
        assert driver_simulation.is_sleeping is False

        # Channel value should be preserved
        assert abs(driver_simulation.get_channel(0) - 0.5) < 0.01

    def test_sleep_with_bus(
        self, driver_with_bus: PCA9685, mock_bus: MagicMock
    ) -> None:
        """Test sleep with I2C bus."""
        driver_with_bus.sleep()

        assert driver_with_bus.is_sleeping is True
        # Verify MODE1 register was written
        mock_bus.write_register_byte.assert_called()


# =============================================================================
# Test Configuration Methods
# =============================================================================


class TestPCA9685Configuration:
    """Tests for configuration methods."""

    def test_set_all_call_enabled(self, driver_simulation: PCA9685) -> None:
        """Test enabling all-call address."""
        # Should not raise in simulation
        driver_simulation.set_all_call_enabled(True)
        driver_simulation.set_all_call_enabled(False)

    def test_set_output_inverted(self, driver_simulation: PCA9685) -> None:
        """Test setting output inversion."""
        driver_simulation.set_output_inverted(True)
        driver_simulation.set_output_inverted(False)

    def test_set_totem_pole(self, driver_simulation: PCA9685) -> None:
        """Test setting output driver type."""
        driver_simulation.set_totem_pole(True)
        driver_simulation.set_totem_pole(False)

    def test_configuration_with_bus(
        self, driver_with_bus: PCA9685, mock_bus: MagicMock
    ) -> None:
        """Test configuration methods write to bus."""
        initial_calls = mock_bus.write_register_byte.call_count

        driver_with_bus.set_all_call_enabled(True)
        driver_with_bus.set_output_inverted(True)
        driver_with_bus.set_totem_pole(False)

        assert mock_bus.write_register_byte.call_count > initial_calls


# =============================================================================
# Test Driver Registration
# =============================================================================


class TestPCA9685Registration:
    """Tests for driver registration."""

    def test_pca9685_registered(self) -> None:
        """Test that PCA9685 is registered."""
        # Re-register since test_core_driver.py may clear the registry
        register_driver("pca9685")(PCA9685)

        driver_cls = get_driver("pca9685")
        assert driver_cls is PCA9685

    def test_create_from_registry(self) -> None:
        """Test creating driver from registry."""
        register_driver("pca9685")(PCA9685)

        driver_cls = get_driver("pca9685")
        driver = driver_cls(address=0x41)

        assert driver.address == 0x41
        assert driver.channels == 16


# =============================================================================
# Test Integration with Actuators
# =============================================================================


class TestPCA9685Integration:
    """Tests for integration with actuators."""

    def test_multiple_channel_control(self, driver_simulation: PCA9685) -> None:
        """Test controlling multiple channels for servo-like usage."""
        # Simulate controlling 4 servos
        servo_channels = [0, 1, 2, 3]
        positions = [0.25, 0.5, 0.75, 1.0]

        for ch, pos in zip(servo_channels, positions):
            driver_simulation.set_channel(ch, pos)

        for ch, expected in zip(servo_channels, positions):
            actual = driver_simulation.get_channel(ch)
            assert abs(actual - expected) < 0.01

    def test_led_dimming(self, driver_simulation: PCA9685) -> None:
        """Test LED dimming pattern."""
        # Fade up
        for brightness in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
            driver_simulation.set_channel(0, brightness)
            assert abs(driver_simulation.get_channel(0) - brightness) < 0.01

        # Fade down
        for brightness in [1.0, 0.8, 0.6, 0.4, 0.2, 0.0]:
            driver_simulation.set_channel(0, brightness)
            assert abs(driver_simulation.get_channel(0) - brightness) < 0.01

    def test_high_frequency_operation(self, driver_simulation: PCA9685) -> None:
        """Test operation at high frequency (for LEDs)."""
        driver_simulation.set_frequency(1000)  # 1kHz for LEDs

        driver_simulation.set_channel(0, 0.5)
        assert abs(driver_simulation.get_channel(0) - 0.5) < 0.01


# =============================================================================
# Test Edge Cases
# =============================================================================


class TestPCA9685EdgeCases:
    """Tests for edge cases."""

    def test_set_channel_negative(self, driver_simulation: PCA9685) -> None:
        """Test setting negative value (should clamp to 0)."""
        driver_simulation.set_channel(0, -0.5)
        assert driver_simulation.get_channel(0) == 0.0

    def test_set_channel_over_one(self, driver_simulation: PCA9685) -> None:
        """Test setting value over 1 (should clamp to 1)."""
        driver_simulation.set_channel(0, 1.5)
        assert driver_simulation.get_channel(0) == 1.0

    def test_operations_while_sleeping(self, driver_simulation: PCA9685) -> None:
        """Test that PWM operations work while device is sleeping in simulation."""
        driver_simulation.sleep()

        # In simulation mode, this should still work (just stores values)
        driver_simulation.set_channel(0, 0.5)
        assert abs(driver_simulation.get_channel(0) - 0.5) < 0.01

    def test_rapid_channel_changes(self, driver_simulation: PCA9685) -> None:
        """Test rapid channel value changes."""
        for _ in range(100):
            for ch in range(16):
                driver_simulation.set_channel(ch, 0.5)
                driver_simulation.set_channel(ch, 0.0)

        # All channels should be at 0
        for ch in range(16):
            assert driver_simulation.get_channel(ch) == 0.0
