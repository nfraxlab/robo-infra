"""Unit tests for power.drivers module.

Tests for INA219, INA226, and simulated power monitor drivers.
"""

from __future__ import annotations

import struct
from unittest.mock import MagicMock

import pytest

from robo_infra.power.drivers import (
    INA219_DEFAULT_ADDRESS,
    INA226_DEFAULT_ADDRESS,
    INA219ADCResolution,
    INA219BusVoltageRange,
    INA219Config,
    INA219Driver,
    INA219Gain,
    INA226AveragingMode,
    INA226Config,
    INA226ConversionTime,
    INA226Driver,
    PowerReading,
    SimulatedPowerMonitor,
)


# =============================================================================
# PowerReading Tests
# =============================================================================


class TestPowerReading:
    """Tests for PowerReading dataclass."""

    def test_basic_reading(self) -> None:
        """Test basic reading creation."""
        reading = PowerReading(
            voltage=12.0,
            current=1.5,
            power=18.0,
            shunt_voltage=15.0,  # 15mV
            timestamp=1234567890.0,
        )
        assert reading.voltage == 12.0
        assert reading.current == 1.5
        assert reading.power == 18.0
        assert reading.shunt_voltage == 15.0
        assert reading.timestamp == 1234567890.0


# =============================================================================
# INA219 Configuration Tests
# =============================================================================


class TestINA219Config:
    """Tests for INA219Config."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = INA219Config()
        assert config.address == INA219_DEFAULT_ADDRESS
        assert config.shunt_ohms == 0.1
        assert config.max_expected_current == 3.2
        assert config.bus_voltage_range == INA219BusVoltageRange.RANGE_32V
        assert config.gain == INA219Gain.GAIN_8_320MV
        assert config.bus_adc_resolution == INA219ADCResolution.ADC_12BIT_1S
        assert config.shunt_adc_resolution == INA219ADCResolution.ADC_12BIT_1S

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = INA219Config(
            address=0x41,
            shunt_ohms=0.05,
            max_expected_current=5.0,
            bus_voltage_range=INA219BusVoltageRange.RANGE_16V,
            gain=INA219Gain.GAIN_4_160MV,
        )
        assert config.address == 0x41
        assert config.shunt_ohms == 0.05
        assert config.max_expected_current == 5.0
        assert config.bus_voltage_range == INA219BusVoltageRange.RANGE_16V
        assert config.gain == INA219Gain.GAIN_4_160MV

    def test_address_validation(self) -> None:
        """Test I2C address validation."""
        INA219Config(address=0x00)
        INA219Config(address=0x7F)

        # Should fail with out of range
        with pytest.raises(ValueError):
            INA219Config(address=0x80)

    def test_shunt_validation(self) -> None:
        """Test shunt resistor validation."""
        INA219Config(shunt_ohms=0.001)
        INA219Config(shunt_ohms=1.0)

        with pytest.raises(ValueError):
            INA219Config(shunt_ohms=0)

        with pytest.raises(ValueError):
            INA219Config(shunt_ohms=-0.1)


# =============================================================================
# INA219 Enum Tests
# =============================================================================


class TestINA219Enums:
    """Tests for INA219 enums."""

    def test_bus_voltage_range(self) -> None:
        """Test bus voltage range enum."""
        assert INA219BusVoltageRange.RANGE_16V == 0
        assert INA219BusVoltageRange.RANGE_32V == 1

    def test_gain(self) -> None:
        """Test PGA gain enum."""
        assert INA219Gain.GAIN_1_40MV == 0
        assert INA219Gain.GAIN_2_80MV == 1
        assert INA219Gain.GAIN_4_160MV == 2
        assert INA219Gain.GAIN_8_320MV == 3

    def test_adc_resolution(self) -> None:
        """Test ADC resolution enum."""
        assert INA219ADCResolution.ADC_9BIT_1S == 0
        assert INA219ADCResolution.ADC_12BIT_1S == 3
        assert INA219ADCResolution.ADC_12BIT_128S == 15


# =============================================================================
# INA219 Driver Tests
# =============================================================================


class TestINA219Driver:
    """Tests for INA219Driver."""

    def test_initialization_default(self) -> None:
        """Test default initialization."""
        driver = INA219Driver()
        assert driver.config.address == 0x40
        assert driver.config.shunt_ohms == 0.1
        assert driver.is_enabled() is False

    def test_initialization_with_params(self) -> None:
        """Test initialization with parameters."""
        driver = INA219Driver(address=0x41, shunt_ohms=0.05)
        assert driver.config.address == 0x41
        assert driver.config.shunt_ohms == 0.05

    def test_initialization_with_config(self) -> None:
        """Test initialization with config object."""
        config = INA219Config(address=0x42, shunt_ohms=0.02)
        driver = INA219Driver(config=config)
        assert driver.config.address == 0x42
        assert driver.config.shunt_ohms == 0.02

    def test_enable_disable(self) -> None:
        """Test enable/disable cycle."""
        driver = INA219Driver()

        assert driver.is_enabled() is False
        driver.enable()
        assert driver.is_enabled() is True
        driver.disable()
        assert driver.is_enabled() is False

    def test_enable_idempotent(self) -> None:
        """Test that enable is idempotent."""
        driver = INA219Driver()
        driver.enable()
        driver.enable()  # Should not raise
        assert driver.is_enabled() is True

    def test_read_requires_enable(self) -> None:
        """Test that reading requires enable."""
        driver = INA219Driver()

        with pytest.raises(RuntimeError, match="not enabled"):
            driver.read_voltage()

        with pytest.raises(RuntimeError, match="not enabled"):
            driver.read_current()

        with pytest.raises(RuntimeError, match="not enabled"):
            driver.read_power()

    def test_read_voltage_simulated(self) -> None:
        """Test reading voltage in simulated mode."""
        driver = INA219Driver()
        driver.set_simulated_values(voltage=12.0, current=1.0)
        driver.enable()

        assert driver.read_voltage() == 12.0

    def test_read_current_simulated(self) -> None:
        """Test reading current in simulated mode."""
        driver = INA219Driver()
        driver.set_simulated_values(voltage=12.0, current=1.5)
        driver.enable()

        assert driver.read_current() == 1.5

    def test_read_power_simulated(self) -> None:
        """Test reading power in simulated mode."""
        driver = INA219Driver()
        driver.set_simulated_values(voltage=12.0, current=2.0)
        driver.enable()

        assert driver.read_power() == 24.0  # V * I

    def test_read_shunt_voltage_simulated(self) -> None:
        """Test reading shunt voltage in simulated mode."""
        driver = INA219Driver()  # 0.1 ohm shunt
        driver.set_simulated_values(voltage=12.0, current=1.0)
        driver.enable()

        # Shunt voltage = current * shunt_ohms * 1000 (mV)
        assert driver.read_shunt_voltage() == 100.0  # 1.0A * 0.1Ω = 0.1V = 100mV

    def test_read_all(self) -> None:
        """Test reading all values at once."""
        driver = INA219Driver()
        driver.set_simulated_values(voltage=12.0, current=1.5)
        driver.enable()

        reading = driver.read_all()
        assert isinstance(reading, PowerReading)
        assert reading.voltage == 12.0
        assert reading.current == 1.5
        assert reading.power == 18.0

    def test_calibration_calculation(self) -> None:
        """Test that calibration is calculated correctly."""
        driver = INA219Driver(shunt_ohms=0.1)
        driver.enable()

        # Check that calibration values are set
        assert driver._calibration > 0
        assert driver._current_lsb > 0
        assert driver._power_lsb > 0

    def test_with_mock_bus(self) -> None:
        """Test with mocked I2C bus."""
        mock_bus = MagicMock()
        # Mock read to return valid voltage (12V = 3000 << 3 with 4mV LSB)
        voltage_raw = 3000 << 3
        mock_bus.read.return_value = struct.pack(">H", voltage_raw)

        driver = INA219Driver(bus=mock_bus)
        driver.enable()

        # Verify writes happened (config, calibration)
        assert mock_bus.write.call_count >= 2


# =============================================================================
# INA226 Configuration Tests
# =============================================================================


class TestINA226Config:
    """Tests for INA226Config."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = INA226Config()
        assert config.address == INA226_DEFAULT_ADDRESS
        assert config.shunt_ohms == 0.1
        assert config.max_expected_current == 10.0
        assert config.averaging_mode == INA226AveragingMode.AVG_1
        assert config.bus_conversion_time == INA226ConversionTime.TIME_1100US
        assert config.shunt_conversion_time == INA226ConversionTime.TIME_1100US

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = INA226Config(
            address=0x45,
            shunt_ohms=0.01,
            max_expected_current=20.0,
            averaging_mode=INA226AveragingMode.AVG_16,
        )
        assert config.address == 0x45
        assert config.shunt_ohms == 0.01
        assert config.max_expected_current == 20.0
        assert config.averaging_mode == INA226AveragingMode.AVG_16

    def test_alert_thresholds(self) -> None:
        """Test alert threshold configuration."""
        config = INA226Config(
            over_voltage_limit=30.0,
            under_voltage_limit=10.0,
            over_current_limit=15.0,
        )
        assert config.over_voltage_limit == 30.0
        assert config.under_voltage_limit == 10.0
        assert config.over_current_limit == 15.0


# =============================================================================
# INA226 Enum Tests
# =============================================================================


class TestINA226Enums:
    """Tests for INA226 enums."""

    def test_averaging_mode(self) -> None:
        """Test averaging mode enum."""
        assert INA226AveragingMode.AVG_1 == 0
        assert INA226AveragingMode.AVG_4 == 1
        assert INA226AveragingMode.AVG_16 == 2
        assert INA226AveragingMode.AVG_64 == 3
        assert INA226AveragingMode.AVG_128 == 4
        assert INA226AveragingMode.AVG_256 == 5
        assert INA226AveragingMode.AVG_512 == 6
        assert INA226AveragingMode.AVG_1024 == 7

    def test_conversion_time(self) -> None:
        """Test conversion time enum."""
        assert INA226ConversionTime.TIME_140US == 0
        assert INA226ConversionTime.TIME_1100US == 4
        assert INA226ConversionTime.TIME_8244US == 7


# =============================================================================
# INA226 Driver Tests
# =============================================================================


class TestINA226Driver:
    """Tests for INA226Driver."""

    def test_initialization_default(self) -> None:
        """Test default initialization."""
        driver = INA226Driver()
        assert driver.config.address == 0x40
        assert driver.config.shunt_ohms == 0.01  # Different default from INA219
        assert driver.is_enabled() is False

    def test_initialization_with_params(self) -> None:
        """Test initialization with parameters."""
        driver = INA226Driver(address=0x44, shunt_ohms=0.005)
        assert driver.config.address == 0x44
        assert driver.config.shunt_ohms == 0.005

    def test_initialization_with_config(self) -> None:
        """Test initialization with config object."""
        config = INA226Config(address=0x45, shunt_ohms=0.002)
        driver = INA226Driver(config=config)
        assert driver.config.address == 0x45
        assert driver.config.shunt_ohms == 0.002

    def test_enable_disable(self) -> None:
        """Test enable/disable cycle."""
        driver = INA226Driver()

        assert driver.is_enabled() is False
        driver.enable()
        assert driver.is_enabled() is True
        driver.disable()
        assert driver.is_enabled() is False

    def test_read_requires_enable(self) -> None:
        """Test that reading requires enable."""
        driver = INA226Driver()

        with pytest.raises(RuntimeError, match="not enabled"):
            driver.read_voltage()

    def test_read_voltage_simulated(self) -> None:
        """Test reading voltage in simulated mode."""
        driver = INA226Driver()
        driver.set_simulated_values(voltage=24.0, current=2.0)
        driver.enable()

        assert driver.read_voltage() == 24.0

    def test_read_current_simulated(self) -> None:
        """Test reading current in simulated mode."""
        driver = INA226Driver()
        driver.set_simulated_values(voltage=24.0, current=3.0)
        driver.enable()

        assert driver.read_current() == 3.0

    def test_read_power_simulated(self) -> None:
        """Test reading power in simulated mode."""
        driver = INA226Driver()
        driver.set_simulated_values(voltage=24.0, current=2.5)
        driver.enable()

        assert driver.read_power() == 60.0  # 24V * 2.5A

    def test_read_all(self) -> None:
        """Test reading all values at once."""
        driver = INA226Driver()
        driver.set_simulated_values(voltage=24.0, current=2.0)
        driver.enable()

        reading = driver.read_all()
        assert isinstance(reading, PowerReading)
        assert reading.voltage == 24.0
        assert reading.current == 2.0
        assert reading.power == 48.0

    def test_get_manufacturer_id(self) -> None:
        """Test getting manufacturer ID."""
        driver = INA226Driver()
        driver.enable()

        # Simulated returns TI ID
        assert driver.get_manufacturer_id() == 0x5449

    def test_get_die_id(self) -> None:
        """Test getting die ID."""
        driver = INA226Driver()
        driver.enable()

        # Simulated returns INA226 ID
        assert driver.get_die_id() == 0x2260

    def test_higher_precision_than_ina219(self) -> None:
        """Test that INA226 has different calibration than INA219."""
        # INA226 has 16-bit ADC vs INA219's 12-bit
        # INA226 shunt voltage LSB is 2.5µV vs INA219's 10µV
        ina219 = INA219Driver(shunt_ohms=0.1)
        ina226 = INA226Driver(shunt_ohms=0.1)

        ina219.enable()
        ina226.enable()

        # Calibration formulas are different
        # INA219: cal = 0.04096 / (current_lsb * shunt)
        # INA226: cal = 0.00512 / (current_lsb * shunt)
        # So they should have different calibration values
        assert ina219._calibration != ina226._calibration


# =============================================================================
# SimulatedPowerMonitor Tests
# =============================================================================


class TestSimulatedPowerMonitor:
    """Tests for SimulatedPowerMonitor."""

    def test_initialization(self) -> None:
        """Test initialization."""
        monitor = SimulatedPowerMonitor(voltage=12.0, current=1.0)
        assert monitor.name == "simulated_power_monitor"
        assert monitor.is_enabled() is False

    def test_custom_name(self) -> None:
        """Test custom name."""
        monitor = SimulatedPowerMonitor(name="custom_monitor")
        assert monitor.name == "custom_monitor"

    def test_enable_disable(self) -> None:
        """Test enable/disable."""
        monitor = SimulatedPowerMonitor()

        monitor.enable()
        assert monitor.is_enabled() is True

        monitor.disable()
        assert monitor.is_enabled() is False

    def test_read_requires_enable(self) -> None:
        """Test that read requires enable."""
        monitor = SimulatedPowerMonitor()

        with pytest.raises(RuntimeError, match="not enabled"):
            monitor.read_voltage()

    def test_read_values(self) -> None:
        """Test reading simulated values."""
        monitor = SimulatedPowerMonitor(voltage=15.0, current=2.5)
        monitor.enable()

        assert monitor.read_voltage() == 15.0
        assert monitor.read_current() == 2.5
        assert monitor.read_power() == 37.5

    def test_set_values(self) -> None:
        """Test setting values after creation."""
        monitor = SimulatedPowerMonitor()
        monitor.enable()

        monitor.set_values(voltage=18.0, current=3.0)

        assert monitor.read_voltage() == 18.0
        assert monitor.read_current() == 3.0
        assert monitor.read_power() == 54.0

    def test_implements_interface(self) -> None:
        """Test that SimulatedPowerMonitor implements PowerMonitorDriver."""
        monitor = SimulatedPowerMonitor()

        # Check required methods exist
        assert hasattr(monitor, "enable")
        assert hasattr(monitor, "disable")
        assert hasattr(monitor, "is_enabled")
        assert hasattr(monitor, "read_voltage")
        assert hasattr(monitor, "read_current")
        assert hasattr(monitor, "read_power")


# =============================================================================
# Integration Tests
# =============================================================================


class TestPowerDriverIntegration:
    """Integration tests for power drivers."""

    def test_driver_workflow(self) -> None:
        """Test typical driver workflow."""
        driver = INA219Driver(shunt_ohms=0.1)

        # Set simulated values
        driver.set_simulated_values(voltage=11.5, current=0.75)

        # Enable
        driver.enable()

        # Read values
        voltage = driver.read_voltage()
        current = driver.read_current()
        power = driver.read_power()

        assert voltage == 11.5
        assert current == 0.75
        assert abs(power - 8.625) < 0.01

        # Read all at once
        reading = driver.read_all()
        assert reading.voltage == voltage
        assert reading.current == current

        # Disable
        driver.disable()
        assert driver.is_enabled() is False

    def test_multiple_drivers(self) -> None:
        """Test multiple drivers can coexist."""
        driver1 = INA219Driver(address=0x40, shunt_ohms=0.1)
        driver2 = INA219Driver(address=0x41, shunt_ohms=0.1)
        driver3 = INA226Driver(address=0x44, shunt_ohms=0.01)

        driver1.set_simulated_values(voltage=12.0, current=1.0)
        driver2.set_simulated_values(voltage=5.0, current=0.5)
        driver3.set_simulated_values(voltage=24.0, current=2.0)

        driver1.enable()
        driver2.enable()
        driver3.enable()

        assert driver1.read_voltage() == 12.0
        assert driver2.read_voltage() == 5.0
        assert driver3.read_voltage() == 24.0
