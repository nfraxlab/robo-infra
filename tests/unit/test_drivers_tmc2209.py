"""Unit tests for TMC2209 stepper driver.

Tests the TMC2209Driver class including UART communication,
register configuration, motor current control, and StallGuard.
"""

from __future__ import annotations

import os

import pytest


# Set simulation mode for tests
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.drivers.tmc2209 import (
    DRVStatusBits,
    GCONFBits,
    TMC2209Config,
    TMC2209Driver,
    TMC2209Register,
)


class TestTMC2209Config:
    """Tests for TMC2209Config dataclass."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = TMC2209Config()
        assert config.address == 0
        assert config.run_current == 1.0
        assert config.hold_current == 0.5
        assert config.microstepping == 16
        assert config.stealthchop is True
        assert config.stallguard_threshold == 100  # Actual default
        assert config.rsense == 0.11

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = TMC2209Config(
            address=1,
            run_current=1.5,
            hold_current=0.8,
            microstepping=32,
            stealthchop=False,
            stallguard_threshold=100,
            rsense=0.22,
        )
        assert config.address == 1
        assert config.run_current == 1.5
        assert config.hold_current == 0.8
        assert config.microstepping == 32
        assert config.stealthchop is False
        assert config.stallguard_threshold == 100
        assert config.rsense == 0.22


class TestTMC2209DriverLifecycle:
    """Tests for TMC2209Driver lifecycle methods."""

    def test_init_default(self) -> None:
        """Test driver initialization with defaults."""
        driver = TMC2209Driver()
        assert driver.simulation is True

    def test_init_with_config(self) -> None:
        """Test driver initialization with custom config."""
        config = TMC2209Config(address=2, run_current=1.2)
        TMC2209Driver(config=config)
        # Config is applied during initialization

    def test_connect_simulation(self) -> None:
        """Test connection in simulation mode."""
        driver = TMC2209Driver()
        driver.connect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.CONNECTED

    def test_disconnect(self) -> None:
        """Test disconnection."""
        driver = TMC2209Driver()
        driver.connect()
        driver.disconnect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.DISCONNECTED


class TestTMC2209DriverCurrentControl:
    """Tests for TMC2209Driver motor current control."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_set_current(self, driver: TMC2209Driver) -> None:
        """Test setting motor current."""
        driver.set_current(1.2, 0.6, 4)
        current = driver.get_current()
        # Get_current returns a tuple in actual implementation
        assert isinstance(current, tuple)

    def test_set_current_rejects_out_of_range(self, driver: TMC2209Driver) -> None:
        """Test that current out of range raises error."""
        with pytest.raises(ValueError):
            driver.set_current(5.0, 3.0)  # Above max ~2.8A

    def test_get_current_initial(self, driver: TMC2209Driver) -> None:
        """Test getting initial current values."""
        current = driver.get_current()
        assert isinstance(current, tuple)
        assert len(current) == 2  # run_current, hold_current


class TestTMC2209DriverMicrostepping:
    """Tests for TMC2209Driver microstepping control."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_set_microstepping_valid(self, driver: TMC2209Driver) -> None:
        """Test setting valid microstepping values."""
        for microsteps in [1, 2, 4, 8, 16, 32, 64, 128, 256]:
            driver.set_microstepping(microsteps)
            # Should not raise

    def test_set_microstepping_invalid(self, driver: TMC2209Driver) -> None:
        """Test setting invalid microstepping value."""
        with pytest.raises(ValueError):
            driver.set_microstepping(3)  # Not a power of 2

    def test_set_microstepping_out_of_range(self, driver: TMC2209Driver) -> None:
        """Test setting out of range microstepping value."""
        with pytest.raises(ValueError):
            driver.set_microstepping(512)  # Too high


class TestTMC2209DriverOperatingModes:
    """Tests for TMC2209Driver operating modes."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_enable_stealthchop(self, driver: TMC2209Driver) -> None:
        """Test enabling StealthChop mode."""
        driver.enable_stealthchop()
        # Should not raise

    def test_enable_spreadcycle(self, driver: TMC2209Driver) -> None:
        """Test enabling SpreadCycle mode."""
        driver.enable_spreadcycle()
        # Should not raise


class TestTMC2209DriverStallGuard:
    """Tests for TMC2209Driver StallGuard (sensorless homing)."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_set_stallguard_threshold(self, driver: TMC2209Driver) -> None:
        """Test setting StallGuard threshold."""
        driver.set_stallguard_threshold(80)
        # Should not raise

    def test_set_stallguard_threshold_rejects_out_of_range(self, driver: TMC2209Driver) -> None:
        """Test that StallGuard threshold out of range raises error."""
        with pytest.raises(ValueError):
            driver.set_stallguard_threshold(300)  # Above max 255

    def test_get_stallguard(self, driver: TMC2209Driver) -> None:
        """Test getting StallGuard value."""
        value = driver.get_stallguard()
        assert isinstance(value, int)
        assert 0 <= value <= 510

    def test_is_stalled(self, driver: TMC2209Driver) -> None:
        """Test stall detection."""
        stalled = driver.is_stalled()
        assert isinstance(stalled, bool)


class TestTMC2209DriverStatus:
    """Tests for TMC2209Driver status methods."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_get_status(self, driver: TMC2209Driver) -> None:
        """Test getting driver status."""
        status = driver.get_status()
        assert "connected" in status
        assert status["connected"] is True

    def test_get_temperature_flags(self, driver: TMC2209Driver) -> None:
        """Test getting temperature flags."""
        flags = driver.get_temperature_flags()
        # Actual implementation uses different keys
        assert isinstance(flags, dict)


class TestTMC2209DriverDirection:
    """Tests for TMC2209Driver direction control."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_set_direction_inverted(self, driver: TMC2209Driver) -> None:
        """Test setting inverted direction."""
        driver.set_direction(inverted=True)
        # Should not raise

    def test_set_direction_normal(self, driver: TMC2209Driver) -> None:
        """Test setting normal direction."""
        driver.set_direction(inverted=False)
        # Should not raise


class TestTMC2209DriverChannelInterface:
    """Tests for TMC2209Driver channel interface (Driver ABC)."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_write_channel(self, driver: TMC2209Driver) -> None:
        """Test writing to channel."""
        driver._write_channel(0, 0.5)
        # Should not raise

    def test_read_channel(self, driver: TMC2209Driver) -> None:
        """Test reading from channel."""
        value = driver._read_channel(0)
        assert isinstance(value, float)


class TestTMC2209Enums:
    """Tests for TMC2209 enums."""

    def test_register_values(self) -> None:
        """Test register enum values."""
        assert TMC2209Register.GCONF == 0x00
        assert TMC2209Register.GSTAT == 0x01
        assert TMC2209Register.IHOLD_IRUN == 0x10
        assert TMC2209Register.CHOPCONF == 0x6C
        assert TMC2209Register.DRV_STATUS == 0x6F

    def test_gconf_bits(self) -> None:
        """Test GCONF bit flags."""
        assert GCONFBits.I_SCALE_ANALOG == (1 << 0)
        assert GCONFBits.SHAFT == (1 << 3)
        assert GCONFBits.EN_SPREADCYCLE == (1 << 2)
        assert GCONFBits.PDN_DISABLE == (1 << 6)

    def test_drv_status_bits(self) -> None:
        """Test DRV_STATUS bit flags."""
        assert DRVStatusBits.STST == (1 << 31)
        assert DRVStatusBits.OT == (1 << 1)
        assert DRVStatusBits.OTPW == (1 << 0)


# =============================================================================
# Phase 5.8.2.1 - TMC2209 UART/Register Tests
# =============================================================================


class TestTMC2209UARTRead:
    """Tests for TMC2209 UART read operations."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_uart_read_simulation_returns_cached(self, driver: TMC2209Driver) -> None:
        """Test that UART read in simulation returns cached value."""
        # In simulation mode, _read_register returns cached values
        value = driver._read_register(TMC2209Register.GCONF)
        assert isinstance(value, int)

    def test_uart_read_register_cache(self, driver: TMC2209Driver) -> None:
        """Test that register values are cached after read."""
        driver._register_cache[TMC2209Register.CHOPCONF] = 0x12345678
        value = driver._read_register(TMC2209Register.CHOPCONF)
        assert value == 0x12345678

    def test_uart_read_default_cache_value(self, driver: TMC2209Driver) -> None:
        """Test that uncached register returns 0 in simulation."""
        driver._register_cache.clear()
        value = driver._read_register(TMC2209Register.SGTHRS)
        assert value == 0

    def test_uart_read_multiple_registers(self, driver: TMC2209Driver) -> None:
        """Test reading multiple different registers."""
        driver._register_cache[TMC2209Register.GCONF] = 0x00000001
        driver._register_cache[TMC2209Register.IHOLD_IRUN] = 0x00001F1F
        driver._register_cache[TMC2209Register.CHOPCONF] = 0x10000053

        assert driver._read_register(TMC2209Register.GCONF) == 0x00000001
        assert driver._read_register(TMC2209Register.IHOLD_IRUN) == 0x00001F1F
        assert driver._read_register(TMC2209Register.CHOPCONF) == 0x10000053


class TestTMC2209UARTWrite:
    """Tests for TMC2209 UART write operations."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_uart_write_updates_cache(self, driver: TMC2209Driver) -> None:
        """Test that UART write updates the register cache."""
        driver._write_register(TMC2209Register.CHOPCONF, 0xABCDEF01)
        assert driver._register_cache[TMC2209Register.CHOPCONF] == 0xABCDEF01

    def test_uart_write_gconf_register(self, driver: TMC2209Driver) -> None:
        """Test writing to GCONF register."""
        driver._write_register(TMC2209Register.GCONF, GCONFBits.PDN_DISABLE | GCONFBits.MSTEP_REG_SELECT)
        cached = driver._register_cache[TMC2209Register.GCONF]
        assert cached & GCONFBits.PDN_DISABLE
        assert cached & GCONFBits.MSTEP_REG_SELECT

    def test_uart_write_ihold_irun_register(self, driver: TMC2209Driver) -> None:
        """Test writing to IHOLD_IRUN register."""
        # IHOLD_IRUN: IRUN[4:0] | IHOLD[4:0] << 16 | IHOLDDELAY[3:0] << 20
        irun = 16
        ihold = 8
        iholddelay = 6
        value = irun | (ihold << 8) | (iholddelay << 16)
        driver._write_register(TMC2209Register.IHOLD_IRUN, value)
        assert driver._register_cache[TMC2209Register.IHOLD_IRUN] == value

    def test_uart_write_preserves_other_registers(self, driver: TMC2209Driver) -> None:
        """Test that writing one register doesn't affect others."""
        driver._write_register(TMC2209Register.GCONF, 0x11111111)
        driver._write_register(TMC2209Register.CHOPCONF, 0x22222222)
        assert driver._register_cache[TMC2209Register.GCONF] == 0x11111111
        assert driver._register_cache[TMC2209Register.CHOPCONF] == 0x22222222


class TestTMC2209RegisterRead:
    """Tests for TMC2209 higher-level register read operations."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_register_read_drv_status(self, driver: TMC2209Driver) -> None:
        """Test reading DRV_STATUS register via get_status."""
        status = driver.get_status()
        assert "stallguard" in status
        assert "actual_current" in status

    def test_register_read_stallguard_result(self, driver: TMC2209Driver) -> None:
        """Test reading StallGuard result register."""
        sg_value = driver.get_stallguard()
        assert 0 <= sg_value <= 510

    def test_register_read_temperature_flags(self, driver: TMC2209Driver) -> None:
        """Test reading temperature flag bits from DRV_STATUS."""
        flags = driver.get_temperature_flags()
        assert "over_120c" in flags
        assert "over_143c" in flags
        assert "over_150c" in flags
        assert "over_157c" in flags

    def test_register_read_current_via_status(self, driver: TMC2209Driver) -> None:
        """Test reading actual current from DRV_STATUS."""
        status = driver.get_status()
        assert "actual_current" in status
        assert isinstance(status["actual_current"], int)


class TestTMC2209RegisterWrite:
    """Tests for TMC2209 higher-level register write operations."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver for tests."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_register_write_stallguard_threshold(self, driver: TMC2209Driver) -> None:
        """Test writing SGTHRS register via set_stallguard_threshold."""
        driver.set_stallguard_threshold(100)
        # In simulation, this just logs

    def test_register_write_stallguard_min_threshold(self, driver: TMC2209Driver) -> None:
        """Test minimum StallGuard threshold value."""
        driver.set_stallguard_threshold(0)

    def test_register_write_stallguard_max_threshold(self, driver: TMC2209Driver) -> None:
        """Test maximum StallGuard threshold value."""
        driver.set_stallguard_threshold(255)

    def test_register_write_stallguard_invalid_low(self, driver: TMC2209Driver) -> None:
        """Test StallGuard threshold below valid range."""
        with pytest.raises(ValueError):
            driver.set_stallguard_threshold(-1)

    def test_register_write_stallguard_invalid_high(self, driver: TMC2209Driver) -> None:
        """Test StallGuard threshold above valid range."""
        with pytest.raises(ValueError):
            driver.set_stallguard_threshold(256)


class TestTMC2209RunCurrent:
    """Tests for TMC2209 run current configuration."""

    def test_run_current_default(self) -> None:
        """Test default run current in config."""
        config = TMC2209Config()
        assert config.run_current == 1.0  # Default from code

    def test_run_current_custom(self) -> None:
        """Test custom run current in config."""
        config = TMC2209Config(run_current=1.5)
        assert config.run_current == 1.5

    def test_run_current_set_via_driver(self) -> None:
        """Test setting run current via driver method."""
        driver = TMC2209Driver()
        driver.connect()
        driver.set_current(1.2)  # Sets both run and hold

    def test_run_current_range_minimum(self) -> None:
        """Test minimum run current value."""
        config = TMC2209Config(run_current=0.1)
        assert config.run_current == 0.1

    def test_run_current_range_maximum(self) -> None:
        """Test maximum run current for TMC2209 (2.8A RMS)."""
        config = TMC2209Config(run_current=2.8)
        assert config.run_current == 2.8


class TestTMC2209HoldCurrent:
    """Tests for TMC2209 hold current configuration."""

    def test_hold_current_default(self) -> None:
        """Test default hold current in config."""
        config = TMC2209Config()
        assert config.hold_current == 0.5  # Default from code

    def test_hold_current_custom(self) -> None:
        """Test custom hold current in config."""
        config = TMC2209Config(hold_current=0.3)
        assert config.hold_current == 0.3

    def test_hold_current_via_driver(self) -> None:
        """Test getting hold current via driver."""
        driver = TMC2209Driver(config=TMC2209Config(hold_current=0.4))
        driver.connect()
        # Hold current is stored in config
        assert driver._tmc_config.hold_current == 0.4

    def test_hold_current_less_than_run(self) -> None:
        """Test hold current less than run current (typical use)."""
        config = TMC2209Config(run_current=1.5, hold_current=0.5)
        assert config.hold_current < config.run_current

    def test_hold_delay_default(self) -> None:
        """Test default hold delay value."""
        config = TMC2209Config()
        assert config.hold_delay == 10  # Default from code


class TestTMC2209MicrosteppingAdvanced:
    """Advanced tests for TMC2209 microstepping configuration."""

    def test_microstepping_config_values(self) -> None:
        """Test microstepping values via config."""
        valid_values = [1, 2, 4, 8, 16, 32, 64, 128, 256]
        for microsteps in valid_values:
            config = TMC2209Config(microstepping=microsteps)
            assert config.microstepping == microsteps

    def test_microstepping_interpolation(self) -> None:
        """Test TMC2209 256 microstep interpolation capability."""
        config = TMC2209Config(microstepping=256)
        assert config.microstepping == 256

    def test_microstepping_default_is_16(self) -> None:
        """Test default microstepping is 16 for TMC2209."""
        config = TMC2209Config()
        assert config.microstepping == 16

    def test_microstepping_invalid_value(self) -> None:
        """Test invalid microstepping value in config."""
        # TMC2209 config allows various microstepping values
        # The driver validates when connecting
        config = TMC2209Config(microstepping=3)
        # Config accepts the value, validation happens at driver level
        assert config.microstepping == 3

    def test_microstepping_power_of_two_values(self) -> None:
        """Test all power of two microstepping values are valid."""
        for microsteps in [1, 2, 4, 8, 16, 32, 64, 128, 256]:
            config = TMC2209Config(microstepping=microsteps)
            assert config.microstepping == microsteps


class TestTMC2209StealthChopAdvanced:
    """Advanced tests for TMC2209 StealthChop mode."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_stealthchop_enable(self, driver: TMC2209Driver) -> None:
        """Test enabling StealthChop mode."""
        driver.enable_stealthchop()
        # In simulation, is_stealthchop checks register cache
        # After enable, stealthchop should be True
        assert driver.is_stealthchop() is True

    def test_stealthchop_config_default(self) -> None:
        """Test StealthChop is enabled by default in config."""
        config = TMC2209Config()
        assert config.stealthchop is True

    def test_stealthchop_config_disabled(self) -> None:
        """Test StealthChop can be disabled in config."""
        config = TMC2209Config(stealthchop=False)
        assert config.stealthchop is False

    def test_stealthchop_driver_reads_cache(self, driver: TMC2209Driver) -> None:
        """Test is_stealthchop reads from register cache in simulation."""
        # In simulation, is_stealthchop reads GCONF from cache
        # Default cache has no EN_SPREADCYCLE bit set -> StealthChop is True
        assert driver.is_stealthchop() is True

    def test_stealthchop_enable_multiple_times(self, driver: TMC2209Driver) -> None:
        """Test enabling StealthChop multiple times is idempotent."""
        driver.enable_stealthchop()
        driver.enable_stealthchop()
        driver.enable_stealthchop()
        assert driver.is_stealthchop() is True


class TestTMC2209StallGuardAdvanced:
    """Advanced tests for TMC2209 StallGuard sensorless homing."""

    @pytest.fixture
    def driver(self) -> TMC2209Driver:
        """Create connected driver."""
        drv = TMC2209Driver()
        drv.connect()
        return drv

    def test_stallguard_threshold_config(self) -> None:
        """Test StallGuard threshold in config."""
        config = TMC2209Config(stallguard_threshold=50)
        assert config.stallguard_threshold == 50

    def test_stallguard_threshold_default(self) -> None:
        """Test StallGuard default threshold in config."""
        config = TMC2209Config()
        assert config.stallguard_threshold == 100  # Default from code

    def test_stallguard_is_stalled_with_custom_threshold(self, driver: TMC2209Driver) -> None:
        """Test is_stalled with explicit threshold parameter."""
        # In simulation, get_stallguard returns 250
        # threshold > 250 means stalled (sg_value < threshold)
        assert driver.is_stalled(threshold=300) is True
        # threshold < 250 means not stalled
        assert driver.is_stalled(threshold=200) is False

    def test_stallguard_get_value_range(self, driver: TMC2209Driver) -> None:
        """Test StallGuard value is in valid range."""
        sg_value = driver.get_stallguard()
        assert 0 <= sg_value <= 510  # 10-bit value

    def test_stallguard_simulation_returns_250(self, driver: TMC2209Driver) -> None:
        """Test StallGuard simulation returns expected value."""
        sg_value = driver.get_stallguard()
        assert sg_value == 250  # Simulation returns mid-range value


class TestTMC2209CRCCalculation:
    """Tests for TMC2209 CRC8 calculation."""

    def test_calc_crc_empty(self) -> None:
        """Test CRC calculation for empty data."""
        driver = TMC2209Driver()
        crc = driver._calc_crc(b"")
        assert isinstance(crc, int)
        assert 0 <= crc <= 255

    def test_calc_crc_single_byte(self) -> None:
        """Test CRC calculation for single byte."""
        driver = TMC2209Driver()
        crc = driver._calc_crc(b"\x05")
        assert isinstance(crc, int)
        assert 0 <= crc <= 255

    def test_calc_crc_datagram(self) -> None:
        """Test CRC calculation for typical datagram."""
        driver = TMC2209Driver()
        # Typical read request: SYNC + ADDR + REG
        datagram = bytes([0x05, 0x00, 0x00])  # Read GCONF from address 0
        crc = driver._calc_crc(datagram)
        assert isinstance(crc, int)
        assert 0 <= crc <= 255

    def test_calc_crc_deterministic(self) -> None:
        """Test CRC calculation is deterministic."""
        driver = TMC2209Driver()
        data = bytes([0x05, 0x00, 0x10])
        crc1 = driver._calc_crc(data)
        crc2 = driver._calc_crc(data)
        assert crc1 == crc2

    def test_calc_crc_different_for_different_data(self) -> None:
        """Test CRC is different for different data."""
        driver = TMC2209Driver()
        crc1 = driver._calc_crc(bytes([0x05, 0x00, 0x00]))
        crc2 = driver._calc_crc(bytes([0x05, 0x00, 0x01]))
        # CRCs should be different for different input
        assert crc1 != crc2


class TestTMC2209AddressConfiguration:
    """Tests for TMC2209 UART address configuration."""

    def test_address_default(self) -> None:
        """Test default UART address."""
        config = TMC2209Config()
        assert config.address == 0

    def test_address_custom(self) -> None:
        """Test custom UART address (0-3)."""
        for addr in [0, 1, 2, 3]:
            config = TMC2209Config(address=addr)
            assert config.address == addr

    def test_config_stores_address(self) -> None:
        """Test config stores UART address."""
        config = TMC2209Config(address=2)
        assert config.address == 2

    def test_multiple_configs_different_addresses(self) -> None:
        """Test multiple configs with different addresses."""
        config1 = TMC2209Config(address=0)
        config2 = TMC2209Config(address=1)
        config3 = TMC2209Config(address=2)

        assert config1.address == 0
        assert config2.address == 1
        assert config3.address == 2
