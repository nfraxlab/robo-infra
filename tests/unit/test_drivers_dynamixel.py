"""Unit tests for Dynamixel smart servo driver.

Tests the DynamixelDriver class including Protocol 2.0 communication,
servo control, sync operations, and telemetry.
"""

from __future__ import annotations

import os

import pytest


# Set simulation mode for tests
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.drivers.dynamixel import (
    ControlTable,
    DynamixelConfig,
    DynamixelDriver,
    HardwareErrorStatus,
    Instruction,
    OperatingMode,
)


class TestDynamixelConfig:
    """Tests for DynamixelConfig dataclass."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = DynamixelConfig()
        assert config.port == "/dev/ttyUSB0"
        assert config.baudrate == 1000000
        assert config.timeout == 0.1
        assert config.latency_timer == 4

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = DynamixelConfig(
            port="/dev/ttyACM0",
            baudrate=57600,
            timeout=0.5,
            latency_timer=8,
        )
        assert config.port == "/dev/ttyACM0"
        assert config.baudrate == 57600
        assert config.timeout == 0.5
        assert config.latency_timer == 8


class TestDynamixelDriverLifecycle:
    """Tests for DynamixelDriver lifecycle methods."""

    def test_init_default(self) -> None:
        """Test driver initialization with defaults."""
        driver = DynamixelDriver()
        assert driver.simulation is True
        assert driver.port == "/dev/ttyUSB0"

    def test_init_with_port(self) -> None:
        """Test driver initialization with custom port."""
        driver = DynamixelDriver(port="/dev/ttyACM0")
        assert driver._dxl_config.port == "/dev/ttyACM0"

    def test_init_with_baudrate(self) -> None:
        """Test driver initialization with custom baudrate."""
        driver = DynamixelDriver(baudrate=57600)
        assert driver._dxl_config.baudrate == 57600

    def test_init_with_config(self) -> None:
        """Test driver initialization with config."""
        config = DynamixelConfig(port="/dev/ttyUSB1", baudrate=115200)
        driver = DynamixelDriver(config=config)
        assert driver._dxl_config.baudrate == 115200

    def test_connect_simulation(self) -> None:
        """Test connection in simulation mode."""
        driver = DynamixelDriver()
        driver.connect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.CONNECTED
        # Check simulated servos were initialized
        assert len(driver._sim_servos) == 4

    def test_disconnect(self) -> None:
        """Test disconnection."""
        driver = DynamixelDriver()
        driver.connect()
        driver.disconnect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.DISCONNECTED


class TestDynamixelDriverPing:
    """Tests for DynamixelDriver ping operations."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_ping_found(self, driver: DynamixelDriver) -> None:
        """Test pinging existing servo."""
        result = driver.ping(1)
        assert result is not None
        assert "model" in result
        assert "firmware" in result

    def test_ping_not_found(self, driver: DynamixelDriver) -> None:
        """Test pinging non-existent servo."""
        result = driver.ping(100)  # Not in sim_servos
        assert result is None

    def test_ping_scan(self, driver: DynamixelDriver) -> None:
        """Test scanning for servos."""
        found = driver.ping_scan(1, 10)
        assert 1 in found
        assert 2 in found
        assert 3 in found
        assert 4 in found
        assert 5 not in found


class TestDynamixelDriverReboot:
    """Tests for DynamixelDriver reboot."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_reboot(self, driver: DynamixelDriver) -> None:
        """Test rebooting a servo."""
        driver.reboot(1)
        # Should not raise in simulation


class TestDynamixelDriverTorque:
    """Tests for DynamixelDriver torque control."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_set_torque_enable(self, driver: DynamixelDriver) -> None:
        """Test enabling torque."""
        driver.set_torque_enable(1, True)
        assert driver._sim_servos[1]["torque_enable"] is True

    def test_set_torque_disable(self, driver: DynamixelDriver) -> None:
        """Test disabling torque."""
        driver.set_torque_enable(1, True)
        driver.set_torque_enable(1, False)
        assert driver._sim_servos[1]["torque_enable"] is False

    def test_get_torque_enable(self, driver: DynamixelDriver) -> None:
        """Test getting torque enable status."""
        driver.set_torque_enable(1, True)
        assert driver.get_torque_enable(1) is True
        driver.set_torque_enable(1, False)
        assert driver.get_torque_enable(1) is False


class TestDynamixelDriverPosition:
    """Tests for DynamixelDriver position control."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_set_goal_position(self, driver: DynamixelDriver) -> None:
        """Test setting goal position."""
        driver.set_goal_position(1, 2048)
        assert driver._sim_servos[1]["position"] == 2048

    def test_get_present_position(self, driver: DynamixelDriver) -> None:
        """Test getting present position."""
        driver.set_goal_position(1, 1024)
        pos = driver.get_present_position(1)
        assert pos == 1024


class TestDynamixelDriverVelocity:
    """Tests for DynamixelDriver velocity control."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_set_goal_velocity(self, driver: DynamixelDriver) -> None:
        """Test setting goal velocity."""
        driver.set_goal_velocity(1, 100)
        assert driver._sim_servos[1]["velocity"] == 100

    def test_get_present_velocity(self, driver: DynamixelDriver) -> None:
        """Test getting present velocity."""
        driver.set_goal_velocity(1, 50)
        vel = driver.get_present_velocity(1)
        assert vel == 50


class TestDynamixelDriverCurrent:
    """Tests for DynamixelDriver current control."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_set_goal_current(self, driver: DynamixelDriver) -> None:
        """Test setting goal current."""
        driver.set_goal_current(1, 200)
        # Should not raise in simulation

    def test_get_present_current(self, driver: DynamixelDriver) -> None:
        """Test getting present current."""
        current = driver.get_present_current(1)
        assert current == 0  # Simulated value


class TestDynamixelDriverOperatingMode:
    """Tests for DynamixelDriver operating mode."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_set_operating_mode_velocity(self, driver: DynamixelDriver) -> None:
        """Test setting velocity control mode."""
        driver.set_operating_mode(1, OperatingMode.VELOCITY_CONTROL)
        assert driver._sim_servos[1]["mode"] == OperatingMode.VELOCITY_CONTROL

    def test_set_operating_mode_position(self, driver: DynamixelDriver) -> None:
        """Test setting position control mode."""
        driver.set_operating_mode(1, OperatingMode.POSITION_CONTROL)
        assert driver._sim_servos[1]["mode"] == OperatingMode.POSITION_CONTROL

    def test_get_operating_mode(self, driver: DynamixelDriver) -> None:
        """Test getting operating mode."""
        driver.set_operating_mode(1, OperatingMode.EXTENDED_POSITION_CONTROL)
        mode = driver.get_operating_mode(1)
        assert mode == OperatingMode.EXTENDED_POSITION_CONTROL


class TestDynamixelDriverTelemetry:
    """Tests for DynamixelDriver telemetry."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_get_present_temperature(self, driver: DynamixelDriver) -> None:
        """Test getting temperature."""
        temp = driver.get_present_temperature(1)
        assert temp == 25  # Simulated default

    def test_get_present_voltage(self, driver: DynamixelDriver) -> None:
        """Test getting voltage."""
        voltage = driver.get_present_voltage(1)
        assert voltage == 12.0  # Simulated default (120 / 10)

    def test_is_moving(self, driver: DynamixelDriver) -> None:
        """Test checking if moving."""
        moving = driver.is_moving(1)
        assert moving is False  # Simulated default

    def test_get_hardware_error(self, driver: DynamixelDriver) -> None:
        """Test getting hardware error status."""
        error = driver.get_hardware_error(1)
        assert error == 0


class TestDynamixelDriverLED:
    """Tests for DynamixelDriver LED control."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_set_led_on(self, driver: DynamixelDriver) -> None:
        """Test turning LED on."""
        driver.set_led(1, True)
        # Should not raise in simulation

    def test_set_led_off(self, driver: DynamixelDriver) -> None:
        """Test turning LED off."""
        driver.set_led(1, False)
        # Should not raise in simulation


class TestDynamixelDriverProfile:
    """Tests for DynamixelDriver profile settings."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_set_profile_velocity(self, driver: DynamixelDriver) -> None:
        """Test setting profile velocity."""
        driver.set_profile_velocity(1, 100)
        # Should not raise in simulation

    def test_set_profile_acceleration(self, driver: DynamixelDriver) -> None:
        """Test setting profile acceleration."""
        driver.set_profile_acceleration(1, 50)
        # Should not raise in simulation


class TestDynamixelDriverSyncWrite:
    """Tests for DynamixelDriver sync write operations."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_sync_write_position(self, driver: DynamixelDriver) -> None:
        """Test sync writing positions to multiple servos."""
        positions = {1: 1000, 2: 2000, 3: 3000, 4: 4000}
        driver.sync_write_position(positions)
        # Should not raise in simulation


class TestDynamixelDriverStatus:
    """Tests for DynamixelDriver status methods."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_get_status(self, driver: DynamixelDriver) -> None:
        """Test getting overall status."""
        driver.set_goal_position(1, 1500)

        status = driver.get_status()

        assert status["connected"] is True
        assert status["simulation"] is True
        assert status["baudrate"] == 1000000
        assert len(status["servos"]) > 0


class TestDynamixelDriverChannelInterface:
    """Tests for DynamixelDriver channel interface (Driver ABC)."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_write_channel(self, driver: DynamixelDriver) -> None:
        """Test writing to channel (sets position)."""
        driver._write_channel(1, 0.5)  # 50% of 4095
        assert driver._sim_servos[1]["position"] == 2047

    def test_read_channel(self, driver: DynamixelDriver) -> None:
        """Test reading from channel (returns normalized position)."""
        driver.set_goal_position(1, 2048)
        value = driver._read_channel(1)
        assert abs(value - 0.5) < 0.01  # ~50% of 4095


class TestDynamixelEnums:
    """Tests for Dynamixel enums."""

    def test_instruction_values(self) -> None:
        """Test instruction enum values."""
        assert Instruction.PING == 0x01
        assert Instruction.READ == 0x02
        assert Instruction.WRITE == 0x03
        assert Instruction.SYNC_READ == 0x82
        assert Instruction.SYNC_WRITE == 0x83

    def test_operating_mode_values(self) -> None:
        """Test operating mode enum values."""
        assert OperatingMode.CURRENT_CONTROL == 0
        assert OperatingMode.VELOCITY_CONTROL == 1
        assert OperatingMode.POSITION_CONTROL == 3
        assert OperatingMode.EXTENDED_POSITION_CONTROL == 4
        assert OperatingMode.PWM_CONTROL == 16

    def test_hardware_error_status_values(self) -> None:
        """Test hardware error status enum values."""
        assert HardwareErrorStatus.INPUT_VOLTAGE == 0x01
        assert HardwareErrorStatus.OVERHEATING == 0x04
        assert HardwareErrorStatus.OVERLOAD == 0x20

    def test_control_table_addresses(self) -> None:
        """Test control table address values."""
        assert ControlTable.TORQUE_ENABLE == 64
        assert ControlTable.LED == 65
        assert ControlTable.GOAL_POSITION == 116
        assert ControlTable.PRESENT_POSITION == 132
        assert ControlTable.PRESENT_TEMPERATURE == 146


class TestDynamixelDriverReadWrite:
    """Tests for DynamixelDriver low-level read/write."""

    @pytest.fixture
    def driver(self) -> DynamixelDriver:
        """Create connected driver."""
        drv = DynamixelDriver()
        drv.connect()
        return drv

    def test_read_1(self, driver: DynamixelDriver) -> None:
        """Test reading 1-byte value."""
        value = driver.read_1(1, ControlTable.TORQUE_ENABLE)
        assert value == 0

    def test_read_2(self, driver: DynamixelDriver) -> None:
        """Test reading 2-byte value."""
        value = driver.read_2(1, ControlTable.PRESENT_INPUT_VOLTAGE)
        assert value == 0

    def test_read_4(self, driver: DynamixelDriver) -> None:
        """Test reading 4-byte value."""
        value = driver.read_4(1, ControlTable.PRESENT_POSITION)
        assert value == 0

    def test_write_1(self, driver: DynamixelDriver) -> None:
        """Test writing 1-byte value."""
        driver.write_1(1, ControlTable.LED, 1)
        # Should not raise in simulation

    def test_write_2(self, driver: DynamixelDriver) -> None:
        """Test writing 2-byte value."""
        driver.write_2(1, ControlTable.GOAL_CURRENT, 100)
        # Should not raise in simulation

    def test_write_4(self, driver: DynamixelDriver) -> None:
        """Test writing 4-byte value."""
        driver.write_4(1, ControlTable.GOAL_POSITION, 2048)
        # Should not raise in simulation
