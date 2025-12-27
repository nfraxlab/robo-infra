"""Unit tests for VESC brushless motor controller driver.

Tests the VESCDriver class including UART communication,
motor control modes, and telemetry.
"""

from __future__ import annotations

import os

import pytest


# Set simulation mode for tests
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.drivers.vesc import (
    VESCConfig,
    VESCControlMode,
    VESCDriver,
    VESCFaultCode,
    VESCPacketID,
    VESCState,
)


class TestVESCConfig:
    """Tests for VESCConfig dataclass."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = VESCConfig()
        assert config.port == "/dev/ttyUSB0"
        assert config.baudrate == 115200
        assert config.timeout == 0.5
        assert config.current_limit == 60.0
        assert config.rpm_limit == 100000

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = VESCConfig(
            port="/dev/ttyACM0",
            baudrate=230400,
            timeout=1.0,
            current_limit=80.0,
            rpm_limit=50000,
        )
        assert config.port == "/dev/ttyACM0"
        assert config.baudrate == 230400
        assert config.timeout == 1.0
        assert config.current_limit == 80.0
        assert config.rpm_limit == 50000


class TestVESCState:
    """Tests for VESCState dataclass."""

    def test_default_state(self) -> None:
        """Test default state values."""
        state = VESCState()
        assert state.temp_mos == 0.0
        assert state.temp_motor == 0.0
        assert state.avg_motor_current == 0.0
        assert state.rpm == 0
        assert state.v_in == 0.0
        assert state.fault == VESCFaultCode.NONE

    def test_custom_state(self) -> None:
        """Test custom state values."""
        state = VESCState(
            temp_mos=45.0,
            temp_motor=50.0,
            avg_motor_current=20.0,
            rpm=5000,
            v_in=24.0,
            fault=VESCFaultCode.OVER_TEMP_FET,
        )
        assert state.temp_mos == 45.0
        assert state.temp_motor == 50.0
        assert state.avg_motor_current == 20.0
        assert state.rpm == 5000
        assert state.v_in == 24.0
        assert state.fault == VESCFaultCode.OVER_TEMP_FET


class TestVESCDriverLifecycle:
    """Tests for VESCDriver lifecycle methods."""

    def test_init_default(self) -> None:
        """Test driver initialization with defaults."""
        driver = VESCDriver()
        assert driver.simulation is True
        assert driver.port == "/dev/ttyUSB0"

    def test_init_with_port(self) -> None:
        """Test driver initialization with custom port."""
        driver = VESCDriver(port="/dev/ttyACM0")
        assert driver._vesc_config.port == "/dev/ttyACM0"

    def test_init_with_config(self) -> None:
        """Test driver initialization with config."""
        config = VESCConfig(port="/dev/ttyUSB1", current_limit=50.0)
        driver = VESCDriver(config=config)
        assert driver._vesc_config.current_limit == 50.0

    def test_connect_simulation(self) -> None:
        """Test connection in simulation mode."""
        driver = VESCDriver()
        driver.connect()
        from robo_infra.core.driver import DriverState

        assert driver._state == DriverState.CONNECTED

    def test_disconnect(self) -> None:
        """Test disconnection."""
        driver = VESCDriver()
        driver.connect()
        driver.disconnect()
        from robo_infra.core.driver import DriverState

        assert driver._state == DriverState.DISCONNECTED


class TestVESCDriverFirmware:
    """Tests for VESCDriver firmware info."""

    @pytest.fixture
    def driver(self) -> VESCDriver:
        """Create connected driver."""
        drv = VESCDriver()
        drv.connect()
        return drv

    def test_get_firmware_version(self, driver: VESCDriver) -> None:
        """Test getting firmware version."""
        major, minor = driver.get_firmware_version()
        assert major == 5
        assert minor == 3


class TestVESCDriverMotorControl:
    """Tests for VESCDriver motor control."""

    @pytest.fixture
    def driver(self) -> VESCDriver:
        """Create connected driver."""
        drv = VESCDriver()
        drv.connect()
        return drv

    def test_set_duty_cycle(self, driver: VESCDriver) -> None:
        """Test setting duty cycle."""
        driver.set_duty_cycle(0.5)
        assert driver._sim_state.duty_cycle == 0.5
        assert driver._sim_control_mode == VESCControlMode.DUTY_CYCLE

    def test_set_duty_cycle_clamps_positive(self, driver: VESCDriver) -> None:
        """Test that duty cycle is clamped to 1.0."""
        driver.set_duty_cycle(1.5)
        assert driver._sim_state.duty_cycle == 1.0

    def test_set_duty_cycle_clamps_negative(self, driver: VESCDriver) -> None:
        """Test that duty cycle is clamped to -1.0."""
        driver.set_duty_cycle(-1.5)
        assert driver._sim_state.duty_cycle == -1.0

    def test_set_current(self, driver: VESCDriver) -> None:
        """Test setting motor current."""
        driver.set_current(20.0)
        assert driver._sim_state.avg_motor_current == 20.0
        assert driver._sim_control_mode == VESCControlMode.CURRENT

    def test_set_current_clamps(self, driver: VESCDriver) -> None:
        """Test that current is clamped to limit."""
        driver.set_current(100.0)  # Above 60A limit
        assert driver._sim_state.avg_motor_current == 60.0

    def test_set_current_brake(self, driver: VESCDriver) -> None:
        """Test setting braking current."""
        driver.set_current_brake(15.0)
        assert driver._sim_control_mode == VESCControlMode.CURRENT_BRAKE

    def test_set_rpm(self, driver: VESCDriver) -> None:
        """Test setting RPM."""
        driver.set_rpm(5000)
        assert driver._sim_state.rpm == 5000
        assert driver._sim_control_mode == VESCControlMode.SPEED

    def test_set_rpm_clamps(self, driver: VESCDriver) -> None:
        """Test that RPM is clamped to limit."""
        driver.set_rpm(200000)  # Above 100000 limit
        assert driver._sim_state.rpm == 100000

    def test_set_position(self, driver: VESCDriver) -> None:
        """Test setting position."""
        driver.set_position(180.0)
        assert driver._sim_state.pid_pos == 180.0
        assert driver._sim_control_mode == VESCControlMode.POSITION

    def test_set_handbrake(self, driver: VESCDriver) -> None:
        """Test setting handbrake."""
        driver.set_handbrake(10.0)
        # Should not raise in simulation


class TestVESCDriverTelemetry:
    """Tests for VESCDriver telemetry."""

    @pytest.fixture
    def driver(self) -> VESCDriver:
        """Create connected driver."""
        drv = VESCDriver()
        drv.connect()
        return drv

    def test_get_state(self, driver: VESCDriver) -> None:
        """Test getting VESC state."""
        driver.set_duty_cycle(0.5)
        driver._sim_state.v_in = 24.0
        driver._sim_state.temp_mos = 35.0

        state = driver.get_state()

        assert state.duty_cycle == 0.5
        assert state.v_in == 24.0
        assert state.temp_mos == 35.0

    def test_get_voltage(self, driver: VESCDriver) -> None:
        """Test getting input voltage."""
        driver._sim_state.v_in = 36.0
        voltage = driver.get_voltage()
        assert voltage == 36.0

    def test_get_rpm(self, driver: VESCDriver) -> None:
        """Test getting RPM."""
        driver.set_rpm(8000)
        rpm = driver.get_rpm()
        assert rpm == 8000

    def test_get_motor_current(self, driver: VESCDriver) -> None:
        """Test getting motor current."""
        driver.set_current(25.0)
        current = driver.get_motor_current()
        assert current == 25.0

    def test_get_temperature(self, driver: VESCDriver) -> None:
        """Test getting temperatures."""
        driver._sim_state.temp_mos = 40.0
        driver._sim_state.temp_motor = 55.0

        mos_temp, motor_temp = driver.get_temperature()
        assert mos_temp == 40.0
        assert motor_temp == 55.0

    def test_get_tachometer(self, driver: VESCDriver) -> None:
        """Test getting tachometer value."""
        driver._sim_state.tachometer = 12345
        tach = driver.get_tachometer()
        assert tach == 12345

    def test_get_position(self, driver: VESCDriver) -> None:
        """Test getting PID position."""
        driver.set_position(90.0)
        pos = driver.get_position()
        assert pos == 90.0


class TestVESCDriverFaultHandling:
    """Tests for VESCDriver fault handling."""

    @pytest.fixture
    def driver(self) -> VESCDriver:
        """Create connected driver."""
        drv = VESCDriver()
        drv.connect()
        return drv

    def test_get_fault_no_fault(self, driver: VESCDriver) -> None:
        """Test getting fault when no fault."""
        fault = driver.get_fault()
        assert fault == VESCFaultCode.NONE

    def test_is_faulted_false(self, driver: VESCDriver) -> None:
        """Test is_faulted when no fault."""
        assert driver.is_faulted() is False

    def test_is_faulted_true(self, driver: VESCDriver) -> None:
        """Test is_faulted when faulted."""
        driver._sim_state.fault = VESCFaultCode.OVER_VOLTAGE
        assert driver.is_faulted() is True


class TestVESCDriverKeepalive:
    """Tests for VESCDriver keepalive and reboot."""

    @pytest.fixture
    def driver(self) -> VESCDriver:
        """Create connected driver."""
        drv = VESCDriver()
        drv.connect()
        return drv

    def test_send_alive(self, driver: VESCDriver) -> None:
        """Test sending keepalive packet."""
        driver.send_alive()
        # Should not raise in simulation

    def test_reboot(self, driver: VESCDriver) -> None:
        """Test rebooting VESC."""
        driver.reboot()
        # Should not raise in simulation


class TestVESCDriverStatus:
    """Tests for VESCDriver status methods."""

    @pytest.fixture
    def driver(self) -> VESCDriver:
        """Create connected driver."""
        drv = VESCDriver()
        drv.connect()
        return drv

    def test_get_status(self, driver: VESCDriver) -> None:
        """Test getting overall status."""
        driver.set_duty_cycle(0.3)
        driver._sim_state.v_in = 24.0
        driver._sim_state.temp_mos = 35.0

        status = driver.get_status()

        assert status["connected"] is True
        assert status["simulation"] is True
        assert status["voltage"] == 24.0
        assert status["duty_cycle"] == 0.3
        assert status["temp_mos"] == 35.0
        assert status["fault"] == "NONE"


class TestVESCDriverChannelInterface:
    """Tests for VESCDriver channel interface (Driver ABC)."""

    @pytest.fixture
    def driver(self) -> VESCDriver:
        """Create connected driver."""
        drv = VESCDriver()
        drv.connect()
        return drv

    def test_write_channel(self, driver: VESCDriver) -> None:
        """Test writing to channel (sets duty cycle)."""
        driver._write_channel(0, 0.7)
        assert driver._sim_state.duty_cycle == 0.7

    def test_read_channel(self, driver: VESCDriver) -> None:
        """Test reading from channel (returns normalized RPM)."""
        driver.set_rpm(50000)  # Half of 100000 limit
        value = driver._read_channel(0)
        assert value == 0.5


class TestVESCEnums:
    """Tests for VESC enums."""

    def test_fault_code_values(self) -> None:
        """Test fault code enum values."""
        assert VESCFaultCode.NONE == 0
        assert VESCFaultCode.OVER_VOLTAGE == 1
        assert VESCFaultCode.UNDER_VOLTAGE == 2
        assert VESCFaultCode.OVER_TEMP_FET == 5
        assert VESCFaultCode.OVER_TEMP_MOTOR == 6

    def test_control_mode_values(self) -> None:
        """Test control mode enum values."""
        assert VESCControlMode.DUTY_CYCLE == 0
        assert VESCControlMode.SPEED == 1
        assert VESCControlMode.CURRENT == 2
        assert VESCControlMode.CURRENT_BRAKE == 3
        assert VESCControlMode.POSITION == 4

    def test_packet_id_values(self) -> None:
        """Test packet ID enum values."""
        assert VESCPacketID.FW_VERSION == 0
        assert VESCPacketID.GET_VALUES == 4
        assert VESCPacketID.SET_DUTY == 5
        assert VESCPacketID.SET_CURRENT == 6
        assert VESCPacketID.SET_RPM == 8
