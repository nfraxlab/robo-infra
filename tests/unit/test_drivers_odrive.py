"""Unit tests for ODrive brushless motor controller driver.

Tests the ODriveDriver class including connection, calibration,
control modes, and motion control.
"""

from __future__ import annotations

import os

import pytest


# Set simulation mode for tests
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.drivers.odrive import (
    AxisState,
    ControlMode,
    EncoderMode,
    InputMode,
    MotorType,
    ODriveConfig,
    ODriveDriver,
)


class TestODriveConfig:
    """Tests for ODriveConfig dataclass."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = ODriveConfig()
        assert config.serial_number is None
        assert config.timeout == 10.0
        assert config.velocity_limit == 50.0
        assert config.current_limit == 60.0

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = ODriveConfig(
            serial_number="12345678",
            timeout=5.0,
            velocity_limit=100.0,
            current_limit=80.0,
        )
        assert config.serial_number == "12345678"
        assert config.timeout == 5.0
        assert config.velocity_limit == 100.0
        assert config.current_limit == 80.0


class TestODriveDriverLifecycle:
    """Tests for ODriveDriver lifecycle methods."""

    def test_init_default(self) -> None:
        """Test driver initialization with defaults."""
        driver = ODriveDriver()
        assert driver.simulation is True
        assert driver.serial_number is None

    def test_init_with_serial(self) -> None:
        """Test driver initialization with serial number."""
        driver = ODriveDriver(serial_number="ABCD1234")
        assert driver._serial_number == "ABCD1234"

    def test_init_with_config(self) -> None:
        """Test driver initialization with config."""
        config = ODriveConfig(serial_number="TEST123", velocity_limit=75.0)
        driver = ODriveDriver(config=config)
        assert driver._odrive_config.velocity_limit == 75.0

    def test_connect_simulation(self) -> None:
        """Test connection in simulation mode."""
        driver = ODriveDriver()
        driver.connect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.CONNECTED

    def test_disconnect(self) -> None:
        """Test disconnection."""
        driver = ODriveDriver()
        driver.connect()
        driver.disconnect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.DISCONNECTED


class TestODriveDriverAxisState:
    """Tests for ODriveDriver axis state control."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_calibrate(self, driver: ODriveDriver) -> None:
        """Test axis calibration."""
        driver.calibrate(0)
        # In simulation, this should complete without error
        assert driver.get_axis_state(0) == AxisState.IDLE

    def test_set_closed_loop(self, driver: ODriveDriver) -> None:
        """Test entering closed-loop control."""
        driver.set_closed_loop(0)
        assert driver.get_axis_state(0) == AxisState.CLOSED_LOOP_CONTROL

    def test_set_idle(self, driver: ODriveDriver) -> None:
        """Test setting axis to idle."""
        driver.set_closed_loop(0)
        driver.set_idle(0)
        assert driver.get_axis_state(0) == AxisState.IDLE
        assert driver._sim_velocities[0] == 0.0

    def test_get_axis_state_both_axes(self, driver: ODriveDriver) -> None:
        """Test getting state for both axes."""
        driver.set_closed_loop(0)
        driver.set_idle(1)
        assert driver.get_axis_state(0) == AxisState.CLOSED_LOOP_CONTROL
        assert driver.get_axis_state(1) == AxisState.IDLE

    def test_invalid_axis(self, driver: ODriveDriver) -> None:
        """Test invalid axis number."""
        with pytest.raises(ValueError):
            driver._get_axis(2)


class TestODriveDriverControlMode:
    """Tests for ODriveDriver control mode."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_set_control_mode(self, driver: ODriveDriver) -> None:
        """Test setting control mode."""
        driver.set_control_mode(0, ControlMode.VELOCITY_CONTROL)
        assert driver.get_control_mode(0) == ControlMode.VELOCITY_CONTROL

    def test_set_control_mode_torque(self, driver: ODriveDriver) -> None:
        """Test setting torque control mode."""
        driver.set_control_mode(0, ControlMode.TORQUE_CONTROL)
        assert driver.get_control_mode(0) == ControlMode.TORQUE_CONTROL


class TestODriveDriverMotionControl:
    """Tests for ODriveDriver motion control."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_set_velocity(self, driver: ODriveDriver) -> None:
        """Test setting velocity."""
        driver.set_velocity(0, 10.0)
        assert driver._sim_velocities[0] == 10.0
        assert driver._sim_states[0] == AxisState.CLOSED_LOOP_CONTROL
        assert driver._sim_control_modes[0] == ControlMode.VELOCITY_CONTROL

    def test_set_position(self, driver: ODriveDriver) -> None:
        """Test setting position."""
        driver.set_position(0, 5.0)
        assert driver._sim_positions[0] == 5.0
        assert driver._sim_states[0] == AxisState.CLOSED_LOOP_CONTROL
        assert driver._sim_control_modes[0] == ControlMode.POSITION_CONTROL

    def test_set_torque(self, driver: ODriveDriver) -> None:
        """Test setting torque."""
        driver.set_torque(0, 2.5)
        assert driver._sim_states[0] == AxisState.CLOSED_LOOP_CONTROL
        assert driver._sim_control_modes[0] == ControlMode.TORQUE_CONTROL

    def test_set_velocity_with_feedforward(self, driver: ODriveDriver) -> None:
        """Test setting velocity with torque feedforward."""
        driver.set_velocity(0, 5.0, torque_feedforward=0.5)
        assert driver._sim_velocities[0] == 5.0


class TestODriveDriverEncoderFeedback:
    """Tests for ODriveDriver encoder feedback."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_get_encoder_position(self, driver: ODriveDriver) -> None:
        """Test getting encoder position."""
        driver._sim_positions[0] = 12.5
        pos = driver.get_encoder_position(0)
        assert pos == 12.5

    def test_get_encoder_velocity(self, driver: ODriveDriver) -> None:
        """Test getting encoder velocity."""
        driver._sim_velocities[0] = 8.0
        vel = driver.get_encoder_velocity(0)
        assert vel == 8.0

    def test_set_encoder_position(self, driver: ODriveDriver) -> None:
        """Test setting encoder position (homing)."""
        driver.set_encoder_position(0, 0.0)
        assert driver._sim_positions[0] == 0.0


class TestODriveDriverMotorFeedback:
    """Tests for ODriveDriver motor feedback."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_get_motor_current(self, driver: ODriveDriver) -> None:
        """Test getting motor current."""
        iq, id = driver.get_motor_current(0)
        assert iq == 0.0
        assert id == 0.0

    def test_get_bus_voltage(self, driver: ODriveDriver) -> None:
        """Test getting bus voltage."""
        voltage = driver.get_bus_voltage()
        assert voltage == 24.0  # Simulated value


class TestODriveDriverConfiguration:
    """Tests for ODriveDriver configuration."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_set_velocity_limit(self, driver: ODriveDriver) -> None:
        """Test setting velocity limit."""
        driver.set_velocity_limit(0, 100.0)
        # In simulation, this is a no-op but should not raise

    def test_set_current_limit(self, driver: ODriveDriver) -> None:
        """Test setting current limit."""
        driver.set_current_limit(0, 50.0)
        # In simulation, this is a no-op but should not raise

    def test_save_configuration(self, driver: ODriveDriver) -> None:
        """Test saving configuration."""
        driver.save_configuration()
        # In simulation, this is a no-op but should not raise


class TestODriveDriverErrors:
    """Tests for ODriveDriver error handling."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_get_errors(self, driver: ODriveDriver) -> None:
        """Test getting error flags."""
        errors = driver.get_errors(0)
        assert errors["axis"] == 0
        assert errors["motor"] == 0
        assert errors["encoder"] == 0
        assert errors["controller"] == 0

    def test_clear_errors(self, driver: ODriveDriver) -> None:
        """Test clearing errors."""
        driver.clear_errors(0)
        # Should not raise in simulation


class TestODriveDriverStatus:
    """Tests for ODriveDriver status methods."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_get_status(self, driver: ODriveDriver) -> None:
        """Test getting overall status."""
        driver.set_position(0, 10.0)
        driver.set_velocity(1, 5.0)

        status = driver.get_status()

        assert status["connected"] is True
        assert status["simulation"] is True
        assert status["bus_voltage"] == 24.0
        assert len(status["axes"]) == 2
        assert status["axes"][0]["position"] == 10.0
        assert status["axes"][1]["velocity"] == 5.0


class TestODriveDriverChannelInterface:
    """Tests for ODriveDriver channel interface (Driver ABC)."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_write_channel(self, driver: ODriveDriver) -> None:
        """Test writing to channel."""
        driver._write_channel(0, 0.5)  # 50% of velocity limit
        assert driver._sim_velocities[0] == 0.5 * 50.0  # 25.0

    def test_read_channel(self, driver: ODriveDriver) -> None:
        """Test reading from channel."""
        driver._sim_positions[0] = 7.5
        value = driver._read_channel(0)
        assert value == 7.5


class TestODriveEnums:
    """Tests for ODrive enums."""

    def test_axis_state_values(self) -> None:
        """Test axis state enum values."""
        assert AxisState.UNDEFINED == 0
        assert AxisState.IDLE == 1
        assert AxisState.FULL_CALIBRATION_SEQUENCE == 3
        assert AxisState.CLOSED_LOOP_CONTROL == 8

    def test_control_mode_values(self) -> None:
        """Test control mode enum values."""
        assert ControlMode.VOLTAGE_CONTROL == 0
        assert ControlMode.TORQUE_CONTROL == 1
        assert ControlMode.VELOCITY_CONTROL == 2
        assert ControlMode.POSITION_CONTROL == 3

    def test_input_mode_values(self) -> None:
        """Test input mode enum values."""
        assert InputMode.INACTIVE == 0
        assert InputMode.PASSTHROUGH == 1
        assert InputMode.TRAP_TRAJ == 5

    def test_motor_type_values(self) -> None:
        """Test motor type enum values."""
        assert MotorType.HIGH_CURRENT == 0
        assert MotorType.GIMBAL == 2

    def test_encoder_mode_values(self) -> None:
        """Test encoder mode enum values."""
        assert EncoderMode.INCREMENTAL == 0
        assert EncoderMode.HALL == 1
