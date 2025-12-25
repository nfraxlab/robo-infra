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


# =============================================================================
# Phase 5.8.4.1 - ODrive Comprehensive Tests
# =============================================================================


class TestODriveInitialization:
    """Comprehensive tests for ODrive initialization."""

    def test_default_channels(self) -> None:
        """Test default number of channels (axes)."""
        driver = ODriveDriver()
        assert driver._config.channels == 2

    def test_default_auto_connect(self) -> None:
        """Test auto_connect is disabled by default."""
        driver = ODriveDriver()
        assert driver._config.auto_connect is False

    def test_default_name(self) -> None:
        """Test default driver name."""
        driver = ODriveDriver()
        assert driver._config.name == "ODrive"

    def test_custom_name(self) -> None:
        """Test custom driver name."""
        driver = ODriveDriver(name="ODrive Pro")
        assert driver._config.name == "ODrive Pro"

    def test_simulation_state_init(self) -> None:
        """Test simulated state initialization."""
        driver = ODriveDriver()
        assert driver._sim_positions == {0: 0.0, 1: 0.0}
        assert driver._sim_velocities == {0: 0.0, 1: 0.0}
        assert driver._sim_states[0] == AxisState.IDLE
        assert driver._sim_states[1] == AxisState.IDLE

    def test_simulation_control_modes_init(self) -> None:
        """Test simulated control modes initialization."""
        driver = ODriveDriver()
        assert driver._sim_control_modes[0] == ControlMode.POSITION_CONTROL
        assert driver._sim_control_modes[1] == ControlMode.POSITION_CONTROL


class TestODriveAxisStateAdvanced:
    """Advanced tests for ODrive axis state control."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_all_axis_states_defined(self) -> None:
        """Test all axis states are properly defined."""
        assert AxisState.UNDEFINED == 0
        assert AxisState.IDLE == 1
        assert AxisState.STARTUP_SEQUENCE == 2
        assert AxisState.FULL_CALIBRATION_SEQUENCE == 3
        assert AxisState.MOTOR_CALIBRATION == 4
        assert AxisState.ENCODER_INDEX_SEARCH == 6
        assert AxisState.ENCODER_OFFSET_CALIBRATION == 7
        assert AxisState.CLOSED_LOOP_CONTROL == 8
        assert AxisState.LOCKIN_SPIN == 9
        assert AxisState.ENCODER_DIR_FIND == 10
        assert AxisState.HOMING == 11

    def test_calibrate_both_axes(self, driver: ODriveDriver) -> None:
        """Test calibrating both axes."""
        driver.calibrate(0)
        driver.calibrate(1)
        assert driver.get_axis_state(0) == AxisState.IDLE
        assert driver.get_axis_state(1) == AxisState.IDLE

    def test_state_transition_idle_to_closed_loop(self, driver: ODriveDriver) -> None:
        """Test state transition from idle to closed loop."""
        assert driver.get_axis_state(0) == AxisState.IDLE
        driver.set_closed_loop(0)
        assert driver.get_axis_state(0) == AxisState.CLOSED_LOOP_CONTROL


class TestODrivePositionControl:
    """Tests for ODrive position control mode."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_position_control_mode_set(self, driver: ODriveDriver) -> None:
        """Test position control mode is set."""
        driver.set_position(0, 5.0)
        assert driver.get_control_mode(0) == ControlMode.POSITION_CONTROL

    def test_position_negative(self, driver: ODriveDriver) -> None:
        """Test negative position values."""
        driver.set_position(0, -10.0)
        assert driver.get_encoder_position(0) == -10.0

    def test_position_with_velocity_feedforward(self, driver: ODriveDriver) -> None:
        """Test position with velocity feedforward."""
        driver.set_position(0, 5.0, velocity_feedforward=2.0)
        assert driver.get_encoder_position(0) == 5.0

    def test_position_with_torque_feedforward(self, driver: ODriveDriver) -> None:
        """Test position with torque feedforward."""
        driver.set_position(0, 3.0, torque_feedforward=0.5)
        assert driver.get_encoder_position(0) == 3.0

    def test_position_control_both_axes(self, driver: ODriveDriver) -> None:
        """Test position control on both axes."""
        driver.set_position(0, 10.0)
        driver.set_position(1, 20.0)
        assert driver.get_encoder_position(0) == 10.0
        assert driver.get_encoder_position(1) == 20.0


class TestODriveVelocityControl:
    """Tests for ODrive velocity control mode."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_velocity_control_mode_set(self, driver: ODriveDriver) -> None:
        """Test velocity control mode is set."""
        driver.set_velocity(0, 10.0)
        assert driver.get_control_mode(0) == ControlMode.VELOCITY_CONTROL

    def test_velocity_negative(self, driver: ODriveDriver) -> None:
        """Test negative velocity (reverse direction)."""
        driver.set_velocity(0, -5.0)
        assert driver.get_encoder_velocity(0) == -5.0

    def test_velocity_zero(self, driver: ODriveDriver) -> None:
        """Test zero velocity (stop)."""
        driver.set_velocity(0, 10.0)
        driver.set_velocity(0, 0.0)
        assert driver.get_encoder_velocity(0) == 0.0

    def test_velocity_control_both_axes(self, driver: ODriveDriver) -> None:
        """Test velocity control on both axes."""
        driver.set_velocity(0, 5.0)
        driver.set_velocity(1, -5.0)
        assert driver.get_encoder_velocity(0) == 5.0
        assert driver.get_encoder_velocity(1) == -5.0


class TestODriveTorqueControl:
    """Tests for ODrive torque control mode."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_torque_control_mode_set(self, driver: ODriveDriver) -> None:
        """Test torque control mode is set."""
        driver.set_torque(0, 1.0)
        assert driver.get_control_mode(0) == ControlMode.TORQUE_CONTROL

    def test_torque_enters_closed_loop(self, driver: ODriveDriver) -> None:
        """Test torque control enters closed loop."""
        driver.set_torque(0, 1.0)
        assert driver.get_axis_state(0) == AxisState.CLOSED_LOOP_CONTROL

    def test_torque_both_axes(self, driver: ODriveDriver) -> None:
        """Test torque control on both axes."""
        driver.set_torque(0, 1.5)
        driver.set_torque(1, 2.0)
        assert driver.get_control_mode(0) == ControlMode.TORQUE_CONTROL
        assert driver.get_control_mode(1) == ControlMode.TORQUE_CONTROL


class TestODriveCalibrationAdvanced:
    """Advanced tests for ODrive calibration."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_calibration_returns_to_idle(self, driver: ODriveDriver) -> None:
        """Test calibration returns to idle state."""
        driver.calibrate(0)
        assert driver.get_axis_state(0) == AxisState.IDLE

    def test_calibration_both_axes_sequential(self, driver: ODriveDriver) -> None:
        """Test sequential calibration of both axes."""
        driver.calibrate(0)
        driver.calibrate(1)
        assert driver.get_axis_state(0) == AxisState.IDLE
        assert driver.get_axis_state(1) == AxisState.IDLE


class TestODriveInputModes:
    """Tests for ODrive input modes."""

    def test_all_input_modes_defined(self) -> None:
        """Test all input modes are properly defined."""
        assert InputMode.INACTIVE == 0
        assert InputMode.PASSTHROUGH == 1
        assert InputMode.VEL_RAMP == 2
        assert InputMode.POS_FILTER == 3
        assert InputMode.MIX_CHANNELS == 4
        assert InputMode.TRAP_TRAJ == 5
        assert InputMode.TORQUE_RAMP == 6
        assert InputMode.MIRROR == 7
        assert InputMode.TUNING == 8


class TestODriveMotorTypes:
    """Tests for ODrive motor types."""

    def test_all_motor_types_defined(self) -> None:
        """Test all motor types are properly defined."""
        assert MotorType.HIGH_CURRENT == 0
        assert MotorType.GIMBAL == 2
        assert MotorType.ACIM == 3


class TestODriveEncoderModes:
    """Tests for ODrive encoder modes."""

    def test_all_encoder_modes_defined(self) -> None:
        """Test all encoder modes are properly defined."""
        assert EncoderMode.INCREMENTAL == 0
        assert EncoderMode.HALL == 1
        assert EncoderMode.SINCOS == 2
        assert EncoderMode.SPI_ABS_CUI == 256
        assert EncoderMode.SPI_ABS_AMS == 257
        assert EncoderMode.SPI_ABS_AEAT == 258
        assert EncoderMode.SPI_ABS_RLS == 259
        assert EncoderMode.SPI_ABS_MA732 == 260


class TestODriveConfigOptions:
    """Tests for ODrive configuration options."""

    def test_config_velocity_limit_range(self) -> None:
        """Test velocity limit configuration."""
        config = ODriveConfig(velocity_limit=100.0)
        assert config.velocity_limit == 100.0

    def test_config_current_limit_range(self) -> None:
        """Test current limit configuration."""
        config = ODriveConfig(current_limit=120.0)
        assert config.current_limit == 120.0

    def test_config_timeout_range(self) -> None:
        """Test timeout configuration."""
        config = ODriveConfig(timeout=30.0)
        assert config.timeout == 30.0

    def test_config_serial_number_format(self) -> None:
        """Test serial number configuration."""
        config = ODriveConfig(serial_number="ABCD1234EFGH")
        assert config.serial_number == "ABCD1234EFGH"


class TestODriveErrorsAdvanced:
    """Advanced tests for ODrive error handling."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_errors_structure(self, driver: ODriveDriver) -> None:
        """Test errors dictionary structure."""
        errors = driver.get_errors(0)
        assert "axis" in errors
        assert "motor" in errors
        assert "encoder" in errors
        assert "controller" in errors

    def test_errors_both_axes(self, driver: ODriveDriver) -> None:
        """Test getting errors for both axes."""
        errors0 = driver.get_errors(0)
        errors1 = driver.get_errors(1)
        assert errors0["axis"] == 0
        assert errors1["axis"] == 0

    def test_clear_errors_both_axes(self, driver: ODriveDriver) -> None:
        """Test clearing errors on both axes."""
        driver.clear_errors(0)
        driver.clear_errors(1)
        # Should not raise


class TestODriveStatusAdvanced:
    """Advanced tests for ODrive status."""

    @pytest.fixture
    def driver(self) -> ODriveDriver:
        """Create connected driver."""
        drv = ODriveDriver()
        drv.connect()
        return drv

    def test_status_structure(self, driver: ODriveDriver) -> None:
        """Test status dictionary structure."""
        status = driver.get_status()
        assert "connected" in status
        assert "serial_number" in status
        assert "simulation" in status
        assert "bus_voltage" in status
        assert "axes" in status

    def test_status_axes_structure(self, driver: ODriveDriver) -> None:
        """Test axes status structure."""
        status = driver.get_status()
        assert len(status["axes"]) == 2
        for axis_status in status["axes"]:
            assert "state" in axis_status
            assert "control_mode" in axis_status
            assert "position" in axis_status
            assert "velocity" in axis_status
            assert "errors" in axis_status
