"""ODrive motor controller driver.

This module provides a driver for the ODrive brushless motor controller,
which supports high-performance FOC (Field Oriented Control) for BLDC motors.

The ODrive is commonly used in robotics for precision motor control with
features like position, velocity, and torque control modes.

Example:
    >>> from robo_infra.drivers.odrive import ODriveDriver
    >>>
    >>> # Connect to ODrive (auto-discovers first available)
    >>> driver = ODriveDriver()
    >>> driver.connect()
    >>>
    >>> # Calibrate motors
    >>> driver.calibrate(axis=0)
    >>>
    >>> # Set velocity mode and run
    >>> driver.set_velocity(axis=0, velocity=10.0)  # 10 turns/sec
    >>>
    >>> # Read encoder position
    >>> pos = driver.get_encoder_position(axis=0)
    >>> print(f"Position: {pos} turns")
    >>>
    >>> # Position control
    >>> driver.set_position(axis=0, position=5.0)  # Move to 5 turns
    >>>
    >>> driver.disconnect()

Hardware Reference:
    ODrive v3.x:
        - 2 motor axes
        - 12-56V input voltage
        - Up to 120A peak per axis
        - USB, UART, CAN, SPI interfaces

    ODrive S1/Pro:
        - 1 motor axis per board
        - 12-58V (S1) / 12-56V (Pro)
        - Up to 120A continuous
        - USB, UART, CAN interfaces
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import CommunicationError, HardwareNotFoundError


logger = logging.getLogger(__name__)


# =============================================================================
# ODrive Enums
# =============================================================================


class AxisState(IntEnum):
    """ODrive axis states."""

    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13


class ControlMode(IntEnum):
    """ODrive control modes."""

    VOLTAGE_CONTROL = 0
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3


class InputMode(IntEnum):
    """ODrive input modes."""

    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7
    TUNING = 8


class MotorType(IntEnum):
    """ODrive motor types."""

    HIGH_CURRENT = 0
    GIMBAL = 2
    ACIM = 3


class EncoderMode(IntEnum):
    """ODrive encoder modes."""

    INCREMENTAL = 0
    HALL = 1
    SINCOS = 2
    SPI_ABS_CUI = 256
    SPI_ABS_AMS = 257
    SPI_ABS_AEAT = 258
    SPI_ABS_RLS = 259
    SPI_ABS_MA732 = 260


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class ODriveConfig:
    """Configuration for ODrive driver.

    Attributes:
        serial_number: ODrive serial number (None for auto-discover).
        timeout: Connection timeout in seconds.
        velocity_limit: Maximum velocity in turns/sec.
        current_limit: Maximum current in Amps.
    """

    serial_number: str | None = None
    timeout: float = 10.0
    velocity_limit: float = 50.0
    current_limit: float = 60.0


# =============================================================================
# ODrive Driver
# =============================================================================


@register_driver("odrive")
class ODriveDriver(Driver):
    """Driver for ODrive brushless motor controller.

    Supports ODrive v3.x, S1, and Pro hardware.

    Example:
        >>> driver = ODriveDriver(serial_number="12345678")
        >>> driver.connect()
        >>> driver.calibrate(0)
        >>> driver.set_velocity(0, 5.0)
    """

    def __init__(
        self,
        serial_number: str | None = None,
        config: ODriveConfig | None = None,
        simulation: bool | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize ODrive driver.

        Args:
            serial_number: ODrive serial number (None for auto-discover).
            config: Driver configuration.
            simulation: If True, use simulation mode.
            name: Optional human-readable name.
        """
        if simulation is None:
            simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

        super().__init__(
            config=DriverConfig(
                name=name or "ODrive",
                channels=2,  # 2 axes
                auto_connect=False,
            )
        )

        self._serial_number = serial_number
        self._odrive_config = config or ODriveConfig(serial_number=serial_number)
        self._simulation = simulation

        # ODrive object (set on connect)
        self._odrv: Any = None

        # Simulated state
        self._sim_positions: dict[int, float] = {0: 0.0, 1: 0.0}
        self._sim_velocities: dict[int, float] = {0: 0.0, 1: 0.0}
        self._sim_states: dict[int, AxisState] = {0: AxisState.IDLE, 1: AxisState.IDLE}
        self._sim_control_modes: dict[int, ControlMode] = {
            0: ControlMode.POSITION_CONTROL,
            1: ControlMode.POSITION_CONTROL,
        }

    @property
    def simulation(self) -> bool:
        """Whether running in simulation mode."""
        return self._simulation

    @property
    def serial_number(self) -> str | None:
        """ODrive serial number."""
        return self._serial_number

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the ODrive.

        Raises:
            HardwareNotFoundError: If ODrive is not found.
            CommunicationError: If connection fails.
        """
        if self._simulation:
            logger.info("ODrive connecting in simulation mode")
            self._state = DriverState.CONNECTED
            return

        try:
            import odrive

            logger.info("Searching for ODrive...")
            if self._serial_number:
                self._odrv = odrive.find_any(
                    serial_number=self._serial_number,
                    timeout=self._odrive_config.timeout,
                )
            else:
                self._odrv = odrive.find_any(timeout=self._odrive_config.timeout)

            if self._odrv is None:
                raise HardwareNotFoundError("ODrive not found")

            self._serial_number = str(self._odrv.serial_number)
            logger.info("Connected to ODrive %s", self._serial_number)

        except ImportError as e:
            raise HardwareNotFoundError(
                "ODrive library not installed. Install with: pip install odrive"
            ) from e
        except Exception as e:
            raise CommunicationError(f"Failed to connect to ODrive: {e}") from e

        self._state = DriverState.CONNECTED

    def disconnect(self) -> None:
        """Disconnect from the ODrive."""
        if not self._simulation and self._odrv is not None:
            # Set both axes to idle
            try:
                self.set_idle(0)
                self.set_idle(1)
            except Exception as e:
                logger.warning("Error setting axes to idle: %s", e)

        self._odrv = None
        self._state = DriverState.DISCONNECTED
        logger.info("ODrive disconnected")

    # -------------------------------------------------------------------------
    # Axis State Control
    # -------------------------------------------------------------------------

    def _get_axis(self, axis: int) -> Any:
        """Get axis object.

        Args:
            axis: Axis number (0 or 1).

        Returns:
            Axis object.

        Raises:
            ValueError: If axis is invalid.
        """
        if axis not in (0, 1):
            raise ValueError(f"Invalid axis {axis}. Must be 0 or 1.")

        if self._simulation:
            return None

        if self._odrv is None:
            raise CommunicationError("Not connected to ODrive")

        return self._odrv.axis0 if axis == 0 else self._odrv.axis1

    def calibrate(self, axis: int) -> None:
        """Run full calibration sequence on an axis.

        This will calibrate the motor and encoder. The motor may spin during
        calibration, ensure the mechanical system is free to move.

        Args:
            axis: Axis number (0 or 1).
        """
        if self._simulation:
            logger.info("ODrive simulating calibration on axis %d", axis)
            time.sleep(1.0)  # Simulate calibration time
            self._sim_states[axis] = AxisState.IDLE
            return

        ax = self._get_axis(axis)
        ax.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

        # Wait for calibration to complete
        while ax.current_state != AxisState.IDLE:
            time.sleep(0.1)

        if ax.motor.is_calibrated and ax.encoder.is_ready:
            logger.info("Axis %d calibration complete", axis)
        else:
            raise CommunicationError(f"Axis {axis} calibration failed")

    def set_closed_loop(self, axis: int) -> None:
        """Enter closed-loop control mode.

        Args:
            axis: Axis number (0 or 1).
        """
        if self._simulation:
            self._sim_states[axis] = AxisState.CLOSED_LOOP_CONTROL
            return

        ax = self._get_axis(axis)
        ax.requested_state = AxisState.CLOSED_LOOP_CONTROL

        # Wait for state transition
        time.sleep(0.1)
        if ax.current_state != AxisState.CLOSED_LOOP_CONTROL:
            raise CommunicationError(f"Failed to enter closed-loop on axis {axis}")

    def set_idle(self, axis: int) -> None:
        """Set axis to idle state.

        Args:
            axis: Axis number (0 or 1).
        """
        if self._simulation:
            self._sim_states[axis] = AxisState.IDLE
            self._sim_velocities[axis] = 0.0
            return

        ax = self._get_axis(axis)
        ax.requested_state = AxisState.IDLE

    def get_axis_state(self, axis: int) -> AxisState:
        """Get the current axis state.

        Args:
            axis: Axis number (0 or 1).

        Returns:
            Current axis state.
        """
        if self._simulation:
            return self._sim_states[axis]

        ax = self._get_axis(axis)
        return AxisState(ax.current_state)

    # -------------------------------------------------------------------------
    # Control Mode
    # -------------------------------------------------------------------------

    def set_control_mode(self, axis: int, mode: ControlMode) -> None:
        """Set the control mode for an axis.

        Args:
            axis: Axis number (0 or 1).
            mode: Control mode (voltage, torque, velocity, position).
        """
        if self._simulation:
            self._sim_control_modes[axis] = mode
            return

        ax = self._get_axis(axis)
        ax.controller.config.control_mode = mode

    def get_control_mode(self, axis: int) -> ControlMode:
        """Get the current control mode.

        Args:
            axis: Axis number (0 or 1).

        Returns:
            Current control mode.
        """
        if self._simulation:
            return self._sim_control_modes[axis]

        ax = self._get_axis(axis)
        return ControlMode(ax.controller.config.control_mode)

    # -------------------------------------------------------------------------
    # Motion Control
    # -------------------------------------------------------------------------

    def set_velocity(self, axis: int, velocity: float, torque_feedforward: float = 0.0) -> None:
        """Set velocity setpoint.

        Args:
            axis: Axis number (0 or 1).
            velocity: Target velocity in turns/sec.
            torque_feedforward: Feedforward torque in Nm.
        """
        if self._simulation:
            if self._sim_states[axis] != AxisState.CLOSED_LOOP_CONTROL:
                self.set_closed_loop(axis)
            self._sim_control_modes[axis] = ControlMode.VELOCITY_CONTROL
            self._sim_velocities[axis] = velocity
            logger.debug("ODrive axis %d velocity: %.2f turns/s", axis, velocity)
            return

        ax = self._get_axis(axis)
        ax.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        ax.controller.input_vel = velocity
        ax.controller.input_torque = torque_feedforward

    def set_position(
        self,
        axis: int,
        position: float,
        velocity_feedforward: float = 0.0,
        torque_feedforward: float = 0.0,
    ) -> None:
        """Set position setpoint.

        Args:
            axis: Axis number (0 or 1).
            position: Target position in turns.
            velocity_feedforward: Feedforward velocity in turns/sec.
            torque_feedforward: Feedforward torque in Nm.
        """
        if self._simulation:
            if self._sim_states[axis] != AxisState.CLOSED_LOOP_CONTROL:
                self.set_closed_loop(axis)
            self._sim_control_modes[axis] = ControlMode.POSITION_CONTROL
            self._sim_positions[axis] = position
            logger.debug("ODrive axis %d position: %.2f turns", axis, position)
            return

        ax = self._get_axis(axis)
        ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
        ax.controller.input_pos = position
        ax.controller.input_vel = velocity_feedforward
        ax.controller.input_torque = torque_feedforward

    def set_torque(self, axis: int, torque: float) -> None:
        """Set torque setpoint.

        Args:
            axis: Axis number (0 or 1).
            torque: Target torque in Nm.
        """
        if self._simulation:
            if self._sim_states[axis] != AxisState.CLOSED_LOOP_CONTROL:
                self.set_closed_loop(axis)
            self._sim_control_modes[axis] = ControlMode.TORQUE_CONTROL
            logger.debug("ODrive axis %d torque: %.2f Nm", axis, torque)
            return

        ax = self._get_axis(axis)
        ax.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        ax.controller.input_torque = torque

    # -------------------------------------------------------------------------
    # Encoder Feedback
    # -------------------------------------------------------------------------

    def get_encoder_position(self, axis: int) -> float:
        """Get encoder position.

        Args:
            axis: Axis number (0 or 1).

        Returns:
            Position in turns.
        """
        if self._simulation:
            return self._sim_positions[axis]

        ax = self._get_axis(axis)
        return ax.encoder.pos_estimate

    def get_encoder_velocity(self, axis: int) -> float:
        """Get encoder velocity.

        Args:
            axis: Axis number (0 or 1).

        Returns:
            Velocity in turns/sec.
        """
        if self._simulation:
            return self._sim_velocities[axis]

        ax = self._get_axis(axis)
        return ax.encoder.vel_estimate

    def set_encoder_position(self, axis: int, position: float) -> None:
        """Set encoder position (for homing).

        Args:
            axis: Axis number (0 or 1).
            position: New position value in turns.
        """
        if self._simulation:
            self._sim_positions[axis] = position
            return

        ax = self._get_axis(axis)
        ax.encoder.set_linear_count(int(position * ax.encoder.config.cpr))

    # -------------------------------------------------------------------------
    # Motor Feedback
    # -------------------------------------------------------------------------

    def get_motor_current(self, axis: int) -> tuple[float, float]:
        """Get motor current (Iq and Id).

        Args:
            axis: Axis number (0 or 1).

        Returns:
            Tuple of (Iq, Id) currents in Amps.
        """
        if self._simulation:
            return (0.0, 0.0)

        ax = self._get_axis(axis)
        return (ax.motor.current_control.Iq_measured, ax.motor.current_control.Id_measured)

    def get_bus_voltage(self) -> float:
        """Get DC bus voltage.

        Returns:
            Bus voltage in Volts.
        """
        if self._simulation:
            return 24.0  # Simulated voltage

        if self._odrv is None:
            raise CommunicationError("Not connected to ODrive")

        return self._odrv.vbus_voltage

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------

    def set_velocity_limit(self, axis: int, limit: float) -> None:
        """Set velocity limit.

        Args:
            axis: Axis number (0 or 1).
            limit: Maximum velocity in turns/sec.
        """
        if self._simulation:
            return

        ax = self._get_axis(axis)
        ax.controller.config.vel_limit = limit

    def set_current_limit(self, axis: int, limit: float) -> None:
        """Set current limit.

        Args:
            axis: Axis number (0 or 1).
            limit: Maximum current in Amps.
        """
        if self._simulation:
            return

        ax = self._get_axis(axis)
        ax.motor.config.current_lim = limit

    def save_configuration(self) -> None:
        """Save configuration to ODrive NVRAM."""
        if self._simulation:
            logger.info("ODrive save configuration (simulated)")
            return

        if self._odrv is None:
            raise CommunicationError("Not connected to ODrive")

        self._odrv.save_configuration()
        logger.info("ODrive configuration saved")

    # -------------------------------------------------------------------------
    # Errors
    # -------------------------------------------------------------------------

    def get_errors(self, axis: int) -> dict:
        """Get error flags for an axis.

        Args:
            axis: Axis number (0 or 1).

        Returns:
            Dictionary of error flags.
        """
        if self._simulation:
            return {
                "axis": 0,
                "motor": 0,
                "encoder": 0,
                "controller": 0,
            }

        ax = self._get_axis(axis)
        return {
            "axis": ax.error,
            "motor": ax.motor.error,
            "encoder": ax.encoder.error,
            "controller": ax.controller.error,
        }

    def clear_errors(self, axis: int) -> None:
        """Clear errors on an axis.

        Args:
            axis: Axis number (0 or 1).
        """
        if self._simulation:
            return

        ax = self._get_axis(axis)
        ax.error = 0
        ax.motor.error = 0
        ax.encoder.error = 0
        ax.controller.error = 0

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def get_status(self) -> dict:
        """Get overall driver status.

        Returns:
            Dictionary with status information.
        """
        return {
            "connected": self._state == DriverState.CONNECTED,
            "serial_number": self._serial_number,
            "simulation": self._simulation,
            "bus_voltage": self.get_bus_voltage(),
            "axes": [
                {
                    "state": self.get_axis_state(i).name,
                    "control_mode": self.get_control_mode(i).name,
                    "position": self.get_encoder_position(i),
                    "velocity": self.get_encoder_velocity(i),
                    "errors": self.get_errors(i),
                }
                for i in range(2)
            ],
        }

    # -------------------------------------------------------------------------
    # Driver Abstract Methods
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write to channel (sets velocity)."""
        self.set_velocity(channel, value * self._odrive_config.velocity_limit)

    def _read_channel(self, channel: int) -> float:
        """Read from channel (returns normalized position)."""
        return self.get_encoder_position(channel)
