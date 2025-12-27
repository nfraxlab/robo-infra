"""VESC motor controller driver.

This module provides a driver for VESC (Vedder Electronic Speed Controller)
brushless motor controllers. VESC is open-source and supports various control
modes including duty cycle, current, RPM, and position control.

Example:
    >>> from robo_infra.drivers.vesc import VESCDriver
    >>>
    >>> # Connect via UART
    >>> driver = VESCDriver(port="/dev/ttyUSB0")
    >>> driver.connect()
    >>>
    >>> # Set duty cycle (throttle)
    >>> driver.set_duty_cycle(0.5)  # 50% throttle
    >>>
    >>> # RPM control
    >>> driver.set_rpm(5000)  # 5000 ERPM
    >>>
    >>> # Current control (torque)
    >>> driver.set_current(10.0)  # 10 Amps
    >>>
    >>> # Read telemetry
    >>> state = driver.get_state()
    >>> print(f"RPM: {state['rpm']}, Voltage: {state['voltage']}V")
    >>>
    >>> driver.disconnect()

Hardware Reference:
    VESC 4.x/6.x:
        - 8-60V input voltage (model dependent)
        - Up to 250A peak current (model dependent)
        - FOC or BLDC control
        - UART, CAN, PPM, ADC interfaces

Protocol Reference:
    PyVESC library is used for communication:
    https://github.com/LiamBindle/PyVESC
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

from robo_infra.core.bus import SerialBus, SerialConfig
from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import CommunicationError, HardwareNotFoundError


logger = logging.getLogger(__name__)


# =============================================================================
# VESC Enums
# =============================================================================


class VESCFaultCode(IntEnum):
    """VESC fault codes."""

    NONE = 0
    OVER_VOLTAGE = 1
    UNDER_VOLTAGE = 2
    DRV = 3
    ABS_OVER_CURRENT = 4
    OVER_TEMP_FET = 5
    OVER_TEMP_MOTOR = 6
    GATE_DRIVER_OVER_VOLTAGE = 7
    GATE_DRIVER_UNDER_VOLTAGE = 8
    MCU_UNDER_VOLTAGE = 9
    BOOTING_FROM_WATCHDOG_RESET = 10
    ENCODER_SPI = 11
    ENCODER_SINCOS_BELOW_MIN_AMPLITUDE = 12
    ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE = 13
    FLASH_CORRUPTION = 14
    HIGH_OFFSET_CURRENT_SENSOR_1 = 15
    HIGH_OFFSET_CURRENT_SENSOR_2 = 16
    HIGH_OFFSET_CURRENT_SENSOR_3 = 17
    UNBALANCED_CURRENTS = 18
    BRK = 19
    RESOLVER_LOT = 20
    RESOLVER_DOS = 21
    RESOLVER_LOS = 22
    FLASH_CORRUPTION_APP_CFG = 23
    FLASH_CORRUPTION_MC_CFG = 24
    ENCODER_NO_MAGNET = 25


class VESCControlMode(IntEnum):
    """VESC control modes."""

    DUTY_CYCLE = 0
    SPEED = 1
    CURRENT = 2
    CURRENT_BRAKE = 3
    POSITION = 4


# =============================================================================
# VESC Packet Types
# =============================================================================


class VESCPacketID(IntEnum):
    """VESC command/response packet IDs."""

    FW_VERSION = 0
    JUMP_TO_BOOTLOADER = 1
    ERASE_NEW_APP = 2
    WRITE_NEW_APP_DATA = 3
    GET_VALUES = 4
    SET_DUTY = 5
    SET_CURRENT = 6
    SET_CURRENT_BRAKE = 7
    SET_RPM = 8
    SET_POS = 9
    SET_HANDBRAKE = 10
    SET_DETECT = 11
    SET_SERVO_POS = 12
    SET_MCCONF = 13
    GET_MCCONF = 14
    GET_MCCONF_DEFAULT = 15
    SET_APPCONF = 16
    GET_APPCONF = 17
    GET_APPCONF_DEFAULT = 18
    SAMPLE_PRINT = 19
    TERMINAL_CMD = 20
    PRINT = 21
    ROTOR_POSITION = 22
    EXPERIMENT_SAMPLE = 23
    DETECT_MOTOR_PARAM = 24
    DETECT_MOTOR_R_L = 25
    DETECT_MOTOR_FLUX_LINKAGE = 26
    DETECT_ENCODER = 27
    DETECT_HALL_FOC = 28
    REBOOT = 29
    ALIVE = 30
    GET_DECODED_PPM = 31
    GET_DECODED_ADC = 32
    GET_DECODED_CHUK = 33
    FORWARD_CAN = 34
    SET_CHUCK_DATA = 35
    CUSTOM_APP_DATA = 36
    NRF_START_PAIRING = 37
    GPD_SET_FSW = 38
    GPD_BUFFER_NOTIFY = 39
    GPD_BUFFER_SIZE_LEFT = 40
    GPD_FILL_BUFFER = 41
    GPD_OUTPUT_SAMPLE = 42
    GPD_SET_MODE = 43
    GPD_FILL_BUFFER_INT8 = 44
    GPD_FILL_BUFFER_INT16 = 45
    GPD_SET_BUFFER_INT_SCALE = 46
    GET_VALUES_SELECTIVE = 50
    GET_IMU_DATA = 65


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class VESCConfig:
    """Configuration for VESC driver.

    Attributes:
        port: Serial port (e.g., /dev/ttyUSB0).
        baudrate: UART baud rate.
        timeout: Communication timeout in seconds.
        current_limit: Maximum motor current in Amps.
        rpm_limit: Maximum ERPM (electrical RPM).
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout: float = 0.5
    current_limit: float = 60.0
    rpm_limit: int = 100000


# =============================================================================
# VESC State Data
# =============================================================================


@dataclass
class VESCState:
    """VESC telemetry state.

    Attributes:
        temp_mos: MOSFET temperature in Celsius.
        temp_motor: Motor temperature in Celsius.
        avg_motor_current: Average motor current in Amps.
        avg_input_current: Average input current in Amps.
        avg_id: Average Id (direct current) in Amps.
        avg_iq: Average Iq (quadrature current) in Amps.
        duty_cycle: Current duty cycle (0.0 to 1.0).
        rpm: Electrical RPM.
        v_in: Input voltage in Volts.
        amp_hours: Amp-hours used.
        amp_hours_charged: Amp-hours charged (regen).
        watt_hours: Watt-hours used.
        watt_hours_charged: Watt-hours charged (regen).
        tachometer: Tachometer value (half steps).
        tachometer_abs: Absolute tachometer value.
        fault: Current fault code.
        pid_pos: Current PID position (if in position mode).
    """

    temp_mos: float = 0.0
    temp_motor: float = 0.0
    avg_motor_current: float = 0.0
    avg_input_current: float = 0.0
    avg_id: float = 0.0
    avg_iq: float = 0.0
    duty_cycle: float = 0.0
    rpm: int = 0
    v_in: float = 0.0
    amp_hours: float = 0.0
    amp_hours_charged: float = 0.0
    watt_hours: float = 0.0
    watt_hours_charged: float = 0.0
    tachometer: int = 0
    tachometer_abs: int = 0
    fault: VESCFaultCode = VESCFaultCode.NONE
    pid_pos: float = 0.0


# =============================================================================
# VESC Driver
# =============================================================================


@register_driver("vesc")
class VESCDriver(Driver):
    """Driver for VESC brushless motor controller.

    Supports VESC 4.x, 6.x, and compatible hardware.

    Example:
        >>> driver = VESCDriver(port="/dev/ttyUSB0")
        >>> driver.connect()
        >>> driver.set_duty_cycle(0.5)  # 50% throttle
        >>> state = driver.get_state()
        >>> print(f"RPM: {state.rpm}")
    """

    def __init__(
        self,
        port: str | None = None,
        config: VESCConfig | None = None,
        simulation: bool | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize VESC driver.

        Args:
            port: Serial port path.
            config: Driver configuration.
            simulation: If True, use simulation mode.
            name: Optional human-readable name.
        """
        if simulation is None:
            simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

        super().__init__(
            config=DriverConfig(
                name=name or "VESC",
                channels=1,
                auto_connect=False,
            )
        )

        self._vesc_config = config or VESCConfig(port=port or "/dev/ttyUSB0")
        self._simulation = simulation

        # Serial connection
        self._serial: SerialBus | None = None

        # pyvesc import (lazy load)
        self._pyvesc: Any = None

        # Simulated state
        self._sim_state = VESCState()
        self._sim_control_mode = VESCControlMode.DUTY_CYCLE
        self._sim_setpoint: float = 0.0

    @property
    def simulation(self) -> bool:
        """Whether running in simulation mode."""
        return self._simulation

    @property
    def port(self) -> str:
        """Serial port."""
        return self._vesc_config.port

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the VESC.

        Raises:
            HardwareNotFoundError: If VESC is not found.
            CommunicationError: If connection fails.
        """
        if self._simulation:
            logger.info("VESC connecting in simulation mode")
            self._state = DriverState.CONNECTED
            return

        try:
            # Try to import pyvesc
            import pyvesc

            self._pyvesc = pyvesc
        except ImportError as e:
            raise HardwareNotFoundError(
                "pyvesc library not installed. Install with: pip install pyvesc"
            ) from e

        try:
            self._serial = SerialBus(
                SerialConfig(
                    port=self._vesc_config.port,
                    baudrate=self._vesc_config.baudrate,
                    timeout=self._vesc_config.timeout,
                )
            )
            self._serial.connect()

            # Verify connection by getting firmware version
            fw = self.get_firmware_version()
            logger.info("Connected to VESC firmware %d.%d", fw[0], fw[1])

        except Exception as e:
            raise CommunicationError(f"Failed to connect to VESC: {e}") from e

        self._state = DriverState.CONNECTED

    def disconnect(self) -> None:
        """Disconnect from the VESC."""
        # Set motor to idle
        if self._state == DriverState.CONNECTED:
            try:
                self.set_duty_cycle(0.0)
            except Exception as e:
                logger.warning("Error setting VESC to idle: %s", e)

        if self._serial is not None:
            self._serial.disconnect()
            self._serial = None

        self._state = DriverState.DISCONNECTED
        logger.info("VESC disconnected")

    # -------------------------------------------------------------------------
    # Low-level Communication
    # -------------------------------------------------------------------------

    def _send_packet(self, packet: Any) -> None:
        """Send a packet to the VESC.

        Args:
            packet: pyvesc packet object.
        """
        if self._simulation:
            return

        if self._serial is None:
            raise CommunicationError("Not connected to VESC")

        data = self._pyvesc.encode(packet)
        self._serial.write(data)

    def _receive_packet(self) -> Any:
        """Receive a packet from the VESC.

        Returns:
            Decoded packet or None if no packet received.
        """
        if self._simulation:
            return None

        if self._serial is None:
            raise CommunicationError("Not connected to VESC")

        # Read response
        data = self._serial.read(256)
        if not data:
            return None

        # Decode packet
        return self._pyvesc.decode(data)

    # -------------------------------------------------------------------------
    # Firmware Info
    # -------------------------------------------------------------------------

    def get_firmware_version(self) -> tuple[int, int]:
        """Get VESC firmware version.

        Returns:
            Tuple of (major, minor) version numbers.
        """
        if self._simulation:
            return (5, 3)  # Simulated version

        self._send_packet(self._pyvesc.GetFirmwareVersion())
        response = self._receive_packet()

        if response and hasattr(response, "fw_version_major"):
            return (response.fw_version_major, response.fw_version_minor)

        raise CommunicationError("Failed to get firmware version")

    # -------------------------------------------------------------------------
    # Motor Control
    # -------------------------------------------------------------------------

    def set_duty_cycle(self, duty: float) -> None:
        """Set motor duty cycle (throttle).

        Args:
            duty: Duty cycle from -1.0 to 1.0.
        """
        duty = max(-1.0, min(1.0, duty))

        if self._simulation:
            self._sim_control_mode = VESCControlMode.DUTY_CYCLE
            self._sim_setpoint = duty
            self._sim_state.duty_cycle = duty
            self._sim_state.rpm = int(duty * 10000)
            logger.debug("VESC duty cycle: %.2f", duty)
            return

        self._send_packet(self._pyvesc.SetDutyCycle(duty))

    def set_current(self, current: float) -> None:
        """Set motor current (torque control).

        Args:
            current: Motor current in Amps. Positive for forward, negative for reverse.
        """
        current = max(
            -self._vesc_config.current_limit, min(self._vesc_config.current_limit, current)
        )

        if self._simulation:
            self._sim_control_mode = VESCControlMode.CURRENT
            self._sim_setpoint = current
            self._sim_state.avg_motor_current = current
            logger.debug("VESC current: %.2f A", current)
            return

        self._send_packet(self._pyvesc.SetCurrent(int(current * 1000)))

    def set_current_brake(self, current: float) -> None:
        """Set braking current.

        Args:
            current: Braking current in Amps (positive value).
        """
        current = max(0.0, min(self._vesc_config.current_limit, current))

        if self._simulation:
            self._sim_control_mode = VESCControlMode.CURRENT_BRAKE
            self._sim_setpoint = current
            logger.debug("VESC brake current: %.2f A", current)
            return

        self._send_packet(self._pyvesc.SetCurrentBrake(int(current * 1000)))

    def set_rpm(self, rpm: int) -> None:
        """Set motor electrical RPM (speed control).

        Args:
            rpm: Target electrical RPM.
        """
        rpm = max(-self._vesc_config.rpm_limit, min(self._vesc_config.rpm_limit, rpm))

        if self._simulation:
            self._sim_control_mode = VESCControlMode.SPEED
            self._sim_setpoint = float(rpm)
            self._sim_state.rpm = rpm
            logger.debug("VESC RPM: %d", rpm)
            return

        self._send_packet(self._pyvesc.SetRPM(rpm))

    def set_position(self, degrees: float) -> None:
        """Set motor position (position control).

        Requires encoder or hall sensor feedback.

        Args:
            degrees: Target position in degrees.
        """
        if self._simulation:
            self._sim_control_mode = VESCControlMode.POSITION
            self._sim_setpoint = degrees
            self._sim_state.pid_pos = degrees
            logger.debug("VESC position: %.2f deg", degrees)
            return

        # Position is sent as degrees * 1000000
        self._send_packet(self._pyvesc.SetPosition(int(degrees * 1000000)))

    def set_handbrake(self, current: float) -> None:
        """Set handbrake (hold motor with current).

        Args:
            current: Holding current in Amps.
        """
        current = max(0.0, min(self._vesc_config.current_limit, current))

        if self._simulation:
            logger.debug("VESC handbrake: %.2f A", current)
            return

        self._send_packet(self._pyvesc.SetHandbrake(int(current * 1000)))

    # -------------------------------------------------------------------------
    # Telemetry
    # -------------------------------------------------------------------------

    def get_state(self) -> VESCState:
        """Get VESC telemetry state.

        Returns:
            VESCState object with current telemetry.
        """
        if self._simulation:
            return self._sim_state

        self._send_packet(self._pyvesc.GetValues())
        response = self._receive_packet()

        if response and hasattr(response, "rpm"):
            return VESCState(
                temp_mos=response.temp_mos,
                temp_motor=response.temp_motor,
                avg_motor_current=response.avg_motor_current,
                avg_input_current=response.avg_input_current,
                avg_id=response.avg_id,
                avg_iq=response.avg_iq,
                duty_cycle=response.duty_cycle_now,
                rpm=response.rpm,
                v_in=response.v_in,
                amp_hours=response.amp_hours,
                amp_hours_charged=response.amp_hours_charged,
                watt_hours=response.watt_hours,
                watt_hours_charged=response.watt_hours_charged,
                tachometer=response.tachometer,
                tachometer_abs=response.tachometer_abs,
                fault=VESCFaultCode(response.mc_fault_code)
                if hasattr(response, "mc_fault_code")
                else VESCFaultCode.NONE,
                pid_pos=response.pid_pos_now if hasattr(response, "pid_pos_now") else 0.0,
            )

        raise CommunicationError("Failed to get VESC state")

    def get_voltage(self) -> float:
        """Get input voltage.

        Returns:
            Input voltage in Volts.
        """
        state = self.get_state()
        return state.v_in

    def get_rpm(self) -> int:
        """Get current electrical RPM.

        Returns:
            Electrical RPM.
        """
        state = self.get_state()
        return state.rpm

    def get_motor_current(self) -> float:
        """Get motor current.

        Returns:
            Motor current in Amps.
        """
        state = self.get_state()
        return state.avg_motor_current

    def get_temperature(self) -> tuple[float, float]:
        """Get MOSFET and motor temperatures.

        Returns:
            Tuple of (MOSFET temp, motor temp) in Celsius.
        """
        state = self.get_state()
        return (state.temp_mos, state.temp_motor)

    def get_tachometer(self) -> int:
        """Get tachometer value.

        Returns:
            Tachometer value in half electrical rotations.
        """
        state = self.get_state()
        return state.tachometer

    def get_position(self) -> float:
        """Get current PID position.

        Returns:
            Position in degrees.
        """
        state = self.get_state()
        return state.pid_pos

    # -------------------------------------------------------------------------
    # Fault Handling
    # -------------------------------------------------------------------------

    def get_fault(self) -> VESCFaultCode:
        """Get current fault code.

        Returns:
            Current fault code.
        """
        state = self.get_state()
        return state.fault

    def is_faulted(self) -> bool:
        """Check if VESC has an active fault.

        Returns:
            True if faulted.
        """
        return self.get_fault() != VESCFaultCode.NONE

    # -------------------------------------------------------------------------
    # Keep-alive
    # -------------------------------------------------------------------------

    def send_alive(self) -> None:
        """Send keep-alive packet.

        Should be called periodically to prevent VESC timeout.
        """
        if self._simulation:
            return

        self._send_packet(self._pyvesc.Alive())

    def reboot(self) -> None:
        """Reboot the VESC."""
        if self._simulation:
            logger.info("VESC reboot (simulated)")
            return

        self._send_packet(self._pyvesc.Reboot())
        logger.info("VESC reboot command sent")

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def get_status(self) -> dict:
        """Get overall driver status.

        Returns:
            Dictionary with status information.
        """
        state = self.get_state()
        return {
            "connected": self._state == DriverState.CONNECTED,
            "port": self._vesc_config.port,
            "simulation": self._simulation,
            "voltage": state.v_in,
            "rpm": state.rpm,
            "duty_cycle": state.duty_cycle,
            "motor_current": state.avg_motor_current,
            "temp_mos": state.temp_mos,
            "temp_motor": state.temp_motor,
            "fault": state.fault.name,
        }

    # -------------------------------------------------------------------------
    # Driver Abstract Methods
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write to channel (sets duty cycle)."""
        self.set_duty_cycle(value)

    def _read_channel(self, channel: int) -> float:
        """Read from channel (returns normalized RPM)."""
        rpm = self.get_rpm()
        return rpm / self._vesc_config.rpm_limit
