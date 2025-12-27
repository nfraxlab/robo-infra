"""Dynamixel smart servo driver.

This module provides a driver for Dynamixel smart servos, which are
high-performance actuators with built-in position feedback, temperature
sensing, and torque control.

Supports Dynamixel Protocol 2.0 servos including:
- XL330, XL430, XC430 series (low-cost)
- XM430, XM540 series (mid-range)
- XH430, XH540 series (high-performance)
- XW430, XW540 series (industrial)

Example:
    >>> from robo_infra.drivers.dynamixel import DynamixelDriver
    >>>
    >>> # Connect to servo bus
    >>> driver = DynamixelDriver(port="/dev/ttyUSB0", baudrate=1000000)
    >>> driver.connect()
    >>>
    >>> # Ping to find servos
    >>> servo_ids = driver.ping_scan()
    >>> print(f"Found servos: {servo_ids}")
    >>>
    >>> # Enable torque
    >>> driver.set_torque_enable(1, True)
    >>>
    >>> # Move to position
    >>> driver.set_goal_position(1, 2048)  # Center position
    >>>
    >>> # Read current position
    >>> pos = driver.get_present_position(1)
    >>> print(f"Position: {pos}")
    >>>
    >>> # Sync write to multiple servos
    >>> driver.sync_write_position({1: 1024, 2: 2048, 3: 3072})
    >>>
    >>> driver.disconnect()

Hardware Reference:
    Protocol 2.0 uses:
    - Header: 0xFF 0xFF 0xFD 0x00
    - ID byte (0-253, 254 = broadcast)
    - Length (2 bytes)
    - Instruction byte
    - Parameters
    - CRC (2 bytes)
"""

from __future__ import annotations

import logging
import os
import struct
from dataclasses import dataclass
from enum import IntEnum

from robo_infra.core.bus import SerialBus, SerialConfig
from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import CommunicationError


logger = logging.getLogger(__name__)


# =============================================================================
# Dynamixel Protocol 2.0 Constants
# =============================================================================


# Packet header
HEADER = bytes([0xFF, 0xFF, 0xFD, 0x00])

# Broadcast ID
BROADCAST_ID = 0xFE


class Instruction(IntEnum):
    """Dynamixel Protocol 2.0 instructions."""

    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    FACTORY_RESET = 0x06
    REBOOT = 0x08
    CLEAR = 0x10
    CONTROL_TABLE_BACKUP = 0x20
    STATUS = 0x55
    SYNC_READ = 0x82
    SYNC_WRITE = 0x83
    FAST_SYNC_READ = 0x8A
    BULK_READ = 0x92
    BULK_WRITE = 0x93
    FAST_BULK_READ = 0x9A


class OperatingMode(IntEnum):
    """Dynamixel operating modes."""

    CURRENT_CONTROL = 0
    VELOCITY_CONTROL = 1
    POSITION_CONTROL = 3
    EXTENDED_POSITION_CONTROL = 4
    CURRENT_BASED_POSITION_CONTROL = 5
    PWM_CONTROL = 16


class HardwareErrorStatus(IntEnum):
    """Hardware error status bits."""

    INPUT_VOLTAGE = 0x01
    MOTOR_HALL_SENSOR = 0x02
    OVERHEATING = 0x04
    MOTOR_ENCODER = 0x08
    ELECTRICAL_SHOCK = 0x10
    OVERLOAD = 0x20


# =============================================================================
# Control Table Addresses (XL/XM/XH series Protocol 2.0)
# =============================================================================


class ControlTable(IntEnum):
    """Dynamixel Protocol 2.0 control table addresses."""

    # EEPROM (persisted)
    MODEL_NUMBER = 0
    MODEL_INFORMATION = 2
    FIRMWARE_VERSION = 6
    ID = 7
    BAUD_RATE = 8
    RETURN_DELAY_TIME = 9
    DRIVE_MODE = 10
    OPERATING_MODE = 11
    SECONDARY_ID = 12
    PROTOCOL_TYPE = 13
    HOMING_OFFSET = 20
    MOVING_THRESHOLD = 24
    TEMPERATURE_LIMIT = 31
    MAX_VOLTAGE_LIMIT = 32
    MIN_VOLTAGE_LIMIT = 34
    PWM_LIMIT = 36
    CURRENT_LIMIT = 38
    VELOCITY_LIMIT = 44
    MAX_POSITION_LIMIT = 48
    MIN_POSITION_LIMIT = 52
    STARTUP_CONFIGURATION = 60
    PWM_SLOPE = 62
    SHUTDOWN = 63

    # RAM (volatile)
    TORQUE_ENABLE = 64
    LED = 65
    STATUS_RETURN_LEVEL = 68
    REGISTERED_INSTRUCTION = 69
    HARDWARE_ERROR_STATUS = 70
    VELOCITY_I_GAIN = 76
    VELOCITY_P_GAIN = 78
    POSITION_D_GAIN = 80
    POSITION_I_GAIN = 82
    POSITION_P_GAIN = 84
    FEEDFORWARD_2ND_GAIN = 88
    FEEDFORWARD_1ST_GAIN = 90
    BUS_WATCHDOG = 98
    GOAL_PWM = 100
    GOAL_CURRENT = 102
    GOAL_VELOCITY = 104
    PROFILE_ACCELERATION = 108
    PROFILE_VELOCITY = 112
    GOAL_POSITION = 116
    REALTIME_TICK = 120
    MOVING = 122
    MOVING_STATUS = 123
    PRESENT_PWM = 124
    PRESENT_CURRENT = 126
    PRESENT_VELOCITY = 128
    PRESENT_POSITION = 132
    VELOCITY_TRAJECTORY = 136
    POSITION_TRAJECTORY = 140
    PRESENT_INPUT_VOLTAGE = 144
    PRESENT_TEMPERATURE = 146


# Control table data sizes
CONTROL_TABLE_SIZE = {
    ControlTable.MODEL_NUMBER: 2,
    ControlTable.MODEL_INFORMATION: 4,
    ControlTable.FIRMWARE_VERSION: 1,
    ControlTable.ID: 1,
    ControlTable.BAUD_RATE: 1,
    ControlTable.RETURN_DELAY_TIME: 1,
    ControlTable.DRIVE_MODE: 1,
    ControlTable.OPERATING_MODE: 1,
    ControlTable.SECONDARY_ID: 1,
    ControlTable.PROTOCOL_TYPE: 1,
    ControlTable.HOMING_OFFSET: 4,
    ControlTable.MOVING_THRESHOLD: 4,
    ControlTable.TEMPERATURE_LIMIT: 1,
    ControlTable.MAX_VOLTAGE_LIMIT: 2,
    ControlTable.MIN_VOLTAGE_LIMIT: 2,
    ControlTable.PWM_LIMIT: 2,
    ControlTable.CURRENT_LIMIT: 2,
    ControlTable.VELOCITY_LIMIT: 4,
    ControlTable.MAX_POSITION_LIMIT: 4,
    ControlTable.MIN_POSITION_LIMIT: 4,
    ControlTable.STARTUP_CONFIGURATION: 1,
    ControlTable.PWM_SLOPE: 1,
    ControlTable.SHUTDOWN: 1,
    ControlTable.TORQUE_ENABLE: 1,
    ControlTable.LED: 1,
    ControlTable.STATUS_RETURN_LEVEL: 1,
    ControlTable.REGISTERED_INSTRUCTION: 1,
    ControlTable.HARDWARE_ERROR_STATUS: 1,
    ControlTable.VELOCITY_I_GAIN: 2,
    ControlTable.VELOCITY_P_GAIN: 2,
    ControlTable.POSITION_D_GAIN: 2,
    ControlTable.POSITION_I_GAIN: 2,
    ControlTable.POSITION_P_GAIN: 2,
    ControlTable.FEEDFORWARD_2ND_GAIN: 2,
    ControlTable.FEEDFORWARD_1ST_GAIN: 2,
    ControlTable.BUS_WATCHDOG: 1,
    ControlTable.GOAL_PWM: 2,
    ControlTable.GOAL_CURRENT: 2,
    ControlTable.GOAL_VELOCITY: 4,
    ControlTable.PROFILE_ACCELERATION: 4,
    ControlTable.PROFILE_VELOCITY: 4,
    ControlTable.GOAL_POSITION: 4,
    ControlTable.REALTIME_TICK: 2,
    ControlTable.MOVING: 1,
    ControlTable.MOVING_STATUS: 1,
    ControlTable.PRESENT_PWM: 2,
    ControlTable.PRESENT_CURRENT: 2,
    ControlTable.PRESENT_VELOCITY: 4,
    ControlTable.PRESENT_POSITION: 4,
    ControlTable.VELOCITY_TRAJECTORY: 4,
    ControlTable.POSITION_TRAJECTORY: 4,
    ControlTable.PRESENT_INPUT_VOLTAGE: 2,
    ControlTable.PRESENT_TEMPERATURE: 1,
}


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class DynamixelConfig:
    """Configuration for Dynamixel driver.

    Attributes:
        port: Serial port path.
        baudrate: Communication baudrate.
        timeout: Response timeout in seconds.
        latency_timer: USB latency timer in ms.
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 1000000  # 1 Mbps is common
    timeout: float = 0.1
    latency_timer: int = 4


# =============================================================================
# CRC16 Calculation
# =============================================================================


# CRC16 lookup table for Dynamixel Protocol 2.0
CRC_TABLE = [
    0x0000,
    0x8005,
    0x800F,
    0x000A,
    0x801B,
    0x001E,
    0x0014,
    0x8011,
    0x8033,
    0x0036,
    0x003C,
    0x8039,
    0x0028,
    0x802D,
    0x8027,
    0x0022,
    0x8063,
    0x0066,
    0x006C,
    0x8069,
    0x0078,
    0x807D,
    0x8077,
    0x0072,
    0x0050,
    0x8055,
    0x805F,
    0x005A,
    0x804B,
    0x004E,
    0x0044,
    0x8041,
    0x80C3,
    0x00C6,
    0x00CC,
    0x80C9,
    0x00D8,
    0x80DD,
    0x80D7,
    0x00D2,
    0x00F0,
    0x80F5,
    0x80FF,
    0x00FA,
    0x80EB,
    0x00EE,
    0x00E4,
    0x80E1,
    0x00A0,
    0x80A5,
    0x80AF,
    0x00AA,
    0x80BB,
    0x00BE,
    0x00B4,
    0x80B1,
    0x8093,
    0x0096,
    0x009C,
    0x8099,
    0x0088,
    0x808D,
    0x8087,
    0x0082,
    0x8183,
    0x0186,
    0x018C,
    0x8189,
    0x0198,
    0x819D,
    0x8197,
    0x0192,
    0x01B0,
    0x81B5,
    0x81BF,
    0x01BA,
    0x81AB,
    0x01AE,
    0x01A4,
    0x81A1,
    0x01E0,
    0x81E5,
    0x81EF,
    0x01EA,
    0x81FB,
    0x01FE,
    0x01F4,
    0x81F1,
    0x81D3,
    0x01D6,
    0x01DC,
    0x81D9,
    0x01C8,
    0x81CD,
    0x81C7,
    0x01C2,
    0x0140,
    0x8145,
    0x814F,
    0x014A,
    0x815B,
    0x015E,
    0x0154,
    0x8151,
    0x8173,
    0x0176,
    0x017C,
    0x8179,
    0x0168,
    0x816D,
    0x8167,
    0x0162,
    0x8123,
    0x0126,
    0x012C,
    0x8129,
    0x0138,
    0x813D,
    0x8137,
    0x0132,
    0x0110,
    0x8115,
    0x811F,
    0x011A,
    0x810B,
    0x010E,
    0x0104,
    0x8101,
    0x8303,
    0x0306,
    0x030C,
    0x8309,
    0x0318,
    0x831D,
    0x8317,
    0x0312,
    0x0330,
    0x8335,
    0x833F,
    0x033A,
    0x832B,
    0x032E,
    0x0324,
    0x8321,
    0x0360,
    0x8365,
    0x836F,
    0x036A,
    0x837B,
    0x037E,
    0x0374,
    0x8371,
    0x8353,
    0x0356,
    0x035C,
    0x8359,
    0x0348,
    0x834D,
    0x8347,
    0x0342,
    0x03C0,
    0x83C5,
    0x83CF,
    0x03CA,
    0x83DB,
    0x03DE,
    0x03D4,
    0x83D1,
    0x83F3,
    0x03F6,
    0x03FC,
    0x83F9,
    0x03E8,
    0x83ED,
    0x83E7,
    0x03E2,
    0x83A3,
    0x03A6,
    0x03AC,
    0x83A9,
    0x03B8,
    0x83BD,
    0x83B7,
    0x03B2,
    0x0390,
    0x8395,
    0x839F,
    0x039A,
    0x838B,
    0x038E,
    0x0384,
    0x8381,
    0x0280,
    0x8285,
    0x828F,
    0x028A,
    0x829B,
    0x029E,
    0x0294,
    0x8291,
    0x82B3,
    0x02B6,
    0x02BC,
    0x82B9,
    0x02A8,
    0x82AD,
    0x82A7,
    0x02A2,
    0x82E3,
    0x02E6,
    0x02EC,
    0x82E9,
    0x02F8,
    0x82FD,
    0x82F7,
    0x02F2,
    0x02D0,
    0x82D5,
    0x82DF,
    0x02DA,
    0x82CB,
    0x02CE,
    0x02C4,
    0x82C1,
    0x8243,
    0x0246,
    0x024C,
    0x8249,
    0x0258,
    0x825D,
    0x8257,
    0x0252,
    0x0270,
    0x8275,
    0x827F,
    0x027A,
    0x826B,
    0x026E,
    0x0264,
    0x8261,
    0x0220,
    0x8225,
    0x822F,
    0x022A,
    0x823B,
    0x023E,
    0x0234,
    0x8231,
    0x8213,
    0x0216,
    0x021C,
    0x8219,
    0x0208,
    0x820D,
    0x8207,
    0x0202,
]


def _calculate_crc(data: bytes) -> int:
    """Calculate CRC16 for Dynamixel Protocol 2.0.

    Args:
        data: Data bytes to calculate CRC for.

    Returns:
        16-bit CRC value.
    """
    crc = 0
    for byte in data:
        i = ((crc >> 8) ^ byte) & 0xFF
        crc = (crc << 8) ^ CRC_TABLE[i]
        crc &= 0xFFFF
    return crc


# =============================================================================
# Dynamixel Driver
# =============================================================================


@register_driver("dynamixel")
class DynamixelDriver(Driver):
    """Driver for Dynamixel smart servos (Protocol 2.0).

    Supports XL/XM/XH/XW series servos.

    Example:
        >>> driver = DynamixelDriver(port="/dev/ttyUSB0")
        >>> driver.connect()
        >>> driver.set_torque_enable(1, True)
        >>> driver.set_goal_position(1, 2048)
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = 1000000,
        config: DynamixelConfig | None = None,
        simulation: bool | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize Dynamixel driver.

        Args:
            port: Serial port path.
            baudrate: Communication baudrate.
            config: Driver configuration.
            simulation: If True, use simulation mode.
            name: Optional human-readable name.
        """
        if simulation is None:
            simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

        super().__init__(
            config=DriverConfig(
                name=name or "Dynamixel",
                channels=253,  # Max servos on bus
                auto_connect=False,
            )
        )

        self._dxl_config = config or DynamixelConfig(
            port=port or "/dev/ttyUSB0",
            baudrate=baudrate,
        )
        self._simulation = simulation

        # Serial connection
        self._serial: SerialBus | None = None

        # Simulated state
        self._sim_servos: dict[int, dict] = {}  # servo_id -> state dict

    @property
    def simulation(self) -> bool:
        """Whether running in simulation mode."""
        return self._simulation

    @property
    def port(self) -> str:
        """Serial port."""
        return self._dxl_config.port

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the Dynamixel bus.

        Raises:
            HardwareNotFoundError: If serial port not found.
            CommunicationError: If connection fails.
        """
        if self._simulation:
            logger.info("Dynamixel connecting in simulation mode")
            # Initialize some simulated servos
            for i in range(1, 5):
                self._sim_servos[i] = {
                    "position": 2048,
                    "velocity": 0,
                    "torque_enable": False,
                    "temperature": 25,
                    "voltage": 120,  # 12.0V * 10
                    "moving": False,
                    "mode": OperatingMode.POSITION_CONTROL,
                }
            self._state = DriverState.CONNECTED
            return

        try:
            self._serial = SerialBus(
                SerialConfig(
                    port=self._dxl_config.port,
                    baudrate=self._dxl_config.baudrate,
                    timeout=self._dxl_config.timeout,
                )
            )
            self._serial.connect()
            logger.info("Connected to Dynamixel bus on %s", self._dxl_config.port)

        except Exception as e:
            raise CommunicationError(f"Failed to connect to Dynamixel bus: {e}") from e

        self._state = DriverState.CONNECTED

    def disconnect(self) -> None:
        """Disconnect from the Dynamixel bus."""
        # Disable torque on all known servos
        if self._state == DriverState.CONNECTED:
            for servo_id in list(self._sim_servos.keys()):
                try:
                    self.set_torque_enable(servo_id, False)
                except Exception as e:
                    logger.warning("Error disabling torque on servo %d: %s", servo_id, e)

        if self._serial is not None:
            self._serial.disconnect()
            self._serial = None

        self._state = DriverState.DISCONNECTED
        logger.info("Dynamixel disconnected")

    # -------------------------------------------------------------------------
    # Low-level Communication
    # -------------------------------------------------------------------------

    def _build_packet(self, servo_id: int, instruction: Instruction, params: bytes = b"") -> bytes:
        """Build a Dynamixel Protocol 2.0 packet.

        Args:
            servo_id: Target servo ID.
            instruction: Instruction byte.
            params: Parameter bytes.

        Returns:
            Complete packet bytes.
        """
        length = len(params) + 3  # instruction + CRC (2 bytes)

        packet = (
            HEADER + bytes([servo_id]) + struct.pack("<H", length) + bytes([instruction]) + params
        )

        crc = _calculate_crc(packet)
        packet += struct.pack("<H", crc)

        return packet

    def _send_packet(self, packet: bytes) -> None:
        """Send a packet to the bus.

        Args:
            packet: Packet bytes to send.
        """
        if self._simulation:
            return

        if self._serial is None:
            raise CommunicationError("Not connected to Dynamixel bus")

        self._serial.write(packet)

    def _receive_status(self, timeout: float | None = None) -> tuple[int, int, bytes]:
        """Receive a status packet.

        Args:
            timeout: Optional timeout override.

        Returns:
            Tuple of (servo_id, error_byte, parameters).
        """
        if self._simulation:
            return (0, 0, b"")

        if self._serial is None:
            raise CommunicationError("Not connected to Dynamixel bus")

        # Read header
        header = self._serial.read(4)
        if header != HEADER:
            raise CommunicationError("Invalid response header")

        # Read ID
        servo_id = self._serial.read(1)[0]

        # Read length
        length_bytes = self._serial.read(2)
        length = struct.unpack("<H", length_bytes)[0]

        # Read instruction (should be STATUS)
        instruction = self._serial.read(1)[0]
        if instruction != Instruction.STATUS:
            raise CommunicationError(f"Unexpected instruction: {instruction}")

        # Read error byte
        error = self._serial.read(1)[0]

        # Read parameters
        param_length = length - 4  # instruction + error + CRC
        params = self._serial.read(param_length) if param_length > 0 else b""

        # Read and verify CRC
        crc_bytes = self._serial.read(2)
        received_crc = struct.unpack("<H", crc_bytes)[0]

        # Reconstruct packet for CRC check
        check_data = (
            HEADER + bytes([servo_id]) + length_bytes + bytes([instruction, error]) + params
        )
        calculated_crc = _calculate_crc(check_data)

        if received_crc != calculated_crc:
            raise CommunicationError("CRC mismatch in response")

        return (servo_id, error, params)

    # -------------------------------------------------------------------------
    # Basic Operations
    # -------------------------------------------------------------------------

    def ping(self, servo_id: int) -> dict | None:
        """Ping a servo.

        Args:
            servo_id: Servo ID to ping.

        Returns:
            Dictionary with model number and firmware, or None if not found.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return {"model": 1060, "firmware": 45}  # XL330
            return None

        packet = self._build_packet(servo_id, Instruction.PING)
        self._send_packet(packet)

        try:
            _, error, params = self._receive_status()
            if error == 0 and len(params) >= 3:
                model = struct.unpack("<H", params[:2])[0]
                firmware = params[2]
                return {"model": model, "firmware": firmware}
        except Exception:
            pass

        return None

    def ping_scan(self, start_id: int = 1, end_id: int = 253) -> list[int]:
        """Scan for servos on the bus.

        Args:
            start_id: Starting ID to scan.
            end_id: Ending ID to scan.

        Returns:
            List of found servo IDs.
        """
        found = []

        for servo_id in range(start_id, end_id + 1):
            if self.ping(servo_id) is not None:
                found.append(servo_id)

        logger.info("Found %d servos: %s", len(found), found)
        return found

    def reboot(self, servo_id: int) -> None:
        """Reboot a servo.

        Args:
            servo_id: Servo ID.
        """
        if self._simulation:
            logger.info("Reboot servo %d (simulated)", servo_id)
            return

        packet = self._build_packet(servo_id, Instruction.REBOOT)
        self._send_packet(packet)

    # -------------------------------------------------------------------------
    # Read/Write Operations
    # -------------------------------------------------------------------------

    def read(self, servo_id: int, address: int, length: int) -> bytes:
        """Read from control table.

        Args:
            servo_id: Servo ID.
            address: Control table address.
            length: Number of bytes to read.

        Returns:
            Read data bytes.
        """
        if self._simulation:
            return bytes(length)  # Return zeros

        params = struct.pack("<HH", address, length)
        packet = self._build_packet(servo_id, Instruction.READ, params)
        self._send_packet(packet)

        _, error, data = self._receive_status()
        if error != 0:
            raise CommunicationError(f"Read error: {error}")

        return data

    def write(self, servo_id: int, address: int, data: bytes) -> None:
        """Write to control table.

        Args:
            servo_id: Servo ID.
            address: Control table address.
            data: Data bytes to write.
        """
        if self._simulation:
            return

        params = struct.pack("<H", address) + data
        packet = self._build_packet(servo_id, Instruction.WRITE, params)
        self._send_packet(packet)

        _, error, _ = self._receive_status()
        if error != 0:
            raise CommunicationError(f"Write error: {error}")

    def read_1(self, servo_id: int, address: int) -> int:
        """Read 1-byte value.

        Args:
            servo_id: Servo ID.
            address: Control table address.

        Returns:
            1-byte value.
        """
        data = self.read(servo_id, address, 1)
        return data[0]

    def read_2(self, servo_id: int, address: int) -> int:
        """Read 2-byte value.

        Args:
            servo_id: Servo ID.
            address: Control table address.

        Returns:
            2-byte value.
        """
        data = self.read(servo_id, address, 2)
        return struct.unpack("<H", data)[0]

    def read_4(self, servo_id: int, address: int) -> int:
        """Read 4-byte value.

        Args:
            servo_id: Servo ID.
            address: Control table address.

        Returns:
            4-byte value.
        """
        data = self.read(servo_id, address, 4)
        return struct.unpack("<I", data)[0]

    def write_1(self, servo_id: int, address: int, value: int) -> None:
        """Write 1-byte value.

        Args:
            servo_id: Servo ID.
            address: Control table address.
            value: Value to write.
        """
        self.write(servo_id, address, bytes([value & 0xFF]))

    def write_2(self, servo_id: int, address: int, value: int) -> None:
        """Write 2-byte value.

        Args:
            servo_id: Servo ID.
            address: Control table address.
            value: Value to write.
        """
        self.write(servo_id, address, struct.pack("<H", value & 0xFFFF))

    def write_4(self, servo_id: int, address: int, value: int) -> None:
        """Write 4-byte value.

        Args:
            servo_id: Servo ID.
            address: Control table address.
            value: Value to write.
        """
        self.write(servo_id, address, struct.pack("<I", value & 0xFFFFFFFF))

    # -------------------------------------------------------------------------
    # Sync Operations
    # -------------------------------------------------------------------------

    def sync_write(self, address: int, data_length: int, data: dict[int, bytes]) -> None:
        """Sync write to multiple servos.

        Args:
            address: Control table address.
            data_length: Length of data per servo.
            data: Dictionary of servo_id -> data bytes.
        """
        if self._simulation:
            return

        # Build sync write packet
        params = struct.pack("<HH", address, data_length)
        for servo_id, servo_data in data.items():
            params += bytes([servo_id]) + servo_data

        packet = self._build_packet(BROADCAST_ID, Instruction.SYNC_WRITE, params)
        self._send_packet(packet)
        # Sync write has no response

    def sync_write_position(self, positions: dict[int, int]) -> None:
        """Sync write goal positions.

        Args:
            positions: Dictionary of servo_id -> goal position.
        """
        data = {servo_id: struct.pack("<I", pos) for servo_id, pos in positions.items()}
        self.sync_write(ControlTable.GOAL_POSITION, 4, data)

    # -------------------------------------------------------------------------
    # High-level API
    # -------------------------------------------------------------------------

    def set_torque_enable(self, servo_id: int, enable: bool) -> None:
        """Enable or disable torque.

        Args:
            servo_id: Servo ID.
            enable: True to enable, False to disable.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                self._sim_servos[servo_id]["torque_enable"] = enable
            return

        self.write_1(servo_id, ControlTable.TORQUE_ENABLE, 1 if enable else 0)

    def get_torque_enable(self, servo_id: int) -> bool:
        """Get torque enable status.

        Args:
            servo_id: Servo ID.

        Returns:
            True if torque is enabled.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return self._sim_servos[servo_id].get("torque_enable", False)
            return False

        return self.read_1(servo_id, ControlTable.TORQUE_ENABLE) != 0

    def set_goal_position(self, servo_id: int, position: int) -> None:
        """Set goal position.

        Args:
            servo_id: Servo ID.
            position: Goal position (0-4095 for most servos).
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                self._sim_servos[servo_id]["position"] = position
            return

        self.write_4(servo_id, ControlTable.GOAL_POSITION, position)

    def get_present_position(self, servo_id: int) -> int:
        """Get current position.

        Args:
            servo_id: Servo ID.

        Returns:
            Current position.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return self._sim_servos[servo_id].get("position", 2048)
            return 2048

        return self.read_4(servo_id, ControlTable.PRESENT_POSITION)

    def set_goal_velocity(self, servo_id: int, velocity: int) -> None:
        """Set goal velocity.

        Args:
            servo_id: Servo ID.
            velocity: Goal velocity.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                self._sim_servos[servo_id]["velocity"] = velocity
            return

        self.write_4(servo_id, ControlTable.GOAL_VELOCITY, velocity)

    def get_present_velocity(self, servo_id: int) -> int:
        """Get current velocity.

        Args:
            servo_id: Servo ID.

        Returns:
            Current velocity.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return self._sim_servos[servo_id].get("velocity", 0)
            return 0

        return self.read_4(servo_id, ControlTable.PRESENT_VELOCITY)

    def set_goal_current(self, servo_id: int, current: int) -> None:
        """Set goal current (torque).

        Args:
            servo_id: Servo ID.
            current: Goal current.
        """
        if self._simulation:
            return

        self.write_2(servo_id, ControlTable.GOAL_CURRENT, current)

    def get_present_current(self, servo_id: int) -> int:
        """Get current load.

        Args:
            servo_id: Servo ID.

        Returns:
            Current load.
        """
        if self._simulation:
            return 0

        return self.read_2(servo_id, ControlTable.PRESENT_CURRENT)

    def set_operating_mode(self, servo_id: int, mode: OperatingMode) -> None:
        """Set operating mode.

        Must disable torque first.

        Args:
            servo_id: Servo ID.
            mode: Operating mode.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                self._sim_servos[servo_id]["mode"] = mode
            return

        # Torque must be disabled to change operating mode
        self.set_torque_enable(servo_id, False)
        self.write_1(servo_id, ControlTable.OPERATING_MODE, mode)

    def get_operating_mode(self, servo_id: int) -> OperatingMode:
        """Get operating mode.

        Args:
            servo_id: Servo ID.

        Returns:
            Current operating mode.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return self._sim_servos[servo_id].get("mode", OperatingMode.POSITION_CONTROL)
            return OperatingMode.POSITION_CONTROL

        return OperatingMode(self.read_1(servo_id, ControlTable.OPERATING_MODE))

    def get_present_temperature(self, servo_id: int) -> int:
        """Get present temperature.

        Args:
            servo_id: Servo ID.

        Returns:
            Temperature in Celsius.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return self._sim_servos[servo_id].get("temperature", 25)
            return 25

        return self.read_1(servo_id, ControlTable.PRESENT_TEMPERATURE)

    def get_present_voltage(self, servo_id: int) -> float:
        """Get present input voltage.

        Args:
            servo_id: Servo ID.

        Returns:
            Voltage in Volts.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return self._sim_servos[servo_id].get("voltage", 120) / 10.0
            return 12.0

        raw = self.read_2(servo_id, ControlTable.PRESENT_INPUT_VOLTAGE)
        return raw / 10.0

    def is_moving(self, servo_id: int) -> bool:
        """Check if servo is moving.

        Args:
            servo_id: Servo ID.

        Returns:
            True if moving.
        """
        if self._simulation:
            if servo_id in self._sim_servos:
                return self._sim_servos[servo_id].get("moving", False)
            return False

        return self.read_1(servo_id, ControlTable.MOVING) != 0

    def get_hardware_error(self, servo_id: int) -> int:
        """Get hardware error status.

        Args:
            servo_id: Servo ID.

        Returns:
            Hardware error status bits.
        """
        if self._simulation:
            return 0

        return self.read_1(servo_id, ControlTable.HARDWARE_ERROR_STATUS)

    def set_led(self, servo_id: int, on: bool) -> None:
        """Set LED state.

        Args:
            servo_id: Servo ID.
            on: True to turn on.
        """
        if self._simulation:
            return

        self.write_1(servo_id, ControlTable.LED, 1 if on else 0)

    # -------------------------------------------------------------------------
    # Profile Settings
    # -------------------------------------------------------------------------

    def set_profile_velocity(self, servo_id: int, velocity: int) -> None:
        """Set profile velocity (max velocity in position mode).

        Args:
            servo_id: Servo ID.
            velocity: Profile velocity.
        """
        if self._simulation:
            return

        self.write_4(servo_id, ControlTable.PROFILE_VELOCITY, velocity)

    def set_profile_acceleration(self, servo_id: int, acceleration: int) -> None:
        """Set profile acceleration.

        Args:
            servo_id: Servo ID.
            acceleration: Profile acceleration.
        """
        if self._simulation:
            return

        self.write_4(servo_id, ControlTable.PROFILE_ACCELERATION, acceleration)

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def get_status(self) -> dict:
        """Get overall driver status.

        Returns:
            Dictionary with status information.
        """
        servo_info = []
        for servo_id in list(self._sim_servos.keys()):
            info = self.ping(servo_id)
            if info:
                servo_info.append(
                    {
                        "id": servo_id,
                        "model": info["model"],
                        "position": self.get_present_position(servo_id),
                        "temperature": self.get_present_temperature(servo_id),
                        "voltage": self.get_present_voltage(servo_id),
                        "torque_enabled": self.get_torque_enable(servo_id),
                        "moving": self.is_moving(servo_id),
                    }
                )

        return {
            "connected": self._state == DriverState.CONNECTED,
            "port": self._dxl_config.port,
            "baudrate": self._dxl_config.baudrate,
            "simulation": self._simulation,
            "servos": servo_info,
        }

    # -------------------------------------------------------------------------
    # Driver Abstract Methods
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write to channel (sets position)."""
        # Map 0.0-1.0 to 0-4095
        position = int(value * 4095)
        position = max(0, min(4095, position))
        self.set_goal_position(channel, position)

    def _read_channel(self, channel: int) -> float:
        """Read from channel (returns normalized position)."""
        position = self.get_present_position(channel)
        return position / 4095.0
