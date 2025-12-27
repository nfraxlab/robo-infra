"""Modbus RTU and TCP protocol implementation.

Modbus is a widely used industrial communication protocol for connecting
PLCs, sensors, actuators, and other devices. This module provides both
RTU (serial) and TCP (Ethernet) variants.

Supported Function Codes:
    - 0x01: Read Coils
    - 0x02: Read Discrete Inputs
    - 0x03: Read Holding Registers
    - 0x04: Read Input Registers
    - 0x05: Write Single Coil
    - 0x06: Write Single Register
    - 0x0F: Write Multiple Coils
    - 0x10: Write Multiple Registers

Example (RTU):
    >>> from robo_infra.protocols.modbus import ModbusRTU
    >>>
    >>> # Connect to serial Modbus device
    >>> modbus = ModbusRTU("/dev/ttyUSB0", baudrate=9600)
    >>> modbus.open()
    >>>
    >>> # Read holding registers from slave 1
    >>> registers = modbus.read_holding_registers(1, address=0, count=10)
    >>>
    >>> # Write to coil
    >>> modbus.write_coil(1, address=0, value=True)
    >>>
    >>> modbus.close()

Example (TCP):
    >>> from robo_infra.protocols.modbus import ModbusTCP
    >>>
    >>> # Connect to TCP Modbus device
    >>> modbus = ModbusTCP("192.168.1.100", port=502)
    >>> modbus.open()
    >>>
    >>> # Read input registers
    >>> values = modbus.read_input_registers(1, address=100, count=5)
    >>>
    >>> modbus.close()
"""

from __future__ import annotations

import builtins
import contextlib
import logging
import os
import socket
import struct
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

from robo_infra.core.exceptions import CommunicationError, TimeoutError


if TYPE_CHECKING:
    import serial  # type: ignore

logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================


class FunctionCode(IntEnum):
    """Modbus function codes."""

    READ_COILS = 0x01
    READ_DISCRETE_INPUTS = 0x02
    READ_HOLDING_REGISTERS = 0x03
    READ_INPUT_REGISTERS = 0x04
    WRITE_SINGLE_COIL = 0x05
    WRITE_SINGLE_REGISTER = 0x06
    READ_EXCEPTION_STATUS = 0x07
    DIAGNOSTICS = 0x08
    GET_COMM_EVENT_COUNTER = 0x0B
    GET_COMM_EVENT_LOG = 0x0C
    WRITE_MULTIPLE_COILS = 0x0F
    WRITE_MULTIPLE_REGISTERS = 0x10
    REPORT_SLAVE_ID = 0x11
    READ_FILE_RECORD = 0x14
    WRITE_FILE_RECORD = 0x15
    MASK_WRITE_REGISTER = 0x16
    READ_WRITE_MULTIPLE_REGISTERS = 0x17
    READ_FIFO_QUEUE = 0x18
    ENCAPSULATED_INTERFACE = 0x2B


class ExceptionCode(IntEnum):
    """Modbus exception codes."""

    ILLEGAL_FUNCTION = 0x01
    ILLEGAL_DATA_ADDRESS = 0x02
    ILLEGAL_DATA_VALUE = 0x03
    SLAVE_DEVICE_FAILURE = 0x04
    ACKNOWLEDGE = 0x05
    SLAVE_DEVICE_BUSY = 0x06
    NEGATIVE_ACKNOWLEDGE = 0x07
    MEMORY_PARITY_ERROR = 0x08
    GATEWAY_PATH_UNAVAILABLE = 0x0A
    GATEWAY_TARGET_FAILED = 0x0B


# Coil values
COIL_ON = 0xFF00
COIL_OFF = 0x0000


# =============================================================================
# Exceptions
# =============================================================================


@dataclass
class ModbusError(Exception):
    """Modbus protocol error."""

    function_code: int
    exception_code: int
    message: str = ""

    def __post_init__(self) -> None:
        """Generate message from exception code."""
        if not self.message:
            self.message = self._get_exception_message()

    def _get_exception_message(self) -> str:
        """Get human-readable exception message."""
        messages = {
            ExceptionCode.ILLEGAL_FUNCTION: "Illegal function",
            ExceptionCode.ILLEGAL_DATA_ADDRESS: "Illegal data address",
            ExceptionCode.ILLEGAL_DATA_VALUE: "Illegal data value",
            ExceptionCode.SLAVE_DEVICE_FAILURE: "Slave device failure",
            ExceptionCode.ACKNOWLEDGE: "Acknowledge",
            ExceptionCode.SLAVE_DEVICE_BUSY: "Slave device busy",
            ExceptionCode.NEGATIVE_ACKNOWLEDGE: "Negative acknowledge",
            ExceptionCode.MEMORY_PARITY_ERROR: "Memory parity error",
            ExceptionCode.GATEWAY_PATH_UNAVAILABLE: "Gateway path unavailable",
            ExceptionCode.GATEWAY_TARGET_FAILED: "Gateway target device failed",
        }
        return messages.get(self.exception_code, f"Unknown exception (0x{self.exception_code:02X})")


# =============================================================================
# CRC Calculation
# =============================================================================


def _crc16_table() -> list[int]:
    """Generate CRC16 lookup table."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
        table.append(crc)
    return table


_CRC_TABLE = _crc16_table()


def calculate_crc16(data: bytes) -> int:
    """Calculate Modbus CRC16.

    Args:
        data: Data to calculate CRC for.

    Returns:
        CRC16 value.
    """
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def verify_crc16(data: bytes) -> bool:
    """Verify Modbus CRC16.

    Args:
        data: Data including CRC (last 2 bytes).

    Returns:
        True if CRC is valid.
    """
    if len(data) < 3:
        return False
    received_crc = struct.unpack("<H", data[-2:])[0]
    calculated_crc = calculate_crc16(data[:-2])
    return received_crc == calculated_crc


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class ModbusRTUConfig:
    """Modbus RTU configuration.

    Attributes:
        port: Serial port (e.g., "/dev/ttyUSB0", "COM1").
        baudrate: Baud rate (default 9600).
        bytesize: Data bits (7 or 8).
        parity: Parity ('N', 'E', 'O').
        stopbits: Stop bits (1 or 2).
        timeout: Read timeout in seconds.
        inter_frame_delay: Delay between frames (auto-calculated if None).
    """

    port: str
    baudrate: int = 9600
    bytesize: int = 8
    parity: str = "N"
    stopbits: int = 1
    timeout: float = 1.0
    inter_frame_delay: float | None = None

    def __post_init__(self) -> None:
        """Calculate inter-frame delay if not specified."""
        if self.inter_frame_delay is None:
            # Modbus RTU requires 3.5 character times between frames
            char_time = 11.0 / self.baudrate  # 11 bits per character
            self.inter_frame_delay = 3.5 * char_time


@dataclass
class ModbusTCPConfig:
    """Modbus TCP configuration.

    Attributes:
        host: Server hostname or IP address.
        port: TCP port (default 502).
        timeout: Connection and read timeout in seconds.
        unit_id: Default unit identifier (0-247).
    """

    host: str
    port: int = 502
    timeout: float = 1.0
    unit_id: int = 1


@dataclass
class ModbusStatistics:
    """Modbus communication statistics.

    Attributes:
        requests_sent: Total requests sent.
        responses_received: Total responses received.
        errors: Total errors.
        timeouts: Total timeouts.
        crc_errors: Total CRC errors (RTU only).
        last_request_time: Timestamp of last request.
        last_response_time: Timestamp of last response.
    """

    requests_sent: int = 0
    responses_received: int = 0
    errors: int = 0
    timeouts: int = 0
    crc_errors: int = 0
    last_request_time: float = 0.0
    last_response_time: float = 0.0


# =============================================================================
# Abstract Base
# =============================================================================


class ModbusClient(ABC):
    """Abstract Modbus client base class."""

    def __init__(self, name: str = "modbus") -> None:
        """Initialize Modbus client.

        Args:
            name: Client identifier.
        """
        self._name = name
        self._is_open = False
        self._stats = ModbusStatistics()

    @property
    def name(self) -> str:
        """Client name."""
        return self._name

    @property
    def is_open(self) -> bool:
        """Whether client is connected."""
        return self._is_open

    @property
    def statistics(self) -> ModbusStatistics:
        """Communication statistics."""
        return self._stats

    @abstractmethod
    def open(self) -> None:
        """Open connection."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Close connection."""
        ...

    def __enter__(self) -> ModbusClient:
        """Enter context manager."""
        self.open()
        return self

    def __exit__(self, exc_type: type | None, exc_val: Exception | None, exc_tb: object) -> None:
        """Exit context manager."""
        self.close()

    # =========================================================================
    # Read Operations
    # =========================================================================

    @abstractmethod
    def read_coils(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[bool]:
        """Read coils (function 0x01).

        Args:
            slave_id: Slave device ID (1-247).
            address: Starting address (0-65535).
            count: Number of coils to read (1-2000).

        Returns:
            List of coil values.
        """
        ...

    @abstractmethod
    def read_discrete_inputs(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[bool]:
        """Read discrete inputs (function 0x02).

        Args:
            slave_id: Slave device ID (1-247).
            address: Starting address (0-65535).
            count: Number of inputs to read (1-2000).

        Returns:
            List of input values.
        """
        ...

    @abstractmethod
    def read_holding_registers(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[int]:
        """Read holding registers (function 0x03).

        Args:
            slave_id: Slave device ID (1-247).
            address: Starting address (0-65535).
            count: Number of registers to read (1-125).

        Returns:
            List of register values (16-bit unsigned).
        """
        ...

    @abstractmethod
    def read_input_registers(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[int]:
        """Read input registers (function 0x04).

        Args:
            slave_id: Slave device ID (1-247).
            address: Starting address (0-65535).
            count: Number of registers to read (1-125).

        Returns:
            List of register values (16-bit unsigned).
        """
        ...

    # =========================================================================
    # Write Operations
    # =========================================================================

    @abstractmethod
    def write_coil(
        self,
        slave_id: int,
        address: int,
        value: bool,
    ) -> None:
        """Write single coil (function 0x05).

        Args:
            slave_id: Slave device ID (1-247).
            address: Coil address (0-65535).
            value: Coil value.
        """
        ...

    @abstractmethod
    def write_register(
        self,
        slave_id: int,
        address: int,
        value: int,
    ) -> None:
        """Write single register (function 0x06).

        Args:
            slave_id: Slave device ID (1-247).
            address: Register address (0-65535).
            value: Register value (0-65535).
        """
        ...

    @abstractmethod
    def write_coils(
        self,
        slave_id: int,
        address: int,
        values: list[bool],
    ) -> None:
        """Write multiple coils (function 0x0F).

        Args:
            slave_id: Slave device ID (1-247).
            address: Starting address (0-65535).
            values: List of coil values.
        """
        ...

    @abstractmethod
    def write_registers(
        self,
        slave_id: int,
        address: int,
        values: list[int],
    ) -> None:
        """Write multiple registers (function 0x10).

        Args:
            slave_id: Slave device ID (1-247).
            address: Starting address (0-65535).
            values: List of register values (0-65535 each).
        """
        ...

    # =========================================================================
    # Helper Methods
    # =========================================================================

    def read_holding_register(self, slave_id: int, address: int) -> int:
        """Read single holding register.

        Args:
            slave_id: Slave device ID.
            address: Register address.

        Returns:
            Register value.
        """
        return self.read_holding_registers(slave_id, address, 1)[0]

    def read_input_register(self, slave_id: int, address: int) -> int:
        """Read single input register.

        Args:
            slave_id: Slave device ID.
            address: Register address.

        Returns:
            Register value.
        """
        return self.read_input_registers(slave_id, address, 1)[0]

    def read_coil(self, slave_id: int, address: int) -> bool:
        """Read single coil.

        Args:
            slave_id: Slave device ID.
            address: Coil address.

        Returns:
            Coil value.
        """
        return self.read_coils(slave_id, address, 1)[0]

    def read_discrete_input(self, slave_id: int, address: int) -> bool:
        """Read single discrete input.

        Args:
            slave_id: Slave device ID.
            address: Input address.

        Returns:
            Input value.
        """
        return self.read_discrete_inputs(slave_id, address, 1)[0]

    # =========================================================================
    # Data Type Helpers
    # =========================================================================

    def read_float32(
        self,
        slave_id: int,
        address: int,
        byte_order: str = "big",
    ) -> float:
        """Read 32-bit float from two consecutive registers.

        Args:
            slave_id: Slave device ID.
            address: Starting register address.
            byte_order: "big" or "little".

        Returns:
            Float value.
        """
        regs = self.read_holding_registers(slave_id, address, 2)
        raw = regs[0] << 16 | regs[1] if byte_order == "big" else regs[1] << 16 | regs[0]
        return struct.unpack(">f", struct.pack(">I", raw))[0]

    def write_float32(
        self,
        slave_id: int,
        address: int,
        value: float,
        byte_order: str = "big",
    ) -> None:
        """Write 32-bit float to two consecutive registers.

        Args:
            slave_id: Slave device ID.
            address: Starting register address.
            value: Float value.
            byte_order: "big" or "little".
        """
        raw = struct.unpack(">I", struct.pack(">f", value))[0]
        regs = [raw >> 16, raw & 65535] if byte_order == "big" else [raw & 65535, raw >> 16]
        self.write_registers(slave_id, address, regs)

    def read_int32(
        self,
        slave_id: int,
        address: int,
        signed: bool = True,
        byte_order: str = "big",
    ) -> int:
        """Read 32-bit integer from two consecutive registers.

        Args:
            slave_id: Slave device ID.
            address: Starting register address.
            signed: Whether to interpret as signed.
            byte_order: "big" or "little".

        Returns:
            Integer value.
        """
        regs = self.read_holding_registers(slave_id, address, 2)
        raw = regs[0] << 16 | regs[1] if byte_order == "big" else regs[1] << 16 | regs[0]
        if signed and raw >= 0x80000000:
            raw -= 0x100000000
        return raw

    def write_int32(
        self,
        slave_id: int,
        address: int,
        value: int,
        byte_order: str = "big",
    ) -> None:
        """Write 32-bit integer to two consecutive registers.

        Args:
            slave_id: Slave device ID.
            address: Starting register address.
            value: Integer value.
            byte_order: "big" or "little".
        """
        if value < 0:
            value += 0x100000000
        regs = [value >> 16, value & 65535] if byte_order == "big" else [value & 65535, value >> 16]
        self.write_registers(slave_id, address, regs)


# =============================================================================
# Modbus RTU Implementation
# =============================================================================


class ModbusRTU(ModbusClient):
    """Modbus RTU client over serial connection."""

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        bytesize: int = 8,
        parity: str = "N",
        stopbits: int = 1,
        timeout: float = 1.0,
        name: str = "modbus-rtu",
    ) -> None:
        """Initialize Modbus RTU client.

        Args:
            port: Serial port path.
            baudrate: Baud rate.
            bytesize: Data bits.
            parity: Parity.
            stopbits: Stop bits.
            timeout: Read timeout.
            name: Client name.
        """
        super().__init__(name)
        self._config = ModbusRTUConfig(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout,
        )
        self._serial: serial.Serial | None = None
        self._simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes")

    @property
    def config(self) -> ModbusRTUConfig:
        """RTU configuration."""
        return self._config

    def open(self) -> None:
        """Open serial connection."""
        if self._is_open:
            return

        if self._simulation:
            logger.warning("⚠️ SIMULATION MODE - Modbus RTU simulated")
            self._is_open = True
            return

        try:
            import serial as pyserial

            self._serial = pyserial.Serial(
                port=self._config.port,
                baudrate=self._config.baudrate,
                bytesize=self._config.bytesize,
                parity=self._config.parity,
                stopbits=self._config.stopbits,
                timeout=self._config.timeout,
            )
            self._is_open = True
            logger.info("Modbus RTU opened: %s @ %d", self._config.port, self._config.baudrate)
        except ImportError as e:
            raise CommunicationError("pyserial not installed. Run: pip install pyserial") from e
        except Exception as e:
            raise CommunicationError(f"Failed to open serial port: {e}") from e

    def close(self) -> None:
        """Close serial connection."""
        if self._serial:
            self._serial.close()
            self._serial = None
        self._is_open = False
        logger.info("Modbus RTU closed")

    def _send_receive(self, request: bytes) -> bytes:
        """Send request and receive response.

        Args:
            request: Request PDU (without CRC).

        Returns:
            Response PDU (without CRC).
        """
        self._stats.requests_sent += 1
        self._stats.last_request_time = time.time()

        if self._simulation:
            return self._simulate_response(request)

        if not self._serial:
            raise CommunicationError("Serial port not open")

        # Add CRC
        crc = calculate_crc16(request)
        frame = request + struct.pack("<H", crc)

        # Ensure inter-frame delay
        time.sleep(self._config.inter_frame_delay or 0.001)

        # Send
        self._serial.write(frame)

        # Receive response
        # First, read header (slave_id + function_code)
        header = self._serial.read(2)
        if len(header) < 2:
            self._stats.timeouts += 1
            raise TimeoutError("Modbus RTU response timeout")

        function_code = header[1]

        # Check for exception response
        if function_code & 0x80:
            # Exception: 1 more byte + 2 CRC
            rest = self._serial.read(3)
            response = header + rest
        else:
            # Determine response length based on function code
            length = self._get_response_length(function_code, request)
            rest = self._serial.read(length)
            response = header + rest

        # Verify CRC
        if not verify_crc16(response):
            self._stats.crc_errors += 1
            raise CommunicationError("CRC error in Modbus RTU response")

        self._stats.responses_received += 1
        self._stats.last_response_time = time.time()

        # Return PDU without CRC
        return response[:-2]

    def _get_response_length(self, function_code: int, request: bytes) -> int:
        """Get expected response length (excluding header and CRC).

        Args:
            function_code: Function code.
            request: Original request.

        Returns:
            Expected additional bytes to read.
        """
        if function_code in (FunctionCode.READ_COILS, FunctionCode.READ_DISCRETE_INPUTS):
            # Byte count + data + CRC
            count = struct.unpack(">H", request[4:6])[0]
            byte_count = (count + 7) // 8
            return 1 + byte_count + 2
        elif function_code in (
            FunctionCode.READ_HOLDING_REGISTERS,
            FunctionCode.READ_INPUT_REGISTERS,
        ):
            # Byte count + data + CRC
            count = struct.unpack(">H", request[4:6])[0]
            return 1 + (count * 2) + 2
        elif function_code in (
            FunctionCode.WRITE_SINGLE_COIL,
            FunctionCode.WRITE_SINGLE_REGISTER,
        ):
            # Echo: address + value + CRC
            return 4 + 2
        elif function_code in (
            FunctionCode.WRITE_MULTIPLE_COILS,
            FunctionCode.WRITE_MULTIPLE_REGISTERS,
        ):
            # Echo: address + quantity + CRC
            return 4 + 2
        else:
            # Default: read until timeout
            return 255

    def _simulate_response(self, request: bytes) -> bytes:
        """Generate simulated response.

        Args:
            request: Request PDU.

        Returns:
            Simulated response PDU.
        """
        slave_id = request[0]
        function_code = request[1]

        if function_code == FunctionCode.READ_COILS:
            count = struct.unpack(">H", request[4:6])[0]
            byte_count = (count + 7) // 8
            data = bytes(byte_count)  # All coils OFF
            return bytes([slave_id, function_code, byte_count]) + data

        elif function_code == FunctionCode.READ_DISCRETE_INPUTS:
            count = struct.unpack(">H", request[4:6])[0]
            byte_count = (count + 7) // 8
            data = bytes(byte_count)  # All inputs OFF
            return bytes([slave_id, function_code, byte_count]) + data

        elif function_code in (
            FunctionCode.READ_HOLDING_REGISTERS,
            FunctionCode.READ_INPUT_REGISTERS,
        ):
            count = struct.unpack(">H", request[4:6])[0]
            byte_count = count * 2
            data = bytes(byte_count)  # All registers 0
            return bytes([slave_id, function_code, byte_count]) + data

        elif function_code in (
            FunctionCode.WRITE_SINGLE_COIL,
            FunctionCode.WRITE_SINGLE_REGISTER,
        ):
            # Echo request
            return request

        elif function_code in (
            FunctionCode.WRITE_MULTIPLE_COILS,
            FunctionCode.WRITE_MULTIPLE_REGISTERS,
        ):
            # Echo address and quantity
            return request[:6]

        else:
            # Exception: illegal function
            return bytes([slave_id, function_code | 0x80, ExceptionCode.ILLEGAL_FUNCTION])

    # =========================================================================
    # Read Operations
    # =========================================================================

    def read_coils(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[bool]:
        """Read coils (function 0x01)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")
        if not 1 <= count <= 2000:
            raise ValueError("Count must be 1-2000")

        request = struct.pack(">BBHH", slave_id, FunctionCode.READ_COILS, address, count)
        response = self._send_receive(request)

        # Check for exception
        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])

        # Parse response
        response[2]
        coils = []
        for i in range(count):
            byte_idx = 3 + (i // 8)
            bit_idx = i % 8
            coils.append(bool((response[byte_idx] >> bit_idx) & 1))

        return coils

    def read_discrete_inputs(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[bool]:
        """Read discrete inputs (function 0x02)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")
        if not 1 <= count <= 2000:
            raise ValueError("Count must be 1-2000")

        request = struct.pack(">BBHH", slave_id, FunctionCode.READ_DISCRETE_INPUTS, address, count)
        response = self._send_receive(request)

        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])

        response[2]
        inputs = []
        for i in range(count):
            byte_idx = 3 + (i // 8)
            bit_idx = i % 8
            inputs.append(bool((response[byte_idx] >> bit_idx) & 1))

        return inputs

    def read_holding_registers(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[int]:
        """Read holding registers (function 0x03)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")
        if not 1 <= count <= 125:
            raise ValueError("Count must be 1-125")

        request = struct.pack(
            ">BBHH", slave_id, FunctionCode.READ_HOLDING_REGISTERS, address, count
        )
        response = self._send_receive(request)

        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])

        response[2]
        registers = []
        for i in range(count):
            value = struct.unpack(">H", response[3 + i * 2 : 5 + i * 2])[0]
            registers.append(value)

        return registers

    def read_input_registers(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[int]:
        """Read input registers (function 0x04)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")
        if not 1 <= count <= 125:
            raise ValueError("Count must be 1-125")

        request = struct.pack(">BBHH", slave_id, FunctionCode.READ_INPUT_REGISTERS, address, count)
        response = self._send_receive(request)

        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])

        response[2]
        registers = []
        for i in range(count):
            value = struct.unpack(">H", response[3 + i * 2 : 5 + i * 2])[0]
            registers.append(value)

        return registers

    # =========================================================================
    # Write Operations
    # =========================================================================

    def write_coil(
        self,
        slave_id: int,
        address: int,
        value: bool,
    ) -> None:
        """Write single coil (function 0x05)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")

        coil_value = COIL_ON if value else COIL_OFF
        request = struct.pack(
            ">BBHH", slave_id, FunctionCode.WRITE_SINGLE_COIL, address, coil_value
        )
        response = self._send_receive(request)

        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])

    def write_register(
        self,
        slave_id: int,
        address: int,
        value: int,
    ) -> None:
        """Write single register (function 0x06)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")
        if not 0 <= value <= 65535:
            raise ValueError("Register value must be 0-65535")

        request = struct.pack(">BBHH", slave_id, FunctionCode.WRITE_SINGLE_REGISTER, address, value)
        response = self._send_receive(request)

        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])

    def write_coils(
        self,
        slave_id: int,
        address: int,
        values: list[bool],
    ) -> None:
        """Write multiple coils (function 0x0F)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")
        if not 1 <= len(values) <= 1968:
            raise ValueError("Must write 1-1968 coils")

        count = len(values)
        byte_count = (count + 7) // 8

        # Pack coil values into bytes
        data = bytearray(byte_count)
        for i, value in enumerate(values):
            if value:
                data[i // 8] |= 1 << (i % 8)

        request = struct.pack(
            ">BBHHB", slave_id, FunctionCode.WRITE_MULTIPLE_COILS, address, count, byte_count
        ) + bytes(data)
        response = self._send_receive(request)

        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])

    def write_registers(
        self,
        slave_id: int,
        address: int,
        values: list[int],
    ) -> None:
        """Write multiple registers (function 0x10)."""
        if not 1 <= slave_id <= 247:
            raise ValueError("Slave ID must be 1-247")
        if not 1 <= len(values) <= 123:
            raise ValueError("Must write 1-123 registers")

        count = len(values)
        byte_count = count * 2

        # Pack register values
        data = b"".join(struct.pack(">H", v) for v in values)

        request = (
            struct.pack(
                ">BBHHB",
                slave_id,
                FunctionCode.WRITE_MULTIPLE_REGISTERS,
                address,
                count,
                byte_count,
            )
            + data
        )
        response = self._send_receive(request)

        if response[1] & 0x80:
            raise ModbusError(response[1] & 0x7F, response[2])


# =============================================================================
# Modbus TCP Implementation
# =============================================================================


class ModbusTCP(ModbusClient):
    """Modbus TCP client over TCP/IP connection."""

    def __init__(
        self,
        host: str,
        port: int = 502,
        timeout: float = 1.0,
        name: str = "modbus-tcp",
    ) -> None:
        """Initialize Modbus TCP client.

        Args:
            host: Server hostname or IP.
            port: TCP port.
            timeout: Connection and read timeout.
            name: Client name.
        """
        super().__init__(name)
        self._config = ModbusTCPConfig(
            host=host,
            port=port,
            timeout=timeout,
        )
        self._socket: socket.socket | None = None
        self._transaction_id = 0
        self._simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes")

    @property
    def config(self) -> ModbusTCPConfig:
        """TCP configuration."""
        return self._config

    def open(self) -> None:
        """Open TCP connection."""
        if self._is_open:
            return

        if self._simulation:
            logger.warning("⚠️ SIMULATION MODE - Modbus TCP simulated")
            self._is_open = True
            return

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(self._config.timeout)
            self._socket.connect((self._config.host, self._config.port))
            self._is_open = True
            logger.info("Modbus TCP connected: %s:%d", self._config.host, self._config.port)
        except builtins.TimeoutError as e:
            raise TimeoutError(
                f"Connection timeout to {self._config.host}:{self._config.port}"
            ) from e
        except Exception as e:
            raise CommunicationError(f"Failed to connect: {e}") from e

    def close(self) -> None:
        """Close TCP connection."""
        if self._socket:
            with contextlib.suppress(Exception):
                self._socket.close()
            self._socket = None
        self._is_open = False
        logger.info("Modbus TCP closed")

    def _next_transaction_id(self) -> int:
        """Get next transaction ID."""
        self._transaction_id = (self._transaction_id + 1) % 65536
        return self._transaction_id

    def _send_receive(self, slave_id: int, request_pdu: bytes) -> bytes:
        """Send request and receive response.

        Args:
            slave_id: Unit identifier.
            request_pdu: Request PDU (function code + data).

        Returns:
            Response PDU (function code + data).
        """
        self._stats.requests_sent += 1
        self._stats.last_request_time = time.time()

        if self._simulation:
            return self._simulate_response(slave_id, request_pdu)

        if not self._socket:
            raise CommunicationError("Socket not connected")

        # Build MBAP header
        transaction_id = self._next_transaction_id()
        protocol_id = 0  # Modbus protocol
        length = len(request_pdu) + 1  # PDU + unit ID

        header = struct.pack(">HHHB", transaction_id, protocol_id, length, slave_id)
        frame = header + request_pdu

        # Send
        try:
            self._socket.sendall(frame)
        except OSError as e:
            self._stats.errors += 1
            raise CommunicationError(f"Send failed: {e}") from e

        # Receive MBAP header (7 bytes)
        try:
            header_data = self._socket.recv(7)
            if len(header_data) < 7:
                raise CommunicationError("Incomplete MBAP header")

            (
                resp_trans_id,
                _resp_proto_id,
                resp_length,
                _resp_unit_id,
            ) = struct.unpack(">HHHB", header_data)

            # Verify transaction ID
            if resp_trans_id != transaction_id:
                logger.warning(
                    "Transaction ID mismatch: expected %d, got %d",
                    transaction_id,
                    resp_trans_id,
                )

            # Receive PDU
            pdu_length = resp_length - 1
            pdu_data = self._socket.recv(pdu_length)

            self._stats.responses_received += 1
            self._stats.last_response_time = time.time()

            return pdu_data

        except builtins.TimeoutError as e:
            self._stats.timeouts += 1
            raise TimeoutError("Modbus TCP response timeout") from e
        except OSError as e:
            self._stats.errors += 1
            raise CommunicationError(f"Receive failed: {e}") from e

    def _simulate_response(self, slave_id: int, request_pdu: bytes) -> bytes:
        """Generate simulated response.

        Args:
            slave_id: Unit identifier.
            request_pdu: Request PDU.

        Returns:
            Simulated response PDU.
        """
        function_code = request_pdu[0]

        if function_code in (FunctionCode.READ_COILS, FunctionCode.READ_DISCRETE_INPUTS):
            count = struct.unpack(">H", request_pdu[3:5])[0]
            byte_count = (count + 7) // 8
            data = bytes(byte_count)
            return bytes([function_code, byte_count]) + data

        elif function_code in (
            FunctionCode.READ_HOLDING_REGISTERS,
            FunctionCode.READ_INPUT_REGISTERS,
        ):
            count = struct.unpack(">H", request_pdu[3:5])[0]
            byte_count = count * 2
            data = bytes(byte_count)
            return bytes([function_code, byte_count]) + data

        elif function_code in (
            FunctionCode.WRITE_SINGLE_COIL,
            FunctionCode.WRITE_SINGLE_REGISTER,
        ):
            return request_pdu

        elif function_code in (
            FunctionCode.WRITE_MULTIPLE_COILS,
            FunctionCode.WRITE_MULTIPLE_REGISTERS,
        ):
            return request_pdu[:5]

        else:
            return bytes([function_code | 0x80, ExceptionCode.ILLEGAL_FUNCTION])

    # =========================================================================
    # Read Operations
    # =========================================================================

    def read_coils(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[bool]:
        """Read coils (function 0x01)."""
        if not 1 <= count <= 2000:
            raise ValueError("Count must be 1-2000")

        pdu = struct.pack(">BHH", FunctionCode.READ_COILS, address, count)
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])

        response[1]
        coils = []
        for i in range(count):
            byte_idx = 2 + (i // 8)
            bit_idx = i % 8
            coils.append(bool((response[byte_idx] >> bit_idx) & 1))

        return coils

    def read_discrete_inputs(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[bool]:
        """Read discrete inputs (function 0x02)."""
        if not 1 <= count <= 2000:
            raise ValueError("Count must be 1-2000")

        pdu = struct.pack(">BHH", FunctionCode.READ_DISCRETE_INPUTS, address, count)
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])

        response[1]
        inputs = []
        for i in range(count):
            byte_idx = 2 + (i // 8)
            bit_idx = i % 8
            inputs.append(bool((response[byte_idx] >> bit_idx) & 1))

        return inputs

    def read_holding_registers(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[int]:
        """Read holding registers (function 0x03)."""
        if not 1 <= count <= 125:
            raise ValueError("Count must be 1-125")

        pdu = struct.pack(">BHH", FunctionCode.READ_HOLDING_REGISTERS, address, count)
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])

        response[1]
        registers = []
        for i in range(count):
            value = struct.unpack(">H", response[2 + i * 2 : 4 + i * 2])[0]
            registers.append(value)

        return registers

    def read_input_registers(
        self,
        slave_id: int,
        address: int,
        count: int = 1,
    ) -> list[int]:
        """Read input registers (function 0x04)."""
        if not 1 <= count <= 125:
            raise ValueError("Count must be 1-125")

        pdu = struct.pack(">BHH", FunctionCode.READ_INPUT_REGISTERS, address, count)
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])

        response[1]
        registers = []
        for i in range(count):
            value = struct.unpack(">H", response[2 + i * 2 : 4 + i * 2])[0]
            registers.append(value)

        return registers

    # =========================================================================
    # Write Operations
    # =========================================================================

    def write_coil(
        self,
        slave_id: int,
        address: int,
        value: bool,
    ) -> None:
        """Write single coil (function 0x05)."""
        coil_value = COIL_ON if value else COIL_OFF
        pdu = struct.pack(">BHH", FunctionCode.WRITE_SINGLE_COIL, address, coil_value)
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])

    def write_register(
        self,
        slave_id: int,
        address: int,
        value: int,
    ) -> None:
        """Write single register (function 0x06)."""
        if not 0 <= value <= 65535:
            raise ValueError("Register value must be 0-65535")

        pdu = struct.pack(">BHH", FunctionCode.WRITE_SINGLE_REGISTER, address, value)
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])

    def write_coils(
        self,
        slave_id: int,
        address: int,
        values: list[bool],
    ) -> None:
        """Write multiple coils (function 0x0F)."""
        if not 1 <= len(values) <= 1968:
            raise ValueError("Must write 1-1968 coils")

        count = len(values)
        byte_count = (count + 7) // 8

        data = bytearray(byte_count)
        for i, value in enumerate(values):
            if value:
                data[i // 8] |= 1 << (i % 8)

        pdu = struct.pack(
            ">BHHB", FunctionCode.WRITE_MULTIPLE_COILS, address, count, byte_count
        ) + bytes(data)
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])

    def write_registers(
        self,
        slave_id: int,
        address: int,
        values: list[int],
    ) -> None:
        """Write multiple registers (function 0x10)."""
        if not 1 <= len(values) <= 123:
            raise ValueError("Must write 1-123 registers")

        count = len(values)
        byte_count = count * 2
        data = b"".join(struct.pack(">H", v) for v in values)

        pdu = (
            struct.pack(">BHHB", FunctionCode.WRITE_MULTIPLE_REGISTERS, address, count, byte_count)
            + data
        )
        response = self._send_receive(slave_id, pdu)

        if response[0] & 0x80:
            raise ModbusError(response[0] & 0x7F, response[1])


# =============================================================================
# Simulated Modbus Server (for testing)
# =============================================================================


class SimulatedModbusServer:
    """Simulated Modbus server for testing.

    Holds coils, discrete inputs, holding registers, and input registers
    that can be read/written by test code.
    """

    def __init__(
        self,
        coils_size: int = 1000,
        discrete_inputs_size: int = 1000,
        holding_registers_size: int = 1000,
        input_registers_size: int = 1000,
    ) -> None:
        """Initialize simulated server.

        Args:
            coils_size: Number of coils.
            discrete_inputs_size: Number of discrete inputs.
            holding_registers_size: Number of holding registers.
            input_registers_size: Number of input registers.
        """
        self.coils = [False] * coils_size
        self.discrete_inputs = [False] * discrete_inputs_size
        self.holding_registers = [0] * holding_registers_size
        self.input_registers = [0] * input_registers_size

    def set_coil(self, address: int, value: bool) -> None:
        """Set a coil value."""
        if 0 <= address < len(self.coils):
            self.coils[address] = value

    def get_coil(self, address: int) -> bool:
        """Get a coil value."""
        if 0 <= address < len(self.coils):
            return self.coils[address]
        return False

    def set_discrete_input(self, address: int, value: bool) -> None:
        """Set a discrete input value."""
        if 0 <= address < len(self.discrete_inputs):
            self.discrete_inputs[address] = value

    def set_holding_register(self, address: int, value: int) -> None:
        """Set a holding register value."""
        if 0 <= address < len(self.holding_registers):
            self.holding_registers[address] = value & 0xFFFF

    def get_holding_register(self, address: int) -> int:
        """Get a holding register value."""
        if 0 <= address < len(self.holding_registers):
            return self.holding_registers[address]
        return 0

    def set_input_register(self, address: int, value: int) -> None:
        """Set an input register value."""
        if 0 <= address < len(self.input_registers):
            self.input_registers[address] = value & 0xFFFF
