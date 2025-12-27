"""CANopen protocol implementation for industrial automation.

CANopen is a high-level communication protocol based on CAN,
widely used in industrial automation, robotics, and embedded systems.

This module provides CANopen node implementation with support for:
- NMT (Network Management): Start, stop, reset nodes
- SDO (Service Data Object): Read/write object dictionary entries
- PDO (Process Data Object): Real-time data exchange
- SYNC: Synchronized data acquisition
- EMCY (Emergency): Error reporting
- Heartbeat: Node monitoring

Example:
    >>> from robo_infra.core.can_bus import get_can
    >>> from robo_infra.protocols.canopen import CANOpenNode, CANOpenMaster
    >>>
    >>> # Create CAN bus and master
    >>> can = get_can("socketcan", "can0", 500000)
    >>> can.open()
    >>> master = CANOpenMaster(can)
    >>>
    >>> # Access a node
    >>> node = master.get_node(1)
    >>> node.nmt_start()
    >>>
    >>> # Read object dictionary entry
    >>> vendor_id = node.sdo_read(0x1018, 1)  # Identity object, vendor ID
    >>>
    >>> # Write parameter
    >>> node.sdo_write(0x6040, 0, b"\\x06\\x00")  # Control word
    >>>
    >>> can.close()

CANopen Object Dictionary:
    0x1000-0x1FFF: Communication profile
    0x2000-0x5FFF: Manufacturer-specific
    0x6000-0x9FFF: Device profile

Common CIAs (CANopen in Automation) profiles:
    - CiA 301: CANopen application layer
    - CiA 402: Drives and motion control
    - CiA 406: Encoders
    - CiA 408: Hydraulic drives
"""

from __future__ import annotations

import asyncio
import logging
import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import TYPE_CHECKING, Any

from robo_infra.core.exceptions import TimeoutError


if TYPE_CHECKING:
    from robo_infra.core.can_bus import CANBus, CANMessage

logger = logging.getLogger(__name__)


# =============================================================================
# CANopen Constants
# =============================================================================


class NMTCommand(IntEnum):
    """NMT (Network Management) commands."""

    START_REMOTE_NODE = 0x01
    STOP_REMOTE_NODE = 0x02
    ENTER_PRE_OPERATIONAL = 0x80
    RESET_NODE = 0x81
    RESET_COMMUNICATION = 0x82


class NMTState(IntEnum):
    """NMT state machine states."""

    INITIALIZING = 0
    STOPPED = 4
    OPERATIONAL = 5
    PRE_OPERATIONAL = 127


class SDOCommand(IntEnum):
    """SDO command specifiers."""

    # Download (write) commands
    DOWNLOAD_SEGMENT_REQUEST = 0x00
    DOWNLOAD_INITIATE_REQUEST = 0x20
    DOWNLOAD_INITIATE_RESPONSE = 0x60
    DOWNLOAD_SEGMENT_RESPONSE = 0x20

    # Upload (read) commands
    UPLOAD_SEGMENT_REQUEST = 0x60
    UPLOAD_INITIATE_REQUEST = 0x40
    UPLOAD_INITIATE_RESPONSE = 0x40
    UPLOAD_SEGMENT_RESPONSE = 0x00

    # Abort
    ABORT_TRANSFER = 0x80


class SDOAbortCode(IntEnum):
    """SDO abort codes."""

    TOGGLE_BIT_NOT_ALTERNATED = 0x05030000
    SDO_PROTOCOL_TIMEOUT = 0x05040000
    INVALID_COMMAND = 0x05040001
    INVALID_BLOCK_SIZE = 0x05040002
    INVALID_SEQUENCE_NUMBER = 0x05040003
    CRC_ERROR = 0x05040004
    OUT_OF_MEMORY = 0x05040005
    OBJECT_DOES_NOT_EXIST = 0x06020000
    OBJECT_CANNOT_BE_MAPPED = 0x06040041
    PDO_LENGTH_EXCEEDED = 0x06040042
    GENERAL_INCOMPATIBILITY = 0x06040043
    HARDWARE_ERROR = 0x06060000
    DATA_TYPE_MISMATCH = 0x06070010
    DATA_TYPE_TOO_LONG = 0x06070012
    DATA_TYPE_TOO_SHORT = 0x06070013
    SUBINDEX_DOES_NOT_EXIST = 0x06090011
    VALUE_TOO_HIGH = 0x06090030
    VALUE_TOO_LOW = 0x06090031
    MAXIMUM_LESS_THAN_MINIMUM = 0x06090032
    GENERAL_ERROR = 0x08000000
    DATA_STORAGE_ERROR = 0x08000020
    LOCAL_CONTROL_ERROR = 0x08000021
    DEVICE_STATE_ERROR = 0x08000022


class COB_ID(IntEnum):
    """CANopen COB-ID function codes."""

    NMT = 0x000
    SYNC = 0x080
    EMCY = 0x080  # + node_id
    TIMESTAMP = 0x100
    TPDO1 = 0x180  # + node_id
    RPDO1 = 0x200  # + node_id
    TPDO2 = 0x280  # + node_id
    RPDO2 = 0x300  # + node_id
    TPDO3 = 0x380  # + node_id
    RPDO3 = 0x400  # + node_id
    TPDO4 = 0x480  # + node_id
    RPDO4 = 0x500  # + node_id
    SDO_TX = 0x580  # + node_id (node response)
    SDO_RX = 0x600  # + node_id (master request)
    HEARTBEAT = 0x700  # + node_id


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class SDOError(Exception):
    """SDO transfer error."""

    abort_code: int
    message: str = ""

    def __post_init__(self) -> None:
        """Generate message from abort code."""
        if not self.message:
            self.message = self._get_abort_message()

    def _get_abort_message(self) -> str:
        """Get human-readable abort message."""
        messages = {
            SDOAbortCode.OBJECT_DOES_NOT_EXIST: "Object does not exist",
            SDOAbortCode.SUBINDEX_DOES_NOT_EXIST: "Subindex does not exist",
            SDOAbortCode.DATA_TYPE_MISMATCH: "Data type mismatch",
            SDOAbortCode.DEVICE_STATE_ERROR: "Device state error",
            SDOAbortCode.GENERAL_ERROR: "General error",
            SDOAbortCode.SDO_PROTOCOL_TIMEOUT: "SDO protocol timeout",
        }
        return messages.get(self.abort_code, f"Unknown error (0x{self.abort_code:08X})")


@dataclass
class ObjectEntry:
    """CANopen object dictionary entry.

    Attributes:
        index: Object index (0x0000-0xFFFF).
        subindex: Object subindex (0x00-0xFF).
        data: Entry data.
        name: Human-readable name.
        data_type: Data type code.
        access: Access type (ro, wo, rw).
    """

    index: int
    subindex: int
    data: bytes = field(default_factory=bytes)
    name: str = ""
    data_type: int = 0
    access: str = "rw"


@dataclass
class PDOMapping:
    """PDO mapping entry.

    Attributes:
        index: Object dictionary index.
        subindex: Object dictionary subindex.
        bit_length: Number of bits for this mapping.
    """

    index: int
    subindex: int
    bit_length: int

    def to_bytes(self) -> bytes:
        """Convert to 4-byte PDO mapping format."""
        value = (self.index << 16) | (self.subindex << 8) | self.bit_length
        return struct.pack("<I", value)

    @classmethod
    def from_bytes(cls, data: bytes) -> PDOMapping:
        """Create from 4-byte PDO mapping format."""
        value = struct.unpack("<I", data)[0]
        return cls(
            index=(value >> 16) & 0xFFFF,
            subindex=(value >> 8) & 0xFF,
            bit_length=value & 0xFF,
        )


@dataclass
class EMCYMessage:
    """Emergency message.

    Attributes:
        node_id: Source node ID.
        error_code: Emergency error code.
        error_register: Error register value.
        manufacturer_data: Manufacturer-specific data.
        timestamp: When the message was received.
    """

    node_id: int
    error_code: int
    error_register: int
    manufacturer_data: bytes
    timestamp: float = field(default_factory=time.time)


# =============================================================================
# CANopen Node
# =============================================================================


class CANOpenNode:
    """CANopen node abstraction.

    Represents a single CANopen device on the network.
    Provides methods for NMT control, SDO read/write, and PDO access.
    """

    def __init__(
        self,
        bus: CANBus,
        node_id: int,
        sdo_timeout: float = 1.0,
        name: str | None = None,
    ) -> None:
        """Initialize CANopen node.

        Args:
            bus: CAN bus instance.
            node_id: Node ID (1-127).
            sdo_timeout: SDO transfer timeout in seconds.
            name: Optional node name.

        Raises:
            ValueError: If node_id is out of range.
        """
        if not 1 <= node_id <= 127:
            raise ValueError(f"Node ID must be 1-127, got {node_id}")

        self._bus = bus
        self._node_id = node_id
        self._sdo_timeout = sdo_timeout
        self._name = name or f"Node-{node_id}"
        self._state = NMTState.INITIALIZING
        self._last_heartbeat: float = 0.0

        # COB-IDs for this node
        self._sdo_rx_cobid = COB_ID.SDO_RX + node_id
        self._sdo_tx_cobid = COB_ID.SDO_TX + node_id
        self._heartbeat_cobid = COB_ID.HEARTBEAT + node_id
        self._emcy_cobid = COB_ID.EMCY + node_id

    @property
    def node_id(self) -> int:
        """Node ID."""
        return self._node_id

    @property
    def name(self) -> str:
        """Node name."""
        return self._name

    @property
    def state(self) -> NMTState:
        """Current NMT state."""
        return self._state

    @property
    def last_heartbeat(self) -> float:
        """Timestamp of last heartbeat."""
        return self._last_heartbeat

    # =========================================================================
    # NMT (Network Management)
    # =========================================================================

    def nmt_send(self, command: NMTCommand) -> None:
        """Send NMT command to this node.

        Args:
            command: NMT command to send.
        """
        data = bytes([command, self._node_id])
        self._bus.send(COB_ID.NMT, data)
        logger.debug("NMT command %s sent to node %d", command.name, self._node_id)

    def nmt_start(self) -> None:
        """Start the node (enter operational state)."""
        self.nmt_send(NMTCommand.START_REMOTE_NODE)
        self._state = NMTState.OPERATIONAL

    def nmt_stop(self) -> None:
        """Stop the node (enter stopped state)."""
        self.nmt_send(NMTCommand.STOP_REMOTE_NODE)
        self._state = NMTState.STOPPED

    def nmt_pre_operational(self) -> None:
        """Enter pre-operational state."""
        self.nmt_send(NMTCommand.ENTER_PRE_OPERATIONAL)
        self._state = NMTState.PRE_OPERATIONAL

    def nmt_reset(self) -> None:
        """Reset the node."""
        self.nmt_send(NMTCommand.RESET_NODE)
        self._state = NMTState.INITIALIZING

    def nmt_reset_communication(self) -> None:
        """Reset communication."""
        self.nmt_send(NMTCommand.RESET_COMMUNICATION)

    # =========================================================================
    # SDO (Service Data Object) - Read
    # =========================================================================

    def sdo_read(self, index: int, subindex: int = 0) -> bytes:
        """Read from object dictionary using SDO upload.

        Args:
            index: Object dictionary index (0x0000-0xFFFF).
            subindex: Object dictionary subindex (0x00-0xFF).

        Returns:
            Data read from object dictionary.

        Raises:
            SDOError: If transfer fails.
            TimeoutError: If no response received.
        """
        # Build SDO upload initiate request
        # CCS=2 (upload initiate), index, subindex
        request = bytes(
            [
                0x40,  # Command: upload initiate request
                index & 0xFF,
                (index >> 8) & 0xFF,
                subindex,
                0,
                0,
                0,
                0,  # Reserved
            ]
        )

        self._bus.send(self._sdo_rx_cobid, request)

        # Wait for response
        response = self._wait_sdo_response()

        # Check for abort
        if response.data[0] == 0x80:
            abort_code = struct.unpack("<I", response.data[4:8])[0]
            raise SDOError(abort_code)

        # Parse response
        command = response.data[0]

        # Check if expedited transfer (data in this frame)
        if command & 0x02:  # Expedited
            # Determine data length
            if command & 0x01:  # Size indicated
                n = (command >> 2) & 0x03  # Number of empty bytes
                data_len = 4 - n
            else:
                data_len = 4
            return bytes(response.data[4 : 4 + data_len])

        # Segmented transfer - read data length
        data_len = struct.unpack("<I", response.data[4:8])[0]
        data = bytearray()
        toggle = 0

        # Read segments
        while len(data) < data_len:
            # Send segment request
            segment_request = bytes([0x60 | (toggle << 4)] + [0] * 7)
            self._bus.send(self._sdo_rx_cobid, segment_request)

            response = self._wait_sdo_response()

            if response.data[0] == 0x80:
                abort_code = struct.unpack("<I", response.data[4:8])[0]
                raise SDOError(abort_code)

            # Verify toggle bit
            resp_toggle = (response.data[0] >> 4) & 0x01
            if resp_toggle != toggle:
                raise SDOError(SDOAbortCode.TOGGLE_BIT_NOT_ALTERNATED)

            # Extract segment data
            n = (response.data[0] >> 1) & 0x07  # Empty bytes
            segment_len = 7 - n
            data.extend(response.data[1 : 1 + segment_len])

            # Check if last segment
            if response.data[0] & 0x01:
                break

            toggle ^= 1

        return bytes(data[:data_len])

    def sdo_read_u8(self, index: int, subindex: int = 0) -> int:
        """Read unsigned 8-bit integer from object dictionary."""
        data = self.sdo_read(index, subindex)
        return struct.unpack("<B", data[:1])[0]

    def sdo_read_u16(self, index: int, subindex: int = 0) -> int:
        """Read unsigned 16-bit integer from object dictionary."""
        data = self.sdo_read(index, subindex)
        return struct.unpack("<H", data[:2])[0]

    def sdo_read_u32(self, index: int, subindex: int = 0) -> int:
        """Read unsigned 32-bit integer from object dictionary."""
        data = self.sdo_read(index, subindex)
        return struct.unpack("<I", data[:4])[0]

    def sdo_read_i8(self, index: int, subindex: int = 0) -> int:
        """Read signed 8-bit integer from object dictionary."""
        data = self.sdo_read(index, subindex)
        return struct.unpack("<b", data[:1])[0]

    def sdo_read_i16(self, index: int, subindex: int = 0) -> int:
        """Read signed 16-bit integer from object dictionary."""
        data = self.sdo_read(index, subindex)
        return struct.unpack("<h", data[:2])[0]

    def sdo_read_i32(self, index: int, subindex: int = 0) -> int:
        """Read signed 32-bit integer from object dictionary."""
        data = self.sdo_read(index, subindex)
        return struct.unpack("<i", data[:4])[0]

    def sdo_read_string(self, index: int, subindex: int = 0) -> str:
        """Read visible string from object dictionary."""
        data = self.sdo_read(index, subindex)
        return data.rstrip(b"\x00").decode("ascii", errors="replace")

    # =========================================================================
    # SDO (Service Data Object) - Write
    # =========================================================================

    def sdo_write(self, index: int, subindex: int, data: bytes) -> None:
        """Write to object dictionary using SDO download.

        Args:
            index: Object dictionary index.
            subindex: Object dictionary subindex.
            data: Data to write.

        Raises:
            SDOError: If transfer fails.
            TimeoutError: If no response received.
        """
        data_len = len(data)

        if data_len <= 4:
            # Expedited transfer
            self._sdo_write_expedited(index, subindex, data)
        else:
            # Segmented transfer
            self._sdo_write_segmented(index, subindex, data)

    def _sdo_write_expedited(self, index: int, subindex: int, data: bytes) -> None:
        """Write using expedited SDO transfer (<=4 bytes)."""
        data_len = len(data)
        n = 4 - data_len  # Number of empty bytes

        # Command: download initiate, expedited, size indicated
        command = 0x23 | (n << 2)

        # Pad data to 4 bytes
        padded_data = data + bytes(4 - data_len)

        request = (
            bytes(
                [
                    command,
                    index & 0xFF,
                    (index >> 8) & 0xFF,
                    subindex,
                ]
            )
            + padded_data
        )

        self._bus.send(self._sdo_rx_cobid, request)

        # Wait for response
        response = self._wait_sdo_response()

        if response.data[0] == 0x80:
            abort_code = struct.unpack("<I", response.data[4:8])[0]
            raise SDOError(abort_code)

        # Verify response is download initiate response (0x60)
        if (response.data[0] & 0xE0) != 0x60:
            raise SDOError(SDOAbortCode.INVALID_COMMAND)

    def _sdo_write_segmented(self, index: int, subindex: int, data: bytes) -> None:
        """Write using segmented SDO transfer (>4 bytes)."""
        data_len = len(data)

        # Download initiate request with size
        request = bytes(
            [
                0x21,  # Download initiate, size indicated, not expedited
                index & 0xFF,
                (index >> 8) & 0xFF,
                subindex,
            ]
        ) + struct.pack("<I", data_len)

        self._bus.send(self._sdo_rx_cobid, request)

        response = self._wait_sdo_response()
        if response.data[0] == 0x80:
            abort_code = struct.unpack("<I", response.data[4:8])[0]
            raise SDOError(abort_code)

        # Send segments
        toggle = 0
        offset = 0

        while offset < data_len:
            # Calculate segment
            remaining = data_len - offset
            segment_len = min(7, remaining)
            last_segment = remaining <= 7
            n = 7 - segment_len if last_segment else 0

            # Build segment command
            command = (toggle << 4) | (n << 1) | (1 if last_segment else 0)

            # Pad segment to 7 bytes
            segment_data = data[offset : offset + segment_len]
            segment_data += bytes(7 - len(segment_data))

            request = bytes([command]) + segment_data
            self._bus.send(self._sdo_rx_cobid, request)

            response = self._wait_sdo_response()
            if response.data[0] == 0x80:
                abort_code = struct.unpack("<I", response.data[4:8])[0]
                raise SDOError(abort_code)

            # Verify toggle
            resp_toggle = (response.data[0] >> 4) & 0x01
            if resp_toggle != toggle:
                raise SDOError(SDOAbortCode.TOGGLE_BIT_NOT_ALTERNATED)

            toggle ^= 1
            offset += segment_len

    def sdo_write_u8(self, index: int, subindex: int, value: int) -> None:
        """Write unsigned 8-bit integer to object dictionary."""
        self.sdo_write(index, subindex, struct.pack("<B", value))

    def sdo_write_u16(self, index: int, subindex: int, value: int) -> None:
        """Write unsigned 16-bit integer to object dictionary."""
        self.sdo_write(index, subindex, struct.pack("<H", value))

    def sdo_write_u32(self, index: int, subindex: int, value: int) -> None:
        """Write unsigned 32-bit integer to object dictionary."""
        self.sdo_write(index, subindex, struct.pack("<I", value))

    def sdo_write_i8(self, index: int, subindex: int, value: int) -> None:
        """Write signed 8-bit integer to object dictionary."""
        self.sdo_write(index, subindex, struct.pack("<b", value))

    def sdo_write_i16(self, index: int, subindex: int, value: int) -> None:
        """Write signed 16-bit integer to object dictionary."""
        self.sdo_write(index, subindex, struct.pack("<h", value))

    def sdo_write_i32(self, index: int, subindex: int, value: int) -> None:
        """Write signed 32-bit integer to object dictionary."""
        self.sdo_write(index, subindex, struct.pack("<i", value))

    # =========================================================================
    # PDO (Process Data Object)
    # =========================================================================

    def pdo_read(self, pdo_number: int) -> bytes | None:
        """Read latest TPDO data.

        Args:
            pdo_number: PDO number (1-4).

        Returns:
            PDO data or None if not received.
        """
        # TPDO COB-IDs
        tpdo_base = [COB_ID.TPDO1, COB_ID.TPDO2, COB_ID.TPDO3, COB_ID.TPDO4]
        if not 1 <= pdo_number <= 4:
            raise ValueError("PDO number must be 1-4")

        cobid = tpdo_base[pdo_number - 1] + self._node_id

        # Look for matching message in receive buffer
        msg = self._bus.recv(timeout=0.0)
        while msg:
            if msg.arbitration_id == cobid:
                return msg.data
            msg = self._bus.recv(timeout=0.0)

        return None

    def pdo_write(self, pdo_number: int, data: bytes) -> None:
        """Write RPDO data.

        Args:
            pdo_number: PDO number (1-4).
            data: PDO data (0-8 bytes).
        """
        rpdo_base = [COB_ID.RPDO1, COB_ID.RPDO2, COB_ID.RPDO3, COB_ID.RPDO4]
        if not 1 <= pdo_number <= 4:
            raise ValueError("PDO number must be 1-4")

        cobid = rpdo_base[pdo_number - 1] + self._node_id
        self._bus.send(cobid, data)

    def pdo_configure(
        self,
        pdo_number: int,
        is_transmit: bool,
        mappings: list[PDOMapping],
        transmission_type: int = 255,
    ) -> None:
        """Configure PDO mapping.

        Args:
            pdo_number: PDO number (1-4).
            is_transmit: True for TPDO, False for RPDO.
            mappings: List of PDO mappings.
            transmission_type: PDO transmission type (255=async).
        """
        if not 1 <= pdo_number <= 4:
            raise ValueError("PDO number must be 1-4")

        # Determine object dictionary indices
        if is_transmit:
            comm_index = 0x1800 + pdo_number - 1
            map_index = 0x1A00 + pdo_number - 1
        else:
            comm_index = 0x1400 + pdo_number - 1
            map_index = 0x1600 + pdo_number - 1

        # Disable PDO for configuration
        cobid = self.sdo_read_u32(comm_index, 1)
        self.sdo_write_u32(comm_index, 1, cobid | 0x80000000)

        # Clear mapping
        self.sdo_write_u8(map_index, 0, 0)

        # Write new mappings
        for i, mapping in enumerate(mappings):
            self.sdo_write(map_index, i + 1, mapping.to_bytes())

        # Set number of mapped objects
        self.sdo_write_u8(map_index, 0, len(mappings))

        # Set transmission type
        self.sdo_write_u8(comm_index, 2, transmission_type)

        # Re-enable PDO
        self.sdo_write_u32(comm_index, 1, cobid & ~0x80000000)

    # =========================================================================
    # Helpers
    # =========================================================================

    def _wait_sdo_response(self) -> CANMessage:
        """Wait for SDO response from this node.

        Returns:
            Received SDO response message.

        Raises:
            TimeoutError: If no response received.
        """
        deadline = time.time() + self._sdo_timeout

        while time.time() < deadline:
            msg = self._bus.recv(timeout=0.1)
            if msg and msg.arbitration_id == self._sdo_tx_cobid:
                return msg

        raise TimeoutError(f"SDO timeout waiting for response from node {self._node_id}")

    def process_heartbeat(self, data: bytes) -> None:
        """Process received heartbeat message.

        Args:
            data: Heartbeat message data (1 byte = NMT state).
        """
        if len(data) >= 1:
            self._state = NMTState(data[0])
            self._last_heartbeat = time.time()

    def is_alive(self, timeout: float = 2.0) -> bool:
        """Check if node is alive (heartbeat received recently).

        Args:
            timeout: Maximum time since last heartbeat.

        Returns:
            True if node is responding.
        """
        return (time.time() - self._last_heartbeat) < timeout


# =============================================================================
# CANopen Master
# =============================================================================


class CANOpenMaster:
    """CANopen network master.

    Manages multiple CANopen nodes on a network and provides
    network-wide services like SYNC generation and NMT broadcast.
    """

    def __init__(
        self,
        bus: CANBus,
        sdo_timeout: float = 1.0,
    ) -> None:
        """Initialize CANopen master.

        Args:
            bus: CAN bus instance.
            sdo_timeout: Default SDO timeout for nodes.
        """
        self._bus = bus
        self._sdo_timeout = sdo_timeout
        self._nodes: dict[int, CANOpenNode] = {}
        self._emcy_callback: Any = None

    @property
    def bus(self) -> CANBus:
        """CAN bus instance."""
        return self._bus

    @property
    def nodes(self) -> dict[int, CANOpenNode]:
        """Dictionary of managed nodes."""
        return self._nodes.copy()

    def get_node(self, node_id: int, name: str | None = None) -> CANOpenNode:
        """Get or create a node instance.

        Args:
            node_id: Node ID (1-127).
            name: Optional node name.

        Returns:
            CANOpenNode instance.
        """
        if node_id not in self._nodes:
            self._nodes[node_id] = CANOpenNode(
                self._bus,
                node_id,
                sdo_timeout=self._sdo_timeout,
                name=name,
            )
        return self._nodes[node_id]

    def remove_node(self, node_id: int) -> None:
        """Remove a node from management.

        Args:
            node_id: Node ID to remove.
        """
        self._nodes.pop(node_id, None)

    # =========================================================================
    # Network Management
    # =========================================================================

    def nmt_broadcast(self, command: NMTCommand) -> None:
        """Send NMT command to all nodes.

        Args:
            command: NMT command to broadcast.
        """
        data = bytes([command, 0])  # Node ID 0 = broadcast
        self._bus.send(COB_ID.NMT, data)
        logger.info("NMT broadcast: %s", command.name)

    def nmt_start_all(self) -> None:
        """Start all nodes."""
        self.nmt_broadcast(NMTCommand.START_REMOTE_NODE)

    def nmt_stop_all(self) -> None:
        """Stop all nodes."""
        self.nmt_broadcast(NMTCommand.STOP_REMOTE_NODE)

    def nmt_reset_all(self) -> None:
        """Reset all nodes."""
        self.nmt_broadcast(NMTCommand.RESET_NODE)

    # =========================================================================
    # SYNC
    # =========================================================================

    def send_sync(self, counter: int = 0) -> None:
        """Send SYNC message.

        Args:
            counter: Optional SYNC counter (0-240).
        """
        data = bytes([counter]) if counter else b""
        self._bus.send(COB_ID.SYNC, data)

    async def sync_loop(
        self,
        interval: float = 0.01,
        with_counter: bool = False,
    ) -> None:
        """Run SYNC generation loop.

        Args:
            interval: SYNC interval in seconds.
            with_counter: Include SYNC counter.
        """
        counter = 0
        while True:
            self.send_sync(counter if with_counter else 0)
            if with_counter:
                counter = (counter + 1) % 241
            await asyncio.sleep(interval)

    # =========================================================================
    # Message Processing
    # =========================================================================

    def process_message(self, msg: CANMessage) -> None:
        """Process received CAN message.

        Args:
            msg: Received CAN message.
        """
        arb_id = msg.arbitration_id

        # Check for heartbeat (0x700 + node_id)
        if 0x701 <= arb_id <= 0x77F:
            node_id = arb_id - COB_ID.HEARTBEAT
            if node_id in self._nodes:
                self._nodes[node_id].process_heartbeat(msg.data)

        # Check for emergency (0x080 + node_id)
        elif 0x081 <= arb_id <= 0x0FF:
            node_id = arb_id - COB_ID.EMCY
            self._process_emcy(node_id, msg.data)

    def _process_emcy(self, node_id: int, data: bytes) -> None:
        """Process emergency message.

        Args:
            node_id: Source node ID.
            data: Emergency message data.
        """
        if len(data) >= 8:
            error_code = struct.unpack("<H", data[0:2])[0]
            error_register = data[2]
            manufacturer_data = data[3:8]

            emcy = EMCYMessage(
                node_id=node_id,
                error_code=error_code,
                error_register=error_register,
                manufacturer_data=manufacturer_data,
            )

            logger.warning(
                "EMCY from node %d: error=0x%04X, register=0x%02X",
                node_id,
                error_code,
                error_register,
            )

            if self._emcy_callback:
                self._emcy_callback(emcy)

    def set_emcy_callback(self, callback: Any) -> None:
        """Set callback for emergency messages.

        Args:
            callback: Function taking EMCYMessage parameter.
        """
        self._emcy_callback = callback

    def scan_nodes(self, timeout: float = 0.1) -> list[int]:
        """Scan for active nodes on the network.

        Uses SDO read of device type (0x1000) to detect nodes.

        Args:
            timeout: Timeout per node scan.

        Returns:
            List of responding node IDs.
        """
        active_nodes = []
        original_timeout = self._sdo_timeout

        for node_id in range(1, 128):
            node = CANOpenNode(self._bus, node_id, sdo_timeout=timeout)
            try:
                node.sdo_read(0x1000, 0)  # Device type
                active_nodes.append(node_id)
                logger.debug("Found node %d", node_id)
            except (SDOError, TimeoutError):
                pass

        self._sdo_timeout = original_timeout
        return active_nodes
