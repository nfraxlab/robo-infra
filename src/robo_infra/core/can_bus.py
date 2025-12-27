"""CAN bus abstraction for Controller Area Network communication.

This module provides CAN bus interfaces for robotics applications,
supporting various hardware interfaces like SocketCAN, PCAN, Kvaser, etc.

CAN (Controller Area Network) is a robust serial communication protocol
commonly used in automotive, industrial automation, and robotics for
real-time control and sensor communication.

Example:
    >>> from robo_infra.core.can_bus import get_can, CANBus, CANMessage
    >>>
    >>> # Get a CAN bus (simulated if no hardware)
    >>> can = get_can(interface="socketcan", channel="can0", bitrate=500000)
    >>> can.open()
    >>>
    >>> # Send a message
    >>> can.send(0x123, bytes([0x01, 0x02, 0x03]))
    >>>
    >>> # Receive a message
    >>> msg = can.recv(timeout=1.0)
    >>> if msg:
    ...     print(f"ID: {hex(msg.arbitration_id)}, Data: {msg.data.hex()}")
    >>>
    >>> can.close()

Hardware Reference:
    SocketCAN (Linux):
        - Native Linux CAN interface
        - Requires: can-utils, python-can
        - Setup: `sudo ip link set can0 up type can bitrate 500000`

    PCAN:
        - Peak Systems CAN adapters (USB, PCI)
        - Windows, Linux, macOS
        - Requires: PCAN drivers, python-can

    Kvaser:
        - Kvaser CAN adapters
        - Requires: Kvaser drivers, python-can

    CANable/Canable:
        - USB-CAN adapter using slcan protocol
        - Platform independent
"""

from __future__ import annotations

import asyncio
import logging
import os
import struct
import time
from abc import abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import TYPE_CHECKING, Any

from robo_infra.core.bus import Bus, BusType
from robo_infra.core.exceptions import CommunicationError, HardwareNotFoundError


if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Iterator

logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class CANInterface(IntEnum):
    """Supported CAN interface types."""

    SOCKETCAN = 0  # Linux SocketCAN
    PCAN = 1  # Peak Systems
    KVASER = 2  # Kvaser
    SLCAN = 3  # Serial Line CAN (CANable, etc.)
    VIRTUAL = 4  # Virtual/simulated
    IXXAT = 5  # IXXAT
    VECTOR = 6  # Vector


class CANBitrate(IntEnum):
    """Standard CAN bitrates."""

    BITRATE_10K = 10_000
    BITRATE_20K = 20_000
    BITRATE_50K = 50_000
    BITRATE_100K = 100_000
    BITRATE_125K = 125_000
    BITRATE_250K = 250_000
    BITRATE_500K = 500_000
    BITRATE_800K = 800_000
    BITRATE_1M = 1_000_000


class CANState(IntEnum):
    """CAN bus state."""

    STOPPED = 0
    STARTED = 1
    ERROR_ACTIVE = 2
    ERROR_WARNING = 3
    ERROR_PASSIVE = 4
    BUS_OFF = 5


class CANErrorFlag(IntFlag):
    """CAN error flags."""

    NONE = 0
    TX_OVERFLOW = 1 << 0
    RX_OVERFLOW = 1 << 1
    BUS_ERROR = 1 << 2
    TX_TIMEOUT = 1 << 3
    RX_TIMEOUT = 1 << 4
    ACK_ERROR = 1 << 5
    FORM_ERROR = 1 << 6
    STUFF_ERROR = 1 << 7
    CRC_ERROR = 1 << 8
    BIT_ERROR = 1 << 9


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class CANMessage:
    """CAN message container.

    Attributes:
        arbitration_id: CAN message ID (11-bit standard or 29-bit extended).
        data: Message data (0-8 bytes for CAN 2.0, 0-64 for CAN FD).
        timestamp: Message timestamp in seconds.
        is_extended_id: True if using 29-bit extended ID.
        is_remote_frame: True if this is a remote transmission request.
        is_error_frame: True if this is an error frame.
        is_fd: True if this is a CAN FD message.
        bitrate_switch: True if CAN FD bitrate switch is used.
        dlc: Data Length Code.
        channel: Channel this message was received on.
    """

    arbitration_id: int
    data: bytes = field(default_factory=bytes)
    timestamp: float = field(default_factory=time.time)
    is_extended_id: bool = False
    is_remote_frame: bool = False
    is_error_frame: bool = False
    is_fd: bool = False
    bitrate_switch: bool = False
    dlc: int | None = None
    channel: str | None = None

    def __post_init__(self) -> None:
        """Validate message."""
        if self.dlc is None:
            self.dlc = len(self.data)

        # Validate ID range
        max_id = 0x1FFFFFFF if self.is_extended_id else 0x7FF
        if not 0 <= self.arbitration_id <= max_id:
            raise ValueError(
                f"Arbitration ID {hex(self.arbitration_id)} out of range for "
                f"{'extended' if self.is_extended_id else 'standard'} ID"
            )

        # Validate data length
        max_len = 64 if self.is_fd else 8
        if len(self.data) > max_len:
            raise ValueError(f"Data length {len(self.data)} exceeds maximum {max_len}")

    def __repr__(self) -> str:
        """String representation."""
        id_str = (
            f"0x{self.arbitration_id:08X}"
            if self.is_extended_id
            else f"0x{self.arbitration_id:03X}"
        )
        data_str = self.data.hex() if self.data else "(empty)"
        return f"CANMessage(id={id_str}, data={data_str}, ts={self.timestamp:.6f})"


@dataclass
class CANFilter:
    """CAN message filter.

    Attributes:
        can_id: CAN ID to filter.
        can_mask: Mask for ID matching (1 bits must match).
        extended: True if filtering extended IDs.
    """

    can_id: int = 0
    can_mask: int = 0
    extended: bool = False


@dataclass
class CANConfig:
    """Configuration for CAN bus.

    Attributes:
        interface: CAN interface type (socketcan, pcan, etc.).
        channel: Channel identifier (can0, PCAN_USBBUS1, etc.).
        bitrate: CAN bitrate in bps.
        data_bitrate: CAN FD data bitrate (if different from nominal).
        fd: Enable CAN FD mode.
        receive_own_messages: Receive messages sent by this interface.
        filters: List of message filters.
        timeout: Default receive timeout in seconds.
    """

    interface: str = "socketcan"
    channel: str = "can0"
    bitrate: int = 500_000
    data_bitrate: int | None = None
    fd: bool = False
    receive_own_messages: bool = False
    filters: list[CANFilter] = field(default_factory=list)
    timeout: float | None = 1.0


@dataclass
class CANStatistics:
    """CAN bus statistics.

    Attributes:
        tx_count: Number of messages transmitted.
        rx_count: Number of messages received.
        tx_error_count: Transmit error counter.
        rx_error_count: Receive error counter.
        tx_dropped: Number of dropped transmit messages.
        rx_dropped: Number of dropped receive messages.
        bus_state: Current bus state.
        error_flags: Active error flags.
    """

    tx_count: int = 0
    rx_count: int = 0
    tx_error_count: int = 0
    rx_error_count: int = 0
    tx_dropped: int = 0
    rx_dropped: int = 0
    bus_state: CANState = CANState.STOPPED
    error_flags: CANErrorFlag = CANErrorFlag.NONE


# =============================================================================
# Abstract CAN Bus
# =============================================================================


class CANBus(Bus):
    """Abstract base class for CAN bus communication.

    CAN (Controller Area Network) is a robust serial communication protocol
    designed for real-time distributed control systems.

    Features:
        - Multi-master arbitration
        - Message-based protocol
        - Error detection and handling
        - Configurable bitrates up to 1 Mbps (8 Mbps for CAN FD)
    """

    def __init__(self, config: CANConfig | None = None, name: str | None = None) -> None:
        """Initialize CAN bus.

        Args:
            config: CAN configuration. Uses defaults if not provided.
            name: Optional human-readable name.
        """
        super().__init__(name)
        self.config = config or CANConfig()
        self._statistics = CANStatistics()

    @property
    def bus_type(self) -> BusType:
        """Type of this bus."""
        return BusType.CAN

    @property
    def statistics(self) -> CANStatistics:
        """Get bus statistics."""
        return self._statistics

    @abstractmethod
    def send(
        self,
        arbitration_id: int,
        data: bytes | list[int],
        is_extended_id: bool = False,
        is_remote_frame: bool = False,
        timeout: float | None = None,
    ) -> None:
        """Send a CAN message.

        Args:
            arbitration_id: CAN message ID.
            data: Message data (0-8 bytes).
            is_extended_id: Use 29-bit extended ID.
            is_remote_frame: Send as remote transmission request.
            timeout: Send timeout in seconds (None for default).

        Raises:
            CommunicationError: If send fails.
        """
        ...

    @abstractmethod
    def recv(self, timeout: float | None = None) -> CANMessage | None:
        """Receive a CAN message.

        Args:
            timeout: Receive timeout in seconds (None for default, 0 for non-blocking).

        Returns:
            Received message, or None if timeout.

        Raises:
            CommunicationError: If receive fails.
        """
        ...

    async def recv_async(self, timeout: float | None = None) -> CANMessage | None:
        """Receive a CAN message asynchronously.

        Args:
            timeout: Receive timeout in seconds.

        Returns:
            Received message, or None if timeout.
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.recv, timeout)

    def send_message(self, message: CANMessage, timeout: float | None = None) -> None:
        """Send a CANMessage object.

        Args:
            message: CAN message to send.
            timeout: Send timeout in seconds.
        """
        self.send(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
            is_remote_frame=message.is_remote_frame,
            timeout=timeout,
        )

    def recv_all(self, timeout: float = 0.0, max_messages: int = 100) -> list[CANMessage]:
        """Receive all pending messages.

        Args:
            timeout: Timeout for each receive attempt.
            max_messages: Maximum messages to receive.

        Returns:
            List of received messages.
        """
        messages = []
        for _ in range(max_messages):
            msg = self.recv(timeout=timeout)
            if msg is None:
                break
            messages.append(msg)
        return messages

    def flush_rx(self) -> int:
        """Flush receive buffer.

        Returns:
            Number of messages flushed.
        """
        count = 0
        while True:
            msg = self.recv(timeout=0.0)
            if msg is None:
                break
            count += 1
        return count

    @abstractmethod
    def set_filters(self, filters: list[CANFilter]) -> None:
        """Set message filters.

        Args:
            filters: List of filters to apply.
        """
        ...

    def clear_filters(self) -> None:
        """Clear all message filters."""
        self.set_filters([])

    @abstractmethod
    def get_state(self) -> CANState:
        """Get current bus state.

        Returns:
            Current CAN bus state.
        """
        ...

    def reset_statistics(self) -> None:
        """Reset bus statistics."""
        self._statistics = CANStatistics()

    def __iter__(self) -> Iterator[CANMessage]:
        """Iterate over received messages."""
        while self.is_open:
            msg = self.recv(timeout=1.0)
            if msg is not None:
                yield msg

    async def stream(self, timeout: float = 1.0) -> AsyncIterator[CANMessage]:
        """Async stream of received messages.

        Args:
            timeout: Receive timeout per message.

        Yields:
            Received CAN messages.
        """
        while self.is_open:
            msg = await self.recv_async(timeout=timeout)
            if msg is not None:
                yield msg


# =============================================================================
# Simulated CAN Bus
# =============================================================================


class SimulatedCANBus(CANBus):
    """Simulated CAN bus for testing without hardware.

    Provides a virtual CAN bus that stores sent messages and can
    have messages injected for receive testing.
    """

    def __init__(self, config: CANConfig | None = None, name: str | None = None) -> None:
        """Initialize simulated CAN bus.

        Args:
            config: CAN configuration.
            name: Optional name.
        """
        super().__init__(config, name or "SimulatedCAN")
        self._tx_buffer: list[CANMessage] = []
        self._rx_buffer: list[CANMessage] = []
        self._state = CANState.STOPPED
        self._filters: list[CANFilter] = []
        self._loopback = False  # If True, sent messages appear in rx

    def open(self) -> None:
        """Open the simulated CAN bus."""
        if self._is_open:
            return

        self._is_open = True
        self._state = CANState.STARTED
        self._statistics.bus_state = CANState.STARTED
        logger.info("%s opened (simulated, channel=%s)", self.name, self.config.channel)

    def close(self) -> None:
        """Close the simulated CAN bus."""
        if not self._is_open:
            return

        self._is_open = False
        self._state = CANState.STOPPED
        self._statistics.bus_state = CANState.STOPPED
        self._tx_buffer.clear()
        self._rx_buffer.clear()
        logger.info("%s closed", self.name)

    def send(
        self,
        arbitration_id: int,
        data: bytes | list[int],
        is_extended_id: bool = False,
        is_remote_frame: bool = False,
        timeout: float | None = None,
    ) -> None:
        """Send a CAN message (simulated)."""
        if not self._is_open:
            raise CommunicationError("CAN bus not open")

        if isinstance(data, list):
            data = bytes(data)

        msg = CANMessage(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id,
            is_remote_frame=is_remote_frame,
            channel=self.config.channel,
        )

        self._tx_buffer.append(msg)
        self._statistics.tx_count += 1

        # Loopback mode: sent messages appear in receive buffer
        if self._loopback or self.config.receive_own_messages:
            self._rx_buffer.append(msg)

        logger.debug("Simulated CAN TX: %s", msg)

    def recv(self, timeout: float | None = None) -> CANMessage | None:
        """Receive a CAN message (simulated)."""
        if not self._is_open:
            raise CommunicationError("CAN bus not open")

        if not self._rx_buffer:
            # Simulate timeout
            if timeout and timeout > 0:
                time.sleep(min(timeout, 0.01))  # Don't actually wait long
            return None

        msg = self._rx_buffer.pop(0)
        self._statistics.rx_count += 1
        logger.debug("Simulated CAN RX: %s", msg)
        return msg

    def set_filters(self, filters: list[CANFilter]) -> None:
        """Set message filters (simulated)."""
        self._filters = filters.copy()
        self.config.filters = filters.copy()
        logger.debug("Set %d CAN filters", len(filters))

    def get_state(self) -> CANState:
        """Get current bus state."""
        return self._state

    def inject_message(self, message: CANMessage) -> None:
        """Inject a message into the receive buffer (for testing).

        Args:
            message: Message to inject.
        """
        self._rx_buffer.append(message)

    def inject_messages(self, messages: list[CANMessage]) -> None:
        """Inject multiple messages into the receive buffer.

        Args:
            messages: Messages to inject.
        """
        self._rx_buffer.extend(messages)

    def get_sent_messages(self) -> list[CANMessage]:
        """Get all sent messages (for testing).

        Returns:
            List of sent messages.
        """
        return self._tx_buffer.copy()

    def clear_buffers(self) -> None:
        """Clear all message buffers."""
        self._tx_buffer.clear()
        self._rx_buffer.clear()

    def enable_loopback(self, enable: bool = True) -> None:
        """Enable/disable loopback mode.

        Args:
            enable: If True, sent messages appear in receive buffer.
        """
        self._loopback = enable


# =============================================================================
# Hardware CAN Bus (python-can)
# =============================================================================


class PythonCANBus(CANBus):
    """CAN bus using python-can library.

    Supports multiple hardware interfaces through the python-can library:
    - SocketCAN (Linux)
    - PCAN (Peak Systems)
    - Kvaser
    - IXXAT
    - Vector
    - Serial/SLCAN

    Requires:
        pip install python-can
    """

    def __init__(self, config: CANConfig | None = None, name: str | None = None) -> None:
        """Initialize python-can CAN bus.

        Args:
            config: CAN configuration.
            name: Optional name.
        """
        super().__init__(config, name or f"CAN-{config.channel if config else 'can0'}")
        self._bus: Any = None

    def open(self) -> None:
        """Open the CAN bus."""
        if self._is_open:
            return

        try:
            import can
        except ImportError as e:
            raise HardwareNotFoundError(
                "python-can library not installed. Install with: pip install robo-infra[can]"
            ) from e

        try:
            # Build bus configuration
            kwargs: dict[str, Any] = {
                "interface": self.config.interface,
                "channel": self.config.channel,
                "bitrate": self.config.bitrate,
                "receive_own_messages": self.config.receive_own_messages,
            }

            # Add FD configuration if enabled
            if self.config.fd:
                kwargs["fd"] = True
                if self.config.data_bitrate:
                    kwargs["data_bitrate"] = self.config.data_bitrate

            self._bus = can.Bus(**kwargs)
            self._is_open = True
            self._statistics.bus_state = CANState.STARTED

            # Apply filters if configured
            if self.config.filters:
                self._apply_filters()

            logger.info(
                "%s opened (%s, channel=%s, bitrate=%d)",
                self.name,
                self.config.interface,
                self.config.channel,
                self.config.bitrate,
            )

        except Exception as e:
            raise CommunicationError(f"Failed to open CAN bus: {e}") from e

    def close(self) -> None:
        """Close the CAN bus."""
        if not self._is_open:
            return

        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception as e:
                logger.warning("Error closing CAN bus: %s", e)
            self._bus = None

        self._is_open = False
        self._statistics.bus_state = CANState.STOPPED
        logger.info("%s closed", self.name)

    def send(
        self,
        arbitration_id: int,
        data: bytes | list[int],
        is_extended_id: bool = False,
        is_remote_frame: bool = False,
        timeout: float | None = None,
    ) -> None:
        """Send a CAN message."""
        if not self._is_open or self._bus is None:
            raise CommunicationError("CAN bus not open")

        try:
            import can

            if isinstance(data, list):
                data = bytes(data)

            msg = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=is_extended_id,
                is_remote_frame=is_remote_frame,
            )

            self._bus.send(msg, timeout=timeout)
            self._statistics.tx_count += 1

        except Exception as e:
            self._statistics.tx_error_count += 1
            raise CommunicationError(f"CAN send failed: {e}") from e

    def recv(self, timeout: float | None = None) -> CANMessage | None:
        """Receive a CAN message."""
        if not self._is_open or self._bus is None:
            raise CommunicationError("CAN bus not open")

        if timeout is None:
            timeout = self.config.timeout

        try:
            msg = self._bus.recv(timeout=timeout)
            if msg is None:
                return None

            self._statistics.rx_count += 1

            return CANMessage(
                arbitration_id=msg.arbitration_id,
                data=bytes(msg.data),
                timestamp=msg.timestamp or time.time(),
                is_extended_id=msg.is_extended_id,
                is_remote_frame=msg.is_remote_frame,
                is_error_frame=msg.is_error_frame,
                is_fd=getattr(msg, "is_fd", False),
                bitrate_switch=getattr(msg, "bitrate_switch", False),
                dlc=msg.dlc,
                channel=str(msg.channel) if msg.channel else self.config.channel,
            )

        except Exception as e:
            self._statistics.rx_error_count += 1
            raise CommunicationError(f"CAN recv failed: {e}") from e

    def set_filters(self, filters: list[CANFilter]) -> None:
        """Set message filters."""
        self.config.filters = filters.copy()
        if self._is_open:
            self._apply_filters()

    def _apply_filters(self) -> None:
        """Apply configured filters to the bus."""
        if not self._bus:
            return

        try:
            can_filters = [
                {"can_id": f.can_id, "can_mask": f.can_mask, "extended": f.extended}
                for f in self.config.filters
            ]
            self._bus.set_filters(can_filters)
        except Exception as e:
            logger.warning("Failed to apply CAN filters: %s", e)

    def get_state(self) -> CANState:
        """Get current bus state."""
        if not self._is_open:
            return CANState.STOPPED

        try:
            state = self._bus.state
            # Map python-can BusState to our CANState
            state_map = {
                0: CANState.STARTED,  # ACTIVE
                1: CANState.ERROR_WARNING,
                2: CANState.ERROR_PASSIVE,
                3: CANState.BUS_OFF,
            }
            return state_map.get(state, CANState.STARTED)
        except Exception:
            return CANState.STARTED


# =============================================================================
# Factory Functions
# =============================================================================


def get_can(
    interface: str = "socketcan",
    channel: str = "can0",
    bitrate: int = 500_000,
    simulation: bool | None = None,
    **kwargs: Any,
) -> CANBus:
    """Get a CAN bus instance.

    Factory function that returns either a real hardware CAN bus
    or a simulated one based on environment and parameters.

    Args:
        interface: CAN interface type (socketcan, pcan, kvaser, slcan, virtual).
        channel: Channel identifier.
        bitrate: CAN bitrate in bps.
        simulation: Force simulation mode. If None, auto-detect.
        **kwargs: Additional configuration options.

    Returns:
        CANBus instance (real or simulated).

    Example:
        >>> can = get_can("socketcan", "can0", 500000)
        >>> can.open()
        >>> can.send(0x123, b"\\x01\\x02\\x03")
    """
    # Check for simulation mode
    if simulation is None:
        simulation = os.environ.get("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

    config = CANConfig(
        interface=interface,
        channel=channel,
        bitrate=bitrate,
        **kwargs,
    )

    if simulation or interface == "virtual":
        logger.info("Using simulated CAN bus")
        return SimulatedCANBus(config)

    # Try to use real hardware
    try:
        import can  # noqa: F401

        return PythonCANBus(config)
    except ImportError:
        logger.warning("python-can not available, using simulation")
        return SimulatedCANBus(config)


# =============================================================================
# CAN Utilities
# =============================================================================


def decode_can_id(arbitration_id: int, is_extended: bool = False) -> dict[str, int]:
    """Decode a CAN arbitration ID into components.

    For standard CAN:
        - Bits 0-10: Message ID

    For extended CAN (CAN 2.0B):
        - Bits 0-17: Extended ID
        - Bits 18-28: Base ID

    Args:
        arbitration_id: The arbitration ID to decode.
        is_extended: Whether this is an extended ID.

    Returns:
        Dictionary with ID components.
    """
    if is_extended:
        return {
            "base_id": (arbitration_id >> 18) & 0x7FF,
            "extended_id": arbitration_id & 0x3FFFF,
            "full_id": arbitration_id,
        }
    return {"id": arbitration_id & 0x7FF}


def pack_can_data(values: list[tuple[str, Any]], byte_order: str = "little") -> bytes:
    """Pack multiple values into CAN data bytes.

    Args:
        values: List of (format, value) tuples using struct format chars.
        byte_order: 'little' or 'big'.

    Returns:
        Packed bytes.

    Example:
        >>> data = pack_can_data([('H', 1000), ('B', 5), ('B', 0)])
        >>> # Packs: uint16 1000, uint8 5, uint8 0
    """
    order_char = "<" if byte_order == "little" else ">"
    fmt = order_char + "".join(f for f, _ in values)
    vals = [v for _, v in values]
    return struct.pack(fmt, *vals)


def unpack_can_data(data: bytes, format_str: str, byte_order: str = "little") -> tuple:
    """Unpack CAN data bytes into values.

    Args:
        data: Data bytes to unpack.
        format_str: Struct format string (without byte order).
        byte_order: 'little' or 'big'.

    Returns:
        Tuple of unpacked values.

    Example:
        >>> values = unpack_can_data(data, "HBB")
        >>> rpm, status, error = values
    """
    order_char = "<" if byte_order == "little" else ">"
    return struct.unpack(order_char + format_str, data)
