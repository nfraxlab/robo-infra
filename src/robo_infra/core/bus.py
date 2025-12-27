"""Bus abstractions for communication protocols (I2C, SPI, UART).

This module provides abstract base classes and simulated implementations
for common hardware communication buses used in robotics.

Example:
    >>> from robo_infra.core.bus import get_i2c, get_spi, get_serial
    >>>
    >>> # Get an I2C bus (simulated if no hardware)
    >>> i2c = get_i2c(1)
    >>> devices = i2c.scan()
    >>> print(f"Found devices at: {[hex(d) for d in devices]}")
    >>>
    >>> # Read from a device register
    >>> data = i2c.read_register(0x40, 0x00, 2)
    >>>
    >>> # Get SPI bus
    >>> spi = get_spi(0, 0)
    >>> response = spi.transfer(bytes([0x01, 0x02]))
    >>>
    >>> # Get serial port
    >>> serial = get_serial("/dev/ttyUSB0", 115200)
    >>> serial.write(b"AT\\r\\n")
    >>> response = serial.readline()
"""

from __future__ import annotations

import logging
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

from robo_infra.core.exceptions import HardwareNotFoundError


if TYPE_CHECKING:
    from collections.abc import Sequence

logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class BusType(Enum):
    """Types of communication buses."""

    I2C = "i2c"
    SPI = "spi"
    UART = "uart"
    ONEWIRE = "onewire"
    CAN = "can"


class SPIMode(Enum):
    """SPI clock polarity and phase modes.

    Mode 0: CPOL=0, CPHA=0 - Clock idle low, sample on rising edge
    Mode 1: CPOL=0, CPHA=1 - Clock idle low, sample on falling edge
    Mode 2: CPOL=1, CPHA=0 - Clock idle high, sample on falling edge
    Mode 3: CPOL=1, CPHA=1 - Clock idle high, sample on rising edge
    """

    MODE_0 = 0
    MODE_1 = 1
    MODE_2 = 2
    MODE_3 = 3


class Parity(Enum):
    """Serial parity options."""

    NONE = "N"
    EVEN = "E"
    ODD = "O"
    MARK = "M"
    SPACE = "S"


class StopBits(Enum):
    """Serial stop bits options."""

    ONE = 1
    ONE_POINT_FIVE = 1.5
    TWO = 2


class ByteOrder(Enum):
    """Byte order for multi-byte operations."""

    BIG_ENDIAN = "big"
    LITTLE_ENDIAN = "little"


# =============================================================================
# Configurations
# =============================================================================


@dataclass
class I2CConfig:
    """Configuration for I2C bus.

    Attributes:
        bus_number: I2C bus number (e.g., 1 for /dev/i2c-1).
        frequency: Clock frequency in Hz (standard: 100kHz, fast: 400kHz).
        timeout: Operation timeout in seconds.
    """

    bus_number: int = 1
    frequency: int = 100_000
    timeout: float = 1.0


@dataclass
class SPIConfig:
    """Configuration for SPI bus.

    Attributes:
        bus: SPI bus number.
        device: SPI device/chip-select number.
        max_speed_hz: Maximum clock speed in Hz.
        mode: SPI mode (0-3).
        bits_per_word: Bits per word (usually 8).
        lsb_first: If True, transmit LSB first.
    """

    bus: int = 0
    device: int = 0
    max_speed_hz: int = 1_000_000
    mode: SPIMode = SPIMode.MODE_0
    bits_per_word: int = 8
    lsb_first: bool = False


@dataclass
class SerialConfig:
    """Configuration for serial/UART bus.

    Attributes:
        port: Serial port path (e.g., /dev/ttyUSB0, COM3).
        baudrate: Baud rate (e.g., 9600, 115200).
        bytesize: Number of data bits (5-8).
        parity: Parity checking.
        stopbits: Number of stop bits.
        timeout: Read timeout in seconds (None for blocking).
        write_timeout: Write timeout in seconds.
        xonxoff: Enable software flow control.
        rtscts: Enable hardware (RTS/CTS) flow control.
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 9600
    bytesize: int = 8
    parity: Parity = Parity.NONE
    stopbits: StopBits = StopBits.ONE
    timeout: float | None = 1.0
    write_timeout: float | None = 1.0
    xonxoff: bool = False
    rtscts: bool = False


# =============================================================================
# Abstract Base Classes
# =============================================================================


class Bus(ABC):
    """Abstract base class for all communication buses.

    Provides a common interface for bus lifecycle management and
    ensures consistent behavior across different bus types.
    """

    def __init__(self, name: str | None = None) -> None:
        """Initialize the bus.

        Args:
            name: Optional human-readable name for the bus.
        """
        self._name = name or self.__class__.__name__
        self._is_open = False

    @property
    def name(self) -> str:
        """Human-readable name of the bus."""
        return self._name

    @property
    def is_open(self) -> bool:
        """Whether the bus is currently open."""
        return self._is_open

    @property
    @abstractmethod
    def bus_type(self) -> BusType:
        """Type of this bus."""
        ...

    @abstractmethod
    def open(self) -> None:
        """Open the bus for communication.

        Raises:
            CommunicationError: If the bus cannot be opened.
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """Close the bus and release resources."""
        ...

    def __enter__(self) -> Bus:
        """Context manager entry."""
        self.open()
        return self

    def __exit__(self, exc_type: type | None, exc_val: Exception | None, exc_tb: object) -> None:
        """Context manager exit."""
        self.close()


class I2CBus(Bus):
    """Abstract base class for I2C bus communication.

    I2C (Inter-Integrated Circuit) is a multi-master, multi-slave,
    synchronous, packet-switched, single-ended serial communication bus.
    """

    def __init__(self, config: I2CConfig | None = None, name: str | None = None) -> None:
        """Initialize I2C bus.

        Args:
            config: I2C configuration. Uses defaults if not provided.
            name: Optional human-readable name.
        """
        super().__init__(name)
        self.config = config or I2CConfig()

    @property
    def bus_type(self) -> BusType:
        """Type of this bus."""
        return BusType.I2C

    @abstractmethod
    def scan(self) -> list[int]:
        """Scan the bus for connected devices.

        Returns:
            List of detected device addresses (7-bit).

        Raises:
            CommunicationError: If scan fails.
        """
        ...

    @abstractmethod
    def write(self, address: int, data: bytes | Sequence[int]) -> int:
        """Write data to a device.

        Args:
            address: 7-bit device address.
            data: Data bytes to write.

        Returns:
            Number of bytes written.

        Raises:
            CommunicationError: If write fails.
        """
        ...

    @abstractmethod
    def read(self, address: int, length: int) -> bytes:
        """Read data from a device.

        Args:
            address: 7-bit device address.
            length: Number of bytes to read.

        Returns:
            Bytes read from device.

        Raises:
            CommunicationError: If read fails.
        """
        ...

    def write_byte(self, address: int, value: int) -> None:
        """Write a single byte to a device.

        Args:
            address: 7-bit device address.
            value: Byte value to write (0-255).
        """
        self.write(address, bytes([value]))

    def read_byte(self, address: int) -> int:
        """Read a single byte from a device.

        Args:
            address: 7-bit device address.

        Returns:
            Byte value read (0-255).
        """
        return self.read(address, 1)[0]

    @abstractmethod
    def write_register(self, address: int, register: int, data: bytes | Sequence[int]) -> int:
        """Write data to a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            data: Data bytes to write.

        Returns:
            Number of bytes written.

        Raises:
            CommunicationError: If write fails.
        """
        ...

    @abstractmethod
    def read_register(self, address: int, register: int, length: int) -> bytes:
        """Read data from a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            length: Number of bytes to read.

        Returns:
            Bytes read from register.

        Raises:
            CommunicationError: If read fails.
        """
        ...

    def write_register_byte(self, address: int, register: int, value: int) -> None:
        """Write a single byte to a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            value: Byte value to write (0-255).
        """
        self.write_register(address, register, bytes([value]))

    def read_register_byte(self, address: int, register: int) -> int:
        """Read a single byte from a device register.

        Args:
            address: 7-bit device address.
            register: Register address.

        Returns:
            Byte value read (0-255).
        """
        return self.read_register(address, register, 1)[0]

    def read_register_word(
        self, address: int, register: int, byte_order: ByteOrder = ByteOrder.BIG_ENDIAN
    ) -> int:
        """Read a 16-bit word from a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            byte_order: Byte order (big or little endian).

        Returns:
            16-bit word value.
        """
        data = self.read_register(address, register, 2)
        return int.from_bytes(data, byteorder=byte_order.value)

    def write_register_word(
        self,
        address: int,
        register: int,
        value: int,
        byte_order: ByteOrder = ByteOrder.BIG_ENDIAN,
    ) -> None:
        """Write a 16-bit word to a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            value: 16-bit word value.
            byte_order: Byte order (big or little endian).
        """
        data = value.to_bytes(2, byteorder=byte_order.value)
        self.write_register(address, register, data)


class SPIBus(Bus):
    """Abstract base class for SPI bus communication.

    SPI (Serial Peripheral Interface) is a synchronous serial communication
    interface for short-distance communication in embedded systems.
    """

    def __init__(self, config: SPIConfig | None = None, name: str | None = None) -> None:
        """Initialize SPI bus.

        Args:
            config: SPI configuration. Uses defaults if not provided.
            name: Optional human-readable name.
        """
        super().__init__(name)
        self.config = config or SPIConfig()

    @property
    def bus_type(self) -> BusType:
        """Type of this bus."""
        return BusType.SPI

    @abstractmethod
    def transfer(self, data: bytes | Sequence[int]) -> bytes:
        """Perform a simultaneous read/write transfer.

        SPI is full-duplex: data is sent and received simultaneously.
        The returned bytes correspond to what was received while
        sending the input data.

        Args:
            data: Data bytes to send.

        Returns:
            Bytes received during transfer.

        Raises:
            CommunicationError: If transfer fails.
        """
        ...

    def write(self, data: bytes | Sequence[int]) -> None:
        """Write data (ignoring received data).

        Args:
            data: Data bytes to send.
        """
        self.transfer(data)

    def read(self, length: int, fill: int = 0x00) -> bytes:
        """Read data by sending fill bytes.

        Args:
            length: Number of bytes to read.
            fill: Byte value to send while reading (default 0x00).

        Returns:
            Bytes received.
        """
        return self.transfer(bytes([fill] * length))

    @abstractmethod
    def set_speed(self, speed_hz: int) -> None:
        """Set the clock speed.

        Args:
            speed_hz: Clock speed in Hz.
        """
        ...

    @abstractmethod
    def set_mode(self, mode: SPIMode) -> None:
        """Set the SPI mode.

        Args:
            mode: SPI mode (0-3).
        """
        ...


class SerialBus(Bus):
    """Abstract base class for serial/UART bus communication.

    UART (Universal Asynchronous Receiver-Transmitter) is a hardware
    communication protocol for asynchronous serial communication.
    """

    def __init__(self, config: SerialConfig | None = None, name: str | None = None) -> None:
        """Initialize serial bus.

        Args:
            config: Serial configuration. Uses defaults if not provided.
            name: Optional human-readable name.
        """
        super().__init__(name)
        self.config = config or SerialConfig()

    @property
    def bus_type(self) -> BusType:
        """Type of this bus."""
        return BusType.UART

    @abstractmethod
    def write(self, data: bytes | str) -> int:
        """Write data to the serial port.

        Args:
            data: Data to write (bytes or string).

        Returns:
            Number of bytes written.

        Raises:
            CommunicationError: If write fails.
        """
        ...

    @abstractmethod
    def read(self, length: int) -> bytes:
        """Read exact number of bytes.

        Args:
            length: Number of bytes to read.

        Returns:
            Bytes read (may be fewer if timeout).

        Raises:
            CommunicationError: If read fails.
        """
        ...

    @abstractmethod
    def readline(self) -> bytes:
        """Read a line (until newline or timeout).

        Returns:
            Line read including the newline character.
        """
        ...

    @abstractmethod
    def read_until(self, terminator: bytes = b"\n") -> bytes:
        """Read until a terminator is found.

        Args:
            terminator: Byte sequence to stop at.

        Returns:
            Data read including the terminator.
        """
        ...

    @property
    @abstractmethod
    def in_waiting(self) -> int:
        """Number of bytes waiting to be read."""
        ...

    @property
    @abstractmethod
    def out_waiting(self) -> int:
        """Number of bytes waiting to be written."""
        ...

    @abstractmethod
    def flush(self) -> None:
        """Wait until all data is written."""
        ...

    @abstractmethod
    def reset_input_buffer(self) -> None:
        """Clear input buffer."""
        ...

    @abstractmethod
    def reset_output_buffer(self) -> None:
        """Clear output buffer."""
        ...

    @abstractmethod
    def set_baudrate(self, baudrate: int) -> None:
        """Change the baud rate.

        Args:
            baudrate: New baud rate.
        """
        ...


# =============================================================================
# Real Hardware Implementations
# =============================================================================


class SMBus2I2CBus(I2CBus):
    """Real I2C bus using smbus2 library.

    This implementation uses the smbus2 library to communicate with
    real I2C hardware on Linux systems (Raspberry Pi, etc.).

    Requires: pip install smbus2

    Example:
        >>> bus = SMBus2I2CBus(I2CConfig(bus_number=1))
        >>> bus.open()
        >>> devices = bus.scan()
        >>> print(f"Found devices: {[hex(d) for d in devices]}")
    """

    def __init__(self, config: I2CConfig | None = None, name: str | None = None) -> None:
        """Initialize real I2C bus.

        Args:
            config: I2C configuration.
            name: Optional human-readable name.
        """
        super().__init__(config, name or f"I2C-{config.bus_number if config else 1}")
        self._bus: object | None = None  # SMBus instance

    def open(self) -> None:
        """Open the I2C bus."""
        try:
            from smbus2 import SMBus
        except ImportError as e:
            raise ImportError(
                "smbus2 library required for real I2C. Install with: pip install smbus2"
            ) from e

        self._bus = SMBus(self.config.bus_number)
        self._is_open = True
        logger.info("Opened real I2C bus %d", self.config.bus_number)

    def close(self) -> None:
        """Close the I2C bus."""
        if self._bus is not None:
            self._bus.close()  # type: ignore[union-attr]
            self._bus = None
        self._is_open = False
        logger.debug("Closed real I2C bus %d", self.config.bus_number)

    def _ensure_open(self) -> None:
        """Ensure bus is open."""
        if not self._is_open or self._bus is None:
            raise RuntimeError("I2C bus is not open")

    def scan(self) -> list[int]:
        """Scan the bus for connected devices.

        Probes addresses 0x03-0x77 (valid 7-bit I2C addresses).

        Returns:
            List of detected device addresses.
        """
        self._ensure_open()
        devices = []

        for address in range(0x03, 0x78):
            try:
                self._bus.read_byte(address)  # type: ignore[union-attr]
                devices.append(address)
            except OSError:
                # No device at this address
                pass

        logger.debug("I2C scan found %d devices: %s", len(devices), [hex(a) for a in devices])
        return devices

    def write(self, address: int, data: bytes | Sequence[int]) -> int:
        """Write data to a device.

        Args:
            address: 7-bit device address.
            data: Data bytes to write.

        Returns:
            Number of bytes written.
        """
        self._ensure_open()
        data_bytes = bytes(data)
        self._bus.write_i2c_block_data(address, data_bytes[0], list(data_bytes[1:]))  # type: ignore[union-attr]
        logger.debug("I2C write to %s: %s", hex(address), data_bytes.hex())
        return len(data_bytes)

    def read(self, address: int, length: int) -> bytes:
        """Read data from a device.

        Args:
            address: 7-bit device address.
            length: Number of bytes to read.

        Returns:
            Bytes read from device.
        """
        self._ensure_open()
        result = bytes(self._bus.read_i2c_block_data(address, 0, length))  # type: ignore[union-attr]
        logger.debug("I2C read from %s (%d bytes): %s", hex(address), length, result.hex())
        return result

    def write_register(self, address: int, register: int, data: bytes | Sequence[int]) -> int:
        """Write data to a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            data: Data bytes to write.

        Returns:
            Number of bytes written.
        """
        self._ensure_open()
        data_bytes = bytes(data)
        self._bus.write_i2c_block_data(address, register, list(data_bytes))  # type: ignore[union-attr]
        logger.debug(
            "I2C write register %s[%s] = %s", hex(address), hex(register), data_bytes.hex()
        )
        return len(data_bytes)

    def read_register(self, address: int, register: int, length: int) -> bytes:
        """Read data from a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            length: Number of bytes to read.

        Returns:
            Bytes read from register.
        """
        self._ensure_open()
        result = bytes(self._bus.read_i2c_block_data(address, register, length))  # type: ignore[union-attr]
        logger.debug(
            "I2C read register %s[%s] (%d bytes): %s",
            hex(address),
            hex(register),
            length,
            result.hex(),
        )
        return result

    def write_byte(self, address: int, value: int) -> None:
        """Write a single byte to a device.

        Args:
            address: 7-bit device address.
            value: Byte value to write (0-255).
        """
        self._ensure_open()
        self._bus.write_byte(address, value)  # type: ignore[union-attr]

    def read_byte(self, address: int) -> int:
        """Read a single byte from a device.

        Args:
            address: 7-bit device address.

        Returns:
            Byte value read (0-255).
        """
        self._ensure_open()
        return self._bus.read_byte(address)  # type: ignore[union-attr]

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        """Write a byte to a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            value: Byte value to write (0-255).
        """
        self._ensure_open()
        self._bus.write_byte_data(address, register, value)  # type: ignore[union-attr]

    def read_byte_data(self, address: int, register: int) -> int:
        """Read a byte from a device register.

        Args:
            address: 7-bit device address.
            register: Register address.

        Returns:
            Byte value read (0-255).
        """
        self._ensure_open()
        return self._bus.read_byte_data(address, register)  # type: ignore[union-attr]

    def write_word_data(self, address: int, register: int, value: int) -> None:
        """Write a 16-bit word to a device register.

        Args:
            address: 7-bit device address.
            register: Register address.
            value: 16-bit word value.
        """
        self._ensure_open()
        self._bus.write_word_data(address, register, value)  # type: ignore[union-attr]

    def read_word_data(self, address: int, register: int) -> int:
        """Read a 16-bit word from a device register.

        Args:
            address: 7-bit device address.
            register: Register address.

        Returns:
            16-bit word value.
        """
        self._ensure_open()
        return self._bus.read_word_data(address, register)  # type: ignore[union-attr]


class SpiDevSPIBus(SPIBus):
    """Real SPI bus using spidev library.

    This implementation uses the spidev library to communicate with
    real SPI hardware on Linux systems.

    Requires: pip install spidev

    Example:
        >>> bus = SpiDevSPIBus(SPIConfig(bus=0, device=0))
        >>> bus.open()
        >>> response = bus.transfer(bytes([0x01, 0x02, 0x03]))
    """

    def __init__(self, config: SPIConfig | None = None, name: str | None = None) -> None:
        """Initialize real SPI bus.

        Args:
            config: SPI configuration.
            name: Optional human-readable name.
        """
        cfg = config or SPIConfig()
        super().__init__(cfg, name or f"SPI-{cfg.bus}:{cfg.device}")
        self._spi: object | None = None  # SpiDev instance

    def open(self) -> None:
        """Open the SPI bus."""
        try:
            from spidev import SpiDev
        except ImportError as e:
            raise ImportError(
                "spidev library required for real SPI. Install with: pip install spidev"
            ) from e

        self._spi = SpiDev()
        self._spi.open(self.config.bus, self.config.device)  # type: ignore[union-attr]
        self._spi.max_speed_hz = self.config.max_speed_hz  # type: ignore[union-attr]
        self._spi.mode = self.config.mode.value  # type: ignore[union-attr]
        self._spi.bits_per_word = self.config.bits_per_word  # type: ignore[union-attr]
        self._spi.lsbfirst = self.config.lsb_first  # type: ignore[union-attr]
        self._is_open = True
        logger.info("Opened real SPI bus %d:%d", self.config.bus, self.config.device)

    def close(self) -> None:
        """Close the SPI bus."""
        if self._spi is not None:
            self._spi.close()  # type: ignore[union-attr]
            self._spi = None
        self._is_open = False
        logger.debug("Closed real SPI bus %d:%d", self.config.bus, self.config.device)

    def _ensure_open(self) -> None:
        """Ensure bus is open."""
        if not self._is_open or self._spi is None:
            raise RuntimeError("SPI bus is not open")

    def transfer(self, data: bytes | Sequence[int]) -> bytes:
        """Perform a simultaneous read/write transfer.

        Args:
            data: Data bytes to send.

        Returns:
            Bytes received during transfer.
        """
        self._ensure_open()
        result = bytes(self._spi.xfer2(list(data)))  # type: ignore[union-attr]
        logger.debug("SPI transfer: sent=%s, received=%s", bytes(data).hex(), result.hex())
        return result

    def set_speed(self, speed_hz: int) -> None:
        """Set the clock speed.

        Args:
            speed_hz: Clock speed in Hz.
        """
        self._ensure_open()
        self._spi.max_speed_hz = speed_hz  # type: ignore[union-attr]
        self.config.max_speed_hz = speed_hz
        logger.debug("SPI speed set to %d Hz", speed_hz)

    def set_mode(self, mode: SPIMode) -> None:
        """Set the SPI mode.

        Args:
            mode: SPI mode (0-3).
        """
        self._ensure_open()
        self._spi.mode = mode.value  # type: ignore[union-attr]
        self.config.mode = mode
        logger.debug("SPI mode set to %s", mode)


class PySerialBus(SerialBus):
    """Real serial bus using pyserial library.

    This implementation uses the pyserial library to communicate with
    real serial ports (USB-to-serial, hardware UART, etc.).

    Requires: pip install pyserial

    Example:
        >>> bus = PySerialBus(SerialConfig(port="/dev/ttyUSB0", baudrate=115200))
        >>> bus.open()
        >>> bus.write(b"AT\\r\\n")
        >>> response = bus.readline()
    """

    def __init__(self, config: SerialConfig | None = None, name: str | None = None) -> None:
        """Initialize real serial bus.

        Args:
            config: Serial configuration.
            name: Optional human-readable name.
        """
        cfg = config or SerialConfig()
        super().__init__(cfg, name or f"Serial-{cfg.port}")
        self._serial: object | None = None  # Serial instance

    def open(self) -> None:
        """Open the serial port."""
        try:
            from serial import Serial
        except ImportError as e:
            raise ImportError(
                "pyserial library required for real serial. Install with: pip install pyserial"
            ) from e

        self._serial = Serial(
            port=self.config.port,
            baudrate=self.config.baudrate,
            bytesize=self.config.bytesize,
            parity=self.config.parity.value,
            stopbits=self.config.stopbits.value,
            timeout=self.config.timeout,
            write_timeout=self.config.write_timeout,
            xonxoff=self.config.xonxoff,
            rtscts=self.config.rtscts,
        )
        self._is_open = True
        logger.info("Opened real serial port %s at %d baud", self.config.port, self.config.baudrate)

    def close(self) -> None:
        """Close the serial port."""
        if self._serial is not None:
            self._serial.close()  # type: ignore[union-attr]
            self._serial = None
        self._is_open = False
        logger.debug("Closed real serial port %s", self.config.port)

    def _ensure_open(self) -> None:
        """Ensure port is open."""
        if not self._is_open or self._serial is None:
            raise RuntimeError("Serial port is not open")

    def write(self, data: bytes | str) -> int:
        """Write data to the serial port.

        Args:
            data: Data to write (bytes or string).

        Returns:
            Number of bytes written.
        """
        self._ensure_open()
        if isinstance(data, str):
            data = data.encode()
        result = self._serial.write(data)  # type: ignore[union-attr]
        logger.debug("Serial write: %s", data.hex())
        return result  # type: ignore[return-value]

    def read(self, length: int) -> bytes:
        """Read exact number of bytes.

        Args:
            length: Number of bytes to read.

        Returns:
            Bytes read (may be fewer if timeout).
        """
        self._ensure_open()
        result = self._serial.read(length)  # type: ignore[union-attr]
        logger.debug("Serial read (%d bytes): %s", length, result.hex())
        return result  # type: ignore[return-value]

    def readline(self) -> bytes:
        """Read a line (until newline or timeout).

        Returns:
            Line read including the newline character.
        """
        self._ensure_open()
        result = self._serial.readline()  # type: ignore[union-attr]
        logger.debug("Serial readline: %s", result.hex())
        return result  # type: ignore[return-value]

    def read_until(self, terminator: bytes = b"\n") -> bytes:
        """Read until a terminator is found.

        Args:
            terminator: Byte sequence to stop at.

        Returns:
            Data read including the terminator.
        """
        self._ensure_open()
        result = self._serial.read_until(terminator)  # type: ignore[union-attr]
        logger.debug("Serial read_until(%s): %s", terminator.hex(), result.hex())
        return result  # type: ignore[return-value]

    @property
    def in_waiting(self) -> int:
        """Number of bytes waiting to be read."""
        self._ensure_open()
        return self._serial.in_waiting  # type: ignore[union-attr, return-value]

    @property
    def out_waiting(self) -> int:
        """Number of bytes waiting to be written."""
        self._ensure_open()
        return self._serial.out_waiting  # type: ignore[union-attr, return-value]

    def flush(self) -> None:
        """Wait until all data is written."""
        self._ensure_open()
        self._serial.flush()  # type: ignore[union-attr]
        logger.debug("Serial flush")

    def reset_input_buffer(self) -> None:
        """Clear input buffer."""
        self._ensure_open()
        self._serial.reset_input_buffer()  # type: ignore[union-attr]
        logger.debug("Serial reset input buffer")

    def reset_output_buffer(self) -> None:
        """Clear output buffer."""
        self._ensure_open()
        self._serial.reset_output_buffer()  # type: ignore[union-attr]
        logger.debug("Serial reset output buffer")

    def set_baudrate(self, baudrate: int) -> None:
        """Change the baud rate.

        Args:
            baudrate: New baud rate.
        """
        self._ensure_open()
        self._serial.baudrate = baudrate  # type: ignore[union-attr]
        self.config.baudrate = baudrate
        logger.debug("Serial baudrate set to %d", baudrate)


# =============================================================================
# Simulated Implementations
# =============================================================================


@dataclass
class SimulatedI2CDevice:
    """A simulated I2C device for testing.

    Attributes:
        address: 7-bit device address.
        registers: Dictionary of register values.
        name: Optional device name for logging.
    """

    address: int
    registers: dict[int, int] = field(default_factory=dict)
    name: str = "SimulatedDevice"


class SimulatedI2CBus(I2CBus):
    """Simulated I2C bus for testing without hardware.

    Allows registering simulated devices and their register values
    for testing I2C communication code.

    Example:
        >>> bus = SimulatedI2CBus()
        >>> bus.add_device(0x40, {0x00: 0x12, 0x01: 0x34})
        >>> bus.open()
        >>> print(bus.scan())
        [64]
        >>> print(hex(bus.read_register_byte(0x40, 0x00)))
        0x12
    """

    def __init__(self, config: I2CConfig | None = None, name: str | None = None) -> None:
        """Initialize simulated I2C bus.

        Args:
            config: I2C configuration.
            name: Optional human-readable name.
        """
        super().__init__(config, name or "SimulatedI2C")
        self._devices: dict[int, SimulatedI2CDevice] = {}

    def add_device(
        self, address: int, registers: dict[int, int] | None = None, name: str | None = None
    ) -> SimulatedI2CDevice:
        """Add a simulated device to the bus.

        Args:
            address: 7-bit device address.
            registers: Initial register values.
            name: Optional device name.

        Returns:
            The created SimulatedI2CDevice.
        """
        device = SimulatedI2CDevice(
            address=address,
            registers=registers or {},
            name=name or f"Device@{hex(address)}",
        )
        self._devices[address] = device
        logger.debug("Added simulated I2C device %s at %s", device.name, hex(address))
        return device

    def remove_device(self, address: int) -> None:
        """Remove a simulated device from the bus.

        Args:
            address: Device address to remove.
        """
        if address in self._devices:
            del self._devices[address]
            logger.debug("Removed simulated I2C device at %s", hex(address))

    def get_device(self, address: int) -> SimulatedI2CDevice | None:
        """Get a simulated device by address.

        Args:
            address: Device address.

        Returns:
            The device or None if not found.
        """
        return self._devices.get(address)

    def open(self) -> None:
        """Open the simulated bus."""
        self._is_open = True
        logger.debug("Opened simulated I2C bus %d", self.config.bus_number)

    def close(self) -> None:
        """Close the simulated bus."""
        self._is_open = False
        logger.debug("Closed simulated I2C bus %d", self.config.bus_number)

    def scan(self) -> list[int]:
        """Scan for simulated devices.

        Returns:
            List of addresses with registered devices.
        """
        addresses = sorted(self._devices.keys())
        logger.debug("I2C scan found %d devices: %s", len(addresses), [hex(a) for a in addresses])
        return addresses

    def write(self, address: int, data: bytes | Sequence[int]) -> int:
        """Write to simulated device.

        For simulation, this just logs the write operation.
        Subclasses can override to add behavior.

        Args:
            address: Device address.
            data: Data to write.

        Returns:
            Number of bytes "written".
        """
        data_bytes = bytes(data)
        logger.debug("I2C write to %s: %s", hex(address), data_bytes.hex())
        return len(data_bytes)

    def read(self, address: int, length: int) -> bytes:
        """Read from simulated device.

        Returns zeros for addresses without registered devices.

        Args:
            address: Device address.
            length: Number of bytes to read.

        Returns:
            Bytes read (zeros if no device).
        """
        result = bytes([0x00] * length)
        logger.debug("I2C read from %s (%d bytes): %s", hex(address), length, result.hex())
        return result

    def write_register(self, address: int, register: int, data: bytes | Sequence[int]) -> int:
        """Write to a simulated device register.

        Updates the simulated device's register values.

        Args:
            address: Device address.
            register: Register address.
            data: Data to write.

        Returns:
            Number of bytes written.
        """
        data_bytes = bytes(data)
        device = self._devices.get(address)

        if device:
            for i, byte in enumerate(data_bytes):
                device.registers[register + i] = byte
            logger.debug(
                "I2C write register %s[%s] = %s",
                hex(address),
                hex(register),
                data_bytes.hex(),
            )
        else:
            logger.debug(
                "I2C write register to unknown device %s[%s] = %s",
                hex(address),
                hex(register),
                data_bytes.hex(),
            )

        return len(data_bytes)

    def read_register(self, address: int, register: int, length: int) -> bytes:
        """Read from a simulated device register.

        Returns registered values or zeros.

        Args:
            address: Device address.
            register: Register address.
            length: Number of bytes to read.

        Returns:
            Register values.
        """
        device = self._devices.get(address)

        if device:
            result = bytes([device.registers.get(register + i, 0x00) for i in range(length)])
        else:
            result = bytes([0x00] * length)

        logger.debug(
            "I2C read register %s[%s] (%d bytes): %s",
            hex(address),
            hex(register),
            length,
            result.hex(),
        )
        return result


class SimulatedSPIBus(SPIBus):
    """Simulated SPI bus for testing without hardware.

    By default, returns the input data as output (loopback mode).
    Can be configured with a custom transfer function for testing.

    Example:
        >>> bus = SimulatedSPIBus()
        >>> bus.open()
        >>> result = bus.transfer(bytes([0x01, 0x02, 0x03]))
        >>> print(result.hex())
        010203
    """

    def __init__(self, config: SPIConfig | None = None, name: str | None = None) -> None:
        """Initialize simulated SPI bus.

        Args:
            config: SPI configuration.
            name: Optional human-readable name.
        """
        super().__init__(config, name or "SimulatedSPI")
        self._response_data: bytes = b""
        self._response_offset: int = 0

    def set_response(self, data: bytes) -> None:
        """Set data to return on next transfer(s).

        Args:
            data: Response data bytes.
        """
        self._response_data = data
        self._response_offset = 0

    def open(self) -> None:
        """Open the simulated bus."""
        self._is_open = True
        logger.debug("Opened simulated SPI bus %d:%d", self.config.bus, self.config.device)

    def close(self) -> None:
        """Close the simulated bus."""
        self._is_open = False
        logger.debug("Closed simulated SPI bus %d:%d", self.config.bus, self.config.device)

    def transfer(self, data: bytes | Sequence[int]) -> bytes:
        """Perform simulated transfer.

        Returns response data if set, otherwise returns input (loopback).

        Args:
            data: Data to send.

        Returns:
            Response data or loopback of input.
        """
        data_bytes = bytes(data)
        length = len(data_bytes)

        if self._response_data:
            # Return configured response data
            remaining = len(self._response_data) - self._response_offset
            to_return = min(length, remaining)
            result = self._response_data[self._response_offset : self._response_offset + to_return]
            self._response_offset += to_return

            # Pad with zeros if not enough response data
            if to_return < length:
                result += bytes([0x00] * (length - to_return))
        else:
            # Loopback mode
            result = data_bytes

        logger.debug("SPI transfer: sent=%s, received=%s", data_bytes.hex(), result.hex())
        return result

    def set_speed(self, speed_hz: int) -> None:
        """Set simulated clock speed.

        Args:
            speed_hz: Clock speed in Hz.
        """
        self.config.max_speed_hz = speed_hz
        logger.debug("SPI speed set to %d Hz", speed_hz)

    def set_mode(self, mode: SPIMode) -> None:
        """Set simulated SPI mode.

        Args:
            mode: SPI mode.
        """
        self.config.mode = mode
        logger.debug("SPI mode set to %s", mode)


class SimulatedSerialBus(SerialBus):
    """Simulated serial bus for testing without hardware.

    Provides an in-memory buffer for testing serial communication.

    Example:
        >>> bus = SimulatedSerialBus()
        >>> bus.open()
        >>> bus.set_rx_data(b"OK\\r\\n")
        >>> bus.write(b"AT\\r\\n")
        >>> response = bus.readline()
        >>> print(response)
        b'OK\\r\\n'
    """

    def __init__(self, config: SerialConfig | None = None, name: str | None = None) -> None:
        """Initialize simulated serial bus.

        Args:
            config: Serial configuration.
            name: Optional human-readable name.
        """
        super().__init__(config, name or "SimulatedSerial")
        self._rx_buffer: bytearray = bytearray()
        self._tx_buffer: bytearray = bytearray()

    def set_rx_data(self, data: bytes) -> None:
        """Set data to be received on next read.

        Args:
            data: Data to receive.
        """
        self._rx_buffer.extend(data)

    def get_tx_data(self) -> bytes:
        """Get data that was written.

        Returns:
            Data from TX buffer.
        """
        data = bytes(self._tx_buffer)
        self._tx_buffer.clear()
        return data

    def open(self) -> None:
        """Open the simulated port."""
        self._is_open = True
        logger.debug("Opened simulated serial port %s", self.config.port)

    def close(self) -> None:
        """Close the simulated port."""
        self._is_open = False
        logger.debug("Closed simulated serial port %s", self.config.port)

    def write(self, data: bytes | str) -> int:
        """Write to simulated port.

        Args:
            data: Data to write.

        Returns:
            Number of bytes written.
        """
        if isinstance(data, str):
            data = data.encode()

        self._tx_buffer.extend(data)
        logger.debug("Serial write: %s", data.hex())
        return len(data)

    def read(self, length: int) -> bytes:
        """Read from simulated port.

        Args:
            length: Number of bytes to read.

        Returns:
            Bytes read from RX buffer.
        """
        data = bytes(self._rx_buffer[:length])
        del self._rx_buffer[:length]
        logger.debug("Serial read (%d bytes): %s", length, data.hex())
        return data

    def readline(self) -> bytes:
        """Read a line from simulated port.

        Returns:
            Line including newline, or all available data.
        """
        return self.read_until(b"\n")

    def read_until(self, terminator: bytes = b"\n") -> bytes:
        """Read until terminator.

        Args:
            terminator: Byte sequence to stop at.

        Returns:
            Data including terminator.
        """
        idx = self._rx_buffer.find(terminator)
        if idx >= 0:
            end = idx + len(terminator)
            data = bytes(self._rx_buffer[:end])
            del self._rx_buffer[:end]
        else:
            data = bytes(self._rx_buffer)
            self._rx_buffer.clear()

        logger.debug("Serial read_until(%s): %s", terminator.hex(), data.hex())
        return data

    @property
    def in_waiting(self) -> int:
        """Number of bytes in RX buffer."""
        return len(self._rx_buffer)

    @property
    def out_waiting(self) -> int:
        """Number of bytes in TX buffer."""
        return len(self._tx_buffer)

    def flush(self) -> None:
        """Clear TX buffer (simulate flush)."""
        self._tx_buffer.clear()
        logger.debug("Serial flush")

    def reset_input_buffer(self) -> None:
        """Clear RX buffer."""
        self._rx_buffer.clear()
        logger.debug("Serial reset input buffer")

    def reset_output_buffer(self) -> None:
        """Clear TX buffer."""
        self._tx_buffer.clear()
        logger.debug("Serial reset output buffer")

    def set_baudrate(self, baudrate: int) -> None:
        """Set simulated baud rate.

        Args:
            baudrate: New baud rate.
        """
        self.config.baudrate = baudrate
        logger.debug("Serial baudrate set to %d", baudrate)


# =============================================================================
# Factory Functions
# =============================================================================


def get_i2c(
    bus_number: int = 1,
    frequency: int = 100_000,
    *,
    simulate: bool | None = None,
) -> I2CBus:
    """Get an I2C bus instance.

    Attempts to use real hardware if available, falls back to simulation.

    Args:
        bus_number: I2C bus number (e.g., 1 for /dev/i2c-1).
        frequency: Clock frequency in Hz.
        simulate: Force simulation mode. If None, auto-detect.

    Returns:
        I2CBus instance (real or simulated).

    Example:
        >>> i2c = get_i2c(1)
        >>> i2c.open()
        >>> devices = i2c.scan()
    """
    config = I2CConfig(bus_number=bus_number, frequency=frequency)

    # Track if simulation was explicitly requested vs auto-detected
    simulate_explicit = simulate is not None

    if simulate is None:
        # Try to detect if we have real hardware
        try:
            import smbus2  # noqa: F401

            simulate = False
        except ImportError:
            simulate = True

    if simulate:
        # Only require ROBO_SIMULATION env var for auto-detected simulation
        # (explicit simulate=True is allowed without it)
        if not simulate_explicit and not os.getenv("ROBO_SIMULATION"):
            raise HardwareNotFoundError(
                "No I2C hardware detected (smbus2 not available). "
                "Set ROBO_SIMULATION=true to use simulated hardware."
            )
        logger.warning("⚠️ SIMULATION MODE — Using simulated I2C bus %d", bus_number)
        return SimulatedI2CBus(config)

    # Use real hardware implementation
    logger.info("Using real I2C bus %d with smbus2", bus_number)
    return SMBus2I2CBus(config)


def get_spi(
    bus: int = 0,
    device: int = 0,
    max_speed_hz: int = 1_000_000,
    mode: SPIMode = SPIMode.MODE_0,
    *,
    simulate: bool | None = None,
) -> SPIBus:
    """Get an SPI bus instance.

    Attempts to use real hardware if available, falls back to simulation.

    Args:
        bus: SPI bus number.
        device: SPI device/chip-select number.
        max_speed_hz: Maximum clock speed.
        mode: SPI mode (0-3).
        simulate: Force simulation mode. If None, auto-detect.

    Returns:
        SPIBus instance (real or simulated).

    Example:
        >>> spi = get_spi(0, 0)
        >>> spi.open()
        >>> response = spi.transfer(bytes([0x01, 0x02]))
    """
    config = SPIConfig(bus=bus, device=device, max_speed_hz=max_speed_hz, mode=mode)

    # Track if simulation was explicitly requested vs auto-detected
    simulate_explicit = simulate is not None

    if simulate is None:
        # Try to detect if we have real hardware
        try:
            import spidev  # noqa: F401

            simulate = False
        except ImportError:
            simulate = True

    if simulate:
        # Only require ROBO_SIMULATION env var for auto-detected simulation
        if not simulate_explicit and not os.getenv("ROBO_SIMULATION"):
            raise HardwareNotFoundError(
                "No SPI hardware detected (spidev not available). "
                "Set ROBO_SIMULATION=true to use simulated hardware."
            )
        logger.warning("⚠️ SIMULATION MODE — Using simulated SPI bus %d:%d", bus, device)
        return SimulatedSPIBus(config)

    # Use real hardware implementation
    logger.info("Using real SPI bus %d:%d with spidev", bus, device)
    return SpiDevSPIBus(config)


def get_serial(
    port: str = "/dev/ttyUSB0",
    baudrate: int = 9600,
    timeout: float | None = 1.0,
    *,
    simulate: bool | None = None,
) -> SerialBus:
    """Get a serial bus instance.

    Attempts to use real hardware if available, falls back to simulation.

    Args:
        port: Serial port path.
        baudrate: Baud rate.
        timeout: Read timeout in seconds.
        simulate: Force simulation mode. If None, auto-detect.

    Returns:
        SerialBus instance (real or simulated).

    Example:
        >>> serial = get_serial("/dev/ttyUSB0", 115200)
        >>> serial.open()
        >>> serial.write(b"AT\\r\\n")
        >>> response = serial.readline()
    """
    config = SerialConfig(port=port, baudrate=baudrate, timeout=timeout)

    # Track if simulation was explicitly requested vs auto-detected
    simulate_explicit = simulate is not None

    if simulate is None:
        # Try to detect if we have real hardware
        try:
            import serial  # noqa: F401

            simulate = False
        except ImportError:
            simulate = True

    if simulate:
        # Only require ROBO_SIMULATION env var for auto-detected simulation
        if not simulate_explicit and not os.getenv("ROBO_SIMULATION"):
            raise HardwareNotFoundError(
                f"No serial hardware detected (pyserial not available for {port}). "
                "Set ROBO_SIMULATION=true to use simulated hardware."
            )
        logger.warning("⚠️ SIMULATION MODE — Using simulated serial port %s", port)
        return SimulatedSerialBus(config)

    # Use real hardware implementation
    logger.info("Using real serial port %s with pyserial", port)
    return PySerialBus(config)
