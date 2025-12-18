"""Arduino/Serial driver for hardware control.

This module provides a driver for controlling hardware via Arduino or similar
microcontrollers using serial communication. Supports both a simple command
protocol and the Firmata protocol for more advanced control.

Simple Protocol:
    The default protocol uses simple ASCII commands:
    - Set PWM: P<channel><value> (e.g., "P0127" sets channel 0 to 127)
    - Read analog: A<channel> → returns value (e.g., "A0" reads analog 0)
    - Read digital: D<pin> → returns 0/1 (e.g., "D13" reads digital pin 13)
    - Write digital: W<pin><state> (e.g., "W131" sets pin 13 HIGH)
    - Set servo: S<channel><angle> (e.g., "S090" sets servo 0 to 90 degrees)

Firmata Protocol:
    Optional Firmata support for standardized microcontroller control.
    Requires pyfirmata or pyfirmata2 library.

Example:
    >>> from robo_infra.drivers.arduino import ArduinoDriver
    >>>
    >>> # Connect to Arduino on serial port
    >>> driver = ArduinoDriver(port="/dev/ttyUSB0", baudrate=115200)
    >>> driver.connect()
    >>>
    >>> # Set PWM on channel 0 (0-255)
    >>> driver.set_pwm(0, 127)  # 50% duty cycle
    >>>
    >>> # Read analog value (0-1023)
    >>> value = driver.read_analog(0)
    >>>
    >>> # Read/write digital pins
    >>> state = driver.read_digital(13)
    >>> driver.write_digital(13, True)
    >>>
    >>> # Using as a standard Driver
    >>> driver.set_channel(0, 0.5)  # Sets PWM to 127
    >>>
    >>> driver.disconnect()

Hardware Reference:
    - Default baud rate: 115200
    - PWM resolution: 8-bit (0-255)
    - Analog resolution: 10-bit (0-1023)
    - Default response timeout: 1.0 second
"""

from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import TYPE_CHECKING, Any

from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import (
    CommunicationError,
    HardwareNotFoundError,
    TimeoutError,
)


if TYPE_CHECKING:
    import serial as serial_module


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

# Default serial settings
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 1.0  # seconds
DEFAULT_WRITE_TIMEOUT = 1.0  # seconds
DEFAULT_RESPONSE_TIMEOUT = 1.0  # seconds

# Arduino limits
ARDUINO_PWM_RESOLUTION = 256  # 8-bit (0-255)
ARDUINO_PWM_MAX = 255
ARDUINO_ANALOG_RESOLUTION = 1024  # 10-bit (0-1023)
ARDUINO_ANALOG_MAX = 1023
ARDUINO_DEFAULT_CHANNELS = 6  # Default analog channels (A0-A5)
ARDUINO_DEFAULT_PWM_PINS = 6  # Typical PWM pins (3, 5, 6, 9, 10, 11)

# Command protocol
CMD_SET_PWM = ord("P")
CMD_READ_ANALOG = ord("A")
CMD_READ_DIGITAL = ord("D")
CMD_WRITE_DIGITAL = ord("W")
CMD_SET_SERVO = ord("S")
CMD_SET_MODE = ord("M")
CMD_RESPONSE = ord("R")
CMD_ERROR = ord("E")
CMD_OK = ord("K")

# Protocol terminators
PROTOCOL_TERMINATOR = b"\n"
PROTOCOL_SEPARATOR = b","

# Firmata constants (if using Firmata mode)
FIRMATA_VERSION_QUERY = 0xF9
FIRMATA_ANALOG_MESSAGE = 0xE0
FIRMATA_DIGITAL_MESSAGE = 0x90
FIRMATA_REPORT_ANALOG = 0xC0
FIRMATA_REPORT_DIGITAL = 0xD0
FIRMATA_SET_PIN_MODE = 0xF4
FIRMATA_SET_DIGITAL_PIN_VALUE = 0xF5


# =============================================================================
# Enums
# =============================================================================


class ArduinoProtocol(Enum):
    """Communication protocol to use with Arduino."""

    SIMPLE = "simple"  # Simple ASCII command protocol
    FIRMATA = "firmata"  # Standard Firmata protocol
    CUSTOM = "custom"  # Custom user-defined protocol


class PinMode(IntEnum):
    """Arduino pin modes."""

    INPUT = 0
    OUTPUT = 1
    INPUT_PULLUP = 2
    PWM = 3
    SERVO = 4
    ANALOG = 5


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class SerialConfig:
    """Serial port configuration.

    Attributes:
        port: Serial port path (e.g., "/dev/ttyUSB0", "COM3").
        baudrate: Communication speed in bits per second.
        timeout: Read timeout in seconds.
        write_timeout: Write timeout in seconds.
        bytesize: Number of data bits (5, 6, 7, or 8).
        parity: Parity checking ('N', 'E', 'O', 'M', 'S').
        stopbits: Number of stop bits (1, 1.5, or 2).
        xonxoff: Enable software flow control.
        rtscts: Enable hardware flow control.
        dsrdtr: Enable DSR/DTR flow control.
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = DEFAULT_BAUDRATE
    timeout: float = DEFAULT_TIMEOUT
    write_timeout: float = DEFAULT_WRITE_TIMEOUT
    bytesize: int = 8
    parity: str = "N"
    stopbits: float = 1
    xonxoff: bool = False
    rtscts: bool = False
    dsrdtr: bool = False


@dataclass
class ArduinoConfig:
    """Arduino driver configuration.

    Attributes:
        serial: Serial port configuration.
        protocol: Communication protocol to use.
        pwm_channels: Number of PWM channels (default 6).
        analog_channels: Number of analog input channels (default 6).
        digital_pins: Number of digital pins (default 14).
        response_timeout: Timeout for command responses.
        auto_detect_port: Try to auto-detect Arduino port.
        wait_for_ready: Wait for Arduino ready signal after connect.
        ready_timeout: Timeout for Arduino ready signal.
        simulation: Force simulation mode.
        pwm_pins: Mapping of channel numbers to PWM-capable pins.
        analog_pins: Mapping of channel numbers to analog pins.
    """

    serial: SerialConfig = field(default_factory=SerialConfig)
    protocol: ArduinoProtocol = ArduinoProtocol.SIMPLE
    pwm_channels: int = ARDUINO_DEFAULT_PWM_PINS
    analog_channels: int = ARDUINO_DEFAULT_CHANNELS
    digital_pins: int = 14
    response_timeout: float = DEFAULT_RESPONSE_TIMEOUT
    auto_detect_port: bool = True
    wait_for_ready: bool = True
    ready_timeout: float = 5.0
    simulation: bool = False
    pwm_pins: dict[int, int] = field(default_factory=lambda: {0: 3, 1: 5, 2: 6, 3: 9, 4: 10, 5: 11})
    analog_pins: dict[int, int] = field(
        default_factory=lambda: {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}  # A0-A5
    )


@dataclass
class ArduinoPinState:
    """State of an Arduino pin.

    Attributes:
        pin: Physical pin number.
        mode: Pin mode (INPUT, OUTPUT, PWM, etc.).
        value: Current value (0-255 for PWM, 0/1 for digital, 0-1023 for analog).
        last_update: Timestamp of last update.
    """

    pin: int
    mode: PinMode = PinMode.OUTPUT
    value: int = 0
    last_update: float = 0.0


# =============================================================================
# Arduino Driver Implementation
# =============================================================================


@register_driver("arduino")
class ArduinoDriver(Driver):
    """Driver for Arduino and compatible microcontrollers via serial.

    This driver communicates with Arduino using either a simple ASCII protocol
    or the Firmata protocol. It supports:
    - PWM output (8-bit, 0-255)
    - Analog input (10-bit, 0-1023)
    - Digital I/O
    - Servo control

    The simple protocol is lightweight and requires matching firmware on the
    Arduino. Firmata mode uses the standard Firmata protocol.

    Thread Safety:
        All serial operations are protected by a lock for thread-safe access.

    Simulation Mode:
        When the serial port is not available, the driver can operate in
        simulation mode for testing purposes.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = DEFAULT_BAUDRATE,
        config: ArduinoConfig | None = None,
        simulation: bool = False,
    ) -> None:
        """Initialize the Arduino driver.

        Args:
            port: Serial port path. If None, will try to auto-detect.
            baudrate: Serial baud rate (default: 115200).
            config: Full configuration object (overrides port/baudrate).
            simulation: Force simulation mode (no hardware).
        """
        # Build configuration
        if config is None:
            config = ArduinoConfig()
            if port is not None:
                config.serial.port = port
            config.serial.baudrate = baudrate
            config.simulation = simulation
        elif simulation:
            config.simulation = True

        self._arduino_config = config

        # Initialize base Driver
        super().__init__(
            name="ArduinoDriver",
            channels=config.pwm_channels,
            config=DriverConfig(
                name="ArduinoDriver",
                channels=config.pwm_channels,
            ),
        )

        # Serial connection
        self._serial: serial_module.Serial | None = None
        self._serial_lock = threading.RLock()

        # Simulation state
        self._simulation_mode = config.simulation or os.getenv("ROBO_SIMULATION") == "true"
        self._simulated_analog: dict[int, int] = {}
        self._simulated_digital: dict[int, bool] = {}

        # Pin states
        self._pin_states: dict[int, ArduinoPinState] = {}
        self._pwm_values: dict[int, int] = {}  # Channel -> PWM value (0-255)
        self._analog_values: dict[int, int] = {}  # Channel -> analog value (0-1023)

        # Initialize PWM channels
        for ch in range(config.pwm_channels):
            self._pwm_values[ch] = 0

        # Firmata-specific
        self._firmata_board: Any = None

        logger.debug(
            "ArduinoDriver initialized: port=%s, baudrate=%d, simulation=%s",
            config.serial.port,
            config.serial.baudrate,
            self._simulation_mode,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def arduino_config(self) -> ArduinoConfig:
        """Get the Arduino-specific configuration."""
        return self._arduino_config

    @property
    def port(self) -> str:
        """Get the serial port path."""
        return self._arduino_config.serial.port

    @property
    def baudrate(self) -> int:
        """Get the serial baud rate."""
        return self._arduino_config.serial.baudrate

    @property
    def is_simulation(self) -> bool:
        """Check if running in simulation mode."""
        return self._simulation_mode

    @property
    def protocol(self) -> ArduinoProtocol:
        """Get the communication protocol in use."""
        return self._arduino_config.protocol

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the Arduino.

        Establishes serial communication with the Arduino. In simulation mode,
        simulates a successful connection.

        Raises:
            CommunicationError: If connection fails.
            HardwareNotFoundError: If port is not found.
        """
        if self._state == DriverState.CONNECTED:
            logger.debug("ArduinoDriver already connected")
            return

        self._state = DriverState.CONNECTING

        if self._simulation_mode:
            logger.warning("⚠️ SIMULATION MODE - ArduinoDriver not connected to real hardware")
            self._state = DriverState.CONNECTED
            return

        # Check for Firmata mode
        if self._arduino_config.protocol == ArduinoProtocol.FIRMATA:
            self._connect_firmata()
            return

        # Simple protocol - use pyserial
        try:
            serial = self._get_serial_module()

            # Auto-detect port if needed
            port = self._arduino_config.serial.port
            if self._arduino_config.auto_detect_port and port == "/dev/ttyUSB0":
                detected = self._detect_arduino_port()
                if detected:
                    port = detected
                    self._arduino_config.serial.port = port

            # Open serial connection
            self._serial = serial.Serial(
                port=port,
                baudrate=self._arduino_config.serial.baudrate,
                timeout=self._arduino_config.serial.timeout,
                write_timeout=self._arduino_config.serial.write_timeout,
                bytesize=self._arduino_config.serial.bytesize,
                parity=self._arduino_config.serial.parity,
                stopbits=self._arduino_config.serial.stopbits,
                xonxoff=self._arduino_config.serial.xonxoff,
                rtscts=self._arduino_config.serial.rtscts,
                dsrdtr=self._arduino_config.serial.dsrdtr,
            )

            # Wait for Arduino to reset (it resets on serial connection)
            if self._arduino_config.wait_for_ready:
                self._wait_for_ready()

            self._state = DriverState.CONNECTED
            logger.info(
                "ArduinoDriver connected on %s at %d baud",
                port,
                self._arduino_config.serial.baudrate,
            )

        except Exception as e:
            self._state = DriverState.ERROR
            if "No such file or directory" in str(e) or "FileNotFoundError" in str(e):
                raise HardwareNotFoundError(f"Arduino port not found: {port}") from e
            raise CommunicationError(f"Failed to connect to Arduino: {e}") from e

    def disconnect(self) -> None:
        """Disconnect from the Arduino.

        Closes the serial connection and resets all outputs to safe state.
        """
        if self._state == DriverState.DISCONNECTED:
            return

        # Reset all PWM channels to 0
        try:
            if self.is_connected and not self._simulation_mode:
                for ch in range(self._arduino_config.pwm_channels):
                    self._send_pwm_command(ch, 0)
        except Exception as e:
            logger.warning("Error resetting channels on disconnect: %s", e)

        # Close serial connection
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception as e:
                    logger.warning("Error closing serial port: %s", e)
                finally:
                    self._serial = None

        # Close Firmata board
        if self._firmata_board is not None:
            try:
                self._firmata_board.exit()
            except Exception as e:
                logger.warning("Error closing Firmata board: %s", e)
            finally:
                self._firmata_board = None

        self._state = DriverState.DISCONNECTED
        logger.info("ArduinoDriver disconnected")

    def _connect_firmata(self) -> None:
        """Connect using Firmata protocol."""
        try:
            try:
                from pyfirmata2 import Arduino
            except ImportError:
                try:
                    from pyfirmata import Arduino
                except ImportError as e:
                    raise ImportError(
                        "Firmata mode requires pyfirmata or pyfirmata2. "
                        "Install with: pip install pyfirmata2"
                    ) from e

            port = self._arduino_config.serial.port
            if self._arduino_config.auto_detect_port and port == "/dev/ttyUSB0":
                detected = self._detect_arduino_port()
                if detected:
                    port = detected

            self._firmata_board = Arduino(port)
            self._state = DriverState.CONNECTED
            logger.info("ArduinoDriver connected via Firmata on %s", port)

        except Exception as e:
            self._state = DriverState.ERROR
            raise CommunicationError(f"Failed to connect via Firmata: {e}") from e

    def _get_serial_module(self) -> Any:
        """Import and return the pyserial module."""
        try:
            import serial

            return serial
        except ImportError as e:
            raise ImportError("pyserial is required. Install with: pip install pyserial") from e

    def _detect_arduino_port(self) -> str | None:
        """Try to auto-detect Arduino serial port.

        Returns:
            Detected port path, or None if not found.
        """
        try:
            from serial.tools import list_ports

            # Common Arduino USB identifiers
            arduino_vids = [0x2341, 0x2A03, 0x1A86]  # Arduino, Arduino clone, CH340

            for port in list_ports.comports():
                if port.vid in arduino_vids:
                    logger.info("Auto-detected Arduino on %s", port.device)
                    return port.device

            # Fallback: Look for common patterns
            for port in list_ports.comports():
                if any(name in port.device.lower() for name in ["usb", "acm", "arduino"]):
                    logger.info("Auto-detected likely Arduino on %s", port.device)
                    return port.device

        except Exception as e:
            logger.debug("Port detection failed: %s", e)

        return None

    def _wait_for_ready(self) -> None:
        """Wait for Arduino to be ready after connection.

        Arduino resets when serial connection is opened. This waits for
        the ready signal or a timeout.
        """
        if self._serial is None:
            return

        deadline = time.time() + self._arduino_config.ready_timeout

        # Flush any startup garbage
        time.sleep(0.1)
        self._serial.reset_input_buffer()

        # Wait for ready signal or timeout
        while time.time() < deadline:
            if self._serial.in_waiting > 0:
                line = self._serial.readline()
                if b"READY" in line.upper() or b"OK" in line.upper():
                    logger.debug("Arduino ready signal received")
                    return
            time.sleep(0.1)

        logger.debug("No ready signal received, proceeding anyway")

    # -------------------------------------------------------------------------
    # Driver Interface Implementation
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write a value to a PWM channel.

        Implements the abstract Driver method. Converts 0.0-1.0 to 0-255.

        Args:
            channel: PWM channel number (0-indexed).
            value: Value from 0.0 to 1.0.
        """
        pwm_value = int(value * ARDUINO_PWM_MAX)
        self.set_pwm(channel, pwm_value)

    def _read_channel(self, channel: int) -> float:
        """Read the current value of a channel.

        Implements the abstract Driver method. Returns the last set PWM value.

        Args:
            channel: Channel number (0-indexed).

        Returns:
            Current value from 0.0 to 1.0.
        """
        pwm_value = self._pwm_values.get(channel, 0)
        return pwm_value / ARDUINO_PWM_MAX

    # -------------------------------------------------------------------------
    # PWM Operations
    # -------------------------------------------------------------------------

    def set_pwm(self, channel: int, value: int) -> None:
        """Set PWM value on a channel.

        Args:
            channel: PWM channel number (0-indexed).
            value: PWM value from 0 to 255.

        Raises:
            ValueError: If channel or value is out of range.
            CommunicationError: If communication fails.
        """
        if channel < 0 or channel >= self._arduino_config.pwm_channels:
            raise ValueError(
                f"Channel {channel} out of range (0-{self._arduino_config.pwm_channels - 1})"
            )

        # Clamp value
        value = max(0, min(ARDUINO_PWM_MAX, value))
        self._pwm_values[channel] = value

        if self._simulation_mode:
            logger.debug("Simulated PWM: channel=%d, value=%d", channel, value)
            return

        if self._arduino_config.protocol == ArduinoProtocol.FIRMATA:
            self._set_pwm_firmata(channel, value)
        else:
            self._send_pwm_command(channel, value)

    def get_pwm(self, channel: int) -> int:
        """Get the current PWM value of a channel.

        Args:
            channel: PWM channel number (0-indexed).

        Returns:
            Current PWM value (0-255).
        """
        if channel < 0 or channel >= self._arduino_config.pwm_channels:
            raise ValueError(
                f"Channel {channel} out of range (0-{self._arduino_config.pwm_channels - 1})"
            )
        return self._pwm_values.get(channel, 0)

    def _send_pwm_command(self, channel: int, value: int) -> None:
        """Send PWM command using simple protocol."""
        # Get the physical pin number
        pin = self._arduino_config.pwm_pins.get(channel, channel)

        # Format: P<pin><value> with zero-padded 3-digit values
        command = f"P{pin:02d}{value:03d}\n".encode()
        self._send_command(command)

    def _set_pwm_firmata(self, channel: int, value: int) -> None:
        """Set PWM using Firmata protocol."""
        if self._firmata_board is None:
            raise CommunicationError("Firmata board not connected")

        pin = self._arduino_config.pwm_pins.get(channel, channel)
        try:
            # Get or configure pin for PWM
            digital_pin = self._firmata_board.digital[pin]
            digital_pin.mode = 3  # PWM mode
            digital_pin.write(value / ARDUINO_PWM_MAX)
        except Exception as e:
            raise CommunicationError(f"Firmata PWM write failed: {e}") from e

    # -------------------------------------------------------------------------
    # Analog Operations
    # -------------------------------------------------------------------------

    def read_analog(self, channel: int) -> int:
        """Read analog value from a channel.

        Args:
            channel: Analog channel number (0-indexed, typically A0-A5).

        Returns:
            Analog value from 0 to 1023.

        Raises:
            ValueError: If channel is out of range.
            CommunicationError: If communication fails.
        """
        if channel < 0 or channel >= self._arduino_config.analog_channels:
            raise ValueError(
                f"Analog channel {channel} out of range (0-{self._arduino_config.analog_channels - 1})"
            )

        if self._simulation_mode:
            value = self._simulated_analog.get(channel, 0)
            logger.debug("Simulated analog read: channel=%d, value=%d", channel, value)
            return value

        if self._arduino_config.protocol == ArduinoProtocol.FIRMATA:
            return self._read_analog_firmata(channel)

        return self._send_analog_read_command(channel)

    def read_analog_voltage(self, channel: int, vref: float = 5.0) -> float:
        """Read analog value as voltage.

        Args:
            channel: Analog channel number.
            vref: Reference voltage (default 5.0V for Arduino).

        Returns:
            Voltage value.
        """
        raw = self.read_analog(channel)
        return (raw / ARDUINO_ANALOG_MAX) * vref

    def _send_analog_read_command(self, channel: int) -> int:
        """Read analog using simple protocol."""
        pin = self._arduino_config.analog_pins.get(channel, channel)

        # Format: A<pin>
        command = f"A{pin:02d}\n".encode()
        response = self._send_command(command, expect_response=True)

        try:
            value = int(response.strip())
            value = max(0, min(ARDUINO_ANALOG_MAX, value))
            self._analog_values[channel] = value
            return value
        except ValueError as e:
            raise CommunicationError(f"Invalid analog response: {response}") from e

    def _read_analog_firmata(self, channel: int) -> int:
        """Read analog using Firmata protocol."""
        if self._firmata_board is None:
            raise CommunicationError("Firmata board not connected")

        try:
            # Enable analog reporting if needed
            analog_pin = self._firmata_board.analog[channel]
            analog_pin.enable_reporting()

            # Wait for value
            time.sleep(0.02)

            # Read value (Firmata returns 0.0-1.0)
            value = analog_pin.read()
            if value is None:
                return self._analog_values.get(channel, 0)

            int_value = int(value * ARDUINO_ANALOG_MAX)
            self._analog_values[channel] = int_value
            return int_value

        except Exception as e:
            raise CommunicationError(f"Firmata analog read failed: {e}") from e

    def set_simulated_analog(self, channel: int, value: int) -> None:
        """Set simulated analog value for testing.

        Args:
            channel: Analog channel number.
            value: Value to simulate (0-1023).
        """
        self._simulated_analog[channel] = max(0, min(ARDUINO_ANALOG_MAX, value))

    # -------------------------------------------------------------------------
    # Digital Operations
    # -------------------------------------------------------------------------

    def read_digital(self, pin: int) -> bool:
        """Read digital value from a pin.

        Args:
            pin: Digital pin number.

        Returns:
            True if HIGH, False if LOW.

        Raises:
            ValueError: If pin is out of range.
            CommunicationError: If communication fails.
        """
        if pin < 0 or pin >= self._arduino_config.digital_pins:
            raise ValueError(
                f"Digital pin {pin} out of range (0-{self._arduino_config.digital_pins - 1})"
            )

        if self._simulation_mode:
            state = self._simulated_digital.get(pin, False)
            logger.debug("Simulated digital read: pin=%d, state=%s", pin, state)
            return state

        if self._arduino_config.protocol == ArduinoProtocol.FIRMATA:
            return self._read_digital_firmata(pin)

        return self._send_digital_read_command(pin)

    def write_digital(self, pin: int, state: bool) -> None:
        """Write digital value to a pin.

        Args:
            pin: Digital pin number.
            state: True for HIGH, False for LOW.

        Raises:
            ValueError: If pin is out of range.
            CommunicationError: If communication fails.
        """
        if pin < 0 or pin >= self._arduino_config.digital_pins:
            raise ValueError(
                f"Digital pin {pin} out of range (0-{self._arduino_config.digital_pins - 1})"
            )

        if self._simulation_mode:
            self._simulated_digital[pin] = state
            logger.debug("Simulated digital write: pin=%d, state=%s", pin, state)
            return

        if self._arduino_config.protocol == ArduinoProtocol.FIRMATA:
            self._write_digital_firmata(pin, state)
        else:
            self._send_digital_write_command(pin, state)

    def _send_digital_read_command(self, pin: int) -> bool:
        """Read digital using simple protocol."""
        command = f"D{pin:02d}\n".encode()
        response = self._send_command(command, expect_response=True)

        try:
            value = int(response.strip())
            return value != 0
        except ValueError as e:
            raise CommunicationError(f"Invalid digital response: {response}") from e

    def _send_digital_write_command(self, pin: int, state: bool) -> None:
        """Write digital using simple protocol."""
        value = 1 if state else 0
        command = f"W{pin:02d}{value}\n".encode()
        self._send_command(command)

    def _read_digital_firmata(self, pin: int) -> bool:
        """Read digital using Firmata protocol."""
        if self._firmata_board is None:
            raise CommunicationError("Firmata board not connected")

        try:
            digital_pin = self._firmata_board.digital[pin]
            digital_pin.mode = 0  # Input mode
            value = digital_pin.read()
            return bool(value)
        except Exception as e:
            raise CommunicationError(f"Firmata digital read failed: {e}") from e

    def _write_digital_firmata(self, pin: int, state: bool) -> None:
        """Write digital using Firmata protocol."""
        if self._firmata_board is None:
            raise CommunicationError("Firmata board not connected")

        try:
            digital_pin = self._firmata_board.digital[pin]
            digital_pin.mode = 1  # Output mode
            digital_pin.write(1 if state else 0)
        except Exception as e:
            raise CommunicationError(f"Firmata digital write failed: {e}") from e

    def set_simulated_digital(self, pin: int, state: bool) -> None:
        """Set simulated digital value for testing.

        Args:
            pin: Digital pin number.
            state: State to simulate (True/False).
        """
        self._simulated_digital[pin] = state

    # -------------------------------------------------------------------------
    # Servo Operations
    # -------------------------------------------------------------------------

    def set_servo(self, channel: int, angle: int) -> None:
        """Set servo angle.

        Args:
            channel: Servo channel number.
            angle: Angle in degrees (typically 0-180).

        Raises:
            ValueError: If angle is out of range.
            CommunicationError: If communication fails.
        """
        if angle < 0 or angle > 180:
            raise ValueError(f"Servo angle {angle} out of range (0-180)")

        if self._simulation_mode:
            logger.debug("Simulated servo: channel=%d, angle=%d", channel, angle)
            return

        if self._arduino_config.protocol == ArduinoProtocol.FIRMATA:
            self._set_servo_firmata(channel, angle)
        else:
            self._send_servo_command(channel, angle)

    def _send_servo_command(self, channel: int, angle: int) -> None:
        """Send servo command using simple protocol."""
        pin = self._arduino_config.pwm_pins.get(channel, channel)
        command = f"S{pin:02d}{angle:03d}\n".encode()
        self._send_command(command)

    def _set_servo_firmata(self, channel: int, angle: int) -> None:
        """Set servo using Firmata protocol."""
        if self._firmata_board is None:
            raise CommunicationError("Firmata board not connected")

        pin = self._arduino_config.pwm_pins.get(channel, channel)
        try:
            servo_pin = self._firmata_board.digital[pin]
            servo_pin.mode = 4  # Servo mode
            servo_pin.write(angle)
        except Exception as e:
            raise CommunicationError(f"Firmata servo write failed: {e}") from e

    # -------------------------------------------------------------------------
    # Pin Mode Control
    # -------------------------------------------------------------------------

    def set_pin_mode(self, pin: int, mode: PinMode) -> None:
        """Set the mode of a pin.

        Args:
            pin: Pin number.
            mode: Pin mode to set.

        Raises:
            CommunicationError: If communication fails.
        """
        if self._simulation_mode:
            self._pin_states[pin] = ArduinoPinState(pin=pin, mode=mode)
            return

        if self._arduino_config.protocol == ArduinoProtocol.FIRMATA:
            self._set_pin_mode_firmata(pin, mode)
        else:
            self._send_pin_mode_command(pin, mode)

        self._pin_states[pin] = ArduinoPinState(pin=pin, mode=mode)

    def _send_pin_mode_command(self, pin: int, mode: PinMode) -> None:
        """Set pin mode using simple protocol."""
        command = f"M{pin:02d}{mode.value}\n".encode()
        self._send_command(command)

    def _set_pin_mode_firmata(self, pin: int, mode: PinMode) -> None:
        """Set pin mode using Firmata protocol."""
        if self._firmata_board is None:
            raise CommunicationError("Firmata board not connected")

        try:
            digital_pin = self._firmata_board.digital[pin]
            digital_pin.mode = mode.value
        except Exception as e:
            raise CommunicationError(f"Firmata pin mode set failed: {e}") from e

    # -------------------------------------------------------------------------
    # Low-level Communication
    # -------------------------------------------------------------------------

    def _send_command(
        self,
        command: bytes,
        expect_response: bool = False,
    ) -> str:
        """Send a command and optionally wait for response.

        Args:
            command: Command bytes to send.
            expect_response: Whether to wait for and return response.

        Returns:
            Response string if expect_response is True, empty string otherwise.

        Raises:
            CommunicationError: If communication fails.
            TimeoutError: If response times out.
        """
        if self._serial is None:
            raise CommunicationError("Serial port not connected")

        with self._serial_lock:
            try:
                # Clear input buffer
                self._serial.reset_input_buffer()

                # Send command
                self._serial.write(command)
                self._serial.flush()

                logger.debug("Sent command: %s", command.strip())

                if not expect_response:
                    return ""

                # Wait for response
                deadline = time.time() + self._arduino_config.response_timeout
                response_data = b""

                while time.time() < deadline:
                    if self._serial.in_waiting > 0:
                        chunk = self._serial.read(self._serial.in_waiting)
                        response_data += chunk

                        # Check for complete response (ends with newline)
                        if response_data.endswith(PROTOCOL_TERMINATOR):
                            response = response_data.decode("utf-8", errors="replace")
                            logger.debug("Received response: %s", response.strip())
                            return response

                    time.sleep(0.01)

                # Timeout
                raise TimeoutError(
                    operation="arduino_read_response",
                    timeout=self._arduino_config.response_timeout,
                )

            except TimeoutError:
                raise
            except Exception as e:
                raise CommunicationError(f"Serial communication failed: {e}") from e

    def send_raw(self, data: bytes) -> bytes:
        """Send raw bytes and return response.

        For advanced use cases or custom protocols.

        Args:
            data: Raw bytes to send.

        Returns:
            Response bytes.
        """
        if self._simulation_mode:
            logger.debug("Simulated raw send: %s", data)
            return b"OK\n"

        if self._serial is None:
            raise CommunicationError("Serial port not connected")

        with self._serial_lock:
            self._serial.write(data)
            self._serial.flush()

            # Read response with timeout
            deadline = time.time() + self._arduino_config.response_timeout
            response = b""

            while time.time() < deadline:
                if self._serial.in_waiting > 0:
                    response += self._serial.read(self._serial.in_waiting)
                    if response.endswith(PROTOCOL_TERMINATOR):
                        return response
                time.sleep(0.01)

            return response

    # -------------------------------------------------------------------------
    # Utility Methods
    # -------------------------------------------------------------------------

    def reset(self) -> None:
        """Reset the Arduino.

        Uses DTR line to trigger a reset on most Arduino boards.
        """
        if self._simulation_mode:
            logger.debug("Simulated Arduino reset")
            self._pwm_values = dict.fromkeys(range(self._arduino_config.pwm_channels), 0)
            self._analog_values.clear()
            self._simulated_analog.clear()
            self._simulated_digital.clear()
            return

        if self._serial is None:
            raise CommunicationError("Serial port not connected")

        with self._serial_lock:
            # Toggle DTR to reset Arduino
            self._serial.dtr = False
            time.sleep(0.1)
            self._serial.dtr = True

            # Wait for Arduino to reset
            if self._arduino_config.wait_for_ready:
                self._wait_for_ready()

    def get_diagnostics(self) -> dict[str, Any]:
        """Get driver diagnostics.

        Returns:
            Dictionary of diagnostic information.
        """
        return {
            "name": self._name,
            "port": self._arduino_config.serial.port,
            "baudrate": self._arduino_config.serial.baudrate,
            "protocol": self._arduino_config.protocol.value,
            "state": self._state.value,
            "simulation": self._simulation_mode,
            "pwm_channels": self._arduino_config.pwm_channels,
            "analog_channels": self._arduino_config.analog_channels,
            "digital_pins": self._arduino_config.digital_pins,
            "pwm_values": dict(self._pwm_values),
            "analog_values": dict(self._analog_values),
            "connected": self.is_connected,
        }

    def __repr__(self) -> str:
        """Return string representation."""
        return (
            f"ArduinoDriver("
            f"port={self._arduino_config.serial.port!r}, "
            f"baudrate={self._arduino_config.serial.baudrate}, "
            f"protocol={self._arduino_config.protocol.value}, "
            f"simulation={self._simulation_mode})"
        )


# =============================================================================
# Factory Functions
# =============================================================================


def get_arduino_driver(
    port: str | None = None,
    baudrate: int = DEFAULT_BAUDRATE,
    simulation: bool = False,
) -> ArduinoDriver:
    """Create an Arduino driver with auto-detection.

    Args:
        port: Serial port path. If None, will try to auto-detect.
        baudrate: Serial baud rate.
        simulation: Force simulation mode.

    Returns:
        Configured ArduinoDriver instance.
    """
    config = ArduinoConfig()
    if port is not None:
        config.serial.port = port
    config.serial.baudrate = baudrate
    config.simulation = simulation

    return ArduinoDriver(config=config)


def list_arduino_ports() -> list[dict[str, Any]]:
    """List available serial ports that might be Arduino.

    Returns:
        List of port information dictionaries.
    """
    try:
        from serial.tools import list_ports

        ports = []
        arduino_vids = [0x2341, 0x2A03, 0x1A86]

        for port in list_ports.comports():
            is_arduino = port.vid in arduino_vids if port.vid else False
            ports.append(
                {
                    "device": port.device,
                    "description": port.description,
                    "hwid": port.hwid,
                    "vid": port.vid,
                    "pid": port.pid,
                    "serial_number": port.serial_number,
                    "is_arduino": is_arduino,
                }
            )

        return ports

    except ImportError:
        logger.warning("pyserial not installed, cannot list ports")
        return []
