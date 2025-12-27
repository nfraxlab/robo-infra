"""Arduino platform implementation via Firmata protocol.

Supports Arduino boards connected via USB using the Firmata protocol:
- Arduino Uno, Nano, Mini (ATmega328P)
- Arduino Mega 2560
- Arduino Leonardo, Micro (ATmega32U4)
- Arduino Due (SAM3X8E)
- Arduino Zero, MKR series (SAMD21)

This module uses pyfirmata2 for communication, which is a more actively
maintained fork of the original pyfirmata library.

Example:
    >>> from robo_infra.platforms.arduino import ArduinoPlatform
    >>>
    >>> # Auto-detect Arduino
    >>> platform = ArduinoPlatform()
    >>> print(f"Connected to: {platform.port}")
    >>>
    >>> # Digital output (LED on pin 13)
    >>> led = platform.get_pin(13, mode=PinMode.OUTPUT)
    >>> led.high()
    >>>
    >>> # Analog input
    >>> sensor = platform.get_pin("A0")
    >>> print(f"Value: {sensor.read()}")
    >>>
    >>> # PWM output
    >>> motor = platform.get_pin(9, mode=PinMode.PWM)
    >>> motor.write(0.5)  # 50% duty cycle
    >>>
    >>> # Servo control
    >>> servo = platform.get_servo(10)
    >>> servo.write(90)  # Move to 90 degrees
    >>>
    >>> platform.cleanup()
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from robo_infra.core.exceptions import HardwareNotFoundError
from robo_infra.core.pin import AnalogPin, DigitalPin, PinMode, PinState, PWMPin
from robo_infra.platforms.base import (
    BasePlatform,
    PlatformCapability,
    PlatformConfig,
    PlatformInfo,
    PlatformType,
)


if TYPE_CHECKING:
    from robo_infra.core.bus import Bus
    from robo_infra.core.pin import Pin


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================


class ArduinoBoard(Enum):
    """Supported Arduino board types."""

    UNO = "uno"
    NANO = "nano"
    MINI = "mini"
    MEGA = "mega"
    MEGA_ADK = "mega_adk"
    LEONARDO = "leonardo"
    MICRO = "micro"
    DUE = "due"
    ZERO = "zero"
    MKR1000 = "mkr1000"
    UNKNOWN = "unknown"


class FirmataCommand(Enum):
    """Firmata protocol commands."""

    DIGITAL_MESSAGE = 0x90
    ANALOG_MESSAGE = 0xE0
    REPORT_ANALOG = 0xC0
    REPORT_DIGITAL = 0xD0
    SET_PIN_MODE = 0xF4
    REPORT_VERSION = 0xF9
    SYSTEM_RESET = 0xFF
    START_SYSEX = 0xF0
    END_SYSEX = 0xF7


# Arduino USB Vendor/Product IDs for auto-detection
ARDUINO_USB_IDS = {
    # Official Arduino boards
    (0x2341, 0x0043): ArduinoBoard.UNO,  # Arduino Uno
    (0x2341, 0x0001): ArduinoBoard.UNO,  # Arduino Uno (older)
    (0x2341, 0x0010): ArduinoBoard.MEGA,  # Arduino Mega 2560
    (0x2341, 0x0042): ArduinoBoard.MEGA,  # Arduino Mega 2560 R3
    (0x2341, 0x8036): ArduinoBoard.LEONARDO,  # Arduino Leonardo
    (0x2341, 0x003D): ArduinoBoard.DUE,  # Arduino Due (prog)
    (0x2341, 0x003E): ArduinoBoard.DUE,  # Arduino Due (native)
    (0x2341, 0x804D): ArduinoBoard.ZERO,  # Arduino Zero
    # Arduino.cc boards
    (0x2A03, 0x0043): ArduinoBoard.UNO,  # Arduino Uno (arduino.cc)
    # Clone boards (CH340/CH341 USB-Serial)
    (0x1A86, 0x7523): ArduinoBoard.UNKNOWN,  # Generic CH340
    # FTDI-based boards
    (0x0403, 0x6001): ArduinoBoard.UNKNOWN,  # FTDI FT232RL
}


# Pin capabilities by board type
BOARD_CAPABILITIES = {
    ArduinoBoard.UNO: {
        "digital_pins": list(range(14)),
        "analog_pins": list(range(6)),  # A0-A5
        "pwm_pins": [3, 5, 6, 9, 10, 11],
        "adc_resolution": 10,
        "pwm_resolution": 8,
    },
    ArduinoBoard.NANO: {
        "digital_pins": list(range(14)),
        "analog_pins": list(range(8)),  # A0-A7
        "pwm_pins": [3, 5, 6, 9, 10, 11],
        "adc_resolution": 10,
        "pwm_resolution": 8,
    },
    ArduinoBoard.MEGA: {
        "digital_pins": list(range(54)),
        "analog_pins": list(range(16)),  # A0-A15
        "pwm_pins": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46],
        "adc_resolution": 10,
        "pwm_resolution": 8,
    },
    ArduinoBoard.LEONARDO: {
        "digital_pins": list(range(20)),
        "analog_pins": list(range(12)),  # A0-A11
        "pwm_pins": [3, 5, 6, 9, 10, 11, 13],
        "adc_resolution": 10,
        "pwm_resolution": 8,
    },
    ArduinoBoard.DUE: {
        "digital_pins": list(range(54)),
        "analog_pins": list(range(12)),  # A0-A11
        "pwm_pins": list(range(2, 14)),  # 2-13
        "adc_resolution": 12,
        "pwm_resolution": 8,
        "dac_pins": [0, 1],  # DAC0, DAC1
    },
}


# Default capabilities for unknown boards
DEFAULT_CAPABILITIES = {
    "digital_pins": list(range(14)),
    "analog_pins": list(range(6)),
    "pwm_pins": [3, 5, 6, 9, 10, 11],
    "adc_resolution": 10,
    "pwm_resolution": 8,
}


# =============================================================================
# Arduino Pin Classes
# =============================================================================


@dataclass
class ArduinoPinConfig:
    """Configuration for an Arduino pin."""

    pin: int | str  # Digital pin number or "A0"-"A5" for analog
    mode: str = "input"  # input, output, pwm, servo
    pullup: bool = False


class ArduinoDigitalPin(DigitalPin):
    """Digital GPIO pin for Arduino via Firmata."""

    def __init__(
        self,
        number: int,
        mode: PinMode = PinMode.OUTPUT,
        *,
        name: str | None = None,
        inverted: bool = False,
        initial: PinState = PinState.LOW,
        board: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize an Arduino digital pin.

        Args:
            number: Digital pin number (0-13 for Uno)
            mode: Pin mode (INPUT, OUTPUT, INPUT_PULLUP)
            name: Optional human-readable name
            inverted: Invert logic
            initial: Initial state for output pins
            board: pyfirmata2 board instance
            simulation: Run in simulation mode
        """
        super().__init__(number, mode, name=name, inverted=inverted, initial=initial)
        self._board = board
        self._simulation = simulation
        self._pin_obj: Any = None

    def setup(self) -> None:
        """Initialize the pin."""
        if self._initialized:
            return

        if self._simulation:
            self._setup_simulation()
        else:
            self._setup_firmata()

        self._initialized = True
        logger.debug("Initialized Arduino pin %d as %s", self._number, self._mode.value)

    def _setup_firmata(self) -> None:
        """Setup using pyfirmata2."""
        if self._board is None:
            raise HardwareNotFoundError(
                device="Arduino board",
                details="Board not connected",
            )

        # Get pin from board
        pin_str = f"d:{self._number}:"
        if self._mode == PinMode.OUTPUT:
            pin_str += "o"
        elif self._mode == PinMode.INPUT_PULLUP:
            pin_str += "u"
        else:
            pin_str += "i"

        self._pin_obj = self._board.get_pin(pin_str)

        # Set initial state for output pins
        if self._mode == PinMode.OUTPUT and self._initial == PinState.HIGH:
            self._pin_obj.write(1)

    def _setup_simulation(self) -> None:
        """Setup in simulation mode."""
        self._pin_obj = {
            "simulated": True,
            "value": self._initial == PinState.HIGH,
        }

    def read(self) -> bool:
        """Read the pin state."""
        if not self._initialized:
            self.setup()

        if self._simulation:
            value = bool(self._pin_obj.get("value", False))
        else:
            raw = self._pin_obj.read()
            value = bool(raw) if raw is not None else False

        if self._inverted:
            value = not value

        self._state = PinState.HIGH if value else PinState.LOW
        return value

    def write(self, value: bool) -> None:
        """Write to the pin."""
        if not self._initialized:
            self.setup()

        if self._inverted:
            value = not value

        if self._simulation:
            self._pin_obj["value"] = value
        else:
            self._pin_obj.write(1 if value else 0)

        self._state = PinState.HIGH if value else PinState.LOW

    def cleanup(self) -> None:
        """Release pin resources."""
        if not self._initialized:
            return

        try:
            if not self._simulation and self._pin_obj is not None:
                # Set to input to release
                self._pin_obj.mode = 0
        except Exception as e:
            logger.warning("Error cleaning up Arduino pin %d: %s", self._number, e)
        finally:
            self._initialized = False
            self._pin_obj = None


class ArduinoPWMPin(PWMPin):
    """PWM pin for Arduino via Firmata."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        frequency: int = 490,  # Default Arduino PWM frequency
        duty_cycle: float = 0.0,
        board: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize a PWM pin.

        Note: Arduino PWM frequency is fixed per timer. This frequency
        parameter is informational only - actual frequency depends on board.
        """
        super().__init__(number, name=name, frequency=frequency, duty_cycle=duty_cycle)
        self._board = board
        self._simulation = simulation
        self._pin_obj: Any = None

    def setup(self) -> None:
        """Initialize PWM output."""
        if self._initialized:
            return

        if self._simulation:
            self._setup_simulation()
        else:
            self._setup_firmata()

        self._initialized = True

    def _setup_firmata(self) -> None:
        """Setup PWM using pyfirmata2."""
        if self._board is None:
            raise HardwareNotFoundError(
                device="Arduino board",
                details="Board not connected",
            )

        self._pin_obj = self._board.get_pin(f"d:{self._number}:p")
        self._pin_obj.write(self._duty_cycle)

    def _setup_simulation(self) -> None:
        """Setup simulated PWM."""
        self._pin_obj = {
            "simulated": True,
            "duty_cycle": self._duty_cycle,
            "running": True,
        }

    @property
    def frequency(self) -> int:
        """Get PWM frequency (informational - Arduino PWM is fixed)."""
        return self._frequency

    @frequency.setter
    def frequency(self, value: int) -> None:
        """Set PWM frequency (informational only)."""
        self._frequency = value
        # Note: Arduino PWM frequency is fixed, can't change at runtime

    @property
    def duty_cycle(self) -> float:
        """Get duty cycle (0.0-1.0)."""
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        """Set duty cycle (0.0-1.0)."""
        self._duty_cycle = max(0.0, min(1.0, value))
        if self._initialized:
            if self._simulation:
                self._pin_obj["duty_cycle"] = self._duty_cycle
            else:
                self._pin_obj.write(self._duty_cycle)

    def set_duty_cycle(self, duty: float) -> None:
        """Set the PWM duty cycle (0.0-1.0)."""
        self.duty_cycle = duty

    def set_frequency(self, frequency: int) -> None:
        """Set the PWM frequency (informational only for Arduino)."""
        self.frequency = frequency

    def set_pulse_width(self, width_us: float) -> None:
        """Set PWM by pulse width in microseconds."""
        period_us = 1_000_000 / self._frequency
        self.duty_cycle = width_us / period_us

    def start(self) -> None:
        """Start PWM output."""
        if not self._initialized:
            self.setup()

    def stop(self) -> None:
        """Stop PWM output."""
        self.duty_cycle = 0.0

    def cleanup(self) -> None:
        """Release pin resources."""
        if not self._initialized:
            return

        try:
            if not self._simulation and self._pin_obj is not None:
                self._pin_obj.write(0)
        except Exception as e:
            logger.warning("Error cleaning up Arduino PWM pin %d: %s", self._number, e)
        finally:
            self._initialized = False
            self._pin_obj = None


class ArduinoAnalogPin(AnalogPin):
    """Analog input pin for Arduino via Firmata."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        resolution: int = 10,
        reference_voltage: float = 5.0,
        board: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize an analog input pin.

        Args:
            number: Analog pin number (0-5 for Uno = A0-A5)
            name: Optional name
            resolution: ADC resolution in bits
            reference_voltage: ADC reference voltage
            board: pyfirmata2 board instance
            simulation: Run in simulation mode
        """
        super().__init__(
            number, name=name, resolution=resolution, reference_voltage=reference_voltage
        )
        self._board = board
        self._simulation = simulation
        self._pin_obj: Any = None

    def setup(self) -> None:
        """Initialize analog input."""
        if self._initialized:
            return

        if self._simulation:
            self._setup_simulation()
        else:
            self._setup_firmata()

        self._initialized = True

    def _setup_firmata(self) -> None:
        """Setup using pyfirmata2."""
        if self._board is None:
            raise HardwareNotFoundError(
                device="Arduino board",
                details="Board not connected",
            )

        self._pin_obj = self._board.get_pin(f"a:{self._number}:i")
        # Enable analog reporting
        self._pin_obj.enable_reporting()

    def _setup_simulation(self) -> None:
        """Setup simulated analog input."""
        self._pin_obj = {
            "simulated": True,
            "value": 0.5,  # Mid-range
        }

    def read_raw(self) -> int:
        """Read raw ADC value."""
        if not self._initialized:
            self.setup()

        if self._simulation:
            normalized = self._pin_obj.get("value", 0.5)
        else:
            # Firmata returns 0.0-1.0
            normalized = self._pin_obj.read()
            if normalized is None:
                normalized = 0.0

        return int(normalized * self._max_value)

    def cleanup(self) -> None:
        """Release pin resources."""
        if not self._initialized:
            return

        try:
            if not self._simulation and self._pin_obj is not None:
                self._pin_obj.disable_reporting()
        except Exception as e:
            logger.warning("Error cleaning up Arduino analog pin %d: %s", self._number, e)
        finally:
            self._initialized = False
            self._pin_obj = None


class ArduinoServoPin:
    """Servo control pin for Arduino via Firmata."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        min_pulse: int = 544,
        max_pulse: int = 2400,
        board: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize a servo pin.

        Args:
            number: Digital pin number
            name: Optional name
            min_pulse: Minimum pulse width in microseconds
            max_pulse: Maximum pulse width in microseconds
            board: pyfirmata2 board instance
            simulation: Run in simulation mode
        """
        self._number = number
        self._name = name or f"servo-{number}"
        self._min_pulse = min_pulse
        self._max_pulse = max_pulse
        self._board = board
        self._simulation = simulation
        self._pin_obj: Any = None
        self._angle: float = 90.0
        self._initialized = False

    @property
    def number(self) -> int:
        """Get pin number."""
        return self._number

    @property
    def name(self) -> str:
        """Get pin name."""
        return self._name

    @property
    def angle(self) -> float:
        """Get current angle (0-180 degrees)."""
        return self._angle

    @property
    def initialized(self) -> bool:
        """Check if initialized."""
        return self._initialized

    def setup(self) -> None:
        """Initialize servo control."""
        if self._initialized:
            return

        if self._simulation:
            self._pin_obj = {"simulated": True, "angle": 90.0}
        else:
            if self._board is None:
                raise HardwareNotFoundError(
                    device="Arduino board",
                    details="Board not connected",
                )
            self._pin_obj = self._board.get_pin(f"d:{self._number}:s")

        self._initialized = True

    def write(self, angle: float) -> None:
        """Set servo angle (0-180 degrees)."""
        if not self._initialized:
            self.setup()

        self._angle = max(0.0, min(180.0, angle))

        if self._simulation:
            self._pin_obj["angle"] = self._angle
        else:
            self._pin_obj.write(self._angle)

    def write_microseconds(self, us: int) -> None:
        """Set servo pulse width in microseconds."""
        # Convert to angle
        us = max(self._min_pulse, min(self._max_pulse, us))
        angle = (us - self._min_pulse) / (self._max_pulse - self._min_pulse) * 180.0
        self.write(angle)

    def detach(self) -> None:
        """Detach servo (stop sending pulses)."""
        if not self._initialized:
            return

        try:
            if not self._simulation and self._pin_obj is not None:
                # Set to input mode to detach
                self._pin_obj.mode = 0
        except Exception as e:
            logger.warning("Error detaching servo on pin %d: %s", self._number, e)

    def cleanup(self) -> None:
        """Release servo resources."""
        self.detach()
        self._initialized = False
        self._pin_obj = None


# =============================================================================
# Arduino Platform
# =============================================================================


class ArduinoPlatform(BasePlatform):
    """Arduino platform via Firmata protocol.

    Connects to Arduino boards over USB serial using the StandardFirmata
    protocol for GPIO, analog, PWM, and servo control.

    Example:
        >>> platform = ArduinoPlatform()  # Auto-detect
        >>> platform = ArduinoPlatform(port="/dev/ttyUSB0")  # Specific port
        >>>
        >>> # Digital I/O
        >>> led = platform.get_pin(13, mode=PinMode.OUTPUT)
        >>> led.high()
        >>>
        >>> # Analog input
        >>> sensor = platform.get_analog_pin(0)  # A0
        >>> value = sensor.read()
        >>>
        >>> # PWM
        >>> motor = platform.get_pin(9, mode=PinMode.PWM)
        >>> motor.write(0.5)
        >>>
        >>> # Servo
        >>> servo = platform.get_servo(10)
        >>> servo.write(90)
        >>>
        >>> platform.cleanup()
    """

    def __init__(
        self,
        config: PlatformConfig | None = None,
        *,
        port: str | None = None,
        baudrate: int = 57600,
        board_type: ArduinoBoard | None = None,
    ) -> None:
        """Initialize Arduino platform.

        Args:
            config: Platform configuration
            port: Serial port (e.g., "/dev/ttyUSB0", "COM3"). Auto-detect if None.
            baudrate: Serial baudrate (default 57600 for Firmata)
            board_type: Board type hint. Auto-detect if None.
        """
        if config is None:
            config = PlatformConfig(
                name="Arduino",
                platform_type=PlatformType.ARDUINO,
            )

        super().__init__(config)

        self._port = port
        self._baudrate = baudrate
        self._board_type = board_type
        self._board: Any = None
        self._simulation = self._check_simulation()
        self._servos: dict[int, ArduinoServoPin] = {}

        if not self._simulation:
            self._connect()
        # In simulation mode, default to UNO if no board type specified
        elif self._board_type is None:
            self._board_type = ArduinoBoard.UNO

        logger.info(
            "Arduino platform initialized (port=%s, simulation=%s)",
            self._port,
            self._simulation,
        )

    def _check_simulation(self) -> bool:
        """Check if running in simulation mode."""
        if os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes"):
            return True

        # Check if pyfirmata2 is available
        try:
            import pyfirmata2  # noqa: F401

            return False
        except ImportError:
            return True

    def _connect(self) -> None:
        """Connect to Arduino board."""
        try:
            from pyfirmata2 import Arduino

            port = self._port or self._find_arduino_port()
            if port is None:
                raise HardwareNotFoundError(
                    device="Arduino",
                    details="No Arduino found. Check USB connection.",
                )

            self._board = Arduino(port)
            self._port = port

            # Start iterator for analog readings
            self._board.samplingOn()

            # Give board time to initialize
            time.sleep(0.5)

            # Detect board type if not specified
            if self._board_type is None:
                self._board_type = self._detect_board_type()

        except ImportError:
            raise HardwareNotFoundError(
                device="pyfirmata2",
                details="Install with: pip install pyfirmata2",
            ) from None
        except Exception as e:
            raise HardwareNotFoundError(
                device="Arduino",
                details=str(e),
            ) from e

    def _find_arduino_port(self) -> str | None:
        """Auto-detect Arduino serial port."""
        try:
            import serial.tools.list_ports

            for port in serial.tools.list_ports.comports():
                # Check VID/PID
                if (
                    port.vid is not None
                    and port.pid is not None
                    and (port.vid, port.pid) in ARDUINO_USB_IDS
                ):
                    return port.device

                # Check description for Arduino
                if port.description and "arduino" in port.description.lower():
                    return port.device

                # Common USB-Serial chips used by Arduinos
                if port.vid in (0x2341, 0x2A03, 0x1A86, 0x0403):
                    return port.device

        except ImportError:
            pass

        return None

    def _detect_board_type(self) -> ArduinoBoard:
        """Detect Arduino board type."""
        if self._simulation:
            return ArduinoBoard.UNO

        try:
            import serial.tools.list_ports

            for port in serial.tools.list_ports.comports():
                if port.device == self._port and port.vid is not None and port.pid is not None:
                    board = ARDUINO_USB_IDS.get((port.vid, port.pid))
                    if board is not None:
                        return board
        except ImportError:
            pass

        return ArduinoBoard.UNKNOWN

    @property
    def port(self) -> str | None:
        """Get the serial port."""
        return self._port

    @property
    def board_type(self) -> ArduinoBoard:
        """Get the detected board type."""
        return self._board_type or ArduinoBoard.UNKNOWN

    @property
    def is_available(self) -> bool:
        """Check if Arduino is available."""
        if self._simulation:
            return True
        return self._board is not None

    def get_board_capabilities(self) -> dict[str, Any]:
        """Get capabilities for current board type."""
        return BOARD_CAPABILITIES.get(self.board_type, DEFAULT_CAPABILITIES)

    def get_analog_pin(self, pin: int, **kwargs: Any) -> ArduinoAnalogPin:
        """Get an analog input pin.

        Args:
            pin: Analog pin number (0 for A0, 5 for A5, etc.)
            **kwargs: Additional pin options
        """
        capabilities = self.get_board_capabilities()
        resolution = capabilities.get("adc_resolution", 10)

        analog_pin = ArduinoAnalogPin(
            pin,
            name=kwargs.get("name", f"A{pin}"),
            resolution=resolution,
            reference_voltage=kwargs.get("reference_voltage", 5.0),
            board=self._board,
            simulation=self._simulation,
        )
        return analog_pin

    def get_servo(self, pin: int, **kwargs: Any) -> ArduinoServoPin:
        """Get a servo control pin.

        Args:
            pin: Digital pin number for servo
            **kwargs: min_pulse, max_pulse options
        """
        if pin in self._servos:
            return self._servos[pin]

        servo = ArduinoServoPin(
            pin,
            name=kwargs.get("name"),
            min_pulse=kwargs.get("min_pulse", 544),
            max_pulse=kwargs.get("max_pulse", 2400),
            board=self._board,
            simulation=self._simulation,
        )
        self._servos[pin] = servo
        return servo

    def _detect_info(self) -> PlatformInfo:
        """Detect platform information."""
        capabilities: set[PlatformCapability] = {
            PlatformCapability.GPIO,
            PlatformCapability.PWM,
        }

        board_caps = self.get_board_capabilities()

        if board_caps.get("analog_pins"):
            capabilities.add(PlatformCapability.I2C)  # Analog available

        return PlatformInfo(
            platform_type=PlatformType.ARDUINO,
            model=self.board_type.value,
            revision="",
            serial="",
            capabilities=capabilities,
            gpio_chips=[],
            i2c_buses=[],
            spi_buses=[],
            uart_ports=[self._port] if self._port else [],
        )

    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create an Arduino pin."""
        # Parse pin ID
        if isinstance(pin_id, str):
            if pin_id.startswith("A") or pin_id.startswith("a"):
                # Analog pin
                analog_num = int(pin_id[1:])
                return self.get_analog_pin(analog_num, **kwargs)
            else:
                pin_num = int(pin_id)
        else:
            pin_num = pin_id

        mode = kwargs.get("mode", PinMode.OUTPUT)

        if mode == PinMode.PWM:
            return ArduinoPWMPin(
                pin_num,
                name=kwargs.get("name"),
                duty_cycle=kwargs.get("duty_cycle", 0.0),
                board=self._board,
                simulation=self._simulation,
            )
        elif mode == PinMode.ANALOG:
            return self.get_analog_pin(pin_num, **kwargs)
        else:
            initial = kwargs.get("initial", PinState.LOW)
            if isinstance(initial, bool):
                initial = PinState.HIGH if initial else PinState.LOW

            return ArduinoDigitalPin(
                pin_num,
                mode=mode,
                name=kwargs.get("name"),
                inverted=kwargs.get("inverted", False),
                initial=initial,
                board=self._board,
                simulation=self._simulation,
            )

    def _create_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Create a communication bus (limited support on Arduino)."""
        raise HardwareNotFoundError(
            device=f"Bus type: {bus_type}",
            details="Arduino Firmata has limited bus support. Use I2C/SPI libraries on Arduino side.",
        )

    def cleanup(self) -> None:
        """Cleanup all resources."""
        # Cleanup servos
        for servo in self._servos.values():
            servo.cleanup()
        self._servos.clear()

        # Cleanup pins from parent
        super().cleanup()

        # Disconnect from board
        if self._board is not None and not self._simulation:
            try:
                self._board.exit()
            except Exception as e:
                logger.warning("Error closing Arduino connection: %s", e)
            self._board = None


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Constants
    "ARDUINO_USB_IDS",
    "BOARD_CAPABILITIES",
    "DEFAULT_CAPABILITIES",
    # Pin classes
    "ArduinoAnalogPin",
    # Enums
    "ArduinoBoard",
    "ArduinoDigitalPin",
    "ArduinoPWMPin",
    # Platform
    "ArduinoPlatform",
    "ArduinoServoPin",
    "FirmataCommand",
]
