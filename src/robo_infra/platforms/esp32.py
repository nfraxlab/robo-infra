"""ESP32/ESP8266 platform implementation via MicroPython REPL.

Supports ESP32 and ESP8266 boards running MicroPython:
- ESP32 (Espressif)
- ESP32-S2, ESP32-S3, ESP32-C3
- ESP8266 (NodeMCU, Wemos D1)

Communication modes:
- USB Serial (default)
- WiFi WebREPL
- BLE (ESP32 only)

Example:
    >>> from robo_infra.platforms.esp32 import ESP32Platform
    >>>
    >>> # USB Serial connection
    >>> platform = ESP32Platform(port="/dev/ttyUSB0")
    >>>
    >>> # WiFi WebREPL connection
    >>> platform = ESP32Platform(
    ...     host="192.168.1.100",
    ...     webrepl_password="secret"
    ... )
    >>>
    >>> # Digital output
    >>> led = platform.get_pin(2, mode=PinMode.OUTPUT)
    >>> led.high()
    >>>
    >>> # Touch pin (ESP32 only)
    >>> touch = platform.get_touch_pin(4)
    >>> print(f"Touch value: {touch.read()}")
    >>>
    >>> # DAC output (ESP32 only)
    >>> dac = platform.get_dac_pin(25)
    >>> dac.write(128)  # 0-255
    >>>
    >>> platform.cleanup()
"""

from __future__ import annotations

import contextlib
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


class ESP32Chip(Enum):
    """ESP32 chip variants."""

    ESP8266 = "esp8266"
    ESP32 = "esp32"
    ESP32_S2 = "esp32-s2"
    ESP32_S3 = "esp32-s3"
    ESP32_C3 = "esp32-c3"
    UNKNOWN = "unknown"


class ESP32ConnectionMode(Enum):
    """Connection mode to ESP device."""

    SERIAL = "serial"
    WEBREPL = "webrepl"
    BLE = "ble"


# ESP32 Pin capabilities
ESP32_CAPABILITIES = {
    ESP32Chip.ESP32: {
        "gpio_pins": list(range(40)),  # GPIO0-39
        "adc_pins": [32, 33, 34, 35, 36, 37, 38, 39],  # ADC1: 32-39
        "adc2_pins": [0, 2, 4, 12, 13, 14, 15, 25, 26, 27],  # ADC2
        "dac_pins": [25, 26],  # DAC1, DAC2
        "touch_pins": [0, 2, 4, 12, 13, 14, 15, 27, 32, 33],  # Touch0-9
        "pwm_channels": 16,
        "adc_resolution": 12,
        "dac_resolution": 8,
        "i2c_buses": 2,
        "spi_buses": 2,
        "uart_ports": 3,
    },
    ESP32Chip.ESP32_S2: {
        "gpio_pins": list(range(46)),
        "adc_pins": list(range(1, 11)),
        "dac_pins": [17, 18],
        "touch_pins": list(range(1, 15)),
        "pwm_channels": 8,
        "adc_resolution": 13,
        "dac_resolution": 8,
        "i2c_buses": 2,
        "spi_buses": 2,
        "uart_ports": 2,
    },
    ESP32Chip.ESP32_S3: {
        "gpio_pins": list(range(49)),
        "adc_pins": list(range(1, 11)),
        "dac_pins": [],  # S3 has no DAC
        "touch_pins": list(range(1, 15)),
        "pwm_channels": 8,
        "adc_resolution": 12,
        "dac_resolution": 0,
        "i2c_buses": 2,
        "spi_buses": 2,
        "uart_ports": 3,
    },
    ESP32Chip.ESP32_C3: {
        "gpio_pins": list(range(22)),
        "adc_pins": [0, 1, 2, 3, 4],
        "dac_pins": [],  # C3 has no DAC
        "touch_pins": [],  # C3 has no touch
        "pwm_channels": 6,
        "adc_resolution": 12,
        "dac_resolution": 0,
        "i2c_buses": 1,
        "spi_buses": 2,
        "uart_ports": 2,
    },
    ESP32Chip.ESP8266: {
        "gpio_pins": [0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16],
        "adc_pins": [0],  # ADC0 only
        "dac_pins": [],
        "touch_pins": [],
        "pwm_channels": 8,
        "adc_resolution": 10,
        "dac_resolution": 0,
        "i2c_buses": 1,
        "spi_buses": 1,
        "uart_ports": 2,
    },
}

DEFAULT_ESP_CAPABILITIES = ESP32_CAPABILITIES[ESP32Chip.ESP32]

# MicroPython REPL control characters
REPL_CTRL_A = b"\x01"  # Enter raw REPL mode
REPL_CTRL_B = b"\x02"  # Exit raw REPL mode
REPL_CTRL_C = b"\x03"  # Interrupt
REPL_CTRL_D = b"\x04"  # Soft reset / end raw paste


# =============================================================================
# ESP32 Pin Classes
# =============================================================================


@dataclass
class ESP32PinConfig:
    """Configuration for an ESP32 pin."""

    pin: int
    mode: str = "input"  # input, output, pwm, adc, dac, touch
    pull: str | None = None  # up, down, None


class ESP32DigitalPin(DigitalPin):
    """Digital GPIO pin for ESP32 via MicroPython REPL."""

    def __init__(
        self,
        number: int,
        mode: PinMode = PinMode.OUTPUT,
        *,
        name: str | None = None,
        inverted: bool = False,
        initial: PinState = PinState.LOW,
        repl: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize an ESP32 digital pin.

        Args:
            number: GPIO pin number
            mode: Pin mode (INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN)
            name: Optional human-readable name
            inverted: Invert logic
            initial: Initial state for output pins
            repl: MicroPython REPL connection
            simulation: Run in simulation mode
        """
        super().__init__(number, mode, name=name, inverted=inverted, initial=initial)
        self._repl = repl
        self._simulation = simulation

    def setup(self) -> None:
        """Initialize the pin."""
        if self._initialized:
            return

        if self._simulation:
            self._setup_simulation()
        else:
            self._setup_repl()

        self._initialized = True
        logger.debug("Initialized ESP32 pin %d as %s", self._number, self._mode.value)

    def _setup_simulation(self) -> None:
        """Setup in simulation mode."""
        self._sim_value = self._initial == PinState.HIGH

    def _setup_repl(self) -> None:
        """Setup using MicroPython REPL."""
        if self._repl is None:
            raise HardwareNotFoundError(
                device="ESP32",
                details="REPL connection not established",
            )

        # Determine mode and pull
        mode_str = "Pin.OUT" if self._mode == PinMode.OUTPUT else "Pin.IN"
        pull_str = ""
        if self._mode == PinMode.INPUT_PULLUP:
            pull_str = ", Pin.PULL_UP"
        elif self._mode == PinMode.INPUT_PULLDOWN:
            pull_str = ", Pin.PULL_DOWN"

        # Create pin on device
        cmd = f"from machine import Pin; p{self._number}=Pin({self._number},{mode_str}{pull_str})"
        self._repl.exec(cmd)

        # Set initial value for output
        if self._mode == PinMode.OUTPUT:
            val = 1 if self._initial == PinState.HIGH else 0
            self._repl.exec(f"p{self._number}.value({val})")

    def read(self) -> bool:
        """Read the pin state."""
        if not self._initialized:
            self.setup()

        if self._simulation:
            value = self._sim_value
        else:
            result = self._repl.eval(f"p{self._number}.value()")
            value = bool(int(result))

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
            self._sim_value = value
        else:
            self._repl.exec(f"p{self._number}.value({1 if value else 0})")

        self._state = PinState.HIGH if value else PinState.LOW

    def cleanup(self) -> None:
        """Release pin resources."""
        if not self._initialized:
            return
        self._initialized = False


class ESP32PWMPin(PWMPin):
    """PWM pin for ESP32 via MicroPython REPL."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        frequency: int = 1000,
        duty_cycle: float = 0.0,
        resolution: int = 10,  # 0-1023
        repl: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize a PWM pin.

        Args:
            number: GPIO pin number
            name: Optional name
            frequency: PWM frequency in Hz (1-40000000 for ESP32)
            duty_cycle: Initial duty cycle (0.0-1.0)
            resolution: Duty cycle resolution in bits
            repl: MicroPython REPL connection
            simulation: Run in simulation mode
        """
        super().__init__(number, name=name, frequency=frequency, duty_cycle=duty_cycle)
        self._resolution = resolution
        self._max_duty = (1 << resolution) - 1
        self._repl = repl
        self._simulation = simulation

    def setup(self) -> None:
        """Initialize PWM output."""
        if self._initialized:
            return

        if self._simulation:
            self._sim_duty = self._duty_cycle
            self._sim_running = True
        else:
            if self._repl is None:
                raise HardwareNotFoundError(
                    device="ESP32",
                    details="REPL connection not established",
                )

            cmd = f"from machine import Pin,PWM; pwm{self._number}=PWM(Pin({self._number}),freq={self._frequency})"
            self._repl.exec(cmd)
            self._set_duty_hardware(self._duty_cycle)

        self._initialized = True

    def _set_duty_hardware(self, duty: float) -> None:
        """Set duty cycle on hardware."""
        duty_val = int(duty * self._max_duty)
        self._repl.exec(f"pwm{self._number}.duty({duty_val})")

    @property
    def frequency(self) -> int:
        """Get PWM frequency."""
        return self._frequency

    @frequency.setter
    def frequency(self, value: int) -> None:
        """Set PWM frequency."""
        self._frequency = max(1, min(40000000, value))
        if self._initialized and not self._simulation:
            self._repl.exec(f"pwm{self._number}.freq({self._frequency})")

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
                self._sim_duty = self._duty_cycle
            else:
                self._set_duty_hardware(self._duty_cycle)

    def set_duty_cycle(self, duty: float) -> None:
        """Set the PWM duty cycle (0.0-1.0)."""
        self.duty_cycle = duty

    def set_frequency(self, frequency: int) -> None:
        """Set the PWM frequency in Hz."""
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
        """Stop PWM output (set duty to 0)."""
        self.duty_cycle = 0.0

    def deinit(self) -> None:
        """Deinitialize PWM."""
        if self._initialized and not self._simulation and self._repl is not None:
            try:
                self._repl.exec(f"pwm{self._number}.deinit()")
            except Exception as e:
                logger.warning("Error deinitializing PWM on pin %d: %s", self._number, e)

    def cleanup(self) -> None:
        """Release pin resources."""
        self.deinit()
        self._initialized = False


class ESP32AnalogPin(AnalogPin):
    """Analog input pin for ESP32 via MicroPython REPL."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        resolution: int = 12,
        reference_voltage: float = 3.3,
        attenuation: int = 3,  # 0=0dB, 1=2.5dB, 2=6dB, 3=11dB
        repl: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize an analog input pin.

        Args:
            number: GPIO pin number (must be ADC-capable)
            name: Optional name
            resolution: ADC resolution in bits (9-12 for ESP32)
            reference_voltage: Reference voltage
            attenuation: ADC attenuation (0-3)
            repl: MicroPython REPL connection
            simulation: Run in simulation mode
        """
        super().__init__(
            number, name=name, resolution=resolution, reference_voltage=reference_voltage
        )
        self._attenuation = attenuation
        self._repl = repl
        self._simulation = simulation

    def setup(self) -> None:
        """Initialize analog input."""
        if self._initialized:
            return

        if self._simulation:
            self._sim_value = 0.5
        else:
            if self._repl is None:
                raise HardwareNotFoundError(
                    device="ESP32",
                    details="REPL connection not established",
                )

            # Create ADC on device
            cmd = f"from machine import ADC,Pin; adc{self._number}=ADC(Pin({self._number}))"
            self._repl.exec(cmd)

            # Set attenuation
            atten_map = ["ADC.ATTN_0DB", "ADC.ATTN_2_5DB", "ADC.ATTN_6DB", "ADC.ATTN_11DB"]
            self._repl.exec(f"adc{self._number}.atten({atten_map[self._attenuation]})")

        self._initialized = True

    def read_raw(self) -> int:
        """Read raw ADC value."""
        if not self._initialized:
            self.setup()

        if self._simulation:
            return int(self._sim_value * self._max_value)

        result = self._repl.eval(f"adc{self._number}.read()")
        return int(result)

    def cleanup(self) -> None:
        """Release pin resources."""
        self._initialized = False


class ESP32DACPin:
    """DAC output pin for ESP32 via MicroPython REPL.

    ESP32 has two 8-bit DAC channels on GPIO25 and GPIO26.
    """

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        repl: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize a DAC pin.

        Args:
            number: GPIO pin (25 or 26 for ESP32)
            name: Optional name
            repl: MicroPython REPL connection
            simulation: Run in simulation mode
        """
        if number not in (25, 26) and not simulation:
            raise ValueError("ESP32 DAC only available on GPIO 25 and 26")

        self._number = number
        self._name = name or f"dac-{number}"
        self._repl = repl
        self._simulation = simulation
        self._value: int = 0
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
    def value(self) -> int:
        """Get current DAC value (0-255)."""
        return self._value

    @property
    def initialized(self) -> bool:
        """Check if initialized."""
        return self._initialized

    def setup(self) -> None:
        """Initialize DAC output."""
        if self._initialized:
            return

        if self._simulation:
            pass
        else:
            if self._repl is None:
                raise HardwareNotFoundError(
                    device="ESP32",
                    details="REPL connection not established",
                )

            self._repl.exec(
                f"from machine import DAC,Pin; dac{self._number}=DAC(Pin({self._number}))"
            )

        self._initialized = True

    def write(self, value: int) -> None:
        """Write DAC value (0-255)."""
        if not self._initialized:
            self.setup()

        self._value = max(0, min(255, value))

        if not self._simulation and self._repl is not None:
            self._repl.exec(f"dac{self._number}.write({self._value})")

    def write_voltage(self, voltage: float, vref: float = 3.3) -> None:
        """Write voltage (0 to vref)."""
        value = int((voltage / vref) * 255)
        self.write(value)

    def cleanup(self) -> None:
        """Release pin resources."""
        self._initialized = False


class ESP32TouchPin:
    """Capacitive touch pin for ESP32 via MicroPython REPL.

    ESP32 has 10 touch-capable pins (Touch0-Touch9).
    """

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        threshold: int | None = None,
        repl: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize a touch pin.

        Args:
            number: GPIO pin (must be touch-capable)
            name: Optional name
            threshold: Touch detection threshold
            repl: MicroPython REPL connection
            simulation: Run in simulation mode
        """
        self._number = number
        self._name = name or f"touch-{number}"
        self._threshold = threshold
        self._repl = repl
        self._simulation = simulation
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
    def initialized(self) -> bool:
        """Check if initialized."""
        return self._initialized

    def setup(self) -> None:
        """Initialize touch input."""
        if self._initialized:
            return

        if self._simulation:
            self._sim_value = 500  # Untouched typical value
        else:
            if self._repl is None:
                raise HardwareNotFoundError(
                    device="ESP32",
                    details="REPL connection not established",
                )

            self._repl.exec(
                f"from machine import TouchPad,Pin; touch{self._number}=TouchPad(Pin({self._number}))"
            )

        self._initialized = True

    def read(self) -> int:
        """Read touch sensor value.

        Lower values indicate touch (higher capacitance).
        Typical untouched values are 400-600.
        Touched values are typically below 100.
        """
        if not self._initialized:
            self.setup()

        if self._simulation:
            return self._sim_value

        result = self._repl.eval(f"touch{self._number}.read()")
        return int(result)

    def is_touched(self, threshold: int | None = None) -> bool:
        """Check if touched (value below threshold).

        Args:
            threshold: Detection threshold. Lower = more sensitive.
                       Default is 100 or value set in constructor.
        """
        th = threshold or self._threshold or 100
        return self.read() < th

    def calibrate(self, samples: int = 10) -> int:
        """Calibrate touch threshold from untouched readings.

        Args:
            samples: Number of samples to average

        Returns:
            Suggested threshold (half of average untouched value)
        """
        if not self._initialized:
            self.setup()

        total = 0
        for _ in range(samples):
            total += self.read()
            time.sleep(0.01)

        average = total // samples
        self._threshold = average // 2
        return self._threshold

    def cleanup(self) -> None:
        """Release pin resources."""
        self._initialized = False


class ESP32HallSensor:
    """Hall effect sensor for ESP32 via MicroPython REPL.

    The ESP32 has a built-in hall effect sensor.
    """

    def __init__(
        self,
        *,
        repl: Any = None,
        simulation: bool = False,
    ) -> None:
        """Initialize the hall sensor.

        Args:
            repl: MicroPython REPL connection
            simulation: Run in simulation mode
        """
        self._repl = repl
        self._simulation = simulation
        self._initialized = False

    @property
    def initialized(self) -> bool:
        """Check if initialized."""
        return self._initialized

    def setup(self) -> None:
        """Initialize hall sensor."""
        if self._initialized:
            return

        if not self._simulation and self._repl is not None:
            self._repl.exec("import esp32")

        self._initialized = True

    def read(self) -> int:
        """Read hall sensor value.

        Returns a signed value. Positive/negative indicates
        magnetic field polarity.
        """
        if not self._initialized:
            self.setup()

        if self._simulation:
            return 0  # No magnetic field

        result = self._repl.eval("esp32.hall_sensor()")
        return int(result)

    def cleanup(self) -> None:
        """Release resources."""
        self._initialized = False


# =============================================================================
# MicroPython REPL Connection
# =============================================================================


class MicroPythonREPL:
    """MicroPython REPL connection handler.

    Supports both serial and WebREPL connections.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = 115200,
        host: str | None = None,
        webrepl_password: str | None = None,
        timeout: float = 5.0,
    ) -> None:
        """Initialize REPL connection.

        Args:
            port: Serial port for USB connection
            baudrate: Serial baudrate
            host: WebREPL host for WiFi connection
            webrepl_password: WebREPL password
            timeout: Connection timeout
        """
        self._port = port
        self._baudrate = baudrate
        self._host = host
        self._password = webrepl_password
        self._timeout = timeout
        self._serial: Any = None
        self._webrepl: Any = None
        self._mode: ESP32ConnectionMode | None = None

    @property
    def mode(self) -> ESP32ConnectionMode | None:
        """Get connection mode."""
        return self._mode

    @property
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._serial is not None or self._webrepl is not None

    def connect(self) -> None:
        """Establish connection."""
        if self._host:
            self._connect_webrepl()
        elif self._port:
            self._connect_serial()
        else:
            # Auto-detect
            self._auto_connect()

    def _connect_serial(self) -> None:
        """Connect via serial port."""
        try:
            import serial

            self._serial = serial.Serial(
                self._port,
                self._baudrate,
                timeout=self._timeout,
            )
            self._mode = ESP32ConnectionMode.SERIAL

            # Enter raw REPL mode
            self._serial.write(REPL_CTRL_C)
            time.sleep(0.1)
            self._serial.write(REPL_CTRL_A)
            time.sleep(0.1)
            self._serial.read_all()

            logger.info("Connected to ESP32 via serial: %s", self._port)

        except ImportError:
            raise HardwareNotFoundError(
                device="pyserial",
                details="Install with: pip install pyserial",
            ) from None
        except Exception as e:
            raise HardwareNotFoundError(
                device="ESP32 serial",
                details=str(e),
            ) from e

    def _connect_webrepl(self) -> None:
        """Connect via WebREPL."""
        try:
            import websocket

            url = f"ws://{self._host}:8266"
            self._webrepl = websocket.create_connection(url, timeout=self._timeout)
            self._mode = ESP32ConnectionMode.WEBREPL

            # Read welcome message
            self._webrepl.recv()

            # Send password
            if self._password:
                self._webrepl.send(self._password + "\r\n")
                time.sleep(0.5)
                self._webrepl.recv()

            # Enter raw REPL
            self._webrepl.send(REPL_CTRL_A.decode())
            time.sleep(0.1)

            logger.info("Connected to ESP32 via WebREPL: %s", self._host)

        except ImportError:
            raise HardwareNotFoundError(
                device="websocket-client",
                details="Install with: pip install websocket-client",
            ) from None
        except Exception as e:
            raise HardwareNotFoundError(
                device="ESP32 WebREPL",
                details=str(e),
            ) from e

    def _auto_connect(self) -> None:
        """Auto-detect and connect."""
        port = self._find_esp_port()
        if port:
            self._port = port
            self._connect_serial()
        else:
            raise HardwareNotFoundError(
                device="ESP32",
                details="No ESP32 found. Specify port or host.",
            )

    def _find_esp_port(self) -> str | None:
        """Find ESP32/ESP8266 serial port."""
        try:
            import serial.tools.list_ports

            # Common ESP32/ESP8266 USB VIDs
            esp_vids = {
                0x10C4,  # Silicon Labs CP210x
                0x1A86,  # CH340
                0x0403,  # FTDI
            }

            for port in serial.tools.list_ports.comports():
                if port.vid in esp_vids:
                    return port.device
                if port.description and (
                    "esp" in port.description.lower() or "cp210" in port.description.lower()
                ):
                    return port.device

        except ImportError:
            pass

        return None

    def exec(self, code: str) -> None:
        """Execute MicroPython code."""
        if not self.is_connected:
            raise HardwareNotFoundError(device="ESP32", details="Not connected")

        if self._mode == ESP32ConnectionMode.SERIAL:
            self._exec_serial(code)
        else:
            self._exec_webrepl(code)

    def _exec_serial(self, code: str) -> None:
        """Execute via serial."""
        # Send code in raw REPL mode
        self._serial.write(code.encode() + REPL_CTRL_D)
        time.sleep(0.05)
        # Read response
        response = self._serial.read_until(b"OK")
        if b"Traceback" in response:
            raise RuntimeError(f"MicroPython error: {response.decode()}")

    def _exec_webrepl(self, code: str) -> None:
        """Execute via WebREPL."""
        self._webrepl.send(code + "\x04")
        time.sleep(0.05)
        response = self._webrepl.recv()
        if "Traceback" in response:
            raise RuntimeError(f"MicroPython error: {response}")

    def eval(self, expression: str) -> str:
        """Evaluate expression and return result."""
        if not self.is_connected:
            raise HardwareNotFoundError(device="ESP32", details="Not connected")

        if self._mode == ESP32ConnectionMode.SERIAL:
            return self._eval_serial(expression)
        else:
            return self._eval_webrepl(expression)

    def _eval_serial(self, expression: str) -> str:
        """Evaluate via serial."""
        code = f"print({expression})"
        self._serial.write(code.encode() + REPL_CTRL_D)
        time.sleep(0.05)
        response = self._serial.read_until(b">")
        # Parse result from response
        lines = response.decode().strip().split("\n")
        for line in lines:
            if line.strip() and not line.startswith("OK"):
                return line.strip()
        return ""

    def _eval_webrepl(self, expression: str) -> str:
        """Evaluate via WebREPL."""
        code = f"print({expression})"
        self._webrepl.send(code + "\x04")
        time.sleep(0.05)
        response = self._webrepl.recv()
        # Parse result
        lines = response.strip().split("\n")
        for line in lines:
            if line.strip() and not line.startswith("OK"):
                return line.strip()
        return ""

    def disconnect(self) -> None:
        """Close connection."""
        try:
            if self._serial:
                # Exit raw REPL
                self._serial.write(REPL_CTRL_B)
                self._serial.close()
                self._serial = None
            if self._webrepl:
                self._webrepl.close()
                self._webrepl = None
        except Exception as e:
            logger.warning("Error disconnecting from ESP32: %s", e)
        finally:
            self._mode = None


# =============================================================================
# ESP32 Platform
# =============================================================================


class ESP32Platform(BasePlatform):
    """ESP32/ESP8266 platform via MicroPython REPL.

    Connects to ESP devices running MicroPython over USB serial
    or WiFi WebREPL for GPIO, PWM, ADC, DAC, and touch control.

    Example:
        >>> # USB connection
        >>> platform = ESP32Platform(port="/dev/ttyUSB0")
        >>>
        >>> # WiFi connection
        >>> platform = ESP32Platform(host="192.168.1.100", webrepl_password="secret")
        >>>
        >>> # Digital I/O
        >>> led = platform.get_pin(2, mode=PinMode.OUTPUT)
        >>> led.high()
        >>>
        >>> # PWM (any GPIO pin)
        >>> motor = platform.get_pin(5, mode=PinMode.PWM)
        >>> motor.write(0.5)
        >>>
        >>> # ADC
        >>> sensor = platform.get_analog_pin(32)
        >>> print(sensor.read())
        >>>
        >>> # DAC (GPIO25 or 26)
        >>> dac = platform.get_dac_pin(25)
        >>> dac.write(128)
        >>>
        >>> # Touch sensor
        >>> touch = platform.get_touch_pin(4)
        >>> if touch.is_touched():
        ...     print("Touched!")
        >>>
        >>> platform.cleanup()
    """

    def __init__(
        self,
        config: PlatformConfig | None = None,
        *,
        port: str | None = None,
        baudrate: int = 115200,
        host: str | None = None,
        webrepl_password: str | None = None,
        chip: ESP32Chip | None = None,
    ) -> None:
        """Initialize ESP32 platform.

        Args:
            config: Platform configuration
            port: Serial port for USB connection
            baudrate: Serial baudrate
            host: WebREPL host for WiFi connection
            webrepl_password: WebREPL password
            chip: ESP32 chip variant hint
        """
        if config is None:
            config = PlatformConfig(
                name="ESP32",
                platform_type=PlatformType.ESP32,
            )

        super().__init__(config)

        self._port = port
        self._baudrate = baudrate
        self._host = host
        self._webrepl_password = webrepl_password
        self._chip = chip or ESP32Chip.ESP32
        self._repl: MicroPythonREPL | None = None
        self._simulation = self._check_simulation()

        # Special pin storage
        self._dac_pins: dict[int, ESP32DACPin] = {}
        self._touch_pins: dict[int, ESP32TouchPin] = {}
        self._hall_sensor: ESP32HallSensor | None = None

        if not self._simulation:
            self._connect()

        logger.info(
            "ESP32 platform initialized (chip=%s, simulation=%s)",
            self._chip.value,
            self._simulation,
        )

    def _check_simulation(self) -> bool:
        """Check if running in simulation mode."""
        if os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes"):
            return True

        # Check if pyserial is available
        try:
            import serial  # noqa: F401

            return False
        except ImportError:
            return True

    def _connect(self) -> None:
        """Connect to ESP32."""
        self._repl = MicroPythonREPL(
            port=self._port,
            baudrate=self._baudrate,
            host=self._host,
            webrepl_password=self._webrepl_password,
        )
        self._repl.connect()

        # Detect chip if not specified
        if self._chip == ESP32Chip.ESP32:
            self._chip = self._detect_chip()

    def _detect_chip(self) -> ESP32Chip:
        """Detect ESP32 chip variant."""
        if self._simulation or self._repl is None:
            return ESP32Chip.ESP32

        try:
            result = self._repl.eval("import sys; sys.platform")
            if "esp8266" in result.lower():
                return ESP32Chip.ESP8266
            elif "esp32-s2" in result.lower():
                return ESP32Chip.ESP32_S2
            elif "esp32-s3" in result.lower():
                return ESP32Chip.ESP32_S3
            elif "esp32-c3" in result.lower():
                return ESP32Chip.ESP32_C3
            elif "esp32" in result.lower():
                return ESP32Chip.ESP32
        except Exception:
            pass

        return ESP32Chip.UNKNOWN

    @property
    def port(self) -> str | None:
        """Get the serial port."""
        return self._port

    @property
    def host(self) -> str | None:
        """Get the WebREPL host."""
        return self._host

    @property
    def chip(self) -> ESP32Chip:
        """Get the chip variant."""
        return self._chip

    @property
    def connection_mode(self) -> ESP32ConnectionMode | None:
        """Get the connection mode."""
        if self._repl:
            return self._repl.mode
        return None

    @property
    def is_available(self) -> bool:
        """Check if ESP32 is available."""
        if self._simulation:
            return True
        return self._repl is not None and self._repl.is_connected

    def get_chip_capabilities(self) -> dict[str, Any]:
        """Get capabilities for current chip."""
        return ESP32_CAPABILITIES.get(self._chip, DEFAULT_ESP_CAPABILITIES)

    def get_analog_pin(self, pin: int, **kwargs: Any) -> ESP32AnalogPin:
        """Get an analog input pin.

        Args:
            pin: GPIO pin number (must be ADC-capable)
            **kwargs: Additional pin options
        """
        capabilities = self.get_chip_capabilities()
        resolution = capabilities.get("adc_resolution", 12)

        analog_pin = ESP32AnalogPin(
            pin,
            name=kwargs.get("name"),
            resolution=resolution,
            reference_voltage=kwargs.get("reference_voltage", 3.3),
            attenuation=kwargs.get("attenuation", 3),
            repl=self._repl,
            simulation=self._simulation,
        )
        return analog_pin

    def get_dac_pin(self, pin: int, **kwargs: Any) -> ESP32DACPin:
        """Get a DAC output pin (GPIO25 or 26).

        Args:
            pin: GPIO pin number (25 or 26)
            **kwargs: Additional options
        """
        if pin in self._dac_pins:
            return self._dac_pins[pin]

        dac = ESP32DACPin(
            pin,
            name=kwargs.get("name"),
            repl=self._repl,
            simulation=self._simulation,
        )
        self._dac_pins[pin] = dac
        return dac

    def get_touch_pin(self, pin: int, **kwargs: Any) -> ESP32TouchPin:
        """Get a touch sensor pin.

        Args:
            pin: GPIO pin number (must be touch-capable)
            **kwargs: threshold option
        """
        if pin in self._touch_pins:
            return self._touch_pins[pin]

        touch = ESP32TouchPin(
            pin,
            name=kwargs.get("name"),
            threshold=kwargs.get("threshold"),
            repl=self._repl,
            simulation=self._simulation,
        )
        self._touch_pins[pin] = touch
        return touch

    def get_hall_sensor(self) -> ESP32HallSensor:
        """Get the built-in hall effect sensor."""
        if self._hall_sensor is None:
            self._hall_sensor = ESP32HallSensor(
                repl=self._repl,
                simulation=self._simulation,
            )
        return self._hall_sensor

    def _detect_info(self) -> PlatformInfo:
        """Detect platform information."""
        capabilities: set[PlatformCapability] = {
            PlatformCapability.GPIO,
            PlatformCapability.PWM,
            PlatformCapability.I2C,
            PlatformCapability.SPI,
        }

        chip_caps = self.get_chip_capabilities()

        if chip_caps.get("dac_pins"):
            capabilities.add(PlatformCapability.CAN)  # Reusing for DAC

        return PlatformInfo(
            platform_type=PlatformType.ESP32,
            model=self._chip.value,
            revision="",
            serial="",
            capabilities=capabilities,
            gpio_chips=[],
            i2c_buses=list(range(chip_caps.get("i2c_buses", 1))),
            spi_buses=list(range(chip_caps.get("spi_buses", 1))),
            uart_ports=[self._port] if self._port else [],
        )

    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create an ESP32 pin."""
        pin_num = int(pin_id) if isinstance(pin_id, str) else pin_id

        mode = kwargs.get("mode", PinMode.OUTPUT)

        if mode == PinMode.PWM:
            return ESP32PWMPin(
                pin_num,
                name=kwargs.get("name"),
                frequency=kwargs.get("frequency", 1000),
                duty_cycle=kwargs.get("duty_cycle", 0.0),
                repl=self._repl,
                simulation=self._simulation,
            )
        elif mode == PinMode.ANALOG:
            return self.get_analog_pin(pin_num, **kwargs)
        else:
            initial = kwargs.get("initial", PinState.LOW)
            if isinstance(initial, bool):
                initial = PinState.HIGH if initial else PinState.LOW

            return ESP32DigitalPin(
                pin_num,
                mode=mode,
                name=kwargs.get("name"),
                inverted=kwargs.get("inverted", False),
                initial=initial,
                repl=self._repl,
                simulation=self._simulation,
            )

    def _create_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Create a communication bus."""
        raise HardwareNotFoundError(
            device=f"Bus type: {bus_type}",
            details="ESP32 bus operations should use MicroPython machine.I2C/SPI directly.",
        )

    def soft_reset(self) -> None:
        """Soft reset the ESP32."""
        if self._simulation:
            logger.info("Simulation: ESP32 soft reset")
            return

        if self._repl is not None:
            # Reset will disconnect, so we suppress any exceptions
            with contextlib.suppress(Exception):
                self._repl.exec("import machine; machine.soft_reset()")

    def cleanup(self) -> None:
        """Cleanup all resources."""
        # Cleanup DAC pins
        for dac in self._dac_pins.values():
            dac.cleanup()
        self._dac_pins.clear()

        # Cleanup touch pins
        for touch in self._touch_pins.values():
            touch.cleanup()
        self._touch_pins.clear()

        # Cleanup hall sensor
        if self._hall_sensor:
            self._hall_sensor.cleanup()
            self._hall_sensor = None

        # Cleanup pins from parent
        super().cleanup()

        # Disconnect from device
        if self._repl is not None:
            self._repl.disconnect()
            self._repl = None


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "DEFAULT_ESP_CAPABILITIES",
    # Constants
    "ESP32_CAPABILITIES",
    # Pin classes
    "ESP32AnalogPin",
    # Enums
    "ESP32Chip",
    "ESP32ConnectionMode",
    "ESP32DACPin",
    "ESP32DigitalPin",
    "ESP32HallSensor",
    "ESP32PWMPin",
    # Platform
    "ESP32Platform",
    "ESP32TouchPin",
    # REPL
    "MicroPythonREPL",
]
