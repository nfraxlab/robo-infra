"""BeagleBone platform implementation.

Supports BeagleBone boards including:
- BeagleBone Black
- BeagleBone Green
- BeagleBone AI
- BeagleBone AI-64
- PocketBeagle

This module provides hardware access via:
1. Adafruit_BBIO (recommended)
2. libgpiod (gpiod)
3. Direct sysfs access (fallback)

Features:
- GPIO with P8/P9 header naming
- PWM output (EHRPWM and ECAP)
- ADC input (7 channels, 12-bit)
- PRU (Programmable Real-time Unit) interface
- Device tree overlay management

Example:
    >>> from robo_infra.platforms.beaglebone import BeagleBonePlatform
    >>>
    >>> # Auto-detect platform
    >>> platform = BeagleBonePlatform()
    >>>
    >>> # Get a GPIO pin using header notation
    >>> led = platform.get_pin("P9_12", mode=PinMode.OUTPUT)
    >>> led.high()
    >>>
    >>> # Get ADC channel
    >>> adc = platform.get_adc(0)  # AIN0
    >>> voltage = adc.read()
    >>>
    >>> # Cleanup
    >>> platform.cleanup()
"""

from __future__ import annotations

import contextlib
import logging
import os
import re
import subprocess
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
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


class BeagleBoneModel(Enum):
    """BeagleBone model identifiers."""

    BLACK = "BeagleBone Black"
    BLACK_WIRELESS = "BeagleBone Black Wireless"
    GREEN = "BeagleBone Green"
    GREEN_WIRELESS = "BeagleBone Green Wireless"
    AI = "BeagleBone AI"
    AI_64 = "BeagleBone AI-64"
    POCKET = "PocketBeagle"
    BLUE = "BeagleBone Blue"
    UNKNOWN = "Unknown BeagleBone"


class BBIOBackend(Enum):
    """Available GPIO backends for BeagleBone."""

    ADAFRUIT_BBIO = "adafruit_bbio"
    GPIOD = "gpiod"
    SYSFS = "sysfs"
    SIMULATION = "simulation"


class PRUState(Enum):
    """PRU (Programmable Real-time Unit) states."""

    STOPPED = "stopped"
    RUNNING = "running"
    OFFLINE = "offline"
    ERROR = "error"


# Pin mapping: header notation to GPIO chip/line
# BeagleBone has two headers: P8 (46 pins) and P9 (46 pins)
# Format: "P8_XX" or "P9_XX"

# GPIO number mappings for BeagleBone Black
# Calculated as: gpio_bank * 32 + gpio_pin
# e.g., GPIO1_28 = 1 * 32 + 28 = 60
P8_GPIO_MAP: dict[int, int] = {
    3: 38,  # P8_3  -> GPIO1_6
    4: 39,  # P8_4  -> GPIO1_7
    5: 34,  # P8_5  -> GPIO1_2
    6: 35,  # P8_6  -> GPIO1_3
    7: 66,  # P8_7  -> GPIO2_2
    8: 67,  # P8_8  -> GPIO2_3
    9: 69,  # P8_9  -> GPIO2_5
    10: 68,  # P8_10 -> GPIO2_4
    11: 45,  # P8_11 -> GPIO1_13
    12: 44,  # P8_12 -> GPIO1_12
    13: 23,  # P8_13 -> GPIO0_23 (EHRPWM2B)
    14: 26,  # P8_14 -> GPIO0_26
    15: 47,  # P8_15 -> GPIO1_15
    16: 46,  # P8_16 -> GPIO1_14
    17: 27,  # P8_17 -> GPIO0_27
    18: 65,  # P8_18 -> GPIO2_1
    19: 22,  # P8_19 -> GPIO0_22 (EHRPWM2A)
    20: 63,  # P8_20 -> GPIO1_31
    21: 62,  # P8_21 -> GPIO1_30
    22: 37,  # P8_22 -> GPIO1_5
    23: 36,  # P8_23 -> GPIO1_4
    24: 33,  # P8_24 -> GPIO1_1
    25: 32,  # P8_25 -> GPIO1_0
    26: 61,  # P8_26 -> GPIO1_29
    27: 86,  # P8_27 -> GPIO2_22
    28: 88,  # P8_28 -> GPIO2_24
    29: 87,  # P8_29 -> GPIO2_23
    30: 89,  # P8_30 -> GPIO2_25
    31: 10,  # P8_31 -> GPIO0_10
    32: 11,  # P8_32 -> GPIO0_11
    33: 9,  # P8_33 -> GPIO0_9
    34: 81,  # P8_34 -> GPIO2_17 (EHRPWM1B)
    35: 8,  # P8_35 -> GPIO0_8
    36: 80,  # P8_36 -> GPIO2_16 (EHRPWM1A)
    37: 78,  # P8_37 -> GPIO2_14
    38: 79,  # P8_38 -> GPIO2_15
    39: 76,  # P8_39 -> GPIO2_12
    40: 77,  # P8_40 -> GPIO2_13
    41: 74,  # P8_41 -> GPIO2_10
    42: 75,  # P8_42 -> GPIO2_11
    43: 72,  # P8_43 -> GPIO2_8
    44: 73,  # P8_44 -> GPIO2_9
    45: 70,  # P8_45 -> GPIO2_6 (EHRPWM2A)
    46: 71,  # P8_46 -> GPIO2_7 (EHRPWM2B)
}

P9_GPIO_MAP: dict[int, int] = {
    11: 30,  # P9_11 -> GPIO0_30
    12: 60,  # P9_12 -> GPIO1_28
    13: 31,  # P9_13 -> GPIO0_31
    14: 50,  # P9_14 -> GPIO1_18 (EHRPWM1A)
    15: 48,  # P9_15 -> GPIO1_16
    16: 51,  # P9_16 -> GPIO1_19 (EHRPWM1B)
    17: 5,  # P9_17 -> GPIO0_5
    18: 4,  # P9_18 -> GPIO0_4
    21: 3,  # P9_21 -> GPIO0_3 (EHRPWM0B)
    22: 2,  # P9_22 -> GPIO0_2 (EHRPWM0A)
    23: 49,  # P9_23 -> GPIO1_17
    24: 15,  # P9_24 -> GPIO0_15
    25: 117,  # P9_25 -> GPIO3_21
    26: 14,  # P9_26 -> GPIO0_14
    27: 115,  # P9_27 -> GPIO3_19
    28: 113,  # P9_28 -> GPIO3_17
    29: 111,  # P9_29 -> GPIO3_15
    30: 112,  # P9_30 -> GPIO3_16
    31: 110,  # P9_31 -> GPIO3_14
    41: 20,  # P9_41 -> GPIO0_20
    42: 7,  # P9_42 -> GPIO0_7 (ECAP0)
}

# PWM-capable pins: maps header pin to PWM module
# EHRPWM: Enhanced High-Resolution PWM
# ECAP: Enhanced Capture
PWM_PINS: dict[str, tuple[str, str]] = {
    "P8_13": ("ehrpwm2", "B"),  # EHRPWM2B
    "P8_19": ("ehrpwm2", "A"),  # EHRPWM2A
    "P8_34": ("ehrpwm1", "B"),  # EHRPWM1B
    "P8_36": ("ehrpwm1", "A"),  # EHRPWM1A
    "P8_45": ("ehrpwm2", "A"),  # EHRPWM2A (alt)
    "P8_46": ("ehrpwm2", "B"),  # EHRPWM2B (alt)
    "P9_14": ("ehrpwm1", "A"),  # EHRPWM1A
    "P9_16": ("ehrpwm1", "B"),  # EHRPWM1B
    "P9_21": ("ehrpwm0", "B"),  # EHRPWM0B
    "P9_22": ("ehrpwm0", "A"),  # EHRPWM0A
    "P9_42": ("ecap0", "0"),  # ECAP0
}

# ADC channels: BeagleBone has 7 ADC channels (AIN0-AIN6)
# Located on P9 header
ADC_PINS: dict[int, str] = {
    0: "P9_39",  # AIN0
    1: "P9_40",  # AIN1
    2: "P9_37",  # AIN2
    3: "P9_38",  # AIN3
    4: "P9_33",  # AIN4
    5: "P9_36",  # AIN5
    6: "P9_35",  # AIN6
}

# ADC reference voltage
ADC_VREF = 1.8  # 1.8V reference voltage
ADC_RESOLUTION = 12  # 12-bit ADC

# PRU paths
PRU_PATH = Path("/sys/class/remoteproc")
PRU0_PATH = PRU_PATH / "remoteproc1"  # PRU0
PRU1_PATH = PRU_PATH / "remoteproc2"  # PRU1


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class BeagleBoneCapabilities:
    """Capabilities for a specific BeagleBone model."""

    model: BeagleBoneModel
    gpio_count: int
    pwm_count: int
    adc_channels: int
    adc_resolution: int
    has_pru: bool
    has_hdmi: bool
    has_emmc: bool
    ram_mb: int
    processor: str
    description: str = ""


# Model capabilities
BB_CAPABILITIES: dict[BeagleBoneModel, BeagleBoneCapabilities] = {
    BeagleBoneModel.BLACK: BeagleBoneCapabilities(
        model=BeagleBoneModel.BLACK,
        gpio_count=65,
        pwm_count=8,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=True,
        has_emmc=True,
        ram_mb=512,
        processor="AM3358",
        description="BeagleBone Black with AM3358 1GHz ARM Cortex-A8",
    ),
    BeagleBoneModel.BLACK_WIRELESS: BeagleBoneCapabilities(
        model=BeagleBoneModel.BLACK_WIRELESS,
        gpio_count=65,
        pwm_count=8,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=True,
        has_emmc=True,
        ram_mb=512,
        processor="AM3358",
        description="BeagleBone Black Wireless with WiFi/BLE",
    ),
    BeagleBoneModel.GREEN: BeagleBoneCapabilities(
        model=BeagleBoneModel.GREEN,
        gpio_count=65,
        pwm_count=8,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=False,  # No HDMI, has Grove connectors
        has_emmc=True,
        ram_mb=512,
        processor="AM3358",
        description="BeagleBone Green with Grove connectors",
    ),
    BeagleBoneModel.GREEN_WIRELESS: BeagleBoneCapabilities(
        model=BeagleBoneModel.GREEN_WIRELESS,
        gpio_count=65,
        pwm_count=8,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=False,
        has_emmc=True,
        ram_mb=512,
        processor="AM3358",
        description="BeagleBone Green Wireless with WiFi/BLE",
    ),
    BeagleBoneModel.AI: BeagleBoneCapabilities(
        model=BeagleBoneModel.AI,
        gpio_count=92,
        pwm_count=12,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=True,
        has_emmc=True,
        ram_mb=1024,
        processor="AM5729",
        description="BeagleBone AI with dual-core ARM Cortex-A15",
    ),
    BeagleBoneModel.AI_64: BeagleBoneCapabilities(
        model=BeagleBoneModel.AI_64,
        gpio_count=100,
        pwm_count=16,
        adc_channels=8,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=True,
        has_emmc=True,
        ram_mb=4096,
        processor="TDA4VM",
        description="BeagleBone AI-64 with TDA4VM SoC",
    ),
    BeagleBoneModel.POCKET: BeagleBoneCapabilities(
        model=BeagleBoneModel.POCKET,
        gpio_count=44,
        pwm_count=4,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=False,
        has_emmc=False,  # SD card only
        ram_mb=512,
        processor="AM3358",
        description="PocketBeagle, ultra-compact keychain-sized board",
    ),
    BeagleBoneModel.BLUE: BeagleBoneCapabilities(
        model=BeagleBoneModel.BLUE,
        gpio_count=69,
        pwm_count=8,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=False,
        has_emmc=True,
        ram_mb=512,
        processor="AM3358",
        description="BeagleBone Blue for robotics",
    ),
    BeagleBoneModel.UNKNOWN: BeagleBoneCapabilities(
        model=BeagleBoneModel.UNKNOWN,
        gpio_count=65,
        pwm_count=8,
        adc_channels=7,
        adc_resolution=12,
        has_pru=True,
        has_hdmi=True,
        has_emmc=True,
        ram_mb=512,
        processor="AM335x",
        description="Unknown BeagleBone model",
    ),
}


@dataclass
class DeviceTreeOverlay:
    """Represents a device tree overlay."""

    name: str
    path: Path | None = None
    loaded: bool = False
    pins_used: list[str] = field(default_factory=list)


# =============================================================================
# Pin Classes
# =============================================================================


class BeagleBoneDigitalPin(DigitalPin):
    """BeagleBone GPIO pin implementation."""

    def __init__(
        self,
        pin_name: str,
        mode: PinMode = PinMode.INPUT,
        backend: BBIOBackend = BBIOBackend.SIMULATION,
        pull: str | None = None,
        initial: PinState | None = None,
    ) -> None:
        """Initialize BeagleBone digital pin.

        Args:
            pin_name: Pin name in header notation (e.g., "P8_10", "P9_12")
            mode: Pin mode (INPUT or OUTPUT)
            backend: GPIO backend to use
            pull: Pull resistor configuration ("up", "down", or None)
            initial: Initial state for output pins
        """
        self._pin_name = pin_name
        self._backend = backend
        self._pull = pull
        self._gpio_num = self._resolve_gpio_number(pin_name)
        self._bbio_gpio: Any = None
        self._gpiod_line: Any = None

        # Call parent with proper arguments
        super().__init__(
            number=self._gpio_num,
            mode=mode,
            name=pin_name,
            initial=initial or PinState.LOW,
        )

    def _resolve_gpio_number(self, pin_name: str) -> int:
        """Resolve header pin name to GPIO number."""
        match = re.match(r"P([89])_(\d+)", pin_name.upper())
        if not match:
            raise ValueError(f"Invalid pin name: {pin_name}. Use format P8_XX or P9_XX")

        header = int(match.group(1))
        pin = int(match.group(2))

        if header == 8:
            if pin not in P8_GPIO_MAP:
                raise ValueError(f"Pin P8_{pin} is not a GPIO pin")
            return P8_GPIO_MAP[pin]
        else:  # header == 9
            if pin not in P9_GPIO_MAP:
                raise ValueError(f"Pin P9_{pin} is not a GPIO pin")
            return P9_GPIO_MAP[pin]

    def _setup_hardware(self) -> None:
        """Setup hardware GPIO."""
        if self._backend == BBIOBackend.ADAFRUIT_BBIO:
            try:
                from Adafruit_BBIO import GPIO

                direction = GPIO.OUT if self.mode == PinMode.OUTPUT else GPIO.IN
                pull_ud = None
                if self._pull == "up":
                    pull_ud = GPIO.PUD_UP
                elif self._pull == "down":
                    pull_ud = GPIO.PUD_DOWN

                GPIO.setup(self._pin_name, direction, pull_up_down=pull_ud)
                self._bbio_gpio = GPIO

                if self.mode == PinMode.OUTPUT and self._initial == PinState.HIGH:
                    self._write_state(True)

            except ImportError:
                raise HardwareNotFoundError(
                    device="Adafruit_BBIO",
                    details="Install with: pip install Adafruit-BBIO",
                ) from None
        elif self._backend == BBIOBackend.GPIOD:
            try:
                import gpiod

                chip = gpiod.Chip("gpiochip0")
                self._gpiod_line = chip.get_line(self._gpio_num)

                if self.mode == PinMode.OUTPUT:
                    self._gpiod_line.request(
                        consumer="robo_infra",
                        type=gpiod.LINE_REQ_DIR_OUT,
                        default_val=1 if self._initial == PinState.HIGH else 0,
                    )
                else:
                    flags = gpiod.LINE_REQ_FLAG_BIAS_DISABLE
                    if self._pull == "up":
                        flags = gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
                    elif self._pull == "down":
                        flags = gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN
                    self._gpiod_line.request(
                        consumer="robo_infra",
                        type=gpiod.LINE_REQ_DIR_IN,
                        flags=flags,
                    )

            except ImportError:
                raise HardwareNotFoundError(
                    device="gpiod",
                    details="Install with: pip install gpiod",
                ) from None

    def setup(self) -> None:
        """Initialize the pin hardware."""
        if self._initialized:
            return

        if self._backend != BBIOBackend.SIMULATION:
            self._setup_hardware()
        else:
            logger.debug("Simulation: Setting up digital pin %s", self._pin_name)

        self._initialized = True

    def _write_state(self, value: bool) -> None:
        """Write state to hardware."""
        hw_value = 1 if value else 0
        if self._bbio_gpio is not None:
            self._bbio_gpio.output(self._pin_name, hw_value)
        elif self._gpiod_line is not None:
            self._gpiod_line.set_value(hw_value)
        self._state = PinState.HIGH if value else PinState.LOW

    def _read_state(self) -> bool:
        """Read state from hardware."""
        if self._bbio_gpio is not None:
            value = self._bbio_gpio.input(self._pin_name)
            return bool(value)
        elif self._gpiod_line is not None:
            value = self._gpiod_line.get_value()
            return bool(value)
        return self._state == PinState.HIGH

    def high(self) -> None:
        """Set pin HIGH."""
        self.write(True)

    def low(self) -> None:
        """Set pin LOW."""
        self.write(False)

    def read(self) -> bool:
        """Read pin state.

        Returns:
            True if HIGH (or LOW if inverted), False otherwise
        """
        raw = self._read_state()
        return not raw if self._inverted else raw

    def write(self, value: bool) -> None:
        """Write a digital value to the pin.

        Args:
            value: True for HIGH, False for LOW (inverted if self.inverted)
        """
        if self.mode not in (PinMode.OUTPUT,):
            raise ValueError("Cannot write to input pin")
        hw_value = not value if self._inverted else value
        self._write_state(hw_value)

    def toggle(self) -> None:
        """Toggle pin state."""
        self.write(not self.read())

    def cleanup(self) -> None:
        """Release pin resources."""
        if self._bbio_gpio is not None:
            self._bbio_gpio.cleanup(self._pin_name)
        elif self._gpiod_line is not None:
            self._gpiod_line.release()

    @property
    def pin_name(self) -> str:
        """Get pin name."""
        return self._pin_name

    @property
    def gpio_number(self) -> int:
        """Get GPIO number."""
        return self._gpio_num


class BeagleBonePWMPin(PWMPin):
    """BeagleBone PWM pin implementation."""

    def __init__(
        self,
        pin_name: str,
        frequency: int = 1000,
        duty_cycle: float = 0.0,
        backend: BBIOBackend = BBIOBackend.SIMULATION,
    ) -> None:
        """Initialize BeagleBone PWM pin.

        Args:
            pin_name: Pin name in header notation (e.g., "P9_14")
            frequency: PWM frequency in Hz
            duty_cycle: Initial duty cycle (0.0-1.0)
            backend: GPIO backend to use
        """
        self._pin_name = pin_name
        self._backend = backend
        self._running = False
        self._bbio_pwm: Any = None

        if pin_name not in PWM_PINS:
            raise ValueError(
                f"Pin {pin_name} does not support PWM. Valid PWM pins: {list(PWM_PINS.keys())}"
            )

        self._pwm_module, self._pwm_channel = PWM_PINS[pin_name]

        # Use channel number from PWM_PINS as the pin number
        channel_num = int(self._pwm_channel.replace("A", "0").replace("B", "1"))

        # Call parent with proper arguments
        super().__init__(
            number=channel_num,
            name=pin_name,
            frequency=frequency,
            duty_cycle=duty_cycle,
        )

    def _setup_hardware(self) -> None:
        """Setup hardware PWM."""
        if self._backend == BBIOBackend.ADAFRUIT_BBIO:
            try:
                from Adafruit_BBIO import PWM

                # Start PWM with initial duty cycle (Adafruit_BBIO uses 0-100)
                PWM.start(self._pin_name, self._duty_cycle * 100, self._frequency)
                self._bbio_pwm = PWM
                self._running = True

            except ImportError:
                raise HardwareNotFoundError(
                    device="Adafruit_BBIO.PWM",
                    details="Install with: pip install Adafruit-BBIO",
                ) from None
            except Exception as e:
                raise HardwareNotFoundError(
                    device=f"PWM {self._pin_name}",
                    details=str(e),
                ) from e

    def setup(self) -> None:
        """Initialize the PWM pin hardware."""
        if self._initialized:
            return

        if self._backend != BBIOBackend.SIMULATION:
            self._setup_hardware()
        else:
            logger.debug("Simulation: Setting up PWM pin %s", self._pin_name)

        self._initialized = True

    def set_duty_cycle(self, duty: float) -> None:
        """Set the PWM duty cycle.

        Args:
            duty: Duty cycle from 0.0 (0%) to 1.0 (100%)
        """
        if not 0.0 <= duty <= 1.0:
            raise ValueError(f"Duty cycle must be 0.0-1.0, got {duty}")
        self._duty_cycle = duty
        if self._bbio_pwm is not None:
            self._bbio_pwm.set_duty_cycle(self._pin_name, duty * 100)

    def set_frequency(self, frequency: int) -> None:
        """Set the PWM frequency.

        Args:
            frequency: Frequency in Hz
        """
        if frequency <= 0:
            raise ValueError(f"Frequency must be positive, got {frequency}")
        self._frequency = frequency
        if self._bbio_pwm is not None:
            self._bbio_pwm.set_frequency(self._pin_name, frequency)

    def start(self) -> None:
        """Start PWM output."""
        if self._backend == BBIOBackend.SIMULATION:
            self._running = True
            logger.debug("Simulation: PWM %s started", self._pin_name)
        elif self._bbio_pwm is not None and not self._running:
            self._bbio_pwm.start(self._pin_name, self._duty_cycle * 100, self._frequency)
            self._running = True

    def stop(self) -> None:
        """Stop PWM output."""
        if self._backend == BBIOBackend.SIMULATION:
            self._running = False
            logger.debug("Simulation: PWM %s stopped", self._pin_name)
        elif self._bbio_pwm is not None and self._running:
            self._bbio_pwm.stop(self._pin_name)
            self._running = False

    def cleanup(self) -> None:
        """Release PWM resources."""
        if self._bbio_pwm is not None:
            self._bbio_pwm.stop(self._pin_name)
            self._bbio_pwm.cleanup()
        self._running = False
        self._initialized = False

    @property
    def is_running(self) -> bool:
        """Check if PWM is running."""
        return self._running

    @property
    def pin_name(self) -> str:
        """Get pin name."""
        return self._pin_name

    @property
    def pwm_module(self) -> str:
        """Get PWM module name."""
        return self._pwm_module


class BeagleBoneADCPin(AnalogPin):
    """BeagleBone ADC pin implementation."""

    def __init__(
        self,
        channel: int,
        backend: BBIOBackend = BBIOBackend.SIMULATION,
        reference_voltage: float = ADC_VREF,
    ) -> None:
        """Initialize BeagleBone ADC channel.

        Args:
            channel: ADC channel (0-6 for AIN0-AIN6)
            backend: GPIO backend to use
            reference_voltage: ADC reference voltage (default 1.8V)
        """
        if channel not in ADC_PINS:
            raise ValueError(f"Invalid ADC channel {channel}. Valid channels: 0-6")

        self._channel = channel
        self._pin_name = ADC_PINS[channel]
        self._backend = backend
        self._bbio_adc: Any = None
        self._simulated_value = 0.5  # 50% of range

        # Call parent with proper arguments
        super().__init__(
            number=channel,
            name=self._pin_name,
            resolution=ADC_RESOLUTION,
            reference_voltage=reference_voltage,
        )

    def _setup_hardware(self) -> None:
        """Setup hardware ADC."""
        if self._backend == BBIOBackend.ADAFRUIT_BBIO:
            try:
                from Adafruit_BBIO import ADC

                ADC.setup()
                self._bbio_adc = ADC

            except ImportError:
                raise HardwareNotFoundError(
                    device="Adafruit_BBIO.ADC",
                    details="Install with: pip install Adafruit-BBIO",
                ) from None

    def setup(self) -> None:
        """Initialize the ADC pin hardware."""
        if self._initialized:
            return

        if self._backend != BBIOBackend.SIMULATION:
            self._setup_hardware()
        else:
            logger.debug("Simulation: Setting up ADC channel %d", self._channel)

        self._initialized = True

    def cleanup(self) -> None:
        """Release ADC resources."""
        self._bbio_adc = None
        self._initialized = False

    def read_raw(self) -> int:
        """Read raw ADC value (0-4095 for 12-bit)."""
        if self._backend == BBIOBackend.SIMULATION:
            return int(self._simulated_value * self._max_value)
        if self._bbio_adc is not None:
            return self._bbio_adc.read_raw(self._pin_name)
        return 0

    def read_voltage(self) -> float:
        """Read voltage (0.0 to reference_voltage)."""
        return self.read()

    @property
    def channel(self) -> int:
        """Get ADC channel number."""
        return self._channel

    @property
    def pin_name(self) -> str:
        """Get associated pin name."""
        return self._pin_name

    def set_simulated_value(self, value: float) -> None:
        """Set simulated value for testing (0.0-1.0)."""
        self._simulated_value = max(0.0, min(1.0, value))


# =============================================================================
# PRU Interface
# =============================================================================


class PRUInterface:
    """Interface for BeagleBone Programmable Real-time Units (PRU).

    The PRU subsystem provides two independent PRU cores that can run
    at 200MHz for real-time control without OS interference.
    """

    def __init__(self, pru_id: int = 0, simulation: bool = False) -> None:
        """Initialize PRU interface.

        Args:
            pru_id: PRU unit (0 or 1)
            simulation: Run in simulation mode
        """
        if pru_id not in (0, 1):
            raise ValueError(f"PRU ID must be 0 or 1, got {pru_id}")

        self._pru_id = pru_id
        self._simulation = simulation
        self._pru_path = PRU0_PATH if pru_id == 0 else PRU1_PATH
        self._state = PRUState.OFFLINE

        if not simulation and self._pru_path.exists():
            self._state = self._get_state()

    def _get_state(self) -> PRUState:
        """Get current PRU state."""
        state_file = self._pru_path / "state"
        if not state_file.exists():
            return PRUState.OFFLINE

        try:
            state = state_file.read_text().strip()
            if state == "running":
                return PRUState.RUNNING
            elif state == "offline":
                return PRUState.OFFLINE
            return PRUState.STOPPED
        except Exception:
            return PRUState.ERROR

    @property
    def state(self) -> PRUState:
        """Get PRU state."""
        if self._simulation:
            return self._state
        return self._get_state()

    @property
    def pru_id(self) -> int:
        """Get PRU ID."""
        return self._pru_id

    def load_firmware(self, firmware_path: str | Path) -> bool:
        """Load PRU firmware.

        Args:
            firmware_path: Path to .out firmware file

        Returns:
            True if firmware loaded successfully
        """
        if self._simulation:
            logger.info("Simulation: Loading PRU%d firmware: %s", self._pru_id, firmware_path)
            return True

        firmware_path = Path(firmware_path)
        if not firmware_path.exists():
            raise FileNotFoundError(f"Firmware not found: {firmware_path}")

        try:
            # Copy firmware to /lib/firmware
            fw_name = f"am335x-pru{self._pru_id}-fw"
            subprocess.run(
                ["sudo", "cp", str(firmware_path), f"/lib/firmware/{fw_name}"],
                check=True,
            )
            return True
        except subprocess.CalledProcessError as e:
            logger.error("Failed to load PRU firmware: %s", e)
            return False

    def start(self) -> bool:
        """Start PRU execution."""
        if self._simulation:
            self._state = PRUState.RUNNING
            logger.info("Simulation: PRU%d started", self._pru_id)
            return True

        state_file = self._pru_path / "state"
        try:
            with open(state_file, "w") as f:
                f.write("start")
            return True
        except Exception as e:
            logger.error("Failed to start PRU%d: %s", self._pru_id, e)
            return False

    def stop(self) -> bool:
        """Stop PRU execution."""
        if self._simulation:
            self._state = PRUState.STOPPED
            logger.info("Simulation: PRU%d stopped", self._pru_id)
            return True

        state_file = self._pru_path / "state"
        try:
            with open(state_file, "w") as f:
                f.write("stop")
            return True
        except Exception as e:
            logger.error("Failed to stop PRU%d: %s", self._pru_id, e)
            return False

    @property
    def is_running(self) -> bool:
        """Check if PRU is running."""
        return self.state == PRUState.RUNNING


# =============================================================================
# Device Tree Overlay Manager
# =============================================================================


class DeviceTreeOverlayManager:
    """Manages BeagleBone device tree overlays.

    Device tree overlays configure pin multiplexing and enable
    peripherals like PWM, I2C, SPI, etc.
    """

    CAPE_MANAGER = Path("/sys/devices/platform/bone_capemgr")
    SLOTS_FILE = CAPE_MANAGER / "slots"

    def __init__(self, simulation: bool = False) -> None:
        """Initialize overlay manager.

        Args:
            simulation: Run in simulation mode
        """
        self._simulation = simulation
        self._loaded_overlays: list[str] = []

        if not simulation:
            self._loaded_overlays = self._get_loaded_overlays()

    def _get_loaded_overlays(self) -> list[str]:
        """Get currently loaded overlays."""
        if not self.SLOTS_FILE.exists():
            return []

        try:
            content = self.SLOTS_FILE.read_text()
            overlays = []
            for line in content.strip().split("\n"):
                # Format: "0: PF----  -1"
                # or:     "1: P-O-L-  Override Board Name,00A0,Override Manuf,BB-I2C2"
                if "Override" in line or line.strip():
                    parts = line.split(",")
                    if len(parts) >= 4:
                        overlays.append(parts[-1].strip())
            return overlays
        except Exception:
            return []

    @property
    def loaded_overlays(self) -> list[str]:
        """Get list of loaded overlay names."""
        if self._simulation:
            return self._loaded_overlays.copy()
        return self._get_loaded_overlays()

    def load(self, overlay_name: str) -> bool:
        """Load a device tree overlay.

        Args:
            overlay_name: Name of the overlay (e.g., "BB-PWM1")

        Returns:
            True if overlay loaded successfully
        """
        if self._simulation:
            if overlay_name not in self._loaded_overlays:
                self._loaded_overlays.append(overlay_name)
            logger.info("Simulation: Loaded overlay %s", overlay_name)
            return True

        if overlay_name in self.loaded_overlays:
            logger.debug("Overlay %s already loaded", overlay_name)
            return True

        try:
            with open(self.SLOTS_FILE, "w") as f:
                f.write(overlay_name)
            logger.info("Loaded overlay: %s", overlay_name)
            return True
        except Exception as e:
            logger.error("Failed to load overlay %s: %s", overlay_name, e)
            return False

    def unload(self, overlay_name: str) -> bool:
        """Unload a device tree overlay.

        Args:
            overlay_name: Name of the overlay to unload

        Returns:
            True if overlay unloaded successfully
        """
        if self._simulation:
            if overlay_name in self._loaded_overlays:
                self._loaded_overlays.remove(overlay_name)
            logger.info("Simulation: Unloaded overlay %s", overlay_name)
            return True

        # Find slot number for the overlay
        try:
            content = self.SLOTS_FILE.read_text()
            for line in content.strip().split("\n"):
                if overlay_name in line:
                    slot = line.split(":")[0].strip()
                    with open(self.SLOTS_FILE, "w") as f:
                        f.write(f"-{slot}")
                    logger.info("Unloaded overlay: %s (slot %s)", overlay_name, slot)
                    return True
            logger.warning("Overlay %s not found in slots", overlay_name)
            return False
        except Exception as e:
            logger.error("Failed to unload overlay %s: %s", overlay_name, e)
            return False

    def is_loaded(self, overlay_name: str) -> bool:
        """Check if an overlay is loaded."""
        return overlay_name in self.loaded_overlays


# =============================================================================
# Platform Class
# =============================================================================


class BeagleBonePlatform(BasePlatform):
    """BeagleBone platform implementation.

    Supports BeagleBone Black, Green, AI, AI-64, PocketBeagle, and Blue.

    Example:
        >>> platform = BeagleBonePlatform()
        >>> led = platform.get_pin("P9_12", mode=PinMode.OUTPUT)
        >>> led.high()
        >>>
        >>> adc = platform.get_adc(0)
        >>> voltage = adc.read_voltage()
    """

    def __init__(
        self,
        backend: BBIOBackend | str | None = None,
        config: PlatformConfig | None = None,
        simulation: bool | None = None,
    ) -> None:
        """Initialize BeagleBone platform.

        Args:
            backend: GPIO backend to use (auto-detect if None)
            config: Platform configuration
            simulation: Force simulation mode (auto-detect if None)
        """
        # Convert string backend to enum
        if isinstance(backend, str):
            backend = BBIOBackend(backend.lower())

        # Detect simulation mode
        if simulation is None:
            simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes")
            if not simulation:
                simulation = not self._is_beaglebone()

        self._simulation = simulation
        self._backend = backend or (
            BBIOBackend.SIMULATION if simulation else self._detect_backend()
        )
        self._model = BeagleBoneModel.UNKNOWN
        self._capabilities: BeagleBoneCapabilities | None = None

        # Pin tracking
        self._digital_pins: dict[str, BeagleBoneDigitalPin] = {}
        self._pwm_pins: dict[str, BeagleBonePWMPin] = {}
        self._adc_pins: dict[int, BeagleBoneADCPin] = {}

        # PRU and overlay managers
        self._pru0: PRUInterface | None = None
        self._pru1: PRUInterface | None = None
        self._overlay_manager = DeviceTreeOverlayManager(simulation=simulation)

        if not simulation:
            self._model = self._detect_model()
            self._capabilities = BB_CAPABILITIES.get(
                self._model, BB_CAPABILITIES[BeagleBoneModel.UNKNOWN]
            )
        else:
            self._model = BeagleBoneModel.BLACK
            self._capabilities = BB_CAPABILITIES[BeagleBoneModel.BLACK]

        # Initialize base class
        base_config = config or PlatformConfig(
            name=f"BeagleBone ({self._model.value})",
            platform_type=PlatformType.BEAGLEBONE,
            simulation_fallback=True,
        )
        super().__init__(config=base_config)

        logger.info(
            "BeagleBone platform initialized: model=%s, backend=%s, simulation=%s",
            self._model.value,
            self._backend.value,
            self._simulation,
        )

    def _is_beaglebone(self) -> bool:
        """Check if running on a BeagleBone."""
        # Check device tree model
        model_path = Path("/proc/device-tree/model")
        if model_path.exists():
            try:
                model = model_path.read_text().lower()
                return "beaglebone" in model or "am335x" in model
            except Exception:
                pass

        # Check cpuinfo
        cpuinfo = Path("/proc/cpuinfo")
        if cpuinfo.exists():
            try:
                content = cpuinfo.read_text().lower()
                return "am33" in content or "beagle" in content
            except Exception:
                pass

        return False

    def _detect_backend(self) -> BBIOBackend:
        """Auto-detect available backend."""
        # Try Adafruit_BBIO first (most feature-complete)
        try:
            import Adafruit_BBIO.GPIO  # noqa: F401

            return BBIOBackend.ADAFRUIT_BBIO
        except ImportError:
            pass

        # Try gpiod
        try:
            import gpiod  # noqa: F401

            return BBIOBackend.GPIOD
        except ImportError:
            pass

        # Fall back to simulation
        logger.warning("No GPIO backend found, using simulation mode")
        return BBIOBackend.SIMULATION

    def _detect_model(self) -> BeagleBoneModel:
        """Detect BeagleBone model."""
        model_path = Path("/proc/device-tree/model")
        if model_path.exists():
            try:
                model_str = model_path.read_text().strip().rstrip("\x00")

                for model in BeagleBoneModel:
                    if model.value.lower() in model_str.lower():
                        return model

                # Check for partial matches
                if "ai-64" in model_str.lower():
                    return BeagleBoneModel.AI_64
                if "ai" in model_str.lower() and "beaglebone" in model_str.lower():
                    return BeagleBoneModel.AI
                if "pocket" in model_str.lower():
                    return BeagleBoneModel.POCKET
                if "green" in model_str.lower():
                    if "wireless" in model_str.lower():
                        return BeagleBoneModel.GREEN_WIRELESS
                    return BeagleBoneModel.GREEN
                if "black" in model_str.lower():
                    if "wireless" in model_str.lower():
                        return BeagleBoneModel.BLACK_WIRELESS
                    return BeagleBoneModel.BLACK
                if "blue" in model_str.lower():
                    return BeagleBoneModel.BLUE

            except Exception:
                pass

        return BeagleBoneModel.UNKNOWN

    # -------------------------------------------------------------------------
    # Platform Interface
    # -------------------------------------------------------------------------

    @property
    def platform_type(self) -> PlatformType:
        """Get platform type."""
        return PlatformType.BEAGLEBONE

    @property
    def capabilities(self) -> set[PlatformCapability]:
        """Get platform capabilities."""
        caps = {
            PlatformCapability.GPIO,
            PlatformCapability.PWM,
            PlatformCapability.ADC,
            PlatformCapability.I2C,
            PlatformCapability.SPI,
            PlatformCapability.UART,
        }
        # PRU is BeagleBone-specific, not in PlatformCapability enum
        # So we don't add it here - check bb_capabilities.has_pru instead
        return caps

    def get_info(self) -> PlatformInfo:
        """Get platform information."""
        return PlatformInfo(
            platform_type=PlatformType.BEAGLEBONE,
            model=f"BeagleBone ({self._model.value})",
            capabilities=self.capabilities,
            gpio_chips=["gpiochip0", "gpiochip1", "gpiochip2", "gpiochip3"],
            i2c_buses=[0, 1, 2],
            spi_buses=[(0, 0), (0, 1), (1, 0), (1, 1)],
            uart_ports=["/dev/ttyO0", "/dev/ttyO1", "/dev/ttyO2", "/dev/ttyO4"],
        )

    # -------------------------------------------------------------------------
    # Pin Access
    # -------------------------------------------------------------------------

    def get_pin(
        self,
        pin_id: int | str,
        mode: PinMode = PinMode.INPUT,
        pull: str | None = None,
        initial: PinState | None = None,
    ) -> BeagleBoneDigitalPin:
        """Get a digital GPIO pin.

        Args:
            pin_id: Pin identifier (e.g., "P8_10", "P9_12", or GPIO number)
            mode: Pin mode (INPUT or OUTPUT)
            pull: Pull resistor ("up", "down", or None)
            initial: Initial state for output pins

        Returns:
            BeagleBoneDigitalPin instance
        """
        # Convert GPIO number to pin name if needed
        pin_name = self._gpio_to_pin_name(pin_id) if isinstance(pin_id, int) else pin_id.upper()

        # Validate pin name format
        if not re.match(r"P[89]_\d+", pin_name):
            raise ValueError(f"Invalid pin format: {pin_id}. Use P8_XX or P9_XX")

        # Return cached pin or create new one
        cache_key = f"{pin_name}_{mode.value}"
        if cache_key in self._digital_pins:
            return self._digital_pins[cache_key]

        pin = BeagleBoneDigitalPin(
            pin_name=pin_name,
            mode=mode,
            backend=self._backend,
            pull=pull,
            initial=initial,
        )
        self._digital_pins[cache_key] = pin
        return pin

    def _gpio_to_pin_name(self, gpio_num: int) -> str:
        """Convert GPIO number to pin name."""
        # Search P8 header
        for pin, gpio in P8_GPIO_MAP.items():
            if gpio == gpio_num:
                return f"P8_{pin}"

        # Search P9 header
        for pin, gpio in P9_GPIO_MAP.items():
            if gpio == gpio_num:
                return f"P9_{pin}"

        raise ValueError(f"GPIO {gpio_num} not found on P8/P9 headers")

    def get_pwm_pin(
        self,
        pin_id: str,
        frequency: int = 1000,
        duty_cycle: float = 0.0,
    ) -> BeagleBonePWMPin:
        """Get a PWM-capable pin.

        Args:
            pin_id: Pin identifier (e.g., "P9_14")
            frequency: PWM frequency in Hz
            duty_cycle: Initial duty cycle (0.0-1.0)

        Returns:
            BeagleBonePWMPin instance
        """
        pin_name = pin_id.upper()

        if pin_name in self._pwm_pins:
            return self._pwm_pins[pin_name]

        pin = BeagleBonePWMPin(
            pin_name=pin_name,
            frequency=frequency,
            duty_cycle=duty_cycle,
            backend=self._backend,
        )
        self._pwm_pins[pin_name] = pin
        return pin

    def get_adc(self, channel: int) -> BeagleBoneADCPin:
        """Get an ADC channel.

        Args:
            channel: ADC channel (0-6)

        Returns:
            BeagleBoneADCPin instance
        """
        if channel in self._adc_pins:
            return self._adc_pins[channel]

        pin = BeagleBoneADCPin(
            channel=channel,
            backend=self._backend,
        )
        self._adc_pins[channel] = pin
        return pin

    # -------------------------------------------------------------------------
    # Bus Access
    # -------------------------------------------------------------------------

    def get_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Get a communication bus.

        Args:
            bus_type: Bus type ("i2c", "spi", "uart")
            **kwargs: Bus-specific configuration

        Returns:
            Bus instance
        """
        if self._simulation:
            # Return simulation bus
            from robo_infra.core.bus import I2CBus, SPIBus, UARTBus

            if bus_type.lower() == "i2c":
                bus_num = kwargs.get("bus", 1)
                return I2CBus(bus_number=bus_num, simulation=True)
            elif bus_type.lower() == "spi":
                bus_num = kwargs.get("bus", 0)
                device = kwargs.get("device", 0)
                return SPIBus(bus_number=bus_num, device=device, simulation=True)
            elif bus_type.lower() == "uart":
                port = kwargs.get("port", "/dev/ttyO1")
                baudrate = kwargs.get("baudrate", 115200)
                return UARTBus(port=port, baudrate=baudrate, simulation=True)
            else:
                raise ValueError(f"Unknown bus type: {bus_type}")

        # Real hardware implementation would go here
        raise NotImplementedError("Hardware bus access not yet implemented")

    # -------------------------------------------------------------------------
    # PRU Access
    # -------------------------------------------------------------------------

    def get_pru(self, pru_id: int = 0) -> PRUInterface:
        """Get PRU interface.

        Args:
            pru_id: PRU unit (0 or 1)

        Returns:
            PRUInterface instance
        """
        if pru_id == 0:
            if self._pru0 is None:
                self._pru0 = PRUInterface(pru_id=0, simulation=self._simulation)
            return self._pru0
        elif pru_id == 1:
            if self._pru1 is None:
                self._pru1 = PRUInterface(pru_id=1, simulation=self._simulation)
            return self._pru1
        else:
            raise ValueError(f"PRU ID must be 0 or 1, got {pru_id}")

    # -------------------------------------------------------------------------
    # Overlay Management
    # -------------------------------------------------------------------------

    @property
    def overlay_manager(self) -> DeviceTreeOverlayManager:
        """Get device tree overlay manager."""
        return self._overlay_manager

    def load_overlay(self, overlay_name: str) -> bool:
        """Load a device tree overlay.

        Args:
            overlay_name: Name of overlay to load

        Returns:
            True if successful
        """
        return self._overlay_manager.load(overlay_name)

    def unload_overlay(self, overlay_name: str) -> bool:
        """Unload a device tree overlay.

        Args:
            overlay_name: Name of overlay to unload

        Returns:
            True if successful
        """
        return self._overlay_manager.unload(overlay_name)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def model(self) -> BeagleBoneModel:
        """Get detected BeagleBone model."""
        return self._model

    @property
    def backend(self) -> BBIOBackend:
        """Get active GPIO backend."""
        return self._backend

    @property
    def is_simulation(self) -> bool:
        """Check if running in simulation mode."""
        return self._simulation

    @property
    def bb_capabilities(self) -> BeagleBoneCapabilities | None:
        """Get model-specific capabilities."""
        return self._capabilities

    # -------------------------------------------------------------------------
    # Abstract Method Implementations
    # -------------------------------------------------------------------------

    @property
    def is_available(self) -> bool:
        """Check if BeagleBone platform is available."""
        if self._simulation:
            return True
        return self._is_beaglebone()

    def _detect_info(self) -> PlatformInfo:
        """Detect platform information."""
        self._capabilities or BB_CAPABILITIES[BeagleBoneModel.UNKNOWN]
        return PlatformInfo(
            platform_type=PlatformType.BEAGLEBONE,
            model=f"BeagleBone {self._model.value}",
            capabilities=self.capabilities,
            gpio_chips=["gpiochip0", "gpiochip1", "gpiochip2", "gpiochip3"],
            i2c_buses=[0, 1, 2],
            spi_buses=[(0, 0), (0, 1), (1, 0), (1, 1)],
            uart_ports=["/dev/ttyO0", "/dev/ttyO1", "/dev/ttyO2", "/dev/ttyO4"],
        )

    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create a platform-specific pin."""
        mode = kwargs.get("mode", PinMode.INPUT)
        pull = kwargs.get("pull")
        initial = kwargs.get("initial")

        return self.get_pin(pin_id, mode=mode, pull=pull, initial=initial)

    def _create_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Create a platform-specific bus."""
        from robo_infra.core.bus import I2CBus, SPIBus, UARTBus

        if bus_type.lower() == "i2c":
            bus_num = kwargs.get("bus", 1)
            if self._simulation:
                from robo_infra.core.bus import SimulatedI2CBus

                return SimulatedI2CBus(bus_num)
            return I2CBus(bus_num)
        elif bus_type.lower() == "spi":
            bus_num = kwargs.get("bus", 0)
            device = kwargs.get("device", 0)
            if self._simulation:
                from robo_infra.core.bus import SimulatedSPIBus

                return SimulatedSPIBus(bus_num, device)
            return SPIBus(bus_num, device)
        elif bus_type.lower() == "uart":
            port = kwargs.get("port", "/dev/ttyO1")
            baudrate = kwargs.get("baudrate", 115200)
            if self._simulation:
                from robo_infra.core.bus import SimulatedUARTBus

                return SimulatedUARTBus(port, baudrate)
            return UARTBus(port, baudrate)
        else:
            raise ValueError(f"Unsupported bus type: {bus_type}")

    # -------------------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------------------

    def cleanup(self) -> None:
        """Cleanup all resources."""
        # Cleanup digital pins
        for pin in self._digital_pins.values():
            pin.cleanup()
        self._digital_pins.clear()

        # Cleanup PWM pins
        for pin in self._pwm_pins.values():
            pin.cleanup()
        self._pwm_pins.clear()

        # Stop PRUs
        if self._pru0 is not None and self._pru0.is_running:
            self._pru0.stop()
        if self._pru1 is not None and self._pru1.is_running:
            self._pru1.stop()

        # Cleanup ADC (Adafruit_BBIO cleanup)
        if self._backend == BBIOBackend.ADAFRUIT_BBIO:
            with contextlib.suppress(Exception):
                from Adafruit_BBIO import ADC

                ADC.cleanup()

        logger.info("BeagleBone platform cleanup complete")
