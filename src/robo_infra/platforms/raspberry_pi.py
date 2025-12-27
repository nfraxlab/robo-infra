"""Raspberry Pi platform implementation.

Supports all Raspberry Pi models including:
- Raspberry Pi Zero, Zero W, Zero 2 W
- Raspberry Pi 3 Model A+/B/B+
- Raspberry Pi 4 Model B
- Raspberry Pi 5
- Raspberry Pi 400
- Raspberry Pi Compute Modules

This module provides hardware access via multiple backends:
1. gpiozero (recommended, highest level)
2. lgpio (Pi 5 compatible, low level)
3. RPi.GPIO (legacy, not Pi 5 compatible)
4. pigpio (daemon-based, supports remote GPIO)

Example:
    >>> from robo_infra.platforms.raspberry_pi import RaspberryPiPlatform
    >>>
    >>> # Auto-detect platform and backend
    >>> platform = RaspberryPiPlatform()
    >>>
    >>> # Get a GPIO pin
    >>> led = platform.get_pin(17, mode=PinMode.OUTPUT)
    >>> led.high()
    >>>
    >>> # Get I2C bus
    >>> i2c = platform.get_bus("i2c", bus=1)
    >>> devices = i2c.scan()
    >>>
    >>> # Cleanup
    >>> platform.cleanup()
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Any

from robo_infra.core.exceptions import HardwareNotFoundError
from robo_infra.core.pin import DigitalPin, PinMode, PinState, PWMPin
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


class PinNumbering(Enum):
    """Pin numbering schemes for Raspberry Pi."""

    BCM = "BCM"  # Broadcom GPIO numbers (recommended)
    BOARD = "BOARD"  # Physical board pin numbers


class GPIOBackend(Enum):
    """Available GPIO backends."""

    GPIOZERO = "gpiozero"
    LGPIO = "lgpio"
    RPI_GPIO = "rpi_gpio"
    PIGPIO = "pigpio"
    SIMULATION = "simulation"


# Raspberry Pi model identifiers from /proc/cpuinfo or device tree
PI_MODELS = {
    # Pi 5
    "Raspberry Pi 5": "Pi 5",
    # Pi 4 family
    "Raspberry Pi 4 Model B": "Pi 4B",
    "Raspberry Pi 400": "Pi 400",
    "Raspberry Pi Compute Module 4": "CM4",
    # Pi 3 family
    "Raspberry Pi 3 Model B Plus": "Pi 3B+",
    "Raspberry Pi 3 Model B": "Pi 3B",
    "Raspberry Pi 3 Model A Plus": "Pi 3A+",
    # Pi 2
    "Raspberry Pi 2 Model B": "Pi 2B",
    # Pi 1
    "Raspberry Pi Model B Plus": "Pi 1B+",
    "Raspberry Pi Model B": "Pi 1B",
    "Raspberry Pi Model A Plus": "Pi 1A+",
    "Raspberry Pi Model A": "Pi 1A",
    # Zero family
    "Raspberry Pi Zero 2 W": "Zero 2 W",
    "Raspberry Pi Zero W": "Zero W",
    "Raspberry Pi Zero": "Zero",
    # Compute Modules
    "Raspberry Pi Compute Module 3 Plus": "CM3+",
    "Raspberry Pi Compute Module 3": "CM3",
    "Raspberry Pi Compute Module": "CM1",
}


# Hardware PWM capable pins (BCM numbering)
# Pi 5 has 4 PWM channels, earlier models have 2
HARDWARE_PWM_PINS_STANDARD = {12, 13, 18, 19}  # PWM0, PWM1
HARDWARE_PWM_PINS_PI5 = {12, 13, 18, 19}  # Same pins but different driver


# GPIO chip paths
GPIO_CHIP_LEGACY = Path("/dev/gpiochip0")
GPIO_CHIP_PI5 = Path("/dev/gpiochip4")  # Pi 5 uses a different chip


# =============================================================================
# Raspberry Pi Pin Classes
# =============================================================================


@dataclass
class RaspberryPiPinConfig:
    """Configuration for a Raspberry Pi GPIO pin."""

    pin: int  # BCM or BOARD number
    numbering: PinNumbering = PinNumbering.BCM
    pull_up_down: str | None = None  # "up", "down", or None
    initial: bool | None = None  # Initial state for outputs


class RaspberryPiDigitalPin(DigitalPin):
    """Digital GPIO pin for Raspberry Pi.

    Wraps the underlying GPIO library to provide consistent interface.
    """

    def __init__(
        self,
        number: int,
        mode: PinMode = PinMode.OUTPUT,
        *,
        name: str | None = None,
        inverted: bool = False,
        initial: PinState = PinState.LOW,
        backend: GPIOBackend = GPIOBackend.SIMULATION,
        numbering: PinNumbering = PinNumbering.BCM,
    ) -> None:
        """Initialize a Raspberry Pi digital pin.

        Args:
            number: GPIO pin number (BCM by default)
            mode: Pin mode (INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN)
            name: Optional human-readable name
            inverted: Invert logic
            initial: Initial state for output pins
            backend: GPIO backend to use
            numbering: Pin numbering scheme
        """
        super().__init__(number, mode, name=name, inverted=inverted, initial=initial)
        self._backend = backend
        self._numbering = numbering
        self._gpio_obj: Any = None  # Backend-specific object

    def setup(self) -> None:
        """Initialize the GPIO pin."""
        if self._initialized:
            return

        if self._backend == GPIOBackend.GPIOZERO:
            self._setup_gpiozero()
        elif self._backend == GPIOBackend.LGPIO:
            self._setup_lgpio()
        elif self._backend == GPIOBackend.RPI_GPIO:
            self._setup_rpi_gpio()
        elif self._backend == GPIOBackend.PIGPIO:
            self._setup_pigpio()
        else:
            self._setup_simulation()

        self._initialized = True
        logger.debug("Initialized pin %d with backend %s", self._number, self._backend.value)

    def _setup_gpiozero(self) -> None:
        """Setup using gpiozero library."""
        try:
            if self._mode == PinMode.OUTPUT:
                from gpiozero import DigitalOutputDevice

                initial = self._initial == PinState.HIGH
                self._gpio_obj = DigitalOutputDevice(self._number, initial_value=initial)
            else:
                from gpiozero import DigitalInputDevice

                pull_up = self._mode == PinMode.INPUT_PULLUP
                self._gpio_obj = DigitalInputDevice(self._number, pull_up=pull_up)
        except ImportError as e:
            raise HardwareNotFoundError(
                device="gpiozero",
                details="Install with: pip install gpiozero",
            ) from e

    def _setup_lgpio(self) -> None:
        """Setup using lgpio library (Pi 5 compatible)."""
        try:
            import lgpio

            # Determine GPIO chip (Pi 5 uses chip 4)
            chip_num = 4 if GPIO_CHIP_PI5.exists() else 0
            self._gpio_obj = {"handle": lgpio.gpiochip_open(chip_num), "chip": chip_num}

            if self._mode == PinMode.OUTPUT:
                lgpio.gpio_claim_output(
                    self._gpio_obj["handle"],
                    self._number,
                    1 if self._initial == PinState.HIGH else 0,
                )
            else:
                flags = 0
                if self._mode == PinMode.INPUT_PULLUP:
                    flags = lgpio.SET_PULL_UP
                elif self._mode == PinMode.INPUT_PULLDOWN:
                    flags = lgpio.SET_PULL_DOWN
                lgpio.gpio_claim_input(self._gpio_obj["handle"], self._number, flags)
        except ImportError as e:
            raise HardwareNotFoundError(
                device="lgpio",
                details="Install with: pip install lgpio",
            ) from e

    def _setup_rpi_gpio(self) -> None:
        """Setup using RPi.GPIO library (legacy, not Pi 5 compatible)."""
        try:
            from RPi import GPIO

            if self._numbering == PinNumbering.BCM:
                GPIO.setmode(GPIO.BCM)
            else:
                GPIO.setmode(GPIO.BOARD)

            if self._mode == PinMode.OUTPUT:
                initial = GPIO.HIGH if self._initial == PinState.HIGH else GPIO.LOW
                GPIO.setup(self._number, GPIO.OUT, initial=initial)
            else:
                pud = GPIO.PUD_OFF
                if self._mode == PinMode.INPUT_PULLUP:
                    pud = GPIO.PUD_UP
                elif self._mode == PinMode.INPUT_PULLDOWN:
                    pud = GPIO.PUD_DOWN
                GPIO.setup(self._number, GPIO.IN, pull_up_down=pud)

            self._gpio_obj = {"GPIO": GPIO}
        except ImportError as e:
            raise HardwareNotFoundError(
                device="RPi.GPIO",
                details="Install with: pip install RPi.GPIO",
            ) from e

    def _setup_pigpio(self) -> None:
        """Setup using pigpio library (daemon-based)."""
        try:
            import pigpio

            pi = pigpio.pi()
            if not pi.connected:
                raise HardwareNotFoundError(
                    device="pigpio daemon",
                    details="Start with: sudo pigpiod",
                )

            if self._mode == PinMode.OUTPUT:
                pi.set_mode(self._number, pigpio.OUTPUT)
                pi.write(self._number, 1 if self._initial == PinState.HIGH else 0)
            else:
                pi.set_mode(self._number, pigpio.INPUT)
                if self._mode == PinMode.INPUT_PULLUP:
                    pi.set_pull_up_down(self._number, pigpio.PUD_UP)
                elif self._mode == PinMode.INPUT_PULLDOWN:
                    pi.set_pull_up_down(self._number, pigpio.PUD_DOWN)

            self._gpio_obj = {"pi": pi}
        except ImportError as e:
            raise HardwareNotFoundError(
                device="pigpio",
                details="Install with: pip install pigpio",
            ) from e

    def _setup_simulation(self) -> None:
        """Setup in simulation mode."""
        self._gpio_obj = {"simulated": True, "value": self._initial == PinState.HIGH}

    def read(self) -> bool:
        """Read the pin state."""
        if not self._initialized:
            self.setup()

        value: bool
        if self._backend == GPIOBackend.GPIOZERO:
            if hasattr(self._gpio_obj, "value"):
                value = bool(self._gpio_obj.value)
            else:
                value = bool(self._gpio_obj.is_active)
        elif self._backend == GPIOBackend.LGPIO:
            import lgpio

            value = bool(lgpio.gpio_read(self._gpio_obj["handle"], self._number))
        elif self._backend == GPIOBackend.RPI_GPIO:
            value = bool(self._gpio_obj["GPIO"].input(self._number))
        elif self._backend == GPIOBackend.PIGPIO:
            value = bool(self._gpio_obj["pi"].read(self._number))
        else:
            value = self._gpio_obj.get("value", False)

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

        if self._backend == GPIOBackend.GPIOZERO:
            if value:
                self._gpio_obj.on()
            else:
                self._gpio_obj.off()
        elif self._backend == GPIOBackend.LGPIO:
            import lgpio

            lgpio.gpio_write(self._gpio_obj["handle"], self._number, 1 if value else 0)
        elif self._backend == GPIOBackend.RPI_GPIO:
            self._gpio_obj["GPIO"].output(self._number, value)
        elif self._backend == GPIOBackend.PIGPIO:
            self._gpio_obj["pi"].write(self._number, 1 if value else 0)
        else:
            self._gpio_obj["value"] = value

        self._state = PinState.HIGH if value else PinState.LOW

    def cleanup(self) -> None:
        """Release pin resources."""
        if not self._initialized:
            return

        try:
            if self._backend == GPIOBackend.GPIOZERO:
                self._gpio_obj.close()
            elif self._backend == GPIOBackend.LGPIO:
                import lgpio

                lgpio.gpio_free(self._gpio_obj["handle"], self._number)
            elif self._backend == GPIOBackend.PIGPIO:
                self._gpio_obj["pi"].stop()
            # RPi.GPIO cleanup is done globally
        except Exception as e:
            logger.warning("Error cleaning up pin %d: %s", self._number, e)
        finally:
            self._initialized = False
            self._gpio_obj = None


class RaspberryPiPWMPin(PWMPin):
    """PWM pin for Raspberry Pi.

    Supports both hardware PWM (GPIO 12, 13, 18, 19) and software PWM.
    """

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        frequency: int = 50,
        duty_cycle: float = 0.0,
        backend: GPIOBackend = GPIOBackend.SIMULATION,
        hardware_pwm: bool | None = None,  # Auto-detect if None
    ) -> None:
        """Initialize a PWM pin.

        Args:
            number: GPIO pin number (BCM)
            name: Optional name
            frequency: PWM frequency in Hz
            duty_cycle: Initial duty cycle (0.0-1.0)
            backend: GPIO backend to use
            hardware_pwm: Force hardware/software PWM. None=auto-detect
        """
        super().__init__(number, name=name, frequency=frequency, duty_cycle=duty_cycle)
        self._backend = backend
        self._pwm_obj: Any = None

        # Auto-detect hardware PWM capability
        if hardware_pwm is None:
            self._hardware_pwm = number in HARDWARE_PWM_PINS_STANDARD
        else:
            self._hardware_pwm = hardware_pwm

    def setup(self) -> None:
        """Initialize PWM output."""
        if self._initialized:
            return

        if self._backend == GPIOBackend.GPIOZERO:
            self._setup_gpiozero()
        elif self._backend == GPIOBackend.LGPIO:
            self._setup_lgpio()
        elif self._backend == GPIOBackend.RPI_GPIO:
            self._setup_rpi_gpio()
        elif self._backend == GPIOBackend.PIGPIO:
            self._setup_pigpio()
        else:
            self._setup_simulation()

        self._initialized = True

    def _setup_gpiozero(self) -> None:
        """Setup PWM using gpiozero."""
        try:
            from gpiozero import PWMOutputDevice

            self._pwm_obj = PWMOutputDevice(
                self._number,
                frequency=self._frequency,
                initial_value=self._duty_cycle,
            )
        except ImportError as e:
            raise HardwareNotFoundError(
                device="gpiozero",
                details="Install with: pip install gpiozero",
            ) from e

    def _setup_lgpio(self) -> None:
        """Setup PWM using lgpio."""
        try:
            import lgpio

            chip_num = 4 if GPIO_CHIP_PI5.exists() else 0
            handle = lgpio.gpiochip_open(chip_num)
            lgpio.gpio_claim_output(handle, self._number, 0)

            # Start PWM
            duty_percent = int(self._duty_cycle * 100)
            lgpio.tx_pwm(handle, self._number, self._frequency, duty_percent)

            self._pwm_obj = {"handle": handle}
        except ImportError as e:
            raise HardwareNotFoundError(
                device="lgpio",
                details="Install with: pip install lgpio",
            ) from e

    def _setup_rpi_gpio(self) -> None:
        """Setup PWM using RPi.GPIO."""
        try:
            from RPi import GPIO

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._number, GPIO.OUT)
            pwm = GPIO.PWM(self._number, self._frequency)
            pwm.start(self._duty_cycle * 100)
            self._pwm_obj = {"pwm": pwm, "GPIO": GPIO}
        except ImportError as e:
            raise HardwareNotFoundError(
                device="RPi.GPIO",
                details="Install with: pip install RPi.GPIO",
            ) from e

    def _setup_pigpio(self) -> None:
        """Setup PWM using pigpio (hardware PWM if available)."""
        try:
            import pigpio

            pi = pigpio.pi()
            if not pi.connected:
                raise HardwareNotFoundError(
                    device="pigpio daemon",
                    details="Start with: sudo pigpiod",
                )

            if self._hardware_pwm and self._number in HARDWARE_PWM_PINS_STANDARD:
                # Use hardware PWM
                duty = int(self._duty_cycle * 1_000_000)
                pi.hardware_PWM(self._number, self._frequency, duty)
            else:
                # Use software PWM
                pi.set_mode(self._number, pigpio.OUTPUT)
                pi.set_PWM_frequency(self._number, self._frequency)
                pi.set_PWM_dutycycle(self._number, int(self._duty_cycle * 255))

            self._pwm_obj = {"pi": pi, "hardware": self._hardware_pwm}
        except ImportError as e:
            raise HardwareNotFoundError(
                device="pigpio",
                details="Install with: pip install pigpio",
            ) from e

    def _setup_simulation(self) -> None:
        """Setup simulated PWM."""
        self._pwm_obj = {
            "simulated": True,
            "frequency": self._frequency,
            "duty_cycle": self._duty_cycle,
        }

    @property
    def frequency(self) -> int:
        """Get PWM frequency."""
        return self._frequency

    @frequency.setter
    def frequency(self, value: int) -> None:
        """Set PWM frequency."""
        self._frequency = value
        if self._initialized:
            self._update_frequency()

    def _update_frequency(self) -> None:
        """Update the PWM frequency on hardware."""
        if self._backend == GPIOBackend.GPIOZERO:
            self._pwm_obj.frequency = self._frequency
        elif self._backend == GPIOBackend.LGPIO:
            import lgpio

            duty_percent = int(self._duty_cycle * 100)
            lgpio.tx_pwm(self._pwm_obj["handle"], self._number, self._frequency, duty_percent)
        elif self._backend == GPIOBackend.RPI_GPIO:
            self._pwm_obj["pwm"].ChangeFrequency(self._frequency)
        elif self._backend == GPIOBackend.PIGPIO:
            if self._pwm_obj.get("hardware"):
                duty = int(self._duty_cycle * 1_000_000)
                self._pwm_obj["pi"].hardware_PWM(self._number, self._frequency, duty)
            else:
                self._pwm_obj["pi"].set_PWM_frequency(self._number, self._frequency)
        else:
            self._pwm_obj["frequency"] = self._frequency

    @property
    def duty_cycle(self) -> float:
        """Get duty cycle (0.0-1.0)."""
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        """Set duty cycle (0.0-1.0)."""
        self._duty_cycle = max(0.0, min(1.0, value))
        if self._initialized:
            self._update_duty_cycle()

    def _update_duty_cycle(self) -> None:
        """Update the PWM duty cycle on hardware."""
        if self._backend == GPIOBackend.GPIOZERO:
            self._pwm_obj.value = self._duty_cycle
        elif self._backend == GPIOBackend.LGPIO:
            import lgpio

            duty_percent = int(self._duty_cycle * 100)
            lgpio.tx_pwm(self._pwm_obj["handle"], self._number, self._frequency, duty_percent)
        elif self._backend == GPIOBackend.RPI_GPIO:
            self._pwm_obj["pwm"].ChangeDutyCycle(self._duty_cycle * 100)
        elif self._backend == GPIOBackend.PIGPIO:
            if self._pwm_obj.get("hardware"):
                duty = int(self._duty_cycle * 1_000_000)
                self._pwm_obj["pi"].hardware_PWM(self._number, self._frequency, duty)
            else:
                self._pwm_obj["pi"].set_PWM_dutycycle(self._number, int(self._duty_cycle * 255))
        else:
            self._pwm_obj["duty_cycle"] = self._duty_cycle

    def set_pulse_width(self, width_us: float) -> None:
        """Set PWM by pulse width in microseconds.

        Useful for servo control (typically 500-2500us).
        """
        period_us = 1_000_000 / self._frequency
        self.duty_cycle = width_us / period_us

    def set_duty_cycle(self, duty: float) -> None:
        """Set the PWM duty cycle (0.0-1.0)."""
        self.duty_cycle = duty

    def set_frequency(self, frequency: int) -> None:
        """Set the PWM frequency in Hz."""
        self.frequency = frequency

    def start(self) -> None:
        """Start PWM output."""
        if not self._initialized:
            self.setup()

    def stop(self) -> None:
        """Stop PWM output."""
        # Set duty cycle to 0 to stop PWM
        self.duty_cycle = 0.0

    def cleanup(self) -> None:
        """Stop PWM and release resources."""
        if not self._initialized:
            return

        try:
            if self._backend == GPIOBackend.GPIOZERO:
                self._pwm_obj.close()
            elif self._backend == GPIOBackend.LGPIO:
                import lgpio

                lgpio.tx_pwm(self._pwm_obj["handle"], self._number, 0, 0)
                lgpio.gpio_free(self._pwm_obj["handle"], self._number)
            elif self._backend == GPIOBackend.RPI_GPIO:
                self._pwm_obj["pwm"].stop()
            elif self._backend == GPIOBackend.PIGPIO:
                if self._pwm_obj.get("hardware"):
                    self._pwm_obj["pi"].hardware_PWM(self._number, 0, 0)
                else:
                    self._pwm_obj["pi"].set_PWM_dutycycle(self._number, 0)
                self._pwm_obj["pi"].stop()
        except Exception as e:
            logger.warning("Error cleaning up PWM pin %d: %s", self._number, e)
        finally:
            self._initialized = False
            self._pwm_obj = None


# =============================================================================
# Raspberry Pi Platform
# =============================================================================


class RaspberryPiPlatform(BasePlatform):
    """Raspberry Pi platform implementation.

    Provides access to GPIO, I2C, SPI, and UART on all Raspberry Pi models.

    Example:
        >>> platform = RaspberryPiPlatform()
        >>> print(f"Model: {platform.model}")
        >>>
        >>> # GPIO
        >>> led = platform.get_pin(17, mode=PinMode.OUTPUT)
        >>> led.high()
        >>>
        >>> # I2C
        >>> i2c = platform.get_bus("i2c", bus=1)
        >>> devices = i2c.scan()
        >>>
        >>> # SPI
        >>> spi = platform.get_bus("spi", bus=0, device=0)
        >>>
        >>> platform.cleanup()
    """

    def __init__(
        self,
        config: PlatformConfig | None = None,
        *,
        backend: GPIOBackend | None = None,
        numbering: PinNumbering = PinNumbering.BCM,
    ) -> None:
        """Initialize Raspberry Pi platform.

        Args:
            config: Platform configuration
            backend: GPIO backend to use. None = auto-detect
            numbering: Pin numbering scheme (BCM or BOARD)
        """
        # Create default config for RPi
        if config is None:
            config = PlatformConfig(
                name="Raspberry Pi",
                platform_type=PlatformType.RASPBERRY_PI,
                pin_numbering=numbering.value,
            )

        super().__init__(config)

        self._numbering = numbering
        self._backend = backend or self._detect_backend()
        self._model: str | None = None

        logger.info("Raspberry Pi platform initialized with backend: %s", self._backend.value)

    def _detect_backend(self) -> GPIOBackend:
        """Detect the best available GPIO backend.

        Priority:
        1. lgpio (Pi 5 compatible, recommended for new code)
        2. gpiozero (high-level, easy to use)
        3. RPi.GPIO (legacy, widely used)
        4. pigpio (if daemon running)
        5. simulation (fallback)
        """
        # Check if we're on simulation mode
        if os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes"):
            return GPIOBackend.SIMULATION

        # Pi 5 requires lgpio
        if GPIO_CHIP_PI5.exists():
            try:
                import lgpio

                return GPIOBackend.LGPIO
            except ImportError:
                pass

        # Try gpiozero (wraps multiple backends)
        try:
            from gpiozero import Device  # noqa: F401

            return GPIOBackend.GPIOZERO
        except ImportError:
            pass

        # Try lgpio
        try:
            import lgpio  # noqa: F401

            return GPIOBackend.LGPIO
        except ImportError:
            pass

        # Try RPi.GPIO (not Pi 5 compatible)
        try:
            import RPi.GPIO  # noqa: F401

            return GPIOBackend.RPI_GPIO
        except ImportError:
            pass

        # Try pigpio
        try:
            import pigpio

            pi = pigpio.pi()
            if pi.connected:
                pi.stop()
                return GPIOBackend.PIGPIO
        except (ImportError, Exception):
            pass

        logger.warning("No GPIO backend available, using simulation")
        return GPIOBackend.SIMULATION

    @property
    def is_available(self) -> bool:
        """Check if Raspberry Pi hardware is available."""
        # Check device tree model
        model_path = Path("/sys/firmware/devicetree/base/model")
        if model_path.exists():
            try:
                model = model_path.read_text().strip("\x00")
                return "Raspberry Pi" in model
            except OSError:
                pass

        # Check /proc/cpuinfo
        cpuinfo_path = Path("/proc/cpuinfo")
        if cpuinfo_path.exists():
            try:
                content = cpuinfo_path.read_text()
                return "BCM" in content or "Raspberry Pi" in content
            except OSError:
                pass

        return False

    @property
    def model(self) -> str:
        """Get the Raspberry Pi model name."""
        if self._model is None:
            self._model = self._detect_model()
        return self._model

    def _detect_model(self) -> str:
        """Detect the Raspberry Pi model."""
        # Try device tree first
        model_path = Path("/sys/firmware/devicetree/base/model")
        if model_path.exists():
            try:
                model = model_path.read_text().strip("\x00")
                for full_name, short_name in PI_MODELS.items():
                    if full_name in model:
                        return short_name
                return model
            except OSError:
                pass

        # Fallback to /proc/cpuinfo
        cpuinfo_path = Path("/proc/cpuinfo")
        if cpuinfo_path.exists():
            try:
                content = cpuinfo_path.read_text()
                for line in content.split("\n"):
                    if line.startswith("Model"):
                        model = line.split(":", 1)[1].strip()
                        for full_name, short_name in PI_MODELS.items():
                            if full_name in model:
                                return short_name
                        return model
            except OSError:
                pass

        return "Unknown"

    @property
    def backend(self) -> GPIOBackend:
        """Get the GPIO backend being used."""
        return self._backend

    @property
    def is_pi5(self) -> bool:
        """Check if running on Raspberry Pi 5."""
        return "5" in self.model or GPIO_CHIP_PI5.exists()

    def _detect_info(self) -> PlatformInfo:
        """Detect platform information."""
        capabilities: set[PlatformCapability] = set()

        # All Pi models have GPIO
        capabilities.add(PlatformCapability.GPIO)
        capabilities.add(PlatformCapability.PWM)

        # Check for hardware PWM
        if GPIO_CHIP_LEGACY.exists() or GPIO_CHIP_PI5.exists():
            capabilities.add(PlatformCapability.HARDWARE_PWM)

        # Check for I2C
        i2c_buses: list[int] = []
        for i in range(10):
            if Path(f"/dev/i2c-{i}").exists():
                i2c_buses.append(i)
                capabilities.add(PlatformCapability.I2C)

        # Check for SPI
        spi_buses: list[tuple[int, int]] = []
        for bus in range(2):
            for dev in range(3):
                if Path(f"/dev/spidev{bus}.{dev}").exists():
                    spi_buses.append((bus, dev))
                    capabilities.add(PlatformCapability.SPI)

        # Check for UART
        uart_ports: list[str] = []
        for port in ["/dev/ttyAMA0", "/dev/ttyS0", "/dev/serial0"]:
            if Path(port).exists():
                uart_ports.append(port)
                capabilities.add(PlatformCapability.UART)

        # Check for camera
        if Path("/dev/video0").exists():
            capabilities.add(PlatformCapability.CAMERA_USB)
        # CSI camera detection is more complex (v4l2)

        # Get GPIO chips
        gpio_chips: list[str] = []
        for chip_path in Path("/dev").glob("gpiochip*"):
            gpio_chips.append(chip_path.name)

        # Get revision and serial
        revision = ""
        serial = ""
        cpuinfo_path = Path("/proc/cpuinfo")
        if cpuinfo_path.exists():
            try:
                content = cpuinfo_path.read_text()
                for line in content.split("\n"):
                    if line.startswith("Revision"):
                        revision = line.split(":", 1)[1].strip()
                    elif line.startswith("Serial"):
                        serial = line.split(":", 1)[1].strip()
            except OSError:
                pass

        return PlatformInfo(
            platform_type=PlatformType.RASPBERRY_PI,
            model=self.model,
            revision=revision,
            serial=serial,
            capabilities=capabilities,
            gpio_chips=gpio_chips,
            i2c_buses=i2c_buses,
            spi_buses=spi_buses,
            uart_ports=uart_ports,
        )

    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create a Raspberry Pi GPIO pin."""
        pin_num = int(pin_id) if isinstance(pin_id, str) else pin_id
        mode = kwargs.get("mode", PinMode.OUTPUT)

        if mode == PinMode.PWM:
            return RaspberryPiPWMPin(
                pin_num,
                name=kwargs.get("name"),
                frequency=kwargs.get("frequency", 50),
                duty_cycle=kwargs.get("duty_cycle", 0.0),
                backend=self._backend,
                hardware_pwm=kwargs.get("hardware_pwm"),
            )
        else:
            initial = kwargs.get("initial", PinState.LOW)
            if isinstance(initial, bool):
                initial = PinState.HIGH if initial else PinState.LOW

            return RaspberryPiDigitalPin(
                pin_num,
                mode=mode,
                name=kwargs.get("name"),
                inverted=kwargs.get("inverted", False),
                initial=initial,
                backend=self._backend,
                numbering=self._numbering,
            )

    def _create_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Create a communication bus."""
        bus_type_lower = bus_type.lower()

        if bus_type_lower == "i2c":
            return self._create_i2c_bus(**kwargs)
        elif bus_type_lower == "spi":
            return self._create_spi_bus(**kwargs)
        elif bus_type_lower in ("uart", "serial"):
            return self._create_uart_bus(**kwargs)
        else:
            raise HardwareNotFoundError(
                device=f"Bus type: {bus_type}",
                details="Supported: i2c, spi, uart",
            )

    def _create_i2c_bus(self, **kwargs: Any) -> Bus:
        """Create an I2C bus.

        Uses smbus2 if available, falls back to simulated.
        """
        from robo_infra.core.bus import I2CConfig, SimulatedI2CBus

        bus_num = kwargs.get("bus", 1)
        config = I2CConfig(bus_number=bus_num)

        # Try to use real smbus2 implementation
        if self._backend != GPIOBackend.SIMULATION:
            try:
                from robo_infra.core.bus import SMBus2I2CBus

                return SMBus2I2CBus(config=config)
            except (ImportError, AttributeError):
                logger.warning("smbus2 not available, using simulated I2C")

        return SimulatedI2CBus(config=config)

    def _create_spi_bus(self, **kwargs: Any) -> Bus:
        """Create an SPI bus.

        Uses spidev if available, falls back to simulated.
        """
        from robo_infra.core.bus import SimulatedSPIBus, SPIConfig

        bus_num = kwargs.get("bus", 0)
        device = kwargs.get("device", 0)
        config = SPIConfig(bus=bus_num, device=device)

        # Try to use real spidev implementation
        if self._backend != GPIOBackend.SIMULATION:
            try:
                from robo_infra.core.bus import SpiDevSPIBus

                return SpiDevSPIBus(config=config)
            except (ImportError, AttributeError):
                logger.warning("spidev not available, using simulated SPI")

        return SimulatedSPIBus(config=config)

    def _create_uart_bus(self, **kwargs: Any) -> Bus:
        """Create a UART/Serial bus.

        Uses pyserial if available, falls back to simulated.
        """
        from robo_infra.core.bus import SerialConfig, SimulatedSerialBus

        port = kwargs.get("port", "/dev/ttyAMA0")
        baudrate = kwargs.get("baudrate", 9600)
        config = SerialConfig(port=port, baudrate=baudrate)

        # Try to use real pyserial implementation
        if self._backend != GPIOBackend.SIMULATION:
            try:
                from robo_infra.core.bus import PySerialBus

                return PySerialBus(config=config)
            except (ImportError, AttributeError):
                logger.warning("pyserial not available, using simulated Serial")

        return SimulatedSerialBus(config=config)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Constants
    "HARDWARE_PWM_PINS_PI5",
    "HARDWARE_PWM_PINS_STANDARD",
    "PI_MODELS",
    # Enums
    "GPIOBackend",
    "PinNumbering",
    # Pin classes
    "RaspberryPiDigitalPin",
    "RaspberryPiPWMPin",
    # Platform
    "RaspberryPiPlatform",
]
