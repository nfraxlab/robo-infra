"""Generic Linux SBC platform implementation using libgpiod.

Supports any Linux single-board computer with GPIO access via:
- libgpiod (modern character device interface)
- sysfs (legacy, deprecated but widely supported)

This provides a fallback platform for boards not explicitly supported:
- Orange Pi (all variants)
- Rock Pi 4, Rock 5B
- Pine64 (Pine A64, Rock64, ROCKPro64)
- Banana Pi
- Odroid (C4, N2, XU4)
- NanoPi
- Khadas VIM
- Libre Computer boards
- Any other Linux SBC with /dev/gpiochip*

Example:
    >>> from robo_infra.platforms.linux_generic import LinuxGenericPlatform
    >>>
    >>> # Auto-detect GPIO chips
    >>> platform = LinuxGenericPlatform()
    >>>
    >>> # Get a GPIO pin (chip 0, line 17)
    >>> led = platform.get_pin(17, chip=0, mode=PinMode.OUTPUT)
    >>> led.high()
    >>>
    >>> # Or use line name if available
    >>> button = platform.get_pin("GPIO17", mode=PinMode.INPUT)
    >>>
    >>> platform.cleanup()
"""

from __future__ import annotations

import contextlib
import logging
import os
from dataclasses import dataclass, field
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


class LinuxSBCType(Enum):
    """Known Linux SBC types."""

    ORANGE_PI = "Orange Pi"
    ORANGE_PI_ZERO = "Orange Pi Zero"
    ORANGE_PI_5 = "Orange Pi 5"
    ROCK_PI_4 = "Rock Pi 4"
    ROCK_5B = "Rock 5B"
    PINE64 = "Pine A64"
    ROCK64 = "Rock64"
    ROCKPRO64 = "ROCKPro64"
    BANANA_PI = "Banana Pi"
    ODROID_C4 = "Odroid C4"
    ODROID_N2 = "Odroid N2"
    NANOPI = "NanoPi"
    KHADAS_VIM = "Khadas VIM"
    LIBRE_COMPUTER = "Libre Computer"
    GENERIC = "Generic Linux SBC"


class GPIOBackend(Enum):
    """Available GPIO backends."""

    GPIOD = "gpiod"
    SYSFS = "sysfs"
    SIMULATION = "simulation"


class GPIOEdge(Enum):
    """Edge detection types for GPIO events."""

    NONE = "none"
    RISING = "rising"
    FALLING = "falling"
    BOTH = "both"


# sysfs paths
SYSFS_GPIO_PATH = Path("/sys/class/gpio")
SYSFS_EXPORT = SYSFS_GPIO_PATH / "export"
SYSFS_UNEXPORT = SYSFS_GPIO_PATH / "unexport"

# gpiochip path
GPIOCHIP_PATH = Path("/dev")

# PWM sysfs path
PWM_SYSFS_PATH = Path("/sys/class/pwm")


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class GPIOChipInfo:
    """Information about a GPIO chip."""

    name: str
    label: str
    num_lines: int
    path: Path
    chip_id: int


@dataclass
class GPIOLineInfo:
    """Information about a GPIO line."""

    offset: int
    name: str
    consumer: str
    direction: str
    active_low: bool
    used: bool


@dataclass
class LinuxSBCCapabilities:
    """Capabilities for a Linux SBC."""

    sbc_type: LinuxSBCType
    gpio_chips: list[GPIOChipInfo] = field(default_factory=list)
    pwm_chips: list[str] = field(default_factory=list)
    i2c_buses: list[int] = field(default_factory=list)
    spi_buses: list[int] = field(default_factory=list)
    uart_ports: list[str] = field(default_factory=list)
    description: str = ""


# =============================================================================
# Pin Classes
# =============================================================================


class LinuxDigitalPin(DigitalPin):
    """Linux GPIO pin implementation using libgpiod or sysfs."""

    def __init__(
        self,
        line: int,
        chip: int = 0,
        mode: PinMode = PinMode.INPUT,
        backend: GPIOBackend = GPIOBackend.SIMULATION,
        pull: str | None = None,
        initial: PinState | None = None,
        active_low: bool = False,
        line_name: str | None = None,
    ) -> None:
        """Initialize Linux GPIO pin.

        Args:
            line: GPIO line number (offset within chip)
            chip: GPIO chip number (default 0)
            mode: Pin mode (INPUT or OUTPUT)
            backend: GPIO backend to use
            pull: Pull resistor configuration ("up", "down", or None)
            initial: Initial state for output pins
            active_low: Invert logic level
            line_name: Optional line name for identification
        """
        self._line = line
        self._chip = chip
        self._backend = backend
        self._pull = pull
        self._active_low = active_low
        self._line_name = line_name or f"gpio{chip}_{line}"

        # Backend handles
        self._gpiod_line: Any = None
        self._gpiod_chip: Any = None
        self._sysfs_path: Path | None = None

        # Call parent with proper arguments
        super().__init__(
            number=line,
            mode=mode,
            name=self._line_name,
            inverted=active_low,
            initial=initial or PinState.LOW,
        )

    def _setup_hardware(self) -> None:
        """Setup hardware GPIO."""
        if self._backend == GPIOBackend.GPIOD:
            self._setup_gpiod()
        elif self._backend == GPIOBackend.SYSFS:
            self._setup_sysfs()

    def _setup_gpiod(self) -> None:
        """Setup using libgpiod."""
        try:
            import gpiod

            chip_path = f"/dev/gpiochip{self._chip}"
            self._gpiod_chip = gpiod.Chip(chip_path)
            self._gpiod_line = self._gpiod_chip.get_line(self._line)

            flags = 0
            if self._active_low:
                flags |= gpiod.LINE_REQ_FLAG_ACTIVE_LOW

            if self._pull == "up":
                flags |= gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
            elif self._pull == "down":
                flags |= gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN

            if self.mode == PinMode.OUTPUT:
                self._gpiod_line.request(
                    consumer="robo_infra",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    flags=flags,
                    default_val=1 if self._initial == PinState.HIGH else 0,
                )
            else:
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
        except Exception as e:
            raise HardwareNotFoundError(
                device=f"GPIO chip{self._chip} line{self._line}",
                details=str(e),
            ) from e

    def _setup_sysfs(self) -> None:
        """Setup using sysfs (legacy)."""
        # Calculate global GPIO number (this is board-specific)
        gpio_num = self._line  # For generic boards, assume line == gpio number

        self._sysfs_path = SYSFS_GPIO_PATH / f"gpio{gpio_num}"

        # Export GPIO if not already exported
        if not self._sysfs_path.exists():
            try:
                with open(SYSFS_EXPORT, "w") as f:
                    f.write(str(gpio_num))
            except OSError as e:
                raise HardwareNotFoundError(
                    device=f"GPIO {gpio_num}",
                    details=f"Failed to export: {e}",
                ) from e

        # Set direction
        direction_path = self._sysfs_path / "direction"
        try:
            with open(direction_path, "w") as f:
                if self.mode == PinMode.OUTPUT:
                    # Set direction with initial value
                    if self._initial == PinState.HIGH:
                        f.write("high")
                    else:
                        f.write("low")
                else:
                    f.write("in")
        except OSError as e:
            raise HardwareNotFoundError(
                device=f"GPIO {gpio_num}",
                details=f"Failed to set direction: {e}",
            ) from e

        # Set active_low
        if self._active_low:
            active_low_path = self._sysfs_path / "active_low"
            with contextlib.suppress(OSError), open(active_low_path, "w") as f:
                f.write("1")

    def setup(self) -> None:
        """Initialize the pin hardware."""
        if self._initialized:
            return

        if self._backend != GPIOBackend.SIMULATION:
            self._setup_hardware()
        else:
            logger.debug("Simulation: Setting up GPIO line %d", self._line)

        self._initialized = True

    def _write_state(self, value: bool) -> None:
        """Write state to hardware."""
        hw_value = 1 if value else 0

        if self._gpiod_line is not None:
            self._gpiod_line.set_value(hw_value)
        elif self._sysfs_path is not None:
            value_path = self._sysfs_path / "value"
            with open(value_path, "w") as f:
                f.write(str(hw_value))

        self._state = PinState.HIGH if value else PinState.LOW

    def _read_state(self) -> bool:
        """Read state from hardware."""
        if self._gpiod_line is not None:
            value = self._gpiod_line.get_value()
            return bool(value)
        elif self._sysfs_path is not None:
            value_path = self._sysfs_path / "value"
            with open(value_path) as f:
                value = int(f.read().strip())
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
        if self._backend == GPIOBackend.SIMULATION:
            raw = self._state == PinState.HIGH
        else:
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
        if self._gpiod_line is not None:
            self._gpiod_line.release()
        elif self._sysfs_path is not None:
            # Unexport GPIO
            gpio_num = self._line
            with contextlib.suppress(OSError), open(SYSFS_UNEXPORT, "w") as f:
                f.write(str(gpio_num))
        self._initialized = False

    @property
    def line(self) -> int:
        """Get GPIO line number."""
        return self._line

    @property
    def chip(self) -> int:
        """Get GPIO chip number."""
        return self._chip

    @property
    def line_name(self) -> str:
        """Get line name."""
        return self._line_name


class LinuxPWMPin(PWMPin):
    """Linux PWM pin using sysfs PWM interface."""

    def __init__(
        self,
        chip: int,
        channel: int,
        frequency: int = 1000,
        duty_cycle: float = 0.0,
        backend: GPIOBackend = GPIOBackend.SIMULATION,
    ) -> None:
        """Initialize Linux PWM pin.

        Args:
            chip: PWM chip number (e.g., 0 for pwmchip0)
            channel: PWM channel within chip
            frequency: PWM frequency in Hz
            duty_cycle: Initial duty cycle (0.0-1.0)
            backend: Backend (only SYSFS and SIMULATION supported)
        """
        self._chip = chip
        self._channel = channel
        self._backend = backend
        self._running = False
        self._pwm_path: Path | None = None
        self._period_ns = int(1_000_000_000 / frequency)  # Period in nanoseconds

        # Call parent with proper arguments
        super().__init__(
            number=channel,
            name=f"pwmchip{chip}_pwm{channel}",
            frequency=frequency,
            duty_cycle=duty_cycle,
        )

    def _setup_hardware(self) -> None:
        """Setup hardware PWM."""
        chip_path = PWM_SYSFS_PATH / f"pwmchip{self._chip}"
        if not chip_path.exists():
            raise HardwareNotFoundError(
                device=f"PWM chip {self._chip}",
                details=f"Path {chip_path} does not exist",
            )

        self._pwm_path = chip_path / f"pwm{self._channel}"

        # Export PWM channel if not already exported
        if not self._pwm_path.exists():
            export_path = chip_path / "export"
            try:
                with open(export_path, "w") as f:
                    f.write(str(self._channel))
            except OSError as e:
                raise HardwareNotFoundError(
                    device=f"PWM chip{self._chip} channel{self._channel}",
                    details=f"Failed to export: {e}",
                ) from e

        # Set period
        self._set_period(self._period_ns)

        # Set initial duty cycle
        self._set_duty_cycle_ns(int(self._duty_cycle * self._period_ns))

    def setup(self) -> None:
        """Initialize the PWM pin hardware."""
        if self._initialized:
            return

        if self._backend != GPIOBackend.SIMULATION:
            self._setup_hardware()
        else:
            logger.debug("Simulation: Setting up PWM chip%d/pwm%d", self._chip, self._channel)

        self._initialized = True

    def _set_period(self, period_ns: int) -> None:
        """Set PWM period in nanoseconds."""
        if self._pwm_path is None:
            return
        period_path = self._pwm_path / "period"
        with open(period_path, "w") as f:
            f.write(str(period_ns))

    def _set_duty_cycle_ns(self, duty_ns: int) -> None:
        """Set PWM duty cycle in nanoseconds."""
        if self._pwm_path is None:
            return
        duty_path = self._pwm_path / "duty_cycle"
        with open(duty_path, "w") as f:
            f.write(str(duty_ns))

    def set_duty_cycle(self, duty: float) -> None:
        """Set the PWM duty cycle.

        Args:
            duty: Duty cycle from 0.0 (0%) to 1.0 (100%)
        """
        if not 0.0 <= duty <= 1.0:
            raise ValueError(f"Duty cycle must be 0.0-1.0, got {duty}")
        self._duty_cycle = duty
        if self._backend != GPIOBackend.SIMULATION:
            self._set_duty_cycle_ns(int(duty * self._period_ns))

    def set_frequency(self, frequency: int) -> None:
        """Set the PWM frequency.

        Args:
            frequency: Frequency in Hz
        """
        if frequency <= 0:
            raise ValueError(f"Frequency must be positive, got {frequency}")
        self._frequency = frequency
        self._period_ns = int(1_000_000_000 / frequency)
        if self._backend != GPIOBackend.SIMULATION:
            # Need to disable, set period, then re-enable
            was_running = self._running
            if was_running:
                self.stop()
            self._set_period(self._period_ns)
            self._set_duty_cycle_ns(int(self._duty_cycle * self._period_ns))
            if was_running:
                self.start()

    def start(self) -> None:
        """Start PWM output."""
        if self._backend == GPIOBackend.SIMULATION:
            self._running = True
            logger.debug("Simulation: PWM chip%d/pwm%d started", self._chip, self._channel)
        elif self._pwm_path is not None:
            enable_path = self._pwm_path / "enable"
            with open(enable_path, "w") as f:
                f.write("1")
            self._running = True

    def stop(self) -> None:
        """Stop PWM output."""
        if self._backend == GPIOBackend.SIMULATION:
            self._running = False
            logger.debug("Simulation: PWM chip%d/pwm%d stopped", self._chip, self._channel)
        elif self._pwm_path is not None:
            enable_path = self._pwm_path / "enable"
            with open(enable_path, "w") as f:
                f.write("0")
            self._running = False

    def cleanup(self) -> None:
        """Release PWM resources."""
        if self._running:
            self.stop()
        # Unexport PWM
        if self._pwm_path is not None:
            chip_path = PWM_SYSFS_PATH / f"pwmchip{self._chip}"
            unexport_path = chip_path / "unexport"
            with contextlib.suppress(OSError), open(unexport_path, "w") as f:
                f.write(str(self._channel))
        self._initialized = False

    @property
    def is_running(self) -> bool:
        """Check if PWM is running."""
        return self._running

    @property
    def chip_id(self) -> int:
        """Get PWM chip ID."""
        return self._chip

    @property
    def channel(self) -> int:
        """Get PWM channel."""
        return self._channel


# =============================================================================
# Platform Class
# =============================================================================


class LinuxGenericPlatform(BasePlatform):
    """Generic Linux SBC platform using libgpiod.

    Provides GPIO access for any Linux SBC with /dev/gpiochip* devices.
    Falls back to sysfs if libgpiod is not available.

    Example:
        >>> platform = LinuxGenericPlatform()
        >>> led = platform.get_pin(17, chip=0, mode=PinMode.OUTPUT)
        >>> led.high()
    """

    def __init__(
        self,
        backend: GPIOBackend | str | None = None,
        config: PlatformConfig | None = None,
        simulation: bool | None = None,
        default_chip: int = 0,
    ) -> None:
        """Initialize Linux Generic platform.

        Args:
            backend: GPIO backend to use (auto-detect if None)
            config: Platform configuration
            simulation: Force simulation mode (auto-detect if None)
            default_chip: Default GPIO chip number
        """
        # Convert string backend to enum
        if isinstance(backend, str):
            backend = GPIOBackend(backend.lower())

        # Detect simulation mode
        if simulation is None:
            simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes")
            if not simulation:
                simulation = not self._has_gpio_chips()

        self._simulation = simulation
        self._backend = backend or (
            GPIOBackend.SIMULATION if simulation else self._detect_backend()
        )
        self._default_chip = default_chip
        self._sbc_type = LinuxSBCType.GENERIC

        # Pin tracking
        self._digital_pins: dict[str, LinuxDigitalPin] = {}
        self._pwm_pins: dict[str, LinuxPWMPin] = {}

        # Discover hardware
        self._gpio_chips: list[GPIOChipInfo] = []
        self._pwm_chips: list[str] = []
        self._line_names: dict[str, tuple[int, int]] = {}  # name -> (chip, line)

        if not simulation:
            self._discover_gpio_chips()
            self._discover_pwm_chips()
            self._sbc_type = self._detect_sbc_type()
        else:
            # Simulated chips
            self._gpio_chips = [
                GPIOChipInfo(
                    name="gpiochip0",
                    label="simulated-gpio",
                    num_lines=32,
                    path=Path("/dev/gpiochip0"),
                    chip_id=0,
                ),
            ]

        # Initialize base class
        base_config = config or PlatformConfig(
            name=f"Linux Generic ({self._sbc_type.value})",
            platform_type=PlatformType.LINUX_GENERIC,
            simulation_fallback=True,
        )
        super().__init__(config=base_config)

        logger.info(
            "Linux Generic platform initialized: type=%s, backend=%s, simulation=%s, chips=%d",
            self._sbc_type.value,
            self._backend.value,
            self._simulation,
            len(self._gpio_chips),
        )

    def _has_gpio_chips(self) -> bool:
        """Check if GPIO chips are available."""
        return any(GPIOCHIP_PATH.glob("gpiochip*"))

    def _detect_backend(self) -> GPIOBackend:
        """Auto-detect available backend."""
        # Try libgpiod first (modern, preferred)
        try:
            import gpiod  # noqa: F401

            return GPIOBackend.GPIOD
        except ImportError:
            pass

        # Fall back to sysfs
        if SYSFS_GPIO_PATH.exists():
            return GPIOBackend.SYSFS

        # Simulation mode
        logger.warning("No GPIO backend available, using simulation mode")
        return GPIOBackend.SIMULATION

    def _discover_gpio_chips(self) -> None:
        """Discover available GPIO chips."""
        if self._backend == GPIOBackend.GPIOD:
            self._discover_gpiod_chips()
        else:
            self._discover_sysfs_chips()

    def _discover_gpiod_chips(self) -> None:
        """Discover GPIO chips using libgpiod."""
        try:
            import gpiod

            for chip_path in sorted(GPIOCHIP_PATH.glob("gpiochip*")):
                try:
                    chip = gpiod.Chip(str(chip_path))
                    chip_id = int(chip_path.name.replace("gpiochip", ""))

                    info = GPIOChipInfo(
                        name=chip_path.name,
                        label=chip.label,
                        num_lines=chip.num_lines,
                        path=chip_path,
                        chip_id=chip_id,
                    )
                    self._gpio_chips.append(info)

                    # Collect line names
                    for i in range(chip.num_lines):
                        line = chip.get_line(i)
                        if line.name:
                            self._line_names[line.name] = (chip_id, i)

                    chip.close()

                except Exception as e:
                    logger.warning("Failed to open GPIO chip %s: %s", chip_path, e)

        except ImportError:
            pass

    def _discover_sysfs_chips(self) -> None:
        """Discover GPIO chips using sysfs."""
        for chip_path in sorted(GPIOCHIP_PATH.glob("gpiochip*")):
            chip_id = int(chip_path.name.replace("gpiochip", ""))

            # Read chip info from sysfs
            sysfs_chip = Path(f"/sys/class/gpio/gpiochip{chip_id}")
            if not sysfs_chip.exists():
                continue

            label = "unknown"
            num_lines = 0

            label_path = sysfs_chip / "label"
            if label_path.exists():
                label = label_path.read_text().strip()

            ngpio_path = sysfs_chip / "ngpio"
            if ngpio_path.exists():
                num_lines = int(ngpio_path.read_text().strip())

            info = GPIOChipInfo(
                name=chip_path.name,
                label=label,
                num_lines=num_lines,
                path=chip_path,
                chip_id=chip_id,
            )
            self._gpio_chips.append(info)

    def _discover_pwm_chips(self) -> None:
        """Discover available PWM chips."""
        for pwm_path in sorted(PWM_SYSFS_PATH.glob("pwmchip*")):
            self._pwm_chips.append(pwm_path.name)

    def _detect_sbc_type(self) -> LinuxSBCType:
        """Detect SBC type from device tree or other sources."""
        model_path = Path("/proc/device-tree/model")
        if model_path.exists():
            try:
                model = model_path.read_text().strip().rstrip("\x00").lower()

                if "orange pi" in model:
                    if "zero" in model:
                        return LinuxSBCType.ORANGE_PI_ZERO
                    if "5" in model:
                        return LinuxSBCType.ORANGE_PI_5
                    return LinuxSBCType.ORANGE_PI
                if "rock pi 4" in model or "rockpi 4" in model:
                    return LinuxSBCType.ROCK_PI_4
                if "rock 5b" in model or "rock5b" in model:
                    return LinuxSBCType.ROCK_5B
                if "pine64" in model or "pine a64" in model:
                    return LinuxSBCType.PINE64
                if "rock64" in model:
                    return LinuxSBCType.ROCK64
                if "rockpro64" in model:
                    return LinuxSBCType.ROCKPRO64
                if "banana pi" in model or "bananapi" in model:
                    return LinuxSBCType.BANANA_PI
                if "odroid" in model:
                    if "c4" in model:
                        return LinuxSBCType.ODROID_C4
                    if "n2" in model:
                        return LinuxSBCType.ODROID_N2
                if "nanopi" in model:
                    return LinuxSBCType.NANOPI
                if "khadas" in model or "vim" in model:
                    return LinuxSBCType.KHADAS_VIM
                if "libre" in model:
                    return LinuxSBCType.LIBRE_COMPUTER

            except Exception:
                pass

        return LinuxSBCType.GENERIC

    # -------------------------------------------------------------------------
    # Platform Interface
    # -------------------------------------------------------------------------

    @property
    def platform_type(self) -> PlatformType:
        """Get platform type."""
        return PlatformType.LINUX_GENERIC

    @property
    def capabilities(self) -> set[PlatformCapability]:
        """Get platform capabilities."""
        caps = {PlatformCapability.GPIO}

        if self._pwm_chips:
            caps.add(PlatformCapability.PWM)

        # Check for I2C buses
        if any(Path(f"/dev/i2c-{i}").exists() for i in range(10)):
            caps.add(PlatformCapability.I2C)

        # Check for SPI buses
        if any(Path(f"/dev/spidev{i}.{j}").exists() for i in range(3) for j in range(3)):
            caps.add(PlatformCapability.SPI)

        # Check for UART
        if any(Path(f"/dev/ttyS{i}").exists() for i in range(5)) or any(
            Path(f"/dev/ttyAMA{i}").exists() for i in range(5)
        ):
            caps.add(PlatformCapability.UART)

        return caps

    def get_info(self) -> PlatformInfo:
        """Get platform information."""
        gpio_chips = [c.name for c in self._gpio_chips]
        i2c_buses = [i for i in range(10) if Path(f"/dev/i2c-{i}").exists()]
        spi_buses = [
            (i, j) for i in range(3) for j in range(3) if Path(f"/dev/spidev{i}.{j}").exists()
        ]
        uart_ports = [f"/dev/ttyS{i}" for i in range(5) if Path(f"/dev/ttyS{i}").exists()]

        return PlatformInfo(
            platform_type=PlatformType.LINUX_GENERIC,
            model=f"Linux Generic ({self._sbc_type.value})",
            capabilities=self.capabilities,
            gpio_chips=gpio_chips if gpio_chips else ["gpiochip0"],
            i2c_buses=i2c_buses if i2c_buses else [1],
            spi_buses=spi_buses if spi_buses else [(0, 0)],
            uart_ports=uart_ports if uart_ports else ["/dev/ttyS0"],
        )

    # -------------------------------------------------------------------------
    # GPIO Chip Information
    # -------------------------------------------------------------------------

    def list_gpio_chips(self) -> list[GPIOChipInfo]:
        """List all GPIO chips."""
        return self._gpio_chips.copy()

    def get_gpio_chip_info(self, chip: int) -> GPIOChipInfo | None:
        """Get info for a specific GPIO chip."""
        for info in self._gpio_chips:
            if info.chip_id == chip:
                return info
        return None

    def list_gpio_lines(self, chip: int = 0) -> list[GPIOLineInfo]:
        """List all lines on a GPIO chip."""
        if self._simulation:
            # Simulated lines
            return [
                GPIOLineInfo(
                    offset=i,
                    name=f"line{i}",
                    consumer="",
                    direction="input",
                    active_low=False,
                    used=False,
                )
                for i in range(32)
            ]

        if self._backend != GPIOBackend.GPIOD:
            return []

        try:
            import gpiod

            chip_path = f"/dev/gpiochip{chip}"
            gpiod_chip = gpiod.Chip(chip_path)
            lines = []

            for i in range(gpiod_chip.num_lines):
                line = gpiod_chip.get_line(i)
                lines.append(
                    GPIOLineInfo(
                        offset=i,
                        name=line.name or "",
                        consumer=line.consumer or "",
                        direction="output"
                        if line.direction == gpiod.Line.DIRECTION_OUTPUT
                        else "input",
                        active_low=line.is_active_low,
                        used=line.is_used,
                    )
                )

            gpiod_chip.close()
            return lines

        except Exception as e:
            logger.warning("Failed to list GPIO lines: %s", e)
            return []

    # -------------------------------------------------------------------------
    # Pin Access
    # -------------------------------------------------------------------------

    def get_pin(
        self,
        pin_id: int | str,
        chip: int | None = None,
        mode: PinMode = PinMode.INPUT,
        pull: str | None = None,
        initial: PinState | None = None,
        active_low: bool = False,
    ) -> LinuxDigitalPin:
        """Get a digital GPIO pin.

        Args:
            pin_id: GPIO line number or line name
            chip: GPIO chip number (uses default if None)
            mode: Pin mode (INPUT or OUTPUT)
            pull: Pull resistor ("up", "down", or None)
            initial: Initial state for output pins
            active_low: Invert logic level

        Returns:
            LinuxDigitalPin instance
        """
        if chip is None:
            chip = self._default_chip

        # Resolve line name to chip/line
        if isinstance(pin_id, str):
            if pin_id in self._line_names:
                chip, line = self._line_names[pin_id]
            else:
                # Try to parse as number
                try:
                    line = int(pin_id)
                except ValueError:
                    raise ValueError(f"Unknown GPIO line name: {pin_id}") from None
        else:
            line = pin_id

        # Cache key
        cache_key = f"chip{chip}_line{line}_{mode.value}"
        if cache_key in self._digital_pins:
            return self._digital_pins[cache_key]

        pin = LinuxDigitalPin(
            line=line,
            chip=chip,
            mode=mode,
            backend=self._backend,
            pull=pull,
            initial=initial,
            active_low=active_low,
        )
        self._digital_pins[cache_key] = pin
        return pin

    def get_pwm_pin(
        self,
        chip: int,
        channel: int,
        frequency: int = 1000,
        duty_cycle: float = 0.0,
    ) -> LinuxPWMPin:
        """Get a PWM pin.

        Args:
            chip: PWM chip number (pwmchipN)
            channel: PWM channel within chip
            frequency: PWM frequency in Hz
            duty_cycle: Initial duty cycle (0.0-1.0)

        Returns:
            LinuxPWMPin instance
        """
        cache_key = f"pwm_chip{chip}_ch{channel}"
        if cache_key in self._pwm_pins:
            return self._pwm_pins[cache_key]

        pin = LinuxPWMPin(
            chip=chip,
            channel=channel,
            frequency=frequency,
            duty_cycle=duty_cycle,
            backend=self._backend,
        )
        self._pwm_pins[cache_key] = pin
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
            from robo_infra.core.bus import I2CBus, SPIBus, UARTBus

            if bus_type.lower() == "i2c":
                bus_num = kwargs.get("bus", 1)
                return I2CBus(bus_number=bus_num, simulation=True)
            elif bus_type.lower() == "spi":
                bus_num = kwargs.get("bus", 0)
                device = kwargs.get("device", 0)
                return SPIBus(bus_number=bus_num, device=device, simulation=True)
            elif bus_type.lower() == "uart":
                port = kwargs.get("port", "/dev/ttyS0")
                baudrate = kwargs.get("baudrate", 115200)
                return UARTBus(port=port, baudrate=baudrate, simulation=True)
            else:
                raise ValueError(f"Unknown bus type: {bus_type}")

        raise NotImplementedError("Hardware bus access not yet implemented")

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def sbc_type(self) -> LinuxSBCType:
        """Get detected SBC type."""
        return self._sbc_type

    @property
    def backend(self) -> GPIOBackend:
        """Get active GPIO backend."""
        return self._backend

    @property
    def is_simulation(self) -> bool:
        """Check if running in simulation mode."""
        return self._simulation

    @property
    def gpio_chips(self) -> list[GPIOChipInfo]:
        """Get discovered GPIO chips."""
        return self._gpio_chips.copy()

    @property
    def pwm_chips(self) -> list[str]:
        """Get discovered PWM chips."""
        return self._pwm_chips.copy()

    @property
    def default_chip(self) -> int:
        """Get default GPIO chip number."""
        return self._default_chip

    # -------------------------------------------------------------------------
    # Abstract Method Implementations
    # -------------------------------------------------------------------------

    @property
    def is_available(self) -> bool:
        """Check if Linux Generic platform is available."""
        if self._simulation:
            return True
        return self._has_gpio_chips()

    def _detect_info(self) -> PlatformInfo:
        """Detect platform information."""
        gpio_chips = [c.name for c in self._gpio_chips]
        i2c_buses = [i for i in range(10) if Path(f"/dev/i2c-{i}").exists()]
        spi_buses = [
            (i, j) for i in range(3) for j in range(3) if Path(f"/dev/spidev{i}.{j}").exists()
        ]
        uart_ports = [f"/dev/ttyS{i}" for i in range(5) if Path(f"/dev/ttyS{i}").exists()]

        return PlatformInfo(
            platform_type=PlatformType.LINUX_GENERIC,
            model=self._sbc_type.value,
            capabilities=self.capabilities,
            gpio_chips=gpio_chips,
            i2c_buses=i2c_buses,
            spi_buses=spi_buses,
            uart_ports=uart_ports,
        )

    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create a platform-specific pin."""
        chip = kwargs.get("chip", self._default_chip)
        mode = kwargs.get("mode", PinMode.INPUT)
        pull = kwargs.get("pull")
        initial = kwargs.get("initial")
        active_low = kwargs.get("active_low", False)

        return self.get_pin(
            pin_id,
            chip=chip,
            mode=mode,
            pull=pull,
            initial=initial,
            active_low=active_low,
        )

    def _create_bus(self, bus_type: str, **kwargs: Any) -> Bus:
        """Create a platform-specific bus."""
        return self.get_bus(bus_type, **kwargs)

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

        logger.info("Linux Generic platform cleanup complete")
