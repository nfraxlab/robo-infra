"""NVIDIA Jetson platform implementation.

Supports all NVIDIA Jetson models including:
- Jetson Nano, Nano 2GB
- Jetson TX1, TX2
- Jetson Xavier NX, AGX Xavier
- Jetson Orin Nano, Orin NX, AGX Orin

This module provides hardware access via the Jetson.GPIO library,
which is compatible with RPi.GPIO API for easy migration.

Example:
    >>> from robo_infra.platforms.jetson import JetsonPlatform
    >>>
    >>> # Auto-detect platform
    >>> platform = JetsonPlatform()
    >>> print(f"Model: {platform.model}")
    >>> print(f"Power mode: {platform.power_mode}")
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
import subprocess
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


class JetsonPinNumbering(Enum):
    """Pin numbering schemes for NVIDIA Jetson."""

    BOARD = "BOARD"  # Physical board pin numbers (40-pin header)
    BCM = "BCM"  # Broadcom-style numbering (RPi compatible)
    CVM = "CVM"  # Carrier board module numbering
    TEGRA_SOC = "TEGRA_SOC"  # Tegra SoC pin names (e.g., "GPIO_PE6")


class JetsonModel(Enum):
    """NVIDIA Jetson models."""

    # Nano family
    NANO = "Jetson Nano"
    NANO_2GB = "Jetson Nano 2GB"

    # TX family
    TX1 = "Jetson TX1"
    TX2 = "Jetson TX2"
    TX2_NX = "Jetson TX2 NX"

    # Xavier family
    XAVIER_NX = "Jetson Xavier NX"
    AGX_XAVIER = "Jetson AGX Xavier"

    # Orin family
    ORIN_NANO = "Jetson Orin Nano"
    ORIN_NX = "Jetson Orin NX"
    AGX_ORIN = "Jetson AGX Orin"

    UNKNOWN = "Unknown Jetson"


class JetsonPowerMode(Enum):
    """Power modes for Jetson boards."""

    # Nano modes
    MODE_5W = "5W"
    MODE_10W = "10W"

    # TX2 modes
    MODE_15W = "15W"

    # Xavier/Orin modes
    MODE_20W = "20W"
    MODE_30W = "30W"
    MODE_40W = "40W"
    MODE_50W = "50W"

    # Max performance
    MAXN = "MAXN"

    UNKNOWN = "Unknown"


# Model identifiers from /etc/nv_tegra_release or /proc/device-tree/model
JETSON_MODELS = {
    # Nano
    "p3448-0000": JetsonModel.NANO,
    "p3448-0002": JetsonModel.NANO_2GB,
    "jetson-nano": JetsonModel.NANO,
    # TX1
    "p2371-2180": JetsonModel.TX1,
    "jetson-tx1": JetsonModel.TX1,
    # TX2
    "p2771-0000": JetsonModel.TX2,
    "p3489-0000": JetsonModel.TX2_NX,
    "jetson-tx2": JetsonModel.TX2,
    "jetson-tx2-nx": JetsonModel.TX2_NX,
    # Xavier
    "p2888-0001": JetsonModel.AGX_XAVIER,
    "p3668-0001": JetsonModel.XAVIER_NX,
    "jetson-xavier": JetsonModel.AGX_XAVIER,
    "jetson-xavier-nx": JetsonModel.XAVIER_NX,
    # Orin
    "p3767-0000": JetsonModel.ORIN_NANO,
    "p3767-0001": JetsonModel.ORIN_NX,
    "p3701-0000": JetsonModel.AGX_ORIN,
    "jetson-orin-nano": JetsonModel.ORIN_NANO,
    "jetson-orin-nx": JetsonModel.ORIN_NX,
    "jetson-agx-orin": JetsonModel.AGX_ORIN,
}


# Hardware PWM capable pins (varies by model, these are common)
# BCM-style numbering on 40-pin header
HARDWARE_PWM_PINS = {32, 33}  # PWM0, PWM1 on most Jetson boards


# Power mode files
NVPMODEL_PATH = Path("/usr/bin/nvpmodel")
POWER_MODE_FILE = Path("/etc/nvpmodel.conf")


# =============================================================================
# Jetson Pin Classes
# =============================================================================


@dataclass
class JetsonPinConfig:
    """Configuration for a Jetson GPIO pin."""

    pin: int | str  # Pin number or name (TEGRA_SOC)
    numbering: JetsonPinNumbering = JetsonPinNumbering.BOARD
    pull_up_down: str | None = None
    initial: bool | None = None


class JetsonDigitalPin(DigitalPin):
    """Digital GPIO pin for NVIDIA Jetson.

    Uses Jetson.GPIO library which is API-compatible with RPi.GPIO.
    """

    def __init__(
        self,
        number: int,
        mode: PinMode = PinMode.OUTPUT,
        *,
        name: str | None = None,
        inverted: bool = False,
        initial: PinState = PinState.LOW,
        numbering: JetsonPinNumbering = JetsonPinNumbering.BOARD,
        simulation: bool = False,
    ) -> None:
        """Initialize a Jetson digital pin.

        Args:
            number: GPIO pin number
            mode: Pin mode (INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN)
            name: Optional human-readable name
            inverted: Invert logic
            initial: Initial state for output pins
            numbering: Pin numbering scheme
            simulation: Run in simulation mode
        """
        super().__init__(number, mode, name=name, inverted=inverted, initial=initial)
        self._numbering = numbering
        self._simulation = simulation
        self._gpio_module: Any = None

    def setup(self) -> None:
        """Initialize the GPIO pin."""
        if self._initialized:
            return

        if self._simulation:
            self._setup_simulation()
        else:
            self._setup_jetson_gpio()

        self._initialized = True
        logger.debug(
            "Initialized Jetson pin %d with numbering %s",
            self._number,
            self._numbering.value,
        )

    def _setup_jetson_gpio(self) -> None:
        """Setup using Jetson.GPIO library."""
        try:
            from Jetson import GPIO

            self._gpio_module = GPIO

            # Set pin numbering mode
            mode_map = {
                JetsonPinNumbering.BOARD: GPIO.BOARD,
                JetsonPinNumbering.BCM: GPIO.BCM,
                JetsonPinNumbering.CVM: GPIO.CVM,
                JetsonPinNumbering.TEGRA_SOC: GPIO.TEGRA_SOC,
            }
            GPIO.setmode(mode_map[self._numbering])

            # Suppress warnings for pins already in use
            GPIO.setwarnings(False)

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

        except ImportError as e:
            raise HardwareNotFoundError(
                device="Jetson.GPIO",
                details="Install with: pip install Jetson.GPIO (on Jetson only)",
            ) from e

    def _setup_simulation(self) -> None:
        """Setup in simulation mode."""
        self._gpio_module = {
            "simulated": True,
            "value": self._initial == PinState.HIGH,
        }

    def read(self) -> bool:
        """Read the pin state."""
        if not self._initialized:
            self.setup()

        value: bool
        if self._simulation:
            value = bool(self._gpio_module.get("value", False))
        else:
            value = bool(self._gpio_module.input(self._number))

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
            self._gpio_module["value"] = value
        else:
            self._gpio_module.output(self._number, value)

        self._state = PinState.HIGH if value else PinState.LOW

    def cleanup(self) -> None:
        """Release pin resources."""
        if not self._initialized:
            return

        try:
            if not self._simulation and self._gpio_module is not None:
                self._gpio_module.cleanup(self._number)
        except Exception as e:
            logger.warning("Error cleaning up Jetson pin %d: %s", self._number, e)
        finally:
            self._initialized = False
            self._gpio_module = None


class JetsonPWMPin(PWMPin):
    """PWM pin for NVIDIA Jetson.

    Supports hardware PWM on pins 32 and 33 (BOARD numbering).
    """

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        frequency: int = 50,
        duty_cycle: float = 0.0,
        numbering: JetsonPinNumbering = JetsonPinNumbering.BOARD,
        simulation: bool = False,
    ) -> None:
        """Initialize a PWM pin.

        Args:
            number: GPIO pin number
            name: Optional name
            frequency: PWM frequency in Hz
            duty_cycle: Initial duty cycle (0.0-1.0)
            numbering: Pin numbering scheme
            simulation: Run in simulation mode
        """
        super().__init__(number, name=name, frequency=frequency, duty_cycle=duty_cycle)
        self._numbering = numbering
        self._simulation = simulation
        self._gpio_module: Any = None
        self._pwm_obj: Any = None
        self._hardware_pwm = number in HARDWARE_PWM_PINS

    def setup(self) -> None:
        """Initialize PWM output."""
        if self._initialized:
            return

        if self._simulation:
            self._setup_simulation()
        else:
            self._setup_jetson_gpio()

        self._initialized = True

    def _setup_jetson_gpio(self) -> None:
        """Setup PWM using Jetson.GPIO."""
        try:
            from Jetson import GPIO

            self._gpio_module = GPIO

            # Set pin numbering
            mode_map = {
                JetsonPinNumbering.BOARD: GPIO.BOARD,
                JetsonPinNumbering.BCM: GPIO.BCM,
                JetsonPinNumbering.CVM: GPIO.CVM,
                JetsonPinNumbering.TEGRA_SOC: GPIO.TEGRA_SOC,
            }
            GPIO.setmode(mode_map[self._numbering])
            GPIO.setwarnings(False)

            # Setup pin for PWM
            GPIO.setup(self._number, GPIO.OUT)
            self._pwm_obj = GPIO.PWM(self._number, self._frequency)
            self._pwm_obj.start(self._duty_cycle * 100)

        except ImportError as e:
            raise HardwareNotFoundError(
                device="Jetson.GPIO",
                details="Install with: pip install Jetson.GPIO (on Jetson only)",
            ) from e

    def _setup_simulation(self) -> None:
        """Setup simulated PWM."""
        self._pwm_obj = {
            "simulated": True,
            "frequency": self._frequency,
            "duty_cycle": self._duty_cycle,
            "running": True,
        }

    @property
    def frequency(self) -> int:
        """Get PWM frequency."""
        return self._frequency

    @frequency.setter
    def frequency(self, value: int) -> None:
        """Set PWM frequency."""
        self._frequency = value
        if self._initialized and not self._simulation:
            self._pwm_obj.ChangeFrequency(value)
        elif self._initialized:
            self._pwm_obj["frequency"] = value

    @property
    def duty_cycle(self) -> float:
        """Get duty cycle (0.0-1.0)."""
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        """Set duty cycle (0.0-1.0)."""
        self._duty_cycle = max(0.0, min(1.0, value))
        if self._initialized and not self._simulation:
            self._pwm_obj.ChangeDutyCycle(self._duty_cycle * 100)
        elif self._initialized:
            self._pwm_obj["duty_cycle"] = self._duty_cycle

    def set_pulse_width(self, width_us: float) -> None:
        """Set PWM by pulse width in microseconds."""
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
        elif not self._simulation:
            self._pwm_obj.start(self._duty_cycle * 100)
        else:
            self._pwm_obj["running"] = True

    def stop(self) -> None:
        """Stop PWM output."""
        if self._initialized:
            if not self._simulation:
                self._pwm_obj.stop()
            else:
                self._pwm_obj["running"] = False

    def cleanup(self) -> None:
        """Stop PWM and release resources."""
        if not self._initialized:
            return

        try:
            if not self._simulation:
                self._pwm_obj.stop()
                self._gpio_module.cleanup(self._number)
        except Exception as e:
            logger.warning("Error cleaning up Jetson PWM pin %d: %s", self._number, e)
        finally:
            self._initialized = False
            self._pwm_obj = None
            self._gpio_module = None


# =============================================================================
# Jetson Platform
# =============================================================================


class JetsonPlatform(BasePlatform):
    """NVIDIA Jetson platform implementation.

    Provides access to GPIO, I2C, SPI, UART, and Jetson-specific features
    like CUDA detection and power mode management.

    Example:
        >>> platform = JetsonPlatform()
        >>> print(f"Model: {platform.model}")
        >>> print(f"Power mode: {platform.power_mode}")
        >>> print(f"CUDA available: {platform.cuda_available}")
        >>>
        >>> # GPIO
        >>> led = platform.get_pin(17, mode=PinMode.OUTPUT)
        >>> led.high()
        >>>
        >>> platform.cleanup()
    """

    def __init__(
        self,
        config: PlatformConfig | None = None,
        *,
        numbering: JetsonPinNumbering = JetsonPinNumbering.BOARD,
    ) -> None:
        """Initialize Jetson platform.

        Args:
            config: Platform configuration
            numbering: Pin numbering scheme
        """
        if config is None:
            config = PlatformConfig(
                name="NVIDIA Jetson",
                platform_type=PlatformType.JETSON,
                pin_numbering=numbering.value,
            )

        super().__init__(config)

        self._numbering = numbering
        self._model: JetsonModel | None = None
        self._simulation = self._check_simulation()

        logger.info("Jetson platform initialized (simulation=%s)", self._simulation)

    def _check_simulation(self) -> bool:
        """Check if running in simulation mode."""
        if os.getenv("ROBO_SIMULATION", "").lower() in ("1", "true", "yes"):
            return True

        # Check if Jetson.GPIO is available
        try:
            import Jetson.GPIO  # noqa: F401

            return False
        except ImportError:
            return True

    @property
    def is_available(self) -> bool:
        """Check if Jetson hardware is available."""
        if self._simulation:
            return True

        # Check for Jetson-specific files
        tegra_release = Path("/etc/nv_tegra_release")
        if tegra_release.exists():
            return True

        # Check device tree
        model_path = Path("/proc/device-tree/model")
        if model_path.exists():
            try:
                model = model_path.read_text().strip("\x00").lower()
                return "jetson" in model or "tegra" in model
            except OSError:
                pass

        return False

    @property
    def model(self) -> JetsonModel:
        """Get the Jetson model."""
        if self._model is None:
            self._model = self._detect_model()
        return self._model

    def _detect_model(self) -> JetsonModel:
        """Detect the Jetson model."""
        if self._simulation:
            return JetsonModel.NANO  # Default for simulation

        # Try /proc/device-tree/model
        model_path = Path("/proc/device-tree/model")
        if model_path.exists():
            try:
                model = model_path.read_text().strip("\x00").lower()
                for pattern, jetson_model in JETSON_MODELS.items():
                    if pattern in model:
                        return jetson_model
            except OSError:
                pass

        # Try /etc/nv_tegra_release
        tegra_path = Path("/etc/nv_tegra_release")
        if tegra_path.exists():
            try:
                content = tegra_path.read_text().lower()
                # Parse release info to determine model
                if "t210" in content:  # Nano/TX1
                    return JetsonModel.NANO
                elif "t186" in content:  # TX2
                    return JetsonModel.TX2
                elif "t194" in content:  # Xavier
                    return JetsonModel.AGX_XAVIER
                elif "t234" in content:  # Orin
                    return JetsonModel.AGX_ORIN
            except OSError:
                pass

        # Try using Jetson.GPIO model detection
        try:
            from Jetson import GPIO

            model_name = GPIO.model.lower()
            for pattern, jetson_model in JETSON_MODELS.items():
                if pattern in model_name:
                    return jetson_model
        except (ImportError, AttributeError):
            pass

        return JetsonModel.UNKNOWN

    @property
    def power_mode(self) -> JetsonPowerMode:
        """Get the current power mode."""
        if self._simulation:
            return JetsonPowerMode.MAXN

        try:
            # Use nvpmodel to get current mode
            result = subprocess.run(
                ["nvpmodel", "-q"],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )
            if result.returncode == 0:
                output = result.stdout.lower()
                if "5w" in output:
                    return JetsonPowerMode.MODE_5W
                elif "10w" in output:
                    return JetsonPowerMode.MODE_10W
                elif "15w" in output:
                    return JetsonPowerMode.MODE_15W
                elif "20w" in output:
                    return JetsonPowerMode.MODE_20W
                elif "30w" in output:
                    return JetsonPowerMode.MODE_30W
                elif "maxn" in output:
                    return JetsonPowerMode.MAXN
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
            pass

        return JetsonPowerMode.UNKNOWN

    def set_power_mode(self, mode: JetsonPowerMode | str) -> bool:
        """Set the power mode.

        Args:
            mode: Power mode to set (enum or mode ID like "0", "1")

        Returns:
            True if successful, False otherwise
        """
        if self._simulation:
            logger.info("Simulation: Would set power mode to %s", mode)
            return True

        try:
            if isinstance(mode, JetsonPowerMode):
                # Map enum to mode ID (model-specific)
                mode_id = self._power_mode_to_id(mode)
            else:
                mode_id = str(mode)

            result = subprocess.run(
                ["sudo", "nvpmodel", "-m", mode_id],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
            logger.error("Failed to set power mode: %s", e)
            return False

    def _power_mode_to_id(self, mode: JetsonPowerMode) -> str:
        """Convert power mode enum to nvpmodel ID."""
        # Mode IDs are model-specific, these are common mappings
        mode_map = {
            JetsonPowerMode.MAXN: "0",
            JetsonPowerMode.MODE_10W: "1",
            JetsonPowerMode.MODE_5W: "2",
            JetsonPowerMode.MODE_15W: "3",
            JetsonPowerMode.MODE_20W: "2",
            JetsonPowerMode.MODE_30W: "3",
        }
        return mode_map.get(mode, "0")

    @property
    def cuda_available(self) -> bool:
        """Check if CUDA is available."""
        if self._simulation:
            return False

        # Check for CUDA libraries
        cuda_paths = [
            Path("/usr/local/cuda"),
            Path("/usr/lib/aarch64-linux-gnu/libcuda.so"),
        ]
        for path in cuda_paths:
            if path.exists():
                return True

        # Try importing torch with CUDA
        try:
            import torch  # type: ignore[import-not-found]

            return bool(torch.cuda.is_available())
        except ImportError:
            pass

        return False

    @property
    def cuda_version(self) -> str | None:
        """Get CUDA version if available."""
        if self._simulation:
            return None

        version_file = Path("/usr/local/cuda/version.txt")
        if version_file.exists():
            try:
                content = version_file.read_text()
                # Parse "CUDA Version X.Y.Z"
                parts = content.strip().split()
                if len(parts) >= 3:
                    return parts[2]
            except OSError:
                pass

        # Try using nvcc
        try:
            result = subprocess.run(
                ["nvcc", "--version"],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )
            if result.returncode == 0:
                # Parse version from output
                for line in result.stdout.split("\n"):
                    if "release" in line.lower():
                        parts = line.split("release")
                        if len(parts) >= 2:
                            version = parts[1].split(",")[0].strip()
                            return version
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
            pass

        return None

    def get_csi_cameras(self) -> list[dict[str, Any]]:
        """Get list of available CSI cameras.

        Returns:
            List of camera info dictionaries
        """
        cameras: list[dict[str, Any]] = []

        if self._simulation:
            return cameras

        # Check for v4l2 devices
        for i in range(4):
            device = Path(f"/dev/video{i}")
            if device.exists():
                try:
                    # Get device info using v4l2-ctl
                    result = subprocess.run(
                        ["v4l2-ctl", "--device", str(device), "--info"],
                        capture_output=True,
                        text=True,
                        timeout=5,
                        check=False,
                    )
                    if result.returncode == 0 and "Argus" in result.stdout:
                        cameras.append(
                            {
                                "device": str(device),
                                "index": i,
                                "type": "csi",
                                "info": result.stdout,
                            }
                        )
                except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
                    pass

        return cameras

    @property
    def jetpack_version(self) -> str | None:
        """Get JetPack version if available."""
        if self._simulation:
            return None

        # Try /etc/nv_tegra_release
        tegra_path = Path("/etc/nv_tegra_release")
        if tegra_path.exists():
            try:
                content = tegra_path.read_text()
                # First line contains version info
                first_line = content.split("\n")[0]
                return first_line.strip()
            except OSError:
                pass

        return None

    def _detect_info(self) -> PlatformInfo:
        """Detect platform information."""
        capabilities: set[PlatformCapability] = set()

        # All Jetsons have GPIO
        capabilities.add(PlatformCapability.GPIO)
        capabilities.add(PlatformCapability.PWM)

        # Check for I2C
        i2c_buses: list[int] = []
        for i in range(10):
            if Path(f"/dev/i2c-{i}").exists():
                i2c_buses.append(i)
                capabilities.add(PlatformCapability.I2C)

        # Check for SPI
        spi_buses: list[tuple[int, int]] = []
        for bus in range(4):
            for dev in range(4):
                if Path(f"/dev/spidev{bus}.{dev}").exists():
                    spi_buses.append((bus, dev))
                    capabilities.add(PlatformCapability.SPI)

        # Check for UART
        uart_ports: list[str] = []
        for port in ["/dev/ttyTHS0", "/dev/ttyTHS1", "/dev/ttyTHS2", "/dev/ttyS0"]:
            if Path(port).exists():
                uart_ports.append(port)
                capabilities.add(PlatformCapability.UART)

        # Check for cameras
        for i in range(4):
            if Path(f"/dev/video{i}").exists():
                capabilities.add(PlatformCapability.CAMERA_USB)

        # Check for CUDA
        if self.cuda_available:
            capabilities.add(PlatformCapability.HARDWARE_PWM)  # Jetson has HW PWM

        # Get GPIO chips
        gpio_chips: list[str] = []
        for chip_path in Path("/dev").glob("gpiochip*"):
            gpio_chips.append(chip_path.name)

        return PlatformInfo(
            platform_type=PlatformType.JETSON,
            model=self.model.value,
            revision=self.jetpack_version or "",
            serial="",
            capabilities=capabilities,
            gpio_chips=gpio_chips,
            i2c_buses=i2c_buses,
            spi_buses=spi_buses,
            uart_ports=uart_ports,
        )

    def _create_pin(self, pin_id: int | str, **kwargs: Any) -> Pin:
        """Create a Jetson GPIO pin."""
        pin_num = int(pin_id) if isinstance(pin_id, str) else pin_id
        mode = kwargs.get("mode", PinMode.OUTPUT)

        if mode == PinMode.PWM:
            return JetsonPWMPin(
                pin_num,
                name=kwargs.get("name"),
                frequency=kwargs.get("frequency", 50),
                duty_cycle=kwargs.get("duty_cycle", 0.0),
                numbering=self._numbering,
                simulation=self._simulation,
            )
        else:
            initial = kwargs.get("initial", PinState.LOW)
            if isinstance(initial, bool):
                initial = PinState.HIGH if initial else PinState.LOW

            return JetsonDigitalPin(
                pin_num,
                mode=mode,
                name=kwargs.get("name"),
                inverted=kwargs.get("inverted", False),
                initial=initial,
                numbering=self._numbering,
                simulation=self._simulation,
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
        """Create an I2C bus."""
        from robo_infra.core.bus import I2CConfig, SimulatedI2CBus

        # Jetson typically uses bus 0 or 1
        bus_num = kwargs.get("bus", 1)
        config = I2CConfig(bus_number=bus_num)

        if not self._simulation:
            try:
                from robo_infra.core.bus import SMBus2I2CBus

                return SMBus2I2CBus(config=config)
            except (ImportError, AttributeError):
                logger.warning("smbus2 not available, using simulated I2C")

        return SimulatedI2CBus(config=config)

    def _create_spi_bus(self, **kwargs: Any) -> Bus:
        """Create an SPI bus."""
        from robo_infra.core.bus import SimulatedSPIBus, SPIConfig

        bus_num = kwargs.get("bus", 0)
        device = kwargs.get("device", 0)
        config = SPIConfig(bus=bus_num, device=device)

        if not self._simulation:
            try:
                from robo_infra.core.bus import SpiDevSPIBus

                return SpiDevSPIBus(config=config)
            except (ImportError, AttributeError):
                logger.warning("spidev not available, using simulated SPI")

        return SimulatedSPIBus(config=config)

    def _create_uart_bus(self, **kwargs: Any) -> Bus:
        """Create a UART bus."""
        from robo_infra.core.bus import SerialConfig, SimulatedSerialBus

        # Jetson uses /dev/ttyTHS* for UART
        port = kwargs.get("port", "/dev/ttyTHS1")
        baudrate = kwargs.get("baudrate", 115200)
        config = SerialConfig(port=port, baudrate=baudrate)

        if not self._simulation:
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
    "HARDWARE_PWM_PINS",
    "JETSON_MODELS",
    # Pin classes
    "JetsonDigitalPin",
    # Enums
    "JetsonModel",
    "JetsonPWMPin",
    "JetsonPinNumbering",
    # Platform
    "JetsonPlatform",
    "JetsonPowerMode",
]
