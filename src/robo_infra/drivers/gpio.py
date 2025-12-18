"""GPIO driver for direct pin control.

This module provides a driver for direct GPIO pin control across multiple
platforms (Raspberry Pi, Jetson, BeagleBone, etc.) with software PWM support.

The GPIO driver provides:
- Digital output for any GPIO pin
- Software PWM when hardware PWM is unavailable
- Platform detection for automatic backend selection
- Simulation mode for testing without hardware

Example:
    >>> from robo_infra.drivers.gpio import GPIODriver
    >>>
    >>> # Create driver (auto-detects platform or uses simulation)
    >>> driver = GPIODriver()
    >>> driver.connect()
    >>>
    >>> # Digital output
    >>> driver.digital_write(17, True)  # Set pin 17 HIGH
    >>> driver.digital_write(17, False)  # Set pin 17 LOW
    >>>
    >>> # PWM output
    >>> driver.pwm_start(18, frequency=1000, duty_cycle=0.5)
    >>> driver.pwm_set_duty_cycle(18, 0.75)
    >>> driver.pwm_stop(18)
    >>>
    >>> # Read digital input
    >>> value = driver.digital_read(27)
    >>>
    >>> driver.disconnect()

Supported Platforms:
    - raspberry_pi: Raspberry Pi (all models)
    - jetson: NVIDIA Jetson (Nano, TX2, Xavier, Orin)
    - beaglebone: BeagleBone Black/Green
    - simulation: No hardware (for testing)

Software PWM:
    When hardware PWM is unavailable, this driver provides software PWM
    using threading. Software PWM is less precise but works on any GPIO pin.
    For precise PWM, use hardware PWM pins or a dedicated driver like PCA9685.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from robo_infra.core.driver import (
    Driver,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import HardwareNotFoundError
from robo_infra.core.pin import detect_platform


if TYPE_CHECKING:
    from collections.abc import Callable

logger = logging.getLogger(__name__)


# =============================================================================
# Enums and Constants
# =============================================================================


class GPIODirection(Enum):
    """GPIO pin direction."""

    INPUT = "input"
    OUTPUT = "output"


class GPIOPull(Enum):
    """GPIO internal pull resistor configuration."""

    NONE = "none"
    UP = "up"
    DOWN = "down"


class GPIOEdge(Enum):
    """GPIO edge detection for interrupts."""

    NONE = "none"
    RISING = "rising"
    FALLING = "falling"
    BOTH = "both"


class Platform(Enum):
    """Supported hardware platforms."""

    RASPBERRY_PI = "raspberry_pi"
    JETSON = "jetson"
    BEAGLEBONE = "beaglebone"
    SIMULATION = "simulation"


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class GPIOPinConfig:
    """Configuration for a single GPIO pin.

    Attributes:
        direction: Pin direction (INPUT or OUTPUT)
        pull: Internal pull resistor configuration
        initial_state: Initial state for output pins
        inverted: If True, invert logic levels
        edge: Edge detection for interrupts (INPUT pins only)
    """

    direction: GPIODirection = GPIODirection.OUTPUT
    pull: GPIOPull = GPIOPull.NONE
    initial_state: bool = False
    inverted: bool = False
    edge: GPIOEdge = GPIOEdge.NONE


@dataclass
class SoftwarePWMConfig:
    """Configuration for software PWM.

    Attributes:
        frequency: PWM frequency in Hz (default 1000)
        duty_cycle: Initial duty cycle 0.0-1.0
        running: Whether PWM is currently running
    """

    frequency: float = 1000.0
    duty_cycle: float = 0.0
    running: bool = False


@dataclass
class GPIOConfig:
    """Configuration for GPIO driver.

    Attributes:
        platform: Target platform (None for auto-detect)
        numbering_mode: Pin numbering mode (BCM or BOARD for Pi)
        warnings: Enable GPIO warnings
        cleanup_on_disconnect: Cleanup all pins on disconnect
        software_pwm_default_frequency: Default frequency for software PWM
    """

    platform: Platform | None = None
    numbering_mode: str = "BCM"
    warnings: bool = True
    cleanup_on_disconnect: bool = True
    software_pwm_default_frequency: float = 1000.0


# =============================================================================
# Pin State
# =============================================================================


@dataclass
class GPIOPinState:
    """Current state of a GPIO pin.

    Attributes:
        pin: Pin number
        direction: Current direction
        value: Current digital value (True=HIGH, False=LOW)
        pwm_config: Software PWM config if active
        configured: Whether pin has been configured
    """

    pin: int
    direction: GPIODirection = GPIODirection.OUTPUT
    value: bool = False
    pwm_config: SoftwarePWMConfig | None = None
    configured: bool = False


# =============================================================================
# Software PWM Thread
# =============================================================================


class SoftwarePWMThread(threading.Thread):
    """Thread for generating software PWM signal.

    Software PWM is implemented by toggling a GPIO pin in a loop with
    precise timing based on the duty cycle and frequency.

    Note:
        Software PWM is less precise than hardware PWM due to OS scheduling.
        For precision-critical applications, use hardware PWM or PCA9685.
    """

    # Maximum PWM frequency for software PWM (limited by thread scheduling)
    MAX_FREQUENCY = 10000.0

    # Minimum period (100us) to prevent CPU overload
    MIN_PERIOD = 0.0001

    def __init__(
        self,
        pin: int,
        frequency: float,
        duty_cycle: float,
        write_callback: Callable[[int, bool], None],
    ) -> None:
        """Initialize software PWM thread.

        Args:
            pin: GPIO pin number
            frequency: PWM frequency in Hz
            duty_cycle: Duty cycle 0.0-1.0
            write_callback: Callback to write pin state
        """
        super().__init__(daemon=True, name=f"SoftPWM-{pin}")
        self._pin = pin
        self._frequency = min(frequency, self.MAX_FREQUENCY)
        self._duty_cycle = max(0.0, min(1.0, duty_cycle))
        self._write_callback = write_callback
        self._running = threading.Event()
        self._lock = threading.Lock()

    @property
    def frequency(self) -> float:
        """Get current frequency."""
        with self._lock:
            return self._frequency

    @frequency.setter
    def frequency(self, value: float) -> None:
        """Set frequency."""
        with self._lock:
            self._frequency = min(value, self.MAX_FREQUENCY)

    @property
    def duty_cycle(self) -> float:
        """Get current duty cycle."""
        with self._lock:
            return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        """Set duty cycle."""
        with self._lock:
            self._duty_cycle = max(0.0, min(1.0, value))

    def start_pwm(self) -> None:
        """Start PWM generation."""
        self._running.set()
        if not self.is_alive():
            self.start()

    def stop_pwm(self) -> None:
        """Stop PWM generation."""
        self._running.clear()

    def run(self) -> None:
        """PWM generation loop."""
        while True:
            if not self._running.wait(timeout=0.1):
                # Check if we should exit
                continue

            with self._lock:
                frequency = self._frequency
                duty_cycle = self._duty_cycle

            if frequency <= 0:
                continue

            period = 1.0 / frequency
            on_time = period * duty_cycle
            off_time = period - on_time

            # Handle edge cases
            if duty_cycle >= 1.0:
                self._write_callback(self._pin, True)
                time.sleep(max(period, self.MIN_PERIOD))
            elif duty_cycle <= 0.0:
                self._write_callback(self._pin, False)
                time.sleep(max(period, self.MIN_PERIOD))
            else:
                # Normal PWM cycle
                self._write_callback(self._pin, True)
                time.sleep(max(on_time, self.MIN_PERIOD))
                self._write_callback(self._pin, False)
                time.sleep(max(off_time, self.MIN_PERIOD))


# =============================================================================
# GPIO Driver
# =============================================================================


@register_driver("gpio")
class GPIODriver(Driver):
    """GPIO driver for direct pin control.

    This driver provides direct GPIO access across multiple platforms with
    automatic platform detection and software PWM support.

    Features:
        - Digital input/output on any GPIO pin
        - Software PWM for pins without hardware PWM
        - Platform auto-detection (Pi, Jetson, BeagleBone)
        - Simulation mode for testing
        - Thread-safe operations

    Example:
        >>> driver = GPIODriver()
        >>> driver.connect()
        >>>
        >>> # Configure and write
        >>> driver.setup_pin(17, GPIODirection.OUTPUT)
        >>> driver.digital_write(17, True)
        >>>
        >>> # Read input
        >>> driver.setup_pin(27, GPIODirection.INPUT, pull=GPIOPull.UP)
        >>> value = driver.digital_read(27)
        >>>
        >>> driver.disconnect()
    """

    # Default values
    DEFAULT_PWM_FREQUENCY = 1000.0

    def __init__(
        self,
        config: GPIOConfig | None = None,
    ) -> None:
        """Initialize GPIO driver.

        Args:
            config: Driver configuration (uses defaults if None)
        """
        super().__init__(name="GPIO", channels=0)

        self._gpio_config: GPIOConfig = config or GPIOConfig()
        self._platform: Platform | None = None
        self._gpio_backend: object | None = None

        # Pin tracking
        self._pins: dict[int, GPIOPinState] = {}
        self._pwm_threads: dict[int, SoftwarePWMThread] = {}

        # Thread safety
        self._lock = threading.RLock()

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def platform(self) -> Platform:
        """Get the detected platform."""
        if self._platform is None:
            self._detect_platform()
        return self._platform  # type: ignore[return-value]

    @property
    def simulation_mode(self) -> bool:
        """Check if running in simulation mode."""
        return self.platform == Platform.SIMULATION

    @property
    def configured_pins(self) -> list[int]:
        """Get list of configured pin numbers."""
        with self._lock:
            return [pin for pin, state in self._pins.items() if state.configured]

    @property
    def active_pwm_pins(self) -> list[int]:
        """Get list of pins with active software PWM."""
        with self._lock:
            return [pin for pin, thread in self._pwm_threads.items() if thread._running.is_set()]

    # =========================================================================
    # Platform Detection
    # =========================================================================

    def _detect_platform(self) -> None:
        """Detect the current hardware platform."""
        if self._gpio_config.platform is not None:
            self._platform = self._gpio_config.platform
            return

        detected = detect_platform()
        platform_map = {
            "raspberry_pi": Platform.RASPBERRY_PI,
            "jetson": Platform.JETSON,
            "beaglebone": Platform.BEAGLEBONE,
            "simulation": Platform.SIMULATION,
        }
        self._platform = platform_map.get(detected, Platform.SIMULATION)
        logger.info("Detected platform: %s", self._platform.value)

    def _init_gpio_backend(self) -> None:
        """Initialize the GPIO backend for the current platform.

        For real hardware, this would import and initialize the appropriate
        GPIO library (RPi.GPIO, Jetson.GPIO, Adafruit_BBIO, etc.).
        """
        if self._platform == Platform.RASPBERRY_PI:
            self._init_raspberry_pi()
        elif self._platform == Platform.JETSON:
            self._init_jetson()
        elif self._platform == Platform.BEAGLEBONE:
            self._init_beaglebone()
        else:
            # Simulation mode - no backend needed
            self._gpio_backend = None
            logger.info("GPIO driver running in simulation mode")

    def _init_raspberry_pi(self) -> None:
        """Initialize Raspberry Pi GPIO."""
        try:
            from RPi import GPIO  # type: ignore[import-not-found]

            if self._gpio_config.numbering_mode == "BCM":
                GPIO.setmode(GPIO.BCM)
            else:
                GPIO.setmode(GPIO.BOARD)

            if not self._gpio_config.warnings:
                GPIO.setwarnings(False)

            self._gpio_backend = GPIO
            logger.info("Raspberry Pi GPIO initialized")
        except ImportError:
            logger.warning("RPi.GPIO not available, falling back to simulation")
            self._platform = Platform.SIMULATION
            self._gpio_backend = None

    def _init_jetson(self) -> None:
        """Initialize NVIDIA Jetson GPIO."""
        try:
            from Jetson import GPIO  # type: ignore[import-not-found]

            GPIO.setmode(GPIO.BCM)
            if not self._gpio_config.warnings:
                GPIO.setwarnings(False)

            self._gpio_backend = GPIO
            logger.info("Jetson GPIO initialized")
        except ImportError:
            logger.warning("Jetson.GPIO not available, falling back to simulation")
            self._platform = Platform.SIMULATION
            self._gpio_backend = None

    def _init_beaglebone(self) -> None:
        """Initialize BeagleBone GPIO."""
        try:
            from Adafruit_BBIO import GPIO  # type: ignore[import-not-found]

            self._gpio_backend = GPIO
            logger.info("BeagleBone GPIO initialized")
        except ImportError:
            logger.warning("Adafruit_BBIO not available, falling back to simulation")
            self._platform = Platform.SIMULATION
            self._gpio_backend = None

    # =========================================================================
    # Lifecycle Methods
    # =========================================================================

    def connect(self) -> None:
        """Connect to GPIO hardware.

        Detects the platform and initializes the appropriate GPIO backend.
        """
        if self._state == DriverState.CONNECTED:
            logger.warning("GPIO driver already connected")
            return

        self._detect_platform()
        self._init_gpio_backend()
        self._state = DriverState.CONNECTED
        platform_name = self._platform.value if self._platform else "unknown"
        logger.info(
            "GPIO driver connected (platform=%s, simulation=%s)",
            platform_name,
            self.simulation_mode,
        )

    def disconnect(self) -> None:
        """Disconnect from GPIO hardware.

        Stops all PWM, cleans up pins, and releases the GPIO backend.
        """
        if self._state == DriverState.DISCONNECTED:
            logger.warning("GPIO driver already disconnected")
            return

        # Stop all PWM threads
        for pin, thread in list(self._pwm_threads.items()):
            thread.stop_pwm()
            logger.debug("Stopped PWM on pin %d", pin)

        # Cleanup pins
        if self._gpio_config.cleanup_on_disconnect:
            self._cleanup_all_pins()

        # Cleanup backend
        if self._gpio_backend is not None:
            try:
                self._gpio_backend.cleanup()  # type: ignore[union-attr]
            except Exception as e:
                logger.warning("Error during GPIO cleanup: %s", e)
            self._gpio_backend = None

        self._pins.clear()
        self._pwm_threads.clear()
        self._state = DriverState.DISCONNECTED
        logger.info("GPIO driver disconnected")

    def _cleanup_all_pins(self) -> None:
        """Cleanup all configured pins."""
        for pin in list(self._pins.keys()):
            self._cleanup_pin(pin)

    def _cleanup_pin(self, pin: int) -> None:
        """Cleanup a single pin."""
        if self._gpio_backend is not None:
            try:
                self._gpio_backend.cleanup(pin)  # type: ignore[union-attr]
            except Exception as e:
                logger.warning("Error cleaning up pin %d: %s", pin, e)

    # =========================================================================
    # Pin Configuration
    # =========================================================================

    def setup_pin(
        self,
        pin: int,
        direction: GPIODirection,
        *,
        pull: GPIOPull = GPIOPull.NONE,
        initial: bool = False,
        inverted: bool = False,
    ) -> None:
        """Configure a GPIO pin.

        Args:
            pin: GPIO pin number
            direction: INPUT or OUTPUT
            pull: Pull resistor configuration (INPUT only)
            initial: Initial state for OUTPUT pins
            inverted: Invert logic levels

        Raises:
            HardwareNotFoundError: If driver not connected
        """
        self._require_connected()

        with self._lock:
            # Configure in backend
            if self._gpio_backend is not None:
                self._setup_pin_backend(pin, direction, pull, initial)

            # Track state
            self._pins[pin] = GPIOPinState(
                pin=pin,
                direction=direction,
                value=initial,
                configured=True,
            )
            logger.debug(
                "Configured pin %d as %s (pull=%s, initial=%s, inverted=%s)",
                pin,
                direction.value,
                pull.value,
                initial,
                inverted,
            )

    def _setup_pin_backend(
        self,
        pin: int,
        direction: GPIODirection,
        pull: GPIOPull,
        initial: bool,
    ) -> None:
        """Configure pin in the hardware backend."""
        if self._gpio_backend is None:
            return

        GPIO = self._gpio_backend

        # Map direction
        gpio_dir = GPIO.OUT if direction == GPIODirection.OUTPUT else GPIO.IN

        # Map pull
        pull_map = {
            GPIOPull.NONE: GPIO.PUD_OFF if hasattr(GPIO, "PUD_OFF") else None,
            GPIOPull.UP: GPIO.PUD_UP,
            GPIOPull.DOWN: GPIO.PUD_DOWN,
        }
        gpio_pull = pull_map.get(pull)

        # Setup the pin
        if direction == GPIODirection.OUTPUT:
            initial_state = GPIO.HIGH if initial else GPIO.LOW
            GPIO.setup(pin, gpio_dir, initial=initial_state)
        elif gpio_pull is not None:
            GPIO.setup(pin, gpio_dir, pull_up_down=gpio_pull)
        else:
            GPIO.setup(pin, gpio_dir)

    def is_pin_configured(self, pin: int) -> bool:
        """Check if a pin is configured.

        Args:
            pin: GPIO pin number

        Returns:
            True if pin is configured
        """
        with self._lock:
            return pin in self._pins and self._pins[pin].configured

    def get_pin_state(self, pin: int) -> GPIOPinState | None:
        """Get current state of a pin.

        Args:
            pin: GPIO pin number

        Returns:
            Pin state or None if not configured
        """
        with self._lock:
            return self._pins.get(pin)

    # =========================================================================
    # Digital I/O
    # =========================================================================

    def digital_write(self, pin: int, value: bool) -> None:
        """Write a digital value to a pin.

        Args:
            pin: GPIO pin number
            value: True for HIGH, False for LOW

        Raises:
            HardwareNotFoundError: If driver not connected
            ValueError: If pin not configured as output
        """
        self._require_connected()

        with self._lock:
            state = self._pins.get(pin)
            if state is None or not state.configured:
                # Auto-configure as output
                self.setup_pin(pin, GPIODirection.OUTPUT, initial=value)
                state = self._pins[pin]

            if state.direction != GPIODirection.OUTPUT:
                raise ValueError(f"Pin {pin} is not configured as output")

            # Write to backend
            if self._gpio_backend is not None:
                gpio_value = self._gpio_backend.HIGH if value else self._gpio_backend.LOW
                self._gpio_backend.output(pin, gpio_value)

            # Update state
            state.value = value

    def digital_read(self, pin: int) -> bool:
        """Read a digital value from a pin.

        Args:
            pin: GPIO pin number

        Returns:
            True if HIGH, False if LOW

        Raises:
            HardwareNotFoundError: If driver not connected
        """
        self._require_connected()

        with self._lock:
            state = self._pins.get(pin)
            if state is None or not state.configured:
                # Auto-configure as input
                self.setup_pin(pin, GPIODirection.INPUT)
                state = self._pins[pin]

            # Read from backend
            if self._gpio_backend is not None:
                value = self._gpio_backend.input(pin)
                state.value = bool(value)
            else:
                # Simulation: return stored value
                pass

            return state.value

    def set_high(self, pin: int) -> None:
        """Set a pin HIGH.

        Args:
            pin: GPIO pin number
        """
        self.digital_write(pin, True)

    def set_low(self, pin: int) -> None:
        """Set a pin LOW.

        Args:
            pin: GPIO pin number
        """
        self.digital_write(pin, False)

    def toggle(self, pin: int) -> bool:
        """Toggle a pin state.

        Args:
            pin: GPIO pin number

        Returns:
            New state after toggle
        """
        with self._lock:
            state = self._pins.get(pin)
            if state is None:
                self.digital_write(pin, True)
                return True

            new_value = not state.value
            self.digital_write(pin, new_value)
            return new_value

    # =========================================================================
    # Software PWM
    # =========================================================================

    def pwm_start(
        self,
        pin: int,
        *,
        frequency: float | None = None,
        duty_cycle: float = 0.0,
    ) -> None:
        """Start software PWM on a pin.

        Args:
            pin: GPIO pin number
            frequency: PWM frequency in Hz (default from config)
            duty_cycle: Initial duty cycle 0.0-1.0

        Raises:
            HardwareNotFoundError: If driver not connected
        """
        self._require_connected()

        freq = frequency or self._gpio_config.software_pwm_default_frequency

        with self._lock:
            # Ensure pin is configured as output
            state = self._pins.get(pin)
            if state is None or not state.configured:
                self.setup_pin(pin, GPIODirection.OUTPUT)
                state = self._pins[pin]

            # Create or get PWM thread
            if pin not in self._pwm_threads:
                thread = SoftwarePWMThread(
                    pin=pin,
                    frequency=freq,
                    duty_cycle=duty_cycle,
                    write_callback=self._pwm_write_callback,
                )
                self._pwm_threads[pin] = thread
            else:
                thread = self._pwm_threads[pin]
                thread.frequency = freq
                thread.duty_cycle = duty_cycle

            # Update state
            state.pwm_config = SoftwarePWMConfig(
                frequency=freq,
                duty_cycle=duty_cycle,
                running=True,
            )

            # Start PWM
            thread.start_pwm()
            logger.debug(
                "Started PWM on pin %d (freq=%0.1f Hz, duty=%0.1f%%)",
                pin,
                freq,
                duty_cycle * 100,
            )

    def _pwm_write_callback(self, pin: int, value: bool) -> None:
        """Callback for PWM thread to write pin state.

        This is called from the PWM thread, so it must be thread-safe.
        """
        if self._gpio_backend is not None:
            gpio_value = self._gpio_backend.HIGH if value else self._gpio_backend.LOW
            self._gpio_backend.output(pin, gpio_value)
        else:
            # Simulation: update state
            with self._lock:
                if pin in self._pins:
                    self._pins[pin].value = value

    def pwm_stop(self, pin: int) -> None:
        """Stop software PWM on a pin.

        Args:
            pin: GPIO pin number
        """
        with self._lock:
            if pin in self._pwm_threads:
                self._pwm_threads[pin].stop_pwm()
                logger.debug("Stopped PWM on pin %d", pin)

            state = self._pins.get(pin)
            if state is not None and state.pwm_config is not None:
                state.pwm_config.running = False

            # Set pin low when stopping
            self.digital_write(pin, False)

    def pwm_set_duty_cycle(self, pin: int, duty_cycle: float) -> None:
        """Set PWM duty cycle.

        Args:
            pin: GPIO pin number
            duty_cycle: Duty cycle 0.0-1.0

        Raises:
            ValueError: If PWM not started on pin
        """
        with self._lock:
            if pin not in self._pwm_threads:
                raise ValueError(f"PWM not started on pin {pin}")

            self._pwm_threads[pin].duty_cycle = duty_cycle

            state = self._pins.get(pin)
            if state is not None and state.pwm_config is not None:
                state.pwm_config.duty_cycle = duty_cycle

    def pwm_set_frequency(self, pin: int, frequency: float) -> None:
        """Set PWM frequency.

        Args:
            pin: GPIO pin number
            frequency: Frequency in Hz

        Raises:
            ValueError: If PWM not started on pin
        """
        with self._lock:
            if pin not in self._pwm_threads:
                raise ValueError(f"PWM not started on pin {pin}")

            self._pwm_threads[pin].frequency = frequency

            state = self._pins.get(pin)
            if state is not None and state.pwm_config is not None:
                state.pwm_config.frequency = frequency

    def pwm_get_duty_cycle(self, pin: int) -> float:
        """Get current PWM duty cycle.

        Args:
            pin: GPIO pin number

        Returns:
            Current duty cycle 0.0-1.0

        Raises:
            ValueError: If PWM not started on pin
        """
        with self._lock:
            if pin not in self._pwm_threads:
                raise ValueError(f"PWM not started on pin {pin}")

            return self._pwm_threads[pin].duty_cycle

    # =========================================================================
    # Driver Interface Methods
    # =========================================================================

    def set_channel(self, channel: int, value: float) -> None:
        """Set channel value (GPIO pin as PWM).

        This implements the Driver interface, treating each GPIO pin as a
        channel with PWM output.

        Args:
            channel: GPIO pin number
            value: PWM duty cycle 0.0-1.0 (or -1.0 to 1.0, maps to 0-1)
        """
        self._require_connected()
        self._write_channel(channel, value)

    def get_channel(self, channel: int) -> float:
        """Get channel value (GPIO pin PWM duty cycle).

        Args:
            channel: GPIO pin number

        Returns:
            Current PWM duty cycle or digital value
        """
        return self._read_channel(channel)

    def _write_channel(self, channel: int, value: float) -> None:
        """Write a value to a GPIO pin (as PWM duty cycle).

        Args:
            channel: GPIO pin number
            value: PWM duty cycle 0.0-1.0 (or -1.0 to 1.0, maps to 0-1)
        """
        # Normalize value to 0.0-1.0
        duty = abs(value)
        duty = max(0.0, min(1.0, duty))

        with self._lock:
            if channel in self._pwm_threads:
                self._pwm_threads[channel].duty_cycle = duty
            else:
                # Start PWM if not already started
                self.pwm_start(channel, duty_cycle=duty)

    def _read_channel(self, channel: int) -> float:
        """Read a value from a GPIO pin.

        Args:
            channel: GPIO pin number

        Returns:
            Current PWM duty cycle or digital value
        """
        with self._lock:
            if channel in self._pwm_threads:
                return self._pwm_threads[channel].duty_cycle

            state = self._pins.get(channel)
            if state is not None:
                return 1.0 if state.value else 0.0

            return 0.0

    # =========================================================================
    # Validation
    # =========================================================================

    def _require_connected(self) -> None:
        """Ensure driver is connected.

        Raises:
            HardwareNotFoundError: If not connected
        """
        if self._state != DriverState.CONNECTED:
            raise HardwareNotFoundError("GPIO driver not connected")

    # =========================================================================
    # Representation
    # =========================================================================

    def __repr__(self) -> str:
        return (
            f"GPIODriver("
            f"platform={self._platform.value if self._platform else 'unknown'}, "
            f"pins={len(self._pins)}, "
            f"simulation={self.simulation_mode}, "
            f"state={self._state.value}"
            f")"
        )


# =============================================================================
# Convenience Functions
# =============================================================================


def get_gpio_driver(
    platform: Platform | str | None = None,
) -> GPIODriver:
    """Create a GPIO driver with specified or auto-detected platform.

    Args:
        platform: Platform to use (auto-detect if None)

    Returns:
        Configured GPIODriver instance
    """
    if isinstance(platform, str):
        platform = Platform(platform)

    config = GPIOConfig(platform=platform)
    return GPIODriver(config=config)
