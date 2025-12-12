"""Pin abstractions for GPIO, PWM, and analog I/O."""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from pathlib import Path


class PinMode(Enum):
    """Pin operating modes."""

    INPUT = "input"
    OUTPUT = "output"
    INPUT_PULLUP = "input_pullup"
    INPUT_PULLDOWN = "input_pulldown"
    PWM = "pwm"
    ANALOG = "analog"


class PinState(Enum):
    """Digital pin states."""

    LOW = 0
    HIGH = 1


class Pin(ABC):
    """Abstract base class for all pin types.

    Pins represent the lowest level of hardware interaction - a single
    GPIO pin that can be configured for various modes.
    """

    def __init__(
        self,
        number: int,
        mode: PinMode = PinMode.OUTPUT,
        *,
        name: str | None = None,
        inverted: bool = False,
    ) -> None:
        """Initialize a pin.

        Args:
            number: The physical or logical pin number
            mode: The pin operating mode
            name: Optional human-readable name for the pin
            inverted: If True, logic is inverted (high=low, low=high)
        """
        self._number = number
        self._mode = mode
        self._name = name or f"pin-{number}"
        self._inverted = inverted
        self._initialized = False

    @property
    def number(self) -> int:
        """Get the pin number."""
        return self._number

    @property
    def mode(self) -> PinMode:
        """Get the pin mode."""
        return self._mode

    @property
    def name(self) -> str:
        """Get the pin name."""
        return self._name

    @property
    def inverted(self) -> bool:
        """Check if pin logic is inverted."""
        return self._inverted

    @property
    def initialized(self) -> bool:
        """Check if pin has been initialized."""
        return self._initialized

    @abstractmethod
    def setup(self) -> None:
        """Initialize the pin hardware.

        Must be called before using the pin.
        """
        ...

    @abstractmethod
    def cleanup(self) -> None:
        """Release pin resources and reset to safe state."""
        ...

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self._number}, mode={self._mode.value}, name={self._name!r})"


class DigitalPin(Pin):
    """Pin for digital input/output operations."""

    def __init__(
        self,
        number: int,
        mode: PinMode = PinMode.OUTPUT,
        *,
        name: str | None = None,
        inverted: bool = False,
        initial: PinState = PinState.LOW,
    ) -> None:
        """Initialize a digital pin.

        Args:
            number: The pin number
            mode: INPUT, OUTPUT, INPUT_PULLUP, or INPUT_PULLDOWN
            name: Optional name
            inverted: If True, invert logic
            initial: Initial state for output pins
        """
        if mode not in (
            PinMode.INPUT,
            PinMode.OUTPUT,
            PinMode.INPUT_PULLUP,
            PinMode.INPUT_PULLDOWN,
        ):
            raise ValueError(f"Invalid mode for DigitalPin: {mode}")
        super().__init__(number, mode, name=name, inverted=inverted)
        self._initial = initial
        self._state = initial

    @property
    def state(self) -> PinState:
        """Get current pin state."""
        return self._state

    @abstractmethod
    def read(self) -> bool:
        """Read the digital pin state.

        Returns:
            True if HIGH (or LOW if inverted), False otherwise
        """
        ...

    @abstractmethod
    def write(self, value: bool) -> None:
        """Write a digital value to the pin.

        Args:
            value: True for HIGH, False for LOW (inverted if self.inverted)
        """
        ...

    def high(self) -> None:
        """Set the pin HIGH."""
        self.write(True)

    def low(self) -> None:
        """Set the pin LOW."""
        self.write(False)

    def toggle(self) -> None:
        """Toggle the pin state."""
        self.write(not self.read())

    def on(self) -> None:
        """Alias for high()."""
        self.high()

    def off(self) -> None:
        """Alias for low()."""
        self.low()


class PWMPin(Pin):
    """Pin for PWM (Pulse Width Modulation) output."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        frequency: int = 50,
        duty_cycle: float = 0.0,
    ) -> None:
        """Initialize a PWM pin.

        Args:
            number: The pin number
            name: Optional name
            frequency: PWM frequency in Hz (default 50 for servos)
            duty_cycle: Initial duty cycle 0.0-1.0
        """
        super().__init__(number, PinMode.PWM, name=name)
        self._frequency = frequency
        self._duty_cycle = max(0.0, min(1.0, duty_cycle))

    @property
    def frequency(self) -> int:
        """Get PWM frequency in Hz."""
        return self._frequency

    @property
    def duty_cycle(self) -> float:
        """Get current duty cycle (0.0-1.0)."""
        return self._duty_cycle

    @abstractmethod
    def set_duty_cycle(self, duty: float) -> None:
        """Set the PWM duty cycle.

        Args:
            duty: Duty cycle from 0.0 (0%) to 1.0 (100%)
        """
        ...

    @abstractmethod
    def set_frequency(self, frequency: int) -> None:
        """Set the PWM frequency.

        Args:
            frequency: Frequency in Hz
        """
        ...

    @abstractmethod
    def start(self) -> None:
        """Start PWM output."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Stop PWM output."""
        ...

    def set_pulse_width(self, width_us: float) -> None:
        """Set pulse width in microseconds.

        Convenience method for servo control. Converts pulse width
        to duty cycle based on current frequency.

        Args:
            width_us: Pulse width in microseconds (e.g., 1500 for center)
        """
        period_us = 1_000_000 / self._frequency
        duty = width_us / period_us
        self.set_duty_cycle(duty)


class AnalogPin(Pin):
    """Pin for analog input (ADC) operations."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        resolution: int = 12,
        reference_voltage: float = 3.3,
    ) -> None:
        """Initialize an analog input pin.

        Args:
            number: The pin/channel number
            name: Optional name
            resolution: ADC resolution in bits (e.g., 10, 12, 16)
            reference_voltage: ADC reference voltage
        """
        super().__init__(number, PinMode.ANALOG, name=name)
        self._resolution = resolution
        self._reference_voltage = reference_voltage
        self._max_value = (1 << resolution) - 1

    @property
    def resolution(self) -> int:
        """Get ADC resolution in bits."""
        return self._resolution

    @property
    def reference_voltage(self) -> float:
        """Get ADC reference voltage."""
        return self._reference_voltage

    @property
    def max_value(self) -> int:
        """Get maximum raw ADC value."""
        return self._max_value

    @abstractmethod
    def read_raw(self) -> int:
        """Read raw ADC value.

        Returns:
            Raw integer value from 0 to max_value
        """
        ...

    def read(self) -> float:
        """Read voltage.

        Returns:
            Voltage based on reference and resolution
        """
        raw = self.read_raw()
        return (raw / self._max_value) * self._reference_voltage

    def read_normalized(self) -> float:
        """Read normalized value.

        Returns:
            Value from 0.0 to 1.0
        """
        return self.read_raw() / self._max_value


# =============================================================================
# Simulation Implementations
# =============================================================================


class SimulatedDigitalPin(DigitalPin):
    """Simulated digital pin for testing without hardware."""

    def __init__(
        self,
        number: int,
        mode: PinMode = PinMode.OUTPUT,
        *,
        name: str | None = None,
        inverted: bool = False,
        initial: PinState = PinState.LOW,
    ) -> None:
        super().__init__(number, mode, name=name, inverted=inverted, initial=initial)
        self._simulated_state = initial == PinState.HIGH

    def setup(self) -> None:
        """Initialize the simulated pin."""
        self._initialized = True
        self._simulated_state = self._initial == PinState.HIGH

    def cleanup(self) -> None:
        """Clean up the simulated pin."""
        self._simulated_state = False
        self._initialized = False

    def read(self) -> bool:
        """Read the simulated pin state."""
        value = self._simulated_state
        if self._inverted:
            value = not value
        self._state = PinState.HIGH if value else PinState.LOW
        return value

    def write(self, value: bool) -> None:
        """Write to the simulated pin."""
        if self._inverted:
            value = not value
        self._simulated_state = value
        self._state = PinState.HIGH if value else PinState.LOW


class SimulatedPWMPin(PWMPin):
    """Simulated PWM pin for testing without hardware."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        frequency: int = 50,
        duty_cycle: float = 0.0,
    ) -> None:
        super().__init__(number, name=name, frequency=frequency, duty_cycle=duty_cycle)
        self._running = False

    def setup(self) -> None:
        """Initialize the simulated PWM pin."""
        self._initialized = True

    def cleanup(self) -> None:
        """Clean up the simulated PWM pin."""
        self._running = False
        self._duty_cycle = 0.0
        self._initialized = False

    def set_duty_cycle(self, duty: float) -> None:
        """Set simulated duty cycle."""
        self._duty_cycle = max(0.0, min(1.0, duty))

    def set_frequency(self, frequency: int) -> None:
        """Set simulated frequency."""
        self._frequency = frequency

    def start(self) -> None:
        """Start simulated PWM."""
        self._running = True

    def stop(self) -> None:
        """Stop simulated PWM."""
        self._running = False

    @property
    def running(self) -> bool:
        """Check if PWM is running."""
        return self._running


class SimulatedAnalogPin(AnalogPin):
    """Simulated analog pin for testing without hardware."""

    def __init__(
        self,
        number: int,
        *,
        name: str | None = None,
        resolution: int = 12,
        reference_voltage: float = 3.3,
        initial_value: float = 0.0,
    ) -> None:
        super().__init__(
            number, name=name, resolution=resolution, reference_voltage=reference_voltage
        )
        self._simulated_voltage = initial_value

    def setup(self) -> None:
        """Initialize the simulated analog pin."""
        self._initialized = True

    def cleanup(self) -> None:
        """Clean up the simulated analog pin."""
        self._simulated_voltage = 0.0
        self._initialized = False

    def read_raw(self) -> int:
        """Read simulated raw ADC value."""
        normalized = self._simulated_voltage / self._reference_voltage
        normalized = max(0.0, min(1.0, normalized))
        return int(normalized * self._max_value)

    def set_simulated_voltage(self, voltage: float) -> None:
        """Set the simulated voltage for testing.

        Args:
            voltage: Voltage to simulate (clamped to reference)
        """
        self._simulated_voltage = max(0.0, min(self._reference_voltage, voltage))

    def set_simulated_normalized(self, value: float) -> None:
        """Set simulated value as normalized 0-1.

        Args:
            value: Normalized value 0.0 to 1.0
        """
        self._simulated_voltage = max(0.0, min(1.0, value)) * self._reference_voltage


# =============================================================================
# Pin Factory
# =============================================================================

_platform_detected: str | None = None


def detect_platform() -> str:
    """Detect the current hardware platform.

    Returns:
        Platform identifier: 'raspberry_pi', 'jetson', 'beaglebone', or 'simulation'
    """
    global _platform_detected  # noqa: PLW0603
    if _platform_detected is not None:
        return _platform_detected

    # Try to detect Raspberry Pi
    cpuinfo_path = Path("/proc/cpuinfo")
    if cpuinfo_path.exists():
        try:
            cpuinfo = cpuinfo_path.read_text()
            if "BCM" in cpuinfo or "Raspberry" in cpuinfo:
                _platform_detected = "raspberry_pi"
                return _platform_detected
        except (PermissionError, OSError):
            pass

    # Try to detect Jetson
    tegra_path = Path("/etc/nv_tegra_release")
    if tegra_path.exists():
        _platform_detected = "jetson"
        return _platform_detected

    # Default to simulation
    _platform_detected = "simulation"
    return _platform_detected


def get_digital_pin(
    number: int,
    mode: PinMode = PinMode.OUTPUT,
    *,
    name: str | None = None,
    inverted: bool = False,
    initial: PinState = PinState.LOW,
    platform: str | None = None,
) -> DigitalPin:
    """Get a digital pin for the current platform.

    Args:
        number: Pin number
        mode: Pin mode
        name: Optional name
        inverted: Invert logic
        initial: Initial state
        platform: Force specific platform (None for auto-detect)

    Returns:
        DigitalPin instance appropriate for the platform
    """
    platform = platform or detect_platform()

    if platform == "simulation":
        return SimulatedDigitalPin(number, mode, name=name, inverted=inverted, initial=initial)

    # For real hardware, we'd return platform-specific implementations
    # For now, fall back to simulation
    return SimulatedDigitalPin(number, mode, name=name, inverted=inverted, initial=initial)


def get_pwm_pin(
    number: int,
    *,
    name: str | None = None,
    frequency: int = 50,
    duty_cycle: float = 0.0,
    platform: str | None = None,
) -> PWMPin:
    """Get a PWM pin for the current platform.

    Args:
        number: Pin number
        name: Optional name
        frequency: PWM frequency in Hz
        duty_cycle: Initial duty cycle
        platform: Force specific platform (None for auto-detect)

    Returns:
        PWMPin instance appropriate for the platform
    """
    platform = platform or detect_platform()

    if platform == "simulation":
        return SimulatedPWMPin(number, name=name, frequency=frequency, duty_cycle=duty_cycle)

    # For real hardware, we'd return platform-specific implementations
    return SimulatedPWMPin(number, name=name, frequency=frequency, duty_cycle=duty_cycle)


def get_analog_pin(
    number: int,
    *,
    name: str | None = None,
    resolution: int = 12,
    reference_voltage: float = 3.3,
    platform: str | None = None,
) -> AnalogPin:
    """Get an analog input pin for the current platform.

    Args:
        number: Pin/channel number
        name: Optional name
        resolution: ADC resolution in bits
        reference_voltage: ADC reference voltage
        platform: Force specific platform (None for auto-detect)

    Returns:
        AnalogPin instance appropriate for the platform
    """
    platform = platform or detect_platform()

    if platform == "simulation":
        return SimulatedAnalogPin(
            number, name=name, resolution=resolution, reference_voltage=reference_voltage
        )

    # For real hardware, we'd return platform-specific implementations
    return SimulatedAnalogPin(
        number, name=name, resolution=resolution, reference_voltage=reference_voltage
    )
