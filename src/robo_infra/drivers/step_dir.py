"""A4988 and DRV8825 step/direction stepper drivers.

This module provides drivers for simple step/direction stepper motor controllers
like the A4988 and DRV8825, commonly used in 3D printers and CNC machines.

These drivers use GPIO pins for step, direction, and enable signals, making
them simple to use but without advanced features like UART configuration.

Example:
    >>> from robo_infra.drivers.step_dir import A4988Driver, DRV8825Driver
    >>>
    >>> # Create A4988 driver
    >>> driver = A4988Driver(
    ...     step_pin=17,
    ...     dir_pin=27,
    ...     enable_pin=22,
    ...     ms_pins=(5, 6, 13),  # MS1, MS2, MS3 for microstepping
    ... )
    >>> driver.connect()
    >>>
    >>> # Configure microstepping
    >>> driver.set_microstepping(16)
    >>>
    >>> # Enable and step
    >>> driver.enable()
    >>> driver.set_direction(forward=True)
    >>> driver.step(100)  # 100 steps
    >>>
    >>> driver.disable()
    >>> driver.disconnect()

Hardware Reference:
    A4988:
        - Supply: 8-35V motor, 3-5.5V logic
        - Current: up to 2A (with heatsink)
        - Microstepping: 1, 2, 4, 8, 16

    DRV8825:
        - Supply: 8.2-45V motor, 3.3-5.25V logic
        - Current: up to 2.5A (with heatsink)
        - Microstepping: 1, 2, 4, 8, 16, 32
"""

from __future__ import annotations

import logging
import os
import time
from abc import ABC
from dataclasses import dataclass
from typing import TYPE_CHECKING

from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import CommunicationError


if TYPE_CHECKING:
    from robo_infra.core.pin import Pin

logger = logging.getLogger(__name__)


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class StepDirConfig:
    """Configuration for step/direction drivers.

    Attributes:
        step_pin: GPIO pin for step signal.
        dir_pin: GPIO pin for direction signal.
        enable_pin: GPIO pin for enable signal (active low).
        ms_pins: Tuple of GPIO pins for microstepping (MS1, MS2, MS3).
        steps_per_rev: Full steps per revolution (typically 200).
        step_pulse_us: Step pulse width in microseconds.
        step_delay_us: Delay between steps in microseconds.
        invert_dir: Invert direction signal.
        invert_enable: Invert enable signal (default True = active low).
        name: Optional name for the driver.
    """

    step_pin: int = 0
    dir_pin: int = 1
    enable_pin: int | None = None
    ms_pins: tuple[int, ...] | None = None
    steps_per_rev: int = 200
    step_pulse_us: int = 2
    step_delay_us: int = 100
    invert_dir: bool = False
    invert_enable: bool = True
    name: str = "StepDirDriver"


# =============================================================================
# Base Step/Dir Driver
# =============================================================================


class StepDirDriver(Driver, ABC):
    """Base class for step/direction stepper drivers.

    This provides common functionality for simple GPIO-based stepper drivers
    that use step, direction, and enable signals.
    """

    # Subclasses define supported microstepping values
    MICROSTEP_TABLE: dict[int, tuple[bool, ...]] = {}

    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        enable_pin: int | None = None,
        ms_pins: tuple[int, ...] | None = None,
        config: StepDirConfig | None = None,
        simulation: bool | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize step/direction driver.

        Args:
            step_pin: GPIO pin for step signal.
            dir_pin: GPIO pin for direction signal.
            enable_pin: GPIO pin for enable signal.
            ms_pins: Tuple of GPIO pins for microstepping.
            config: Driver configuration.
            simulation: If True, use simulation mode.
            name: Optional human-readable name.
        """
        if simulation is None:
            simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

        super().__init__(
            config=DriverConfig(
                name=name or self.__class__.__name__,
                channels=1,
                auto_connect=False,
            )
        )

        self._step_pin_num = step_pin
        self._dir_pin_num = dir_pin
        self._enable_pin_num = enable_pin
        self._ms_pin_nums = ms_pins

        self._config = config or StepDirConfig(
            step_pin=step_pin,
            dir_pin=dir_pin,
            enable_pin=enable_pin,
            ms_pins=ms_pins,
        )

        self._simulation = simulation
        self._enabled = False
        self._direction = True  # True = forward
        self._microstepping = 1
        self._position = 0  # Track position in steps

        # GPIO pin objects (set on connect)
        self._step_gpio: Pin | None = None
        self._dir_gpio: Pin | None = None
        self._enable_gpio: Pin | None = None
        self._ms_gpios: list[Pin] = []

    @property
    def simulation(self) -> bool:
        """Whether running in simulation mode."""
        return self._simulation

    @property
    def position(self) -> int:
        """Current position in steps."""
        return self._position

    @property
    def enabled(self) -> bool:
        """Whether the driver is enabled."""
        return self._enabled

    @property
    def direction(self) -> bool:
        """Current direction (True = forward)."""
        return self._direction

    @property
    def microstepping(self) -> int:
        """Current microstepping value."""
        return self._microstepping

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect and initialize GPIO pins."""
        if self._simulation:
            logger.info("%s connecting in simulation mode", self._config.name)
            self._state = DriverState.CONNECTED
            return

        try:
            from robo_infra.core.pin import get_gpio

            # Initialize step pin
            self._step_gpio = get_gpio(self._step_pin_num, mode="output")
            self._step_gpio.low()

            # Initialize direction pin
            self._dir_gpio = get_gpio(self._dir_pin_num, mode="output")
            self._dir_gpio.low()

            # Initialize enable pin if specified
            if self._enable_pin_num is not None:
                self._enable_gpio = get_gpio(self._enable_pin_num, mode="output")
                # Disable by default (active low, so set high)
                if self._config.invert_enable:
                    self._enable_gpio.high()
                else:
                    self._enable_gpio.low()

            # Initialize microstepping pins if specified
            self._ms_gpios = []
            if self._ms_pin_nums:
                for pin_num in self._ms_pin_nums:
                    gpio = get_gpio(pin_num, mode="output")
                    gpio.low()
                    self._ms_gpios.append(gpio)

        except Exception as e:
            raise CommunicationError(f"Failed to initialize GPIO: {e}") from e

        self._state = DriverState.CONNECTED
        logger.info("%s connected", self._config.name)

    def disconnect(self) -> None:
        """Disconnect and release GPIO pins."""
        self.disable()
        self._state = DriverState.DISCONNECTED
        logger.info("%s disconnected", self._config.name)

    # -------------------------------------------------------------------------
    # Enable/Disable
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the driver (allow motor current)."""
        if self._simulation:
            self._enabled = True
            logger.debug("%s enabled", self._config.name)
            return

        if self._enable_gpio is not None:
            if self._config.invert_enable:
                self._enable_gpio.low()  # Active low
            else:
                self._enable_gpio.high()

        self._enabled = True
        logger.debug("%s enabled", self._config.name)

    def disable(self) -> None:
        """Disable the driver (remove motor current)."""
        if self._simulation:
            self._enabled = False
            logger.debug("%s disabled", self._config.name)
            return

        if self._enable_gpio is not None:
            if self._config.invert_enable:
                self._enable_gpio.high()  # Active low
            else:
                self._enable_gpio.low()

        self._enabled = False
        logger.debug("%s disabled", self._config.name)

    # -------------------------------------------------------------------------
    # Direction Control
    # -------------------------------------------------------------------------

    def set_direction(self, forward: bool = True) -> None:
        """Set the motor direction.

        Args:
            forward: If True, move in forward direction.
        """
        self._direction = forward

        if self._simulation:
            logger.debug("%s direction: %s", self._config.name, "forward" if forward else "reverse")
            return

        if self._dir_gpio is not None:
            actual = forward
            if self._config.invert_dir:
                actual = not actual

            if actual:
                self._dir_gpio.high()
            else:
                self._dir_gpio.low()

    # -------------------------------------------------------------------------
    # Stepping
    # -------------------------------------------------------------------------

    def step(self, steps: int = 1) -> None:
        """Generate step pulses.

        Args:
            steps: Number of steps to take. Negative for reverse.
        """
        if steps < 0:
            self.set_direction(False)
            steps = abs(steps)
        elif self._direction is False:
            self.set_direction(True)

        pulse_time = self._config.step_pulse_us / 1_000_000
        delay_time = self._config.step_delay_us / 1_000_000

        for _ in range(steps):
            self._single_step(pulse_time, delay_time)

            if self._direction:
                self._position += 1
            else:
                self._position -= 1

    def _single_step(self, pulse_time: float, delay_time: float) -> None:
        """Generate a single step pulse.

        Args:
            pulse_time: Pulse width in seconds.
            delay_time: Delay after pulse in seconds.
        """
        if self._simulation:
            time.sleep(pulse_time + delay_time)
            return

        if self._step_gpio is not None:
            self._step_gpio.high()
            time.sleep(pulse_time)
            self._step_gpio.low()
            time.sleep(delay_time)

    def move_to(self, position: int) -> None:
        """Move to an absolute position.

        Args:
            position: Target position in steps.
        """
        delta = position - self._position
        self.step(delta)

    def reset_position(self, position: int = 0) -> None:
        """Reset the position counter.

        Args:
            position: New position value.
        """
        self._position = position

    # -------------------------------------------------------------------------
    # Microstepping
    # -------------------------------------------------------------------------

    def set_microstepping(self, microsteps: int) -> None:
        """Set the microstepping resolution.

        Args:
            microsteps: Microsteps per full step (1, 2, 4, 8, 16, 32).

        Raises:
            ValueError: If microsteps is not supported.
        """
        if microsteps not in self.MICROSTEP_TABLE:
            valid = list(self.MICROSTEP_TABLE.keys())
            raise ValueError(f"Microsteps {microsteps} not supported. Use: {valid}")

        self._microstepping = microsteps

        if self._simulation:
            logger.info("%s set microstepping: %d", self._config.name, microsteps)
            return

        if not self._ms_gpios:
            logger.warning("No microstepping pins configured")
            return

        pin_states = self.MICROSTEP_TABLE[microsteps]

        for i, gpio in enumerate(self._ms_gpios):
            if i < len(pin_states):
                if pin_states[i]:
                    gpio.high()
                else:
                    gpio.low()

        logger.debug("Set microstepping=%d, pins=%s", microsteps, pin_states)

    def get_microstepping(self) -> int:
        """Get the current microstepping value.

        Returns:
            Microsteps per full step.
        """
        return self._microstepping

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def get_status(self) -> dict:
        """Get driver status.

        Returns:
            Dictionary with status information.
        """
        return {
            "enabled": self._enabled,
            "direction": "forward" if self._direction else "reverse",
            "position": self._position,
            "microstepping": self._microstepping,
            "simulation": self._simulation,
        }

    # -------------------------------------------------------------------------
    # Driver Abstract Methods
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write to channel (steps forward/backward based on value)."""
        steps = int(value * 100)  # Scale value to steps
        self.step(steps)

    def _read_channel(self, channel: int) -> float:
        """Read channel (returns normalized position)."""
        return self._position / 1000.0


# =============================================================================
# A4988 Driver
# =============================================================================


@register_driver("a4988")
class A4988Driver(StepDirDriver):
    """Driver for A4988 stepper motor controller.

    The A4988 is a common stepper driver with:
    - Up to 2A output current
    - 1/1, 1/2, 1/4, 1/8, 1/16 microstepping
    - Built-in overcurrent protection
    - Thermal shutdown

    Microstepping is controlled by MS1, MS2, MS3 pins:
        MS1=L, MS2=L, MS3=L -> Full step (1)
        MS1=H, MS2=L, MS3=L -> Half step (2)
        MS1=L, MS2=H, MS3=L -> Quarter step (4)
        MS1=H, MS2=H, MS3=L -> Eighth step (8)
        MS1=H, MS2=H, MS3=H -> Sixteenth step (16)
    """

    MICROSTEP_TABLE = {
        1: (False, False, False),  # Full step
        2: (True, False, False),  # 1/2 step
        4: (False, True, False),  # 1/4 step
        8: (True, True, False),  # 1/8 step
        16: (True, True, True),  # 1/16 step
    }

    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        enable_pin: int | None = None,
        ms_pins: tuple[int, int, int] | None = None,
        simulation: bool | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize A4988 driver.

        Args:
            step_pin: GPIO pin for step signal.
            dir_pin: GPIO pin for direction signal.
            enable_pin: GPIO pin for enable signal (active low).
            ms_pins: Tuple of (MS1, MS2, MS3) GPIO pins.
            simulation: If True, use simulation mode.
            name: Optional human-readable name.
        """
        super().__init__(
            step_pin=step_pin,
            dir_pin=dir_pin,
            enable_pin=enable_pin,
            ms_pins=ms_pins,
            simulation=simulation,
            name=name or "A4988",
        )


# =============================================================================
# DRV8825 Driver
# =============================================================================


@register_driver("drv8825")
class DRV8825Driver(StepDirDriver):
    """Driver for DRV8825 stepper motor controller.

    The DRV8825 is an upgraded version of A4988 with:
    - Up to 2.5A output current
    - 1/1, 1/2, 1/4, 1/8, 1/16, 1/32 microstepping
    - Higher voltage range (8.2-45V)
    - Built-in indexer
    - Fault output pin

    Microstepping is controlled by M0, M1, M2 pins:
        M0=L, M1=L, M2=L -> Full step (1)
        M0=H, M1=L, M2=L -> Half step (2)
        M0=L, M1=H, M2=L -> Quarter step (4)
        M0=H, M1=H, M2=L -> Eighth step (8)
        M0=L, M1=L, M2=H -> Sixteenth step (16)
        M0=H, M1=L, M2=H -> 1/32 step (32)
    """

    MICROSTEP_TABLE = {
        1: (False, False, False),  # Full step
        2: (True, False, False),  # 1/2 step
        4: (False, True, False),  # 1/4 step
        8: (True, True, False),  # 1/8 step
        16: (False, False, True),  # 1/16 step
        32: (True, False, True),  # 1/32 step
    }

    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        enable_pin: int | None = None,
        ms_pins: tuple[int, int, int] | None = None,
        fault_pin: int | None = None,
        simulation: bool | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize DRV8825 driver.

        Args:
            step_pin: GPIO pin for step signal.
            dir_pin: GPIO pin for direction signal.
            enable_pin: GPIO pin for enable signal (active low).
            ms_pins: Tuple of (M0, M1, M2) GPIO pins.
            fault_pin: GPIO pin for fault input (active low).
            simulation: If True, use simulation mode.
            name: Optional human-readable name.
        """
        super().__init__(
            step_pin=step_pin,
            dir_pin=dir_pin,
            enable_pin=enable_pin,
            ms_pins=ms_pins,
            simulation=simulation,
            name=name or "DRV8825",
        )

        self._fault_pin_num = fault_pin
        self._fault_gpio: Pin | None = None

    def connect(self) -> None:
        """Connect and initialize GPIO pins."""
        super().connect()

        if self._simulation:
            return

        # Initialize fault pin if specified
        if self._fault_pin_num is not None:
            try:
                from robo_infra.core.pin import get_gpio

                self._fault_gpio = get_gpio(self._fault_pin_num, mode="input")
            except Exception as e:
                logger.warning("Failed to initialize fault pin: %s", e)

    def is_fault(self) -> bool:
        """Check if driver has a fault condition.

        Returns:
            True if fault is detected (overcurrent, thermal shutdown, etc.).
        """
        if self._simulation:
            return False

        if self._fault_gpio is None:
            return False

        # Fault pin is active low
        return not self._fault_gpio.read()

    def get_status(self) -> dict:
        """Get driver status including fault state.

        Returns:
            Dictionary with status information.
        """
        status = super().get_status()
        status["fault"] = self.is_fault()
        return status
