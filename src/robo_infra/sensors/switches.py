"""Switch sensor implementations.

Phase 4.4 provides switch-based sensors:
- LimitSwitch (mechanical limit detection with debouncing)
- Button (user input with press/release events)
- HallEffect (magnetic field detection, digital and analog)

All switches extend `Sensor` via a shared `Switch` base class.

Notes:
- The core `Sensor` abstraction expects `_read_raw() -> int`.
- Switches return binary state (0 or 1) as the raw value.
- Async wait methods use polling with configurable intervals.
"""

from __future__ import annotations

import asyncio
import logging
import time
from abc import abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.sensor import Sensor
from robo_infra.core.types import Limits, Unit


logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from collections.abc import Callable

    from robo_infra.core.driver import Driver
    from robo_infra.core.pin import AnalogPin, DigitalPin


# -----------------------------------------------------------------------------
# Enums
# -----------------------------------------------------------------------------


class SwitchState(Enum):
    """State of a switch."""

    OPEN = "open"
    CLOSED = "closed"


class TriggerEdge(Enum):
    """Edge trigger type."""

    RISING = "rising"
    FALLING = "falling"
    BOTH = "both"


# -----------------------------------------------------------------------------
# Base class
# -----------------------------------------------------------------------------


class Switch(Sensor):
    """Base class for switch sensors.

    Switches are binary sensors that detect on/off, open/closed,
    or triggered/not-triggered states.

    Provides:
    - `is_active()`: Check if switch is in active state
    - `wait_for_state()`: Async wait for specific state
    - Debouncing support for mechanical switches
    """

    def __init__(
        self,
        name: str,
        *,
        pin: DigitalPin | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        normally_open: bool = True,
        debounce_ms: float = 10.0,
        **kwargs: Any,
    ) -> None:
        """Initialize switch.

        Args:
            name: Sensor name
            pin: Digital input pin
            driver: Optional driver for hardware communication
            channel: Driver channel number
            normally_open: True if switch is normally open (NO)
            debounce_ms: Debounce time in milliseconds
            **kwargs: Additional Sensor arguments
        """
        super().__init__(
            name=name,
            driver=driver,
            channel=channel,
            unit=Unit.COUNT,
            limits=Limits(min=0, max=1),
            **kwargs,
        )
        self._pin = pin
        self._normally_open = normally_open
        self._debounce_ms = debounce_ms
        self._last_change_time: float = 0.0
        self._debounced_state: bool = False
        self._raw_state: bool = False

    @property
    def normally_open(self) -> bool:
        """True if switch is normally open (NO)."""
        return self._normally_open

    @property
    def debounce_ms(self) -> float:
        """Debounce time in milliseconds."""
        return self._debounce_ms

    @debounce_ms.setter
    def debounce_ms(self, value: float) -> None:
        """Set debounce time."""
        self._debounce_ms = max(0.0, value)

    @abstractmethod
    def _read_pin_state(self) -> bool:
        """Read raw pin state without debouncing.

        Returns:
            Raw pin state (True = HIGH, False = LOW).
        """
        ...

    def _read_raw(self) -> int:
        """Read raw switch state with debouncing.

        Returns:
            1 if switch is active/triggered, 0 otherwise.
        """
        current_raw = self._read_pin_state()
        now = time.time()

        # Check if state changed
        if current_raw != self._raw_state:
            self._last_change_time = now
            self._raw_state = current_raw

        # Apply debouncing
        elapsed_ms = (now - self._last_change_time) * 1000
        if elapsed_ms >= self._debounce_ms:
            self._debounced_state = self._raw_state

        # Determine active state based on normally_open
        # NO switch: active when closed (HIGH), NC switch: active when open (LOW)
        is_active = self._debounced_state if self._normally_open else not self._debounced_state

        return 1 if is_active else 0

    def is_active(self) -> bool:
        """Check if switch is in active/triggered state.

        Takes debouncing into account.

        Returns:
            True if switch is active.
        """
        reading = self.read()
        return reading.value > 0

    def get_state(self) -> SwitchState:
        """Get switch state.

        Returns:
            SwitchState.CLOSED if active, SwitchState.OPEN otherwise.
        """
        return SwitchState.CLOSED if self.is_active() else SwitchState.OPEN

    async def wait_for_state(
        self,
        active: bool,
        *,
        timeout: float = 30.0,  # Default 30s timeout for safety
        poll_interval: float = 0.01,
        max_iterations: int = 100_000,  # Safety limit
    ) -> bool:
        """Wait for switch to reach specified state.

        Args:
            active: Target state (True = active/triggered)
            timeout: Maximum wait time in seconds (default 30s)
            poll_interval: Polling interval in seconds
            max_iterations: Safety limit to prevent infinite loops

        Returns:
            True if target state reached, False if timeout.

        Note:
            Always has a timeout for safety. Set timeout to a high value
            if you need longer waits, but never infinite.
        """
        start = time.time()
        iterations = 0

        while iterations < max_iterations:
            if self.is_active() == active:
                return True

            if (time.time() - start) >= timeout:
                return False

            await asyncio.sleep(poll_interval)
            iterations += 1

        logger.warning(
            "wait_for_state reached max_iterations limit (%d) - stopping",
            max_iterations,
        )
        return False


# -----------------------------------------------------------------------------
# Limit Switch
# -----------------------------------------------------------------------------


class LimitSwitchConfig(BaseModel):
    """Configuration for limit switches."""

    name: str = "LimitSwitch"
    normally_open: bool = Field(default=True, description="True if normally open (NO)")
    debounce_ms: float = Field(default=10.0, ge=0, description="Debounce time in ms")
    invert_logic: bool = Field(default=False, description="Invert the logic")

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class LimitSwitchStatus:
    """Status of a limit switch."""

    triggered: bool = False
    state: SwitchState = SwitchState.OPEN
    trigger_count: int = 0
    last_trigger_time: float | None = None


class LimitSwitch(Switch):
    """Mechanical limit switch for end-stop detection.

    Commonly used for:
    - Homing routines (axis end detection)
    - Safety limits (over-travel protection)
    - Position sensing (object presence)

    Supports:
    - Normally open (NO) or normally closed (NC) configurations
    - Hardware debouncing via time-based filtering
    - Trigger counting for diagnostics

    Example:
        >>> from robo_infra.core.pin import SimulatedDigitalPin, PinMode
        >>> pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        >>> limit = LimitSwitch(pin=pin, normally_open=True)
        >>> limit.enable()
        >>> if limit.is_triggered():
        ...     print("Limit reached!")
    """

    def __init__(
        self,
        *,
        pin: DigitalPin | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        name: str = "LimitSwitch",
        normally_open: bool = True,
        debounce_ms: float = 10.0,
        config: LimitSwitchConfig | None = None,
    ) -> None:
        """Initialize limit switch.

        Args:
            pin: Digital input pin
            driver: Alternative driver
            channel: Driver channel number
            name: Switch name
            normally_open: True if normally open (NO)
            debounce_ms: Debounce time in milliseconds
            config: Full configuration object
        """
        if config is not None:
            self._config = config
            name = config.name
            normally_open = config.normally_open
            debounce_ms = config.debounce_ms
        else:
            self._config = LimitSwitchConfig(
                name=name,
                normally_open=normally_open,
                debounce_ms=debounce_ms,
            )

        self._status = LimitSwitchStatus()
        self._last_triggered: bool = False

        super().__init__(
            name=name,
            pin=pin,
            driver=driver,
            channel=channel,
            normally_open=normally_open,
            debounce_ms=debounce_ms,
        )

    def enable(self) -> None:
        """Enable limit switch and set up pin."""
        super().enable()
        if self._pin is not None and not self._pin.initialized:
            self._pin.setup()

    def _read_pin_state(self) -> bool:
        """Read raw pin state."""
        if self._pin is not None:
            state = self._pin.read()
            if self._config.invert_logic:
                state = not state
            return state
        elif self._driver is not None:
            return bool(self._driver.get_channel(self._channel))
        return False

    def _read_raw(self) -> int:
        """Read raw state and update trigger tracking."""
        result = super()._read_raw()
        is_triggered = result > 0

        # Track trigger events
        if is_triggered and not self._last_triggered:
            self._status.trigger_count += 1
            self._status.last_trigger_time = time.time()

        self._last_triggered = is_triggered
        self._status.triggered = is_triggered
        self._status.state = SwitchState.CLOSED if is_triggered else SwitchState.OPEN

        return result

    def is_triggered(self) -> bool:
        """Check if limit switch is triggered.

        Alias for is_active().

        Returns:
            True if limit is triggered.
        """
        return self.is_active()

    async def wait_for_trigger(
        self,
        *,
        timeout: float = 30.0,
        poll_interval: float = 0.01,
    ) -> bool:
        """Wait for limit switch to be triggered.

        Args:
            timeout: Maximum wait time in seconds
            poll_interval: Polling interval in seconds

        Returns:
            True if triggered, False if timeout.
        """
        return await self.wait_for_state(True, timeout=timeout, poll_interval=poll_interval)

    async def wait_for_release(
        self,
        *,
        timeout: float = 30.0,
        poll_interval: float = 0.01,
    ) -> bool:
        """Wait for limit switch to be released.

        Args:
            timeout: Maximum wait time in seconds
            poll_interval: Polling interval in seconds

        Returns:
            True if released, False if timeout.
        """
        return await self.wait_for_state(False, timeout=timeout, poll_interval=poll_interval)

    def reset_trigger_count(self) -> None:
        """Reset the trigger counter."""
        self._status.trigger_count = 0

    def status(self) -> LimitSwitchStatus:  # type: ignore[override]
        """Get limit switch status."""
        return self._status


# -----------------------------------------------------------------------------
# Button
# -----------------------------------------------------------------------------


class ButtonConfig(BaseModel):
    """Configuration for buttons."""

    name: str = "Button"
    normally_open: bool = Field(default=True, description="True if normally open")
    debounce_ms: float = Field(default=50.0, ge=0, description="Debounce time in ms")
    long_press_ms: float = Field(default=500.0, ge=0, description="Long press threshold")
    invert_logic: bool = Field(default=False, description="Invert the logic")

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class ButtonStatus:
    """Status of a button."""

    pressed: bool = False
    press_count: int = 0
    last_press_time: float | None = None
    last_release_time: float | None = None
    press_duration: float = 0.0


class Button(Switch):
    """Push button for user input.

    Provides:
    - Press/release detection with debouncing
    - Press counting and timing
    - Long press detection
    - Callback-based event handling

    Example:
        >>> from robo_infra.core.pin import SimulatedDigitalPin, PinMode
        >>> pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        >>> button = Button(pin=pin)
        >>> button.enable()
        >>> button.on_press(lambda: print("Button pressed!"))
        >>> if button.is_pressed():
        ...     print("Currently pressed")
    """

    def __init__(
        self,
        *,
        pin: DigitalPin | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        name: str = "Button",
        normally_open: bool = True,
        debounce_ms: float = 50.0,
        config: ButtonConfig | None = None,
    ) -> None:
        """Initialize button.

        Args:
            pin: Digital input pin
            driver: Alternative driver
            channel: Driver channel number
            name: Button name
            normally_open: True if normally open
            debounce_ms: Debounce time in milliseconds
            config: Full configuration object
        """
        if config is not None:
            self._config = config
            name = config.name
            normally_open = config.normally_open
            debounce_ms = config.debounce_ms
        else:
            self._config = ButtonConfig(
                name=name,
                normally_open=normally_open,
                debounce_ms=debounce_ms,
            )

        self._status = ButtonStatus()
        self._last_pressed: bool = False
        self._press_start_time: float = 0.0

        # Event callbacks
        self._on_press_callbacks: list[Callable[[], None]] = []
        self._on_release_callbacks: list[Callable[[], None]] = []
        self._on_long_press_callbacks: list[Callable[[], None]] = []

        super().__init__(
            name=name,
            pin=pin,
            driver=driver,
            channel=channel,
            normally_open=normally_open,
            debounce_ms=debounce_ms,
        )

    def enable(self) -> None:
        """Enable button and set up pin."""
        super().enable()
        if self._pin is not None and not self._pin.initialized:
            self._pin.setup()

    def _read_pin_state(self) -> bool:
        """Read raw pin state."""
        if self._pin is not None:
            state = self._pin.read()
            if self._config.invert_logic:
                state = not state
            return state
        elif self._driver is not None:
            return bool(self._driver.get_channel(self._channel))
        return False

    def _read_raw(self) -> int:
        """Read raw state and handle events."""
        result = super()._read_raw()
        is_pressed = result > 0
        now = time.time()

        # Detect press event (rising edge)
        if is_pressed and not self._last_pressed:
            self._status.press_count += 1
            self._status.last_press_time = now
            self._press_start_time = now
            self._fire_callbacks(self._on_press_callbacks)

        # Detect release event (falling edge)
        if not is_pressed and self._last_pressed:
            self._status.last_release_time = now
            self._status.press_duration = now - self._press_start_time

            # Check for long press
            if self._status.press_duration >= (self._config.long_press_ms / 1000):
                self._fire_callbacks(self._on_long_press_callbacks)

            self._fire_callbacks(self._on_release_callbacks)

        self._last_pressed = is_pressed
        self._status.pressed = is_pressed

        return result

    def _fire_callbacks(self, callbacks: list[Callable[[], None]]) -> None:
        """Fire all callbacks in a list."""
        for callback in callbacks:
            try:
                callback()
            except Exception as e:
                # Callbacks are user code - log but don't crash
                logger.warning("Button callback failed: %s", e)

    def is_pressed(self) -> bool:
        """Check if button is currently pressed.

        Returns:
            True if button is pressed.
        """
        return self.is_active()

    def is_held(self) -> bool:
        """Check if button is being held (long press).

        Returns:
            True if button is held past long_press_ms threshold.
        """
        if not self.is_pressed():
            return False
        duration = time.time() - self._press_start_time
        return duration >= (self._config.long_press_ms / 1000)

    async def wait_for_press(
        self,
        *,
        timeout: float = 30.0,
        poll_interval: float = 0.01,
    ) -> bool:
        """Wait for button to be pressed.

        Args:
            timeout: Maximum wait time in seconds
            poll_interval: Polling interval in seconds

        Returns:
            True if pressed, False if timeout.
        """
        return await self.wait_for_state(True, timeout=timeout, poll_interval=poll_interval)

    async def wait_for_release(
        self,
        *,
        timeout: float = 30.0,
        poll_interval: float = 0.01,
    ) -> bool:
        """Wait for button to be released.

        Args:
            timeout: Maximum wait time in seconds
            poll_interval: Polling interval in seconds

        Returns:
            True if released, False if timeout.
        """
        return await self.wait_for_state(False, timeout=timeout, poll_interval=poll_interval)

    async def wait_for_click(
        self,
        *,
        timeout: float = 30.0,
        poll_interval: float = 0.01,
    ) -> bool:
        """Wait for a complete click (press then release).

        Args:
            timeout: Maximum wait time in seconds
            poll_interval: Polling interval in seconds

        Returns:
            True if clicked, False if timeout.
        """
        start = time.time()

        # Wait for press
        remaining = timeout - (time.time() - start)
        if remaining <= 0:
            return False
        if not await self.wait_for_press(timeout=remaining, poll_interval=poll_interval):
            return False

        remaining = timeout - (time.time() - start)
        if remaining <= 0:
            return False

        # Wait for release
        return await self.wait_for_release(timeout=remaining, poll_interval=poll_interval)

    def on_press(self, callback: Callable[[], None]) -> None:
        """Register callback for press events.

        Args:
            callback: Function to call when button is pressed.
        """
        self._on_press_callbacks.append(callback)

    def on_release(self, callback: Callable[[], None]) -> None:
        """Register callback for release events.

        Args:
            callback: Function to call when button is released.
        """
        self._on_release_callbacks.append(callback)

    def on_long_press(self, callback: Callable[[], None]) -> None:
        """Register callback for long press events.

        Called when button is released after being held for long_press_ms.

        Args:
            callback: Function to call on long press.
        """
        self._on_long_press_callbacks.append(callback)

    def clear_callbacks(self) -> None:
        """Remove all registered callbacks."""
        self._on_press_callbacks.clear()
        self._on_release_callbacks.clear()
        self._on_long_press_callbacks.clear()

    def reset_press_count(self) -> None:
        """Reset the press counter."""
        self._status.press_count = 0

    def status(self) -> ButtonStatus:  # type: ignore[override]
        """Get button status."""
        return self._status


# -----------------------------------------------------------------------------
# Hall Effect Sensor
# -----------------------------------------------------------------------------


class HallEffectMode(Enum):
    """Operating mode for Hall effect sensors."""

    DIGITAL = "digital"
    ANALOG = "analog"


class HallEffectConfig(BaseModel):
    """Configuration for Hall effect sensors."""

    name: str = "HallEffect"
    mode: HallEffectMode = Field(default=HallEffectMode.DIGITAL)
    normally_open: bool = Field(default=True, description="For digital mode")
    debounce_ms: float = Field(default=5.0, ge=0, description="Debounce time for digital")

    # Analog settings
    threshold: float = Field(default=0.5, ge=0, le=1, description="Analog threshold (0-1)")
    hysteresis: float = Field(default=0.1, ge=0, description="Hysteresis band")

    # Polarity
    active_high: bool = Field(default=True, description="Active on north pole (high field)")

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class HallEffectStatus:
    """Status of a Hall effect sensor."""

    detected: bool = False
    analog_value: float = 0.0
    state: SwitchState = SwitchState.OPEN
    detection_count: int = 0
    field_strength: float = 0.0  # Normalized 0-1


class HallEffect(Sensor):
    """Hall effect sensor for magnetic field detection.

    Operates in two modes:
    - Digital: Binary detection with threshold (like a switch)
    - Analog: Continuous field strength measurement

    Commonly used for:
    - Position sensing (magnet detection)
    - Speed measurement (rotating magnets)
    - Proximity detection (magnetic targets)

    Example:
        >>> from robo_infra.core.pin import SimulatedDigitalPin, PinMode
        >>> pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        >>> hall = HallEffect(pin=pin, mode=HallEffectMode.DIGITAL)
        >>> hall.enable()
        >>> if hall.is_detected():
        ...     print("Magnet detected!")
    """

    def __init__(
        self,
        *,
        pin: DigitalPin | None = None,
        analog_pin: AnalogPin | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        name: str = "HallEffect",
        mode: HallEffectMode = HallEffectMode.DIGITAL,
        config: HallEffectConfig | None = None,
    ) -> None:
        """Initialize Hall effect sensor.

        Args:
            pin: Digital input pin (for digital mode)
            analog_pin: Analog input pin (for analog mode)
            driver: Alternative driver
            channel: Driver channel number
            name: Sensor name
            mode: Operating mode (digital or analog)
            config: Full configuration object
        """
        if config is not None:
            self._config = config
            name = config.name
            mode = config.mode
        else:
            self._config = HallEffectConfig(name=name, mode=mode)

        self._pin = pin
        self._analog_pin = analog_pin
        self._status = HallEffectStatus()
        self._last_detected: bool = False

        # For hysteresis tracking in analog mode
        self._above_threshold: bool = False

        super().__init__(
            name=name,
            driver=driver,
            channel=channel,
            unit=Unit.COUNT if mode == HallEffectMode.DIGITAL else Unit.RATIO,
            limits=Limits(min=0, max=1),
        )

    def enable(self) -> None:
        """Enable sensor and set up pins."""
        super().enable()
        if self._pin is not None and not self._pin.initialized:
            self._pin.setup()
        if self._analog_pin is not None and not self._analog_pin.initialized:
            self._analog_pin.setup()

    def _read_raw(self) -> int:
        """Read raw sensor value."""
        if self._config.mode == HallEffectMode.DIGITAL:
            return self._read_digital()
        else:
            return self._read_analog()

    def _read_digital(self) -> int:
        """Read digital Hall sensor."""
        state = False

        if self._pin is not None:
            state = self._pin.read()
        elif self._driver is not None:
            state = bool(self._driver.get_channel(self._channel))

        # Apply active_high logic
        if not self._config.active_high:
            state = not state

        is_detected = state
        self._update_detection(is_detected, 1.0 if is_detected else 0.0)

        return 1 if is_detected else 0

    def _read_analog(self) -> int:
        """Read analog Hall sensor with hysteresis."""
        value = 0.0

        if self._analog_pin is not None:
            value = self._analog_pin.read_normalized()
        elif self._driver is not None:
            value = float(self._driver.get_channel(self._channel))

        self._status.analog_value = value
        self._status.field_strength = value

        # Apply hysteresis for threshold detection
        threshold = self._config.threshold
        hysteresis = self._config.hysteresis

        if self._above_threshold:
            # Currently above - need to drop below (threshold - hysteresis) to turn off
            if value < (threshold - hysteresis):
                self._above_threshold = False
        elif value > (threshold + hysteresis):
            # Currently below - need to rise above (threshold + hysteresis) to turn on
            self._above_threshold = True

        is_detected = self._above_threshold
        if not self._config.active_high:
            is_detected = not is_detected

        self._update_detection(is_detected, value)

        # Return normalized value scaled to int
        return int(value * 1000)  # Scale for precision

    def _update_detection(self, is_detected: bool, field_value: float) -> None:
        """Update detection state and tracking."""
        # Track detection events
        if is_detected and not self._last_detected:
            self._status.detection_count += 1

        self._last_detected = is_detected
        self._status.detected = is_detected
        self._status.field_strength = field_value
        self._status.state = SwitchState.CLOSED if is_detected else SwitchState.OPEN

    def is_detected(self) -> bool:
        """Check if magnetic field is detected.

        For digital mode: Returns True if field exceeds threshold.
        For analog mode: Returns True if field exceeds threshold with hysteresis.

        Returns:
            True if magnet/field detected.
        """
        _ = self.read()
        return self._status.detected

    def read_field_strength(self) -> float:
        """Read normalized field strength (analog mode).

        Returns:
            Field strength from 0.0 to 1.0.
        """
        _ = self.read()
        return self._status.field_strength

    def get_state(self) -> SwitchState:
        """Get detection state.

        Returns:
            SwitchState.CLOSED if detected, SwitchState.OPEN otherwise.
        """
        return self._status.state

    async def wait_for_detection(
        self,
        *,
        timeout: float = 30.0,  # Default 30s timeout for safety
        poll_interval: float = 0.01,
        max_iterations: int = 100_000,  # Safety limit
    ) -> bool:
        """Wait for magnetic field to be detected.

        Args:
            timeout: Maximum wait time in seconds (default 30s)
            poll_interval: Polling interval in seconds
            max_iterations: Safety limit to prevent infinite loops

        Returns:
            True if detected, False if timeout.

        Note:
            Always has a timeout for safety. Set timeout to a high value
            if you need longer waits, but never infinite.
        """
        start = time.time()
        iterations = 0

        while iterations < max_iterations:
            if self.is_detected():
                return True

            if (time.time() - start) >= timeout:
                return False

            await asyncio.sleep(poll_interval)
            iterations += 1

        logger.warning(
            "wait_for_detection reached max_iterations limit (%d) - stopping",
            max_iterations,
        )
        return False

    def reset_detection_count(self) -> None:
        """Reset the detection counter."""
        self._status.detection_count = 0

    def status(self) -> HallEffectStatus:  # type: ignore[override]
        """Get Hall effect sensor status."""
        return self._status
