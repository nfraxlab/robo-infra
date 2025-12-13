"""Unit tests for switch sensors (Phase 4.4)."""

from __future__ import annotations

import asyncio

import pytest

from robo_infra.core.pin import PinMode, SimulatedAnalogPin, SimulatedDigitalPin
from robo_infra.sensors.switches import (
    Button,
    ButtonConfig,
    ButtonStatus,
    HallEffect,
    HallEffectConfig,
    HallEffectMode,
    HallEffectStatus,
    LimitSwitch,
    LimitSwitchConfig,
    LimitSwitchStatus,
    SwitchState,
    TriggerEdge,
)


# =============================================================================
# Switch State and Enums Tests
# =============================================================================


class TestEnums:
    """Tests for switch enums."""

    def test_switch_state_values(self) -> None:
        """Test SwitchState enum values."""
        assert SwitchState.OPEN.value == "open"
        assert SwitchState.CLOSED.value == "closed"

    def test_trigger_edge_values(self) -> None:
        """Test TriggerEdge enum values."""
        assert TriggerEdge.RISING.value == "rising"
        assert TriggerEdge.FALLING.value == "falling"
        assert TriggerEdge.BOTH.value == "both"

    def test_hall_effect_mode_values(self) -> None:
        """Test HallEffectMode enum values."""
        assert HallEffectMode.DIGITAL.value == "digital"
        assert HallEffectMode.ANALOG.value == "analog"


# =============================================================================
# Limit Switch Config Tests
# =============================================================================


class TestLimitSwitchConfig:
    """Tests for LimitSwitchConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = LimitSwitchConfig()

        assert config.name == "LimitSwitch"
        assert config.normally_open is True
        assert config.debounce_ms == 10.0
        assert config.invert_logic is False

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = LimitSwitchConfig(
            name="EndStop",
            normally_open=False,
            debounce_ms=20.0,
            invert_logic=True,
        )

        assert config.name == "EndStop"
        assert config.normally_open is False
        assert config.debounce_ms == 20.0
        assert config.invert_logic is True


# =============================================================================
# Limit Switch Tests
# =============================================================================


class TestLimitSwitch:
    """Tests for LimitSwitch."""

    def test_initialization_default(self) -> None:
        """Test default limit switch initialization."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        limit = LimitSwitch(pin=pin)

        assert limit.name == "LimitSwitch"
        assert limit.normally_open is True

    def test_initialization_with_config(self) -> None:
        """Test limit switch with config."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        config = LimitSwitchConfig(
            name="XAxisLimit",
            normally_open=False,
            debounce_ms=15.0,
        )
        limit = LimitSwitch(pin=pin, config=config)

        assert limit.name == "XAxisLimit"
        assert limit.normally_open is False

    def test_normally_open_not_triggered(self) -> None:
        """Test NO switch not triggered when pin LOW."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)  # Pin LOW

        limit = LimitSwitch(pin=pin, normally_open=True, debounce_ms=0)
        limit.enable()

        assert not limit.is_triggered()
        assert limit.get_state() == SwitchState.OPEN

    def test_normally_open_triggered(self) -> None:
        """Test NO switch triggered when pin HIGH."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(True)  # Pin HIGH

        limit = LimitSwitch(pin=pin, normally_open=True, debounce_ms=0)
        limit.enable()

        assert limit.is_triggered()
        assert limit.get_state() == SwitchState.CLOSED

    def test_normally_closed_not_triggered(self) -> None:
        """Test NC switch not triggered when pin HIGH."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(True)  # Pin HIGH

        limit = LimitSwitch(pin=pin, normally_open=False, debounce_ms=0)
        limit.enable()

        assert not limit.is_triggered()
        assert limit.get_state() == SwitchState.OPEN

    def test_normally_closed_triggered(self) -> None:
        """Test NC switch triggered when pin LOW."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)  # Pin LOW

        limit = LimitSwitch(pin=pin, normally_open=False, debounce_ms=0)
        limit.enable()

        assert limit.is_triggered()
        assert limit.get_state() == SwitchState.CLOSED

    def test_trigger_count(self) -> None:
        """Test trigger counting."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()

        limit = LimitSwitch(pin=pin, normally_open=True, debounce_ms=0)
        limit.enable()

        # Initially not triggered
        pin.write(False)
        _ = limit.read()
        assert limit.status().trigger_count == 0

        # First trigger
        pin.write(True)
        _ = limit.read()
        assert limit.status().trigger_count == 1

        # Release and trigger again
        pin.write(False)
        _ = limit.read()
        pin.write(True)
        _ = limit.read()
        assert limit.status().trigger_count == 2

    def test_reset_trigger_count(self) -> None:
        """Test reset trigger count."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(True)

        limit = LimitSwitch(pin=pin, normally_open=True, debounce_ms=0)
        limit.enable()
        _ = limit.read()

        limit.reset_trigger_count()
        assert limit.status().trigger_count == 0

    def test_status(self) -> None:
        """Test status retrieval."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        limit = LimitSwitch(pin=pin)
        limit.enable()

        status = limit.status()

        assert isinstance(status, LimitSwitchStatus)
        assert hasattr(status, "triggered")
        assert hasattr(status, "state")
        assert hasattr(status, "trigger_count")

    @pytest.mark.asyncio
    async def test_wait_for_trigger(self) -> None:
        """Test async wait for trigger."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)

        limit = LimitSwitch(pin=pin, normally_open=True, debounce_ms=0)
        limit.enable()

        async def trigger_after_delay() -> None:
            await asyncio.sleep(0.05)
            pin.write(True)

        # Start trigger task
        task = asyncio.create_task(trigger_after_delay())

        # Wait for trigger
        result = await limit.wait_for_trigger(timeout=1.0)
        assert result is True
        await task

    @pytest.mark.asyncio
    async def test_wait_for_trigger_timeout(self) -> None:
        """Test async wait for trigger with timeout."""
        pin = SimulatedDigitalPin(5, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)

        limit = LimitSwitch(pin=pin, normally_open=True, debounce_ms=0)
        limit.enable()

        result = await limit.wait_for_trigger(timeout=0.05)
        assert result is False


# =============================================================================
# Button Config Tests
# =============================================================================


class TestButtonConfig:
    """Tests for ButtonConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = ButtonConfig()

        assert config.name == "Button"
        assert config.normally_open is True
        assert config.debounce_ms == 50.0
        assert config.long_press_ms == 500.0

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = ButtonConfig(
            name="StartButton",
            debounce_ms=30.0,
            long_press_ms=1000.0,
        )

        assert config.name == "StartButton"
        assert config.debounce_ms == 30.0
        assert config.long_press_ms == 1000.0


# =============================================================================
# Button Tests
# =============================================================================


class TestButton:
    """Tests for Button."""

    def test_initialization_default(self) -> None:
        """Test default button initialization."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        button = Button(pin=pin)

        assert button.name == "Button"
        assert button.normally_open is True

    def test_initialization_with_config(self) -> None:
        """Test button with config."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        config = ButtonConfig(name="PowerButton", debounce_ms=25.0)
        button = Button(pin=pin, config=config)

        assert button.name == "PowerButton"

    def test_not_pressed(self) -> None:
        """Test button not pressed."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)

        button = Button(pin=pin, debounce_ms=0)
        button.enable()

        assert not button.is_pressed()

    def test_pressed(self) -> None:
        """Test button pressed."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(True)

        button = Button(pin=pin, debounce_ms=0)
        button.enable()

        assert button.is_pressed()

    def test_press_count(self) -> None:
        """Test press counting."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()

        button = Button(pin=pin, debounce_ms=0)
        button.enable()

        # Press and release twice
        for _ in range(2):
            pin.write(False)
            _ = button.read()
            pin.write(True)
            _ = button.read()

        assert button.status().press_count == 2

    def test_on_press_callback(self) -> None:
        """Test press callback."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)

        button = Button(pin=pin, debounce_ms=0)
        button.enable()

        callback_called = []

        def on_press() -> None:
            callback_called.append(True)

        button.on_press(on_press)

        # Trigger press
        _ = button.read()  # Initial read
        pin.write(True)
        _ = button.read()

        assert len(callback_called) == 1

    def test_on_release_callback(self) -> None:
        """Test release callback."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(True)

        button = Button(pin=pin, debounce_ms=0)
        button.enable()

        callback_called = []

        def on_release() -> None:
            callback_called.append(True)

        button.on_release(on_release)

        # Initial state pressed
        _ = button.read()

        # Release
        pin.write(False)
        _ = button.read()

        assert len(callback_called) == 1

    def test_clear_callbacks(self) -> None:
        """Test clearing callbacks."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        button = Button(pin=pin)

        callback_called = []
        button.on_press(lambda: callback_called.append(True))
        button.clear_callbacks()

        assert len(button._on_press_callbacks) == 0

    def test_reset_press_count(self) -> None:
        """Test reset press count."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(True)

        button = Button(pin=pin, debounce_ms=0)
        button.enable()
        _ = button.read()

        button.reset_press_count()
        assert button.status().press_count == 0

    def test_status(self) -> None:
        """Test status retrieval."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        button = Button(pin=pin)
        button.enable()

        status = button.status()

        assert isinstance(status, ButtonStatus)
        assert hasattr(status, "pressed")
        assert hasattr(status, "press_count")

    @pytest.mark.asyncio
    async def test_wait_for_press(self) -> None:
        """Test async wait for press."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)

        button = Button(pin=pin, debounce_ms=0)
        button.enable()

        async def press_after_delay() -> None:
            await asyncio.sleep(0.05)
            pin.write(True)

        task = asyncio.create_task(press_after_delay())

        result = await button.wait_for_press(timeout=1.0)
        assert result is True
        await task

    @pytest.mark.asyncio
    async def test_wait_for_click(self) -> None:
        """Test async wait for complete click."""
        pin = SimulatedDigitalPin(2, mode=PinMode.INPUT_PULLUP)
        pin.setup()
        pin.write(False)

        button = Button(pin=pin, debounce_ms=0)
        button.enable()

        async def click_sequence() -> None:
            await asyncio.sleep(0.02)
            pin.write(True)  # Press
            await asyncio.sleep(0.02)
            pin.write(False)  # Release

        task = asyncio.create_task(click_sequence())

        result = await button.wait_for_click(timeout=1.0)
        assert result is True
        await task


# =============================================================================
# Hall Effect Config Tests
# =============================================================================


class TestHallEffectConfig:
    """Tests for HallEffectConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = HallEffectConfig()

        assert config.name == "HallEffect"
        assert config.mode == HallEffectMode.DIGITAL
        assert config.normally_open is True
        assert config.threshold == 0.5
        assert config.hysteresis == 0.1
        assert config.active_high is True

    def test_analog_config(self) -> None:
        """Test analog mode configuration."""
        config = HallEffectConfig(
            mode=HallEffectMode.ANALOG,
            threshold=0.7,
            hysteresis=0.15,
        )

        assert config.mode == HallEffectMode.ANALOG
        assert config.threshold == 0.7
        assert config.hysteresis == 0.15


# =============================================================================
# Hall Effect Tests
# =============================================================================


class TestHallEffect:
    """Tests for HallEffect sensor."""

    def test_initialization_digital(self) -> None:
        """Test digital Hall effect initialization."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        hall = HallEffect(pin=pin, mode=HallEffectMode.DIGITAL)

        assert hall.name == "HallEffect"

    def test_initialization_analog(self) -> None:
        """Test analog Hall effect initialization."""
        pin = SimulatedAnalogPin(0)
        hall = HallEffect(analog_pin=pin, mode=HallEffectMode.ANALOG)

        assert hall.name == "HallEffect"

    def test_digital_not_detected(self) -> None:
        """Test digital mode not detected."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        pin.setup()
        pin.write(False)

        hall = HallEffect(pin=pin, mode=HallEffectMode.DIGITAL)
        hall.enable()

        assert not hall.is_detected()
        assert hall.get_state() == SwitchState.OPEN

    def test_digital_detected(self) -> None:
        """Test digital mode detected."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        pin.setup()
        pin.write(True)

        config = HallEffectConfig(mode=HallEffectMode.DIGITAL, active_high=True)
        hall = HallEffect(pin=pin, config=config)
        hall.enable()

        assert hall.is_detected()
        assert hall.get_state() == SwitchState.CLOSED

    def test_digital_active_low(self) -> None:
        """Test digital mode with active low."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        pin.setup()
        pin.write(False)

        config = HallEffectConfig(mode=HallEffectMode.DIGITAL, active_high=False)
        hall = HallEffect(pin=pin, config=config)
        hall.enable()

        assert hall.is_detected()

    def test_analog_below_threshold(self) -> None:
        """Test analog mode below threshold."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.3)  # Below midpoint

        config = HallEffectConfig(
            mode=HallEffectMode.ANALOG,
            threshold=0.5,
            hysteresis=0.1,
        )
        hall = HallEffect(analog_pin=pin, config=config)
        hall.enable()

        # Read to establish state
        _ = hall.read()

        assert not hall.is_detected()

    def test_analog_above_threshold(self) -> None:
        """Test analog mode above threshold with hysteresis."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        config = HallEffectConfig(
            mode=HallEffectMode.ANALOG,
            threshold=0.5,
            hysteresis=0.1,
        )
        hall = HallEffect(analog_pin=pin, config=config)
        hall.enable()

        # Start below threshold
        pin.set_simulated_normalized(0.3)  # 30%
        _ = hall.read()
        assert not hall.is_detected()

        # Go above threshold + hysteresis
        pin.set_simulated_normalized(0.65)  # 65% > 50% + 10%
        _ = hall.read()
        assert hall.is_detected()

    def test_analog_hysteresis(self) -> None:
        """Test hysteresis prevents oscillation."""
        pin = SimulatedAnalogPin(0)
        pin.setup()

        config = HallEffectConfig(
            mode=HallEffectMode.ANALOG,
            threshold=0.5,
            hysteresis=0.1,
        )
        hall = HallEffect(analog_pin=pin, config=config)
        hall.enable()

        # Start above threshold + hysteresis
        pin.set_simulated_normalized(0.65)  # 65%
        _ = hall.read()
        assert hall.is_detected()

        # Drop to within hysteresis band (still detected)
        pin.set_simulated_normalized(0.45)  # 45% > 40%
        _ = hall.read()
        assert hall.is_detected()  # Still detected due to hysteresis

        # Drop below threshold - hysteresis
        pin.set_simulated_normalized(0.35)  # 35% < 40%
        _ = hall.read()
        assert not hall.is_detected()

    def test_read_field_strength(self) -> None:
        """Test reading field strength in analog mode."""
        pin = SimulatedAnalogPin(0)
        pin.setup()
        pin.set_simulated_normalized(0.75)  # 75%

        config = HallEffectConfig(mode=HallEffectMode.ANALOG)
        hall = HallEffect(analog_pin=pin, config=config)
        hall.enable()

        strength = hall.read_field_strength()
        assert strength == pytest.approx(0.75, abs=0.01)

    def test_detection_count(self) -> None:
        """Test detection counting."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        pin.setup()

        config = HallEffectConfig(mode=HallEffectMode.DIGITAL)
        hall = HallEffect(pin=pin, config=config)
        hall.enable()

        # Initial not detected
        pin.write(False)
        _ = hall.read()
        assert hall.status().detection_count == 0

        # First detection
        pin.write(True)
        _ = hall.read()
        assert hall.status().detection_count == 1

        # Undetect and detect again
        pin.write(False)
        _ = hall.read()
        pin.write(True)
        _ = hall.read()
        assert hall.status().detection_count == 2

    def test_reset_detection_count(self) -> None:
        """Test reset detection count."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        pin.setup()
        pin.write(True)

        config = HallEffectConfig(mode=HallEffectMode.DIGITAL)
        hall = HallEffect(pin=pin, config=config)
        hall.enable()
        _ = hall.read()

        hall.reset_detection_count()
        assert hall.status().detection_count == 0

    def test_status(self) -> None:
        """Test status retrieval."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        hall = HallEffect(pin=pin)
        hall.enable()

        status = hall.status()

        assert isinstance(status, HallEffectStatus)
        assert hasattr(status, "detected")
        assert hasattr(status, "analog_value")
        assert hasattr(status, "field_strength")

    @pytest.mark.asyncio
    async def test_wait_for_detection(self) -> None:
        """Test async wait for detection."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        pin.setup()
        pin.write(False)

        config = HallEffectConfig(mode=HallEffectMode.DIGITAL)
        hall = HallEffect(pin=pin, config=config)
        hall.enable()

        async def detect_after_delay() -> None:
            await asyncio.sleep(0.05)
            pin.write(True)

        task = asyncio.create_task(detect_after_delay())

        result = await hall.wait_for_detection(timeout=1.0)
        assert result is True
        await task

    @pytest.mark.asyncio
    async def test_wait_for_detection_timeout(self) -> None:
        """Test async wait for detection with timeout."""
        pin = SimulatedDigitalPin(3, mode=PinMode.INPUT)
        pin.setup()
        pin.write(False)

        config = HallEffectConfig(mode=HallEffectMode.DIGITAL)
        hall = HallEffect(pin=pin, config=config)
        hall.enable()

        result = await hall.wait_for_detection(timeout=0.05)
        assert result is False
