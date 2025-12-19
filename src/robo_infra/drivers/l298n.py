"""L298N dual H-bridge motor driver.

This module provides a driver for the L298N dual H-bridge motor driver,
commonly used for controlling DC motors, stepper motors, and other high-current
loads.

The L298N uses GPIO pins for direction control and PWM for speed control:
- 2 motor channels (A and B)
- Each motor has: 2 direction pins (IN1/IN2 or IN3/IN4) + 1 enable/PWM pin (ENA/ENB)
- Direction control: IN1/IN2 HIGH/LOW for forward, LOW/HIGH for reverse
- Speed control: PWM duty cycle on enable pin
- Brake: Both direction pins HIGH
- Coast (free-run): Both direction pins LOW

Example:
    >>> from robo_infra.drivers.l298n import L298N
    >>>
    >>> # Create driver with simulation (no real hardware)
    >>> driver = L298N()
    >>> driver.connect()
    >>>
    >>> # Motor A forward at 75% speed
    >>> driver.set_motor(0, speed=0.75, direction=MotorDirection.FORWARD)
    >>>
    >>> # Motor B reverse at 50% speed
    >>> driver.set_motor(1, speed=0.5, direction=MotorDirection.REVERSE)
    >>>
    >>> # Stop motor A with braking
    >>> driver.brake(0)
    >>>
    >>> # Stop motor B with coasting (free-run)
    >>> driver.coast(1)
    >>>
    >>> driver.disconnect()

Hardware Reference:
    - Operating voltage: 5V-35V (motor power)
    - Logic voltage: 5V
    - Max current per channel: 2A continuous, 3A peak
    - Built-in 5V regulator (can be disabled)
    - Thermal shutdown protection
    - 6 control pins: IN1, IN2, IN3, IN4, ENA, ENB
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import TYPE_CHECKING

from robo_infra.core.driver import (
    Driver,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import (
    DisabledError,
    HardwareNotFoundError,
)


if TYPE_CHECKING:
    from robo_infra.core.pin import DigitalPin, PWMPin


logger = logging.getLogger(__name__)


# =============================================================================
# Enums and Constants
# =============================================================================


class MotorDirection(Enum):
    """Motor direction states."""

    FORWARD = "forward"
    REVERSE = "reverse"
    BRAKE = "brake"
    COAST = "coast"


class MotorChannel(IntEnum):
    """L298N motor channels."""

    A = 0  # Motor A (IN1, IN2, ENA)
    B = 1  # Motor B (IN3, IN4, ENB)


class BrakeMode(Enum):
    """Motor stopping modes."""

    BRAKE = "brake"  # Active braking (both direction pins HIGH)
    COAST = "coast"  # Free-run (both direction pins LOW)


# L298N pin mapping
# =============================================================================
# Configuration
# =============================================================================


@dataclass
class MotorConfig:
    """Configuration for a single motor channel.

    Attributes:
        inverted: If True, swap forward/reverse direction.
        max_speed: Maximum speed limit (0.0-1.0).
        min_speed: Minimum speed to prevent stalling (0.0-1.0).
        acceleration: Rate of speed change per second (for soft start/stop).
        default_brake_mode: Default stopping mode (brake or coast).
    """

    inverted: bool = False
    max_speed: float = 1.0
    min_speed: float = 0.0
    acceleration: float | None = None  # None = instant
    default_brake_mode: BrakeMode = BrakeMode.COAST


@dataclass
class L298NConfig:
    """Configuration for the L298N driver.

    Attributes:
        pwm_frequency: PWM frequency for speed control (Hz).
        motor_a: Configuration for motor A.
        motor_b: Configuration for motor B.
        enable_on_connect: If True, enable motors on connect.
    """

    pwm_frequency: int = 1000
    motor_a: MotorConfig = field(default_factory=MotorConfig)
    motor_b: MotorConfig = field(default_factory=MotorConfig)
    enable_on_connect: bool = True


# =============================================================================
# Motor State
# =============================================================================


@dataclass
class MotorState:
    """Current state of a motor channel.

    Attributes:
        speed: Current speed (0.0-1.0).
        direction: Current direction.
        enabled: Whether the motor is enabled.
        in1_state: State of IN1/IN3 pin (True=HIGH).
        in2_state: State of IN2/IN4 pin (True=HIGH).
    """

    speed: float = 0.0
    direction: MotorDirection = MotorDirection.COAST
    enabled: bool = False
    in1_state: bool = False
    in2_state: bool = False


# =============================================================================
# L298N Driver
# =============================================================================


@register_driver("l298n")
class L298N(Driver):
    """L298N dual H-bridge motor driver.

    Controls 2 DC motors with direction and speed control.
    Uses GPIO pins for direction and PWM for speed.

    Attributes:
        in1: Direction pin 1 for motor A (DigitalPin or None for simulation).
        in2: Direction pin 2 for motor A (DigitalPin or None for simulation).
        in3: Direction pin 1 for motor B (DigitalPin or None for simulation).
        in4: Direction pin 2 for motor B (DigitalPin or None for simulation).
        ena: Enable/PWM pin for motor A (PWMPin or None for simulation).
        enb: Enable/PWM pin for motor B (PWMPin or None for simulation).
    """

    # Default PWM frequency for motor control
    DEFAULT_PWM_FREQUENCY = 1000

    # Number of motor channels
    NUM_CHANNELS = 2

    def __init__(
        self,
        *,
        in1: DigitalPin | None = None,
        in2: DigitalPin | None = None,
        in3: DigitalPin | None = None,
        in4: DigitalPin | None = None,
        ena: PWMPin | None = None,
        enb: PWMPin | None = None,
        config: L298NConfig | None = None,
    ) -> None:
        """Initialize the L298N driver.

        If no pins are provided, the driver operates in simulation mode.

        Args:
            in1: Direction pin 1 for motor A.
            in2: Direction pin 2 for motor A.
            in3: Direction pin 1 for motor B.
            in4: Direction pin 2 for motor B.
            ena: Enable/PWM pin for motor A.
            enb: Enable/PWM pin for motor B.
            config: Driver configuration.
        """
        super().__init__(name="L298N", channels=self.NUM_CHANNELS)

        # Store pin references
        self._in1 = in1
        self._in2 = in2
        self._in3 = in3
        self._in4 = in4
        self._ena = ena
        self._enb = enb

        # Configuration
        self._l298n_config = config or L298NConfig()
        self._pwm_frequency: int = self._l298n_config.pwm_frequency

        # Determine simulation mode
        self._simulation_mode = all(pin is None for pin in [in1, in2, in3, in4, ena, enb])

        # Validate pin configuration
        if not self._simulation_mode:
            self._validate_pins()

        # Motor states
        self._motor_states: dict[int, MotorState] = {
            MotorChannel.A: MotorState(),
            MotorChannel.B: MotorState(),
        }

        # Motor configurations
        self._motor_configs: dict[int, MotorConfig] = {
            MotorChannel.A: self._l298n_config.motor_a,
            MotorChannel.B: self._l298n_config.motor_b,
        }

        logger.debug(
            "L298N initialized (simulation=%s, pwm_freq=%dHz)",
            self._simulation_mode,
            self._pwm_frequency,
        )

    def _validate_pins(self) -> None:
        """Validate pin configuration.

        Raises:
            ValueError: If pin configuration is invalid.
        """
        # Check motor A pins
        motor_a_pins = [self._in1, self._in2, self._ena]
        motor_a_present = [p is not None for p in motor_a_pins]

        if any(motor_a_present) and not all(motor_a_present):
            raise ValueError("Motor A requires all pins (in1, in2, ena) or none")

        # Check motor B pins
        motor_b_pins = [self._in3, self._in4, self._enb]
        motor_b_present = [p is not None for p in motor_b_pins]

        if any(motor_b_present) and not all(motor_b_present):
            raise ValueError("Motor B requires all pins (in3, in4, enb) or none")

        # At least one motor must be configured
        if not any(motor_a_present) and not any(motor_b_present):
            raise ValueError("At least one motor must be configured")

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def simulation_mode(self) -> bool:
        """Check if driver is in simulation mode."""
        return self._simulation_mode

    @property
    def motor_a_state(self) -> MotorState:
        """Get motor A current state."""
        return self._motor_states[MotorChannel.A]

    @property
    def motor_b_state(self) -> MotorState:
        """Get motor B current state."""
        return self._motor_states[MotorChannel.B]

    def get_motor_state(self, channel: int | MotorChannel) -> MotorState:
        """Get motor state for a channel.

        Args:
            channel: Motor channel (0=A, 1=B).

        Returns:
            Current motor state.

        Raises:
            ValueError: If channel is invalid.
        """
        channel = self._validate_motor_channel(channel)
        return self._motor_states[channel]

    def get_motor_config(self, channel: int | MotorChannel) -> MotorConfig:
        """Get motor configuration for a channel.

        Args:
            channel: Motor channel (0=A, 1=B).

        Returns:
            Motor configuration.

        Raises:
            ValueError: If channel is invalid.
        """
        channel = self._validate_motor_channel(channel)
        return self._motor_configs[channel]

    def _validate_motor_channel(self, channel: int | MotorChannel) -> int:
        """Validate and normalize channel.

        Args:
            channel: Motor channel.

        Returns:
            Normalized channel number.

        Raises:
            ValueError: If channel is invalid.
        """
        if isinstance(channel, MotorChannel):
            channel = int(channel)

        if channel not in (0, 1):
            raise ValueError(f"Invalid motor channel: {channel}. Must be 0 (A) or 1 (B)")

        return channel

    # -------------------------------------------------------------------------
    # Lifecycle Methods
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the L298N driver.

        Initializes GPIO pins and sets motors to coast mode.
        """
        if self._state == DriverState.CONNECTED:
            logger.warning("L298N already connected")
            return

        self._state = DriverState.CONNECTING
        logger.debug("Connecting L298N...")

        if not self._simulation_mode:
            try:
                # Setup motor A pins
                if self._in1 is not None:
                    self._in1.setup()
                    self._in2.setup()  # type: ignore[union-attr]
                    self._ena.setup()  # type: ignore[union-attr]
                    self._ena.set_frequency(self._pwm_frequency)  # type: ignore[union-attr]

                # Setup motor B pins
                if self._in3 is not None:
                    self._in3.setup()
                    self._in4.setup()  # type: ignore[union-attr]
                    self._enb.setup()  # type: ignore[union-attr]
                    self._enb.set_frequency(self._pwm_frequency)  # type: ignore[union-attr]

            except Exception as e:
                self._state = DriverState.ERROR
                raise HardwareNotFoundError(f"Failed to setup L298N pins: {e}") from e

        # Initialize motors to safe state (coast)
        for channel in (MotorChannel.A, MotorChannel.B):
            self._set_direction_pins(channel, in1=False, in2=False)
            self._set_speed_pin(channel, 0.0)
            self._motor_states[channel] = MotorState(
                speed=0.0,
                direction=MotorDirection.COAST,
                enabled=self._l298n_config.enable_on_connect,
                in1_state=False,
                in2_state=False,
            )

        self._state = DriverState.CONNECTED
        logger.info("L298N connected (simulation=%s)", self._simulation_mode)

    def disconnect(self) -> None:
        """Disconnect from the L298N driver.

        Stops all motors and cleans up GPIO resources.
        """
        if self._state == DriverState.DISCONNECTED:
            logger.warning("L298N already disconnected")
            return

        logger.debug("Disconnecting L298N...")

        # Stop all motors
        for channel in (MotorChannel.A, MotorChannel.B):
            try:
                self.coast(channel)
            except Exception as e:
                logger.warning("Error stopping motor %d: %s", channel, e)

        if not self._simulation_mode:
            try:
                # Cleanup motor A pins
                if self._in1 is not None:
                    self._ena.stop()  # type: ignore[union-attr]
                    self._in1.cleanup()
                    self._in2.cleanup()  # type: ignore[union-attr]
                    self._ena.cleanup()  # type: ignore[union-attr]

                # Cleanup motor B pins
                if self._in3 is not None:
                    self._enb.stop()  # type: ignore[union-attr]
                    self._in3.cleanup()
                    self._in4.cleanup()  # type: ignore[union-attr]
                    self._enb.cleanup()  # type: ignore[union-attr]

            except Exception as e:
                logger.warning("Error cleaning up L298N pins: %s", e)

        self._state = DriverState.DISCONNECTED
        logger.info("L298N disconnected")

    # -------------------------------------------------------------------------
    # Motor Control Methods
    # -------------------------------------------------------------------------

    def set_motor(
        self,
        channel: int | MotorChannel,
        speed: float,
        direction: MotorDirection = MotorDirection.FORWARD,
    ) -> None:
        """Set motor speed and direction.

        Args:
            channel: Motor channel (0=A, 1=B).
            speed: Speed from 0.0 to 1.0.
            direction: Motor direction (FORWARD, REVERSE, BRAKE, COAST).

        Raises:
            ValueError: If channel or speed is invalid.
            DisabledError: If driver is disabled.
        """
        self._require_connected()
        channel = self._validate_motor_channel(channel)

        # Get motor config
        config = self._motor_configs[channel]
        state = self._motor_states[channel]

        if not state.enabled:
            raise DisabledError(f"Motor {channel} is disabled")

        # Clamp and apply speed limits
        speed = max(0.0, min(1.0, speed))
        if speed > 0:
            speed = max(config.min_speed, min(config.max_speed, speed))

        # Handle direction with inversion
        actual_direction = direction
        if config.inverted and direction in (MotorDirection.FORWARD, MotorDirection.REVERSE):
            actual_direction = (
                MotorDirection.REVERSE
                if direction == MotorDirection.FORWARD
                else MotorDirection.FORWARD
            )

        # Set direction pins based on direction
        in1_state: bool
        in2_state: bool

        if actual_direction == MotorDirection.FORWARD:
            in1_state, in2_state = True, False
        elif actual_direction == MotorDirection.REVERSE:
            in1_state, in2_state = False, True
        elif actual_direction == MotorDirection.BRAKE:
            in1_state, in2_state = True, True
            speed = 1.0  # Full brake
        else:  # COAST
            in1_state, in2_state = False, False
            speed = 0.0  # No power

        # Apply to hardware
        self._set_direction_pins(channel, in1_state, in2_state)
        self._set_speed_pin(channel, speed)

        # Update state
        state.speed = speed
        state.direction = direction  # Store original direction (not inverted)
        state.in1_state = in1_state
        state.in2_state = in2_state

        logger.debug(
            "Motor %d: speed=%.2f, direction=%s (in1=%s, in2=%s)",
            channel,
            speed,
            direction.value,
            in1_state,
            in2_state,
        )

    def forward(self, channel: int | MotorChannel, speed: float = 1.0) -> None:
        """Run motor forward at specified speed.

        Args:
            channel: Motor channel (0=A, 1=B).
            speed: Speed from 0.0 to 1.0.
        """
        self.set_motor(channel, speed, MotorDirection.FORWARD)

    def reverse(self, channel: int | MotorChannel, speed: float = 1.0) -> None:
        """Run motor in reverse at specified speed.

        Args:
            channel: Motor channel (0=A, 1=B).
            speed: Speed from 0.0 to 1.0.
        """
        self.set_motor(channel, speed, MotorDirection.REVERSE)

    def brake(self, channel: int | MotorChannel) -> None:
        """Stop motor with active braking.

        Uses short brake mode (both direction pins HIGH) to stop quickly.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        self.set_motor(channel, 1.0, MotorDirection.BRAKE)

    def coast(self, channel: int | MotorChannel) -> None:
        """Stop motor with free-running (coast).

        Motor will slowly decelerate due to friction only.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        self.set_motor(channel, 0.0, MotorDirection.COAST)

    def stop(
        self,
        channel: int | MotorChannel,
        mode: BrakeMode | None = None,
    ) -> None:
        """Stop motor with specified mode.

        Args:
            channel: Motor channel (0=A, 1=B).
            mode: Stopping mode. If None, uses motor's default_brake_mode.
        """
        channel = self._validate_motor_channel(channel)
        config = self._motor_configs[channel]

        if mode is None:
            mode = config.default_brake_mode

        if mode == BrakeMode.BRAKE:
            self.brake(channel)
        else:
            self.coast(channel)

    def stop_all(self, mode: BrakeMode | None = None) -> None:
        """Stop all motors.

        Args:
            mode: Stopping mode. If None, uses each motor's default_brake_mode.
        """
        for channel in (MotorChannel.A, MotorChannel.B):
            try:
                self.stop(channel, mode)
            except Exception as e:
                logger.warning("Error stopping motor %d: %s", channel, e)

    def emergency_stop(self) -> None:
        """Emergency stop all motors with active braking.

        This method should be used for safety-critical stops.
        It attempts to brake all motors immediately.
        """
        logger.warning("L298N EMERGENCY STOP")

        # Disable first for safety
        for channel in (MotorChannel.A, MotorChannel.B):
            try:
                self._set_direction_pins(channel, True, True)  # Brake
                self._set_speed_pin(channel, 1.0)
                self._motor_states[channel].speed = 0.0
                self._motor_states[channel].direction = MotorDirection.BRAKE
                self._motor_states[channel].in1_state = True
                self._motor_states[channel].in2_state = True
            except Exception as e:
                logger.error("Error during emergency stop on motor %d: %s", channel, e)

        # Disable driver
        self._enabled = False

    # -------------------------------------------------------------------------
    # Motor Enable/Disable
    # -------------------------------------------------------------------------

    def enable_motor(self, channel: int | MotorChannel) -> None:
        """Enable a motor channel.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        channel = self._validate_motor_channel(channel)
        self._motor_states[channel].enabled = True
        logger.debug("Motor %d enabled", channel)

    def disable_motor(self, channel: int | MotorChannel) -> None:
        """Disable a motor channel.

        Stops the motor before disabling.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        channel = self._validate_motor_channel(channel)

        # Stop the motor first (best-effort, log failures)
        if self._state == DriverState.CONNECTED:
            try:
                self.coast(channel)
            except Exception as e:
                logger.warning("Failed to coast motor %d during disable: %s", channel, e)

        self._motor_states[channel].enabled = False
        logger.debug("Motor %d disabled", channel)

    def enable_all(self) -> None:
        """Enable all motor channels."""
        for channel in (MotorChannel.A, MotorChannel.B):
            self.enable_motor(channel)

    def disable_all(self) -> None:
        """Disable all motor channels."""
        for channel in (MotorChannel.A, MotorChannel.B):
            self.disable_motor(channel)

    # -------------------------------------------------------------------------
    # Driver Channel Interface (for compatibility)
    # -------------------------------------------------------------------------

    def set_channel(self, channel: int, value: float) -> None:
        """Set channel value (normalized -1.0 to 1.0).

        This implements the Driver interface for compatibility.
        Positive values = forward, negative = reverse.

        Args:
            channel: Motor channel (0=A, 1=B).
            value: Speed from -1.0 (full reverse) to 1.0 (full forward).
        """
        channel = self._validate_motor_channel(channel)

        if value >= 0:
            self.set_motor(channel, value, MotorDirection.FORWARD)
        else:
            self.set_motor(channel, abs(value), MotorDirection.REVERSE)

    def get_channel(self, channel: int) -> float:
        """Get channel value (normalized -1.0 to 1.0).

        Args:
            channel: Motor channel (0=A, 1=B).

        Returns:
            Current speed with sign indicating direction.
        """
        channel = self._validate_motor_channel(channel)
        state = self._motor_states[channel]

        if state.direction == MotorDirection.REVERSE:
            return -state.speed
        return state.speed

    # -------------------------------------------------------------------------
    # Low-Level Pin Control
    # -------------------------------------------------------------------------

    def _set_direction_pins(
        self,
        channel: int,
        in1: bool,
        in2: bool,
    ) -> None:
        """Set direction pins for a motor channel.

        Args:
            channel: Motor channel (0 or 1).
            in1: State for IN1/IN3 pin.
            in2: State for IN2/IN4 pin.
        """
        if self._simulation_mode:
            return

        if channel == MotorChannel.A:
            if self._in1 is not None:
                self._in1.write(in1)
                self._in2.write(in2)  # type: ignore[union-attr]
        elif self._in3 is not None:
            self._in3.write(in1)
            self._in4.write(in2)  # type: ignore[union-attr]

    def _set_speed_pin(self, channel: int, speed: float) -> None:
        """Set speed (PWM duty cycle) for a motor channel.

        Args:
            channel: Motor channel (0 or 1).
            speed: Speed from 0.0 to 1.0.
        """
        if self._simulation_mode:
            return

        if channel == MotorChannel.A:
            if self._ena is not None:
                self._ena.set_duty_cycle(speed)
                if speed > 0:
                    self._ena.start()
                else:
                    self._ena.stop()
        elif self._enb is not None:
            self._enb.set_duty_cycle(speed)
            if speed > 0:
                self._enb.start()
            else:
                self._enb.stop()

    # -------------------------------------------------------------------------
    # Abstract Method Implementations (from Driver base class)
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write a value to a motor channel.

        Implements the abstract method from Driver.
        Maps to set_channel which handles direction based on sign.

        Args:
            channel: Channel number (0 or 1).
            value: Value from -1.0 to 1.0 (positive=forward, negative=reverse).
        """
        # set_channel already handles the mapping
        if value >= 0:
            self.set_motor(channel, value, MotorDirection.FORWARD)
        else:
            self.set_motor(channel, abs(value), MotorDirection.REVERSE)

    def _read_channel(self, channel: int) -> float:
        """Read the current value of a motor channel.

        Implements the abstract method from Driver.

        Args:
            channel: Channel number (0 or 1).

        Returns:
            Current speed with sign indicating direction.
        """
        state = self._motor_states[channel]
        if state.direction == MotorDirection.REVERSE:
            return -state.speed
        return state.speed

    # -------------------------------------------------------------------------
    # Utility Methods
    # -------------------------------------------------------------------------

    def _require_connected(self) -> None:
        """Require driver to be connected.

        Raises:
            HardwareNotFoundError: If not connected.
        """
        if self._state != DriverState.CONNECTED:
            raise HardwareNotFoundError(f"L298N not connected (state={self._state.value})")

    # -------------------------------------------------------------------------
    # Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """Return string representation."""
        return (
            f"L298N("
            f"state={self._state.value}, "
            f"simulation={self._simulation_mode}, "
            f"motor_a={self._motor_states[MotorChannel.A]}, "
            f"motor_b={self._motor_states[MotorChannel.B]})"
        )
