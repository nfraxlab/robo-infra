"""TB6612FNG dual motor driver.

This module provides a driver for the TB6612FNG dual motor driver,
a more efficient alternative to the L298N with lower voltage drop.

The TB6612FNG uses GPIO pins for direction control and PWM for speed control:
- 2 motor channels (A and B)
- Each motor has: 2 direction pins (AIN1/AIN2 or BIN1/BIN2) + 1 PWM pin (PWMA/PWMB)
- Global standby pin (STBY) to enable/disable all outputs
- Direction control: AIN1/AIN2 HIGH/LOW for forward, LOW/HIGH for reverse
- Speed control: PWM duty cycle on PWM pin
- Brake: Both direction pins HIGH
- Coast (free-run): Both direction pins LOW

Example:
    >>> from robo_infra.drivers.tb6612 import TB6612
    >>>
    >>> # Create driver with simulation (no real hardware)
    >>> driver = TB6612()
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
    >>> # Put driver in standby (low power mode)
    >>> driver.standby()
    >>>
    >>> driver.disconnect()

Hardware Reference:
    - Operating voltage: 2.5V-13.5V (motor power, VM)
    - Logic voltage: 2.7V-5.5V (VCC)
    - Max current per channel: 1.2A continuous, 3.2A peak
    - Low voltage drop: ~0.5V (vs ~2V for L298N)
    - Built-in thermal shutdown and overcurrent protection
    - 7 control pins: AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY

Comparison to L298N:
    - More efficient (MOSFET vs BJT)
    - Lower voltage drop (~0.5V vs ~2V)
    - Lower maximum current (1.2A vs 2A continuous)
    - Lower maximum voltage (13.5V vs 35V)
    - Standby pin for power saving
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


class TB6612Direction(Enum):
    """Motor direction states for TB6612."""

    FORWARD = "forward"
    REVERSE = "reverse"
    BRAKE = "brake"
    COAST = "coast"


class TB6612Channel(IntEnum):
    """TB6612 motor channels."""

    A = 0  # Motor A (AIN1, AIN2, PWMA)
    B = 1  # Motor B (BIN1, BIN2, PWMB)


class TB6612BrakeMode(Enum):
    """Motor stopping modes for TB6612."""

    BRAKE = "brake"  # Active braking (both direction pins HIGH)
    COAST = "coast"  # Free-run (both direction pins LOW)


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class TB6612MotorConfig:
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
    default_brake_mode: TB6612BrakeMode = TB6612BrakeMode.COAST


@dataclass
class TB6612Config:
    """Configuration for the TB6612 driver.

    Attributes:
        pwm_frequency: PWM frequency for speed control (Hz).
        motor_a: Configuration for motor A.
        motor_b: Configuration for motor B.
        enable_on_connect: If True, enable motors on connect.
        exit_standby_on_connect: If True, exit standby mode on connect.
    """

    pwm_frequency: int = 10000  # TB6612 supports higher PWM frequencies
    motor_a: TB6612MotorConfig = field(default_factory=TB6612MotorConfig)
    motor_b: TB6612MotorConfig = field(default_factory=TB6612MotorConfig)
    enable_on_connect: bool = True
    exit_standby_on_connect: bool = True


# =============================================================================
# Motor State
# =============================================================================


@dataclass
class TB6612MotorState:
    """Current state of a motor channel.

    Attributes:
        speed: Current speed (0.0-1.0).
        direction: Current direction.
        enabled: Whether the motor is enabled.
        in1_state: State of AIN1/BIN1 pin (True=HIGH).
        in2_state: State of AIN2/BIN2 pin (True=HIGH).
    """

    speed: float = 0.0
    direction: TB6612Direction = TB6612Direction.COAST
    enabled: bool = False
    in1_state: bool = False
    in2_state: bool = False


# =============================================================================
# TB6612 Driver
# =============================================================================


@register_driver("tb6612")
class TB6612(Driver):
    """TB6612FNG dual motor driver.

    Controls 2 DC motors with direction and speed control.
    Uses GPIO pins for direction and PWM for speed.
    Includes standby pin for power saving mode.

    Attributes:
        ain1: Direction pin 1 for motor A (DigitalPin or None for simulation).
        ain2: Direction pin 2 for motor A (DigitalPin or None for simulation).
        bin1: Direction pin 1 for motor B (DigitalPin or None for simulation).
        bin2: Direction pin 2 for motor B (DigitalPin or None for simulation).
        pwma: PWM pin for motor A (PWMPin or None for simulation).
        pwmb: PWM pin for motor B (PWMPin or None for simulation).
        stby: Standby pin (DigitalPin or None for simulation).
    """

    # Default PWM frequency for motor control (TB6612 supports up to 100kHz)
    DEFAULT_PWM_FREQUENCY = 10000

    # Number of motor channels
    NUM_CHANNELS = 2

    def __init__(
        self,
        *,
        ain1: DigitalPin | None = None,
        ain2: DigitalPin | None = None,
        bin1: DigitalPin | None = None,
        bin2: DigitalPin | None = None,
        pwma: PWMPin | None = None,
        pwmb: PWMPin | None = None,
        stby: DigitalPin | None = None,
        config: TB6612Config | None = None,
    ) -> None:
        """Initialize the TB6612 driver.

        If no pins are provided, the driver operates in simulation mode.

        Args:
            ain1: Direction pin 1 for motor A.
            ain2: Direction pin 2 for motor A.
            bin1: Direction pin 1 for motor B.
            bin2: Direction pin 2 for motor B.
            pwma: PWM pin for motor A.
            pwmb: PWM pin for motor B.
            stby: Standby control pin (HIGH=active, LOW=standby).
            config: Driver configuration.
        """
        super().__init__(name="TB6612", channels=self.NUM_CHANNELS)

        # Store pin references
        self._ain1 = ain1
        self._ain2 = ain2
        self._bin1 = bin1
        self._bin2 = bin2
        self._pwma = pwma
        self._pwmb = pwmb
        self._stby = stby

        # Configuration
        self._tb6612_config = config or TB6612Config()
        self._pwm_frequency: int = self._tb6612_config.pwm_frequency

        # Determine simulation mode
        self._simulation_mode = all(
            pin is None for pin in [ain1, ain2, bin1, bin2, pwma, pwmb, stby]
        )

        # Validate pin configuration
        if not self._simulation_mode:
            self._validate_pins()

        # Motor states
        self._motor_states: dict[int, TB6612MotorState] = {
            TB6612Channel.A: TB6612MotorState(),
            TB6612Channel.B: TB6612MotorState(),
        }

        # Motor configurations
        self._motor_configs: dict[int, TB6612MotorConfig] = {
            TB6612Channel.A: self._tb6612_config.motor_a,
            TB6612Channel.B: self._tb6612_config.motor_b,
        }

        # Standby state
        self._in_standby = True  # Start in standby

        logger.debug(
            "TB6612 initialized (simulation=%s, pwm_freq=%dHz)",
            self._simulation_mode,
            self._pwm_frequency,
        )

    def _validate_pins(self) -> None:
        """Validate pin configuration.

        Raises:
            ValueError: If pin configuration is invalid.
        """
        # Check motor A pins
        motor_a_pins = [self._ain1, self._ain2, self._pwma]
        motor_a_present = [p is not None for p in motor_a_pins]

        if any(motor_a_present) and not all(motor_a_present):
            raise ValueError("Motor A requires all pins (ain1, ain2, pwma) or none")

        # Check motor B pins
        motor_b_pins = [self._bin1, self._bin2, self._pwmb]
        motor_b_present = [p is not None for p in motor_b_pins]

        if any(motor_b_present) and not all(motor_b_present):
            raise ValueError("Motor B requires all pins (bin1, bin2, pwmb) or none")

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
    def in_standby(self) -> bool:
        """Check if driver is in standby mode."""
        return self._in_standby

    @property
    def motor_a_state(self) -> TB6612MotorState:
        """Get motor A current state."""
        return self._motor_states[TB6612Channel.A]

    @property
    def motor_b_state(self) -> TB6612MotorState:
        """Get motor B current state."""
        return self._motor_states[TB6612Channel.B]

    def get_motor_state(self, channel: int | TB6612Channel) -> TB6612MotorState:
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

    def get_motor_config(self, channel: int | TB6612Channel) -> TB6612MotorConfig:
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

    def _validate_motor_channel(self, channel: int | TB6612Channel) -> int:
        """Validate and normalize channel.

        Args:
            channel: Motor channel.

        Returns:
            Normalized channel number.

        Raises:
            ValueError: If channel is invalid.
        """
        if isinstance(channel, TB6612Channel):
            channel = int(channel)

        if channel not in (0, 1):
            raise ValueError(f"Invalid motor channel: {channel}. Must be 0 (A) or 1 (B)")

        return channel

    # -------------------------------------------------------------------------
    # Lifecycle Methods
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the TB6612 driver.

        Initializes GPIO pins and sets motors to coast mode.
        """
        if self._state == DriverState.CONNECTED:
            logger.warning("TB6612 already connected")
            return

        self._state = DriverState.CONNECTING
        logger.debug("Connecting TB6612...")

        if not self._simulation_mode:
            try:
                # Setup motor A pins
                if self._ain1 is not None:
                    self._ain1.setup()
                    self._ain2.setup()  # type: ignore[union-attr]
                    self._pwma.setup()  # type: ignore[union-attr]
                    self._pwma.set_frequency(self._pwm_frequency)  # type: ignore[union-attr]

                # Setup motor B pins
                if self._bin1 is not None:
                    self._bin1.setup()
                    self._bin2.setup()  # type: ignore[union-attr]
                    self._pwmb.setup()  # type: ignore[union-attr]
                    self._pwmb.set_frequency(self._pwm_frequency)  # type: ignore[union-attr]

                # Setup standby pin
                if self._stby is not None:
                    self._stby.setup()

            except Exception as e:
                self._state = DriverState.ERROR
                raise HardwareNotFoundError(f"Failed to setup TB6612 pins: {e}") from e

        # Initialize motors to safe state (coast)
        for channel in (TB6612Channel.A, TB6612Channel.B):
            self._set_direction_pins(channel, in1=False, in2=False)
            self._set_speed_pin(channel, 0.0)
            self._motor_states[channel] = TB6612MotorState(
                speed=0.0,
                direction=TB6612Direction.COAST,
                enabled=self._tb6612_config.enable_on_connect,
                in1_state=False,
                in2_state=False,
            )

        # Exit standby if configured
        if self._tb6612_config.exit_standby_on_connect:
            self._set_standby(False)
        else:
            self._set_standby(True)

        self._state = DriverState.CONNECTED
        logger.info("TB6612 connected (simulation=%s)", self._simulation_mode)

    def disconnect(self) -> None:
        """Disconnect from the TB6612 driver.

        Stops all motors, enters standby, and cleans up GPIO resources.
        """
        if self._state == DriverState.DISCONNECTED:
            logger.warning("TB6612 already disconnected")
            return

        logger.debug("Disconnecting TB6612...")

        # Stop all motors and enter standby
        for channel in (TB6612Channel.A, TB6612Channel.B):
            try:
                self.coast(channel)
            except Exception as e:
                logger.warning("Error stopping motor %d: %s", channel, e)

        # Enter standby mode
        self._set_standby(True)

        if not self._simulation_mode:
            try:
                # Cleanup motor A pins
                if self._ain1 is not None:
                    self._pwma.stop()  # type: ignore[union-attr]
                    self._ain1.cleanup()
                    self._ain2.cleanup()  # type: ignore[union-attr]
                    self._pwma.cleanup()  # type: ignore[union-attr]

                # Cleanup motor B pins
                if self._bin1 is not None:
                    self._pwmb.stop()  # type: ignore[union-attr]
                    self._bin1.cleanup()
                    self._bin2.cleanup()  # type: ignore[union-attr]
                    self._pwmb.cleanup()  # type: ignore[union-attr]

                # Cleanup standby pin
                if self._stby is not None:
                    self._stby.cleanup()

            except Exception as e:
                logger.warning("Error cleaning up TB6612 pins: %s", e)

        self._state = DriverState.DISCONNECTED
        logger.info("TB6612 disconnected")

    # -------------------------------------------------------------------------
    # Standby Control
    # -------------------------------------------------------------------------

    def standby(self) -> None:
        """Enter standby mode (low power, all outputs Hi-Z).

        In standby mode, all motor outputs are in high-impedance state.
        Motors will coast to a stop.
        """
        self._require_connected()
        self._set_standby(True)
        logger.info("TB6612 entered standby mode")

    def wake(self) -> None:
        """Exit standby mode (resume normal operation).

        After exiting standby, motors will be in their previous state.
        """
        self._require_connected()
        self._set_standby(False)
        logger.info("TB6612 exited standby mode")

    def _set_standby(self, standby: bool) -> None:
        """Set standby state.

        Args:
            standby: True to enter standby, False to exit.
        """
        self._in_standby = standby

        if not self._simulation_mode and self._stby is not None:
            # STBY pin: HIGH = active, LOW = standby
            self._stby.write(not standby)

    # -------------------------------------------------------------------------
    # Motor Control Methods
    # -------------------------------------------------------------------------

    def set_motor(
        self,
        channel: int | TB6612Channel,
        speed: float,
        direction: TB6612Direction = TB6612Direction.FORWARD,
    ) -> None:
        """Set motor speed and direction.

        Args:
            channel: Motor channel (0=A, 1=B).
            speed: Speed from 0.0 to 1.0.
            direction: Motor direction (FORWARD, REVERSE, BRAKE, COAST).

        Raises:
            ValueError: If channel or speed is invalid.
            DisabledError: If driver is disabled or in standby.
        """
        self._require_connected()
        self._require_not_standby()
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
        if config.inverted and direction in (TB6612Direction.FORWARD, TB6612Direction.REVERSE):
            actual_direction = (
                TB6612Direction.REVERSE
                if direction == TB6612Direction.FORWARD
                else TB6612Direction.FORWARD
            )

        # Set direction pins based on direction
        in1_state: bool
        in2_state: bool

        if actual_direction == TB6612Direction.FORWARD:
            in1_state, in2_state = True, False
        elif actual_direction == TB6612Direction.REVERSE:
            in1_state, in2_state = False, True
        elif actual_direction == TB6612Direction.BRAKE:
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

    def forward(self, channel: int | TB6612Channel, speed: float = 1.0) -> None:
        """Run motor forward at specified speed.

        Args:
            channel: Motor channel (0=A, 1=B).
            speed: Speed from 0.0 to 1.0.
        """
        self.set_motor(channel, speed, TB6612Direction.FORWARD)

    def reverse(self, channel: int | TB6612Channel, speed: float = 1.0) -> None:
        """Run motor in reverse at specified speed.

        Args:
            channel: Motor channel (0=A, 1=B).
            speed: Speed from 0.0 to 1.0.
        """
        self.set_motor(channel, speed, TB6612Direction.REVERSE)

    def brake(self, channel: int | TB6612Channel) -> None:
        """Stop motor with active braking.

        Uses short brake mode (both direction pins HIGH) to stop quickly.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        self.set_motor(channel, 1.0, TB6612Direction.BRAKE)

    def coast(self, channel: int | TB6612Channel) -> None:
        """Stop motor with free-running (coast).

        Motor will slowly decelerate due to friction only.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        self.set_motor(channel, 0.0, TB6612Direction.COAST)

    def stop(
        self,
        channel: int | TB6612Channel,
        mode: TB6612BrakeMode | None = None,
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

        if mode == TB6612BrakeMode.BRAKE:
            self.brake(channel)
        else:
            self.coast(channel)

    def stop_all(self, mode: TB6612BrakeMode | None = None) -> None:
        """Stop all motors.

        Args:
            mode: Stopping mode. If None, uses each motor's default_brake_mode.
        """
        for channel in (TB6612Channel.A, TB6612Channel.B):
            try:
                self.stop(channel, mode)
            except Exception as e:
                logger.warning("Error stopping motor %d: %s", channel, e)

    def emergency_stop(self) -> None:
        """Emergency stop all motors with active braking.

        This method should be used for safety-critical stops.
        It attempts to brake all motors immediately, then enters standby.
        """
        logger.warning("TB6612 EMERGENCY STOP")

        # Brake all motors immediately
        for channel in (TB6612Channel.A, TB6612Channel.B):
            try:
                self._set_direction_pins(channel, True, True)  # Brake
                self._set_speed_pin(channel, 1.0)
                self._motor_states[channel].speed = 0.0
                self._motor_states[channel].direction = TB6612Direction.BRAKE
                self._motor_states[channel].in1_state = True
                self._motor_states[channel].in2_state = True
            except Exception as e:
                logger.error("Error during emergency stop on motor %d: %s", channel, e)

        # Enter standby for safety
        self._set_standby(True)

        # Disable driver
        self._enabled = False

    # -------------------------------------------------------------------------
    # Motor Enable/Disable
    # -------------------------------------------------------------------------

    def enable_motor(self, channel: int | TB6612Channel) -> None:
        """Enable a motor channel.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        channel = self._validate_motor_channel(channel)
        self._motor_states[channel].enabled = True
        logger.debug("Motor %d enabled", channel)

    def disable_motor(self, channel: int | TB6612Channel) -> None:
        """Disable a motor channel.

        Stops the motor before disabling.

        Args:
            channel: Motor channel (0=A, 1=B).
        """
        channel = self._validate_motor_channel(channel)

        # Stop the motor first (best-effort, log failures)
        if self._state == DriverState.CONNECTED and not self._in_standby:
            try:
                self.coast(channel)
            except Exception as e:
                logger.warning("Failed to coast motor %d during disable: %s", channel, e)

        self._motor_states[channel].enabled = False
        logger.debug("Motor %d disabled", channel)

    def enable_all(self) -> None:
        """Enable all motor channels."""
        for channel in (TB6612Channel.A, TB6612Channel.B):
            self.enable_motor(channel)

    def disable_all(self) -> None:
        """Disable all motor channels."""
        for channel in (TB6612Channel.A, TB6612Channel.B):
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
            self.set_motor(channel, value, TB6612Direction.FORWARD)
        else:
            self.set_motor(channel, abs(value), TB6612Direction.REVERSE)

    def get_channel(self, channel: int) -> float:
        """Get channel value (normalized -1.0 to 1.0).

        Args:
            channel: Motor channel (0=A, 1=B).

        Returns:
            Current speed with sign indicating direction.
        """
        channel = self._validate_motor_channel(channel)
        state = self._motor_states[channel]

        if state.direction == TB6612Direction.REVERSE:
            return -state.speed
        return state.speed

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
        if value >= 0:
            self.set_motor(channel, value, TB6612Direction.FORWARD)
        else:
            self.set_motor(channel, abs(value), TB6612Direction.REVERSE)

    def _read_channel(self, channel: int) -> float:
        """Read the current value of a motor channel.

        Implements the abstract method from Driver.

        Args:
            channel: Channel number (0 or 1).

        Returns:
            Current speed with sign indicating direction.
        """
        state = self._motor_states[channel]
        if state.direction == TB6612Direction.REVERSE:
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
            in1: State for AIN1/BIN1 pin.
            in2: State for AIN2/BIN2 pin.
        """
        if self._simulation_mode:
            return

        if channel == TB6612Channel.A:
            if self._ain1 is not None:
                self._ain1.write(in1)
                self._ain2.write(in2)  # type: ignore[union-attr]
        elif self._bin1 is not None:
            self._bin1.write(in1)
            self._bin2.write(in2)  # type: ignore[union-attr]

    def _set_speed_pin(self, channel: int, speed: float) -> None:
        """Set speed (PWM duty cycle) for a motor channel.

        Args:
            channel: Motor channel (0 or 1).
            speed: Speed from 0.0 to 1.0.
        """
        if self._simulation_mode:
            return

        if channel == TB6612Channel.A:
            if self._pwma is not None:
                self._pwma.set_duty_cycle(speed)
                if speed > 0:
                    self._pwma.start()
                else:
                    self._pwma.stop()
        elif self._pwmb is not None:
            self._pwmb.set_duty_cycle(speed)
            if speed > 0:
                self._pwmb.start()
            else:
                self._pwmb.stop()

    # -------------------------------------------------------------------------
    # Utility Methods
    # -------------------------------------------------------------------------

    def _require_connected(self) -> None:
        """Require driver to be connected.

        Raises:
            HardwareNotFoundError: If not connected.
        """
        if self._state != DriverState.CONNECTED:
            raise HardwareNotFoundError(f"TB6612 not connected (state={self._state.value})")

    def _require_not_standby(self) -> None:
        """Require driver to not be in standby mode.

        Raises:
            DisabledError: If in standby mode.
        """
        if self._in_standby:
            raise DisabledError("TB6612 is in standby mode. Call wake() first.")

    # -------------------------------------------------------------------------
    # Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """Return string representation."""
        return (
            f"TB6612("
            f"state={self._state.value}, "
            f"simulation={self._simulation_mode}, "
            f"standby={self._in_standby}, "
            f"motor_a={self._motor_states[TB6612Channel.A]}, "
            f"motor_b={self._motor_states[TB6612Channel.B]})"
        )
