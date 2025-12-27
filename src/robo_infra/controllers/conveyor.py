"""Conveyor belt controller for linear motion systems.

This module provides a Conveyor controller for managing conveyor belts,
linear motion systems, and indexing tables. Common examples include:
- Factory conveyor belts
- Pick-and-place feeders
- Assembly line conveyors
- Packaging line indexers
- Material handling systems

Example:
    >>> from robo_infra.controllers.conveyor import Conveyor, ConveyorConfig
    >>> from robo_infra.actuators.dc_motor import DCMotor
    >>>
    >>> # Create motor for conveyor
    >>> motor = DCMotor(name="conveyor_motor")
    >>>
    >>> # Create conveyor controller
    >>> config = ConveyorConfig(
    ...     name="main_conveyor",
    ...     belt_length=2.0,      # 2 meters
    ...     speed_max=0.5,        # 0.5 m/s max
    ...     index_distance=0.1,   # 10cm increments
    ... )
    >>> conveyor = Conveyor(
    ...     name="main_conveyor",
    ...     motor=motor,
    ...     config=config,
    ... )
    >>> conveyor.enable()
    >>>
    >>> # Conveyor commands
    >>> conveyor.run(speed=0.3)           # Run at 0.3 m/s
    >>> conveyor.run(speed=0.3, reverse=True)  # Reverse
    >>> conveyor.jog(distance=0.5)        # Move 50cm
    >>> conveyor.index(count=3)           # Move 3 increments
    >>> conveyor.stop()
"""

from __future__ import annotations

import logging
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.controller import Controller, ControllerConfig


if TYPE_CHECKING:
    from collections.abc import Callable

    from robo_infra.core.actuator import Actuator
    from robo_infra.core.sensor import Sensor


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class ConveyorState(Enum):
    """States a Conveyor can be in."""

    IDLE = "idle"
    RUNNING = "running"
    JOGGING = "jogging"
    INDEXING = "indexing"
    STOPPING = "stopping"
    ERROR = "error"
    DISABLED = "disabled"


class ConveyorDirection(Enum):
    """Direction of conveyor motion."""

    FORWARD = 1
    REVERSE = -1


# =============================================================================
# Configuration Models
# =============================================================================


class ConveyorConfig(BaseModel):
    """Configuration for a Conveyor controller.

    Attributes:
        name: Human-readable name for the conveyor.
        belt_length: Total length of the conveyor belt in meters.
        speed_max: Maximum belt speed in meters per second.
        speed_min: Minimum practical belt speed in meters per second.
        acceleration: Belt acceleration rate in m/s².
        deceleration: Belt deceleration rate in m/s².
        index_distance: Distance per index increment in meters.
        encoder_ppr: Encoder pulses per revolution (if encoder is used).
        wheel_diameter: Drive wheel diameter in meters (for distance calc).

    Example:
        >>> config = ConveyorConfig(
        ...     name="assembly_line",
        ...     belt_length=5.0,       # 5 meter belt
        ...     speed_max=1.0,         # 1 m/s max speed
        ...     index_distance=0.25,   # 25cm index steps
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Conveyor name")
    description: str = Field(default="", description="Human-readable description")

    # Belt dimensions
    belt_length: float = Field(
        default=1.0,
        gt=0,
        description="Total belt length in meters",
    )
    belt_width: float = Field(
        default=0.3,
        gt=0,
        description="Belt width in meters",
    )

    # Speed configuration
    speed_max: float = Field(
        default=1.0,
        gt=0,
        description="Maximum belt speed in m/s",
    )
    speed_min: float = Field(
        default=0.01,
        gt=0,
        description="Minimum practical belt speed in m/s",
    )
    default_speed: float = Field(
        default=0.5,
        gt=0,
        description="Default operating speed in m/s",
    )

    # Acceleration
    acceleration: float = Field(
        default=0.5,
        gt=0,
        description="Acceleration rate in m/s²",
    )
    deceleration: float = Field(
        default=0.5,
        gt=0,
        description="Deceleration rate in m/s²",
    )

    # Indexing
    index_distance: float = Field(
        default=0.1,
        gt=0,
        description="Distance per index increment in meters",
    )

    # Encoder/feedback (optional)
    encoder_ppr: int | None = Field(
        default=None,
        gt=0,
        description="Encoder pulses per revolution",
    )
    wheel_diameter: float = Field(
        default=0.1,
        gt=0,
        description="Drive wheel diameter in meters",
    )

    # Safety
    soft_start: bool = Field(
        default=True,
        description="Enable soft start/stop ramping",
    )
    emergency_stop_distance: float = Field(
        default=0.1,
        gt=0,
        description="Maximum stopping distance in meters",
    )


# =============================================================================
# Status Dataclass
# =============================================================================


class ConveyorStatus:
    """Current status of a Conveyor."""

    def __init__(
        self,
        state: ConveyorState,
        current_speed: float,
        target_speed: float,
        direction: ConveyorDirection,
        distance_traveled: float,
        is_enabled: bool,
        error: str | None = None,
    ) -> None:
        """Initialize ConveyorStatus."""
        self.state = state
        self.current_speed = current_speed
        self.target_speed = target_speed
        self.direction = direction
        self.distance_traveled = distance_traveled
        self.is_enabled = is_enabled
        self.error = error


# =============================================================================
# Conveyor Controller
# =============================================================================


class Conveyor(Controller):
    """Controller for conveyor belts and linear motion systems.

    Conveyor provides high-level control for belt-driven systems,
    with support for continuous running, jogging, and indexing.

    Features:
    - Continuous run at variable speed
    - Bidirectional motion
    - Distance-based jogging
    - Count-based indexing
    - Soft start/stop ramping
    - Position tracking (with encoder)

    Example:
        >>> conveyor = Conveyor(
        ...     name="feeder",
        ...     motor=dc_motor,
        ...     config=ConveyorConfig(name="feeder", index_distance=0.1),
        ... )
        >>> conveyor.enable()
        >>> conveyor.run(speed=0.5)
        >>> conveyor.index(count=5)  # Move 5 * 0.1m = 0.5m
    """

    def __init__(
        self,
        name: str,
        motor: Actuator,
        *,
        config: ConveyorConfig | None = None,
        encoder: Sensor | None = None,
    ) -> None:
        """Initialize Conveyor controller.

        Args:
            name: Controller name
            motor: Motor actuator for driving the belt
            config: Optional configuration
            encoder: Optional encoder for position feedback

        Raises:
            ValueError: If motor is None
        """
        if motor is None:
            raise ValueError("Conveyor motor is required")

        # Create config if not provided
        if config is None:
            config = ConveyorConfig(name=name)

        # Initialize base controller
        super().__init__(name, config=ControllerConfig(name=name))

        # Store conveyor-specific config
        self._conveyor_config = config

        # Store motor
        self._motor = motor
        self.add_actuator("motor", motor)

        # Store encoder (optional)
        self._encoder = encoder
        if encoder is not None:
            self.add_sensor("encoder", encoder)

        # Conveyor-specific state
        self._conveyor_state = ConveyorState.DISABLED
        self._current_speed: float = 0.0
        self._target_speed: float = 0.0
        self._direction = ConveyorDirection.FORWARD
        self._distance_traveled: float = 0.0
        self._is_running: bool = False

        # Jog/index tracking
        self._jog_target: float | None = None
        self._index_remaining: int = 0

        logger.debug(
            "Conveyor '%s' initialized with motor=%s, encoder=%s",
            name,
            motor.name,
            encoder.name if encoder else None,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def motor(self) -> Actuator:
        """Conveyor motor actuator."""
        return self._motor

    @property
    def encoder(self) -> Sensor | None:
        """Optional encoder sensor."""
        return self._encoder

    @property
    def conveyor_config(self) -> ConveyorConfig:
        """Conveyor configuration."""
        return self._conveyor_config

    @property
    def conveyor_state(self) -> ConveyorState:
        """Current conveyor state."""
        return self._conveyor_state

    @property
    def current_speed(self) -> float:
        """Current belt speed in m/s."""
        return self._current_speed

    @property
    def target_speed(self) -> float:
        """Target belt speed in m/s."""
        return self._target_speed

    @property
    def direction(self) -> ConveyorDirection:
        """Current direction."""
        return self._direction

    @property
    def distance_traveled(self) -> float:
        """Total distance traveled in meters."""
        return self._distance_traveled

    @property
    def is_running(self) -> bool:
        """Check if conveyor is running."""
        return self._is_running

    # -------------------------------------------------------------------------
    # Controller Lifecycle
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the conveyor controller."""
        super().enable()
        self._conveyor_state = ConveyorState.IDLE
        self._motor.enable()
        logger.info("Conveyor '%s' enabled", self.name)

    def disable(self) -> None:
        """Disable the conveyor controller."""
        self._stop_motor()
        self._conveyor_state = ConveyorState.DISABLED
        self._motor.disable()
        super().disable()
        logger.info("Conveyor '%s' disabled", self.name)

    def home(self) -> None:
        """Reset distance counter to zero."""
        self._require_enabled()
        self._distance_traveled = 0.0
        self._is_homed = True
        logger.info("Conveyor '%s' homed (distance reset)", self.name)

    def _do_home(self) -> None:
        """Perform homing sequence for the conveyor.

        Resets distance counter to zero.
        """
        self._distance_traveled = 0.0
        self._is_homed = True

    def _do_stop(self) -> None:
        """Perform emergency stop for the conveyor.

        Immediately stops the motor and resets state.
        """
        self._stop_motor()
        self._conveyor_state = ConveyorState.IDLE
        self._is_running = False
        self._jog_target = None
        self._index_remaining = 0

    # -------------------------------------------------------------------------
    # Conveyor Operations
    # -------------------------------------------------------------------------

    def run(
        self,
        speed: float | None = None,
        *,
        reverse: bool = False,
    ) -> None:
        """Run the conveyor continuously at specified speed.

        Args:
            speed: Belt speed in m/s (uses default if None)
            reverse: Run in reverse direction

        Raises:
            DisabledError: If conveyor is not enabled
            ValueError: If speed exceeds limits
        """

        self._require_enabled()

        # Use default speed if not specified
        if speed is None:
            speed = self._conveyor_config.default_speed

        # Validate speed
        speed = abs(speed)
        if speed > self._conveyor_config.speed_max:
            raise ValueError(f"Speed {speed} exceeds maximum {self._conveyor_config.speed_max}")

        # Set direction
        self._direction = ConveyorDirection.REVERSE if reverse else ConveyorDirection.FORWARD

        # Set target and start
        self._target_speed = speed
        self._start_motor(speed, self._direction)
        self._conveyor_state = ConveyorState.RUNNING
        self._is_running = True

        logger.info(
            "Conveyor '%s' running at %.2f m/s %s",
            self.name,
            speed,
            "reverse" if reverse else "forward",
        )

    def stop(self) -> None:
        """Stop the conveyor (soft stop with deceleration).

        This performs a controlled stop. For emergency stop, use emergency_stop().
        """
        if self._conveyor_state == ConveyorState.DISABLED:
            return

        self._conveyor_state = ConveyorState.STOPPING
        self._stop_motor()
        self._conveyor_state = ConveyorState.IDLE
        self._is_running = False
        self._jog_target = None
        self._index_remaining = 0

        logger.info("Conveyor '%s' stopped", self.name)

    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motion."""
        self._stop_motor()
        self._conveyor_state = ConveyorState.IDLE
        self._is_running = False
        self._jog_target = None
        self._index_remaining = 0
        super().stop()  # Call base class emergency stop

        logger.warning("Conveyor '%s' emergency stopped", self.name)

    def jog(
        self,
        distance: float,
        speed: float | None = None,
    ) -> None:
        """Move the conveyor a specific distance.

        Args:
            distance: Distance to move in meters (negative for reverse)
            speed: Movement speed in m/s (uses default if None)

        Raises:
            DisabledError: If conveyor is not enabled
            ValueError: If speed exceeds limits
        """
        self._require_enabled()

        # Use default speed if not specified
        if speed is None:
            speed = self._conveyor_config.default_speed

        # Validate speed
        speed = abs(speed)
        if speed > self._conveyor_config.speed_max:
            raise ValueError(f"Speed {speed} exceeds maximum {self._conveyor_config.speed_max}")

        # Determine direction from distance sign
        reverse = distance < 0
        distance = abs(distance)

        self._direction = ConveyorDirection.REVERSE if reverse else ConveyorDirection.FORWARD
        self._jog_target = self._distance_traveled + (distance * self._direction.value)

        # Start movement
        self._target_speed = speed
        self._start_motor(speed, self._direction)
        self._conveyor_state = ConveyorState.JOGGING
        self._is_running = True

        # Simulate jog completion (in real system, encoder would track)
        self._simulate_jog(distance, speed)

        logger.info(
            "Conveyor '%s' jogging %.3f m at %.2f m/s",
            self.name,
            distance,
            speed,
        )

    def index(
        self,
        count: int = 1,
        speed: float | None = None,
    ) -> None:
        """Move the conveyor by a fixed number of index increments.

        Args:
            count: Number of increments to move (negative for reverse)
            speed: Movement speed in m/s (uses default if None)

        Raises:
            DisabledError: If conveyor is not enabled
            ValueError: If speed exceeds limits
        """
        self._require_enabled()

        # Calculate total distance
        distance = count * self._conveyor_config.index_distance

        # Use jog to move the calculated distance
        self._conveyor_state = ConveyorState.INDEXING
        self._index_remaining = abs(count)

        self.jog(distance, speed)

        logger.info(
            "Conveyor '%s' indexing %d increments (%.3f m)",
            self.name,
            count,
            abs(distance),
        )

    def set_speed(self, speed: float) -> None:
        """Change speed while running.

        Args:
            speed: New belt speed in m/s

        Raises:
            DisabledError: If conveyor is not enabled
            ValueError: If speed exceeds limits
        """
        self._require_enabled()

        speed = abs(speed)
        if speed > self._conveyor_config.speed_max:
            raise ValueError(f"Speed {speed} exceeds maximum {self._conveyor_config.speed_max}")

        self._target_speed = speed

        if self._is_running:
            self._start_motor(speed, self._direction)

        logger.debug("Conveyor '%s' speed set to %.2f m/s", self.name, speed)

    def reverse(self) -> None:
        """Reverse direction while maintaining speed."""
        self._require_enabled()

        if self._direction == ConveyorDirection.FORWARD:
            self._direction = ConveyorDirection.REVERSE
        else:
            self._direction = ConveyorDirection.FORWARD

        if self._is_running:
            self._start_motor(self._target_speed, self._direction)

        logger.info("Conveyor '%s' reversed to %s", self.name, self._direction.name)

    def reset_distance(self) -> None:
        """Reset the distance counter to zero."""
        self._distance_traveled = 0.0
        logger.debug("Conveyor '%s' distance reset", self.name)

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def status(self) -> ConveyorStatus:
        """Get current conveyor status."""
        return ConveyorStatus(
            state=self._conveyor_state,
            current_speed=self._current_speed,
            target_speed=self._target_speed,
            direction=self._direction,
            distance_traveled=self._distance_traveled,
            is_enabled=self._is_enabled,
            error=self._error,
        )

    # -------------------------------------------------------------------------
    # Tool Generation
    # -------------------------------------------------------------------------

    def as_tools(self) -> list[dict[str, Any] | Callable[..., Any]]:
        """Generate ai-infra compatible tools for this controller.

        Returns:
            List of callable tools for LLM agent control.
        """

        def run_conveyor(speed: float = 0.5, reverse: bool = False) -> str:
            """Run the conveyor belt continuously.

            Args:
                speed: Belt speed in meters per second (0.0 to max).
                reverse: True to run in reverse direction.

            Returns:
                Status message confirming operation.
            """
            self.run(speed=speed, reverse=reverse)
            direction = "reverse" if reverse else "forward"
            return f"Conveyor running at {speed} m/s {direction}"

        def stop_conveyor() -> str:
            """Stop the conveyor belt.

            Returns:
                Status message confirming stop.
            """
            self.stop()
            return "Conveyor stopped"

        def jog_conveyor(distance: float, speed: float = 0.5) -> str:
            """Move the conveyor a specific distance.

            Args:
                distance: Distance to move in meters (negative for reverse).
                speed: Movement speed in meters per second.

            Returns:
                Status message with distance moved.
            """
            self.jog(distance=distance, speed=speed)
            return f"Conveyor jogged {distance} meters"

        def index_conveyor(count: int = 1) -> str:
            """Move the conveyor by index increments.

            Args:
                count: Number of increments to move (negative for reverse).

            Returns:
                Status message with index count.
            """
            self.index(count=count)
            distance = count * self._conveyor_config.index_distance
            return f"Conveyor indexed {count} positions ({distance:.3f}m)"

        def get_conveyor_status() -> dict[str, Any]:
            """Get current conveyor status.

            Returns:
                Dict with state, speed, direction, and distance.
            """
            s = self.status()
            return {
                "state": s.state.value,
                "current_speed": s.current_speed,
                "target_speed": s.target_speed,
                "direction": s.direction.value,
                "distance_traveled": s.distance_traveled,
                "is_enabled": s.is_enabled,
            }

        return [
            run_conveyor,
            stop_conveyor,
            jog_conveyor,
            index_conveyor,
            get_conveyor_status,
        ]

    # -------------------------------------------------------------------------
    # Private Methods
    # -------------------------------------------------------------------------

    def _require_enabled(self) -> None:
        """Raise DisabledError if conveyor is not enabled."""
        from robo_infra.core.exceptions import DisabledError

        if not self._is_enabled:
            raise DisabledError(f"Conveyor '{self.name}' is not enabled")

    def _start_motor(self, speed: float, direction: ConveyorDirection) -> None:
        """Start the motor at specified speed and direction."""
        # Convert speed to motor value (-1.0 to 1.0)
        motor_value = (speed / self._conveyor_config.speed_max) * direction.value
        motor_value = max(-1.0, min(1.0, motor_value))

        self._motor.set(motor_value)
        self._current_speed = speed

    def _stop_motor(self) -> None:
        """Stop the motor."""
        self._motor.set(0.0)
        self._current_speed = 0.0
        self._target_speed = 0.0

    def _simulate_jog(self, distance: float, speed: float) -> None:
        """Simulate jog completion by updating distance.

        In a real system, this would be done by encoder feedback.
        """
        # Update distance traveled
        direction_sign = 1 if self._direction == ConveyorDirection.FORWARD else -1
        self._distance_traveled += distance * direction_sign

        # Mark as complete
        self._conveyor_state = ConveyorState.IDLE
        self._is_running = False
        self._jog_target = None

        # Stop motor
        self._stop_motor()


# =============================================================================
# Factory Functions
# =============================================================================


def create_conveyor(
    name: str = "conveyor",
    *,
    motor: Actuator | None = None,
    encoder: Sensor | None = None,
    config: ConveyorConfig | None = None,
    belt_length: float = 1.0,
    speed_max: float = 1.0,
    index_distance: float = 0.1,
) -> Conveyor:
    """Create a Conveyor controller with optional configuration.

    Args:
        name: Controller name
        motor: Motor actuator (creates simulated if None)
        encoder: Optional encoder sensor
        config: Full configuration (overrides other params if provided)
        belt_length: Belt length in meters
        speed_max: Maximum speed in m/s
        index_distance: Index increment distance in meters

    Returns:
        Configured Conveyor controller.

    Example:
        >>> conveyor = create_conveyor(
        ...     "assembly_line",
        ...     belt_length=5.0,
        ...     speed_max=0.5,
        ... )
    """
    from robo_infra.actuators.dc_motor import DCMotor

    # Create motor if not provided
    if motor is None:
        motor = DCMotor(
            pin_a=0,
            pin_b=1,
            enable=2,
            name=f"{name}_motor",
        )

    # Create config if not provided
    if config is None:
        config = ConveyorConfig(
            name=name,
            belt_length=belt_length,
            speed_max=speed_max,
            index_distance=index_distance,
        )

    return Conveyor(
        name=name,
        motor=motor,
        config=config,
        encoder=encoder,
    )


# =============================================================================
# Tool Generation Function
# =============================================================================


def conveyor_status(conveyor: Conveyor) -> dict[str, Any]:
    """Get conveyor status as a dictionary.

    Args:
        conveyor: Conveyor controller instance.

    Returns:
        Dictionary with conveyor status information.
    """
    s = conveyor.status()
    return {
        "name": conveyor.name,
        "state": s.state.value,
        "current_speed": s.current_speed,
        "target_speed": s.target_speed,
        "direction": s.direction.name,
        "distance_traveled": s.distance_traveled,
        "is_enabled": s.is_enabled,
        "error": s.error,
    }
