"""Gripper controller for robot grippers and end effectors.

This module provides a Gripper controller for managing robotic grippers,
claws, and other gripping mechanisms. Common examples include:
- Robot arm end effectors
- Pick-and-place grippers
- Claws and pincers
- Parallel jaw grippers
- Vacuum grippers (with force sensing)

Example:
    >>> from robo_infra.controllers.gripper import Gripper, GripperConfig
    >>> from robo_infra.actuators.servo import Servo
    >>>
    >>> # Create gripper actuator (servo-based)
    >>> actuator = Servo(name="gripper_servo", angle_range=(0, 90))
    >>>
    >>> # Create gripper controller
    >>> config = GripperConfig(
    ...     name="gripper",
    ...     open_position=0,
    ...     closed_position=90,
    ... )
    >>> gripper = Gripper(
    ...     name="my_gripper",
    ...     actuator=actuator,
    ...     config=config,
    ... )
    >>> gripper.enable()
    >>>
    >>> # Gripper commands
    >>> gripper.open()
    >>> gripper.close()
    >>> gripper.grip()  # Close until force detected or fully closed
    >>> gripper.release()
"""

from __future__ import annotations

import logging
from enum import Enum
from typing import TYPE_CHECKING

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator
from robo_infra.core.controller import Controller, ControllerConfig


if TYPE_CHECKING:
    from robo_infra.core.sensor import Sensor


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class GripperState(Enum):
    """States a Gripper can be in."""

    OPEN = "open"
    CLOSED = "closed"
    GRIPPING = "gripping"
    MOVING = "moving"
    ERROR = "error"
    DISABLED = "disabled"


# =============================================================================
# Configuration Models
# =============================================================================


class GripperConfig(BaseModel):
    """Configuration for a Gripper controller.

    Attributes:
        name: Human-readable name for the gripper.
        open_position: Position value when gripper is fully open.
        closed_position: Position value when gripper is fully closed.
        grip_threshold: Position tolerance for grip detection.
        force_sensor: Name of force sensor to use for grip detection.
        grip_force_threshold: Force threshold for grip detection.

    Example:
        >>> config = GripperConfig(
        ...     name="parallel_gripper",
        ...     open_position=0,      # Fully open at 0
        ...     closed_position=100,  # Fully closed at 100
        ...     grip_threshold=5,     # Within 5 units = gripped
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Gripper name")
    description: str = Field(default="", description="Human-readable description")

    # Position configuration
    open_position: float = Field(
        default=0.0,
        description="Position value when gripper is fully open",
    )
    closed_position: float = Field(
        default=100.0,
        description="Position value when gripper is fully closed",
    )
    grip_threshold: float = Field(
        default=0.1,
        ge=0,
        description="Position tolerance for grip detection (relative to closed)",
    )

    # Force sensing (optional)
    force_sensor: str | None = Field(
        default=None,
        description="Name of force sensor for grip detection",
    )
    grip_force_threshold: float = Field(
        default=0.5,
        gt=0,
        description="Force threshold for grip detection (sensor units)",
    )

    # Speed configuration
    default_speed: float = Field(
        default=1.0,
        ge=0.0,
        le=1.0,
        description="Default movement speed (0.0 to 1.0)",
    )
    grip_speed: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Speed when gripping (slower for force detection)",
    )

    # Safety
    max_grip_attempts: int = Field(
        default=3,
        ge=1,
        description="Maximum grip attempts before giving up",
    )
    grip_timeout: float = Field(
        default=5.0,
        gt=0,
        description="Timeout for grip operation in seconds",
    )

    # Computed properties
    @property
    def range(self) -> float:
        """Calculate the total range of motion."""
        return abs(self.closed_position - self.open_position)

    @property
    def is_inverted(self) -> bool:
        """Check if gripper motion is inverted (close < open)."""
        return self.closed_position < self.open_position


# =============================================================================
# Gripper Controller (stub for 2.3.1 - full implementation in 2.3.2)
# =============================================================================


class Gripper(Controller):
    """Controller for robotic grippers and end effectors.

    Gripper provides high-level control for gripping mechanisms,
    with optional force sensing for grip detection.

    Features:
    - Open/close control
    - Partial position control
    - Force-based grip detection (optional)
    - Grip state tracking

    Full implementation in section 2.3.2.
    """

    def __init__(
        self,
        name: str,
        actuator: Actuator,
        *,
        config: GripperConfig | None = None,
        force_sensor: Sensor | None = None,
    ) -> None:
        """Initialize Gripper controller.

        Args:
            name: Controller name
            actuator: Gripper actuator (servo, motor, etc.)
            config: Optional configuration
            force_sensor: Optional force sensor for grip detection

        Raises:
            ValueError: If actuator is None
        """
        if actuator is None:
            raise ValueError("Gripper actuator is required")

        # Create config if not provided
        if config is None:
            config = GripperConfig(name=name)

        # Initialize base controller
        super().__init__(name, config=ControllerConfig(name=name))

        # Store Gripper-specific config
        self._gripper_config = config

        # Store actuator
        self._actuator = actuator

        # Add actuator to base controller
        self.add_actuator("gripper", actuator)

        # Store force sensor (optional)
        self._force_sensor = force_sensor
        if force_sensor is not None:
            self.add_sensor("force", force_sensor)

        # Gripper-specific state
        self._gripper_state = GripperState.DISABLED
        self._is_gripping = False

        logger.debug(
            "Gripper '%s' initialized with actuator=%s, force_sensor=%s",
            name,
            actuator.name,
            force_sensor.name if force_sensor else None,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def actuator(self) -> Actuator:
        """Gripper actuator."""
        return self._actuator

    @property
    def force_sensor(self) -> Sensor | None:
        """Force sensor (if configured)."""
        return self._force_sensor

    @property
    def gripper_config(self) -> GripperConfig:
        """Gripper-specific configuration."""
        return self._gripper_config

    @property
    def gripper_state(self) -> GripperState:
        """Current gripper state."""
        return self._gripper_state

    @property
    def position(self) -> float:
        """Current gripper position."""
        return self._actuator.get()

    @property
    def is_open(self) -> bool:
        """Check if gripper is at open position."""
        tolerance = self._gripper_config.grip_threshold
        return abs(self.position - self._gripper_config.open_position) <= tolerance

    @property
    def is_closed(self) -> bool:
        """Check if gripper is at closed position."""
        tolerance = self._gripper_config.grip_threshold
        return abs(self.position - self._gripper_config.closed_position) <= tolerance

    @property
    def is_gripping(self) -> bool:
        """Check if gripper has detected a grip (object held)."""
        return self._is_gripping

    # -------------------------------------------------------------------------
    # Controller Abstract Methods
    # -------------------------------------------------------------------------

    def _do_home(self) -> None:
        """Implement abstract home method from Controller.

        For gripper, 'home' means opening fully.
        """
        self.open()

    def _do_stop(self) -> None:
        """Implement abstract stop method from Controller."""
        self.stop()

    # -------------------------------------------------------------------------
    # Gripper Control Methods
    # -------------------------------------------------------------------------

    def stop(self) -> None:
        """Stop gripper movement immediately."""
        if hasattr(self._actuator, "stop"):
            self._actuator.stop()
        self._update_state_from_position()
        logger.debug("Gripper '%s' stopped at position %.2f", self.name, self.position)

    def open(self, speed: float = 1.0) -> None:
        """Open the gripper to fully open position.

        Args:
            speed: Movement speed (0.0 to 1.0). Defaults to 1.0.
                   Uses config.default_speed if not specified.

        Raises:
            RuntimeError: If gripper is not enabled.

        Example:
            >>> gripper.enable()
            >>> gripper.open()
            >>> assert gripper.is_open
        """
        self._check_enabled("open")

        # Use default speed if not overridden
        if speed == 1.0:
            speed = self._gripper_config.default_speed

        self._gripper_state = GripperState.MOVING
        self._is_gripping = False

        logger.debug(
            "Gripper '%s' opening to %.2f at speed %.2f",
            self.name,
            self._gripper_config.open_position,
            speed,
        )

        self._move_to(self._gripper_config.open_position, speed)
        self._gripper_state = GripperState.OPEN

        logger.debug("Gripper '%s' is now open", self.name)

    def close(self, speed: float = 1.0) -> None:
        """Close the gripper to fully closed position.

        Args:
            speed: Movement speed (0.0 to 1.0). Defaults to 1.0.
                   Uses config.default_speed if not specified.

        Raises:
            RuntimeError: If gripper is not enabled.

        Example:
            >>> gripper.enable()
            >>> gripper.close()
            >>> assert gripper.is_closed
        """
        self._check_enabled("close")

        # Use default speed if not overridden
        if speed == 1.0:
            speed = self._gripper_config.default_speed

        self._gripper_state = GripperState.MOVING

        logger.debug(
            "Gripper '%s' closing to %.2f at speed %.2f",
            self.name,
            self._gripper_config.closed_position,
            speed,
        )

        self._move_to(self._gripper_config.closed_position, speed)
        self._gripper_state = GripperState.CLOSED

        logger.debug("Gripper '%s' is now closed", self.name)

    def set(self, position: float) -> None:
        """Move gripper to a specific position.

        Args:
            position: Target position (between open and closed).

        Raises:
            RuntimeError: If gripper is not enabled.
            ValueError: If position is out of range.

        Example:
            >>> gripper.enable()
            >>> gripper.set(50.0)  # Half-way between open and closed
        """
        self._check_enabled("set")

        # Validate position is within range
        min_pos = min(self._gripper_config.open_position, self._gripper_config.closed_position)
        max_pos = max(self._gripper_config.open_position, self._gripper_config.closed_position)

        if position < min_pos or position > max_pos:
            raise ValueError(f"Position {position} out of range [{min_pos}, {max_pos}]")

        self._gripper_state = GripperState.MOVING

        logger.debug(
            "Gripper '%s' moving to position %.2f",
            self.name,
            position,
        )

        self._move_to(position, self._gripper_config.default_speed)
        self._update_state_from_position()

        logger.debug(
            "Gripper '%s' reached position %.2f, state=%s",
            self.name,
            self.position,
            self._gripper_state.value,
        )

    def grip(self) -> bool:
        """Close gripper until force threshold or fully closed.

        This method closes the gripper slowly, checking for force feedback
        (if a force sensor is configured) to detect when an object is gripped.

        Returns:
            True if object is gripped (force detected before fully closed),
            False if gripper closed fully without detecting an object.

        Raises:
            RuntimeError: If gripper is not enabled.

        Example:
            >>> gripper.enable()
            >>> gripper.open()
            >>> # Place object between gripper jaws
            >>> success = gripper.grip()
            >>> if success:
            ...     print("Object gripped!")
        """
        self._check_enabled("grip")

        self._gripper_state = GripperState.MOVING
        self._is_gripping = False

        logger.debug("Gripper '%s' attempting to grip", self.name)

        # Use slower grip speed for force detection
        speed = self._gripper_config.grip_speed

        # If we have a force sensor, close incrementally and check force
        if self._force_sensor is not None:
            gripped = self._grip_with_force_sensing(speed)
        else:
            # No force sensor - just close and assume gripped if not fully closed
            self._move_to(self._gripper_config.closed_position, speed)
            gripped = False  # Can't detect grip without force sensor

        if gripped:
            self._is_gripping = True
            self._gripper_state = GripperState.GRIPPING
            logger.info("Gripper '%s' gripped object at position %.2f", self.name, self.position)
        else:
            self._gripper_state = GripperState.CLOSED
            logger.debug("Gripper '%s' closed without detecting object", self.name)

        return gripped

    def release(self) -> None:
        """Release gripped object by opening gripper.

        This is an alias for open() for semantic clarity.

        Raises:
            RuntimeError: If gripper is not enabled.

        Example:
            >>> success = gripper.grip()
            >>> if success:
            ...     # Do something with object
            ...     gripper.release()
        """
        self._is_gripping = False
        self.open()
        logger.debug("Gripper '%s' released object", self.name)

    # -------------------------------------------------------------------------
    # Enable/Disable Overrides
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the gripper controller."""
        super().enable()
        self._actuator.enable()
        self._update_state_from_position()
        logger.info("Gripper '%s' enabled", self.name)

    def disable(self) -> None:
        """Disable the gripper controller."""
        super().disable()
        self._actuator.disable()
        self._gripper_state = GripperState.DISABLED
        self._is_gripping = False
        logger.info("Gripper '%s' disabled", self.name)

    # -------------------------------------------------------------------------
    # Private Helper Methods
    # -------------------------------------------------------------------------

    def _check_enabled(self, operation: str) -> None:
        """Check if gripper is enabled before operation.

        Args:
            operation: Name of operation being attempted.

        Raises:
            RuntimeError: If gripper is not enabled.
        """
        if not self.is_enabled:
            raise RuntimeError(
                f"Cannot {operation}: Gripper '{self.name}' is not enabled. "
                f"Call gripper.enable() first."
            )

    def _move_to(self, position: float, speed: float) -> None:
        """Move actuator to position at given speed.

        Args:
            position: Target position.
            speed: Movement speed (0.0 to 1.0).
        """
        # If actuator supports speed parameter, use it
        if hasattr(self._actuator, "set_speed"):
            self._actuator.set_speed(speed)

        self._actuator.set(position)

    def _update_state_from_position(self) -> None:
        """Update gripper state based on current position."""
        if self.is_open:
            self._gripper_state = GripperState.OPEN
            self._is_gripping = False
        elif self.is_closed:
            self._gripper_state = GripperState.CLOSED
        elif self._is_gripping:
            self._gripper_state = GripperState.GRIPPING
        else:
            # Intermediate position - could be partially closed
            self._gripper_state = GripperState.CLOSED

    def _grip_with_force_sensing(self, speed: float) -> bool:
        """Close gripper with force sensing to detect grip.

        Args:
            speed: Movement speed (0.0 to 1.0).

        Returns:
            True if force threshold was exceeded (object gripped),
            False if closed fully without force detection.
        """
        assert self._force_sensor is not None

        threshold = self._gripper_config.grip_force_threshold
        closed_pos = self._gripper_config.closed_position

        # Move incrementally toward closed position
        # In a real implementation, this would be a control loop
        # For now, we move to closed and check force
        self._move_to(closed_pos, speed)

        # Check force sensor - read() returns a Reading object
        reading = self._force_sensor.read()
        force: float = reading.value

        if force >= threshold:
            logger.debug(
                "Gripper '%s' detected force %.2f >= threshold %.2f",
                self.name,
                force,
                threshold,
            )
            return True

        return False

    # -------------------------------------------------------------------------
    # String Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"Gripper(name='{self.name}', "
            f"position={self.position:.2f}, "
            f"state={self._gripper_state.value})"
        )


__all__ = [
    "Gripper",
    "GripperConfig",
    "GripperState",
]
