"""Differential drive controller for wheeled robots.

This module provides a DifferentialDrive controller for robots with two
independently driven wheels (tank-style steering). Common examples include:
- Rovers and mobile robots
- Tank-drive robots
- Warehouse AGVs
- Robotic vacuum cleaners

Example:
    >>> from robo_infra.controllers.differential import DifferentialDrive, DifferentialDriveConfig
    >>> from robo_infra.actuators.dc_motor import DCMotor
    >>>
    >>> # Create motors
    >>> left_motor = DCMotor(name="left_wheel")
    >>> right_motor = DCMotor(name="right_wheel")
    >>>
    >>> # Create differential drive controller
    >>> config = DifferentialDriveConfig(
    ...     name="rover",
    ...     wheel_diameter=0.1,  # 100mm wheels
    ...     track_width=0.3,     # 300mm between wheels
    ... )
    >>> rover = DifferentialDrive(
    ...     name="my_rover",
    ...     left=left_motor,
    ...     right=right_motor,
    ...     config=config,
    ... )
    >>> rover.enable()
    >>>
    >>> # Drive commands
    >>> rover.forward(speed=0.5)
    >>> rover.turn_left(speed=0.3)
    >>> rover.stop()
"""

from __future__ import annotations

import logging
from enum import Enum

from pydantic import BaseModel, Field

from robo_infra.actuators.dc_motor import DCMotor
from robo_infra.core.controller import Controller, ControllerConfig


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class DifferentialDriveState(Enum):
    """States a DifferentialDrive can be in."""

    IDLE = "idle"
    MOVING = "moving"
    TURNING = "turning"
    SPINNING = "spinning"
    BRAKING = "braking"
    ERROR = "error"
    DISABLED = "disabled"


# =============================================================================
# Configuration Models
# =============================================================================


class DifferentialDriveConfig(BaseModel):
    """Configuration for a DifferentialDrive controller.

    Attributes:
        name: Human-readable name for the drive system.
        wheel_diameter: Diameter of drive wheels in meters.
        track_width: Distance between wheel centers in meters.
        max_speed: Maximum linear speed in meters per second.
        invert_left: Invert left motor direction.
        invert_right: Invert right motor direction.

    Example:
        >>> config = DifferentialDriveConfig(
        ...     name="rover_drive",
        ...     wheel_diameter=0.1,   # 100mm wheels
        ...     track_width=0.3,      # 300mm track width
        ...     max_speed=1.0,        # 1 m/s max
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Drive system name")
    description: str = Field(default="", description="Human-readable description")

    # Physical dimensions (in meters)
    wheel_diameter: float = Field(
        default=0.065,
        gt=0,
        description="Wheel diameter in meters (default: 65mm)",
    )
    track_width: float = Field(
        default=0.15,
        gt=0,
        description="Distance between wheel centers in meters (default: 150mm)",
    )

    # Speed limits
    max_speed: float = Field(
        default=1.0,
        gt=0,
        description="Maximum linear speed in m/s",
    )
    max_angular_speed: float = Field(
        default=3.14,
        gt=0,
        description="Maximum angular speed in rad/s (default: ~180 deg/s)",
    )

    # Motor configuration
    invert_left: bool = Field(
        default=False,
        description="Invert left motor direction",
    )
    invert_right: bool = Field(
        default=False,
        description="Invert right motor direction",
    )

    # Control parameters
    ramp_rate: float = Field(
        default=2.0,
        gt=0,
        description="Speed ramp rate (speed units per second)",
    )
    deadband: float = Field(
        default=0.05,
        ge=0,
        le=0.5,
        description="Motor deadband threshold (speeds below this are treated as 0)",
    )

    # Calculated properties
    @property
    def wheel_circumference(self) -> float:
        """Calculate wheel circumference in meters."""
        import math

        return math.pi * self.wheel_diameter

    @property
    def turning_radius(self) -> float:
        """Calculate minimum turning radius (spin in place = 0)."""
        return self.track_width / 2


# =============================================================================
# DifferentialDrive Controller
# =============================================================================


class DifferentialDrive(Controller):
    """Controller for differential drive (two-wheel) robots.

    Differential drive robots use two independently controlled wheels
    for locomotion. By varying the relative speeds of the left and right
    wheels, the robot can move forward, backward, turn, or spin in place.

    This is the most common drive system for:
    - Mobile robots and rovers
    - Tank-drive robots
    - Warehouse AGVs
    - Educational robots

    Movement Methods:
        - forward(speed): Drive forward
        - reverse(speed): Drive backward
        - turn_left(speed): Turn left while moving
        - turn_right(speed): Turn right while moving
        - spin(speed, clockwise): Spin in place
        - arc(speed, radius): Follow a curved path
        - tank(left, right): Direct wheel control
        - stop(): Stop both motors
        - brake(): Apply braking

    Example:
        >>> from robo_infra.controllers.differential import DifferentialDrive
        >>> from robo_infra.actuators.dc_motor import DCMotor
        >>>
        >>> left = DCMotor(name="left_wheel")
        >>> right = DCMotor(name="right_wheel")
        >>> rover = DifferentialDrive("my_rover", left, right)
        >>> rover.enable()
        >>> rover.forward(0.5)
        >>> rover.turn_left(0.3)
        >>> rover.stop()
    """

    def __init__(
        self,
        name: str,
        left: DCMotor,
        right: DCMotor,
        *,
        config: DifferentialDriveConfig | None = None,
    ) -> None:
        """Initialize DifferentialDrive controller.

        Args:
            name: Controller name
            left: Left wheel motor
            right: Right wheel motor
            config: Optional configuration

        Raises:
            ValueError: If motors are None
        """
        if left is None or right is None:
            raise ValueError("Both left and right motors are required")

        # Create config if not provided
        if config is None:
            config = DifferentialDriveConfig(name=name)

        # Initialize base controller
        super().__init__(name, config=ControllerConfig(name=name))

        # Store DifferentialDrive-specific config
        self._dd_config = config

        # Store motors
        self._left_motor = left
        self._right_motor = right

        # Add motors as actuators to base controller
        self.add_actuator("left", left)
        self.add_actuator("right", right)

        # DifferentialDrive-specific state
        self._dd_state = DifferentialDriveState.DISABLED
        self._current_left_speed: float = 0.0
        self._current_right_speed: float = 0.0

        logger.debug(
            "DifferentialDrive '%s' initialized with motors: left=%s, right=%s",
            name,
            left.name,
            right.name,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def left_motor(self) -> DCMotor:
        """Left wheel motor."""
        return self._left_motor

    @property
    def right_motor(self) -> DCMotor:
        """Right wheel motor."""
        return self._right_motor

    @property
    def dd_config(self) -> DifferentialDriveConfig:
        """DifferentialDrive-specific configuration."""
        return self._dd_config

    @property
    def dd_state(self) -> DifferentialDriveState:
        """Current DifferentialDrive state."""
        return self._dd_state

    @property
    def current_speed(self) -> tuple[float, float]:
        """Current speed of left and right motors as (left, right) tuple."""
        return (self._current_left_speed, self._current_right_speed)

    # -------------------------------------------------------------------------
    # Enable/Disable Methods
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the differential drive and both motors.

        Must be called before any movement commands.

        Example:
            >>> rover.enable()
            >>> rover.forward(0.5)
        """
        self._left_motor.enable()
        self._right_motor.enable()
        self._is_enabled = True
        self._dd_state = DifferentialDriveState.IDLE
        self._on_enable()
        logger.info("DifferentialDrive '%s' enabled", self.name)

    def disable(self) -> None:
        """Disable the differential drive and both motors.

        Stops all movement and disables both motors.

        Example:
            >>> rover.disable()
        """
        self.stop()
        self._left_motor.disable()
        self._right_motor.disable()
        self._is_enabled = False
        self._dd_state = DifferentialDriveState.DISABLED
        self._on_disable()
        logger.info("DifferentialDrive '%s' disabled", self.name)

    # -------------------------------------------------------------------------
    # Controller Abstract Methods
    # -------------------------------------------------------------------------

    def _do_home(self) -> None:
        """Implement abstract home method from Controller.

        For differential drive, 'home' means stopping both motors.
        """
        self.stop()

    def _do_stop(self) -> None:
        """Implement abstract stop method from Controller."""
        self.stop()

    # -------------------------------------------------------------------------
    # Speed Helpers
    # -------------------------------------------------------------------------

    def _clamp_speed(self, speed: float) -> float:
        """Clamp speed to valid range [-1.0, 1.0].

        Args:
            speed: Speed value to clamp

        Returns:
            Clamped speed value
        """
        return max(-1.0, min(1.0, speed))

    def _apply_deadband(self, speed: float) -> float:
        """Apply deadband to speed value.

        Speeds below the deadband threshold are treated as zero.

        Args:
            speed: Speed value

        Returns:
            Speed with deadband applied
        """
        if abs(speed) < self._dd_config.deadband:
            return 0.0
        return speed

    def _apply_inversion(self, left: float, right: float) -> tuple[float, float]:
        """Apply motor inversion based on configuration.

        Args:
            left: Left motor speed
            right: Right motor speed

        Returns:
            Tuple of (left, right) speeds with inversion applied
        """
        if self._dd_config.invert_left:
            left = -left
        if self._dd_config.invert_right:
            right = -right
        return (left, right)

    def _set_motor_speeds(self, left: float, right: float) -> None:
        """Set motor speeds with clamping, deadband, and inversion.

        Args:
            left: Left motor speed (-1.0 to 1.0)
            right: Right motor speed (-1.0 to 1.0)
        """
        # Clamp speeds
        left = self._clamp_speed(left)
        right = self._clamp_speed(right)

        # Apply deadband
        left = self._apply_deadband(left)
        right = self._apply_deadband(right)

        # Apply inversion
        left, right = self._apply_inversion(left, right)

        # Set motors
        self._left_motor.set(left)
        self._right_motor.set(right)

        # Update state
        self._current_left_speed = left
        self._current_right_speed = right

    def _calculate_arc_speeds(self, speed: float, radius: float) -> tuple[float, float]:
        """Calculate wheel speeds for arc movement.

        Uses differential drive kinematics to calculate the required
        wheel speeds to follow an arc of the given radius.

        Args:
            speed: Linear speed (-1.0 to 1.0)
            radius: Arc radius in meters (positive = turn left, negative = turn right)

        Returns:
            Tuple of (left_speed, right_speed)
        """
        track_width = self._dd_config.track_width

        # Handle straight line (infinite radius)
        if abs(radius) > 1000:  # Effectively infinite
            return (speed, speed)

        # Handle spin in place (zero radius)
        if abs(radius) < 0.001:  # Effectively zero
            return (speed, -speed) if radius >= 0 else (-speed, speed)

        # Calculate differential speeds
        # For arc motion: v_outer / v_inner = (R + W/2) / (R - W/2)
        # where R = radius, W = track_width
        half_track = track_width / 2

        if radius > 0:  # Turn left
            # Left wheel is inner, right wheel is outer
            inner_ratio = (radius - half_track) / radius
            outer_ratio = (radius + half_track) / radius
            left_speed = speed * inner_ratio
            right_speed = speed * outer_ratio
        else:  # Turn right
            # Right wheel is inner, left wheel is outer
            radius = abs(radius)
            inner_ratio = (radius - half_track) / radius
            outer_ratio = (radius + half_track) / radius
            left_speed = speed * outer_ratio
            right_speed = speed * inner_ratio

        return (left_speed, right_speed)

    # -------------------------------------------------------------------------
    # Movement Methods
    # -------------------------------------------------------------------------

    def forward(self, speed: float = 1.0) -> None:
        """Drive forward at the specified speed.

        Args:
            speed: Speed value (0.0 to 1.0, default 1.0)

        Example:
            >>> rover.forward(0.5)  # Drive forward at half speed
        """
        speed = abs(speed)  # Forward is always positive
        self._set_motor_speeds(speed, speed)
        self._dd_state = DifferentialDriveState.MOVING
        logger.debug("DifferentialDrive '%s' forward at %.2f", self.name, speed)

    def reverse(self, speed: float = 1.0) -> None:
        """Drive backward at the specified speed.

        Args:
            speed: Speed value (0.0 to 1.0, default 1.0)

        Example:
            >>> rover.reverse(0.5)  # Drive backward at half speed
        """
        speed = -abs(speed)  # Reverse is always negative
        self._set_motor_speeds(speed, speed)
        self._dd_state = DifferentialDriveState.MOVING
        logger.debug("DifferentialDrive '%s' reverse at %.2f", self.name, abs(speed))

    def turn_left(self, speed: float = 0.5) -> None:
        """Turn left while moving forward.

        Reduces left wheel speed to create a left turn.

        Args:
            speed: Turn intensity (0.0 to 1.0, default 0.5)

        Example:
            >>> rover.turn_left(0.3)  # Gentle left turn
        """
        speed = abs(speed)
        # Right wheel faster, left wheel slower
        left_speed = speed * 0.5
        right_speed = speed
        self._set_motor_speeds(left_speed, right_speed)
        self._dd_state = DifferentialDriveState.TURNING
        logger.debug("DifferentialDrive '%s' turning left at %.2f", self.name, speed)

    def turn_right(self, speed: float = 0.5) -> None:
        """Turn right while moving forward.

        Reduces right wheel speed to create a right turn.

        Args:
            speed: Turn intensity (0.0 to 1.0, default 0.5)

        Example:
            >>> rover.turn_right(0.3)  # Gentle right turn
        """
        speed = abs(speed)
        # Left wheel faster, right wheel slower
        left_speed = speed
        right_speed = speed * 0.5
        self._set_motor_speeds(left_speed, right_speed)
        self._dd_state = DifferentialDriveState.TURNING
        logger.debug("DifferentialDrive '%s' turning right at %.2f", self.name, speed)

    def spin(self, speed: float = 0.5, *, clockwise: bool = True) -> None:
        """Spin in place (rotate around center).

        Both wheels turn at equal speed in opposite directions.

        Args:
            speed: Rotation speed (0.0 to 1.0, default 0.5)
            clockwise: If True, spin clockwise; if False, counter-clockwise

        Example:
            >>> rover.spin(0.5, clockwise=True)   # Spin right
            >>> rover.spin(0.5, clockwise=False)  # Spin left
        """
        speed = abs(speed)
        if clockwise:
            # Left forward, right backward = clockwise
            self._set_motor_speeds(speed, -speed)
        else:
            # Left backward, right forward = counter-clockwise
            self._set_motor_speeds(-speed, speed)
        self._dd_state = DifferentialDriveState.SPINNING
        direction = "clockwise" if clockwise else "counter-clockwise"
        logger.debug("DifferentialDrive '%s' spinning %s at %.2f", self.name, direction, speed)

    def arc(self, speed: float, radius: float) -> None:
        """Drive in an arc of the specified radius.

        Uses differential drive kinematics to follow a curved path.

        Args:
            speed: Linear speed (-1.0 to 1.0)
            radius: Arc radius in meters (positive = turn left, negative = turn right)

        Example:
            >>> rover.arc(0.5, 1.0)   # Arc left with 1m radius
            >>> rover.arc(0.5, -0.5)  # Tight arc right with 0.5m radius
        """
        left_speed, right_speed = self._calculate_arc_speeds(speed, radius)
        self._set_motor_speeds(left_speed, right_speed)
        self._dd_state = DifferentialDriveState.MOVING
        direction = "left" if radius > 0 else "right"
        logger.debug(
            "DifferentialDrive '%s' arc %s (r=%.2fm) at %.2f",
            self.name,
            direction,
            abs(radius),
            speed,
        )

    def tank(self, left_speed: float, right_speed: float) -> None:
        """Direct tank-style control of wheel speeds.

        Allows independent control of each wheel for custom movements.

        Args:
            left_speed: Left wheel speed (-1.0 to 1.0)
            right_speed: Right wheel speed (-1.0 to 1.0)

        Example:
            >>> rover.tank(0.5, 0.5)   # Forward
            >>> rover.tank(0.5, -0.5)  # Spin right
            >>> rover.tank(0.3, 0.7)   # Arc left
        """
        self._set_motor_speeds(left_speed, right_speed)

        # Determine state based on speeds
        if left_speed == 0 and right_speed == 0:
            self._dd_state = DifferentialDriveState.IDLE
        elif left_speed == -right_speed:
            self._dd_state = DifferentialDriveState.SPINNING
        elif left_speed == right_speed:
            self._dd_state = DifferentialDriveState.MOVING
        else:
            self._dd_state = DifferentialDriveState.TURNING

        logger.debug(
            "DifferentialDrive '%s' tank: left=%.2f, right=%.2f",
            self.name,
            left_speed,
            right_speed,
        )

    def stop(self) -> None:
        """Stop both motors immediately.

        Sets both motor speeds to zero without braking.

        Example:
            >>> rover.stop()
        """
        self._left_motor.set(0.0)
        self._right_motor.set(0.0)
        self._current_left_speed = 0.0
        self._current_right_speed = 0.0
        self._dd_state = DifferentialDriveState.IDLE
        logger.debug("DifferentialDrive '%s' stopped", self.name)

    def brake(self) -> None:
        """Apply braking to both motors.

        For DC motors, this typically means shorting the motor terminals
        to create dynamic braking. The actual braking behavior depends
        on the motor driver implementation.

        Example:
            >>> rover.brake()
        """
        # Call motor brake if available, otherwise just stop
        if hasattr(self._left_motor, "brake"):
            self._left_motor.brake()
        else:
            self._left_motor.set(0.0)

        if hasattr(self._right_motor, "brake"):
            self._right_motor.brake()
        else:
            self._right_motor.set(0.0)

        self._current_left_speed = 0.0
        self._current_right_speed = 0.0
        self._dd_state = DifferentialDriveState.BRAKING
        logger.debug("DifferentialDrive '%s' braking", self.name)

    # -------------------------------------------------------------------------
    # String Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """String representation."""
        return f"DifferentialDrive(name='{self.name}', state={self._dd_state.value})"


__all__ = [
    "DifferentialDrive",
    "DifferentialDriveConfig",
    "DifferentialDriveState",
]
