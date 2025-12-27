"""Turntable controller for rotary positioning stages.

This module provides a Turntable controller for managing rotary stages,
indexing tables, and rotational positioning systems. Common examples include:
- Rotary indexing tables
- Photography turntables
- 3D scanning platforms
- CNC rotary axes
- Assembly rotary stations

Example:
    >>> from robo_infra.controllers.turntable import Turntable, TurntableConfig
    >>> from robo_infra.actuators.stepper import Stepper
    >>>
    >>> # Create stepper motor for turntable
    >>> motor = Stepper(name="turntable_motor", steps_per_rev=200)
    >>>
    >>> # Create turntable controller
    >>> config = TurntableConfig(
    ...     name="scan_platform",
    ...     gear_ratio=10.0,      # 10:1 reduction
    ...     home_offset=0.0,
    ... )
    >>> turntable = Turntable(
    ...     name="scan_platform",
    ...     motor=motor,
    ...     config=config,
    ... )
    >>> turntable.enable()
    >>>
    >>> # Turntable commands
    >>> turntable.rotate_to(90.0)         # Rotate to 90 degrees
    >>> turntable.rotate_by(45.0)         # Rotate by 45 degrees
    >>> turntable.home()                  # Return to home
    >>> turntable.continuous(rpm=10.0)    # Continuous rotation at 10 RPM
    >>> turntable.stop()
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


class TurntableState(Enum):
    """States a Turntable can be in."""

    IDLE = "idle"
    MOVING = "moving"
    HOMING = "homing"
    CONTINUOUS = "continuous"
    STOPPING = "stopping"
    ERROR = "error"
    DISABLED = "disabled"


class RotationDirection(Enum):
    """Direction of rotation."""

    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


# =============================================================================
# Configuration Models
# =============================================================================


class TurntableConfig(BaseModel):
    """Configuration for a Turntable controller.

    Attributes:
        name: Human-readable name for the turntable.
        gear_ratio: Gear ratio between motor and table (>1 = reduction).
        home_offset: Angular offset from home sensor to zero position.
        angle_min: Minimum angle limit in degrees (None for unlimited).
        angle_max: Maximum angle limit in degrees (None for unlimited).
        speed_max: Maximum rotation speed in degrees per second.
        acceleration: Angular acceleration in deg/s².
        index_positions: Predefined index positions in degrees.

    Example:
        >>> config = TurntableConfig(
        ...     name="indexer",
        ...     gear_ratio=5.0,
        ...     index_positions=[0, 90, 180, 270],  # 4-position indexer
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Turntable name")
    description: str = Field(default="", description="Human-readable description")

    # Mechanical configuration
    gear_ratio: float = Field(
        default=1.0,
        gt=0,
        description="Gear ratio (motor:table, >1 = reduction)",
    )
    home_offset: float = Field(
        default=0.0,
        description="Offset from home sensor to zero in degrees",
    )

    # Angle limits (None = unlimited/continuous)
    angle_min: float | None = Field(
        default=None,
        description="Minimum angle limit in degrees",
    )
    angle_max: float | None = Field(
        default=None,
        description="Maximum angle limit in degrees",
    )

    # Speed configuration
    speed_max: float = Field(
        default=360.0,
        gt=0,
        description="Maximum rotation speed in deg/s",
    )
    speed_default: float = Field(
        default=90.0,
        gt=0,
        description="Default rotation speed in deg/s",
    )
    rpm_max: float = Field(
        default=60.0,
        gt=0,
        description="Maximum rotation speed in RPM",
    )

    # Acceleration
    acceleration: float = Field(
        default=180.0,
        gt=0,
        description="Angular acceleration in deg/s²",
    )
    deceleration: float = Field(
        default=180.0,
        gt=0,
        description="Angular deceleration in deg/s²",
    )

    # Indexing
    index_positions: list[float] = Field(
        default_factory=list,
        description="Predefined index positions in degrees",
    )
    index_tolerance: float = Field(
        default=0.1,
        ge=0,
        description="Position tolerance for index positions in degrees",
    )

    # Homing
    home_speed: float = Field(
        default=30.0,
        gt=0,
        description="Speed during homing in deg/s",
    )
    home_direction: int = Field(
        default=-1,
        ge=-1,
        le=1,
        description="Direction to search for home (-1 or 1)",
    )

    @property
    def is_limited(self) -> bool:
        """Check if turntable has angle limits."""
        return self.angle_min is not None or self.angle_max is not None


# =============================================================================
# Status Dataclass
# =============================================================================


class TurntableStatus:
    """Current status of a Turntable."""

    def __init__(
        self,
        state: TurntableState,
        current_angle: float,
        target_angle: float | None,
        current_speed: float,
        direction: RotationDirection,
        is_enabled: bool,
        is_homed: bool,
        error: str | None = None,
    ) -> None:
        """Initialize TurntableStatus."""
        self.state = state
        self.current_angle = current_angle
        self.target_angle = target_angle
        self.current_speed = current_speed
        self.direction = direction
        self.is_enabled = is_enabled
        self.is_homed = is_homed
        self.error = error


# =============================================================================
# Turntable Controller
# =============================================================================


class Turntable(Controller):
    """Controller for rotary positioning stages.

    Turntable provides high-level control for rotational positioning,
    with support for absolute positioning, relative moves, and continuous rotation.

    Features:
    - Absolute and relative positioning
    - Continuous rotation at fixed RPM
    - Predefined index positions
    - Homing with home sensor
    - Angle limits (optional)

    Example:
        >>> turntable = Turntable(
        ...     name="scanner",
        ...     motor=stepper,
        ...     config=TurntableConfig(name="scanner"),
        ... )
        >>> turntable.enable()
        >>> turntable.home()
        >>> turntable.rotate_to(90.0)
    """

    def __init__(
        self,
        name: str,
        motor: Actuator,
        *,
        config: TurntableConfig | None = None,
        home_sensor: Sensor | None = None,
        encoder: Sensor | None = None,
    ) -> None:
        """Initialize Turntable controller.

        Args:
            name: Controller name
            motor: Motor actuator for rotation
            config: Optional configuration
            home_sensor: Optional home position sensor
            encoder: Optional encoder for position feedback

        Raises:
            ValueError: If motor is None
        """
        if motor is None:
            raise ValueError("Turntable motor is required")

        # Create config if not provided
        if config is None:
            config = TurntableConfig(name=name)

        # Initialize base controller
        super().__init__(name, config=ControllerConfig(name=name))

        # Store turntable-specific config
        self._turntable_config = config

        # Store motor
        self._motor = motor
        self.add_actuator("motor", motor)

        # Store sensors (optional)
        self._home_sensor = home_sensor
        if home_sensor is not None:
            self.add_sensor("home", home_sensor)

        self._encoder = encoder
        if encoder is not None:
            self.add_sensor("encoder", encoder)

        # Turntable-specific state
        self._turntable_state = TurntableState.DISABLED
        self._current_angle: float = 0.0
        self._target_angle: float | None = None
        self._current_speed: float = 0.0
        self._direction = RotationDirection.CLOCKWISE
        self._is_continuous: bool = False

        logger.debug(
            "Turntable '%s' initialized with motor=%s, home_sensor=%s, encoder=%s",
            name,
            motor.name,
            home_sensor.name if home_sensor else None,
            encoder.name if encoder else None,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def motor(self) -> Actuator:
        """Turntable motor actuator."""
        return self._motor

    @property
    def home_sensor(self) -> Sensor | None:
        """Optional home position sensor."""
        return self._home_sensor

    @property
    def encoder(self) -> Sensor | None:
        """Optional encoder sensor."""
        return self._encoder

    @property
    def turntable_config(self) -> TurntableConfig:
        """Turntable configuration."""
        return self._turntable_config

    @property
    def turntable_state(self) -> TurntableState:
        """Current turntable state."""
        return self._turntable_state

    @property
    def current_angle(self) -> float:
        """Current angle in degrees."""
        return self._current_angle

    @property
    def target_angle(self) -> float | None:
        """Target angle in degrees (None if not moving to target)."""
        return self._target_angle

    @property
    def current_speed(self) -> float:
        """Current rotation speed in deg/s."""
        return self._current_speed

    @property
    def direction(self) -> RotationDirection:
        """Current rotation direction."""
        return self._direction

    @property
    def angle(self) -> float:
        """Alias for current_angle."""
        return self._current_angle

    # -------------------------------------------------------------------------
    # Controller Lifecycle
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the turntable controller."""
        super().enable()
        self._turntable_state = TurntableState.IDLE
        self._motor.enable()
        logger.info("Turntable '%s' enabled", self.name)

    def disable(self) -> None:
        """Disable the turntable controller."""
        self._stop_motor()
        self._turntable_state = TurntableState.DISABLED
        self._motor.disable()
        super().disable()
        logger.info("Turntable '%s' disabled", self.name)

    def home(self) -> None:
        """Home the turntable to zero position.

        If a home sensor is present, searches for it.
        Otherwise, assumes current position is home.

        Raises:
            DisabledError: If turntable is not enabled
        """
        self._require_enabled()

        self._turntable_state = TurntableState.HOMING

        if self._home_sensor is not None:
            # Search for home sensor
            logger.info("Turntable '%s' searching for home...", self.name)
            # In real implementation, would move until sensor triggers
            # For now, simulate immediate home
            self._current_angle = self._turntable_config.home_offset
        else:
            # No sensor - assume current position is home
            self._current_angle = 0.0

        self._turntable_state = TurntableState.IDLE
        self._is_homed = True
        logger.info("Turntable '%s' homed to %.2f degrees", self.name, self._current_angle)

    def _do_home(self) -> None:
        """Perform homing sequence for the turntable.

        Resets position to home offset or zero.
        """
        if self._home_sensor is not None:
            self._current_angle = self._turntable_config.home_offset
        else:
            self._current_angle = 0.0
        self._is_homed = True

    def _do_stop(self) -> None:
        """Perform emergency stop for the turntable.

        Immediately stops the motor and resets state.
        """
        self._stop_motor()
        self._turntable_state = TurntableState.IDLE
        self._is_continuous = False
        self._target_angle = self._current_angle

    # -------------------------------------------------------------------------
    # Turntable Operations
    # -------------------------------------------------------------------------

    def rotate_to(
        self,
        angle: float,
        *,
        speed: float | None = None,
        shortest_path: bool = True,
    ) -> None:
        """Rotate to an absolute angle.

        Args:
            angle: Target angle in degrees
            speed: Rotation speed in deg/s (uses default if None)
            shortest_path: Take shortest rotation path (for unlimited turntables)

        Raises:
            DisabledError: If turntable is not enabled
            ValueError: If angle is outside limits
        """
        self._require_enabled()

        # Validate angle against limits
        config = self._turntable_config
        if config.angle_min is not None and angle < config.angle_min:
            raise ValueError(f"Angle {angle} below minimum {config.angle_min}")
        if config.angle_max is not None and angle > config.angle_max:
            raise ValueError(f"Angle {angle} above maximum {config.angle_max}")

        # Use default speed if not specified
        if speed is None:
            speed = config.speed_default

        # Validate speed
        speed = abs(speed)
        if speed > config.speed_max:
            raise ValueError(f"Speed {speed} exceeds maximum {config.speed_max}")

        # Calculate delta
        delta = angle - self._current_angle

        # Take shortest path for unlimited turntables
        if shortest_path and not config.is_limited:
            delta = self._normalize_angle(delta)

        # Set direction
        if delta >= 0:
            self._direction = RotationDirection.CLOCKWISE
        else:
            self._direction = RotationDirection.COUNTERCLOCKWISE

        # Start movement
        self._target_angle = angle
        self._current_speed = speed
        self._turntable_state = TurntableState.MOVING
        self._start_rotation(speed, self._direction)

        # Simulate movement completion
        self._simulate_rotation(angle)

        logger.info(
            "Turntable '%s' rotated to %.2f degrees",
            self.name,
            angle,
        )

    def rotate_by(
        self,
        delta: float,
        *,
        speed: float | None = None,
    ) -> None:
        """Rotate by a relative angle.

        Args:
            delta: Angle to rotate in degrees (positive = CW, negative = CCW)
            speed: Rotation speed in deg/s (uses default if None)

        Raises:
            DisabledError: If turntable is not enabled
            ValueError: If target angle is outside limits
        """
        target = self._current_angle + delta
        self.rotate_to(target, speed=speed, shortest_path=False)

        logger.info(
            "Turntable '%s' rotated by %.2f degrees",
            self.name,
            delta,
        )

    def continuous(
        self,
        rpm: float,
    ) -> None:
        """Start continuous rotation at specified RPM.

        Args:
            rpm: Rotation speed in revolutions per minute (negative = CCW)

        Raises:
            DisabledError: If turntable is not enabled
            ValueError: If turntable has angle limits
            ValueError: If RPM exceeds maximum
        """
        self._require_enabled()

        config = self._turntable_config

        # Cannot do continuous rotation with limits
        if config.is_limited:
            raise ValueError("Continuous rotation not allowed with angle limits")

        # Validate RPM
        if abs(rpm) > config.rpm_max:
            raise ValueError(f"RPM {abs(rpm)} exceeds maximum {config.rpm_max}")

        # Calculate speed in deg/s
        speed = abs(rpm) * 360.0 / 60.0  # Convert RPM to deg/s

        # Set direction
        if rpm >= 0:
            self._direction = RotationDirection.CLOCKWISE
        else:
            self._direction = RotationDirection.COUNTERCLOCKWISE

        # Start continuous rotation
        self._target_angle = None
        self._current_speed = speed
        self._turntable_state = TurntableState.CONTINUOUS
        self._is_continuous = True
        self._start_rotation(speed, self._direction)

        logger.info(
            "Turntable '%s' continuous rotation at %.1f RPM %s",
            self.name,
            abs(rpm),
            "CW" if rpm >= 0 else "CCW",
        )

    def stop(self) -> None:
        """Stop the turntable (controlled deceleration).

        This performs a controlled stop. For emergency stop, use emergency_stop().
        """
        if self._turntable_state == TurntableState.DISABLED:
            return

        self._turntable_state = TurntableState.STOPPING
        self._stop_motor()
        self._turntable_state = TurntableState.IDLE
        self._target_angle = None
        self._is_continuous = False

        logger.info("Turntable '%s' stopped at %.2f degrees", self.name, self._current_angle)

    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt rotation."""
        self._stop_motor()
        self._turntable_state = TurntableState.IDLE
        self._target_angle = None
        self._is_continuous = False
        super().stop()

        logger.warning("Turntable '%s' emergency stopped", self.name)

    def go_to_index(
        self,
        index: int,
        *,
        speed: float | None = None,
    ) -> None:
        """Move to a predefined index position.

        Args:
            index: Index number (0-based)
            speed: Rotation speed in deg/s (uses default if None)

        Raises:
            DisabledError: If turntable is not enabled
            ValueError: If index is out of range
        """
        self._require_enabled()

        positions = self._turntable_config.index_positions
        if not positions:
            raise ValueError("No index positions defined")

        if index < 0 or index >= len(positions):
            raise ValueError(f"Index {index} out of range (0 to {len(positions) - 1})")

        target = positions[index]
        self.rotate_to(target, speed=speed)

        logger.info(
            "Turntable '%s' moved to index %d (%.2f degrees)",
            self.name,
            index,
            target,
        )

    def next_index(self, *, speed: float | None = None) -> None:
        """Move to the next index position.

        Args:
            speed: Rotation speed in deg/s (uses default if None)

        Raises:
            DisabledError: If turntable is not enabled
            ValueError: If no index positions defined
        """
        self._require_enabled()

        positions = self._turntable_config.index_positions
        if not positions:
            raise ValueError("No index positions defined")

        # Find next position
        tolerance = self._turntable_config.index_tolerance
        current = self._current_angle

        for i, pos in enumerate(positions):
            if pos > current + tolerance:
                self.go_to_index(i, speed=speed)
                return

        # Wrap around to first position
        self.go_to_index(0, speed=speed)

    def prev_index(self, *, speed: float | None = None) -> None:
        """Move to the previous index position.

        Args:
            speed: Rotation speed in deg/s (uses default if None)

        Raises:
            DisabledError: If turntable is not enabled
            ValueError: If no index positions defined
        """
        self._require_enabled()

        positions = self._turntable_config.index_positions
        if not positions:
            raise ValueError("No index positions defined")

        # Find previous position
        tolerance = self._turntable_config.index_tolerance
        current = self._current_angle

        for i in range(len(positions) - 1, -1, -1):
            if positions[i] < current - tolerance:
                self.go_to_index(i, speed=speed)
                return

        # Wrap around to last position
        self.go_to_index(len(positions) - 1, speed=speed)

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def status(self) -> TurntableStatus:
        """Get current turntable status."""
        return TurntableStatus(
            state=self._turntable_state,
            current_angle=self._current_angle,
            target_angle=self._target_angle,
            current_speed=self._current_speed,
            direction=self._direction,
            is_enabled=self._is_enabled,
            is_homed=self._is_homed,
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

        def rotate_turntable_to(angle: float, speed: float | None = None) -> str:
            """Rotate turntable to an absolute angle.

            Args:
                angle: Target angle in degrees.
                speed: Optional rotation speed in degrees per second.

            Returns:
                Status message confirming rotation.
            """
            self.rotate_to(angle, speed=speed)
            return f"Turntable rotated to {angle} degrees"

        def rotate_turntable_by(delta: float) -> str:
            """Rotate turntable by a relative angle.

            Args:
                delta: Angle to rotate in degrees (positive = CW, negative = CCW).

            Returns:
                Status message with new position.
            """
            self.rotate_by(delta)
            return f"Turntable rotated by {delta} degrees to {self._current_angle} degrees"

        def home_turntable() -> str:
            """Home the turntable to zero position.

            Returns:
                Status message confirming homing.
            """
            self.home()
            return f"Turntable homed to {self._current_angle} degrees"

        def continuous_rotation(rpm: float) -> str:
            """Start continuous rotation at specified RPM.

            Args:
                rpm: Speed in revolutions per minute (negative for CCW).

            Returns:
                Status message confirming continuous rotation.
            """
            self.continuous(rpm)
            direction = "clockwise" if rpm >= 0 else "counter-clockwise"
            return f"Turntable rotating continuously at {abs(rpm)} RPM {direction}"

        def stop_turntable() -> str:
            """Stop the turntable.

            Returns:
                Status message with stopped position.
            """
            self.stop()
            return f"Turntable stopped at {self._current_angle} degrees"

        def get_turntable_status() -> dict[str, Any]:
            """Get current turntable status.

            Returns:
                Dict with state, angle, speed, and direction.
            """
            s = self.status()
            return {
                "state": s.state.value,
                "current_angle": s.current_angle,
                "target_angle": s.target_angle,
                "current_speed": s.current_speed,
                "direction": s.direction.name,
                "is_enabled": s.is_enabled,
                "is_homed": s.is_homed,
            }

        return [
            rotate_turntable_to,
            rotate_turntable_by,
            home_turntable,
            continuous_rotation,
            stop_turntable,
            get_turntable_status,
        ]

    # -------------------------------------------------------------------------
    # Private Methods
    # -------------------------------------------------------------------------

    def _require_enabled(self) -> None:
        """Raise DisabledError if turntable is not enabled."""
        from robo_infra.core.exceptions import DisabledError

        if not self._is_enabled:
            raise DisabledError(f"Turntable '{self.name}' is not enabled")

    def _start_rotation(self, speed: float, direction: RotationDirection) -> None:
        """Start rotation at specified speed and direction."""
        # Convert speed to motor value (-1.0 to 1.0)
        motor_value = (speed / self._turntable_config.speed_max) * direction.value
        motor_value = max(-1.0, min(1.0, motor_value))

        self._motor.set(motor_value)

    def _stop_motor(self) -> None:
        """Stop the motor."""
        self._motor.set(0.0)
        self._current_speed = 0.0

    def _simulate_rotation(self, target_angle: float) -> None:
        """Simulate rotation completion.

        In a real system, this would be done by encoder feedback.
        """
        self._current_angle = target_angle
        self._target_angle = None
        self._turntable_state = TurntableState.IDLE
        self._stop_motor()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] range for shortest path calculation."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle


# =============================================================================
# Factory Functions
# =============================================================================


def create_turntable(
    name: str = "turntable",
    *,
    motor: Actuator | None = None,
    home_sensor: Sensor | None = None,
    encoder: Sensor | None = None,
    config: TurntableConfig | None = None,
    gear_ratio: float = 1.0,
    speed_max: float = 360.0,
    index_positions: list[float] | None = None,
) -> Turntable:
    """Create a Turntable controller with optional configuration.

    Args:
        name: Controller name
        motor: Motor actuator (creates simulated if None)
        home_sensor: Optional home position sensor
        encoder: Optional encoder sensor
        config: Full configuration (overrides other params if provided)
        gear_ratio: Gear ratio between motor and table
        speed_max: Maximum rotation speed in deg/s
        index_positions: Predefined index positions in degrees

    Returns:
        Configured Turntable controller.

    Example:
        >>> turntable = create_turntable(
        ...     "indexer",
        ...     gear_ratio=10.0,
        ...     index_positions=[0, 90, 180, 270],
        ... )
    """
    from robo_infra.actuators.stepper import Stepper

    # Create motor if not provided
    if motor is None:
        motor = Stepper(
            step_pin=0,
            dir_pin=1,
            name=f"{name}_motor",
        )

    # Create config if not provided
    if config is None:
        config = TurntableConfig(
            name=name,
            gear_ratio=gear_ratio,
            speed_max=speed_max,
            index_positions=index_positions or [],
        )

    return Turntable(
        name=name,
        motor=motor,
        config=config,
        home_sensor=home_sensor,
        encoder=encoder,
    )


# =============================================================================
# Tool Generation Function
# =============================================================================


def turntable_status(turntable: Turntable) -> dict[str, Any]:
    """Get turntable status as a dictionary.

    Args:
        turntable: Turntable controller instance.

    Returns:
        Dictionary with turntable status information.
    """
    s = turntable.status()
    return {
        "name": turntable.name,
        "state": s.state.value,
        "current_angle": s.current_angle,
        "target_angle": s.target_angle,
        "current_speed": s.current_speed,
        "direction": s.direction.name,
        "is_enabled": s.is_enabled,
        "is_homed": s.is_homed,
        "error": s.error,
    }
