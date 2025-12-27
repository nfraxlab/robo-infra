"""Pan-Tilt head controller for camera and sensor positioning.

This module provides a PanTilt controller for managing 2-DOF pan-tilt
mechanisms commonly used for camera positioning, sensor pointing, and
tracking applications. Common examples include:
- Security camera pan-tilt heads
- Video conferencing cameras
- Telescopes and astronomy mounts
- LIDAR/radar scanning heads
- Robotic vision systems

Example:
    >>> from robo_infra.controllers.pan_tilt import PanTilt, PanTiltConfig
    >>> from robo_infra.actuators.servo import Servo
    >>>
    >>> # Create servos for pan and tilt
    >>> pan_servo = Servo(name="pan_servo", angle_range=(-90, 90))
    >>> tilt_servo = Servo(name="tilt_servo", angle_range=(-45, 45))
    >>>
    >>> # Create pan-tilt controller
    >>> config = PanTiltConfig(
    ...     name="camera_head",
    ...     pan_range=(-90, 90),
    ...     tilt_range=(-45, 45),
    ... )
    >>> pt = PanTilt(
    ...     name="camera_head",
    ...     pan_actuator=pan_servo,
    ...     tilt_actuator=tilt_servo,
    ...     config=config,
    ... )
    >>> pt.enable()
    >>>
    >>> # Pan-tilt commands
    >>> pt.look_at(pan=45.0, tilt=30.0)   # Look right and up
    >>> pt.center()                        # Return to center
    >>> pt.pan_to(90.0)                    # Pan only
    >>> pt.tilt_to(-30.0)                  # Tilt only
    >>> pt.track((320, 240))               # Track pixel coordinates
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


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class PanTiltState(Enum):
    """States a PanTilt head can be in."""

    IDLE = "idle"
    MOVING = "moving"
    TRACKING = "tracking"
    SCANNING = "scanning"
    ERROR = "error"
    DISABLED = "disabled"


# =============================================================================
# Configuration Models
# =============================================================================


class PanTiltConfig(BaseModel):
    """Configuration for a PanTilt controller.

    Attributes:
        name: Human-readable name for the pan-tilt head.
        pan_range: Pan angle range in degrees (min, max).
        tilt_range: Tilt angle range in degrees (min, max).
        pan_speed: Pan movement speed in deg/s.
        tilt_speed: Tilt movement speed in deg/s.
        center_pan: Pan angle for center position.
        center_tilt: Tilt angle for center position.
        invert_pan: Invert pan direction.
        invert_tilt: Invert tilt direction.
        image_width: Image width in pixels (for tracking).
        image_height: Image height in pixels (for tracking).
        fov_horizontal: Horizontal field of view in degrees.
        fov_vertical: Vertical field of view in degrees.

    Example:
        >>> config = PanTiltConfig(
        ...     name="security_cam",
        ...     pan_range=(-180, 180),
        ...     tilt_range=(-30, 90),
        ...     fov_horizontal=60,
        ...     fov_vertical=45,
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="PanTilt name")
    description: str = Field(default="", description="Human-readable description")

    # Angle ranges
    pan_range: tuple[float, float] = Field(
        default=(-90.0, 90.0),
        description="Pan angle range (min, max) in degrees",
    )
    tilt_range: tuple[float, float] = Field(
        default=(-45.0, 45.0),
        description="Tilt angle range (min, max) in degrees",
    )

    # Center position
    center_pan: float = Field(
        default=0.0,
        description="Pan angle for center position",
    )
    center_tilt: float = Field(
        default=0.0,
        description="Tilt angle for center position",
    )

    # Speed configuration
    pan_speed: float = Field(
        default=90.0,
        gt=0,
        description="Pan movement speed in deg/s",
    )
    tilt_speed: float = Field(
        default=90.0,
        gt=0,
        description="Tilt movement speed in deg/s",
    )

    # Direction inversion
    invert_pan: bool = Field(
        default=False,
        description="Invert pan direction",
    )
    invert_tilt: bool = Field(
        default=False,
        description="Invert tilt direction",
    )

    # Camera/sensor parameters (for tracking)
    image_width: int = Field(
        default=640,
        gt=0,
        description="Image width in pixels",
    )
    image_height: int = Field(
        default=480,
        gt=0,
        description="Image height in pixels",
    )
    fov_horizontal: float = Field(
        default=60.0,
        gt=0,
        description="Horizontal field of view in degrees",
    )
    fov_vertical: float = Field(
        default=45.0,
        gt=0,
        description="Vertical field of view in degrees",
    )

    # Tracking parameters
    tracking_gain: float = Field(
        default=0.5,
        ge=0,
        le=1.0,
        description="Tracking gain (0.0 to 1.0)",
    )
    deadband: float = Field(
        default=5.0,
        ge=0,
        description="Deadband for tracking in pixels",
    )

    @property
    def pan_min(self) -> float:
        """Minimum pan angle."""
        return self.pan_range[0]

    @property
    def pan_max(self) -> float:
        """Maximum pan angle."""
        return self.pan_range[1]

    @property
    def tilt_min(self) -> float:
        """Minimum tilt angle."""
        return self.tilt_range[0]

    @property
    def tilt_max(self) -> float:
        """Maximum tilt angle."""
        return self.tilt_range[1]

    @property
    def degrees_per_pixel_h(self) -> float:
        """Degrees per pixel horizontally."""
        return self.fov_horizontal / self.image_width

    @property
    def degrees_per_pixel_v(self) -> float:
        """Degrees per pixel vertically."""
        return self.fov_vertical / self.image_height


# =============================================================================
# Status Dataclass
# =============================================================================


class PanTiltStatus:
    """Current status of a PanTilt head."""

    def __init__(
        self,
        state: PanTiltState,
        pan_angle: float,
        tilt_angle: float,
        target_pan: float | None,
        target_tilt: float | None,
        is_enabled: bool,
        error: str | None = None,
    ) -> None:
        """Initialize PanTiltStatus."""
        self.state = state
        self.pan_angle = pan_angle
        self.tilt_angle = tilt_angle
        self.target_pan = target_pan
        self.target_tilt = target_tilt
        self.is_enabled = is_enabled
        self.error = error


# =============================================================================
# PanTilt Controller
# =============================================================================


class PanTilt(Controller):
    """Controller for 2-DOF pan-tilt heads.

    PanTilt provides high-level control for camera pan-tilt mechanisms,
    with support for absolute positioning, relative moves, tracking,
    and scanning patterns.

    Features:
    - Absolute pan/tilt positioning
    - Relative pan/tilt moves
    - Center position command
    - Pixel coordinate tracking
    - Scanning patterns

    Example:
        >>> pt = PanTilt(
        ...     name="camera",
        ...     pan_actuator=pan_servo,
        ...     tilt_actuator=tilt_servo,
        ... )
        >>> pt.enable()
        >>> pt.look_at(pan=45, tilt=30)
        >>> pt.track((320, 240))  # Track object at pixel coordinates
    """

    def __init__(
        self,
        name: str,
        pan_actuator: Actuator,
        tilt_actuator: Actuator,
        *,
        config: PanTiltConfig | None = None,
    ) -> None:
        """Initialize PanTilt controller.

        Args:
            name: Controller name
            pan_actuator: Actuator for pan axis
            tilt_actuator: Actuator for tilt axis
            config: Optional configuration

        Raises:
            ValueError: If any actuator is None
        """
        if pan_actuator is None:
            raise ValueError("Pan actuator is required")
        if tilt_actuator is None:
            raise ValueError("Tilt actuator is required")

        # Create config if not provided
        if config is None:
            config = PanTiltConfig(name=name)

        # Initialize base controller
        super().__init__(name, config=ControllerConfig(name=name))

        # Store pan-tilt-specific config
        self._pt_config = config

        # Store actuators
        self._pan_actuator = pan_actuator
        self._tilt_actuator = tilt_actuator
        self.add_actuator("pan", pan_actuator)
        self.add_actuator("tilt", tilt_actuator)

        # PanTilt-specific state
        self._pt_state = PanTiltState.DISABLED
        self._pan_angle: float = config.center_pan
        self._tilt_angle: float = config.center_tilt
        self._target_pan: float | None = None
        self._target_tilt: float | None = None
        self._tracking_target: tuple[float, float] | None = None

        logger.debug(
            "PanTilt '%s' initialized with pan=%s, tilt=%s",
            name,
            pan_actuator.name,
            tilt_actuator.name,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def pan_actuator(self) -> Actuator:
        """Pan axis actuator."""
        return self._pan_actuator

    @property
    def tilt_actuator(self) -> Actuator:
        """Tilt axis actuator."""
        return self._tilt_actuator

    @property
    def pt_config(self) -> PanTiltConfig:
        """PanTilt configuration."""
        return self._pt_config

    @property
    def pt_state(self) -> PanTiltState:
        """Current pan-tilt state."""
        return self._pt_state

    @property
    def pan_angle(self) -> float:
        """Current pan angle in degrees."""
        return self._pan_angle

    @property
    def tilt_angle(self) -> float:
        """Current tilt angle in degrees."""
        return self._tilt_angle

    @property
    def pan(self) -> float:
        """Alias for pan_angle."""
        return self._pan_angle

    @property
    def tilt(self) -> float:
        """Alias for tilt_angle."""
        return self._tilt_angle

    @property
    def position(self) -> tuple[float, float]:
        """Current pan and tilt angles as tuple."""
        return (self._pan_angle, self._tilt_angle)

    # -------------------------------------------------------------------------
    # Controller Lifecycle
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the pan-tilt controller."""
        super().enable()
        self._pt_state = PanTiltState.IDLE
        self._pan_actuator.enable()
        self._tilt_actuator.enable()
        logger.info("PanTilt '%s' enabled", self.name)

    def disable(self) -> None:
        """Disable the pan-tilt controller."""
        self._pt_state = PanTiltState.DISABLED
        self._pan_actuator.disable()
        self._tilt_actuator.disable()
        super().disable()
        logger.info("PanTilt '%s' disabled", self.name)

    def home(self) -> None:
        """Move to center position."""
        self._require_enabled()
        self.center()
        self._is_homed = True
        logger.info("PanTilt '%s' homed", self.name)

    def _do_home(self) -> None:
        """Perform homing sequence for the pan-tilt.

        Centers both pan and tilt axes.
        """
        self._pan_angle = 0.0
        self._tilt_angle = 0.0
        self._target_pan = 0.0
        self._target_tilt = 0.0
        self._is_homed = True

    def _do_stop(self) -> None:
        """Perform emergency stop for the pan-tilt.

        Stops any motion and tracking.
        """
        self._pt_state = PanTiltState.IDLE
        self._is_tracking = False
        self._target_pan = self._pan_angle
        self._target_tilt = self._tilt_angle

    # -------------------------------------------------------------------------
    # PanTilt Operations
    # -------------------------------------------------------------------------

    def look_at(
        self,
        pan: float,
        tilt: float,
    ) -> None:
        """Move to absolute pan and tilt angles.

        Args:
            pan: Pan angle in degrees
            tilt: Tilt angle in degrees

        Raises:
            DisabledError: If pan-tilt is not enabled
            ValueError: If angles are outside limits
        """
        self._require_enabled()

        # Validate and clamp angles
        pan = self._validate_pan(pan)
        tilt = self._validate_tilt(tilt)

        # Set targets
        self._target_pan = pan
        self._target_tilt = tilt
        self._pt_state = PanTiltState.MOVING

        # Apply to actuators
        self._set_pan(pan)
        self._set_tilt(tilt)

        # Update current angles
        self._pan_angle = pan
        self._tilt_angle = tilt
        self._pt_state = PanTiltState.IDLE
        self._target_pan = None
        self._target_tilt = None

        logger.info(
            "PanTilt '%s' look_at pan=%.1f, tilt=%.1f",
            self.name,
            pan,
            tilt,
        )

    def center(self) -> None:
        """Move to center position.

        Raises:
            DisabledError: If pan-tilt is not enabled
        """
        self.look_at(
            self._pt_config.center_pan,
            self._pt_config.center_tilt,
        )
        logger.info("PanTilt '%s' centered", self.name)

    def pan_to(self, angle: float) -> None:
        """Move pan axis only.

        Args:
            angle: Pan angle in degrees

        Raises:
            DisabledError: If pan-tilt is not enabled
            ValueError: If angle is outside limits
        """
        self.look_at(angle, self._tilt_angle)

    def tilt_to(self, angle: float) -> None:
        """Move tilt axis only.

        Args:
            angle: Tilt angle in degrees

        Raises:
            DisabledError: If pan-tilt is not enabled
            ValueError: If angle is outside limits
        """
        self.look_at(self._pan_angle, angle)

    def pan_by(self, delta: float) -> None:
        """Move pan axis by relative amount.

        Args:
            delta: Pan angle change in degrees

        Raises:
            DisabledError: If pan-tilt is not enabled
            ValueError: If resulting angle is outside limits
        """
        self.pan_to(self._pan_angle + delta)

    def tilt_by(self, delta: float) -> None:
        """Move tilt axis by relative amount.

        Args:
            delta: Tilt angle change in degrees

        Raises:
            DisabledError: If pan-tilt is not enabled
            ValueError: If resulting angle is outside limits
        """
        self.tilt_to(self._tilt_angle + delta)

    def move_by(self, pan_delta: float, tilt_delta: float) -> None:
        """Move both axes by relative amounts.

        Args:
            pan_delta: Pan angle change in degrees
            tilt_delta: Tilt angle change in degrees

        Raises:
            DisabledError: If pan-tilt is not enabled
            ValueError: If resulting angles are outside limits
        """
        self.look_at(
            self._pan_angle + pan_delta,
            self._tilt_angle + tilt_delta,
        )

    def track(
        self,
        target: tuple[float, float],
        *,
        gain: float | None = None,
    ) -> None:
        """Track a target at pixel coordinates.

        Converts pixel coordinates to pan/tilt angles and moves
        to center the target in the frame.

        Args:
            target: Target pixel coordinates (x, y)
            gain: Tracking gain (0.0 to 1.0), uses config default if None

        Raises:
            DisabledError: If pan-tilt is not enabled
        """
        self._require_enabled()

        config = self._pt_config
        if gain is None:
            gain = config.tracking_gain

        # Calculate error from center
        center_x = config.image_width / 2
        center_y = config.image_height / 2
        error_x = target[0] - center_x
        error_y = target[1] - center_y

        # Check deadband
        if abs(error_x) < config.deadband and abs(error_y) < config.deadband:
            return

        # Convert pixel error to angle error
        pan_error = error_x * config.degrees_per_pixel_h
        tilt_error = error_y * config.degrees_per_pixel_v

        # Apply inversion
        if config.invert_pan:
            pan_error = -pan_error
        if config.invert_tilt:
            tilt_error = -tilt_error

        # Apply gain and move
        pan_delta = pan_error * gain
        tilt_delta = tilt_error * gain

        self._pt_state = PanTiltState.TRACKING
        self._tracking_target = target

        # Move incrementally
        new_pan = self._clamp_pan(self._pan_angle + pan_delta)
        new_tilt = self._clamp_tilt(self._tilt_angle + tilt_delta)
        self.look_at(new_pan, new_tilt)

        self._pt_state = PanTiltState.IDLE

        logger.debug(
            "PanTilt '%s' tracking target (%d, %d) -> pan=%.1f, tilt=%.1f",
            self.name,
            int(target[0]),
            int(target[1]),
            new_pan,
            new_tilt,
        )

    def stop(self) -> None:
        """Stop any movement.

        This is a soft stop. For emergency stop, use emergency_stop().
        """
        if self._pt_state == PanTiltState.DISABLED:
            return

        self._pt_state = PanTiltState.IDLE
        self._target_pan = None
        self._target_tilt = None
        self._tracking_target = None

        logger.info("PanTilt '%s' stopped", self.name)

    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motion."""
        self._pt_state = PanTiltState.IDLE
        self._target_pan = None
        self._target_tilt = None
        self._tracking_target = None
        super().stop()

        logger.warning("PanTilt '%s' emergency stopped", self.name)

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def status(self) -> PanTiltStatus:
        """Get current pan-tilt status."""
        return PanTiltStatus(
            state=self._pt_state,
            pan_angle=self._pan_angle,
            tilt_angle=self._tilt_angle,
            target_pan=self._target_pan,
            target_tilt=self._target_tilt,
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

        def look_at_position(pan: float, tilt: float) -> str:
            """Move the camera to look at a specific pan/tilt position.

            Args:
                pan: Pan angle in degrees (horizontal, positive = right).
                tilt: Tilt angle in degrees (vertical, positive = up).

            Returns:
                Status message confirming movement.
            """
            self.look_at(pan, tilt)
            return f"Camera moved to pan={pan}, tilt={tilt} degrees"

        def center_camera() -> str:
            """Center the camera (return to home position).

            Returns:
                Status message confirming centering.
            """
            self.center()
            return "Camera centered"

        def pan_camera(angle: float) -> str:
            """Pan the camera to a specific angle.

            Args:
                angle: Pan angle in degrees (positive = right).

            Returns:
                Status message confirming pan.
            """
            self.pan_to(angle)
            return f"Camera panned to {angle} degrees"

        def tilt_camera(angle: float) -> str:
            """Tilt the camera to a specific angle.

            Args:
                angle: Tilt angle in degrees (positive = up).

            Returns:
                Status message confirming tilt.
            """
            self.tilt_to(angle)
            return f"Camera tilted to {angle} degrees"

        def track_target(x: float, y: float) -> str:
            """Track a target at pixel coordinates.

            Args:
                x: Target X pixel coordinate.
                y: Target Y pixel coordinate.

            Returns:
                Status message with new camera position.
            """
            self.track((x, y))
            return f"Tracking target at ({x}, {y}), camera at pan={self._pan_angle:.1f}, tilt={self._tilt_angle:.1f}"

        def get_camera_status() -> dict[str, Any]:
            """Get current camera pan-tilt status.

            Returns:
                Dict with state, pan angle, and tilt angle.
            """
            s = self.status()
            return {
                "state": s.state.value,
                "pan_angle": s.pan_angle,
                "tilt_angle": s.tilt_angle,
                "is_enabled": s.is_enabled,
            }

        return [
            look_at_position,
            center_camera,
            pan_camera,
            tilt_camera,
            track_target,
            get_camera_status,
        ]

    # -------------------------------------------------------------------------
    # Private Methods
    # -------------------------------------------------------------------------

    def _require_enabled(self) -> None:
        """Raise DisabledError if pan-tilt is not enabled."""
        from robo_infra.core.exceptions import DisabledError

        if not self._is_enabled:
            raise DisabledError(f"PanTilt '{self.name}' is not enabled")

    def _validate_pan(self, angle: float) -> float:
        """Validate and return pan angle."""
        config = self._pt_config
        if angle < config.pan_min:
            raise ValueError(f"Pan {angle} below minimum {config.pan_min}")
        if angle > config.pan_max:
            raise ValueError(f"Pan {angle} above maximum {config.pan_max}")
        return angle

    def _validate_tilt(self, angle: float) -> float:
        """Validate and return tilt angle."""
        config = self._pt_config
        if angle < config.tilt_min:
            raise ValueError(f"Tilt {angle} below minimum {config.tilt_min}")
        if angle > config.tilt_max:
            raise ValueError(f"Tilt {angle} above maximum {config.tilt_max}")
        return angle

    def _clamp_pan(self, angle: float) -> float:
        """Clamp pan angle to valid range."""
        config = self._pt_config
        return max(config.pan_min, min(config.pan_max, angle))

    def _clamp_tilt(self, angle: float) -> float:
        """Clamp tilt angle to valid range."""
        config = self._pt_config
        return max(config.tilt_min, min(config.tilt_max, angle))

    def _set_pan(self, angle: float) -> None:
        """Set pan actuator to angle."""
        if self._pt_config.invert_pan:
            angle = -angle
        self._pan_actuator.set(angle)

    def _set_tilt(self, angle: float) -> None:
        """Set tilt actuator to angle."""
        if self._pt_config.invert_tilt:
            angle = -angle
        self._tilt_actuator.set(angle)


# =============================================================================
# Factory Functions
# =============================================================================


def create_pan_tilt(
    name: str = "pan_tilt",
    *,
    pan_actuator: Actuator | None = None,
    tilt_actuator: Actuator | None = None,
    config: PanTiltConfig | None = None,
    pan_range: tuple[float, float] = (-90.0, 90.0),
    tilt_range: tuple[float, float] = (-45.0, 45.0),
) -> PanTilt:
    """Create a PanTilt controller with optional configuration.

    Args:
        name: Controller name
        pan_actuator: Pan axis actuator (creates simulated if None)
        tilt_actuator: Tilt axis actuator (creates simulated if None)
        config: Full configuration (overrides other params if provided)
        pan_range: Pan angle range (min, max) in degrees
        tilt_range: Tilt angle range (min, max) in degrees

    Returns:
        Configured PanTilt controller.

    Example:
        >>> pt = create_pan_tilt(
        ...     "camera",
        ...     pan_range=(-180, 180),
        ...     tilt_range=(-30, 90),
        ... )
    """
    from robo_infra.actuators.servo import Servo

    # Create pan actuator if not provided
    if pan_actuator is None:
        pan_actuator = Servo(
            name=f"{name}_pan",
            angle_range=pan_range,
        )

    # Create tilt actuator if not provided
    if tilt_actuator is None:
        tilt_actuator = Servo(
            name=f"{name}_tilt",
            angle_range=tilt_range,
        )

    # Create config if not provided
    if config is None:
        config = PanTiltConfig(
            name=name,
            pan_range=pan_range,
            tilt_range=tilt_range,
        )

    return PanTilt(
        name=name,
        pan_actuator=pan_actuator,
        tilt_actuator=tilt_actuator,
        config=config,
    )


# =============================================================================
# Tool Generation Function
# =============================================================================


def pan_tilt_status(pan_tilt: PanTilt) -> dict[str, Any]:
    """Get pan-tilt status as a dictionary.

    Args:
        pan_tilt: PanTilt controller instance.

    Returns:
        Dictionary with pan-tilt status information.
    """
    s = pan_tilt.status()
    return {
        "name": pan_tilt.name,
        "state": s.state.value,
        "pan_angle": s.pan_angle,
        "tilt_angle": s.tilt_angle,
        "target_pan": s.target_pan,
        "target_tilt": s.target_tilt,
        "is_enabled": s.is_enabled,
        "error": s.error,
    }
