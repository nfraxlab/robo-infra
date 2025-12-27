"""Hexapod and Quadruped legged robot controllers.

This module provides controllers for multi-legged walking robots,
including hexapods (6 legs) and quadrupeds (4 legs). It includes
gait generation algorithms for various walking patterns.

Example:
    >>> from robo_infra.controllers.hexapod import (
    ...     Hexapod, HexapodConfig, Quadruped, QuadrupedConfig,
    ...     create_hexapod, create_quadruped,
    ... )
    >>>
    >>> # Create a simulated hexapod
    >>> hexapod = create_hexapod(name="spider", simulated=True)
    >>> hexapod.enable()
    >>> hexapod.stand(height=0.1)
    >>> hexapod.walk(direction=0, speed=0.5)
    >>>
    >>> # Create a simulated quadruped
    >>> quadruped = create_quadruped(name="spot", simulated=True)
    >>> quadruped.enable()
    >>> quadruped.stand(height=0.15)
    >>> quadruped.trot(vx=0.1, vy=0.0, yaw_rate=0.0)
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.controllers.leg import (
    FootPosition,
    Leg,
    LegConfig,
    LegState,
    create_leg,
)
from robo_infra.core.controller import (
    Controller,
)
from robo_infra.core.exceptions import (
    DisabledError,
    KinematicsError,
    LimitsExceededError,
)


if TYPE_CHECKING:
    from collections.abc import Callable


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class GaitType(Enum):
    """Walking gait pattern."""

    TRIPOD = "tripod"  # 3 legs moving at once (fast, stable for hexapod)
    WAVE = "wave"  # 1 leg at a time (slow, very stable)
    RIPPLE = "ripple"  # 2 legs at a time (medium speed, stable)
    TROT = "trot"  # Diagonal pairs (quadruped)
    WALK = "walk"  # 1 leg at a time (quadruped)
    CREEP = "creep"  # Very slow, 3 legs always grounded (quadruped)
    PACE = "pace"  # Same-side pairs (quadruped)
    BOUND = "bound"  # Front/rear pairs (quadruped)
    GALLOP = "gallop"  # Asymmetric bound (quadruped, fast)


class LeggedRobotState(Enum):
    """State of a legged robot."""

    DISABLED = "disabled"
    IDLE = "idle"
    STANDING = "standing"
    WALKING = "walking"
    ROTATING = "rotating"
    POSING = "posing"  # Adjusting body pose
    TRANSITIONING = "transitioning"  # Changing between states
    EMERGENCY = "emergency"
    ERROR = "error"


class BodyAxis(Enum):
    """Body rotation axis."""

    ROLL = "roll"  # Rotation around X (forward) axis
    PITCH = "pitch"  # Rotation around Y (left) axis
    YAW = "yaw"  # Rotation around Z (up) axis


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class BodyPose:
    """Body pose relative to neutral standing position.

    All angles in degrees, translations in meters.
    """

    roll: float = 0.0  # Roll angle (degrees)
    pitch: float = 0.0  # Pitch angle (degrees)
    yaw: float = 0.0  # Yaw angle (degrees)
    height: float = 0.0  # Height offset from default standing height
    x_offset: float = 0.0  # Forward/backward translation
    y_offset: float = 0.0  # Left/right translation

    def as_dict(self) -> dict[str, float]:
        """Convert to dictionary."""
        return {
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "height": self.height,
            "x_offset": self.x_offset,
            "y_offset": self.y_offset,
        }


@dataclass(slots=True)
class GaitParameters:
    """Parameters for gait generation.

    These parameters control the walking motion characteristics.
    """

    step_height: float = 0.03  # Height of foot lift during swing (meters)
    step_length: float = 0.06  # Length of each step (meters)
    cycle_time: float = 1.0  # Time for one complete gait cycle (seconds)
    duty_factor: float = 0.5  # Fraction of cycle foot is on ground
    phase_offset: float = 0.0  # Phase offset for this leg (0.0 to 1.0)

    def validate(self) -> None:
        """Validate parameters."""
        if self.step_height <= 0:
            raise ValueError("step_height must be positive")
        if self.step_length <= 0:
            raise ValueError("step_length must be positive")
        if self.cycle_time <= 0:
            raise ValueError("cycle_time must be positive")
        if not 0 < self.duty_factor < 1:
            raise ValueError("duty_factor must be between 0 and 1")
        if not 0 <= self.phase_offset < 1:
            raise ValueError("phase_offset must be between 0 and 1")


@dataclass
class LeggedRobotStatus:
    """Status of a legged robot."""

    state: LeggedRobotState
    is_enabled: bool
    is_standing: bool
    body_pose: BodyPose
    gait_type: GaitType
    gait_phase: float  # Current phase in gait cycle (0.0 to 1.0)
    velocity: tuple[float, float, float]  # (vx, vy, yaw_rate)
    leg_states: dict[str, LegState]
    error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Convert to dictionary."""
        return {
            "state": self.state.value,
            "is_enabled": self.is_enabled,
            "is_standing": self.is_standing,
            "body_pose": self.body_pose.as_dict(),
            "gait_type": self.gait_type.value,
            "gait_phase": self.gait_phase,
            "velocity": {
                "vx": self.velocity[0],
                "vy": self.velocity[1],
                "yaw_rate": self.velocity[2],
            },
            "leg_states": {name: state.value for name, state in self.leg_states.items()},
            "error": self.error,
        }


# =============================================================================
# Gait Patterns
# =============================================================================


# Hexapod gait phase offsets (when each leg starts its swing)
HEXAPOD_TRIPOD_PHASES: dict[str, float] = {
    "front_left": 0.0,
    "front_right": 0.5,
    "middle_left": 0.5,
    "middle_right": 0.0,
    "rear_left": 0.0,
    "rear_right": 0.5,
}

HEXAPOD_WAVE_PHASES: dict[str, float] = {
    "front_left": 0.0,
    "middle_left": 1 / 6,
    "rear_left": 2 / 6,
    "rear_right": 3 / 6,
    "middle_right": 4 / 6,
    "front_right": 5 / 6,
}

HEXAPOD_RIPPLE_PHASES: dict[str, float] = {
    "front_left": 0.0,
    "middle_right": 0.0,
    "rear_left": 1 / 3,
    "front_right": 1 / 3,
    "middle_left": 2 / 3,
    "rear_right": 2 / 3,
}

# Quadruped gait phase offsets
QUADRUPED_TROT_PHASES: dict[str, float] = {
    "front_left": 0.0,
    "front_right": 0.5,
    "rear_left": 0.5,
    "rear_right": 0.0,
}

QUADRUPED_WALK_PHASES: dict[str, float] = {
    "front_left": 0.0,
    "rear_right": 0.25,
    "front_right": 0.5,
    "rear_left": 0.75,
}

QUADRUPED_PACE_PHASES: dict[str, float] = {
    "front_left": 0.0,
    "rear_left": 0.0,
    "front_right": 0.5,
    "rear_right": 0.5,
}

QUADRUPED_BOUND_PHASES: dict[str, float] = {
    "front_left": 0.0,
    "front_right": 0.0,
    "rear_left": 0.5,
    "rear_right": 0.5,
}


def get_hexapod_phases(gait: GaitType) -> dict[str, float]:
    """Get phase offsets for a hexapod gait.

    Args:
        gait: Gait type

    Returns:
        Dict mapping leg names to phase offsets (0.0 to 1.0)
    """
    if gait == GaitType.TRIPOD:
        return HEXAPOD_TRIPOD_PHASES.copy()
    elif gait == GaitType.WAVE:
        return HEXAPOD_WAVE_PHASES.copy()
    elif gait == GaitType.RIPPLE:
        return HEXAPOD_RIPPLE_PHASES.copy()
    else:
        # Default to tripod
        return HEXAPOD_TRIPOD_PHASES.copy()


def get_quadruped_phases(gait: GaitType) -> dict[str, float]:
    """Get phase offsets for a quadruped gait.

    Args:
        gait: Gait type

    Returns:
        Dict mapping leg names to phase offsets (0.0 to 1.0)
    """
    if gait == GaitType.TROT:
        return QUADRUPED_TROT_PHASES.copy()
    elif gait in (GaitType.WALK, GaitType.CREEP):
        return QUADRUPED_WALK_PHASES.copy()
    elif gait == GaitType.PACE:
        return QUADRUPED_PACE_PHASES.copy()
    elif gait in (GaitType.BOUND, GaitType.GALLOP):
        return QUADRUPED_BOUND_PHASES.copy()
    else:
        # Default to trot
        return QUADRUPED_TROT_PHASES.copy()


def calculate_foot_trajectory(
    phase: float,
    duty_factor: float,
    step_length: float,
    step_height: float,
    direction: float,
) -> tuple[float, float, float]:
    """Calculate foot position for a given phase in the gait cycle.

    This generates a cycloid-like trajectory for smooth foot motion.

    Args:
        phase: Current phase in gait cycle (0.0 to 1.0)
        duty_factor: Fraction of cycle foot is on ground
        step_length: Total step length (meters)
        step_height: Maximum foot lift height (meters)
        direction: Walking direction in radians

    Returns:
        Tuple of (x_offset, y_offset, z_offset) from neutral position
    """
    # Normalize phase to 0-1
    phase = phase % 1.0

    if phase < duty_factor:
        # Stance phase - foot on ground, moving backward
        stance_progress = phase / duty_factor
        x_offset = step_length * (0.5 - stance_progress) * math.cos(direction)
        y_offset = step_length * (0.5 - stance_progress) * math.sin(direction)
        z_offset = 0.0
    else:
        # Swing phase - foot in air, moving forward
        swing_progress = (phase - duty_factor) / (1.0 - duty_factor)
        x_offset = step_length * (swing_progress - 0.5) * math.cos(direction)
        y_offset = step_length * (swing_progress - 0.5) * math.sin(direction)
        # Cycloid-like height profile for smooth lift/place
        z_offset = step_height * math.sin(swing_progress * math.pi)

    return (x_offset, y_offset, z_offset)


# =============================================================================
# Configuration
# =============================================================================


class HexapodConfig(BaseModel):
    """Configuration for a hexapod (6-legged) robot.

    Example:
        >>> config = HexapodConfig(
        ...     body_length=0.2,
        ...     body_width=0.15,
        ...     default_height=0.1,
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    # Body dimensions (meters)
    body_length: float = Field(
        default=0.2,
        gt=0,
        description="Body length (front to rear) in meters",
    )
    body_width: float = Field(
        default=0.15,
        gt=0,
        description="Body width (left to right) in meters",
    )
    body_height: float = Field(
        default=0.05,
        gt=0,
        description="Body height (thickness) in meters",
    )

    # Default stance
    default_height: float = Field(
        default=0.1,
        gt=0,
        description="Default standing height in meters",
    )
    default_foot_spread: float = Field(
        default=0.12,
        gt=0,
        description="Default horizontal distance from body to foot in meters",
    )

    # Leg configuration (shared for all legs)
    leg_config: LegConfig = Field(
        default_factory=LegConfig,
        description="Configuration for each leg",
    )

    # Gait defaults
    default_gait: GaitType = Field(
        default=GaitType.TRIPOD,
        description="Default walking gait",
    )
    default_step_height: float = Field(
        default=0.03,
        gt=0,
        description="Default step height in meters",
    )
    default_step_length: float = Field(
        default=0.06,
        gt=0,
        description="Default step length in meters",
    )
    default_cycle_time: float = Field(
        default=1.0,
        gt=0,
        description="Default gait cycle time in seconds",
    )

    # Limits
    max_speed: float = Field(
        default=0.3,
        gt=0,
        description="Maximum walking speed in m/s",
    )
    max_rotation_rate: float = Field(
        default=1.0,
        gt=0,
        description="Maximum rotation rate in rad/s",
    )
    max_body_roll: float = Field(
        default=20.0,
        ge=0,
        description="Maximum body roll in degrees",
    )
    max_body_pitch: float = Field(
        default=20.0,
        ge=0,
        description="Maximum body pitch in degrees",
    )

    # Controller behavior
    home_on_enable: bool = Field(default=False, description="Auto-home when enabled")


class QuadrupedConfig(BaseModel):
    """Configuration for a quadruped (4-legged) robot.

    Example:
        >>> config = QuadrupedConfig(
        ...     body_length=0.4,
        ...     body_width=0.2,
        ...     default_height=0.25,
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    # Body dimensions (meters)
    body_length: float = Field(
        default=0.4,
        gt=0,
        description="Body length (front to rear) in meters",
    )
    body_width: float = Field(
        default=0.2,
        gt=0,
        description="Body width (left to right) in meters",
    )
    body_height: float = Field(
        default=0.08,
        gt=0,
        description="Body height (thickness) in meters",
    )

    # Default stance
    default_height: float = Field(
        default=0.25,
        gt=0,
        description="Default standing height in meters",
    )
    default_foot_spread: float = Field(
        default=0.15,
        gt=0,
        description="Default horizontal distance from body to foot in meters",
    )

    # Leg configuration (shared for all legs)
    leg_config: LegConfig = Field(
        default_factory=lambda: LegConfig(
            coxa_length=0.05,
            femur_length=0.15,
            tibia_length=0.18,
        ),
        description="Configuration for each leg",
    )

    # Gait defaults
    default_gait: GaitType = Field(
        default=GaitType.TROT,
        description="Default walking gait",
    )
    default_step_height: float = Field(
        default=0.05,
        gt=0,
        description="Default step height in meters",
    )
    default_step_length: float = Field(
        default=0.1,
        gt=0,
        description="Default step length in meters",
    )
    default_cycle_time: float = Field(
        default=0.8,
        gt=0,
        description="Default gait cycle time in seconds",
    )

    # Limits
    max_speed: float = Field(
        default=1.0,
        gt=0,
        description="Maximum walking speed in m/s",
    )
    max_rotation_rate: float = Field(
        default=2.0,
        gt=0,
        description="Maximum rotation rate in rad/s",
    )
    max_body_roll: float = Field(
        default=30.0,
        ge=0,
        description="Maximum body roll in degrees",
    )
    max_body_pitch: float = Field(
        default=30.0,
        ge=0,
        description="Maximum body pitch in degrees",
    )

    # Controller behavior
    home_on_enable: bool = Field(default=False, description="Auto-home when enabled")


# =============================================================================
# Hexapod Controller
# =============================================================================


class Hexapod(Controller):
    """Six-legged walking robot controller.

    Controls a hexapod robot with tripod, wave, and ripple gaits.
    Provides body pose control and walking in any direction.

    Example:
        >>> from robo_infra.controllers.hexapod import Hexapod, create_hexapod
        >>>
        >>> hexapod = create_hexapod("spider", simulated=True)
        >>> hexapod.enable()
        >>> hexapod.stand(height=0.1)
        >>> hexapod.walk(direction=0, speed=0.5)
    """

    # Leg names for hexapod
    LEG_NAMES = [
        "front_left",
        "front_right",
        "middle_left",
        "middle_right",
        "rear_left",
        "rear_right",
    ]

    def __init__(
        self,
        name: str,
        legs: dict[str, Leg],
        config: HexapodConfig | None = None,
    ) -> None:
        """Initialize hexapod controller.

        Args:
            name: Name of this hexapod
            legs: Dict mapping leg names to Leg controllers
            config: Hexapod configuration
        """
        super().__init__(name=name)

        # Validate legs
        missing = set(self.LEG_NAMES) - set(legs.keys())
        if missing:
            raise ValueError(f"Missing legs: {missing}")

        self._config = config or HexapodConfig()
        self._legs = legs
        self._robot_state = LeggedRobotState.DISABLED
        self._body_pose = BodyPose()
        self._current_gait = self._config.default_gait
        self._gait_phase = 0.0
        self._gait_params = GaitParameters(
            step_height=self._config.default_step_height,
            step_length=self._config.default_step_length,
            cycle_time=self._config.default_cycle_time,
        )
        self._velocity = (0.0, 0.0, 0.0)  # (vx, vy, yaw_rate)
        self._is_standing = False
        self._walk_start_time: float | None = None

        # Neutral foot positions (when standing)
        self._neutral_positions: dict[str, FootPosition] = {}
        self._calculate_neutral_positions()

        # Add legs as actuators (for base class tracking)
        for leg_name, leg in legs.items():
            # Register leg's actuators with prefixed names
            for actuator_name, actuator in leg.actuators.items():
                self.add_actuator(f"{leg_name}_{actuator_name}", actuator)

        logger.info(f"Hexapod '{name}' initialized with {len(legs)} legs")

    def _calculate_neutral_positions(self) -> None:
        """Calculate neutral foot positions for standing pose."""
        spread = self._config.default_foot_spread
        half_length = self._config.body_length / 2
        half_width = self._config.body_width / 2

        # Calculate positions for each leg
        self._neutral_positions = {
            "front_left": FootPosition(
                x=half_length + spread * 0.7,
                y=half_width + spread * 0.7,
                z=-self._config.default_height,
            ),
            "front_right": FootPosition(
                x=half_length + spread * 0.7,
                y=-(half_width + spread * 0.7),
                z=-self._config.default_height,
            ),
            "middle_left": FootPosition(
                x=0,
                y=half_width + spread,
                z=-self._config.default_height,
            ),
            "middle_right": FootPosition(
                x=0,
                y=-(half_width + spread),
                z=-self._config.default_height,
            ),
            "rear_left": FootPosition(
                x=-(half_length + spread * 0.7),
                y=half_width + spread * 0.7,
                z=-self._config.default_height,
            ),
            "rear_right": FootPosition(
                x=-(half_length + spread * 0.7),
                y=-(half_width + spread * 0.7),
                z=-self._config.default_height,
            ),
        }

    @property
    def hexapod_config(self) -> HexapodConfig:
        """Get hexapod configuration."""
        return self._config

    @property
    def robot_state(self) -> LeggedRobotState:
        """Get current robot state."""
        return self._robot_state

    @property
    def legs(self) -> dict[str, Leg]:
        """Get all legs."""
        return self._legs

    @property
    def body_pose(self) -> BodyPose:
        """Get current body pose."""
        return self._body_pose

    @property
    def current_gait(self) -> GaitType:
        """Get current gait type."""
        return self._current_gait

    @property
    def is_walking(self) -> bool:
        """Check if robot is currently walking."""
        return self._robot_state == LeggedRobotState.WALKING

    def enable(self) -> None:
        """Enable the hexapod controller."""
        super().enable()
        for leg in self._legs.values():
            leg.enable()
        self._robot_state = LeggedRobotState.IDLE

    def disable(self) -> None:
        """Disable the hexapod controller."""
        for leg in self._legs.values():
            leg.disable()
        super().disable()
        self._robot_state = LeggedRobotState.DISABLED

    def stop(self) -> None:
        """Stop all motion immediately and return to standing position.

        This is a soft stop that halts walking/rotation but keeps the robot
        standing. For a hard emergency stop, use emergency_stop().
        """
        self._velocity = (0.0, 0.0, 0.0)
        self._walk_start_time = None

        # Return to standing position if we were standing before
        if self._is_standing:
            self._robot_state = LeggedRobotState.STANDING
        else:
            self._robot_state = LeggedRobotState.IDLE
        logger.info(f"Hexapod '{self.name}' stopped motion")

    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motion and disable.

        This performs a hard stop that disables all legs.
        """
        for leg in self._legs.values():
            leg.stop()
        super().stop()

    def _do_home(self) -> None:
        """Perform homing sequence (implements abstract method)."""
        for leg in self._legs.values():
            leg._do_home()
        self._is_standing = False
        self._robot_state = LeggedRobotState.IDLE

    def _do_stop(self) -> None:
        """Perform emergency stop (implements abstract method)."""
        self._velocity = (0.0, 0.0, 0.0)
        self._walk_start_time = None
        for leg in self._legs.values():
            leg._do_stop()
        self._robot_state = LeggedRobotState.IDLE

    def home(self) -> None:
        """Move all legs to home position."""
        if not self.is_enabled:
            raise DisabledError(f"Hexapod '{self.name}' is not enabled")

        for leg in self._legs.values():
            leg.home()

        self._is_standing = False
        self._robot_state = LeggedRobotState.IDLE
        logger.info(f"Hexapod '{self.name}' homed")

    def stand(self, height: float | None = None) -> None:
        """Stand up to specified height.

        Args:
            height: Standing height in meters (None = use default)
        """
        if not self.is_enabled:
            raise DisabledError(f"Hexapod '{self.name}' is not enabled")

        if height is None:
            height = self._config.default_height

        self._robot_state = LeggedRobotState.TRANSITIONING

        # Update neutral positions with new height
        for pos in self._neutral_positions.values():
            pos.z = -height

        # Move each leg to neutral position
        for leg_name, leg in self._legs.items():
            neutral = self._neutral_positions[leg_name]
            try:
                leg.set_foot_position(neutral.x, neutral.y, neutral.z)
                leg.set_stance()
            except (KinematicsError, LimitsExceededError) as e:
                logger.warning(f"Failed to position leg {leg_name}: {e}")

        self._body_pose.height = height
        self._is_standing = True
        self._robot_state = LeggedRobotState.STANDING
        logger.info(f"Hexapod '{self.name}' standing at height {height:.3f}m")

    def sit(self) -> None:
        """Sit down (lower body and fold legs)."""
        if not self.is_enabled:
            raise DisabledError(f"Hexapod '{self.name}' is not enabled")

        self._robot_state = LeggedRobotState.TRANSITIONING

        # Move legs to folded position
        for leg in self._legs.values():
            leg.home()

        self._is_standing = False
        self._robot_state = LeggedRobotState.IDLE
        logger.info(f"Hexapod '{self.name}' sitting")

    def set_gait(self, gait: GaitType | str) -> None:
        """Set the walking gait pattern.

        Args:
            gait: Gait type (tripod, wave, ripple)
        """
        if isinstance(gait, str):
            gait = GaitType(gait)

        if gait not in (GaitType.TRIPOD, GaitType.WAVE, GaitType.RIPPLE):
            raise ValueError(f"Invalid hexapod gait: {gait}. Use tripod, wave, or ripple.")

        self._current_gait = gait
        logger.info(f"Hexapod '{self.name}' gait set to {gait.value}")

    def walk(self, direction: float, speed: float) -> None:
        """Start walking in a direction.

        Args:
            direction: Walking direction in radians (0 = forward)
            speed: Walking speed (0.0 to 1.0, relative to max_speed)
        """
        if not self.is_enabled:
            raise DisabledError(f"Hexapod '{self.name}' is not enabled")

        if not self._is_standing:
            self.stand()

        # Clamp speed
        speed = max(0.0, min(1.0, speed))

        # Calculate velocity components
        actual_speed = speed * self._config.max_speed
        vx = actual_speed * math.cos(direction)
        vy = actual_speed * math.sin(direction)

        self._velocity = (vx, vy, 0.0)
        self._robot_state = LeggedRobotState.WALKING
        self._walk_start_time = time.time()

        logger.info(
            f"Hexapod '{self.name}' walking: direction={math.degrees(direction):.1f}°, "
            f"speed={speed:.2f}"
        )

    def rotate(self, angular_speed: float) -> None:
        """Rotate in place.

        Args:
            angular_speed: Rotation speed (-1.0 to 1.0, + = CCW)
        """
        if not self.is_enabled:
            raise DisabledError(f"Hexapod '{self.name}' is not enabled")

        if not self._is_standing:
            self.stand()

        # Clamp angular speed
        angular_speed = max(-1.0, min(1.0, angular_speed))
        actual_rate = angular_speed * self._config.max_rotation_rate

        self._velocity = (0.0, 0.0, actual_rate)
        self._robot_state = LeggedRobotState.ROTATING
        self._walk_start_time = time.time()

        logger.info(f"Hexapod '{self.name}' rotating at {math.degrees(actual_rate):.1f}°/s")

    def set_body_pose(
        self,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        height: float | None = None,
    ) -> None:
        """Set body pose.

        Args:
            roll: Roll angle in degrees (None = keep current)
            pitch: Pitch angle in degrees (None = keep current)
            yaw: Yaw angle in degrees (None = keep current)
            height: Body height in meters (None = keep current)
        """
        if not self.is_enabled:
            raise DisabledError(f"Hexapod '{self.name}' is not enabled")

        if roll is not None:
            roll = max(-self._config.max_body_roll, min(self._config.max_body_roll, roll))
            self._body_pose.roll = roll

        if pitch is not None:
            pitch = max(-self._config.max_body_pitch, min(self._config.max_body_pitch, pitch))
            self._body_pose.pitch = pitch

        if yaw is not None:
            self._body_pose.yaw = yaw

        if height is not None:
            self._body_pose.height = height

        self._robot_state = LeggedRobotState.POSING
        # In real implementation, this would adjust all leg positions
        self._robot_state = (
            LeggedRobotState.STANDING if self._is_standing else LeggedRobotState.IDLE
        )

        logger.debug(f"Hexapod '{self.name}' pose: {self._body_pose}")

    def update(self, dt: float | None = None) -> None:
        """Update gait cycle (call this in control loop).

        Args:
            dt: Time step in seconds (None = calculate from clock)
        """
        if self._robot_state not in (LeggedRobotState.WALKING, LeggedRobotState.ROTATING):
            return

        if self._walk_start_time is None:
            self._walk_start_time = time.time()

        if dt is None:
            current_time = time.time()
            elapsed = current_time - self._walk_start_time
        else:
            elapsed = dt

        # Calculate current phase in gait cycle
        self._gait_phase = (elapsed / self._gait_params.cycle_time) % 1.0

        # Get phase offsets for current gait
        phases = get_hexapod_phases(self._current_gait)

        # Calculate direction from velocity
        if self._velocity[0] != 0 or self._velocity[1] != 0:
            direction = math.atan2(self._velocity[1], self._velocity[0])
        else:
            direction = 0.0

        # Update each leg position
        for leg_name, leg in self._legs.items():
            leg_phase = (self._gait_phase + phases[leg_name]) % 1.0
            neutral = self._neutral_positions[leg_name]

            # Calculate foot offset from neutral
            x_off, y_off, z_off = calculate_foot_trajectory(
                phase=leg_phase,
                duty_factor=self._gait_params.duty_factor,
                step_length=self._gait_params.step_length,
                step_height=self._gait_params.step_height,
                direction=direction,
            )

            # Set foot position
            try:
                leg.set_foot_position(
                    neutral.x + x_off,
                    neutral.y + y_off,
                    neutral.z + z_off,
                )
                # Update ground contact based on phase
                leg.set_ground_contact(z_off == 0)
            except (KinematicsError, LimitsExceededError):
                # Skip if position unreachable
                pass

    def hexapod_status(self) -> LeggedRobotStatus:
        """Get current hexapod status.

        Returns:
            Complete status of the hexapod.
        """
        leg_states = {name: leg.leg_state for name, leg in self._legs.items()}

        return LeggedRobotStatus(
            state=self._robot_state,
            is_enabled=self.is_enabled,
            is_standing=self._is_standing,
            body_pose=self._body_pose,
            gait_type=self._current_gait,
            gait_phase=self._gait_phase,
            velocity=self._velocity,
            leg_states=leg_states,
            error=self._error,
        )

    def as_tools(self) -> list[dict[str, Any] | Callable[..., Any]]:
        """Generate AI tools for controlling the hexapod.

        Returns:
            List of callable tools for ai-infra Agent integration.
        """

        def stand_up(height: float | None = None) -> str:
            """Stand the hexapod up to a specified height.

            Args:
                height: Standing height in meters (default: 0.1m)

            Returns:
                Status message
            """
            try:
                self.stand(height)
                return f"Standing at height {self._body_pose.height:.3f}m"
            except Exception as e:
                return f"Failed: {e}"

        def sit_down() -> str:
            """Sit the hexapod down.

            Returns:
                Status message
            """
            self.sit()
            return "Sitting down"

        def walk_forward(speed: float = 0.5) -> str:
            """Walk forward at specified speed.

            Args:
                speed: Speed from 0.0 to 1.0

            Returns:
                Status message
            """
            self.walk(direction=0, speed=speed)
            return f"Walking forward at speed {speed:.2f}"

        def walk_direction(direction_degrees: float, speed: float = 0.5) -> str:
            """Walk in a specified direction.

            Args:
                direction_degrees: Direction in degrees (0=forward, 90=left)
                speed: Speed from 0.0 to 1.0

            Returns:
                Status message
            """
            self.walk(direction=math.radians(direction_degrees), speed=speed)
            return f"Walking at {direction_degrees}° at speed {speed:.2f}"

        def rotate_robot(angular_speed: float) -> str:
            """Rotate in place.

            Args:
                angular_speed: Speed from -1.0 (CW) to 1.0 (CCW)

            Returns:
                Status message
            """
            self.rotate(angular_speed)
            return f"Rotating at speed {angular_speed:.2f}"

        def stop_motion() -> str:
            """Stop all motion immediately.

            Returns:
                Status message
            """
            self.stop()
            return "Stopped"

        def set_gait_pattern(gait: str) -> str:
            """Set the walking gait pattern.

            Args:
                gait: Gait type ("tripod", "wave", "ripple")

            Returns:
                Status message
            """
            try:
                self.set_gait(gait)
                return f"Gait set to {gait}"
            except ValueError as e:
                return f"Failed: {e}"

        def get_hexapod_status() -> dict:
            """Get current hexapod status.

            Returns:
                Status dict with state, pose, gait, and leg states
            """
            return self.hexapod_status().as_dict()

        return [
            stand_up,
            sit_down,
            walk_forward,
            walk_direction,
            rotate_robot,
            stop_motion,
            set_gait_pattern,
            get_hexapod_status,
        ]


# =============================================================================
# Quadruped Controller
# =============================================================================


class Quadruped(Controller):
    """Four-legged walking robot controller (Spot-like).

    Controls a quadruped robot with trot, walk, creep, pace, and bound gaits.
    Provides body pose control and omnidirectional walking.

    Example:
        >>> from robo_infra.controllers.hexapod import Quadruped, create_quadruped
        >>>
        >>> quadruped = create_quadruped("spot", simulated=True)
        >>> quadruped.enable()
        >>> quadruped.stand(height=0.25)
        >>> quadruped.trot(vx=0.5, vy=0.0, yaw_rate=0.0)
    """

    # Leg names for quadruped
    LEG_NAMES = [
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    ]

    def __init__(
        self,
        name: str,
        legs: dict[str, Leg],
        config: QuadrupedConfig | None = None,
    ) -> None:
        """Initialize quadruped controller.

        Args:
            name: Name of this quadruped
            legs: Dict mapping leg names to Leg controllers
            config: Quadruped configuration
        """
        super().__init__(name=name)

        # Validate legs
        missing = set(self.LEG_NAMES) - set(legs.keys())
        if missing:
            raise ValueError(f"Missing legs: {missing}")

        self._config = config or QuadrupedConfig()
        self._legs = legs
        self._robot_state = LeggedRobotState.DISABLED
        self._body_pose = BodyPose()
        self._current_gait = self._config.default_gait
        self._gait_phase = 0.0
        self._gait_params = GaitParameters(
            step_height=self._config.default_step_height,
            step_length=self._config.default_step_length,
            cycle_time=self._config.default_cycle_time,
        )
        self._velocity = (0.0, 0.0, 0.0)  # (vx, vy, yaw_rate)
        self._is_standing = False
        self._walk_start_time: float | None = None

        # Neutral foot positions
        self._neutral_positions: dict[str, FootPosition] = {}
        self._calculate_neutral_positions()

        # Add legs as actuators
        for leg_name, leg in legs.items():
            for actuator_name, actuator in leg.actuators.items():
                self.add_actuator(f"{leg_name}_{actuator_name}", actuator)

        logger.info(f"Quadruped '{name}' initialized with {len(legs)} legs")

    def _calculate_neutral_positions(self) -> None:
        """Calculate neutral foot positions for standing pose."""
        spread = self._config.default_foot_spread
        half_length = self._config.body_length / 2
        half_width = self._config.body_width / 2

        self._neutral_positions = {
            "front_left": FootPosition(
                x=half_length,
                y=half_width + spread,
                z=-self._config.default_height,
            ),
            "front_right": FootPosition(
                x=half_length,
                y=-(half_width + spread),
                z=-self._config.default_height,
            ),
            "rear_left": FootPosition(
                x=-half_length,
                y=half_width + spread,
                z=-self._config.default_height,
            ),
            "rear_right": FootPosition(
                x=-half_length,
                y=-(half_width + spread),
                z=-self._config.default_height,
            ),
        }

    @property
    def quadruped_config(self) -> QuadrupedConfig:
        """Get quadruped configuration."""
        return self._config

    @property
    def robot_state(self) -> LeggedRobotState:
        """Get current robot state."""
        return self._robot_state

    @property
    def legs(self) -> dict[str, Leg]:
        """Get all legs."""
        return self._legs

    @property
    def body_pose(self) -> BodyPose:
        """Get current body pose."""
        return self._body_pose

    @property
    def current_gait(self) -> GaitType:
        """Get current gait type."""
        return self._current_gait

    @property
    def is_walking(self) -> bool:
        """Check if robot is currently walking."""
        return self._robot_state == LeggedRobotState.WALKING

    def enable(self) -> None:
        """Enable the quadruped controller."""
        super().enable()
        for leg in self._legs.values():
            leg.enable()
        self._robot_state = LeggedRobotState.IDLE

    def disable(self) -> None:
        """Disable the quadruped controller."""
        for leg in self._legs.values():
            leg.disable()
        super().disable()
        self._robot_state = LeggedRobotState.DISABLED

    def stop(self) -> None:
        """Stop all motion immediately and return to standing position.

        This is a soft stop that halts walking/rotation but keeps the robot
        standing. For a hard emergency stop, use emergency_stop().
        """
        self._velocity = (0.0, 0.0, 0.0)
        self._walk_start_time = None

        # Return to standing position if we were standing before
        if self._is_standing:
            self._robot_state = LeggedRobotState.STANDING
        else:
            self._robot_state = LeggedRobotState.IDLE
        logger.info(f"Quadruped '{self.name}' stopped motion")

    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motion and disable.

        This performs a hard stop that disables all legs.
        """
        for leg in self._legs.values():
            leg.stop()
        super().stop()

    def _do_home(self) -> None:
        """Perform homing sequence (implements abstract method)."""
        for leg in self._legs.values():
            leg._do_home()
        self._is_standing = False
        self._robot_state = LeggedRobotState.IDLE

    def _do_stop(self) -> None:
        """Perform emergency stop (implements abstract method)."""
        self._velocity = (0.0, 0.0, 0.0)
        self._walk_start_time = None
        for leg in self._legs.values():
            leg._do_stop()
        self._robot_state = LeggedRobotState.IDLE

    def home(self) -> None:
        """Move all legs to home position."""
        if not self.is_enabled:
            raise DisabledError(f"Quadruped '{self.name}' is not enabled")

        for leg in self._legs.values():
            leg.home()

        self._is_standing = False
        self._robot_state = LeggedRobotState.IDLE
        logger.info(f"Quadruped '{self.name}' homed")

    def stand(self, height: float | None = None) -> None:
        """Stand up to specified height.

        Args:
            height: Standing height in meters (None = use default)
        """
        if not self.is_enabled:
            raise DisabledError(f"Quadruped '{self.name}' is not enabled")

        if height is None:
            height = self._config.default_height

        self._robot_state = LeggedRobotState.TRANSITIONING

        # Update neutral positions with new height
        for pos in self._neutral_positions.values():
            pos.z = -height

        # Move each leg to neutral position
        for leg_name, leg in self._legs.items():
            neutral = self._neutral_positions[leg_name]
            try:
                leg.set_foot_position(neutral.x, neutral.y, neutral.z)
                leg.set_stance()
            except (KinematicsError, LimitsExceededError) as e:
                logger.warning(f"Failed to position leg {leg_name}: {e}")

        self._body_pose.height = height
        self._is_standing = True
        self._robot_state = LeggedRobotState.STANDING
        logger.info(f"Quadruped '{self.name}' standing at height {height:.3f}m")

    def sit(self) -> None:
        """Sit down (lower body and fold legs)."""
        if not self.is_enabled:
            raise DisabledError(f"Quadruped '{self.name}' is not enabled")

        self._robot_state = LeggedRobotState.TRANSITIONING

        for leg in self._legs.values():
            leg.home()

        self._is_standing = False
        self._robot_state = LeggedRobotState.IDLE
        logger.info(f"Quadruped '{self.name}' sitting")

    def set_gait(self, gait: GaitType | str) -> None:
        """Set the walking gait pattern.

        Args:
            gait: Gait type (trot, walk, creep, pace, bound, gallop)
        """
        if isinstance(gait, str):
            gait = GaitType(gait)

        valid_gaits = (
            GaitType.TROT,
            GaitType.WALK,
            GaitType.CREEP,
            GaitType.PACE,
            GaitType.BOUND,
            GaitType.GALLOP,
        )
        if gait not in valid_gaits:
            raise ValueError(
                f"Invalid quadruped gait: {gait}. Use trot, walk, creep, pace, bound, or gallop."
            )

        self._current_gait = gait
        logger.info(f"Quadruped '{self.name}' gait set to {gait.value}")

    def trot(self, vx: float, vy: float, yaw_rate: float) -> None:
        """Move with trot gait (diagonal leg pairs).

        Args:
            vx: Forward velocity (m/s, relative to max_speed)
            vy: Lateral velocity (m/s, relative to max_speed)
            yaw_rate: Rotation rate (rad/s, relative to max_rotation_rate)
        """
        self._current_gait = GaitType.TROT
        self._move(vx, vy, yaw_rate)

    def creep(self, vx: float, vy: float) -> None:
        """Move with creep gait (very slow, stable).

        Args:
            vx: Forward velocity (relative to max_speed)
            vy: Lateral velocity (relative to max_speed)
        """
        self._current_gait = GaitType.CREEP
        self._gait_params.duty_factor = 0.75  # More time on ground
        self._move(vx, vy, 0.0)

    def _move(self, vx: float, vy: float, yaw_rate: float) -> None:
        """Internal method to start movement.

        Args:
            vx: Forward velocity (relative, -1.0 to 1.0)
            vy: Lateral velocity (relative, -1.0 to 1.0)
            yaw_rate: Rotation rate (relative, -1.0 to 1.0)
        """
        if not self.is_enabled:
            raise DisabledError(f"Quadruped '{self.name}' is not enabled")

        if not self._is_standing:
            self.stand()

        # Clamp velocities
        vx = max(-1.0, min(1.0, vx))
        vy = max(-1.0, min(1.0, vy))
        yaw_rate = max(-1.0, min(1.0, yaw_rate))

        # Scale to actual values
        actual_vx = vx * self._config.max_speed
        actual_vy = vy * self._config.max_speed
        actual_yaw = yaw_rate * self._config.max_rotation_rate

        self._velocity = (actual_vx, actual_vy, actual_yaw)
        self._robot_state = LeggedRobotState.WALKING
        self._walk_start_time = time.time()

        logger.info(f"Quadruped '{self.name}' moving: vx={vx:.2f}, vy={vy:.2f}, yaw={yaw_rate:.2f}")

    def set_body_pose(
        self,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        height: float | None = None,
    ) -> None:
        """Set body pose.

        Args:
            roll: Roll angle in degrees
            pitch: Pitch angle in degrees
            yaw: Yaw angle in degrees
            height: Body height in meters
        """
        if not self.is_enabled:
            raise DisabledError(f"Quadruped '{self.name}' is not enabled")

        if roll is not None:
            roll = max(-self._config.max_body_roll, min(self._config.max_body_roll, roll))
            self._body_pose.roll = roll

        if pitch is not None:
            pitch = max(-self._config.max_body_pitch, min(self._config.max_body_pitch, pitch))
            self._body_pose.pitch = pitch

        if yaw is not None:
            self._body_pose.yaw = yaw

        if height is not None:
            self._body_pose.height = height

        self._robot_state = LeggedRobotState.POSING
        self._robot_state = (
            LeggedRobotState.STANDING if self._is_standing else LeggedRobotState.IDLE
        )

        logger.debug(f"Quadruped '{self.name}' pose: {self._body_pose}")

    def update(self, dt: float | None = None) -> None:
        """Update gait cycle (call this in control loop).

        Args:
            dt: Time step in seconds (None = calculate from clock)
        """
        if self._robot_state != LeggedRobotState.WALKING:
            return

        if self._walk_start_time is None:
            self._walk_start_time = time.time()

        if dt is None:
            current_time = time.time()
            elapsed = current_time - self._walk_start_time
        else:
            elapsed = dt

        # Calculate current phase
        self._gait_phase = (elapsed / self._gait_params.cycle_time) % 1.0

        # Get phase offsets
        phases = get_quadruped_phases(self._current_gait)

        # Calculate direction from velocity
        if self._velocity[0] != 0 or self._velocity[1] != 0:
            direction = math.atan2(self._velocity[1], self._velocity[0])
        else:
            direction = 0.0

        # Update each leg
        for leg_name, leg in self._legs.items():
            leg_phase = (self._gait_phase + phases[leg_name]) % 1.0
            neutral = self._neutral_positions[leg_name]

            x_off, y_off, z_off = calculate_foot_trajectory(
                phase=leg_phase,
                duty_factor=self._gait_params.duty_factor,
                step_length=self._gait_params.step_length,
                step_height=self._gait_params.step_height,
                direction=direction,
            )

            try:
                leg.set_foot_position(
                    neutral.x + x_off,
                    neutral.y + y_off,
                    neutral.z + z_off,
                )
                leg.set_ground_contact(z_off == 0)
            except (KinematicsError, LimitsExceededError):
                pass

    def quadruped_status(self) -> LeggedRobotStatus:
        """Get current quadruped status.

        Returns:
            Complete status of the quadruped.
        """
        leg_states = {name: leg.leg_state for name, leg in self._legs.items()}

        return LeggedRobotStatus(
            state=self._robot_state,
            is_enabled=self.is_enabled,
            is_standing=self._is_standing,
            body_pose=self._body_pose,
            gait_type=self._current_gait,
            gait_phase=self._gait_phase,
            velocity=self._velocity,
            leg_states=leg_states,
            error=self._error,
        )

    def as_tools(self) -> list[dict[str, Any] | Callable[..., Any]]:
        """Generate AI tools for controlling the quadruped.

        Returns:
            List of callable tools for ai-infra Agent integration.
        """

        def stand_up(height: float | None = None) -> str:
            """Stand the quadruped up to a specified height.

            Args:
                height: Standing height in meters (default: 0.25m)

            Returns:
                Status message
            """
            try:
                self.stand(height)
                return f"Standing at height {self._body_pose.height:.3f}m"
            except Exception as e:
                return f"Failed: {e}"

        def sit_down() -> str:
            """Sit the quadruped down.

            Returns:
                Status message
            """
            self.sit()
            return "Sitting down"

        def trot_move(vx: float = 0.5, vy: float = 0.0, yaw_rate: float = 0.0) -> str:
            """Move with trot gait.

            Args:
                vx: Forward velocity (-1.0 to 1.0)
                vy: Lateral velocity (-1.0 to 1.0)
                yaw_rate: Rotation rate (-1.0 to 1.0)

            Returns:
                Status message
            """
            self.trot(vx, vy, yaw_rate)
            return f"Trotting: vx={vx:.2f}, vy={vy:.2f}, yaw={yaw_rate:.2f}"

        def creep_move(vx: float = 0.3, vy: float = 0.0) -> str:
            """Move slowly with creep gait (very stable).

            Args:
                vx: Forward velocity (-1.0 to 1.0)
                vy: Lateral velocity (-1.0 to 1.0)

            Returns:
                Status message
            """
            self.creep(vx, vy)
            return f"Creeping: vx={vx:.2f}, vy={vy:.2f}"

        def stop_motion() -> str:
            """Stop all motion immediately.

            Returns:
                Status message
            """
            self.stop()
            return "Stopped"

        def set_gait_pattern(gait: str) -> str:
            """Set the walking gait pattern.

            Args:
                gait: Gait type ("trot", "walk", "creep", "pace", "bound", "gallop")

            Returns:
                Status message
            """
            try:
                self.set_gait(gait)
                return f"Gait set to {gait}"
            except ValueError as e:
                return f"Failed: {e}"

        def get_quadruped_status() -> dict:
            """Get current quadruped status.

            Returns:
                Status dict with state, pose, gait, and leg states
            """
            return self.quadruped_status().as_dict()

        return [
            stand_up,
            sit_down,
            trot_move,
            creep_move,
            stop_motion,
            set_gait_pattern,
            get_quadruped_status,
        ]


# =============================================================================
# Factory Functions
# =============================================================================


def create_hexapod(
    name: str,
    body_length: float = 0.2,
    body_width: float = 0.15,
    default_height: float = 0.1,
    *,
    leg_config: LegConfig | None = None,
    simulated: bool = True,
) -> Hexapod:
    """Create a hexapod controller with default legs.

    Args:
        name: Name for the hexapod
        body_length: Body length in meters
        body_width: Body width in meters
        default_height: Default standing height in meters
        leg_config: Configuration for each leg
        simulated: If True, use simulated actuators

    Returns:
        Configured Hexapod controller

    Example:
        >>> hexapod = create_hexapod("spider", simulated=True)
        >>> hexapod.enable()
        >>> hexapod.stand()
    """
    config = HexapodConfig(
        body_length=body_length,
        body_width=body_width,
        default_height=default_height,
        leg_config=leg_config or LegConfig(),
    )

    # Create legs using full leg_config to preserve joint limits
    legs = {}
    for leg_name in Hexapod.LEG_NAMES:
        legs[leg_name] = create_leg(
            name=f"{name}_{leg_name}",
            position=leg_name,
            simulated=simulated,
            config=config.leg_config,
        )

    return Hexapod(name=name, legs=legs, config=config)


def create_quadruped(
    name: str,
    body_length: float = 0.4,
    body_width: float = 0.2,
    default_height: float = 0.25,
    *,
    leg_config: LegConfig | None = None,
    simulated: bool = True,
) -> Quadruped:
    """Create a quadruped controller with default legs.

    Args:
        name: Name for the quadruped
        body_length: Body length in meters
        body_width: Body width in meters
        default_height: Default standing height in meters
        leg_config: Configuration for each leg
        simulated: If True, use simulated actuators

    Returns:
        Configured Quadruped controller

    Example:
        >>> quadruped = create_quadruped("spot", simulated=True)
        >>> quadruped.enable()
        >>> quadruped.stand()
    """
    config = QuadrupedConfig(
        body_length=body_length,
        body_width=body_width,
        default_height=default_height,
        leg_config=leg_config
        or LegConfig(
            coxa_length=0.05,
            femur_length=0.15,
            tibia_length=0.18,
        ),
    )

    # Create legs using full leg_config to preserve joint limits
    legs = {}
    for leg_name in Quadruped.LEG_NAMES:
        legs[leg_name] = create_leg(
            name=f"{name}_{leg_name}",
            position=leg_name,
            simulated=simulated,
            config=config.leg_config,
        )

    return Quadruped(name=name, legs=legs, config=config)
