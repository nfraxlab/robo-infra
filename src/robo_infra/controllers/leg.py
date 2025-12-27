"""Single robot leg controller with 3-DOF inverse kinematics.

This module provides a controller for a single robot leg, typically
used in legged robots (hexapods, quadrupeds, bipeds). The leg is
modeled as a 3-DOF serial manipulator with coxa (hip), femur (thigh),
and tibia (shin) joints.

Example:
    >>> from robo_infra.controllers.leg import Leg, LegConfig, create_leg
    >>> from robo_infra.actuators import Servo
    >>>
    >>> # Create a simulated leg
    >>> leg = create_leg(
    ...     name="front_left",
    ...     coxa_length=0.03,
    ...     femur_length=0.08,
    ...     tibia_length=0.12,
    ...     simulated=True,
    ... )
    >>>
    >>> # Enable and home
    >>> leg.enable()
    >>> leg.home()
    >>>
    >>> # Move foot to position
    >>> leg.set_foot_position(0.05, 0.0, -0.15)
    >>> print(leg.get_foot_position())
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.controller import (
    Controller,
    MotionConfig,
)
from robo_infra.core.exceptions import (
    DisabledError,
    KinematicsError,
    LimitsExceededError,
)
from robo_infra.core.types import Limits


if TYPE_CHECKING:
    from collections.abc import Callable

    from robo_infra.core.actuator import Actuator


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class LegState(Enum):
    """Leg operational state."""

    DISABLED = "disabled"
    IDLE = "idle"
    HOMING = "homing"
    MOVING = "moving"
    STANCE = "stance"  # Foot on ground, supporting weight
    SWING = "swing"  # Foot in air, moving forward
    ERROR = "error"


class LegPosition(Enum):
    """Position of leg on robot body."""

    FRONT_LEFT = "front_left"
    FRONT_RIGHT = "front_right"
    MIDDLE_LEFT = "middle_left"
    MIDDLE_RIGHT = "middle_right"
    REAR_LEFT = "rear_left"
    REAR_RIGHT = "rear_right"
    # For quadrupeds
    LEFT_FRONT = "left_front"  # Alias for FRONT_LEFT
    RIGHT_FRONT = "right_front"  # Alias for FRONT_RIGHT
    LEFT_REAR = "left_rear"  # Alias for REAR_LEFT
    RIGHT_REAR = "right_rear"  # Alias for REAR_RIGHT


class JointType(Enum):
    """Type of leg joint."""

    COXA = "coxa"  # Hip rotation (horizontal plane)
    FEMUR = "femur"  # Thigh rotation (vertical plane)
    TIBIA = "tibia"  # Shin rotation (vertical plane)


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class FootPosition:
    """3D position of the foot in leg coordinate frame.

    The coordinate frame:
    - X: Forward (positive = away from body)
    - Y: Left (positive = left of body center)
    - Z: Up (positive = up, negative = down toward ground)

    Origin is at the coxa (hip) joint.
    """

    x: float = 0.0  # Forward distance from hip
    y: float = 0.0  # Lateral distance from hip
    z: float = 0.0  # Vertical distance from hip (usually negative)

    def __iter__(self):
        """Allow tuple unpacking."""
        yield self.x
        yield self.y
        yield self.z

    def as_tuple(self) -> tuple[float, float, float]:
        """Convert to tuple."""
        return (self.x, self.y, self.z)

    def distance_to(self, other: FootPosition) -> float:
        """Calculate Euclidean distance to another position."""
        return math.sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )


@dataclass(slots=True)
class JointAngles:
    """Joint angles for a 3-DOF leg.

    All angles in degrees.
    """

    coxa: float = 0.0  # Hip joint angle
    femur: float = 0.0  # Thigh joint angle
    tibia: float = 0.0  # Shin joint angle

    def __iter__(self):
        """Allow tuple unpacking."""
        yield self.coxa
        yield self.femur
        yield self.tibia

    def as_tuple(self) -> tuple[float, float, float]:
        """Convert to tuple."""
        return (self.coxa, self.femur, self.tibia)

    def as_dict(self) -> dict[str, float]:
        """Convert to dict."""
        return {"coxa": self.coxa, "femur": self.femur, "tibia": self.tibia}


@dataclass(slots=True)
class LegDimensions:
    """Physical dimensions of the leg segments.

    All lengths in meters.
    """

    coxa_length: float  # Hip to femur joint
    femur_length: float  # Femur joint to tibia joint
    tibia_length: float  # Tibia joint to foot

    @property
    def total_length(self) -> float:
        """Total reach of the leg when fully extended."""
        return self.coxa_length + self.femur_length + self.tibia_length

    @property
    def min_reach(self) -> float:
        """Minimum reach (leg fully folded)."""
        return abs(self.femur_length - self.tibia_length)

    @property
    def max_reach(self) -> float:
        """Maximum reach in XZ plane (excluding coxa)."""
        return self.femur_length + self.tibia_length


@dataclass
class LegStatus:
    """Current status of the leg controller."""

    state: LegState
    is_enabled: bool
    is_homed: bool
    foot_position: FootPosition
    joint_angles: JointAngles
    ground_contact: bool = False
    load: float = 0.0  # Load on foot (0.0 = no load)
    error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Convert to dictionary."""
        return {
            "state": self.state.value,
            "is_enabled": self.is_enabled,
            "is_homed": self.is_homed,
            "foot_position": {
                "x": self.foot_position.x,
                "y": self.foot_position.y,
                "z": self.foot_position.z,
            },
            "joint_angles": {
                "coxa": self.joint_angles.coxa,
                "femur": self.joint_angles.femur,
                "tibia": self.joint_angles.tibia,
            },
            "ground_contact": self.ground_contact,
            "load": self.load,
            "error": self.error,
        }


# =============================================================================
# Configuration
# =============================================================================


class LegConfig(BaseModel):
    """Configuration for a 3-DOF robot leg.

    Example:
        >>> config = LegConfig(
        ...     coxa_length=0.03,
        ...     femur_length=0.08,
        ...     tibia_length=0.12,
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    # Segment lengths (meters)
    coxa_length: float = Field(
        default=0.03,
        gt=0,
        description="Length from hip to femur joint (meters)",
    )
    femur_length: float = Field(
        default=0.08,
        gt=0,
        description="Length from femur to tibia joint (meters)",
    )
    tibia_length: float = Field(
        default=0.12,
        gt=0,
        description="Length from tibia joint to foot (meters)",
    )

    # Joint limits (degrees)
    coxa_min: float = Field(default=-45.0, description="Minimum coxa angle (degrees)")
    coxa_max: float = Field(default=45.0, description="Maximum coxa angle (degrees)")
    coxa_home: float = Field(default=0.0, description="Home position for coxa (degrees)")

    femur_min: float = Field(default=-90.0, description="Minimum femur angle (degrees)")
    femur_max: float = Field(default=90.0, description="Maximum femur angle (degrees)")
    femur_home: float = Field(default=0.0, description="Home position for femur (degrees)")

    tibia_min: float = Field(default=-135.0, description="Minimum tibia angle (degrees)")
    tibia_max: float = Field(default=0.0, description="Maximum tibia angle (degrees)")
    tibia_home: float = Field(default=-90.0, description="Home position for tibia (degrees)")

    # Mounting
    mount_angle: float = Field(
        default=0.0,
        description="Angle of leg mount relative to body center (degrees)",
    )
    mount_offset_x: float = Field(
        default=0.0,
        description="X offset of leg mount from body center (meters)",
    )
    mount_offset_y: float = Field(
        default=0.0,
        description="Y offset of leg mount from body center (meters)",
    )

    # Inversion flags (for mirroring left/right legs)
    invert_coxa: bool = Field(default=False, description="Invert coxa direction")
    invert_femur: bool = Field(default=False, description="Invert femur direction")
    invert_tibia: bool = Field(default=False, description="Invert tibia direction")

    # Controller behavior
    home_on_enable: bool = Field(default=False, description="Auto-home when enabled")

    @property
    def dimensions(self) -> LegDimensions:
        """Get leg dimensions as a LegDimensions object."""
        return LegDimensions(
            coxa_length=self.coxa_length,
            femur_length=self.femur_length,
            tibia_length=self.tibia_length,
        )

    @property
    def coxa_limits(self) -> Limits:
        """Get coxa joint limits."""
        return Limits(min=self.coxa_min, max=self.coxa_max, default=self.coxa_home)

    @property
    def femur_limits(self) -> Limits:
        """Get femur joint limits."""
        return Limits(min=self.femur_min, max=self.femur_max, default=self.femur_home)

    @property
    def tibia_limits(self) -> Limits:
        """Get tibia joint limits."""
        return Limits(min=self.tibia_min, max=self.tibia_max, default=self.tibia_home)


# =============================================================================
# Inverse Kinematics
# =============================================================================


def inverse_kinematics_3dof(
    x: float,
    y: float,
    z: float,
    coxa_length: float,
    femur_length: float,
    tibia_length: float,
) -> tuple[float, float, float]:
    """Calculate joint angles for a 3-DOF leg to reach a foot position.

    This implements the analytical inverse kinematics solution for a
    3-DOF leg with:
    - Coxa: Rotates in horizontal plane (around Z axis)
    - Femur: Rotates in vertical plane (shoulder joint)
    - Tibia: Rotates in vertical plane (elbow joint)

    Args:
        x: Forward distance from hip (meters)
        y: Lateral distance from hip (meters)
        z: Vertical distance from hip (meters, usually negative)
        coxa_length: Length of coxa segment (meters)
        femur_length: Length of femur segment (meters)
        tibia_length: Length of tibia segment (meters)

    Returns:
        Tuple of (coxa_angle, femur_angle, tibia_angle) in degrees.

    Raises:
        KinematicsError: If position is unreachable.

    Example:
        >>> coxa, femur, tibia = inverse_kinematics_3dof(
        ...     x=0.05, y=0.0, z=-0.15,
        ...     coxa_length=0.03, femur_length=0.08, tibia_length=0.12,
        ... )
    """
    # Calculate coxa angle (rotation in horizontal plane)
    coxa_angle = math.degrees(math.atan2(y, x))

    # Project into vertical plane after coxa rotation
    # The horizontal distance from coxa joint to foot
    horizontal_dist = math.sqrt(x * x + y * y)

    # Distance from femur joint to foot in vertical plane
    # Subtract coxa_length since femur starts after coxa
    x_prime = horizontal_dist - coxa_length
    z_prime = z  # z stays the same

    # Distance from femur joint to foot
    distance = math.sqrt(x_prime * x_prime + z_prime * z_prime)

    # Check if position is reachable
    max_reach = femur_length + tibia_length
    min_reach = abs(femur_length - tibia_length)

    if distance > max_reach:
        raise KinematicsError(
            f"Position ({x:.3f}, {y:.3f}, {z:.3f}) is too far. "
            f"Max reach: {max_reach:.3f}m, requested: {distance:.3f}m"
        )

    if distance < min_reach:
        raise KinematicsError(
            f"Position ({x:.3f}, {y:.3f}, {z:.3f}) is too close. "
            f"Min reach: {min_reach:.3f}m, requested: {distance:.3f}m"
        )

    # Law of cosines to find tibia angle
    # c² = a² + b² - 2ab*cos(C)
    # where a = femur_length, b = tibia_length, c = distance
    cos_tibia = (
        femur_length * femur_length + tibia_length * tibia_length - distance * distance
    ) / (2 * femur_length * tibia_length)

    # Clamp to valid range for acos (handle floating point errors)
    cos_tibia = max(-1.0, min(1.0, cos_tibia))

    # Tibia angle (measured as deviation from straight line with femur)
    # When tibia_angle = 0, leg is fully extended
    # When tibia_angle = -180, leg is fully folded
    tibia_angle_rad = math.acos(cos_tibia)
    tibia_angle = math.degrees(tibia_angle_rad) - 180.0  # Offset so 0 = extended

    # Calculate femur angle
    # First, angle from horizontal to the line from femur joint to foot
    alpha = math.atan2(-z_prime, x_prime)

    # Second, angle between femur and the line to foot (law of cosines)
    cos_beta = (femur_length * femur_length + distance * distance - tibia_length * tibia_length) / (
        2 * femur_length * distance
    )
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    # Femur angle (0 = horizontal forward)
    femur_angle = math.degrees(alpha + beta)

    return (coxa_angle, femur_angle, tibia_angle)


def forward_kinematics_3dof(
    coxa_angle: float,
    femur_angle: float,
    tibia_angle: float,
    coxa_length: float,
    femur_length: float,
    tibia_length: float,
) -> tuple[float, float, float]:
    """Calculate foot position from joint angles (forward kinematics).

    This is the inverse of inverse_kinematics_3dof and uses the same
    angle convention:
    - coxa_angle: rotation in horizontal plane (0 = +x direction)
    - femur_angle: angle from horizontal (0 = horizontal, positive = up)
    - tibia_angle: angle relative to femur (0 = extended, -180 = folded back)

    Coordinate convention:
    - x: forward (positive = away from body)
    - y: lateral (positive = right when facing forward)
    - z: vertical (negative = down)

    Args:
        coxa_angle: Coxa joint angle in degrees
        femur_angle: Femur joint angle in degrees
        tibia_angle: Tibia joint angle in degrees
        coxa_length: Length of coxa segment (meters)
        femur_length: Length of femur segment (meters)
        tibia_length: Length of tibia segment (meters)

    Returns:
        Tuple of (x, y, z) foot position in meters.

    Example:
        >>> x, y, z = forward_kinematics_3dof(
        ...     coxa_angle=0, femur_angle=45, tibia_angle=-90,
        ...     coxa_length=0.03, femur_length=0.08, tibia_length=0.12,
        ... )
    """
    coxa_rad = math.radians(coxa_angle)
    femur_rad = math.radians(femur_angle)
    tibia_rad = math.radians(tibia_angle)

    # Calculate in the vertical plane (local frame before coxa rotation)
    # x' is horizontal distance from body

    # Femur contribution in local frame
    # femur_angle = 0 means horizontal, positive means up
    femur_x_local = femur_length * math.cos(femur_rad)
    femur_z_internal = femur_length * math.sin(femur_rad)  # internal positive up

    # Tibia direction relative to femur
    # tibia_angle = 0 means extended (same direction as femur)
    # tibia_angle = -90 means 90° bent (pointing down relative to femur)
    tibia_abs_rad = femur_rad + tibia_rad

    tibia_x_local = tibia_length * math.cos(tibia_abs_rad)
    tibia_z_internal = tibia_length * math.sin(tibia_abs_rad)  # internal positive up

    # Total horizontal distance in local frame
    x_prime = coxa_length + femur_x_local + tibia_x_local

    # z in world frame (negative = down, matching IK convention)
    z_internal = femur_z_internal + tibia_z_internal
    z = -z_internal  # Flip sign: IK uses negative z for down

    # Apply coxa rotation to get world coordinates
    x = x_prime * math.cos(coxa_rad)
    y = x_prime * math.sin(coxa_rad)

    return (x, y, z)


# =============================================================================
# Leg Controller
# =============================================================================


class Leg(Controller):
    """Single robot leg controller (3-DOF).

    Controls a leg with coxa (hip), femur (thigh), and tibia (shin) joints.
    Provides inverse kinematics for foot position control and forward
    kinematics for reading current position.

    Example:
        >>> from robo_infra.controllers.leg import Leg, LegConfig
        >>> from robo_infra.actuators import Servo
        >>>
        >>> config = LegConfig(
        ...     coxa_length=0.03,
        ...     femur_length=0.08,
        ...     tibia_length=0.12,
        ... )
        >>>
        >>> leg = Leg(
        ...     name="front_left",
        ...     coxa=Servo(channel=0),
        ...     femur=Servo(channel=1),
        ...     tibia=Servo(channel=2),
        ...     config=config,
        ... )
        >>>
        >>> leg.enable()
        >>> leg.home()
        >>> leg.set_foot_position(0.05, 0.0, -0.15)
    """

    def __init__(
        self,
        name: str,
        coxa: Actuator,
        femur: Actuator,
        tibia: Actuator,
        config: LegConfig | None = None,
        *,
        position: LegPosition | str | None = None,
    ) -> None:
        """Initialize leg controller.

        Args:
            name: Name of this leg
            coxa: Coxa (hip) joint actuator
            femur: Femur (thigh) joint actuator
            tibia: Tibia (shin) joint actuator
            config: Leg configuration
            position: Position of leg on robot body
        """
        super().__init__(name=name)

        self._config = config or LegConfig()
        self._leg_state = LegState.DISABLED
        self._ground_contact = False
        self._load = 0.0

        # Store position
        if isinstance(position, str):
            try:
                self._position = LegPosition(position)
            except ValueError:
                self._position = None
        else:
            self._position = position

        # Store joint actuators with proper limits
        self._coxa = coxa
        self._femur = femur
        self._tibia = tibia

        # Add actuators to controller
        self.add_actuator("coxa", coxa)
        self.add_actuator("femur", femur)
        self.add_actuator("tibia", tibia)

        # Apply config limits to actuators
        self._apply_config_limits()

        logger.info(f"Leg '{name}' initialized with config: {self._config}")

    def _apply_config_limits(self) -> None:
        """Apply configuration limits to actuators."""
        # This is a no-op if actuators don't support setting limits
        # In practice, actuators should be created with appropriate limits
        pass

    @property
    def leg_config(self) -> LegConfig:
        """Get leg configuration."""
        return self._config

    @property
    def leg_state(self) -> LegState:
        """Get current leg state."""
        return self._leg_state

    @property
    def dimensions(self) -> LegDimensions:
        """Get leg dimensions."""
        return self._config.dimensions

    @property
    def position(self) -> LegPosition | None:
        """Get leg position on body."""
        return self._position

    @property
    def is_grounded(self) -> bool:
        """Check if foot is in contact with ground."""
        return self._ground_contact

    @property
    def coxa(self) -> Actuator:
        """Get coxa joint actuator."""
        return self._coxa

    @property
    def femur(self) -> Actuator:
        """Get femur joint actuator."""
        return self._femur

    @property
    def tibia(self) -> Actuator:
        """Get tibia joint actuator."""
        return self._tibia

    def get_joint_angles(self) -> JointAngles:
        """Get current joint angles.

        Returns:
            Current joint angles in degrees.
        """
        return JointAngles(
            coxa=self._coxa.get(),
            femur=self._femur.get(),
            tibia=self._tibia.get(),
        )

    def set_joint_angles(
        self,
        coxa: float | None = None,
        femur: float | None = None,
        tibia: float | None = None,
        *,
        validate: bool = True,
    ) -> None:
        """Set joint angles directly.

        Args:
            coxa: Coxa joint angle in degrees (None = keep current)
            femur: Femur joint angle in degrees (None = keep current)
            tibia: Tibia joint angle in degrees (None = keep current)
            validate: Whether to validate angles against limits

        Raises:
            DisabledError: If leg is not enabled
            LimitsExceededError: If angles exceed joint limits
        """
        if not self.is_enabled:
            raise DisabledError(f"Leg '{self.name}' is not enabled")

        if validate:
            if coxa is not None and not (self._config.coxa_min <= coxa <= self._config.coxa_max):
                raise LimitsExceededError(
                    f"Coxa angle {coxa}° exceeds limits "
                    f"[{self._config.coxa_min}, {self._config.coxa_max}]"
                )
            if femur is not None and not (
                self._config.femur_min <= femur <= self._config.femur_max
            ):
                raise LimitsExceededError(
                    f"Femur angle {femur}° exceeds limits "
                    f"[{self._config.femur_min}, {self._config.femur_max}]"
                )
            if tibia is not None and not (
                self._config.tibia_min <= tibia <= self._config.tibia_max
            ):
                raise LimitsExceededError(
                    f"Tibia angle {tibia}° exceeds limits "
                    f"[{self._config.tibia_min}, {self._config.tibia_max}]"
                )

        self._leg_state = LegState.MOVING

        # Apply inversion if configured
        if coxa is not None:
            actual_coxa = -coxa if self._config.invert_coxa else coxa
            self._coxa.set(actual_coxa)

        if femur is not None:
            actual_femur = -femur if self._config.invert_femur else femur
            self._femur.set(actual_femur)

        if tibia is not None:
            actual_tibia = -tibia if self._config.invert_tibia else tibia
            self._tibia.set(actual_tibia)

        self._leg_state = LegState.IDLE
        logger.debug(f"Leg '{self.name}' set angles: coxa={coxa}, femur={femur}, tibia={tibia}")

    def get_foot_position(self) -> FootPosition:
        """Get current foot position using forward kinematics.

        Returns:
            Current foot position in leg coordinate frame.
        """
        angles = self.get_joint_angles()
        x, y, z = forward_kinematics_3dof(
            coxa_angle=angles.coxa,
            femur_angle=angles.femur,
            tibia_angle=angles.tibia,
            coxa_length=self._config.coxa_length,
            femur_length=self._config.femur_length,
            tibia_length=self._config.tibia_length,
        )
        return FootPosition(x=x, y=y, z=z)

    def set_foot_position(
        self,
        x: float,
        y: float,
        z: float,
        *,
        validate_limits: bool = True,
    ) -> None:
        """Set foot position using inverse kinematics.

        Args:
            x: Forward distance from hip (meters)
            y: Lateral distance from hip (meters)
            z: Vertical distance from hip (meters, usually negative)
            validate_limits: Whether to check joint limits

        Raises:
            DisabledError: If leg is not enabled
            KinematicsError: If position is unreachable
            LimitsExceededError: If required angles exceed joint limits
        """
        if not self.is_enabled:
            raise DisabledError(f"Leg '{self.name}' is not enabled")

        # Calculate joint angles
        coxa, femur, tibia = inverse_kinematics_3dof(
            x=x,
            y=y,
            z=z,
            coxa_length=self._config.coxa_length,
            femur_length=self._config.femur_length,
            tibia_length=self._config.tibia_length,
        )

        # Set the angles
        self.set_joint_angles(coxa=coxa, femur=femur, tibia=tibia, validate=validate_limits)

        logger.debug(f"Leg '{self.name}' moved to position: ({x:.3f}, {y:.3f}, {z:.3f})")

    def move_to(self, targets: dict[str, float], config: MotionConfig | None = None) -> None:
        """Move joints to target positions.

        Overrides base Controller method to handle leg-specific behavior.

        Args:
            targets: Dict mapping joint names ("coxa", "femur", "tibia") to angles
            config: Motion configuration
        """
        coxa = targets.get("coxa")
        femur = targets.get("femur")
        tibia = targets.get("tibia")
        self.set_joint_angles(coxa=coxa, femur=femur, tibia=tibia)

    def home(self) -> None:
        """Move leg to home position."""
        if not self.is_enabled:
            raise DisabledError(f"Leg '{self.name}' is not enabled")

        self._leg_state = LegState.HOMING
        self.set_joint_angles(
            coxa=self._config.coxa_home,
            femur=self._config.femur_home,
            tibia=self._config.tibia_home,
            validate=False,  # Home position should always be valid
        )
        self._is_homed = True
        self._leg_state = LegState.IDLE

        logger.info(f"Leg '{self.name}' homed")

    def _do_home(self) -> None:
        """Perform homing sequence (implements abstract method)."""
        self.set_joint_angles(
            coxa=self._config.coxa_home,
            femur=self._config.femur_home,
            tibia=self._config.tibia_home,
            validate=False,
        )
        self._is_homed = True
        self._leg_state = LegState.IDLE

    def _do_stop(self) -> None:
        """Perform emergency stop (implements abstract method)."""
        # Disable all actuators immediately
        self._coxa.disable()
        self._femur.disable()
        self._tibia.disable()
        self._leg_state = LegState.IDLE

    def enable(self) -> None:
        """Enable the leg controller."""
        super().enable()
        self._leg_state = LegState.IDLE

    def disable(self) -> None:
        """Disable the leg controller."""
        super().disable()
        self._leg_state = LegState.DISABLED

    def stop(self) -> None:
        """Stop all leg motion immediately."""
        super().stop()
        self._leg_state = LegState.IDLE

    def set_stance(self) -> None:
        """Set leg to stance mode (supporting weight)."""
        self._leg_state = LegState.STANCE
        self._ground_contact = True

    def set_swing(self) -> None:
        """Set leg to swing mode (moving in air)."""
        self._leg_state = LegState.SWING
        self._ground_contact = False

    def set_ground_contact(self, contact: bool) -> None:
        """Set ground contact state.

        Args:
            contact: True if foot is touching ground
        """
        self._ground_contact = contact
        if contact:
            self._leg_state = LegState.STANCE
        else:
            self._leg_state = (
                LegState.SWING if self._leg_state == LegState.STANCE else self._leg_state
            )

    def set_load(self, load: float) -> None:
        """Set load on the foot.

        Args:
            load: Load value (0.0 = no load, 1.0 = full load)
        """
        self._load = max(0.0, min(1.0, load))

    def leg_status(self) -> LegStatus:
        """Get current leg status.

        Returns:
            Complete status of the leg.
        """
        return LegStatus(
            state=self._leg_state,
            is_enabled=self.is_enabled,
            is_homed=self._is_homed,
            foot_position=self.get_foot_position(),
            joint_angles=self.get_joint_angles(),
            ground_contact=self._ground_contact,
            load=self._load,
            error=self._error,
        )

    def as_tools(self) -> list[dict[str, Any] | Callable[..., Any]]:
        """Generate AI tools for controlling the leg.

        Returns:
            List of callable tools for ai-infra Agent integration.
        """

        def set_foot_position(x: float, y: float, z: float) -> str:
            """Move leg foot to a 3D position.

            Args:
                x: Forward distance from hip in meters
                y: Lateral distance from hip in meters
                z: Vertical distance from hip in meters (negative = down)

            Returns:
                Status message
            """
            try:
                self.set_foot_position(x, y, z)
                return f"Moved foot to ({x:.3f}, {y:.3f}, {z:.3f})"
            except (KinematicsError, LimitsExceededError) as e:
                return f"Failed: {e}"

        def set_joint_angles(
            coxa: float | None = None,
            femur: float | None = None,
            tibia: float | None = None,
        ) -> str:
            """Set leg joint angles directly.

            Args:
                coxa: Coxa (hip) angle in degrees
                femur: Femur (thigh) angle in degrees
                tibia: Tibia (shin) angle in degrees

            Returns:
                Status message
            """
            try:
                self.set_joint_angles(coxa=coxa, femur=femur, tibia=tibia)
                return f"Set angles: coxa={coxa}, femur={femur}, tibia={tibia}"
            except LimitsExceededError as e:
                return f"Failed: {e}"

        def get_leg_status() -> dict:
            """Get current leg status.

            Returns:
                Dict with leg state, position, angles, and ground contact
            """
            return self.leg_status().as_dict()

        def home_leg() -> str:
            """Move leg to home position.

            Returns:
                Status message
            """
            self.home()
            return f"Leg '{self.name}' homed"

        return [set_foot_position, set_joint_angles, get_leg_status, home_leg]


# =============================================================================
# Factory Function
# =============================================================================


def create_leg(
    name: str,
    coxa_length: float = 0.03,
    femur_length: float = 0.08,
    tibia_length: float = 0.12,
    *,
    position: LegPosition | str | None = None,
    simulated: bool = True,
    config: LegConfig | None = None,
) -> Leg:
    """Create a leg controller with default actuators.

    This factory function simplifies leg creation by automatically
    creating simulated actuators with appropriate limits.

    Args:
        name: Name for the leg
        coxa_length: Coxa segment length in meters (ignored if config provided)
        femur_length: Femur segment length in meters (ignored if config provided)
        tibia_length: Tibia segment length in meters (ignored if config provided)
        position: Position of leg on robot body
        simulated: If True, use simulated actuators
        config: Full LegConfig (if provided, length args are ignored)

    Returns:
        Configured Leg controller

    Example:
        >>> leg = create_leg("front_left", simulated=True)
        >>> leg.enable()
        >>> leg.home()
    """
    if config is None:
        config = LegConfig(
            coxa_length=coxa_length,
            femur_length=femur_length,
            tibia_length=tibia_length,
        )

    # Create actuators
    coxa = SimulatedActuator(
        name=f"{name}_coxa",
        limits=config.coxa_limits,
    )
    femur = SimulatedActuator(
        name=f"{name}_femur",
        limits=config.femur_limits,
    )
    tibia = SimulatedActuator(
        name=f"{name}_tibia",
        limits=config.tibia_limits,
    )

    return Leg(
        name=name,
        coxa=coxa,
        femur=femur,
        tibia=tibia,
        config=config,
        position=position,
    )
