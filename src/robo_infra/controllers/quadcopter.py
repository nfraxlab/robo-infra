"""Quadcopter/Drone flight controller.

This module provides a Quadcopter controller for multi-rotor aerial vehicles.
Supports various frame configurations (X, +, H) and flight modes.

Example:
    >>> from robo_infra.controllers.quadcopter import Quadcopter, QuadcopterConfig
    >>> from robo_infra.actuators.brushless import BrushlessMotor
    >>>
    >>> # Create motors (front-left, front-right, rear-left, rear-right)
    >>> motors = [BrushlessMotor(name=f"motor_{i}") for i in range(4)]
    >>>
    >>> # Create quadcopter controller
    >>> config = QuadcopterConfig(
    ...     name="drone",
    ...     arm_length=0.25,  # 250mm arm length
    ...     motor_kv=2300,
    ...     prop_diameter=5.0,
    ...     frame_type="X",
    ... )
    >>> quad = Quadcopter(
    ...     name="my_drone",
    ...     motors=motors,
    ...     config=config,
    ... )
    >>> quad.enable()
    >>> quad.arm()
    >>> quad.takeoff(altitude=2.0)
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field, field_validator

from robo_infra.core.controller import Controller, ControllerConfig
from robo_infra.core.exceptions import DisabledError, SafetyError


if TYPE_CHECKING:
    from collections.abc import Callable

    from robo_infra.actuators.brushless import BrushlessMotor
    from robo_infra.sensors.imu import IMU


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

# Standard motor positions (looking down, front is +X)
# For X configuration:
#   Front-Left (0): +X, +Y, CCW
#   Front-Right (1): +X, -Y, CW
#   Rear-Left (2): -X, +Y, CW
#   Rear-Right (3): -X, -Y, CCW

# Mixing matrix for X configuration [throttle, roll, pitch, yaw]
MIXER_X = [
    [1.0, -1.0, +1.0, -1.0],  # Front-Left (CCW)
    [1.0, +1.0, +1.0, +1.0],  # Front-Right (CW)
    [1.0, -1.0, -1.0, +1.0],  # Rear-Left (CW)
    [1.0, +1.0, -1.0, -1.0],  # Rear-Right (CCW)
]

# Mixing matrix for + configuration [throttle, roll, pitch, yaw]
MIXER_PLUS = [
    [1.0, 0.0, +1.0, -1.0],  # Front (CCW)
    [1.0, -1.0, 0.0, +1.0],  # Left (CW)
    [1.0, +1.0, 0.0, +1.0],  # Right (CW)
    [1.0, 0.0, -1.0, -1.0],  # Rear (CCW)
]

# Mixing matrix for H configuration (same as X but different geometry)
MIXER_H = MIXER_X

# Default values
DEFAULT_MAX_THROTTLE = 1.0
DEFAULT_MIN_THROTTLE = 0.0
DEFAULT_IDLE_THROTTLE = 0.05
DEFAULT_TAKEOFF_THROTTLE = 0.5
DEFAULT_HOVER_THROTTLE = 0.45
DEFAULT_MAX_TILT_ANGLE = 35.0  # degrees
DEFAULT_MAX_YAW_RATE = 180.0  # degrees per second
DEFAULT_MAX_CLIMB_RATE = 3.0  # m/s
DEFAULT_MAX_DESCENT_RATE = 2.0  # m/s


# =============================================================================
# Enums
# =============================================================================


class QuadcopterState(Enum):
    """States a Quadcopter can be in."""

    DISABLED = "disabled"
    DISARMED = "disarmed"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    HOVERING = "hovering"
    FLYING = "flying"
    LANDING = "landing"
    EMERGENCY = "emergency"
    ERROR = "error"


class FlightMode(Enum):
    """Flight control modes."""

    # Stabilized modes (attitude control)
    STABILIZE = "stabilize"  # Manual throttle, auto-level
    ACRO = "acro"  # Rate mode, no auto-level

    # Altitude hold modes
    ALT_HOLD = "alt_hold"  # Auto throttle for altitude
    LOITER = "loiter"  # Position hold with GPS

    # Autonomous modes
    AUTO = "auto"  # Follow waypoints
    GUIDED = "guided"  # Accept goto commands
    RTL = "rtl"  # Return to launch

    # Special modes
    LAND = "land"  # Auto landing
    TAKEOFF = "takeoff"  # Auto takeoff


class FrameType(Enum):
    """Quadcopter frame configuration."""

    X = "X"  # X configuration (most common)
    PLUS = "+"  # Plus configuration
    H = "H"  # H-frame configuration


class MotorPosition(Enum):
    """Motor positions for quadcopter."""

    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    REAR_LEFT = 2
    REAR_RIGHT = 3


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class Attitude:
    """Aircraft attitude (orientation).

    All angles in degrees.
    """

    roll: float = 0.0  # Bank angle (-180 to 180)
    pitch: float = 0.0  # Nose up/down (-90 to 90)
    yaw: float = 0.0  # Heading (0 to 360)

    def to_radians(self) -> tuple[float, float, float]:
        """Convert to radians."""
        return (
            math.radians(self.roll),
            math.radians(self.pitch),
            math.radians(self.yaw),
        )

    @classmethod
    def from_radians(cls, roll: float, pitch: float, yaw: float) -> Attitude:
        """Create from radians."""
        return cls(
            roll=math.degrees(roll),
            pitch=math.degrees(pitch),
            yaw=math.degrees(yaw),
        )


@dataclass(slots=True)
class Velocity:
    """3D velocity vector.

    All values in meters per second.
    """

    vx: float = 0.0  # Forward/back velocity
    vy: float = 0.0  # Left/right velocity
    vz: float = 0.0  # Up/down velocity
    yaw_rate: float = 0.0  # Yaw rate in degrees/second

    @property
    def speed(self) -> float:
        """Calculate total speed magnitude."""
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

    @property
    def horizontal_speed(self) -> float:
        """Calculate horizontal speed magnitude."""
        return math.sqrt(self.vx**2 + self.vy**2)


@dataclass(slots=True)
class Position3D:
    """3D position.

    All values in meters relative to home/origin.
    """

    x: float = 0.0  # Forward/back position
    y: float = 0.0  # Left/right position
    z: float = 0.0  # Altitude (up is positive)
    yaw: float = 0.0  # Heading in degrees

    def distance_to(self, other: Position3D) -> float:
        """Calculate distance to another position."""
        return math.sqrt(
            (other.x - self.x) ** 2 + (other.y - self.y) ** 2 + (other.z - self.z) ** 2
        )

    def horizontal_distance_to(self, other: Position3D) -> float:
        """Calculate horizontal distance to another position."""
        return math.sqrt((other.x - self.x) ** 2 + (other.y - self.y) ** 2)


@dataclass
class MotorOutputs:
    """Motor throttle outputs (0.0 to 1.0)."""

    front_left: float = 0.0
    front_right: float = 0.0
    rear_left: float = 0.0
    rear_right: float = 0.0

    def __post_init__(self) -> None:
        """Clamp all values to valid range."""
        self.front_left = max(0.0, min(1.0, self.front_left))
        self.front_right = max(0.0, min(1.0, self.front_right))
        self.rear_left = max(0.0, min(1.0, self.rear_left))
        self.rear_right = max(0.0, min(1.0, self.rear_right))

    def as_list(self) -> list[float]:
        """Return outputs as list [FL, FR, RL, RR]."""
        return [self.front_left, self.front_right, self.rear_left, self.rear_right]

    @classmethod
    def from_list(cls, values: list[float]) -> MotorOutputs:
        """Create from list of values."""
        if len(values) != 4:
            msg = f"Expected 4 values, got {len(values)}"
            raise ValueError(msg)
        return cls(
            front_left=values[0],
            front_right=values[1],
            rear_left=values[2],
            rear_right=values[3],
        )

    def scaled(self, factor: float) -> MotorOutputs:
        """Return scaled motor outputs."""
        return MotorOutputs(
            front_left=self.front_left * factor,
            front_right=self.front_right * factor,
            rear_left=self.rear_left * factor,
            rear_right=self.rear_right * factor,
        )


@dataclass
class QuadcopterStatus:
    """Current status of the quadcopter."""

    state: QuadcopterState
    flight_mode: FlightMode
    is_enabled: bool
    is_armed: bool
    attitude: Attitude
    velocity: Velocity
    position: Position3D
    motor_outputs: MotorOutputs
    battery_voltage: float | None = None
    battery_percent: float | None = None
    gps_fix: bool = False
    satellites: int = 0
    armed_time: float = 0.0
    flight_time: float = 0.0
    error: str | None = None


# =============================================================================
# Configuration Models
# =============================================================================


class QuadcopterConfig(BaseModel):
    """Configuration for a Quadcopter controller.

    Attributes:
        name: Human-readable name for the quadcopter.
        arm_length: Distance from center to motor in meters.
        motor_kv: Motor KV rating (RPM per volt).
        prop_diameter: Propeller diameter in inches.
        frame_type: Frame configuration (X, +, or H).
        weight: Total weight in kilograms.

    Example:
        >>> config = QuadcopterConfig(
        ...     name="racing_quad",
        ...     arm_length=0.125,  # 5" quad
        ...     motor_kv=2400,
        ...     prop_diameter=5.0,
        ...     frame_type="X",
        ...     weight=0.35,
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Quadcopter name")
    description: str = Field(default="", description="Human-readable description")

    # Physical dimensions
    arm_length: float = Field(
        default=0.25,
        gt=0,
        description="Distance from center to motor in meters",
    )
    motor_kv: int = Field(
        default=2300,
        gt=0,
        description="Motor KV rating (RPM per volt)",
    )
    prop_diameter: float = Field(
        default=5.0,
        gt=0,
        description="Propeller diameter in inches",
    )
    frame_type: str = Field(
        default="X",
        description="Frame configuration: X, +, or H",
    )
    weight: float = Field(
        default=1.0,
        gt=0,
        description="Total weight in kilograms",
    )

    # Battery
    battery_cells: int = Field(
        default=4,
        ge=1,
        le=12,
        description="Number of LiPo cells (S count)",
    )
    battery_capacity: int = Field(
        default=1500,
        gt=0,
        description="Battery capacity in mAh",
    )

    # Flight limits
    max_tilt_angle: float = Field(
        default=DEFAULT_MAX_TILT_ANGLE,
        ge=0,
        le=80,
        description="Maximum tilt angle in degrees",
    )
    max_yaw_rate: float = Field(
        default=DEFAULT_MAX_YAW_RATE,
        gt=0,
        description="Maximum yaw rate in degrees per second",
    )
    max_climb_rate: float = Field(
        default=DEFAULT_MAX_CLIMB_RATE,
        gt=0,
        description="Maximum climb rate in m/s",
    )
    max_descent_rate: float = Field(
        default=DEFAULT_MAX_DESCENT_RATE,
        gt=0,
        description="Maximum descent rate in m/s",
    )
    max_speed: float = Field(
        default=10.0,
        gt=0,
        description="Maximum horizontal speed in m/s",
    )
    max_altitude: float = Field(
        default=120.0,
        gt=0,
        description="Maximum altitude in meters (regulatory limit)",
    )

    # Throttle settings
    idle_throttle: float = Field(
        default=DEFAULT_IDLE_THROTTLE,
        ge=0,
        le=1,
        description="Idle throttle when armed",
    )
    hover_throttle: float = Field(
        default=DEFAULT_HOVER_THROTTLE,
        ge=0,
        le=1,
        description="Throttle for stable hover",
    )
    min_throttle: float = Field(
        default=DEFAULT_MIN_THROTTLE,
        ge=0,
        le=1,
        description="Minimum throttle",
    )
    max_throttle: float = Field(
        default=DEFAULT_MAX_THROTTLE,
        ge=0,
        le=1,
        description="Maximum throttle",
    )

    # Safety
    low_battery_voltage: float = Field(
        default=3.5,
        gt=0,
        description="Low battery warning voltage per cell",
    )
    critical_battery_voltage: float = Field(
        default=3.3,
        gt=0,
        description="Critical battery voltage per cell (auto-land)",
    )
    arm_safety_check: bool = Field(
        default=True,
        description="Require safety checks before arming",
    )
    geofence_radius: float = Field(
        default=100.0,
        gt=0,
        description="Maximum distance from home in meters",
    )
    geofence_altitude: float = Field(
        default=120.0,
        gt=0,
        description="Maximum altitude in meters",
    )

    @field_validator("frame_type")
    @classmethod
    def validate_frame_type(cls, v: str) -> str:
        """Validate frame type."""
        valid = {"X", "+", "H"}
        if v not in valid:
            msg = f"frame_type must be one of {valid}"
            raise ValueError(msg)
        return v

    @property
    def frame_type_enum(self) -> FrameType:
        """Get frame type as enum."""
        if self.frame_type == "+":
            return FrameType.PLUS
        return FrameType(self.frame_type)

    @property
    def mixer_matrix(self) -> list[list[float]]:
        """Get the motor mixing matrix for this frame type."""
        if self.frame_type == "+":
            return MIXER_PLUS
        if self.frame_type == "H":
            return MIXER_H
        return MIXER_X

    @property
    def wheelbase(self) -> float:
        """Calculate motor-to-motor diagonal distance in meters."""
        return self.arm_length * 2

    @property
    def prop_diameter_m(self) -> float:
        """Propeller diameter in meters."""
        return self.prop_diameter * 0.0254

    @property
    def battery_voltage_full(self) -> float:
        """Fully charged battery voltage."""
        return self.battery_cells * 4.2

    @property
    def battery_voltage_empty(self) -> float:
        """Empty battery voltage (at critical level)."""
        return self.battery_cells * self.critical_battery_voltage


# =============================================================================
# Motor Mixing Functions
# =============================================================================


def mix_motors(
    throttle: float,
    roll: float,
    pitch: float,
    yaw: float,
    mixer: list[list[float]],
) -> MotorOutputs:
    """Apply motor mixing to convert control inputs to motor outputs.

    Args:
        throttle: Base throttle (0.0 to 1.0)
        roll: Roll control (-1.0 to 1.0, positive = right)
        pitch: Pitch control (-1.0 to 1.0, positive = forward)
        yaw: Yaw control (-1.0 to 1.0, positive = clockwise)
        mixer: 4x4 mixing matrix

    Returns:
        Motor outputs clamped to 0.0-1.0
    """
    outputs = []
    for i in range(4):
        output = (
            mixer[i][0] * throttle + mixer[i][1] * roll + mixer[i][2] * pitch + mixer[i][3] * yaw
        )
        outputs.append(max(0.0, min(1.0, output)))

    return MotorOutputs.from_list(outputs)


def normalize_motor_outputs(outputs: MotorOutputs, min_output: float = 0.0) -> MotorOutputs:
    """Normalize motor outputs to maintain proportions while staying in range.

    If any output exceeds 1.0, all outputs are scaled down proportionally.
    Ensures minimum output for armed motors.

    Args:
        outputs: Raw motor outputs
        min_output: Minimum output for any motor (idle throttle)

    Returns:
        Normalized motor outputs
    """
    values = outputs.as_list()
    max_val = max(values)

    # Scale down if any output exceeds 1.0
    if max_val > 1.0:
        values = [v / max_val for v in values]

    # Ensure minimum output
    values = [max(min_output, v) for v in values]

    return MotorOutputs.from_list(values)


# =============================================================================
# Quadcopter Controller
# =============================================================================


class Quadcopter(Controller):
    """Controller for quadcopter/drone aircraft.

    Provides high-level flight control for quadcopter drones with
    four motors. Supports multiple frame configurations and flight modes.

    Key Features:
        - Motor mixing for X, +, and H frame configurations
        - Arm/disarm with safety checks
        - Takeoff, land, hover commands
        - Attitude control (roll, pitch, yaw)
        - Velocity control
        - Position goto commands
        - Altitude hold mode
        - Emergency stop

    Safety Features:
        - Pre-arm safety checks
        - Low battery auto-land
        - Geofence enforcement
        - Motor cutoff on disarm

    Example:
        >>> from robo_infra.controllers.quadcopter import Quadcopter
        >>> from robo_infra.actuators.brushless import BrushlessMotor
        >>>
        >>> motors = [BrushlessMotor(name=f"m{i}") for i in range(4)]
        >>> quad = Quadcopter("my_drone", motors=motors)
        >>> quad.enable()
        >>> quad.arm()
        >>> quad.takeoff(altitude=2.0)
        >>> quad.hover()
        >>> quad.land()
        >>> quad.disarm()
    """

    def __init__(
        self,
        name: str,
        *,
        motors: list[BrushlessMotor] | None = None,
        imu: IMU | None = None,
        config: QuadcopterConfig | None = None,
        simulated: bool = True,
    ) -> None:
        """Initialize quadcopter controller.

        Args:
            name: Controller name
            motors: List of 4 brushless motors [FL, FR, RL, RR]
            imu: Optional IMU sensor for attitude
            config: Optional configuration
            simulated: Use simulated motors if True

        Raises:
            ValueError: If motors list is not exactly 4 motors
        """
        # Create controller config from quadcopter config
        ctrl_config = ControllerConfig(name=name)
        super().__init__(name, config=ctrl_config)

        self._quad_config = config or QuadcopterConfig(name=name)
        self._simulated = simulated

        # Motors
        if motors is not None:
            if len(motors) != 4:
                msg = f"Quadcopter requires exactly 4 motors, got {len(motors)}"
                raise ValueError(msg)
            self._motors = motors
            # Add motors to controller's actuator registry
            for i, motor in enumerate(motors):
                self.add_actuator(MotorPosition(i).name.lower(), motor)
        else:
            self._motors = None

        # Optional IMU
        self._imu = imu
        if imu is not None:
            self.add_sensor("imu", imu)

        # State
        self._quad_state = QuadcopterState.DISABLED
        self._flight_mode = FlightMode.STABILIZE
        self._is_armed = False
        self._armed_time: float | None = None
        self._takeoff_time: float | None = None

        # Current values
        self._attitude = Attitude()
        self._velocity = Velocity()
        self._position = Position3D()
        self._target_position: Position3D | None = None
        self._motor_outputs = MotorOutputs()

        # Simulated state
        self._sim_battery_voltage = self._quad_config.battery_voltage_full
        self._sim_altitude = 0.0

        logger.debug("Quadcopter '%s' initialized (frame=%s)", name, self._quad_config.frame_type)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def quad_config(self) -> QuadcopterConfig:
        """Get quadcopter configuration."""
        return self._quad_config

    @property
    def quad_state(self) -> QuadcopterState:
        """Get current quadcopter state."""
        return self._quad_state

    @property
    def flight_mode(self) -> FlightMode:
        """Get current flight mode."""
        return self._flight_mode

    @property
    def is_armed(self) -> bool:
        """Check if quadcopter is armed."""
        return self._is_armed

    @property
    def is_flying(self) -> bool:
        """Check if quadcopter is in flight."""
        return self._quad_state in {
            QuadcopterState.TAKING_OFF,
            QuadcopterState.HOVERING,
            QuadcopterState.FLYING,
        }

    @property
    def attitude(self) -> Attitude:
        """Get current attitude."""
        return self._attitude

    @property
    def velocity(self) -> Velocity:
        """Get current velocity."""
        return self._velocity

    @property
    def position(self) -> Position3D:
        """Get current position."""
        return self._position

    @property
    def altitude(self) -> float:
        """Get current altitude in meters."""
        return self._position.z

    @property
    def motor_outputs(self) -> MotorOutputs:
        """Get current motor outputs."""
        return self._motor_outputs

    @property
    def motors(self) -> list[BrushlessMotor] | None:
        """Get motor list."""
        return self._motors

    @property
    def armed_time(self) -> float:
        """Get time since arming in seconds."""
        if self._armed_time is None:
            return 0.0
        return time.time() - self._armed_time

    @property
    def flight_time(self) -> float:
        """Get time since takeoff in seconds."""
        if self._takeoff_time is None:
            return 0.0
        return time.time() - self._takeoff_time

    # -------------------------------------------------------------------------
    # Controller Implementation
    # -------------------------------------------------------------------------

    def _do_home(self) -> None:
        """Perform homing - not applicable for quadcopter."""
        # Quadcopters don't have a home sequence like arms
        # Home position is set at power-on
        logger.debug("Quadcopter homing: setting current position as home")
        self._position = Position3D(x=0, y=0, z=0, yaw=0)
        self._is_homed = True

    def _do_stop(self) -> None:
        """Emergency stop - cut all motors immediately."""
        logger.warning("EMERGENCY STOP: Cutting all motors!")
        self._quad_state = QuadcopterState.EMERGENCY
        self._is_armed = False
        self._motor_outputs = MotorOutputs()

        # Cut motors if available
        if self._motors:
            for motor in self._motors:
                try:
                    motor.set_throttle(0.0)
                    motor.disable()
                except Exception as e:
                    logger.error("Failed to stop motor: %s", e)

    def _on_enable(self) -> None:
        """Called when controller is enabled."""
        self._quad_state = QuadcopterState.DISARMED
        logger.info("Quadcopter '%s' enabled, ready to arm", self._name)

    def _on_disable(self) -> None:
        """Called when controller is disabled."""
        if self._is_armed:
            self.disarm()
        self._quad_state = QuadcopterState.DISABLED
        logger.info("Quadcopter '%s' disabled", self._name)

    # -------------------------------------------------------------------------
    # Arming
    # -------------------------------------------------------------------------

    def can_arm(self) -> tuple[bool, str]:
        """Check if quadcopter can be armed.

        Returns:
            Tuple of (can_arm, reason)
        """
        if not self._is_enabled:
            return False, "Controller not enabled"

        if self._quad_state == QuadcopterState.EMERGENCY:
            return False, "In emergency state - reset required"

        if self._quad_state not in {QuadcopterState.DISARMED, QuadcopterState.DISABLED}:
            return False, f"Cannot arm in state: {self._quad_state.value}"

        # Safety checks
        if self._quad_config.arm_safety_check:
            # Check battery
            if self._sim_battery_voltage < self._quad_config.battery_voltage_empty:
                return False, "Battery too low"

            # Check attitude is level
            if abs(self._attitude.roll) > 30 or abs(self._attitude.pitch) > 30:
                return False, "Aircraft not level"

            # Check throttle is at minimum (in real system, check RC)
            # For simulation, we skip this check

        return True, "Ready to arm"

    def arm(self) -> None:
        """Arm the quadcopter motors.

        Raises:
            SafetyError: If safety checks fail
            DisabledError: If controller not enabled
        """
        can_arm, reason = self.can_arm()
        if not can_arm:
            if not self._is_enabled:
                raise DisabledError(reason)
            raise SafetyError(reason)

        # Enable motors
        if self._motors:
            for motor in self._motors:
                motor.enable()

        # Set state
        self._is_armed = True
        self._armed_time = time.time()
        self._quad_state = QuadcopterState.ARMED

        # Set idle throttle
        self._set_motor_outputs(
            throttle=self._quad_config.idle_throttle,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        logger.info("Quadcopter '%s' ARMED", self._name)

    def disarm(self) -> None:
        """Disarm the quadcopter motors.

        Motors are stopped immediately. Should only be called on ground.
        """
        if not self._is_armed:
            return

        # Safety check - warn if in flight
        if self.is_flying:
            logger.warning("Disarming while in flight! Motors will stop.")

        # Stop motors
        self._motor_outputs = MotorOutputs()
        if self._motors:
            for motor in self._motors:
                motor.set_throttle(0.0)
                motor.disable()

        # Update state
        self._is_armed = False
        self._armed_time = None
        self._takeoff_time = None
        self._quad_state = QuadcopterState.DISARMED

        logger.info("Quadcopter '%s' DISARMED", self._name)

    # -------------------------------------------------------------------------
    # Flight Commands
    # -------------------------------------------------------------------------

    def takeoff(self, altitude: float = 1.5) -> None:
        """Take off to specified altitude.

        Args:
            altitude: Target altitude in meters

        Raises:
            DisabledError: If not enabled
            SafetyError: If not armed or already flying
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_armed:
            raise SafetyError("Must be armed to takeoff")
        if self.is_flying:
            raise SafetyError("Already in flight")

        # Validate altitude
        if altitude <= 0:
            raise ValueError("Altitude must be positive")
        if altitude > self._quad_config.max_altitude:
            altitude = self._quad_config.max_altitude
            logger.warning("Altitude clamped to max: %.1fm", altitude)

        self._quad_state = QuadcopterState.TAKING_OFF
        self._flight_mode = FlightMode.TAKEOFF
        self._takeoff_time = time.time()
        self._target_position = Position3D(
            x=self._position.x,
            y=self._position.y,
            z=altitude,
            yaw=self._position.yaw,
        )

        # Set takeoff throttle
        self._set_motor_outputs(
            throttle=self._quad_config.hover_throttle + 0.1,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        # In simulation, immediately update altitude
        if self._simulated:
            self._position = Position3D(
                x=self._position.x,
                y=self._position.y,
                z=altitude,
                yaw=self._position.yaw,
            )
            self._quad_state = QuadcopterState.HOVERING
            self._flight_mode = FlightMode.ALT_HOLD

        logger.info("Taking off to %.1fm", altitude)

    def land(self) -> None:
        """Land the quadcopter.

        Raises:
            DisabledError: If not enabled
            SafetyError: If not in flight
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self.is_flying and self._quad_state != QuadcopterState.ARMED:
            raise SafetyError("Not in flight")

        self._quad_state = QuadcopterState.LANDING
        self._flight_mode = FlightMode.LAND
        self._target_position = Position3D(
            x=self._position.x,
            y=self._position.y,
            z=0.0,
            yaw=self._position.yaw,
        )

        # Reduce throttle for descent
        self._set_motor_outputs(
            throttle=self._quad_config.hover_throttle - 0.15,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        # In simulation, immediately land
        if self._simulated:
            self._position = Position3D(
                x=self._position.x,
                y=self._position.y,
                z=0.0,
                yaw=self._position.yaw,
            )
            self._quad_state = QuadcopterState.ARMED
            self._flight_mode = FlightMode.STABILIZE
            self._takeoff_time = None

        logger.info("Landing initiated")

    def hover(self) -> None:
        """Enter hover mode at current position.

        Raises:
            DisabledError: If not enabled
            SafetyError: If not in flight
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self.is_flying:
            raise SafetyError("Not in flight")

        self._quad_state = QuadcopterState.HOVERING
        self._flight_mode = FlightMode.ALT_HOLD
        self._target_position = Position3D(
            x=self._position.x,
            y=self._position.y,
            z=self._position.z,
            yaw=self._position.yaw,
        )

        # Set hover throttle
        self._set_motor_outputs(
            throttle=self._quad_config.hover_throttle,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        logger.info("Hovering at %.1fm", self._position.z)

    def set_attitude(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        throttle: float | None = None,
    ) -> None:
        """Set target attitude.

        Args:
            roll: Roll angle in degrees (positive = right bank)
            pitch: Pitch angle in degrees (positive = nose up)
            yaw: Yaw angle in degrees (absolute heading)
            throttle: Throttle level (0.0 to 1.0), None for current

        Raises:
            DisabledError: If not enabled
            SafetyError: If not armed
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_armed:
            raise SafetyError("Must be armed")

        # Clamp to limits
        max_tilt = self._quad_config.max_tilt_angle
        roll = max(-max_tilt, min(max_tilt, roll))
        pitch = max(-max_tilt, min(max_tilt, pitch))
        yaw = yaw % 360

        # Normalize to control inputs (-1 to 1)
        roll_input = roll / max_tilt
        pitch_input = pitch / max_tilt
        yaw_input = (yaw - self._attitude.yaw) / self._quad_config.max_yaw_rate
        yaw_input = max(-1.0, min(1.0, yaw_input))

        # Use current or specified throttle
        if throttle is None:
            throttle = self._quad_config.hover_throttle
        throttle = max(0.0, min(1.0, throttle))

        self._quad_state = QuadcopterState.FLYING
        self._set_motor_outputs(throttle, roll_input, pitch_input, yaw_input)

        # Update simulated attitude
        if self._simulated:
            self._attitude = Attitude(roll=roll, pitch=pitch, yaw=yaw)

    def set_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        """Set target velocity.

        Args:
            vx: Forward velocity in m/s (positive = forward)
            vy: Lateral velocity in m/s (positive = right)
            vz: Vertical velocity in m/s (positive = up)
            yaw_rate: Yaw rate in degrees/second (positive = clockwise)

        Raises:
            DisabledError: If not enabled
            SafetyError: If not armed
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_armed:
            raise SafetyError("Must be armed")

        # Clamp velocities
        max_speed = self._quad_config.max_speed
        vx = max(-max_speed, min(max_speed, vx))
        vy = max(-max_speed, min(max_speed, vy))
        vz = max(-self._quad_config.max_descent_rate, min(self._quad_config.max_climb_rate, vz))
        yaw_rate = max(
            -self._quad_config.max_yaw_rate, min(self._quad_config.max_yaw_rate, yaw_rate)
        )

        # Store velocity
        self._velocity = Velocity(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

        # Convert to attitude commands (simplified)
        # Pitch for forward/back, roll for left/right
        pitch = -vx / max_speed * self._quad_config.max_tilt_angle
        roll = vy / max_speed * self._quad_config.max_tilt_angle

        # Throttle adjustment for vertical
        throttle = self._quad_config.hover_throttle
        throttle += vz / self._quad_config.max_climb_rate * 0.2

        # Yaw input
        yaw_input = yaw_rate / self._quad_config.max_yaw_rate

        self._quad_state = QuadcopterState.FLYING
        self._set_motor_outputs(
            throttle=throttle,
            roll=roll / self._quad_config.max_tilt_angle,
            pitch=pitch / self._quad_config.max_tilt_angle,
            yaw=yaw_input,
        )

    def goto(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float | None = None,
    ) -> None:
        """Go to specified position.

        Args:
            x: Target X position in meters
            y: Target Y position in meters
            z: Target Z position (altitude) in meters
            yaw: Target yaw in degrees (None = maintain current)

        Raises:
            DisabledError: If not enabled
            SafetyError: If not in flight
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self.is_flying:
            raise SafetyError("Must be in flight for goto")

        # Validate altitude
        z = max(0.5, min(self._quad_config.max_altitude, z))

        # Check geofence
        distance = math.sqrt(x**2 + y**2)
        if distance > self._quad_config.geofence_radius:
            raise SafetyError(
                f"Position outside geofence ({distance:.1f}m > {self._quad_config.geofence_radius}m)"
            )

        # Set target
        self._target_position = Position3D(
            x=x,
            y=y,
            z=z,
            yaw=yaw if yaw is not None else self._position.yaw,
        )

        self._quad_state = QuadcopterState.FLYING
        self._flight_mode = FlightMode.GUIDED

        # In simulation, immediately update position
        if self._simulated:
            self._position = Position3D(
                x=x,
                y=y,
                z=z,
                yaw=yaw if yaw is not None else self._position.yaw,
            )

        logger.info("Going to position (%.1f, %.1f, %.1f)", x, y, z)

    def return_to_launch(self) -> None:
        """Return to launch/home position.

        Raises:
            DisabledError: If not enabled
            SafetyError: If not in flight
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self.is_flying:
            raise SafetyError("Must be in flight for RTL")

        self._flight_mode = FlightMode.RTL
        self._target_position = Position3D(x=0, y=0, z=self._position.z, yaw=0)

        # In simulation, go home
        if self._simulated:
            self._position = Position3D(x=0, y=0, z=self._position.z, yaw=0)

        logger.info("Returning to launch")

    # -------------------------------------------------------------------------
    # Motor Control
    # -------------------------------------------------------------------------

    def _set_motor_outputs(
        self,
        throttle: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> None:
        """Calculate and apply motor outputs.

        Args:
            throttle: Base throttle (0.0 to 1.0)
            roll: Roll control (-1.0 to 1.0)
            pitch: Pitch control (-1.0 to 1.0)
            yaw: Yaw control (-1.0 to 1.0)
        """
        # Get mixing matrix
        mixer = self._quad_config.mixer_matrix

        # Apply mixing
        outputs = mix_motors(throttle, roll, pitch, yaw, mixer)

        # Normalize and ensure minimum for armed state
        min_output = self._quad_config.idle_throttle if self._is_armed else 0.0
        outputs = normalize_motor_outputs(outputs, min_output)

        self._motor_outputs = outputs

        # Apply to motors
        if self._motors:
            values = outputs.as_list()
            for i, motor in enumerate(self._motors):
                motor.set_throttle(values[i])

    def get_motor_output(self, position: MotorPosition) -> float:
        """Get output for specific motor.

        Args:
            position: Motor position

        Returns:
            Motor output (0.0 to 1.0)
        """
        return self._motor_outputs.as_list()[position.value]

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def status(self) -> QuadcopterStatus:
        """Get current quadcopter status.

        Returns:
            Complete status information
        """
        return QuadcopterStatus(
            state=self._quad_state,
            flight_mode=self._flight_mode,
            is_enabled=self._is_enabled,
            is_armed=self._is_armed,
            attitude=self._attitude,
            velocity=self._velocity,
            position=self._position,
            motor_outputs=self._motor_outputs,
            battery_voltage=self._sim_battery_voltage,
            battery_percent=self._get_battery_percent(),
            gps_fix=self._simulated,
            satellites=12 if self._simulated else 0,
            armed_time=self.armed_time,
            flight_time=self.flight_time,
            error=self._error,
        )

    def _get_battery_percent(self) -> float:
        """Calculate battery percentage."""
        full = self._quad_config.battery_voltage_full
        empty = self._quad_config.battery_voltage_empty
        if full <= empty:
            return 100.0
        percent = (self._sim_battery_voltage - empty) / (full - empty) * 100
        return max(0.0, min(100.0, percent))

    def set_flight_mode(self, mode: FlightMode) -> None:
        """Set flight mode.

        Args:
            mode: Target flight mode

        Raises:
            DisabledError: If not enabled
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        self._flight_mode = mode
        logger.info("Flight mode set to %s", mode.value)

    def reset_emergency(self) -> None:
        """Reset from emergency state.

        Must be called to recover after emergency stop.
        """
        if self._quad_state == QuadcopterState.EMERGENCY:
            self._quad_state = QuadcopterState.DISARMED
            self._error = None
            logger.info("Emergency state reset")

    # -------------------------------------------------------------------------
    # Integration Methods (ai-infra/svc-infra)
    # -------------------------------------------------------------------------

    def as_tools(self) -> list[dict[str, Any] | Callable[..., Any]]:
        """Generate ai-infra compatible tools for LLM control.

        Returns:
            List of function tools for AI agent
        """

        def arm_drone() -> str:
            """Arm the drone motors for flight."""
            try:
                self.arm()
                return "Drone armed successfully"
            except Exception as e:
                return f"Failed to arm: {e}"

        def disarm_drone() -> str:
            """Disarm the drone motors. Use only when landed."""
            self.disarm()
            return "Drone disarmed"

        def takeoff_to_altitude(altitude: float = 1.5) -> str:
            """Take off to specified altitude in meters.

            Args:
                altitude: Target altitude (default 1.5m)
            """
            try:
                self.takeoff(altitude)
                return f"Taking off to {altitude}m"
            except Exception as e:
                return f"Takeoff failed: {e}"

        def land_drone() -> str:
            """Land the drone at current position."""
            try:
                self.land()
                return "Landing initiated"
            except Exception as e:
                return f"Landing failed: {e}"

        def goto_position(x: float, y: float, z: float) -> str:
            """Move drone to specified position.

            Args:
                x: Forward/back position in meters
                y: Left/right position in meters
                z: Altitude in meters
            """
            try:
                self.goto(x, y, z)
                return f"Moving to ({x}, {y}, {z})"
            except Exception as e:
                return f"Goto failed: {e}"

        def get_drone_status() -> dict:
            """Get current drone status."""
            status = self.status()
            return {
                "state": status.state.value,
                "is_armed": status.is_armed,
                "altitude": status.position.z,
                "battery_percent": status.battery_percent,
                "flight_mode": status.flight_mode.value,
            }

        def emergency_stop_drone() -> str:
            """EMERGENCY: Immediately stop all motors. Use only in emergency."""
            self.stop()
            return "EMERGENCY STOP EXECUTED"

        return [
            arm_drone,
            disarm_drone,
            takeoff_to_altitude,
            land_drone,
            goto_position,
            get_drone_status,
            emergency_stop_drone,
        ]


# =============================================================================
# Factory Function
# =============================================================================


def create_quadcopter(
    name: str = "quadcopter",
    *,
    frame_type: str = "X",
    arm_length: float = 0.25,
    simulated: bool = True,
) -> Quadcopter:
    """Create a quadcopter with default configuration.

    Args:
        name: Quadcopter name
        frame_type: Frame configuration (X, +, H)
        arm_length: Arm length in meters
        simulated: Use simulated motors

    Returns:
        Configured Quadcopter controller

    Example:
        >>> quad = create_quadcopter("my_drone", frame_type="X")
        >>> quad.enable()
        >>> quad.arm()
    """
    config = QuadcopterConfig(
        name=name,
        frame_type=frame_type,
        arm_length=arm_length,
    )

    return Quadcopter(
        name=name,
        config=config,
        simulated=simulated,
    )
