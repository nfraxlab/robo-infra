"""JointGroup controller for coordinating multiple joints as a robot arm.

This module provides a JointGroup controller that coordinates multiple
actuators (typically servos) as a unified robot arm or multi-joint system.

Example:
    >>> from robo_infra.controllers.joint_group import JointGroup, JointGroupConfig
    >>> from robo_infra.actuators.servo import Servo
    >>>
    >>> # Create joints
    >>> joints = {
    ...     "shoulder": Servo(name="shoulder", angle_range=(0, 180)),
    ...     "elbow": Servo(name="elbow", angle_range=(0, 180)),
    ...     "wrist": Servo(name="wrist", angle_range=(0, 180)),
    ... }
    >>>
    >>> # Create arm controller
    >>> arm = JointGroup(name="robot_arm", joints=joints)
    >>> arm.enable()
    >>> arm.home()
    >>>
    >>> # Move to position
    >>> arm.move_to({"shoulder": 45, "elbow": 90, "wrist": 135})
    >>>
    >>> # Save and recall positions
    >>> arm.save_position("ready")
    >>> arm.go_to_named("ready")
"""

from __future__ import annotations

import asyncio
import logging
import time
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator, ActuatorConfig
from robo_infra.core.controller import Controller, ControllerConfig, Position
from robo_infra.core.exceptions import (
    DisabledError,
    LimitsExceededError,
)


if TYPE_CHECKING:
    from collections.abc import Generator


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class JointGroupState(Enum):
    """States a JointGroup can be in."""

    IDLE = "idle"
    MOVING = "moving"
    HOMING = "homing"
    ERROR = "error"
    DISABLED = "disabled"


# =============================================================================
# Configuration Models
# =============================================================================


class JointGroupConfig(BaseModel):
    """Configuration for a JointGroup controller.

    Attributes:
        name: Human-readable name for the joint group.
        joints: Configuration for each joint actuator.
        home_positions: Default home position for each joint.
        named_positions: Dictionary of named positions for quick recall.
        interpolation_steps: Number of steps for motion interpolation.
        default_speed: Default movement speed (0.0 to 1.0).

    Example:
        >>> config = JointGroupConfig(
        ...     name="robot_arm",
        ...     home_positions={"shoulder": 90, "elbow": 90, "wrist": 90},
        ...     named_positions={
        ...         "ready": {"shoulder": 45, "elbow": 90, "wrist": 135},
        ...         "rest": {"shoulder": 0, "elbow": 0, "wrist": 0},
        ...     },
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Joint group name")
    description: str = Field(default="", description="Human-readable description")

    # Joint configurations (optional, can be derived from actuators)
    joints: dict[str, ActuatorConfig] = Field(
        default_factory=dict,
        description="Configuration for each joint actuator",
    )

    # Position presets
    home_positions: dict[str, float] = Field(
        default_factory=dict,
        description="Home position for each joint",
    )
    named_positions: dict[str, dict[str, float]] = Field(
        default_factory=dict,
        description="Named positions for quick recall",
    )

    # Motion parameters
    interpolation_steps: int = Field(
        default=50,
        ge=1,
        le=1000,
        description="Number of steps for motion interpolation",
    )
    default_speed: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Default movement speed (0.0 to 1.0)",
    )
    move_timeout: float = Field(
        default=10.0,
        gt=0,
        description="Maximum time for a move operation in seconds",
    )

    # Safety
    clamp_to_limits: bool = Field(
        default=True,
        description="Clamp values to joint limits instead of raising error",
    )
    enable_collision_check: bool = Field(
        default=False,
        description="Enable self-collision checking (requires IK)",
    )


# =============================================================================
# JointGroup Controller
# =============================================================================


class JointGroup(Controller):
    """Controller for coordinating multiple joints as a robot arm.

    JointGroup provides high-level control for multi-joint robots like
    robot arms, pan-tilt mechanisms, and articulated systems.

    Features:
    - Synchronized multi-joint movement
    - Motion interpolation for smooth trajectories
    - Named position presets
    - Home/stop/disable functionality
    - Async movement support

    Example:
        >>> joints = {
        ...     "base": Servo(name="base", angle_range=(0, 180)),
        ...     "shoulder": Servo(name="shoulder", angle_range=(0, 180)),
        ...     "elbow": Servo(name="elbow", angle_range=(0, 180)),
        ... }
        >>> arm = JointGroup(name="arm", joints=joints)
        >>> arm.enable()
        >>> arm.move_to({"base": 90, "shoulder": 45, "elbow": 120})
    """

    def __init__(
        self,
        name: str,
        joints: dict[str, Actuator],
        *,
        config: JointGroupConfig | None = None,
    ) -> None:
        """Initialize JointGroup controller.

        Args:
            name: Controller name
            joints: Dictionary of joint name to Actuator
            config: Optional configuration

        Raises:
            ValueError: If joints dict is empty
        """
        if not joints:
            raise ValueError("JointGroup requires at least one joint")

        # Create config if not provided
        if config is None:
            config = JointGroupConfig(name=name)

        # Initialize base controller
        super().__init__(name, config=ControllerConfig(name=name))

        # Store JointGroup-specific config
        self._jg_config = config

        # Store joints
        self._joints = joints.copy()

        # Add joints as actuators to base controller
        for joint_name, actuator in joints.items():
            self.add_actuator(joint_name, actuator)

        # JointGroup-specific state
        self._jg_state = JointGroupState.DISABLED
        self._is_moving = False
        self._move_task: asyncio.Task[None] | None = None

        # Initialize named positions from config
        for pos_name, pos_values in config.named_positions.items():
            self._positions[pos_name] = Position(name=pos_name, values=pos_values)

        logger.debug(
            "JointGroup '%s' initialized with %d joints: %s",
            name,
            len(joints),
            list(joints.keys()),
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def joints(self) -> dict[str, Actuator]:
        """Dictionary of joints by name."""
        return self._joints

    @property
    def joint_names(self) -> list[str]:
        """List of joint names."""
        return list(self._joints.keys())

    @property
    def jg_state(self) -> JointGroupState:
        """Current JointGroup state."""
        return self._jg_state

    @property
    def is_moving(self) -> bool:
        """Whether the joint group is currently moving."""
        return self._is_moving

    @property
    def jg_config(self) -> JointGroupConfig:
        """JointGroup-specific configuration."""
        return self._jg_config

    # -------------------------------------------------------------------------
    # Position Management
    # -------------------------------------------------------------------------

    def get_positions(self) -> dict[str, float]:
        """Get current position of all joints.

        Returns:
            Dictionary mapping joint name to current position

        Example:
            >>> positions = arm.get_positions()
            >>> print(positions)
            {'shoulder': 90.0, 'elbow': 45.0, 'wrist': 135.0}
        """
        return {name: actuator.get() for name, actuator in self._joints.items()}

    def get_joint_position(self, name: str) -> float:
        """Get current position of a specific joint.

        Args:
            name: Joint name

        Returns:
            Current position value

        Raises:
            KeyError: If joint name doesn't exist
        """
        if name not in self._joints:
            raise KeyError(f"Unknown joint: '{name}'. Available: {self.joint_names}")
        return self._joints[name].get()

    # -------------------------------------------------------------------------
    # Movement Methods
    # -------------------------------------------------------------------------

    def move_joints(
        self,
        positions: dict[str, float],
        *,
        speed: float = 1.0,
        interpolate: bool = True,
    ) -> None:
        """Move joints to specified positions.

        Args:
            positions: Dictionary mapping joint names to target positions
            speed: Movement speed factor (0.0 to 1.0)
            interpolate: Whether to use motion interpolation

        Raises:
            DisabledError: If controller is not enabled
            KeyError: If unknown joint name provided
            LimitsExceededError: If position exceeds limits (when clamp disabled)

        Example:
            >>> arm.move_to({"shoulder": 45, "elbow": 90})
            >>> arm.move_to({"wrist": 135}, speed=0.5)
        """
        if not self.is_enabled:
            raise DisabledError("JointGroup must be enabled before moving")

        # Validate joint names
        for name in positions:
            if name not in self._joints:
                raise KeyError(f"Unknown joint: '{name}'. Available: {self.joint_names}")

        # Start movement
        self._is_moving = True
        self._jg_state = JointGroupState.MOVING

        try:
            if interpolate and len(positions) > 0:
                # Interpolated movement
                self._execute_interpolated_move(positions, speed)
            else:
                # Direct movement
                self._execute_direct_move(positions)
        finally:
            self._is_moving = False
            if self._jg_state == JointGroupState.MOVING:
                self._jg_state = JointGroupState.IDLE

    async def amove_joints(
        self,
        positions: dict[str, float],
        *,
        speed: float = 1.0,
        interpolate: bool = True,
    ) -> None:
        """Async version of move_joints.

        Args:
            positions: Dictionary mapping joint names to target positions
            speed: Movement speed factor (0.0 to 1.0)
            interpolate: Whether to use motion interpolation

        Raises:
            DisabledError: If controller is not enabled
            KeyError: If unknown joint name provided
            LimitsExceededError: If position exceeds limits (when clamp disabled)

        Example:
            >>> await arm.amove_to({"shoulder": 45, "elbow": 90})
        """
        if not self.is_enabled:
            raise DisabledError("JointGroup must be enabled before moving")

        # Validate joint names
        for name in positions:
            if name not in self._joints:
                raise KeyError(f"Unknown joint: '{name}'. Available: {self.joint_names}")

        # Start movement
        self._is_moving = True
        self._jg_state = JointGroupState.MOVING

        try:
            if interpolate and len(positions) > 0:
                # Interpolated movement with async delays
                await self._execute_interpolated_move_async(positions, speed)
            else:
                # Direct movement
                self._execute_direct_move(positions)
        finally:
            self._is_moving = False
            if self._jg_state == JointGroupState.MOVING:
                self._jg_state = JointGroupState.IDLE

    def _execute_direct_move(self, positions: dict[str, float]) -> None:
        """Execute a direct move without interpolation.

        Args:
            positions: Target positions for joints
        """
        for name, target in positions.items():
            actuator = self._joints[name]
            clamped_target = target

            # Clamp or raise based on config
            if hasattr(actuator, "limits") and actuator.limits:
                if target < actuator.limits.min:
                    if self._jg_config.clamp_to_limits:
                        clamped_target = actuator.limits.min
                        logger.warning("Joint '%s' clamped to min: %f", name, clamped_target)
                    else:
                        raise LimitsExceededError(
                            value=target,
                            min_limit=actuator.limits.min,
                            max_limit=actuator.limits.max,
                            name=name,
                        )
                elif target > actuator.limits.max:
                    if self._jg_config.clamp_to_limits:
                        clamped_target = actuator.limits.max
                        logger.warning("Joint '%s' clamped to max: %f", name, clamped_target)
                    else:
                        raise LimitsExceededError(
                            value=target,
                            min_limit=actuator.limits.min,
                            max_limit=actuator.limits.max,
                            name=name,
                        )

            actuator.set(clamped_target)

    def _execute_interpolated_move(
        self,
        positions: dict[str, float],
        speed: float,
    ) -> None:
        """Execute interpolated move synchronously.

        Args:
            positions: Target positions for joints
            speed: Movement speed factor
        """
        start_positions = self.get_positions()
        steps = max(1, int(self._jg_config.interpolation_steps * speed))
        step_delay = self._jg_config.move_timeout / steps / 10  # Reasonable delay

        for step_positions in self._interpolate_move(start_positions, positions, steps):
            self._execute_direct_move(step_positions)
            time.sleep(step_delay)

    async def _execute_interpolated_move_async(
        self,
        positions: dict[str, float],
        speed: float,
    ) -> None:
        """Execute interpolated move asynchronously.

        Args:
            positions: Target positions for joints
            speed: Movement speed factor
        """
        start_positions = self.get_positions()
        steps = max(1, int(self._jg_config.interpolation_steps * speed))
        step_delay = self._jg_config.move_timeout / steps / 10  # Reasonable delay

        for step_positions in self._interpolate_move(start_positions, positions, steps):
            self._execute_direct_move(step_positions)
            await asyncio.sleep(step_delay)

    def _interpolate_move(
        self,
        start: dict[str, float],
        end: dict[str, float],
        steps: int,
    ) -> Generator[dict[str, float], None, None]:
        """Generate interpolated positions between start and end.

        Uses linear interpolation for smooth movement.

        Args:
            start: Starting positions for all joints
            end: Target positions (may be partial)
            steps: Number of interpolation steps

        Yields:
            Dictionary of positions for each interpolation step
        """
        # For joints not in end, use their current position
        targets = start.copy()
        targets.update(end)

        for i in range(1, steps + 1):
            t = i / steps  # Interpolation factor (0 to 1)
            step_positions = {}

            for name in end:  # Only interpolate joints that are moving
                start_val = start.get(name, targets[name])
                end_val = targets[name]
                step_positions[name] = start_val + (end_val - start_val) * t

            yield step_positions

    # -------------------------------------------------------------------------
    # Named Positions
    # -------------------------------------------------------------------------

    def save_position(self, name: str, metadata: dict[str, Any] | None = None) -> Position:
        """Save current joint positions as a named position.

        Args:
            name: Name for the saved position
            metadata: Optional metadata to attach

        Returns:
            The saved Position object

        Example:
            >>> arm.move_joints({"shoulder": 45, "elbow": 90})
            >>> arm.save_position("ready")
        """
        current = self.get_positions()
        position = Position(name=name, values=current, metadata=metadata or {})
        self._positions[name] = position
        logger.info("Saved position '%s': %s", name, current)
        return position

    def go_to_named(self, name: str, *, speed: float = 1.0) -> None:
        """Move to a previously saved named position.

        Args:
            name: Name of the saved position
            speed: Movement speed factor

        Raises:
            KeyError: If position name doesn't exist

        Example:
            >>> arm.go_to_named("ready")
        """
        if name not in self._positions:
            available = list(self._positions.keys())
            raise KeyError(f"Unknown position: '{name}'. Available: {available}")

        position = self._positions[name]
        self.move_joints(position.values, speed=speed)

    async def ago_to_named(self, name: str, *, speed: float = 1.0) -> None:
        """Async version of go_to_named.

        Args:
            name: Name of the saved position
            speed: Movement speed factor

        Raises:
            KeyError: If position name doesn't exist
        """
        if name not in self._positions:
            available = list(self._positions.keys())
            raise KeyError(f"Unknown position: '{name}'. Available: {available}")

        position = self._positions[name]
        await self.amove_joints(position.values, speed=speed)

    def get_named_positions(self) -> dict[str, dict[str, float]]:
        """Get all named positions.

        Returns:
            Dictionary mapping position names to joint values
        """
        return {name: pos.values for name, pos in self._positions.items()}

    # -------------------------------------------------------------------------
    # Control Methods
    # -------------------------------------------------------------------------

    def home(self) -> None:
        """Move all joints to their home positions.

        Uses home_positions from config if set, otherwise uses
        each joint's default value.

        Raises:
            DisabledError: If controller is not enabled
        """
        if not self.is_enabled:
            raise DisabledError("JointGroup must be enabled before homing")

        self._jg_state = JointGroupState.HOMING

        try:
            home_positions = {}

            for name, actuator in self._joints.items():
                # Use config home position or actuator default
                if name in self._jg_config.home_positions:
                    home_positions[name] = self._jg_config.home_positions[name]
                elif (
                    hasattr(actuator, "limits")
                    and actuator.limits
                    and actuator.limits.default is not None
                ):
                    home_positions[name] = actuator.limits.default
                elif hasattr(actuator, "default_value"):
                    home_positions[name] = actuator.default_value
                else:
                    # Skip joints without a defined home
                    continue

            if home_positions:
                self.move_joints(home_positions, speed=0.5)

            self._is_homed = True
            logger.info("JointGroup '%s' homed to: %s", self.name, home_positions)
        finally:
            if self._jg_state == JointGroupState.HOMING:
                self._jg_state = JointGroupState.IDLE

    def stop(self) -> None:
        """Stop all joint movement immediately.

        Cancels any ongoing interpolated movement and stops
        all actuators at their current position.
        """
        self._is_moving = False

        # Cancel async move task if running
        if self._move_task and not self._move_task.done():
            self._move_task.cancel()
            self._move_task = None

        # Stop all actuators
        for actuator in self._joints.values():
            if hasattr(actuator, "stop"):
                actuator.stop()

        self._jg_state = JointGroupState.IDLE
        logger.info("JointGroup '%s' stopped", self.name)

    def disable(self) -> None:
        """Disable the joint group and all joints.

        Stops all movement and disables all actuators.
        """
        self.stop()

        for actuator in self._joints.values():
            if hasattr(actuator, "disable"):
                actuator.disable()

        self._is_enabled = False
        self._jg_state = JointGroupState.DISABLED
        self._on_disable()
        logger.info("JointGroup '%s' disabled", self.name)

    def enable(self) -> None:
        """Enable the joint group and all joints.

        Enables all actuators and prepares for movement.
        """
        for actuator in self._joints.values():
            if hasattr(actuator, "enable"):
                actuator.enable()

        self._is_enabled = True
        self._jg_state = JointGroupState.IDLE
        self._start_time = time.time()
        self._on_enable()
        logger.info("JointGroup '%s' enabled", self.name)

    # -------------------------------------------------------------------------
    # Controller Abstract Methods
    # -------------------------------------------------------------------------

    def _do_home(self) -> None:
        """Implement abstract home method from Controller."""
        self.home()

    def _do_stop(self) -> None:
        """Implement abstract stop method from Controller."""
        self.stop()

    # -------------------------------------------------------------------------
    # String Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"JointGroup(name='{self.name}', "
            f"joints={self.joint_names}, "
            f"state={self._jg_state.value})"
        )
