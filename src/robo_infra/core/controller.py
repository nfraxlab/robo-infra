"""Controller abstractions for coordinating actuators and sensors.

This module provides abstract base classes for building robot controllers
that coordinate multiple actuators and sensors as a unified system.

Example:
    >>> from robo_infra.core.controller import Controller, SimulatedController
    >>> from robo_infra.core.actuator import SimulatedActuator
    >>> from robo_infra.core.sensor import SimulatedSensor
    >>> from robo_infra.core.types import Limits
    >>>
    >>> # Create a controller with actuators and sensors
    >>> controller = SimulatedController(name="arm")
    >>> controller.add_actuator("shoulder", SimulatedActuator(
    ...     name="shoulder",
    ...     limits=Limits(min=0, max=180, default=90),
    ... ))
    >>> controller.add_sensor("encoder", SimulatedSensor(
    ...     name="encoder",
    ...     limits=Limits(min=0, max=360),
    ... ))
    >>> controller.enable()
    >>> controller.home()
"""

from __future__ import annotations

import asyncio
import logging
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.exceptions import DisabledError, SafetyError


if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from robo_infra.core.actuator import Actuator, ActuatorStatus
    from robo_infra.core.sensor import Sensor, SensorStatus


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class ControllerState(Enum):
    """Controller operational state."""

    DISABLED = "disabled"
    IDLE = "idle"
    HOMING = "homing"
    MOVING = "moving"
    RUNNING = "running"
    STOPPED = "stopped"  # Emergency stop state
    ERROR = "error"


class ControllerMode(Enum):
    """Controller operating mode."""

    MANUAL = "manual"  # Direct control
    AUTOMATIC = "automatic"  # Following a program/trajectory
    REMOTE = "remote"  # Controlled via API
    TEACH = "teach"  # Recording positions


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class ControllerStatus:
    """Current status of a controller."""

    state: ControllerState
    mode: ControllerMode
    is_enabled: bool
    is_homed: bool
    is_running: bool
    error: str | None = None
    actuator_count: int = 0
    sensor_count: int = 0
    uptime: float = 0.0


@dataclass
class Position:
    """A named position/pose for the controller."""

    name: str
    values: dict[str, float]
    metadata: dict[str, Any] = field(default_factory=dict)

    def __repr__(self) -> str:
        """String representation."""
        return f"Position(name='{self.name}', values={self.values})"


@dataclass
class MotionConfig:
    """Configuration for motion execution."""

    speed: float = 1.0  # 0.0 to 1.0, relative speed
    acceleration: float = 1.0  # 0.0 to 1.0, relative accel
    interpolation: str = "linear"  # linear, joint, cubic
    timeout: float = 30.0  # Maximum time for motion
    blocking: bool = True  # Wait for completion


# =============================================================================
# Pydantic Config Model
# =============================================================================


class ControllerConfig(BaseModel):
    """Pydantic configuration model for controllers.

    Allows loading controller configuration from YAML/JSON/dict.

    Example:
        >>> config = ControllerConfig(
        ...     name="robot_arm",
        ...     description="6-DOF Robot Arm Controller",
        ...     home_on_enable=True,
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Controller name")
    description: str = Field(default="", description="Human-readable description")
    mode: ControllerMode = Field(default=ControllerMode.MANUAL, description="Operating mode")

    # Behavior
    home_on_enable: bool = Field(default=False, description="Auto-home when enabled")
    disable_on_error: bool = Field(default=True, description="Auto-disable on error")
    enable_safety_limits: bool = Field(default=True, description="Enable safety checks")

    # Motion defaults
    default_speed: float = Field(default=0.5, ge=0, le=1, description="Default motion speed")
    default_acceleration: float = Field(default=0.5, ge=0, le=1, description="Default acceleration")

    # Timing
    loop_rate: float = Field(default=100.0, gt=0, description="Control loop rate in Hz")
    watchdog_timeout: float = Field(default=1.0, gt=0, description="Watchdog timeout in seconds")

    # Metadata
    metadata: dict[str, Any] = Field(default_factory=dict, description="Additional metadata")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ControllerConfig:
        """Create config from dictionary."""
        if "mode" in data and isinstance(data["mode"], str):
            data["mode"] = ControllerMode(data["mode"])
        return cls(**data)


# =============================================================================
# Abstract Base Class
# =============================================================================


class Controller(ABC):
    """Abstract base class for robot controllers.

    Controllers coordinate multiple actuators and sensors to perform
    coordinated movements and respond to sensor feedback.

    Subclasses must implement:
    - `_do_home()`: Perform homing sequence
    - `_do_stop()`: Perform emergency stop

    Optionally override:
    - `_on_enable()`: Called when controller is enabled
    - `_on_disable()`: Called when controller is disabled
    - `_control_loop()`: Main control loop logic

    Example:
        >>> class MyArm(Controller):
        ...     def _do_home(self) -> None:
        ...         for actuator in self.actuators.values():
        ...             actuator.go_to_default()
        ...
        ...     def _do_stop(self) -> None:
        ...         for actuator in self.actuators.values():
        ...             actuator.disable()
    """

    def __init__(
        self,
        name: str,
        *,
        config: ControllerConfig | None = None,
    ) -> None:
        """Initialize controller.

        Args:
            name: Controller name
            config: Optional configuration
        """
        self._name = name
        self._config = config or ControllerConfig(name=name)

        # Components
        self._actuators: dict[str, Actuator] = {}
        self._sensors: dict[str, Sensor] = {}

        # State
        self._state = ControllerState.DISABLED
        self._mode = self._config.mode
        self._is_enabled = False
        self._is_homed = False
        self._is_running = False
        self._error: str | None = None
        self._start_time: float | None = None

        # Named positions
        self._positions: dict[str, Position] = {}

        # Callbacks
        self._on_state_change: list[Callable[[ControllerState], None]] = []
        self._on_error: list[Callable[[str], None]] = []

        # Control loop
        self._control_task: asyncio.Task[None] | None = None
        self._stop_requested = False

        logger.debug("Controller '%s' initialized", name)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def name(self) -> str:
        """Controller name."""
        return self._name

    @property
    def config(self) -> ControllerConfig:
        """Controller configuration."""
        return self._config

    @property
    def state(self) -> ControllerState:
        """Current controller state."""
        return self._state

    @property
    def mode(self) -> ControllerMode:
        """Current operating mode."""
        return self._mode

    @property
    def is_enabled(self) -> bool:
        """Whether controller is enabled."""
        return self._is_enabled

    @property
    def is_homed(self) -> bool:
        """Whether controller has been homed."""
        return self._is_homed

    @property
    def is_running(self) -> bool:
        """Whether control loop is running."""
        return self._is_running

    @property
    def actuators(self) -> dict[str, Actuator]:
        """Dictionary of actuators by name."""
        return self._actuators

    @property
    def sensors(self) -> dict[str, Sensor]:
        """Dictionary of sensors by name."""
        return self._sensors

    @property
    def positions(self) -> dict[str, Position]:
        """Dictionary of named positions."""
        return self._positions

    @property
    def uptime(self) -> float:
        """Time since controller was enabled, in seconds."""
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

    # -------------------------------------------------------------------------
    # Abstract Methods
    # -------------------------------------------------------------------------

    @abstractmethod
    def _do_home(self) -> None:
        """Perform homing sequence.

        Must be implemented by subclasses.
        Should move all actuators to their home/default positions.
        """

    @abstractmethod
    def _do_stop(self) -> None:
        """Perform emergency stop.

        Must be implemented by subclasses.
        Should immediately stop all motion and disable actuators.
        """

    # -------------------------------------------------------------------------
    # Optional Override Methods
    # -------------------------------------------------------------------------

    def _on_enable(self) -> None:  # noqa: B027
        """Called when controller is enabled.

        Override to add custom enable logic.
        """

    def _on_disable(self) -> None:  # noqa: B027
        """Called when controller is disabled.

        Override to add custom disable logic.
        """

    def _control_loop_step(self) -> None:  # noqa: B027
        """Single step of the control loop.

        Override to implement custom control logic.
        Called at the configured loop rate.
        """

    # -------------------------------------------------------------------------
    # Component Management
    # -------------------------------------------------------------------------

    def add_actuator(self, name: str, actuator: Actuator) -> None:
        """Add an actuator to the controller.

        Args:
            name: Name to reference the actuator
            actuator: Actuator instance
        """
        self._actuators[name] = actuator
        logger.debug("Added actuator '%s' to controller '%s'", name, self._name)

    def remove_actuator(self, name: str) -> Actuator | None:
        """Remove an actuator from the controller.

        Args:
            name: Actuator name

        Returns:
            Removed actuator or None if not found
        """
        return self._actuators.pop(name, None)

    def get_actuator(self, name: str) -> Actuator | None:
        """Get an actuator by name.

        Args:
            name: Actuator name

        Returns:
            Actuator or None if not found
        """
        return self._actuators.get(name)

    def add_sensor(self, name: str, sensor: Sensor) -> None:
        """Add a sensor to the controller.

        Args:
            name: Name to reference the sensor
            sensor: Sensor instance
        """
        self._sensors[name] = sensor
        logger.debug("Added sensor '%s' to controller '%s'", name, self._name)

    def remove_sensor(self, name: str) -> Sensor | None:
        """Remove a sensor from the controller.

        Args:
            name: Sensor name

        Returns:
            Removed sensor or None if not found
        """
        return self._sensors.pop(name, None)

    def get_sensor(self, name: str) -> Sensor | None:
        """Get a sensor by name.

        Args:
            name: Sensor name

        Returns:
            Sensor or None if not found
        """
        return self._sensors.get(name)

    # -------------------------------------------------------------------------
    # Enable/Disable
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the controller and all components."""
        if self._is_enabled:
            return

        try:
            # Enable all actuators
            for actuator in self._actuators.values():
                actuator.enable()

            # Enable all sensors
            for sensor in self._sensors.values():
                sensor.enable()

            self._is_enabled = True
            self._start_time = time.time()
            self._set_state(ControllerState.IDLE)
            self._on_enable()

            # Auto-home if configured
            if self._config.home_on_enable and not self._is_homed:
                self.home()

            logger.info("Controller '%s' enabled", self._name)

        except Exception as e:
            self._handle_error(f"Enable failed: {e}")
            raise

    def disable(self) -> None:
        """Disable the controller and all components."""
        if not self._is_enabled:
            return

        # Stop control loop if running
        self._stop_requested = True

        # Disable all actuators
        for actuator in self._actuators.values():
            actuator.disable()

        # Disable all sensors
        for sensor in self._sensors.values():
            sensor.disable()

        self._is_enabled = False
        self._is_running = False
        self._set_state(ControllerState.DISABLED)
        self._on_disable()

        logger.info("Controller '%s' disabled", self._name)

    # -------------------------------------------------------------------------
    # Homing
    # -------------------------------------------------------------------------

    def home(self) -> None:
        """Home the controller - move all actuators to default positions.

        Raises:
            DisabledError: If controller is disabled.
        """
        if not self._is_enabled:
            raise DisabledError(self._name)

        try:
            self._set_state(ControllerState.HOMING)
            logger.info("Controller '%s' homing...", self._name)

            self._do_home()

            self._is_homed = True
            self._set_state(ControllerState.IDLE)
            logger.info("Controller '%s' homed", self._name)

        except Exception as e:
            self._handle_error(f"Homing failed: {e}")
            raise

    # -------------------------------------------------------------------------
    # Emergency Stop
    # -------------------------------------------------------------------------

    def stop(self) -> None:
        """Emergency stop - immediately halt all motion.

        This should be called in emergency situations.
        Does not require controller to be enabled.

        Raises:
            SafetyError: If any actuator fails to disable (logged but raised).
        """
        logger.warning("Controller '%s' EMERGENCY STOP", self._name)

        self._stop_requested = True
        self._set_state(ControllerState.STOPPED)

        # Track _do_stop error to propagate after actuator disable attempts
        do_stop_error: Exception | None = None
        try:
            self._do_stop()
        except Exception as e:
            # Log but continue to disable actuators - propagate at end
            logger.error("Error during emergency stop: %s", e)
            do_stop_error = e

        # Disable all actuators - TRACK FAILURES, DON'T SUPPRESS
        failed_actuators: list[str] = []
        for name, actuator in self._actuators.items():
            try:
                actuator.disable()
            except Exception as e:
                failed_actuators.append(name)
                logger.critical("E-STOP FAILED to disable actuator '%s': %s", name, e)

        self._is_running = False

        # Raise if _do_stop failed or any actuator failed to disable
        if failed_actuators or do_stop_error:
            from robo_infra.core.exceptions import SafetyError

            error_msg = []
            if do_stop_error:
                error_msg.append(f"_do_stop failed: {do_stop_error}")
            if failed_actuators:
                error_msg.append(
                    f"Failed to disable {len(failed_actuators)} actuators: {failed_actuators}"
                )
            raise SafetyError(
                "; ".join(error_msg),
                action_taken="partial_disable",
            )

    def reset_stop(self) -> None:
        """Reset from emergency stop state.

        Requires re-homing before operation.
        """
        if self._state != ControllerState.STOPPED:
            return

        self._is_homed = False
        self._stop_requested = False
        self._error = None

        if self._is_enabled:
            self._set_state(ControllerState.IDLE)
        else:
            self._set_state(ControllerState.DISABLED)

        logger.info("Controller '%s' stop reset - rehoming required", self._name)

    # -------------------------------------------------------------------------
    # Motion
    # -------------------------------------------------------------------------

    def move_to(
        self,
        targets: dict[str, float],
        *,
        config: MotionConfig | None = None,
    ) -> None:
        """Move actuators to target positions.

        Args:
            targets: Dictionary mapping actuator names to target values
            config: Motion configuration

        Raises:
            DisabledError: If controller is disabled.
            SafetyError: If safety limits are violated.
        """
        if not self._is_enabled:
            raise DisabledError(self._name)

        if self._state == ControllerState.STOPPED:
            raise SafetyError("Controller is in emergency stop state", "Call reset_stop() first")

        config = config or MotionConfig()

        # Validate targets
        for name in targets:
            if name not in self._actuators:
                raise ValueError(f"Unknown actuator: {name}")

        self._set_state(ControllerState.MOVING)

        try:
            # Set each actuator
            for name, value in targets.items():
                actuator = self._actuators[name]
                actuator.set(value)

            self._set_state(ControllerState.IDLE)

        except Exception as e:
            self._handle_error(f"Motion failed: {e}")
            raise

    def move_to_position(self, position_name: str, *, config: MotionConfig | None = None) -> None:
        """Move to a named position.

        Args:
            position_name: Name of the saved position
            config: Motion configuration

        Raises:
            KeyError: If position not found.
        """
        if position_name not in self._positions:
            raise KeyError(f"Position '{position_name}' not found")

        position = self._positions[position_name]
        self.move_to(position.values, config=config)

    # -------------------------------------------------------------------------
    # Position Management
    # -------------------------------------------------------------------------

    def save_position(self, name: str, metadata: dict[str, Any] | None = None) -> Position:
        """Save current actuator positions as a named position.

        Args:
            name: Position name
            metadata: Optional metadata

        Returns:
            Saved Position object
        """
        values = {}
        for actuator_name, actuator in self._actuators.items():
            values[actuator_name] = actuator.get()

        position = Position(name=name, values=values, metadata=metadata or {})
        self._positions[name] = position
        logger.debug("Saved position '%s': %s", name, values)
        return position

    def delete_position(self, name: str) -> bool:
        """Delete a named position.

        Args:
            name: Position name

        Returns:
            True if deleted, False if not found
        """
        if name in self._positions:
            del self._positions[name]
            return True
        return False

    def add_position(self, position: Position) -> None:
        """Add a position directly.

        Args:
            position: Position to add
        """
        self._positions[position.name] = position

    # -------------------------------------------------------------------------
    # Control Loop
    # -------------------------------------------------------------------------

    async def run(self) -> None:
        """Start the control loop.

        Runs asynchronously at the configured loop rate.

        Raises:
            DisabledError: If controller is disabled.
        """
        if not self._is_enabled:
            raise DisabledError(self._name)

        self._is_running = True
        self._stop_requested = False
        self._set_state(ControllerState.RUNNING)
        interval = 1.0 / self._config.loop_rate

        logger.info(
            "Controller '%s' control loop started at %.1f Hz", self._name, self._config.loop_rate
        )

        try:
            while not self._stop_requested:
                loop_start = time.perf_counter()

                # Execute control step
                try:
                    self._control_loop_step()
                except Exception as e:
                    self._handle_error(f"Control loop error: {e}")
                    if self._config.disable_on_error:
                        break

                # Maintain loop rate with jitter detection
                elapsed = time.perf_counter() - loop_start
                if elapsed > interval:
                    logger.warning(
                        "Control loop overrun in '%s': %.1fms > %.1fms target",
                        self._name,
                        elapsed * 1000,
                        interval * 1000,
                    )
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)

        finally:
            self._is_running = False
            if self._state == ControllerState.RUNNING:
                self._set_state(ControllerState.IDLE)
            logger.info("Controller '%s' control loop stopped", self._name)

    def start(self) -> asyncio.Task[None]:
        """Start control loop as a background task.

        Returns:
            The asyncio Task running the control loop.
        """
        self._control_task = asyncio.create_task(self.run())
        return self._control_task

    def stop_loop(self) -> None:
        """Request the control loop to stop gracefully."""
        self._stop_requested = True

    # -------------------------------------------------------------------------
    # Status and Readings
    # -------------------------------------------------------------------------

    def status(self) -> ControllerStatus:
        """Get current controller status.

        Returns:
            ControllerStatus with current state information.
        """
        return ControllerStatus(
            state=self._state,
            mode=self._mode,
            is_enabled=self._is_enabled,
            is_homed=self._is_homed,
            is_running=self._is_running,
            error=self._error,
            actuator_count=len(self._actuators),
            sensor_count=len(self._sensors),
            uptime=self.uptime,
        )

    def get_actuator_statuses(self) -> dict[str, ActuatorStatus]:
        """Get status of all actuators.

        Returns:
            Dictionary mapping actuator names to their status.
        """
        return {name: actuator.status() for name, actuator in self._actuators.items()}

    def get_sensor_statuses(self) -> dict[str, SensorStatus]:
        """Get status of all sensors.

        Returns:
            Dictionary mapping sensor names to their status.
        """
        return {name: sensor.status() for name, sensor in self._sensors.items()}

    def get_actuator_values(self) -> dict[str, float]:
        """Get current values of all actuators.

        Returns:
            Dictionary mapping actuator names to their current values.
        """
        return {name: actuator.get() for name, actuator in self._actuators.items()}

    def read_sensors(self) -> dict[str, float]:
        """Read all sensors and return their values.

        Returns:
            Dictionary mapping sensor names to their current values.
        """
        values = {}
        for name, sensor in self._sensors.items():
            if sensor.is_enabled:
                try:
                    values[name] = sensor.read().value
                except Exception as e:
                    logger.warning("Failed to read sensor '%s': %s", name, e)
        return values

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def on_state_change(self, callback: Callable[[ControllerState], None]) -> None:
        """Register a callback for state changes.

        Args:
            callback: Function called with new state
        """
        self._on_state_change.append(callback)

    def on_error(self, callback: Callable[[str], None]) -> None:
        """Register a callback for errors.

        Args:
            callback: Function called with error message
        """
        self._on_error.append(callback)

    # -------------------------------------------------------------------------
    # Integration Hooks (for ai-infra and svc-infra)
    # -------------------------------------------------------------------------

    def as_tools(self) -> list[dict[str, Any] | Callable[..., Any]]:
        """Export controller as AI tools (stub for ai-infra integration).

        Returns:
            List of tool definitions (dicts) or function tools (Callables) for AI agents.

        Note:
            This is a placeholder. Full implementation will be in Phase 7
            when integrating with ai-infra.
        """
        tools: list[dict[str, Any] | Callable[..., Any]] = []

        # Basic movement tool
        tools.append(
            {
                "name": f"{self._name}_move",
                "description": f"Move {self._name} actuators to target positions",
                "parameters": {
                    "type": "object",
                    "properties": {
                        name: {"type": "number", "description": f"Target position for {name}"}
                        for name in self._actuators
                    },
                },
            }
        )

        # Home tool
        tools.append(
            {
                "name": f"{self._name}_home",
                "description": f"Home {self._name} to default positions",
                "parameters": {"type": "object", "properties": {}},
            }
        )

        # Stop tool
        tools.append(
            {
                "name": f"{self._name}_stop",
                "description": f"Emergency stop {self._name}",
                "parameters": {"type": "object", "properties": {}},
            }
        )

        # Status tool
        tools.append(
            {
                "name": f"{self._name}_status",
                "description": f"Get status of {self._name}",
                "parameters": {"type": "object", "properties": {}},
            }
        )

        return tools

    def as_router(self) -> dict[str, Any]:
        """Export controller as API router (stub for svc-infra integration).

        Returns:
            Router definition dictionary.

        Note:
            This is a placeholder. Full implementation will be in Phase 7
            when integrating with svc-infra.
        """
        return {
            "prefix": f"/{self._name}",
            "tags": [self._name],
            "endpoints": [
                {"method": "GET", "path": "/status", "handler": "status"},
                {"method": "POST", "path": "/enable", "handler": "enable"},
                {"method": "POST", "path": "/disable", "handler": "disable"},
                {"method": "POST", "path": "/home", "handler": "home"},
                {"method": "POST", "path": "/stop", "handler": "stop"},
                {"method": "POST", "path": "/move", "handler": "move_to"},
                {"method": "GET", "path": "/actuators", "handler": "get_actuator_values"},
                {"method": "GET", "path": "/sensors", "handler": "read_sensors"},
            ],
        }

    # -------------------------------------------------------------------------
    # State Management
    # -------------------------------------------------------------------------

    def _set_state(self, state: ControllerState) -> None:
        """Set controller state and notify callbacks.

        Args:
            state: New state
        """
        if state != self._state:
            old_state = self._state
            self._state = state
            logger.debug(
                "Controller '%s' state: %s -> %s", self._name, old_state.value, state.value
            )

            for callback in self._on_state_change:
                try:
                    callback(state)
                except Exception as e:
                    logger.warning("State change callback error: %s", e)

    def _handle_error(self, message: str) -> None:
        """Handle an error condition.

        Args:
            message: Error message
        """
        self._error = message
        self._set_state(ControllerState.ERROR)
        logger.error("Controller '%s' error: %s", self._name, message)

        for callback in self._on_error:
            try:
                callback(message)
            except Exception as e:
                logger.warning("Error callback error: %s", e)

        if self._config.disable_on_error:
            self.disable()

    # -------------------------------------------------------------------------
    # Context Manager
    # -------------------------------------------------------------------------

    def __enter__(self) -> Controller:
        """Enter context manager - enable controller."""
        self.enable()
        return self

    def __exit__(self, exc_type: type | None, exc_val: Exception | None, exc_tb: Any) -> None:
        """Exit context manager - disable controller."""
        self.disable()

    async def __aenter__(self) -> Controller:
        """Async context manager entry - enable controller.

        For controllers with async initialization, override enable_async().
        Default implementation calls sync enable().
        """
        await self.enable_async()
        return self

    async def __aexit__(
        self,
        exc_type: type | None,
        exc_val: Exception | None,
        exc_tb: Any,
    ) -> None:
        """Async context manager exit - disable controller."""
        await self.disable_async()

    async def enable_async(self) -> None:
        """Async version of enable.

        Override this for controllers that need async initialization.
        Default implementation calls sync enable().
        """
        self.enable()

    async def disable_async(self) -> None:
        """Async version of disable.

        Override this for controllers that need async cleanup.
        Default implementation calls sync disable().
        """
        self.disable()

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"<{self.__class__.__name__} name='{self._name}' "
            f"state={self._state.value} actuators={len(self._actuators)} "
            f"sensors={len(self._sensors)}>"
        )


# =============================================================================
# Simulated Implementation
# =============================================================================


class SimulatedController(Controller):
    """Simulated controller for testing and development.

    Provides a full controller implementation that can be used without hardware.

    Example:
        >>> controller = SimulatedController(name="test_arm")
        >>> controller.add_actuator("joint1", SimulatedActuator(...))
        >>> controller.enable()
        >>> controller.home()
        >>> controller.move_to({"joint1": 45.0})
    """

    def __init__(
        self,
        name: str,
        *,
        config: ControllerConfig | None = None,
    ) -> None:
        """Initialize simulated controller.

        Args:
            name: Controller name
            config: Optional configuration
        """
        super().__init__(name, config=config)

    def _do_home(self) -> None:
        """Move all actuators to their default positions."""
        for actuator in self._actuators.values():
            if hasattr(actuator, "go_to_default"):
                actuator.go_to_default()

    def _do_stop(self) -> None:
        """Disable all actuators immediately."""
        for actuator in self._actuators.values():
            actuator.disable()

    @classmethod
    def from_config(cls, config: ControllerConfig) -> SimulatedController:
        """Create controller from configuration.

        Args:
            config: Controller configuration

        Returns:
            Configured SimulatedController instance
        """
        return cls(name=config.name, config=config)


# =============================================================================
# Controller Group
# =============================================================================


class ControllerGroup:
    """Group of controllers for coordinated operation.

    Useful for systems with multiple controllers that need
    coordinated enable/disable and emergency stop.

    Example:
        >>> group = ControllerGroup("robot_system")
        >>> group.add("arm", arm_controller)
        >>> group.add("gripper", gripper_controller)
        >>> group.enable_all()
        >>> group.home_all()
    """

    def __init__(self, name: str) -> None:
        """Initialize controller group.

        Args:
            name: Group name
        """
        self.name = name
        self.controllers: dict[str, Controller] = {}

    def add(self, name: str, controller: Controller) -> None:
        """Add controller to group.

        Args:
            name: Name for the controller in the group
            controller: Controller instance
        """
        self.controllers[name] = controller

    def remove(self, name: str) -> Controller | None:
        """Remove controller from group.

        Args:
            name: Controller name

        Returns:
            Removed controller or None
        """
        return self.controllers.pop(name, None)

    def get(self, name: str) -> Controller | None:
        """Get controller by name.

        Args:
            name: Controller name

        Returns:
            Controller or None
        """
        return self.controllers.get(name)

    def enable_all(self) -> None:
        """Enable all controllers."""
        for controller in self.controllers.values():
            controller.enable()

    def disable_all(self) -> None:
        """Disable all controllers."""
        for controller in self.controllers.values():
            controller.disable()

    def home_all(self) -> None:
        """Home all controllers."""
        for controller in self.controllers.values():
            if controller.is_enabled:
                controller.home()

    def stop_all(self) -> None:
        """Emergency stop all controllers."""
        for controller in self.controllers.values():
            controller.stop()

    def status_all(self) -> dict[str, ControllerStatus]:
        """Get status of all controllers.

        Returns:
            Dictionary mapping names to status.
        """
        return {name: controller.status() for name, controller in self.controllers.items()}

    def __len__(self) -> int:
        """Number of controllers."""
        return len(self.controllers)

    def __iter__(self) -> Iterator[str]:
        """Iterate over controller names."""
        return iter(self.controllers)

    def __contains__(self, name: str) -> bool:
        """Check if controller exists."""
        return name in self.controllers

    def __enter__(self) -> ControllerGroup:
        """Enter context - enable all."""
        self.enable_all()
        return self

    def __exit__(self, exc_type: type | None, exc_val: Exception | None, exc_tb: Any) -> None:
        """Exit context - disable all."""
        self.disable_all()

    async def __aenter__(self) -> ControllerGroup:
        """Async context manager entry - enable all controllers."""
        await self.enable_all_async()
        return self

    async def __aexit__(
        self,
        exc_type: type | None,
        exc_val: Exception | None,
        exc_tb: Any,
    ) -> None:
        """Async context manager exit - disable all controllers."""
        await self.disable_all_async()

    async def enable_all_async(self) -> None:
        """Enable all controllers asynchronously."""
        for controller in self.controllers.values():
            await controller.enable_async()

    async def disable_all_async(self) -> None:
        """Disable all controllers asynchronously."""
        for controller in self.controllers.values():
            await controller.disable_async()


# =============================================================================
# Factory Functions
# =============================================================================


def create_controller(
    name: str,
    *,
    actuators: dict[str, Actuator] | None = None,
    sensors: dict[str, Sensor] | None = None,
    config: ControllerConfig | None = None,
) -> SimulatedController:
    """Factory function to create a controller.

    Args:
        name: Controller name
        actuators: Dictionary of actuators to add
        sensors: Dictionary of sensors to add
        config: Controller configuration

    Returns:
        Configured SimulatedController instance
    """
    controller = SimulatedController(name=name, config=config)

    if actuators:
        for actuator_name, actuator in actuators.items():
            controller.add_actuator(actuator_name, actuator)

    if sensors:
        for sensor_name, sensor in sensors.items():
            controller.add_sensor(sensor_name, sensor)

    return controller
