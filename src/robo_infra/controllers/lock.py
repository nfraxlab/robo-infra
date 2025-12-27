"""Lock controller for robotic locks and latches.

This module provides a Lock controller for managing locking mechanisms,
latches, and secure access points. Common examples include:
- Electronic door locks
- Safe locks
- Latch mechanisms
- Secure compartment locks
- Gate locks

Example:
    >>> from robo_infra.controllers.lock import Lock, LockConfig
    >>> from robo_infra.actuators.servo import Servo
    >>>
    >>> # Create lock actuator (servo-based)
    >>> actuator = Servo(name="lock_servo", angle_range=(0, 90))
    >>>
    >>> # Create lock controller
    >>> config = LockConfig(
    ...     name="door_lock",
    ...     locked_position=0,
    ...     unlocked_position=90,
    ... )
    >>> lock = Lock(
    ...     name="my_lock",
    ...     actuator=actuator,
    ...     config=config,
    ... )
    >>> lock.enable()
    >>>
    >>> # Lock commands
    >>> lock.unlock()
    >>> lock.lock()
    >>> lock.toggle()
"""

from __future__ import annotations

import asyncio
import logging
from enum import Enum

from pydantic import BaseModel, Field

from robo_infra.core.actuator import Actuator
from robo_infra.core.controller import Controller, ControllerConfig


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class LockState(Enum):
    """States a Lock can be in."""

    LOCKED = "locked"
    UNLOCKED = "unlocked"
    TRANSITIONING = "transitioning"
    ERROR = "error"
    DISABLED = "disabled"


# =============================================================================
# Configuration Models
# =============================================================================


class LockConfig(BaseModel):
    """Configuration for a Lock controller.

    Attributes:
        name: Human-readable name for the lock.
        locked_position: Position value when lock is in locked state.
        unlocked_position: Position value when lock is in unlocked state.
        transition_time: Time in seconds for lock to transition states.
        require_confirmation: Whether to require confirmation before unlocking.
        position_tolerance: Tolerance for position detection.

    Example:
        >>> config = LockConfig(
        ...     name="door_lock",
        ...     locked_position=0,       # Fully locked at 0
        ...     unlocked_position=90,    # Fully unlocked at 90
        ...     transition_time=0.5,     # 500ms transition
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Lock name")
    description: str = Field(default="", description="Human-readable description")

    # Position configuration
    locked_position: float = Field(
        default=0.0,
        description="Position value when lock is locked",
    )
    unlocked_position: float = Field(
        default=90.0,
        description="Position value when lock is unlocked",
    )
    position_tolerance: float = Field(
        default=1.0,
        ge=0,
        description="Position tolerance for state detection",
    )

    # Timing configuration
    transition_time: float = Field(
        default=0.5,
        ge=0,
        description="Time in seconds for lock to transition between states",
    )

    # Security configuration
    require_confirmation: bool = Field(
        default=False,
        description="Require confirmation before unlocking",
    )
    auto_lock_timeout: float | None = Field(
        default=None,
        ge=0,
        description="Auto-lock after this many seconds (None = disabled)",
    )

    # Initial state
    start_locked: bool = Field(
        default=True,
        description="Whether lock should start in locked state",
    )

    # Computed properties
    @property
    def range(self) -> float:
        """Calculate the total range of motion."""
        return abs(self.unlocked_position - self.locked_position)

    @property
    def is_inverted(self) -> bool:
        """Check if lock motion is inverted (unlocked < locked)."""
        return self.unlocked_position < self.locked_position


# =============================================================================
# Lock Controller
# =============================================================================


class Lock(Controller):
    """Controller for robotic locks and latches.

    Lock provides high-level control for locking mechanisms,
    with state tracking and optional security features.

    Features:
    - Lock/unlock control
    - State tracking (locked/unlocked/transitioning)
    - Async support for transition timing
    - Optional auto-lock timeout
    - Toggle convenience method

    Example:
        >>> lock = Lock("door", servo, config=LockConfig(name="door"))
        >>> lock.enable()
        >>> lock.unlock()
        >>> assert lock.is_unlocked
        >>> lock.lock()
        >>> assert lock.is_locked
    """

    def __init__(
        self,
        name: str,
        actuator: Actuator,
        *,
        config: LockConfig | None = None,
    ) -> None:
        """Initialize Lock controller.

        Args:
            name: Controller name
            actuator: Lock actuator (servo, motor, solenoid, etc.)
            config: Optional configuration

        Raises:
            ValueError: If actuator is None
        """
        if actuator is None:
            raise ValueError("Lock actuator is required")

        # Create config if not provided
        if config is None:
            config = LockConfig(name=name)

        # Initialize base controller
        super().__init__(name, config=ControllerConfig(name=name))

        # Store Lock-specific config
        self._lock_config = config

        # Store actuator
        self._actuator = actuator

        # Add actuator to base controller
        self.add_actuator("lock", actuator)

        # Lock-specific state
        self._lock_state = LockState.DISABLED
        self._auto_lock_task: asyncio.Task[None] | None = None

        logger.debug(
            "Lock '%s' initialized with actuator=%s, start_locked=%s",
            name,
            actuator.name,
            config.start_locked,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def actuator(self) -> Actuator:
        """Lock actuator."""
        return self._actuator

    @property
    def lock_config(self) -> LockConfig:
        """Lock-specific configuration."""
        return self._lock_config

    @property
    def lock_state(self) -> LockState:
        """Current lock state."""
        return self._lock_state

    @property
    def state_str(self) -> str:
        """Current state as string ("locked", "unlocked", "transitioning")."""
        return self._lock_state.value

    @property
    def position(self) -> float:
        """Current lock position."""
        return self._actuator.get()

    @property
    def is_locked(self) -> bool:
        """Check if lock is in locked state."""
        tolerance = self._lock_config.position_tolerance
        return abs(self.position - self._lock_config.locked_position) <= tolerance

    @property
    def is_unlocked(self) -> bool:
        """Check if lock is in unlocked state."""
        tolerance = self._lock_config.position_tolerance
        return abs(self.position - self._lock_config.unlocked_position) <= tolerance

    # -------------------------------------------------------------------------
    # Controller Abstract Methods
    # -------------------------------------------------------------------------

    def _do_home(self) -> None:
        """Implement abstract home method from Controller.

        For lock, 'home' means going to initial state (locked if start_locked).
        """
        if self._lock_config.start_locked:
            self.lock()
        else:
            self.unlock()

    def _do_stop(self) -> None:
        """Implement abstract stop method from Controller."""
        self.stop()

    # -------------------------------------------------------------------------
    # Lock Control Methods
    # -------------------------------------------------------------------------

    def stop(self) -> None:
        """Stop lock movement immediately."""
        if hasattr(self._actuator, "stop"):
            self._actuator.stop()
        self._update_state_from_position()
        logger.debug("Lock '%s' stopped at position %.2f", self.name, self.position)

    def lock(self) -> None:
        """Lock the mechanism.

        Moves the actuator to the locked position.

        Raises:
            RuntimeError: If lock is not enabled.

        Example:
            >>> lock.enable()
            >>> lock.lock()
            >>> assert lock.is_locked
        """
        self._check_enabled("lock")

        # Cancel any pending auto-lock
        self._cancel_auto_lock()

        self._lock_state = LockState.TRANSITIONING

        logger.debug(
            "Lock '%s' locking to position %.2f",
            self.name,
            self._lock_config.locked_position,
        )

        self._actuator.set(self._lock_config.locked_position)
        self._lock_state = LockState.LOCKED

        logger.info("Lock '%s' is now locked", self.name)

    def unlock(self) -> None:
        """Unlock the mechanism.

        Moves the actuator to the unlocked position.

        Raises:
            RuntimeError: If lock is not enabled.

        Example:
            >>> lock.enable()
            >>> lock.unlock()
            >>> assert lock.is_unlocked
        """
        self._check_enabled("unlock")

        self._lock_state = LockState.TRANSITIONING

        logger.debug(
            "Lock '%s' unlocking to position %.2f",
            self.name,
            self._lock_config.unlocked_position,
        )

        self._actuator.set(self._lock_config.unlocked_position)
        self._lock_state = LockState.UNLOCKED

        logger.info("Lock '%s' is now unlocked", self.name)

        # Start auto-lock timer if configured
        if self._lock_config.auto_lock_timeout is not None:
            self._start_auto_lock()

    def toggle(self) -> None:
        """Toggle the lock state.

        If locked, unlocks. If unlocked, locks.

        Raises:
            RuntimeError: If lock is not enabled.

        Example:
            >>> lock.enable()
            >>> lock.lock()
            >>> lock.toggle()
            >>> assert lock.is_unlocked
        """
        self._check_enabled("toggle")

        if self.is_locked:
            self.unlock()
        else:
            self.lock()

    async def alock(self) -> None:
        """Lock the mechanism asynchronously.

        Moves the actuator to the locked position and waits
        for the transition time.

        Raises:
            RuntimeError: If lock is not enabled.

        Example:
            >>> lock.enable()
            >>> await lock.alock()
            >>> assert lock.is_locked
        """
        self._check_enabled("alock")

        # Cancel any pending auto-lock
        self._cancel_auto_lock()

        self._lock_state = LockState.TRANSITIONING

        logger.debug(
            "Lock '%s' async locking to position %.2f",
            self.name,
            self._lock_config.locked_position,
        )

        self._actuator.set(self._lock_config.locked_position)

        # Wait for transition
        await asyncio.sleep(self._lock_config.transition_time)

        self._lock_state = LockState.LOCKED
        logger.info("Lock '%s' is now locked", self.name)

    async def aunlock(self) -> None:
        """Unlock the mechanism asynchronously.

        Moves the actuator to the unlocked position and waits
        for the transition time.

        Raises:
            RuntimeError: If lock is not enabled.

        Example:
            >>> lock.enable()
            >>> await lock.aunlock()
            >>> assert lock.is_unlocked
        """
        self._check_enabled("aunlock")

        self._lock_state = LockState.TRANSITIONING

        logger.debug(
            "Lock '%s' async unlocking to position %.2f",
            self.name,
            self._lock_config.unlocked_position,
        )

        self._actuator.set(self._lock_config.unlocked_position)

        # Wait for transition
        await asyncio.sleep(self._lock_config.transition_time)

        self._lock_state = LockState.UNLOCKED
        logger.info("Lock '%s' is now unlocked", self.name)

        # Start auto-lock timer if configured
        if self._lock_config.auto_lock_timeout is not None:
            self._start_auto_lock()

    # -------------------------------------------------------------------------
    # Enable/Disable Overrides
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the lock controller."""
        super().enable()
        self._actuator.enable()

        # Go to initial state
        if self._lock_config.start_locked:
            self._actuator.set(self._lock_config.locked_position)
            self._lock_state = LockState.LOCKED
        else:
            self._actuator.set(self._lock_config.unlocked_position)
            self._lock_state = LockState.UNLOCKED

        logger.info("Lock '%s' enabled, state=%s", self.name, self._lock_state.value)

    def disable(self) -> None:
        """Disable the lock controller."""
        # Cancel auto-lock task
        self._cancel_auto_lock()

        super().disable()
        self._actuator.disable()
        self._lock_state = LockState.DISABLED

        logger.info("Lock '%s' disabled", self.name)

    # -------------------------------------------------------------------------
    # Private Helper Methods
    # -------------------------------------------------------------------------

    def _check_enabled(self, operation: str) -> None:
        """Check if lock is enabled before operation.

        Args:
            operation: Name of operation being attempted.

        Raises:
            RuntimeError: If lock is not enabled.
        """
        if not self.is_enabled:
            raise RuntimeError(
                f"Cannot {operation}: Lock '{self.name}' is not enabled. Call lock.enable() first."
            )

    def _update_state_from_position(self) -> None:
        """Update lock state based on current position."""
        if self.is_locked:
            self._lock_state = LockState.LOCKED
        elif self.is_unlocked:
            self._lock_state = LockState.UNLOCKED
        else:
            # Intermediate position
            self._lock_state = LockState.TRANSITIONING

    def _start_auto_lock(self) -> None:
        """Start auto-lock timer."""
        self._cancel_auto_lock()

        timeout = self._lock_config.auto_lock_timeout
        if timeout is not None and timeout > 0:
            logger.debug("Lock '%s' starting auto-lock timer: %.1fs", self.name, timeout)

            async def auto_lock_coro() -> None:
                await asyncio.sleep(timeout)
                if self.is_enabled and self.is_unlocked:
                    logger.info("Lock '%s' auto-locking", self.name)
                    self.lock()

            try:
                loop = asyncio.get_running_loop()
                self._auto_lock_task = loop.create_task(auto_lock_coro())
            except RuntimeError:
                # No running loop - auto-lock only works in async context
                logger.debug("Lock '%s': auto-lock requires async context", self.name)

    def _cancel_auto_lock(self) -> None:
        """Cancel pending auto-lock timer."""
        if self._auto_lock_task is not None:
            self._auto_lock_task.cancel()
            self._auto_lock_task = None
            logger.debug("Lock '%s' auto-lock cancelled", self.name)

    # -------------------------------------------------------------------------
    # String Representation
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"Lock(name='{self.name}', "
            f"position={self.position:.2f}, "
            f"state={self._lock_state.value})"
        )


__all__ = [
    "Lock",
    "LockConfig",
    "LockState",
]
