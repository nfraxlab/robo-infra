"""Power distribution and rail management.

This module provides power distribution control for robotic systems,
managing multiple power rails with enable/disable control and
emergency shutdown capabilities.

Key Features:
- Individual power rail control via GPIO
- Multi-rail power distribution boards
- Emergency shutdown with ordered rail disable
- Current monitoring per rail (optional)
- Power budget management

Example:
    >>> from robo_infra.power import PowerRail, PowerDistributionBoard
    >>> motors = PowerRail("motors", enable_pin=17)
    >>> sensors = PowerRail("sensors", enable_pin=27)
    >>> pdb = PowerDistributionBoard([motors, sensors])
    >>> pdb.enable_all()
    >>> # ... do work ...
    >>> pdb.emergency_shutdown()
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field


if TYPE_CHECKING:
    from robo_infra.core.pin import Pin
    from robo_infra.power.drivers import PowerMonitorDriver


logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class PowerRailState(IntEnum):
    """Power rail state."""

    DISABLED = 0
    ENABLED = 1
    FAULT = 2
    UNKNOWN = 3


class ShutdownPriority(IntEnum):
    """Shutdown priority for power rails.

    Lower numbers are shut down first during emergency shutdown.
    """

    CRITICAL = 0  # Shut down last (e.g., safety systems)
    HIGH = 1  # Core systems
    NORMAL = 2  # Standard loads
    LOW = 3  # Non-essential
    LOWEST = 4  # First to shut down


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class PowerRailReading:
    """Power rail reading data."""

    name: str
    state: PowerRailState
    voltage: float | None  # Rail voltage if monitored
    current: float | None  # Rail current if monitored
    power: float | None  # Rail power if monitored
    timestamp: float = field(default_factory=time.time)

    @property
    def is_enabled(self) -> bool:
        """Check if rail is enabled."""
        return self.state == PowerRailState.ENABLED


# =============================================================================
# Configuration
# =============================================================================


class PowerRailConfig(BaseModel):
    """Configuration for power rail."""

    model_config = {"frozen": False, "extra": "allow"}

    # Rail identification
    name: str = Field(description="Power rail name")

    # GPIO configuration
    enable_pin: int = Field(
        ge=0,
        description="GPIO pin number for enable control",
    )
    active_high: bool = Field(
        default=True,
        description="True if high signal enables the rail",
    )

    # Power limits
    max_current: float | None = Field(
        default=None,
        description="Maximum current limit (A)",
    )
    max_power: float | None = Field(
        default=None,
        description="Maximum power limit (W)",
    )
    nominal_voltage: float | None = Field(
        default=None,
        description="Nominal rail voltage (V)",
    )

    # Timing
    enable_delay_ms: int = Field(
        default=100,
        ge=0,
        description="Delay after enabling (ms)",
    )
    disable_delay_ms: int = Field(
        default=50,
        ge=0,
        description="Delay after disabling (ms)",
    )

    # Shutdown priority
    shutdown_priority: ShutdownPriority = Field(
        default=ShutdownPriority.NORMAL,
        description="Priority during emergency shutdown",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


class PowerDistributionConfig(BaseModel):
    """Configuration for power distribution board."""

    model_config = {"frozen": False, "extra": "allow"}

    name: str = Field(
        default="power_distribution",
        description="Distribution board name",
    )

    # Emergency shutdown
    estop_pin: int | None = Field(
        default=None,
        description="GPIO pin for emergency stop input",
    )
    estop_active_low: bool = Field(
        default=True,
        description="True if low signal triggers e-stop",
    )

    # Power budget
    total_power_budget: float | None = Field(
        default=None,
        description="Total power budget (W)",
    )
    total_current_limit: float | None = Field(
        default=None,
        description="Total current limit (A)",
    )

    # Startup sequence
    startup_delay_ms: int = Field(
        default=100,
        ge=0,
        description="Delay between enabling rails during startup (ms)",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# Power Rail
# =============================================================================


class PowerRail:
    """Single power rail with GPIO enable control.

    Represents a power rail that can be enabled/disabled via a GPIO pin.
    Optionally monitors current/voltage with a power monitor driver.

    Example:
        >>> from robo_infra.power import PowerRail
        >>> motors = PowerRail("motors", enable_pin=17)
        >>> motors.enable()
        >>> print(f"Motors enabled: {motors.is_enabled}")
        >>> motors.disable()
    """

    def __init__(
        self,
        name: str,
        enable_pin: int | Pin | None = None,
        config: PowerRailConfig | None = None,
        monitor: PowerMonitorDriver | None = None,
    ) -> None:
        """Initialize power rail.

        Args:
            name: Rail name.
            enable_pin: GPIO pin number or Pin instance.
            config: Full configuration (overrides name/enable_pin).
            monitor: Optional power monitor for current/voltage.
        """
        # Handle configuration
        if config is not None:
            self._config = config
        else:
            self._config = PowerRailConfig(
                name=name,
                enable_pin=enable_pin if isinstance(enable_pin, int) else 0,
            )

        self._name = self._config.name
        self._monitor = monitor

        # GPIO pin
        self._pin: Pin | None = None
        if isinstance(enable_pin, int):
            # Will be set up by platform
            self._pin_number = enable_pin
        elif enable_pin is not None:
            self._pin = enable_pin
            self._pin_number = getattr(enable_pin, "number", 0)
        else:
            self._pin_number = self._config.enable_pin

        # State
        self._state = PowerRailState.DISABLED
        self._enabled_at: float | None = None

        # Simulated state
        self._simulated = self._pin is None

        logger.debug(
            "PowerRail '%s' initialized: pin=%d, active_high=%s",
            self._name,
            self._pin_number,
            self._config.active_high,
        )

    @property
    def name(self) -> str:
        """Get rail name."""
        return self._name

    @property
    def config(self) -> PowerRailConfig:
        """Get configuration."""
        return self._config

    @property
    def state(self) -> PowerRailState:
        """Get rail state."""
        return self._state

    @property
    def is_enabled(self) -> bool:
        """Check if rail is enabled."""
        return self._state == PowerRailState.ENABLED

    @property
    def shutdown_priority(self) -> ShutdownPriority:
        """Get shutdown priority."""
        return self._config.shutdown_priority

    def set_pin(self, pin: Pin) -> None:
        """Set the GPIO pin.

        Args:
            pin: Pin instance for enable control.
        """
        self._pin = pin
        self._simulated = False

    def enable(self) -> None:
        """Enable the power rail."""
        if self._state == PowerRailState.ENABLED:
            return

        # Set GPIO
        if self._pin is not None:
            if self._config.active_high:
                self._pin.high()
            else:
                self._pin.low()

        # Wait for power stabilization
        if self._config.enable_delay_ms > 0:
            time.sleep(self._config.enable_delay_ms / 1000)

        self._state = PowerRailState.ENABLED
        self._enabled_at = time.time()

        logger.info("PowerRail '%s' enabled", self._name)

    def disable(self) -> None:
        """Disable the power rail."""
        if self._state == PowerRailState.DISABLED:
            return

        # Set GPIO
        if self._pin is not None:
            if self._config.active_high:
                self._pin.low()
            else:
                self._pin.high()

        # Wait for discharge
        if self._config.disable_delay_ms > 0:
            time.sleep(self._config.disable_delay_ms / 1000)

        self._state = PowerRailState.DISABLED
        self._enabled_at = None

        logger.info("PowerRail '%s' disabled", self._name)

    def read(self) -> PowerRailReading:
        """Read current rail state and power metrics.

        Returns:
            PowerRailReading with state and optional power data.
        """
        voltage: float | None = None
        current: float | None = None
        power: float | None = None

        if self._monitor is not None and self._monitor.is_enabled():
            voltage = self._monitor.read_voltage()
            current = self._monitor.read_current()
            power = self._monitor.read_power()

        return PowerRailReading(
            name=self._name,
            state=self._state,
            voltage=voltage,
            current=current,
            power=power,
        )

    def uptime(self) -> float | None:
        """Get time since rail was enabled.

        Returns:
            Seconds since enable, or None if disabled.
        """
        if self._enabled_at is None:
            return None
        return time.time() - self._enabled_at


# =============================================================================
# Power Distribution Board
# =============================================================================


class PowerDistributionBoard:
    """Multi-rail power distribution management.

    Manages multiple power rails with coordinated enable/disable,
    emergency shutdown, and optional power budget management.

    Features:
    - Ordered startup sequence
    - Priority-based shutdown
    - Emergency stop handling
    - Power budget tracking

    Example:
        >>> from robo_infra.power import PowerRail, PowerDistributionBoard
        >>> motors = PowerRail("motors", enable_pin=17)
        >>> sensors = PowerRail("sensors", enable_pin=27)
        >>> logic = PowerRail("logic", enable_pin=22)
        >>> pdb = PowerDistributionBoard([motors, sensors, logic])
        >>> pdb.enable_all()
        >>> # ... do work ...
        >>> pdb.disable("motors")  # Disable specific rail
        >>> pdb.emergency_shutdown()  # Emergency stop all
    """

    def __init__(
        self,
        rails: list[PowerRail] | None = None,
        config: PowerDistributionConfig | None = None,
        estop_pin: Pin | None = None,
    ) -> None:
        """Initialize power distribution board.

        Args:
            rails: List of power rails to manage.
            config: Configuration options.
            estop_pin: Optional GPIO pin for emergency stop input.
        """
        self._config = config or PowerDistributionConfig()
        self._rails: dict[str, PowerRail] = {}
        self._estop_pin = estop_pin
        self._estop_triggered = False

        # Add initial rails
        if rails:
            for rail in rails:
                self.add_rail(rail)

        logger.debug(
            "PowerDistributionBoard '%s' initialized with %d rails",
            self._config.name,
            len(self._rails),
        )

    @property
    def name(self) -> str:
        """Get board name."""
        return self._config.name

    @property
    def config(self) -> PowerDistributionConfig:
        """Get configuration."""
        return self._config

    @property
    def rails(self) -> dict[str, PowerRail]:
        """Get all rails by name."""
        return self._rails.copy()

    @property
    def is_estop_triggered(self) -> bool:
        """Check if emergency stop is triggered."""
        return self._estop_triggered

    # -------------------------------------------------------------------------
    # Rail Management
    # -------------------------------------------------------------------------

    def add_rail(self, rail: PowerRail) -> None:
        """Add a power rail.

        Args:
            rail: Power rail to add.
        """
        if rail.name in self._rails:
            raise ValueError(f"Rail '{rail.name}' already exists")

        self._rails[rail.name] = rail
        logger.debug("Added rail '%s' to distribution board", rail.name)

    def remove_rail(self, name: str) -> PowerRail | None:
        """Remove a power rail.

        Args:
            name: Name of rail to remove.

        Returns:
            Removed rail or None if not found.
        """
        rail = self._rails.pop(name, None)
        if rail:
            rail.disable()
            logger.debug("Removed rail '%s' from distribution board", name)
        return rail

    def get_rail(self, name: str) -> PowerRail | None:
        """Get a rail by name.

        Args:
            name: Rail name.

        Returns:
            Rail or None if not found.
        """
        return self._rails.get(name)

    # -------------------------------------------------------------------------
    # Enable/Disable
    # -------------------------------------------------------------------------

    def enable(self, name: str) -> None:
        """Enable a specific rail.

        Args:
            name: Rail name to enable.
        """
        if self._estop_triggered:
            raise RuntimeError("Cannot enable rails: E-STOP is triggered")

        rail = self._rails.get(name)
        if rail is None:
            raise KeyError(f"Rail '{name}' not found")

        rail.enable()

    def disable(self, name: str) -> None:
        """Disable a specific rail.

        Args:
            name: Rail name to disable.
        """
        rail = self._rails.get(name)
        if rail is None:
            raise KeyError(f"Rail '{name}' not found")

        rail.disable()

    def enable_all(self) -> None:
        """Enable all rails in priority order.

        Rails are enabled in reverse shutdown priority order
        (CRITICAL first, LOWEST last) with delays between.
        """
        if self._estop_triggered:
            raise RuntimeError("Cannot enable rails: E-STOP is triggered")

        # Sort by shutdown priority (reversed for enable)
        sorted_rails = sorted(
            self._rails.values(),
            key=lambda r: r.shutdown_priority,
        )

        for rail in sorted_rails:
            rail.enable()
            if self._config.startup_delay_ms > 0:
                time.sleep(self._config.startup_delay_ms / 1000)

        logger.info("All %d rails enabled", len(self._rails))

    def disable_all(self) -> None:
        """Disable all rails in reverse priority order.

        Rails are disabled in shutdown priority order
        (LOWEST first, CRITICAL last).
        """
        # Sort by shutdown priority (reversed for proper order)
        sorted_rails = sorted(
            self._rails.values(),
            key=lambda r: -r.shutdown_priority,
        )

        for rail in sorted_rails:
            rail.disable()

        logger.info("All %d rails disabled", len(self._rails))

    # -------------------------------------------------------------------------
    # Emergency Shutdown
    # -------------------------------------------------------------------------

    def emergency_shutdown(self) -> None:
        """Execute emergency shutdown.

        Immediately disables ALL rails as fast as possible,
        ignoring delays. This is a safety-critical operation.

        CRITICAL: This method must complete successfully.
        """
        logger.critical("EMERGENCY SHUTDOWN initiated!")

        self._estop_triggered = True

        # Disable all rails as fast as possible
        # Sort by shutdown priority (LOWEST first = non-essential first)
        sorted_rails = sorted(
            self._rails.values(),
            key=lambda r: -r.shutdown_priority,
        )

        errors: list[str] = []
        for rail in sorted_rails:
            try:
                # Bypass normal disable - go straight to GPIO
                if rail._pin is not None:
                    if rail._config.active_high:
                        rail._pin.low()
                    else:
                        rail._pin.high()
                rail._state = PowerRailState.DISABLED
            except Exception as e:
                # Log but continue - must disable ALL rails
                error_msg = f"Failed to disable rail '{rail.name}': {e}"
                logger.error(error_msg)
                errors.append(error_msg)

        logger.critical(
            "EMERGENCY SHUTDOWN complete: %d rails disabled, %d errors",
            len(self._rails),
            len(errors),
        )

        if errors:
            # Log all errors for investigation
            for error in errors:
                logger.error("E-STOP error: %s", error)

    def reset_estop(self) -> None:
        """Reset emergency stop state.

        Must be called before rails can be re-enabled after
        an emergency shutdown.
        """
        self._estop_triggered = False
        logger.info("E-STOP reset")

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def get_status(self) -> dict[str, PowerRailReading]:
        """Get status of all rails.

        Returns:
            Dict mapping rail names to readings.
        """
        return {name: rail.read() for name, rail in self._rails.items()}

    def get_total_power(self) -> float:
        """Get total power consumption across all rails.

        Returns:
            Total power in W (0 if not monitored).
        """
        total = 0.0
        for rail in self._rails.values():
            reading = rail.read()
            if reading.power is not None:
                total += reading.power
        return total

    def get_total_current(self) -> float:
        """Get total current draw across all rails.

        Returns:
            Total current in A (0 if not monitored).
        """
        total = 0.0
        for rail in self._rails.values():
            reading = rail.read()
            if reading.current is not None:
                total += reading.current
        return total

    def check_power_budget(self) -> tuple[bool, float, float]:
        """Check if power consumption is within budget.

        Returns:
            Tuple of (within_budget, current_power, budget).
        """
        if self._config.total_power_budget is None:
            return (True, 0.0, 0.0)

        current = self.get_total_power()
        budget = self._config.total_power_budget
        return (current <= budget, current, budget)


# =============================================================================
# Factory Functions
# =============================================================================


def create_power_rail(
    name: str,
    enable_pin: int,
    active_high: bool = True,
    priority: ShutdownPriority = ShutdownPriority.NORMAL,
    **kwargs: Any,
) -> PowerRail:
    """Create a power rail with common configuration.

    Args:
        name: Rail name.
        enable_pin: GPIO pin for enable.
        active_high: Enable polarity.
        priority: Shutdown priority.
        **kwargs: Additional config options.

    Returns:
        Configured PowerRail instance.
    """
    config = PowerRailConfig(
        name=name,
        enable_pin=enable_pin,
        active_high=active_high,
        shutdown_priority=priority,
        **kwargs,
    )
    return PowerRail(name=name, config=config)


def create_distribution_board(
    rails: list[tuple[str, int]],
    estop_pin: int | None = None,
    name: str = "pdb",
) -> PowerDistributionBoard:
    """Create a power distribution board from rail specs.

    Args:
        rails: List of (name, enable_pin) tuples.
        estop_pin: Optional e-stop GPIO pin.
        name: Board name.

    Returns:
        Configured PowerDistributionBoard.

    Example:
        >>> pdb = create_distribution_board([
        ...     ("logic", 17),
        ...     ("sensors", 27),
        ...     ("motors", 22),
        ... ])
    """
    power_rails = [PowerRail(name=rail_name, enable_pin=pin) for rail_name, pin in rails]

    config = PowerDistributionConfig(
        name=name,
        estop_pin=estop_pin,
    )

    return PowerDistributionBoard(rails=power_rails, config=config)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "PowerDistributionBoard",
    "PowerDistributionConfig",
    # Classes
    "PowerRail",
    # Config
    "PowerRailConfig",
    # Data classes
    "PowerRailReading",
    # Enums
    "PowerRailState",
    "ShutdownPriority",
    "create_distribution_board",
    # Factory functions
    "create_power_rail",
]
