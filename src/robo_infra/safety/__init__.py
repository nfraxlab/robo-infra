"""Safety systems and monitoring for robotics.

This module provides critical safety infrastructure:

- **EStop**: Emergency stop system that immediately disables all actuators
- **Watchdog**: Control loop watchdog timer that triggers E-stop on timeout
- **SafetyMonitor**: Monitors current, temperature, voltage with auto-disable
- **LimitEnforcer**: Active limit enforcement with velocity/acceleration limiting

SAFETY CRITICAL: These systems are designed to prevent physical harm.
All errors propagate (never silently suppressed) and events are logged
for safety auditing.

Example:
    >>> from robo_infra.safety import EStop, Watchdog, SafetyMonitor
    >>>
    >>> # Create E-stop
    >>> estop = EStop()
    >>> estop.register_actuator(servo)
    >>> estop.register_actuator(motor)
    >>>
    >>> # Create watchdog
    >>> watchdog = Watchdog(timeout=0.1, estop=estop)
    >>> watchdog.start()
    >>>
    >>> # Create safety monitor
    >>> monitor = SafetyMonitor(estop=estop)
    >>> monitor.add_current_limit("motor", max_current=5.0)
    >>> monitor.start()
"""

from robo_infra.safety.estop import (
    Disableable,
    EStop,
    EStopConfig,
    EStopError,
    EStopEvent,
    EStopState,
    HardwareEStop,
)
from robo_infra.safety.limits import (
    EnforcerConfig,
    LimitEnforcer,
    LimitGuard,
    LimitViolation,
    LimitViolationType,
)
from robo_infra.safety.monitor import (
    CollisionDetector,
    LimitConfig,
    LimitStatus,
    MonitorState,
    MonitorStatus,
    SafetyLevel,
    SafetyMonitor,
    SafetyViolation,
)
from robo_infra.safety.watchdog import (
    ControlLoopTimer,
    Watchdog,
    WatchdogConfig,
    WatchdogError,
    WatchdogState,
    WatchdogStatus,
)


__all__ = [
    "CollisionDetector",
    "ControlLoopTimer",
    "Disableable",
    # E-Stop
    "EStop",
    "EStopConfig",
    "EStopError",
    "EStopEvent",
    "EStopState",
    "EnforcerConfig",
    "HardwareEStop",
    "LimitConfig",
    # Limit Enforcement
    "LimitEnforcer",
    "LimitGuard",
    "LimitStatus",
    "LimitViolation",
    "LimitViolationType",
    "MonitorState",
    "MonitorStatus",
    "SafetyLevel",
    # Safety Monitor
    "SafetyMonitor",
    "SafetyViolation",
    # Watchdog
    "Watchdog",
    "WatchdogConfig",
    "WatchdogError",
    "WatchdogState",
    "WatchdogStatus",
]
