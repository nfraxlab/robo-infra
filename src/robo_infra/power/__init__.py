"""Power management module for robo-infra.

This module provides battery monitoring, power distribution, and power
management utilities for robotics applications.

Key Components:
- BatteryMonitor: Monitor battery state (voltage, current, percentage)
- BatteryChemistry: Supported battery chemistries with voltage curves
- PowerRail: Controllable power rail with enable/disable
- PowerDistributionBoard: Multi-rail power distribution management
- INA219Driver: I2C power monitor driver
- INA226Driver: High-precision I2C power monitor driver

Example:
    >>> from robo_infra.power import BatteryMonitor, BatteryChemistry
    >>> battery = BatteryMonitor(
    ...     cells=3,
    ...     chemistry=BatteryChemistry.LIPO,
    ... )
    >>> battery.enable()
    >>> print(f"Battery: {battery.percentage:.1f}%")
"""

from robo_infra.power.battery import (
    BatteryChemistry,
    BatteryConfig,
    BatteryMonitor,
    BatteryReading,
    BatteryState,
    get_battery_monitor,
)
from robo_infra.power.distribution import (
    PowerDistributionBoard,
    PowerDistributionConfig,
    PowerRail,
    PowerRailConfig,
    PowerRailState,
)
from robo_infra.power.drivers import (
    INA219Config,
    INA219Driver,
    INA226Config,
    INA226Driver,
    PowerMonitorDriver,
)


__all__ = [
    # Battery
    "BatteryChemistry",
    "BatteryConfig",
    "BatteryMonitor",
    "BatteryReading",
    "BatteryState",
    # Drivers
    "INA219Config",
    "INA219Driver",
    "INA226Config",
    "INA226Driver",
    # Distribution
    "PowerDistributionBoard",
    "PowerDistributionConfig",
    "PowerMonitorDriver",
    "PowerRail",
    "PowerRailConfig",
    "PowerRailState",
    "get_battery_monitor",
]
