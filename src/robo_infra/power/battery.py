"""Battery monitoring and state tracking.

This module provides battery monitoring capabilities for various battery
chemistries commonly used in robotics: LiPo, Li-Ion, NiMH, LiFePO4, and
Lead Acid.

Key Features:
- Voltage-based state of charge estimation
- Low/critical battery detection
- Coulomb counting for accurate SoC
- Temperature compensation (optional)
- Multi-cell battery support

Example:
    >>> from robo_infra.power import BatteryMonitor, BatteryChemistry
    >>> battery = BatteryMonitor(
    ...     cells=3,
    ...     chemistry=BatteryChemistry.LIPO,
    ... )
    >>> battery.enable()
    >>> print(f"Voltage: {battery.voltage:.2f}V ({battery.percentage:.0f}%)")
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field


if TYPE_CHECKING:
    from robo_infra.power.drivers import PowerMonitorDriver


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

# Voltage curves for different chemistries (per cell)
# Format: (percentage, voltage) tuples
LIPO_VOLTAGE_CURVE = [
    (100, 4.20),
    (95, 4.15),
    (90, 4.10),
    (85, 4.05),
    (80, 4.00),
    (75, 3.95),
    (70, 3.90),
    (65, 3.85),
    (60, 3.80),
    (55, 3.75),
    (50, 3.70),
    (45, 3.65),
    (40, 3.60),
    (35, 3.55),
    (30, 3.50),
    (25, 3.45),
    (20, 3.40),
    (15, 3.35),
    (10, 3.30),
    (5, 3.20),
    (0, 3.00),
]

LIION_VOLTAGE_CURVE = [
    (100, 4.20),
    (90, 4.05),
    (80, 3.95),
    (70, 3.85),
    (60, 3.75),
    (50, 3.70),
    (40, 3.65),
    (30, 3.55),
    (20, 3.45),
    (10, 3.30),
    (0, 3.00),
]

LIFEPO4_VOLTAGE_CURVE = [
    (100, 3.60),
    (90, 3.35),
    (80, 3.30),
    (70, 3.28),
    (60, 3.26),
    (50, 3.24),
    (40, 3.22),
    (30, 3.20),
    (20, 3.15),
    (10, 3.00),
    (0, 2.50),
]

NIMH_VOLTAGE_CURVE = [
    (100, 1.40),
    (90, 1.35),
    (80, 1.30),
    (70, 1.27),
    (60, 1.25),
    (50, 1.23),
    (40, 1.21),
    (30, 1.19),
    (20, 1.17),
    (10, 1.10),
    (0, 1.00),
]

LEAD_ACID_VOLTAGE_CURVE = [
    (100, 2.12),
    (90, 2.10),
    (80, 2.08),
    (70, 2.06),
    (60, 2.04),
    (50, 2.02),
    (40, 2.00),
    (30, 1.98),
    (20, 1.95),
    (10, 1.90),
    (0, 1.80),
]


# =============================================================================
# Enums
# =============================================================================


class BatteryChemistry(str, Enum):
    """Supported battery chemistries."""

    LIPO = "lipo"
    LIION = "liion"
    LIFEPO4 = "lifepo4"
    NIMH = "nimh"
    LEAD_ACID = "lead_acid"

    def get_voltage_curve(self) -> list[tuple[int, float]]:
        """Get voltage curve for this chemistry."""
        curves = {
            BatteryChemistry.LIPO: LIPO_VOLTAGE_CURVE,
            BatteryChemistry.LIION: LIION_VOLTAGE_CURVE,
            BatteryChemistry.LIFEPO4: LIFEPO4_VOLTAGE_CURVE,
            BatteryChemistry.NIMH: NIMH_VOLTAGE_CURVE,
            BatteryChemistry.LEAD_ACID: LEAD_ACID_VOLTAGE_CURVE,
        }
        return curves[self]

    def get_nominal_voltage(self) -> float:
        """Get nominal voltage per cell."""
        voltages = {
            BatteryChemistry.LIPO: 3.7,
            BatteryChemistry.LIION: 3.7,
            BatteryChemistry.LIFEPO4: 3.2,
            BatteryChemistry.NIMH: 1.2,
            BatteryChemistry.LEAD_ACID: 2.0,
        }
        return voltages[self]

    def get_full_voltage(self) -> float:
        """Get fully charged voltage per cell."""
        voltages = {
            BatteryChemistry.LIPO: 4.2,
            BatteryChemistry.LIION: 4.2,
            BatteryChemistry.LIFEPO4: 3.6,
            BatteryChemistry.NIMH: 1.4,
            BatteryChemistry.LEAD_ACID: 2.12,
        }
        return voltages[self]

    def get_empty_voltage(self) -> float:
        """Get empty/cutoff voltage per cell."""
        voltages = {
            BatteryChemistry.LIPO: 3.0,
            BatteryChemistry.LIION: 3.0,
            BatteryChemistry.LIFEPO4: 2.5,
            BatteryChemistry.NIMH: 1.0,
            BatteryChemistry.LEAD_ACID: 1.8,
        }
        return voltages[self]


class BatteryState(IntEnum):
    """Battery state."""

    UNKNOWN = 0
    DISCHARGING = 1
    CHARGING = 2
    FULL = 3
    LOW = 4
    CRITICAL = 5
    FAULT = 6


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class BatteryReading:
    """Battery reading data."""

    voltage: float  # Total pack voltage (V)
    current: float  # Current (A, positive = discharging)
    power: float  # Power (W)
    percentage: float  # State of charge (0-100%)
    cell_voltage: float  # Voltage per cell (V)
    state: BatteryState
    temperature: float | None  # Temperature (°C)
    timestamp: float = field(default_factory=time.time)

    @property
    def is_low(self) -> bool:
        """Check if battery is low (<20%)."""
        return self.percentage < 20.0

    @property
    def is_critical(self) -> bool:
        """Check if battery is critical (<10%)."""
        return self.percentage < 10.0

    @property
    def is_full(self) -> bool:
        """Check if battery is full (>95%)."""
        return self.percentage > 95.0


# =============================================================================
# Configuration
# =============================================================================


class BatteryConfig(BaseModel):
    """Configuration for battery monitor."""

    model_config = {"frozen": False, "extra": "allow"}

    # Battery specification
    cells: int = Field(default=3, ge=1, le=20, description="Number of cells in series")
    chemistry: BatteryChemistry = Field(
        default=BatteryChemistry.LIPO,
        description="Battery chemistry type",
    )
    capacity_mah: int = Field(
        default=5000,
        ge=100,
        description="Battery capacity in mAh",
    )

    # Thresholds
    low_threshold: float = Field(
        default=20.0,
        ge=5.0,
        le=50.0,
        description="Low battery threshold (%)",
    )
    critical_threshold: float = Field(
        default=10.0,
        ge=1.0,
        le=30.0,
        description="Critical battery threshold (%)",
    )

    # Voltage limits (per cell)
    min_cell_voltage: float | None = Field(
        default=None,
        description="Minimum cell voltage (overrides chemistry default)",
    )
    max_cell_voltage: float | None = Field(
        default=None,
        description="Maximum cell voltage (overrides chemistry default)",
    )

    # Update settings
    update_interval: float = Field(
        default=1.0,
        ge=0.1,
        description="Update interval in seconds",
    )

    # Coulomb counting
    enable_coulomb_counting: bool = Field(
        default=False,
        description="Enable coulomb counting for SoC",
    )

    metadata: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# Battery Monitor
# =============================================================================


class BatteryMonitor:
    """Battery state monitoring.

    Monitors battery voltage, current, and estimates state of charge
    based on voltage curves for various battery chemistries.

    Features:
    - Voltage-based SoC estimation with chemistry-specific curves
    - Low and critical battery detection
    - Optional coulomb counting for accurate SoC
    - Multi-cell battery support

    Example:
        >>> battery = BatteryMonitor(cells=3, chemistry=BatteryChemistry.LIPO)
        >>> battery.enable()
        >>> print(f"Voltage: {battery.voltage:.2f}V")
        >>> print(f"Percentage: {battery.percentage:.0f}%")
        >>> if battery.is_low:
        ...     print("Battery low!")
    """

    def __init__(
        self,
        cells: int = 3,
        chemistry: BatteryChemistry | str = BatteryChemistry.LIPO,
        config: BatteryConfig | None = None,
        driver: PowerMonitorDriver | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize battery monitor.

        Args:
            cells: Number of cells in series.
            chemistry: Battery chemistry type.
            config: Full configuration (overrides cells/chemistry).
            driver: Power monitor driver for readings.
            name: Optional name for this monitor.
        """
        # Handle chemistry as string
        if isinstance(chemistry, str):
            chemistry = BatteryChemistry(chemistry.lower())

        # Create or use provided config
        if config is not None:
            self._config = config
        else:
            self._config = BatteryConfig(cells=cells, chemistry=chemistry)

        self._driver = driver
        self._name = name or f"battery_{self._config.cells}s_{self._config.chemistry.value}"
        self._enabled = False

        # Voltage curve for this chemistry
        self._voltage_curve = self._config.chemistry.get_voltage_curve()

        # Coulomb counting state
        self._coulombs_remaining: float | None = None
        self._last_reading_time: float | None = None

        # Last reading cache
        self._last_reading: BatteryReading | None = None

        # Simulated values (used when no driver)
        self._simulated_voltage = self._config.chemistry.get_nominal_voltage() * self._config.cells
        self._simulated_current = 0.5  # 0.5A discharge

        logger.debug(
            "BatteryMonitor initialized: %s (%dS %s)",
            self._name,
            self._config.cells,
            self._config.chemistry.value,
        )

    @property
    def name(self) -> str:
        """Get monitor name."""
        return self._name

    @property
    def config(self) -> BatteryConfig:
        """Get configuration."""
        return self._config

    @property
    def is_enabled(self) -> bool:
        """Check if monitor is enabled."""
        return self._enabled

    @property
    def cells(self) -> int:
        """Get number of cells."""
        return self._config.cells

    @property
    def chemistry(self) -> BatteryChemistry:
        """Get battery chemistry."""
        return self._config.chemistry

    # -------------------------------------------------------------------------
    # Enable/Disable
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable battery monitoring."""
        if self._enabled:
            return

        # Initialize driver if present
        if self._driver is not None:
            self._driver.enable()

        # Initialize coulomb counting
        if self._config.enable_coulomb_counting:
            # Assume full capacity initially
            self._coulombs_remaining = self._config.capacity_mah * 3.6  # mAh to Coulombs
            self._last_reading_time = time.time()

        self._enabled = True
        logger.info("BatteryMonitor '%s' enabled", self._name)

    def disable(self) -> None:
        """Disable battery monitoring."""
        if not self._enabled:
            return

        if self._driver is not None:
            self._driver.disable()

        self._enabled = False
        logger.info("BatteryMonitor '%s' disabled", self._name)

    # -------------------------------------------------------------------------
    # Readings
    # -------------------------------------------------------------------------

    def read(self) -> BatteryReading:
        """Read current battery state.

        Returns:
            BatteryReading with current voltage, current, and SoC.
        """
        if not self._enabled:
            raise RuntimeError(f"BatteryMonitor '{self._name}' is not enabled")

        # Get raw readings
        if self._driver is not None:
            voltage = self._driver.read_voltage()
            current = self._driver.read_current()
            power = self._driver.read_power()
        else:
            # Simulated readings
            voltage = self._simulated_voltage
            current = self._simulated_current
            power = voltage * current

        # Calculate per-cell voltage
        cell_voltage = voltage / self._config.cells

        # Calculate percentage from voltage curve
        percentage = self._voltage_to_percentage(cell_voltage)

        # Update coulomb counting if enabled
        if self._config.enable_coulomb_counting and self._last_reading_time is not None:
            now = time.time()
            dt = now - self._last_reading_time
            self._last_reading_time = now

            # Subtract coulombs (current * time)
            if self._coulombs_remaining is not None:
                self._coulombs_remaining -= current * dt
                self._coulombs_remaining = max(0, self._coulombs_remaining)

                # Use coulomb counting for percentage if available
                total_coulombs = self._config.capacity_mah * 3.6
                percentage = (self._coulombs_remaining / total_coulombs) * 100

        # Determine state
        state = self._determine_state(percentage, current)

        reading = BatteryReading(
            voltage=voltage,
            current=current,
            power=power,
            percentage=percentage,
            cell_voltage=cell_voltage,
            state=state,
            temperature=None,  # Would come from temperature sensor
        )

        self._last_reading = reading
        return reading

    @property
    def voltage(self) -> float:
        """Get current voltage (V)."""
        if self._last_reading is None:
            self.read()
        assert self._last_reading is not None
        return self._last_reading.voltage

    @property
    def current(self) -> float:
        """Get current draw (A)."""
        if self._last_reading is None:
            self.read()
        assert self._last_reading is not None
        return self._last_reading.current

    @property
    def power(self) -> float:
        """Get power consumption (W)."""
        if self._last_reading is None:
            self.read()
        assert self._last_reading is not None
        return self._last_reading.power

    @property
    def percentage(self) -> float:
        """Get state of charge (0-100%)."""
        if self._last_reading is None:
            self.read()
        assert self._last_reading is not None
        return self._last_reading.percentage

    @property
    def cell_voltage(self) -> float:
        """Get voltage per cell (V)."""
        if self._last_reading is None:
            self.read()
        assert self._last_reading is not None
        return self._last_reading.cell_voltage

    @property
    def state(self) -> BatteryState:
        """Get battery state."""
        if self._last_reading is None:
            self.read()
        assert self._last_reading is not None
        return self._last_reading.state

    @property
    def is_low(self) -> bool:
        """Check if battery is low."""
        return self.percentage < self._config.low_threshold

    @property
    def is_critical(self) -> bool:
        """Check if battery is critical."""
        return self.percentage < self._config.critical_threshold

    @property
    def is_full(self) -> bool:
        """Check if battery is full."""
        return self.percentage > 95.0

    # -------------------------------------------------------------------------
    # Simulation
    # -------------------------------------------------------------------------

    def set_simulated_values(self, voltage: float, current: float = 0.5) -> None:
        """Set simulated values for testing.

        Args:
            voltage: Total pack voltage.
            current: Current draw (positive = discharge).
        """
        self._simulated_voltage = voltage
        self._simulated_current = current

    # -------------------------------------------------------------------------
    # Internal Methods
    # -------------------------------------------------------------------------

    def _voltage_to_percentage(self, cell_voltage: float) -> float:
        """Convert cell voltage to percentage using voltage curve.

        Args:
            cell_voltage: Voltage per cell.

        Returns:
            State of charge percentage (0-100).
        """
        curve = self._voltage_curve

        # Above full
        if cell_voltage >= curve[0][1]:
            return 100.0

        # Below empty
        if cell_voltage <= curve[-1][1]:
            return 0.0

        # Linear interpolation between curve points
        for i in range(len(curve) - 1):
            high_pct, high_v = curve[i]
            low_pct, low_v = curve[i + 1]

            if low_v <= cell_voltage <= high_v:
                # Linear interpolation
                ratio = (cell_voltage - low_v) / (high_v - low_v)
                return low_pct + ratio * (high_pct - low_pct)

        return 0.0

    def _determine_state(self, percentage: float, current: float) -> BatteryState:
        """Determine battery state from readings.

        Args:
            percentage: State of charge.
            current: Current draw (positive = discharge).

        Returns:
            Battery state.
        """
        if percentage < self._config.critical_threshold:
            return BatteryState.CRITICAL
        elif percentage < self._config.low_threshold:
            return BatteryState.LOW
        elif percentage > 95.0:
            if current < 0:
                return BatteryState.CHARGING
            return BatteryState.FULL
        elif current < 0:
            return BatteryState.CHARGING
        else:
            return BatteryState.DISCHARGING


# =============================================================================
# Simulated Battery Monitor
# =============================================================================


class SimulatedBatteryMonitor(BatteryMonitor):
    """Simulated battery monitor for testing.

    Simulates battery discharge and charge cycles without hardware.
    """

    def __init__(
        self,
        cells: int = 3,
        chemistry: BatteryChemistry | str = BatteryChemistry.LIPO,
        initial_percentage: float = 80.0,
        discharge_rate: float = 0.01,  # % per second
        **kwargs: Any,
    ) -> None:
        """Initialize simulated battery monitor.

        Args:
            cells: Number of cells.
            chemistry: Battery chemistry.
            initial_percentage: Initial state of charge.
            discharge_rate: Simulated discharge rate (% per second).
            **kwargs: Additional arguments passed to BatteryMonitor.
        """
        super().__init__(cells=cells, chemistry=chemistry, **kwargs)

        self._initial_percentage = initial_percentage
        self._discharge_rate = discharge_rate
        self._start_time: float | None = None

    def enable(self) -> None:
        """Enable simulated battery."""
        super().enable()
        self._start_time = time.time()

    def read(self) -> BatteryReading:
        """Read simulated battery state."""
        if not self._enabled:
            raise RuntimeError("SimulatedBatteryMonitor is not enabled")

        # Calculate elapsed time and simulated discharge
        if self._start_time is None:
            self._start_time = time.time()

        elapsed = time.time() - self._start_time
        percentage = max(0, self._initial_percentage - (elapsed * self._discharge_rate))

        # Calculate voltage from percentage
        cell_voltage = self._percentage_to_voltage(percentage)
        voltage = cell_voltage * self._config.cells
        current = 0.5  # Simulated constant current
        power = voltage * current

        state = self._determine_state(percentage, current)

        reading = BatteryReading(
            voltage=voltage,
            current=current,
            power=power,
            percentage=percentage,
            cell_voltage=cell_voltage,
            state=state,
            temperature=25.0,  # Room temperature
        )

        self._last_reading = reading
        return reading

    def _percentage_to_voltage(self, percentage: float) -> float:
        """Convert percentage to cell voltage using voltage curve.

        Args:
            percentage: State of charge (0-100).

        Returns:
            Cell voltage.
        """
        curve = self._voltage_curve

        # Above full
        if percentage >= 100:
            return curve[0][1]

        # Below empty
        if percentage <= 0:
            return curve[-1][1]

        # Linear interpolation
        for i in range(len(curve) - 1):
            high_pct, high_v = curve[i]
            low_pct, low_v = curve[i + 1]

            if low_pct <= percentage <= high_pct:
                ratio = (percentage - low_pct) / (high_pct - low_pct)
                return low_v + ratio * (high_v - low_v)

        return curve[-1][1]


# =============================================================================
# Factory Function
# =============================================================================


def get_battery_monitor(
    cells: int = 3,
    chemistry: BatteryChemistry | str = BatteryChemistry.LIPO,
    simulated: bool = False,
    **kwargs: Any,
) -> BatteryMonitor:
    """Get a battery monitor instance.

    Args:
        cells: Number of cells in series.
        chemistry: Battery chemistry type.
        simulated: Whether to use simulated monitor.
        **kwargs: Additional configuration options.

    Returns:
        BatteryMonitor or SimulatedBatteryMonitor instance.

    Example:
        >>> battery = get_battery_monitor(cells=4, chemistry="lipo")
        >>> battery = get_battery_monitor(simulated=True)
    """
    if simulated:
        return SimulatedBatteryMonitor(cells=cells, chemistry=chemistry, **kwargs)
    return BatteryMonitor(cells=cells, chemistry=chemistry, **kwargs)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "LEAD_ACID_VOLTAGE_CURVE",
    "LIFEPO4_VOLTAGE_CURVE",
    "LIION_VOLTAGE_CURVE",
    # Voltage curves
    "LIPO_VOLTAGE_CURVE",
    "NIMH_VOLTAGE_CURVE",
    # Enums
    "BatteryChemistry",
    # Config
    "BatteryConfig",
    # Classes
    "BatteryMonitor",
    # Data classes
    "BatteryReading",
    "BatteryState",
    "SimulatedBatteryMonitor",
    # Factory
    "get_battery_monitor",
]
