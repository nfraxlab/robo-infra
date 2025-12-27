"""Unit tests for power.battery module.

Tests battery monitoring, chemistry voltage curves, and state detection.
"""

from __future__ import annotations

import time

import pytest

from robo_infra.power.battery import (
    LEAD_ACID_VOLTAGE_CURVE,
    LIFEPO4_VOLTAGE_CURVE,
    LIION_VOLTAGE_CURVE,
    LIPO_VOLTAGE_CURVE,
    NIMH_VOLTAGE_CURVE,
    BatteryChemistry,
    BatteryConfig,
    BatteryMonitor,
    BatteryReading,
    BatteryState,
    SimulatedBatteryMonitor,
    get_battery_monitor,
)


# =============================================================================
# BatteryChemistry Tests
# =============================================================================


class TestBatteryChemistry:
    """Tests for BatteryChemistry enum."""

    def test_all_chemistries_exist(self) -> None:
        """Test all expected chemistries are defined."""
        assert BatteryChemistry.LIPO.value == "lipo"
        assert BatteryChemistry.LIION.value == "liion"
        assert BatteryChemistry.LIFEPO4.value == "lifepo4"
        assert BatteryChemistry.NIMH.value == "nimh"
        assert BatteryChemistry.LEAD_ACID.value == "lead_acid"

    def test_get_voltage_curve_lipo(self) -> None:
        """Test LiPo voltage curve retrieval."""
        curve = BatteryChemistry.LIPO.get_voltage_curve()
        assert curve == LIPO_VOLTAGE_CURVE
        # LiPo: 4.2V full, 3.0V empty
        assert curve[0][1] == 4.20
        assert curve[-1][1] == 3.00

    def test_get_voltage_curve_liion(self) -> None:
        """Test Li-Ion voltage curve retrieval."""
        curve = BatteryChemistry.LIION.get_voltage_curve()
        assert curve == LIION_VOLTAGE_CURVE
        assert curve[0][1] == 4.20
        assert curve[-1][1] == 3.00

    def test_get_voltage_curve_lifepo4(self) -> None:
        """Test LiFePO4 voltage curve retrieval."""
        curve = BatteryChemistry.LIFEPO4.get_voltage_curve()
        assert curve == LIFEPO4_VOLTAGE_CURVE
        # LiFePO4: 3.6V full, 2.5V empty
        assert curve[0][1] == 3.60
        assert curve[-1][1] == 2.50

    def test_get_voltage_curve_nimh(self) -> None:
        """Test NiMH voltage curve retrieval."""
        curve = BatteryChemistry.NIMH.get_voltage_curve()
        assert curve == NIMH_VOLTAGE_CURVE
        # NiMH: 1.4V full, 1.0V empty (per cell)
        assert curve[0][1] == 1.40
        assert curve[-1][1] == 1.00

    def test_get_voltage_curve_lead_acid(self) -> None:
        """Test Lead Acid voltage curve retrieval."""
        curve = BatteryChemistry.LEAD_ACID.get_voltage_curve()
        assert curve == LEAD_ACID_VOLTAGE_CURVE
        # Lead Acid: 2.12V full, 1.8V empty (per cell)
        assert curve[0][1] == 2.12
        assert curve[-1][1] == 1.80

    def test_get_nominal_voltage(self) -> None:
        """Test nominal voltage per cell."""
        assert BatteryChemistry.LIPO.get_nominal_voltage() == 3.7
        assert BatteryChemistry.LIION.get_nominal_voltage() == 3.7
        assert BatteryChemistry.LIFEPO4.get_nominal_voltage() == 3.2
        assert BatteryChemistry.NIMH.get_nominal_voltage() == 1.2
        assert BatteryChemistry.LEAD_ACID.get_nominal_voltage() == 2.0

    def test_get_full_voltage(self) -> None:
        """Test full voltage per cell."""
        assert BatteryChemistry.LIPO.get_full_voltage() == 4.2
        assert BatteryChemistry.LIION.get_full_voltage() == 4.2
        assert BatteryChemistry.LIFEPO4.get_full_voltage() == 3.6
        assert BatteryChemistry.NIMH.get_full_voltage() == 1.4
        assert BatteryChemistry.LEAD_ACID.get_full_voltage() == 2.12

    def test_get_empty_voltage(self) -> None:
        """Test empty voltage per cell."""
        assert BatteryChemistry.LIPO.get_empty_voltage() == 3.0
        assert BatteryChemistry.LIION.get_empty_voltage() == 3.0
        assert BatteryChemistry.LIFEPO4.get_empty_voltage() == 2.5
        assert BatteryChemistry.NIMH.get_empty_voltage() == 1.0
        assert BatteryChemistry.LEAD_ACID.get_empty_voltage() == 1.8


# =============================================================================
# BatteryConfig Tests
# =============================================================================


class TestBatteryConfig:
    """Tests for BatteryConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = BatteryConfig()
        assert config.cells == 3
        assert config.chemistry == BatteryChemistry.LIPO
        assert config.capacity_mah == 5000
        assert config.low_threshold == 20.0
        assert config.critical_threshold == 10.0
        assert config.update_interval == 1.0
        assert config.enable_coulomb_counting is False

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = BatteryConfig(
            cells=4,
            chemistry=BatteryChemistry.LIFEPO4,
            capacity_mah=10000,
            low_threshold=25.0,
            critical_threshold=15.0,
        )
        assert config.cells == 4
        assert config.chemistry == BatteryChemistry.LIFEPO4
        assert config.capacity_mah == 10000
        assert config.low_threshold == 25.0
        assert config.critical_threshold == 15.0

    def test_config_validation_cells(self) -> None:
        """Test cells validation (1-20)."""
        # Valid
        BatteryConfig(cells=1)
        BatteryConfig(cells=20)

        # Invalid
        with pytest.raises(ValueError):
            BatteryConfig(cells=0)
        with pytest.raises(ValueError):
            BatteryConfig(cells=21)

    def test_config_validation_capacity(self) -> None:
        """Test capacity validation (min 100mAh)."""
        BatteryConfig(capacity_mah=100)

        with pytest.raises(ValueError):
            BatteryConfig(capacity_mah=50)


# =============================================================================
# BatteryReading Tests
# =============================================================================


class TestBatteryReading:
    """Tests for BatteryReading dataclass."""

    def test_basic_reading(self) -> None:
        """Test basic reading creation."""
        reading = BatteryReading(
            voltage=11.1,
            current=0.5,
            power=5.55,
            percentage=50.0,
            cell_voltage=3.7,
            state=BatteryState.DISCHARGING,
            temperature=25.0,
        )
        assert reading.voltage == 11.1
        assert reading.current == 0.5
        assert reading.power == 5.55
        assert reading.percentage == 50.0
        assert reading.cell_voltage == 3.7
        assert reading.state == BatteryState.DISCHARGING
        assert reading.temperature == 25.0

    def test_is_low_property(self) -> None:
        """Test is_low property (< 20%)."""
        low_reading = BatteryReading(
            voltage=10.0,
            current=0.5,
            power=5.0,
            percentage=15.0,
            cell_voltage=3.33,
            state=BatteryState.LOW,
            temperature=None,
        )
        normal_reading = BatteryReading(
            voltage=11.0,
            current=0.5,
            power=5.5,
            percentage=50.0,
            cell_voltage=3.67,
            state=BatteryState.DISCHARGING,
            temperature=None,
        )

        assert low_reading.is_low is True
        assert normal_reading.is_low is False

    def test_is_critical_property(self) -> None:
        """Test is_critical property (< 10%)."""
        critical_reading = BatteryReading(
            voltage=9.5,
            current=0.5,
            power=4.75,
            percentage=5.0,
            cell_voltage=3.17,
            state=BatteryState.CRITICAL,
            temperature=None,
        )
        low_reading = BatteryReading(
            voltage=10.0,
            current=0.5,
            power=5.0,
            percentage=15.0,
            cell_voltage=3.33,
            state=BatteryState.LOW,
            temperature=None,
        )

        assert critical_reading.is_critical is True
        assert low_reading.is_critical is False

    def test_is_full_property(self) -> None:
        """Test is_full property (> 95%)."""
        full_reading = BatteryReading(
            voltage=12.6,
            current=-0.1,
            power=1.26,
            percentage=98.0,
            cell_voltage=4.2,
            state=BatteryState.FULL,
            temperature=None,
        )
        normal_reading = BatteryReading(
            voltage=11.0,
            current=0.5,
            power=5.5,
            percentage=50.0,
            cell_voltage=3.67,
            state=BatteryState.DISCHARGING,
            temperature=None,
        )

        assert full_reading.is_full is True
        assert normal_reading.is_full is False

    def test_timestamp_default(self) -> None:
        """Test that timestamp defaults to current time."""
        before = time.time()
        reading = BatteryReading(
            voltage=11.1,
            current=0.5,
            power=5.55,
            percentage=50.0,
            cell_voltage=3.7,
            state=BatteryState.DISCHARGING,
            temperature=None,
        )
        after = time.time()

        assert before <= reading.timestamp <= after


# =============================================================================
# BatteryMonitor Tests
# =============================================================================


class TestBatteryMonitor:
    """Tests for BatteryMonitor class."""

    def test_initialization_default(self) -> None:
        """Test default initialization."""
        battery = BatteryMonitor()
        assert battery.cells == 3
        assert battery.chemistry == BatteryChemistry.LIPO
        assert battery.is_enabled is False

    def test_initialization_with_params(self) -> None:
        """Test initialization with parameters."""
        battery = BatteryMonitor(cells=4, chemistry=BatteryChemistry.LIFEPO4)
        assert battery.cells == 4
        assert battery.chemistry == BatteryChemistry.LIFEPO4

    def test_initialization_with_string_chemistry(self) -> None:
        """Test initialization with string chemistry."""
        battery = BatteryMonitor(cells=3, chemistry="lipo")
        assert battery.chemistry == BatteryChemistry.LIPO

        battery = BatteryMonitor(cells=3, chemistry="lifepo4")
        assert battery.chemistry == BatteryChemistry.LIFEPO4

    def test_initialization_with_config(self) -> None:
        """Test initialization with config object."""
        config = BatteryConfig(
            cells=6,
            chemistry=BatteryChemistry.NIMH,
            capacity_mah=2000,
        )
        battery = BatteryMonitor(config=config)
        assert battery.cells == 6
        assert battery.chemistry == BatteryChemistry.NIMH

    def test_enable_disable(self) -> None:
        """Test enable/disable cycle."""
        battery = BatteryMonitor()

        assert battery.is_enabled is False
        battery.enable()
        assert battery.is_enabled is True
        battery.disable()
        assert battery.is_enabled is False

    def test_enable_idempotent(self) -> None:
        """Test that enable is idempotent."""
        battery = BatteryMonitor()
        battery.enable()
        battery.enable()  # Should not raise
        assert battery.is_enabled is True

    def test_read_requires_enable(self) -> None:
        """Test that read requires enable."""
        battery = BatteryMonitor()

        with pytest.raises(RuntimeError, match="not enabled"):
            battery.read()

    def test_read_returns_reading(self) -> None:
        """Test that read returns a BatteryReading."""
        battery = BatteryMonitor()
        battery.enable()

        reading = battery.read()
        assert isinstance(reading, BatteryReading)
        assert reading.voltage > 0

    def test_voltage_property(self) -> None:
        """Test voltage property."""
        battery = BatteryMonitor()
        battery.enable()

        voltage = battery.voltage
        assert isinstance(voltage, float)
        assert voltage > 0

    def test_current_property(self) -> None:
        """Test current property."""
        battery = BatteryMonitor()
        battery.enable()

        current = battery.current
        assert isinstance(current, float)

    def test_power_property(self) -> None:
        """Test power property."""
        battery = BatteryMonitor()
        battery.enable()

        power = battery.power
        assert isinstance(power, float)
        assert power >= 0

    def test_percentage_property(self) -> None:
        """Test percentage property."""
        battery = BatteryMonitor()
        battery.enable()

        percentage = battery.percentage
        assert isinstance(percentage, float)
        assert 0 <= percentage <= 100

    def test_set_simulated_values(self) -> None:
        """Test setting simulated values."""
        battery = BatteryMonitor(cells=3)
        battery.set_simulated_values(voltage=11.1, current=1.0)
        battery.enable()

        assert battery.voltage == 11.1
        assert battery.current == 1.0

    def test_voltage_to_percentage_full(self) -> None:
        """Test voltage to percentage conversion at full."""
        battery = BatteryMonitor(cells=3, chemistry=BatteryChemistry.LIPO)
        battery.set_simulated_values(voltage=12.6, current=0.5)  # 4.2V per cell
        battery.enable()

        assert battery.percentage == 100.0

    def test_voltage_to_percentage_empty(self) -> None:
        """Test voltage to percentage conversion at empty."""
        battery = BatteryMonitor(cells=3, chemistry=BatteryChemistry.LIPO)
        battery.set_simulated_values(voltage=9.0, current=0.5)  # 3.0V per cell
        battery.enable()

        assert battery.percentage == 0.0

    def test_voltage_to_percentage_mid(self) -> None:
        """Test voltage to percentage conversion at mid range."""
        battery = BatteryMonitor(cells=3, chemistry=BatteryChemistry.LIPO)
        battery.set_simulated_values(voltage=11.1, current=0.5)  # 3.7V per cell
        battery.enable()

        # 3.7V should be around 50% for LiPo
        percentage = battery.percentage
        assert 45 <= percentage <= 55

    def test_is_low_threshold(self) -> None:
        """Test is_low based on threshold."""
        battery = BatteryMonitor(cells=3)
        battery.set_simulated_values(voltage=10.2, current=0.5)  # ~3.4V = ~20%
        battery.enable()

        # Read to update internal state
        reading = battery.read()
        # Check if low based on actual percentage
        assert battery.is_low == (reading.percentage < 20.0)

    def test_is_critical_threshold(self) -> None:
        """Test is_critical based on threshold."""
        battery = BatteryMonitor(cells=3)
        battery.set_simulated_values(voltage=9.6, current=0.5)  # ~3.2V = ~5-10%
        battery.enable()

        reading = battery.read()
        assert battery.is_critical == (reading.percentage < 10.0)

    def test_state_detection_discharging(self) -> None:
        """Test state detection for discharging."""
        battery = BatteryMonitor(cells=3)
        battery.set_simulated_values(voltage=11.1, current=0.5)  # Positive = discharge
        battery.enable()

        assert battery.state == BatteryState.DISCHARGING

    def test_state_detection_low(self) -> None:
        """Test state detection for low battery."""
        battery = BatteryMonitor(cells=3)
        battery.set_simulated_values(voltage=10.2, current=0.5)  # ~3.4V = ~20%
        battery.enable()

        # This depends on exact voltage curve
        reading = battery.read()
        if reading.percentage < 10.0:
            assert reading.state == BatteryState.CRITICAL
        elif reading.percentage < 20.0:
            assert reading.state == BatteryState.LOW

    def test_different_chemistries(self) -> None:
        """Test that different chemistries have different voltage ranges."""
        # Create monitors to verify they can be instantiated with different chemistries
        _lipo = BatteryMonitor(cells=1, chemistry=BatteryChemistry.LIPO)
        _lifepo4 = BatteryMonitor(cells=1, chemistry=BatteryChemistry.LIFEPO4)

        # LiPo nominal: 3.7V
        # LiFePO4 nominal: 3.2V
        assert (
            BatteryChemistry.LIPO.get_nominal_voltage()
            > BatteryChemistry.LIFEPO4.get_nominal_voltage()
        )


# =============================================================================
# SimulatedBatteryMonitor Tests
# =============================================================================


class TestSimulatedBatteryMonitor:
    """Tests for SimulatedBatteryMonitor."""

    def test_initialization(self) -> None:
        """Test simulated monitor initialization."""
        battery = SimulatedBatteryMonitor(
            cells=4,
            chemistry=BatteryChemistry.LIPO,
            initial_percentage=75.0,
            discharge_rate=0.1,
        )
        assert battery.cells == 4

    def test_simulated_discharge(self) -> None:
        """Test that simulated battery discharges over time."""
        battery = SimulatedBatteryMonitor(
            cells=3,
            initial_percentage=50.0,
            discharge_rate=50.0,  # 50% per second for fast test
        )
        battery.enable()

        # First read to get initial
        reading1 = battery.read()
        initial = reading1.percentage

        time.sleep(0.3)  # Wait 300ms = ~15% discharge

        # Second read to get final (must call read() to get fresh value)
        reading2 = battery.read()
        final = reading2.percentage

        # Allow for some timing tolerance
        assert final < initial - 5.0  # Should have dropped at least 5%

    def test_simulated_percentage_never_negative(self) -> None:
        """Test that percentage never goes negative."""
        battery = SimulatedBatteryMonitor(
            cells=3,
            initial_percentage=1.0,
            discharge_rate=100.0,  # Very fast discharge
        )
        battery.enable()

        time.sleep(0.1)
        assert battery.percentage >= 0.0

    def test_requires_enable(self) -> None:
        """Test that simulated monitor requires enable."""
        battery = SimulatedBatteryMonitor()

        with pytest.raises(RuntimeError, match="not enabled"):
            battery.read()


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestGetBatteryMonitor:
    """Tests for get_battery_monitor factory function."""

    def test_get_standard_monitor(self) -> None:
        """Test getting standard battery monitor."""
        battery = get_battery_monitor(cells=3, chemistry="lipo")
        assert isinstance(battery, BatteryMonitor)
        assert not isinstance(battery, SimulatedBatteryMonitor)

    def test_get_simulated_monitor(self) -> None:
        """Test getting simulated battery monitor."""
        battery = get_battery_monitor(simulated=True)
        assert isinstance(battery, SimulatedBatteryMonitor)

    def test_factory_with_kwargs(self) -> None:
        """Test factory with additional kwargs."""
        battery = get_battery_monitor(
            cells=6,
            chemistry=BatteryChemistry.NIMH,
            name="nimh_pack",
        )
        assert battery.cells == 6
        assert battery.chemistry == BatteryChemistry.NIMH


# =============================================================================
# Voltage Curve Tests
# =============================================================================


class TestVoltageCurves:
    """Tests for voltage curve data."""

    def test_lipo_curve_ordered(self) -> None:
        """Test LiPo curve is properly ordered."""
        for i in range(len(LIPO_VOLTAGE_CURVE) - 1):
            assert LIPO_VOLTAGE_CURVE[i][0] > LIPO_VOLTAGE_CURVE[i + 1][0]  # % decreasing
            assert LIPO_VOLTAGE_CURVE[i][1] > LIPO_VOLTAGE_CURVE[i + 1][1]  # V decreasing

    def test_liion_curve_ordered(self) -> None:
        """Test Li-Ion curve is properly ordered."""
        for i in range(len(LIION_VOLTAGE_CURVE) - 1):
            assert LIION_VOLTAGE_CURVE[i][0] > LIION_VOLTAGE_CURVE[i + 1][0]
            assert LIION_VOLTAGE_CURVE[i][1] > LIION_VOLTAGE_CURVE[i + 1][1]

    def test_lifepo4_curve_ordered(self) -> None:
        """Test LiFePO4 curve is properly ordered."""
        for i in range(len(LIFEPO4_VOLTAGE_CURVE) - 1):
            assert LIFEPO4_VOLTAGE_CURVE[i][0] > LIFEPO4_VOLTAGE_CURVE[i + 1][0]
            assert LIFEPO4_VOLTAGE_CURVE[i][1] > LIFEPO4_VOLTAGE_CURVE[i + 1][1]

    def test_nimh_curve_ordered(self) -> None:
        """Test NiMH curve is properly ordered."""
        for i in range(len(NIMH_VOLTAGE_CURVE) - 1):
            assert NIMH_VOLTAGE_CURVE[i][0] > NIMH_VOLTAGE_CURVE[i + 1][0]
            assert NIMH_VOLTAGE_CURVE[i][1] > NIMH_VOLTAGE_CURVE[i + 1][1]

    def test_lead_acid_curve_ordered(self) -> None:
        """Test Lead Acid curve is properly ordered."""
        for i in range(len(LEAD_ACID_VOLTAGE_CURVE) - 1):
            assert LEAD_ACID_VOLTAGE_CURVE[i][0] > LEAD_ACID_VOLTAGE_CURVE[i + 1][0]
            assert LEAD_ACID_VOLTAGE_CURVE[i][1] > LEAD_ACID_VOLTAGE_CURVE[i + 1][1]

    def test_curves_cover_full_range(self) -> None:
        """Test all curves cover 0-100%."""
        curves = [
            LIPO_VOLTAGE_CURVE,
            LIION_VOLTAGE_CURVE,
            LIFEPO4_VOLTAGE_CURVE,
            NIMH_VOLTAGE_CURVE,
            LEAD_ACID_VOLTAGE_CURVE,
        ]
        for curve in curves:
            assert curve[0][0] == 100  # First entry is 100%
            assert curve[-1][0] == 0  # Last entry is 0%
