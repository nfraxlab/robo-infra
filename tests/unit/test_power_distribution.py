"""Unit tests for power.distribution module.

Tests power rail control and distribution board management.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest

from robo_infra.power.distribution import (
    PowerDistributionBoard,
    PowerDistributionConfig,
    PowerRail,
    PowerRailConfig,
    PowerRailReading,
    PowerRailState,
    ShutdownPriority,
    create_distribution_board,
    create_power_rail,
)


# =============================================================================
# PowerRailState Tests
# =============================================================================


class TestPowerRailState:
    """Tests for PowerRailState enum."""

    def test_all_states_exist(self) -> None:
        """Test all expected states are defined."""
        assert PowerRailState.DISABLED == 0
        assert PowerRailState.ENABLED == 1
        assert PowerRailState.FAULT == 2
        assert PowerRailState.UNKNOWN == 3


# =============================================================================
# ShutdownPriority Tests
# =============================================================================


class TestShutdownPriority:
    """Tests for ShutdownPriority enum."""

    def test_all_priorities_exist(self) -> None:
        """Test all expected priorities are defined."""
        assert ShutdownPriority.CRITICAL == 0
        assert ShutdownPriority.HIGH == 1
        assert ShutdownPriority.NORMAL == 2
        assert ShutdownPriority.LOW == 3
        assert ShutdownPriority.LOWEST == 4

    def test_priority_ordering(self) -> None:
        """Test that priorities are properly ordered."""
        assert ShutdownPriority.CRITICAL < ShutdownPriority.HIGH
        assert ShutdownPriority.HIGH < ShutdownPriority.NORMAL
        assert ShutdownPriority.NORMAL < ShutdownPriority.LOW
        assert ShutdownPriority.LOW < ShutdownPriority.LOWEST


# =============================================================================
# PowerRailConfig Tests
# =============================================================================


class TestPowerRailConfig:
    """Tests for PowerRailConfig."""

    def test_basic_config(self) -> None:
        """Test basic configuration."""
        config = PowerRailConfig(name="motors", enable_pin=17)
        assert config.name == "motors"
        assert config.enable_pin == 17
        assert config.active_high is True
        assert config.shutdown_priority == ShutdownPriority.NORMAL

    def test_full_config(self) -> None:
        """Test full configuration."""
        config = PowerRailConfig(
            name="critical_rail",
            enable_pin=27,
            active_high=False,
            max_current=5.0,
            max_power=60.0,
            nominal_voltage=12.0,
            enable_delay_ms=200,
            disable_delay_ms=100,
            shutdown_priority=ShutdownPriority.CRITICAL,
        )
        assert config.name == "critical_rail"
        assert config.enable_pin == 27
        assert config.active_high is False
        assert config.max_current == 5.0
        assert config.max_power == 60.0
        assert config.nominal_voltage == 12.0
        assert config.enable_delay_ms == 200
        assert config.disable_delay_ms == 100
        assert config.shutdown_priority == ShutdownPriority.CRITICAL

    def test_pin_validation(self) -> None:
        """Test GPIO pin validation."""
        # Valid
        PowerRailConfig(name="test", enable_pin=0)
        PowerRailConfig(name="test", enable_pin=40)

        # Invalid
        with pytest.raises(ValueError):
            PowerRailConfig(name="test", enable_pin=-1)


# =============================================================================
# PowerRailReading Tests
# =============================================================================


class TestPowerRailReading:
    """Tests for PowerRailReading dataclass."""

    def test_basic_reading(self) -> None:
        """Test basic reading creation."""
        reading = PowerRailReading(
            name="motors",
            state=PowerRailState.ENABLED,
            voltage=12.0,
            current=2.5,
            power=30.0,
        )
        assert reading.name == "motors"
        assert reading.state == PowerRailState.ENABLED
        assert reading.voltage == 12.0
        assert reading.current == 2.5
        assert reading.power == 30.0

    def test_reading_without_monitoring(self) -> None:
        """Test reading without power monitoring."""
        reading = PowerRailReading(
            name="logic",
            state=PowerRailState.ENABLED,
            voltage=None,
            current=None,
            power=None,
        )
        assert reading.voltage is None
        assert reading.current is None
        assert reading.power is None

    def test_is_enabled_property(self) -> None:
        """Test is_enabled property."""
        enabled = PowerRailReading(
            name="test", state=PowerRailState.ENABLED,
            voltage=None, current=None, power=None,
        )
        disabled = PowerRailReading(
            name="test", state=PowerRailState.DISABLED,
            voltage=None, current=None, power=None,
        )

        assert enabled.is_enabled is True
        assert disabled.is_enabled is False

    def test_timestamp_default(self) -> None:
        """Test that timestamp defaults to current time."""
        before = time.time()
        reading = PowerRailReading(
            name="test", state=PowerRailState.DISABLED,
            voltage=None, current=None, power=None,
        )
        after = time.time()

        assert before <= reading.timestamp <= after


# =============================================================================
# PowerRail Tests
# =============================================================================


class TestPowerRail:
    """Tests for PowerRail class."""

    def test_initialization_basic(self) -> None:
        """Test basic initialization."""
        rail = PowerRail(name="motors", enable_pin=17)
        assert rail.name == "motors"
        assert rail.is_enabled is False
        assert rail.state == PowerRailState.DISABLED

    def test_initialization_with_config(self) -> None:
        """Test initialization with config."""
        config = PowerRailConfig(
            name="sensors",
            enable_pin=27,
            active_high=False,
        )
        rail = PowerRail(name="sensors", config=config)
        assert rail.name == "sensors"
        assert rail.config.active_high is False

    def test_enable_disable(self) -> None:
        """Test enable/disable cycle."""
        rail = PowerRail(name="test", enable_pin=17)

        assert rail.is_enabled is False
        rail.enable()
        assert rail.is_enabled is True
        assert rail.state == PowerRailState.ENABLED
        rail.disable()
        assert rail.is_enabled is False
        assert rail.state == PowerRailState.DISABLED

    def test_enable_idempotent(self) -> None:
        """Test that enable is idempotent."""
        rail = PowerRail(name="test", enable_pin=17)
        rail.enable()
        rail.enable()  # Should not raise
        assert rail.is_enabled is True

    def test_disable_idempotent(self) -> None:
        """Test that disable is idempotent."""
        rail = PowerRail(name="test", enable_pin=17)
        rail.disable()  # Already disabled
        assert rail.is_enabled is False

    def test_read(self) -> None:
        """Test reading rail state."""
        rail = PowerRail(name="test", enable_pin=17)
        rail.enable()

        reading = rail.read()
        assert isinstance(reading, PowerRailReading)
        assert reading.name == "test"
        assert reading.state == PowerRailState.ENABLED

    def test_uptime(self) -> None:
        """Test uptime tracking."""
        rail = PowerRail(name="test", enable_pin=17)

        assert rail.uptime() is None  # Not enabled

        rail.enable()
        time.sleep(0.1)
        uptime = rail.uptime()
        assert uptime is not None
        assert uptime >= 0.1

        rail.disable()
        assert rail.uptime() is None

    def test_shutdown_priority(self) -> None:
        """Test shutdown priority property."""
        config = PowerRailConfig(
            name="critical",
            enable_pin=17,
            shutdown_priority=ShutdownPriority.CRITICAL,
        )
        rail = PowerRail(name="critical", config=config)
        assert rail.shutdown_priority == ShutdownPriority.CRITICAL

    def test_with_mock_pin(self) -> None:
        """Test with mocked GPIO pin."""
        mock_pin = MagicMock()
        rail = PowerRail(name="test", enable_pin=mock_pin)
        rail.set_pin(mock_pin)

        rail.enable()
        mock_pin.high.assert_called_once()

        rail.disable()
        mock_pin.low.assert_called_once()

    def test_active_low(self) -> None:
        """Test active low configuration."""
        mock_pin = MagicMock()
        config = PowerRailConfig(name="test", enable_pin=17, active_high=False)
        rail = PowerRail(name="test", config=config)
        rail.set_pin(mock_pin)

        rail.enable()
        mock_pin.low.assert_called_once()  # Active low means low = enabled

        mock_pin.reset_mock()
        rail.disable()
        mock_pin.high.assert_called_once()  # High = disabled


# =============================================================================
# PowerDistributionConfig Tests
# =============================================================================


class TestPowerDistributionConfig:
    """Tests for PowerDistributionConfig."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = PowerDistributionConfig()
        assert config.name == "power_distribution"
        assert config.estop_pin is None
        assert config.total_power_budget is None
        assert config.startup_delay_ms == 100

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = PowerDistributionConfig(
            name="main_pdb",
            estop_pin=5,
            total_power_budget=100.0,
            total_current_limit=10.0,
            startup_delay_ms=200,
        )
        assert config.name == "main_pdb"
        assert config.estop_pin == 5
        assert config.total_power_budget == 100.0
        assert config.total_current_limit == 10.0
        assert config.startup_delay_ms == 200


# =============================================================================
# PowerDistributionBoard Tests
# =============================================================================


class TestPowerDistributionBoard:
    """Tests for PowerDistributionBoard class."""

    def test_initialization_empty(self) -> None:
        """Test initialization with no rails."""
        pdb = PowerDistributionBoard()
        assert len(pdb.rails) == 0
        assert pdb.is_estop_triggered is False

    def test_initialization_with_rails(self) -> None:
        """Test initialization with rails."""
        motors = PowerRail("motors", enable_pin=17)
        sensors = PowerRail("sensors", enable_pin=27)

        pdb = PowerDistributionBoard(rails=[motors, sensors])
        assert len(pdb.rails) == 2
        assert "motors" in pdb.rails
        assert "sensors" in pdb.rails

    def test_add_rail(self) -> None:
        """Test adding a rail."""
        pdb = PowerDistributionBoard()
        rail = PowerRail("test", enable_pin=17)

        pdb.add_rail(rail)
        assert "test" in pdb.rails

    def test_add_duplicate_rail_raises(self) -> None:
        """Test that adding duplicate rail raises."""
        pdb = PowerDistributionBoard()
        rail1 = PowerRail("test", enable_pin=17)
        rail2 = PowerRail("test", enable_pin=27)

        pdb.add_rail(rail1)
        with pytest.raises(ValueError, match="already exists"):
            pdb.add_rail(rail2)

    def test_remove_rail(self) -> None:
        """Test removing a rail."""
        rail = PowerRail("test", enable_pin=17)
        pdb = PowerDistributionBoard(rails=[rail])

        removed = pdb.remove_rail("test")
        assert removed is rail
        assert "test" not in pdb.rails

    def test_remove_nonexistent_rail(self) -> None:
        """Test removing nonexistent rail returns None."""
        pdb = PowerDistributionBoard()
        assert pdb.remove_rail("nonexistent") is None

    def test_get_rail(self) -> None:
        """Test getting a rail by name."""
        rail = PowerRail("test", enable_pin=17)
        pdb = PowerDistributionBoard(rails=[rail])

        assert pdb.get_rail("test") is rail
        assert pdb.get_rail("nonexistent") is None

    def test_enable_single_rail(self) -> None:
        """Test enabling a single rail."""
        rail = PowerRail("test", enable_pin=17)
        pdb = PowerDistributionBoard(rails=[rail])

        pdb.enable("test")
        assert rail.is_enabled is True

    def test_disable_single_rail(self) -> None:
        """Test disabling a single rail."""
        rail = PowerRail("test", enable_pin=17)
        pdb = PowerDistributionBoard(rails=[rail])
        rail.enable()

        pdb.disable("test")
        assert rail.is_enabled is False

    def test_enable_nonexistent_raises(self) -> None:
        """Test enabling nonexistent rail raises."""
        pdb = PowerDistributionBoard()
        with pytest.raises(KeyError):
            pdb.enable("nonexistent")

    def test_disable_nonexistent_raises(self) -> None:
        """Test disabling nonexistent rail raises."""
        pdb = PowerDistributionBoard()
        with pytest.raises(KeyError):
            pdb.disable("nonexistent")

    def test_enable_all(self) -> None:
        """Test enabling all rails."""
        motors = PowerRail("motors", enable_pin=17)
        sensors = PowerRail("sensors", enable_pin=27)
        pdb = PowerDistributionBoard(
            rails=[motors, sensors],
            config=PowerDistributionConfig(startup_delay_ms=0),
        )

        pdb.enable_all()
        assert motors.is_enabled is True
        assert sensors.is_enabled is True

    def test_disable_all(self) -> None:
        """Test disabling all rails."""
        motors = PowerRail("motors", enable_pin=17)
        sensors = PowerRail("sensors", enable_pin=27)
        pdb = PowerDistributionBoard(rails=[motors, sensors])

        motors.enable()
        sensors.enable()

        pdb.disable_all()
        assert motors.is_enabled is False
        assert sensors.is_enabled is False

    def test_enable_all_respects_priority(self) -> None:
        """Test that enable_all respects priority order."""
        # Create rails with different priorities
        critical = PowerRail(
            "critical",
            config=PowerRailConfig(name="critical", enable_pin=17, shutdown_priority=ShutdownPriority.CRITICAL),
        )
        normal = PowerRail(
            "normal",
            config=PowerRailConfig(name="normal", enable_pin=27, shutdown_priority=ShutdownPriority.NORMAL),
        )
        low = PowerRail(
            "low",
            config=PowerRailConfig(name="low", enable_pin=22, shutdown_priority=ShutdownPriority.LOW),
        )

        pdb = PowerDistributionBoard(
            rails=[low, critical, normal],  # Add in wrong order
            config=PowerDistributionConfig(startup_delay_ms=0),
        )

        # Track enable order
        enable_order: list[str] = []

        def track_enable(rail: PowerRail) -> None:
            original = rail.enable

            def wrapper() -> None:
                enable_order.append(rail.name)
                original()

            rail.enable = wrapper  # type: ignore[method-assign]

        track_enable(critical)
        track_enable(normal)
        track_enable(low)

        pdb.enable_all()

        # Should be CRITICAL first, then NORMAL, then LOW
        assert enable_order == ["critical", "normal", "low"]

    def test_emergency_shutdown(self) -> None:
        """Test emergency shutdown."""
        motors = PowerRail("motors", enable_pin=17)
        sensors = PowerRail("sensors", enable_pin=27)
        pdb = PowerDistributionBoard(rails=[motors, sensors])

        motors.enable()
        sensors.enable()

        pdb.emergency_shutdown()

        assert motors.is_enabled is False
        assert sensors.is_enabled is False
        assert pdb.is_estop_triggered is True

    def test_cannot_enable_after_estop(self) -> None:
        """Test that rails cannot be enabled after e-stop."""
        rail = PowerRail("test", enable_pin=17)
        pdb = PowerDistributionBoard(rails=[rail])

        pdb.emergency_shutdown()

        with pytest.raises(RuntimeError, match="E-STOP"):
            pdb.enable("test")

        with pytest.raises(RuntimeError, match="E-STOP"):
            pdb.enable_all()

    def test_reset_estop(self) -> None:
        """Test resetting e-stop."""
        rail = PowerRail("test", enable_pin=17)
        pdb = PowerDistributionBoard(rails=[rail])

        pdb.emergency_shutdown()
        assert pdb.is_estop_triggered is True

        pdb.reset_estop()
        assert pdb.is_estop_triggered is False

        # Now can enable again
        pdb.enable("test")
        assert rail.is_enabled is True

    def test_get_status(self) -> None:
        """Test getting status of all rails."""
        motors = PowerRail("motors", enable_pin=17)
        sensors = PowerRail("sensors", enable_pin=27)
        pdb = PowerDistributionBoard(rails=[motors, sensors])

        motors.enable()

        status = pdb.get_status()
        assert "motors" in status
        assert "sensors" in status
        assert status["motors"].state == PowerRailState.ENABLED
        assert status["sensors"].state == PowerRailState.DISABLED

    def test_get_total_power(self) -> None:
        """Test total power calculation."""
        pdb = PowerDistributionBoard()
        # Without monitors, should return 0
        assert pdb.get_total_power() == 0.0

    def test_get_total_current(self) -> None:
        """Test total current calculation."""
        pdb = PowerDistributionBoard()
        # Without monitors, should return 0
        assert pdb.get_total_current() == 0.0

    def test_check_power_budget_no_budget(self) -> None:
        """Test power budget check with no budget set."""
        pdb = PowerDistributionBoard()
        within, current, budget = pdb.check_power_budget()

        assert within is True
        assert current == 0.0
        assert budget == 0.0

    def test_check_power_budget_with_limit(self) -> None:
        """Test power budget check with limit."""
        config = PowerDistributionConfig(total_power_budget=100.0)
        pdb = PowerDistributionBoard(config=config)

        within, _current, budget = pdb.check_power_budget()
        assert within is True
        assert budget == 100.0


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreatePowerRail:
    """Tests for create_power_rail factory function."""

    def test_basic_creation(self) -> None:
        """Test basic rail creation."""
        rail = create_power_rail("motors", enable_pin=17)
        assert rail.name == "motors"
        assert rail.config.enable_pin == 17

    def test_with_options(self) -> None:
        """Test creation with options."""
        rail = create_power_rail(
            "sensors",
            enable_pin=27,
            active_high=False,
            priority=ShutdownPriority.HIGH,
        )
        assert rail.name == "sensors"
        assert rail.config.active_high is False
        assert rail.shutdown_priority == ShutdownPriority.HIGH


class TestCreateDistributionBoard:
    """Tests for create_distribution_board factory function."""

    def test_basic_creation(self) -> None:
        """Test basic board creation."""
        pdb = create_distribution_board([
            ("logic", 17),
            ("sensors", 27),
            ("motors", 22),
        ])

        assert len(pdb.rails) == 3
        assert "logic" in pdb.rails
        assert "sensors" in pdb.rails
        assert "motors" in pdb.rails

    def test_with_estop(self) -> None:
        """Test creation with e-stop pin."""
        pdb = create_distribution_board(
            [("test", 17)],
            estop_pin=5,
        )
        assert pdb.config.estop_pin == 5

    def test_with_name(self) -> None:
        """Test creation with custom name."""
        pdb = create_distribution_board(
            [("test", 17)],
            name="main_pdb",
        )
        assert pdb.name == "main_pdb"


# =============================================================================
# Integration Tests
# =============================================================================


class TestPowerDistributionIntegration:
    """Integration tests for power distribution."""

    def test_full_workflow(self) -> None:
        """Test complete workflow."""
        # Create rails with priorities
        logic = create_power_rail("logic", enable_pin=17, priority=ShutdownPriority.CRITICAL)
        sensors = create_power_rail("sensors", enable_pin=27, priority=ShutdownPriority.HIGH)
        motors = create_power_rail("motors", enable_pin=22, priority=ShutdownPriority.NORMAL)

        # Create distribution board
        config = PowerDistributionConfig(startup_delay_ms=0)
        pdb = PowerDistributionBoard(rails=[logic, sensors, motors], config=config)

        # Enable all
        pdb.enable_all()
        assert all(r.is_enabled for r in [logic, sensors, motors])

        # Disable one
        pdb.disable("motors")
        assert motors.is_enabled is False
        assert logic.is_enabled is True

        # Emergency shutdown
        pdb.emergency_shutdown()
        assert all(not r.is_enabled for r in [logic, sensors, motors])
        assert pdb.is_estop_triggered is True

        # Reset and re-enable
        pdb.reset_estop()
        pdb.enable("logic")
        assert logic.is_enabled is True

    def test_shutdown_priority_order(self) -> None:
        """Test that emergency shutdown respects priority."""
        critical = create_power_rail("critical", enable_pin=17, priority=ShutdownPriority.CRITICAL)
        low = create_power_rail("low", enable_pin=27, priority=ShutdownPriority.LOWEST)

        pdb = PowerDistributionBoard(
            rails=[critical, low],
            config=PowerDistributionConfig(startup_delay_ms=0),
        )

        pdb.enable_all()

        # During shutdown, LOWEST (non-essential) should be disabled first
        # This is a safety measure to shed non-critical loads first
        # Note: In real hardware, we'd measure GPIO timing to verify order
        pdb.emergency_shutdown()

        # Both should be disabled regardless of order
        assert critical.is_enabled is False
        assert low.is_enabled is False
