"""Tests for DegradedModeController (Phase 5.15.3).

Tests for graceful degradation when hardware fails.
"""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from robo_infra.utils.degraded import (
    DegradedComponent,
    DegradedModeController,
    DegradedModeStatus,
)


# =============================================================================
# DegradedComponent Tests
# =============================================================================


class TestDegradedComponent:
    """Tests for DegradedComponent dataclass."""

    def test_component_creation(self) -> None:
        """Test creating a DegradedComponent."""
        component = DegradedComponent(
            name="motor_1",
            reason="Communication failure",
        )

        assert component.name == "motor_1"
        assert component.reason == "Communication failure"
        assert component.degraded_at is not None
        assert component.last_known_value is None

    def test_component_with_last_value(self) -> None:
        """Test DegradedComponent with last known value."""
        component = DegradedComponent(
            name="sensor_1",
            reason="Read timeout",
            last_known_value=42.5,
        )

        assert component.last_known_value == 42.5


class TestDegradedModeStatus:
    """Tests for DegradedModeStatus dataclass."""

    def test_status_creation(self) -> None:
        """Test creating DegradedModeStatus."""
        status = DegradedModeStatus(
            is_degraded=True,
            degraded_actuators={"motor_1"},
            degraded_sensors={"temp_1"},
        )

        assert status.is_degraded is True
        assert "motor_1" in status.degraded_actuators
        assert "temp_1" in status.degraded_sensors

    def test_status_not_degraded(self) -> None:
        """Test status when not degraded."""
        status = DegradedModeStatus()

        assert status.is_degraded is False
        assert len(status.degraded_actuators) == 0
        assert len(status.degraded_sensors) == 0


# =============================================================================
# DegradedModeController Basic Tests
# =============================================================================


class TestDegradedModeControllerBasics:
    """Basic tests for DegradedModeController."""

    def test_initial_state_not_degraded(self) -> None:
        """Test controller starts in non-degraded state."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        assert controller.is_degraded is False
        assert len(controller.degraded_actuators) == 0
        assert len(controller.degraded_sensors) == 0

    def test_mark_actuator_degraded(self) -> None:
        """Test marking an actuator as degraded."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Communication failure")

        assert controller.is_degraded is True
        assert "motor_1" in controller.degraded_actuators

    def test_mark_sensor_degraded(self) -> None:
        """Test marking a sensor as degraded."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_sensor_degraded("temp_1", "Sensor malfunction")

        assert controller.is_degraded is True
        assert "temp_1" in controller.degraded_sensors

    def test_mark_multiple_degraded(self) -> None:
        """Test marking multiple components as degraded."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Failure 1")
        controller.mark_actuator_degraded("motor_2", "Failure 2")
        controller.mark_sensor_degraded("temp_1", "Failure 3")

        assert controller.is_degraded is True
        assert len(controller.degraded_actuators) == 2
        assert len(controller.degraded_sensors) == 1


# =============================================================================
# DegradedModeController Restore Tests
# =============================================================================


class TestDegradedModeControllerRestore:
    """Tests for restoring degraded components."""

    def test_restore_actuator(self) -> None:
        """Test restoring a degraded actuator."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Failure")
        assert "motor_1" in controller.degraded_actuators

        result = controller.restore_actuator("motor_1")
        assert result is True
        assert "motor_1" not in controller.degraded_actuators

    def test_restore_sensor(self) -> None:
        """Test restoring a degraded sensor."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_sensor_degraded("temp_1", "Failure")
        assert "temp_1" in controller.degraded_sensors

        result = controller.restore_sensor("temp_1")
        assert result is True
        assert "temp_1" not in controller.degraded_sensors

    def test_restore_nonexistent_actuator(self) -> None:
        """Test restoring non-existent actuator returns False."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        result = controller.restore_actuator("nonexistent")
        assert result is False

    def test_restore_all(self) -> None:
        """Test restoring all components."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Failure")
        controller.mark_sensor_degraded("temp_1", "Failure")

        count = controller.restore_all()

        assert count == 2
        assert controller.is_degraded is False


# =============================================================================
# DegradedModeController Move Tests
# =============================================================================


class TestDegradedModeControllerMove:
    """Tests for move operation with degraded actuators."""

    @pytest.mark.asyncio
    async def test_move_all_healthy(self) -> None:
        """Test move with all healthy actuators."""
        mock_driver = AsyncMock()
        controller = DegradedModeController(mock_driver)

        positions = {"motor_1": 100.0, "motor_2": 200.0}
        status = await controller.move(positions)

        assert status.is_degraded is False
        mock_driver.move.assert_called_once_with(positions)

    @pytest.mark.asyncio
    async def test_move_filters_degraded(self) -> None:
        """Test move filters out degraded actuators."""
        mock_driver = AsyncMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Failure")

        positions = {"motor_1": 100.0, "motor_2": 200.0}
        status = await controller.move(positions)

        # Should only move motor_2
        mock_driver.move.assert_called_once_with({"motor_2": 200.0})
        assert status.is_degraded is True
        assert "motor_1" in status.skipped_targets

    @pytest.mark.asyncio
    async def test_move_all_degraded_no_op(self) -> None:
        """Test move does nothing when all actuators degraded."""
        mock_driver = AsyncMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Failure")
        controller.mark_actuator_degraded("motor_2", "Failure")

        positions = {"motor_1": 100.0, "motor_2": 200.0}
        status = await controller.move(positions)

        # Should not call move at all
        mock_driver.move.assert_not_called()
        assert status.is_degraded is True


# =============================================================================
# DegradedModeController Sensor Tests
# =============================================================================


class TestDegradedModeControllerSensor:
    """Tests for sensor reading with degraded sensors."""

    @pytest.mark.asyncio
    async def test_read_healthy_sensor(self) -> None:
        """Test reading a healthy sensor."""
        mock_driver = AsyncMock()
        mock_driver.read_sensor.return_value = 25.5
        controller = DegradedModeController(mock_driver)

        value, is_cached = await controller.read_sensor("temp_1")

        assert value == 25.5
        assert is_cached is False
        mock_driver.read_sensor.assert_called_once_with("temp_1")

    @pytest.mark.asyncio
    async def test_read_degraded_sensor_returns_cached(self) -> None:
        """Test reading degraded sensor returns cached value."""
        mock_driver = AsyncMock()
        mock_driver.read_sensor.return_value = 25.5
        controller = DegradedModeController(mock_driver)

        # First read to cache value
        await controller.read_sensor("temp_1")

        # Mark as degraded
        controller.mark_sensor_degraded("temp_1", "Failure")

        # Should return cached value
        mock_driver.read_sensor.reset_mock()
        value, is_cached = await controller.read_sensor("temp_1")

        assert value == 25.5
        assert is_cached is True
        mock_driver.read_sensor.assert_not_called()

    @pytest.mark.asyncio
    async def test_read_degraded_sensor_no_cache_raises(self) -> None:
        """Test reading degraded sensor with no cache raises error."""
        mock_driver = AsyncMock()
        controller = DegradedModeController(mock_driver)

        # Mark as degraded without prior read
        controller.mark_sensor_degraded("temp_1", "Failure")

        with pytest.raises(KeyError) as exc_info:
            await controller.read_sensor("temp_1")

        assert "degraded" in str(exc_info.value).lower()


# =============================================================================
# Status and Info Tests
# =============================================================================


class TestDegradedModeControllerStatus:
    """Tests for status and info methods."""

    def test_status_method(self) -> None:
        """Test status method returns correct info."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Failure")
        controller.mark_sensor_degraded("temp_1", "Failure")

        status = controller.status()

        assert status.is_degraded is True
        assert "motor_1" in status.degraded_actuators
        assert "temp_1" in status.degraded_sensors

    def test_get_degraded_info(self) -> None:
        """Test get_degraded_info returns detailed info."""
        mock_driver = MagicMock()
        controller = DegradedModeController(mock_driver)

        controller.mark_actuator_degraded("motor_1", "Communication timeout")

        info = controller.get_degraded_info()

        assert "actuators" in info
        assert "sensors" in info
        assert "motor_1" in info["actuators"]
        assert info["actuators"]["motor_1"]["reason"] == "Communication timeout"


# =============================================================================
# Export Tests
# =============================================================================


class TestExports:
    """Tests for module exports."""

    def test_all_exports_importable(self) -> None:
        """Test all exports are importable from utils."""
        from robo_infra.utils import (
            DegradedComponent,
            DegradedModeController,
            DegradedModeStatus,
        )

        assert DegradedModeController is not None
        assert DegradedComponent is not None
        assert DegradedModeStatus is not None
