"""Unit tests for observability integration module.

Tests for Prometheus metrics, health checks, and structured logging
for robo-infra controllers and actuators.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.controller import SimulatedController
from robo_infra.core.types import Limits


# --- Fixtures ---


@pytest.fixture
def mock_controller() -> SimulatedController:
    """Create a mock controller for testing."""
    controller = SimulatedController(name="test_arm")
    controller.add_actuator(
        "shoulder",
        SimulatedActuator(
            name="shoulder",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
    )
    return controller


@pytest.fixture
def mock_actuator() -> SimulatedActuator:
    """Create a mock actuator for testing."""
    return SimulatedActuator(
        name="test_servo",
        limits=Limits(min=0, max=180, default=90),
        unit="degrees",
    )


# --- Metric Recording Tests ---


class TestRecordCommand:
    """Tests for record_command function."""

    def test_record_command_without_prometheus(self) -> None:
        """Test that record_command doesn't fail without prometheus."""
        from robo_infra.integrations.observability import record_command

        # Should not raise even without prometheus-client
        record_command(
            controller="arm",
            command="move",
            status="success",
            duration_seconds=0.5,
        )

    def test_record_command_with_mocked_metrics(self) -> None:
        """Test record_command with mocked prometheus metrics."""
        from robo_infra.integrations import observability

        mock_counter = MagicMock()
        mock_histogram = MagicMock()

        # Mock the registry's metrics
        observability._metrics._initialized = True
        original_commands = observability._metrics.commands_total
        original_duration = observability._metrics.command_duration_seconds
        observability._metrics.commands_total = mock_counter
        observability._metrics.command_duration_seconds = mock_histogram

        try:
            observability.record_command(
                controller="arm",
                command="move",
                status="success",
                duration_seconds=0.25,
            )

            mock_counter.labels.assert_called_once_with(
                controller="arm",
                command="move",
                status="success",
            )
            mock_counter.labels().inc.assert_called_once()

            mock_histogram.labels.assert_called_once_with(
                controller="arm",
                command="move",
            )
            mock_histogram.labels().observe.assert_called_once_with(0.25)
        finally:
            # Restore
            observability._metrics.commands_total = original_commands
            observability._metrics.command_duration_seconds = original_duration


class TestRecordPosition:
    """Tests for record_position function."""

    def test_record_position_without_prometheus(self) -> None:
        """Test that record_position doesn't fail without prometheus."""
        from robo_infra.integrations.observability import record_position

        # Should not raise even without prometheus-client
        record_position(
            controller="arm",
            actuator="shoulder",
            position=45.0,
        )

    def test_record_position_with_mocked_metrics(self) -> None:
        """Test record_position with mocked prometheus metrics."""
        from robo_infra.integrations import observability

        mock_gauge = MagicMock()

        observability._metrics._initialized = True
        original = observability._metrics.actuator_position
        observability._metrics.actuator_position = mock_gauge

        try:
            observability.record_position(
                controller="arm",
                actuator="shoulder",
                position=45.0,
            )

            mock_gauge.labels.assert_called_once_with(
                controller="arm",
                actuator="shoulder",
            )
            mock_gauge.labels().set.assert_called_once_with(45.0)
        finally:
            observability._metrics.actuator_position = original


class TestRecordSafetyTrigger:
    """Tests for record_safety_trigger function."""

    def test_record_safety_trigger_without_prometheus(self) -> None:
        """Test that record_safety_trigger doesn't fail without prometheus."""
        from robo_infra.integrations.observability import record_safety_trigger

        # Should not raise even without prometheus-client
        record_safety_trigger(
            controller="arm",
            trigger_type="estop",
        )

    def test_record_safety_trigger_with_mocked_metrics(self) -> None:
        """Test record_safety_trigger with mocked prometheus metrics."""
        from robo_infra.integrations import observability

        mock_counter = MagicMock()

        observability._metrics._initialized = True
        original = observability._metrics.safety_triggers_total
        observability._metrics.safety_triggers_total = mock_counter

        try:
            observability.record_safety_trigger(
                controller="arm",
                trigger_type="limit_exceeded",
            )

            mock_counter.labels.assert_called_once_with(
                controller="arm",
                trigger_type="limit_exceeded",
            )
            mock_counter.labels().inc.assert_called_once()
        finally:
            observability._metrics.safety_triggers_total = original


class TestRecordSensorValue:
    """Tests for record_sensor_value function."""

    def test_record_sensor_value_without_prometheus(self) -> None:
        """Test that record_sensor_value doesn't fail without prometheus."""
        from robo_infra.integrations.observability import record_sensor_value

        # Should not raise even without prometheus-client
        record_sensor_value(
            controller="arm",
            sensor="temperature",
            value=45.5,
            unit="celsius",
        )

    def test_record_sensor_value_with_mocked_metrics(self) -> None:
        """Test record_sensor_value with mocked prometheus metrics."""
        from robo_infra.integrations import observability

        mock_gauge = MagicMock()

        observability._metrics._initialized = True
        original = observability._metrics.sensor_value
        observability._metrics.sensor_value = mock_gauge

        try:
            observability.record_sensor_value(
                controller="arm",
                sensor="temperature",
                value=45.5,
                unit="celsius",
            )

            mock_gauge.labels.assert_called_once_with(
                controller="arm",
                sensor="temperature",
                unit="celsius",
            )
            mock_gauge.labels().set.assert_called_once_with(45.5)
        finally:
            observability._metrics.sensor_value = original


class TestSafetyTriggerHelpers:
    """Tests for safety trigger helper functions."""

    def test_record_estop_triggered(self) -> None:
        """Test E-stop trigger recording."""
        from robo_infra.integrations.observability import record_estop_triggered

        with patch(
            "robo_infra.integrations.observability.record_safety_trigger"
        ) as mock_record:
            record_estop_triggered("arm")

            mock_record.assert_called_once_with("arm", "estop")

    def test_record_limit_exceeded(self) -> None:
        """Test limit exceeded recording."""
        from robo_infra.integrations.observability import record_limit_exceeded

        with patch(
            "robo_infra.integrations.observability.record_safety_trigger"
        ) as mock_record:
            record_limit_exceeded("arm", actuator="joint1", limit_type="max")

            mock_record.assert_called_once_with("arm", "limit_exceeded")

    def test_record_watchdog_timeout(self) -> None:
        """Test watchdog timeout recording."""
        from robo_infra.integrations.observability import record_watchdog_timeout

        with patch(
            "robo_infra.integrations.observability.record_safety_trigger"
        ) as mock_record:
            record_watchdog_timeout("arm")

            mock_record.assert_called_once_with("arm", "watchdog_timeout")

    def test_record_monitor_alert(self) -> None:
        """Test monitor alert recording."""
        from robo_infra.integrations.observability import record_monitor_alert

        with patch(
            "robo_infra.integrations.observability.record_safety_trigger"
        ) as mock_record:
            record_monitor_alert("arm", condition="velocity exceeded")

            mock_record.assert_called_once_with("arm", "monitor_alert")


class TestSafetyTriggerType:
    """Tests for SafetyTriggerType constants."""

    def test_safety_trigger_type_constants(self) -> None:
        """Test SafetyTriggerType has expected constants."""
        from robo_infra.integrations.observability import SafetyTriggerType

        assert SafetyTriggerType.ESTOP == "estop"
        assert SafetyTriggerType.LIMIT_EXCEEDED == "limit_exceeded"
        assert SafetyTriggerType.WATCHDOG_TIMEOUT == "watchdog_timeout"
        assert SafetyTriggerType.MONITOR_ALERT == "monitor_alert"
        assert SafetyTriggerType.COLLISION_DETECTED == "collision_detected"
        assert SafetyTriggerType.COMMUNICATION_LOSS == "communication_loss"
        assert SafetyTriggerType.OVERCURRENT == "overcurrent"
        assert SafetyTriggerType.OVERTEMPERATURE == "overtemperature"

    def test_safety_trigger_type_with_record(self) -> None:
        """Test using SafetyTriggerType constants with record_safety_trigger."""
        from robo_infra.integrations.observability import (
            SafetyTriggerType,
            record_safety_trigger,
        )

        # Should not raise
        record_safety_trigger("arm", SafetyTriggerType.ESTOP)
        record_safety_trigger("arm", SafetyTriggerType.WATCHDOG_TIMEOUT)


# --- Track Command Decorator Tests ---


class TestTrackCommandDecorator:
    """Tests for track_command decorator."""

    @pytest.mark.asyncio
    async def test_track_command_success(self) -> None:
        """Test decorator records successful command."""
        from robo_infra.integrations.observability import track_command

        class MockController:
            name = "test_arm"

            @track_command("move")
            async def move(self, target: float) -> dict:
                return {"position": target}

        with patch(
            "robo_infra.integrations.observability.record_command"
        ) as mock_record:
            controller = MockController()
            result = await controller.move(45.0)

            assert result == {"position": 45.0}
            mock_record.assert_called_once()

            call_args = mock_record.call_args
            assert call_args.kwargs["controller"] == "test_arm"
            assert call_args.kwargs["command"] == "move"
            assert call_args.kwargs["status"] == "success"
            assert call_args.kwargs["duration_seconds"] >= 0

    @pytest.mark.asyncio
    async def test_track_command_error(self) -> None:
        """Test decorator records failed command."""
        from robo_infra.integrations.observability import track_command

        class MockController:
            name = "test_arm"

            @track_command("move")
            async def move(self, target: float) -> dict:
                raise ValueError("Target out of range")

        with patch(
            "robo_infra.integrations.observability.record_command"
        ) as mock_record:
            controller = MockController()

            with pytest.raises(ValueError, match="Target out of range"):
                await controller.move(999.0)

            mock_record.assert_called_once()
            call_args = mock_record.call_args
            assert call_args.kwargs["status"] == "error"

    @pytest.mark.asyncio
    async def test_track_command_custom_attr(self) -> None:
        """Test decorator with custom controller attribute name."""
        from robo_infra.integrations.observability import track_command

        class MockController:
            controller_id = "custom_arm"

            @track_command("stop", controller_attr="controller_id")
            async def stop(self) -> dict:
                return {"stopped": True}

        with patch(
            "robo_infra.integrations.observability.record_command"
        ) as mock_record:
            controller = MockController()
            await controller.stop()

            call_args = mock_record.call_args
            assert call_args.kwargs["controller"] == "custom_arm"

    @pytest.mark.asyncio
    async def test_track_command_unknown_controller(self) -> None:
        """Test decorator handles missing controller name."""
        from robo_infra.integrations.observability import track_command

        class MockController:
            # No name attribute

            @track_command("home")
            async def home(self) -> dict:
                return {"homed": True}

        with patch(
            "robo_infra.integrations.observability.record_command"
        ) as mock_record:
            controller = MockController()
            await controller.home()

            call_args = mock_record.call_args
            assert call_args.kwargs["controller"] == "unknown"


# --- Health Check Tests ---


class TestCreateControllerHealthCheck:
    """Tests for create_controller_health_check function."""

    @pytest.mark.asyncio
    async def test_healthy_controller(
        self, mock_controller: SimulatedController
    ) -> None:
        """Test health check for a healthy controller."""
        from robo_infra.integrations.observability import create_controller_health_check

        health_check = create_controller_health_check(mock_controller)
        result = await health_check()

        assert result.status.value == "healthy"
        assert result.name == "test_arm"
        assert result.latency_ms >= 0

    @pytest.mark.asyncio
    async def test_disabled_controller(
        self, mock_controller: SimulatedController
    ) -> None:
        """Test health check for a disabled controller."""
        from robo_infra.integrations.observability import create_controller_health_check

        # Mock status to return disabled (sync, not async)
        def mock_status():
            return {"enabled": False, "error": None}

        mock_controller.status = mock_status

        health_check = create_controller_health_check(mock_controller)
        result = await health_check()

        assert result.status.value == "degraded"
        assert "disabled" in result.message.lower()

    @pytest.mark.asyncio
    async def test_controller_in_error_state(
        self, mock_controller: SimulatedController
    ) -> None:
        """Test health check for a controller in error state."""
        from robo_infra.integrations.observability import create_controller_health_check

        def mock_status():
            return {"enabled": True, "error": "Hardware fault"}

        mock_controller.status = mock_status

        health_check = create_controller_health_check(mock_controller)
        result = await health_check()

        assert result.status.value == "unhealthy"
        assert "error state" in result.message.lower()

    @pytest.mark.asyncio
    async def test_controller_estop_active(
        self, mock_controller: SimulatedController
    ) -> None:
        """Test health check with emergency stop active."""
        from robo_infra.integrations.observability import create_controller_health_check

        def mock_status():
            return {"enabled": True, "error": None, "safety": {"estop_active": True}}

        mock_controller.status = mock_status

        health_check = create_controller_health_check(mock_controller)
        result = await health_check()

        assert result.status.value == "unhealthy"
        assert "emergency stop" in result.message.lower()

    @pytest.mark.asyncio
    async def test_controller_health_check_exception(
        self, mock_controller: SimulatedController
    ) -> None:
        """Test health check handles exceptions gracefully."""
        from robo_infra.integrations.observability import create_controller_health_check

        def mock_status():
            raise RuntimeError("Communication error")

        mock_controller.status = mock_status

        health_check = create_controller_health_check(mock_controller)
        result = await health_check()

        assert result.status.value == "unhealthy"
        assert "failed" in result.message.lower()


class TestCreateActuatorHealthCheck:
    """Tests for create_actuator_health_check function."""

    @pytest.mark.asyncio
    async def test_healthy_actuator(self, mock_actuator: SimulatedActuator) -> None:
        """Test health check for a healthy actuator."""
        from robo_infra.integrations.observability import create_actuator_health_check

        health_check = create_actuator_health_check(mock_actuator)
        result = await health_check()

        assert result.status.value == "healthy"
        assert result.name == "test_servo"

    @pytest.mark.asyncio
    async def test_actuator_in_fault_state(
        self, mock_actuator: SimulatedActuator
    ) -> None:
        """Test health check for an actuator in fault state."""
        from robo_infra.integrations.observability import create_actuator_health_check

        # Simulate fault state
        mock_actuator.is_fault = True

        health_check = create_actuator_health_check(mock_actuator)
        result = await health_check()

        assert result.status.value == "unhealthy"
        assert "fault" in result.message.lower()

    @pytest.mark.asyncio
    async def test_actuator_health_check_exception(
        self, mock_actuator: SimulatedActuator
    ) -> None:
        """Test health check handles exceptions gracefully."""
        from robo_infra.integrations.observability import create_actuator_health_check

        # Make current_position raise an exception
        def raise_error():
            raise RuntimeError("Communication error")

        mock_actuator.current_position = raise_error

        health_check = create_actuator_health_check(mock_actuator)
        result = await health_check()

        assert result.status.value == "unhealthy"
        assert "failed" in result.message.lower()


# --- Structured Logging Tests ---


class TestSetupRoboticsLogging:
    """Tests for setup_robotics_logging function."""

    def test_setup_logging_with_svc_infra(self) -> None:
        """Test logging setup uses svc-infra when available."""
        from robo_infra.integrations.observability import setup_robotics_logging

        with patch("svc_infra.app.setup_logging") as mock_setup:
            setup_robotics_logging(level="DEBUG", json_output=True)
            mock_setup.assert_called_once_with(level="DEBUG", fmt="json")

    def test_setup_logging_fallback(self) -> None:
        """Test logging setup falls back to standard logging."""
        from robo_infra.integrations.observability import setup_robotics_logging

        with (
            patch("svc_infra.app.setup_logging", side_effect=ImportError),
            patch("logging.basicConfig") as mock_basic,
        ):
            setup_robotics_logging(level="WARNING")
            mock_basic.assert_called_once()


# --- Integration Tests ---


class TestMetricsInitialization:
    """Tests for metrics initialization."""

    def test_init_metrics_without_prometheus(self) -> None:
        """Test initialization gracefully handles missing prometheus."""
        from robo_infra.integrations import observability

        # Reset state
        observability._metrics._initialized = False

        with patch(
            "svc_infra.obs.metrics.base.counter", side_effect=ImportError
        ):
            result = observability._init_metrics()
            assert result is False

    def test_init_metrics_with_prometheus(self) -> None:
        """Test initialization succeeds with prometheus available."""
        from robo_infra.integrations import observability

        # Reset state
        observability._metrics._initialized = False

        mock_counter = MagicMock()
        mock_gauge = MagicMock()
        mock_histogram = MagicMock()

        with (
            patch("svc_infra.obs.metrics.base.counter", return_value=mock_counter),
            patch("svc_infra.obs.metrics.base.gauge", return_value=mock_gauge),
            patch(
                "svc_infra.obs.metrics.base.histogram", return_value=mock_histogram
            ),
        ):
            result = observability._init_metrics()
            assert result is True
            assert observability._metrics._initialized is True

    def test_init_metrics_only_once(self) -> None:
        """Test that metrics are only initialized once."""
        from robo_infra.integrations import observability

        observability._metrics._initialized = True

        # Should return True without calling anything
        result = observability._init_metrics()
        assert result is True


# --- Export Tests ---


class TestModuleExports:
    """Tests for module exports."""

    def test_all_exports_importable(self) -> None:
        """Test that all __all__ exports are importable."""
        from robo_infra.integrations import observability

        for name in observability.__all__:
            assert hasattr(observability, name), f"Missing export: {name}"

    def test_integration_package_exports(self) -> None:
        """Test that observability is exported from integrations package."""
        from robo_infra.integrations import (
            SafetyTriggerType,
            create_actuator_health_check,
            create_controller_health_check,
            record_command,
            record_estop_triggered,
            record_limit_exceeded,
            record_monitor_alert,
            record_position,
            record_safety_trigger,
            record_sensor_value,
            record_watchdog_timeout,
            setup_robotics_logging,
            track_command,
        )

        # Verify they're callable/usable
        assert callable(record_command)
        assert callable(record_position)
        assert callable(record_safety_trigger)
        assert callable(record_sensor_value)
        assert callable(record_estop_triggered)
        assert callable(record_limit_exceeded)
        assert callable(record_watchdog_timeout)
        assert callable(record_monitor_alert)
        assert callable(track_command)
        assert callable(create_controller_health_check)
        assert callable(create_actuator_health_check)
        assert callable(setup_robotics_logging)
        # Verify class
        assert SafetyTriggerType.ESTOP == "estop"
