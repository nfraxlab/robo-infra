"""Tests for exception hierarchy (Phase 5.15.1).

Tests for error codes, log_exception, and exception structure.
"""

from __future__ import annotations

import logging

import pytest

from robo_infra.core.exceptions import (
    CalibrationError,
    CommunicationError,
    ConfigurationError,
    ConnectionLostError,
    DisabledError,
    HardwareNotFoundError,
    KinematicsError,
    LimitsExceededError,
    NotCalibratedError,
    ResourceExhaustedError,
    RoboInfraError,
    SafetyError,
    StateError,
    log_exception,
)
from robo_infra.core.exceptions import TimeoutError as RoboTimeoutError


# =============================================================================
# Error Code Tests
# =============================================================================


class TestErrorCodes:
    """Tests for exception error codes."""

    def test_base_error_code(self) -> None:
        """Test RoboInfraError has base error code."""
        exc = RoboInfraError("Test error")
        assert hasattr(exc, "code")
        assert exc.code == "ROBO_ERROR"

    def test_safety_error_code(self) -> None:
        """Test SafetyError has safety error code."""
        exc = SafetyError("Safety condition", "halt")
        assert exc.code == "ROBO_SAFETY"

    def test_communication_error_code(self) -> None:
        """Test CommunicationError has communication error code."""
        exc = CommunicationError("i2c", 0x50)
        assert exc.code == "ROBO_COMMUNICATION"

    def test_configuration_error_code(self) -> None:
        """Test ConfigurationError has config error code."""
        exc = ConfigurationError("param", "missing value")
        assert exc.code == "ROBO_CONFIGURATION"

    def test_timeout_error_code(self) -> None:
        """Test TimeoutError has timeout error code."""
        exc = RoboTimeoutError("read", 5.0)
        assert exc.code == "ROBO_TIMEOUT"

    def test_calibration_error_code(self) -> None:
        """Test CalibrationError has calibration error code."""
        exc = CalibrationError("motor", "failed")
        assert exc.code == "ROBO_CALIBRATION"

    def test_limits_exceeded_error_code(self) -> None:
        """Test LimitsExceededError has limits error code."""
        exc = LimitsExceededError(150.0, min_limit=0, max_limit=100)
        assert exc.code == "ROBO_LIMITS_EXCEEDED"

    def test_connection_lost_error_code(self) -> None:
        """Test ConnectionLostError has connection lost error code."""
        exc = ConnectionLostError("motor", "timeout")
        assert exc.code == "ROBO_CONNECTION_LOST"

    def test_resource_exhausted_error_code(self) -> None:
        """Test ResourceExhaustedError has resource error code."""
        exc = ResourceExhaustedError("memory", limit=1024)
        assert exc.code == "ROBO_RESOURCE_EXHAUSTED"

    def test_state_error_code(self) -> None:
        """Test StateError has state error code."""
        exc = StateError("Invalid state", current_state="running", attempted_state="stopped")
        assert exc.code == "ROBO_STATE"

    def test_hardware_not_found_error_code(self) -> None:
        """Test HardwareNotFoundError has hardware error code."""
        exc = HardwareNotFoundError("motor1")
        assert exc.code == "ROBO_HARDWARE_NOT_FOUND"

    def test_kinematics_error_code(self) -> None:
        """Test KinematicsError has kinematics error code."""
        exc = KinematicsError("Unreachable position")
        assert exc.code == "ROBO_KINEMATICS"

    def test_disabled_error_code(self) -> None:
        """Test DisabledError has disabled error code."""
        exc = DisabledError("motor")
        assert exc.code == "ROBO_DISABLED"

    def test_not_calibrated_error_code(self) -> None:
        """Test NotCalibratedError has calibration error code."""
        exc = NotCalibratedError("encoder")
        assert exc.code == "ROBO_NOT_CALIBRATED"


# =============================================================================
# Exception Hierarchy Tests
# =============================================================================


class TestExceptionHierarchy:
    """Tests for exception inheritance hierarchy."""

    def test_all_inherit_from_robo_infra_error(self) -> None:
        """Test all exceptions inherit from RoboInfraError."""
        exceptions = [
            SafetyError("condition", "action"),
            CommunicationError("bus", 0x50),
            ConfigurationError("param", "reason"),
            RoboTimeoutError("op", 5.0),
            CalibrationError("component", "reason"),
            LimitsExceededError(150.0, min_limit=0, max_limit=100),
            ConnectionLostError("device", "reason"),
            ResourceExhaustedError("resource", limit=100),
            StateError("Invalid state", current_state="current", attempted_state="expected"),
            HardwareNotFoundError("device"),
            KinematicsError("message"),
            DisabledError("component"),
            NotCalibratedError("component"),
        ]

        for exc in exceptions:
            assert isinstance(exc, RoboInfraError), f"{type(exc).__name__} should inherit from RoboInfraError"

    def test_safety_subclasses(self) -> None:
        """Test SafetyError subclasses."""
        assert isinstance(LimitsExceededError(150.0, max_limit=100), SafetyError)

    def test_communication_subclasses(self) -> None:
        """Test CommunicationError subclasses."""
        assert isinstance(ConnectionLostError("device", "reason"), CommunicationError)

    def test_calibration_subclasses(self) -> None:
        """Test CalibrationError subclasses."""
        assert isinstance(NotCalibratedError("component"), CalibrationError)

    def test_state_subclasses(self) -> None:
        """Test StateError subclasses."""
        assert isinstance(DisabledError("component"), StateError)


# =============================================================================
# Exception Structure Tests
# =============================================================================


class TestExceptionStructure:
    """Tests for exception structure and attributes."""

    def test_message_attribute(self) -> None:
        """Test message is stored."""
        exc = RoboInfraError("Test message")
        assert exc.message == "Test message"
        assert str(exc) == "Test message"

    def test_details_attribute(self) -> None:
        """Test details dict is stored."""
        details = {"key": "value", "count": 42}
        exc = RoboInfraError("Test", details=details)
        assert exc.details == details
        assert exc.details["key"] == "value"
        assert exc.details["count"] == 42

    def test_default_details_is_empty_dict(self) -> None:
        """Test details defaults to empty dict."""
        exc = RoboInfraError("Test")
        assert exc.details == {}

    def test_timeout_error_attributes(self) -> None:
        """Test TimeoutError has operation and timeout attributes."""
        exc = RoboTimeoutError("read_sensor", 5.0)
        assert exc.operation == "read_sensor"
        assert exc.timeout == 5.0

    def test_limits_exceeded_attributes(self) -> None:
        """Test LimitsExceededError has limit attributes."""
        exc = LimitsExceededError(150.0, min_limit=0, max_limit=100.0)
        assert exc.value == 150.0
        assert exc.max_limit == 100.0


# =============================================================================
# log_exception Tests
# =============================================================================


class TestLogException:
    """Tests for log_exception helper."""

    def test_logs_exception_with_code(self, caplog: pytest.LogCaptureFixture) -> None:
        """Test log_exception includes error code context."""
        logger = logging.getLogger("test.exceptions")
        exc = SafetyError("Safety violation", "halt")

        with caplog.at_level(logging.WARNING):
            log_exception(logger, "Move failed", exc)

        assert "SafetyError" in caplog.text
        assert "Safety" in caplog.text

    def test_logs_at_custom_level(self, caplog: pytest.LogCaptureFixture) -> None:
        """Test log_exception can use custom log level."""
        logger = logging.getLogger("test.custom")
        exc = ConfigurationError("param", "bad value")

        with caplog.at_level(logging.ERROR):
            log_exception(logger, "Config error", exc, level="error")

        assert "Config" in caplog.text


# =============================================================================
# Edge Cases
# =============================================================================


class TestEdgeCases:
    """Tests for edge cases and special scenarios."""

    def test_exception_str_representation(self) -> None:
        """Test exception string representation."""
        exc = RoboInfraError("Error message")
        assert str(exc) == "Error message"

    def test_exception_repr(self) -> None:
        """Test exception repr includes class name."""
        exc = SafetyError("Safety issue", "halt")
        repr_str = repr(exc)
        assert "SafetyError" in repr_str

    def test_exception_args(self) -> None:
        """Test exception args for compatibility."""
        exc = RoboInfraError("Test message")
        assert exc.args[0] == "Test message"

    def test_exception_can_be_raised_and_caught(self) -> None:
        """Test exceptions can be raised and caught normally."""
        with pytest.raises(SafetyError):
            raise SafetyError("Safety violation", "halt")

    def test_exception_chaining(self) -> None:
        """Test exception chaining works."""
        try:
            try:
                raise ValueError("Original error")
            except ValueError as e:
                raise CommunicationError("i2c", 0x50, "bus error") from e
        except CommunicationError as exc:
            assert exc.__cause__ is not None
            assert isinstance(exc.__cause__, ValueError)


# =============================================================================
# Export Tests
# =============================================================================


class TestExports:
    """Tests for module exports."""

    def test_all_exceptions_importable(self) -> None:
        """Test all exceptions are importable from core.exceptions."""
        from robo_infra.core.exceptions import (
            CalibrationError,
            CommunicationError,
            ConfigurationError,
            ConnectionLostError,
            DisabledError,
            HardwareNotFoundError,
            KinematicsError,
            LimitsExceededError,
            NotCalibratedError,
            ResourceExhaustedError,
            RoboInfraError,
            SafetyError,
            StateError,
            log_exception,
        )

        # All should be exception classes
        for exc_class in [
            RoboInfraError,
            SafetyError,
            CommunicationError,
            ConfigurationError,
            CalibrationError,
            LimitsExceededError,
            ConnectionLostError,
            ResourceExhaustedError,
            StateError,
            HardwareNotFoundError,
            KinematicsError,
            DisabledError,
            NotCalibratedError,
        ]:
            assert issubclass(exc_class, Exception)

        assert callable(log_exception)
