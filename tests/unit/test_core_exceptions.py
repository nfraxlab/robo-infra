"""Unit tests for core exceptions."""

from robo_infra.core.exceptions import (
    CalibrationError,
    CommunicationError,
    DisabledError,
    HardwareNotFoundError,
    LimitsExceededError,
    RoboInfraError,
    SafetyError,
    TimeoutError,
)


class TestRoboInfraError:
    """Test base RoboInfraError."""

    def test_creation(self) -> None:
        """Test error creation."""
        error = RoboInfraError("test error")
        assert str(error) == "test error"

    def test_is_exception(self) -> None:
        """Test it's a proper Exception."""
        error = RoboInfraError("test")
        assert isinstance(error, Exception)


class TestHardwareNotFoundError:
    """Test HardwareNotFoundError."""

    def test_creation(self) -> None:
        """Test error creation with device info."""
        error = HardwareNotFoundError("servo-1")
        assert "servo-1" in str(error)
        assert error.device == "servo-1"

    def test_with_details(self) -> None:
        """Test error creation with details."""
        error = HardwareNotFoundError("servo-1", "Not responding on I2C")
        assert "servo-1" in str(error)
        assert "Not responding" in str(error)

    def test_inheritance(self) -> None:
        """Test inherits from RoboInfraError."""
        error = HardwareNotFoundError("device")
        assert isinstance(error, RoboInfraError)


class TestLimitsExceededError:
    """Test LimitsExceededError."""

    def test_creation(self) -> None:
        """Test error creation with limit info."""
        error = LimitsExceededError(
            value=200.0,
            min_limit=0.0,
            max_limit=180.0,
            name="angle",
        )
        assert error.value == 200.0
        assert error.min_limit == 0.0
        assert error.max_limit == 180.0
        assert error.name == "angle"

    def test_message_format(self) -> None:
        """Test error message format."""
        error = LimitsExceededError(
            value=200.0,
            min_limit=0.0,
            max_limit=180.0,
            name="angle",
        )
        msg = str(error)
        assert "200.0" in msg
        assert "0.0" in msg
        assert "180.0" in msg
        assert "angle" in msg

    def test_inheritance(self) -> None:
        """Test inherits from RoboInfraError."""
        error = LimitsExceededError(value=0, min_limit=0, max_limit=0)
        assert isinstance(error, RoboInfraError)


class TestCommunicationError:
    """Test CommunicationError."""

    def test_creation(self) -> None:
        """Test error creation."""
        error = CommunicationError("i2c", details="Bus timeout")
        assert error.bus == "i2c"
        assert "Bus timeout" in str(error)

    def test_with_address(self) -> None:
        """Test error creation with address."""
        error = CommunicationError("i2c", address=0x40, details="No ACK")
        assert "0x40" in str(error) or "64" in str(error)

    def test_inheritance(self) -> None:
        """Test inherits from RoboInfraError."""
        error = CommunicationError("spi")
        assert isinstance(error, RoboInfraError)


class TestCalibrationError:
    """Test CalibrationError."""

    def test_creation(self) -> None:
        """Test error creation."""
        error = CalibrationError("servo-1", "Failed to find center")
        assert error.component == "servo-1"
        assert "Failed to find center" in str(error)

    def test_inheritance(self) -> None:
        """Test inherits from RoboInfraError."""
        error = CalibrationError("device")
        assert isinstance(error, RoboInfraError)


class TestSafetyError:
    """Test SafetyError."""

    def test_creation(self) -> None:
        """Test error creation."""
        error = SafetyError("collision_detection", "Emergency stop activated")
        assert error.condition == "collision_detection"
        assert "collision_detection" in str(error)

    def test_inheritance(self) -> None:
        """Test inherits from RoboInfraError."""
        error = SafetyError("system")
        assert isinstance(error, RoboInfraError)


class TestTimeoutError:
    """Test TimeoutError."""

    def test_creation(self) -> None:
        """Test error creation."""
        error = TimeoutError("move_to_position", 5.0)
        assert error.operation == "move_to_position"
        assert error.timeout == 5.0

    def test_message_format(self) -> None:
        """Test error message format."""
        error = TimeoutError("move", 5.0)
        msg = str(error)
        assert "move" in msg
        assert "5.0" in msg

    def test_inheritance(self) -> None:
        """Test inherits from RoboInfraError."""
        error = TimeoutError("op", 1.0)
        assert isinstance(error, RoboInfraError)


class TestDisabledError:
    """Test DisabledError."""

    def test_creation(self) -> None:
        """Test error creation."""
        error = DisabledError("motor-1")
        assert "motor-1" in str(error)

    def test_inheritance(self) -> None:
        """Test inherits from RoboInfraError."""
        error = DisabledError("device")
        assert isinstance(error, RoboInfraError)
