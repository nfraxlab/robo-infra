"""Unit tests for Step/Dir stepper drivers (A4988, DRV8825).

Tests the StepDirDriver base class and A4988/DRV8825 implementations
including GPIO control, microstepping, and position tracking.
"""

from __future__ import annotations

import os
from unittest.mock import MagicMock, patch

import pytest

# Set simulation mode for tests
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.drivers.step_dir import (
    A4988Driver,
    DRV8825Driver,
    StepDirConfig,
    StepDirDriver,
)


class TestStepDirConfig:
    """Tests for StepDirConfig dataclass."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = StepDirConfig(step_pin=17, dir_pin=27)
        assert config.step_pin == 17
        assert config.dir_pin == 27
        assert config.enable_pin is None
        assert config.ms_pins is None  # Default is None
        assert config.step_pulse_us == 2
        assert config.step_delay_us == 100  # Actual default from code
        assert config.invert_dir is False
        assert config.invert_enable is True  # Actual default from code

    def test_custom_config(self) -> None:
        """Test custom configuration values."""
        config = StepDirConfig(
            step_pin=17,
            dir_pin=27,
            enable_pin=22,
            ms_pins=(5, 6, 13),
            step_pulse_us=5,
            step_delay_us=10,
            invert_dir=True,
            invert_enable=False,
            name="MyMotor",
        )
        assert config.step_pin == 17
        assert config.dir_pin == 27
        assert config.enable_pin == 22
        assert config.ms_pins == (5, 6, 13)
        assert config.step_pulse_us == 5
        assert config.step_delay_us == 10
        assert config.invert_dir is True
        assert config.invert_enable is False
        assert config.name == "MyMotor"


class TestA4988DriverLifecycle:
    """Tests for A4988Driver lifecycle methods."""

    def test_init_minimal(self) -> None:
        """Test driver initialization with minimal config."""
        driver = A4988Driver(step_pin=2, dir_pin=3)
        assert driver.simulation is True
        # Access internal pin number attributes
        assert driver._step_pin_num == 2
        assert driver._dir_pin_num == 3

    def test_init_with_enable(self) -> None:
        """Test driver initialization with enable pin."""
        driver = A4988Driver(step_pin=2, dir_pin=3, enable_pin=4)
        assert driver._enable_pin_num == 4

    def test_init_with_ms_pins(self) -> None:
        """Test driver initialization with microstepping pins."""
        driver = A4988Driver(step_pin=2, dir_pin=3, ms_pins=(5, 6, 7))
        assert driver._ms_pin_nums == (5, 6, 7)

    def test_connect(self) -> None:
        """Test connection in simulation mode."""
        driver = A4988Driver(step_pin=2, dir_pin=3)
        driver.connect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.CONNECTED

    def test_disconnect(self) -> None:
        """Test disconnection."""
        driver = A4988Driver(step_pin=2, dir_pin=3)
        driver.connect()
        driver.disconnect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.DISCONNECTED


class TestA4988DriverEnableDisable:
    """Tests for A4988Driver enable/disable."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3, enable_pin=4)
        drv.connect()
        return drv

    def test_enable(self, driver: A4988Driver) -> None:
        """Test enabling the driver."""
        driver.enable()
        assert driver._enabled is True

    def test_disable(self, driver: A4988Driver) -> None:
        """Test disabling the driver."""
        driver.enable()
        driver.disable()
        assert driver._enabled is False


class TestA4988DriverDirection:
    """Tests for A4988Driver direction control."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_set_direction_forward(self, driver: A4988Driver) -> None:
        """Test setting forward direction."""
        driver.set_direction(forward=True)
        assert driver._direction is True

    def test_set_direction_reverse(self, driver: A4988Driver) -> None:
        """Test setting reverse direction."""
        driver.set_direction(forward=False)
        assert driver._direction is False


class TestA4988DriverStepping:
    """Tests for A4988Driver step generation."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_step_positive(self, driver: A4988Driver) -> None:
        """Test stepping forward."""
        initial_pos = driver._position
        driver.step(10)
        assert driver._position == initial_pos + 10

    def test_step_negative(self, driver: A4988Driver) -> None:
        """Test stepping backward."""
        driver._position = 100
        driver.step(-10)
        assert driver._position == 90


class TestA4988DriverPosition:
    """Tests for A4988Driver position control."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_move_to_positive(self, driver: A4988Driver) -> None:
        """Test moving to positive position."""
        driver.move_to(100)
        assert driver._position == 100

    def test_move_to_negative(self, driver: A4988Driver) -> None:
        """Test moving to negative position."""
        driver.move_to(-50)
        assert driver._position == -50

    def test_move_to_same_position(self, driver: A4988Driver) -> None:
        """Test moving to current position (no movement)."""
        driver.move_to(0)
        driver.move_to(0)  # Should do nothing
        assert driver._position == 0


class TestA4988DriverMicrostepping:
    """Tests for A4988Driver microstepping control."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver with MS pins."""
        drv = A4988Driver(step_pin=2, dir_pin=3, ms_pins=(5, 6, 7))
        drv.connect()
        return drv

    def test_microstepping_values(self) -> None:
        """Test A4988 microstepping table."""
        # Actual implementation uses boolean tuples
        assert A4988Driver.MICROSTEP_TABLE == {
            1: (False, False, False),
            2: (True, False, False),
            4: (False, True, False),
            8: (True, True, False),
            16: (True, True, True),
        }

    def test_set_microstepping_valid(self, driver: A4988Driver) -> None:
        """Test setting valid microstepping values."""
        for microsteps in [1, 2, 4, 8, 16]:
            driver.set_microstepping(microsteps)
            assert driver._microstepping == microsteps

    def test_set_microstepping_invalid(self, driver: A4988Driver) -> None:
        """Test setting invalid microstepping value."""
        with pytest.raises(ValueError):
            driver.set_microstepping(32)  # Not supported by A4988


class TestA4988DriverStatus:
    """Tests for A4988Driver status methods."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_get_status(self, driver: A4988Driver) -> None:
        """Test getting driver status."""
        driver.move_to(50)
        status = driver.get_status()

        # Actual status keys from implementation
        assert "enabled" in status
        assert "direction" in status
        assert "position" in status
        assert "microstepping" in status
        assert "simulation" in status

        assert status["position"] == 50
        assert status["simulation"] is True


class TestDRV8825DriverLifecycle:
    """Tests for DRV8825Driver lifecycle methods."""

    def test_init_minimal(self) -> None:
        """Test driver initialization with minimal config."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3)
        assert driver.simulation is True

    def test_init_with_fault_pin(self) -> None:
        """Test driver initialization with fault pin."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3, fault_pin=8)
        # Actual attribute name
        assert driver._fault_pin_num == 8

    def test_connect(self) -> None:
        """Test connection in simulation mode."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3)
        driver.connect()
        from robo_infra.core.driver import DriverState
        assert driver._state == DriverState.CONNECTED


class TestDRV8825DriverMicrostepping:
    """Tests for DRV8825Driver microstepping control."""

    @pytest.fixture
    def driver(self) -> DRV8825Driver:
        """Create connected driver with MS pins."""
        drv = DRV8825Driver(step_pin=2, dir_pin=3, ms_pins=(5, 6, 7))
        drv.connect()
        return drv

    def test_microstepping_values(self) -> None:
        """Test DRV8825 microstepping table (includes 32 microsteps)."""
        # Actual implementation uses boolean tuples
        assert DRV8825Driver.MICROSTEP_TABLE == {
            1: (False, False, False),
            2: (True, False, False),
            4: (False, True, False),
            8: (True, True, False),
            16: (False, False, True),
            32: (True, False, True),
        }

    def test_set_microstepping_32(self, driver: DRV8825Driver) -> None:
        """Test setting 32 microstepping (DRV8825 only)."""
        driver.set_microstepping(32)
        assert driver._microstepping == 32


class TestDRV8825DriverFault:
    """Tests for DRV8825Driver fault detection."""

    @pytest.fixture
    def driver(self) -> DRV8825Driver:
        """Create connected driver with fault pin."""
        drv = DRV8825Driver(step_pin=2, dir_pin=3, fault_pin=8)
        drv.connect()
        return drv

    def test_is_fault_no_fault(self, driver: DRV8825Driver) -> None:
        """Test fault detection with no fault."""
        assert driver.is_fault() is False

    def test_is_fault_no_pin(self) -> None:
        """Test fault detection without fault pin configured."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3)
        driver.connect()
        assert driver.is_fault() is False


class TestDRV8825DriverStatus:
    """Tests for DRV8825Driver status methods."""

    @pytest.fixture
    def driver(self) -> DRV8825Driver:
        """Create connected driver with fault pin."""
        drv = DRV8825Driver(step_pin=2, dir_pin=3, fault_pin=8)
        drv.connect()
        return drv

    def test_get_status_includes_fault(self, driver: DRV8825Driver) -> None:
        """Test that status includes fault field."""
        status = driver.get_status()
        assert "fault" in status
        assert status["fault"] is False


class TestStepDirDriverChannelInterface:
    """Tests for StepDirDriver channel interface (Driver ABC)."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_write_channel(self, driver: A4988Driver) -> None:
        """Test writing to channel (moves based on value)."""
        initial = driver._position
        driver._write_channel(0, 1.0)  # Value * 100 = 100 steps
        assert driver._position == initial + 100

    def test_read_channel(self, driver: A4988Driver) -> None:
        """Test reading from channel (returns normalized position)."""
        driver.move_to(500)
        value = driver._read_channel(0)
        # Position / 1000.0 = 0.5
        assert value == 0.5
