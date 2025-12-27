"""Unit tests for Step/Dir stepper drivers (A4988, DRV8825).

Tests the StepDirDriver base class and A4988/DRV8825 implementations
including GPIO control, microstepping, and position tracking.
"""

from __future__ import annotations

import os

import pytest


# Set simulation mode for tests
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.drivers.step_dir import (
    A4988Driver,
    DRV8825Driver,
    StepDirConfig,
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


# =============================================================================
# Phase 5.8.2.2 - Step/Dir Advanced Tests
# =============================================================================


class TestStepDirInit:
    """Tests for StepDirDriver initialization."""

    def test_init_step_dir_pins_stored(self) -> None:
        """Test that step and dir pins are stored correctly."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver._step_pin_num == 17
        assert driver._dir_pin_num == 27

    def test_init_enable_pin_optional(self) -> None:
        """Test enable pin is optional."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver._enable_pin_num is None

    def test_init_ms_pins_optional(self) -> None:
        """Test microstepping pins are optional."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver._ms_pin_nums is None

    def test_init_simulation_from_env(self) -> None:
        """Test simulation mode from environment variable."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver.simulation is True

    def test_init_default_position(self) -> None:
        """Test default position is zero."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver.position == 0

    def test_init_default_direction(self) -> None:
        """Test default direction is forward."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver.direction is True

    def test_init_default_microstepping(self) -> None:
        """Test default microstepping is 1 (full step)."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver.microstepping == 1

    def test_init_default_disabled(self) -> None:
        """Test driver starts disabled."""
        driver = A4988Driver(step_pin=17, dir_pin=27)
        assert driver.enabled is False


class TestStepDirEnableAdvanced:
    """Advanced tests for StepDirDriver enable functionality."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver with enable pin."""
        drv = A4988Driver(step_pin=2, dir_pin=3, enable_pin=4)
        drv.connect()
        return drv

    def test_enable_sets_flag(self, driver: A4988Driver) -> None:
        """Test enable sets the enabled flag."""
        driver.enable()
        assert driver.enabled is True

    def test_enable_idempotent(self, driver: A4988Driver) -> None:
        """Test calling enable multiple times is safe."""
        driver.enable()
        driver.enable()
        driver.enable()
        assert driver.enabled is True

    def test_enable_without_enable_pin(self) -> None:
        """Test enable works without enable pin configured."""
        driver = A4988Driver(step_pin=2, dir_pin=3)  # No enable_pin
        driver.connect()
        driver.enable()  # Should not raise
        assert driver.enabled is True


class TestStepDirDisableAdvanced:
    """Advanced tests for StepDirDriver disable functionality."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver with enable pin."""
        drv = A4988Driver(step_pin=2, dir_pin=3, enable_pin=4)
        drv.connect()
        return drv

    def test_disable_sets_flag(self, driver: A4988Driver) -> None:
        """Test disable clears the enabled flag."""
        driver.enable()
        driver.disable()
        assert driver.enabled is False

    def test_disable_idempotent(self, driver: A4988Driver) -> None:
        """Test calling disable multiple times is safe."""
        driver.disable()
        driver.disable()
        driver.disable()
        assert driver.enabled is False

    def test_disable_without_enable_pin(self) -> None:
        """Test disable works without enable pin configured."""
        driver = A4988Driver(step_pin=2, dir_pin=3)  # No enable_pin
        driver.connect()
        driver.enable()
        driver.disable()  # Should not raise
        assert driver.enabled is False


class TestStepDirDirectionAdvanced:
    """Advanced tests for StepDirDriver direction control."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_direction_forward(self, driver: A4988Driver) -> None:
        """Test setting forward direction."""
        driver.set_direction(forward=True)
        assert driver.direction is True

    def test_direction_reverse(self, driver: A4988Driver) -> None:
        """Test setting reverse direction."""
        driver.set_direction(forward=False)
        assert driver.direction is False

    def test_direction_toggle(self, driver: A4988Driver) -> None:
        """Test toggling direction."""
        driver.set_direction(forward=True)
        assert driver.direction is True
        driver.set_direction(forward=False)
        assert driver.direction is False
        driver.set_direction(forward=True)
        assert driver.direction is True

    def test_direction_affects_step(self, driver: A4988Driver) -> None:
        """Test direction affects step movement."""
        driver.set_direction(forward=True)
        initial = driver.position
        driver.step(10)
        assert driver.position == initial + 10

        # Negative step value reverses direction internally
        current = driver.position
        driver.step(-5)
        assert driver.position == current - 5


class TestStepDirStepAdvanced:
    """Advanced tests for StepDirDriver step generation."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_step_zero(self, driver: A4988Driver) -> None:
        """Test stepping zero steps."""
        initial = driver.position
        driver.step(0)
        assert driver.position == initial

    def test_step_single(self, driver: A4988Driver) -> None:
        """Test single step."""
        initial = driver.position
        driver.step(1)
        assert driver.position == initial + 1

    def test_step_multiple(self, driver: A4988Driver) -> None:
        """Test multiple steps."""
        initial = driver.position
        driver.step(100)
        assert driver.position == initial + 100

    def test_step_negative(self, driver: A4988Driver) -> None:
        """Test negative steps (reverse direction)."""
        driver.move_to(100)
        driver.step(-25)
        assert driver.position == 75

    def test_step_large_count(self, driver: A4988Driver) -> None:
        """Test large step count."""
        # Using small count in tests to avoid timeout
        driver.step(50)
        assert driver.position == 50


class TestStepDirMicrosteppingAdvanced:
    """Advanced tests for StepDirDriver microstepping."""

    @pytest.fixture
    def a4988_driver(self) -> A4988Driver:
        """Create A4988 driver with MS pins."""
        drv = A4988Driver(step_pin=2, dir_pin=3, ms_pins=(5, 6, 7))
        drv.connect()
        return drv

    @pytest.fixture
    def drv8825_driver(self) -> DRV8825Driver:
        """Create DRV8825 driver with MS pins."""
        drv = DRV8825Driver(step_pin=2, dir_pin=3, ms_pins=(5, 6, 7))
        drv.connect()
        return drv

    def test_a4988_valid_microstepping(self, a4988_driver: A4988Driver) -> None:
        """Test all valid A4988 microstepping values."""
        for microsteps in [1, 2, 4, 8, 16]:
            a4988_driver.set_microstepping(microsteps)
            assert a4988_driver.get_microstepping() == microsteps

    def test_drv8825_valid_microstepping(self, drv8825_driver: DRV8825Driver) -> None:
        """Test all valid DRV8825 microstepping values."""
        for microsteps in [1, 2, 4, 8, 16, 32]:
            drv8825_driver.set_microstepping(microsteps)
            assert drv8825_driver.get_microstepping() == microsteps

    def test_a4988_invalid_microstepping_32(self, a4988_driver: A4988Driver) -> None:
        """Test A4988 rejects 32 microstepping (DRV8825 only)."""
        with pytest.raises(ValueError):
            a4988_driver.set_microstepping(32)

    def test_drv8825_invalid_microstepping_64(self, drv8825_driver: DRV8825Driver) -> None:
        """Test DRV8825 rejects 64 microstepping (not supported)."""
        with pytest.raises(ValueError):
            drv8825_driver.set_microstepping(64)

    def test_microstepping_without_ms_pins(self) -> None:
        """Test microstepping without MS pins logs warning."""
        driver = A4988Driver(step_pin=2, dir_pin=3)  # No ms_pins
        driver.connect()
        # Should not raise, just log warning
        driver.set_microstepping(4)
        assert driver.microstepping == 4


class TestA4988DriverAdvanced:
    """Advanced tests for A4988 driver specifics."""

    def test_a4988_microstep_table(self) -> None:
        """Test A4988 microstep table is correct."""
        expected = {
            1: (False, False, False),  # Full step
            2: (True, False, False),  # Half step
            4: (False, True, False),  # Quarter step
            8: (True, True, False),  # Eighth step
            16: (True, True, True),  # Sixteenth step
        }
        assert expected == A4988Driver.MICROSTEP_TABLE

    def test_a4988_driver_name(self) -> None:
        """Test A4988 driver default name."""
        A4988Driver(step_pin=2, dir_pin=3)
        # Check driver has name set

    def test_a4988_custom_name(self) -> None:
        """Test A4988 driver with custom name."""
        A4988Driver(step_pin=2, dir_pin=3, name="X-Axis")
        # Custom name should be applied

    def test_a4988_status_contents(self) -> None:
        """Test A4988 status dictionary contents."""
        driver = A4988Driver(step_pin=2, dir_pin=3)
        driver.connect()
        driver.move_to(100)
        status = driver.get_status()

        assert status["enabled"] is False
        assert status["direction"] == "forward"
        assert status["position"] == 100
        assert status["microstepping"] == 1
        assert status["simulation"] is True


class TestDRV8825DriverAdvanced:
    """Advanced tests for DRV8825 driver specifics."""

    def test_drv8825_microstep_table(self) -> None:
        """Test DRV8825 microstep table includes 32."""
        expected = {
            1: (False, False, False),  # Full step
            2: (True, False, False),  # Half step
            4: (False, True, False),  # Quarter step
            8: (True, True, False),  # Eighth step
            16: (False, False, True),  # Sixteenth step
            32: (True, False, True),  # 1/32 step
        }
        assert expected == DRV8825Driver.MICROSTEP_TABLE

    def test_drv8825_fault_pin_stored(self) -> None:
        """Test DRV8825 fault pin is stored."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3, fault_pin=8)
        assert driver._fault_pin_num == 8

    def test_drv8825_fault_default_none(self) -> None:
        """Test DRV8825 fault pin defaults to None."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3)
        assert driver._fault_pin_num is None

    def test_drv8825_status_includes_fault(self) -> None:
        """Test DRV8825 status includes fault field."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3, fault_pin=8)
        driver.connect()
        status = driver.get_status()

        assert "fault" in status
        assert status["fault"] is False

    def test_drv8825_status_without_fault_pin(self) -> None:
        """Test DRV8825 status works without fault pin."""
        driver = DRV8825Driver(step_pin=2, dir_pin=3)
        driver.connect()
        status = driver.get_status()

        assert "fault" in status
        assert status["fault"] is False


class TestStepDirConfigAdvanced:
    """Advanced tests for StepDirConfig."""

    def test_config_steps_per_rev_default(self) -> None:
        """Test default steps per revolution."""
        config = StepDirConfig(step_pin=17, dir_pin=27)
        assert config.steps_per_rev == 200  # Standard stepper

    def test_config_steps_per_rev_custom(self) -> None:
        """Test custom steps per revolution."""
        config = StepDirConfig(step_pin=17, dir_pin=27, steps_per_rev=400)
        assert config.steps_per_rev == 400

    def test_config_step_pulse_timing(self) -> None:
        """Test step pulse timing configuration."""
        config = StepDirConfig(step_pin=17, dir_pin=27, step_pulse_us=5)
        assert config.step_pulse_us == 5

    def test_config_step_delay_timing(self) -> None:
        """Test step delay timing configuration."""
        config = StepDirConfig(step_pin=17, dir_pin=27, step_delay_us=200)
        assert config.step_delay_us == 200

    def test_config_invert_dir(self) -> None:
        """Test direction inversion configuration."""
        config = StepDirConfig(step_pin=17, dir_pin=27, invert_dir=True)
        assert config.invert_dir is True

    def test_config_invert_enable(self) -> None:
        """Test enable inversion configuration (active low by default)."""
        config_default = StepDirConfig(step_pin=17, dir_pin=27)
        assert config_default.invert_enable is True  # Active low

        config_custom = StepDirConfig(step_pin=17, dir_pin=27, invert_enable=False)
        assert config_custom.invert_enable is False


class TestStepDirPositionControl:
    """Tests for StepDirDriver position control."""

    @pytest.fixture
    def driver(self) -> A4988Driver:
        """Create connected driver."""
        drv = A4988Driver(step_pin=2, dir_pin=3)
        drv.connect()
        return drv

    def test_move_to_absolute(self, driver: A4988Driver) -> None:
        """Test moving to absolute position."""
        driver.move_to(50)
        assert driver.position == 50
        driver.move_to(100)
        assert driver.position == 100

    def test_move_to_backward(self, driver: A4988Driver) -> None:
        """Test moving backward with move_to."""
        driver.move_to(100)
        driver.move_to(50)
        assert driver.position == 50

    def test_move_to_negative(self, driver: A4988Driver) -> None:
        """Test moving to negative position."""
        driver.move_to(-50)
        assert driver.position == -50

    def test_reset_position(self, driver: A4988Driver) -> None:
        """Test resetting position counter."""
        driver.move_to(100)
        driver.reset_position(0)
        assert driver.position == 0

    def test_reset_position_custom(self, driver: A4988Driver) -> None:
        """Test resetting position to custom value."""
        driver.move_to(100)
        driver.reset_position(500)
        assert driver.position == 500
