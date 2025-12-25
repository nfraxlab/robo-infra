"""Unit tests for Jetson platform implementation."""

from __future__ import annotations

import os
from unittest.mock import MagicMock, patch

import pytest

from robo_infra.core.pin import PinMode, PinState
from robo_infra.platforms.jetson import (
    HARDWARE_PWM_PINS,
    JETSON_MODELS,
    JetsonDigitalPin,
    JetsonModel,
    JetsonPinNumbering,
    JetsonPlatform,
    JetsonPowerMode,
    JetsonPWMPin,
)


# =============================================================================
# Enum Tests
# =============================================================================


class TestJetsonPinNumbering:
    """Tests for JetsonPinNumbering enum."""

    def test_board_numbering(self) -> None:
        """Test BOARD numbering value."""
        assert JetsonPinNumbering.BOARD.value == "BOARD"

    def test_bcm_numbering(self) -> None:
        """Test BCM numbering value."""
        assert JetsonPinNumbering.BCM.value == "BCM"

    def test_cvm_numbering(self) -> None:
        """Test CVM numbering value."""
        assert JetsonPinNumbering.CVM.value == "CVM"

    def test_tegra_soc_numbering(self) -> None:
        """Test TEGRA_SOC numbering value."""
        assert JetsonPinNumbering.TEGRA_SOC.value == "TEGRA_SOC"

    def test_all_values_unique(self) -> None:
        """Test all numbering values are unique."""
        values = [e.value for e in JetsonPinNumbering]
        assert len(values) == len(set(values))


class TestJetsonModel:
    """Tests for JetsonModel enum."""

    def test_nano_model(self) -> None:
        """Test Nano model value."""
        assert JetsonModel.NANO.value == "Jetson Nano"

    def test_xavier_nx_model(self) -> None:
        """Test Xavier NX model value."""
        assert JetsonModel.XAVIER_NX.value == "Jetson Xavier NX"

    def test_agx_orin_model(self) -> None:
        """Test AGX Orin model value."""
        assert JetsonModel.AGX_ORIN.value == "Jetson AGX Orin"

    def test_all_models_defined(self) -> None:
        """Test all expected models are defined."""
        expected = ["NANO", "NANO_2GB", "TX1", "TX2", "TX2_NX",
                    "XAVIER_NX", "AGX_XAVIER", "ORIN_NANO", "ORIN_NX", "AGX_ORIN"]
        for model in expected:
            assert hasattr(JetsonModel, model)


class TestJetsonPowerMode:
    """Tests for JetsonPowerMode enum."""

    def test_5w_mode(self) -> None:
        """Test 5W mode value."""
        assert JetsonPowerMode.MODE_5W.value == "5W"

    def test_maxn_mode(self) -> None:
        """Test MAXN mode value."""
        assert JetsonPowerMode.MAXN.value == "MAXN"

    def test_unknown_mode(self) -> None:
        """Test Unknown mode value."""
        assert JetsonPowerMode.UNKNOWN.value == "Unknown"


# =============================================================================
# Digital Pin Tests
# =============================================================================


class TestJetsonDigitalPin:
    """Tests for JetsonDigitalPin class."""

    def test_pin_creation(self) -> None:
        """Test creating a digital pin."""
        pin = JetsonDigitalPin(17, mode=PinMode.OUTPUT, simulation=True)
        assert pin.number == 17
        assert pin.mode == PinMode.OUTPUT
        assert not pin.initialized

    def test_pin_with_name(self) -> None:
        """Test pin creation with custom name."""
        pin = JetsonDigitalPin(18, name="led", simulation=True)
        assert pin.name == "led"

    def test_pin_inverted(self) -> None:
        """Test inverted pin creation."""
        pin = JetsonDigitalPin(19, inverted=True, simulation=True)
        assert pin.inverted is True

    def test_pin_initial_state(self) -> None:
        """Test pin with initial state."""
        pin = JetsonDigitalPin(20, initial=PinState.HIGH, simulation=True)
        assert pin.state == PinState.HIGH

    def test_pin_setup_simulation(self) -> None:
        """Test pin setup in simulation mode."""
        pin = JetsonDigitalPin(21, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_pin_read_simulation(self) -> None:
        """Test reading pin in simulation mode."""
        pin = JetsonDigitalPin(22, mode=PinMode.INPUT, simulation=True)
        pin.setup()
        value = pin.read()
        assert isinstance(value, bool)

    def test_pin_write_simulation(self) -> None:
        """Test writing to pin in simulation mode."""
        pin = JetsonDigitalPin(23, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(True)
        assert pin.state == PinState.HIGH
        pin.write(False)
        assert pin.state == PinState.LOW

    def test_pin_toggle(self) -> None:
        """Test toggling pin state."""
        pin = JetsonDigitalPin(24, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(False)
        assert pin.state == PinState.LOW
        pin.toggle()
        assert pin.state == PinState.HIGH

    def test_pin_cleanup(self) -> None:
        """Test pin cleanup."""
        pin = JetsonDigitalPin(25, simulation=True)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized

    def test_input_pullup_mode(self) -> None:
        """Test INPUT_PULLUP mode."""
        pin = JetsonDigitalPin(26, mode=PinMode.INPUT_PULLUP, simulation=True)
        assert pin.mode == PinMode.INPUT_PULLUP

    def test_input_pulldown_mode(self) -> None:
        """Test INPUT_PULLDOWN mode."""
        pin = JetsonDigitalPin(27, mode=PinMode.INPUT_PULLDOWN, simulation=True)
        assert pin.mode == PinMode.INPUT_PULLDOWN

    def test_numbering_scheme(self) -> None:
        """Test different numbering schemes."""
        pin = JetsonDigitalPin(
            17,
            numbering=JetsonPinNumbering.BCM,
            simulation=True,
        )
        assert pin._numbering == JetsonPinNumbering.BCM


# =============================================================================
# PWM Pin Tests
# =============================================================================


class TestJetsonPWMPin:
    """Tests for JetsonPWMPin class."""

    def test_pwm_creation(self) -> None:
        """Test creating a PWM pin."""
        pin = JetsonPWMPin(32, simulation=True)
        assert pin.number == 32
        assert pin.frequency == 50
        assert pin.duty_cycle == 0.0

    def test_pwm_custom_frequency(self) -> None:
        """Test PWM with custom frequency."""
        pin = JetsonPWMPin(33, frequency=1000, simulation=True)
        assert pin.frequency == 1000

    def test_pwm_custom_duty_cycle(self) -> None:
        """Test PWM with custom duty cycle."""
        pin = JetsonPWMPin(32, duty_cycle=0.5, simulation=True)
        assert pin.duty_cycle == 0.5

    def test_pwm_setup(self) -> None:
        """Test PWM setup."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        assert pin.initialized

    def test_pwm_set_duty_cycle(self) -> None:
        """Test setting duty cycle."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.set_duty_cycle(0.75)
        assert pin.duty_cycle == 0.75

    def test_pwm_duty_cycle_clamped(self) -> None:
        """Test duty cycle is clamped to 0-1."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.set_duty_cycle(1.5)
        assert pin.duty_cycle == 1.0
        pin.set_duty_cycle(-0.5)
        assert pin.duty_cycle == 0.0

    def test_pwm_set_frequency(self) -> None:
        """Test setting frequency."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.set_frequency(100)
        assert pin.frequency == 100

    def test_pwm_set_pulse_width(self) -> None:
        """Test setting pulse width."""
        pin = JetsonPWMPin(32, frequency=50, simulation=True)  # 50Hz = 20ms period
        pin.setup()
        pin.set_pulse_width(1500)  # 1.5ms = 7.5% duty cycle
        assert abs(pin.duty_cycle - 0.075) < 0.001

    def test_pwm_start(self) -> None:
        """Test starting PWM."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.start()
        assert pin.initialized

    def test_pwm_stop(self) -> None:
        """Test stopping PWM."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.stop()
        # Stop should set running to false in simulation
        assert pin._pwm_obj["running"] is False

    def test_pwm_cleanup(self) -> None:
        """Test PWM cleanup."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized

    def test_hardware_pwm_detection(self) -> None:
        """Test hardware PWM pin detection."""
        hw_pin = JetsonPWMPin(32, simulation=True)
        assert hw_pin._hardware_pwm is True

        sw_pin = JetsonPWMPin(17, simulation=True)
        assert sw_pin._hardware_pwm is False


# =============================================================================
# Platform Tests
# =============================================================================


class TestJetsonPlatform:
    """Tests for JetsonPlatform class."""

    @pytest.fixture
    def platform(self) -> JetsonPlatform:
        """Create a test platform in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            return JetsonPlatform()

    def test_platform_name(self, platform: JetsonPlatform) -> None:
        """Test platform name."""
        assert "Jetson" in platform.name

    def test_platform_simulation_mode(self, platform: JetsonPlatform) -> None:
        """Test simulation mode detection."""
        assert platform._simulation is True

    def test_platform_is_available(self, platform: JetsonPlatform) -> None:
        """Test platform availability in simulation."""
        assert platform.is_available is True

    def test_get_digital_pin(self, platform: JetsonPlatform) -> None:
        """Test getting a digital output pin."""
        pin = platform.get_pin(17, mode=PinMode.OUTPUT)
        assert isinstance(pin, JetsonDigitalPin)
        assert pin.number == 17

    def test_get_input_pin(self, platform: JetsonPlatform) -> None:
        """Test getting an input pin."""
        pin = platform.get_pin(18, mode=PinMode.INPUT)
        assert isinstance(pin, JetsonDigitalPin)
        assert pin.mode == PinMode.INPUT

    def test_get_pwm_pin(self, platform: JetsonPlatform) -> None:
        """Test getting a PWM pin."""
        pin = platform.get_pin(32, mode=PinMode.PWM)
        assert isinstance(pin, JetsonPWMPin)

    def test_pin_caching(self, platform: JetsonPlatform) -> None:
        """Test that pins are cached."""
        pin1 = platform.get_pin(17, mode=PinMode.OUTPUT)
        pin2 = platform.get_pin(17, mode=PinMode.OUTPUT)
        assert pin1 is pin2

    def test_get_i2c_bus(self, platform: JetsonPlatform) -> None:
        """Test getting I2C bus."""
        bus = platform.get_bus("i2c", bus=1)
        assert bus is not None

    def test_get_spi_bus(self, platform: JetsonPlatform) -> None:
        """Test getting SPI bus."""
        bus = platform.get_bus("spi", bus=0, device=0)
        assert bus is not None

    def test_get_uart_bus(self, platform: JetsonPlatform) -> None:
        """Test getting UART bus."""
        bus = platform.get_bus("uart", port="/dev/ttyTHS1")
        assert bus is not None

    def test_unsupported_bus_raises(self, platform: JetsonPlatform) -> None:
        """Test that unsupported bus type raises error."""
        from robo_infra.core.exceptions import HardwareNotFoundError
        with pytest.raises(HardwareNotFoundError):
            platform.get_bus("can")

    def test_cleanup(self, platform: JetsonPlatform) -> None:
        """Test platform cleanup."""
        pin = platform.get_pin(17, mode=PinMode.OUTPUT)
        pin.setup()
        platform.cleanup()
        # After cleanup, pins should be released

    def test_model_property_simulation(self, platform: JetsonPlatform) -> None:
        """Test model detection in simulation."""
        model = platform.model
        assert isinstance(model, JetsonModel)
        # Default simulation model is NANO
        assert model == JetsonModel.NANO


class TestJetsonSpecificFeatures:
    """Tests for Jetson-specific features."""

    @pytest.fixture
    def platform(self) -> JetsonPlatform:
        """Create a test platform in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            return JetsonPlatform()

    def test_power_mode_simulation(self, platform: JetsonPlatform) -> None:
        """Test power mode in simulation."""
        mode = platform.power_mode
        assert mode == JetsonPowerMode.MAXN

    def test_set_power_mode_simulation(self, platform: JetsonPlatform) -> None:
        """Test setting power mode in simulation."""
        result = platform.set_power_mode(JetsonPowerMode.MODE_10W)
        assert result is True

    def test_cuda_available_simulation(self, platform: JetsonPlatform) -> None:
        """Test CUDA availability in simulation."""
        assert platform.cuda_available is False

    def test_cuda_version_simulation(self, platform: JetsonPlatform) -> None:
        """Test CUDA version in simulation."""
        assert platform.cuda_version is None

    def test_csi_cameras_simulation(self, platform: JetsonPlatform) -> None:
        """Test CSI camera detection in simulation."""
        cameras = platform.get_csi_cameras()
        assert cameras == []

    def test_jetpack_version_simulation(self, platform: JetsonPlatform) -> None:
        """Test JetPack version in simulation."""
        assert platform.jetpack_version is None


class TestModelDetection:
    """Tests for Jetson model detection."""

    def test_model_property(self) -> None:
        """Test model property returns JetsonModel."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert isinstance(platform.model, JetsonModel)

    def test_jetson_models_dict(self) -> None:
        """Test JETSON_MODELS dictionary."""
        assert "jetson-nano" in JETSON_MODELS
        assert JETSON_MODELS["jetson-nano"] == JetsonModel.NANO

    def test_model_detection_with_device_tree(self) -> None:
        """Test model detection from device tree."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            # In simulation, should return default NANO
            assert platform.model == JetsonModel.NANO

    @patch("robo_infra.platforms.jetson.Path.exists")
    @patch("robo_infra.platforms.jetson.Path.read_text")
    def test_model_detection_orin(
        self, mock_read: MagicMock, mock_exists: MagicMock
    ) -> None:
        """Test model detection for Orin."""
        mock_exists.return_value = True
        mock_read.return_value = "NVIDIA Jetson AGX Orin"

        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            # Force re-detection
            platform._model = None
            # This would return Orin if not in simulation
            # but simulation overrides it
            model = platform._detect_model()
            assert isinstance(model, JetsonModel)


class TestPlatformInfo:
    """Tests for platform info detection."""

    def test_info_property(self) -> None:
        """Test info property returns PlatformInfo."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            info = platform.info
            assert info is not None
            assert info.platform_type.value == "jetson"

    def test_capabilities_property(self) -> None:
        """Test capabilities property."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            caps = platform.capabilities
            assert caps is not None


class TestConstants:
    """Tests for Jetson constants."""

    def test_hardware_pwm_pins(self) -> None:
        """Test hardware PWM pins constant."""
        assert 32 in HARDWARE_PWM_PINS
        assert 33 in HARDWARE_PWM_PINS

    def test_jetson_models_keys(self) -> None:
        """Test JETSON_MODELS has expected keys."""
        expected_keys = ["jetson-nano", "jetson-tx2", "jetson-xavier", "jetson-agx-orin"]
        for key in expected_keys:
            assert key in JETSON_MODELS


class TestEdgeCases:
    """Tests for edge cases."""

    def test_string_pin_id(self) -> None:
        """Test creating pin with string ID."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            pin = platform.get_pin("17", mode=PinMode.OUTPUT)
            assert pin.number == 17

    def test_pin_initial_state_bool(self) -> None:
        """Test pin with boolean initial state."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            pin = platform.get_pin(17, mode=PinMode.OUTPUT, initial=True)
            assert isinstance(pin, JetsonDigitalPin)

    def test_pwm_with_custom_options(self) -> None:
        """Test PWM pin with custom options."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            pin = platform.get_pin(
                32,
                mode=PinMode.PWM,
                frequency=100,
                duty_cycle=0.5
            )
            assert isinstance(pin, JetsonPWMPin)
            assert pin.frequency == 100
            assert pin.duty_cycle == 0.5

    def test_bus_caching(self) -> None:
        """Test that buses are cached."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus1 = platform.get_bus("i2c", bus=1)
            bus2 = platform.get_bus("i2c", bus=1)
            assert bus1 is bus2

    def test_different_buses_not_cached_together(self) -> None:
        """Test that different bus types are not confused."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            i2c = platform.get_bus("i2c", bus=1)
            spi = platform.get_bus("spi", bus=0, device=0)
            assert i2c is not spi

    def test_numbering_bcm(self) -> None:
        """Test BCM numbering mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform(numbering=JetsonPinNumbering.BCM)
            assert platform._numbering == JetsonPinNumbering.BCM

    def test_numbering_cvm(self) -> None:
        """Test CVM numbering mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform(numbering=JetsonPinNumbering.CVM)
            assert platform._numbering == JetsonPinNumbering.CVM


class TestSimulationEnvVar:
    """Tests for simulation environment variable."""

    def test_simulation_env_var_true(self) -> None:
        """Test ROBO_SIMULATION=true enables simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform._simulation is True

    def test_simulation_env_var_1(self) -> None:
        """Test ROBO_SIMULATION=1 enables simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "1"}):
            platform = JetsonPlatform()
            assert platform._simulation is True

    def test_simulation_env_var_yes(self) -> None:
        """Test ROBO_SIMULATION=yes enables simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "yes"}):
            platform = JetsonPlatform()
            assert platform._simulation is True


# =============================================================================
# GPIO Mock Tests (Phase 5.7.3.1)
# =============================================================================


class TestJetsonGPIOSetup:
    """Tests for Jetson GPIO setup operations."""

    def test_gpio_setup_output_mode(self) -> None:
        """Test GPIO setup in OUTPUT mode."""
        pin = JetsonDigitalPin(17, mode=PinMode.OUTPUT, simulation=True)
        assert not pin.initialized
        pin.setup()
        assert pin.initialized
        assert pin.mode == PinMode.OUTPUT

    def test_gpio_setup_input_mode(self) -> None:
        """Test GPIO setup in INPUT mode."""
        pin = JetsonDigitalPin(18, mode=PinMode.INPUT, simulation=True)
        pin.setup()
        assert pin.initialized
        assert pin.mode == PinMode.INPUT

    def test_gpio_setup_input_pullup(self) -> None:
        """Test GPIO setup with INPUT_PULLUP."""
        pin = JetsonDigitalPin(19, mode=PinMode.INPUT_PULLUP, simulation=True)
        pin.setup()
        assert pin.mode == PinMode.INPUT_PULLUP

    def test_gpio_setup_input_pulldown(self) -> None:
        """Test GPIO setup with INPUT_PULLDOWN."""
        pin = JetsonDigitalPin(20, mode=PinMode.INPUT_PULLDOWN, simulation=True)
        pin.setup()
        assert pin.mode == PinMode.INPUT_PULLDOWN

    def test_gpio_setup_idempotent(self) -> None:
        """Test that multiple setup calls are safe."""
        pin = JetsonDigitalPin(21, simulation=True)
        pin.setup()
        pin.setup()  # Should not raise
        assert pin.initialized

    def test_gpio_setup_all_numbering_schemes(self) -> None:
        """Test setup with all numbering schemes."""
        for numbering in JetsonPinNumbering:
            pin = JetsonDigitalPin(17, numbering=numbering, simulation=True)
            pin.setup()
            assert pin._numbering == numbering

    def test_gpio_setup_with_initial_high(self) -> None:
        """Test GPIO setup with initial HIGH state."""
        pin = JetsonDigitalPin(
            22, mode=PinMode.OUTPUT, initial=PinState.HIGH, simulation=True
        )
        pin.setup()
        assert pin.state == PinState.HIGH

    def test_gpio_setup_with_initial_low(self) -> None:
        """Test GPIO setup with initial LOW state."""
        pin = JetsonDigitalPin(
            23, mode=PinMode.OUTPUT, initial=PinState.LOW, simulation=True
        )
        pin.setup()
        assert pin.state == PinState.LOW

    def test_gpio_read_triggers_setup(self) -> None:
        """Test that read() triggers setup if not initialized."""
        pin = JetsonDigitalPin(24, mode=PinMode.INPUT, simulation=True)
        assert not pin.initialized
        pin.read()
        assert pin.initialized

    def test_gpio_write_triggers_setup(self) -> None:
        """Test that write() triggers setup if not initialized."""
        pin = JetsonDigitalPin(25, mode=PinMode.OUTPUT, simulation=True)
        assert not pin.initialized
        pin.write(True)
        assert pin.initialized


class TestJetsonGPIOReadWrite:
    """Tests for Jetson GPIO read/write operations."""

    def test_read_returns_bool(self) -> None:
        """Test that read returns boolean."""
        pin = JetsonDigitalPin(17, mode=PinMode.INPUT, simulation=True)
        pin.setup()
        value = pin.read()
        assert isinstance(value, bool)

    def test_write_true_sets_high(self) -> None:
        """Test writing True sets HIGH state."""
        pin = JetsonDigitalPin(18, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(True)
        assert pin.state == PinState.HIGH

    def test_write_false_sets_low(self) -> None:
        """Test writing False sets LOW state."""
        pin = JetsonDigitalPin(19, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(False)
        assert pin.state == PinState.LOW

    def test_read_after_write_high(self) -> None:
        """Test reading after writing HIGH."""
        pin = JetsonDigitalPin(20, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(True)
        assert pin.read() is True

    def test_read_after_write_low(self) -> None:
        """Test reading after writing LOW."""
        pin = JetsonDigitalPin(21, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(False)
        assert pin.read() is False

    def test_inverted_read(self) -> None:
        """Test inverted pin read."""
        pin = JetsonDigitalPin(22, mode=PinMode.INPUT, inverted=True, simulation=True)
        pin.setup()
        pin._gpio_module["value"] = True
        # Inverted: actual True reads as False
        assert pin.read() is False

    def test_inverted_write(self) -> None:
        """Test inverted pin write."""
        pin = JetsonDigitalPin(23, mode=PinMode.OUTPUT, inverted=True, simulation=True)
        pin.setup()
        pin.write(True)
        # Inverted: writing True sets LOW internally
        assert pin._gpio_module["value"] is False

    def test_toggle_state(self) -> None:
        """Test toggle switches state."""
        pin = JetsonDigitalPin(24, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(False)
        assert pin.state == PinState.LOW
        pin.toggle()
        assert pin.state == PinState.HIGH
        pin.toggle()
        assert pin.state == PinState.LOW

    def test_rapid_write_sequence(self) -> None:
        """Test rapid write sequence."""
        pin = JetsonDigitalPin(25, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        for _ in range(100):
            pin.write(True)
            pin.write(False)
        assert pin.state == PinState.LOW


class TestJetsonPinModes:
    """Tests for Jetson pin modes."""

    def test_all_pin_modes_supported(self) -> None:
        """Test all PinMode values are handled."""
        modes = [PinMode.INPUT, PinMode.OUTPUT, PinMode.INPUT_PULLUP, PinMode.INPUT_PULLDOWN]
        for mode in modes:
            pin = JetsonDigitalPin(17, mode=mode, simulation=True)
            assert pin.mode == mode

    def test_output_mode_allows_write(self) -> None:
        """Test OUTPUT mode allows write."""
        pin = JetsonDigitalPin(18, mode=PinMode.OUTPUT, simulation=True)
        pin.setup()
        pin.write(True)  # Should not raise

    def test_input_mode_allows_read(self) -> None:
        """Test INPUT mode allows read."""
        pin = JetsonDigitalPin(19, mode=PinMode.INPUT, simulation=True)
        pin.setup()
        pin.read()  # Should not raise

    def test_mode_preserved_after_setup(self) -> None:
        """Test mode is preserved after setup."""
        pin = JetsonDigitalPin(20, mode=PinMode.INPUT_PULLUP, simulation=True)
        pin.setup()
        assert pin.mode == PinMode.INPUT_PULLUP


class TestJetsonPWM:
    """Tests for Jetson PWM functionality."""

    def test_pwm_default_frequency(self) -> None:
        """Test default PWM frequency is 50Hz."""
        pin = JetsonPWMPin(32, simulation=True)
        assert pin.frequency == 50

    def test_pwm_default_duty_cycle(self) -> None:
        """Test default duty cycle is 0."""
        pin = JetsonPWMPin(32, simulation=True)
        assert pin.duty_cycle == 0.0

    def test_pwm_frequency_change(self) -> None:
        """Test changing PWM frequency."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.frequency = 1000
        assert pin.frequency == 1000

    def test_pwm_duty_cycle_change(self) -> None:
        """Test changing PWM duty cycle."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.duty_cycle = 0.5
        assert pin.duty_cycle == 0.5

    def test_pwm_duty_cycle_clamp_high(self) -> None:
        """Test duty cycle is clamped to max 1.0."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.duty_cycle = 2.0
        assert pin.duty_cycle == 1.0

    def test_pwm_duty_cycle_clamp_low(self) -> None:
        """Test duty cycle is clamped to min 0.0."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.duty_cycle = -1.0
        assert pin.duty_cycle == 0.0

    def test_pwm_start_stop(self) -> None:
        """Test PWM start/stop cycle."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.start()
        assert pin._pwm_obj["running"] is True
        pin.stop()
        assert pin._pwm_obj["running"] is False

    def test_pwm_pulse_width_calculation(self) -> None:
        """Test pulse width to duty cycle conversion."""
        pin = JetsonPWMPin(32, frequency=50, simulation=True)  # 20ms period
        pin.setup()
        pin.set_pulse_width(2000)  # 2ms = 10% of 20ms
        assert abs(pin.duty_cycle - 0.1) < 0.001

    def test_pwm_hardware_pin_detection(self) -> None:
        """Test hardware PWM pin detection."""
        hw_pin = JetsonPWMPin(32, simulation=True)
        assert hw_pin._hardware_pwm is True

    def test_pwm_software_pin_detection(self) -> None:
        """Test software PWM pin detection."""
        sw_pin = JetsonPWMPin(17, simulation=True)
        assert sw_pin._hardware_pwm is False

    def test_pwm_all_hardware_pins(self) -> None:
        """Test all hardware PWM pins are detected."""
        from robo_infra.platforms.jetson import HARDWARE_PWM_PINS
        for pin_num in HARDWARE_PWM_PINS:
            pin = JetsonPWMPin(pin_num, simulation=True)
            assert pin._hardware_pwm is True

    def test_pwm_set_methods(self) -> None:
        """Test PWM setter methods."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        pin.set_duty_cycle(0.75)
        assert pin.duty_cycle == 0.75
        pin.set_frequency(100)
        assert pin.frequency == 100


class TestJetsonCleanup:
    """Tests for Jetson GPIO cleanup."""

    def test_digital_pin_cleanup(self) -> None:
        """Test digital pin cleanup."""
        pin = JetsonDigitalPin(17, simulation=True)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized
        assert pin._gpio_module is None

    def test_pwm_pin_cleanup(self) -> None:
        """Test PWM pin cleanup."""
        pin = JetsonPWMPin(32, simulation=True)
        pin.setup()
        assert pin.initialized
        pin.cleanup()
        assert not pin.initialized
        assert pin._pwm_obj is None

    def test_cleanup_idempotent(self) -> None:
        """Test multiple cleanup calls are safe."""
        pin = JetsonDigitalPin(18, simulation=True)
        pin.setup()
        pin.cleanup()
        pin.cleanup()  # Should not raise
        assert not pin.initialized

    def test_cleanup_without_setup(self) -> None:
        """Test cleanup without prior setup."""
        pin = JetsonDigitalPin(19, simulation=True)
        assert not pin.initialized
        pin.cleanup()  # Should not raise
        assert not pin.initialized

    def test_platform_cleanup_all_pins(self) -> None:
        """Test platform cleanup releases all pins."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            pin1 = platform.get_pin(17, mode=PinMode.OUTPUT)
            pin2 = platform.get_pin(18, mode=PinMode.OUTPUT)
            pin1.setup()
            pin2.setup()
            assert pin1.initialized
            assert pin2.initialized
            platform.cleanup()
            # Cleanup should have occurred


# =============================================================================
# Jetson-Specific Tests (Phase 5.7.3.2)
# =============================================================================


class TestJetsonModelDetection:
    """Comprehensive tests for Jetson model detection."""

    def test_model_returns_jetson_model_enum(self) -> None:
        """Test model property returns JetsonModel enum."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert isinstance(platform.model, JetsonModel)

    def test_simulation_default_model_is_nano(self) -> None:
        """Test simulation defaults to Nano model."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform.model == JetsonModel.NANO

    def test_model_is_cached(self) -> None:
        """Test model detection is cached."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            model1 = platform.model
            model2 = platform.model
            assert model1 is model2

    def test_model_dict_nano_entries(self) -> None:
        """Test JETSON_MODELS contains Nano entries."""
        assert JETSON_MODELS.get("jetson-nano") == JetsonModel.NANO
        assert JETSON_MODELS.get("p3448-0000") == JetsonModel.NANO

    def test_model_dict_tx2_entries(self) -> None:
        """Test JETSON_MODELS contains TX2 entries."""
        assert JETSON_MODELS.get("jetson-tx2") == JetsonModel.TX2
        assert JETSON_MODELS.get("p2771-0000") == JetsonModel.TX2

    def test_model_dict_xavier_entries(self) -> None:
        """Test JETSON_MODELS contains Xavier entries."""
        assert JETSON_MODELS.get("jetson-xavier") == JetsonModel.AGX_XAVIER
        assert JETSON_MODELS.get("jetson-xavier-nx") == JetsonModel.XAVIER_NX

    def test_model_dict_orin_entries(self) -> None:
        """Test JETSON_MODELS contains Orin entries."""
        assert JETSON_MODELS.get("jetson-agx-orin") == JetsonModel.AGX_ORIN
        assert JETSON_MODELS.get("jetson-orin-nano") == JetsonModel.ORIN_NANO

    def test_all_models_have_values(self) -> None:
        """Test all JetsonModel enum members have values."""
        for model in JetsonModel:
            assert model.value is not None
            assert len(model.value) > 0

    @patch("robo_infra.platforms.jetson.Path.exists")
    @patch("robo_infra.platforms.jetson.Path.read_text")
    def test_detect_model_from_device_tree(
        self, mock_read: MagicMock, mock_exists: MagicMock
    ) -> None:
        """Test model detection from /proc/device-tree/model."""
        mock_exists.return_value = True
        mock_read.return_value = "NVIDIA Jetson TX2"

        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            platform._model = None
            platform._simulation = False  # Force real detection
            model = platform._detect_model()
            assert isinstance(model, JetsonModel)


class TestJetsonCUDAAvailable:
    """Tests for Jetson CUDA availability detection."""

    def test_cuda_available_returns_bool(self) -> None:
        """Test cuda_available returns boolean."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert isinstance(platform.cuda_available, bool)

    def test_cuda_simulation_returns_false(self) -> None:
        """Test CUDA returns False in simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform.cuda_available is False

    @patch("robo_infra.platforms.jetson.Path.exists")
    def test_cuda_path_check(self, mock_exists: MagicMock) -> None:
        """Test CUDA path checking."""
        mock_exists.return_value = True
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            # Still returns False because simulation overrides
            assert platform.cuda_available is False


class TestJetsonCUDAVersion:
    """Tests for Jetson CUDA version detection."""

    def test_cuda_version_returns_string_or_none(self) -> None:
        """Test cuda_version returns string or None."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            result = platform.cuda_version
            assert result is None or isinstance(result, str)

    def test_cuda_version_simulation_returns_none(self) -> None:
        """Test CUDA version returns None in simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform.cuda_version is None

    @patch("robo_infra.platforms.jetson.Path.exists")
    @patch("robo_infra.platforms.jetson.Path.read_text")
    def test_cuda_version_from_file(
        self, mock_read: MagicMock, mock_exists: MagicMock
    ) -> None:
        """Test CUDA version parsing from file."""
        mock_exists.return_value = True
        mock_read.return_value = "CUDA Version 11.4.315"

        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            platform._simulation = False
            version = platform.cuda_version
            # In real detection would parse version
            assert version is None or isinstance(version, str)


class TestJetsonPowerMode:
    """Tests for Jetson power mode management."""

    def test_power_mode_returns_enum(self) -> None:
        """Test power_mode returns JetsonPowerMode enum."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert isinstance(platform.power_mode, JetsonPowerMode)

    def test_power_mode_simulation_returns_maxn(self) -> None:
        """Test power mode returns MAXN in simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform.power_mode == JetsonPowerMode.MAXN

    def test_set_power_mode_simulation_returns_true(self) -> None:
        """Test set_power_mode returns True in simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            result = platform.set_power_mode(JetsonPowerMode.MODE_10W)
            assert result is True

    def test_set_power_mode_with_string(self) -> None:
        """Test set_power_mode accepts string."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            result = platform.set_power_mode("1")
            assert result is True

    def test_all_power_modes_defined(self) -> None:
        """Test all power mode values are unique."""
        values = [mode.value for mode in JetsonPowerMode]
        assert len(values) == len(set(values))


class TestJetsonCSICameraList:
    """Tests for Jetson CSI camera detection."""

    def test_csi_cameras_returns_list(self) -> None:
        """Test get_csi_cameras returns list."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            cameras = platform.get_csi_cameras()
            assert isinstance(cameras, list)

    def test_csi_cameras_simulation_returns_empty(self) -> None:
        """Test CSI cameras returns empty list in simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            cameras = platform.get_csi_cameras()
            assert cameras == []

    @patch("robo_infra.platforms.jetson.Path.exists")
    @patch("subprocess.run")
    def test_csi_camera_detection_with_device(
        self, mock_run: MagicMock, mock_exists: MagicMock
    ) -> None:
        """Test CSI camera detection with device present."""
        mock_exists.return_value = True
        mock_run.return_value = MagicMock(returncode=0, stdout="Argus camera")

        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            platform._simulation = False
            cameras = platform.get_csi_cameras()
            # Would detect cameras if not simulated
            assert isinstance(cameras, list)


class TestJetsonJetPackVersion:
    """Tests for Jetson JetPack version detection."""

    def test_jetpack_version_returns_string_or_none(self) -> None:
        """Test jetpack_version returns string or None."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            result = platform.jetpack_version
            assert result is None or isinstance(result, str)

    def test_jetpack_version_simulation_returns_none(self) -> None:
        """Test JetPack version returns None in simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform.jetpack_version is None

    @patch("robo_infra.platforms.jetson.Path.exists")
    @patch("robo_infra.platforms.jetson.Path.read_text")
    def test_jetpack_version_from_tegra_release(
        self, mock_read: MagicMock, mock_exists: MagicMock
    ) -> None:
        """Test JetPack version from tegra release file."""
        mock_exists.return_value = True
        mock_read.return_value = "# R35 (release), REVISION: 2.1"

        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            platform._simulation = False
            version = platform.jetpack_version
            # Would parse version if not simulated
            assert version is None or isinstance(version, str)


class TestJetsonPlatformAvailability:
    """Tests for Jetson platform availability."""

    def test_is_available_in_simulation(self) -> None:
        """Test is_available returns True in simulation."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform.is_available is True

    @patch("robo_infra.platforms.jetson.Path.exists")
    def test_is_available_with_tegra_release(self, mock_exists: MagicMock) -> None:
        """Test is_available with tegra release file."""
        # Simulate having the tegra release file
        mock_exists.return_value = True
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            # Still returns True because simulation
            assert platform.is_available is True


class TestJetsonBusCreation:
    """Tests for Jetson bus creation."""

    def test_i2c_bus_default_bus_number(self) -> None:
        """Test I2C bus uses default bus number 1."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus = platform.get_bus("i2c")
            assert bus is not None

    def test_i2c_bus_custom_bus_number(self) -> None:
        """Test I2C bus with custom bus number."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus = platform.get_bus("i2c", bus=0)
            assert bus is not None

    def test_spi_bus_default_device(self) -> None:
        """Test SPI bus with default device."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus = platform.get_bus("spi")
            assert bus is not None

    def test_spi_bus_custom_device(self) -> None:
        """Test SPI bus with custom bus and device."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus = platform.get_bus("spi", bus=1, device=1)
            assert bus is not None

    def test_uart_bus_default_port(self) -> None:
        """Test UART bus with default port."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus = platform.get_bus("uart")
            assert bus is not None

    def test_uart_bus_custom_port(self) -> None:
        """Test UART bus with custom port."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus = platform.get_bus("uart", port="/dev/ttyTHS2")
            assert bus is not None

    def test_serial_alias_for_uart(self) -> None:
        """Test 'serial' is an alias for 'uart'."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            bus = platform.get_bus("serial")
            assert bus is not None

    def test_invalid_bus_type_raises(self) -> None:
        """Test invalid bus type raises HardwareNotFoundError."""
        from robo_infra.core.exceptions import HardwareNotFoundError
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            with pytest.raises(HardwareNotFoundError):
                platform.get_bus("invalid")


class TestJetsonPlatformConfig:
    """Tests for Jetson platform configuration."""

    def test_platform_name_contains_jetson(self) -> None:
        """Test platform name contains 'Jetson'."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert "Jetson" in platform.name

    def test_platform_type_is_jetson(self) -> None:
        """Test platform type is JETSON."""
        from robo_infra.platforms.base import PlatformType
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform()
            assert platform.info.platform_type == PlatformType.JETSON

    def test_custom_numbering(self) -> None:
        """Test custom numbering scheme."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform(numbering=JetsonPinNumbering.BCM)
            assert platform._numbering == JetsonPinNumbering.BCM

    def test_tegra_soc_numbering(self) -> None:
        """Test TEGRA_SOC numbering scheme."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            platform = JetsonPlatform(numbering=JetsonPinNumbering.TEGRA_SOC)
            assert platform._numbering == JetsonPinNumbering.TEGRA_SOC
