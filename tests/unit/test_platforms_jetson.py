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
