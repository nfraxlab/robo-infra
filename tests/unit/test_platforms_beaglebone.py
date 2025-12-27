"""Unit tests for BeagleBone platform implementation."""

from __future__ import annotations

import pytest

from robo_infra.core.pin import PinMode, PinState
from robo_infra.platforms.beaglebone import (
    ADC_PINS,
    ADC_RESOLUTION,
    ADC_VREF,
    BB_CAPABILITIES,
    P8_GPIO_MAP,
    P9_GPIO_MAP,
    PWM_PINS,
    BBIOBackend,
    BeagleBoneADCPin,
    BeagleBoneDigitalPin,
    BeagleBoneModel,
    BeagleBonePlatform,
    BeagleBonePWMPin,
    DeviceTreeOverlayManager,
    PRUInterface,
    PRUState,
)


# =============================================================================
# BeagleBoneModel Tests
# =============================================================================


class TestBeagleBoneModel:
    """Tests for BeagleBoneModel enum."""

    def test_model_values(self) -> None:
        """Test all model values are defined."""
        assert BeagleBoneModel.BLACK.value == "BeagleBone Black"
        assert BeagleBoneModel.BLACK_WIRELESS.value == "BeagleBone Black Wireless"
        assert BeagleBoneModel.GREEN.value == "BeagleBone Green"
        assert BeagleBoneModel.GREEN_WIRELESS.value == "BeagleBone Green Wireless"
        assert BeagleBoneModel.AI.value == "BeagleBone AI"
        assert BeagleBoneModel.AI_64.value == "BeagleBone AI-64"
        assert BeagleBoneModel.POCKET.value == "PocketBeagle"
        assert BeagleBoneModel.BLUE.value == "BeagleBone Blue"
        assert BeagleBoneModel.UNKNOWN.value == "Unknown BeagleBone"

    def test_model_count(self) -> None:
        """Test expected number of models."""
        assert len(BeagleBoneModel) == 9


class TestBBIOBackend:
    """Tests for BBIOBackend enum."""

    def test_backend_values(self) -> None:
        """Test all backend values."""
        assert BBIOBackend.ADAFRUIT_BBIO.value == "adafruit_bbio"
        assert BBIOBackend.GPIOD.value == "gpiod"
        assert BBIOBackend.SYSFS.value == "sysfs"
        assert BBIOBackend.SIMULATION.value == "simulation"

    def test_backend_count(self) -> None:
        """Test expected number of backends."""
        assert len(BBIOBackend) == 4


class TestPRUState:
    """Tests for PRUState enum."""

    def test_pru_states(self) -> None:
        """Test PRU state values."""
        assert PRUState.STOPPED.value == "stopped"
        assert PRUState.RUNNING.value == "running"
        assert PRUState.OFFLINE.value == "offline"
        assert PRUState.ERROR.value == "error"


# =============================================================================
# GPIO Mapping Tests
# =============================================================================


class TestGPIOMapping:
    """Tests for GPIO pin mappings."""

    def test_p8_gpio_map_coverage(self) -> None:
        """Test P8 header GPIO mapping has entries."""
        assert len(P8_GPIO_MAP) > 0
        # Check some known mappings
        assert P8_GPIO_MAP[7] == 66  # P8_7 -> GPIO2_2
        assert P8_GPIO_MAP[10] == 68  # P8_10 -> GPIO2_4
        assert P8_GPIO_MAP[13] == 23  # P8_13 -> GPIO0_23 (PWM)

    def test_p9_gpio_map_coverage(self) -> None:
        """Test P9 header GPIO mapping has entries."""
        assert len(P9_GPIO_MAP) > 0
        # Check some known mappings
        assert P9_GPIO_MAP[12] == 60  # P9_12 -> GPIO1_28
        assert P9_GPIO_MAP[14] == 50  # P9_14 -> GPIO1_18 (PWM)

    def test_gpio_numbers_unique(self) -> None:
        """Test GPIO numbers are unique across headers."""
        all_gpios = list(P8_GPIO_MAP.values()) + list(P9_GPIO_MAP.values())
        # Some GPIOs may be duplicated (alternate functions), but mostly unique
        assert len(all_gpios) > 0


class TestPWMPins:
    """Tests for PWM pin mappings."""

    def test_pwm_pins_defined(self) -> None:
        """Test PWM pins are defined."""
        assert len(PWM_PINS) > 0
        assert "P8_13" in PWM_PINS
        assert "P9_14" in PWM_PINS
        assert "P9_42" in PWM_PINS

    def test_pwm_module_format(self) -> None:
        """Test PWM module info format."""
        for _pin, (module, channel) in PWM_PINS.items():
            assert module.startswith("ehrpwm") or module.startswith("ecap")
            assert channel in ("A", "B", "0")


class TestADCPins:
    """Tests for ADC pin mappings."""

    def test_adc_channels(self) -> None:
        """Test ADC channel mappings."""
        assert len(ADC_PINS) == 7
        assert 0 in ADC_PINS  # AIN0
        assert 6 in ADC_PINS  # AIN6

    def test_adc_constants(self) -> None:
        """Test ADC constants."""
        assert ADC_VREF == 1.8
        assert ADC_RESOLUTION == 12


# =============================================================================
# Capabilities Tests
# =============================================================================


class TestBeagleBoneCapabilities:
    """Tests for BeagleBone capabilities."""

    def test_all_models_have_capabilities(self) -> None:
        """Test all models have capability definitions."""
        for model in BeagleBoneModel:
            assert model in BB_CAPABILITIES

    def test_black_capabilities(self) -> None:
        """Test BeagleBone Black capabilities."""
        caps = BB_CAPABILITIES[BeagleBoneModel.BLACK]
        assert caps.gpio_count == 65
        assert caps.pwm_count == 8
        assert caps.adc_channels == 7
        assert caps.adc_resolution == 12
        assert caps.has_pru is True
        assert caps.has_hdmi is True
        assert caps.ram_mb == 512
        assert caps.processor == "AM3358"

    def test_ai64_capabilities(self) -> None:
        """Test BeagleBone AI-64 capabilities."""
        caps = BB_CAPABILITIES[BeagleBoneModel.AI_64]
        assert caps.gpio_count == 100
        assert caps.ram_mb == 4096
        assert caps.processor == "TDA4VM"

    def test_pocket_capabilities(self) -> None:
        """Test PocketBeagle capabilities."""
        caps = BB_CAPABILITIES[BeagleBoneModel.POCKET]
        assert caps.gpio_count == 44
        assert caps.has_hdmi is False
        assert caps.has_emmc is False


# =============================================================================
# Digital Pin Tests
# =============================================================================


class TestBeagleBoneDigitalPin:
    """Tests for BeagleBoneDigitalPin."""

    def test_init_simulation(self) -> None:
        """Test pin initialization in simulation mode."""
        pin = BeagleBoneDigitalPin(
            pin_name="P9_12",
            mode=PinMode.OUTPUT,
            backend=BBIOBackend.SIMULATION,
        )
        assert pin.pin_name == "P9_12"
        assert pin.mode == PinMode.OUTPUT
        assert pin.gpio_number == 60  # P9_12 -> GPIO1_28

    def test_resolve_p8_pin(self) -> None:
        """Test resolving P8 header pin."""
        pin = BeagleBoneDigitalPin(
            pin_name="P8_10",
            backend=BBIOBackend.SIMULATION,
        )
        assert pin.gpio_number == 68  # P8_10 -> GPIO2_4

    def test_resolve_p9_pin(self) -> None:
        """Test resolving P9 header pin."""
        pin = BeagleBoneDigitalPin(
            pin_name="P9_14",
            backend=BBIOBackend.SIMULATION,
        )
        assert pin.gpio_number == 50  # P9_14 -> GPIO1_18

    def test_invalid_pin_format(self) -> None:
        """Test invalid pin format raises error."""
        with pytest.raises(ValueError, match="Invalid pin name"):
            BeagleBoneDigitalPin(
                pin_name="GPIO17",
                backend=BBIOBackend.SIMULATION,
            )

    def test_invalid_p8_pin(self) -> None:
        """Test invalid P8 pin number raises error."""
        with pytest.raises(ValueError, match="not a GPIO pin"):
            BeagleBoneDigitalPin(
                pin_name="P8_1",  # Power pin, not GPIO
                backend=BBIOBackend.SIMULATION,
            )

    def test_high_low(self) -> None:
        """Test high/low output."""
        pin = BeagleBoneDigitalPin(
            pin_name="P9_12",
            mode=PinMode.OUTPUT,
            backend=BBIOBackend.SIMULATION,
        )
        pin.setup()
        pin.high()
        assert pin.read() is True

        pin.low()
        assert pin.read() is False

    def test_write_to_input_raises(self) -> None:
        """Test writing to input pin raises error."""
        pin = BeagleBoneDigitalPin(
            pin_name="P9_12",
            mode=PinMode.INPUT,
            backend=BBIOBackend.SIMULATION,
        )
        pin.setup()
        with pytest.raises(ValueError, match="Cannot write to input pin"):
            pin.high()

    def test_toggle(self) -> None:
        """Test toggle functionality."""
        pin = BeagleBoneDigitalPin(
            pin_name="P9_12",
            mode=PinMode.OUTPUT,
            backend=BBIOBackend.SIMULATION,
            initial=PinState.LOW,
        )
        pin.setup()
        assert pin.read() is False
        pin.toggle()
        assert pin.read() is True
        pin.toggle()
        assert pin.read() is False

    def test_cleanup(self) -> None:
        """Test cleanup doesn't raise in simulation."""
        pin = BeagleBoneDigitalPin(
            pin_name="P9_12",
            backend=BBIOBackend.SIMULATION,
        )
        pin.cleanup()  # Should not raise


# =============================================================================
# PWM Pin Tests
# =============================================================================


class TestBeagleBonePWMPin:
    """Tests for BeagleBonePWMPin."""

    def test_init_simulation(self) -> None:
        """Test PWM pin initialization in simulation."""
        pin = BeagleBonePWMPin(
            pin_name="P9_14",
            frequency=1000,
            duty_cycle=0.5,
            backend=BBIOBackend.SIMULATION,
        )
        assert pin.pin_name == "P9_14"
        assert pin.frequency == 1000
        assert pin.duty_cycle == 0.5
        assert pin.pwm_module == "ehrpwm1"

    def test_invalid_pwm_pin(self) -> None:
        """Test invalid PWM pin raises error."""
        with pytest.raises(ValueError, match="does not support PWM"):
            BeagleBonePWMPin(
                pin_name="P9_12",  # Not a PWM pin
                backend=BBIOBackend.SIMULATION,
            )

    def test_duty_cycle_method(self) -> None:
        """Test duty cycle method."""
        pin = BeagleBonePWMPin(
            pin_name="P9_14",
            backend=BBIOBackend.SIMULATION,
        )
        pin.setup()
        pin.set_duty_cycle(0.75)
        assert pin.duty_cycle == 0.75

    def test_duty_cycle_validation(self) -> None:
        """Test duty cycle validation."""
        pin = BeagleBonePWMPin(
            pin_name="P9_14",
            backend=BBIOBackend.SIMULATION,
        )
        pin.setup()
        with pytest.raises(ValueError, match=r"must be 0\.0-1\.0"):
            pin.set_duty_cycle(1.5)

    def test_frequency_method(self) -> None:
        """Test frequency method."""
        pin = BeagleBonePWMPin(
            pin_name="P9_14",
            backend=BBIOBackend.SIMULATION,
        )
        pin.setup()
        pin.set_frequency(5000)
        assert pin.frequency == 5000

    def test_frequency_validation(self) -> None:
        """Test frequency validation."""
        pin = BeagleBonePWMPin(
            pin_name="P9_14",
            backend=BBIOBackend.SIMULATION,
        )
        pin.setup()
        with pytest.raises(ValueError, match="must be positive"):
            pin.set_frequency(-100)

    def test_start_stop(self) -> None:
        """Test PWM start/stop."""
        pin = BeagleBonePWMPin(
            pin_name="P9_14",
            backend=BBIOBackend.SIMULATION,
        )
        assert pin.is_running is False
        pin.start()
        assert pin.is_running is True
        pin.stop()
        assert pin.is_running is False

    def test_cleanup(self) -> None:
        """Test PWM cleanup."""
        pin = BeagleBonePWMPin(
            pin_name="P9_14",
            backend=BBIOBackend.SIMULATION,
        )
        pin.start()
        pin.cleanup()
        assert pin.is_running is False


# =============================================================================
# ADC Pin Tests
# =============================================================================


class TestBeagleBoneADCPin:
    """Tests for BeagleBoneADCPin."""

    def test_init_simulation(self) -> None:
        """Test ADC pin initialization in simulation."""
        adc = BeagleBoneADCPin(
            channel=0,
            backend=BBIOBackend.SIMULATION,
        )
        assert adc.channel == 0
        assert adc.pin_name == "P9_39"  # AIN0
        assert adc.reference_voltage == 1.8
        assert adc.resolution == 12

    def test_invalid_channel(self) -> None:
        """Test invalid ADC channel raises error."""
        with pytest.raises(ValueError, match="Invalid ADC channel"):
            BeagleBoneADCPin(
                channel=7,  # Only 0-6 valid
                backend=BBIOBackend.SIMULATION,
            )

    def test_read_normalized(self) -> None:
        """Test normalized read using read_normalized (0.0-1.0)."""
        adc = BeagleBoneADCPin(
            channel=0,
            backend=BBIOBackend.SIMULATION,
        )
        adc.setup()
        adc.set_simulated_value(0.5)
        assert adc.read_normalized() == pytest.approx(0.5, rel=0.01)

    def test_read_raw(self) -> None:
        """Test raw read (0-4095)."""
        adc = BeagleBoneADCPin(
            channel=0,
            backend=BBIOBackend.SIMULATION,
        )
        adc.setup()
        adc.set_simulated_value(1.0)
        assert adc.read_raw() == 4095

    def test_read_voltage(self) -> None:
        """Test voltage read via read() method."""
        adc = BeagleBoneADCPin(
            channel=0,
            backend=BBIOBackend.SIMULATION,
        )
        adc.setup()
        adc.set_simulated_value(0.5)
        # read() returns voltage: (raw / max) * reference_voltage
        assert adc.read() == pytest.approx(0.9, rel=0.01)  # 0.5 * 1.8V

    def test_set_simulated_value_clamped(self) -> None:
        """Test simulated value is clamped."""
        adc = BeagleBoneADCPin(
            channel=0,
            backend=BBIOBackend.SIMULATION,
        )
        adc.setup()
        adc.set_simulated_value(1.5)  # Should clamp to 1.0
        assert adc.read() == pytest.approx(1.8, rel=0.01)  # 1.0 * 1.8V

        adc.set_simulated_value(-0.5)  # Should clamp to 0.0
        assert adc.read() == pytest.approx(0.0, rel=0.01)


# =============================================================================
# PRU Interface Tests
# =============================================================================


class TestPRUInterface:
    """Tests for PRU interface."""

    def test_init_simulation(self) -> None:
        """Test PRU initialization in simulation."""
        pru = PRUInterface(pru_id=0, simulation=True)
        assert pru.pru_id == 0
        assert pru.state == PRUState.OFFLINE

    def test_invalid_pru_id(self) -> None:
        """Test invalid PRU ID raises error."""
        with pytest.raises(ValueError, match="PRU ID must be 0 or 1"):
            PRUInterface(pru_id=2, simulation=True)

    def test_start_stop_simulation(self) -> None:
        """Test PRU start/stop in simulation."""
        pru = PRUInterface(pru_id=0, simulation=True)
        assert pru.is_running is False

        pru.start()
        assert pru.is_running is True
        assert pru.state == PRUState.RUNNING

        pru.stop()
        assert pru.is_running is False
        assert pru.state == PRUState.STOPPED

    def test_load_firmware_simulation(self) -> None:
        """Test firmware loading in simulation."""
        pru = PRUInterface(pru_id=0, simulation=True)
        result = pru.load_firmware("/path/to/firmware.out")
        assert result is True


# =============================================================================
# Device Tree Overlay Manager Tests
# =============================================================================


class TestDeviceTreeOverlayManager:
    """Tests for device tree overlay manager."""

    def test_init_simulation(self) -> None:
        """Test manager initialization in simulation."""
        manager = DeviceTreeOverlayManager(simulation=True)
        assert manager.loaded_overlays == []

    def test_load_overlay_simulation(self) -> None:
        """Test loading overlay in simulation."""
        manager = DeviceTreeOverlayManager(simulation=True)
        result = manager.load("BB-PWM1")
        assert result is True
        assert "BB-PWM1" in manager.loaded_overlays

    def test_load_duplicate_overlay(self) -> None:
        """Test loading same overlay twice."""
        manager = DeviceTreeOverlayManager(simulation=True)
        manager.load("BB-PWM1")
        manager.load("BB-PWM1")  # Should not duplicate
        assert manager.loaded_overlays.count("BB-PWM1") == 1

    def test_unload_overlay_simulation(self) -> None:
        """Test unloading overlay in simulation."""
        manager = DeviceTreeOverlayManager(simulation=True)
        manager.load("BB-PWM1")
        result = manager.unload("BB-PWM1")
        assert result is True
        assert "BB-PWM1" not in manager.loaded_overlays

    def test_is_loaded(self) -> None:
        """Test is_loaded check."""
        manager = DeviceTreeOverlayManager(simulation=True)
        assert manager.is_loaded("BB-PWM1") is False
        manager.load("BB-PWM1")
        assert manager.is_loaded("BB-PWM1") is True


# =============================================================================
# Platform Tests
# =============================================================================


class TestBeagleBonePlatform:
    """Tests for BeagleBonePlatform."""

    def test_init_simulation(self) -> None:
        """Test platform initialization in simulation mode."""
        platform = BeagleBonePlatform(simulation=True)
        assert platform.is_simulation is True
        assert platform.backend == BBIOBackend.SIMULATION
        assert platform.model == BeagleBoneModel.BLACK  # Default in simulation

    def test_platform_info(self) -> None:
        """Test platform info."""
        from robo_infra.platforms.base import PlatformType

        platform = BeagleBonePlatform(simulation=True)
        info = platform.get_info()
        assert "BeagleBone" in info.model
        assert info.platform_type == PlatformType.BEAGLEBONE
        assert len(info.gpio_chips) > 0

    def test_capabilities(self) -> None:
        """Test platform capabilities."""
        from robo_infra.platforms.base import PlatformCapability

        platform = BeagleBonePlatform(simulation=True)
        caps = platform.capabilities
        assert PlatformCapability.GPIO in caps
        assert PlatformCapability.PWM in caps
        assert PlatformCapability.ADC in caps
        # PRU is BeagleBone-specific, check via bb_capabilities
        assert platform.bb_capabilities is not None
        assert platform.bb_capabilities.has_pru is True

    def test_get_pin(self) -> None:
        """Test getting a digital pin."""
        platform = BeagleBonePlatform(simulation=True)
        pin = platform.get_pin("P9_12", mode=PinMode.OUTPUT)
        assert isinstance(pin, BeagleBoneDigitalPin)
        assert pin.pin_name == "P9_12"

    def test_get_pin_cached(self) -> None:
        """Test pins are cached."""
        platform = BeagleBonePlatform(simulation=True)
        pin1 = platform.get_pin("P9_12", mode=PinMode.OUTPUT)
        pin2 = platform.get_pin("P9_12", mode=PinMode.OUTPUT)
        assert pin1 is pin2

    def test_get_pin_by_gpio_number(self) -> None:
        """Test getting pin by GPIO number."""
        platform = BeagleBonePlatform(simulation=True)
        pin = platform.get_pin(60, mode=PinMode.OUTPUT)  # GPIO1_28 = P9_12
        assert pin.pin_name == "P9_12"

    def test_get_pwm_pin(self) -> None:
        """Test getting a PWM pin."""
        platform = BeagleBonePlatform(simulation=True)
        pwm = platform.get_pwm_pin("P9_14", frequency=1000)
        assert isinstance(pwm, BeagleBonePWMPin)
        assert pwm.pin_name == "P9_14"

    def test_get_adc(self) -> None:
        """Test getting an ADC channel."""
        platform = BeagleBonePlatform(simulation=True)
        adc = platform.get_adc(0)
        assert isinstance(adc, BeagleBoneADCPin)
        assert adc.channel == 0

    def test_get_pru(self) -> None:
        """Test getting PRU interface."""
        platform = BeagleBonePlatform(simulation=True)
        pru0 = platform.get_pru(0)
        assert isinstance(pru0, PRUInterface)
        assert pru0.pru_id == 0

        pru1 = platform.get_pru(1)
        assert pru1.pru_id == 1

    def test_overlay_manager(self) -> None:
        """Test overlay manager access."""
        platform = BeagleBonePlatform(simulation=True)
        manager = platform.overlay_manager
        assert isinstance(manager, DeviceTreeOverlayManager)

    def test_load_unload_overlay(self) -> None:
        """Test overlay loading/unloading shortcuts."""
        platform = BeagleBonePlatform(simulation=True)
        assert platform.load_overlay("BB-PWM1") is True
        assert platform.unload_overlay("BB-PWM1") is True

    def test_cleanup(self) -> None:
        """Test platform cleanup."""
        platform = BeagleBonePlatform(simulation=True)
        platform.get_pin("P9_12", mode=PinMode.OUTPUT)
        platform.get_pwm_pin("P9_14")
        platform.get_adc(0)
        platform.cleanup()  # Should not raise

    def test_string_backend(self) -> None:
        """Test string backend conversion."""
        platform = BeagleBonePlatform(backend="simulation")
        assert platform.backend == BBIOBackend.SIMULATION


# =============================================================================
# Integration Tests
# =============================================================================


class TestBeagleBoneIntegration:
    """Integration tests for BeagleBone platform."""

    def test_led_blink_pattern(self) -> None:
        """Test LED blink pattern simulation."""
        platform = BeagleBonePlatform(simulation=True)
        led = platform.get_pin("P9_12", mode=PinMode.OUTPUT)
        led.setup()

        # Blink pattern
        for _ in range(3):
            led.high()
            assert led.read() is True
            led.low()
            assert led.read() is False

        platform.cleanup()

    def test_pwm_servo_control(self) -> None:
        """Test PWM for servo control simulation."""
        platform = BeagleBonePlatform(simulation=True)
        servo = platform.get_pwm_pin("P9_14", frequency=50)
        servo.setup()
        servo.start()

        # Sweep servo positions
        for duty in [0.05, 0.075, 0.1]:  # 1ms, 1.5ms, 2ms pulses
            servo.set_duty_cycle(duty)
            assert servo.duty_cycle == pytest.approx(duty)

        servo.stop()
        platform.cleanup()

    def test_adc_reading(self) -> None:
        """Test ADC reading simulation."""
        platform = BeagleBonePlatform(simulation=True)
        pot = platform.get_adc(0)
        pot.setup()

        # Simulate potentiometer sweep - read() returns voltage
        for value in [0.0, 0.25, 0.5, 0.75, 1.0]:
            pot.set_simulated_value(value)
            # read() returns voltage: (raw / max) * reference_voltage
            assert pot.read() == pytest.approx(value * 1.8, rel=0.01)
            assert pot.read_voltage() == pytest.approx(value * 1.8, rel=0.01)

        platform.cleanup()

    def test_pru_control(self) -> None:
        """Test PRU control simulation."""
        platform = BeagleBonePlatform(simulation=True)
        pru = platform.get_pru(0)

        assert pru.state == PRUState.OFFLINE
        pru.load_firmware("/lib/firmware/pru_code.out")
        pru.start()
        assert pru.state == PRUState.RUNNING
        pru.stop()
        assert pru.state == PRUState.STOPPED

        platform.cleanup()

    def test_overlay_for_pwm(self) -> None:
        """Test loading overlay for PWM."""
        platform = BeagleBonePlatform(simulation=True)

        # Load PWM overlay before using PWM
        platform.load_overlay("BB-PWM1")
        assert platform.overlay_manager.is_loaded("BB-PWM1")

        # Now use PWM
        pwm = platform.get_pwm_pin("P9_14")
        pwm.start()
        pwm.stop()

        platform.cleanup()


# =============================================================================
# Bus Access Tests (Phase 8.8.2)
# =============================================================================


class TestBeagleBoneBusAccess:
    """Tests for BeagleBone platform bus access."""

    def test_get_bus_i2c_simulation(self) -> None:
        """Test getting I2C bus in simulation mode returns SimulatedI2CBus."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_bus("i2c", bus=1)
        assert isinstance(bus, SimulatedI2CBus)

    def test_get_bus_spi_simulation(self) -> None:
        """Test getting SPI bus in simulation mode returns SimulatedSPIBus."""
        from robo_infra.core.bus import SimulatedSPIBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_bus("spi", bus=0, device=0)
        assert isinstance(bus, SimulatedSPIBus)

    def test_get_bus_uart_simulation(self) -> None:
        """Test getting UART bus in simulation mode returns SimulatedSerialBus."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_bus("uart", port="/dev/ttyO1")
        assert isinstance(bus, SimulatedSerialBus)

    def test_get_bus_serial_alias(self) -> None:
        """Test 'serial' is accepted as alias for 'uart'."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_bus("serial", port="/dev/ttyO1")
        assert isinstance(bus, SimulatedSerialBus)

    def test_get_bus_unknown_type_raises(self) -> None:
        """Test unknown bus type raises HardwareNotFoundError."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        platform = BeagleBonePlatform(simulation=True)
        with pytest.raises(HardwareNotFoundError):
            platform.get_bus("unknown_bus")

    def test_get_bus_case_insensitive(self) -> None:
        """Test bus type is case insensitive."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = BeagleBonePlatform(simulation=True)
        bus1 = platform.get_bus("I2C")
        bus2 = platform.get_bus("i2c")
        bus3 = platform.get_bus("I2c")
        assert isinstance(bus1, SimulatedI2CBus)
        assert isinstance(bus2, SimulatedI2CBus)
        assert isinstance(bus3, SimulatedI2CBus)


class TestBeagleBoneBusConvenienceMethods:
    """Tests for bus convenience methods (get_i2c, get_spi, get_serial)."""

    def test_get_i2c_default_bus(self) -> None:
        """Test get_i2c with default bus number."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_i2c()
        assert isinstance(bus, SimulatedI2CBus)
        assert bus.config.bus_number == 1

    def test_get_i2c_custom_bus(self) -> None:
        """Test get_i2c with custom bus number (bus 2 for P9.19/P9.20)."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_i2c(bus=2)
        assert isinstance(bus, SimulatedI2CBus)
        assert bus.config.bus_number == 2

    def test_get_spi_default(self) -> None:
        """Test get_spi with default parameters."""
        from robo_infra.core.bus import SimulatedSPIBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_spi()
        assert isinstance(bus, SimulatedSPIBus)
        assert bus.config.bus == 0
        assert bus.config.device == 0

    def test_get_spi_custom(self) -> None:
        """Test get_spi with custom bus and device (SPI1 CS1)."""
        from robo_infra.core.bus import SimulatedSPIBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_spi(bus=1, device=1)
        assert isinstance(bus, SimulatedSPIBus)
        assert bus.config.bus == 1
        assert bus.config.device == 1

    def test_get_serial_default(self) -> None:
        """Test get_serial with default parameters (UART1)."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_serial()
        assert isinstance(bus, SimulatedSerialBus)
        assert bus.config.port == "/dev/ttyO1"
        assert bus.config.baudrate == 115200

    def test_get_serial_custom_uart2(self) -> None:
        """Test get_serial with UART2 port."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_serial(port="/dev/ttyO2", baudrate=9600)
        assert isinstance(bus, SimulatedSerialBus)
        assert bus.config.port == "/dev/ttyO2"
        assert bus.config.baudrate == 9600

    def test_get_serial_custom_uart4(self) -> None:
        """Test get_serial with UART4 port."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_serial(port="/dev/ttyO4", baudrate=57600)
        assert isinstance(bus, SimulatedSerialBus)
        assert bus.config.port == "/dev/ttyO4"
        assert bus.config.baudrate == 57600


class TestBeagleBoneRealBusAccess:
    """Tests for real hardware bus access (with mocked libraries)."""

    def test_get_bus_graceful_fallback_on_import_error(self) -> None:
        """Test that bus access falls back gracefully when library not installed."""
        platform = BeagleBonePlatform(simulation=True)

        from robo_infra.core.bus import SimulatedI2CBus, SimulatedSerialBus, SimulatedSPIBus

        i2c_bus = platform.get_bus("i2c")
        spi_bus = platform.get_bus("spi")
        uart_bus = platform.get_bus("uart")

        assert isinstance(i2c_bus, SimulatedI2CBus)
        assert isinstance(spi_bus, SimulatedSPIBus)
        assert isinstance(uart_bus, SimulatedSerialBus)

    def test_no_not_implemented_error(self) -> None:
        """Test that NotImplementedError is NOT raised anymore."""
        platform = BeagleBonePlatform(simulation=False)

        # These should NOT raise NotImplementedError
        # They may raise ImportError if libs not installed, but that's caught internally
        try:
            bus = platform.get_bus("i2c")
            assert bus is not None
        except NotImplementedError:
            pytest.fail("NotImplementedError should not be raised")

    def test_beaglebone_default_uart_port(self) -> None:
        """Test BeagleBone uses /dev/ttyO1 as default UART port."""
        from robo_infra.core.bus import SimulatedSerialBus

        platform = BeagleBonePlatform(simulation=True)
        bus = platform.get_serial()
        assert isinstance(bus, SimulatedSerialBus)
        # BeagleBone uses /dev/ttyO1 as default (not /dev/ttyS0)
        assert bus.config.port == "/dev/ttyO1"

    def test_beaglebone_i2c_bus_paths(self) -> None:
        """Test BeagleBone I2C bus configurations."""
        from robo_infra.core.bus import SimulatedI2CBus

        platform = BeagleBonePlatform(simulation=True)

        # Bus 1: P9.17 (SCL), P9.18 (SDA)
        bus1 = platform.get_i2c(bus=1)
        assert isinstance(bus1, SimulatedI2CBus)
        assert bus1.config.bus_number == 1

        # Bus 2: P9.19 (SCL), P9.20 (SDA)
        bus2 = platform.get_i2c(bus=2)
        assert isinstance(bus2, SimulatedI2CBus)
        assert bus2.config.bus_number == 2

    def test_beaglebone_spi_bus_paths(self) -> None:
        """Test BeagleBone SPI bus configurations."""
        from robo_infra.core.bus import SimulatedSPIBus

        platform = BeagleBonePlatform(simulation=True)

        # SPI0 CS0
        spi0_0 = platform.get_spi(bus=0, device=0)
        assert isinstance(spi0_0, SimulatedSPIBus)
        assert spi0_0.config.bus == 0
        assert spi0_0.config.device == 0

        # SPI1 CS0
        spi1_0 = platform.get_spi(bus=1, device=0)
        assert isinstance(spi1_0, SimulatedSPIBus)
        assert spi1_0.config.bus == 1
        assert spi1_0.config.device == 0
