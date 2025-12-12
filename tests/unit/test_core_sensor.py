"""Tests for robo_infra.core.sensor module."""

from __future__ import annotations

import time

import pytest

from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import DisabledError, NotCalibratedError
from robo_infra.core.sensor import (
    FilterConfig,
    ReadingBuffer,
    SensorConfig,
    SensorGroup,
    SensorState,
    SensorStatus,
    SensorType,
    SimulatedSensor,
    create_distance_sensor,
    create_encoder,
    create_limit_switch,
    create_sensor,
    create_temperature_sensor,
)
from robo_infra.core.types import Limits, Reading, Unit


# =============================================================================
# Test Enums
# =============================================================================


class TestSensorState:
    """Tests for SensorState enum."""

    def test_all_states_defined(self) -> None:
        """All expected states should be defined."""
        assert SensorState.IDLE.value == "idle"
        assert SensorState.READING.value == "reading"
        assert SensorState.STREAMING.value == "streaming"
        assert SensorState.ERROR.value == "error"
        assert SensorState.DISABLED.value == "disabled"

    def test_state_count(self) -> None:
        """Should have expected number of states."""
        assert len(SensorState) == 5


class TestSensorType:
    """Tests for SensorType enum."""

    def test_position_sensors(self) -> None:
        """Position/motion sensor types should be defined."""
        assert SensorType.ENCODER.value == "encoder"
        assert SensorType.POTENTIOMETER.value == "potentiometer"
        assert SensorType.IMU.value == "imu"
        assert SensorType.ACCELEROMETER.value == "accelerometer"
        assert SensorType.GYROSCOPE.value == "gyroscope"

    def test_distance_sensors(self) -> None:
        """Distance sensor types should be defined."""
        assert SensorType.ULTRASONIC.value == "ultrasonic"
        assert SensorType.LIDAR.value == "lidar"
        assert SensorType.INFRARED.value == "infrared"
        assert SensorType.TOF.value == "tof"

    def test_contact_sensors(self) -> None:
        """Contact sensor types should be defined."""
        assert SensorType.LIMIT_SWITCH.value == "limit_switch"
        assert SensorType.BUMPER.value == "bumper"
        assert SensorType.PRESSURE.value == "pressure"
        assert SensorType.FORCE.value == "force"

    def test_environment_sensors(self) -> None:
        """Environment sensor types should be defined."""
        assert SensorType.TEMPERATURE.value == "temperature"
        assert SensorType.HUMIDITY.value == "humidity"
        assert SensorType.LIGHT.value == "light"

    def test_generic_types(self) -> None:
        """Generic types should be defined."""
        assert SensorType.ANALOG.value == "analog"
        assert SensorType.DIGITAL.value == "digital"
        assert SensorType.CUSTOM.value == "custom"


# =============================================================================
# Test SensorConfig
# =============================================================================


class TestSensorConfig:
    """Tests for SensorConfig Pydantic model."""

    def test_minimal_config(self) -> None:
        """Should create config with just name."""
        config = SensorConfig(name="test_sensor")
        assert config.name == "test_sensor"
        assert config.sensor_type == SensorType.CUSTOM
        assert config.unit == Unit.RAW
        assert config.channel == 0
        assert config.scale == 1.0
        assert config.offset == 0.0

    def test_full_config(self) -> None:
        """Should create config with all options."""
        config = SensorConfig(
            name="encoder",
            sensor_type=SensorType.ENCODER,
            unit=Unit.DEGREES,
            limits=Limits(min=0, max=360, default=0),
            channel=3,
            sample_rate=200.0,
            scale=0.088,
            offset=-180.0,
            inverted=True,
            filter_window=5,
            ema_alpha=0.3,
            requires_calibration=True,
            description="Wheel encoder",
            metadata={"resolution": 4096},
        )
        assert config.name == "encoder"
        assert config.sensor_type == SensorType.ENCODER
        assert config.unit == Unit.DEGREES
        assert config.limits.min == 0
        assert config.limits.max == 360
        assert config.channel == 3
        assert config.sample_rate == 200.0
        assert config.scale == 0.088
        assert config.offset == -180.0
        assert config.inverted is True
        assert config.filter_window == 5
        assert config.ema_alpha == 0.3
        assert config.requires_calibration is True
        assert config.description == "Wheel encoder"
        assert config.metadata["resolution"] == 4096

    def test_from_dict(self) -> None:
        """Should create config from dictionary."""
        data = {
            "name": "temp_sensor",
            "sensor_type": "temperature",
            "unit": "°C",
            "limits": {"min": -40, "max": 125, "default": 25},
            "channel": 1,
        }
        config = SensorConfig.from_dict(data)
        assert config.name == "temp_sensor"
        assert config.sensor_type == SensorType.TEMPERATURE
        assert config.unit == Unit.CELSIUS
        assert config.limits.min == -40
        assert config.limits.max == 125

    def test_validation_channel_negative(self) -> None:
        """Should reject negative channel."""
        with pytest.raises(ValueError):
            SensorConfig(name="test", channel=-1)

    def test_validation_sample_rate_zero(self) -> None:
        """Should reject zero sample rate."""
        with pytest.raises(ValueError):
            SensorConfig(name="test", sample_rate=0)


# =============================================================================
# Test FilterConfig
# =============================================================================


class TestFilterConfig:
    """Tests for FilterConfig."""

    def test_defaults(self) -> None:
        """Should have sensible defaults."""
        config = FilterConfig()
        assert config.window_size == 1
        assert config.ema_alpha == 0.0
        assert config.median_window == 0
        assert config.lowpass_cutoff == 0.0
        assert config.deadband == 0.0

    def test_custom_values(self) -> None:
        """Should accept custom values."""
        config = FilterConfig(
            window_size=10,
            ema_alpha=0.5,
            median_window=5,
            deadband=0.1,
        )
        assert config.window_size == 10
        assert config.ema_alpha == 0.5
        assert config.median_window == 5
        assert config.deadband == 0.1


# =============================================================================
# Test SimulatedSensor
# =============================================================================


class TestSimulatedSensor:
    """Tests for SimulatedSensor."""

    def test_create_basic(self) -> None:
        """Should create with minimal args."""
        sensor = SimulatedSensor(name="test")
        assert sensor.name == "test"
        assert sensor.unit == Unit.RAW
        assert sensor.is_enabled is False
        assert sensor.is_calibrated is True
        assert sensor.state == SensorState.DISABLED

    def test_create_with_limits(self) -> None:
        """Should create with limits."""
        sensor = SimulatedSensor(
            name="encoder",
            unit=Unit.DEGREES,
            limits=Limits(min=0, max=360, default=180),
        )
        assert sensor.limits.min == 0
        assert sensor.limits.max == 360

    def test_enable_disable(self) -> None:
        """Should enable and disable."""
        sensor = SimulatedSensor(name="test")
        assert sensor.is_enabled is False

        sensor.enable()
        assert sensor.is_enabled is True
        assert sensor.state == SensorState.IDLE

        sensor.disable()
        assert sensor.is_enabled is False
        assert sensor.state == SensorState.DISABLED

    def test_read_when_disabled(self) -> None:
        """Should raise DisabledError when reading disabled sensor."""
        sensor = SimulatedSensor(name="test")
        with pytest.raises(DisabledError):
            sensor.read()

    def test_read_returns_reading(self) -> None:
        """Should return Reading object."""
        sensor = SimulatedSensor(
            name="test",
            unit=Unit.CELSIUS,
            limits=Limits(min=0, max=100, default=25),
        )
        sensor.enable()
        reading = sensor.read()

        assert isinstance(reading, Reading)
        assert reading.unit == Unit.CELSIUS
        assert reading.timestamp > 0
        assert reading.raw is not None

    def test_read_default_value(self) -> None:
        """Should read default value initially."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100, default=50),
        )
        sensor.enable()
        reading = sensor.read()
        assert reading.value == 50.0

    def test_set_simulated_value(self) -> None:
        """Should allow setting simulated value."""
        sensor = SimulatedSensor(name="test", limits=Limits(min=0, max=100))
        sensor.enable()

        sensor.set_simulated_value(75)
        reading = sensor.read()
        assert reading.value == 75.0

    def test_read_raw(self) -> None:
        """Should read raw value."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100, default=42),
        )
        sensor.enable()
        raw = sensor.read_raw()
        assert raw == 42

    def test_scale_transform(self) -> None:
        """Should apply scale transform."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=1000),
            scale=2.0,
            initial_value=100,
        )
        sensor.enable()
        reading = sensor.read()
        assert reading.value == 200.0

    def test_offset_transform(self) -> None:
        """Should apply offset transform."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=-100, max=100),
            offset=-50.0,
            initial_value=75,
        )
        sensor.enable()
        reading = sensor.read()
        assert reading.value == 25.0

    def test_inverted_transform(self) -> None:
        """Should apply inversion."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100),
            inverted=True,
            initial_value=25,
        )
        sensor.enable()
        reading = sensor.read()
        # Midpoint is 50, so 25 inverts to 75
        assert reading.value == 75.0

    def test_clamp_to_limits(self) -> None:
        """Should clamp to limits."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100),
            scale=10.0,  # Will multiply 50 to 500
            initial_value=50,
        )
        sensor.enable()
        reading = sensor.read()
        assert reading.value == 100.0  # Clamped to max

    def test_noise_simulation(self) -> None:
        """Should add noise when configured."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100, default=50),
            noise=10.0,
        )
        sensor.enable()

        # Take multiple readings and check variance
        readings = [sensor.read().value for _ in range(20)]
        assert min(readings) != max(readings)  # Should have variance
        assert all(40 <= r <= 60 for r in readings)  # Within noise bounds

    def test_drift_simulation(self) -> None:
        """Should accumulate drift."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=200, default=100),
            drift=1.0,
        )
        sensor.enable()

        first = sensor.read().value
        second = sensor.read().value
        third = sensor.read().value

        assert second > first
        assert third > second

    def test_reset_drift(self) -> None:
        """Should reset drift accumulator."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=200, default=100),
            drift=5.0,
        )
        sensor.enable()

        sensor.read()
        sensor.read()
        sensor.reset_drift()

        # Should be back to base value
        sensor.set_simulated_value(100)
        reading = sensor.read()
        # First read after reset should have drift=5 added
        assert abs(reading.value - 105.0) < 0.1

    def test_from_config(self) -> None:
        """Should create from SensorConfig."""
        config = SensorConfig(
            name="configured_sensor",
            sensor_type=SensorType.ENCODER,
            unit=Unit.DEGREES,
            limits=Limits(min=0, max=360),
            channel=2,
            scale=0.5,
            offset=10.0,
            inverted=True,
        )
        sensor = SimulatedSensor.from_config(config)

        assert sensor.name == "configured_sensor"
        assert sensor.unit == Unit.DEGREES
        assert sensor.channel == 2
        assert sensor._scale == 0.5
        assert sensor._offset == 10.0
        assert sensor._inverted is True

    def test_with_driver(self) -> None:
        """Should read from driver when connected."""
        from robo_infra.core.driver import ChannelConfig

        driver = SimulatedDriver(channels=8)
        # Configure channel 3 with larger range for ADC-like values
        driver.set_channel_config(3, ChannelConfig(min_value=0, max_value=4095))
        driver.connect()
        driver.enable()
        driver.set_channel(3, 2048)  # Mid-range ADC value

        sensor = SimulatedSensor(
            name="test",
            driver=driver,
            channel=3,
            limits=Limits(min=0, max=4095),
        )
        sensor.enable()
        reading = sensor.read()
        assert reading.value == 2048.0

    def test_context_manager(self) -> None:
        """Should work as context manager."""
        sensor = SimulatedSensor(name="test")
        assert sensor.is_enabled is False

        with sensor:
            assert sensor.is_enabled is True

        assert sensor.is_enabled is False


# =============================================================================
# Test Calibration
# =============================================================================


class TestSensorCalibration:
    """Tests for sensor calibration."""

    def test_requires_calibration_flag(self) -> None:
        """Should respect requires_calibration flag."""
        sensor = SimulatedSensor(name="test", requires_calibration=True)
        assert sensor.is_calibrated is False

        sensor.enable()
        with pytest.raises(NotCalibratedError):
            sensor.read()

    def test_calibrate_simple(self) -> None:
        """Should calibrate with zero point."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=-100, max=100),
            requires_calibration=False,  # Don't require, so we can test the calibration function
            initial_value=10,
        )
        sensor.enable()

        # Calibrate: make current value (10) equal to 0
        sensor.calibrate(zero_point=0)

        assert sensor.is_calibrated is True
        reading = sensor.read()
        assert abs(reading.value - 0.0) < 0.1

    def test_reset_calibration(self) -> None:
        """Should reset calibration."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=-100, max=100),
            initial_value=50,
        )
        sensor.enable()
        sensor.calibrate(zero_point=0)

        sensor.reset_calibration()
        reading = sensor.read()
        assert reading.value == 50.0

    def test_calibrate_when_disabled(self) -> None:
        """Should raise DisabledError."""
        sensor = SimulatedSensor(name="test")
        with pytest.raises(DisabledError):
            sensor.calibrate()


# =============================================================================
# Test Filtering
# =============================================================================


class TestSensorFiltering:
    """Tests for sensor filtering."""

    def test_moving_average(self) -> None:
        """Should apply moving average filter."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100),
            filter_config=FilterConfig(window_size=3),
        )
        sensor.enable()

        # Set values and read
        sensor.set_simulated_value(10)
        r1 = sensor.read().value  # Buffer: [10], avg = 10

        sensor.set_simulated_value(20)
        r2 = sensor.read().value  # Buffer: [10, 20], avg = 15

        sensor.set_simulated_value(30)
        r3 = sensor.read().value  # Buffer: [10, 20, 30], avg = 20

        assert r1 == 10.0
        assert r2 == 15.0
        assert r3 == 20.0

    def test_ema_filter(self) -> None:
        """Should apply EMA filter."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100),
            filter_config=FilterConfig(ema_alpha=0.5),
        )
        sensor.enable()

        sensor.set_simulated_value(100)
        r1 = sensor.read().value  # First value, EMA = 100

        sensor.set_simulated_value(0)
        r2 = sensor.read().value  # EMA = 0.5*0 + 0.5*100 = 50

        assert r1 == 100.0
        assert r2 == 50.0

    def test_deadband_filter(self) -> None:
        """Should ignore small changes with deadband."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100),
            filter_config=FilterConfig(deadband=5.0),
        )
        sensor.enable()

        sensor.set_simulated_value(50)
        r1 = sensor.read().value

        sensor.set_simulated_value(52)  # Change < deadband
        r2 = sensor.read().value

        sensor.set_simulated_value(60)  # Change > deadband
        r3 = sensor.read().value

        assert r1 == 50.0
        assert r2 == 50.0  # Should not change
        assert r3 == 60.0

    def test_reset_filters(self) -> None:
        """Should reset filter state."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100),
            filter_config=FilterConfig(window_size=5),
        )
        sensor.enable()

        # Fill buffer
        for v in [10, 20, 30, 40, 50]:
            sensor.set_simulated_value(v)
            sensor.read()

        sensor.reset_filters()

        sensor.set_simulated_value(100)
        reading = sensor.read()
        assert reading.value == 100.0  # No averaging with empty buffer


# =============================================================================
# Test SensorGroup
# =============================================================================


class TestSensorGroup:
    """Tests for SensorGroup."""

    def test_create_group(self) -> None:
        """Should create empty group."""
        group = SensorGroup("arm_sensors")
        assert group.name == "arm_sensors"
        assert len(group) == 0

    def test_add_remove_sensors(self) -> None:
        """Should add and remove sensors."""
        group = SensorGroup("test")
        sensor = SimulatedSensor(name="s1")

        group.add("sensor1", sensor)
        assert len(group) == 1
        assert "sensor1" in group

        removed = group.remove("sensor1")
        assert removed is sensor
        assert len(group) == 0

    def test_get_sensor(self) -> None:
        """Should get sensor by name."""
        group = SensorGroup("test")
        sensor = SimulatedSensor(name="s1")
        group.add("sensor1", sensor)

        assert group.get("sensor1") is sensor
        assert group.get("nonexistent") is None

    def test_enable_disable_all(self) -> None:
        """Should enable/disable all sensors."""
        group = SensorGroup("test")
        s1 = SimulatedSensor(name="s1")
        s2 = SimulatedSensor(name="s2")
        group.add("s1", s1)
        group.add("s2", s2)

        group.enable_all()
        assert s1.is_enabled is True
        assert s2.is_enabled is True

        group.disable_all()
        assert s1.is_enabled is False
        assert s2.is_enabled is False

    def test_read_all(self) -> None:
        """Should read all sensors."""
        group = SensorGroup("test")
        s1 = SimulatedSensor(name="s1", limits=Limits(min=0, max=100, default=10))
        s2 = SimulatedSensor(name="s2", limits=Limits(min=0, max=100, default=20))
        group.add("s1", s1)
        group.add("s2", s2)

        group.enable_all()
        readings = group.read_all()

        assert "s1" in readings
        assert "s2" in readings
        assert readings["s1"].value == 10.0
        assert readings["s2"].value == 20.0

    def test_read_all_skips_disabled(self) -> None:
        """Should skip disabled sensors."""
        group = SensorGroup("test")
        s1 = SimulatedSensor(name="s1", limits=Limits(min=0, max=100, default=10))
        s2 = SimulatedSensor(name="s2", limits=Limits(min=0, max=100, default=20))
        group.add("s1", s1)
        group.add("s2", s2)

        s1.enable()
        # s2 stays disabled

        readings = group.read_all()
        assert "s1" in readings
        assert "s2" not in readings

    def test_read_all_raw(self) -> None:
        """Should read raw values from all sensors."""
        group = SensorGroup("test")
        s1 = SimulatedSensor(name="s1", limits=Limits(min=0, max=100, default=10))
        s2 = SimulatedSensor(name="s2", limits=Limits(min=0, max=100, default=20))
        group.add("s1", s1)
        group.add("s2", s2)

        group.enable_all()
        raws = group.read_all_raw()

        assert raws["s1"] == 10
        assert raws["s2"] == 20

    def test_status_all(self) -> None:
        """Should get status of all sensors."""
        group = SensorGroup("test")
        s1 = SimulatedSensor(name="s1")
        s2 = SimulatedSensor(name="s2")
        group.add("s1", s1)
        group.add("s2", s2)

        s1.enable()

        statuses = group.status_all()
        assert statuses["s1"].is_enabled is True
        assert statuses["s2"].is_enabled is False

    def test_calibrate_all(self) -> None:
        """Should calibrate all sensors."""
        group = SensorGroup("test")
        s1 = SimulatedSensor(name="s1", limits=Limits(min=0, max=100, default=50))
        s2 = SimulatedSensor(name="s2", limits=Limits(min=0, max=100, default=50))
        group.add("s1", s1)
        group.add("s2", s2)

        group.enable_all()
        results = group.calibrate_all()

        assert results["s1"] is True
        assert results["s2"] is True

    def test_iteration(self) -> None:
        """Should iterate over sensor names."""
        group = SensorGroup("test")
        group.add("a", SimulatedSensor(name="a"))
        group.add("b", SimulatedSensor(name="b"))

        names = list(group)
        assert "a" in names
        assert "b" in names

    def test_context_manager(self) -> None:
        """Should work as context manager."""
        group = SensorGroup("test")
        s1 = SimulatedSensor(name="s1")
        s2 = SimulatedSensor(name="s2")
        group.add("s1", s1)
        group.add("s2", s2)

        with group:
            assert s1.is_enabled is True
            assert s2.is_enabled is True

        assert s1.is_enabled is False
        assert s2.is_enabled is False


# =============================================================================
# Test ReadingBuffer
# =============================================================================


class TestReadingBuffer:
    """Tests for ReadingBuffer."""

    def test_create_buffer(self) -> None:
        """Should create empty buffer."""
        buffer = ReadingBuffer(max_size=100)
        assert len(buffer) == 0

    def test_add_reading(self) -> None:
        """Should add readings."""
        buffer = ReadingBuffer()
        reading = Reading(value=42.0, unit=Unit.CELSIUS)
        buffer.add(reading)
        assert len(buffer) == 1

    def test_max_size(self) -> None:
        """Should respect max size."""
        buffer = ReadingBuffer(max_size=3)
        for i in range(5):
            buffer.add(Reading(value=float(i)))

        assert len(buffer) == 3
        assert buffer.values == [2.0, 3.0, 4.0]

    def test_clear(self) -> None:
        """Should clear buffer."""
        buffer = ReadingBuffer()
        buffer.add(Reading(value=1.0))
        buffer.add(Reading(value=2.0))
        buffer.clear()
        assert len(buffer) == 0

    def test_mean(self) -> None:
        """Should calculate mean."""
        buffer = ReadingBuffer()
        for v in [10.0, 20.0, 30.0]:
            buffer.add(Reading(value=v))
        assert buffer.mean() == 20.0

    def test_mean_empty(self) -> None:
        """Should return 0 for empty buffer."""
        buffer = ReadingBuffer()
        assert buffer.mean() == 0.0

    def test_median(self) -> None:
        """Should calculate median."""
        buffer = ReadingBuffer()
        for v in [10.0, 30.0, 20.0]:
            buffer.add(Reading(value=v))
        assert buffer.median() == 20.0

    def test_std_dev(self) -> None:
        """Should calculate standard deviation."""
        buffer = ReadingBuffer()
        for v in [10.0, 20.0, 30.0]:
            buffer.add(Reading(value=v))
        assert abs(buffer.std_dev() - 10.0) < 0.01

    def test_min_max_range(self) -> None:
        """Should calculate min, max, range."""
        buffer = ReadingBuffer()
        for v in [5.0, 15.0, 10.0]:
            buffer.add(Reading(value=v))
        assert buffer.min() == 5.0
        assert buffer.max() == 15.0
        assert buffer.range() == 10.0

    def test_latest_oldest(self) -> None:
        """Should get latest and oldest."""
        buffer = ReadingBuffer()
        r1 = Reading(value=1.0)
        r2 = Reading(value=2.0)
        r3 = Reading(value=3.0)
        buffer.add(r1)
        buffer.add(r2)
        buffer.add(r3)

        assert buffer.oldest() is r1
        assert buffer.latest() is r3

    def test_latest_oldest_empty(self) -> None:
        """Should return None for empty buffer."""
        buffer = ReadingBuffer()
        assert buffer.latest() is None
        assert buffer.oldest() is None

    def test_iteration(self) -> None:
        """Should iterate over readings."""
        buffer = ReadingBuffer()
        readings = [Reading(value=float(i)) for i in range(3)]
        for r in readings:
            buffer.add(r)

        for i, r in enumerate(buffer):
            assert r is readings[i]


# =============================================================================
# Test SensorStatus
# =============================================================================


class TestSensorStatus:
    """Tests for SensorStatus dataclass."""

    def test_create_status(self) -> None:
        """Should create status."""
        status = SensorStatus(
            state=SensorState.IDLE,
            value=42.0,
            unit=Unit.CELSIUS,
            is_enabled=True,
            is_calibrated=True,
        )
        assert status.state == SensorState.IDLE
        assert status.value == 42.0
        assert status.unit == Unit.CELSIUS
        assert status.is_enabled is True
        assert status.is_calibrated is True
        assert status.error is None
        assert status.sample_count == 0

    def test_sensor_status_method(self) -> None:
        """Should return status from sensor."""
        sensor = SimulatedSensor(
            name="test",
            unit=Unit.DEGREES,
            limits=Limits(min=0, max=360, default=90),
        )
        sensor.enable()
        sensor.read()

        status = sensor.status()
        assert status.state == SensorState.IDLE
        assert status.value == 90.0
        assert status.unit == Unit.DEGREES
        assert status.is_enabled is True
        assert status.sample_count == 1


# =============================================================================
# Test Factory Functions
# =============================================================================


class TestFactoryFunctions:
    """Tests for factory functions."""

    def test_create_sensor(self) -> None:
        """Should create generic sensor."""
        sensor = create_sensor(
            name="test",
            sensor_type=SensorType.CUSTOM,
            unit=Unit.RAW,
        )
        assert sensor.name == "test"
        assert sensor.unit == Unit.RAW

    def test_create_encoder(self) -> None:
        """Should create encoder sensor."""
        encoder = create_encoder("wheel", resolution=1024)
        assert encoder.name == "wheel"
        assert encoder.unit == Unit.DEGREES
        assert encoder.limits.min == 0
        assert encoder.limits.max == 360

    def test_create_temperature_sensor(self) -> None:
        """Should create temperature sensor."""
        temp = create_temperature_sensor("ambient", min_temp=-20, max_temp=80)
        assert temp.name == "ambient"
        assert temp.unit == Unit.CELSIUS
        assert temp.limits.min == -20
        assert temp.limits.max == 80
        assert temp.limits.default == 25.0

    def test_create_distance_sensor(self) -> None:
        """Should create distance sensor."""
        dist = create_distance_sensor("lidar", max_distance=5000)
        assert dist.name == "lidar"
        assert dist.unit == Unit.MILLIMETERS
        assert dist.limits.max == 5000

    def test_create_limit_switch(self) -> None:
        """Should create limit switch."""
        sw = create_limit_switch("limit", normally_open=True)
        assert sw.name == "limit"
        assert sw.limits.min == 0
        assert sw.limits.max == 1


# =============================================================================
# Test Async Streaming
# =============================================================================


class TestAsyncStreaming:
    """Tests for async streaming functionality."""

    @pytest.mark.asyncio
    async def test_stream_count(self) -> None:
        """Should stream specified number of readings."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100, default=50),
        )
        sensor.enable()

        readings = []
        async for reading in sensor.stream(rate=1000, count=5):
            readings.append(reading)

        assert len(readings) == 5

    @pytest.mark.asyncio
    async def test_stream_duration(self) -> None:
        """Should stream for specified duration."""
        sensor = SimulatedSensor(
            name="test",
            limits=Limits(min=0, max=100, default=50),
        )
        sensor.enable()

        start = time.time()
        readings = []
        async for reading in sensor.stream(rate=100, duration=0.1):
            readings.append(reading)
        elapsed = time.time() - start

        assert elapsed >= 0.1
        assert elapsed < 0.3  # Allow some tolerance
        assert len(readings) >= 5  # Should get at least some readings

    @pytest.mark.asyncio
    async def test_stream_disabled_sensor(self) -> None:
        """Should raise DisabledError for disabled sensor."""
        sensor = SimulatedSensor(name="test")

        with pytest.raises(DisabledError):
            async for _ in sensor.stream():
                pass


# =============================================================================
# Test Sensor ABC
# =============================================================================


class TestSensorABC:
    """Tests for Sensor abstract base class."""

    def test_sensor_properties(self) -> None:
        """Should expose expected properties."""
        sensor = SimulatedSensor(
            name="test",
            unit=Unit.DEGREES,
            limits=Limits(min=0, max=360),
            channel=5,
        )
        assert sensor.name == "test"
        assert sensor.unit == Unit.DEGREES
        assert sensor.limits.min == 0
        assert sensor.limits.max == 360
        assert sensor.channel == 5
        assert sensor.driver is None

    def test_repr(self) -> None:
        """Should have useful repr."""
        sensor = SimulatedSensor(name="encoder", unit=Unit.DEGREES)
        r = repr(sensor)
        assert "SimulatedSensor" in r
        assert "encoder" in r
        assert "DEGREES" in r

    def test_sample_count(self) -> None:
        """Should track sample count."""
        sensor = SimulatedSensor(name="test", limits=Limits(min=0, max=100, default=50))
        sensor.enable()

        assert sensor.sample_count == 0
        sensor.read()
        assert sensor.sample_count == 1
        sensor.read()
        sensor.read()
        assert sensor.sample_count == 3

        # Disable and re-enable resets count
        sensor.disable()
        sensor.enable()
        assert sensor.sample_count == 0
