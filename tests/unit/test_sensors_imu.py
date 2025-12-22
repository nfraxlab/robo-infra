"""Tests for robo_infra.sensors.imu (Phase 4.2)."""

from __future__ import annotations

import pytest

from robo_infra.core.bus import SimulatedI2CBus
from robo_infra.core.driver import SimulatedDriver
from robo_infra.core.exceptions import CalibrationError
from robo_infra.core.types import Unit, Vector3
from robo_infra.sensors.imu import (
    IMU,
    Accelerometer,
    AccelerometerConfig,
    Gyroscope,
    GyroscopeConfig,
    IMUConfig,
    IMUSensor,
    Magnetometer,
    MagnetometerConfig,
)


class TestIMUSensor:
    """Tests for the IMUSensor base class."""

    def test_base_class_read_vector_default(self) -> None:
        """IMUSensor.read_vector returns a Vector3."""

        class MinimalIMU(IMUSensor):
            def _read_raw(self) -> int:
                return 42

        sensor = MinimalIMU(name="test")
        sensor.enable()
        vec = sensor.read_vector()
        assert isinstance(vec, Vector3)
        assert vec.x == 42.0
        assert vec.y == 0.0
        assert vec.z == 0.0


class TestAccelerometer:
    """Tests for the Accelerometer class."""

    def test_simulated_returns_zero_at_rest(self) -> None:
        """Accelerometer with no backend returns zeros."""
        accel = Accelerometer(config=AccelerometerConfig(name="test_accel"))
        accel.enable()

        vec = accel.read_vector()
        assert vec.x == 0.0
        assert vec.y == 0.0
        assert vec.z == 0.0

    def test_driver_backed_reads_channels(self) -> None:
        """Accelerometer can read from driver channels."""
        driver = SimulatedDriver(name="drv", channels=6)
        driver.connect()
        driver.enable()

        # Set raw values for X, Y, Z (channels 0, 1, 2)
        driver.set_channel(0, 1000)
        driver.set_channel(1, 2000)
        driver.set_channel(2, 3000)

        config = AccelerometerConfig(
            name="driver_accel",
            range_g=2.0,
            resolution_bits=16,
        )
        accel = Accelerometer(driver=driver, channel=0, config=config)
        accel.enable()

        vec = accel.read_vector()
        # Scale factor = 2.0 / 32767 ≈ 0.000061
        assert vec.x != 0.0  # Should have converted from raw

    def test_i2c_reads_six_bytes(self) -> None:
        """Accelerometer reads 6 bytes from I2C bus."""
        bus = SimulatedI2CBus()
        # Set up registers for X=1000, Y=0, Z=0 (big-endian)
        bus.add_device(
            0x68,
            registers={
                0x00: 0x03,  # X high
                0x01: 0xE8,  # X low (0x03E8 = 1000)
                0x02: 0x00,  # Y high
                0x03: 0x00,  # Y low
                0x04: 0x00,  # Z high
                0x05: 0x00,  # Z low
            },
        )
        bus.open()

        config = AccelerometerConfig(range_g=2.0, resolution_bits=16)
        accel = Accelerometer(bus=bus, config=config)
        accel.enable()

        vec = accel.read_vector()
        # raw X = 1000, scale = 2.0/32767 ≈ 0.000061
        expected_x = 1000 * (2.0 / 32767)
        assert abs(vec.x - expected_x) < 0.001

    def test_read_acceleration_returns_full_reading(self) -> None:
        """read_acceleration() returns AccelerometerReading."""
        accel = Accelerometer()
        accel.enable()

        reading = accel.read_acceleration()
        assert reading.unit == Unit.METERS_PER_SECOND
        assert isinstance(reading.vector, Vector3)
        assert reading.magnitude >= 0
        assert reading.raw == (0, 0, 0)
        assert reading.timestamp > 0

    def test_calibration_requires_enabled(self) -> None:
        """Calibration raises error if sensor not enabled."""
        accel = Accelerometer()
        with pytest.raises(CalibrationError):
            accel._run_calibration()

    def test_config_scale_factor(self) -> None:
        """AccelerometerConfig.scale_factor is calculated correctly."""
        config = AccelerometerConfig(range_g=4.0, resolution_bits=16)
        expected = 4.0 / 32767
        assert abs(config.scale_factor - expected) < 1e-10


class TestGyroscope:
    """Tests for the Gyroscope class."""

    def test_simulated_returns_zero_at_rest(self) -> None:
        """Gyroscope with no backend returns zeros."""
        gyro = Gyroscope(config=GyroscopeConfig(name="test_gyro"))
        gyro.enable()

        vec = gyro.read_vector()
        assert vec.x == 0.0
        assert vec.y == 0.0
        assert vec.z == 0.0

    def test_driver_backed_reads_channels(self) -> None:
        """Gyroscope can read from driver channels."""
        driver = SimulatedDriver(name="drv", channels=6)
        driver.connect()
        driver.enable()

        driver.set_channel(0, 500)
        driver.set_channel(1, -500)
        driver.set_channel(2, 1000)

        config = GyroscopeConfig(range_dps=250.0, resolution_bits=16)
        gyro = Gyroscope(driver=driver, channel=0, config=config)
        gyro.enable()
        gyro._is_calibrated = True  # Skip calibration requirement

        vec = gyro.read_vector()
        # Values should be scaled
        assert vec.x != 0.0

    def test_i2c_reads_six_bytes(self) -> None:
        """Gyroscope reads 6 bytes from I2C bus."""
        bus = SimulatedI2CBus()
        bus.add_device(
            0x68,
            registers={
                0x00: 0x00,
                0x01: 0x00,
                0x02: 0x00,
                0x03: 0x00,
                0x04: 0x07,  # Z high
                0x05: 0xD0,  # Z low (0x07D0 = 2000)
            },
        )
        bus.open()

        config = GyroscopeConfig(range_dps=250.0, resolution_bits=16)
        gyro = Gyroscope(bus=bus, config=config)
        gyro.enable()
        gyro._is_calibrated = True

        vec = gyro.read_vector()
        # Z = 2000 raw, scale = 250/32767
        expected_z = 2000 * (250.0 / 32767)
        assert abs(vec.z - expected_z) < 0.1

    def test_read_angular_velocity_returns_full_reading(self) -> None:
        """read_angular_velocity() returns GyroscopeReading."""
        gyro = Gyroscope()
        gyro.enable()
        gyro._is_calibrated = True

        reading = gyro.read_angular_velocity()
        assert reading.unit == Unit.DEGREES_PER_SECOND
        assert isinstance(reading.vector, Vector3)
        assert reading.raw == (0, 0, 0)

    def test_calibrate_method(self) -> None:
        """calibrate_zero() convenience method works."""
        gyro = Gyroscope()
        gyro.enable()
        assert not gyro.is_calibrated

        gyro.calibrate_zero()
        assert gyro.is_calibrated

    def test_config_scale_factor(self) -> None:
        """GyroscopeConfig.scale_factor is calculated correctly."""
        config = GyroscopeConfig(range_dps=500.0, resolution_bits=16)
        expected = 500.0 / 32767
        assert abs(config.scale_factor - expected) < 1e-10


class TestMagnetometer:
    """Tests for the Magnetometer class."""

    def test_simulated_returns_zero(self) -> None:
        """Magnetometer with no backend returns zeros."""
        mag = Magnetometer(config=MagnetometerConfig(name="test_mag"))
        mag.enable()
        mag._is_calibrated = True

        vec = mag.read_vector()
        assert vec.x == 0.0
        assert vec.y == 0.0
        assert vec.z == 0.0

    def test_heading_north(self) -> None:
        """Heading calculation for north (X+, Y=0)."""
        bus = SimulatedI2CBus()
        # X = 1000 (pointing north), Y = 0
        bus.add_device(
            0x1E,
            registers={
                0x00: 0x03,  # X high
                0x01: 0xE8,  # X low (1000)
                0x02: 0x00,  # Y high
                0x03: 0x00,  # Y low
                0x04: 0x00,  # Z high
                0x05: 0x00,  # Z low
            },
        )
        bus.open()

        mag = Magnetometer(bus=bus)
        mag.enable()
        mag._is_calibrated = True

        heading = mag.heading()
        # atan2(0, positive) = 0 degrees (North)
        assert abs(heading - 0.0) < 1.0

    def test_heading_east(self) -> None:
        """Heading calculation for east (X=0, Y+)."""
        bus = SimulatedI2CBus()
        # X = 0, Y = 1000 (pointing east)
        bus.add_device(
            0x1E,
            registers={
                0x00: 0x00,
                0x01: 0x00,
                0x02: 0x03,  # Y high
                0x03: 0xE8,  # Y low (1000)
                0x04: 0x00,
                0x05: 0x00,
            },
        )
        bus.open()

        mag = Magnetometer(bus=bus)
        mag.enable()
        mag._is_calibrated = True

        heading = mag.heading()
        # atan2(positive, 0) = 90 degrees (East)
        assert abs(heading - 90.0) < 1.0

    def test_heading_with_declination(self) -> None:
        """Heading adjusts for magnetic declination."""
        config = MagnetometerConfig(declination=10.0)

        bus = SimulatedI2CBus()
        bus.add_device(0x1E, registers=dict.fromkeys(range(6), 0))
        bus.open()

        mag = Magnetometer(bus=bus, config=config)
        mag.enable()
        mag._is_calibrated = True

        # With X=0, Y=0 we get heading=0, but declination adds 10
        heading = mag.heading()
        assert abs(heading - 10.0) < 1.0

    def test_read_magnetic_field_returns_full_reading(self) -> None:
        """read_magnetic_field() returns MagnetometerReading."""
        mag = Magnetometer()
        mag.enable()
        mag._is_calibrated = True

        reading = mag.read_magnetic_field()
        assert reading.unit == Unit.RAW
        assert isinstance(reading.vector, Vector3)
        assert 0 <= reading.heading < 360
        assert reading.raw == (0, 0, 0)

    def test_config_scale_factor(self) -> None:
        """MagnetometerConfig.scale_factor is calculated correctly."""
        config = MagnetometerConfig(range_gauss=2.0, resolution_bits=12)
        expected = 2.0 / 2047
        assert abs(config.scale_factor - expected) < 1e-10


class TestIMU:
    """Tests for the combined IMU class."""

    def test_creates_sub_sensors(self) -> None:
        """IMU creates accelerometer, gyroscope, and magnetometer."""
        imu = IMU()
        assert isinstance(imu.accelerometer, Accelerometer)
        assert isinstance(imu.gyroscope, Gyroscope)
        assert isinstance(imu.magnetometer, Magnetometer)

    def test_enable_enables_all_sub_sensors(self) -> None:
        """enable() enables all sub-sensors."""
        imu = IMU()
        imu.enable()

        assert imu.is_enabled
        assert imu.accelerometer.is_enabled
        assert imu.gyroscope.is_enabled
        assert imu.magnetometer.is_enabled

    def test_disable_disables_all_sub_sensors(self) -> None:
        """disable() disables all sub-sensors."""
        imu = IMU()
        imu.enable()
        imu.disable()

        assert not imu.is_enabled
        assert not imu.accelerometer.is_enabled
        assert not imu.gyroscope.is_enabled
        assert not imu.magnetometer.is_enabled

    def test_read_all_returns_imu_reading(self) -> None:
        """read_all() returns complete IMUReading."""
        imu = IMU()
        imu.enable()
        # Mark calibrated to allow reading
        imu._is_calibrated = True
        imu.gyroscope._is_calibrated = True
        imu.magnetometer._is_calibrated = True

        reading = imu.read_all()

        assert isinstance(reading.acceleration, Vector3)
        assert isinstance(reading.angular_velocity, Vector3)
        assert isinstance(reading.magnetic_field, Vector3)
        assert 0 <= reading.heading < 360
        assert reading.timestamp > 0
        assert reading.roll is not None
        assert reading.pitch is not None
        assert reading.yaw is not None

    def test_read_vector_returns_acceleration(self) -> None:
        """read_vector() returns acceleration from accelerometer."""
        imu = IMU()
        imu.enable()

        vec = imu.read_vector()
        assert isinstance(vec, Vector3)

    def test_calibrate_calibrates_all_sub_sensors(self) -> None:
        """calibrate_all() calibrates all sub-sensors."""
        imu = IMU()
        imu.enable()

        imu.calibrate_all()

        assert imu.is_calibrated
        assert imu.accelerometer.is_calibrated
        assert imu.gyroscope.is_calibrated
        assert imu.magnetometer.is_calibrated

    def test_calibrate_requires_enabled(self) -> None:
        """Calibration raises error if IMU not enabled."""
        imu = IMU()
        with pytest.raises(CalibrationError):
            imu.calibrate_all()

    def test_driver_backed_sub_sensors(self) -> None:
        """IMU with driver creates properly channeled sub-sensors."""
        driver = SimulatedDriver(name="drv", channels=12)
        driver.connect()
        driver.enable()

        # Set values for all channels
        for i in range(9):
            driver.set_channel(i, 100 * (i + 1))

        imu = IMU(driver=driver, channel=0)
        imu.enable()
        imu._is_calibrated = True
        imu.gyroscope._is_calibrated = True
        imu.magnetometer._is_calibrated = True

        # Accelerometer uses channels 0, 1, 2
        # Gyroscope uses channels 3, 4, 5
        # Magnetometer uses channels 6, 7, 8
        reading = imu.read_all()

        # Just verify we got data
        assert isinstance(reading.acceleration, Vector3)

    def test_config_passes_to_sub_sensors(self) -> None:
        """IMUConfig sub-sensor configs are used."""
        accel_config = AccelerometerConfig(name="custom_accel", range_g=8.0)
        gyro_config = GyroscopeConfig(name="custom_gyro", range_dps=500.0)
        mag_config = MagnetometerConfig(name="custom_mag", range_gauss=4.0)

        config = IMUConfig(
            accelerometer=accel_config,
            gyroscope=gyro_config,
            magnetometer=mag_config,
        )

        imu = IMU(config=config)

        assert imu.accelerometer._config.range_g == 8.0
        assert imu.gyroscope._config.range_dps == 500.0
        assert imu.magnetometer._config.range_gauss == 4.0


class TestIMUMathematics:
    """Tests for IMU mathematical calculations."""

    def test_vector3_magnitude(self) -> None:
        """Vector3 magnitude calculation is correct."""
        vec = Vector3(x=3.0, y=4.0, z=0.0)
        assert abs(vec.magnitude() - 5.0) < 0.001

    def test_heading_quadrants(self) -> None:
        """Heading calculation works in all quadrants."""
        mag = Magnetometer()
        mag.enable()
        mag._is_calibrated = True

        # Test _calculate_heading directly
        # North (0°): X+, Y=0
        assert abs(mag._calculate_heading(1.0, 0.0) - 0.0) < 1.0

        # East (90°): X=0, Y+
        assert abs(mag._calculate_heading(0.0, 1.0) - 90.0) < 1.0

        # South (180°): X-, Y=0
        assert abs(mag._calculate_heading(-1.0, 0.0) - 180.0) < 1.0

        # West (270°): X=0, Y-
        assert abs(mag._calculate_heading(0.0, -1.0) - 270.0) < 1.0

    def test_signed_16bit_conversion(self) -> None:
        """I2C values are correctly converted to signed 16-bit."""
        bus = SimulatedI2CBus()
        # Set X to -1000: 0xFFFF - 999 = 0xFC18
        bus.add_device(
            0x68,
            registers={
                0x00: 0xFC,  # X high
                0x01: 0x18,  # X low (-1000 as signed)
                0x02: 0x00,
                0x03: 0x00,
                0x04: 0x00,
                0x05: 0x00,
            },
        )
        bus.open()

        accel = Accelerometer(bus=bus)
        accel.enable()

        raw = accel._read_raw_xyz()
        # 0xFC18 = 64536 as unsigned, but should be -1000 as signed
        assert raw[0] == -1000


class TestIMUErrorHandling:
    """Tests for IMU error handling."""

    def test_accelerometer_handles_missing_device(self) -> None:
        """Accelerometer returns zeros when device not registered."""
        bus = SimulatedI2CBus()
        # Don't add device - reading returns zeros
        bus.open()

        accel = Accelerometer(bus=bus)
        accel.enable()

        # Should not raise - simulated bus returns zeros
        vec = accel.read_vector()
        assert vec.x == 0.0
        assert vec.y == 0.0
        assert vec.z == 0.0

    def test_gyroscope_handles_missing_device(self) -> None:
        """Gyroscope returns zeros when device not registered."""
        bus = SimulatedI2CBus()
        bus.open()

        gyro = Gyroscope(bus=bus)
        gyro.enable()
        gyro._is_calibrated = True

        vec = gyro.read_vector()
        assert vec.x == 0.0
        assert vec.y == 0.0
        assert vec.z == 0.0

    def test_magnetometer_handles_missing_device(self) -> None:
        """Magnetometer returns zeros when device not registered."""
        bus = SimulatedI2CBus()
        bus.open()

        mag = Magnetometer(bus=bus)
        mag.enable()
        mag._is_calibrated = True

        vec = mag.read_vector()
        assert vec.x == 0.0
        assert vec.y == 0.0
        assert vec.z == 0.0
