"""
I2C device hardware tests.

These tests validate I2C communication with real devices.
Requires I2C devices connected to the I2C bus.

Required Hardware:
- I2C bus enabled (e.g., /dev/i2c-1 on Raspberry Pi)
- Optional: MPU6050 IMU at address 0x68
- Optional: PCA9685 PWM driver at address 0x40
- Optional: OLED display at address 0x3C

Environment Variables:
- ROBO_HARDWARE_TESTS=1: Enable hardware tests
- ROBO_TEST_I2C_BUS: I2C bus number (default: 1)
- ROBO_TEST_MPU6050_ADDR: MPU6050 address (default: 0x68)
- ROBO_TEST_PCA9685_ADDR: PCA9685 address (default: 0x40)
"""

from __future__ import annotations

import time

import pytest


pytestmark = [
    pytest.mark.hardware,
    pytest.mark.i2c,
]


class TestI2CBus:
    """Tests for I2C bus operations."""

    def test_i2c_bus_open_close(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
    ) -> None:
        """Test opening and closing I2C bus."""
        from robo_infra.core import I2CBus

        bus = I2CBus(bus_number=test_i2c_bus)

        # Open should succeed
        bus.open()
        assert bus.is_open

        # Close should succeed
        bus.close()
        assert not bus.is_open

    def test_i2c_bus_scan(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
    ) -> None:
        """Test scanning I2C bus for devices."""
        from robo_infra.core import I2CBus

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        devices = bus.scan()
        print(f"\nI2C devices found: {[hex(addr) for addr in devices]}")

        bus.close()

        # Scan should return a list (may be empty if no devices)
        assert isinstance(devices, list)

    def test_i2c_bus_context_manager(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
    ) -> None:
        """Test I2C bus as context manager."""
        from robo_infra.core import I2CBus

        with I2CBus(bus_number=test_i2c_bus) as bus:
            devices = bus.scan()
            print(f"\nI2C devices found: {[hex(addr) for addr in devices]}")


class TestMPU6050:
    """Tests for MPU6050 IMU sensor."""

    def test_mpu6050_detected(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
        mpu6050_address: int,
    ) -> None:
        """Test that MPU6050 is detected on I2C bus."""
        from robo_infra.core import I2CBus

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        devices = bus.scan()
        bus.close()

        if mpu6050_address not in devices:
            pytest.skip(
                f"MPU6050 not found at 0x{mpu6050_address:02X}. "
                f"Detected devices: {[hex(a) for a in devices]}"
            )

    def test_mpu6050_who_am_i(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
        mpu6050_address: int,
    ) -> None:
        """Test reading MPU6050 WHO_AM_I register."""
        from robo_infra.core import I2CBus

        WHO_AM_I_REG = 0x75
        EXPECTED_ID = 0x68  # Default MPU6050 ID

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        # Check if MPU6050 is present
        devices = bus.scan()
        if mpu6050_address not in devices:
            bus.close()
            pytest.skip(f"MPU6050 not found at 0x{mpu6050_address:02X}")

        # Read WHO_AM_I register
        data = bus.read_register(mpu6050_address, WHO_AM_I_REG, 1)
        bus.close()

        who_am_i = data[0]
        assert who_am_i == EXPECTED_ID, (
            f"WHO_AM_I mismatch: expected 0x{EXPECTED_ID:02X}, got 0x{who_am_i:02X}"
        )

    def test_mpu6050_read_acceleration(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
        mpu6050_address: int,
    ) -> None:
        """Test reading acceleration data from MPU6050."""
        from robo_infra.core import I2CBus

        ACCEL_XOUT_H = 0x3B  # First acceleration register

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        # Check if MPU6050 is present
        devices = bus.scan()
        if mpu6050_address not in devices:
            bus.close()
            pytest.skip(f"MPU6050 not found at 0x{mpu6050_address:02X}")

        # Wake up MPU6050 (write 0 to PWR_MGMT_1)
        bus.write_register(mpu6050_address, 0x6B, bytes([0x00]))
        time.sleep(0.1)

        # Read 6 bytes of acceleration data (X, Y, Z high and low bytes)
        data = bus.read_register(mpu6050_address, ACCEL_XOUT_H, 6)
        bus.close()

        # Convert to 16-bit signed values
        def bytes_to_int16(high: int, low: int) -> int:
            value = (high << 8) | low
            if value >= 0x8000:
                value -= 0x10000
            return value

        accel_x = bytes_to_int16(data[0], data[1])
        accel_y = bytes_to_int16(data[2], data[3])
        accel_z = bytes_to_int16(data[4], data[5])

        print(f"\nMPU6050 acceleration: X={accel_x}, Y={accel_y}, Z={accel_z}")

        # At rest, one axis should show ~16384 (1g at default sensitivity)
        # Z-axis typically shows ~+16384 when flat
        total_accel = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5
        assert 10000 < total_accel < 25000, f"Unexpected acceleration magnitude: {total_accel}"


class TestPCA9685:
    """Tests for PCA9685 PWM driver."""

    def test_pca9685_detected(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
        pca9685_address: int,
    ) -> None:
        """Test that PCA9685 is detected on I2C bus."""
        from robo_infra.core import I2CBus

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        devices = bus.scan()
        bus.close()

        if pca9685_address not in devices:
            pytest.skip(
                f"PCA9685 not found at 0x{pca9685_address:02X}. "
                f"Detected devices: {[hex(a) for a in devices]}"
            )

    def test_pca9685_mode_register(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
        pca9685_address: int,
    ) -> None:
        """Test reading PCA9685 MODE1 register."""
        from robo_infra.core import I2CBus

        MODE1_REG = 0x00

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        # Check if PCA9685 is present
        devices = bus.scan()
        if pca9685_address not in devices:
            bus.close()
            pytest.skip(f"PCA9685 not found at 0x{pca9685_address:02X}")

        # Read MODE1 register
        data = bus.read_register(pca9685_address, MODE1_REG, 1)
        bus.close()

        mode1 = data[0]
        print(f"\nPCA9685 MODE1 register: 0x{mode1:02X}")

        # MODE1 should be readable (value depends on state)
        assert isinstance(mode1, int)


class TestI2CThroughput:
    """Tests for I2C throughput and performance."""

    def test_i2c_read_throughput(
        self,
        require_hardware: None,
        require_i2c: None,
        test_i2c_bus: int,
        mpu6050_address: int,
    ) -> None:
        """Test I2C read throughput with MPU6050."""
        from robo_infra.core import I2CBus

        bus = I2CBus(bus_number=test_i2c_bus)
        bus.open()

        # Check if MPU6050 is present
        devices = bus.scan()
        if mpu6050_address not in devices:
            bus.close()
            pytest.skip(f"MPU6050 not found at 0x{mpu6050_address:02X}")

        # Wake up MPU6050
        bus.write_register(mpu6050_address, 0x6B, bytes([0x00]))
        time.sleep(0.1)

        # Measure read throughput
        ACCEL_XOUT_H = 0x3B
        num_reads = 100

        start = time.perf_counter()
        for _ in range(num_reads):
            bus.read_register(mpu6050_address, ACCEL_XOUT_H, 6)
        elapsed = time.perf_counter() - start

        bus.close()

        reads_per_sec = num_reads / elapsed
        print(f"\nI2C read throughput: {reads_per_sec:.1f} reads/sec")

        # Should achieve at least 100 reads/sec at 100kHz I2C
        assert reads_per_sec > 50, f"I2C throughput too low: {reads_per_sec:.1f}/s"
