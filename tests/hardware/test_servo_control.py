"""
Servo control hardware tests.

These tests validate servo motor control using real hardware.
Requires a PCA9685 PWM driver board and a connected servo motor.

Required Hardware:
- PCA9685 PWM driver on I2C bus 1, address 0x40
- SG90 or similar servo on channel 0
- 5V power supply for servo

Environment Variables:
- ROBO_HARDWARE_TESTS=1: Enable hardware tests
- ROBO_TEST_SERVO_CHANNEL: PCA9685 channel (default: 0)
- ROBO_TEST_PCA9685_ADDR: PCA9685 I2C address (default: 0x40)
- ROBO_TEST_I2C_BUS: I2C bus number (default: 1)

Safety Notes:
- Ensure servo has sufficient power supply
- Keep hands clear of servo during tests
- Servo will move through its range of motion
"""

from __future__ import annotations

import time

import pytest

pytestmark = [
    pytest.mark.hardware,
    pytest.mark.servo,
    pytest.mark.i2c,
]


class TestPCA9685Detection:
    """Tests for PCA9685 PWM driver detection."""

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

        assert pca9685_address in devices, (
            f"PCA9685 not found at 0x{pca9685_address:02X}. "
            f"Detected devices: {[hex(a) for a in devices]}"
        )


class TestServoBasicControl:
    """Tests for basic servo control operations."""

    def test_servo_creation(
        self,
        require_hardware: None,
        require_i2c: None,
        test_servo_channel: int,
        pca9685_address: int,
        test_i2c_bus: int,
    ) -> None:
        """Test servo object creation."""
        from robo_infra.core import Limits, Servo

        servo = Servo(
            name="test_servo",
            channel=test_servo_channel,
            limits=Limits(min=0, max=180, default=90),
        )

        assert servo.name == "test_servo"
        assert servo.channel == test_servo_channel

    def test_servo_center_position(
        self,
        require_hardware: None,
        require_i2c: None,
        test_servo_channel: int,
        pca9685_address: int,
        test_i2c_bus: int,
    ) -> None:
        """Test moving servo to center position (90 degrees).

        This will physically move the servo!
        """
        from robo_infra.core import Limits, Servo
        from robo_infra.drivers import PCA9685Driver

        driver = PCA9685Driver(
            bus=test_i2c_bus,
            address=pca9685_address,
        )
        driver.enable()

        servo = Servo(
            name="test_servo",
            driver=driver,
            channel=test_servo_channel,
            limits=Limits(min=0, max=180, default=90),
        )
        servo.enable()

        # Move to center
        servo.set(90)
        time.sleep(0.5)  # Wait for servo to reach position

        position = servo.get()
        assert 85 <= position <= 95, f"Expected ~90°, got {position}°"

        servo.disable()
        driver.disable()

    def test_servo_range_sweep(
        self,
        require_hardware: None,
        require_i2c: None,
        test_servo_channel: int,
        pca9685_address: int,
        test_i2c_bus: int,
    ) -> None:
        """Test sweeping servo through its range of motion.

        This will physically move the servo from 0° to 180° and back!
        """
        from robo_infra.core import Limits, Servo
        from robo_infra.drivers import PCA9685Driver

        driver = PCA9685Driver(
            bus=test_i2c_bus,
            address=pca9685_address,
        )
        driver.enable()

        servo = Servo(
            name="test_servo",
            driver=driver,
            channel=test_servo_channel,
            limits=Limits(min=0, max=180, default=90),
        )
        servo.enable()

        # Sweep from 0 to 180 degrees
        positions = [0, 45, 90, 135, 180, 135, 90, 45, 0]
        for pos in positions:
            servo.set(pos)
            time.sleep(0.3)

        # Return to center
        servo.set(90)
        time.sleep(0.3)

        servo.disable()
        driver.disable()


class TestServoLimits:
    """Tests for servo limit enforcement."""

    def test_servo_min_limit(
        self,
        require_hardware: None,
        require_i2c: None,
        test_servo_channel: int,
        pca9685_address: int,
        test_i2c_bus: int,
    ) -> None:
        """Test that servo respects minimum limit."""
        from robo_infra.core import Limits, Servo
        from robo_infra.drivers import PCA9685Driver

        driver = PCA9685Driver(
            bus=test_i2c_bus,
            address=pca9685_address,
        )
        driver.enable()

        servo = Servo(
            name="test_servo",
            driver=driver,
            channel=test_servo_channel,
            limits=Limits(min=30, max=150, default=90),
        )
        servo.enable()

        # Try to go below minimum
        servo.set(0)  # Should be clamped to 30
        time.sleep(0.3)

        position = servo.get()
        assert position >= 30, f"Servo went below min limit: {position}°"

        servo.disable()
        driver.disable()

    def test_servo_max_limit(
        self,
        require_hardware: None,
        require_i2c: None,
        test_servo_channel: int,
        pca9685_address: int,
        test_i2c_bus: int,
    ) -> None:
        """Test that servo respects maximum limit."""
        from robo_infra.core import Limits, Servo
        from robo_infra.drivers import PCA9685Driver

        driver = PCA9685Driver(
            bus=test_i2c_bus,
            address=pca9685_address,
        )
        driver.enable()

        servo = Servo(
            name="test_servo",
            driver=driver,
            channel=test_servo_channel,
            limits=Limits(min=30, max=150, default=90),
        )
        servo.enable()

        # Try to go above maximum
        servo.set(180)  # Should be clamped to 150
        time.sleep(0.3)

        position = servo.get()
        assert position <= 150, f"Servo went above max limit: {position}°"

        servo.disable()
        driver.disable()


class TestServoTiming:
    """Tests for servo timing and responsiveness."""

    def test_servo_response_time(
        self,
        require_hardware: None,
        require_i2c: None,
        test_servo_channel: int,
        pca9685_address: int,
        test_i2c_bus: int,
    ) -> None:
        """Test servo command latency.

        Measures time from command to acknowledgment (not physical movement).
        """
        import time

        from robo_infra.core import Limits, Servo
        from robo_infra.drivers import PCA9685Driver

        driver = PCA9685Driver(
            bus=test_i2c_bus,
            address=pca9685_address,
        )
        driver.enable()

        servo = Servo(
            name="test_servo",
            driver=driver,
            channel=test_servo_channel,
            limits=Limits(min=0, max=180, default=90),
        )
        servo.enable()

        # Measure command latency
        latencies = []
        for angle in [45, 90, 135, 90]:
            start = time.perf_counter()
            servo.set(angle)
            latency = (time.perf_counter() - start) * 1000  # ms
            latencies.append(latency)
            time.sleep(0.1)

        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)

        print(f"\nServo command latency: avg={avg_latency:.2f}ms, max={max_latency:.2f}ms")

        # Command latency should be under 10ms
        assert max_latency < 10, f"Servo command too slow: {max_latency:.2f}ms"

        servo.disable()
        driver.disable()
