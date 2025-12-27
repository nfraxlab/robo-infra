"""
Motor driver hardware tests.

These tests validate DC motor control with real motor drivers.
Requires motor drivers and motors connected to the system.

Required Hardware:
- GPIO access (e.g., Raspberry Pi)
- L298N or similar H-bridge motor driver
- DC motors connected to driver outputs

Environment Variables:
- ROBO_HARDWARE_TESTS=1: Enable hardware tests
- ROBO_TEST_MOTOR_PWM_PIN: PWM pin for motor speed (default: 18)
- ROBO_TEST_MOTOR_IN1_PIN: Direction pin 1 (default: 23)
- ROBO_TEST_MOTOR_IN2_PIN: Direction pin 2 (default: 24)

Safety:
- Motors will briefly spin during tests
- Ensure motors are secured or under load
- Keep hands and objects clear of moving parts
"""

from __future__ import annotations

import os
import time

import pytest


pytestmark = [
    pytest.mark.hardware,
    pytest.mark.motor,
]


def get_motor_pwm_pin() -> int:
    """Get motor PWM pin from environment or default."""
    return int(os.environ.get("ROBO_TEST_MOTOR_PWM_PIN", "18"))


def get_motor_in1_pin() -> int:
    """Get motor direction pin 1 from environment or default."""
    return int(os.environ.get("ROBO_TEST_MOTOR_IN1_PIN", "23"))


def get_motor_in2_pin() -> int:
    """Get motor direction pin 2 from environment or default."""
    return int(os.environ.get("ROBO_TEST_MOTOR_IN2_PIN", "24"))


class TestL298NDetection:
    """Tests for L298N motor driver detection."""

    def test_motor_pins_available(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test that motor control pins are accessible."""
        from robo_infra.core.pin import Direction, GPIOPin

        pwm_pin = get_motor_pwm_pin()
        in1_pin = get_motor_in1_pin()
        in2_pin = get_motor_in2_pin()

        # Try to configure motor control pins
        try:
            # IN1 pin (direction control)
            pin_in1 = GPIOPin(in1_pin)
            pin_in1.setup(Direction.OUTPUT)
            pin_in1.write(False)

            # IN2 pin (direction control)
            pin_in2 = GPIOPin(in2_pin)
            pin_in2.setup(Direction.OUTPUT)
            pin_in2.write(False)

            # Cleanup
            pin_in1.cleanup()
            pin_in2.cleanup()

            print(f"\nMotor pins configured: PWM={pwm_pin}, IN1={in1_pin}, IN2={in2_pin}")

        except Exception as e:
            pytest.skip(f"Motor pins not available: {e}")


class TestDCMotorBasic:
    """Basic DC motor control tests."""

    def test_motor_creation(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test creating a DC motor controller."""
        from robo_infra.motor import DCMotor

        pwm_pin = get_motor_pwm_pin()
        in1_pin = get_motor_in1_pin()
        in2_pin = get_motor_in2_pin()

        motor = DCMotor(
            pwm_pin=pwm_pin,
            direction_pin1=in1_pin,
            direction_pin2=in2_pin,
        )

        assert motor is not None
        print(f"\nDCMotor created: PWM={pwm_pin}, IN1={in1_pin}, IN2={in2_pin}")

        motor.cleanup()

    def test_motor_stop_initial_state(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test that motor starts in stopped state."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Initial state should be stopped
        assert motor.speed == 0
        assert motor.direction == "stopped"

        motor.cleanup()

    def test_motor_forward_brief(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test brief forward motion (50ms at 50% speed)."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Run forward briefly
        motor.forward(speed=0.5)
        time.sleep(0.05)  # 50ms
        motor.stop()

        assert motor.direction == "stopped"
        assert motor.speed == 0

        motor.cleanup()
        print("\nMotor ran forward briefly (50ms at 50%)")

    def test_motor_reverse_brief(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test brief reverse motion (50ms at 50% speed)."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Run reverse briefly
        motor.reverse(speed=0.5)
        time.sleep(0.05)  # 50ms
        motor.stop()

        assert motor.direction == "stopped"
        assert motor.speed == 0

        motor.cleanup()
        print("\nMotor ran reverse briefly (50ms at 50%)")


class TestMotorSpeed:
    """Tests for motor speed control."""

    def test_motor_speed_ramp(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test ramping motor speed up and down."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Ramp up
        for speed in [0.2, 0.4, 0.6, 0.8, 1.0]:
            motor.forward(speed=speed)
            time.sleep(0.02)

        # Ramp down
        for speed in [0.8, 0.6, 0.4, 0.2, 0.0]:
            motor.forward(speed=speed)
            time.sleep(0.02)

        motor.stop()
        motor.cleanup()
        print("\nMotor speed ramp completed")

    def test_motor_speed_limits(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test that speed is clamped to valid range."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Speed above 1.0 should be clamped
        motor.forward(speed=1.5)
        assert motor.speed <= 1.0

        # Speed below 0.0 should be clamped
        motor.forward(speed=-0.5)
        assert motor.speed >= 0.0

        motor.stop()
        motor.cleanup()


class TestMotorDirection:
    """Tests for motor direction control."""

    def test_motor_direction_change(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test changing motor direction."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Forward
        motor.forward(speed=0.3)
        assert motor.direction == "forward"
        time.sleep(0.03)

        # Stop between direction changes
        motor.stop()
        time.sleep(0.02)

        # Reverse
        motor.reverse(speed=0.3)
        assert motor.direction == "reverse"
        time.sleep(0.03)

        motor.stop()
        motor.cleanup()
        print("\nMotor direction change completed")

    def test_motor_brake(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test motor braking function."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Start motor
        motor.forward(speed=0.5)
        time.sleep(0.03)

        # Brake should stop motor immediately
        motor.brake()
        assert motor.speed == 0
        assert motor.direction == "stopped"

        motor.cleanup()
        print("\nMotor brake completed")


class TestMotorTiming:
    """Tests for motor control timing."""

    def test_motor_response_time(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test motor command response time."""
        from robo_infra.motor import DCMotor

        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
        )

        # Measure time to set speed
        start = time.perf_counter()
        motor.forward(speed=0.5)
        elapsed = time.perf_counter() - start

        motor.stop()
        motor.cleanup()

        print(f"\nMotor command latency: {elapsed * 1000:.2f} ms")

        # Command should complete within 10ms
        assert elapsed < 0.010, f"Motor command too slow: {elapsed * 1000:.2f} ms"

    def test_motor_pwm_frequency(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test setting PWM frequency for motor."""
        from robo_infra.motor import DCMotor

        # Create motor with specific PWM frequency
        motor = DCMotor(
            pwm_pin=get_motor_pwm_pin(),
            direction_pin1=get_motor_in1_pin(),
            direction_pin2=get_motor_in2_pin(),
            pwm_frequency=1000,  # 1kHz
        )

        assert motor.pwm_frequency == 1000
        print(f"\nMotor PWM frequency: {motor.pwm_frequency} Hz")

        motor.cleanup()


class TestDualMotor:
    """Tests for dual motor control (differential drive)."""

    def test_dual_motor_creation(
        self,
        require_hardware: None,
        require_gpio: None,
    ) -> None:
        """Test creating dual motor controller for differential drive."""
        from robo_infra.motor import DifferentialDrive

        # Skip if additional motor pins not configured
        left_pwm = int(os.environ.get("ROBO_TEST_LEFT_MOTOR_PWM", "18"))
        left_in1 = int(os.environ.get("ROBO_TEST_LEFT_MOTOR_IN1", "23"))
        left_in2 = int(os.environ.get("ROBO_TEST_LEFT_MOTOR_IN2", "24"))
        right_pwm = int(os.environ.get("ROBO_TEST_RIGHT_MOTOR_PWM", "12"))
        right_in1 = int(os.environ.get("ROBO_TEST_RIGHT_MOTOR_IN1", "5"))
        right_in2 = int(os.environ.get("ROBO_TEST_RIGHT_MOTOR_IN2", "6"))

        try:
            drive = DifferentialDrive(
                left_pwm_pin=left_pwm,
                left_dir_pin1=left_in1,
                left_dir_pin2=left_in2,
                right_pwm_pin=right_pwm,
                right_dir_pin1=right_in1,
                right_dir_pin2=right_in2,
            )
        except Exception as e:
            pytest.skip(f"Dual motor not available: {e}")

        assert drive is not None
        drive.cleanup()
        print("\nDifferentialDrive created successfully")
