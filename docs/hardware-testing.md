# Hardware Testing Guide

This guide explains how to set up and run hardware tests on real robotics hardware.

## Overview

Hardware tests validate that robo-infra works correctly with physical hardware:

- **Raspberry Pi GPIO** - Pin I/O, PWM, hardware timers
- **I2C Devices** - MPU6050 IMU, PCA9685 PWM driver, sensors
- **SPI Devices** - High-speed communication
- **Servo Motors** - Position control via PCA9685
- **DC Motors** - Speed and direction control via L298N

## Prerequisites

### Hardware Requirements

1. **Raspberry Pi 4** (recommended) or Pi 3B+
2. **I2C devices** (optional):
   - PCA9685 16-channel PWM driver
   - MPU6050 6-axis IMU
3. **Motor hardware** (optional):
   - L298N dual H-bridge motor driver
   - DC motors (3-12V)
4. **Servo motors** (optional):
   - SG90 or MG996R servos

### Software Requirements

```bash
# Enable I2C and SPI on Raspberry Pi
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
# Navigate to: Interface Options → SPI → Enable

# Install system dependencies
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus

# Verify I2C is working
i2cdetect -y 1
```

## Running Hardware Tests

### Enable Hardware Tests

Hardware tests are skipped by default. Enable them with:

```bash
# Enable hardware tests
export ROBO_HARDWARE_TESTS=1

# Run all hardware tests
pytest tests/hardware/ -v

# Run specific categories
pytest tests/hardware/ -v -m raspberry_pi
pytest tests/hardware/ -v -m i2c
pytest tests/hardware/ -v -m servo
pytest tests/hardware/ -v -m motor
```

### Configuration Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBO_HARDWARE_TESTS` | `0` | Set to `1` to enable hardware tests |
| `ROBO_TEST_LED_PIN` | `17` | GPIO pin for LED tests |
| `ROBO_TEST_INPUT_PIN` | `27` | GPIO pin for input tests |
| `ROBO_TEST_I2C_BUS` | `1` | I2C bus number |
| `ROBO_TEST_SERVO_CHANNEL` | `0` | PCA9685 channel for servo tests |
| `ROBO_TEST_MPU6050_ADDR` | `0x68` | MPU6050 I2C address |
| `ROBO_TEST_PCA9685_ADDR` | `0x40` | PCA9685 I2C address |
| `ROBO_TEST_MOTOR_PWM_PIN` | `18` | PWM pin for motor speed |
| `ROBO_TEST_MOTOR_IN1_PIN` | `23` | Motor direction pin 1 |
| `ROBO_TEST_MOTOR_IN2_PIN` | `24` | Motor direction pin 2 |

## Self-Hosted Runner Setup

### Raspberry Pi CI Runner

Set up a self-hosted GitHub Actions runner on your Raspberry Pi:

```bash
# Create a dedicated user
sudo useradd -m -G gpio,i2c,spi github-runner
sudo passwd github-runner

# Switch to runner user
sudo su - github-runner

# Download latest runner
mkdir actions-runner && cd actions-runner
curl -o actions-runner-linux-arm64-2.311.0.tar.gz -L \
  https://github.com/actions/runner/releases/download/v2.311.0/actions-runner-linux-arm64-2.311.0.tar.gz
tar xzf ./actions-runner-linux-arm64-2.311.0.tar.gz

# Configure runner (get token from GitHub repo settings)
./config.sh --url https://github.com/YOUR_ORG/robo-infra \
  --token YOUR_RUNNER_TOKEN \
  --labels raspberry-pi,hardware-tests

# Install as service
sudo ./svc.sh install
sudo ./svc.sh start
```

### Runner Labels

Add these labels to your self-hosted runner:

- `raspberry-pi` - General Raspberry Pi tests
- `hardware-tests` - All hardware tests
- `gpio` - GPIO-specific tests
- `i2c` - I2C device tests

### Security Considerations

- Limit repository access to trusted contributors
- Use a dedicated Pi for CI (not production)
- Set appropriate file permissions on GPIO devices
- Consider network isolation for the test Pi

## Test Structure

### Test Markers

Tests use pytest markers for categorization:

```python
@pytest.mark.hardware       # All hardware tests
@pytest.mark.raspberry_pi   # Raspberry Pi specific
@pytest.mark.gpio           # GPIO tests
@pytest.mark.i2c            # I2C device tests
@pytest.mark.spi            # SPI device tests
@pytest.mark.pwm            # PWM tests
@pytest.mark.servo          # Servo motor tests
@pytest.mark.motor          # DC motor tests
```

### Test Fixtures

Hardware fixtures handle setup and skip logic:

```python
def test_gpio_blink(
    require_hardware: None,    # Skips if ROBO_HARDWARE_TESTS != 1
    require_gpio: None,        # Skips if GPIO not available
    test_led_pin: int,         # Returns configured LED pin
) -> None:
    """Test that uses GPIO hardware."""
    # Test implementation
```

### Skip Decorators

Use decorators for fine-grained control:

```python
from tests.hardware import (
    skip_unless_hardware,
    requires_platform,
    requires_device,
    requires_i2c_address,
)

@skip_unless_hardware
def test_requires_hardware():
    """Only runs when ROBO_HARDWARE_TESTS=1"""

@requires_platform(PlatformType.RASPBERRY_PI)
def test_pi_only():
    """Only runs on Raspberry Pi"""

@requires_device("/dev/i2c-1")
def test_needs_i2c():
    """Only runs when I2C bus exists"""

@requires_i2c_address(1, 0x40)
def test_needs_pca9685():
    """Only runs when PCA9685 is detected"""
```

## Writing Hardware Tests

### Best Practices

1. **Always use fixtures** - Let fixtures handle skip logic
2. **Brief motor/servo operations** - Keep movements short (50-100ms)
3. **Cleanup in finally block** - Ensure hardware is reset
4. **Print useful diagnostics** - Help debug failures
5. **Test at low power first** - Use 50% speed initially

### Example Test

```python
import pytest
import time

@pytest.mark.hardware
@pytest.mark.servo
class TestServo:
    def test_servo_center(
        self,
        require_hardware: None,
        require_i2c: None,
        test_servo_channel: int,
    ) -> None:
        """Test centering servo."""
        from robo_infra.servo import Servo, PCA9685

        pca = PCA9685()
        servo = Servo(pca, channel=test_servo_channel)

        try:
            servo.center()
            time.sleep(0.1)
            assert 85 <= servo.angle <= 95
        finally:
            servo.release()
            pca.cleanup()
```

## Troubleshooting

### I2C Not Detected

```bash
# Check if I2C is enabled
ls /dev/i2c*

# Scan for devices
i2cdetect -y 1

# Check permissions
ls -la /dev/i2c-1
# Should show: crw-rw---- root i2c

# Add user to i2c group
sudo usermod -aG i2c $USER
# Log out and back in
```

### GPIO Permission Denied

```bash
# Check gpio group membership
groups $USER

# Add to gpio group
sudo usermod -aG gpio $USER

# Or use RPi.GPIO with sudo (not recommended)
sudo python3 test_script.py
```

### Motor Not Moving

1. Check power supply (motors need separate power)
2. Verify wiring matches pin configuration
3. Check PWM duty cycle (try 100% first)
4. Verify motor driver enable pin is HIGH

### PCA9685 Not Responding

1. Check I2C wiring (SDA, SCL, VCC, GND)
2. Verify address: `i2cdetect -y 1`
3. Check power (VCC should be 3.3V or 5V)
4. Ensure no address conflict with other devices
