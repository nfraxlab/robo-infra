# Command Line Interface

robo-infra provides a command-line interface for system information, hardware discovery, and development utilities.

## Overview

The CLI provides:

- Version and system information
- Driver and platform listing
- Hardware discovery (coming soon)
- Simulation mode
- Development utilities

## Installation

The CLI is installed automatically with robo-infra:

```bash
pip install robo-infra

# Verify installation
robo-infra version
```

## Commands

### robo-infra version

Display the installed version:

```bash
$ robo-infra version
robo-infra version 0.1.0
```

Aliases: `--version`, `-v`

```bash
robo-infra --version
robo-infra -v
```

### robo-infra help

Show available commands and usage:

```bash
$ robo-infra help
robo-infra - Universal Robotics Infrastructure

Commands:
  robo-infra version          - Show version
  robo-infra help             - Show this help message
  robo-infra info             - Show system information
  robo-infra list drivers     - List available drivers
  robo-infra list platforms   - List supported platforms
  robo-infra discover         - Discover connected hardware
  robo-infra test             - Run hardware tests
  robo-infra simulate         - Run in simulation mode

For programmatic use, import robo_infra in Python:

  from robo_infra import Servo, DCMotor, JointGroup
  servo = Servo(channel=0)
  servo.angle = 90
```

Aliases: `--help`, `-h`

### robo-infra info

Display system information:

```bash
$ robo-infra info
robo-infra System Information
========================================
Version: 0.1.0
Python: 3.11.5 (main, Sep 11 2023, 13:54:46) [GCC 11.4.0]
Platform: linux
Available drivers: 15
Available platforms: 6
```

### robo-infra list drivers

List all available hardware drivers:

```bash
$ robo-infra list drivers
Available Drivers:
------------------------------
  - arduino
  - bmi270
  - bno055
  - dynamixel
  - gpio
  - icm20948
  - l298n
  - lsm6ds3
  - odrive
  - pca9685
  - simulation
  - step_dir
  - tb6612
  - tmc2209
  - vesc

Total: 15 drivers
```

### robo-infra list platforms

List supported hardware platforms:

```bash
$ robo-infra list platforms
Supported Platforms:
------------------------------
  - arduino
  - beaglebone
  - esp32
  - jetson
  - linux_generic
  - raspberry_pi

Total: 6 platforms
```

### robo-infra discover

Discover connected hardware (coming in Phase 9):

```bash
$ robo-infra discover
Hardware discovery not yet implemented.
Coming in Phase 9!
```

When implemented, this will:
- Scan I2C buses for connected devices
- Detect USB serial devices
- Identify GPIO configurations
- List available PWM channels

### robo-infra test

Run hardware tests:

```bash
$ robo-infra test
Hardware tests not yet implemented.
```

### robo-infra simulate

Run in simulation mode:

```bash
$ robo-infra simulate
Simulation mode enabled.

In simulation mode, all hardware operations are mocked.
Use this for development and testing without real hardware.

To use simulation in your code:

  from robo_infra.drivers import SimulationDriver
  driver = SimulationDriver()
```

## Driver Listing

### Available Drivers

| Driver | Description | Use Case |
|--------|-------------|----------|
| `arduino` | Arduino serial driver | Microcontroller communication |
| `bmi270` | BMI270 IMU driver | Inertial measurement |
| `bno055` | BNO055 IMU driver | 9-axis IMU |
| `dynamixel` | Dynamixel servo driver | Smart servos |
| `gpio` | GPIO driver | Digital I/O |
| `icm20948` | ICM-20948 IMU driver | 9-axis IMU |
| `l298n` | L298N motor driver | DC motor control |
| `lsm6ds3` | LSM6DS3 IMU driver | 6-axis IMU |
| `odrive` | ODrive motor driver | Brushless motors |
| `pca9685` | PCA9685 PWM driver | PWM/servo control |
| `simulation` | Simulation driver | Testing without hardware |
| `step_dir` | Step/direction driver | Stepper motors |
| `tb6612` | TB6612FNG driver | DC motor control |
| `tmc2209` | TMC2209 stepper driver | Silent stepper motors |
| `vesc` | VESC motor driver | Brushless motors |

### Using Drivers Programmatically

```python
from robo_infra.cli import AVAILABLE_DRIVERS, AVAILABLE_PLATFORMS

# List drivers in code
print("Drivers:", AVAILABLE_DRIVERS)
print("Platforms:", AVAILABLE_PLATFORMS)
```

## Platform Listing

### Supported Platforms

| Platform | Description |
|----------|-------------|
| `arduino` | Arduino boards (via serial) |
| `beaglebone` | BeagleBone Black/Green |
| `esp32` | ESP32 microcontrollers |
| `jetson` | NVIDIA Jetson (Nano, Xavier, Orin) |
| `linux_generic` | Generic Linux systems |
| `raspberry_pi` | Raspberry Pi (all models) |

## Configuration

### Environment Variables

Configure robo-infra behavior via environment variables:

| Variable | Description | Default |
|----------|-------------|---------|
| `ROBO_LOG_LEVEL` | Logging level | `INFO` |
| `ROBO_LOG_FORMAT` | Log format (`plain`, `json`) | `plain` |
| `ROBO_SIMULATION` | Enable simulation mode | `false` |
| `ROS2_MOCK` | Mock ROS2 for testing | `false` |
| `ROBO_I2C_BUS` | Default I2C bus number | `1` |
| `ROBO_SPI_BUS` | Default SPI bus number | `0` |

Example:

```bash
# Enable debug logging
export ROBO_LOG_LEVEL=DEBUG

# Use JSON log format
export ROBO_LOG_FORMAT=json

# Run in simulation mode
export ROBO_SIMULATION=true

robo-infra info
```

### Config Files

robo-infra looks for configuration in these locations (in order):

1. `./robo.toml` - Project-local config
2. `~/.config/robo-infra/config.toml` - User config
3. `/etc/robo-infra/config.toml` - System config

Example `robo.toml`:

```toml
[logging]
level = "INFO"
format = "json"

[hardware]
i2c_bus = 1
spi_bus = 0
simulation = false

[safety]
enable_limits = true
estop_gpio = 17

[ros2]
mock = false
namespace = "/robot"
```

## Programmatic Usage

### Main Entrypoint

The CLI can be invoked programmatically:

```python
from robo_infra.cli import main
import sys

# Run with arguments
exit_code = main(["version"])
print(f"Exit code: {exit_code}")

# Run info command
main(["info"])

# List drivers
main(["list", "drivers"])
```

### Exit Codes

| Code | Meaning |
|------|---------|
| `0` | Success |
| `1` | Error (invalid command, etc.) |

### Integration with Scripts

```python
#!/usr/bin/env python3
"""Robot initialization script."""

import subprocess
import sys

def check_robo_infra():
    """Verify robo-infra installation."""
    result = subprocess.run(
        ["robo-infra", "version"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print("ERROR: robo-infra not installed")
        sys.exit(1)
    print(f"Found: {result.stdout.strip()}")

def list_available_drivers():
    """Get list of available drivers."""
    from robo_infra.cli import AVAILABLE_DRIVERS
    return AVAILABLE_DRIVERS

if __name__ == "__main__":
    check_robo_infra()
    drivers = list_available_drivers()
    print(f"Available drivers: {len(drivers)}")
```

## Development Commands

### Running Tests

```bash
# Run all tests
pytest tests/

# Run with coverage
pytest tests/ --cov=robo_infra

# Run specific test file
pytest tests/test_cli.py
```

### Type Checking

```bash
# Run mypy
mypy src/robo_infra

# Strict mode
mypy src/robo_infra --strict
```

### Linting

```bash
# Run ruff
ruff check src/robo_infra

# Auto-fix issues
ruff check src/robo_infra --fix
```

## Shell Completion

### Bash

Add to `~/.bashrc`:

```bash
eval "$(_ROBO_INFRA_COMPLETE=bash_source robo-infra)"
```

### Zsh

Add to `~/.zshrc`:

```zsh
eval "$(_ROBO_INFRA_COMPLETE=zsh_source robo-infra)"
```

### Fish

Add to `~/.config/fish/completions/robo-infra.fish`:

```fish
eval (env _ROBO_INFRA_COMPLETE=fish_source robo-infra)
```

## Examples

### Quick System Check

```bash
#!/bin/bash
# Check robo-infra installation and system

echo "=== robo-infra System Check ==="
echo

echo "Version:"
robo-infra version
echo

echo "System Info:"
robo-infra info
echo

echo "Available Drivers:"
robo-infra list drivers
echo

echo "Supported Platforms:"
robo-infra list platforms
```

### Development Workflow

```bash
# 1. Check system
robo-infra info

# 2. Start in simulation mode
export ROBO_SIMULATION=true

# 3. Run your robot code
python my_robot.py

# 4. Switch to real hardware
unset ROBO_SIMULATION
python my_robot.py
```

### CI/CD Integration

```yaml
# .github/workflows/test.yml
name: Test

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      
      - name: Install robo-infra
        run: pip install robo-infra[dev]
      
      - name: Verify installation
        run: robo-infra version
      
      - name: Run tests (simulation mode)
        run: |
          export ROBO_SIMULATION=true
          pytest tests/
```

## Troubleshooting

### Command Not Found

```bash
$ robo-infra
command not found: robo-infra
```

**Solution**: Ensure robo-infra is installed and in your PATH:

```bash
pip install robo-infra
# Or if using pipx
pipx install robo-infra
```

### Permission Errors

```bash
$ robo-infra discover
PermissionError: [Errno 13] Permission denied: '/dev/i2c-1'
```

**Solution**: Add your user to the required groups:

```bash
# For I2C access
sudo usermod -a -G i2c $USER

# For GPIO access
sudo usermod -a -G gpio $USER

# Log out and back in for changes to take effect
```

### Import Errors

```bash
$ robo-infra info
ImportError: No module named 'robo_infra'
```

**Solution**: Verify installation:

```bash
pip show robo-infra
pip install --upgrade robo-infra
```

## See Also

- [Architecture](architecture.md) - System architecture overview
- [Drivers](drivers.md) - Detailed driver documentation
- [Core Concepts](core-concepts.md) - Fundamental concepts
