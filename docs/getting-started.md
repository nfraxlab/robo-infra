# Getting Started with robo-infra

This guide will help you get started with robo-infra, from installation to your first robotic project.

## Prerequisites

- Python 3.11 or higher
- Poetry (recommended) or pip

## Installation

### Using Poetry (Recommended)

```bash
# Clone the repository
git clone https://github.com/nfraxlab/robo-infra.git
cd robo-infra

# Install with Poetry
poetry install

# Activate the virtual environment
poetry shell
```

### Using pip

```bash
# Core package only
pip install robo-infra

# With hardware support
pip install robo-infra[hardware]

# With all optional dependencies
pip install robo-infra[all]
```

## Your First Robot

Let's create a simple robot arm with one servo.

### Step 1: Create a Servo (Simulation Mode)

```python
from robo_infra.actuators import Servo
from robo_infra.core.types import Limits

# Create a servo in simulation mode (no hardware needed)
servo = Servo(
    name="shoulder",
    channel=0,
    limits=Limits(min_value=0, max_value=180),
)

# Move to a position
await servo.move_to(90)
print(f"Servo position: {servo.position}°")
```

### Step 2: Add Safety Limits

```python
from robo_infra.actuators import Servo
from robo_infra.core.types import Limits
from robo_infra.safety import SafetyMonitor

# Create servo with strict limits
servo = Servo(
    name="shoulder",
    channel=0,
    limits=Limits(min_value=30, max_value=150),  # Restricted range
    speed_limit=60.0,  # Max 60°/second
)

# Add safety monitoring
monitor = SafetyMonitor()
monitor.add_device(servo)

# This will be clamped to the limits
await servo.move_to(200)  # Actually moves to 150°
```

### Step 3: Connect to Real Hardware

```python
from robo_infra.drivers import PCA9685Driver
from robo_infra.actuators import Servo

# Initialize the PWM driver
driver = PCA9685Driver(
    i2c_address=0x40,
    frequency=50,  # Standard servo frequency
)

# Create servo attached to the driver
servo = Servo(
    name="shoulder",
    driver=driver,
    channel=0,
)

# Now movements control real hardware
await servo.move_to(90)
```

## Platform Detection

robo-infra automatically detects your platform:

```python
from robo_infra.platforms import detect_platform, PlatformType

platform_type = detect_platform()

if platform_type == PlatformType.RASPBERRY_PI:
    print("Running on Raspberry Pi")
elif platform_type == PlatformType.SIMULATION:
    print("Running in simulation mode")
```

## Cross-Platform Hardware Access

Use the factory functions for portable code across all platforms:

```python
from robo_infra.platforms import get_gpio, get_i2c, get_spi
from robo_infra.core.pin import PinMode

# GPIO - works on any platform
led = get_gpio(17, mode=PinMode.OUTPUT)
led.setup()
led.high()

# I2C - auto-detects platform
i2c = get_i2c(bus=1)
devices = i2c.scan()
print(f"Found I2C devices at: {[hex(d) for d in devices]}")

# SPI - with speed configuration
spi = get_spi(bus=0, device=0, speed_hz=1_000_000)
response = spi.transfer([0x01, 0x02, 0x03])
```

### List Available Hardware

```python
from robo_infra.platforms import (
    list_available_gpio,
    list_available_i2c,
    list_available_spi,
)

# Discover what's available on this platform
print(f"GPIO chips: {list_available_gpio()}")
print(f"I2C buses: {list_available_i2c()}")
print(f"SPI buses: {list_available_spi()}")
```

## Hardware Setup

For connecting real hardware like servos, motors, and sensors:

```python
from robo_infra.platforms import get_i2c, get_gpio
from robo_infra.drivers import PCA9685Driver
from robo_infra.actuators import Servo

# Get platform-appropriate I2C bus
i2c = get_i2c(bus=1)

# Initialize the PWM driver
driver = PCA9685Driver(
    i2c_bus=i2c,
    address=0x40,
    frequency=50,
)

# Create servo attached to the driver
servo = Servo(
    name="shoulder",
    driver=driver,
    channel=0,
)

# Move the servo
await servo.move_to(90)
```

## Simulation Mode

Develop without hardware using simulation mode:

```bash
# Force simulation via environment variable
ROBO_SIMULATION=true python my_robot.py

# Or specific platform simulation
ROBO_PLATFORM=simulation python my_robot.py
```

```python
# Programmatically check if simulating
from robo_infra.platforms.detection import is_simulation_mode

if is_simulation_mode():
    print("Running in simulation - no real hardware")
```

## Next Steps

- [Hardware Setup Guide](hardware-setup.md) - Complete hardware setup tutorial
- [Hardware Wiring](hardware-wiring.md) - Wiring diagrams
- [Hardware Testing](hardware-testing.md) - Test your hardware setup
- [Building a Robot Arm](examples/robot-arm.md) - Complete arm tutorial
- [AI Integration](ai-integration.md) - Control robots with LLMs
- [API Integration](api-integration.md) - Add REST/WebSocket APIs
