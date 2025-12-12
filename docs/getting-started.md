# Getting Started with robo-infra

This guide will help you get started with robo-infra, from installation to your first robotic project.

## Prerequisites

- Python 3.11 or higher
- Poetry (recommended) or pip

## Installation

### Using Poetry (Recommended)

```bash
# Clone the repository
git clone https://github.com/nfraxio/robo-infra.git
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
from robo_infra.platforms import detect_platform, Platform

platform = detect_platform()

if platform == Platform.RASPBERRY_PI:
    print("Running on Raspberry Pi")
elif platform == Platform.SIMULATION:
    print("Running in simulation mode")
```

## Next Steps

- [Hardware Setup Guide](hardware-setup.md) - Connect real hardware
- [Building a Robot Arm](examples/robot-arm.md) - Complete arm tutorial
- [AI Integration](ai-integration.md) - Control robots with LLMs
- [API Integration](api-integration.md) - Add REST/WebSocket APIs
