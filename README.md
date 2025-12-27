# robo-infra

**Universal robotics infrastructure for any hardware, any project.**

[![PyPI](https://img.shields.io/pypi/v/robo-infra)](https://pypi.org/project/robo-infra/)
[![CI](https://github.com/nfraxlab/robo-infra/workflows/CI/badge.svg)](https://github.com/nfraxlab/robo-infra/actions)
[![Python](https://img.shields.io/badge/python-3.11%2B-blue)](https://www.python.org)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

## Overview

`robo-infra` is a hardware-agnostic robotics infrastructure library designed to work with **any** robotics project—from hobby servos to industrial robots, from quadcopters to autonomous vehicles.

### Key Features

- **🔌 Universal Hardware Abstraction** - Works with any servo, motor, sensor, or controller
- **🎮 Simulation-First** - Everything works without hardware by default
- **🤖 AI-Native** - Built-in integration with `ai-infra` for LLM-controlled robots
- **🌐 API-Ready** - Seamless integration with `svc-infra` for REST/WebSocket APIs
- **🛡️ Safety-First** - Comprehensive limits, emergency stops, and collision detection
- **📊 Observable** - Full telemetry, logging, and monitoring built-in

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Your Application                        │
├─────────────────────────────────────────────────────────────┤
│  Controllers    │  Motion Planning  │  Safety Systems       │
├─────────────────────────────────────────────────────────────┤
│      Actuators (Servos, Motors)  │  Sensors (IMU, Distance) │
├─────────────────────────────────────────────────────────────┤
│                    Hardware Drivers                          │
├─────────────────────────────────────────────────────────────┤
│     Buses (I2C, SPI, UART)    │    Pins (GPIO, PWM, ADC)    │
├─────────────────────────────────────────────────────────────┤
│  Platforms: Raspberry Pi │ Arduino │ Jetson │ Simulation    │
└─────────────────────────────────────────────────────────────┘
```

## Installation

### Core Only (Minimal)

For embedded systems or when you just need robotics abstractions:

```bash
pip install robo-infra
```

### With AI Integration

For LLM-controlled robots using `ai-infra`:

```bash
pip install robo-infra[ai]
```

### With API Integration

For REST/WebSocket APIs using `svc-infra`:

```bash
pip install robo-infra[api]
```

### With Hardware Support

For real hardware buses (I2C, SPI, Serial, CAN):

```bash
pip install robo-infra[hardware]
```

### Full Installation

Install everything:

```bash
pip install robo-infra[full]
```

### Platform-Specific Bundles

| Platform | Command |
|----------|---------|
| Raspberry Pi | `pip install robo-infra[raspberry-pi]` |
| NVIDIA Jetson | `pip install robo-infra[jetson]` |
| Generic Linux | `pip install robo-infra[hardware]` |

### Granular Hardware Extras

| Feature | Extra | Command |
|---------|-------|---------|
| I2C sensors | `[i2c]` | `pip install robo-infra[i2c]` |
| SPI devices | `[spi]` | `pip install robo-infra[spi]` |
| Serial/UART | `[serial]` | `pip install robo-infra[serial]` |
| CAN bus | `[can]` | `pip install robo-infra[can]` |
| GPIO | `[gpio]` | `pip install robo-infra[gpio]` |

### Vision & Camera Extras

| Feature | Extra | Command |
|---------|-------|---------|
| OpenCV | `[vision]` | `pip install robo-infra[vision]` |
| Intel RealSense | `[realsense]` | `pip install robo-infra[realsense]` |
| Luxonis OAK | `[oak]` | `pip install robo-infra[oak]` |
| Raspberry Pi Camera | `[picamera]` | `pip install robo-infra[picamera]` |
| All cameras | `[cameras]` | `pip install robo-infra[cameras]` |

## Quick Start

```python
from robo_infra.actuators import Servo
from robo_infra.core.types import Limits

# Create a servo (works in simulation by default)
servo = Servo(
    name="gripper",
    channel=0,
    limits=Limits(min_value=0, max_value=180),
)

# Move to position
await servo.move_to(90)

# Get current position
print(f"Position: {servo.position}°")
```

### With Real Hardware

```python
from robo_infra.drivers import PCA9685Driver
from robo_infra.actuators import Servo

# Initialize hardware driver
driver = PCA9685Driver(i2c_address=0x40)

# Attach servo to driver
servo = Servo(
    name="gripper",
    driver=driver,
    channel=0,
)

await servo.move_to(90)
```

### AI Integration

```python
from ai_infra import Agent
from robo_infra.actuators import Servo
from robo_infra.integrations import RobotTools

# Create robot components
servo = Servo(name="arm", channel=0)

# Export as AI tools
tools = RobotTools([servo])

# Use with AI agent
agent = Agent(tools=tools.as_tools())
await agent.run("Move the arm to 45 degrees")
```

### API Integration

```python
from svc_infra import create_app
from robo_infra.actuators import Servo
from robo_infra.integrations import RobotRouter

# Create robot components
servo = Servo(name="arm", channel=0)

# Export as API router
router = RobotRouter([servo])

# Add to app
app = create_app()
app.include_router(router.as_router(), prefix="/robot")
```

## Core Concepts

### Actuators

Physical components that create movement:

- **Servo** - Position-controlled rotational actuators
- **Motor** - Speed-controlled rotational actuators
- **Stepper** - Precise step-based motors
- **LinearActuator** - Linear motion actuators

### Sensors

Components that measure the environment:

- **IMU** - Inertial measurement (accelerometer, gyroscope)
- **Distance** - Distance measurement (ultrasonic, ToF, IR)
- **Temperature** - Temperature sensors
- **Current** - Current sensing for motor feedback

### Drivers

Hardware interfaces:

- **PCA9685** - 16-channel PWM driver
- **ADS1115** - 4-channel ADC
- **MCP23017** - 16-bit I/O expander

### Controllers

High-level control systems:

- **ArmController** - Robotic arm coordination
- **DriveController** - Differential/holonomic drive
- **FlightController** - Quadcopter/drone control

## Documentation

- [Getting Started](docs/getting-started.md)
- [Hardware Setup](docs/hardware-setup.md)
- [API Reference](docs/api-reference.md)
- [Examples](examples/)

## Part of the nfrax Ecosystem

`robo-infra` is designed to work seamlessly with:

- **[svc-infra](https://github.com/nfraxlab/svc-infra)** - Backend infrastructure (API, auth, database)
- **[ai-infra](https://github.com/nfraxlab/ai-infra)** - AI/LLM infrastructure (agents, tools, embeddings)

## License

MIT License - see [LICENSE](LICENSE) for details.
