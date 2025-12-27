# Core Concepts

This guide explains the fundamental patterns and concepts used throughout robo-infra. Understanding these concepts will help you build robust robotics applications.

## Controller Pattern

Controllers are the primary way to build robots in robo-infra. A controller coordinates multiple actuators and sensors as a unified system.

### Why Use Controllers?

Without controllers, you'd manage each component individually:

```python
# ❌ Manual coordination - error-prone
servo1.set(90)
servo2.set(45)
servo3.set(120)
# What if one fails? How do you home all? How do you stop safely?
```

With controllers, you get coordinated behavior:

```python
# ✅ Controller handles coordination
arm = JointGroup([servo1, servo2, servo3])
arm.enable_all()
arm.home()  # All joints move to home position
arm.move_to({"base": 90, "shoulder": 45, "elbow": 120})
arm.emergency_stop()  # All joints stop immediately
```

### Controller Lifecycle

Every controller follows a predictable lifecycle:

```
DISABLED ──► IDLE ──► HOMING ──► MOVING/RUNNING ──► STOPPED
    │           │                      │               │
    └───────────┴──────────────────────┴───────────────┘
              emergency_stop() or error
```

| State | Description |
|-------|-------------|
| `DISABLED` | Not active, no power to actuators |
| `IDLE` | Enabled but not moving |
| `HOMING` | Moving to home/reference position |
| `MOVING` | Executing a motion command |
| `RUNNING` | Continuous operation (e.g., driving) |
| `STOPPED` | Emergency stop triggered |
| `ERROR` | Fault condition detected |

### Controller Example

```python
from robo_infra.controllers import JointGroup
from robo_infra.actuators import Servo
from robo_infra.safety import EStop

# Create actuators
joints = [
    Servo(name="base", channel=0, angle_range=(0, 180)),
    Servo(name="shoulder", channel=1, angle_range=(0, 180)),
    Servo(name="elbow", channel=2, angle_range=(0, 180)),
]

# Create controller
arm = JointGroup(joints)

# Enable and home
arm.enable_all()
await arm.home()

# Move to named position
arm.add_position("ready", {"base": 90, "shoulder": 45, "elbow": 90})
await arm.move_to_position("ready")

# Execute a sequence
arm.add_sequence("pick_up", [
    {"base": 90, "shoulder": 90, "elbow": 45},   # Extend
    {"base": 90, "shoulder": 120, "elbow": 30},  # Lower
    # ... gripper close
    {"base": 90, "shoulder": 45, "elbow": 90},   # Retract
])
await arm.run_sequence("pick_up")
```

## Driver Abstraction

Drivers provide hardware independence. The same actuator code works with different hardware by swapping drivers.

### The Problem Drivers Solve

Without drivers, actuator code is tied to specific hardware:

```python
# ❌ Hardware-specific code
import smbus2
bus = smbus2.SMBus(1)
bus.write_byte_data(0x40, 0x06, pwm_low)  # PCA9685-specific
bus.write_byte_data(0x40, 0x07, pwm_high)
```

With drivers, hardware details are encapsulated:

```python
# ✅ Hardware-independent code
servo = Servo(name="gripper", driver=driver, channel=0)
servo.set(90)  # Works with any driver
```

### Driver Interface

All drivers implement the same interface:

```python
from robo_infra.core.driver import Driver

class Driver:
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def write_channel(self, channel: int, value: float) -> None: ...
    def read_channel(self, channel: int) -> float: ...
    
    @property
    def connected(self) -> bool: ...
    @property
    def channels(self) -> int: ...
```

### Swapping Drivers

Change hardware without changing application code:

```python
from robo_infra.drivers import PCA9685Driver, SimulationDriver
from robo_infra.actuators import Servo

# Development: simulation
driver = SimulationDriver(channels=16)

# Production: real hardware
# driver = PCA9685Driver(i2c_address=0x40)

# Same servo code works with both
servo = Servo(name="gripper", driver=driver, channel=0)
servo.enable()
servo.set(45)
```

### Driver Registry

Drivers can be discovered by name:

```python
from robo_infra.core.driver import get_driver, list_drivers

# List all available drivers
print(list_drivers())
# ['pca9685', 'l298n', 'simulation', 'dynamixel', ...]

# Get driver class by name
DriverClass = get_driver("pca9685")
driver = DriverClass(i2c_address=0x40)
```

## Bus System

Buses abstract the communication protocol (I2C, SPI, Serial, CAN) from the driver implementation.

### Supported Bus Types

| Bus | Use Case | Speed | Topology |
|-----|----------|-------|----------|
| I2C | Sensors, PWM drivers | 100-400 kHz | Multi-device |
| SPI | High-speed sensors, displays | 1-50 MHz | Point-to-point |
| Serial | Arduino, GPS, smart servos | 9600-3M baud | Point-to-point |
| CAN | Industrial, automotive | 125k-1M bps | Multi-device |

### Bus Factory Functions

Get the appropriate bus implementation for your platform:

```python
from robo_infra.core.bus import get_i2c, get_spi, get_serial
from robo_infra.core.can_bus import get_can

# I2C bus (auto-detects hardware or uses simulation)
i2c = get_i2c(bus_number=1)
devices = i2c.scan()  # Find connected devices

# SPI bus
spi = get_spi(bus=0, device=0, max_speed_hz=1_000_000)

# Serial port
serial = get_serial(port="/dev/ttyUSB0", baudrate=115200)

# CAN bus
can = get_can(interface="can0", bitrate=500_000)
```

### Simulated Buses

All buses have simulated implementations for testing:

```python
import os
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.core.bus import get_i2c

# Returns SimulatedI2CBus instead of real hardware
i2c = get_i2c(1)
i2c.add_device(0x40, registers={0x00: 0x01})  # Mock a device
```

## Simulation Mode

Simulation mode enables development and testing without physical hardware.

### Enabling Simulation

Three ways to enable simulation:

```python
# Option 1: Environment variable
import os
os.environ["ROBO_SIMULATION"] = "true"

# Option 2: Per-driver
from robo_infra.drivers import SimulationDriver
driver = SimulationDriver(channels=16)

# Option 3: Configuration
from robo_infra.core.config import get_config
config = get_config()
config.simulation = True
```

### Simulation Behavior

In simulation mode:

| Component | Behavior |
|-----------|----------|
| Actuators | Track state, validate limits, no hardware writes |
| Sensors | Return configurable mock values |
| Drivers | Log operations, simulate timing |
| Buses | Track reads/writes, mock device responses |

### Testing with Simulation

```python
import pytest
from robo_infra.actuators import Servo
from robo_infra.drivers import SimulationDriver

def test_servo_limits():
    driver = SimulationDriver(channels=1)
    servo = Servo(
        name="test",
        driver=driver,
        channel=0,
        angle_range=(0, 90),
    )
    servo.enable()
    
    # Try to exceed limit
    servo.set(120)
    
    # Value is clamped to limit
    assert servo.get() == 90
```

## Lifecycle Management

All components follow a consistent lifecycle pattern for safe operation.

### Enable/Disable

Every actuator and controller must be enabled before use:

```python
servo = Servo(name="gripper", channel=0)

# Will raise DisabledError
try:
    servo.set(90)
except DisabledError:
    print("Must enable first!")

# Enable before use
servo.enable()
servo.set(90)  # Now works

# Disable when done
servo.disable()
```

### Cleanup with Context Managers

Use context managers for automatic cleanup:

```python
from robo_infra.controllers import JointGroup

async with JointGroup(joints) as arm:
    arm.home()
    arm.move_to(target)
    # Automatic cleanup on exit

# Or with driver
with PCA9685Driver(0x40) as driver:
    servo = Servo(name="test", driver=driver, channel=0)
    servo.enable()
    servo.set(90)
# Driver disconnected automatically
```

### Resource Management

Controllers manage their actuators' lifecycles:

```python
arm = JointGroup([servo1, servo2, servo3])

# Enable all actuators at once
arm.enable_all()

# Disable all on emergency
arm.emergency_stop()  # Disables all, enters STOPPED state

# Clean shutdown
arm.shutdown()  # Disables all, cleans up resources
```

## Limits and Safety

Every actuator enforces configurable limits.

### Limit Types

```python
from robo_infra.core.types import Limits

limits = Limits(
    min_value=0,      # Minimum allowed value
    max_value=180,    # Maximum allowed value
    default_value=90, # Home/default position
    speed_limit=60,   # Maximum speed (units/second)
)
```

### Limit Behavior

When a command exceeds limits:

```python
servo = Servo(
    name="joint",
    channel=0,
    limits=Limits(min_value=0, max_value=90),
)
servo.enable()

# Value is clamped to limit
servo.set(120)  # Actually sets 90
print(servo.get())  # 90

# Or raise exception (configurable)
servo = Servo(
    name="joint",
    channel=0,
    limits=Limits(min_value=0, max_value=90),
    clamp_to_limits=False,  # Raise instead of clamp
)
servo.enable()
servo.set(120)  # Raises LimitsExceededError
```

### Soft vs Hard Limits

```python
from robo_infra.core.types import Limits, HardLimits

# Soft limits - enforced in software
soft = Limits(min_value=10, max_value=170)

# Hard limits - absolute physical boundaries
hard = HardLimits(min_value=0, max_value=180)

servo = Servo(
    name="joint",
    channel=0,
    limits=soft,
    hard_limits=hard,  # Never exceeded, even by calibration
)
```

## State and Status

All components expose their current state and status.

### Actuator Status

```python
from robo_infra.core.actuator import ActuatorStatus

servo = Servo(name="joint", channel=0)
status: ActuatorStatus = servo.status

print(f"State: {status.state}")        # disabled, idle, moving
print(f"Position: {status.position}")  # Current position
print(f"Target: {status.target}")      # Target position
print(f"Error: {status.error}")        # Error message if any
```

### Controller Status

```python
from robo_infra.core.controller import ControllerStatus

arm = JointGroup(joints)
status: ControllerStatus = arm.status

print(f"State: {status.state}")      # disabled, idle, homing, moving
print(f"Is homed: {status.is_homed}")
print(f"Actuators: {status.actuator_count}")
```

### Status Monitoring

```python
# Poll status
while True:
    status = arm.status
    if status.state == ControllerState.ERROR:
        handle_error(status.error)
    time.sleep(0.1)

# Or use callbacks
arm.on_state_change(lambda old, new: print(f"{old} -> {new}"))
arm.on_error(lambda e: alert_operator(e))
```

## Configuration

Components can be configured via code, files, or environment variables.

### Code Configuration

```python
from robo_infra.actuators import Servo
from robo_infra.core.types import Limits

servo = Servo(
    name="gripper",
    channel=0,
    limits=Limits(min_value=0, max_value=90),
    default_position=45,
    speed_limit=30,
)
```

### Pydantic Models

Use Pydantic for validated configuration:

```python
from robo_infra.controllers import JointGroupConfig

config = JointGroupConfig(
    name="arm",
    joints=["base", "shoulder", "elbow"],
    home_positions={"base": 90, "shoulder": 45, "elbow": 90},
    speed_limit=60.0,
)

arm = JointGroup.from_config(config)
```

### Environment Variables

Override settings via environment:

```bash
export ROBO_SIMULATION=true
export ROBO_LOG_LEVEL=DEBUG
export ROBO_DEFAULT_SPEED=30
```

## Next Steps

- [Actuators](actuators.md) - Servo, motor, and linear actuator details
- [Sensors](sensors.md) - IMU, encoder, and distance sensor details
- [Controllers](controllers.md) - Building complex robot controllers
- [Drivers](drivers.md) - Hardware driver reference
- [Safety](safety.md) - E-Stop, watchdog, and safety monitoring
