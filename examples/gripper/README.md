# Gripper Example

A servo-based gripper for pick-and-place operations, claws, and end effectors.

## Overview

This example demonstrates:
- Basic gripper control with `Gripper` controller
- Servo actuator for gripper mechanism
- Open, close, and partial position control
- State tracking (open/closed/gripping/moving)

## Requirements

```bash
# Core requirement
pip install robo-infra
```

## Files

| File | Description |
|------|-------------|
| `gripper.py` | Basic gripper control with position control |

## Running the Examples

### Basic Gripper Control

```bash
python gripper.py
```

This demonstrates:
- Creating a servo actuator for the gripper
- Building a Gripper controller with open/closed positions
- Open and close operations
- Partial position control (percentage-based)
- State checking (is_open, is_closed, is_gripping)

## Gripper Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Open Position | 0° | Servo angle when fully open |
| Closed Position | 90° | Servo angle when fully closed |
| Grip Threshold | 1.0° | Tolerance for state detection |
| Default Speed | 1.0 | Default movement speed (0-1) |
| Grip Speed | 0.5 | Slower speed for gripping |

## Gripper States

The Gripper controller tracks these states:

| State | Description |
|-------|-------------|
| `OPEN` | Gripper is fully open |
| `CLOSED` | Gripper is fully closed |
| `GRIPPING` | Gripper has detected an object |
| `MOVING` | Gripper is transitioning |
| `DISABLED` | Gripper is not enabled |
| `ERROR` | Gripper encountered an error |

## Commands

| Method | Description |
|--------|-------------|
| `open()` | Open to fully open position |
| `close()` | Close to fully closed position |
| `set(position)` | Move to specific position |
| `grip()` | Close until force detected or fully closed |
| `release()` | Open and clear grip state |
| `stop()` | Stop movement immediately |

## Real Hardware

To use with real hardware, connect a servo to a PWM driver:

```python
from robo_infra.actuators import Servo
from robo_infra.controllers import Gripper, GripperConfig
from robo_infra.drivers import PCA9685

# Create PWM driver
driver = PCA9685()
driver.connect()
driver.enable()

# Create servo on channel 0
servo = Servo(
    name="gripper_servo",
    driver=driver,
    channel=0,
    angle_range=(0, 90),
)

# Create gripper controller
config = GripperConfig(
    name="parallel_gripper",
    open_position=0,
    closed_position=90,
    default_speed=1.0,
)
gripper = Gripper(name="gripper", actuator=servo, config=config)
gripper.enable()

# Control
gripper.open()    # Open fully
gripper.close()   # Close fully
gripper.set(45)   # Move to 50% (45 degrees)
```

## With Force Sensor

Add a force sensor for grip detection:

```python
from robo_infra.controllers import Gripper, GripperConfig
from robo_infra.sensors import AnalogSensor

# Create force sensor
force_sensor = AnalogSensor(
    name="grip_force",
    channel=0,
    unit="N",
)

# Gripper with force sensing
config = GripperConfig(
    name="gripper",
    open_position=0,
    closed_position=90,
    grip_force_threshold=0.5,  # Newtons
)
gripper = Gripper(
    name="gripper",
    actuator=servo,
    config=config,
    force_sensor=force_sensor,
)

# Grip with force detection
gripper.grip()  # Closes until force threshold detected
print(f"Gripping object: {gripper.is_gripping}")
```

## Learning More

- **robo-infra docs**: Gripper controller, Servo actuator
- **ai-infra docs**: Use nfrax-docs MCP with query `"ai-infra Agent tools"`
- **svc-infra docs**: Use nfrax-docs MCP with query `"svc-infra controller_to_router"`
