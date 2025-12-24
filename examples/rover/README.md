# Differential Drive Rover Example

A wheeled rover using differential drive (tank-style steering) with DC motors.

## Overview

This example demonstrates:
- Basic rover control with `DifferentialDrive` controller
- Motor control with `DCMotor` actuators
- Adding sensors (ultrasonic distance) for obstacle detection
- Simple obstacle avoidance behavior

## Requirements

```bash
# Core requirement
pip install robo-infra
```

## Files

| File | Description |
|------|-------------|
| `rover.py` | Basic differential drive rover control |
| `rover_with_sensors.py` | Rover with ultrasonic sensor for obstacle avoidance |

## Running the Examples

### Basic Rover Control

```bash
python rover.py
```

This demonstrates:
- Creating DC motor actuators for left and right wheels
- Building a DifferentialDrive controller
- Driving forward, backward, turning, and spinning
- Getting speed and position information
- Emergency stop

### Rover with Sensors

```bash
python rover_with_sensors.py
```

This demonstrates:
- Adding an ultrasonic distance sensor
- Reading distance measurements
- Simple obstacle avoidance logic
- Sensor status monitoring

## Rover Configuration

The example rover uses a typical small robot configuration:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Wheel Diameter | 65mm | Standard hobby robot wheel |
| Track Width | 150mm | Distance between wheel centers |
| Max Speed | 1.0 m/s | Maximum linear velocity |

## Motor Configuration

Each wheel is driven by a DC motor:

| Motor | Direction | Speed Range |
|-------|-----------|-------------|
| Left | Forward/Reverse | -1.0 to 1.0 |
| Right | Forward/Reverse | -1.0 to 1.0 |

## Drive Commands

The DifferentialDrive controller provides these commands:

| Command | Description |
|---------|-------------|
| `forward(speed)` | Drive forward at given speed |
| `backward(speed)` | Drive backward at given speed |
| `turn_left(speed)` | Turn left (arc turn) |
| `turn_right(speed)` | Turn right (arc turn) |
| `spin_left(speed)` | Spin in place counter-clockwise |
| `spin_right(speed)` | Spin in place clockwise |
| `stop()` | Stop all motion |
| `drive(linear, angular)` | Arcade-style drive with linear and angular velocity |

## Sensor Configuration

The ultrasonic sensor example uses:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Range | 20mm - 4000mm | Detection range |
| Unit | Millimeters | Distance unit |
| Noise | ±5mm | Simulated measurement noise |

## Real Hardware

To use with real hardware, connect DC motors to an H-bridge driver:

```python
from robo_infra.actuators import DCMotor
from robo_infra.drivers import L298N

# Create L298N dual H-bridge driver
driver = L298N()
driver.connect()
driver.enable()

# Create motors with driver channels
left_motor = DCMotor(
    name="left_wheel",
    pin_a=0, pin_b=1, enable=2,
    driver=driver,
)
right_motor = DCMotor(
    name="right_wheel",
    pin_a=3, pin_b=4, enable=5,
    driver=driver,
)
```

For ultrasonic sensors:

```python
from robo_infra.sensors import Ultrasonic
from robo_infra.drivers import GPIODriver

# Create GPIO driver
gpio = GPIODriver()
gpio.connect()

# Create ultrasonic sensor with trigger/echo pins
sensor = Ultrasonic(
    trigger_pin=gpio.get_digital_pin(23),
    echo_pin=gpio.get_digital_pin(24),
    name="front_distance",
)
```

## Learning More

- **robo-infra docs**: Controllers, actuators, and sensors
- **ai-infra docs**: Use nfrax-docs MCP with query `"ai-infra Agent tools"`
- **svc-infra docs**: Use nfrax-docs MCP with query `"svc-infra router_from_object"`
