# Migration Guide

This guide documents breaking changes and migration paths between versions of robo-infra.

## Version Compatibility

| robo-infra | Python | svc-infra | ai-infra | Notes |
|------------|--------|-----------|----------|-------|
| 0.1.x | 3.11+ | >=0.1.0 | >=0.1.0 | Current alpha |
| 0.2.x (planned) | 3.11+ | >=0.2.0 | >=0.2.0 | Stable release |

## Migrating to 0.1.x

### From Direct Hardware Libraries

If you're migrating from direct Adafruit/RPi.GPIO usage:

#### Servo Control

```python
# Before (direct Adafruit)
from adafruit_pca9685 import PCA9685
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
pca.channels[0].duty_cycle = 0x7FFF  # 50% = ~90 degrees

# After (robo-infra)
from robo_infra import Servo

servo = Servo(
    channel=0,
    limits=Limits(min=0, max=180),
)
servo.set(90)  # Degrees, with limit enforcement
```

#### Motor Control

```python
# Before (direct GPIO)
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.output(17, GPIO.HIGH)  # Forward
GPIO.output(18, GPIO.LOW)

# After (robo-infra)
from robo_infra import DCMotor

motor = DCMotor(
    forward_pin=17,
    reverse_pin=18,
)
motor.forward(speed=0.5)  # With speed control
```

#### Multi-Joint Control

```python
# Before (manual coordination)
servos = [PCA9685().channels[i] for i in range(4)]
for i, angle in enumerate([45, 90, 90, 45]):
    set_servo_angle(servos[i], angle)  # No coordination

# After (robo-infra)
from robo_infra import JointGroup, Servo

arm = JointGroup([
    Servo(i, limits=Limits(0, 180))
    for i in range(4)
])
arm.move_to([45, 90, 90, 45])  # Coordinated movement
```

## Critical Migration Notes

### Safety Limits

**All actuators MUST have limits:**

```python
# Before (no limits - dangerous)
servo.set_angle(angle)  # Could be 9999!

# After (limits enforced)
from robo_infra import Servo, Limits

servo = Servo(
    channel=0,
    limits=Limits(min=0, max=180),  # REQUIRED
)
servo.set(9999)  # Clamped to 180 with warning
```

### Emergency Stop

**E-stop must be first priority:**

```python
# Before (cleanup can block e-stop)
def emergency_stop():
    save_state()  # Could hang!
    disable_motors()

# After (disable first)
def emergency_stop(self):
    self.disable_all()  # FIRST
    try:
        self.save_state()
    except Exception:
        pass  # Log after stopping
```

### Explicit Simulation

**No silent fallback to simulation:**

```python
# Before (confusing)
driver = PCA9685() or SimulatedDriver()  # Which one?

# After (explicit)
import os
if not os.getenv("ROBO_SIMULATION"):
    raise HardwareNotFoundError("Set ROBO_SIMULATION=true")
```

## Planned Breaking Changes (0.2.x)

### Async-First API

```python
# 0.1.x (sync)
servo.set(90)

# 0.2.x (planned - async for hardware I/O)
await servo.set(90)
```

### Driver Auto-Detection

```python
# 0.1.x (explicit driver)
servo = Servo(channel=0, driver=PCA9685Driver())

# 0.2.x (planned - auto-detect)
servo = Servo(channel=0)  # Detects PCA9685 on I2C bus
```

### Controller Composition

```python
# 0.1.x
arm = JointGroup([Servo(i) for i in range(4)])
gripper = Gripper(Servo(4))

# 0.2.x (planned - robot composition)
robot = Robot(
    arm=JointGroup([Servo(i) for i in range(4)]),
    gripper=Gripper(Servo(4)),
)
```

## Deprecation Notices

### 0.1.5+

- `SimulatedDriver` requires `ROBO_SIMULATION` env var
- Old GPIO-direct APIs deprecated

### 0.2.0+

- Sync APIs deprecated (use async)
- `driver=` parameter may become optional (auto-detect)

## Testing During Migration

Always test in simulation first:

```bash
# Run all tests in simulation mode
ROBO_SIMULATION=true pytest -q

# Then carefully test on hardware
pytest -q tests/integration/
```

## Getting Help

- Check the [error handling guide](error-handling.md) for exception changes
- Test in simulation before hardware
- Open an issue for migration questions
- See [CONTRIBUTING.md](../CONTRIBUTING.md) for contribution guidelines
