# Quickstart

> From zero to a simulated robot in 5 minutes.

## 1. Install

```bash
pip install robo-infra
```

## 2. Create a Simulated Servo

```python
from robo_infra.actuators import Servo
from robo_infra.core.types import Limits

# No hardware needed - runs in simulation mode automatically
servo = Servo(
    name="shoulder",
    channel=0,
    limits=Limits(min_value=0, max_value=180),
)

# Move the servo
await servo.move_to(90)
print(f"Position: {servo.position} degrees")
```

Done. You have a simulated servo.

---

## 3. Add a Control Loop

```python
import asyncio
from robo_infra.actuators import Servo
from robo_infra.core.types import Limits

servo = Servo(
    name="shoulder",
    channel=0,
    limits=Limits(min_value=0, max_value=180),
)

async def control_loop():
    """Simple control loop that sweeps the servo."""
    position = 0
    direction = 1
    
    while True:
        await servo.move_to(position)
        print(f"Position: {position}")
        
        position += direction * 10
        if position >= 180 or position <= 0:
            direction *= -1
        
        await asyncio.sleep(0.1)  # 10 Hz loop

# Run for 5 seconds
async def main():
    task = asyncio.create_task(control_loop())
    await asyncio.sleep(5)
    task.cancel()

asyncio.run(main())
```

---

## 4. Add Safety Limits

```python
from robo_infra.actuators import Servo
from robo_infra.core.types import Limits
from robo_infra.safety import SafetyMonitor

servo = Servo(
    name="shoulder",
    channel=0,
    limits=Limits(min_value=30, max_value=150),  # Restricted range
    speed_limit=60.0,  # Max 60 degrees/second
)

monitor = SafetyMonitor()
monitor.add_device(servo)

# This is clamped to the limit
await servo.move_to(200)
print(f"Position: {servo.position}")  # 150, not 200
```

---

## 5. Read a Sensor

```python
from robo_infra.sensors import Encoder

# Simulated encoder
encoder = Encoder(
    name="wheel_encoder",
    channel=0,
    counts_per_revolution=1024,
)

position = encoder.read()
print(f"Encoder: {position} counts")
print(f"Angle: {encoder.angle} degrees")
```

---

## 6. Complete Example

```python
import asyncio
from robo_infra.actuators import Servo
from robo_infra.sensors import Encoder
from robo_infra.core.types import Limits
from robo_infra.safety import SafetyMonitor

# Create servo with safety limits
servo = Servo(
    name="arm",
    channel=0,
    limits=Limits(min_value=0, max_value=180),
    speed_limit=90.0,
)

# Create encoder for feedback
encoder = Encoder(name="arm_encoder", channel=0)

# Setup safety monitoring
monitor = SafetyMonitor()
monitor.add_device(servo)

async def main():
    # Move to positions
    for target in [45, 90, 135, 90, 45, 0]:
        await servo.move_to(target)
        print(f"Servo: {servo.position}, Encoder: {encoder.read()}")
        await asyncio.sleep(0.5)

asyncio.run(main())
```

---

## Force Simulation Mode

If you're on a Raspberry Pi but want to test without hardware:

```bash
ROBO_SIMULATION=true python my_robot.py
```

Or in code:

```python
import os
os.environ["ROBO_SIMULATION"] = "1"

from robo_infra.actuators import Servo
# Now runs in simulation even on real hardware
```

---

## Next Steps

- [Getting Started](getting-started.md) - Full guide with hardware setup
- [Controllers](controllers.md) - Coordinate multiple actuators
- [Safety](safety.md) - E-stop and safety systems
- [Hardware Setup](hardware-setup.md) - Connect real hardware
