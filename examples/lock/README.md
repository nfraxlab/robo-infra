# Smart Lock Example

A servo-based smart lock for doors, safes, compartments, and access control.

## Overview

This example demonstrates:
- Basic lock control with `Lock` controller
- Servo actuator for lock mechanism
- State tracking (locked/unlocked/transitioning)
- REST API for remote lock control

## Requirements

```bash
# Core requirement
pip install robo-infra

# For API example
pip install svc-infra uvicorn
```

## Files

| File | Description |
|------|-------------|
| `lock.py` | Basic lock control with state tracking |
| `lock_with_api.py` | REST API for remote lock control |

## Running the Examples

### Basic Lock Control

```bash
python lock.py
```

This demonstrates:
- Creating a servo actuator for the lock mechanism
- Building a Lock controller with locked/unlocked positions
- Lock, unlock, and toggle operations
- State checking (is_locked, is_unlocked)
- Async lock/unlock with transition timing

### Lock API Server

```bash
python lock_with_api.py
```

Then access:
- API docs: http://localhost:8000/docs
- Health check: http://localhost:8000/healthz
- Lock status: http://localhost:8000/lock/status

API Endpoints:

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /lock/status | Get lock state |
| POST | /lock/enable | Enable the lock |
| POST | /lock/disable | Disable the lock |
| POST | /lock/lock | Lock the mechanism |
| POST | /lock/unlock | Unlock the mechanism |
| POST | /lock/toggle | Toggle lock state |

## Lock Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Locked Position | 0° | Servo angle when locked |
| Unlocked Position | 90° | Servo angle when unlocked |
| Transition Time | 0.5s | Time to transition between states |
| Position Tolerance | 1.0° | Tolerance for state detection |

## Lock States

The Lock controller tracks these states:

| State | Description |
|-------|-------------|
| `LOCKED` | Lock is in locked position |
| `UNLOCKED` | Lock is in unlocked position |
| `TRANSITIONING` | Lock is moving between positions |
| `DISABLED` | Lock is not enabled |
| `ERROR` | Lock encountered an error |

## Real Hardware

To use with real hardware, connect a servo to a PWM driver:

```python
from robo_infra.actuators import Servo
from robo_infra.controllers import Lock, LockConfig
from robo_infra.drivers import PCA9685

# Create PWM driver (I2C servo controller)
driver = PCA9685()
driver.connect()
driver.enable()

# Create servo on channel 0
servo = Servo(
    name="lock_servo",
    driver=driver,
    channel=0,
    angle_range=(0, 90),
)

# Create lock controller
config = LockConfig(
    name="door_lock",
    locked_position=0,
    unlocked_position=90,
    transition_time=0.5,
)
lock = Lock(name="door_lock", actuator=servo, config=config)
lock.enable()

# Control the lock
lock.unlock()  # Opens the lock
lock.lock()    # Closes the lock
```

## Security Considerations

For production use:
- Use `auth_required=True` in `controller_to_router()` for API endpoints
- Add proper authentication middleware (svc-infra provides this)
- Log all lock/unlock events for audit trail
- Consider auto-lock timeout for security

```python
# Secure lock API (production)
from robo_infra.integrations.svc_infra import controller_to_router

router = controller_to_router(
    lock,
    prefix="/lock",
    tags=["Lock Control"],
    auth_required=True,  # Requires authentication
)
```

## Learning More

- **robo-infra docs**: Lock controller, Servo actuator
- **ai-infra docs**: Use nfrax-docs MCP with query `"ai-infra Agent tools"`
- **svc-infra docs**: Use nfrax-docs MCP with query `"svc-infra easy_service_app"`
