# Error Handling Guide

This guide documents exception hierarchies and error handling patterns in robo-infra.

## ⚠️ Safety-Critical Error Handling

**Robotics errors can cause physical harm. Error handling must prioritize STOPPING safely.**

## Exception Hierarchy

```
Exception
└── RoboInfraError (base for all robo-infra exceptions)
    ├── SafetyError (CRITICAL - always stop immediately)
    │   ├── LimitsExceededError
    │   ├── EmergencyStopError
    │   ├── CollisionDetectedError
    │   └── WatchdogTimeoutError
    ├── HardwareError
    │   ├── HardwareNotFoundError
    │   ├── CommunicationError
    │   ├── I2CError
    │   ├── SPIError
    │   └── DriverInitError
    ├── ActuatorError
    │   ├── ServoError
    │   ├── MotorError
    │   └── CalibrationError
    ├── SensorError
    │   ├── SensorReadError
    │   ├── SensorTimeoutError
    │   └── InvalidReadingError
    └── ConfigurationError
        ├── InvalidLimitsError
        └── MissingDriverError
```

## Critical: Safety Error Handling

### Safety Errors ALWAYS Stop the Robot

```python
from robo_infra.exceptions import SafetyError, LimitsExceededError

try:
    actuator.set(value)
except SafetyError as e:
    # FIRST: Stop everything
    controller.emergency_stop()
    # THEN: Log
    logger.critical(f"Safety error: {e}")
    # NEVER continue after SafetyError
    raise
```

### Hardware Errors

```python
from robo_infra.exceptions import HardwareNotFoundError
import os

try:
    driver = PCA9685Driver()
except HardwareNotFoundError:
    if os.getenv("ROBO_SIMULATION"):
        logger.warning("⚠️ SIMULATION MODE - No real hardware")
        driver = SimulatedDriver()
    else:
        # Don't silently simulate!
        raise
```

### Sensor Timeout

```python
from robo_infra.exceptions import SensorTimeoutError

try:
    distance = await sensor.read(timeout=1.0)
except SensorTimeoutError:
    # Stop movement when sensors fail
    controller.stop()
    logger.error("Sensor timeout - stopping for safety")
    raise
```

## Emergency Stop Handling

**E-stop errors must NEVER be caught and ignored:**

```python
# ❌ WRONG - E-stop failure is catastrophic
try:
    controller.emergency_stop()
except Exception:
    pass  # Robot is now uncontrolled!

# ✅ CORRECT - E-stop must always work
def emergency_stop(self):
    # Disable IMMEDIATELY - no exceptions can block this
    for actuator in self.actuators:
        try:
            actuator.disable()
        except Exception as e:
            # Log but continue disabling others
            logger.error(f"Failed to disable {actuator}: {e}")
    
    # Now log the event
    logger.critical("Emergency stop activated")
```

## Control Loop Error Handling

```python
async def control_loop(self):
    try:
        while self.running:
            await self.update()
    except SafetyError:
        # Safety errors stop immediately
        await self.emergency_stop()
        raise
    except SensorError as e:
        # Sensor failures: stop and alert
        logger.error(f"Sensor failure: {e}")
        await self.stop()
        raise
    except Exception as e:
        # Unknown errors: stop for safety
        logger.critical(f"Unexpected error in control loop: {e}")
        await self.emergency_stop()
        raise
```

## Error Recovery Patterns

### Safe State on Error

```python
class RobotArm:
    async def move_to(self, position):
        try:
            await self._execute_move(position)
        except Exception:
            # Return to safe position on any error
            await self.move_to_home()
            raise
```

### Graceful Degradation

```python
async def get_distance(self):
    try:
        return await self.primary_sensor.read()
    except SensorError:
        logger.warning("Primary sensor failed, using backup")
        return await self.backup_sensor.read()
```

## Best Practices

1. **SafetyError = Stop immediately** - Never catch and continue
2. **E-stop cannot fail** - Disable first, log after
3. **Sensor failure = Stop movement** - Don't move blind
4. **Hardware not found = Explicit error** - No silent simulation
5. **Control loop errors = Safe state** - Return to home/neutral
6. **Log all errors** - But after stopping
7. **Test error paths** - Especially in simulation

## Simulation Testing

```python
# Test that errors trigger safe behavior
def test_limits_error_stops_robot():
    robot = Robot(simulation=True)
    robot.start()
    
    # Try to exceed limits
    with pytest.raises(LimitsExceededError):
        robot.arm.set_angle(9999)
    
    # Verify robot stopped
    assert robot.is_stopped
```
