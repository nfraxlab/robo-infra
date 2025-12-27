# Utilities

robo-infra provides utility modules for building robust, production-ready robotics applications.

## Overview

The utilities module includes:

- **Resilience** - Retry, circuit breaker, timeout handling
- **Graceful Degradation** - Continue operating when components fail
- **Hardware Utilities** - Bus detection, device scanning, simulation
- **Resource Management** - Connection pooling, cleanup handlers
- **Security** - Input validation, privilege management

## Resilience

Utilities for handling transient failures in hardware communication.

### Retry Decorator

Retry operations that may fail transiently:

```python
from robo_infra.utils.resilience import with_retry
from robo_infra.core.exceptions import CommunicationError

# Retry on communication errors
@with_retry(max_attempts=3, retry_on=(CommunicationError,))
async def read_sensor():
    """Read sensor with automatic retry."""
    return await driver.read()

# With exponential backoff
@with_retry(
    max_attempts=5,
    retry_on=(CommunicationError, OSError),
    initial_delay=0.1,
    max_delay=2.0,
    exponential_base=2,
)
async def connect_to_driver():
    """Connect with exponential backoff."""
    return await driver.connect()
```

### Circuit Breaker

Prevent cascading failures with circuit breakers:

```python
from robo_infra.utils.resilience import (
    CircuitBreaker,
    CircuitBreakerError,
    create_driver_circuit_breaker,
)

# Create circuit breaker for motor driver
circuit = create_driver_circuit_breaker(
    "motor-driver",
    failure_threshold=5,      # Open after 5 failures
    recovery_timeout=30.0,    # Try recovery after 30s
)

# Use as context manager
async def send_command(cmd):
    try:
        async with circuit:
            return await driver.send(cmd)
    except CircuitBreakerError:
        # Circuit is open - driver is unavailable
        raise DriverUnavailableError("Motor driver circuit open")
```

#### Pre-configured Circuit Breakers

```python
from robo_infra.utils.resilience import (
    create_driver_circuit_breaker,
    create_sensor_circuit_breaker,
)

# For motor/actuator drivers (strict)
motor_circuit = create_driver_circuit_breaker(
    "motor",
    failure_threshold=5,       # Low threshold - hardware failures are serious
    recovery_timeout=30.0,     # Long timeout - hardware may need reset
)

# For sensors (tolerant)
sensor_circuit = create_sensor_circuit_breaker(
    "temperature",
    failure_threshold=10,      # Higher threshold - glitches are common
    recovery_timeout=10.0,     # Shorter timeout - try again sooner
)
```

### Timeout Handling

Enforce timeouts on operations:

```python
from robo_infra.utils.resilience import with_timeout, run_with_timeout
from robo_infra.core.exceptions import TimeoutError

# Context manager style
async def move_with_timeout():
    try:
        async with with_timeout(5.0, "move"):
            await actuator.move(target)
    except TimeoutError as e:
        print(f"Move timed out after {e.timeout}s")
        actuator.stop()

# Function style
result = await run_with_timeout(
    sensor.read(),
    timeout=1.0,
    operation="sensor_read",
)
```

## Graceful Degradation

Continue operating when individual components fail.

### DegradedModeController

Wrap controllers for graceful degradation:

```python
from robo_infra.utils.degraded import DegradedModeController

# Wrap the controller
controller = JointGroup("arm", joints=[joint1, joint2, joint3])
degraded = DegradedModeController(controller)

# When a component fails, mark it degraded
try:
    await controller.move({"joint2": 90.0})
except CommunicationError:
    degraded.mark_actuator_degraded("joint2", "Lost communication")

# Subsequent moves skip degraded components
result = await degraded.move({
    "joint1": 45.0,
    "joint2": 90.0,  # Will be skipped
    "joint3": 60.0,
})

print(result.is_degraded)        # True
print(result.skipped_targets)    # {"joint2": 90.0}
```

### Degradation Limits

Set limits on how many components can be degraded:

```python
# Refuse operation if more than 50% degraded
degraded = DegradedModeController(
    controller,
    max_degraded_ratio=0.5,
)

# Mark multiple components degraded
degraded.mark_actuator_degraded("joint1", "Fault")
degraded.mark_actuator_degraded("joint2", "Timeout")

# This will raise RuntimeError if > 50% degraded
try:
    await degraded.move({"joint3": 45.0})
except RuntimeError as e:
    print("Too many components failed")
```

### Restoring Components

Restore components after recovery:

```python
# Restore a single component
degraded.restore_actuator("joint2")

# Restore all components
count = degraded.restore_all()
print(f"Restored {count} components")

# Check status
print(degraded.is_degraded)          # False
print(degraded.degraded_actuators)   # set()
```

### Sensor Degradation

Handle sensor failures with fallback values:

```python
# Mark sensor degraded with last known value
degraded.mark_sensor_degraded(
    "temperature",
    "Sensor not responding",
    last_value=45.2,
)

# Read sensor (returns last known value if degraded)
temp = degraded.read_sensor("temperature")
```

## Hardware Utilities

Utilities for hardware detection and simulation.

### Simulation Configuration

Configure realistic simulation behavior:

```python
from robo_infra.utils.hardware import SimulationConfig, FailureMode

# Basic simulation
config = SimulationConfig(
    delay=0.001,          # 1ms I/O latency
    noise=0.02,           # ±2% noise on readings
    failure_rate=0.001,   # 0.1% failure rate
)

# With physics simulation
config = SimulationConfig(
    delay=0.001,
    physics_enabled=True,
    acceleration=100.0,   # units/s²
    max_velocity=50.0,    # units/s
)

# With specific failure mode
config = SimulationConfig(
    failure_rate=0.01,
    failure_mode=FailureMode.TIMEOUT,  # Simulate timeouts
)
```

### Using Simulation

Apply simulation configuration to drivers:

```python
# Apply noise to readings
value = config.apply_noise(sensor_reading)

# Add delay with jitter
await asyncio.sleep(config.get_delay())

# Check for simulated failure
if config.should_fail():
    config.simulate_failure("read_operation")
```

### Hardware Probing

Detect connected hardware:

```python
from robo_infra.utils.hardware import HardwareProbe

probe = HardwareProbe(timeout=2.0, retries=3)

# Check I2C device
if probe.check_i2c_device(bus=1, address=0x40):
    print("PCA9685 found at 0x40")

# Scan I2C bus
devices = probe.scan_i2c_bus(bus=1)
for addr in devices:
    print(f"Device at 0x{addr:02X}")

# Check serial port
if probe.check_serial_port("/dev/ttyUSB0", baudrate=115200):
    print("Serial device responding")
```

### Platform Detection

Detect the current hardware platform:

```python
from robo_infra.utils.hardware import detect_platform, Platform

platform = detect_platform()

if platform == Platform.RASPBERRY_PI:
    print("Running on Raspberry Pi")
elif platform == Platform.JETSON:
    print("Running on NVIDIA Jetson")
elif platform == Platform.LINUX_GENERIC:
    print("Running on generic Linux")
```

## Resource Management

Manage hardware resources and cleanup.

### Managed Resources

Base class for resources needing lifecycle management:

```python
from robo_infra.utils.resources import ManagedResource

class I2CDevice(ManagedResource):
    def __init__(self, bus: int, address: int):
        super().__init__(name=f"i2c-{bus}-{address:02x}")
        self._bus = bus
        self._address = address
    
    def _open(self) -> None:
        # Open I2C connection
        self._handle = smbus2.SMBus(self._bus)
    
    def _close(self) -> None:
        # Close I2C connection
        self._handle.close()

# Use as context manager
with I2CDevice(1, 0x40) as device:
    device.write(0x00, [0x01])

# Or async context manager
async with I2CDevice(1, 0x40) as device:
    await device.write_async(0x00, [0x01])
```

### Connection Pooling

Share connections across components:

```python
from robo_infra.utils.resources import ConnectionPool, PoolConfig

# Create pool for I2C bus
pool = ConnectionPool.get_or_create(
    "i2c_1",
    factory=lambda: smbus2.SMBus(1),
    closer=lambda bus: bus.close(),
    config=PoolConfig(
        max_size=10,
        min_size=1,
        acquire_timeout=5.0,
        idle_timeout=300.0,
    ),
)

# Acquire connection from pool
async with pool.acquire() as bus:
    data = bus.read_i2c_block_data(0x40, 0x00, 2)
```

### Cleanup Handlers

Register cleanup handlers for graceful shutdown:

```python
from robo_infra.utils.resources import (
    register_cleanup,
    ResourceManager,
)

# Register cleanup function
register_cleanup(controller.emergency_stop)
register_cleanup(driver.disconnect)

# Cleanup runs on:
# - Normal exit (atexit)
# - SIGTERM/SIGINT signals
# - Explicit call to ResourceManager.cleanup_all()

# Manual cleanup
ResourceManager.cleanup_all()
```

### Async Context Manager Mixin

Add async context manager support to classes:

```python
from robo_infra.utils.resources import AsyncContextManager

class MyDriver(Driver, AsyncContextManager):
    async def connect_async(self) -> None:
        await self._init_hardware()
    
    async def disconnect_async(self) -> None:
        await self._shutdown_hardware()

# Use with async context manager
async with MyDriver() as driver:
    await driver.do_something()
```

## Security

Input validation and privilege management.

### Joint Angle Validation

Validate joint angles within safe limits:

```python
from robo_infra.utils.security import (
    validate_joint_angle,
    validate_joint_angles,
    JointLimits,
    ValidationError,
)

# Validate single angle
try:
    validate_joint_angle(
        angle=1.5,
        min_angle=-3.14,
        max_angle=3.14,
        joint_name="shoulder",
    )
except ValidationError as e:
    print(f"Invalid: {e}")

# Validate multiple angles with limits
limits = [
    JointLimits(-1.57, 1.57, "base"),
    JointLimits(-2.0, 0.5, "shoulder"),
    JointLimits(-2.5, 2.5, "elbow"),
]

validate_joint_angles([0.5, -0.3, 1.0], limits)
```

### Speed and Acceleration Validation

```python
from robo_infra.utils.security import (
    validate_speed,
    validate_acceleration,
    SpeedLimits,
)

# Validate speed
validate_speed(
    speed=5.0,
    min_speed=0.0,
    max_speed=10.0,
    name="joint_velocity",
)

# Validate acceleration
validate_acceleration(
    accel=50.0,
    min_accel=0.0,
    max_accel=100.0,
)
```

### I2C and CAN Address Validation

```python
from robo_infra.utils.security import (
    validate_i2c_address,
    validate_can_id,
)

# Validate I2C address (7-bit)
validate_i2c_address(0x40)
validate_i2c_address(0x00, allow_reserved=True)  # Allow reserved

# Validate CAN ID
validate_can_id(0x100)                    # Standard 11-bit
validate_can_id(0x1FFFFFFF, extended=True)  # Extended 29-bit
```

### Serial Port Validation

```python
from robo_infra.utils.security import validate_port_name

# Validate port name (platform-specific)
validate_port_name("/dev/ttyUSB0", platform="linux")
validate_port_name("/dev/cu.usbserial-1410", platform="darwin")
validate_port_name("COM3", platform="windows")

# Rejects path traversal attempts
try:
    validate_port_name("../etc/passwd")
except ValidationError:
    print("Invalid port name")
```

### Name Sanitization

Sanitize user-provided names:

```python
from robo_infra.utils.security import sanitize_name

# Valid names
sanitize_name("robot_1")      # OK
sanitize_name("my-robot")     # OK
sanitize_name("Arm2")         # OK

# Invalid names (raise ValidationError)
sanitize_name("../etc/passwd")  # Path traversal
sanitize_name("robot;ls")       # Shell metachar
sanitize_name("")               # Empty
sanitize_name("1robot")         # Must start with letter
```

### Serial Command Sanitization

Prevent command injection:

```python
from robo_infra.utils.security import sanitize_serial_command

# Valid commands
sanitize_serial_command("G1 X10 Y20")
sanitize_serial_command("M104 S200")

# Invalid commands (raise ValidationError)
sanitize_serial_command("echo; rm -rf /")  # Shell injection
sanitize_serial_command("cmd\x00null")     # Null byte
```

### Privilege Checking

Check hardware access permissions:

```python
from robo_infra.utils.security import (
    check_gpio_access,
    check_i2c_access,
    check_spi_access,
    check_serial_access,
    check_can_access,
    check_all_hardware_access,
    PrivilegeError,
    get_required_groups,
)

# Check specific access
try:
    check_gpio_access()
except PrivilegeError as e:
    print(f"Need permissions: {e}")
    print(f"Fix with: {e.fix_command}")

# Check I2C access
try:
    check_i2c_access(bus=1)
except PrivilegeError as e:
    print(f"Add user to group: {e.required_group}")

# Check all hardware access at startup
try:
    check_all_hardware_access()
except PrivilegeError as e:
    print(f"Missing permissions for: {e.resource}")

# Get required groups
groups = get_required_groups()
# {'gpio': 'gpio', 'i2c': 'i2c', 'spi': 'spi', 'serial': 'dialout'}
```

## Complete Example

```python
"""Robust robot controller with utilities."""

import asyncio
from robo_infra.controllers import JointGroup
from robo_infra.actuators import Servo
from robo_infra.drivers import PCA9685Driver
from robo_infra.utils.resilience import (
    with_retry,
    with_timeout,
    create_driver_circuit_breaker,
)
from robo_infra.utils.degraded import DegradedModeController
from robo_infra.utils.resources import register_cleanup
from robo_infra.utils.security import (
    check_i2c_access,
    validate_joint_angles,
    JointLimits,
)
from robo_infra.core.exceptions import CommunicationError


async def main():
    # Check permissions before accessing hardware
    check_i2c_access(bus=1)
    
    # Setup circuit breaker
    driver_circuit = create_driver_circuit_breaker("pca9685")
    
    # Initialize hardware
    driver = PCA9685Driver()
    joints = [
        Servo("base", channel=0, driver=driver),
        Servo("shoulder", channel=1, driver=driver),
        Servo("elbow", channel=2, driver=driver),
    ]
    controller = JointGroup("arm", joints=joints)
    
    # Wrap in degraded mode controller
    degraded = DegradedModeController(controller, max_degraded_ratio=0.5)
    
    # Register cleanup
    register_cleanup(controller.stop)
    register_cleanup(driver.disconnect)
    
    # Define joint limits
    limits = [
        JointLimits(-1.57, 1.57, "base"),
        JointLimits(-2.0, 0.5, "shoulder"),
        JointLimits(-2.5, 2.5, "elbow"),
    ]
    
    # Validate target before moving
    targets = [0.5, -0.3, 1.0]
    validate_joint_angles(targets, limits)
    
    # Move with timeout and retry
    @with_retry(max_attempts=3, retry_on=(CommunicationError,))
    async def safe_move(positions):
        async with with_timeout(10.0, "move"):
            async with driver_circuit:
                return await degraded.move(positions)
    
    try:
        result = await safe_move({
            "base": targets[0],
            "shoulder": targets[1],
            "elbow": targets[2],
        })
        
        if result.is_degraded:
            print(f"Moved in degraded mode, skipped: {result.skipped_targets}")
        else:
            print("Move complete")
            
    except Exception as e:
        print(f"Move failed: {e}")
        controller.stop()


if __name__ == "__main__":
    asyncio.run(main())
```

## Best Practices

### 1. Always Validate External Input

```python
# ✅ Validate before using
positions = request.json()
for name, angle in positions.items():
    sanitize_name(name)
    validate_joint_angle(angle, min_angle=limits[name].min, max_angle=limits[name].max)

# ❌ Trust external input
await controller.move(request.json())  # DANGEROUS
```

### 2. Use Circuit Breakers for Hardware

```python
# ✅ Protect hardware communication
circuit = create_driver_circuit_breaker("motor")
async with circuit:
    await motor.move(target)

# ❌ No protection
await motor.move(target)  # Can cause cascading failures
```

### 3. Set Timeouts on All Operations

```python
# ✅ Always set timeouts
async with with_timeout(5.0, "sensor_read"):
    value = await sensor.read()

# ❌ No timeout
value = await sensor.read()  # Can hang forever
```

### 4. Register Cleanup Handlers

```python
# ✅ Register cleanup at startup
register_cleanup(controller.emergency_stop)
register_cleanup(driver.disconnect)

# ❌ Rely on try/finally everywhere
```

### 5. Use Degraded Mode for Non-Critical Components

```python
# ✅ Continue operating with reduced capability
degraded = DegradedModeController(controller)
result = await degraded.move(targets)

# ❌ Fail completely on any component failure
await controller.move(targets)  # Fails if any joint fails
```

## See Also

- [Safety](safety.md) - Safety systems and e-stop
- [Drivers](drivers.md) - Hardware driver documentation
- [Controllers](controllers.md) - Controller types
- [Observability](observability.md) - Monitoring and logging
