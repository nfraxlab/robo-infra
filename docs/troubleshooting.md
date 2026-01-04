# Troubleshooting Guide

This guide helps you diagnose and resolve common issues with robo-infra.

## Quick Diagnostics

Before diving into specific issues, run this diagnostic:

```python
from robo_infra.platforms import detect_platform, get_platform_capabilities, PlatformType

# Detect current platform
platform = detect_platform()
print(f"Platform: {platform.value}")

# Check platform capabilities
if platform != PlatformType.UNKNOWN:
    caps = get_platform_capabilities(platform)
    print(f"Capabilities: {[c.value for c in caps]}")

# Check if running in simulation mode
import os
simulation = os.getenv("ROBO_SIMULATION", "0") == "1"
print(f"Simulation mode: {simulation}")
```

---

## Hardware Not Detected Errors

### Symptoms

```
HardwareNotFoundError: Could not find device at address 0x40
CommunicationError: Failed to open I2C bus /dev/i2c-1
PlatformType.UNKNOWN detected
```

### Causes

1. Hardware not connected or powered
2. Wrong I2C/SPI address configured
3. Bus not enabled on the platform
4. Missing kernel drivers or device tree overlays

### Solutions

**1. Verify physical connections:**

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Check SPI devices
ls /dev/spidev*

# Check USB devices
lsusb
```

**2. Enable I2C/SPI on Raspberry Pi:**

```bash
# Add to /boot/config.txt
dtparam=i2c_arm=on
dtparam=spi=on

# Or use raspi-config
sudo raspi-config
# -> Interface Options -> I2C -> Enable
# -> Interface Options -> SPI -> Enable
```

**3. Enable I2C on Jetson:**

```bash
# Check if I2C is available
ls /dev/i2c-*

# Load I2C module if needed
sudo modprobe i2c-dev
```

**4. Verify correct address:**

```python
from robo_infra.core.bus import I2CBus

bus = I2CBus(bus_id=1)
bus.open()

# Scan for devices
for addr in range(0x03, 0x78):
    try:
        bus.write_byte(addr, 0)
        print(f"Device found at 0x{addr:02X}")
    except Exception:
        pass
```

**5. Use simulation mode for testing:**

```python
from robo_infra.actuators import Servo

# Servo will auto-simulate if no hardware driver
servo = Servo(name="test_servo")
servo.enable()
print(f"Simulated: {servo._simulation}")
```

---

## Permission Denied for GPIO/I2C

### Symptoms

```
PermissionError: [Errno 13] Permission denied: '/dev/i2c-1'
PermissionError: [Errno 13] Permission denied: '/dev/gpiomem'
OSError: [Errno 13] Permission denied: '/dev/spidev0.0'
```

### Causes

1. User not in required groups (gpio, i2c, spi)
2. udev rules not configured
3. Running without required privileges

### Solutions

**1. Add user to required groups:**

```bash
# Raspberry Pi
sudo usermod -aG gpio,i2c,spi $USER

# Jetson
sudo usermod -aG i2c,dialout,gpio $USER

# Log out and back in for changes to take effect
```

**2. Install udev rules (Raspberry Pi):**

```bash
# Create udev rule for GPIO
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | \
    sudo tee /etc/udev/rules.d/99-gpio.rules

# Create udev rule for I2C
echo 'SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"' | \
    sudo tee /etc/udev/rules.d/99-i2c.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**3. Use simulation mode during development:**

```python
import os
os.environ["ROBO_SIMULATION"] = "1"

from robo_infra.platforms import RaspberryPiPlatform

# Will use simulated GPIO
platform = RaspberryPiPlatform(simulation=True)
```

**4. Check current permissions:**

```bash
# Check device permissions
ls -la /dev/i2c-*
ls -la /dev/spidev*
ls -la /dev/gpiomem

# Check group membership
groups $USER
```

---

## Simulation vs Real Hardware Mode Issues

### Symptoms

```
# Actuators not moving in simulation
# Sensors returning constant values
# Platform detection returning UNKNOWN on development machine
```

### Causes

1. Code running in simulation but expecting real hardware
2. Platform not detected correctly
3. Drivers not falling back to simulation

### Solutions

**1. Explicitly set simulation mode:**

```python
from robo_infra.actuators import Servo, ServoConfig

# Force simulation
servo = Servo(
    name="arm_servo",
    config=ServoConfig(simulation=True)
)
```

**2. Check if running in simulation:**

```python
from robo_infra.actuators import Servo

servo = Servo(name="test")
servo.enable()

if hasattr(servo, '_simulation') and servo._simulation:
    print("Running in SIMULATION mode")
    print("Actuator commands will be logged but not sent to hardware")
else:
    print("Running in REAL HARDWARE mode")
```

**3. Force real hardware mode (will fail if hardware not present):**

```python
from robo_infra.actuators import Servo, ServoConfig

servo = Servo(
    name="arm_servo",
    config=ServoConfig(simulation=False)  # Will raise if no hardware
)
```

**4. Use environment variable for mode switching:**

```python
import os

# Set in environment
os.environ["ROBO_SIMULATION"] = "1"  # Enable simulation
# os.environ["ROBO_SIMULATION"] = "0"  # Disable simulation

from robo_infra.platforms import detect_platform
```

**5. Test with explicit platform:**

```python
from robo_infra.platforms import (
    RaspberryPiPlatform,
    JetsonPlatform,
    LinuxGenericPlatform,
)

# Use specific platform in simulation
platform = LinuxGenericPlatform(simulation=True)
gpio = platform.get_gpio(17)
gpio.set_mode("output")
gpio.write(True)  # Simulated write
```

---

## Safety Limit Triggered Errors

### Symptoms

```
SafetyError: Joint 'shoulder' exceeded position limit (max: 180, got: 185)
LimitsExceededError: Velocity limit exceeded: 2.5 > 2.0 rad/s
EStopError: Emergency stop triggered - all actuators disabled
```

### Causes

1. Commanded position outside joint limits
2. Velocity or acceleration limits exceeded
3. E-stop triggered by safety monitor
4. Watchdog timeout

### Solutions

**1. Check configured limits:**

```python
from robo_infra.actuators import Servo

servo = Servo(name="test")
print(f"Position limits: {servo.limits}")
print(f"Min: {servo.limits.min}, Max: {servo.limits.max}")
```

**2. Configure appropriate limits:**

```python
from robo_infra.actuators import Servo, ServoConfig
from robo_infra.core.types import Limits

servo = Servo(
    name="shoulder",
    config=ServoConfig(
        limits=Limits(min=0, max=180, default=90),
        max_velocity=1.0,  # rad/s
    )
)
```

**3. Handle E-stop events:**

```python
from robo_infra.safety import EStop, EStopConfig

estop = EStop(config=EStopConfig(name="main_estop"))

@estop.on_trigger
def handle_estop(event):
    print(f"E-stop triggered: {event.reason}")
    # Perform cleanup

@estop.on_reset
def handle_reset(event):
    print("E-stop reset, resuming operation")
```

**4. Reset from E-stop state:**

```python
from robo_infra.safety import EStop

estop = EStop()

# Check state
if estop.is_triggered:
    print(f"E-stop is active: {estop.state}")
    
    # Reset (requires intentional action)
    estop.reset()
    print("E-stop reset")
```

**5. Adjust safety monitor thresholds:**

```python
from robo_infra.safety import SafetyMonitor, LimitConfig

monitor = SafetyMonitor()
monitor.add_limit(
    name="joint_velocity",
    config=LimitConfig(
        min_value=-2.0,
        max_value=2.0,
        warning_threshold=0.9,  # Warn at 90% of limit
    )
)
```

**6. Feed the watchdog in control loops:**

```python
from robo_infra.safety import Watchdog, WatchdogConfig

watchdog = Watchdog(config=WatchdogConfig(timeout=0.1))  # 100ms timeout
watchdog.start()

try:
    while running:
        # Your control loop
        process_sensors()
        compute_control()
        send_commands()
        
        # Feed watchdog to prevent timeout
        watchdog.feed()
finally:
    watchdog.stop()
```

---

## Communication Timeout with Actuators

### Symptoms

```
CommunicationError: Timeout waiting for response from device
TimeoutError: I2C read timed out after 1.0s
SerialTimeoutError: No response from Dynamixel servo ID 1
```

### Causes

1. Hardware not powered or connected
2. Wrong baud rate or protocol settings
3. Bus contention (multiple masters)
4. Electrical noise or signal integrity issues

### Solutions

**1. Verify basic connectivity:**

```python
from robo_infra.core.bus import SimulatedI2CBus, I2CBus

# Test with simulation first
sim_bus = SimulatedI2CBus()
sim_bus.open()
print("Simulated bus OK")

# Then try real hardware
try:
    real_bus = I2CBus(bus_id=1)
    real_bus.open()
    print("Real I2C bus OK")
except Exception as e:
    print(f"Real bus failed: {e}")
```

**2. Check and adjust baud rate:**

```python
from robo_infra.drivers import DynamixelDriver

driver = DynamixelDriver(
    port="/dev/ttyUSB0",
    baudrate=1000000,  # Match servo configuration
    timeout=1.0,  # Increase timeout if needed
)
```

**3. Implement retry logic:**

```python
from robo_infra.utils import CircuitBreaker

breaker = CircuitBreaker(
    failure_threshold=3,
    recovery_timeout=5.0,
)

@breaker.protect
def read_sensor():
    return sensor.read()

# Usage
try:
    value = read_sensor()
except Exception as e:
    print(f"Sensor read failed (circuit open): {e}")
```

**4. Add connection recovery:**

```python
from robo_infra.utils import DriverReconnector

reconnector = DriverReconnector(
    driver=my_driver,
    max_retries=3,
    retry_delay=1.0,
)

# Automatic reconnection on failure
with reconnector:
    while True:
        data = my_driver.read()
        process(data)
```

**5. Check for electrical issues:**

```bash
# Check I2C bus speed (lower if getting errors)
# Add to /boot/config.txt on Raspberry Pi
dtparam=i2c_baudrate=50000  # 50kHz instead of 100kHz

# Check for pull-up resistors
# I2C requires 2.2k-10k pull-ups to 3.3V on SDA and SCL
```

**6. Use connection pooling for serial devices:**

```python
from robo_infra.utils import ConnectionPool

pool = ConnectionPool(
    factory=lambda: SerialConnection("/dev/ttyUSB0"),
    max_size=1,  # Single connection for serial
    timeout=5.0,
)

with pool.acquire() as conn:
    conn.write(command)
    response = conn.read()
```

---

## Common Error Messages Reference

| Error | Likely Cause | Quick Fix |
|-------|--------------|-----------|
| `HardwareNotFoundError` | Device not connected | Check wiring, run `i2cdetect` |
| `PermissionError` | User not in gpio/i2c group | `sudo usermod -aG gpio,i2c $USER` |
| `CommunicationError` | Bus/protocol issue | Check baud rate, wiring |
| `SafetyError` | Limit exceeded | Check limits configuration |
| `EStopError` | E-stop triggered | Call `estop.reset()` after clearing |
| `CalibrationError` | Calibration failed | Check sensor, retry calibration |
| `LimitsExceededError` | Command out of range | Clamp values to limits |
| `WatchdogError` | Control loop too slow | Optimize loop, increase timeout |

---

## Getting Help

If you're still stuck:

1. **Check the logs**: Enable debug logging with `ROBO_LOG_LEVEL=DEBUG`
2. **Run diagnostics**: Use `robo_infra.utils.HardwareProbe` to scan hardware
3. **Use simulation**: Test your logic in simulation before hardware
4. **Check examples**: See `examples/` directory for working code
5. **Open an issue**: Include platform, Python version, and full traceback
