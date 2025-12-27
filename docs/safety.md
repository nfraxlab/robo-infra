# Safety

Safety is the highest priority in robotics. This guide covers the safety systems in robo-infra that protect people, equipment, and the environment.

## Safety Philosophy

**SAFETY IS NOT OPTIONAL.** Every robot must have:

1. **Physical E-Stop** - A big red button that cuts power
2. **Software E-Stop** - Immediate disable of all actuators
3. **Limits** - Prevent dangerous positions, speeds, and forces
4. **Watchdog** - Detect frozen or crashed control loops
5. **Monitoring** - Watch for overcurrent, overtemperature, etc.

> ⚠️ **Never trust software alone.** Always have a physical E-stop accessible.

```python
from robo_infra.safety import (
    EStop, EStopState,
    LimitEnforcer,
    Watchdog,
    SafetyMonitor,
)
```

---

## Limits System

### Position Limits

Every actuator should have position limits:

```python
from robo_infra.actuators import Servo

# Define limits in constructor
servo = Servo(
    name="arm_joint",
    channel=0,
    angle_range=(0, 180),  # Hard limits
)

# Servo will clamp commands to these limits
servo.set(200)  # Clamped to 180
```

### LimitEnforcer

For advanced limit enforcement including velocity, acceleration, and jerk:

```python
from robo_infra.safety.limits import LimitEnforcer, EnforcerConfig

# Create enforcer
enforcer = LimitEnforcer(
    position_limits=(0, 180),      # degrees
    velocity_limit=90.0,           # deg/s
    acceleration_limit=180.0,      # deg/s²
    jerk_limit=1000.0,             # deg/s³
)

# In control loop
while True:
    raw_target = get_target()
    safe_target = enforcer.enforce(raw_target, current_position)
    actuator.set(safe_target)
```

### Soft Limits vs Hard Limits

```python
config = EnforcerConfig(
    position_min=0,
    position_max=180,
    soft_limit_margin=0.1,  # 10% margin for warning
    # Soft limits: 18° and 162° (warn before hard limit)
)

enforcer = LimitEnforcer(config=config)

# Violation callbacks
def on_violation(violation):
    if violation.violation_type == LimitViolationType.SOFT_LIMIT:
        logger.warning(f"Approaching limit: {violation}")
    else:
        logger.error(f"Hard limit hit: {violation}")

enforcer.add_callback(on_violation)
```

### Limit Violation Callbacks

```python
from robo_infra.safety.limits import LimitViolationType

def handle_violation(violation):
    print(f"Violation: {violation.violation_type}")
    print(f"Requested: {violation.requested}")
    print(f"Enforced: {violation.enforced}")
    print(f"Limit: {violation.limit}")

enforcer.add_callback(handle_violation)

# Get violation history
violations = enforcer.get_violations()
print(f"Total violations: {enforcer.violation_count}")
```

---

## Emergency Stop (E-Stop)

The E-stop is the most critical safety system. When triggered, it immediately disables ALL registered actuators.

### Creating an E-Stop

```python
from robo_infra.safety import EStop, EStopConfig

# Basic E-stop
estop = EStop()

# Configured E-stop
config = EStopConfig(
    name="MainEStop",
    require_reset_confirmation=True,  # Reset must be called twice
    log_all_triggers=True,
    propagate_errors=True,
    max_disable_attempts=3,
)
estop = EStop(config=config)
```

### Registering Actuators

```python
from robo_infra.actuators import Servo, DCMotor

# Create actuators
servo1 = Servo(name="joint1", channel=0)
servo2 = Servo(name="joint2", channel=1)
motor = DCMotor(name="drive", pin_a=17, pin_b=18)

# Register with E-stop
estop.register_actuator(servo1)
estop.register_actuator(servo2)
estop.register_actuator(motor)

# Or register multiple at once
estop.register_all([servo1, servo2, motor])

# Check registered actuators
print(estop.registered_actuators)
```

### Triggering E-Stop

```python
# In an emergency
estop.trigger("User pressed E-stop button")

# All registered actuators are now disabled
print(estop.state)  # EStopState.TRIGGERED

# Check if E-stop is active
if estop.is_triggered:
    print("E-stop is active, robot is safe")
```

### Resetting E-Stop

```python
# After resolving the issue
try:
    estop.reset()
except Exception as e:
    print(f"Reset failed: {e}")

# With confirmation requirement (if configured)
estop.reset()  # First call - sets pending
estop.reset()  # Second call - actually resets

# Check state
print(estop.state)  # EStopState.ARMED (ready for operation)
```

### E-Stop Callbacks

```python
def on_estop(event):
    print(f"E-STOP TRIGGERED: {event.reason}")
    print(f"Time: {event.timestamp}")
    print(f"Disabled: {event.actuators_disabled}")
    if event.actuators_failed:
        print(f"FAILED TO DISABLE: {event.actuators_failed}")
        # CRITICAL: Manual intervention required!

estop.add_callback(on_estop)
```

### Hardware E-Stop Integration

Connect a physical E-stop button:

```python
from robo_infra.sensors import DigitalInput

# Hardware E-stop button (normally closed)
estop_button = DigitalInput(pin=27, pull_up=True)

def check_hardware_estop():
    """Check hardware E-stop in control loop."""
    if not estop_button.read():  # Button pressed (NC = low when pressed)
        estop.trigger("Hardware E-stop button pressed")

# In control loop
while True:
    check_hardware_estop()
    # ... rest of control loop
```

---

## Watchdog Timer

The watchdog ensures the control loop is running and responsive. If not "fed" within the timeout, it triggers an E-stop.

### Creating a Watchdog

```python
from robo_infra.safety import Watchdog, WatchdogConfig

# Quick setup
watchdog = Watchdog(timeout=0.1, estop=estop)  # 100ms timeout

# Full configuration
config = WatchdogConfig(
    name="ControlLoopWatchdog",
    timeout=0.1,           # 100ms timeout
    trigger_estop=True,    # Trigger E-stop on timeout
    auto_start=False,
    warn_threshold=0.8,    # Warn at 80% of timeout
)
watchdog = Watchdog(config=config, estop=estop)
```

### Heartbeat Monitoring

```python
# Start watchdog before control loop
watchdog.start()

try:
    while running:
        # Control loop code
        sensor_data = read_sensors()
        commands = compute_control(sensor_data)
        send_commands(commands)
        
        # Feed the watchdog - MUST be called every 100ms
        watchdog.feed()
        
        time.sleep(0.01)  # 100 Hz
finally:
    watchdog.stop()
```

### Timeout Configuration

Choose timeout based on your application:

| Application | Recommended Timeout |
|-------------|---------------------|
| High-speed servo control | 10-50 ms |
| General robot control | 50-200 ms |
| Slow, safe operations | 500-1000 ms |
| Remote teleoperation | 1000-5000 ms |

```python
# Fast control loop
watchdog = Watchdog(timeout=0.05, estop=estop)  # 50ms

# Slow, deliberate motion
watchdog = Watchdog(timeout=0.5, estop=estop)  # 500ms
```

### Auto-Disable on Timeout

```python
def on_timeout(status):
    print(f"WATCHDOG TIMEOUT!")
    print(f"Last feed: {status.last_feed_age:.3f}s ago")
    print(f"Timeout count: {status.timeout_count}")

watchdog.add_callback(on_timeout)

# When timeout occurs:
# 1. Callback is called
# 2. E-stop is triggered (if configured)
# 3. All actuators are disabled
```

### Pausing the Watchdog

For operations that legitimately take a long time:

```python
# Pause during homing (which is slow)
watchdog.pause()
try:
    arm.home()  # Takes several seconds
finally:
    watchdog.resume()
    watchdog.feed()  # Immediately feed after resume
```

---

## Safety Monitor

Continuously monitors safety-critical values like current and temperature.

### Creating a Monitor

```python
from robo_infra.safety import SafetyMonitor, LimitConfig, SafetyLevel

monitor = SafetyMonitor(
    estop=estop,
    check_interval=0.01,  # 100 Hz monitoring
)
```

### Current Monitoring

```python
# Add current limit for a motor
monitor.add_limit(LimitConfig(
    component="motor1",
    metric="current",
    unit="A",
    max_value=5.0,        # Max 5A
    warning_max=4.0,      # Warn at 4A
    level=SafetyLevel.CRITICAL,
))

# In control loop, update measured value
monitor.update_current("motor1", measured_current)

# Or use generic update
monitor.update("motor1", "current", measured_current)
```

### Temperature Monitoring

```python
# Add temperature limit
monitor.add_limit(LimitConfig(
    component="motor1",
    metric="temperature",
    unit="°C",
    max_value=80.0,       # Max 80°C
    warning_max=70.0,     # Warn at 70°C
    hysteresis=5.0,       # Must drop to 75°C to clear warning
    level=SafetyLevel.CRITICAL,
))

# Update temperature
monitor.update_temperature("motor1", measured_temp)
```

### Alert Thresholds

```python
# Different severity levels
monitor.add_limit(LimitConfig(
    component="battery",
    metric="voltage",
    unit="V",
    min_value=10.0,       # Critical low voltage
    warning_min=11.0,     # Warning threshold
    level=SafetyLevel.CRITICAL,
))

# Notice level (log only, no E-stop)
monitor.add_limit(LimitConfig(
    component="cpu",
    metric="temperature",
    unit="°C",
    max_value=85.0,
    level=SafetyLevel.NOTICE,  # Just log
))

# Warning level (log + callback, no E-stop)
monitor.add_limit(LimitConfig(
    component="enclosure",
    metric="humidity",
    unit="%",
    max_value=80.0,
    level=SafetyLevel.WARNING,  # Callback but no E-stop
))
```

### Starting the Monitor

```python
# Start monitoring thread
monitor.start()

try:
    while running:
        # Control loop
        motor_current = current_sensor.read()
        motor_temp = temp_sensor.read()
        
        monitor.update("motor1", "current", motor_current)
        monitor.update("motor1", "temperature", motor_temp)
        
        time.sleep(0.01)
finally:
    monitor.stop()
```

### Getting Status

```python
# Check overall status
status = monitor.get_status()
print(f"State: {status.state}")
print(f"Total violations: {status.total_violations}")

# Check specific limit
limit_status = monitor.get_limit_status("motor1", "current")
print(f"Current value: {limit_status.current_value}A")
print(f"Is violated: {limit_status.is_violated}")
print(f"Is warning: {limit_status.is_warning}")
```

---

## Best Practices

### 1. Always Have Physical E-Stop Accessible

Software can fail. **Always** have a physical emergency stop:

- Big, red, easily reachable button
- Cuts power to motors (not just signals)
- Stays engaged until manually reset
- Test it regularly

### 2. Test Safety Systems First

Before running any motion code:

```python
# Test E-stop
print("Testing E-stop...")
estop.trigger("Test trigger")
assert estop.state == EStopState.TRIGGERED
estop.reset()
estop.reset()  # If confirmation required
assert estop.state == EStopState.ARMED
print("E-stop test passed")

# Test limits
print("Testing limits...")
enforcer = LimitEnforcer(position_limits=(0, 100))
assert enforcer.enforce(150, 50) == 100
assert enforcer.enforce(-50, 50) == 0
print("Limits test passed")

# Test watchdog
print("Testing watchdog...")
watchdog = Watchdog(timeout=0.5, estop=estop)
watchdog.start()
time.sleep(0.1)
watchdog.feed()
watchdog.stop()
print("Watchdog test passed")
```

### 3. Limit Movement Speed During Development

Start slow, increase speed only after testing:

```python
# Development mode - slow and safe
DEV_MODE = os.getenv("DEV_MODE", "true").lower() == "true"

if DEV_MODE:
    max_velocity = 10.0   # Very slow
    max_accel = 20.0
else:
    max_velocity = 100.0  # Production speed
    max_accel = 200.0

enforcer = LimitEnforcer(
    position_limits=(0, 180),
    velocity_limit=max_velocity,
    acceleration_limit=max_accel,
)
```

### 4. Never Run Untested Code Near People

```python
# Safety checklist before running
def safety_check():
    checks = [
        ("E-stop accessible", check_estop_accessible()),
        ("Area clear", check_area_clear()),
        ("Limits configured", check_limits_configured()),
        ("Watchdog enabled", watchdog.is_armed),
        ("Test mode disabled", not TEST_MODE),
    ]
    
    for check_name, passed in checks:
        if not passed:
            raise SafetyError(f"Safety check failed: {check_name}")
    
    print("All safety checks passed")

# Run safety check before operation
safety_check()
```

### 5. Use Simulation Mode for Development

```bash
# Enable simulation mode
export ROBO_SIMULATION=true

# Run your code safely
python my_robot_code.py
```

```python
# Check if in simulation
from robo_infra.core import is_simulation

if is_simulation():
    print("Running in simulation mode - no real hardware")
```

---

## Complete Safety Example

```python
from robo_infra.safety import EStop, Watchdog, SafetyMonitor, LimitEnforcer
from robo_infra.actuators import Servo
from robo_infra.sensors import CurrentSensor, TemperatureSensor
import time

# Create E-stop
estop = EStop()

# Create actuators
servo = Servo(name="arm", channel=0, angle_range=(0, 180))
estop.register_actuator(servo)

# Create limit enforcer
enforcer = LimitEnforcer(
    position_limits=(0, 180),
    velocity_limit=90.0,
    acceleration_limit=180.0,
)

# Create watchdog
watchdog = Watchdog(timeout=0.1, estop=estop)

# Create safety monitor
monitor = SafetyMonitor(estop=estop)
monitor.add_current_limit("arm", max_current=2.0, warning=1.5)
monitor.add_temperature_limit("arm", max_temp=60.0, warning=50.0)

# Sensors
current_sensor = CurrentSensor(channel=0)
temp_sensor = TemperatureSensor(channel=0)

# Start safety systems
servo.enable()
watchdog.start()
monitor.start()

try:
    while True:
        # Check E-stop
        if estop.is_triggered:
            print("E-stop active, waiting for reset...")
            time.sleep(0.1)
            continue
        
        # Get target from somewhere
        raw_target = get_target()
        
        # Enforce limits
        safe_target = enforcer.enforce(raw_target, servo.position)
        
        # Send command
        servo.set(safe_target)
        
        # Update safety monitor
        monitor.update("arm", "current", current_sensor.read())
        monitor.update("arm", "temperature", temp_sensor.read())
        
        # Feed watchdog
        watchdog.feed()
        
        time.sleep(0.01)

except Exception as e:
    estop.trigger(f"Exception: {e}")
    raise

finally:
    watchdog.stop()
    monitor.stop()
    servo.disable()
```

---

## Testing Safety Systems

Safety systems require thorough testing. This section covers testing patterns, fixtures, and coverage recommendations.

### Test Coverage Targets

The safety module maintains high test coverage to ensure reliability:

| Module | Coverage Target | Current |
|--------|-----------------|---------|
| estop.py | ≥90% | 98% |
| limits.py | ≥90% | 100% |
| monitor.py | ≥80% | 99% |
| watchdog.py | ≥85% | 100% |
| **Overall** | ≥80% | **99%** |

Run coverage reports with:

```bash
pytest --cov=src/robo_infra/safety tests/unit/test_safety_*.py --cov-report=term-missing
```

### Test Fixture Patterns

#### Mock Actuator Fixture

```python
from unittest.mock import MagicMock
import pytest

@pytest.fixture
def mock_actuator():
    """Create a mock actuator for E-stop testing."""
    actuator = MagicMock()
    actuator.name = "test_actuator"
    actuator.is_enabled = True
    return actuator

def test_estop_disables_actuator(mock_actuator):
    estop = EStop()
    estop.register_actuator(mock_actuator)
    
    estop.trigger("test")
    
    mock_actuator.disable.assert_called_once()
```

#### Watchdog Test Fixture

```python
@pytest.fixture
def watchdog():
    """Create a watchdog with short timeout for testing."""
    wd = Watchdog(timeout=0.05)  # 50ms for fast tests
    yield wd
    wd.stop()  # Cleanup

def test_watchdog_timeout(watchdog):
    watchdog.start()
    time.sleep(0.15)  # Wait for timeout
    assert watchdog.state == WatchdogState.TRIGGERED
```

#### Monitor Test Fixture

```python
@pytest.fixture
def monitor():
    """Create a safety monitor for testing."""
    m = SafetyMonitor(check_interval=0.01)
    m.add_limit(LimitConfig(
        component="test",
        metric="value",
        unit="units",
        max_value=100.0,
        warning_max=80.0,
    ))
    yield m
    m.stop()

def test_monitor_detects_violation(monitor):
    monitor.update("test", "value", 150.0)
    status = monitor.get_status("test", "value")
    assert status.is_violated
```

### Testing Patterns

#### Testing State Transitions

```python
def test_estop_state_transitions():
    """Verify E-stop state machine."""
    estop = EStop()
    
    # Initial state
    assert estop.state == EStopState.ARMED
    
    # Trigger
    estop.trigger("test")
    assert estop.state == EStopState.TRIGGERED
    
    # Reset
    estop.reset()
    assert estop.state == EStopState.ARMED
```

#### Testing Callbacks

```python
def test_callback_receives_correct_data():
    """Verify callback receives proper event data."""
    received_events = []
    
    def capture_callback(event):
        received_events.append(event)
    
    estop = EStop()
    estop.add_callback(capture_callback)
    estop.trigger("test reason")
    
    assert len(received_events) == 1
    assert received_events[0].reason == "test reason"
```

#### Testing Error Handling

```python
def test_callback_failure_doesnt_break_estop():
    """E-stop works even if callback fails."""
    def bad_callback(event):
        raise RuntimeError("Callback error")
    
    estop = EStop()
    estop.add_callback(bad_callback)
    
    # Should not raise, E-stop is safety-critical
    estop.trigger("test")
    
    assert estop.is_triggered
```

#### Testing Thread Safety

```python
def test_concurrent_feeds():
    """Watchdog handles concurrent feeds safely."""
    watchdog = Watchdog(timeout=1.0)
    watchdog.start()
    
    try:
        def feed_loop():
            for _ in range(100):
                watchdog.feed()
                time.sleep(0.001)
        
        threads = [threading.Thread(target=feed_loop) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        assert watchdog.status().feed_count == 500
    finally:
        watchdog.stop()
```

### Running Specific Test Categories

```bash
# E-stop tests only
pytest tests/unit/test_safety_estop.py -v

# Limits tests only
pytest tests/unit/test_safety_limits.py -v

# Monitor tests only
pytest tests/unit/test_safety_monitor.py -v

# Watchdog tests only
pytest tests/unit/test_safety_watchdog.py -v

# All safety tests with coverage
pytest tests/unit/test_safety_*.py --cov=src/robo_infra/safety
```

---

## Next Steps

- [Vision](vision.md) - Computer vision for safety sensing
- [Actuators](actuators.md) - Actuator limits and safety
- [Controllers](controllers.md) - Controller safety patterns
- [Drivers](drivers.md) - Driver-level safety features
