# Actuators

Actuators are devices that convert electrical signals into physical motion. robo-infra provides a comprehensive set of actuator types for robotics applications.

## Overview

All actuators extend the base `Actuator` class and share common patterns:

- **Enable/disable lifecycle** - Must be enabled before use
- **Limit enforcement** - Values clamped to configured limits
- **Driver abstraction** - Works with any compatible driver
- **Simulation support** - Full functionality without hardware

```python
from robo_infra.actuators import Servo

# Create actuator
servo = Servo(name="gripper", channel=0, angle_range=(0, 90))

# Enable before use
servo.enable()

# Control the actuator
servo.set(45)
print(servo.get())  # 45.0

# Disable when done
servo.disable()
```

## Servo

Servo motors provide precise angular positioning via PWM signals. They're ideal for robot joints, grippers, and camera mounts.

### Constructor Parameters

```python
from robo_infra.actuators import Servo

servo = Servo(
    name="shoulder",              # Human-readable name
    driver=driver,                # Optional: hardware driver
    channel=0,                    # Driver channel number
    angle_range=(0, 180),         # Min/max angle in degrees
    pulse_range=(500, 2500),      # PWM pulse width in microseconds
    frequency=50,                 # PWM frequency in Hz
    inverted=False,               # Invert direction
    offset=0.0,                   # Angle offset for calibration
    speed_deg_per_sec=60.0,       # Speed limit (optional)
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | `str` | `"Servo"` | Human-readable name |
| `driver` | `Driver` | `None` | Hardware driver instance |
| `channel` | `int` | `0` | Driver channel number |
| `angle_range` | `tuple[float, float]` | `(0, 180)` | Min/max angle in degrees |
| `pulse_range` | `tuple[int, int]` | `(500, 2500)` | Min/max pulse width (μs) |
| `frequency` | `int` | `50` | PWM frequency in Hz |
| `inverted` | `bool` | `False` | Invert angle direction |
| `offset` | `float` | `0.0` | Calibration offset in degrees |
| `speed_deg_per_sec` | `float` | `None` | Maximum speed limit |

### Methods

```python
# Set angle immediately
servo.set(90)

# Get current angle
angle = servo.get()

# Move to angle asynchronously (respects speed limit)
await servo.move_to(45, speed=30)  # 30°/sec

# Sweep between two angles
await servo.sweep(0, 180, speed=60)

# Calibrate the servo (sets current position as reference)
servo.calibrate(at_angle=90)

# Enable/disable
servo.enable()
servo.disable()
```

### Configuration: Pulse Range and Frequency

Standard hobby servos use:
- **Pulse range**: 500-2500 μs (microseconds)
- **Frequency**: 50 Hz (20ms period)

```python
# Standard servo
servo = Servo(
    name="standard",
    pulse_range=(500, 2500),  # 0.5ms to 2.5ms
    frequency=50,             # 50 Hz
)

# High-frequency digital servo
servo = Servo(
    name="digital",
    pulse_range=(500, 2500),
    frequency=333,  # 333 Hz for faster response
)

# Continuous rotation servo
servo = Servo(
    name="continuous",
    angle_range=(-100, 100),  # Speed control, not position
    pulse_range=(1000, 2000),
)
```

### Example: Basic Servo Control

```python
from robo_infra.actuators import Servo
from robo_infra.drivers import PCA9685Driver

# Initialize PWM driver
driver = PCA9685Driver(i2c_address=0x40)
driver.connect()
driver.enable()

# Create servo on channel 0
servo = Servo(
    name="pan",
    driver=driver,
    channel=0,
    angle_range=(0, 180),
)

# Control the servo
servo.enable()
servo.set(90)  # Center position

# Sweep motion
import asyncio
asyncio.run(servo.sweep(0, 180, speed=60))

# Clean up
servo.disable()
driver.disconnect()
```

---

## DC Motor

DC motors provide continuous rotation with speed and direction control. They require an H-bridge driver for bidirectional control.

### Constructor Parameters

```python
from robo_infra.actuators import DCMotor

motor = DCMotor(
    pin_a=pin_a,           # Direction pin A (DigitalPin or channel int)
    pin_b=pin_b,           # Direction pin B (DigitalPin or channel int)
    enable=pwm_pin,        # PWM enable pin (PWMPin or channel int)
    driver=driver,         # Optional: hardware driver
    name="left_wheel",     # Human-readable name
    inverted=False,        # Swap forward/reverse
    pwm_frequency=1000,    # PWM frequency in Hz
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin_a` | `DigitalPin \| int` | `None` | Direction control pin A |
| `pin_b` | `DigitalPin \| int` | `None` | Direction control pin B |
| `enable` | `PWMPin \| int` | `None` | Speed control (PWM) pin |
| `driver` | `Driver` | `None` | Hardware driver instance |
| `name` | `str` | `"DCMotor"` | Human-readable name |
| `inverted` | `bool` | `False` | Invert direction |
| `pwm_frequency` | `int` | `1000` | PWM frequency in Hz |

### Methods

```python
# Set speed (-1.0 to 1.0)
motor.set(0.5)   # 50% forward
motor.set(-0.5)  # 50% reverse

# Convenience methods
motor.forward(speed=0.8)  # Run forward at 80%
motor.reverse(speed=0.5)  # Run reverse at 50%
motor.stop()              # Coast to stop (no braking)
motor.brake()             # Active braking
motor.coast()             # Same as stop()

# Get current speed
speed = motor.speed       # Returns -1.0 to 1.0
direction = motor.direction  # Direction.FORWARD, REVERSE, or STOP
```

### H-Bridge Truth Table

| pin_a | pin_b | enable | Result |
|-------|-------|--------|--------|
| HIGH | LOW | PWM | Forward at PWM% |
| LOW | HIGH | PWM | Reverse at PWM% |
| LOW | LOW | X | Coast (free spin) |
| HIGH | HIGH | X | Brake (locked) |

### Example: H-Bridge Motor Control

```python
from robo_infra.actuators import DCMotor
from robo_infra.drivers import L298NDriver

# Initialize L298N driver
driver = L298NDriver()
driver.connect()
driver.enable()

# Create motor using driver channels
motor = DCMotor(
    pin_a=0,        # IN1 channel
    pin_b=1,        # IN2 channel
    enable=2,       # ENA channel (PWM)
    driver=driver,
    name="left_wheel",
)

# Control the motor
motor.enable()

motor.forward(0.5)  # 50% forward
import time
time.sleep(2)

motor.reverse(0.3)  # 30% reverse
time.sleep(2)

motor.brake()       # Stop with braking
motor.disable()
```

---

## Stepper Motor

Stepper motors provide precise step-based positioning without feedback. They're ideal for CNC machines, 3D printers, and precision positioning.

### Constructor Parameters

```python
from robo_infra.actuators import Stepper

stepper = Stepper(
    step_pin=step_pin,       # Step pulse pin
    dir_pin=dir_pin,         # Direction pin
    enable_pin=enable_pin,   # Enable pin (optional)
    driver=driver,           # Optional: hardware driver
    name="x_axis",           # Human-readable name
    steps_per_rev=200,       # Steps per revolution (1.8° motor)
    microsteps=16,           # Microstepping divisor
    max_speed=2000,          # Maximum steps/second
    acceleration=0.0,        # Acceleration (steps/sec²)
    enable_active_high=True, # Enable pin polarity
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `step_pin` | `DigitalPin \| int` | required | Step pulse output |
| `dir_pin` | `DigitalPin \| int` | required | Direction control |
| `enable_pin` | `DigitalPin \| int` | `None` | Motor enable (optional) |
| `driver` | `Driver` | `None` | Hardware driver |
| `steps_per_rev` | `int` | `200` | Full steps per revolution |
| `microsteps` | `int` | `1` | Microstepping divisor |
| `max_speed` | `float` | `2000` | Max speed (steps/sec) |
| `acceleration` | `float` | `0.0` | Acceleration (steps/sec²) |

### Methods

```python
# Step a specific number of steps
stepper.step(100)    # 100 steps forward
stepper.step(-50)    # 50 steps reverse

# Move to absolute position
stepper.move_to(1000)  # Move to step 1000

# Move relative to current position
stepper.move_by(200)   # Move 200 steps from current

# Set speed (steps per second)
stepper.set_speed(500)

# Home to limit switch
stepper.home(limit_switch, direction=-1)

# Stop motion
stepper.stop()

# Properties
position = stepper.position       # Current step count
steps_per_rev = stepper.steps_per_rev
microsteps = stepper.microsteps
```

### Microstepping Configuration

Microstepping increases resolution by dividing each full step:

| Microsteps | Steps/Rev (1.8° motor) | Resolution |
|------------|------------------------|------------|
| 1 (full) | 200 | 1.8° |
| 2 (half) | 400 | 0.9° |
| 4 (quarter) | 800 | 0.45° |
| 8 | 1,600 | 0.225° |
| 16 | 3,200 | 0.1125° |
| 32 | 6,400 | 0.05625° |

### Example: Precise Positioning

```python
from robo_infra.actuators import Stepper
from robo_infra.sensors import LimitSwitch

# Create stepper motor
stepper = Stepper(
    step_pin=17,
    dir_pin=18,
    enable_pin=27,
    steps_per_rev=200,
    microsteps=16,
    max_speed=1000,
)

# Create limit switch for homing
limit = LimitSwitch(pin=22, normally_open=True)

stepper.enable()
limit.enable()

# Home to limit switch
stepper.home(limit, direction=-1)
print(f"Homed at position: {stepper.position}")

# Move to specific positions
stepper.move_to(1600)  # One full revolution (200 * 16 microsteps)
stepper.move_to(800)   # Half revolution

stepper.disable()
```

---

## Brushless Motor (ESC)

Brushless motors with Electronic Speed Controllers (ESC) are used for drones, electric vehicles, and high-power applications.

### Constructor Parameters

```python
from robo_infra.actuators import Brushless

motor = Brushless(
    channel=0,              # Driver/PWM channel
    driver=driver,          # Hardware driver
    pwm=pwm_pin,            # Or direct PWM pin
    name="motor_1",         # Human-readable name
    protocol="pwm",         # Protocol: pwm, oneshot, dshot
    frequency=50,           # PWM frequency (Hz)
    min_pulse_us=1000,      # Min throttle pulse (μs)
    max_pulse_us=2000,      # Max throttle pulse (μs)
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `channel` | `int` | `0` | Driver channel |
| `driver` | `Driver` | `None` | Hardware driver |
| `pwm` | `PWMPin` | `None` | Direct PWM pin |
| `protocol` | `str` | `"pwm"` | Control protocol |
| `frequency` | `int` | `50` | PWM frequency |
| `min_pulse_us` | `int` | `1000` | Minimum pulse width |
| `max_pulse_us` | `int` | `2000` | Maximum pulse width |

### Methods

```python
# Arm the ESC (required before throttle)
motor.arm()

# Set throttle (0.0 to 1.0)
motor.set(0.5)  # 50% throttle

# Disarm (stops motor, prevents accidental start)
motor.disarm()

# Calibrate ESC (sets throttle range)
motor.calibrate()

# Properties
throttle = motor.throttle  # Current throttle (0-1)
is_armed = motor.armed     # Arm state
protocol = motor.protocol  # "pwm", "oneshot", "dshot"
```

### Example: Drone Motor Control

```python
from robo_infra.actuators import Brushless
from robo_infra.drivers import PCA9685Driver

# Initialize driver
driver = PCA9685Driver(i2c_address=0x40, frequency=50)
driver.connect()
driver.enable()

# Create 4 motors for quadcopter
motors = [
    Brushless(channel=i, driver=driver, name=f"motor_{i}")
    for i in range(4)
]

# Enable and arm all motors
for motor in motors:
    motor.enable()
    motor.arm()

import time

# Spin up slowly
for throttle in [0.1, 0.2, 0.3]:
    for motor in motors:
        motor.set(throttle)
    time.sleep(1)

# Stop all motors
for motor in motors:
    motor.set(0)
    motor.disarm()
    motor.disable()
```

---

## Linear Actuator

Linear actuators provide stroke-based linear motion, either motor-driven or solenoid-based.

### Constructor Parameters

```python
from robo_infra.actuators import LinearActuator, DCMotor

# Motor-based linear actuator (proportional control)
linear = LinearActuator(
    motor=motor,                    # DCMotor for drive
    position_sensor=potentiometer,  # Optional: feedback sensor
    name="lift",
)

# Solenoid-based linear actuator (binary: extend/retract)
from robo_infra.actuators import Solenoid
linear = LinearActuator(
    solenoid=solenoid,  # Solenoid for drive
    name="latch",
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor` | `DCMotor` | `None` | Motor for proportional control |
| `solenoid` | `Solenoid` | `None` | Solenoid for binary control |
| `position_sensor` | `Sensor` | `None` | Feedback sensor (optional) |
| `name` | `str` | `"LinearActuator"` | Human-readable name |

### Methods

```python
# Extend/retract
linear.extend(speed=0.8)   # Extend at 80% speed
linear.retract(speed=0.5)  # Retract at 50% speed
linear.stop()              # Stop motion

# Move to position (requires feedback sensor)
linear.move_to(0.5)  # Move to 50% extension

# Properties
position = linear.position      # 0.0 (retracted) to 1.0 (extended)
is_extended = linear.is_extended
```

---

## Solenoid / Relay

Solenoids and relays provide simple on/off control for electromagnetic actuators, valves, and switching.

### Constructor Parameters

```python
from robo_infra.actuators import Solenoid

solenoid = Solenoid(
    pin=digital_pin,     # Digital output pin
    channel=0,           # Or driver channel
    driver=driver,       # Hardware driver
    name="valve",        # Human-readable name
    active_high=True,    # True if HIGH = on
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | `DigitalPin` | `None` | Digital output pin |
| `channel` | `int` | `None` | Driver channel |
| `driver` | `Driver` | `None` | Hardware driver |
| `name` | `str` | `"Solenoid"` | Human-readable name |
| `active_high` | `bool` | `True` | Polarity (HIGH=on) |

### Methods

```python
# On/off control
solenoid.activate()    # Turn on
solenoid.deactivate()  # Turn off
solenoid.on()          # Alias for activate()
solenoid.off()         # Alias for deactivate()
solenoid.toggle()      # Toggle state

# Pulse (momentary activation)
solenoid.pulse(duration=0.1)  # Activate for 100ms

# Properties
is_active = solenoid.is_active
```

### Duty Cycle for Holding

Solenoids can overheat if held on continuously. Use PWM for holding current:

```python
from robo_infra.actuators import Solenoid

# Initial pull-in at full power
solenoid.activate()
time.sleep(0.1)  # 100ms pull-in

# Reduce to holding current (30%)
solenoid.set(0.3)  # PWM at 30% duty cycle
```

---

## Relay (Alias)

`Relay` is an alias for `Solenoid`, as they share the same interface:

```python
from robo_infra.actuators import Relay

# Same as Solenoid
relay = Relay(pin=digital_pin, name="pump")
relay.enable()
relay.on()
time.sleep(5)
relay.off()
```

---

## Next Steps

- [Sensors](sensors.md) - Input devices and feedback
- [Controllers](controllers.md) - Coordinating multiple actuators
- [Drivers](drivers.md) - Hardware driver reference
- [Safety](safety.md) - Limit enforcement and emergency stop
