# Drivers

Drivers provide hardware-independent interfaces to physical devices. They abstract communication protocols and device-specific registers, allowing actuators and sensors to work with any compatible hardware.

## Overview

All drivers extend the base `Driver` class and share common patterns:

- **Connect/disconnect lifecycle** - Must connect before use
- **Channel-based I/O** - Read and write via channel numbers
- **Driver registry** - Discover drivers by name
- **Simulation support** - Full functionality without hardware

```python
from robo_infra.drivers import PCA9685

# Create and connect driver
driver = PCA9685(bus=i2c_bus, address=0x40)
driver.connect()
driver.enable()

# Use the driver
driver.set_channel(0, 0.5)  # 50% duty cycle
value = driver.get_channel(0)

# Clean up
driver.disable()
driver.disconnect()
```

---

## SimulationDriver

The `SimulationDriver` enables testing without physical hardware. It tracks all operations and supports callbacks for verification.

```python
from robo_infra.drivers import SimulationDriver

# Create simulation driver
driver = SimulationDriver(
    channels=16,          # Number of channels
    delay=0.001,          # Simulated delay per operation
    log_operations=True,  # Log all operations
)

driver.connect()
driver.enable()

# Works like a real driver
driver.set_channel(0, 0.75)
print(driver.get_channel(0))  # 0.75

# Track changes with callbacks
changes = []
driver.on_channel_change(lambda ch, val: changes.append((ch, val)))
driver.set_channel(1, 0.5)
print(changes)  # [(1, 0.5)]

# Review operation history
for record in driver.operation_history:
    print(f"{record.operation}: ch={record.channel}, val={record.value}")

driver.disconnect()
```

### Features

| Feature | Description |
|---------|-------------|
| Channel tracking | Stores current value for each channel |
| Operation logging | Records all reads/writes with timestamps |
| Callbacks | Notify on channel changes |
| Simulated delays | Mimic hardware timing |
| History tracking | Per-channel value history |

---

## PWM Drivers

PWM drivers control pulse-width modulation outputs for servos, LEDs, and motor speed control.

### PCA9685

16-channel, 12-bit PWM driver over I2C. Most common driver for servo control.

```python
from robo_infra.drivers import PCA9685
from robo_infra.core.bus import get_i2c

# Initialize I2C bus and driver
bus = get_i2c(1)  # I2C bus 1
driver = PCA9685(
    bus=bus,
    address=0x40,     # Default address (A0-A5 configure)
    frequency=50,     # 50Hz for servos
)

driver.connect()
driver.enable()

# Set PWM frequency
driver.set_frequency(50)   # Standard servo frequency
driver.set_frequency(1000) # Higher frequency for LEDs

# Set channel duty cycle (0.0 to 1.0)
driver.set_channel(0, 0.5)  # 50% duty

# Raw 12-bit PWM control
driver.set_pwm(0, on=0, off=2048)  # 50% with custom timing

# Control all channels
driver.set_all_channels(0.0)  # All off

# Servo-specific methods
driver.set_servo_pulse(0, pulse_us=1500)  # Set pulse width in microseconds

driver.disconnect()
```

**Hardware Reference:**
| Parameter | Value |
|-----------|-------|
| Channels | 16 |
| Resolution | 12-bit (4096 steps) |
| Frequency | 24Hz - 1526Hz |
| I2C Address | 0x40 - 0x7F (configurable) |
| Voltage | 2.3V - 5.5V |

### GPIO PWM

Direct GPIO-based PWM using the platform's hardware PWM or software PWM.

```python
from robo_infra.drivers import GPIOPWM

driver = GPIOPWM(
    pins=[12, 13, 18, 19],  # GPIO pins with hardware PWM
    frequency=50,           # PWM frequency
)

driver.connect()
driver.enable()

# Set duty cycle
driver.set_channel(0, 0.5)  # Pin 12 at 50%

driver.disconnect()
```

---

## Motor Drivers

Motor drivers control DC motors, stepper motors, and other high-current loads.

### L298N

Dual H-bridge motor driver for DC motors and bipolar steppers.

```python
from robo_infra.drivers import L298N, MotorDirection

driver = L298N(
    in1_pin=17,    # Direction pin 1
    in2_pin=18,    # Direction pin 2
    ena_pin=12,    # Enable/PWM pin
    in3_pin=22,    # Motor B direction 1
    in4_pin=23,    # Motor B direction 2
    enb_pin=13,    # Motor B enable/PWM
)

driver.connect()
driver.enable()

# Control motor A
driver.set_motor(0, speed=0.75, direction=MotorDirection.FORWARD)
driver.set_motor(0, speed=0.5, direction=MotorDirection.REVERSE)

# Stop methods
driver.brake(0)  # Active braking (both direction pins HIGH)
driver.coast(0)  # Free-run (both direction pins LOW)

# Motor B
driver.set_motor(1, speed=0.6, direction=MotorDirection.FORWARD)

driver.disconnect()
```

**Hardware Reference:**
| Parameter | Value |
|-----------|-------|
| Channels | 2 motors |
| Voltage | 5V - 35V |
| Current | 2A continuous, 3A peak |
| Control | 6 pins (IN1-4, ENA, ENB) |

### TB6612

Dual MOSFET motor driver with better efficiency than L298N.

```python
from robo_infra.drivers import TB6612

driver = TB6612(
    pwma_pin=12, ain1_pin=17, ain2_pin=18,  # Motor A
    pwmb_pin=13, bin1_pin=22, bin2_pin=23,  # Motor B
    stby_pin=27,                             # Standby pin
)

driver.connect()
driver.enable()

# Same interface as L298N
driver.set_motor(0, speed=0.8, direction=MotorDirection.FORWARD)
driver.standby(True)   # Enter low-power standby
driver.standby(False)  # Exit standby

driver.disconnect()
```

### TMC2209

Silent stepper driver with StallGuard and UART configuration.

```python
from robo_infra.drivers import TMC2209

driver = TMC2209(
    step_pin=17,
    dir_pin=18,
    enable_pin=27,
    uart_port="/dev/ttyAMA0",  # Optional UART for advanced config
)

driver.connect()
driver.enable()

# Configure via UART
driver.set_current(800)         # 800mA RMS
driver.set_microsteps(16)       # 16 microsteps
driver.set_stealthchop(True)    # Silent operation

# Detect stalls
if driver.stallguard_triggered():
    print("Motor stalled!")

driver.disconnect()
```

### StepDir

Generic step/direction driver for any stepper driver (A4988, DRV8825, etc.).

```python
from robo_infra.drivers import StepDir

driver = StepDir(
    step_pin=17,
    dir_pin=18,
    enable_pin=27,      # Optional
    microsteps=16,      # Set on physical switches
    steps_per_rev=200,
)

driver.connect()
driver.enable()

# Step control
driver.step(100)         # 100 steps forward
driver.step(-50)         # 50 steps reverse
driver.set_direction(1)  # Set direction

driver.disconnect()
```

---

## Smart Motor Drivers

Smart motor drivers include feedback, communication protocols, and advanced control.

### Dynamixel

Smart servos with position feedback, torque control, and daisy-chaining.

```python
from robo_infra.drivers import DynamixelDriver

driver = DynamixelDriver(
    port="/dev/ttyUSB0",
    baudrate=1000000,
)

driver.connect()

# Scan for servos
servo_ids = driver.ping_scan()
print(f"Found servos: {servo_ids}")

# Enable torque
driver.set_torque_enable(1, True)

# Position control
driver.set_goal_position(1, 2048)  # Center (0-4095)
position = driver.get_present_position(1)

# Velocity control
driver.set_operating_mode(1, OperatingMode.VELOCITY_CONTROL)
driver.set_goal_velocity(1, 100)  # RPM

# Sync write to multiple servos
driver.sync_write_position({
    1: 1024,
    2: 2048,
    3: 3072,
})

# Read status
temp = driver.get_present_temperature(1)
load = driver.get_present_load(1)
voltage = driver.get_present_voltage(1)

driver.disconnect()
```

**Supported Models:**
| Series | Description |
|--------|-------------|
| XL330, XL430, XC430 | Low-cost, hobby |
| XM430, XM540 | Mid-range |
| XH430, XH540 | High-performance |
| XW430, XW540 | Industrial |

### ODrive

BLDC motor controller with encoder feedback and FOC control.

```python
from robo_infra.drivers import ODriveDriver

driver = ODriveDriver(
    serial_port="/dev/ttyACM0",  # Or USB
)

driver.connect()

# Calibrate axis
driver.calibrate(axis=0)

# Control modes
driver.set_control_mode(0, "position")
driver.set_control_mode(0, "velocity")
driver.set_control_mode(0, "torque")

# Position control
driver.set_position(0, 10000)  # Encoder counts

# Velocity control
driver.set_velocity(0, 1000)   # Counts/second

# Read feedback
position = driver.get_position(0)
velocity = driver.get_velocity(0)
current = driver.get_current(0)

driver.disconnect()
```

### VESC

ESC with telemetry and advanced motor control.

```python
from robo_infra.drivers import VESCDriver

driver = VESCDriver(
    port="/dev/ttyUSB0",
    baudrate=115200,
)

driver.connect()

# Duty cycle control
driver.set_duty(0.5)      # 50% duty

# Current control
driver.set_current(10.0)  # 10A

# RPM control
driver.set_rpm(3000)      # 3000 RPM

# Read telemetry
values = driver.get_values()
print(f"Temp: {values.temp_mos}°C")
print(f"Current: {values.current_motor}A")
print(f"RPM: {values.rpm}")
print(f"Voltage: {values.v_in}V")

driver.disconnect()
```

---

## IMU Drivers

IMU drivers communicate with inertial measurement units.

### BNO055

9-axis absolute orientation sensor with hardware fusion.

```python
from robo_infra.drivers import BNO055Driver

driver = BNO055Driver(
    bus=i2c_bus,
    address=0x28,  # or 0x29
)

driver.connect()

# Set operating mode
driver.set_mode("ndof")  # 9-axis fusion

# Read fused orientation
euler = driver.get_euler()
print(f"Heading: {euler.heading}, Pitch: {euler.pitch}, Roll: {euler.roll}")

quaternion = driver.get_quaternion()

# Read raw sensors
accel = driver.get_accelerometer()
gyro = driver.get_gyroscope()
mag = driver.get_magnetometer()

# Calibration status
status = driver.get_calibration_status()
print(f"Sys: {status.system}, Gyro: {status.gyro}, Accel: {status.accel}, Mag: {status.mag}")

driver.disconnect()
```

### BMI270

6-axis IMU (accelerometer + gyroscope) with low power modes.

```python
from robo_infra.drivers import BMI270Driver

driver = BMI270Driver(bus=i2c_bus, address=0x68)
driver.connect()

# Configure
driver.set_accel_range(4)   # ±4g
driver.set_gyro_range(500)  # ±500 °/s
driver.set_output_data_rate(100)  # 100 Hz

# Read data
accel = driver.get_accelerometer()
gyro = driver.get_gyroscope()

driver.disconnect()
```

### ICM20948

9-axis IMU with magnetometer.

```python
from robo_infra.drivers import ICM20948Driver

driver = ICM20948Driver(bus=i2c_bus, address=0x68)
driver.connect()

accel = driver.get_accelerometer()
gyro = driver.get_gyroscope()
mag = driver.get_magnetometer()

driver.disconnect()
```

### LSM6DS3

6-axis IMU commonly found on Arduino boards.

```python
from robo_infra.drivers import LSM6DS3Driver

driver = LSM6DS3Driver(bus=i2c_bus, address=0x6A)
driver.connect()

accel = driver.get_accelerometer()
gyro = driver.get_gyroscope()

driver.disconnect()
```

---

## Platform Drivers

Platform drivers interface with microcontrollers and GPIO systems.

### Arduino

Serial communication with Arduino boards running Firmata or custom firmware.

```python
from robo_infra.drivers import ArduinoDriver

driver = ArduinoDriver(
    port="/dev/ttyUSB0",
    baudrate=115200,
    protocol="firmata",  # or "custom"
)

driver.connect()

# Digital I/O
driver.digital_write(13, True)   # LED on
state = driver.digital_read(2)   # Read button

# Analog I/O
value = driver.analog_read(0)    # 0-1023
driver.analog_write(9, 128)      # PWM (0-255)

# Servo control (via Firmata)
driver.servo_write(9, 90)        # 90 degrees

driver.disconnect()
```

### GPIO

Direct GPIO access for Raspberry Pi, Jetson, and other SBCs.

```python
from robo_infra.drivers import GPIODriver

driver = GPIODriver()
driver.connect()

# Configure pins
driver.setup(17, mode="output")
driver.setup(18, mode="input", pull="up")

# Digital I/O
driver.write(17, True)
state = driver.read(18)

# PWM
driver.setup_pwm(12, frequency=1000)
driver.set_pwm_duty(12, 50)  # 50% duty

# Cleanup
driver.cleanup()
driver.disconnect()
```

---

## Creating Custom Drivers

Extend the `Driver` base class to support new hardware.

### Required Methods

```python
from robo_infra.core.driver import Driver, register_driver

@register_driver("my_driver")
class MyDriver(Driver):
    """Custom driver for My Hardware."""
    
    def __init__(self, bus, address: int = 0x50):
        super().__init__(name="MyDriver", channels=8)
        self.bus = bus
        self.address = address
    
    def connect(self) -> None:
        """Establish connection to hardware."""
        # Initialize hardware
        self.bus.open()
        self._verify_device()
        self._connected = True
    
    def disconnect(self) -> None:
        """Close connection to hardware."""
        self.bus.close()
        self._connected = False
    
    def _write_channel(self, channel: int, value: float) -> None:
        """Write value to a channel (internal implementation)."""
        # Convert 0-1 value to hardware format
        raw = int(value * 255)
        register = 0x10 + channel
        self.bus.write_byte(self.address, register, raw)
    
    def _read_channel(self, channel: int) -> float:
        """Read value from a channel (internal implementation)."""
        register = 0x10 + channel
        raw = self.bus.read_byte(self.address, register)
        return raw / 255.0
```

### Using the Registry

```python
from robo_infra.core.driver import get_driver, list_drivers

# List all registered drivers
print(list_drivers())
# ['pca9685', 'l298n', 'simulation', 'dynamixel', 'my_driver', ...]

# Get driver class by name
DriverClass = get_driver("my_driver")
driver = DriverClass(bus=i2c_bus, address=0x50)
```

### Example: Complete Custom Driver

```python
from robo_infra.core.driver import Driver, DriverConfig, DriverState, register_driver
from robo_infra.core.exceptions import CommunicationError

@register_driver("mcp4728")
class MCP4728(Driver):
    """4-channel 12-bit DAC driver."""
    
    CHANNELS = 4
    MAX_VALUE = 4095  # 12-bit
    
    def __init__(self, bus, address: int = 0x60):
        super().__init__(name="MCP4728", channels=self.CHANNELS)
        self.bus = bus
        self.address = address
        self._values = [0] * self.CHANNELS
    
    def connect(self) -> None:
        try:
            # Check device presence
            self.bus.read_byte(self.address)
            self._connected = True
        except Exception as e:
            raise CommunicationError(f"MCP4728 not found at {hex(self.address)}") from e
    
    def disconnect(self) -> None:
        self._connected = False
    
    def _write_channel(self, channel: int, value: float) -> None:
        """Write DAC value (0.0-1.0)."""
        dac_value = int(value * self.MAX_VALUE)
        dac_value = max(0, min(self.MAX_VALUE, dac_value))
        
        # MCP4728 fast write command
        high = (channel << 4) | ((dac_value >> 8) & 0x0F)
        low = dac_value & 0xFF
        self.bus.write_bytes(self.address, bytes([0x40 | (channel << 1), high, low]))
        
        self._values[channel] = dac_value
    
    def _read_channel(self, channel: int) -> float:
        """Read last written value."""
        return self._values[channel] / self.MAX_VALUE
    
    def write_all(self, values: list[float]) -> None:
        """Write all channels at once."""
        for i, value in enumerate(values[:self.CHANNELS]):
            self._write_channel(i, value)
```

---

## Next Steps

- [Controllers](controllers.md) - Coordinating actuators with drivers
- [Actuators](actuators.md) - Using drivers with actuators
- [Core Concepts](core-concepts.md) - Driver abstraction patterns
- [Platforms](platforms.md) - Platform-specific setup
