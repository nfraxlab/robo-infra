# Sensors

Sensors provide input data from the physical world. robo-infra includes a comprehensive set of sensor types for robotics applications.

## Overview

All sensors extend the base `Sensor` class and share common patterns:

- **Enable/disable lifecycle** - Must be enabled before reading
- **Calibration support** - Optional calibration for accuracy
- **Driver abstraction** - Works with any compatible driver
- **Simulation support** - Mock values without hardware

```python
from robo_infra.sensors import Ultrasonic

# Create sensor
sensor = Ultrasonic(trigger_pin=trig, echo_pin=echo, name="front")

# Enable before use
sensor.enable()

# Read sensor data
reading = sensor.read()
print(f"Distance: {reading.value} {reading.unit}")

# Disable when done
sensor.disable()
```

---

## Distance Sensors

Distance sensors measure the range to nearby objects using ultrasonic, infrared, or laser time-of-flight principles.

### Ultrasonic

Ultrasonic sensors use sound waves to measure distance (HC-SR04, SRF04, etc.).

```python
from robo_infra.sensors import Ultrasonic
from robo_infra.core.types import Unit

sensor = Ultrasonic(
    trigger_pin=trigger,      # Trigger output pin
    echo_pin=echo,            # Echo input pin
    driver=driver,            # Or use driver/channel
    channel=0,
    name="front_sonar",
    unit=Unit.CENTIMETERS,    # CENTIMETERS or MILLIMETERS
    timeout_s=0.02,           # Max echo wait time
    us_per_cm=58.0,           # Conversion factor
)
```

**How it works:**
1. Trigger pin sends 10μs pulse
2. Sensor emits ultrasonic burst
3. Echo pin goes HIGH when sound returns
4. Duration × speed of sound = distance

```python
sensor.enable()

# Read distance
distance = sensor.read_distance()  # Returns float in configured unit
print(f"Distance: {distance} cm")

# Or get full reading with metadata
reading = sensor.read()
print(f"Value: {reading.value}, Unit: {reading.unit}")
```

### Time-of-Flight (ToF)

ToF sensors use laser pulses for precise distance measurement (VL53L0X, VL53L1X).

```python
from robo_infra.sensors import TimeOfFlight

sensor = TimeOfFlight(
    bus=i2c_bus,              # I2C bus
    address=0x29,             # I2C address (default: 0x29)
    name="front_tof",
    range_mode="short",       # "short", "medium", "long"
    timing_budget_ms=50,      # Measurement time
)

sensor.enable()
distance = sensor.read_distance()  # mm
```

### IR Distance

Infrared distance sensors use reflected IR light (Sharp GP2Y0A21).

```python
from robo_infra.sensors import IRDistance

sensor = IRDistance(
    pin=analog_pin,           # Analog input pin
    driver=driver,            # Or driver/channel
    channel=0,
    name="ir_front",
    min_range=10,             # Minimum range (cm)
    max_range=80,             # Maximum range (cm)
)

sensor.enable()
distance = sensor.read_distance()
```

---

## IMU Sensors

Inertial Measurement Units combine accelerometers, gyroscopes, and magnetometers for orientation and motion sensing.

### Accelerometer

Measures linear acceleration in g-force (1g ≈ 9.81 m/s²).

```python
from robo_infra.sensors import Accelerometer, AccelerometerConfig

config = AccelerometerConfig(
    i2c_address=0x68,
    range_g=2.0,              # ±2g, ±4g, ±8g, ±16g
    resolution_bits=16,
)

accel = Accelerometer(bus=i2c_bus, config=config)
accel.enable()

# Read 3-axis data
vector = accel.read_vector()
print(f"X: {vector.x:.2f}g, Y: {vector.y:.2f}g, Z: {vector.z:.2f}g")

# Calculate tilt from gravity
import math
pitch = math.atan2(vector.y, vector.z) * 180 / math.pi
roll = math.atan2(vector.x, vector.z) * 180 / math.pi
```

### Gyroscope

Measures angular velocity in degrees per second (°/s).

```python
from robo_infra.sensors import Gyroscope, GyroscopeConfig

config = GyroscopeConfig(
    i2c_address=0x68,
    range_dps=250.0,          # ±250, ±500, ±1000, ±2000 °/s
)

gyro = Gyroscope(bus=i2c_bus, config=config)
gyro.enable()

# Read angular velocity
vector = gyro.read_vector()
print(f"X: {vector.x:.1f}°/s, Y: {vector.y:.1f}°/s, Z: {vector.z:.1f}°/s")

# Integrate to get angle change
delta_angle = vector.z * dt  # Yaw rate * time
```

### Magnetometer

Measures magnetic field strength for compass heading.

```python
from robo_infra.sensors import Magnetometer

mag = Magnetometer(bus=i2c_bus, address=0x1E)
mag.enable()

# Calibrate (rotate sensor 360°)
await mag.calibrate()

# Read heading
heading = mag.read_heading()  # 0-360° (magnetic north)
vector = mag.read_vector()    # Raw magnetic field (μT)
```

### Combined IMU

Full 9-axis IMU with sensor fusion for orientation.

```python
from robo_infra.sensors import IMU, IMUConfig

config = IMUConfig(
    chip="bno055",            # bno055, mpu6050, icm20948, lsm6ds3
    i2c_address=0x28,
    fusion_mode="ndof",       # Sensor fusion mode
)

imu = IMU(bus=i2c_bus, config=config)
imu.enable()

# Read fused orientation (quaternion)
orientation = imu.read_orientation()
print(f"Quaternion: {orientation}")

# Read Euler angles
euler = imu.read_euler()
print(f"Heading: {euler.heading}°, Pitch: {euler.pitch}°, Roll: {euler.roll}°")

# Read individual sensors
accel = imu.read_acceleration()
gyro = imu.read_gyroscope()
mag = imu.read_magnetometer()
```

### Supported IMU Chips

| Chip | Axes | Fusion | I2C Address |
|------|------|--------|-------------|
| BNO055 | 9 (A+G+M) | Hardware | 0x28/0x29 |
| BMI270 | 6 (A+G) | Software | 0x68/0x69 |
| ICM20948 | 9 (A+G+M) | Software | 0x68/0x69 |
| LSM6DS3 | 6 (A+G) | Software | 0x6A/0x6B |
| MPU6050 | 6 (A+G) | Software | 0x68/0x69 |

---

## Encoders

Encoders measure rotational or linear position by counting pulses.

### Quadrature Encoder

Incremental encoder with A/B channels for direction detection.

```python
from robo_infra.sensors import QuadratureEncoder

encoder = QuadratureEncoder(
    channel_a=pin_a,          # A channel (DigitalPin)
    channel_b=pin_b,          # B channel (DigitalPin)
    driver=driver,            # Or driver/channel
    name="wheel_encoder",
    counts_per_revolution=600,  # PPR × 4 (quadrature)
    diameter_mm=65.0,         # For linear distance
)

encoder.enable()

# Read position
count = encoder.read_count()         # Raw pulse count
position = encoder.read_position()   # In configured unit (degrees/mm)
direction = encoder.direction        # FORWARD, REVERSE, STATIONARY

# Reset to zero
encoder.reset()

# Calculate velocity (with timer)
velocity = encoder.read_velocity()  # Counts/second or configured unit
```

**Quadrature decoding:**
- Rising edge on A while B is LOW = forward
- Rising edge on A while B is HIGH = reverse
- 4× resolution vs single-channel

### Absolute Encoder

Absolute position within a revolution (no homing required).

```python
from robo_infra.sensors import AbsoluteEncoder

encoder = AbsoluteEncoder(
    bus=spi_bus,              # SPI bus
    cs_pin=cs,                # Chip select
    resolution_bits=12,       # 12-bit = 4096 positions
    name="joint_encoder",
)

encoder.enable()

# Read absolute position
position = encoder.read_position()  # 0-360°
raw = encoder.read_count()          # 0-4095 (12-bit)
```

---

## Switches

Binary sensors for detecting mechanical contact or magnetic fields.

### Limit Switch

Mechanical switch for end-of-travel detection.

```python
from robo_infra.sensors import LimitSwitch

switch = LimitSwitch(
    pin=digital_pin,          # Digital input pin
    driver=driver,            # Or driver/channel
    channel=0,
    name="x_min",
    normally_open=True,       # NO or NC switch
    debounce_ms=10.0,         # Debounce time
)

switch.enable()

# Read state
is_triggered = switch.is_active
state = switch.state  # SwitchState.OPEN or CLOSED

# Wait for trigger (async)
await switch.wait_for_state(triggered=True, timeout=10.0)
print("Limit switch triggered!")
```

### Button

User input button with debouncing and event callbacks.

```python
from robo_infra.sensors import Button

button = Button(
    pin=digital_pin,
    name="start_button",
    normally_open=True,
    debounce_ms=50.0,         # Longer debounce for user input
)

button.enable()

# Poll state
if button.is_pressed:
    print("Button pressed")

# Wait for press
await button.wait_for_press(timeout=30.0)

# Event callbacks
button.on_press(lambda: print("Pressed!"))
button.on_release(lambda: print("Released!"))
```

### Hall Effect Sensor

Magnetic field detection for position sensing.

```python
from robo_infra.sensors import HallEffect

# Digital hall sensor (on/off)
hall = HallEffect(
    pin=digital_pin,
    name="magnet_detect",
    trigger_polarity="south",  # "north", "south", or "both"
)

# Analog hall sensor (field strength)
hall = HallEffect(
    pin=analog_pin,
    name="field_strength",
    analog=True,
    sensitivity_mv_per_gauss=1.3,
)

hall.enable()
field_strength = hall.read().value  # Gauss
```

---

## Environmental Sensors

Sensors for measuring ambient conditions.

### Temperature

```python
from robo_infra.sensors import Temperature, TemperatureConfig, TemperatureSensorType

# NTC Thermistor
config = TemperatureConfig(
    sensor_type=TemperatureSensorType.THERMISTOR,
    beta_coefficient=3950,
    nominal_resistance=10000,
    series_resistor=10000,
)
temp = Temperature(pin=analog_pin, config=config)

# DS18B20 (1-Wire)
config = TemperatureConfig(sensor_type=TemperatureSensorType.DS18B20)
temp = Temperature(onewire_bus=onewire, config=config)

temp.enable()
celsius = temp.read().value
fahrenheit = temp.read_fahrenheit()
```

### Humidity

```python
from robo_infra.sensors import Humidity

sensor = Humidity(
    bus=i2c_bus,
    address=0x40,             # HTU21D, SHT31, etc.
    name="ambient_humidity",
)

sensor.enable()
humidity = sensor.read().value  # 0-100 %RH
```

### Pressure (Barometric)

```python
from robo_infra.sensors import Pressure

sensor = Pressure(
    bus=i2c_bus,
    address=0x76,             # BMP280, BME280
    name="barometer",
)

sensor.enable()
pressure = sensor.read().value      # hPa
altitude = sensor.read_altitude()   # Meters (requires calibration)
```

---

## Electrical Sensors

Sensors for monitoring electrical systems.

### Current Sensor

```python
from robo_infra.sensors import CurrentSensor, CurrentSensorConfig, CurrentSensorType

# Hall effect current sensor (ACS712)
config = CurrentSensorConfig(
    sensor_type=CurrentSensorType.HALL_EFFECT,
    sensitivity_mv_per_amp=100.0,   # ACS712-20A
    zero_current_voltage=2.5,       # Vcc/2
    max_current=20.0,
)
sensor = CurrentSensor(pin=analog_pin, config=config)

# I2C current sensor (INA219)
config = CurrentSensorConfig(
    sensor_type=CurrentSensorType.I2C,
    shunt_resistance=0.1,
)
sensor = CurrentSensor(bus=i2c_bus, address=0x40, config=config)

sensor.enable()
current = sensor.read().value  # Amps
```

### Voltage Sensor

```python
from robo_infra.sensors import VoltageSensor, VoltageSensorConfig

config = VoltageSensorConfig(
    voltage_divider_ratio=11.0,   # For 0-25V range with 3.3V ADC
    reference_voltage=3.3,
)
sensor = VoltageSensor(pin=analog_pin, config=config)

sensor.enable()
voltage = sensor.read().value  # Volts
```

---

## Camera

Camera sensors for computer vision applications.

### USB Camera

```python
from robo_infra.sensors.cameras import USBCamera
from robo_infra.sensors.camera import CameraConfig

config = CameraConfig(
    width=1280,
    height=720,
    fps=30,
    format="BGR",
)
camera = USBCamera(device_id=0, config=config)

camera.enable()

# Capture single frame
frame = camera.capture()
print(f"Frame: {frame.width}x{frame.height}")

# Access numpy array
image = frame.data  # numpy array (H, W, 3)

# Stream frames
async for frame in camera.stream():
    process(frame.data)
    if should_stop():
        break

camera.disable()
```

### CSI Camera (Raspberry Pi / Jetson)

```python
from robo_infra.sensors.cameras import CSICamera

camera = CSICamera(
    width=1920,
    height=1080,
    fps=30,
    sensor_id=0,              # CSI port number
)

camera.enable()
frame = camera.capture()
```

### Depth Camera (RealSense)

```python
from robo_infra.sensors.cameras import DepthCamera

camera = DepthCamera(
    width=640,
    height=480,
    fps=30,
    depth_format="z16",       # 16-bit depth
)

camera.enable()

# Get color and depth frames
color_frame = camera.capture_color()
depth_frame = camera.capture_depth()

# Get aligned RGBD
rgbd = camera.capture_rgbd()
color = rgbd.color  # RGB array
depth = rgbd.depth  # Depth array (mm)

# Get 3D point cloud
points = camera.capture_pointcloud()
```

---

## LiDAR

2D and 3D laser scanners for mapping and obstacle detection.

### 2D Scanning LiDAR

```python
from robo_infra.sensors import LIDAR, LIDARConfig

config = LIDARConfig(
    model="rplidar_a2",       # rplidar_a1, ydlidar_x4, etc.
    port="/dev/ttyUSB0",
    baudrate=115200,
)
lidar = LIDAR(config=config)

lidar.enable()
lidar.start_scan()

# Get single scan (360°)
scan = lidar.read_scan()
# scan.ranges: numpy array of distances (meters)
# scan.angles: numpy array of angles (radians)

# Stream scans
async for scan in lidar.stream():
    # Process scan for obstacle avoidance
    obstacles = scan.ranges < 0.5  # Within 50cm
    if obstacles.any():
        print("Obstacle detected!")

lidar.stop_scan()
lidar.disable()
```

### Supported LiDAR Models

| Model | Range | Frequency | Resolution |
|-------|-------|-----------|------------|
| RPLIDAR A1 | 12m | 5.5 Hz | 1.0° |
| RPLIDAR A2 | 18m | 10 Hz | 0.45° |
| RPLIDAR A3 | 25m | 15 Hz | 0.225° |
| YDLidar X2 | 8m | 5 Hz | 0.5° |
| YDLidar X4 | 10m | 7 Hz | 0.5° |

---

## GPS

Global positioning sensors for outdoor navigation.

```python
from robo_infra.sensors import GPS, GPSConfig

config = GPSConfig(
    port="/dev/ttyAMA0",      # Serial port
    baudrate=9600,
    model="ublox",            # generic, ublox, adafruit_ultimate
)
gps = GPS(config=config)

gps.enable()

# Wait for fix
await gps.wait_for_fix(timeout=60.0)

# Read position
position = gps.read_position()
print(f"Lat: {position.latitude:.6f}°")
print(f"Lon: {position.longitude:.6f}°")
print(f"Alt: {position.altitude:.1f}m")

# Read velocity
velocity = gps.read_velocity()
print(f"Speed: {velocity.speed:.1f} m/s")
print(f"Heading: {velocity.heading:.1f}°")

# Fix quality
fix = gps.fix_quality  # FixQuality.GPS_FIX, RTK_FIXED, etc.
satellites = gps.satellites_used
hdop = gps.hdop  # Horizontal dilution of precision
```

### NMEA Parsing

The GPS sensor parses standard NMEA sentences:

| Sentence | Data |
|----------|------|
| GGA | Position, altitude, fix quality |
| RMC | Position, speed, heading, date/time |
| VTG | Course and ground speed |
| GSA | DOP and satellite info |
| GSV | Satellites in view |

### RTK Support

For centimeter-level accuracy:

```python
config = GPSConfig(
    model="ublox",
    rtk_enabled=True,
    rtcm_source="ntrip://user:pass@caster:2101/mount",
)
gps = GPS(config=config)

gps.enable()
await gps.wait_for_fix(quality=FixQuality.RTK_FIXED)
# Now at ~2cm accuracy
```

---

## Next Steps

- [Actuators](actuators.md) - Output devices and motion control
- [Controllers](controllers.md) - Coordinating sensors and actuators
- [Drivers](drivers.md) - Hardware driver reference
- [Vision](vision.md) - Image processing and computer vision
