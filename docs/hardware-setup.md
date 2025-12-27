# Hardware Setup Guide

This guide covers complete hardware setup for robo-infra, from initial Raspberry Pi configuration to connecting servos, motors, and sensors.

## Overview

robo-infra supports multiple hardware platforms. This guide focuses on the most common setup:

- **Raspberry Pi 4/5** as the main controller
- **PCA9685** 16-channel PWM driver for servos
- **MPU6050** IMU for motion sensing
- **L298N** H-bridge for DC motors

For other platforms, see [Platform Support](platforms.md).

---

## Part 1: Raspberry Pi Setup

### 1.1 Operating System

Install Raspberry Pi OS (64-bit recommended):

```bash
# Using Raspberry Pi Imager
# Select: Raspberry Pi OS (64-bit)
# Enable SSH, set hostname, configure WiFi
```

### 1.2 Enable Hardware Interfaces

```bash
# Open raspi-config
sudo raspi-config

# Enable I2C (for PCA9685, MPU6050, sensors)
# Navigate: Interface Options → I2C → Enable

# Enable SPI (for high-speed devices)
# Navigate: Interface Options → SPI → Enable

# Enable Serial/UART (for GPS, LiDAR)
# Navigate: Interface Options → Serial Port → Enable

# Reboot to apply changes
sudo reboot
```

### 1.3 Verify I2C

```bash
# Install I2C tools
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus

# Scan for devices (should show nothing if no devices connected)
i2cdetect -y 1

# Expected output with PCA9685 (0x40) and MPU6050 (0x68):
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
# 70: -- -- -- -- -- -- -- --
```

### 1.4 Install robo-infra

```bash
# Install Python 3.11+
sudo apt-get install -y python3-pip python3-venv

# Create virtual environment
python3 -m venv ~/robo-env
source ~/robo-env/bin/activate

# Install robo-infra with hardware support
pip install robo-infra[hardware]

# Or from source for latest features
git clone https://github.com/nfraxlab/robo-infra.git
cd robo-infra
pip install -e ".[hardware]"
```

---

## Part 2: Hardware Connections

### 2.1 PCA9685 PWM Driver

The PCA9685 provides 16 channels of 12-bit PWM, perfect for controlling servos.

**Wiring:**

| Raspberry Pi | PCA9685 | Description |
|--------------|---------|-------------|
| Pin 1 (3.3V) | VCC | Logic power |
| Pin 3 (GPIO2/SDA) | SDA | I2C data |
| Pin 5 (GPIO3/SCL) | SCL | I2C clock |
| Pin 6 (GND) | GND | Ground |
| External 5-6V | V+ | Servo power |

**Important Notes:**
- V+ is separate from VCC - this powers the servos
- Use a 5-6V power supply rated for your servo count
- Each servo draws ~200-500mA, plan accordingly
- Add a large capacitor (1000µF) across V+ and GND for stability

**Verify Connection:**

```bash
# PCA9685 should appear at address 0x40
i2cdetect -y 1

# Test with Python
python3 -c "from robo_infra.platforms import get_i2c; print(get_i2c().scan())"
# Expected: [64] (0x40 = 64 decimal)
```

### 2.2 Servo Motors

Connect servos to PCA9685 channels:

| Servo Wire | PCA9685 Pin |
|------------|-------------|
| Signal (Orange/Yellow) | PWM (top row) |
| Power (Red) | V+ (middle row) |
| Ground (Brown/Black) | GND (bottom row) |

**Standard Servo Mapping:**

| Channel | Common Use | Pulse Range |
|---------|------------|-------------|
| 0 | Base rotation | 500-2500µs |
| 1 | Shoulder | 500-2500µs |
| 2 | Elbow | 500-2500µs |
| 3 | Wrist pitch | 500-2500µs |
| 4 | Wrist roll | 500-2500µs |
| 5 | Gripper | 500-2500µs |

**Test Servo:**

```python
from robo_infra.platforms import get_i2c
from robo_infra.drivers import PCA9685Driver
from robo_infra.actuators import Servo

# Initialize
i2c = get_i2c(bus=1)
driver = PCA9685Driver(i2c_bus=i2c, frequency=50)

# Create and test servo
servo = Servo(name="test", driver=driver, channel=0)
await servo.move_to(90)  # Center position
```

### 2.3 MPU6050 IMU

The MPU6050 provides 6-axis motion sensing (accelerometer + gyroscope).

**Wiring:**

| Raspberry Pi | MPU6050 | Description |
|--------------|---------|-------------|
| Pin 1 (3.3V) | VCC | Power |
| Pin 3 (GPIO2/SDA) | SDA | I2C data |
| Pin 5 (GPIO3/SCL) | SCL | I2C clock |
| Pin 6 (GND) | GND | Ground |
| Pin 6 (GND) | AD0 | Address select (0x68) |

**Address Configuration:**
- AD0 → GND: Address 0x68 (default)
- AD0 → VCC: Address 0x69

**Verify Connection:**

```bash
# MPU6050 should appear at address 0x68
i2cdetect -y 1

# Test with Python
python3 -c "
from robo_infra.platforms import get_i2c
i2c = get_i2c()
i2c.write_byte(0x68, 0x6B)  # Power management register
print('MPU6050 connected!')
"
```

### 2.4 DC Motors with L298N

The L298N H-bridge controls two DC motors with direction and speed.

**Wiring:**

| Raspberry Pi | L298N | Description |
|--------------|-------|-------------|
| GPIO18 (PWM) | ENA | Motor A speed |
| GPIO23 | IN1 | Motor A direction |
| GPIO24 | IN2 | Motor A direction |
| GPIO19 (PWM) | ENB | Motor B speed |
| GPIO25 | IN3 | Motor B direction |
| GPIO26 | IN4 | Motor B direction |
| Pin 6 (GND) | GND | Common ground |

**Motor Power:**
- Connect motor power (6-12V) to 12V and GND terminals
- Remove 5V jumper if using >12V external power
- Connect motors to OUT1/OUT2 and OUT3/OUT4

**Test Motor:**

```python
from robo_infra.platforms import get_gpio, get_pwm
from robo_infra.core.pin import PinMode

# Setup pins
enable = get_gpio(18, mode=PinMode.PWM, frequency=1000)
in1 = get_gpio(23, mode=PinMode.OUTPUT)
in2 = get_gpio(24, mode=PinMode.OUTPUT)

# Forward at 50% speed
in1.high()
in2.low()
enable.duty_cycle = 50

# Reverse
in1.low()
in2.high()

# Stop
enable.duty_cycle = 0
```

---

## Part 3: Common Hardware Setups

### 3.1 Robot Arm (6-DOF)

Complete 6-axis robot arm with gripper:

```
Raspberry Pi 4
    │
    └── I2C Bus 1
          │
          └── PCA9685 (0x40)
                ├── CH0: Base servo (rotation)
                ├── CH1: Shoulder servo
                ├── CH2: Elbow servo
                ├── CH3: Wrist pitch servo
                ├── CH4: Wrist roll servo
                └── CH5: Gripper servo
```

**Code:**

```python
from robo_infra.platforms import get_i2c
from robo_infra.drivers import PCA9685Driver
from robo_infra.actuators import Servo, ServoGroup
from robo_infra.core.types import Limits

# Initialize hardware
i2c = get_i2c(bus=1)
driver = PCA9685Driver(i2c_bus=i2c, frequency=50)

# Define servos with limits
servos = ServoGroup([
    Servo("base", driver, channel=0, limits=Limits(-90, 90)),
    Servo("shoulder", driver, channel=1, limits=Limits(0, 180)),
    Servo("elbow", driver, channel=2, limits=Limits(0, 135)),
    Servo("wrist_pitch", driver, channel=3, limits=Limits(-90, 90)),
    Servo("wrist_roll", driver, channel=4, limits=Limits(-180, 180)),
    Servo("gripper", driver, channel=5, limits=Limits(0, 45)),
])

# Home position
await servos.home()

# Move to target
await servos.move_to({
    "base": 45,
    "shoulder": 90,
    "elbow": 45,
    "gripper": 30,
})
```

### 3.2 Mobile Robot (Differential Drive)

Two-wheel differential drive robot:

```
Raspberry Pi 4
    │
    ├── GPIO (Direct)
    │     ├── GPIO18: Left motor PWM
    │     ├── GPIO23: Left motor IN1
    │     ├── GPIO24: Left motor IN2
    │     ├── GPIO19: Right motor PWM
    │     ├── GPIO25: Right motor IN1
    │     └── GPIO26: Right motor IN2
    │
    └── I2C Bus 1
          └── MPU6050 (0x68) - IMU for orientation
```

**Code:**

```python
from robo_infra.platforms import get_gpio, get_i2c
from robo_infra.actuators import DCMotor
from robo_infra.sensors import MPU6050
from robo_infra.core.pin import PinMode

# Motor setup
left_motor = DCMotor(
    pwm_pin=get_gpio(18, mode=PinMode.PWM, frequency=1000),
    in1_pin=get_gpio(23),
    in2_pin=get_gpio(24),
)

right_motor = DCMotor(
    pwm_pin=get_gpio(19, mode=PinMode.PWM, frequency=1000),
    in1_pin=get_gpio(25),
    in2_pin=get_gpio(26),
)

# IMU for heading
imu = MPU6050(i2c=get_i2c(bus=1))

# Drive forward
left_motor.forward(speed=0.5)
right_motor.forward(speed=0.5)

# Turn right
left_motor.forward(speed=0.5)
right_motor.backward(speed=0.5)
```

### 3.3 Sensor Array

Multiple I2C sensors on same bus:

```
Raspberry Pi 4
    │
    └── I2C Bus 1
          ├── MPU6050 (0x68) - IMU
          ├── BMP280 (0x76)  - Barometer
          ├── HMC5883L (0x1E) - Magnetometer
          └── VL53L0X (0x29) - ToF distance sensor
```

**Code:**

```python
from robo_infra.platforms import get_i2c

# Single I2C bus, multiple devices
i2c = get_i2c(bus=1)

# Scan for devices
devices = i2c.scan()
print(f"Found {len(devices)} I2C devices: {[hex(d) for d in devices]}")

# Read from each sensor
imu_data = i2c.read_block(0x68, 0x3B, 14)  # MPU6050 accel/gyro
baro_data = i2c.read_block(0x76, 0xF7, 6)  # BMP280 pressure/temp
```

---

## Part 4: Troubleshooting

### 4.1 I2C Issues

**No devices detected:**

```bash
# Check I2C is enabled
ls /dev/i2c*
# Should show: /dev/i2c-1

# Check wiring with oscilloscope or logic analyzer
# SDA and SCL should have ~3.3V pull-ups

# Try slower I2C speed
sudo nano /boot/config.txt
# Add: dtparam=i2c_baudrate=50000
```

**Intermittent communication:**

```bash
# Add external pull-up resistors (4.7kΩ to 3.3V)
# Shorten wires (< 30cm recommended)
# Check power supply is adequate
```

### 4.2 Servo Issues

**Servo jitter:**

```python
# Increase PWM frequency precision
driver = PCA9685Driver(i2c_bus=i2c, frequency=50)

# Add decoupling capacitors near servos
# Use separate power supply for servos
```

**Servo not moving:**

```bash
# Check V+ has power (5-6V)
# Verify correct channel number
# Test with known-good servo
```

### 4.3 Power Issues

**System unstable when motors run:**

```bash
# Use separate power supply for motors
# Add large capacitors (1000-4700µF) near motor driver
# Don't power motors from Raspberry Pi 5V
```

**Voltage drops:**

```bash
# Check power supply capacity
# Each servo: ~200-500mA
# DC motors: ~500mA-2A
# Total capacity should be 2x expected draw
```

### 4.4 Permission Issues

```bash
# Add user to required groups
sudo usermod -aG gpio,i2c,spi $USER

# Reboot or re-login
sudo reboot

# Verify groups
groups
# Should include: gpio i2c spi
```

---

## Part 5: Quick Reference

### Pin Reference (Raspberry Pi 4/5)

| Function | GPIO (BCM) | Pin | Description |
|----------|------------|-----|-------------|
| I2C SDA | 2 | 3 | I2C data |
| I2C SCL | 3 | 5 | I2C clock |
| SPI MOSI | 10 | 19 | SPI data out |
| SPI MISO | 9 | 21 | SPI data in |
| SPI SCLK | 11 | 23 | SPI clock |
| SPI CE0 | 8 | 24 | SPI chip select 0 |
| SPI CE1 | 7 | 26 | SPI chip select 1 |
| PWM0 | 12, 18 | 32, 12 | Hardware PWM |
| PWM1 | 13, 19 | 33, 35 | Hardware PWM |
| UART TX | 14 | 8 | Serial transmit |
| UART RX | 15 | 10 | Serial receive |

### Common I2C Addresses

| Device | Address | Alternate |
|--------|---------|-----------|
| PCA9685 | 0x40 | 0x41-0x7F (configurable) |
| MPU6050 | 0x68 | 0x69 (AD0=VCC) |
| BMP280 | 0x76 | 0x77 (SDO=VCC) |
| HMC5883L | 0x1E | - |
| VL53L0X | 0x29 | Programmable |
| OLED SSD1306 | 0x3C | 0x3D |

### Voltage Levels

| Component | Logic | Power |
|-----------|-------|-------|
| Raspberry Pi GPIO | 3.3V | - |
| PCA9685 | 3.3-5V | 5-6V (servos) |
| MPU6050 | 3.3V | 3.3V |
| L298N | 5V | 6-12V (motors) |
| Servos | 3.3-5V signal | 5-6V |
| DC Motors | - | 6-12V |

---

## Next Steps

- [Hardware Wiring Diagrams](hardware-wiring.md) - Visual wiring guides
- [Hardware Testing](hardware-testing.md) - Validate your setup
- [Platform Support](platforms.md) - Other platforms
- [Building a Robot Arm](examples/robot-arm.md) - Complete tutorial
