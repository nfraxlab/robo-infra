# Bill of Materials (BOM)

This document lists all hardware components needed for robo-infra hardware testing.

## Core Components

| Component | Quantity | Est. Cost | Notes |
|-----------|----------|-----------|-------|
| Raspberry Pi 4 (4GB) | 1 | $55 | Recommended for CI runner |
| MicroSD Card (32GB+) | 1 | $10 | Class 10 recommended |
| Raspberry Pi Power Supply | 1 | $15 | Official 5.1V 3A USB-C |
| Breadboard (full size) | 1 | $5 | For prototyping connections |
| Jumper Wires (M-M, M-F, F-F) | 1 kit | $8 | At least 40 of each type |

**Subtotal: ~$93**

## I2C Devices

| Component | Quantity | Est. Cost | I2C Address | Notes |
|-----------|----------|-----------|-------------|-------|
| PCA9685 PWM Driver | 1 | $6 | 0x40 | 16-channel PWM |
| MPU6050 IMU | 1 | $4 | 0x68 | 6-axis accelerometer/gyro |

**Subtotal: ~$10**

## Motors and Drivers

| Component | Quantity | Est. Cost | Notes |
|-----------|----------|-----------|-------|
| L298N Motor Driver | 1 | $5 | Dual H-bridge, up to 2A per channel |
| DC Motor (3-6V) | 2 | $4 | TT motors or similar |
| SG90 Micro Servo | 2 | $6 | 180° rotation |
| MG996R Servo | 1 | $8 | Higher torque, optional |

**Subtotal: ~$23**

## Power Supply

| Component | Quantity | Est. Cost | Notes |
|-----------|----------|-----------|-------|
| 5V 3A Power Supply | 1 | $10 | For servo power (separate from Pi) |
| 6V-12V Battery Pack | 1 | $8 | For motor power |
| DC Barrel Jack | 1 | $2 | For external power connection |

**Subtotal: ~$20**

## Optional Test Equipment

| Component | Quantity | Est. Cost | Notes |
|-----------|----------|-----------|-------|
| Logic Analyzer | 1 | $15 | For debugging I2C/SPI |
| Multimeter | 1 | $20 | Basic continuity/voltage testing |
| Oscilloscope (USB) | 1 | $50 | PWM signal verification |
| LED Pack (assorted) | 1 | $5 | For GPIO output testing |
| Resistor Pack (assorted) | 1 | $5 | 220Ω-10kΩ range |
| Push Buttons | 5 | $2 | For GPIO input testing |

**Subtotal: ~$97**

---

## Total Estimated Cost

| Configuration | Cost Range |
|---------------|------------|
| **Minimum** (Core + I2C only) | ~$103 |
| **Recommended** (Core + I2C + Motors) | ~$146 |
| **Full Setup** (All components) | ~$243 |

## Sourcing Notes

- **Amazon**: Fast shipping, slightly higher prices
- **Adafruit/SparkFun**: Quality components, good documentation
- **AliExpress**: Budget-friendly, longer shipping times
- **Micro Center**: Good for Raspberry Pi if local store available

## Verified Component Links

> Note: These are example links. Verify compatibility before purchasing.

### Adafruit
- [PCA9685 PWM Driver](https://www.adafruit.com/product/815)
- [MPU6050 Breakout](https://www.adafruit.com/product/3886)

### SparkFun
- [L298N Motor Driver](https://www.sparkfun.com/products/14450)
- [Servo Motor (Generic)](https://www.sparkfun.com/products/11965)

---

## Component Specifications

### PCA9685 PWM Driver

- **I2C Address**: 0x40 (default), configurable 0x40-0x7F
- **Channels**: 16
- **Resolution**: 12-bit (4096 steps)
- **Frequency**: 24Hz - 1526Hz
- **Voltage**: 3.3V or 5V logic, separate V+ for servos

### MPU6050 IMU

- **I2C Address**: 0x68 (AD0 low) or 0x69 (AD0 high)
- **Accelerometer**: ±2g, ±4g, ±8g, ±16g
- **Gyroscope**: ±250, ±500, ±1000, ±2000 °/s
- **Sample Rate**: Up to 8kHz
- **Voltage**: 3.3V (5V tolerant inputs)

### L298N Motor Driver

- **Channels**: 2 (dual H-bridge)
- **Max Current**: 2A per channel
- **Voltage**: 5V-35V motor supply
- **Logic Voltage**: 5V (3.3V compatible with level shifter)
- **PWM Frequency**: Up to 25kHz

### SG90 Servo

- **Rotation**: 180° (0°-180°)
- **Voltage**: 4.8V-6V
- **Torque**: 1.8 kg·cm
- **Speed**: 0.1 sec/60°
- **PWM Period**: 20ms (50Hz)
- **Pulse Width**: 500µs-2400µs
