# Platform Support

robo-infra provides hardware abstraction across multiple platforms, enabling code portability between different single-board computers (SBCs), microcontrollers, and development environments.

## Supported Platforms

| Platform | GPIO | PWM | I2C | SPI | UART | ADC | Status |
|----------|------|-----|-----|-----|------|-----|--------|
| **Raspberry Pi** (all models) | ✅ | ✅ | ✅ | ✅ | ✅ | — | Production |
| **NVIDIA Jetson** (Nano, TX2, Xavier, Orin) | ✅ | ✅ | ✅ | ✅ | ✅ | — | Production |
| **BeagleBone** (Black, Green, AI) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | Production |
| **Arduino** (Uno, Mega, Nano, Due) | ✅ | ✅ | — | — | — | ✅ | Production |
| **ESP32** (all variants) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | Production |
| **Linux Generic** (Orange Pi, Rock Pi, etc.) | ✅ | ✅ | ✅ | ✅ | ✅ | — | Production |
| **Simulation** (development) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | Production |

## Quick Start

### Auto-Detection

robo-infra automatically detects your hardware platform:

```python
from robo_infra.platforms import get_platform

# Auto-detect current platform
platform = get_platform()
print(f"Running on: {platform.name}")

# Get hardware resources
pin = platform.get_pin(17, mode="output")
pin.high()
```

### Explicit Platform Selection

```python
from robo_infra.platforms import RaspberryPiPlatform, JetsonPlatform

# Raspberry Pi
pi = RaspberryPiPlatform()
led = pi.get_pin(17, mode="output")

# NVIDIA Jetson
jetson = JetsonPlatform()
motor = jetson.get_pwm_pin(32, frequency=1000)
```

### Simulation Mode

For development without hardware:

```python
from robo_infra.platforms import SimulationPlatform

# Explicit simulation
platform = SimulationPlatform()

# Or via environment variable
# ROBO_SIMULATION=true python my_robot.py
```

## Platform Details

### Raspberry Pi

Supports all Raspberry Pi models including Pi 5 with the new RP1 GPIO chip.

```python
from robo_infra.platforms import RaspberryPiPlatform, GPIOBackend

# Auto-detect backend (RPi.GPIO, lgpio, gpiod, pigpio)
pi = RaspberryPiPlatform()

# Force specific backend
pi = RaspberryPiPlatform(backend=GPIOBackend.LGPIO)

# Get platform info
info = pi.get_info()
print(f"Model: {info.model}")
print(f"Revision: {info.revision}")
print(f"Is Pi 5: {pi.is_pi5}")
```

**GPIO Backends:**
- `GPIOBackend.RPI_GPIO` - RPi.GPIO library (legacy, not Pi 5)
- `GPIOBackend.LGPIO` - lgpio library (recommended for Pi 5)
- `GPIOBackend.GPIOD` - libgpiod (kernel-level)
- `GPIOBackend.PIGPIO` - pigpio daemon (remote capable)
- `GPIOBackend.SIMULATION` - No hardware required

**Hardware PWM Pins:**
- Standard (BCM): 12, 13, 18, 19
- Pi 5 (additional): 12, 13, 14, 15, 18, 19

### NVIDIA Jetson

Supports Jetson Nano, TX2, Xavier NX/AGX, and Orin models.

```python
from robo_infra.platforms import JetsonPlatform, JetsonModel

jetson = JetsonPlatform()

# Check model
print(f"Model: {jetson.model}")  # e.g., JetsonModel.ORIN_NANO

# Power mode management
jetson.set_power_mode(JetsonPowerMode.MAXN)

# Get tegra stats
stats = jetson.get_tegra_stats()
print(f"GPU: {stats.gpu_percent}%")
print(f"CPU: {stats.cpu_percent}%")
```

**Pin Numbering:**
- `JetsonPinNumbering.BOARD` - Physical pin numbers (1-40)
- `JetsonPinNumbering.BCM` - Broadcom-compatible numbering
- `JetsonPinNumbering.TEGRA` - Tegra GPIO numbering

### BeagleBone

Supports BeagleBone Black, Green, and AI with PRU access.

```python
from robo_infra.platforms import BeagleBonePlatform

bb = BeagleBonePlatform()

# P8/P9 header notation
led = bb.get_pin("P8_10", mode="output")
led.high()

# PWM output
pwm = bb.get_pwm_pin("P9_14", frequency=1000)
pwm.set_duty_cycle(0.5)
pwm.start()

# ADC (7 channels, 12-bit)
adc = bb.get_adc_pin(0)  # AIN0
voltage = adc.read()  # 0.0 - 1.8V

# Check PRU availability
if bb.bb_capabilities.has_pru:
    print("PRU available for real-time control")
```

**Capabilities:**
- `has_pru` - Programmable Real-time Unit
- `has_emmc` - Onboard eMMC storage
- `has_wireless` - WiFi/Bluetooth
- `has_hdmi` - HDMI output
- `has_eqep` - Enhanced Quadrature Encoder Pulse

### Arduino

Communicates via Firmata protocol over USB/Serial.

```python
from robo_infra.platforms import ArduinoPlatform, ArduinoBoard

# Auto-detect connected Arduino
arduino = ArduinoPlatform()

# Or specify port
arduino = ArduinoPlatform(port="/dev/ttyUSB0", board=ArduinoBoard.UNO)

# Digital I/O
led = arduino.get_pin(13, mode="output")
led.high()

button = arduino.get_pin(2, mode="input_pullup")
if button.read():
    print("Button pressed")

# PWM (8-bit)
pwm = arduino.get_pwm_pin(9)
pwm.set_duty_cycle(0.5)  # 50%

# Analog input (10-bit ADC)
sensor = arduino.get_analog_pin(0)
value = sensor.read_raw()  # 0-1023
voltage = sensor.read()    # 0.0-5.0V

# Servo control
servo = arduino.get_servo_pin(9)
servo.write(90)  # 90 degrees
```

**Supported Boards:**
- Arduino Uno (ATmega328P)
- Arduino Mega (ATmega2560)
- Arduino Nano
- Arduino Due (ARM)
- Arduino Leonardo

### ESP32

Communicates via WiFi or Serial with MicroPython/ESP-IDF.

```python
from robo_infra.platforms import ESP32Platform, ESP32Variant

# WiFi connection
esp = ESP32Platform(
    host="192.168.1.100",
    transport="wifi"
)

# Or serial
esp = ESP32Platform(
    port="/dev/ttyUSB0",
    transport="serial"
)

# GPIO
led = esp.get_pin(2, mode="output")
led.high()

# PWM (16-bit resolution)
pwm = esp.get_pwm_pin(25, frequency=5000)
pwm.set_duty_cycle(0.75)

# ADC (12-bit, channels 0-7)
adc = esp.get_adc_pin(34)
voltage = adc.read()

# Touch pins
touch = esp.get_touch_pin(4)
if touch.read() < 40:
    print("Touch detected")

# DAC (8-bit, pins 25, 26)
dac = esp.get_dac_pin(25)
dac.write(128)  # ~1.65V
```

**Variants:**
- `ESP32Variant.ESP32` - Original ESP32
- `ESP32Variant.ESP32_S2` - ESP32-S2
- `ESP32Variant.ESP32_S3` - ESP32-S3
- `ESP32Variant.ESP32_C3` - ESP32-C3 (RISC-V)

### Linux Generic

For Orange Pi, Rock Pi, Pine64, Banana Pi, and other Linux SBCs using libgpiod.

```python
from robo_infra.platforms import LinuxGenericPlatform, LinuxSBCType

# Auto-detect SBC type
linux = LinuxGenericPlatform()
print(f"Detected: {linux.sbc_type}")  # e.g., LinuxSBCType.ORANGE_PI

# GPIO chip/line model
pin = linux.get_pin(17, chip=0, mode="output")
pin.high()

# PWM via sysfs
pwm = linux.get_pwm_pin(chip=0, channel=0, frequency=1000)
pwm.set_duty_cycle(0.5)
pwm.start()

# List available GPIO chips
for chip in linux.list_gpio_chips():
    print(f"Chip {chip.chip_id}: {chip.name} ({chip.num_lines} lines)")
```

**Detected SBC Types:**
- `LinuxSBCType.ORANGE_PI` - Orange Pi boards
- `LinuxSBCType.ROCK_PI` - Rock Pi boards
- `LinuxSBCType.PINE64` - Pine64 boards
- `LinuxSBCType.BANANA_PI` - Banana Pi boards
- `LinuxSBCType.ASUS_TINKER` - ASUS Tinker Board
- `LinuxSBCType.GENERIC` - Unknown Linux SBC

## Platform Detection

robo-infra provides detection utilities for identifying hardware:

```python
from robo_infra.platforms import (
    detect_platform,
    detect_raspberry_pi,
    detect_jetson,
    detect_beaglebone,
    detect_arduino,
    detect_esp32,
    get_platform_info,
)

# Full platform detection
platform_type = detect_platform()
print(f"Platform: {platform_type}")

# Platform-specific detection
if detect_raspberry_pi():
    info = get_platform_info()
    print(f"Raspberry Pi: {info.model}")

# Get comprehensive info
info = get_platform_info()
print(f"Platform: {info.platform_type}")
print(f"Model: {info.model}")
print(f"GPIO Chips: {info.gpio_chips}")
print(f"I2C Buses: {info.i2c_buses}")
print(f"SPI Buses: {info.spi_buses}")
```

## Capabilities System

Each platform declares its capabilities:

```python
from robo_infra.platforms import get_platform, PlatformCapability

platform = get_platform()
caps = platform.capabilities

if PlatformCapability.GPIO in caps:
    print("GPIO supported")

if PlatformCapability.PWM in caps:
    print("PWM supported")

if PlatformCapability.I2C in caps:
    print("I2C supported")
```

**Available Capabilities:**
- `GPIO` - Digital I/O
- `PWM` - Pulse Width Modulation
- `I2C` - I2C bus
- `SPI` - SPI bus
- `UART` - Serial communication
- `ADC` - Analog-to-Digital Converter
- `DAC` - Digital-to-Analog Converter
- `CAN` - CAN bus

## Bus Access

Platforms provide unified bus access:

```python
from robo_infra.platforms import get_platform

platform = get_platform()

# I2C
i2c = platform.get_bus("i2c", bus=1)
data = i2c.read(address=0x48, length=2)
i2c.write(address=0x48, data=bytes([0x01, 0x02]))

# SPI
spi = platform.get_bus("spi", bus=0, device=0)
response = spi.transfer(bytes([0x00, 0x01]))

# UART
uart = platform.get_bus("uart", port="/dev/ttyAMA0", baudrate=115200)
uart.write(b"Hello")
data = uart.read(10)
```

## Simulation Mode

Simulation mode enables development without hardware:

```python
import os
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.platforms import get_platform

platform = get_platform()  # Returns SimulationPlatform

# All operations work but don't touch real hardware
pin = platform.get_pin(17, mode="output")
pin.high()  # Simulated
print(pin.read())  # Returns True (simulated state)
```

Simulation can also be enabled per-platform:

```python
from robo_infra.platforms import RaspberryPiPlatform, GPIOBackend

# Simulation backend
pi = RaspberryPiPlatform(backend=GPIOBackend.SIMULATION)
```

## Cross-Platform Code

Write code that works on any platform:

```python
from robo_infra.platforms import get_platform, PlatformCapability

def blink_led(pin_id: int, count: int = 5):
    """Blink an LED on any platform."""
    platform = get_platform()
    
    if PlatformCapability.GPIO not in platform.capabilities:
        raise RuntimeError("GPIO not supported on this platform")
    
    led = platform.get_pin(pin_id, mode="output")
    led.setup()
    
    for _ in range(count):
        led.high()
        time.sleep(0.5)
        led.low()
        time.sleep(0.5)
    
    led.cleanup()

# Works on Raspberry Pi, Jetson, BeagleBone, etc.
blink_led(17)
```

## Platform Registry

Register custom platforms:

```python
from robo_infra.platforms import (
    BasePlatform,
    PlatformType,
    register_platform,
)

class MyCustomPlatform(BasePlatform):
    """Custom platform implementation."""
    
    @property
    def is_available(self) -> bool:
        return self._detect_my_hardware()
    
    def _detect_info(self) -> PlatformInfo:
        return PlatformInfo(
            platform_type=PlatformType.LINUX,
            model="My Custom Board",
            ...
        )

# Register for auto-detection
register_platform(MyCustomPlatform, priority=50)

# Now get_platform() can return MyCustomPlatform
```

## Error Handling

```python
from robo_infra.platforms import get_platform
from robo_infra.core import PinMode

platform = get_platform()

try:
    pin = platform.get_pin(999, mode=PinMode.OUTPUT)
except ValueError as e:
    print(f"Invalid pin: {e}")

try:
    pin = platform.get_pin(17, mode=PinMode.OUTPUT)
    pin.setup()
    pin.high()
except PermissionError as e:
    print(f"Permission denied: {e}")
except RuntimeError as e:
    print(f"Hardware error: {e}")
finally:
    platform.cleanup()
```

## Best Practices

1. **Use auto-detection** - Let robo-infra detect the platform
2. **Check capabilities** - Verify features before using them
3. **Always cleanup** - Call `platform.cleanup()` on exit
4. **Use simulation for tests** - Set `ROBO_SIMULATION=true` in CI
5. **Handle errors gracefully** - Hardware can fail unexpectedly

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ROBO_SIMULATION` | Force simulation mode | `false` |
| `ROBO_GPIO_BACKEND` | Override GPIO backend | Auto-detect |
| `ROBO_LOG_LEVEL` | Logging verbosity | `INFO` |

## See Also

- [Getting Started](getting-started.md)
- [Error Handling](error-handling.md)
- [API Reference](reference/)
