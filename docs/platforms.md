# Platform Support

robo-infra provides hardware abstraction across multiple platforms, enabling code portability between different single-board computers (SBCs), microcontrollers, and development environments.

## Supported Platforms

| Platform | GPIO | PWM | I2C | SPI | UART | ADC | Status |
|----------|------|-----|-----|-----|------|-----|--------|
| **Raspberry Pi** (all models) | [OK] | [OK] | [OK] | [OK] | [OK] | — | Production |
| **NVIDIA Jetson** (Nano, TX2, Xavier, Orin) | [OK] | [OK] | [OK] | [OK] | [OK] | — | Production |
| **BeagleBone** (Black, Green, AI) | [OK] | [OK] | [OK] | [OK] | [OK] | [OK] | Production |
| **Arduino** (Uno, Mega, Nano, Due) | [OK] | [OK] | — | — | — | [OK] | Production |
| **ESP32** (all variants) | [OK] | [OK] | [OK] | [OK] | [OK] | [OK] | Production |
| **Linux Generic** (Orange Pi, Rock Pi, etc.) | [OK] | [OK] | [OK] | [OK] | [OK] | — | Production |
| **Simulation** (development) | [OK] | [OK] | [OK] | [OK] | [OK] | [OK] | Production |

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

### Cross-Platform Factory Functions (Recommended)

The easiest way to access hardware is through factory functions:

```python
from robo_infra.platforms import get_gpio, get_i2c, get_spi, get_uart
from robo_infra.core.pin import PinMode

# GPIO - works on any platform
led = get_gpio(17, mode=PinMode.OUTPUT)
led.setup()
led.high()

button = get_gpio(18, mode=PinMode.INPUT, pull="up")
button.setup()
if button.read() == 0:
    print("Button pressed!")

# PWM for servos/motors
servo = get_gpio(12, mode=PinMode.PWM, frequency=50)
servo.setup()
servo.duty_cycle = 7.5  # Center position

# I2C - auto-detects platform-appropriate bus
i2c = get_i2c(bus=1)
devices = i2c.scan()
print(f"Found I2C devices at: {[hex(d) for d in devices]}")

# SPI - with speed and mode configuration
spi = get_spi(bus=0, device=0, speed_hz=1_000_000, mode=0)
response = spi.transfer([0x01, 0x02, 0x03])

# UART - for serial devices
uart = get_uart(port="/dev/ttyAMA0", baudrate=9600)
```

### Discover Available Hardware

```python
from robo_infra.platforms import (
    list_available_gpio,
    list_available_i2c,
    list_available_spi,
    list_available_uart,
)

# See what's available on this platform
print(f"GPIO chips: {list_available_gpio()}")
print(f"I2C buses: {list_available_i2c()}")  # e.g., [0, 1]
print(f"SPI buses: {list_available_spi()}")  # e.g., [(0, 0), (0, 1)]
print(f"UART ports: {list_available_uart()}")
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

#### Pi 4 vs Pi 5 Differences

| Feature | Raspberry Pi 4 | Raspberry Pi 5 |
|---------|---------------|----------------|
| GPIO Chip | BCM2711 | RP1 (new) |
| Recommended Backend | RPi.GPIO, lgpio | lgpio, gpiod |
| RPi.GPIO Support | [OK] Full | [X] Not supported |
| Hardware PWM Pins | 12, 13, 18, 19 | 12, 13, 14, 15, 18, 19 |
| I2C Buses | 1 (user), 0 (reserved) | 1 (user), 0, 3 |
| GPIO Speed | Up to 50MHz | Up to 100MHz |
| Kernel Driver | bcm2835-gpio | pinctrl-rp1 |

**Pi 5 Specific Notes:**

```python
from robo_infra.platforms import RaspberryPiPlatform, GPIOBackend
from robo_infra.platforms.detection import is_raspberry_pi

# Check if running on Pi 5
is_pi5, model = is_raspberry_pi()
if is_pi5:
    # Pi 5 requires lgpio or gpiod backend
    pi = RaspberryPiPlatform(backend=GPIOBackend.LGPIO)
    
    # Pi 5 has different GPIO chip path
    # /dev/gpiochip4 instead of /dev/gpiochip0
    print(f"GPIO chips: {pi.info.gpio_chips}")
```

**Common Pi 5 Issues:**

1. **RPi.GPIO not working**: Use lgpio instead
   ```bash
   pip install lgpio
   ```

2. **Permission denied on GPIO**: Add user to gpio group
   ```bash
   sudo usermod -aG gpio $USER
   ```

3. **I2C not detected**: Enable in raspi-config and check /dev/i2c-1

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

#### libgpiod (Kernel GPIO Interface)

libgpiod is the modern way to access GPIO on Linux. It works on all Linux SBCs and is the recommended approach for new projects.

**Installation:**

```bash
# Debian/Ubuntu
sudo apt-get install gpiod libgpiod-dev python3-libgpiod

# Or via pip
pip install gpiod
```

**GPIO Chip Model:**

Unlike Raspberry Pi's BCM numbering, libgpiod uses a chip/line model:

```python
from robo_infra.platforms import get_gpio
from robo_infra.platforms.linux_generic import LinuxGenericPlatform

# Using factory function (auto-detects chip)
led = get_gpio(17)  # Uses default chip

# Explicit chip/line
linux = LinuxGenericPlatform()
led = linux.get_pin(line=17, chip=0, mode="output")

# List all GPIO chips
import subprocess
result = subprocess.run(["gpiodetect"], capture_output=True, text=True)
print(result.stdout)
# Output:
# gpiochip0 [pinctrl-bcm2711] (58 lines)
# gpiochip1 [raspberrypi-exp-gpio] (8 lines)
```

**GPIO Backends for Linux Generic:**

- `LinuxGPIOBackend.GPIOD` - libgpiod (recommended, kernel 4.8+)
- `LinuxGPIOBackend.SYSFS` - Legacy sysfs interface (deprecated)
- `LinuxGPIOBackend.SIMULATION` - No hardware

**Edge Detection:**

```python
from robo_infra.platforms.linux_generic import GPIOEdge

# Setup pin with edge detection
button = linux.get_pin(17, mode="input", edge=GPIOEdge.FALLING)

# Wait for edge (blocking)
button.wait_for_edge(timeout=5.0)

# Or use callback
def on_button_press(pin):
    print(f"Button pressed on line {pin}")

button.add_event_callback(on_button_press)
```

**Finding GPIO Lines:**

```bash
# List all GPIO chips
gpiodetect

# Get info about specific chip
gpioinfo gpiochip0

# Test a GPIO line
gpioget gpiochip0 17
gpioset gpiochip0 17=1
```

## Platform Detection

robo-infra provides comprehensive detection utilities for identifying hardware:

### Basic Detection

```python
from robo_infra.platforms import detect_platform, PlatformType

# Detect current platform type
platform_type = detect_platform()
print(f"Platform: {platform_type}")  # e.g., PlatformType.RASPBERRY_PI

# Check specific platform types
if platform_type == PlatformType.RASPBERRY_PI:
    print("Running on Raspberry Pi")
elif platform_type == PlatformType.JETSON:
    print("Running on NVIDIA Jetson")
elif platform_type == PlatformType.SIMULATION:
    print("Running in simulation mode")
```

### Convenience Functions

Quick boolean checks for common platforms:

```python
from robo_infra.platforms.detection import (
    is_simulation_mode,
    is_raspberry_pi,
    is_jetson,
    is_beaglebone,
    is_arduino_connected,
    is_esp32_connected,
)

# Check simulation mode
if is_simulation_mode():
    print("Running without hardware")

# Check specific platforms
if is_raspberry_pi():
    print("Raspberry Pi detected")

if is_jetson():
    print("NVIDIA Jetson detected")

if is_beaglebone():
    print("BeagleBone detected")

# Check connected microcontrollers (via USB)
if is_arduino_connected():
    print("Arduino connected via USB")

if is_esp32_connected():
    print("ESP32 connected via USB")
```

### Detailed Detection

Get detailed platform information:

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

# Platform-specific detection with model info
is_pi, model = detect_raspberry_pi()
if is_pi:
    print(f"Raspberry Pi: {model}")  # e.g., "Raspberry Pi 4 Model B"

is_jetson, model = detect_jetson()
if is_jetson:
    print(f"Jetson: {model}")  # e.g., "Jetson Orin Nano"

# Comprehensive platform info
info = get_platform_info()
print(f"Platform: {info.platform_type}")
print(f"Model: {info.model}")
print(f"Revision: {info.revision}")
print(f"Serial: {info.serial}")
print(f"Capabilities: {info.capabilities}")
print(f"GPIO Chips: {info.gpio_chips}")
print(f"I2C Buses: {info.i2c_buses}")
print(f"SPI Buses: {info.spi_buses}")
print(f"UART Ports: {info.uart_ports}")
```

### Environment Variable Override

Force a specific platform or simulation mode:

```bash
# Force simulation mode
ROBO_SIMULATION=true python my_robot.py

# Force specific platform
ROBO_PLATFORM=simulation python my_robot.py
ROBO_PLATFORM=raspberry_pi python my_robot.py
ROBO_PLATFORM=jetson python my_robot.py
```

```python
import os
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.platforms import detect_platform, PlatformType

# Will return SIMULATION regardless of hardware
assert detect_platform() == PlatformType.SIMULATION
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
| `ROBO_PLATFORM` | Force specific platform | Auto-detect |
| `ROBO_GPIO_BACKEND` | Override GPIO backend | Auto-detect |
| `ROBO_LOG_LEVEL` | Logging verbosity | `INFO` |

## Troubleshooting

### Platform Detection Issues

**Platform not detected correctly:**

```python
from robo_infra.platforms import detect_platform, get_platform_info
import logging

# Enable debug logging
logging.basicConfig(level=logging.DEBUG)

# Check detection
platform_type = detect_platform()
info = get_platform_info()
print(f"Detected: {platform_type}")
print(f"Model: {info.model}")
print(f"GPIO chips: {info.gpio_chips}")
```

**Force a specific platform:**

```bash
# Override detection
ROBO_PLATFORM=raspberry_pi python my_robot.py
ROBO_PLATFORM=linux python my_robot.py
```

### GPIO Issues

**Permission denied:**

```bash
# Add user to gpio group
sudo usermod -aG gpio $USER

# Re-login or run
newgrp gpio

# Verify
groups  # Should show 'gpio'
```

**GPIO not working on Pi 5:**

```python
# Pi 5 requires lgpio or gpiod, not RPi.GPIO
from robo_infra.platforms import RaspberryPiPlatform, GPIOBackend

# Use lgpio backend
pi = RaspberryPiPlatform(backend=GPIOBackend.LGPIO)

# Or install lgpio
# pip install lgpio
```

**Pin busy or in use:**

```bash
# Check what's using the GPIO
sudo cat /sys/kernel/debug/gpio

# Release pins from sysfs
echo 17 > /sys/class/gpio/unexport
```

### I2C Issues

**I2C bus not found:**

```bash
# Enable I2C
sudo raspi-config
# Interface Options → I2C → Enable

# Verify I2C is enabled
ls /dev/i2c*
# Should show /dev/i2c-1

# Check for devices
i2cdetect -y 1
```

**No devices detected:**

```bash
# Check wiring (SDA, SCL, VCC, GND)
# Verify pull-up resistors (4.7kΩ to 3.3V)
# Try slower I2C speed
sudo nano /boot/config.txt
# Add: dtparam=i2c_baudrate=50000
```

**I2C permission denied:**

```bash
# Add user to i2c group
sudo usermod -aG i2c $USER

# Or run with sudo (not recommended for production)
sudo python my_robot.py
```

### SPI Issues

**SPI not enabled:**

```bash
# Enable SPI
sudo raspi-config
# Interface Options → SPI → Enable

# Verify
ls /dev/spidev*
# Should show /dev/spidev0.0, /dev/spidev0.1
```

**SPI permission denied:**

```bash
# Add user to spi group
sudo usermod -aG spi $USER
```

### Simulation Mode Issues

**Not entering simulation mode:**

```python
import os

# Must set before importing robo_infra
os.environ["ROBO_SIMULATION"] = "true"

# Now import
from robo_infra.platforms import detect_platform, PlatformType
assert detect_platform() == PlatformType.SIMULATION
```

**Simulation not behaving correctly:**

```python
from robo_infra.platforms import reset_platform

# Reset and force simulation
reset_platform()
os.environ["ROBO_SIMULATION"] = "true"

from robo_infra.platforms import get_platform
platform = get_platform()  # Fresh simulation platform
```

### Common Error Messages

| Error | Cause | Solution |
|-------|-------|----------|
| `ModuleNotFoundError: lgpio` | lgpio not installed | `pip install lgpio` |
| `ModuleNotFoundError: gpiod` | gpiod not installed | `pip install gpiod` |
| `PermissionError: /dev/gpiomem` | No GPIO permissions | `sudo usermod -aG gpio $USER` |
| `FileNotFoundError: /dev/i2c-1` | I2C not enabled | Enable in raspi-config |
| `OSError: [Errno 121] Remote I/O error` | I2C device not responding | Check wiring and address |
| `HardwareNotFoundError: GPIO` | Platform lacks GPIO | Check platform capabilities |
| `RuntimeError: Cannot determine SOC peripheral` | Pi detection failed | Use lgpio or set ROBO_PLATFORM |

### Debug Mode

Enable verbose logging for troubleshooting:

```python
import logging

# Set debug level for all robo_infra modules
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Or just platforms module
logging.getLogger('robo_infra.platforms').setLevel(logging.DEBUG)
```

```bash
# Via environment variable
ROBO_LOG_LEVEL=DEBUG python my_robot.py
```

### Getting Help

If you're still having issues:

1. **Check the logs** - Enable DEBUG logging
2. **Verify hardware** - Use `i2cdetect`, `gpiodetect` system tools
3. **Test in simulation** - Isolate software vs hardware issues
4. **Check permissions** - Ensure user is in gpio, i2c, spi groups
5. **Try explicit platform** - Use `ROBO_PLATFORM=xxx` to force detection

## See Also

- [Getting Started](getting-started.md)
- [Hardware Setup](hardware-setup.md)
- [Hardware Testing](hardware-testing.md)
- [Hardware Wiring](hardware-wiring.md)
- [Error Handling](error-handling.md)
- [API Reference](reference/)
