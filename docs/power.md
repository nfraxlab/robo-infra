# Power Management

Power management is critical for mobile and battery-powered robots. This guide covers battery monitoring, power distribution, and best practices.

## Overview

The power module provides:

- **Battery Management** - Monitor voltage, current, and state of charge
- **Power Distribution** - Control power rails with GPIO
- **Power Drivers** - INA219/INA226 power monitors

```python
from robo_infra.power import (
    BatteryMonitor, BatteryChemistry,
    PowerRail, PowerDistributionBoard,
    INA219Driver, INA226Driver,
)
```

---

## Battery Management

### Battery Types

| Chemistry | Nominal V/cell | Full V/cell | Empty V/cell | Notes |
|-----------|---------------|-------------|--------------|-------|
| **LiPo** | 3.7V | 4.2V | 3.0V | High energy, fire risk |
| **Li-ion** | 3.6V | 4.2V | 3.0V | Common, good balance |
| **LiFePO4** | 3.2V | 3.6V | 2.5V | Safe, long life |
| **NiMH** | 1.2V | 1.4V | 1.0V | Environmentally friendly |
| **Lead Acid** | 2.0V | 2.12V | 1.8V | Heavy, cheap |

### BatteryMonitor

```python
from robo_infra.power import BatteryMonitor, BatteryChemistry

# Create battery monitor for 3S LiPo
battery = BatteryMonitor(
    cells=3,
    chemistry=BatteryChemistry.LIPO,
)

battery.enable()

# Read battery state
print(f"Voltage: {battery.voltage:.2f}V")
print(f"Current: {battery.current:.2f}A")
print(f"Percentage: {battery.percentage:.0f}%")
print(f"Remaining capacity: {battery.remaining_mah:.0f}mAh")
print(f"Time remaining: {battery.time_remaining_minutes:.0f}min")

# Check state
if battery.is_low:
    print("WARNING: Battery low!")
if battery.is_critical:
    print("CRITICAL: Battery critical! Shutdown immediately!")

battery.disable()
```

### State of Charge Estimation

Voltage-based estimation:

```python
from robo_infra.power import BatteryMonitor, BatteryChemistry

battery = BatteryMonitor(
    cells=4,                       # 4S battery
    chemistry=BatteryChemistry.LIION,
    capacity_mah=5000,             # 5000mAh capacity
    low_voltage_threshold=3.3,     # Low warning per cell
    critical_voltage_threshold=3.1, # Critical shutdown per cell
)

battery.enable()

# Voltage-based SoC
print(f"SoC: {battery.state_of_charge:.1f}%")
```

Coulomb counting (more accurate):

```python
from robo_infra.power import BatteryMonitor, BatteryChemistry

battery = BatteryMonitor(
    cells=3,
    chemistry=BatteryChemistry.LIPO,
    capacity_mah=2200,
    use_coulomb_counting=True,  # Enable coulomb counting
)

battery.enable()

# Set initial SoC (e.g., from last shutdown)
battery.set_state_of_charge(85.0)

# Read coulomb-counted SoC
while True:
    # Battery current is measured and integrated
    print(f"SoC: {battery.state_of_charge:.1f}%")
    time.sleep(1.0)
```

### Low Voltage Protection

```python
from robo_infra.power import BatteryMonitor, BatteryChemistry
from robo_infra.safety import EStop

estop = EStop()

battery = BatteryMonitor(
    cells=3,
    chemistry=BatteryChemistry.LIPO,
    low_voltage_threshold=3.3,      # 3.3V/cell = ~20%
    critical_voltage_threshold=3.1,  # 3.1V/cell = ~5%
)

battery.enable()

# Callback on low battery
def on_low_battery(battery_status):
    print(f"WARNING: Battery at {battery_status.percentage:.0f}%")
    # Reduce motor power, notify user

battery.on_low_battery = on_low_battery

# Callback on critical battery
def on_critical_battery(battery_status):
    print("CRITICAL: Battery empty! Emergency shutdown!")
    estop.trigger("Critical battery level")

battery.on_critical_battery = on_critical_battery
```

### Charging Monitoring

```python
from robo_infra.power import BatteryMonitor, BatteryState

battery = BatteryMonitor(
    cells=3,
    chemistry=BatteryChemistry.LIPO,
)

battery.enable()

# Check charging state
state = battery.state
if state == BatteryState.CHARGING:
    print(f"Charging: {battery.percentage:.0f}%")
    print(f"Time to full: {battery.time_to_full_minutes:.0f}min")
elif state == BatteryState.FULL:
    print("Battery full!")
elif state == BatteryState.DISCHARGING:
    print(f"Discharging: {battery.percentage:.0f}%")

# Detect charging (current positive into battery)
if battery.current > 0.1:
    print("Battery is charging")
```

---

## Power Distribution

### PowerRail

Control individual power rails via GPIO:

```python
from robo_infra.power import PowerRail

# Create power rail controlled by GPIO 17
motors_rail = PowerRail(
    name="motors",
    enable_pin=17,
    active_high=True,  # High = enabled
)

# Enable/disable
motors_rail.enable()
time.sleep(0.1)  # Stabilization delay
# ... use motors ...
motors_rail.disable()
```

### Power Rail Configuration

```python
from robo_infra.power import PowerRail, PowerRailConfig, ShutdownPriority

config = PowerRailConfig(
    name="motors",
    enable_pin=17,
    active_high=True,
    max_current=10.0,         # Max 10A
    nominal_voltage=12.0,     # 12V rail
    enable_delay_ms=100,      # Wait 100ms after enable
    disable_delay_ms=50,      # Wait 50ms after disable
    shutdown_priority=ShutdownPriority.LOW,  # Shut down early
)

motors_rail = PowerRail(config=config)
```

### PowerDistributionBoard

Manage multiple power rails:

```python
from robo_infra.power import PowerRail, PowerDistributionBoard, ShutdownPriority

# Define power rails
rails = [
    PowerRail("safety", enable_pin=4, priority=ShutdownPriority.CRITICAL),
    PowerRail("compute", enable_pin=17, priority=ShutdownPriority.HIGH),
    PowerRail("sensors", enable_pin=27, priority=ShutdownPriority.NORMAL),
    PowerRail("motors", enable_pin=22, priority=ShutdownPriority.LOW),
    PowerRail("lights", enable_pin=23, priority=ShutdownPriority.LOWEST),
]

# Create power distribution board
pdb = PowerDistributionBoard(rails)

# Enable all rails in order
pdb.enable_all()

# Enable/disable individual rails
pdb.enable("motors")
pdb.disable("lights")

# Get rail status
for rail in pdb.rails:
    status = pdb.get_status(rail.name)
    print(f"{rail.name}: {status.state}")

# Emergency shutdown (disables in priority order)
pdb.emergency_shutdown()  # Lowest priority first, critical last
```

### Current Monitoring

```python
from robo_infra.power import PowerRail, PowerRailConfig
from robo_infra.power.drivers import INA219Driver

# Create current monitor for this rail
current_monitor = INA219Driver(address=0x40, shunt_ohms=0.1)

config = PowerRailConfig(
    name="motors",
    enable_pin=17,
    max_current=10.0,
)

motors_rail = PowerRail(config=config, current_monitor=current_monitor)
motors_rail.enable()

# Read current
reading = motors_rail.get_reading()
print(f"Voltage: {reading.voltage:.2f}V")
print(f"Current: {reading.current:.2f}A")
print(f"Power: {reading.power:.1f}W")

# Check for overcurrent
if reading.current > config.max_current:
    print("OVERCURRENT! Disabling rail.")
    motors_rail.disable()
```

### Fuse/Circuit Protection

```python
from robo_infra.power import PowerRail, PowerDistributionBoard

# Define rails with current limits
motors = PowerRail("motors", enable_pin=17, max_current=15.0)
sensors = PowerRail("sensors", enable_pin=27, max_current=2.0)

pdb = PowerDistributionBoard([motors, sensors])

# Monitor for overcurrent
def check_protection():
    for rail in pdb.rails:
        reading = pdb.get_reading(rail.name)
        if reading.current and reading.current > rail.max_current:
            print(f"FUSE: {rail.name} overcurrent ({reading.current:.1f}A > {rail.max_current}A)")
            pdb.disable(rail.name)

# Run protection check periodically
while True:
    check_protection()
    time.sleep(0.1)
```

---

## Power Drivers

### INA219 (12-bit Power Monitor)

```python
from robo_infra.power.drivers import INA219Driver, INA219Config

config = INA219Config(
    address=0x40,          # I2C address
    shunt_ohms=0.1,        # Shunt resistor value
    max_current=3.2,       # Max expected current
    bus_voltage_range=32,  # 16V or 32V range
)

driver = INA219Driver(config=config)
driver.enable()

# Read values
voltage = driver.read_voltage()    # Bus voltage (V)
current = driver.read_current()    # Current (A)
power = driver.read_power()        # Power (W)
shunt_voltage = driver.read_shunt_voltage()  # Shunt voltage (mV)

print(f"Voltage: {voltage:.2f}V")
print(f"Current: {current:.3f}A")
print(f"Power: {power:.2f}W")

driver.disable()
```

### INA226 (16-bit Power Monitor)

Higher resolution and wider range:

```python
from robo_infra.power.drivers import INA226Driver, INA226Config

config = INA226Config(
    address=0x40,
    shunt_ohms=0.01,       # 10mΩ shunt for high current
    max_current=50.0,      # Up to 50A
    averaging=16,          # 16 samples average
)

driver = INA226Driver(config=config)
driver.enable()

# High-resolution readings
voltage = driver.read_voltage()
current = driver.read_current()
power = driver.read_power()

# Alert configuration
driver.set_alert(
    alert_type="current_over",
    threshold=45.0,  # Alert at 45A
)

driver.disable()
```

### DC-DC Converters

Control adjustable DC-DC converters:

```python
from robo_infra.power.drivers import DCDCConverter

# PWM-controlled buck converter
converter = DCDCConverter(
    pwm_pin=18,
    voltage_min=3.3,
    voltage_max=12.0,
    feedback_adc=0,  # ADC channel for voltage feedback
)

converter.enable()

# Set output voltage
converter.set_voltage(5.0)

# Read actual voltage
actual = converter.read_voltage()
print(f"Output: {actual:.2f}V")

converter.disable()
```

---

## Best Practices

### Battery Safety

[!] **LiPo/Li-ion batteries are dangerous if mishandled!**

```python
from robo_infra.power import BatteryMonitor, BatteryChemistry

# ALWAYS set appropriate limits
battery = BatteryMonitor(
    cells=3,
    chemistry=BatteryChemistry.LIPO,
    low_voltage_threshold=3.3,      # Never go below 3.0V/cell
    critical_voltage_threshold=3.1,
    max_charge_current=2.2,         # 1C charge rate for 2200mAh
    max_discharge_current=22.0,     # 10C discharge
)

# ALWAYS monitor temperature (if available)
def check_temperature():
    temp = battery.temperature
    if temp > 45:
        print("WARNING: Battery overheating!")
        estop.trigger("Battery temperature high")
    if temp < 0:
        print("WARNING: Battery too cold for charging!")

# NEVER leave LiPo unattended while charging
# NEVER charge or discharge below 0°C or above 45°C
# ALWAYS use a proper LiPo charger
# ALWAYS store at ~50% charge
```

### Cable Sizing

Choose wire gauge based on current and length:

| AWG | Max Current (chassis) | Resistance (Ω/m) |
|-----|----------------------|------------------|
| 10 | 30A | 0.00328 |
| 12 | 20A | 0.00521 |
| 14 | 15A | 0.00829 |
| 16 | 10A | 0.0132 |
| 18 | 7A | 0.0210 |
| 20 | 5A | 0.0333 |
| 22 | 3A | 0.0530 |

```python
def calculate_voltage_drop(current_a, length_m, awg):
    """Calculate voltage drop in a wire."""
    resistance_per_m = {
        10: 0.00328, 12: 0.00521, 14: 0.00829,
        16: 0.0132, 18: 0.0210, 20: 0.0333, 22: 0.0530,
    }
    r = resistance_per_m[awg]
    # Round trip (positive and negative)
    total_resistance = r * length_m * 2
    voltage_drop = current_a * total_resistance
    return voltage_drop

# Example: 5A, 2m cable, 16 AWG
drop = calculate_voltage_drop(5, 2, 16)
print(f"Voltage drop: {drop:.2f}V")  # ~0.26V
```

### EMI Considerations

```python
# Best practices for EMI reduction:

# 1. Keep motor wires short and twisted
# 2. Use shielded cables for signals near motors
# 3. Add decoupling capacitors near motors
#    - 100nF ceramic close to driver
#    - 100-1000µF electrolytic on power rail

# 4. Separate power and signal grounds
# 5. Use ferrite beads on motor power lines

# 6. Software filtering for ADC readings
from robo_infra.power import BatteryMonitor

battery = BatteryMonitor(
    cells=3,
    chemistry=BatteryChemistry.LIPO,
    filter_samples=10,  # Average 10 samples
    filter_type="median",  # Median filter (rejects spikes)
)
```

---

## Complete Power Example

```python
from robo_infra.power import (
    BatteryMonitor, BatteryChemistry,
    PowerRail, PowerDistributionBoard, ShutdownPriority,
)
from robo_infra.power.drivers import INA219Driver
from robo_infra.safety import EStop
import time

# Create E-stop
estop = EStop()

# Create battery monitor
battery = BatteryMonitor(
    cells=4,
    chemistry=BatteryChemistry.LIPO,
    capacity_mah=5000,
    low_voltage_threshold=3.4,
    critical_voltage_threshold=3.2,
)

# Create power rails with current monitoring
motor_monitor = INA219Driver(address=0x40, shunt_ohms=0.01)
sensor_monitor = INA219Driver(address=0x41, shunt_ohms=0.1)

rails = [
    PowerRail("safety", enable_pin=4, priority=ShutdownPriority.CRITICAL),
    PowerRail("motors", enable_pin=17, priority=ShutdownPriority.LOW,
              current_monitor=motor_monitor, max_current=20.0),
    PowerRail("sensors", enable_pin=27, priority=ShutdownPriority.NORMAL,
              current_monitor=sensor_monitor, max_current=2.0),
]

pdb = PowerDistributionBoard(rails)

# Register E-stop for emergency shutdown
estop.register_actuator(pdb)

# Callbacks
def on_low_battery(status):
    print(f"Low battery: {status.percentage:.0f}%")
    # Reduce motor power
    pdb.set_power_limit("motors", 0.5)

def on_critical_battery(status):
    print("Critical battery!")
    estop.trigger("Critical battery level")

battery.on_low_battery = on_low_battery
battery.on_critical_battery = on_critical_battery

# Start systems
battery.enable()
pdb.enable_all()

try:
    while True:
        # Monitor battery
        print(f"Battery: {battery.voltage:.1f}V ({battery.percentage:.0f}%)")
        
        # Monitor power rails
        for rail in pdb.rails:
            reading = pdb.get_reading(rail.name)
            if reading.current:
                print(f"  {rail.name}: {reading.current:.2f}A")
                
                # Overcurrent protection
                if rail.max_current and reading.current > rail.max_current:
                    print(f"  OVERCURRENT on {rail.name}!")
                    pdb.disable(rail.name)
        
        time.sleep(1.0)

except KeyboardInterrupt:
    pass
finally:
    pdb.disable_all()
    battery.disable()
```

---

## Next Steps

- [Protocols](protocols.md) - Industrial protocols for power equipment
- [Safety](safety.md) - E-stop and safety systems
- [Drivers](drivers.md) - Motor drivers and power stages
- [Sensors](sensors.md) - Current and voltage sensors
