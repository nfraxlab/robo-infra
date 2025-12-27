# Wiring Diagrams

This document provides wiring diagrams for robo-infra hardware testing setups.

## GPIO Pin Reference (Raspberry Pi 4)

```
                    Raspberry Pi 4 GPIO Header
                    ===========================
                    
        3.3V  (1) (2)  5V
       GPIO2  (3) (4)  5V
       GPIO3  (5) (6)  GND
       GPIO4  (7) (8)  GPIO14 (TX)
         GND  (9) (10) GPIO15 (RX)
      GPIO17 (11) (12) GPIO18 (PWM0)
      GPIO27 (13) (14) GND
      GPIO22 (15) (16) GPIO23
        3.3V (17) (18) GPIO24
      GPIO10 (19) (20) GND
       GPIO9 (21) (22) GPIO25
      GPIO11 (23) (24) GPIO8
         GND (25) (26) GPIO7
       GPIO0 (27) (28) GPIO1
       GPIO5 (29) (30) GND
       GPIO6 (31) (32) GPIO12 (PWM0)
      GPIO13 (33) (34) GND
      GPIO19 (35) (36) GPIO16
      GPIO26 (37) (38) GPIO20
         GND (39) (40) GPIO21

Key:
  I2C: GPIO2 (SDA), GPIO3 (SCL)
  SPI: GPIO10 (MOSI), GPIO9 (MISO), GPIO11 (SCLK), GPIO8 (CE0)
  PWM: GPIO12, GPIO13, GPIO18, GPIO19
```

---

## 1. Basic I2C Connection (PCA9685 + MPU6050)

```
                 Raspberry Pi                    PCA9685
                ┌───────────┐                  ┌─────────┐
                │           │                  │         │
   (Pin 1) 3.3V │●          │──────────────────│ VCC     │
   (Pin 3) SDA  │●          │──────────────────│ SDA     │
   (Pin 5) SCL  │●          │──────────────────│ SCL     │
   (Pin 6) GND  │●          │──────────────────│ GND     │
                │           │                  │         │
                │           │        External  │ V+   ●──┼────┐
                │           │        5V-6V ────│ VCC  ●  │    │
                │           │                  └─────────┘    │
                │           │                                 │
                │           │                  ┌─────────┐    │
                │           │                  │ MPU6050 │    │
   (Pin 1) 3.3V │●          │──────────────────│ VCC     │    │
   (Pin 3) SDA  │●          │──────────────────│ SDA     │    │
   (Pin 5) SCL  │●          │──────────────────│ SCL     │    │
   (Pin 6) GND  │●          │──────────────────│ GND     │    │
                │           │                  │ AD0  ●──┤ GND│
                └───────────┘                  └─────────┘    │
                                                              │
                                              GND ────────────┘
```

**Connections:**
| Raspberry Pi | PCA9685 | MPU6050 |
|--------------|---------|---------|
| Pin 1 (3.3V) | VCC | VCC |
| Pin 3 (SDA/GPIO2) | SDA | SDA |
| Pin 5 (SCL/GPIO3) | SCL | SCL |
| Pin 6 (GND) | GND | GND |
| External 5-6V | V+ | - |

**Notes:**
- PCA9685 V+ is for servo power (separate from logic)
- MPU6050 AD0 to GND sets address to 0x68
- Use 3.3V for logic, not 5V

---

## 2. Servo Control (via PCA9685)

```
                  PCA9685                         Servo (SG90)
                ┌─────────┐                      ┌─────────┐
                │         │                      │         │
                │ CH0  ●──┼──── Signal ──────────│ Orange  │
                │ V+   ●──┼──── Power (5-6V) ────│ Red     │
                │ GND  ●──┼──── Ground ──────────│ Brown   │
                │         │                      │         │
                │ CH1  ●──┼──── Signal ──────────│ Servo 2 │
                │ CH2  ●──┼──── Signal ──────────│ Servo 3 │
                │   ...   │                      │         │
                │ CH15 ●──┼──── Signal ──────────│ Servo 16│
                │         │                      │         │
                └─────────┘                      └─────────┘
```

**Servo Wire Colors:**
| Color | Purpose |
|-------|---------|
| Brown/Black | Ground (GND) |
| Red | Power (V+, 4.8-6V) |
| Orange/Yellow | Signal (PWM) |

**PWM Settings for SG90:**
- Frequency: 50Hz (20ms period)
- 0° position: ~500µs pulse (2.5% duty)
- 90° position: ~1500µs pulse (7.5% duty)
- 180° position: ~2400µs pulse (12% duty)

---

## 3. DC Motor Control (L298N)

```
        Raspberry Pi                   L298N                    Motor
       ┌───────────┐                ┌─────────┐              ┌─────────┐
       │           │                │         │              │         │
(Pin 12) GPIO18 ●──┼─── ENA ────────│ ENA     │              │ Motor A │
(Pin 16) GPIO23 ●──┼─── IN1 ────────│ IN1     │──── OUT1 ────│ Terminal│
(Pin 18) GPIO24 ●──┼─── IN2 ────────│ IN2     │──── OUT2 ────│ Terminal│
       │           │                │         │              │         │
(Pin 32) GPIO12 ●──┼─── ENB ────────│ ENB     │              │ Motor B │
(Pin 29) GPIO5  ●──┼─── IN3 ────────│ IN3     │──── OUT3 ────│ Terminal│
(Pin 31) GPIO6  ●──┼─── IN4 ────────│ IN4     │──── OUT4 ────│ Terminal│
       │           │                │         │              │         │
 (Pin 6) GND    ●──┼─── GND ────────│ GND     │              └─────────┘
       │           │                │         │
       │           │   6-12V ───────│ +12V    │
       │           │   (Battery)    │         │
       │           │                │ +5V ────┤ (Optional: power Pi)
       └───────────┘                └─────────┘
```

**L298N Motor Control Logic:**
| ENA | IN1 | IN2 | Motor A |
|-----|-----|-----|---------|
| LOW | X | X | Stopped |
| HIGH | LOW | LOW | Brake |
| HIGH | HIGH | LOW | Forward |
| HIGH | LOW | HIGH | Reverse |
| HIGH | HIGH | HIGH | Brake |
| PWM | HIGH | LOW | Forward (speed controlled) |

**Default Test Pins:**
| Function | GPIO | Pin |
|----------|------|-----|
| Motor PWM (ENA) | GPIO18 | 12 |
| Motor IN1 | GPIO23 | 16 |
| Motor IN2 | GPIO24 | 18 |

---

## 4. Complete Test Setup

```
                                    ┌────────────────────┐
                                    │   5V Power Supply  │
                                    │   (for servos)     │
                                    └─────────┬──────────┘
                                              │
    ┌─────────────┐                 ┌─────────▼──────────┐
    │             │    I2C          │      PCA9685       │
    │ Raspberry   │◄───────────────►│                    │──────► Servos
    │    Pi 4     │                 │  PWM Driver        │
    │             │                 └────────────────────┘
    │             │
    │             │    I2C          ┌────────────────────┐
    │             │◄───────────────►│      MPU6050       │
    │             │                 │        IMU         │
    │             │                 └────────────────────┘
    │             │
    │             │    GPIO         ┌────────────────────┐        ┌────────┐
    │             │────────────────►│       L298N        │───────►│ Motors │
    │             │                 │   Motor Driver     │        └────────┘
    └─────────────┘                 └─────────┬──────────┘
                                              │
                                    ┌─────────▼──────────┐
                                    │ 6-12V Battery Pack │
                                    │   (for motors)     │
                                    └────────────────────┘
```

---

## 5. GPIO Test Setup (LED + Button)

```
        Raspberry Pi                     LED Circuit
       ┌───────────┐                   ┌─────────────────┐
       │           │                   │                 │
(Pin 11) GPIO17 ●──┼───────────────────┤──►│ LED         │
       │           │                   │    ├──[220Ω]──┤ │
 (Pin 6) GND    ●──┼───────────────────┤───────────────GND│
       │           │                   │                 │
       │           │                   └─────────────────┘
       │           │
       │           │                     Button Circuit
       │           │                   ┌─────────────────┐
       │           │                   │                 │
(Pin 13) GPIO27 ●──┼───────────────────┤──[10kΩ]──┤     │
       │           │                   │          │     │
(Pin 1)  3.3V   ●──┼───────────────────┤──────────┴[BTN]│
 (Pin 6) GND    ●──┼───────────────────┤───────────────GND│
       │           │                   │                 │
       └───────────┘                   └─────────────────┘
```

**LED Circuit (Active High):**
- GPIO17 → 220Ω resistor → LED anode
- LED cathode → GND

**Button Circuit (Pull-down):**
- GPIO27 → 10kΩ resistor → GND (pull-down)
- GPIO27 → Button → 3.3V
- Reads LOW when not pressed, HIGH when pressed

---

## Safety Checklist

Before powering on:

- [ ] Double-check all connections
- [ ] Verify power supply voltages are correct
- [ ] Ensure no shorts between power and ground
- [ ] Use current-limiting resistors for LEDs
- [ ] Keep motor power separate from Pi power
- [ ] Secure motors to prevent movement during tests
- [ ] Have a way to quickly disconnect power

## Troubleshooting

### No I2C devices detected
1. Check SDA/SCL connections
2. Verify I2C is enabled: `sudo raspi-config`
3. Check for 3.3V at VCC pins
4. Run `i2cdetect -y 1`

### Servo jitters
1. Use separate power supply for servos
2. Add capacitor (100-470µF) near servo power
3. Check PWM frequency is 50Hz

### Motor won't spin
1. Check motor power supply (not from Pi!)
2. Verify enable pin is HIGH or PWM
3. Test with 100% duty cycle first
4. Check motor driver heat sink
