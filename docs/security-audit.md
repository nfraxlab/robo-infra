# Security Audit - robo-infra

> **Last Updated**: 2026-01-04
> **Reviewer**: Automated Security Audit
> **Bandit Version**: 1.9.2

## Overview

This document records security-relevant code patterns in robo-infra and their justifications.

## Bandit Skip Justifications

### B603/B607: Subprocess Calls

robo-infra uses subprocess for hardware platform management. All calls use:
- Fixed command lists (no shell=True)
- No user input in command arguments
- Validated paths from internal sources

| File | Line | Command | Justification |
|------|------|---------|---------------|
| `platforms/beaglebone.py` | 884 | `sudo cp` | PRU firmware loading; firmware_path validated via Path.exists() check |
| `platforms/jetson.py` | 619 | `nvpmodel -q` | Fixed command to query power mode |
| `platforms/jetson.py` | 665 | `sudo nvpmodel -m` | mode_id from internal enum mapping only |
| `platforms/jetson.py` | 734 | `nvcc --version` | Fixed command for CUDA version detection |
| `platforms/jetson.py` | 771 | `v4l2-ctl --info` | device path from enumerated /dev/videoN only |

### B404: Subprocess Import

The subprocess module is imported in platform files that require system command execution for hardware control. This is intentional and necessary for:
- PRU (Programmable Realtime Unit) firmware management on BeagleBone
- Power mode management on NVIDIA Jetson
- Hardware capability detection

### B311: Random Module

Standard random is used for:
- Sensor noise simulation (non-security purpose)
- RRT path planning random sampling (algorithmic, not cryptographic)

These are appropriate uses where cryptographic randomness is not required.

### B110: Try-Except-Pass

Used in hardware detection code where:
- Multiple detection methods are attempted
- Failure of one method should not prevent others
- Specific errors are expected when hardware is not present

### B101: Assert Used

Assertions are used for internal invariant checking in controller logic. These are development aids that verify preconditions in non-production paths.

## Hardware Access Patterns

### GPIO/I2C/SPI Access

robo-infra accesses hardware through:
- Standard Linux sysfs interfaces (`/sys/class/gpio/`, `/dev/i2c-*`, `/dev/spidev*`)
- Platform-specific libraries (Adafruit_BBIO, RPi.GPIO, Jetson.GPIO)

**Security Considerations**:
- Hardware access requires appropriate user permissions (gpio group, i2c group)
- No privilege escalation within the library
- Users should follow principle of least privilege when configuring access

### Device File Access

Device files accessed:
- `/dev/i2c-*` - I2C bus communication
- `/dev/spidev*` - SPI bus communication
- `/dev/ttyUSB*`, `/dev/ttyACM*` - Serial communication
- `/dev/video*` - Camera devices

All paths are constructed from enumerated device indices, not user input.

## Recommendations for Users

1. **Run with minimal privileges**: Use udev rules to grant hardware access to non-root users
2. **Validate firmware sources**: Only load PRU/MCU firmware from trusted sources
3. **Network isolation**: If using remote control features, isolate robot networks
4. **Update regularly**: Keep robo-infra and platform libraries updated

## Audit History

| Date | Version | Auditor | Notes |
|------|---------|---------|-------|
| 2026-01-04 | 0.6.x | Automated | Initial audit, nosec comments added |
