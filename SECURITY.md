# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.1.x   | :white_check_mark: |
| < 0.1   | :x:                |

We actively support the latest minor version. Security patches are backported to the current minor release only.

## Reporting a Vulnerability

We take security vulnerabilities seriously. If you discover a security issue, please report it responsibly.

### How to Report

**Report Here**: [nfrax.com/?feedback=1](https://www.nfrax.com/?feedback=1)

Select "Security Issue" as the feedback type.

**Expected Response Time**:
- Initial acknowledgment: within 48 hours
- Status update: within 7 days
- Resolution timeline: depends on severity (critical: 7 days, high: 30 days, medium: 90 days)

### What to Include

Please include the following in your report:

1. **Description**: Clear description of the vulnerability
2. **Reproduction Steps**: Step-by-step instructions to reproduce the issue
3. **Impact Assessment**: What an attacker could achieve by exploiting this
4. **Affected Versions**: Which versions are affected (if known)
5. **Suggested Fix**: If you have ideas for remediation (optional)

### What NOT to Do

- Do not open public GitHub issues for security vulnerabilities
- Do not exploit the vulnerability beyond what's necessary to demonstrate it
- Do not access or modify data belonging to others

## Disclosure Policy

1. **Report received**: We acknowledge receipt within 48 hours
2. **Triage**: We assess severity and validity within 7 days
3. **Fix development**: We develop and test a fix
4. **Coordinated disclosure**: We coordinate with you on disclosure timing
5. **Public disclosure**: We publish a security advisory after the fix is released

We aim for a 90-day disclosure timeline, but may request extensions for complex issues.

## Security Update Process

1. Security fixes are released as patch versions (e.g., 0.1.x → 0.1.x+1)
2. All security releases include a GitHub Security Advisory
3. Users are notified via:
   - GitHub Security Advisories
   - CHANGELOG.md updates
   - PyPI release notes

## Robotics-Specific Security Considerations

robo-infra interacts with physical hardware, which introduces unique security considerations:

### Hardware Safety (`robo_infra.safety`)
- Emergency stop (E-Stop) systems
- Motion limits and safety bounds
- Watchdog timers for hardware communication
- Power monitoring and emergency shutdown

### Hardware Communication
- I2C/SPI bus access controls
- Serial port authentication
- CAN bus message validation
- GPIO pin state protection

**If you find vulnerabilities in hardware safety systems, please report them with CRITICAL priority.**

## Security Best Practices for Users

### Hardware Safety
```python
# Always use safety limits
from robo_infra.safety import EStop, MotionLimits

estop = EStop(callback=emergency_shutdown)
limits = MotionLimits(max_velocity=1.0, max_acceleration=0.5)

# Wrap hardware operations with safety context
with estop.monitor():
    robot.move(target_position, limits=limits)
```

### Environment Configuration
```python
# Never hardcode hardware addresses in production
import os

I2C_ADDRESS = int(os.environ.get("ROBOT_I2C_ADDRESS", "0x40"), 16)
CAN_INTERFACE = os.environ.get("ROBOT_CAN_INTERFACE", "can0")
```

### Input Validation
```python
from robo_infra.core.types import Limits, Position

# Always validate movement commands
limits = Limits(min_value=-180.0, max_value=180.0)
target = Position(x=user_input_x, y=user_input_y)

if not limits.contains(target.x) or not limits.contains(target.y):
    raise ValueError("Target position out of bounds")
```

### Simulation Mode
```python
# Use simulation mode for testing without physical hardware
from robo_infra.core import get_can

# Automatically falls back to simulation if hardware unavailable
can_bus = get_can(interface="can0", allow_simulation=True)
```

## Dependency Security

We regularly audit dependencies for vulnerabilities:

- **pydantic**: Data validation (security-critical for input sanitization)
- **svc-infra**: Backend infrastructure (OAuth, sessions, cryptography)
- **ai-infra**: AI/ML integration (prompt injection prevention)

Run `poetry audit` to check for known vulnerabilities in dependencies.
