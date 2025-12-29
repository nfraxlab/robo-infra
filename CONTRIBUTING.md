# Contributing to robo-infra

Thank you for your interest in contributing to robo-infra! This document provides guidelines for contributing.

## ⚠️ Robotics Safety Warning

**robo-infra controls physical robots. Bugs here can cause PHYSICAL HARM, PROPERTY DAMAGE, or DEATH.**

Before contributing, please read the safety standards in [.github/copilot-instructions.md](.github/copilot-instructions.md).

## Getting Started

### Prerequisites

- Python 3.11+
- Poetry for dependency management
- Git

### Development Setup

```bash
# Clone the repository
git clone https://github.com/nfraxlab/robo-infra.git
cd robo-infra

# Install dependencies
poetry install

# Activate the virtual environment
poetry shell

# Run tests (simulation mode)
ROBO_SIMULATION=true pytest -q

# Run linting
ruff check

# Run type checking
mypy src
```

## Robotics Safety Requirements

### Movement Limits

**All actuators MUST have enforced limits:**

```python
# ✅ Correct - Clamp to safe limits
def set_angle(self, angle: float):
    clamped = max(self.limits.min, min(self.limits.max, angle))
    if angle != clamped:
        logger.warning(f"Angle {angle} clamped to {clamped}")
    self._driver.set_pwm(clamped)

# ❌ WRONG - Could destroy mechanism
def set_angle(self, angle: float):
    self._driver.set_pwm(angle)
```

### Emergency Stop

**E-stop MUST disable first, then log:**

```python
# ✅ Correct - Disable IMMEDIATELY
def emergency_stop(self):
    self.disable_all()  # First priority, no exceptions
    try:
        self.log_event()
    except Exception as e:
        logger.error(f"Post-estop logging failed: {e}")

# ❌ WRONG - E-stop can be blocked
def emergency_stop(self):
    try:
        self.save_state()  # Could hang
    finally:
        self.disable_all()  # Too late
```

### Bounded Movement

**All movements need timeout and distance limits:**

```python
# ✅ Correct
async def move_until_contact(self, max_distance: float, timeout: float):
    deadline = time.time() + timeout
    start_pos = self.encoder.read()
    while not self.sensor.contact:
        if time.time() > deadline:
            raise TimeoutError("Contact not detected")
        if abs(self.encoder.read() - start_pos) > max_distance:
            raise LimitsExceededError("Max distance exceeded")
        self.motor.forward()

# ❌ WRONG - Runs forever if sensor fails
async def move_until_contact(self):
    while not self.sensor.contact:
        self.motor.forward()
```

### Explicit Simulation

**Never silently fall back to simulation:**

```python
# ✅ Correct - Require explicit flag
if not hardware_available:
    if not os.getenv("ROBO_SIMULATION"):
        raise HardwareNotFoundError("Set ROBO_SIMULATION=true to simulate")
    logger.warning("⚠️ SIMULATION MODE")
    driver = SimulatedDriver()

# ❌ WRONG - User thinks robot is real
if not hardware_available:
    driver = SimulatedDriver()
```

## Development Workflow

### Quick Start (Recommended)

Use `make pr` for the fastest workflow:

```bash
# 1. Make your code changes
# 2. Create a PR with one command:
make pr m="feat: add your feature"

# This automatically:
# - Validates gh CLI + origin remote
# - Fast-forwards main (no rebase on main)
# - Creates branch: add-your-feature-12281430 (UTC timestamp)
# - Commits and pushes
# - Creates PR (or detects existing)
# - Returns to main
```

**Context-aware behavior:**
```bash
# On main → creates new branch + PR
make pr m="feat: add caching"

# On feature branch → commits + pushes; creates PR if none exists
make pr m="feat: add more logic"

# On feature branch, sync with main first:
make pr m="feat: stuff" sync=1  # Rebases on main, force-pushes safely
```

### Manual Workflow

If you prefer manual git commands:

```bash
# 1. Create a branch
git checkout -b feature/your-feature-name

# 2. Test in simulation first
ROBO_SIMULATION=true pytest -q

# 3. Run quality checks
ruff format
ruff check
mypy src
pytest -q

# 4. Commit and push
git add -A
git commit -m "feat: your feature"
git push origin feature/your-feature-name

# 5. Open a PR on GitHub
```

### Batching Multiple Commits

For related changes, batch commits before creating a PR:

```bash
make commit m="feat: add base class"
make commit m="feat: add implementation"
make pr m="feat: complete feature"
```

## Testing Requirements

1. **Simulation tests first** - All code must work in simulation
2. **Limit violation tests** - Test at min, max, and beyond limits
3. **E-stop tests** - Verify all actuators disable
4. **Timeout tests** - What happens when hardware doesn't respond

## Project Structure

```
robo-infra/
├── src/robo_infra/    # Main package
│   ├── actuators/     # Servo, motor, etc.
│   ├── sensors/       # Distance, IMU, etc.
│   ├── drivers/       # Hardware drivers
│   ├── controllers/   # High-level control
│   └── safety/        # Limits, e-stop
├── tests/
├── docs/
└── examples/
```

## Required Checks Before PR

- [ ] All actuators have enforced limits
- [ ] E-stop disables immediately
- [ ] Movements have timeout/distance limits
- [ ] Simulation mode is explicit
- [ ] Tests pass in simulation
- [ ] `ruff check` passes
- [ ] `mypy src` passes

Thank you for contributing!
