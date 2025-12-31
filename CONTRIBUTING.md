# Contributing to robo-infra

Thank you for your interest in contributing to robo-infra! This document provides guidelines for contributing.

## [!] Robotics Safety Warning

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
# [OK] Correct - Clamp to safe limits
def set_angle(self, angle: float):
    clamped = max(self.limits.min, min(self.limits.max, angle))
    if angle != clamped:
        logger.warning(f"Angle {angle} clamped to {clamped}")
    self._driver.set_pwm(clamped)

# [X] WRONG - Could destroy mechanism
def set_angle(self, angle: float):
    self._driver.set_pwm(angle)
```

### Emergency Stop

**E-stop MUST disable first, then log:**

```python
# [OK] Correct - Disable IMMEDIATELY
def emergency_stop(self):
    self.disable_all()  # First priority, no exceptions
    try:
        self.log_event()
    except Exception as e:
        logger.error(f"Post-estop logging failed: {e}")

# [X] WRONG - E-stop can be blocked
def emergency_stop(self):
    try:
        self.save_state()  # Could hang
    finally:
        self.disable_all()  # Too late
```

### Bounded Movement

**All movements need timeout and distance limits:**

```python
# [OK] Correct
async def move_until_contact(self, max_distance: float, timeout: float):
    deadline = time.time() + timeout
    start_pos = self.encoder.read()
    while not self.sensor.contact:
        if time.time() > deadline:
            raise TimeoutError("Contact not detected")
        if abs(self.encoder.read() - start_pos) > max_distance:
            raise LimitsExceededError("Max distance exceeded")
        self.motor.forward()

# [X] WRONG - Runs forever if sensor fails
async def move_until_contact(self):
    while not self.sensor.contact:
        self.motor.forward()
```

### Explicit Simulation

**Never silently fall back to simulation:**

```python
# [OK] Correct - Require explicit flag
if not hardware_available:
    if not os.getenv("ROBO_SIMULATION"):
        raise HardwareNotFoundError("Set ROBO_SIMULATION=true to simulate")
    logger.warning("[!] SIMULATION MODE")
    driver = SimulatedDriver()

# [X] WRONG - User thinks robot is real
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
```

### Two-Mode Workflow

**Mode A: Start a new PR** (on default branch OR with `new=1`)
```bash
# On main → creates new branch + PR, stays on new branch
make pr m="feat: add motor control"

# On feature branch → split commits into new PR
make pr m="feat: split this work" new=1
```

**Mode B: Update current PR** (on feature branch)
```bash
# Add more commits to existing PR
make pr m="fix: address review feedback"

# Sync with main before pushing (rebase + force-push)
make pr m="fix: sync and update" sync=1
```

### All Options

| Option | Example | Description |
|--------|---------|-------------|
| `m=` | `m="feat: add X"` | Commit message (required, conventional commits) |
| `sync=1` | `sync=1` | Rebase on base branch before pushing |
| `new=1` | `new=1` | Force create new PR from current HEAD |
| `b=` | `b="my-branch"` | Use explicit branch name |
| `draft=1` | `draft=1` | Create PR as draft |
| `base=` | `base=develop` | Target different base branch |
| `FORCE=1` | `FORCE=1` | Skip conventional commit validation |

### Examples

```bash
# Basic: create PR from main
make pr m="feat: add servo controller"

# Add commits to existing PR
make pr m="fix: handle edge case"

# Sync with main, then push
make pr m="refactor: clean up" sync=1

# Create draft PR
make pr m="feat: work in progress" draft=1

# Target a release branch
make pr m="fix: hotfix" base=release-v1

# Split work into new PR from feature branch
make pr m="feat: extract this part" new=1

# Batch commits before PR
make commit m="feat: add base class"
make commit m="feat: add implementation"
make pr m="feat: complete servo support"
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

### PR Title Enforcement

A GitHub Action automatically ensures your PR title reflects the highest-priority commit type:

1. Scans all commits in the PR for conventional commit prefixes
2. Auto-updates the PR title if needed (e.g., `docs:` → `feat:` if there's a `feat:` commit)
3. Passes with a warning after update

**Priority order:** `feat!` > `feat` > `fix` > `perf` > `refactor` > `docs` > `chore` > `test` > `ci` > `build`

This ensures squash-merge commits trigger the correct semantic-release.

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
