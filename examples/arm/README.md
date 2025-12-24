# Robot Arm Example

A 4-DOF robot arm using simulated servo actuators.

## Overview

This example demonstrates:
- Basic arm control with `JointGroup` controller
- LLM-controlled arm using ai-infra `Agent`
- REST API control using svc-infra dual routers

## Requirements

```bash
# Core requirement
pip install robo-infra

# For AI-controlled example
pip install ai-infra

# For API example
pip install svc-infra uvicorn
```

## Files

| File | Description |
|------|-------------|
| `arm.py` | Basic arm control without external dependencies |
| `arm_with_ai.py` | LLM-controlled arm using ai-infra Agent |
| `arm_with_api.py` | REST API for arm control using svc-infra |

## Running the Examples

### Basic Arm Control

```bash
python arm.py
```

This demonstrates:
- Creating a 4-DOF arm with simulated servos
- Moving joints to specific positions
- Using named positions (ready, rest, pickup)
- Homing the arm

### AI-Controlled Arm

```bash
# Set your LLM provider API key
export OPENAI_API_KEY=your-key-here
# Or for Anthropic:
export ANTHROPIC_API_KEY=your-key-here

python arm_with_ai.py
```

This demonstrates:
- Generating AI tools from the arm controller
- Natural language control ("Move the shoulder to 45 degrees")
- Streaming responses for real-time feedback
- Provider fallbacks for reliability

### REST API Control

```bash
python arm_with_api.py
```

Then visit:
- API docs: http://localhost:8000/docs
- Health check: http://localhost:8000/healthz

API endpoints:
- `GET /arm/status` - Get arm state and positions
- `POST /arm/enable` - Enable the arm
- `POST /arm/home` - Home all joints
- `POST /arm/move` - Move to target positions
- `POST /arm/stop` - Emergency stop

## Arm Configuration

The example arm has 4 joints:

| Joint | Range | Default | Description |
|-------|-------|---------|-------------|
| base | 0-360° | 180° | Pan rotation |
| shoulder | 0-180° | 90° | Shoulder joint |
| elbow | 0-150° | 75° | Elbow joint |
| wrist | 0-180° | 90° | Wrist rotation |

## Named Positions

Pre-defined positions for common operations:

| Name | base | shoulder | elbow | wrist |
|------|------|----------|-------|-------|
| ready | 180° | 45° | 90° | 90° |
| rest | 0° | 0° | 0° | 0° |
| pickup | 90° | 60° | 120° | 45° |

## Real Hardware

To use with real hardware, replace `SimulatedActuator` with actual `Servo` actuators:

```python
from robo_infra.actuators import Servo
from robo_infra.drivers import PCA9685

# Create PCA9685 driver for I2C servo control
driver = PCA9685(address=0x40)
driver.connect()
driver.enable()

# Create real servos
joints = {
    "base": Servo(name="base", driver=driver, channel=0, angle_range=(0, 360)),
    "shoulder": Servo(name="shoulder", driver=driver, channel=1, angle_range=(0, 180)),
    "elbow": Servo(name="elbow", driver=driver, channel=2, angle_range=(0, 150)),
    "wrist": Servo(name="wrist", driver=driver, channel=3, angle_range=(0, 180)),
}
```

## Learning More

- **robo-infra docs**: Controllers, actuators, and drivers
- **ai-infra docs**: Use nfrax-docs MCP with query `"ai-infra Agent tools"`
- **svc-infra docs**: Use nfrax-docs MCP with query `"svc-infra router_from_object"`
