# AI Integration Guide

This guide covers integrating robo-infra with ai-infra for LLM-controlled robotics.

## Overview

robo-infra provides seamless integration with ai-infra, allowing you to control
robots using natural language through LLM agents. The integration uses ai-infra's
generic `tools_from_object()` utility for base method extraction while adding
robotics-specific enhancements.

## Quick Start

```python
from robo_infra.core.controller import SimulatedController
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits
from robo_infra.integrations.ai_infra import controller_to_tools

# Create a controller with actuators
controller = SimulatedController(name="arm")
controller.add_actuator(
    "shoulder",
    SimulatedActuator(
        name="shoulder",
        limits=Limits(min=0, max=180, default=90),
        unit="degrees",
    ),
)
controller.enable()
controller.home()

# Convert to AI tools
tools = controller_to_tools(controller)

# Use with ai-infra Agent
from ai_infra import Agent
agent = Agent(tools=tools)
result = agent.run("Move the shoulder to 45 degrees")
```

## Generated Tools

`controller_to_tools()` generates the following tools for LLM agents:

| Tool | Description |
|------|-------------|
| `{name}_enable` | Enable the controller and all actuators |
| `{name}_disable` | Disable the controller and all actuators |
| `{name}_home` | Move all actuators to home positions |
| `{name}_stop` | Emergency stop - halt all motion immediately |
| `{name}_move` | Move actuators to target positions |
| `{name}_status` | Get current controller state and positions |
| `{name}_sensors` | Read all sensor values (if sensors attached) |

## Using ai-infra Generic Utilities

robo-infra re-exports ai-infra's generic utilities for convenience:

```python
from robo_infra.integrations.ai_infra import (
    tools_from_object,  # Generic object-to-tools conversion
    tool_exclude,       # Decorator to exclude methods from tool generation
)
```

### tools_from_object

The `tools_from_object()` utility from ai-infra automatically converts any
Python object's methods into LLM-compatible tools:

```python
from robo_infra.integrations.ai_infra import tools_from_object

class MyService:
    def process_data(self, data: str) -> str:
        """Process the input data."""
        return f"Processed: {data}"
    
    def get_status(self) -> dict:
        """Get current service status."""
        return {"status": "ok"}

tools = tools_from_object(MyService(), prefix="svc")
# Creates: svc_process_data, svc_get_status
```

### tool_exclude Decorator

Use `@tool_exclude` to mark methods that should not become tools:

```python
from robo_infra.integrations.ai_infra import tool_exclude, tools_from_object

class Service:
    def public_action(self) -> str:
        """This becomes a tool."""
        return "action complete"
    
    @tool_exclude
    def internal_helper(self) -> str:
        """This is excluded from tools."""
        return "internal"

tools = tools_from_object(Service(), prefix="svc")
# Only creates: svc_public_action (internal_helper is excluded)
```

## Robotics-Specific Enhancements

robo-infra's integration adds robotics-specific features beyond the generic
utility:

### Dynamic Docstrings

Tool docstrings are enhanced with runtime information about actuator limits
and available sensors:

```python
# The generated move tool's docstring includes:
# "Available actuators: shoulder (0-180), elbow (0-120)"
```

### Formatted Status Responses

The status tool returns formatted dictionaries with controller state:

```python
status = tools[0]()  # Call status tool
# Returns:
# {
#     "controller": "arm",
#     "state": "idle",
#     "is_enabled": True,
#     "is_homed": True,
#     "actuators": {"shoulder": 90.0, "elbow": 60.0}
# }
```

### Safety Warnings

The emergency stop tool includes safety warnings in its docstring:

```python
# stop tool docstring:
# "EMERGENCY STOP - immediately halt all motion.
#  USE ONLY IN EMERGENCIES. The controller must be
#  re-enabled and re-homed after an emergency stop."
```

## Individual Actuator Tools

For fine-grained control, you can also create tools for individual actuators:

```python
from robo_infra.integrations.ai_infra import actuator_to_tools

actuator = SimulatedActuator(
    name="servo",
    limits=Limits(min=0, max=180, default=90),
    unit="degrees",
)

tools = actuator_to_tools(actuator)
# Creates: servo_enable, servo_disable, servo_set, servo_get
```

## Best Practices

1. **Enable and home before use**: Always call `enable()` and `home()` on
   controllers before generating tools for agent use.

2. **Include safety tools**: The emergency stop tool should always be available
   to the agent for safety.

3. **Use descriptive names**: Controller and actuator names appear in tool names
   and should be meaningful to the LLM.

4. **Test in simulation first**: Use `SimulatedController` and `SimulatedActuator`
   to test agent behavior before connecting real hardware.

## See Also

- [ai-infra tools_from_object documentation](../../../ai-infra/docs/tools.md)
- [Getting Started Guide](getting-started.md)
- [API Integration Guide](api-integration.md)
