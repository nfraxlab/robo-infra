# AI Integration Guide

This guide covers integrating robo-infra with ai-infra for LLM-controlled robotics.

> **Learning ai-infra**: For comprehensive ai-infra documentation, use the **nfrax-docs MCP**
> tool with queries like `"ai-infra Agent streaming"` or `"ai-infra tools_from_object"`.

## Overview

robo-infra provides seamless integration with ai-infra, allowing you to control
robots using natural language through LLM agents. The integration uses ai-infra's
generic `tools_from_object()` utility for base method extraction while adding
robotics-specific enhancements.

**Key Features:**
- Automatic tool generation from controllers and actuators
- Dynamic docstrings with actuator limits and sensor info
- Full compatibility with ai-infra Agent (streaming, HITL, fallbacks)
- Safety-focused emergency stop with prominent warnings

## Quick Start

```python
from robo_infra.core.controller import SimulatedController
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits
from robo_infra.integrations.ai_infra import controller_to_tools
from ai_infra import Agent

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

# Convert to AI function tools
tools = controller_to_tools(controller)

# Use with ai-infra Agent
agent = Agent(tools=tools)
result = agent.run("Move the shoulder to 45 degrees")
print(result)  # "Moved shoulder to 45.0 degrees"
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

## Streaming Responses

For real-time feedback during robot operations, use ai-infra's streaming API:

```python
from ai_infra import Agent
from robo_infra.integrations.ai_infra import controller_to_tools

# Create agent with robot tools
tools = controller_to_tools(arm_controller)
agent = Agent(tools=tools)

# Stream responses for real-time UI updates
async for event in agent.astream("Move arm to pick position"):
    if event.type == "token":
        print(event.content, end="", flush=True)
    elif event.type == "tool_start":
        print(f"\n[Calling {event.tool}...]")
    elif event.type == "tool_end":
        print(f"[{event.tool} complete]")
```

### Visibility Levels

Control how much detail is streamed:

```python
# Minimal: tokens only
async for event in agent.astream("Move arm", visibility="minimal"):
    ...

# Detailed: include tool arguments (useful for debugging)
async for event in agent.astream("Move arm", visibility="detailed"):
    if event.type == "tool_start":
        print(f"Calling {event.tool} with {event.arguments}")
```

### FastAPI SSE Integration

```python
from fastapi import FastAPI
from fastapi.responses import StreamingResponse

app = FastAPI()

@app.post("/robot/chat")
async def robot_chat(message: str):
    async def generate():
        async for event in agent.astream(message, visibility="standard"):
            yield f"data: {event.to_dict()}\n\n"
    
    return StreamingResponse(generate(), media_type="text/event-stream")
```

## Human-in-the-Loop (HITL)

For safety-critical operations, require human approval before execution:

```python
from ai_infra import Agent
from robo_infra.integrations.ai_infra import controller_to_tools

tools = controller_to_tools(arm_controller)
agent = Agent(tools=tools)

# Require approval for dangerous operations
agent.enable_hitl(
    tools=["arm_move", "arm_stop"],  # Require approval for these
    approval_callback=lambda tool, args: (
        input(f"Allow {tool}({args})? [y/n]: ") == "y"
    )
)

# User will be prompted before any move or stop command
result = agent.run("Move arm to maximum extension")
```

### Async HITL for Web Applications

```python
import asyncio
from collections import defaultdict

# Store pending approvals (use Redis/database in production)
pending_approvals: dict[str, dict] = {}
approval_results: dict[str, bool] = {}

async def request_approval(tool_name: str, args: dict, request_id: str):
    """Called when approval needed - notify frontend via WebSocket."""
    pending_approvals[request_id] = {
        "tool": tool_name,
        "args": args,
        "timestamp": time.time(),
    }
    # Send WebSocket message to frontend...
    await notify_frontend(request_id, tool_name, args)

async def wait_for_approval(request_id: str, timeout: float = 30.0) -> bool:
    """Wait for user response from frontend."""
    start = time.time()
    while request_id not in approval_results:
        if time.time() - start > timeout:
            return False  # Timeout = denied
        await asyncio.sleep(0.1)
    return approval_results.pop(request_id)

# Configure agent with async HITL
agent.enable_hitl_async(
    tools=["arm_move"],
    request_approval=request_approval,
    wait_for_approval=wait_for_approval,
)
```

## Provider Fallbacks

Ensure reliability with automatic provider fallback:

```python
from ai_infra import Agent
from robo_infra.integrations.ai_infra import controller_to_tools

tools = controller_to_tools(arm_controller)
agent = Agent(tools=tools)

# Try multiple providers in order
result = agent.run_with_fallbacks(
    messages=[{"role": "user", "content": "Home the arm"}],
    candidates=[
        ("openai", "gpt-4o"),           # Try OpenAI first
        ("anthropic", "claude-sonnet-4-20250514"),  # Fallback to Anthropic
        ("google_genai", "gemini-2.0-flash"),   # Final fallback
    ]
)
```

### Async Fallbacks

```python
result = await agent.arun_with_fallbacks(
    messages=[{"role": "user", "content": "Check arm status"}],
    candidates=[
        ("openai", "gpt-4o"),
        ("anthropic", "claude-sonnet-4-20250514"),
    ]
)
```

This is particularly useful for production robotics systems where uptime is critical.

## Best Practices

1. **Enable and home before use**: Always call `enable()` and `home()` on
   controllers before generating tools for agent use.

2. **Include safety tools**: The emergency stop tool should always be available
   to the agent for safety.

3. **Use descriptive names**: Controller and actuator names appear in tool names
   and should be meaningful to the LLM.

4. **Test in simulation first**: Use `SimulatedController` and `SimulatedActuator`
   to test agent behavior before connecting real hardware.

5. **Use HITL for safety-critical operations**: Enable human-in-the-loop for
   any operations that could cause physical harm or damage.

6. **Configure provider fallbacks**: For production systems, configure fallback
   providers to ensure uptime even if one provider is unavailable.

7. **Stream for real-time UI**: Use streaming to provide real-time feedback
   to operators during robot operations.

## See Also

- **nfrax-docs MCP**: Query with `"ai-infra Agent"`, `"ai-infra streaming"`, or
  `"ai-infra tools_from_object"` for comprehensive ai-infra documentation
- [Getting Started Guide](getting-started.md)
- [API Integration Guide](api-integration.md)
