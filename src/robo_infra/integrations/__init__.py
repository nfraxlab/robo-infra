"""Integration bridges for svc-infra and ai-infra.

This package provides integration utilities to connect robo-infra
controllers and actuators with:

- **ai-infra**: LLM tool generation for AI-controlled robotics
- **svc-infra**: REST API router generation for HTTP control

Example:
    >>> from robo_infra.integrations.ai_infra import controller_to_tools
    >>> from robo_infra.integrations.svc_infra import controller_to_router
    >>>
    >>> # Create AI tools for LLM agents
    >>> tools = controller_to_tools(my_controller)
    >>>
    >>> # Create REST API router
    >>> router = controller_to_router(my_controller)
"""

from robo_infra.integrations.ai_infra import (
    actuator_to_tool,
    controller_to_tools,
    create_movement_tool,
    create_safety_tools,
    create_status_tool,
)
from robo_infra.integrations.svc_infra import (
    actuator_to_router,
    controller_to_router,
    create_websocket_handler,
)


__all__ = [
    "actuator_to_router",
    "actuator_to_tool",
    "controller_to_router",
    "controller_to_tools",
    "create_movement_tool",
    "create_safety_tools",
    "create_status_tool",
    "create_websocket_handler",
]
