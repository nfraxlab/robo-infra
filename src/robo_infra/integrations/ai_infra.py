"""Integration with ai-infra for LLM-controlled robotics.

This module provides utilities to convert robo-infra controllers and
actuators into ai-infra compatible tools for LLM agents.

Example:
    >>> from robo_infra.core.controller import SimulatedController
    >>> from robo_infra.integrations.ai_infra import controller_to_tools
    >>>
    >>> controller = SimulatedController(name="arm")
    >>> tools = controller_to_tools(controller)
    >>>
    >>> # Use with ai-infra Agent
    >>> from ai_infra.llm import CoreAgent
    >>> agent = CoreAgent(tools=tools)

Note:
    This module provides basic tool generation. For full LangChain/LangGraph
    integration, use the tools directly with ai-infra's agent framework.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any


if TYPE_CHECKING:
    from robo_infra.core.actuator import Actuator
    from robo_infra.core.controller import Controller


logger = logging.getLogger(__name__)


__all__ = [
    "actuator_to_tool",
    "controller_to_tools",
    "create_movement_tool",
    "create_safety_tools",
    "create_status_tool",
]


def controller_to_tools(controller: Controller) -> list[dict[str, Any]]:
    """Convert a controller to a list of AI tools.

    Creates tools for:
    - Moving actuators to positions
    - Homing the controller
    - Emergency stop
    - Getting status
    - Reading sensors

    Args:
        controller: The controller to convert.

    Returns:
        List of tool definitions compatible with ai-infra agents.

    Example:
        >>> tools = controller_to_tools(arm_controller)
        >>> # Returns tools like: arm_move, arm_home, arm_stop, arm_status, arm_sensors
    """
    tools = []
    name = controller.name

    # Movement tool
    tools.append(
        {
            "name": f"{name}_move",
            "description": f"Move {name} actuators to target positions. "
            f"Available actuators: {list(controller.actuators.keys())}",
            "parameters": {
                "type": "object",
                "properties": {
                    actuator_name: {
                        "type": "number",
                        "description": f"Target position for {actuator_name} "
                        f"(min: {actuator.limits.min}, max: {actuator.limits.max})",
                    }
                    for actuator_name, actuator in controller.actuators.items()
                },
            },
            "handler": lambda targets, c=controller: c.move_to(targets),
        }
    )

    # Home tool
    tools.append(
        {
            "name": f"{name}_home",
            "description": f"Home {name} to default positions",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: c.home(),
        }
    )

    # Stop tool (emergency)
    tools.append(
        {
            "name": f"{name}_stop",
            "description": f"EMERGENCY STOP - immediately halt all motion on {name}",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: c.stop(),
        }
    )

    # Status tool
    tools.append(
        {
            "name": f"{name}_status",
            "description": f"Get current status of {name}",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: {
                "state": c.status().state.value,
                "is_enabled": c.status().is_enabled,
                "is_homed": c.status().is_homed,
                "actuators": c.get_actuator_values(),
            },
        }
    )

    # Sensors tool
    if controller.sensors:
        tools.append(
            {
                "name": f"{name}_sensors",
                "description": f"Read all sensors on {name}. "
                f"Available sensors: {list(controller.sensors.keys())}",
                "parameters": {"type": "object", "properties": {}},
                "handler": lambda c=controller: c.read_sensors(),
            }
        )

    # Enable/disable tools
    tools.append(
        {
            "name": f"{name}_enable",
            "description": f"Enable {name} controller and all actuators",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: c.enable(),
        }
    )

    tools.append(
        {
            "name": f"{name}_disable",
            "description": f"Disable {name} controller and all actuators",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: c.disable(),
        }
    )

    logger.debug("Created %d tools for controller '%s'", len(tools), name)
    return tools


def actuator_to_tool(actuator: Actuator) -> dict[str, Any]:
    """Convert a single actuator to an AI tool.

    Args:
        actuator: The actuator to convert.

    Returns:
        Tool definition dictionary.

    Example:
        >>> tool = actuator_to_tool(servo)
        >>> # Returns tool like: servo_set
    """
    name = actuator.name
    limits = actuator.limits

    return {
        "name": f"{name}_set",
        "description": f"Set {name} to a position between {limits.min} and {limits.max}",
        "parameters": {
            "type": "object",
            "properties": {
                "value": {
                    "type": "number",
                    "description": f"Target position ({limits.min} to {limits.max})",
                    "minimum": limits.min,
                    "maximum": limits.max,
                }
            },
            "required": ["value"],
        },
        "handler": lambda value, a=actuator: a.set(value),
    }


def create_movement_tool(
    name: str,
    controller: Controller,
    positions: dict[str, dict[str, float]] | None = None,
) -> dict[str, Any]:
    """Create a high-level movement tool with predefined positions.

    Args:
        name: Tool name.
        controller: The controller.
        positions: Optional predefined positions (e.g., {"home": {...}, "pick": {...}}).

    Returns:
        Tool definition with position selection.
    """
    position_names = list(positions.keys()) if positions else list(controller.positions.keys())

    return {
        "name": name,
        "description": f"Move to a named position. Available: {position_names}",
        "parameters": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "string",
                    "enum": position_names,
                    "description": "Name of the position to move to",
                }
            },
            "required": ["position"],
        },
        "handler": lambda position, c=controller, p=positions: (
            c.move_to(p[position]) if p and position in p else c.move_to_position(position)
        ),
    }


def create_status_tool(controller: Controller) -> dict[str, Any]:
    """Create a detailed status tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Tool definition for status query.
    """
    return {
        "name": f"{controller.name}_full_status",
        "description": f"Get detailed status of {controller.name} including all actuators and sensors",
        "parameters": {"type": "object", "properties": {}},
        "handler": lambda c=controller: {
            "controller": {
                "name": c.name,
                "state": c.status().state.value,
                "mode": c.status().mode.value,
                "is_enabled": c.status().is_enabled,
                "is_homed": c.status().is_homed,
                "is_running": c.status().is_running,
                "uptime": c.status().uptime,
            },
            "actuators": {
                name: {
                    "value": act.get(),
                    "is_enabled": act.is_enabled,
                    "limits": {"min": act.limits.min, "max": act.limits.max},
                }
                for name, act in c.actuators.items()
            },
            "sensors": c.read_sensors(),
        },
    }


def create_safety_tools(controller: Controller) -> list[dict[str, Any]]:
    """Create safety-focused tools for a controller.

    Args:
        controller: The controller.

    Returns:
        List of safety tools (stop, reset, limit check).
    """
    name = controller.name

    return [
        {
            "name": f"{name}_emergency_stop",
            "description": f"EMERGENCY STOP {name} - USE ONLY IN EMERGENCIES",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: c.stop(),
        },
        {
            "name": f"{name}_reset_stop",
            "description": f"Reset {name} from emergency stop state (requires re-homing)",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: c.reset_stop(),
        },
    ]
