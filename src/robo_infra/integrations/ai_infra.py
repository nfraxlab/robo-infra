"""Integration with ai-infra for LLM-controlled robotics.

This module provides utilities to convert robo-infra controllers and
actuators into ai-infra compatible tools for LLM agents.

CRITICAL: This module uses ai-infra function tools format (not custom dicts)
following the mandatory integration standards from robo-infra/AGENTS.md.

This module integrates with ai-infra's `tools_from_object()` utility for base
method extraction, while providing robotics-specific enhancements like:
- Dynamic docstrings with actuator limits and sensor info
- Formatted status responses
- Emergency stop with safety warnings

Example - Basic usage:
    >>> from robo_infra.core.controller import SimulatedController
    >>> from robo_infra.integrations.ai_infra import controller_to_tools
    >>>
    >>> controller = SimulatedController(name="arm")
    >>> controller.add_actuator("shoulder", SimulatedActuator(...))
    >>> controller.enable()
    >>> controller.home()
    >>>
    >>> tools = controller_to_tools(controller)
    >>>
    >>> # Use with ai-infra Agent
    >>> from ai_infra import Agent
    >>> agent = Agent(tools=tools)
    >>> result = agent.run("Move the shoulder to 45 degrees")

Example - Pydantic schema tools:
    >>> from robo_infra.integrations.ai_infra import controller_to_schema_tools
    >>>
    >>> tools = controller_to_schema_tools(controller)
    >>> agent = Agent(tools=tools)

Tool Format:
    This module generates plain Python functions as tools. ai-infra
    automatically extracts tool schemas from:
    - Function name → tool name
    - Docstring → tool description (LLM sees this!)
    - Type hints → parameter types
    - Default values → optional parameters

Note:
    We intentionally do NOT use `from __future__ import annotations` here
    because ai-infra needs actual type objects (not string annotations) for
    Pydantic model parameter resolution.
"""

import logging
import warnings
from collections.abc import Callable
from typing import TYPE_CHECKING, Any


# Import tools_from_object from ai-infra for base method extraction
try:
    from ai_infra.tools import tool_exclude, tools_from_object

    _AI_INFRA_AVAILABLE = True
except ImportError:
    _AI_INFRA_AVAILABLE = False
    tool_exclude = None  # type: ignore[assignment]
    tools_from_object = None  # type: ignore[assignment]


def _check_ai_infra() -> None:
    """Raise ImportError if ai-infra is not available."""
    if not _AI_INFRA_AVAILABLE:
        raise ImportError(
            "ai-infra is required for AI integration. Install with: pip install robo-infra[ai]"
        )


if TYPE_CHECKING:
    from robo_infra.core.actuator import Actuator
    from robo_infra.core.controller import Controller


logger = logging.getLogger(__name__)


__all__ = [
    "actuator_to_tool",  # Deprecated
    "actuator_to_tools",
    "controller_to_schema_tools",
    "controller_to_tools",
    "create_disable_tool",
    "create_enable_tool",
    "create_home_tool",
    "create_move_tool",
    "create_movement_tool",  # Deprecated
    "create_safety_tools",  # Deprecated
    "create_sensors_tool",
    "create_status_tool",
    "create_stop_tool",
    "tool_exclude",
    # Re-exported from ai-infra for convenience
    "tools_from_object",
]


# =============================================================================
# Primary Tool Generation (Function Tools)
# =============================================================================


def controller_to_tools(controller: "Controller") -> list[Callable]:
    """Convert a controller to a list of AI function tools.

    This function uses ai-infra's `tools_from_object()` for basic method
    extraction (enable, disable, home) and adds robotics-specific tools
    with enhanced docstrings containing actuator limits and sensor info.

    Creates callable function tools for:
    - Moving actuators to positions (with actuator limits in docstring)
    - Homing the controller
    - Emergency stop (with safety warnings)
    - Getting status (formatted with controller state)
    - Reading sensors (if controller has sensors)
    - Enable/disable

    Args:
        controller: The controller to convert.

    Returns:
        List of callable functions compatible with ai-infra Agent.

    Example:
        >>> tools = controller_to_tools(arm_controller)
        >>> from ai_infra import Agent
        >>> agent = Agent(tools=tools)
        >>> agent.run("Move shoulder to 45 degrees and elbow to 30 degrees")

    Note:
        This function uses ai-infra's `tools_from_object()` internally for
        basic method extraction. Robotics-specific tools (move, status,
        sensors) are created with enhanced docstrings that include runtime
        information about actuator limits and available sensors.

    Raises:
        ImportError: If ai-infra is not installed.
    """
    _check_ai_infra()
    tools: list[Callable] = []
    name = controller.name

    # Get actuator info for docstrings (robotics-specific enhancement)
    actuator_info = ", ".join(
        f"{k} ({v.limits.min}-{v.limits.max})" for k, v in controller.actuators.items()
    )
    sensor_info = ", ".join(controller.sensors.keys()) if controller.sensors else "none"

    # =========================================================================
    # Use tools_from_object() for base method extraction, then wrap for
    # robotics-specific behavior (string confirmations for LLM compatibility)
    # =========================================================================
    base_tools = tools_from_object(
        controller,
        methods=["enable", "disable", "home"],
        prefix=name,
        async_wrapper=False,  # Keep sync for robotics safety
    )

    # Wrap base tools to return string confirmations (required for LLM compatibility)
    # and enhance docstrings with robotics context
    for base_tool in base_tools:
        tool_method = base_tool.__name__.replace(f"{name}_", "")

        if tool_method == "enable":

            def enable() -> str:
                """Enable controller and actuators."""
                controller.enable()
                return f"Enabled {name}"

            enable.__doc__ = f"""Enable {name} controller and all actuators.

Must be called before any motion commands. After enabling,
call home() to move to default positions.

Returns:
    Confirmation that controller was enabled.
"""
            enable.__name__ = f"{name}_enable"
            tools.append(enable)

        elif tool_method == "disable":

            def disable() -> str:
                """Disable controller and actuators."""
                controller.disable()
                return f"Disabled {name}"

            disable.__doc__ = f"""Disable {name} controller and all actuators.

Safely disables all actuators. Use this before powering down
or when robot should be idle.

Returns:
    Confirmation that controller was disabled.
"""
            disable.__name__ = f"{name}_disable"
            tools.append(disable)

        elif tool_method == "home":

            def home() -> str:
                """Home controller to default positions."""
                controller.home()
                return f"Homed {name} successfully"

            home.__doc__ = f"""Home {name} to default positions.

Moves all actuators on {name} to their home/default positions.
This should be called after enabling the controller.

Returns:
    Confirmation that homing completed.
"""
            home.__name__ = f"{name}_home"
            tools.append(home)

    # =========================================================================
    # Robotics-specific tools with custom behavior
    # These need enhanced docstrings and custom return values
    # =========================================================================
    # =========================================================================

    # --- Move tool (needs actuator info in docstring) ---
    def move(targets: dict[str, float]) -> str:
        """Move actuators to target positions."""
        controller.move_to(targets)
        return f"Moved {name} to {targets}"

    move.__doc__ = f"""Move {name} actuators to target positions.

Moves the specified actuators to the given positions.
Available actuators: {actuator_info}

Args:
    targets: Dict mapping actuator names to target positions.
        Example: {{"shoulder": 45.0, "elbow": 30.0}}

Returns:
    Confirmation message with the positions moved to.
"""
    move.__name__ = f"{name}_move"
    tools.append(move)

    # --- Emergency stop tool (needs safety warnings) ---
    def stop() -> str:
        """Emergency stop - halt all motion."""
        controller.stop()
        return f"EMERGENCY STOP executed on {name}"

    stop.__doc__ = f"""EMERGENCY STOP - immediately halt all motion on {name}.

USE ONLY IN EMERGENCIES. This immediately disables all actuators
and halts any motion. The controller must be re-enabled and
re-homed after an emergency stop.

Returns:
    Confirmation that emergency stop was executed.
"""
    stop.__name__ = f"{name}_stop"
    tools.append(stop)

    # --- Status tool (needs formatted response) ---
    def status() -> dict[str, Any]:
        """Get controller status."""
        ctrl_status = controller.status()
        return {
            "controller": name,
            "state": ctrl_status.state.value,
            "is_enabled": ctrl_status.is_enabled,
            "is_homed": ctrl_status.is_homed,
            "actuators": controller.get_actuator_values(),
        }

    status.__doc__ = f"""Get current status of {name}.

Returns the controller state, enabled status, homed status,
and current positions of all actuators.

Returns:
    Dict with state, is_enabled, is_homed, and actuators positions.
"""
    status.__name__ = f"{name}_status"
    tools.append(status)

    # --- Sensors tool (if sensors exist) ---
    if controller.sensors:

        def sensors() -> dict[str, float]:
            """Read all sensors."""
            return controller.read_sensors()

        sensors.__doc__ = f"""Read all sensors on {name}.

Reads current values from all sensors attached to {name}.
Available sensors: {sensor_info}

Returns:
    Dict mapping sensor names to their current readings.
"""
        sensors.__name__ = f"{name}_sensors"
        tools.append(sensors)

    logger.debug("Created %d function tools for controller '%s'", len(tools), name)
    return tools


def actuator_to_tools(actuator: "Actuator") -> list[Callable]:
    """Convert a single actuator to AI function tools.

    This function uses ai-infra's `tools_from_object()` for enable/disable
    methods and adds actuator-specific tools with limit information.

    Creates callable function tools for:
    - Setting the actuator value (with limits in docstring)
    - Getting the current value and status
    - Enabling/disabling

    Args:
        actuator: The actuator to convert.

    Returns:
        List of callable functions compatible with ai-infra Agent.

    Example:
        >>> tools = actuator_to_tools(servo)
        >>> from ai_infra import Agent
        >>> agent = Agent(tools=tools)
        >>> agent.run("Set the servo to 90 degrees")

    Raises:
        ImportError: If ai-infra is not installed.
    """
    _check_ai_infra()
    tools: list[Callable] = []
    name = actuator.name
    limits = actuator.limits

    # =========================================================================
    # Use tools_from_object() for base method discovery, then wrap with
    # string confirmations for LLM compatibility
    # =========================================================================
    base_tools = tools_from_object(
        actuator,
        methods=["enable", "disable"],
        prefix=name,
        async_wrapper=False,
    )

    # Wrap base tools to return string confirmations (required for LLM compatibility)
    for base_tool in base_tools:
        tool_method = base_tool.__name__.replace(f"{name}_", "")

        if tool_method == "enable":

            def enable_act() -> str:
                """Enable actuator."""
                actuator.enable()
                return f"Enabled {name}"

            enable_act.__doc__ = f"""Enable {name} actuator.

Returns:
    Confirmation that actuator was enabled.
"""
            enable_act.__name__ = f"{name}_enable"
            tools.append(enable_act)

        elif tool_method == "disable":

            def disable_act() -> str:
                """Disable actuator."""
                actuator.disable()
                return f"Disabled {name}"

            disable_act.__doc__ = f"""Disable {name} actuator.

Returns:
    Confirmation that actuator was disabled.
"""
            disable_act.__name__ = f"{name}_disable"
            tools.append(disable_act)

    # =========================================================================
    # Actuator-specific tools with limit information
    # =========================================================================

    # --- Set tool (needs limits in docstring) ---
    def set_value(value: float) -> str:
        """Set actuator to a position."""
        actuator.set(value)
        return f"Set {name} to {value}"

    set_value.__doc__ = f"""Set {name} to a specific position.

Sets the actuator to the given value. Valid range: {limits.min} to {limits.max}.

Args:
    value: Target position between {limits.min} and {limits.max}.

Returns:
    Confirmation message with the new position.
"""
    set_value.__name__ = f"{name}_set"
    tools.append(set_value)

    # --- Get tool (needs formatted response) ---
    def get_value() -> dict[str, Any]:
        """Get actuator value and status."""
        return {
            "name": name,
            "value": actuator.get(),
            "is_enabled": actuator.is_enabled,
            "limits": {"min": limits.min, "max": limits.max},
        }

    get_value.__doc__ = f"""Get current value and status of {name}.

Returns:
    Dict with current value, enabled status, and limits.
"""
    get_value.__name__ = f"{name}_get"
    tools.append(get_value)

    logger.debug("Created %d function tools for actuator '%s'", len(tools), name)
    return tools


# =============================================================================
# Pydantic Schema Tools
# =============================================================================


def controller_to_schema_tools(controller: "Controller") -> list[Callable]:
    """Generate Pydantic schema tools for structured LLM interaction.

    Creates tools with explicit Pydantic model inputs for more precise
    LLM parameter handling. Use this when you need strict validation
    of tool inputs.

    Args:
        controller: The controller to convert.

    Returns:
        List of callable functions with Pydantic model parameters.

    Example:
        >>> tools = controller_to_schema_tools(arm_controller)
        >>> from ai_infra import Agent
        >>> agent = Agent(tools=tools)
        >>> agent.run("Move shoulder to 45 degrees")

    Note:
        These tools use Pydantic BaseModel for input validation,
        providing stricter type checking than plain function tools.

    Raises:
        ImportError: If ai-infra is not installed.
    """
    _check_ai_infra()
    try:
        from pydantic import BaseModel, Field
    except ImportError as e:
        msg = "Pydantic is required for schema tools. Install with: pip install pydantic"
        raise ImportError(msg) from e

    tools: list[Callable] = []
    name = controller.name

    # Get actuator names for documentation
    actuator_names = list(controller.actuators.keys())
    actuator_info = ", ".join(
        f"{k} ({v.limits.min}-{v.limits.max})" for k, v in controller.actuators.items()
    )

    # Define Pydantic models for tool inputs
    class MoveInput(BaseModel):
        """Input for move command."""

        targets: dict[str, float] = Field(
            description=f"Dict mapping actuator names to positions. Available: {actuator_names}. "
            f"Limits: {actuator_info}"
        )

    class PositionInput(BaseModel):
        """Input for named position command."""

        position_name: str = Field(
            description=f"Name of the saved position to move to. "
            f"Available: {list(controller.positions.keys())}"
        )

    # --- Move tool with schema ---
    def move(input: MoveInput) -> str:
        """Move actuators to target positions."""
        controller.move_to(input.targets)
        return f"Moved {name} to {input.targets}"

    move.__doc__ = f"""Move {name} actuators to target positions.

Available actuators: {actuator_info}

Args:
    input: MoveInput with targets dict mapping actuator names to positions.

Returns:
    Confirmation message.
"""
    move.__name__ = f"{name}_move"
    tools.append(move)

    # --- Home tool ---
    def home() -> str:
        """Home controller to default positions."""
        controller.home()
        return f"Homed {name}"

    home.__name__ = f"{name}_home"
    tools.append(home)

    # --- Stop tool ---
    def stop() -> str:
        """EMERGENCY STOP - immediately halt all motion."""
        controller.stop()
        return f"EMERGENCY STOP on {name}"

    stop.__name__ = f"{name}_stop"
    tools.append(stop)

    # --- Status tool ---
    def status() -> dict[str, Any]:
        """Get controller status."""
        ctrl_status = controller.status()
        return {
            "controller": name,
            "state": ctrl_status.state.value,
            "is_enabled": ctrl_status.is_enabled,
            "is_homed": ctrl_status.is_homed,
            "actuators": controller.get_actuator_values(),
        }

    status.__name__ = f"{name}_status"
    tools.append(status)

    # --- Named position tool (if positions exist) ---
    if controller.positions:

        def go_to_position(input: PositionInput) -> str:
            """Move to a named/saved position."""
            controller.move_to_position(input.position_name)
            return f"Moved {name} to position '{input.position_name}'"

        go_to_position.__name__ = f"{name}_go_to_position"
        tools.append(go_to_position)

    logger.debug("Created %d schema tools for controller '%s'", len(tools), name)
    return tools


# =============================================================================
# Individual Tool Creators (for custom tool composition)
# =============================================================================


def create_move_tool(controller: "Controller") -> Callable:
    """Create a move tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Callable move tool function.
    """
    name = controller.name
    actuator_info = ", ".join(
        f"{k} ({v.limits.min}-{v.limits.max})" for k, v in controller.actuators.items()
    )

    def move(targets: dict[str, float]) -> str:
        """Move actuators to target positions."""
        controller.move_to(targets)
        return f"Moved {name} to {targets}"

    move.__doc__ = f"""Move {name} actuators to target positions.

Available actuators: {actuator_info}

Args:
    targets: Dict mapping actuator names to target positions.

Returns:
    Confirmation message.
"""
    move.__name__ = f"{name}_move"
    return move


def create_home_tool(controller: "Controller") -> Callable:
    """Create a home tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Callable home tool function.
    """
    name = controller.name

    def home() -> str:
        """Home controller to default positions."""
        controller.home()
        return f"Homed {name}"

    home.__doc__ = f"""Home {name} to default positions.

Returns:
    Confirmation message.
"""
    home.__name__ = f"{name}_home"
    return home


def create_stop_tool(controller: "Controller") -> Callable:
    """Create an emergency stop tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Callable stop tool function.
    """
    name = controller.name

    def stop() -> str:
        """EMERGENCY STOP - immediately halt all motion."""
        controller.stop()
        return f"EMERGENCY STOP on {name}"

    stop.__doc__ = f"""EMERGENCY STOP {name} - immediately halt all motion.

USE ONLY IN EMERGENCIES. Controller must be re-enabled after.

Returns:
    Confirmation message.
"""
    stop.__name__ = f"{name}_stop"
    return stop


def create_status_tool(controller: "Controller") -> Callable:
    """Create a status tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Callable status tool function.
    """
    name = controller.name

    def status() -> dict[str, Any]:
        """Get controller status."""
        ctrl_status = controller.status()
        return {
            "controller": name,
            "state": ctrl_status.state.value,
            "is_enabled": ctrl_status.is_enabled,
            "is_homed": ctrl_status.is_homed,
            "actuators": controller.get_actuator_values(),
        }

    status.__doc__ = f"""Get current status of {name}.

Returns:
    Dict with controller state and actuator positions.
"""
    status.__name__ = f"{name}_status"
    return status


def create_sensors_tool(controller: "Controller") -> Callable:
    """Create a sensors reading tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Callable sensors tool function.
    """
    name = controller.name
    sensor_info = ", ".join(controller.sensors.keys()) if controller.sensors else "none"

    def sensors() -> dict[str, float]:
        """Read all sensors."""
        return controller.read_sensors()

    sensors.__doc__ = f"""Read all sensors on {name}.

Available sensors: {sensor_info}

Returns:
    Dict mapping sensor names to readings.
"""
    sensors.__name__ = f"{name}_sensors"
    return sensors


def create_enable_tool(controller: "Controller") -> Callable:
    """Create an enable tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Callable enable tool function.
    """
    name = controller.name

    def enable() -> str:
        """Enable controller and actuators."""
        controller.enable()
        return f"Enabled {name}"

    enable.__doc__ = f"""Enable {name} controller and actuators.

Returns:
    Confirmation message.
"""
    enable.__name__ = f"{name}_enable"
    return enable


def create_disable_tool(controller: "Controller") -> Callable:
    """Create a disable tool for a controller.

    Args:
        controller: The controller.

    Returns:
        Callable disable tool function.
    """
    name = controller.name

    def disable() -> str:
        """Disable controller and actuators."""
        controller.disable()
        return f"Disabled {name}"

    disable.__doc__ = f"""Disable {name} controller and actuators.

Returns:
    Confirmation message.
"""
    disable.__name__ = f"{name}_disable"
    return disable


# =============================================================================
# Deprecated Functions (for backwards compatibility)
# =============================================================================


def actuator_to_tool(actuator: "Actuator") -> dict[str, Any]:
    """Convert a single actuator to an AI tool dict.

    DEPRECATED: Use actuator_to_tools() instead which returns callable
    functions compatible with ai-infra Agent.

    Args:
        actuator: The actuator to convert.

    Returns:
        Tool definition dictionary (deprecated format).
    """
    warnings.warn(
        "actuator_to_tool() is deprecated. Use actuator_to_tools() instead "
        "which returns callable functions for ai-infra Agent.",
        DeprecationWarning,
        stacklevel=2,
    )
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
    controller: "Controller",
    positions: dict[str, dict[str, float]] | None = None,
) -> dict[str, Any]:
    """Create a high-level movement tool with predefined positions.

    DEPRECATED: Use controller_to_tools() or controller_to_schema_tools()
    instead for ai-infra compatible function tools.

    Args:
        name: Tool name.
        controller: The controller.
        positions: Optional predefined positions.

    Returns:
        Tool definition dictionary (deprecated format).
    """
    warnings.warn(
        "create_movement_tool() is deprecated. Use controller_to_tools() or "
        "controller_to_schema_tools() instead for ai-infra compatible tools.",
        DeprecationWarning,
        stacklevel=2,
    )
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


def create_safety_tools(controller: "Controller") -> list[dict[str, Any]]:
    """Create safety-focused tools for a controller.

    DEPRECATED: Use controller_to_tools() instead which includes
    stop and reset as callable functions.

    Args:
        controller: The controller.

    Returns:
        List of tool definition dictionaries (deprecated format).
    """
    warnings.warn(
        "create_safety_tools() is deprecated. Use controller_to_tools() instead "
        "which includes stop and safety tools as callable functions.",
        DeprecationWarning,
        stacklevel=2,
    )
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
            "description": f"Reset {name} from emergency stop state",
            "parameters": {"type": "object", "properties": {}},
            "handler": lambda c=controller: c.reset_stop(),
        },
    ]
