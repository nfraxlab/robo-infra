"""Integration bridges for svc-infra, ai-infra, ROS2, and observability.

This package provides integration utilities to connect robo-infra
controllers and actuators with:

- **ai-infra**: LLM tool generation for AI-controlled robotics
- **svc-infra**: REST API router generation for HTTP control
- **ROS2**: ROS2 node generation for ROS ecosystem integration
- **observability**: Prometheus metrics, health checks, structured logging

Note:
    ai-infra and svc-infra are optional dependencies. Install with:
    - pip install robo-infra[ai]   # For ai-infra integration
    - pip install robo-infra[api]  # For svc-infra integration
    - pip install robo-infra[full] # For all integrations

Example:
    >>> from robo_infra.integrations.ai_infra import controller_to_tools
    >>> from robo_infra.integrations.svc_infra import controller_to_router
    >>> from robo_infra.integrations.ros2 import controller_to_ros2_node
    >>> from robo_infra.integrations.observability import track_command
    >>>
    >>> # Create AI tools for LLM agents
    >>> tools = controller_to_tools(my_controller)
    >>> from ai_infra import Agent
    >>> agent = Agent(tools=tools)
    >>>
    >>> # Create REST API router
    >>> router = controller_to_router(my_controller)
    >>>
    >>> # Create ROS2 node
    >>> node = controller_to_ros2_node(my_controller)
    >>>
    >>> # Instrument with metrics
    >>> @track_command("move")
    ... async def move(self, target): ...
"""

# AI-infra integration (optional - requires: pip install robo-infra[ai])
try:
    from robo_infra.integrations.ai_infra import (
        actuator_to_tool,
        actuator_to_tools,
        controller_to_schema_tools,
        controller_to_tools,
        create_disable_tool,
        create_enable_tool,
        create_home_tool,
        create_move_tool,
        create_movement_tool,
        create_safety_tools,
        create_sensors_tool,
        create_status_tool,
        create_stop_tool,
    )

    _AI_EXPORTS = [
        "actuator_to_tool",
        "actuator_to_tools",
        "controller_to_schema_tools",
        "controller_to_tools",
        "create_disable_tool",
        "create_enable_tool",
        "create_home_tool",
        "create_move_tool",
        "create_movement_tool",
        "create_safety_tools",
        "create_sensors_tool",
        "create_status_tool",
        "create_stop_tool",
    ]
except ImportError:
    _AI_EXPORTS = []

# Observability integration (optional - requires: pip install robo-infra[api])
try:
    from robo_infra.integrations.observability import (
        SafetyTriggerType,
        add_robotics_health_routes,
        create_actuator_health_check,
        create_controller_health_check,
        get_robotics_request_id,
        log_with_context,
        record_command,
        record_estop_triggered,
        record_limit_exceeded,
        record_monitor_alert,
        record_position,
        record_safety_trigger,
        record_sensor_value,
        record_watchdog_timeout,
        register_controller_health_checks,
        set_robotics_request_id,
        setup_robotics_logging,
        track_command,
    )

    _OBS_EXPORTS = [
        "SafetyTriggerType",
        "add_robotics_health_routes",
        "create_actuator_health_check",
        "create_controller_health_check",
        "get_robotics_request_id",
        "log_with_context",
        "record_command",
        "record_estop_triggered",
        "record_limit_exceeded",
        "record_monitor_alert",
        "record_position",
        "record_safety_trigger",
        "record_sensor_value",
        "record_watchdog_timeout",
        "register_controller_health_checks",
        "set_robotics_request_id",
        "setup_robotics_logging",
        "track_command",
    ]
except ImportError:
    _OBS_EXPORTS = []

# ROS2 integration (optional - requires ROS2 installation)
from robo_infra.integrations.ros2 import (
    ControllerROS2Node,
    LaunchConfig,
    ROS2NodeConfig,
    actuator_to_ros2_node,
    controller_to_ros2_node,
    generate_launch_file,
    generate_parameters_file,
    is_ros2_available,
    is_ros2_mock_mode,
)


# SVC-infra integration (optional - requires: pip install robo-infra[api])
try:
    from robo_infra.integrations.svc_infra import (
        actuator_to_router,
        controller_to_router,
        create_websocket_handler,
        create_websocket_router,
    )

    _SVC_EXPORTS = [
        "actuator_to_router",
        "controller_to_router",
        "create_websocket_handler",
        "create_websocket_router",
    ]
except ImportError:
    _SVC_EXPORTS = []


# ROS2 exports (always available - gracefully degrades internally)
_ROS2_EXPORTS = [
    "ControllerROS2Node",
    "LaunchConfig",
    "ROS2NodeConfig",
    "actuator_to_ros2_node",
    "controller_to_ros2_node",
    "generate_launch_file",
    "generate_parameters_file",
    "is_ros2_available",
    "is_ros2_mock_mode",
]

# Combine all exports - using list concatenation for ruff compatibility
__all__ = _ROS2_EXPORTS + _AI_EXPORTS + _OBS_EXPORTS + _SVC_EXPORTS
