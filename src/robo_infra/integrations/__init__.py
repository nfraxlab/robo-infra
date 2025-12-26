"""Integration bridges for svc-infra, ai-infra, ROS2, and observability.

This package provides integration utilities to connect robo-infra
controllers and actuators with:

- **ai-infra**: LLM tool generation for AI-controlled robotics
- **svc-infra**: REST API router generation for HTTP control
- **ROS2**: ROS2 node generation for ROS ecosystem integration
- **observability**: Prometheus metrics, health checks, structured logging

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
from robo_infra.integrations.svc_infra import (
    actuator_to_router,
    controller_to_router,
    create_websocket_handler,
    create_websocket_router,
)


__all__ = [
    "ControllerROS2Node",
    "LaunchConfig",
    "ROS2NodeConfig",
    "SafetyTriggerType",
    "actuator_to_ros2_node",
    "actuator_to_router",
    "actuator_to_tool",
    "actuator_to_tools",
    "add_robotics_health_routes",
    "controller_to_ros2_node",
    "controller_to_router",
    "controller_to_schema_tools",
    "controller_to_tools",
    "create_actuator_health_check",
    "create_controller_health_check",
    "create_disable_tool",
    "create_enable_tool",
    "create_home_tool",
    "create_move_tool",
    "create_movement_tool",
    "create_safety_tools",
    "create_sensors_tool",
    "create_status_tool",
    "create_stop_tool",
    "create_websocket_handler",
    "create_websocket_router",
    "generate_launch_file",
    "generate_parameters_file",
    "get_robotics_request_id",
    "is_ros2_available",
    "is_ros2_mock_mode",
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
