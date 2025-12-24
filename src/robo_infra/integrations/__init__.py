"""Integration bridges for svc-infra, ai-infra, and ROS2.

This package provides integration utilities to connect robo-infra
controllers and actuators with:

- **ai-infra**: LLM tool generation for AI-controlled robotics
- **svc-infra**: REST API router generation for HTTP control
- **ROS2**: ROS2 node generation for ROS ecosystem integration

Example:
    >>> from robo_infra.integrations.ai_infra import controller_to_tools
    >>> from robo_infra.integrations.svc_infra import controller_to_router
    >>> from robo_infra.integrations.ros2 import controller_to_ros2_node
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
    # ai-infra integration (new function tools format)
    "actuator_to_tools",
    "controller_to_schema_tools",
    "controller_to_tools",
    "create_disable_tool",
    "create_enable_tool",
    "create_home_tool",
    "create_move_tool",
    "create_sensors_tool",
    "create_status_tool",
    "create_stop_tool",
    # ai-infra integration (deprecated dict format)
    "actuator_to_tool",
    "create_movement_tool",
    "create_safety_tools",
    # svc-infra integration
    "actuator_to_router",
    "controller_to_router",
    "create_websocket_handler",
    "create_websocket_router",
    # ROS2 integration
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
