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
    >>>
    >>> # Create REST API router
    >>> router = controller_to_router(my_controller)
    >>>
    >>> # Create ROS2 node
    >>> node = controller_to_ros2_node(my_controller)
"""

from robo_infra.integrations.ai_infra import (
    actuator_to_tool,
    controller_to_tools,
    create_movement_tool,
    create_safety_tools,
    create_status_tool,
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
)


__all__ = [
    # ai-infra integration
    "actuator_to_router",
    "actuator_to_tool",
    "controller_to_router",
    "controller_to_tools",
    "create_movement_tool",
    "create_safety_tools",
    "create_status_tool",
    "create_websocket_handler",
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
