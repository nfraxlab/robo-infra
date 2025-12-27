"""ROS2 integration for robo-infra controllers.

This module provides bridges between robo-infra controllers and ROS2,
allowing controllers to be exposed as ROS2 nodes with standard message types.

The ROS2 integration is OPTIONAL - all functionality works without ROS2 installed.
When ROS2 is not available, a mock mode is used for testing.

Features:
- Convert controllers to ROS2 nodes
- Publish status via standard message types (JointState, Twist, Pose)
- Subscribe to command topics
- Expose services for control operations
- Auto-generate launch files
- Parameter file generation

Example:
    >>> from robo_infra.integrations.ros2 import controller_to_ros2_node
    >>> from robo_infra.controllers import JointGroup
    >>>
    >>> arm = JointGroup("arm", joints=[...])
    >>> node = controller_to_ros2_node(arm)
    >>> rclpy.spin(node)

Note:
    Requires rclpy and ROS2 message packages to be installed for real operation.
    For testing, use `ROS2_MOCK=true` environment variable.
"""

from __future__ import annotations

import logging
import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any


if TYPE_CHECKING:
    import threading
    from collections.abc import Callable

    from robo_infra.core.actuator import Actuator
    from robo_infra.core.controller import Controller

logger = logging.getLogger(__name__)


# =============================================================================
# ROS2 Availability Check
# =============================================================================

_ROS2_AVAILABLE = False


def is_ros2_mock_mode() -> bool:
    """Check if ROS2 mock mode is enabled.

    Returns:
        True if ROS2_MOCK environment variable is set to true/1/yes.

    Note:
        This is checked dynamically to allow setting the environment
        variable before importing the module in tests.
    """
    return os.getenv("ROS2_MOCK", "").lower() in ("true", "1", "yes")


try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    _ROS2_AVAILABLE = True
except ImportError:
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment, misc]
    QoSProfile = None  # type: ignore[assignment, misc]
    ReliabilityPolicy = None  # type: ignore[assignment, misc]
    HistoryPolicy = None  # type: ignore[assignment, misc]

# Try to import standard message types
try:
    from geometry_msgs.msg import Point, Pose, Quaternion, Twist
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64, Header, String
    from std_srvs.srv import SetBool, Trigger

    _ROS2_MESSAGES_AVAILABLE = True
except ImportError:
    JointState = None  # type: ignore[assignment, misc]
    Twist = None  # type: ignore[assignment, misc]
    Pose = None  # type: ignore[assignment, misc]
    Point = None  # type: ignore[assignment, misc]
    Quaternion = None  # type: ignore[assignment, misc]
    Header = None  # type: ignore[assignment, misc]
    Float64 = None  # type: ignore[assignment, misc]
    Bool = None  # type: ignore[assignment, misc]
    String = None  # type: ignore[assignment, misc]
    Trigger = None  # type: ignore[assignment, misc]
    SetBool = None  # type: ignore[assignment, misc]
    _ROS2_MESSAGES_AVAILABLE = False


def is_ros2_available() -> bool:
    """Check if ROS2 is available.

    Returns:
        True if rclpy and standard messages are importable.
    """
    return _ROS2_AVAILABLE and _ROS2_MESSAGES_AVAILABLE


# =============================================================================
# ROS2 Message Types (Mock implementations for testing)
# =============================================================================


class MessageType(Enum):
    """Standard ROS2 message types supported."""

    JOINT_STATE = "sensor_msgs/JointState"
    TWIST = "geometry_msgs/Twist"
    POSE = "geometry_msgs/Pose"
    FLOAT64 = "std_msgs/Float64"
    BOOL = "std_msgs/Bool"
    STRING = "std_msgs/String"


@dataclass
class MockJointState:
    """Mock JointState message for testing without ROS2."""

    header: dict[str, Any] = field(
        default_factory=lambda: {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}
    )
    name: list[str] = field(default_factory=list)
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)


@dataclass
class MockTwist:
    """Mock Twist message for testing without ROS2."""

    linear: dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})
    angular: dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})


@dataclass
class MockPose:
    """Mock Pose message for testing without ROS2."""

    position: dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})
    orientation: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    )


@dataclass
class MockHeader:
    """Mock Header message for testing without ROS2."""

    stamp: dict[str, int] = field(default_factory=lambda: {"sec": 0, "nanosec": 0})
    frame_id: str = ""


# =============================================================================
# ROS2 Node Configuration
# =============================================================================


@dataclass
class ROS2NodeConfig:
    """Configuration for ROS2 node generation.

    Attributes:
        node_name: Name of the ROS2 node.
        namespace: ROS2 namespace for the node.
        publish_rate_hz: Rate to publish status messages.
        use_sim_time: Whether to use simulation time.
        qos_depth: QoS history depth.
        qos_reliable: Whether to use reliable QoS.
        enable_services: Whether to create ROS2 services.
        enable_subscribers: Whether to create command subscribers.
        frame_id: TF frame ID for messages.
    """

    node_name: str = "robo_infra_controller"
    namespace: str = ""
    publish_rate_hz: float = 50.0
    use_sim_time: bool = False
    qos_depth: int = 10
    qos_reliable: bool = True
    enable_services: bool = True
    enable_subscribers: bool = True
    frame_id: str = "base_link"


# =============================================================================
# Mock ROS2 Node (for testing without ROS2)
# =============================================================================


class MockROS2Node:
    """Mock ROS2 node for testing without ROS2 installed.

    This class provides the same interface as rclpy.node.Node but
    operates without ROS2, allowing unit testing of the integration.
    """

    def __init__(
        self,
        node_name: str,
        *,
        namespace: str = "",
        use_sim_time: bool = False,
    ) -> None:
        """Initialize mock ROS2 node.

        Args:
            node_name: Name of the node.
            namespace: ROS2 namespace.
            use_sim_time: Whether to use simulation time.
        """
        self._node_name = node_name
        self._namespace = namespace
        self._use_sim_time = use_sim_time
        self._publishers: dict[str, MockPublisher] = {}
        self._subscribers: dict[str, MockSubscriber] = {}
        self._services: dict[str, MockService] = {}
        self._timers: list[MockTimer] = []
        self._running = False
        self._spin_thread: threading.Thread | None = None
        self._logger = logging.getLogger(f"ros2.{node_name}")

    def get_name(self) -> str:
        """Get node name."""
        return self._node_name

    def get_namespace(self) -> str:
        """Get node namespace."""
        return self._namespace

    def get_logger(self) -> logging.Logger:
        """Get node logger."""
        return self._logger

    def create_publisher(
        self,
        msg_type: type,
        topic: str,
        qos_profile: Any = None,
    ) -> MockPublisher:
        """Create a mock publisher."""
        pub = MockPublisher(msg_type, topic, qos_profile)
        self._publishers[topic] = pub
        self._logger.debug(f"Created publisher: {topic}")
        return pub

    def create_subscription(
        self,
        msg_type: type,
        topic: str,
        callback: Callable[[Any], None],
        qos_profile: Any = None,
    ) -> MockSubscriber:
        """Create a mock subscriber."""
        sub = MockSubscriber(msg_type, topic, callback, qos_profile)
        self._subscribers[topic] = sub
        self._logger.debug(f"Created subscription: {topic}")
        return sub

    def create_service(
        self,
        srv_type: type,
        service_name: str,
        callback: Callable[[Any, Any], Any],
    ) -> MockService:
        """Create a mock service."""
        srv = MockService(srv_type, service_name, callback)
        self._services[service_name] = srv
        self._logger.debug(f"Created service: {service_name}")
        return srv

    def create_timer(
        self,
        timer_period_sec: float,
        callback: Callable[[], None],
    ) -> MockTimer:
        """Create a mock timer."""
        timer = MockTimer(timer_period_sec, callback)
        self._timers.append(timer)
        self._logger.debug(f"Created timer: {timer_period_sec}s")
        return timer

    def destroy_node(self) -> None:
        """Destroy the node and stop all timers."""
        self._running = False
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
        self._publishers.clear()
        self._subscribers.clear()
        self._services.clear()
        self._logger.info(f"Node {self._node_name} destroyed")

    def get_publishers(self) -> dict[str, MockPublisher]:
        """Get all publishers."""
        return self._publishers

    def get_subscribers(self) -> dict[str, MockSubscriber]:
        """Get all subscribers."""
        return self._subscribers

    def get_services(self) -> dict[str, MockService]:
        """Get all services."""
        return self._services


class MockPublisher:
    """Mock ROS2 publisher."""

    def __init__(
        self,
        msg_type: type,
        topic: str,
        qos_profile: Any = None,
    ) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile
        self.messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        """Publish a message (stores for testing)."""
        self.messages.append(msg)

    def get_subscription_count(self) -> int:
        """Get number of subscribers (mock returns 0)."""
        return 0


class MockSubscriber:
    """Mock ROS2 subscriber."""

    def __init__(
        self,
        msg_type: type,
        topic: str,
        callback: Callable[[Any], None],
        qos_profile: Any = None,
    ) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile

    def receive(self, msg: Any) -> None:
        """Simulate receiving a message."""
        self.callback(msg)


class MockService:
    """Mock ROS2 service."""

    def __init__(
        self,
        srv_type: type,
        service_name: str,
        callback: Callable[[Any, Any], Any],
    ) -> None:
        self.srv_type = srv_type
        self.service_name = service_name
        self.callback = callback

    def call(self, request: Any) -> Any:
        """Simulate calling the service."""
        response = type("Response", (), {})()
        return self.callback(request, response)


class MockTimer:
    """Mock ROS2 timer."""

    def __init__(
        self,
        timer_period_sec: float,
        callback: Callable[[], None],
    ) -> None:
        self.timer_period_sec = timer_period_sec
        self.callback = callback
        self._cancelled = False

    def cancel(self) -> None:
        """Cancel the timer."""
        self._cancelled = True

    def is_cancelled(self) -> bool:
        """Check if timer is cancelled."""
        return self._cancelled


# =============================================================================
# Controller ROS2 Node
# =============================================================================


class ControllerROS2NodeBase(ABC):
    """Base class for controller ROS2 nodes.

    This provides a common interface for both real and mock ROS2 nodes
    that wrap robo-infra controllers.
    """

    def __init__(
        self,
        controller: Controller,
        config: ROS2NodeConfig | None = None,
    ) -> None:
        """Initialize the controller ROS2 node.

        Args:
            controller: The robo-infra controller to wrap.
            config: Node configuration.
        """
        self._controller = controller
        self._config = config or ROS2NodeConfig(node_name=controller.name)

    @property
    def controller(self) -> Controller:
        """Get the wrapped controller."""
        return self._controller

    @property
    def config(self) -> ROS2NodeConfig:
        """Get the node configuration."""
        return self._config

    @abstractmethod
    def get_name(self) -> str:
        """Get node name."""
        ...

    @abstractmethod
    def destroy_node(self) -> None:
        """Destroy the node."""
        ...


class ControllerROS2Node(ControllerROS2NodeBase):
    """ROS2 node that wraps a robo-infra controller.

    This node:
    - Publishes controller status via JointState messages
    - Subscribes to command topics for control
    - Exposes services for home, stop, enable/disable

    Works with either real ROS2 or mock mode for testing.
    """

    def __init__(
        self,
        controller: Controller,
        config: ROS2NodeConfig | None = None,
    ) -> None:
        """Initialize the controller ROS2 node.

        Args:
            controller: The robo-infra controller to wrap.
            config: Node configuration.

        Raises:
            RuntimeError: If ROS2 is not available and mock mode is not enabled.
        """
        super().__init__(controller, config)

        # Create node (real or mock)
        if is_ros2_available():
            self._node: Node | MockROS2Node = self._create_real_node()
        elif is_ros2_mock_mode():
            self._node = self._create_mock_node()
        else:
            raise RuntimeError(
                "ROS2 is not available. Install rclpy or set ROS2_MOCK=true for testing."
            )

        # Setup publishers, subscribers, services
        self._setup_publishers()
        if self._config.enable_subscribers:
            self._setup_subscribers()
        if self._config.enable_services:
            self._setup_services()

        # Create status publish timer
        self._setup_status_timer()

        logger.info(
            "Created ROS2 node '%s' for controller '%s'",
            self._config.node_name,
            controller.name,
        )

    def _create_real_node(self) -> Node:
        """Create a real ROS2 node."""
        if not is_ros2_available():
            raise RuntimeError("ROS2 is not available")

        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init()

        node = Node(
            self._config.node_name,
            namespace=self._config.namespace,
            use_intra_process_comms=True,
        )

        # Set use_sim_time parameter
        if self._config.use_sim_time:
            node.declare_parameter("use_sim_time", True)

        return node

    def _create_mock_node(self) -> MockROS2Node:
        """Create a mock ROS2 node for testing."""
        return MockROS2Node(
            self._config.node_name,
            namespace=self._config.namespace,
            use_sim_time=self._config.use_sim_time,
        )

    def _get_qos_profile(self) -> Any:
        """Get QoS profile for publishers/subscribers."""
        if is_ros2_available() and QoSProfile is not None:
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE
                if self._config.qos_reliable
                else ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=self._config.qos_depth,
            )
        return None

    def _setup_publishers(self) -> None:
        """Setup ROS2 publishers."""
        qos = self._get_qos_profile()

        # Joint state publisher
        joint_state_type = JointState if is_ros2_available() else MockJointState
        self._joint_state_pub = self._node.create_publisher(
            joint_state_type,
            f"{self._controller.name}/joint_states",
            qos,
        )

        # Status string publisher
        string_type = String if is_ros2_available() else str
        self._status_pub = self._node.create_publisher(
            string_type,
            f"{self._controller.name}/status",
            qos,
        )

        logger.debug("Created publishers for controller '%s'", self._controller.name)

    def _setup_subscribers(self) -> None:
        """Setup ROS2 subscribers."""
        qos = self._get_qos_profile()

        # Joint command subscriber
        float64_type = Float64 if is_ros2_available() else float
        self._joint_cmd_sub = self._node.create_subscription(
            float64_type,
            f"{self._controller.name}/joint_commands",
            self._joint_command_callback,
            qos,
        )

        # Velocity command subscriber (for mobile robots)
        twist_type = Twist if is_ros2_available() else MockTwist
        self._cmd_vel_sub = self._node.create_subscription(
            twist_type,
            f"{self._controller.name}/cmd_vel",
            self._cmd_vel_callback,
            qos,
        )

        logger.debug("Created subscribers for controller '%s'", self._controller.name)

    def _setup_services(self) -> None:
        """Setup ROS2 services."""
        # Home service
        trigger_type = Trigger if is_ros2_available() else type("Trigger", (), {})
        self._home_srv = self._node.create_service(
            trigger_type,
            f"{self._controller.name}/home",
            self._home_callback,
        )

        # Stop service
        self._stop_srv = self._node.create_service(
            trigger_type,
            f"{self._controller.name}/stop",
            self._stop_callback,
        )

        # Enable/Disable service
        setbool_type = SetBool if is_ros2_available() else type("SetBool", (), {})
        self._enable_srv = self._node.create_service(
            setbool_type,
            f"{self._controller.name}/enable",
            self._enable_callback,
        )

        logger.debug("Created services for controller '%s'", self._controller.name)

    def _setup_status_timer(self) -> None:
        """Setup timer to publish status periodically."""
        period = 1.0 / self._config.publish_rate_hz
        self._status_timer = self._node.create_timer(
            period,
            self._publish_status,
        )

    def _publish_status(self) -> None:
        """Publish controller status."""
        try:
            # Create JointState message
            joint_state = self._create_joint_state_msg()
            self._joint_state_pub.publish(joint_state)

            # Create status string message
            status = self._controller.status()
            if is_ros2_available() and String is not None:
                status_msg = String()
                status_msg.data = str(status.state.value)
            else:
                status_msg = str(status.state.value)
            self._status_pub.publish(status_msg)

        except Exception as e:
            logger.warning("Failed to publish status: %s", e)

    def _create_joint_state_msg(self) -> Any:
        """Create JointState message from controller state."""
        actuators = self._controller.actuators

        names = [a.name for a in actuators]
        positions = [a.current_value for a in actuators]
        velocities = [0.0] * len(actuators)  # TODO: Track velocities
        efforts = [0.0] * len(actuators)  # TODO: Track efforts

        if is_ros2_available() and JointState is not None and Header is not None:
            msg = JointState()
            msg.header = Header()
            msg.header.frame_id = self._config.frame_id
            # msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            msg.velocity = velocities
            msg.effort = efforts
            return msg
        else:
            return MockJointState(
                header={
                    "stamp": {"sec": int(time.time()), "nanosec": 0},
                    "frame_id": self._config.frame_id,
                },
                name=names,
                position=positions,
                velocity=velocities,
                effort=efforts,
            )

    def _joint_command_callback(self, msg: Any) -> None:
        """Handle joint command messages."""
        logger.debug("Received joint command: %s", msg)
        # TODO: Implement joint command handling

    def _cmd_vel_callback(self, msg: Any) -> None:
        """Handle velocity command messages."""
        logger.debug("Received cmd_vel: %s", msg)
        # TODO: Implement velocity command handling for mobile robots

    def _home_callback(self, request: Any, response: Any) -> Any:
        """Handle home service request."""
        try:
            self._controller.home()
            response.success = True
            response.message = f"Controller '{self._controller.name}' homed successfully"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _stop_callback(self, request: Any, response: Any) -> Any:
        """Handle stop service request."""
        try:
            self._controller.stop()
            response.success = True
            response.message = f"Controller '{self._controller.name}' stopped"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _enable_callback(self, request: Any, response: Any) -> Any:
        """Handle enable/disable service request."""
        try:
            if hasattr(request, "data") and request.data:
                self._controller.enable()
                response.success = True
                response.message = f"Controller '{self._controller.name}' enabled"
            else:
                self._controller.disable()
                response.success = True
                response.message = f"Controller '{self._controller.name}' disabled"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def get_name(self) -> str:
        """Get node name."""
        return self._node.get_name()

    def get_node(self) -> Node | MockROS2Node:
        """Get the underlying ROS2 node."""
        return self._node

    def destroy_node(self) -> None:
        """Destroy the node."""
        self._node.destroy_node()


# =============================================================================
# Factory Functions
# =============================================================================


def controller_to_ros2_node(
    controller: Controller,
    *,
    node_name: str | None = None,
    namespace: str = "",
    publish_rate_hz: float = 50.0,
    use_sim_time: bool = False,
    enable_services: bool = True,
    enable_subscribers: bool = True,
) -> ControllerROS2Node:
    """Convert a robo-infra controller to a ROS2 node.

    This function creates a ROS2 node that:
    - Publishes controller status via `/controller_name/joint_states` (sensor_msgs/JointState)
    - Publishes controller state via `/controller_name/status` (std_msgs/String)
    - Subscribes to `/controller_name/cmd_vel` (geometry_msgs/Twist)
    - Subscribes to `/controller_name/joint_commands` (std_msgs/Float64MultiArray)
    - Exposes `/controller_name/home` service (std_srvs/Trigger)
    - Exposes `/controller_name/stop` service (std_srvs/Trigger)
    - Exposes `/controller_name/enable` service (std_srvs/SetBool)

    Args:
        controller: The robo-infra controller to wrap.
        node_name: ROS2 node name. Defaults to controller name.
        namespace: ROS2 namespace.
        publish_rate_hz: Rate to publish status messages.
        use_sim_time: Whether to use simulation time.
        enable_services: Whether to create ROS2 services.
        enable_subscribers: Whether to create command subscribers.

    Returns:
        A ROS2 node wrapping the controller.

    Raises:
        RuntimeError: If ROS2 is not available and mock mode is not enabled.

    Example:
        >>> from robo_infra.controllers import JointGroup
        >>> from robo_infra.integrations.ros2 import controller_to_ros2_node
        >>>
        >>> arm = JointGroup("arm", joints=[...])
        >>> node = controller_to_ros2_node(arm)
        >>> rclpy.spin(node.get_node())
    """
    config = ROS2NodeConfig(
        node_name=node_name or controller.name,
        namespace=namespace,
        publish_rate_hz=publish_rate_hz,
        use_sim_time=use_sim_time,
        enable_services=enable_services,
        enable_subscribers=enable_subscribers,
    )

    return ControllerROS2Node(controller, config)


def actuator_to_ros2_node(
    actuator: Actuator,
    *,
    node_name: str | None = None,
    namespace: str = "",
    publish_rate_hz: float = 50.0,
) -> ControllerROS2Node:
    """Convert a single actuator to a ROS2 node.

    This is a convenience function for wrapping a single actuator
    in a minimal controller and exposing it as a ROS2 node.

    Args:
        actuator: The actuator to wrap.
        node_name: ROS2 node name. Defaults to actuator name.
        namespace: ROS2 namespace.
        publish_rate_hz: Rate to publish status messages.

    Returns:
        A ROS2 node wrapping the actuator.

    Example:
        >>> from robo_infra.actuators import Servo
        >>> from robo_infra.integrations.ros2 import actuator_to_ros2_node
        >>>
        >>> servo = Servo("gripper")
        >>> node = actuator_to_ros2_node(servo)
    """
    # Create a minimal controller wrapper
    from robo_infra.controllers import JointGroup

    wrapper = JointGroup(
        name=node_name or actuator.name,
        joints={actuator.name: actuator},
    )

    return controller_to_ros2_node(
        wrapper,
        node_name=node_name,
        namespace=namespace,
        publish_rate_hz=publish_rate_hz,
    )


# =============================================================================
# Launch File Generation
# =============================================================================


@dataclass
class LaunchConfig:
    """Configuration for ROS2 launch file generation.

    Attributes:
        package_name: ROS2 package name.
        node_executable: Python executable name.
        parameters_file: Path to parameters YAML file.
        remappings: Topic remappings.
        arguments: Additional command-line arguments.
        output: Output type (screen, log, both).
    """

    package_name: str = "robo_infra"
    node_executable: str = "controller_node"
    parameters_file: str | None = None
    remappings: list[tuple[str, str]] = field(default_factory=list)
    arguments: list[str] = field(default_factory=list)
    output: str = "screen"


def generate_launch_file(
    controller: Controller,
    config: LaunchConfig | None = None,
) -> str:
    """Generate a ROS2 launch file for a controller.

    Args:
        controller: The controller to generate launch file for.
        config: Launch configuration.

    Returns:
        Launch file content as a string.

    Example:
        >>> launch_content = generate_launch_file(arm_controller)
        >>> with open("arm_launch.py", "w") as f:
        ...     f.write(launch_content)
    """
    config = config or LaunchConfig()

    # Generate remappings string
    remappings_str = ""
    if config.remappings:
        remappings_list = ",\n            ".join(
            f"('{src}', '{dst}')" for src, dst in config.remappings
        )
        remappings_str = f"""
        remappings=[
            {remappings_list}
        ],"""

    # Generate parameters string
    parameters_str = ""
    if config.parameters_file:
        parameters_str = f"""
        parameters=['{config.parameters_file}'],"""

    # Generate arguments string
    arguments_str = ""
    if config.arguments:
        args_list = ",\n            ".join(f"'{arg}'" for arg in config.arguments)
        arguments_str = f"""
        arguments=[
            {args_list}
        ],"""

    return f'''"""ROS2 Launch file for {controller.name} controller.

Auto-generated by robo-infra.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription([
        Node(
            package='{config.package_name}',
            executable='{config.node_executable}',
            name='{controller.name}',
            output='{config.output}',{parameters_str}{remappings_str}{arguments_str}
        ),
    ])
'''


def generate_parameters_file(
    controller: Controller,
    config: ROS2NodeConfig | None = None,
) -> str:
    """Generate a ROS2 parameters YAML file for a controller.

    Args:
        controller: The controller to generate parameters for.
        config: Node configuration.

    Returns:
        Parameters YAML content as a string.

    Example:
        >>> params_content = generate_parameters_file(arm_controller)
        >>> with open("arm_params.yaml", "w") as f:
        ...     f.write(params_content)
    """
    config = config or ROS2NodeConfig(node_name=controller.name)

    # Get actuator names and limits
    actuator_params = []
    for actuator in controller.actuators.values():
        actuator_params.append(f"""    {actuator.name}:
      enabled: true
      min_limit: {getattr(actuator, "min_limit", -180.0)}
      max_limit: {getattr(actuator, "max_limit", 180.0)}""")

    actuators_yaml = "\n".join(actuator_params) if actuator_params else "    # No actuators"

    return f'''# ROS2 Parameters for {controller.name} controller
# Auto-generated by robo-infra

{config.node_name}:
  ros__parameters:
    # Node configuration
    publish_rate_hz: {config.publish_rate_hz}
    use_sim_time: {str(config.use_sim_time).lower()}
    frame_id: "{config.frame_id}"

    # Controller settings
    controller_name: "{controller.name}"
    enable_on_startup: false

    # Actuator configuration
    actuators:
{actuators_yaml}
'''


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "ControllerROS2Node",
    # Node classes
    "ControllerROS2NodeBase",
    "LaunchConfig",
    # Message types
    "MessageType",
    "MockHeader",
    "MockJointState",
    "MockPose",
    "MockPublisher",
    # Mock node classes
    "MockROS2Node",
    "MockService",
    "MockSubscriber",
    "MockTimer",
    "MockTwist",
    # Configuration
    "ROS2NodeConfig",
    "actuator_to_ros2_node",
    # Factory functions
    "controller_to_ros2_node",
    # File generation
    "generate_launch_file",
    "generate_parameters_file",
    # Availability checks
    "is_ros2_available",
    "is_ros2_mock_mode",
]
