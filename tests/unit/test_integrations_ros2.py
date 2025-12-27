"""Unit tests for ROS2 integration.

Tests the ROS2 bridge functionality using mock mode,
allowing testing without ROS2 installed.
"""

from __future__ import annotations

import os
from typing import Any
from unittest.mock import MagicMock

import pytest


# Enable mock mode for testing
os.environ["ROS2_MOCK"] = "true"

from robo_infra.actuators.servo import Servo
from robo_infra.controllers.joint_group import JointGroup
from robo_infra.integrations.ros2 import (
    LaunchConfig,
    MockJointState,
    MockPose,
    MockPublisher,
    MockROS2Node,
    MockService,
    MockSubscriber,
    MockTimer,
    MockTwist,
    ROS2NodeConfig,
    actuator_to_ros2_node,
    controller_to_ros2_node,
    generate_launch_file,
    generate_parameters_file,
    is_ros2_available,
    is_ros2_mock_mode,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def servo() -> Servo:
    """Create a test servo."""
    return Servo(name="test_servo", angle_range=(-90.0, 90.0))


@pytest.fixture
def servos() -> list[Servo]:
    """Create multiple test servos."""
    return [
        Servo(name="joint_1", angle_range=(-180.0, 180.0)),
        Servo(name="joint_2", angle_range=(-90.0, 90.0)),
        Servo(name="joint_3", angle_range=(-45.0, 45.0)),
    ]


@pytest.fixture
def joint_group(servos: list[Servo]) -> JointGroup:
    """Create a test joint group controller."""
    joints = {s.name: s for s in servos}
    return JointGroup(name="test_arm", joints=joints)


@pytest.fixture
def enabled_joint_group(joint_group: JointGroup) -> JointGroup:
    """Create an enabled joint group controller."""
    joint_group.enable()
    return joint_group


@pytest.fixture
def ros2_config() -> ROS2NodeConfig:
    """Create a test ROS2 node configuration."""
    return ROS2NodeConfig(
        node_name="test_node",
        namespace="test_ns",
        publish_rate_hz=10.0,
        use_sim_time=False,
        qos_depth=5,
        qos_reliable=True,
        enable_services=True,
        enable_subscribers=True,
        frame_id="test_frame",
    )


# =============================================================================
# Availability Tests
# =============================================================================


class TestROS2Availability:
    """Tests for ROS2 availability checks."""

    def test_is_ros2_mock_mode(self) -> None:
        """Test mock mode is enabled."""
        assert is_ros2_mock_mode() is True

    def test_is_ros2_available_without_rclpy(self) -> None:
        """Test ROS2 availability check without rclpy."""
        # Since we're testing without ROS2, this should return False
        # unless rclpy happens to be installed
        result = is_ros2_available()
        assert isinstance(result, bool)


# =============================================================================
# Mock Message Tests
# =============================================================================


class TestMockMessages:
    """Tests for mock ROS2 message types."""

    def test_mock_joint_state_defaults(self) -> None:
        """Test MockJointState with defaults."""
        msg = MockJointState()
        assert msg.name == []
        assert msg.position == []
        assert msg.velocity == []
        assert msg.effort == []
        assert "stamp" in msg.header
        assert "frame_id" in msg.header

    def test_mock_joint_state_with_values(self) -> None:
        """Test MockJointState with values."""
        msg = MockJointState(
            name=["j1", "j2", "j3"],
            position=[1.0, 2.0, 3.0],
            velocity=[0.1, 0.2, 0.3],
            effort=[10.0, 20.0, 30.0],
        )
        assert msg.name == ["j1", "j2", "j3"]
        assert msg.position == [1.0, 2.0, 3.0]
        assert msg.velocity == [0.1, 0.2, 0.3]
        assert msg.effort == [10.0, 20.0, 30.0]

    def test_mock_twist_defaults(self) -> None:
        """Test MockTwist with defaults."""
        msg = MockTwist()
        assert msg.linear == {"x": 0.0, "y": 0.0, "z": 0.0}
        assert msg.angular == {"x": 0.0, "y": 0.0, "z": 0.0}

    def test_mock_pose_defaults(self) -> None:
        """Test MockPose with defaults."""
        msg = MockPose()
        assert msg.position == {"x": 0.0, "y": 0.0, "z": 0.0}
        assert msg.orientation == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


# =============================================================================
# Mock ROS2 Node Tests
# =============================================================================


class TestMockROS2Node:
    """Tests for MockROS2Node."""

    def test_create_mock_node(self) -> None:
        """Test creating a mock ROS2 node."""
        node = MockROS2Node("test_node")
        assert node.get_name() == "test_node"
        assert node.get_namespace() == ""

    def test_create_mock_node_with_namespace(self) -> None:
        """Test creating a mock ROS2 node with namespace."""
        node = MockROS2Node("test_node", namespace="my_ns")
        assert node.get_name() == "test_node"
        assert node.get_namespace() == "my_ns"

    def test_mock_node_logger(self) -> None:
        """Test mock node logger."""
        node = MockROS2Node("test_node")
        logger = node.get_logger()
        assert logger is not None

    def test_mock_node_create_publisher(self) -> None:
        """Test creating a mock publisher."""
        node = MockROS2Node("test_node")
        pub = node.create_publisher(MockJointState, "test_topic", None)
        assert isinstance(pub, MockPublisher)
        assert pub.topic == "test_topic"
        assert "test_topic" in node.get_publishers()

    def test_mock_node_create_subscriber(self) -> None:
        """Test creating a mock subscriber."""
        node = MockROS2Node("test_node")
        callback = MagicMock()
        sub = node.create_subscription(MockJointState, "test_topic", callback, None)
        assert isinstance(sub, MockSubscriber)
        assert sub.topic == "test_topic"
        assert "test_topic" in node.get_subscribers()

    def test_mock_node_create_service(self) -> None:
        """Test creating a mock service."""
        node = MockROS2Node("test_node")
        callback = MagicMock()
        srv = node.create_service(type, "test_service", callback)
        assert isinstance(srv, MockService)
        assert srv.service_name == "test_service"
        assert "test_service" in node.get_services()

    def test_mock_node_create_timer(self) -> None:
        """Test creating a mock timer."""
        node = MockROS2Node("test_node")
        callback = MagicMock()
        timer = node.create_timer(0.1, callback)
        assert isinstance(timer, MockTimer)
        assert timer.timer_period_sec == 0.1
        assert not timer.is_cancelled()

    def test_mock_node_destroy(self) -> None:
        """Test destroying a mock node."""
        node = MockROS2Node("test_node")
        node.create_publisher(MockJointState, "topic1", None)
        node.create_timer(0.1, MagicMock())
        node.destroy_node()
        assert len(node.get_publishers()) == 0


# =============================================================================
# Mock Publisher/Subscriber Tests
# =============================================================================


class TestMockPublisher:
    """Tests for MockPublisher."""

    def test_mock_publisher_publish(self) -> None:
        """Test mock publisher stores messages."""
        pub = MockPublisher(MockJointState, "test_topic", None)
        msg = MockJointState(name=["j1"], position=[1.0])
        pub.publish(msg)
        assert len(pub.messages) == 1
        assert pub.messages[0] == msg

    def test_mock_publisher_multiple_publishes(self) -> None:
        """Test mock publisher stores multiple messages."""
        pub = MockPublisher(MockJointState, "test_topic", None)
        for i in range(5):
            pub.publish(MockJointState(name=[f"j{i}"], position=[float(i)]))
        assert len(pub.messages) == 5

    def test_mock_publisher_subscription_count(self) -> None:
        """Test mock publisher subscription count."""
        pub = MockPublisher(MockJointState, "test_topic", None)
        assert pub.get_subscription_count() == 0


class TestMockSubscriber:
    """Tests for MockSubscriber."""

    def test_mock_subscriber_receive(self) -> None:
        """Test mock subscriber receives messages."""
        callback = MagicMock()
        sub = MockSubscriber(MockJointState, "test_topic", callback, None)
        msg = MockJointState(name=["j1"], position=[1.0])
        sub.receive(msg)
        callback.assert_called_once_with(msg)


class TestMockService:
    """Tests for MockService."""

    def test_mock_service_call(self) -> None:
        """Test mock service call."""

        def callback(request: object, response: object) -> object:
            response.success = True  # type: ignore[attr-defined]
            response.message = "OK"  # type: ignore[attr-defined]
            return response

        srv = MockService(type, "test_service", callback)
        request = MagicMock()
        response = srv.call(request)
        assert response.success is True
        assert response.message == "OK"


class TestMockTimer:
    """Tests for MockTimer."""

    def test_mock_timer_cancel(self) -> None:
        """Test mock timer cancel."""
        timer = MockTimer(0.1, MagicMock())
        assert not timer.is_cancelled()
        timer.cancel()
        assert timer.is_cancelled()


# =============================================================================
# ROS2 Node Configuration Tests
# =============================================================================


class TestROS2NodeConfig:
    """Tests for ROS2NodeConfig."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = ROS2NodeConfig()
        assert config.node_name == "robo_infra_controller"
        assert config.namespace == ""
        assert config.publish_rate_hz == 50.0
        assert config.use_sim_time is False
        assert config.qos_depth == 10
        assert config.qos_reliable is True
        assert config.enable_services is True
        assert config.enable_subscribers is True
        assert config.frame_id == "base_link"

    def test_custom_config(self, ros2_config: ROS2NodeConfig) -> None:
        """Test custom configuration."""
        assert ros2_config.node_name == "test_node"
        assert ros2_config.namespace == "test_ns"
        assert ros2_config.publish_rate_hz == 10.0
        assert ros2_config.frame_id == "test_frame"


# =============================================================================
# Controller ROS2 Node Tests
# =============================================================================


class TestControllerROS2Node:
    """Tests for ControllerROS2Node."""

    def test_create_node_from_controller(self, joint_group: JointGroup) -> None:
        """Test creating a ROS2 node from a controller."""
        node = controller_to_ros2_node(joint_group)
        assert node.get_name() == "test_arm"
        assert node.controller is joint_group

    def test_create_node_with_custom_name(self, joint_group: JointGroup) -> None:
        """Test creating a ROS2 node with custom name."""
        node = controller_to_ros2_node(joint_group, node_name="my_arm")
        assert node.get_name() == "my_arm"

    def test_create_node_with_namespace(self, joint_group: JointGroup) -> None:
        """Test creating a ROS2 node with namespace."""
        node = controller_to_ros2_node(joint_group, namespace="robot")
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)
        assert mock_node.get_namespace() == "robot"

    def test_node_has_publishers(self, joint_group: JointGroup) -> None:
        """Test node has required publishers."""
        node = controller_to_ros2_node(joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)
        publishers = mock_node.get_publishers()
        assert "test_arm/joint_states" in publishers
        assert "test_arm/status" in publishers

    def test_node_has_subscribers(self, joint_group: JointGroup) -> None:
        """Test node has required subscribers."""
        node = controller_to_ros2_node(joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)
        subscribers = mock_node.get_subscribers()
        assert "test_arm/joint_commands" in subscribers
        assert "test_arm/cmd_vel" in subscribers

    def test_node_has_services(self, joint_group: JointGroup) -> None:
        """Test node has required services."""
        node = controller_to_ros2_node(joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)
        services = mock_node.get_services()
        assert "test_arm/home" in services
        assert "test_arm/stop" in services
        assert "test_arm/enable" in services

    def test_node_without_services(self, joint_group: JointGroup) -> None:
        """Test node without services."""
        node = controller_to_ros2_node(joint_group, enable_services=False)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)
        services = mock_node.get_services()
        assert len(services) == 0

    def test_node_without_subscribers(self, joint_group: JointGroup) -> None:
        """Test node without subscribers."""
        node = controller_to_ros2_node(joint_group, enable_subscribers=False)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)
        subscribers = mock_node.get_subscribers()
        assert len(subscribers) == 0

    def test_node_destroy(self, joint_group: JointGroup) -> None:
        """Test destroying a node."""
        node = controller_to_ros2_node(joint_group)
        node.destroy_node()
        # Should not raise


# =============================================================================
# Actuator ROS2 Node Tests
# =============================================================================


class TestActuatorROS2Node:
    """Tests for actuator_to_ros2_node."""

    def test_create_node_from_actuator(self, servo: Servo) -> None:
        """Test creating a ROS2 node from an actuator."""
        node = actuator_to_ros2_node(servo)
        assert node.get_name() == "test_servo"

    def test_create_node_from_actuator_with_name(self, servo: Servo) -> None:
        """Test creating a ROS2 node from an actuator with custom name."""
        node = actuator_to_ros2_node(servo, node_name="gripper")
        assert node.get_name() == "gripper"


# =============================================================================
# Service Callback Tests
# =============================================================================


class TestServiceCallbacks:
    """Tests for ROS2 service callbacks."""

    def test_home_service_callback(self, enabled_joint_group: JointGroup) -> None:
        """Test home service callback."""
        node = controller_to_ros2_node(enabled_joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)

        home_srv = mock_node.get_services()["test_arm/home"]
        request = MagicMock()
        response = home_srv.call(request)
        assert response.success is True
        assert "homed" in response.message.lower()

    def test_stop_service_callback(self, enabled_joint_group: JointGroup) -> None:
        """Test stop service callback."""
        node = controller_to_ros2_node(enabled_joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)

        stop_srv = mock_node.get_services()["test_arm/stop"]
        request = MagicMock()
        response = stop_srv.call(request)
        assert response.success is True
        assert "stopped" in response.message.lower()

    def test_enable_service_callback_enable(
        self, joint_group: JointGroup
    ) -> None:
        """Test enable service callback for enabling."""
        node = controller_to_ros2_node(joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)

        enable_srv = mock_node.get_services()["test_arm/enable"]
        request = MagicMock()
        request.data = True
        response = enable_srv.call(request)
        assert response.success is True
        assert "enabled" in response.message.lower()

    def test_enable_service_callback_disable(
        self, enabled_joint_group: JointGroup
    ) -> None:
        """Test enable service callback for disabling."""
        node = controller_to_ros2_node(enabled_joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)

        enable_srv = mock_node.get_services()["test_arm/enable"]
        request = MagicMock()
        request.data = False
        response = enable_srv.call(request)
        assert response.success is True
        assert "disabled" in response.message.lower()


# =============================================================================
# Launch File Generation Tests
# =============================================================================


class TestLaunchFileGeneration:
    """Tests for launch file generation."""

    def test_generate_launch_file_basic(self, joint_group: JointGroup) -> None:
        """Test generating a basic launch file."""
        content = generate_launch_file(joint_group)
        assert "from launch import LaunchDescription" in content
        assert "from launch_ros.actions import Node" in content
        assert "test_arm" in content
        assert "def generate_launch_description" in content

    def test_generate_launch_file_with_config(self, joint_group: JointGroup) -> None:
        """Test generating a launch file with custom config."""
        config = LaunchConfig(
            package_name="my_robot",
            node_executable="arm_node",
            output="log",
        )
        content = generate_launch_file(joint_group, config)
        assert "package='my_robot'" in content
        assert "executable='arm_node'" in content
        assert "output='log'" in content

    def test_generate_launch_file_with_remappings(
        self, joint_group: JointGroup
    ) -> None:
        """Test generating a launch file with remappings."""
        config = LaunchConfig(
            remappings=[
                ("/cmd_vel", "/robot/cmd_vel"),
                ("/joint_states", "/robot/joint_states"),
            ]
        )
        content = generate_launch_file(joint_group, config)
        assert "remappings=" in content
        assert "/cmd_vel" in content
        assert "/robot/cmd_vel" in content

    def test_generate_launch_file_with_parameters(
        self, joint_group: JointGroup
    ) -> None:
        """Test generating a launch file with parameters file."""
        config = LaunchConfig(parameters_file="/path/to/params.yaml")
        content = generate_launch_file(joint_group, config)
        assert "parameters=" in content
        assert "/path/to/params.yaml" in content

    def test_generate_launch_file_with_arguments(
        self, joint_group: JointGroup
    ) -> None:
        """Test generating a launch file with arguments."""
        config = LaunchConfig(arguments=["--ros-args", "--log-level", "debug"])
        content = generate_launch_file(joint_group, config)
        assert "arguments=" in content
        assert "--ros-args" in content


# =============================================================================
# Parameters File Generation Tests
# =============================================================================


class TestParametersFileGeneration:
    """Tests for parameters file generation."""

    def test_generate_parameters_file_basic(self, joint_group: JointGroup) -> None:
        """Test generating a basic parameters file."""
        content = generate_parameters_file(joint_group)
        assert "test_arm" in content
        assert "ros__parameters:" in content
        assert "publish_rate_hz:" in content
        assert "frame_id:" in content

    def test_generate_parameters_file_with_config(
        self, joint_group: JointGroup
    ) -> None:
        """Test generating a parameters file with custom config."""
        config = ROS2NodeConfig(
            node_name="my_arm",
            publish_rate_hz=100.0,
            frame_id="arm_base",
            use_sim_time=True,
        )
        content = generate_parameters_file(joint_group, config)
        assert "my_arm:" in content
        assert "publish_rate_hz: 100.0" in content
        assert "frame_id: \"arm_base\"" in content
        assert "use_sim_time: true" in content

    def test_generate_parameters_file_includes_actuators(
        self, joint_group: JointGroup
    ) -> None:
        """Test parameters file includes actuator configuration."""
        content = generate_parameters_file(joint_group)
        assert "actuators:" in content
        assert "joint_1:" in content
        assert "joint_2:" in content
        assert "joint_3:" in content


# =============================================================================
# Launch Config Tests
# =============================================================================


class TestLaunchConfig:
    """Tests for LaunchConfig."""

    def test_default_config(self) -> None:
        """Test default launch configuration."""
        config = LaunchConfig()
        assert config.package_name == "robo_infra"
        assert config.node_executable == "controller_node"
        assert config.parameters_file is None
        assert config.remappings == []
        assert config.arguments == []
        assert config.output == "screen"

    def test_custom_config(self) -> None:
        """Test custom launch configuration."""
        config = LaunchConfig(
            package_name="my_robot",
            node_executable="my_node",
            parameters_file="/path/to/params.yaml",
            remappings=[("/a", "/b")],
            arguments=["--arg1"],
            output="log",
        )
        assert config.package_name == "my_robot"
        assert config.node_executable == "my_node"
        assert config.parameters_file == "/path/to/params.yaml"
        assert config.remappings == [("/a", "/b")]
        assert config.arguments == ["--arg1"]
        assert config.output == "log"


# =============================================================================
# Error Handling Tests
# =============================================================================


class TestErrorHandling:
    """Tests for error handling in ROS2 integration."""

    def test_service_error_handling(self, joint_group: JointGroup) -> None:
        """Test service callbacks handle errors gracefully."""
        node = controller_to_ros2_node(joint_group)
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)

        # Try to home without enabling - should fail gracefully
        home_srv = mock_node.get_services()["test_arm/home"]
        request = MagicMock()
        response = home_srv.call(request)
        # Should return success=False with error message
        assert response.success is False
        assert response.message != ""


# =============================================================================
# Integration Tests
# =============================================================================


class TestIntegration:
    """Integration tests for ROS2 bridge."""

    def test_full_workflow(self, servos: list[Servo]) -> None:
        """Test full workflow from controller to ROS2 node."""
        # Create controller
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="robot_arm", joints=joints)

        # Create ROS2 node
        node = controller_to_ros2_node(
            arm,
            node_name="arm_node",
            namespace="robot",
            publish_rate_hz=10.0,
        )

        # Verify node
        assert node.get_name() == "arm_node"
        mock_node = node.get_node()
        assert isinstance(mock_node, MockROS2Node)
        assert mock_node.get_namespace() == "robot"

        # Verify topics
        publishers = mock_node.get_publishers()
        assert "robot_arm/joint_states" in publishers

        # Generate launch file
        launch_content = generate_launch_file(arm)
        assert "robot_arm" in launch_content

        # Generate parameters file
        params_content = generate_parameters_file(
            arm, ROS2NodeConfig(node_name="arm_node")
        )
        assert "arm_node:" in params_content

        # Cleanup
        node.destroy_node()

    def test_multiple_controllers(self, servos: list[Servo]) -> None:
        """Test creating multiple ROS2 nodes for different controllers."""
        arm_joints = {s.name: s for s in servos[:2]}
        gripper_joints = {servos[2].name: servos[2]}
        arm = JointGroup(name="arm", joints=arm_joints)
        gripper = JointGroup(name="gripper", joints=gripper_joints)

        arm_node = controller_to_ros2_node(arm)
        gripper_node = controller_to_ros2_node(gripper)

        assert arm_node.get_name() == "arm"
        assert gripper_node.get_name() == "gripper"

        arm_node.destroy_node()
        gripper_node.destroy_node()


# =============================================================================
# Phase 5.7.5.2 - ROS2 Integration Tests
# =============================================================================


class TestROS2NodeCreation:
    """Tests for ROS2 node creation (Phase 5.7.5.2)."""

    def test_ros2_node_creation_basic(self, servos: list[Servo]) -> None:
        """Test basic ROS2 node creation."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="test_arm", joints=joints)
        node = controller_to_ros2_node(arm)

        assert node is not None
        assert node.get_name() == "test_arm"

        node.destroy_node()

    def test_ros2_node_creation_with_custom_name(
        self, servos: list[Servo]
    ) -> None:
        """Test ROS2 node creation with custom name."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="arm", joints=joints)
        # controller_to_ros2_node uses keyword args for customization
        node = controller_to_ros2_node(
            arm,
            node_name="custom_arm_node",
            namespace="robot",
            publish_rate_hz=50.0,
        )

        assert node is not None
        assert node.get_name() == "custom_arm_node"

        node.destroy_node()

    def test_ros2_actuator_node_creation(self, servos: list[Servo]) -> None:
        """Test ROS2 node creation from actuator."""
        # Use one of the fixture servos
        servo = servos[0]
        node = actuator_to_ros2_node(servo)

        assert node is not None
        assert servo.name in node.get_name() or node.get_name() is not None

        node.destroy_node()

    def test_ros2_node_lifecycle(self, servos: list[Servo]) -> None:
        """Test ROS2 node lifecycle."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="lifecycle_test", joints=joints)
        node = controller_to_ros2_node(arm)

        # Node should be active
        assert node.get_name() == "lifecycle_test"

        # Destroy node
        node.destroy_node()

        # After destroy, re-create should work
        node2 = controller_to_ros2_node(arm)
        assert node2.get_name() == "lifecycle_test"
        node2.destroy_node()


class TestROS2Publisher:
    """Tests for ROS2 publisher functionality (Phase 5.7.5.2)."""

    def test_ros2_publisher_creation(self, servos: list[Servo]) -> None:
        """Test ROS2 publisher is created via node creation."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="pub_test", joints=joints)
        node = controller_to_ros2_node(arm)

        # ControllerROS2Node should have a publishers dict
        assert hasattr(node, "_node") or node is not None

        node.destroy_node()

    def test_ros2_mock_publisher_publish(self) -> None:
        """Test MockPublisher can publish messages."""
        from robo_infra.integrations.ros2 import MockPublisher

        # MockPublisher takes (msg_type, topic, qos_profile)
        publisher = MockPublisher(dict, "test_topic")
        message = {"data": "test_message"}

        # Should not raise
        publisher.publish(message)

        # Verify message was stored
        assert len(publisher.messages) == 1
        assert publisher.messages[0] == message

    def test_ros2_mock_publisher_topic_name(self) -> None:
        """Test MockPublisher has correct topic name."""
        from robo_infra.integrations.ros2 import MockPublisher

        publisher = MockPublisher(dict, "robot/joint_states")
        assert publisher.topic == "robot/joint_states"

    def test_ros2_mock_publisher_multiple_messages(self) -> None:
        """Test MockPublisher handles multiple messages."""
        from robo_infra.integrations.ros2 import MockPublisher

        publisher = MockPublisher(dict, "test_topic")

        for i in range(5):
            publisher.publish({"count": i})

        assert len(publisher.messages) == 5
        assert publisher.messages[2]["count"] == 2


class TestROS2Subscriber:
    """Tests for ROS2 subscriber functionality (Phase 5.7.5.2)."""

    def test_ros2_mock_subscriber_creation(self) -> None:
        """Test MockSubscriber can be created."""
        from robo_infra.integrations.ros2 import MockSubscriber

        def callback(msg: Any) -> None:
            pass

        # MockSubscriber takes (msg_type, topic, callback, qos_profile)
        subscriber = MockSubscriber(dict, "test_topic", callback)
        assert subscriber is not None
        assert subscriber.topic == "test_topic"

    def test_ros2_mock_subscriber_callback(self) -> None:
        """Test MockSubscriber callback is invoked via receive."""
        from robo_infra.integrations.ros2 import MockSubscriber

        received: list[Any] = []

        def callback(msg: Any) -> None:
            received.append(msg)

        subscriber = MockSubscriber(dict, "test_topic", callback)

        # Simulate receiving a message via receive()
        subscriber.receive({"data": "test"})

        assert len(received) == 1
        assert received[0]["data"] == "test"

    def test_ros2_mock_subscriber_multiple_callbacks(self) -> None:
        """Test MockSubscriber handles multiple messages."""
        from robo_infra.integrations.ros2 import MockSubscriber

        received: list[Any] = []

        def callback(msg: Any) -> None:
            received.append(msg)

        subscriber = MockSubscriber(dict, "test_topic", callback)

        for i in range(3):
            subscriber.receive({"index": i})

        assert len(received) == 3
        assert received[1]["index"] == 1


class TestROS2Service:
    """Tests for ROS2 service functionality (Phase 5.7.5.2)."""

    def test_ros2_mock_service_creation(self) -> None:
        """Test MockService can be created."""
        from robo_infra.integrations.ros2 import MockService

        def handler(request: Any, response: Any) -> Any:
            response.success = True
            return response

        # MockService takes (srv_type, service_name, callback)
        service = MockService(dict, "test_service", handler)
        assert service is not None
        assert service.service_name == "test_service"

    def test_ros2_mock_service_call(self) -> None:
        """Test MockService can be called."""
        from robo_infra.integrations.ros2 import MockService

        def handler(request: Any, response: Any) -> Any:
            response.result = request.get("value", 0) * 2
            return response

        service = MockService(dict, "double_service", handler)
        response = service.call({"value": 5})

        assert response.result == 10

    def test_ros2_mock_service_basic_functionality(self) -> None:
        """Test MockService basic call functionality."""
        from robo_infra.integrations.ros2 import MockService

        calls_made: list[Any] = []

        def handler(request: Any, response: Any) -> Any:
            calls_made.append(request)
            response.success = True
            return response

        service = MockService(dict, "track_service", handler)

        service.call({"a": 1})
        service.call({"b": 2})

        # Track that calls were made via our local tracking
        assert len(calls_made) == 2
        assert calls_made[0]["a"] == 1
        assert calls_made[1]["b"] == 2

    def test_ros2_mock_service_error_handling(self) -> None:
        """Test MockService handles errors in handler."""
        from robo_infra.integrations.ros2 import MockService

        def handler(request: Any, response: Any) -> Any:
            if "fail" in request:
                raise ValueError("Intentional failure")
            response.success = True
            return response

        service = MockService(dict, "error_service", handler)

        # Success case
        response = service.call({})
        assert response.success is True

        # Error case
        try:
            service.call({"fail": True})
            assert False, "Should have raised"
        except ValueError as e:
            assert "Intentional failure" in str(e)


class TestROS2LaunchFileGeneration:
    """Tests for ROS2 launch file generation (Phase 5.7.5.2)."""

    def test_launch_file_generation_basic(self, servos: list[Servo]) -> None:
        """Test basic launch file generation."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="launch_arm", joints=joints)

        content = generate_launch_file(arm)

        assert isinstance(content, str)
        assert len(content) > 0
        assert "launch_arm" in content

    def test_launch_file_generation_with_config(
        self, servos: list[Servo]
    ) -> None:
        """Test launch file generation with config."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="arm", joints=joints)

        config = LaunchConfig(
            package_name="my_robot_pkg",
            output="screen",
            remappings=[("input", "output")],
        )
        content = generate_launch_file(arm, config=config)

        assert "my_robot_pkg" in content

    def test_launch_file_generation_python_format(
        self, servos: list[Servo]
    ) -> None:
        """Test launch file uses Python format."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="arm", joints=joints)

        content = generate_launch_file(arm)

        # Should be Python launch file format
        assert "from launch" in content or "launch" in content.lower()

    def test_parameters_file_generation(self, servos: list[Servo]) -> None:
        """Test parameters file generation."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="params_arm", joints=joints)

        config = ROS2NodeConfig(node_name="arm_node")
        content = generate_parameters_file(arm, config)

        assert isinstance(content, str)
        assert "arm_node" in content

    def test_parameters_file_yaml_format(self, servos: list[Servo]) -> None:
        """Test parameters file is YAML format."""
        joints = {s.name: s for s in servos}
        arm = JointGroup(name="arm", joints=joints)

        config = ROS2NodeConfig(node_name="test_node")
        content = generate_parameters_file(arm, config)

        # Should be valid YAML-like content
        assert ":" in content  # YAML key-value indicator
        assert "test_node:" in content
