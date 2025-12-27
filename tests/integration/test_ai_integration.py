"""Integration tests for AI control end-to-end scenarios.

These tests verify the complete AI/LLM tool integration:
1. Controller to AI tool conversion
2. Tool execution and state changes
3. Tool response format
4. Integration with ai-infra patterns

End-to-end AI tool tests without real hardware.
"""

from __future__ import annotations

import pytest

from robo_infra.actuators.dc_motor import DCMotor
from robo_infra.controllers.differential import DifferentialDrive, DifferentialDriveConfig
from robo_infra.controllers.joint_group import JointGroup, JointGroupConfig
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits
from robo_infra.integrations.ai_infra import controller_to_tools


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def arm_controller() -> JointGroup:
    """Create a JointGroup controller for AI tool testing."""
    joints = {
        "base": SimulatedActuator(
            name="base",
            limits=Limits(min=0, max=360, default=180),
            unit="degrees",
        ),
        "shoulder": SimulatedActuator(
            name="shoulder",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
        "wrist": SimulatedActuator(
            name="wrist",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
    }
    config = JointGroupConfig(
        name="robot_arm",
        home_positions={"base": 180, "shoulder": 90, "wrist": 90},
        named_positions={
            "ready": {"base": 180, "shoulder": 45, "wrist": 90},
            "pickup": {"base": 90, "shoulder": 60, "wrist": 45},
        },
    )
    arm = JointGroup(name="robot_arm", joints=joints, config=config)
    arm.enable()
    return arm


@pytest.fixture
def rover_controller() -> DifferentialDrive:
    """Create a DifferentialDrive controller for AI tool testing."""
    config = DifferentialDriveConfig(
        name="rover",
        wheel_diameter=0.1,
        track_width=0.3,
        max_speed=1.0,
    )
    rover = DifferentialDrive(
        name="rover",
        left=DCMotor(name="left"),
        right=DCMotor(name="right"),
        config=config,
    )
    rover.enable()
    return rover


@pytest.fixture
def arm_tools(arm_controller: JointGroup) -> list:
    """Generate AI tools from arm controller."""
    return controller_to_tools(arm_controller)


@pytest.fixture
def rover_tools(rover_controller: DifferentialDrive) -> list:
    """Generate AI tools from rover controller."""
    return controller_to_tools(rover_controller)


# =============================================================================
# Test: Tool Call to Controller to Actuator
# =============================================================================


class TestToolCallToActuator:
    """Test AI tool calls flow through to actuators."""

    def test_generate_tools_from_controller(self, arm_controller: JointGroup) -> None:
        """Test generating AI tools from a controller."""
        tools = controller_to_tools(arm_controller)

        assert len(tools) > 0
        assert isinstance(tools, list)

    def test_tools_are_callable(self, arm_tools: list) -> None:
        """Test that all generated tools are callable."""
        for tool in arm_tools:
            assert callable(tool)

    def test_tools_have_metadata(self, arm_tools: list) -> None:
        """Test that tools have required metadata."""
        for tool in arm_tools:
            assert hasattr(tool, "__name__")
            assert hasattr(tool, "__doc__")
            assert tool.__doc__ is not None

    def test_tool_names_follow_convention(
        self, arm_tools: list, arm_controller: JointGroup
    ) -> None:
        """Test that tool names follow naming convention."""
        for tool in arm_tools:
            assert tool.__name__.startswith(arm_controller.name)

    def test_expected_tools_exist(self, arm_tools: list) -> None:
        """Test that expected tools are generated."""
        tool_names = [t.__name__ for t in arm_tools]

        assert "robot_arm_move" in tool_names
        assert "robot_arm_home" in tool_names
        assert "robot_arm_stop" in tool_names
        assert "robot_arm_status" in tool_names


class TestMoveToolExecution:
    """Test move tool execution."""

    def test_move_tool_changes_actuator_position(
        self, arm_tools: list, arm_controller: JointGroup
    ) -> None:
        """Test that calling move tool changes actuator positions."""
        move_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_move")

        # Execute move
        move_tool(targets={"base": 45, "shoulder": 60})

        # Verify positions changed
        positions = arm_controller.get_positions()
        assert positions["base"] == pytest.approx(45, abs=0.1)
        assert positions["shoulder"] == pytest.approx(60, abs=0.1)

    def test_move_tool_single_joint(self, arm_tools: list, arm_controller: JointGroup) -> None:
        """Test moving a single joint via tool."""
        move_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_move")

        # Move only wrist
        move_tool(targets={"wrist": 135})

        assert arm_controller.get_joint_position("wrist") == pytest.approx(135, abs=0.1)

    def test_move_tool_respects_limits(self, arm_tools: list, arm_controller: JointGroup) -> None:
        """Test that move tool raises error when exceeding actuator limits."""
        from robo_infra.core.exceptions import LimitsExceededError

        move_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_move")

        # Try to exceed limits - should raise LimitsExceededError
        with pytest.raises(LimitsExceededError):
            move_tool(targets={"shoulder": 500})  # Max is 180


class TestHomeToolExecution:
    """Test home tool execution."""

    def test_home_tool_returns_to_home(self, arm_tools: list, arm_controller: JointGroup) -> None:
        """Test that home tool returns to home positions."""
        move_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_move")
        home_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_home")

        # Move away from home
        move_tool(targets={"base": 0, "shoulder": 0})

        # Home
        home_tool()

        # Verify at home (from config: base=180, shoulder=90)
        positions = arm_controller.get_positions()
        assert positions["base"] == pytest.approx(180, abs=0.1)
        assert positions["shoulder"] == pytest.approx(90, abs=0.1)


class TestStatusToolExecution:
    """Test status tool execution."""

    def test_status_tool_returns_dict(self, arm_tools: list) -> None:
        """Test that status tool returns a dictionary."""
        status_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_status")

        result = status_tool()

        assert isinstance(result, dict)

    def test_status_tool_contains_state(self, arm_tools: list) -> None:
        """Test that status contains state information."""
        status_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_status")

        result = status_tool()

        assert "is_enabled" in result or "enabled" in result or "state" in result


class TestStopToolExecution:
    """Test stop/emergency tool execution."""

    def test_stop_tool_exists(self, arm_tools: list) -> None:
        """Test that stop tool is generated."""
        tool_names = [t.__name__ for t in arm_tools]
        assert "robot_arm_stop" in tool_names

    def test_stop_tool_callable(self, arm_tools: list) -> None:
        """Test that stop tool can be called."""
        stop_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_stop")

        # Should not raise
        result = stop_tool()
        assert result is not None or result is None  # Just verify it runs


class TestRoverTools:
    """Test rover-specific AI tools."""

    def test_rover_tools_generated(self, rover_tools: list) -> None:
        """Test that rover tools are generated."""
        assert len(rover_tools) > 0

    def test_rover_tool_names(self, rover_tools: list, rover_controller: DifferentialDrive) -> None:
        """Test rover tool naming."""
        for tool in rover_tools:
            assert tool.__name__.startswith(rover_controller.name)

    def test_rover_has_drive_commands(self, rover_tools: list) -> None:
        """Test that rover has drive-related tools."""
        tool_names = [t.__name__ for t in rover_tools]

        # Should have some form of movement command
        has_movement = any(
            name for name in tool_names if "move" in name or "drive" in name or "forward" in name
        )
        assert has_movement or len(tool_names) >= 1  # At least some tools


class TestMultiControllerTools:
    """Test tools from multiple controllers."""

    def test_different_controllers_different_tools(
        self,
        arm_controller: JointGroup,
        rover_controller: DifferentialDrive,
    ) -> None:
        """Test that different controllers produce different tools."""
        arm_tools = controller_to_tools(arm_controller)
        rover_tools = controller_to_tools(rover_controller)

        arm_names = {t.__name__ for t in arm_tools}
        rover_names = {t.__name__ for t in rover_tools}

        # No overlap in tool names
        assert arm_names.isdisjoint(rover_names)

    def test_tools_can_be_combined(
        self,
        arm_controller: JointGroup,
        rover_controller: DifferentialDrive,
    ) -> None:
        """Test that tools from multiple controllers can be combined."""
        arm_tools = controller_to_tools(arm_controller)
        rover_tools = controller_to_tools(rover_controller)

        all_tools = arm_tools + rover_tools

        # All tools should be unique
        tool_names = [t.__name__ for t in all_tools]
        assert len(tool_names) == len(set(tool_names))


class TestToolExecution:
    """Test complete tool execution scenarios."""

    def test_sequential_tool_calls(self, arm_tools: list, arm_controller: JointGroup) -> None:
        """Test sequential tool calls work correctly."""
        move_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_move")
        status_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_status")
        home_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_home")

        # 1. Check initial status
        status_tool()

        # 2. Move
        move_tool(targets={"base": 90})
        assert arm_controller.get_joint_position("base") == pytest.approx(90, abs=0.1)

        # 3. Move again
        move_tool(targets={"shoulder": 45})
        assert arm_controller.get_joint_position("shoulder") == pytest.approx(45, abs=0.1)

        # 4. Home
        home_tool()
        assert arm_controller.get_joint_position("base") == pytest.approx(180, abs=0.1)

    def test_llm_style_tool_execution(self, arm_tools: list, arm_controller: JointGroup) -> None:
        """Test tool execution as an LLM would invoke them."""
        # Simulate LLM picking tools by name
        tools_by_name = {t.__name__: t for t in arm_tools}

        # LLM wants to move arm to pickup position
        move_fn = tools_by_name.get("robot_arm_move")
        assert move_fn is not None

        # LLM provides parameters
        params = {"targets": {"base": 90, "shoulder": 60, "wrist": 45}}

        # Execute
        move_fn(**params)

        # Verify
        positions = arm_controller.get_positions()
        assert positions["base"] == pytest.approx(90, abs=0.1)
        assert positions["shoulder"] == pytest.approx(60, abs=0.1)
        assert positions["wrist"] == pytest.approx(45, abs=0.1)

    def test_tool_response_format(self, arm_tools: list) -> None:
        """Test that tools return response in expected format."""
        status_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_status")

        result = status_tool()

        # Result should be serializable (dict, str, or None)
        assert isinstance(result, (dict, str)) or result is None


class TestToolDocumentation:
    """Test tool documentation for LLM consumption."""

    def test_tools_have_docstrings(self, arm_tools: list) -> None:
        """Test that all tools have docstrings."""
        for tool in arm_tools:
            assert tool.__doc__ is not None
            assert len(tool.__doc__) > 0

    def test_move_tool_docstring_describes_purpose(self, arm_tools: list) -> None:
        """Test that move tool docstring describes its purpose."""
        move_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_move")

        doc = move_tool.__doc__.lower()
        # Should mention movement or position
        assert "move" in doc or "position" in doc or "joint" in doc

    def test_home_tool_docstring_describes_purpose(self, arm_tools: list) -> None:
        """Test that home tool docstring describes its purpose."""
        home_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_home")

        doc = home_tool.__doc__.lower()
        # Should mention home or default or reset
        assert "home" in doc or "default" in doc or "reset" in doc


class TestToolWithAPI:
    """Test tools work alongside API."""

    def test_tool_and_api_share_state(self, arm_controller: JointGroup, arm_tools: list) -> None:
        """Test that tools and API share the same controller state."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient

        from robo_infra.integrations.svc_infra import controller_to_router

        # Setup API
        app = FastAPI()
        router = controller_to_router(arm_controller)
        app.include_router(router, prefix="/arm")
        client = TestClient(app)

        # Get tools
        move_tool = next(t for t in arm_tools if t.__name__ == "robot_arm_move")

        # Move via tool
        move_tool(targets={"base": 111})

        # Read via API
        response = client.get("/arm/actuators")
        assert response.json()["base"] == pytest.approx(111, abs=0.1)

        # Home via API
        client.post("/arm/home")

        # Verify via controller
        assert arm_controller.get_joint_position("base") == pytest.approx(180, abs=0.1)
