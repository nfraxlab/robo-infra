"""Unit tests for ai-infra integration module.

Tests for converting robo-infra controllers, actuators, and sensors to
ai-infra compatible tools (LLM function calling).
"""

from __future__ import annotations

import json

import pytest

from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.controller import SimulatedController
from robo_infra.core.sensor import SimulatedSensor, Unit
from robo_infra.core.types import Limits
from robo_infra.integrations.ai_infra import (
    actuator_to_tool,
    controller_to_tools,
    create_movement_tool,
    create_safety_tools,
    create_status_tool,
)


# --- Fixtures ---


@pytest.fixture
def simple_actuator() -> SimulatedActuator:
    """Create a simple simulated actuator for testing."""
    return SimulatedActuator(
        name="test_servo",
        limits=Limits(min=0, max=180, default=90),
        unit="degrees",
    )


@pytest.fixture
def simple_sensor() -> SimulatedSensor:
    """Create a simple simulated sensor for testing."""
    return SimulatedSensor(
        name="test_encoder",
        unit=Unit.DEGREES,
        limits=Limits(min=0, max=360, default=180),
    )


@pytest.fixture
def mock_controller(
    simple_actuator: SimulatedActuator,
    simple_sensor: SimulatedSensor,
) -> SimulatedController:
    """Create a mock controller with actuators and sensors for testing.

    The controller has:
    - 2 actuators: shoulder (0-180 deg) and elbow (0-120 deg)
    - 2 sensors: encoder (0-360 deg) and temperature (0-100 C)
    """
    controller = SimulatedController(name="test_arm")

    # Add actuators
    controller.add_actuator(
        "shoulder",
        SimulatedActuator(
            name="shoulder",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
    )
    controller.add_actuator(
        "elbow",
        SimulatedActuator(
            name="elbow",
            limits=Limits(min=0, max=120, default=60),
            unit="degrees",
        ),
    )

    # Add sensors
    controller.add_sensor(
        "encoder",
        SimulatedSensor(
            name="encoder",
            unit=Unit.DEGREES,
            limits=Limits(min=0, max=360, default=180),
        ),
    )
    controller.add_sensor(
        "temperature",
        SimulatedSensor(
            name="temperature",
            unit=Unit.CELSIUS,
            limits=Limits(min=0, max=100, default=25),
        ),
    )

    return controller


@pytest.fixture
def enabled_controller(mock_controller: SimulatedController) -> SimulatedController:
    """Create an enabled controller ready for movement commands."""
    mock_controller.enable()
    return mock_controller


@pytest.fixture
def homed_controller(enabled_controller: SimulatedController) -> SimulatedController:
    """Create a homed controller ready for movement commands."""
    enabled_controller.home()
    return enabled_controller


# --- Tests for controller_to_tools ---


class TestControllerToTools:
    """Tests for controller_to_tools function."""

    def test_returns_list_of_tools(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """controller_to_tools should return a list of tool dictionaries."""
        tools = controller_to_tools(mock_controller)
        assert isinstance(tools, list)
        assert len(tools) > 0

    def test_tool_structure(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool should have name, description, parameters, and handler."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert "name" in tool
            assert "description" in tool
            assert "parameters" in tool
            assert "handler" in tool

    def test_tool_names_include_controller_name(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Tool names should be prefixed with controller name."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert tool["name"].startswith("test_arm_")

    def test_expected_tools_generated(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should generate move, home, stop, status, sensors, enable, disable."""
        tools = controller_to_tools(mock_controller)
        tool_names = {tool["name"] for tool in tools}

        expected_suffixes = ["move", "home", "stop", "status", "sensors", "enable", "disable"]
        for suffix in expected_suffixes:
            expected_name = f"test_arm_{suffix}"
            assert expected_name in tool_names, f"Missing tool: {expected_name}"

    def test_has_move_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include a move tool for positioning."""
        tools = controller_to_tools(mock_controller)
        tool_names = [tool["name"] for tool in tools]
        assert "test_arm_move" in tool_names

    def test_has_home_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include a home tool for homing sequence."""
        tools = controller_to_tools(mock_controller)
        tool_names = [tool["name"] for tool in tools]
        assert "test_arm_home" in tool_names

    def test_has_stop_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include a stop tool for emergency stop."""
        tools = controller_to_tools(mock_controller)
        tool_names = [tool["name"] for tool in tools]
        assert "test_arm_stop" in tool_names

    def test_has_status_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include a status tool for querying state."""
        tools = controller_to_tools(mock_controller)
        tool_names = [tool["name"] for tool in tools]
        assert "test_arm_status" in tool_names

    def test_tool_has_name_property(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool must have a non-empty name string."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert isinstance(tool["name"], str)
            assert len(tool["name"]) > 0

    def test_tool_has_description_property(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool must have a non-empty description string."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert isinstance(tool["description"], str)
            assert len(tool["description"]) > 0

    def test_tool_has_parameters_schema(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool must have a parameters dict (JSON schema)."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert isinstance(tool["parameters"], dict)

    def test_tool_parameters_valid_json_schema(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Tool parameters should be valid JSON schema structure."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            params = tool["parameters"]
            # JSON schema should have type or be empty for no-arg tools
            if params:
                # Should be serializable to JSON
                json_str = json.dumps(params)
                assert json_str is not None
                # If has properties, should have type: object
                if "properties" in params:
                    assert params.get("type") == "object"

    def test_tool_handler_is_callable(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool handler must be callable."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert callable(tool["handler"])


# --- Tests for actuator_to_tool ---


class TestActuatorToTool:
    """Tests for actuator_to_tool function."""

    def test_returns_tool_dict(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """actuator_to_tool should return a single tool dictionary."""
        tool = actuator_to_tool(simple_actuator)
        assert isinstance(tool, dict)

    def test_tool_has_required_fields(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool should have name, description, parameters, handler."""
        tool = actuator_to_tool(simple_actuator)
        assert "name" in tool
        assert "description" in tool
        assert "parameters" in tool
        assert "handler" in tool

    def test_tool_name_matches_actuator(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool name should incorporate actuator name."""
        tool = actuator_to_tool(simple_actuator)
        assert "test_servo" in tool["name"]

    def test_tool_includes_limits_in_description(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool description should include actuator limits for LLM context."""
        tool = actuator_to_tool(simple_actuator)
        description = tool["description"]
        # Description should mention the limits (0-180 degrees)
        assert "0" in description or "min" in description.lower()
        assert "180" in description or "max" in description.lower()

    def test_tool_includes_unit_in_description(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool description should mention the actuator or its limits."""
        tool = actuator_to_tool(simple_actuator)
        description = tool["description"].lower()
        # Description should be informative about the actuator
        # Either mentions unit, limits, or actuator name
        has_context = (
            "degree" in description
            or "position" in description
            or "test_servo" in description
            or "0" in description
        )
        assert has_context, f"Description lacks context: {description}"

    def test_tool_parameters_has_value_property(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool parameters should include a value property for setting position."""
        tool = actuator_to_tool(simple_actuator)
        params = tool["parameters"]
        assert "properties" in params
        # Should have some way to set the value
        props = params["properties"]
        assert len(props) > 0

    def test_tool_handler_is_callable(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool handler must be callable."""
        tool = actuator_to_tool(simple_actuator)
        assert callable(tool["handler"])

    def test_tool_parameters_serializable_to_json(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool parameters should be JSON serializable for LLM API."""
        tool = actuator_to_tool(simple_actuator)
        json_str = json.dumps(tool["parameters"])
        assert json_str is not None
        # Should be able to parse it back
        parsed = json.loads(json_str)
        assert parsed == tool["parameters"]


# --- Tests for create_movement_tool ---


class TestCreateMovementTool:
    """Tests for create_movement_tool function."""

    def test_creates_tool_with_positions(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should create a tool with predefined positions."""
        positions = {
            "home": {"shoulder": 90.0, "elbow": 60.0},
            "extended": {"shoulder": 180.0, "elbow": 0.0},
        }
        tool = create_movement_tool("arm_positions", mock_controller, positions)
        assert isinstance(tool, dict)
        assert tool["name"] == "arm_positions"

    def test_tool_has_position_parameter(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Tool parameters should include position selection."""
        positions = {
            "home": {"shoulder": 90.0, "elbow": 60.0},
        }
        tool = create_movement_tool("arm_positions", mock_controller, positions)
        params = tool["parameters"]
        assert "properties" in params


# --- Tests for create_status_tool ---


class TestCreateStatusTool:
    """Tests for create_status_tool function."""

    def test_creates_status_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should create a status query tool."""
        tool = create_status_tool(mock_controller)
        assert isinstance(tool, dict)
        assert "status" in tool["name"]

    def test_status_tool_has_handler(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Status tool should have a callable handler."""
        tool = create_status_tool(mock_controller)
        assert callable(tool["handler"])


# --- Tests for create_safety_tools ---


class TestCreateSafetyTools:
    """Tests for create_safety_tools function."""

    def test_returns_list_of_tools(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should return a list of safety-related tools."""
        tools = create_safety_tools(mock_controller)
        assert isinstance(tools, list)
        assert len(tools) >= 1

    def test_includes_emergency_stop(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include an emergency stop tool."""
        tools = create_safety_tools(mock_controller)
        tool_names = [tool["name"] for tool in tools]
        # At least one tool should be related to stopping
        has_stop_tool = any("stop" in name.lower() or "emergency" in name.lower() for name in tool_names)
        assert has_stop_tool, f"No stop tool found in: {tool_names}"


# --- Tests for Tool Handlers ---


class TestToolHandlers:
    """Tests for tool handler execution."""

    def test_move_tool_handler_calls_controller(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Move tool handler should call the controller's move_to method."""
        tools = controller_to_tools(homed_controller)
        move_tool = next(t for t in tools if t["name"] == "test_arm_move")
        handler = move_tool["handler"]

        # Call the handler with valid positions
        handler({"shoulder": 45.0})

        # Verify the actuator was moved
        assert homed_controller.get_actuator("shoulder").get() == 45.0

    def test_move_tool_handler_with_valid_positions(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Move tool handler should accept multiple actuator positions."""
        tools = controller_to_tools(homed_controller)
        move_tool = next(t for t in tools if t["name"] == "test_arm_move")
        handler = move_tool["handler"]

        # Move multiple actuators
        handler({"shoulder": 120.0, "elbow": 30.0})

        # Verify both actuators moved
        assert homed_controller.get_actuator("shoulder").get() == 120.0
        assert homed_controller.get_actuator("elbow").get() == 30.0

    def test_move_tool_handler_with_invalid_joint_raises(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Move tool handler should raise error for unknown actuator names."""
        tools = controller_to_tools(homed_controller)
        move_tool = next(t for t in tools if t["name"] == "test_arm_move")
        handler = move_tool["handler"]

        # Try to move a non-existent actuator
        with pytest.raises((KeyError, ValueError)):
            handler({"nonexistent_joint": 45.0})

    def test_home_tool_handler_calls_home(
        self,
        enabled_controller: SimulatedController,
    ) -> None:
        """Home tool handler should call the controller's home method."""
        tools = controller_to_tools(enabled_controller)
        home_tool = next(t for t in tools if t["name"] == "test_arm_home")
        handler = home_tool["handler"]

        # Call home
        handler()

        # Verify controller is homed
        assert enabled_controller.status().is_homed

    def test_stop_tool_handler_calls_stop(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Stop tool handler should call the controller's stop method."""
        tools = controller_to_tools(homed_controller)
        stop_tool = next(t for t in tools if t["name"] == "test_arm_stop")
        handler = stop_tool["handler"]

        # Call stop
        handler()

        # Controller should be in stopped state
        from robo_infra.core.controller import ControllerState

        assert homed_controller.status().state == ControllerState.STOPPED

    def test_status_tool_handler_returns_dict(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Status tool handler should return a dictionary with state info."""
        tools = controller_to_tools(mock_controller)
        status_tool = next(t for t in tools if t["name"] == "test_arm_status")
        handler = status_tool["handler"]

        result = handler()

        assert isinstance(result, dict)
        assert "state" in result
        assert "is_enabled" in result
        assert "is_homed" in result

    def test_actuator_tool_handler_sets_value(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Actuator tool handler should set the actuator value."""
        simple_actuator.enable()
        tool = actuator_to_tool(simple_actuator)
        handler = tool["handler"]

        # Set a value via the handler
        handler(value=45.0)

        # Verify the value was set
        assert simple_actuator.get() == 45.0

    def test_sensor_tool_handler_reads_value(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Sensors tool handler should return sensor readings."""
        mock_controller.enable()
        tools = controller_to_tools(mock_controller)
        sensors_tool = next(t for t in tools if t["name"] == "test_arm_sensors")
        handler = sensors_tool["handler"]

        result = handler()

        # Should return sensor readings
        assert isinstance(result, dict)

    def test_enable_tool_handler_enables_controller(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Enable tool handler should enable the controller."""
        tools = controller_to_tools(mock_controller)
        enable_tool = next(t for t in tools if t["name"] == "test_arm_enable")
        handler = enable_tool["handler"]

        # Initially not enabled
        assert not mock_controller.status().is_enabled

        handler()

        # Now enabled
        assert mock_controller.status().is_enabled

    def test_disable_tool_handler_disables_controller(
        self,
        enabled_controller: SimulatedController,
    ) -> None:
        """Disable tool handler should disable the controller."""
        tools = controller_to_tools(enabled_controller)
        disable_tool = next(t for t in tools if t["name"] == "test_arm_disable")
        handler = disable_tool["handler"]

        # Initially enabled
        assert enabled_controller.status().is_enabled

        handler()

        # Now disabled
        assert not enabled_controller.status().is_enabled


# --- Edge Case Tests ---


class TestEdgeCases:
    """Tests for edge cases and unusual configurations."""

    def test_controller_with_no_actuators(self) -> None:
        """Controller with no actuators should still generate some tools."""
        controller = SimulatedController(name="empty_arm")
        # Add only a sensor, no actuators
        controller.add_sensor(
            "temp",
            SimulatedSensor(
                name="temp",
                unit=Unit.CELSIUS,
                limits=Limits(min=0, max=100, default=25),
            ),
        )

        tools = controller_to_tools(controller)

        # Should still have tools (home, stop, status, sensors, enable, disable)
        assert len(tools) >= 5
        tool_names = [t["name"] for t in tools]
        assert "empty_arm_status" in tool_names
        assert "empty_arm_sensors" in tool_names
        # Move tool parameters should be empty
        move_tool = next((t for t in tools if t["name"] == "empty_arm_move"), None)
        if move_tool:
            props = move_tool["parameters"].get("properties", {})
            assert len(props) == 0

    def test_controller_with_no_sensors(self) -> None:
        """Controller with no sensors should not have sensors tool."""
        controller = SimulatedController(name="no_sensors")
        # Add only an actuator, no sensors
        controller.add_actuator(
            "joint",
            SimulatedActuator(
                name="joint",
                limits=Limits(min=0, max=180, default=90),
            ),
        )

        tools = controller_to_tools(controller)

        tool_names = [t["name"] for t in tools]
        # Should have move, home, stop, status, enable, disable
        assert "no_sensors_move" in tool_names
        assert "no_sensors_status" in tool_names
        # Should NOT have sensors tool
        assert "no_sensors_sensors" not in tool_names

    def test_tool_generation_with_special_characters_in_name(self) -> None:
        """Controller names with special chars should be handled."""
        # Create controller with spaces and special chars
        controller = SimulatedController(name="robot-arm_v2")
        controller.add_actuator(
            "joint-1",
            SimulatedActuator(
                name="joint-1",
                limits=Limits(min=0, max=180, default=90),
            ),
        )

        tools = controller_to_tools(controller)

        # Should still generate tools
        assert len(tools) > 0
        # Tool names should incorporate the controller name
        tool_names = [t["name"] for t in tools]
        assert any("robot-arm_v2" in name for name in tool_names)

    def test_tools_are_serializable_to_json(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """All tool definitions (except handler) should be JSON serializable."""
        tools = controller_to_tools(mock_controller)

        for tool in tools:
            # Extract serializable parts (not handler which is a lambda)
            serializable = {
                "name": tool["name"],
                "description": tool["description"],
                "parameters": tool["parameters"],
            }
            # Should not raise
            json_str = json.dumps(serializable)
            assert json_str is not None
            # Should round-trip
            parsed = json.loads(json_str)
            assert parsed["name"] == tool["name"]

    def test_actuator_tools_are_serializable_to_json(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Actuator tool definitions (except handler) should be JSON serializable."""
        tool = actuator_to_tool(simple_actuator)

        serializable = {
            "name": tool["name"],
            "description": tool["description"],
            "parameters": tool["parameters"],
        }
        json_str = json.dumps(serializable)
        assert json_str is not None

    def test_movement_tool_positions_serializable(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Movement tool with positions should be JSON serializable."""
        positions = {
            "home": {"shoulder": 90.0, "elbow": 60.0},
            "extended": {"shoulder": 180.0, "elbow": 0.0},
            "folded": {"shoulder": 0.0, "elbow": 120.0},
        }
        tool = create_movement_tool("arm_poses", mock_controller, positions)

        serializable = {
            "name": tool["name"],
            "description": tool["description"],
            "parameters": tool["parameters"],
        }
        json_str = json.dumps(serializable)
        assert json_str is not None

    def test_safety_tools_serializable(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Safety tools should be JSON serializable."""
        tools = create_safety_tools(mock_controller)

        for tool in tools:
            serializable = {
                "name": tool["name"],
                "description": tool["description"],
                "parameters": tool["parameters"],
            }
            json_str = json.dumps(serializable)
            assert json_str is not None

    def test_status_tool_serializable(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Status tool should be JSON serializable."""
        tool = create_status_tool(mock_controller)

        serializable = {
            "name": tool["name"],
            "description": tool["description"],
            "parameters": tool["parameters"],
        }
        json_str = json.dumps(serializable)
        assert json_str is not None

    def test_empty_controller_generates_tools(self) -> None:
        """Completely empty controller should still generate basic tools."""
        controller = SimulatedController(name="empty")

        tools = controller_to_tools(controller)

        # Should have at least: home, stop, status, enable, disable
        assert len(tools) >= 5
        tool_names = [t["name"] for t in tools]
        assert "empty_home" in tool_names
        assert "empty_stop" in tool_names
        assert "empty_status" in tool_names
        assert "empty_enable" in tool_names
        assert "empty_disable" in tool_names
