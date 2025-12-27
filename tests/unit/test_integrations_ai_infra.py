"""Unit tests for ai-infra integration module.

Tests for converting robo-infra controllers, actuators, and sensors to
ai-infra compatible function tools (LLM function calling).

The new format uses callable Python functions with:
- __name__: Tool name for LLM
- __doc__: Tool description/docstring
- Function itself is callable with typed parameters
"""

from __future__ import annotations

import inspect

import pytest

from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.controller import SimulatedController
from robo_infra.core.sensor import SimulatedSensor, Unit
from robo_infra.core.types import Limits
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


# --- Tests for controller_to_tools (New Function Format) ---


class TestControllerToTools:
    """Tests for controller_to_tools function (new format)."""

    def test_returns_list_of_callables(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """controller_to_tools should return a list of callable functions."""
        tools = controller_to_tools(mock_controller)
        assert isinstance(tools, list)
        assert len(tools) > 0
        for tool in tools:
            assert callable(tool)

    def test_tools_have_names(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool should have a __name__ attribute."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert hasattr(tool, "__name__")
            assert isinstance(tool.__name__, str)
            assert len(tool.__name__) > 0

    def test_tools_have_docstrings(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool should have a __doc__ attribute (docstring)."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert hasattr(tool, "__doc__")
            assert tool.__doc__ is not None
            assert len(tool.__doc__) > 0

    def test_tool_names_include_controller_name(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Tool names should be prefixed with controller name."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert tool.__name__.startswith("test_arm_")

    def test_expected_tools_generated(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should generate move, home, stop, status, sensors, enable, disable."""
        tools = controller_to_tools(mock_controller)
        tool_names = {tool.__name__ for tool in tools}

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
        tool_names = [tool.__name__ for tool in tools]
        assert "test_arm_move" in tool_names

    def test_has_home_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include a home tool for homing sequence."""
        tools = controller_to_tools(mock_controller)
        tool_names = [tool.__name__ for tool in tools]
        assert "test_arm_home" in tool_names

    def test_has_stop_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include a stop tool for emergency stop."""
        tools = controller_to_tools(mock_controller)
        tool_names = [tool.__name__ for tool in tools]
        assert "test_arm_stop" in tool_names

    def test_has_status_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Should include a status tool for querying state."""
        tools = controller_to_tools(mock_controller)
        tool_names = [tool.__name__ for tool in tools]
        assert "test_arm_status" in tool_names

    def test_move_tool_has_type_hints(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Move tool should have type hints for ai-infra schema generation."""
        tools = controller_to_tools(mock_controller)
        move_tool = next(t for t in tools if t.__name__ == "test_arm_move")

        # Check signature has type hints
        sig = inspect.signature(move_tool)
        assert len(sig.parameters) > 0  # Has at least one parameter

    def test_status_tool_returns_dict(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Status tool return type should be dict."""
        tools = controller_to_tools(mock_controller)
        status_tool = next(t for t in tools if t.__name__ == "test_arm_status")

        # Call it and verify return type
        result = status_tool()
        assert isinstance(result, dict)


# --- Tests for actuator_to_tools (New Function Format) ---


class TestActuatorToTools:
    """Tests for actuator_to_tools function (new format)."""

    def test_returns_list_of_callables(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """actuator_to_tools should return a list of callable functions."""
        tools = actuator_to_tools(simple_actuator)
        assert isinstance(tools, list)
        assert len(tools) > 0
        for tool in tools:
            assert callable(tool)

    def test_returns_four_tools(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Should return 4 tools: set, get, enable, disable."""
        tools = actuator_to_tools(simple_actuator)
        assert len(tools) == 4

    def test_tool_names_match_actuator(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool names should incorporate actuator name."""
        tools = actuator_to_tools(simple_actuator)
        tool_names = {tool.__name__ for tool in tools}

        expected = {
            "test_servo_set",
            "test_servo_get",
            "test_servo_enable",
            "test_servo_disable",
        }
        assert tool_names == expected

    def test_tools_have_docstrings(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Each tool should have a docstring."""
        tools = actuator_to_tools(simple_actuator)
        for tool in tools:
            assert tool.__doc__ is not None
            assert len(tool.__doc__) > 0

    def test_docstrings_include_limits(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Tool docstrings should include actuator limits for LLM context."""
        tools = actuator_to_tools(simple_actuator)
        set_tool = next(t for t in tools if t.__name__ == "test_servo_set")

        # Docstring should mention the limits (0-180)
        assert "0" in set_tool.__doc__
        assert "180" in set_tool.__doc__

    def test_set_tool_has_value_parameter(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Set tool should have a value parameter."""
        tools = actuator_to_tools(simple_actuator)
        set_tool = next(t for t in tools if t.__name__ == "test_servo_set")

        sig = inspect.signature(set_tool)
        assert "value" in sig.parameters

    def test_get_tool_returns_dict(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Get tool should return a dict with value and status."""
        simple_actuator.enable()
        tools = actuator_to_tools(simple_actuator)
        get_tool = next(t for t in tools if t.__name__ == "test_servo_get")

        result = get_tool()
        assert isinstance(result, dict)
        assert "value" in result
        assert "is_enabled" in result


# --- Tests for Individual Tool Creators ---


class TestIndividualToolCreators:
    """Tests for individual create_*_tool functions."""

    def test_create_move_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_move_tool should return a callable function."""
        tool = create_move_tool(mock_controller)
        assert callable(tool)
        assert "move" in tool.__name__
        assert tool.__doc__ is not None

    def test_create_home_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_home_tool should return a callable function."""
        tool = create_home_tool(mock_controller)
        assert callable(tool)
        assert "home" in tool.__name__
        assert tool.__doc__ is not None

    def test_create_stop_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_stop_tool should return a callable function."""
        tool = create_stop_tool(mock_controller)
        assert callable(tool)
        assert "stop" in tool.__name__
        assert tool.__doc__ is not None

    def test_create_status_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_status_tool should return a callable function."""
        tool = create_status_tool(mock_controller)
        assert callable(tool)
        assert "status" in tool.__name__
        assert tool.__doc__ is not None

    def test_create_sensors_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_sensors_tool should return a callable function."""
        tool = create_sensors_tool(mock_controller)
        assert callable(tool)
        assert "sensors" in tool.__name__
        assert tool.__doc__ is not None

    def test_create_enable_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_enable_tool should return a callable function."""
        tool = create_enable_tool(mock_controller)
        assert callable(tool)
        assert "enable" in tool.__name__
        assert tool.__doc__ is not None

    def test_create_disable_tool(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_disable_tool should return a callable function."""
        tool = create_disable_tool(mock_controller)
        assert callable(tool)
        assert "disable" in tool.__name__
        assert tool.__doc__ is not None


# --- Tests for Pydantic Schema Tools ---


class TestControllerToSchemaTools:
    """Tests for controller_to_schema_tools function."""

    def test_returns_list_of_callables(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """controller_to_schema_tools should return a list of callable functions."""
        tools = controller_to_schema_tools(mock_controller)
        assert isinstance(tools, list)
        assert len(tools) > 0
        for tool in tools:
            assert callable(tool)

    def test_tools_have_names(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool should have a __name__ attribute."""
        tools = controller_to_schema_tools(mock_controller)
        for tool in tools:
            assert hasattr(tool, "__name__")
            assert isinstance(tool.__name__, str)

    def test_tools_have_docstrings(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Each tool should have a docstring."""
        tools = controller_to_schema_tools(mock_controller)
        for tool in tools:
            assert tool.__doc__ is not None


# --- Tests for Deprecated Functions ---


class TestDeprecatedFunctions:
    """Tests for deprecated functions (old dict format)."""

    def test_actuator_to_tool_returns_dict(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """actuator_to_tool (deprecated) should return a tool dict."""
        with pytest.warns(DeprecationWarning):
            tool = actuator_to_tool(simple_actuator)
        assert isinstance(tool, dict)
        assert "name" in tool
        assert "description" in tool
        assert "parameters" in tool
        assert "handler" in tool

    def test_create_movement_tool_returns_dict(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_movement_tool (deprecated) should return a tool dict."""
        positions = {
            "home": {"shoulder": 90.0, "elbow": 60.0},
        }
        with pytest.warns(DeprecationWarning):
            tool = create_movement_tool("arm_positions", mock_controller, positions)
        assert isinstance(tool, dict)
        assert tool["name"] == "arm_positions"

    def test_create_safety_tools_returns_list_of_dicts(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """create_safety_tools (deprecated) should return a list of tool dicts."""
        with pytest.warns(DeprecationWarning):
            tools = create_safety_tools(mock_controller)
        assert isinstance(tools, list)
        assert len(tools) >= 1
        for tool in tools:
            assert isinstance(tool, dict)
            assert "name" in tool


# --- Tests for Tool Handler Execution ---


class TestToolHandlers:
    """Tests for tool function execution."""

    def test_move_tool_calls_controller(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Move tool should call the controller's move_to method."""
        tools = controller_to_tools(homed_controller)
        move_tool = next(t for t in tools if t.__name__ == "test_arm_move")

        # Call the tool with valid positions
        move_tool(targets={"shoulder": 45.0})

        # Verify the actuator was moved
        assert homed_controller.get_actuator("shoulder").get() == 45.0

    def test_move_tool_with_multiple_positions(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Move tool should accept multiple actuator positions."""
        tools = controller_to_tools(homed_controller)
        move_tool = next(t for t in tools if t.__name__ == "test_arm_move")

        # Move multiple actuators
        move_tool(targets={"shoulder": 120.0, "elbow": 30.0})

        # Verify both actuators moved
        assert homed_controller.get_actuator("shoulder").get() == 120.0
        assert homed_controller.get_actuator("elbow").get() == 30.0

    def test_move_tool_with_invalid_joint_raises(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Move tool should raise error for unknown actuator names."""
        tools = controller_to_tools(homed_controller)
        move_tool = next(t for t in tools if t.__name__ == "test_arm_move")

        # Try to move a non-existent actuator
        with pytest.raises((KeyError, ValueError)):
            move_tool(targets={"nonexistent_joint": 45.0})

    def test_home_tool_calls_home(
        self,
        enabled_controller: SimulatedController,
    ) -> None:
        """Home tool should call the controller's home method."""
        tools = controller_to_tools(enabled_controller)
        home_tool = next(t for t in tools if t.__name__ == "test_arm_home")

        # Call home
        home_tool()

        # Verify controller is homed
        assert enabled_controller.status().is_homed

    def test_stop_tool_calls_stop(
        self,
        homed_controller: SimulatedController,
    ) -> None:
        """Stop tool should call the controller's stop method."""
        tools = controller_to_tools(homed_controller)
        stop_tool = next(t for t in tools if t.__name__ == "test_arm_stop")

        # Call stop
        stop_tool()

        # Controller should be in stopped state
        from robo_infra.core.controller import ControllerState

        assert homed_controller.status().state == ControllerState.STOPPED

    def test_status_tool_returns_dict(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Status tool should return a dictionary with state info."""
        tools = controller_to_tools(mock_controller)
        status_tool = next(t for t in tools if t.__name__ == "test_arm_status")

        result = status_tool()

        assert isinstance(result, dict)
        assert "state" in result
        assert "is_enabled" in result
        assert "is_homed" in result

    def test_actuator_set_tool_sets_value(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Actuator set tool should set the actuator value."""
        simple_actuator.enable()
        tools = actuator_to_tools(simple_actuator)
        set_tool = next(t for t in tools if t.__name__ == "test_servo_set")

        # Set a value via the tool
        set_tool(value=45.0)

        # Verify the value was set
        assert simple_actuator.get() == 45.0

    def test_sensor_tool_reads_values(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Sensors tool should return sensor readings."""
        mock_controller.enable()
        tools = controller_to_tools(mock_controller)
        sensors_tool = next(t for t in tools if t.__name__ == "test_arm_sensors")

        result = sensors_tool()

        # Should return sensor readings
        assert isinstance(result, dict)

    def test_enable_tool_enables_controller(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Enable tool should enable the controller."""
        tools = controller_to_tools(mock_controller)
        enable_tool = next(t for t in tools if t.__name__ == "test_arm_enable")

        # Initially not enabled
        assert not mock_controller.status().is_enabled

        enable_tool()

        # Now enabled
        assert mock_controller.status().is_enabled

    def test_disable_tool_disables_controller(
        self,
        enabled_controller: SimulatedController,
    ) -> None:
        """Disable tool should disable the controller."""
        tools = controller_to_tools(enabled_controller)
        disable_tool = next(t for t in tools if t.__name__ == "test_arm_disable")

        # Initially enabled
        assert enabled_controller.status().is_enabled

        disable_tool()

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
        tool_names = [t.__name__ for t in tools]
        assert "empty_arm_status" in tool_names
        assert "empty_arm_sensors" in tool_names

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

        tool_names = [t.__name__ for t in tools]
        # Should have move, home, stop, status, enable, disable
        assert "no_sensors_move" in tool_names
        assert "no_sensors_status" in tool_names
        # Should NOT have sensors tool
        assert "no_sensors_sensors" not in tool_names

    def test_tool_generation_with_special_characters_in_name(self) -> None:
        """Controller names with special chars should be handled."""
        # Create controller with dashes and underscores
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
        tool_names = [t.__name__ for t in tools]
        assert any("robot-arm_v2" in name for name in tool_names)

    def test_empty_controller_generates_tools(self) -> None:
        """Completely empty controller should still generate basic tools."""
        controller = SimulatedController(name="empty")

        tools = controller_to_tools(controller)

        # Should have at least: move (even if empty), home, stop, status, enable, disable
        assert len(tools) >= 5
        tool_names = [t.__name__ for t in tools]
        assert "empty_home" in tool_names
        assert "empty_stop" in tool_names
        assert "empty_status" in tool_names
        assert "empty_enable" in tool_names
        assert "empty_disable" in tool_names

    def test_tools_have_proper_signatures(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Tools should have proper function signatures for ai-infra."""
        tools = controller_to_tools(mock_controller)

        for tool in tools:
            sig = inspect.signature(tool)
            # All parameters should have names (no positional-only)
            for param_name, param in sig.parameters.items():
                assert param_name != ""
                # Check annotations exist where expected
                if param_name == "targets":
                    assert param.annotation != inspect.Parameter.empty


# --- ai-infra Compatibility Tests ---


class TestAIInfraCompatibility:
    """Tests verifying tools work with ai-infra Agent format."""

    def test_tools_are_callable(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """All tools should be callable (required by ai-infra Agent)."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert callable(tool)

    def test_tools_have_name_attribute(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """All tools should have __name__ (used for tool name in ai-infra)."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert hasattr(tool, "__name__")
            assert isinstance(tool.__name__, str)
            assert len(tool.__name__) > 0

    def test_tools_have_doc_attribute(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """All tools should have __doc__ (used for tool description in ai-infra)."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            assert hasattr(tool, "__doc__")
            assert tool.__doc__ is not None
            assert len(tool.__doc__) > 0

    def test_can_get_function_signature(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """ai-infra uses inspect.signature to generate schemas."""
        tools = controller_to_tools(mock_controller)
        for tool in tools:
            # Should not raise
            sig = inspect.signature(tool)
            assert sig is not None

    def test_tool_returns_string_or_dict(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Tools should return string or dict (serializable for LLM)."""
        mock_controller.enable()
        mock_controller.home()
        tools = controller_to_tools(mock_controller)

        for tool in tools:
            # Get required params
            sig = inspect.signature(tool)
            kwargs = {}
            for param_name, param in sig.parameters.items():
                if param.default == inspect.Parameter.empty:
                    # Required param - provide a value
                    if param_name == "targets":
                        kwargs["targets"] = {"shoulder": 90.0}

            result = tool(**kwargs)
            assert isinstance(result, (str, dict))


# --- Tests for ai-infra generic utility re-exports ---


class TestAiInfraReExports:
    """Tests verifying ai-infra generic utilities are properly re-exported."""

    def test_tools_from_object_is_exported(self) -> None:
        """tools_from_object should be re-exported from robo-infra."""
        from robo_infra.integrations.ai_infra import tools_from_object

        assert callable(tools_from_object)

    def test_tool_exclude_is_exported(self) -> None:
        """tool_exclude should be re-exported from robo-infra."""
        from robo_infra.integrations.ai_infra import tool_exclude

        assert callable(tool_exclude)


class TestToolsFromObjectUsage:
    """Tests for using tools_from_object generic utility."""

    def test_tools_from_object_creates_tools(self) -> None:
        """tools_from_object should create tools from object methods."""
        from robo_infra.integrations.ai_infra import tools_from_object

        class Calculator:
            def add(self, a: float, b: float) -> float:
                """Add two numbers."""
                return a + b

            def multiply(self, a: float, b: float) -> float:
                """Multiply two numbers."""
                return a * b

        tools = tools_from_object(Calculator(), prefix="calc")

        # Should have 2 tools
        assert len(tools) == 2

        # Tools should be callable
        for tool in tools:
            assert callable(tool)
            assert hasattr(tool, "__name__")
            assert hasattr(tool, "__doc__")

    def test_tools_from_object_with_method_filter(self) -> None:
        """tools_from_object should filter to specific methods."""
        from robo_infra.integrations.ai_infra import tools_from_object

        class MathService:
            def add(self, a: float, b: float) -> float:
                return a + b

            def subtract(self, a: float, b: float) -> float:
                return a - b

            def multiply(self, a: float, b: float) -> float:
                return a * b

        tools = tools_from_object(
            MathService(),
            methods=["add", "subtract"],
            prefix="math",
        )

        # Should only have 2 tools (add and subtract)
        assert len(tools) == 2
        tool_names = [t.__name__ for t in tools]
        assert "math_add" in tool_names
        assert "math_subtract" in tool_names
        assert "math_multiply" not in tool_names

    def test_tool_exclude_decorator(self) -> None:
        """tool_exclude should exclude methods from tool generation."""
        from robo_infra.integrations.ai_infra import tool_exclude, tools_from_object

        class Service:
            def public_action(self) -> str:
                return "public"

            @tool_exclude
            def internal_helper(self) -> str:
                return "internal"

        tools = tools_from_object(Service(), prefix="svc")
        tool_names = [t.__name__ for t in tools]

        # public_action should be included
        assert any("public_action" in name for name in tool_names)
        # internal_helper should be excluded
        assert not any("internal_helper" in name for name in tool_names)
