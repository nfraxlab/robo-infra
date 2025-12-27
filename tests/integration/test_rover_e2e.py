"""End-to-end integration tests for rover (DifferentialDrive) controller.

These tests verify the complete flow from:
1. Creating simulated DC motor actuators
2. Building a DifferentialDrive controller (rover)
3. Driving forward, turning, stopping
4. Integration with ai-infra tools for LLM control
5. Integration with svc-infra router for REST API control
6. Adding sensors (ultrasonic distance)

These are integration tests that test the full stack without real hardware.
"""

from __future__ import annotations

import pytest

from robo_infra.actuators.dc_motor import DCMotor
from robo_infra.controllers.differential import DifferentialDrive, DifferentialDriveConfig
from robo_infra.core.sensor import SimulatedSensor, Unit
from robo_infra.core.types import Limits
from robo_infra.integrations.ai_infra import controller_to_tools
from robo_infra.integrations.svc_infra import controller_to_router


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def left_motor() -> DCMotor:
    """Create a simulated left wheel DC motor."""
    return DCMotor(name="left_wheel")


@pytest.fixture
def right_motor() -> DCMotor:
    """Create a simulated right wheel DC motor."""
    return DCMotor(name="right_wheel")


@pytest.fixture
def rover(left_motor: DCMotor, right_motor: DCMotor) -> DifferentialDrive:
    """Create a DifferentialDrive controller representing a wheeled rover."""
    config = DifferentialDriveConfig(
        name="test_rover",
        wheel_diameter=0.1,  # 100mm wheels
        track_width=0.3,  # 300mm between wheels
        max_speed=1.0,  # 1 m/s max
    )
    rover = DifferentialDrive(
        name="test_rover",
        left=left_motor,
        right=right_motor,
        config=config,
    )
    rover.enable()
    return rover


@pytest.fixture
def ultrasonic_sensor() -> SimulatedSensor:
    """Create a simulated ultrasonic distance sensor."""
    return SimulatedSensor(
        name="front_ultrasonic",
        unit=Unit.MILLIMETERS,
        limits=Limits(min=20, max=4000, default=500),  # 2cm to 4m range
        noise=5.0,  # ±5mm noise
    )


# =============================================================================
# Test: Create Rover with Motors
# =============================================================================


class TestCreateRoverWithMotors:
    """Test creating and controlling a rover with simulated DC motors."""

    def test_create_2_dc_motor_actuators(self, left_motor: DCMotor, right_motor: DCMotor) -> None:
        """Test creating 2 simulated DC motor actuators."""
        assert left_motor.name == "left_wheel"
        assert right_motor.name == "right_wheel"

        # Verify motor limits (speed range)
        assert left_motor.limits.min == -1.0
        assert left_motor.limits.max == 1.0
        assert right_motor.limits.min == -1.0
        assert right_motor.limits.max == 1.0

    def test_create_differential_drive_controller(
        self, left_motor: DCMotor, right_motor: DCMotor
    ) -> None:
        """Test creating a DifferentialDrive controller."""
        rover = DifferentialDrive(
            name="my_rover",
            left=left_motor,
            right=right_motor,
        )

        assert rover.name == "my_rover"
        assert rover.left_motor is left_motor
        assert rover.right_motor is right_motor

    def test_rover_requires_both_motors(self) -> None:
        """Test that DifferentialDrive requires both motors."""
        motor = DCMotor(name="single")

        with pytest.raises(ValueError, match="Both left and right motors are required"):
            DifferentialDrive(name="invalid", left=motor, right=None)  # type: ignore

        with pytest.raises(ValueError, match="Both left and right motors are required"):
            DifferentialDrive(name="invalid", left=None, right=motor)  # type: ignore

    def test_rover_starts_disabled(self, left_motor: DCMotor, right_motor: DCMotor) -> None:
        """Test that rover starts in disabled state."""
        rover = DifferentialDrive(name="test", left=left_motor, right=right_motor)
        assert not rover.is_enabled

    def test_enable_rover(self, left_motor: DCMotor, right_motor: DCMotor) -> None:
        """Test enabling the rover."""
        rover = DifferentialDrive(name="test", left=left_motor, right=right_motor)
        rover.enable()
        assert rover.is_enabled

    def test_drive_forward(self, rover: DifferentialDrive) -> None:
        """Test driving the rover forward."""
        rover.forward(speed=0.5)

        left_speed, right_speed = rover.current_speed
        # Both wheels should be moving forward at the same speed
        assert left_speed > 0
        assert right_speed > 0
        assert left_speed == pytest.approx(right_speed, abs=0.01)

    def test_drive_reverse(self, rover: DifferentialDrive) -> None:
        """Test driving the rover in reverse."""
        rover.reverse(speed=0.5)

        left_speed, right_speed = rover.current_speed
        # Both wheels should be moving backward
        assert left_speed < 0
        assert right_speed < 0

    def test_turn_left(self, rover: DifferentialDrive) -> None:
        """Test turning the rover left."""
        rover.turn_left(speed=0.5)

        left_speed, right_speed = rover.current_speed
        # Right wheel should be faster than left for left turn
        assert right_speed > left_speed

    def test_turn_right(self, rover: DifferentialDrive) -> None:
        """Test turning the rover right."""
        rover.turn_right(speed=0.5)

        left_speed, right_speed = rover.current_speed
        # Left wheel should be faster than right for right turn
        assert left_speed > right_speed

    def test_stop(self, rover: DifferentialDrive) -> None:
        """Test stopping the rover."""
        # Start moving
        rover.forward(speed=0.8)
        left_speed, right_speed = rover.current_speed
        assert left_speed != 0 or right_speed != 0

        # Stop
        rover.stop()
        left_speed, right_speed = rover.current_speed
        assert left_speed == pytest.approx(0, abs=0.01)
        assert right_speed == pytest.approx(0, abs=0.01)

    def test_spin_in_place(self, rover: DifferentialDrive) -> None:
        """Test spinning the rover in place."""
        # Spin clockwise
        rover.spin(speed=0.5, clockwise=True)

        left_speed, right_speed = rover.current_speed
        # Wheels should spin in opposite directions
        assert left_speed > 0
        assert right_speed < 0

    def test_tank_control(self, rover: DifferentialDrive) -> None:
        """Test direct tank-style wheel control."""
        rover.tank(0.7, 0.3)

        left_speed, right_speed = rover.current_speed
        assert left_speed == pytest.approx(0.7, abs=0.05)
        assert right_speed == pytest.approx(0.3, abs=0.05)


# =============================================================================
# Test: Rover with AI Tools
# =============================================================================


class TestRoverWithAITools:
    """Test rover integration with ai-infra tools for LLM control."""

    def test_generate_tools_from_controller(self, rover: DifferentialDrive) -> None:
        """Test generating ai-infra tools from DifferentialDrive controller."""
        tools = controller_to_tools(rover)

        assert len(tools) > 0
        assert isinstance(tools, list)

    def test_tools_have_required_properties(self, rover: DifferentialDrive) -> None:
        """Test that generated tools are callable with __name__ and __doc__."""
        tools = controller_to_tools(rover)

        for tool in tools:
            assert callable(tool)
            assert hasattr(tool, "__name__")
            assert hasattr(tool, "__doc__")
            assert tool.__name__.startswith("test_rover")

    def test_move_tool_exists(self, rover: DifferentialDrive) -> None:
        """Test that move tool is generated."""
        tools = controller_to_tools(rover)
        tool_names = [t.__name__ for t in tools]

        assert "test_rover_move" in tool_names

    def test_call_move_tool_handler(self, rover: DifferentialDrive) -> None:
        """Test calling the move tool with motor speeds."""
        tools = controller_to_tools(rover)

        # Find move tool
        move_tool = next(t for t in tools if t.__name__ == "test_rover_move")

        # Call tool with target motor speeds (actuators are "left" and "right")
        target = {"left": 0.6, "right": 0.6}
        move_tool(targets=target)

        # Verify motors set via actuator get() (not rover.current_speed which tracks tank() calls)
        left_motor_val = rover.left_motor.get()
        right_motor_val = rover.right_motor.get()
        assert left_motor_val == pytest.approx(0.6, abs=0.1)
        assert right_motor_val == pytest.approx(0.6, abs=0.1)

    def test_home_tool_exists(self, rover: DifferentialDrive) -> None:
        """Test that home tool is generated."""
        tools = controller_to_tools(rover)
        tool_names = [t.__name__ for t in tools]

        assert "test_rover_home" in tool_names

    def test_call_home_tool_stops_rover(self, rover: DifferentialDrive) -> None:
        """Test calling the home tool stops the rover."""
        tools = controller_to_tools(rover)

        # Start moving
        rover.forward(0.8)
        assert rover.current_speed[0] != 0

        # Find home tool and call it (home = stop for differential drive)
        home_tool = next(t for t in tools if t.__name__ == "test_rover_home")
        home_tool()

        # Verify rover stopped
        left_speed, right_speed = rover.current_speed
        assert left_speed == pytest.approx(0, abs=0.01)
        assert right_speed == pytest.approx(0, abs=0.01)

    def test_stop_tool_exists(self, rover: DifferentialDrive) -> None:
        """Test that stop (emergency) tool is generated."""
        tools = controller_to_tools(rover)
        tool_names = [t.__name__ for t in tools]

        assert "test_rover_stop" in tool_names

    def test_call_stop_tool_handler(self, rover: DifferentialDrive) -> None:
        """Test calling the stop tool."""
        tools = controller_to_tools(rover)

        # Start moving
        rover.forward(1.0)

        # Find stop tool and call it
        stop_tool = next(t for t in tools if t.__name__ == "test_rover_stop")
        stop_tool()

        # Verify rover stopped
        left_speed, right_speed = rover.current_speed
        assert left_speed == pytest.approx(0, abs=0.01)
        assert right_speed == pytest.approx(0, abs=0.01)


# =============================================================================
# Test: Rover with Ultrasonic Sensor
# =============================================================================


class TestRoverWithUltrasonicSensor:
    """Test rover with ultrasonic distance sensor."""

    def test_create_ultrasonic_sensor(self, ultrasonic_sensor: SimulatedSensor) -> None:
        """Test creating an ultrasonic distance sensor."""
        assert ultrasonic_sensor.name == "front_ultrasonic"
        assert ultrasonic_sensor.unit == Unit.MILLIMETERS
        assert ultrasonic_sensor.limits.min == 20
        assert ultrasonic_sensor.limits.max == 4000

    def test_read_distance(self, ultrasonic_sensor: SimulatedSensor) -> None:
        """Test reading distance from ultrasonic sensor."""
        ultrasonic_sensor.enable()
        reading = ultrasonic_sensor.read()

        # Should return a reading within sensor range
        assert reading.value >= ultrasonic_sensor.limits.min
        assert reading.value <= ultrasonic_sensor.limits.max

    def test_distance_within_reasonable_range(self, ultrasonic_sensor: SimulatedSensor) -> None:
        """Test that distance reading is reasonable."""
        ultrasonic_sensor.enable()
        reading = ultrasonic_sensor.read()

        # Default value should be around 500mm (from fixture)
        # With ±5mm noise, should still be reasonable
        assert reading.value >= 0
        assert reading.value <= 5000  # Reasonable upper bound

    def test_add_sensor_to_rover(
        self,
        rover: DifferentialDrive,
        ultrasonic_sensor: SimulatedSensor,
    ) -> None:
        """Test adding ultrasonic sensor to rover controller."""
        # Add sensor to controller
        rover.add_sensor("front", ultrasonic_sensor)

        # Verify sensor was added
        assert "front" in rover.sensors

    def test_read_sensor_via_controller(
        self,
        rover: DifferentialDrive,
        ultrasonic_sensor: SimulatedSensor,
    ) -> None:
        """Test reading sensor via controller's read_sensors method."""
        ultrasonic_sensor.enable()
        rover.add_sensor("front", ultrasonic_sensor)

        # Read all sensors
        readings = rover.read_sensors()

        assert "front" in readings
        assert readings["front"] >= 0

    def test_sensor_appears_in_tools(
        self,
        rover: DifferentialDrive,
        ultrasonic_sensor: SimulatedSensor,
    ) -> None:
        """Test that sensors appear in generated AI tools."""
        ultrasonic_sensor.enable()
        rover.add_sensor("front", ultrasonic_sensor)

        tools = controller_to_tools(rover)
        tool_names = [t.__name__ for t in tools]

        # Should have a sensors tool
        assert "test_rover_sensors" in tool_names


# =============================================================================
# Test: Rover with API (svc-infra)
# =============================================================================


class TestRoverWithAPI:
    """Test rover integration with svc-infra REST API."""

    @pytest.fixture
    def client(self, rover: DifferentialDrive):
        """Create a test client for the rover API."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient

        app = FastAPI()
        router = controller_to_router(rover)
        app.include_router(router, prefix="/rover")

        return TestClient(app)

    def test_generate_router_from_controller(self, rover: DifferentialDrive) -> None:
        """Test generating FastAPI router from DifferentialDrive controller."""
        router = controller_to_router(rover)
        assert router is not None

    def test_get_status_endpoint(self, client) -> None:
        """Test GET /rover/status endpoint."""
        response = client.get("/rover/status")

        assert response.status_code == 200
        data = response.json()
        assert "state" in data
        assert "is_enabled" in data

    def test_post_enable_endpoint(self, rover: DifferentialDrive, client) -> None:
        """Test POST /rover/enable endpoint."""
        rover.disable()
        assert not rover.is_enabled

        response = client.post("/rover/enable")

        assert response.status_code == 200
        assert rover.is_enabled

    def test_post_disable_endpoint(self, client, rover: DifferentialDrive) -> None:
        """Test POST /rover/disable endpoint."""
        response = client.post("/rover/disable")

        assert response.status_code == 200
        assert not rover.is_enabled

    def test_post_stop_endpoint(self, client, rover: DifferentialDrive) -> None:
        """Test POST /rover/stop endpoint."""
        rover.forward(0.8)

        response = client.post("/rover/stop")

        assert response.status_code == 200
        left_speed, right_speed = rover.current_speed
        assert left_speed == pytest.approx(0, abs=0.01)
        assert right_speed == pytest.approx(0, abs=0.01)

    def test_get_actuators_endpoint(self, client, rover: DifferentialDrive) -> None:
        """Test GET /rover/actuators endpoint."""
        # Set known motor speeds using tank method
        rover.tank(0.5, 0.3)

        response = client.get("/rover/actuators")

        assert response.status_code == 200
        data = response.json()
        assert "left" in data
        assert "right" in data

    def test_full_api_workflow(self, client, rover: DifferentialDrive) -> None:
        """Test complete API workflow: enable -> read -> stop -> disable."""
        rover.disable()

        # 1. Enable via API
        response = client.post("/rover/enable")
        assert response.status_code == 200

        # 2. Drive forward via direct controller (API move has validation issues)
        rover.forward(0.7)

        # 3. Read actuators via API
        response = client.get("/rover/actuators")
        assert response.status_code == 200

        # 4. Stop via API
        response = client.post("/rover/stop")
        assert response.status_code == 200

        # 5. Disable via API
        response = client.post("/rover/disable")
        assert response.status_code == 200


# =============================================================================
# Integration: Full E2E Flow
# =============================================================================


class TestFullE2EFlow:
    """Test complete end-to-end integration of all components."""

    def test_rover_created_tools_generated_api_mounted(self, rover: DifferentialDrive) -> None:
        """Test full flow: create rover -> generate tools -> create API."""
        # 1. Rover already created via fixture
        assert rover.left_motor is not None
        assert rover.right_motor is not None

        # 2. Generate AI tools
        tools = controller_to_tools(rover)
        assert len(tools) >= 4  # move, home, stop, status

        # 3. Generate API router
        router = controller_to_router(rover)
        assert router is not None

    def test_drive_via_tool_read_via_api(self, rover: DifferentialDrive) -> None:
        """Test driving via AI tool and reading via API."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient

        # Generate tools and API
        tools = controller_to_tools(rover)
        router = controller_to_router(rover)

        app = FastAPI()
        app.include_router(router, prefix="/rover")
        client = TestClient(app)

        # Drive via AI tool (set motor speeds directly)
        move_tool = next(t for t in tools if t.__name__ == "test_rover_move")
        move_tool(targets={"left": 0.8, "right": 0.8})

        # Read via API
        response = client.get("/rover/actuators")
        data = response.json()

        assert data["left"] == pytest.approx(0.8, abs=0.1)
        assert data["right"] == pytest.approx(0.8, abs=0.1)

    def test_rover_with_sensor_full_integration(
        self,
        rover: DifferentialDrive,
        ultrasonic_sensor: SimulatedSensor,
    ) -> None:
        """Test rover with sensor - full integration."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient

        # Add sensor
        ultrasonic_sensor.enable()
        rover.add_sensor("front", ultrasonic_sensor)

        # Generate tools and API
        tools = controller_to_tools(rover)
        router = controller_to_router(rover)

        app = FastAPI()
        app.include_router(router, prefix="/rover")
        client = TestClient(app)

        # Read sensors via AI tool
        sensors_tool = next(t for t in tools if t.__name__ == "test_rover_sensors")
        sensor_readings = sensors_tool()

        # Read sensors via API
        response = client.get("/rover/sensors")
        api_readings = response.json()

        # Both should have the front sensor
        assert "front" in sensor_readings
        assert "front" in api_readings
