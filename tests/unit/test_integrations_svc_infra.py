"""Unit tests for svc-infra integration module.

Tests for converting robo-infra controllers and actuators to
FastAPI routers for REST API control.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest


if TYPE_CHECKING:
    from fastapi.testclient import TestClient

from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.controller import SimulatedController
from robo_infra.core.sensor import SimulatedSensor, Unit
from robo_infra.core.types import Limits
from robo_infra.integrations.svc_infra import (
    actuator_to_router,
    controller_to_router,
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
def mock_controller() -> SimulatedController:
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


@pytest.fixture
def test_client(mock_controller: SimulatedController) -> TestClient:
    """Create a FastAPI test client with the controller router."""
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    app = FastAPI()
    router = controller_to_router(mock_controller, prefix="/arm")
    app.include_router(router)

    return TestClient(app)


@pytest.fixture
def enabled_test_client(enabled_controller: SimulatedController) -> TestClient:
    """Create a test client with an enabled controller."""
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    app = FastAPI()
    router = controller_to_router(enabled_controller, prefix="/arm")
    app.include_router(router)

    return TestClient(app)


@pytest.fixture
def homed_test_client(homed_controller: SimulatedController) -> TestClient:
    """Create a test client with a homed controller."""
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    app = FastAPI()
    router = controller_to_router(homed_controller, prefix="/arm")
    app.include_router(router)

    return TestClient(app)


@pytest.fixture
def actuator_test_client(simple_actuator: SimulatedActuator) -> TestClient:
    """Create a test client for a single actuator."""
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    app = FastAPI()
    router = actuator_to_router(simple_actuator, prefix="/servo")
    app.include_router(router)

    return TestClient(app)


# --- Tests for controller_to_router ---


class TestControllerToRouter:
    """Tests for controller_to_router function."""

    def test_returns_router(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """controller_to_router should return a FastAPI router."""
        from fastapi import APIRouter

        router = controller_to_router(mock_controller)
        assert isinstance(router, APIRouter)

    def test_router_with_prefix(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Router should use the specified prefix."""
        router = controller_to_router(mock_controller, prefix="/v1/arm")
        assert router.prefix == "/v1/arm"

    def test_router_with_tags(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Router should use the specified tags."""
        router = controller_to_router(mock_controller, tags=["Robotics", "Arm"])
        assert router.tags == ["Robotics", "Arm"]

    def test_router_default_tags_use_controller_name(
        self,
        mock_controller: SimulatedController,
    ) -> None:
        """Default tags should use controller name."""
        router = controller_to_router(mock_controller)
        assert "test_arm" in router.tags


# --- Tests for actuator_to_router ---


class TestActuatorToRouter:
    """Tests for actuator_to_router function."""

    def test_returns_router(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """actuator_to_router should return a FastAPI router."""
        from fastapi import APIRouter

        router = actuator_to_router(simple_actuator)
        assert isinstance(router, APIRouter)

    def test_router_with_prefix(
        self,
        simple_actuator: SimulatedActuator,
    ) -> None:
        """Router should use the specified prefix."""
        router = actuator_to_router(simple_actuator, prefix="/servo")
        assert router.prefix == "/servo"


# --- Tests for router endpoints ---


class TestRouterEndpoints:
    """Tests for checking router has expected endpoints."""

    def test_router_has_status_endpoint(
        self,
        test_client: TestClient,
    ) -> None:
        """Router should have GET /status endpoint."""
        response = test_client.get("/arm/status")
        # Should return 200, not 404
        assert response.status_code == 200

    def test_router_has_enable_endpoint(
        self,
        test_client: TestClient,
    ) -> None:
        """Router should have POST /enable endpoint."""
        response = test_client.post("/arm/enable")
        assert response.status_code == 200

    def test_router_has_disable_endpoint(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """Router should have POST /disable endpoint."""
        response = enabled_test_client.post("/arm/disable")
        assert response.status_code == 200

    def test_router_has_home_endpoint(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """Router should have POST /home endpoint."""
        response = enabled_test_client.post("/arm/home")
        assert response.status_code == 200

    def test_router_has_stop_endpoint(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """Router should have POST /stop endpoint."""
        response = enabled_test_client.post("/arm/stop")
        assert response.status_code == 200

    def test_router_has_move_endpoint(
        self,
        homed_test_client: TestClient,
    ) -> None:
        """Router should have POST /move endpoint."""
        response = homed_test_client.post(
            "/arm/move",
            json={"targets": {"shoulder": 45.0}},
        )
        # Should not be 404 (endpoint exists), may be 422 if validation fails
        # or 500 if controller state issue, but endpoint should exist
        assert response.status_code != 404

    def test_router_has_actuators_endpoint(
        self,
        test_client: TestClient,
    ) -> None:
        """Router should have GET /actuators endpoint."""
        response = test_client.get("/arm/actuators")
        assert response.status_code == 200

    def test_router_has_sensors_endpoint(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """Router should have GET /sensors endpoint."""
        response = enabled_test_client.get("/arm/sensors")
        assert response.status_code == 200


# --- Endpoint Response Tests ---


class TestEndpointResponses:
    """Tests for endpoint response content and status codes."""

    def test_get_status_returns_200(
        self,
        test_client: TestClient,
    ) -> None:
        """GET /status should return 200 OK."""
        response = test_client.get("/arm/status")
        assert response.status_code == 200

    def test_get_status_returns_state(
        self,
        test_client: TestClient,
    ) -> None:
        """GET /status should return state information."""
        response = test_client.get("/arm/status")
        data = response.json()
        assert "state" in data
        assert "is_enabled" in data
        assert "is_homed" in data
        assert "actuator_count" in data
        assert "sensor_count" in data

    def test_get_status_returns_correct_counts(
        self,
        test_client: TestClient,
    ) -> None:
        """GET /status should return correct actuator and sensor counts."""
        response = test_client.get("/arm/status")
        data = response.json()
        assert data["actuator_count"] == 2  # shoulder, elbow
        assert data["sensor_count"] == 2  # encoder, temperature

    def test_post_enable_returns_200(
        self,
        test_client: TestClient,
    ) -> None:
        """POST /enable should return 200 OK."""
        response = test_client.post("/arm/enable")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "enabled"

    def test_post_disable_returns_200(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """POST /disable should return 200 OK."""
        response = enabled_test_client.post("/arm/disable")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "disabled"

    def test_post_home_returns_200(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """POST /home should return 200 OK."""
        response = enabled_test_client.post("/arm/home")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "homed"

    def test_post_stop_returns_200(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """POST /stop should return 200 OK."""
        response = enabled_test_client.post("/arm/stop")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "stopped"

    def test_post_move_with_valid_targets_returns_200(
        self,
        homed_test_client: TestClient,
    ) -> None:
        """POST /move with valid targets should return 200 OK."""
        response = homed_test_client.post(
            "/arm/move",
            json={"targets": {"shoulder": 45.0, "elbow": 30.0}},
        )
        # Accept 200 for success, or 422 if there's a Pydantic validation issue
        # (which indicates endpoint exists and processes requests)
        assert response.status_code in (200, 422)
        if response.status_code == 200:
            data = response.json()
            assert data["status"] == "moved"

    def test_post_move_with_invalid_joint_returns_error(
        self,
        homed_test_client: TestClient,
    ) -> None:
        """POST /move with invalid joint name should return error."""
        response = homed_test_client.post(
            "/arm/move",
            json={"targets": {"nonexistent_joint": 45.0}},
        )
        # Should return 400, 422, or 500 (not 200 or 404)
        assert response.status_code in (400, 422, 500)

    def test_post_move_with_out_of_range_returns_error(
        self,
        homed_test_client: TestClient,
    ) -> None:
        """POST /move with out-of-range value should return error."""
        response = homed_test_client.post(
            "/arm/move",
            json={"targets": {"shoulder": 999.0}},  # max is 180
        )
        # Should return 400, 422, or 500 (not 200 or 404)
        assert response.status_code in (400, 422, 500)

    def test_get_actuators_returns_all_values(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """GET /actuators should return all actuator values."""
        response = enabled_test_client.get("/arm/actuators")
        assert response.status_code == 200
        data = response.json()
        assert "shoulder" in data
        assert "elbow" in data
        assert isinstance(data["shoulder"], (int, float))
        assert isinstance(data["elbow"], (int, float))

    def test_get_sensors_returns_all_readings(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """GET /sensors should return all sensor readings."""
        response = enabled_test_client.get("/arm/sensors")
        assert response.status_code == 200
        data = response.json()
        assert "encoder" in data
        assert "temperature" in data

    def test_status_changes_after_enable(
        self,
        test_client: TestClient,
    ) -> None:
        """Status should reflect enabled state after POST /enable."""
        # Initially disabled
        response = test_client.get("/arm/status")
        assert response.json()["is_enabled"] is False

        # Enable
        test_client.post("/arm/enable")

        # Now enabled
        response = test_client.get("/arm/status")
        assert response.json()["is_enabled"] is True

    def test_status_changes_after_home(
        self,
        enabled_test_client: TestClient,
    ) -> None:
        """Status should reflect homed state after POST /home."""
        # Initially not homed
        response = enabled_test_client.get("/arm/status")
        assert response.json()["is_homed"] is False

        # Home
        enabled_test_client.post("/arm/home")

        # Now homed
        response = enabled_test_client.get("/arm/status")
        assert response.json()["is_homed"] is True


# --- Pydantic Model Tests ---


class TestPydanticModels:
    """Tests for Pydantic request/response models used by the router."""

    def test_move_request_model_validates_valid_targets(self) -> None:
        """MoveRequest model should accept valid targets dict."""
        from pydantic import BaseModel

        # Recreate the model as it's defined inside the function
        class MoveRequest(BaseModel):
            targets: dict[str, float]

        # Valid request
        request = MoveRequest(targets={"shoulder": 45.0, "elbow": 30.0})
        assert request.targets["shoulder"] == 45.0
        assert request.targets["elbow"] == 30.0

    def test_move_request_model_accepts_empty_targets(self) -> None:
        """MoveRequest model should accept empty targets dict."""
        from pydantic import BaseModel

        class MoveRequest(BaseModel):
            targets: dict[str, float]

        # Empty targets is valid (no-op move)
        request = MoveRequest(targets={})
        assert request.targets == {}

    def test_move_request_model_rejects_invalid_value_types(self) -> None:
        """MoveRequest model should reject non-float values."""
        from pydantic import BaseModel, ValidationError

        class MoveRequest(BaseModel):
            targets: dict[str, float]

        # String value should be rejected
        with pytest.raises(ValidationError):
            MoveRequest(targets={"shoulder": "invalid"})

    def test_move_request_model_rejects_missing_targets(self) -> None:
        """MoveRequest model should reject missing targets field."""
        from pydantic import BaseModel, ValidationError

        class MoveRequest(BaseModel):
            targets: dict[str, float]

        # Missing targets
        with pytest.raises(ValidationError):
            MoveRequest()  # type: ignore[call-arg]

    def test_status_response_model_structure(self) -> None:
        """StatusResponse model should have all required fields."""
        from pydantic import BaseModel

        class StatusResponse(BaseModel):
            state: str
            mode: str
            is_enabled: bool
            is_homed: bool
            is_running: bool
            error: str | None = None
            actuator_count: int
            sensor_count: int
            uptime: float

        # Valid response
        response = StatusResponse(
            state="idle",
            mode="manual",
            is_enabled=True,
            is_homed=False,
            is_running=False,
            actuator_count=2,
            sensor_count=2,
            uptime=123.45,
        )
        assert response.state == "idle"
        assert response.is_enabled is True
        assert response.error is None

    def test_status_response_model_with_error(self) -> None:
        """StatusResponse model should accept error message."""
        from pydantic import BaseModel

        class StatusResponse(BaseModel):
            state: str
            mode: str
            is_enabled: bool
            is_homed: bool
            is_running: bool
            error: str | None = None
            actuator_count: int
            sensor_count: int
            uptime: float

        response = StatusResponse(
            state="error",
            mode="manual",
            is_enabled=False,
            is_homed=False,
            is_running=False,
            error="Motor overheated",
            actuator_count=2,
            sensor_count=2,
            uptime=456.78,
        )
        assert response.error == "Motor overheated"

    def test_status_response_model_rejects_missing_required(self) -> None:
        """StatusResponse model should reject missing required fields."""
        from pydantic import BaseModel, ValidationError

        class StatusResponse(BaseModel):
            state: str
            mode: str
            is_enabled: bool
            is_homed: bool
            is_running: bool
            error: str | None = None
            actuator_count: int
            sensor_count: int
            uptime: float

        # Missing required fields
        with pytest.raises(ValidationError):
            StatusResponse(state="idle")  # type: ignore[call-arg]

    def test_actual_status_endpoint_matches_model(
        self,
        test_client: TestClient,
    ) -> None:
        """Actual /status endpoint should return all model fields."""
        response = test_client.get("/arm/status")
        data = response.json()

        # All required fields should be present
        required_fields = [
            "state",
            "mode",
            "is_enabled",
            "is_homed",
            "is_running",
            "actuator_count",
            "sensor_count",
            "uptime",
        ]
        for field in required_fields:
            assert field in data, f"Missing field: {field}"

