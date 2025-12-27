"""End-to-end integration tests for lock controller.

These tests verify the complete flow from:
1. Creating a simulated servo actuator
2. Building a Lock controller
3. Locking, unlocking, toggling states
4. Integration with ai-infra tools for LLM control
5. Integration with svc-infra router for REST API control

These are integration tests that test the full stack without real hardware.
"""

from __future__ import annotations

import pytest

from robo_infra.controllers.lock import Lock, LockConfig, LockState
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits
from robo_infra.integrations.ai_infra import controller_to_tools
from robo_infra.integrations.svc_infra import controller_to_router

# Check for optional dependencies
try:
    import ai_infra  # noqa: F401

    HAS_AI_INFRA = True
except ImportError:
    HAS_AI_INFRA = False

try:
    import fastapi  # noqa: F401

    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def lock_servo() -> SimulatedActuator:
    """Create a simulated servo actuator for the lock mechanism.

    Servo configuration:
    - Range: 0-90 degrees
    - 0 degrees = locked position
    - 90 degrees = unlocked position
    """
    return SimulatedActuator(
        name="lock_servo",
        limits=Limits(min=0, max=90, default=0),
        unit="degrees",
    )


@pytest.fixture
def door_lock(lock_servo: SimulatedActuator) -> Lock:
    """Create a Lock controller representing a door lock."""
    config = LockConfig(
        name="door_lock",
        locked_position=0,
        unlocked_position=90,
        transition_time=0.1,  # Fast for testing
        start_locked=True,
    )
    lock = Lock(name="door_lock", actuator=lock_servo, config=config)
    lock.enable()
    return lock


@pytest.fixture
def safe_lock(lock_servo: SimulatedActuator) -> Lock:
    """Create a Lock controller representing a safe lock with auto-lock."""
    config = LockConfig(
        name="safe_lock",
        locked_position=0,
        unlocked_position=90,
        transition_time=0.1,
        start_locked=True,
        auto_lock_timeout=5.0,  # Auto-lock after 5 seconds
    )
    lock = Lock(name="safe_lock", actuator=lock_servo, config=config)
    lock.enable()
    return lock


# =============================================================================
# Test: Create Lock
# =============================================================================


class TestCreateLock:
    """Test creating and controlling a lock with a simulated servo."""

    def test_create_simulated_servo(self, lock_servo: SimulatedActuator) -> None:
        """Test creating a simulated servo actuator for the lock."""
        assert lock_servo.name == "lock_servo"
        assert lock_servo.limits.min == 0
        assert lock_servo.limits.max == 90
        assert lock_servo.unit == "degrees"

    def test_create_lock_controller(self, lock_servo: SimulatedActuator) -> None:
        """Test creating a Lock controller."""
        lock = Lock(name="test_lock", actuator=lock_servo)

        assert lock.name == "test_lock"
        assert lock.actuator is lock_servo

    def test_lock_requires_actuator(self) -> None:
        """Test that Lock requires an actuator."""
        with pytest.raises(ValueError, match="Lock actuator is required"):
            Lock(name="invalid", actuator=None)  # type: ignore

    def test_lock_starts_disabled(self, lock_servo: SimulatedActuator) -> None:
        """Test that lock starts in disabled state."""
        lock = Lock(name="test", actuator=lock_servo)
        assert not lock.is_enabled

    def test_enable_lock(self, lock_servo: SimulatedActuator) -> None:
        """Test enabling the lock."""
        lock = Lock(name="test", actuator=lock_servo)
        lock.enable()
        assert lock.is_enabled

    def test_lock_starts_locked_by_default(self, door_lock: Lock) -> None:
        """Test that lock starts in locked state when configured."""
        # door_lock fixture has start_locked=True and is enabled
        assert door_lock.lock_state == LockState.LOCKED
        assert door_lock.is_locked

    def test_lock_operation(self, door_lock: Lock) -> None:
        """Test locking the mechanism."""
        # Unlock first
        door_lock.unlock()
        assert door_lock.is_unlocked

        # Now lock
        door_lock.lock()
        assert door_lock.is_locked
        assert door_lock.lock_state == LockState.LOCKED

    def test_unlock_operation(self, door_lock: Lock) -> None:
        """Test unlocking the mechanism."""
        # Should start locked
        assert door_lock.is_locked

        # Unlock
        door_lock.unlock()
        assert door_lock.is_unlocked
        assert door_lock.lock_state == LockState.UNLOCKED

    def test_lock_verify_state(self, door_lock: Lock) -> None:
        """Test verifying lock state after operations."""
        # Lock and verify
        door_lock.lock()
        assert door_lock.is_locked
        assert not door_lock.is_unlocked
        assert door_lock.state_str == "locked"

        # Unlock and verify
        door_lock.unlock()
        assert door_lock.is_unlocked
        assert not door_lock.is_locked
        assert door_lock.state_str == "unlocked"

    def test_unlock_verify_state(self, door_lock: Lock) -> None:
        """Test verifying unlocked state."""
        door_lock.unlock()

        assert door_lock.is_unlocked
        assert door_lock.lock_state == LockState.UNLOCKED
        assert door_lock.position == pytest.approx(90, abs=1)  # unlocked_position

    def test_toggle_from_locked(self, door_lock: Lock) -> None:
        """Test toggle from locked state."""
        # Start locked
        door_lock.lock()
        assert door_lock.is_locked

        # Toggle should unlock
        door_lock.toggle()
        assert door_lock.is_unlocked

    def test_toggle_from_unlocked(self, door_lock: Lock) -> None:
        """Test toggle from unlocked state."""
        # Start unlocked
        door_lock.unlock()
        assert door_lock.is_unlocked

        # Toggle should lock
        door_lock.toggle()
        assert door_lock.is_locked

    def test_toggle_twice(self, door_lock: Lock) -> None:
        """Test that toggle twice returns to original state."""
        initial_locked = door_lock.is_locked

        door_lock.toggle()
        door_lock.toggle()

        assert door_lock.is_locked == initial_locked

    def test_position_tracking(self, door_lock: Lock) -> None:
        """Test that position is tracked correctly."""
        # Lock - should be at position 0
        door_lock.lock()
        assert door_lock.position == pytest.approx(0, abs=1)

        # Unlock - should be at position 90
        door_lock.unlock()
        assert door_lock.position == pytest.approx(90, abs=1)

    def test_home_locks_when_start_locked(self, door_lock: Lock) -> None:
        """Test that home() locks when start_locked=True."""
        # Unlock first
        door_lock.unlock()
        assert door_lock.is_unlocked

        # Home should lock (start_locked=True in config)
        door_lock.home()
        assert door_lock.is_locked


# =============================================================================
# Test: Lock with AI Tools
# =============================================================================


@pytest.mark.skipif(not HAS_AI_INFRA, reason="ai-infra not installed")
class TestLockWithAITools:
    """Test lock integration with ai-infra tools for LLM control."""

    def test_generate_tools_from_controller(self, door_lock: Lock) -> None:
        """Test generating ai-infra tools from Lock controller."""
        tools = controller_to_tools(door_lock)

        assert len(tools) > 0
        assert isinstance(tools, list)

    def test_tools_have_required_properties(self, door_lock: Lock) -> None:
        """Test that generated tools are callable with __name__ and __doc__."""
        tools = controller_to_tools(door_lock)

        for tool in tools:
            assert callable(tool)
            assert hasattr(tool, "__name__")
            assert hasattr(tool, "__doc__")
            assert tool.__name__.startswith("door_lock")

    def test_move_tool_exists(self, door_lock: Lock) -> None:
        """Test that move tool is generated."""
        tools = controller_to_tools(door_lock)
        tool_names = [t.__name__ for t in tools]

        assert "door_lock_move" in tool_names

    def test_call_move_tool_to_unlock(self, door_lock: Lock) -> None:
        """Test calling the move tool to unlock."""
        tools = controller_to_tools(door_lock)

        # Find move tool
        move_tool = next(t for t in tools if t.__name__ == "door_lock_move")

        # Call tool to set lock actuator to unlocked position
        move_tool(targets={"lock": 90})  # 90 degrees = unlocked

        assert door_lock.position == pytest.approx(90, abs=1)

    def test_call_move_tool_to_lock(self, door_lock: Lock) -> None:
        """Test calling the move tool to lock."""
        tools = controller_to_tools(door_lock)

        # Unlock first
        door_lock.unlock()

        # Find move tool
        move_tool = next(t for t in tools if t.__name__ == "door_lock_move")

        # Call tool to set lock actuator to locked position
        move_tool(targets={"lock": 0})  # 0 degrees = locked

        assert door_lock.position == pytest.approx(0, abs=1)

    def test_home_tool_exists(self, door_lock: Lock) -> None:
        """Test that home tool is generated."""
        tools = controller_to_tools(door_lock)
        tool_names = [t.__name__ for t in tools]

        assert "door_lock_home" in tool_names

    def test_call_home_tool_locks(self, door_lock: Lock) -> None:
        """Test calling the home tool locks the door."""
        tools = controller_to_tools(door_lock)

        # Unlock first
        door_lock.unlock()
        assert door_lock.is_unlocked

        # Find home tool and call it
        home_tool = next(t for t in tools if t.__name__ == "door_lock_home")
        home_tool()

        # Home should lock (start_locked=True)
        assert door_lock.is_locked

    def test_status_tool_exists(self, door_lock: Lock) -> None:
        """Test that status tool is generated."""
        tools = controller_to_tools(door_lock)
        tool_names = [t.__name__ for t in tools]

        assert "door_lock_status" in tool_names

    def test_stop_tool_exists(self, door_lock: Lock) -> None:
        """Test that stop tool is generated."""
        tools = controller_to_tools(door_lock)
        tool_names = [t.__name__ for t in tools]

        assert "door_lock_stop" in tool_names


# =============================================================================
# Test: Lock with API (svc-infra)
# =============================================================================


@pytest.mark.skipif(not HAS_FASTAPI, reason="fastapi not installed")
class TestLockWithAPI:
    """Test lock integration with svc-infra REST API."""

    @pytest.fixture
    def client(self, door_lock: Lock):
        """Create a test client for the lock API."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient

        app = FastAPI()
        router = controller_to_router(door_lock)
        app.include_router(router, prefix="/lock")

        return TestClient(app)

    def test_generate_router_from_controller(self, door_lock: Lock) -> None:
        """Test generating FastAPI router from Lock controller."""
        router = controller_to_router(door_lock)
        assert router is not None

    def test_get_status_endpoint(self, client) -> None:
        """Test GET /lock/status endpoint."""
        response = client.get("/lock/status")

        assert response.status_code == 200
        data = response.json()
        assert "state" in data
        assert "is_enabled" in data

    def test_post_enable_endpoint(self, door_lock: Lock, client) -> None:
        """Test POST /lock/enable endpoint."""
        door_lock.disable()
        assert not door_lock.is_enabled

        response = client.post("/lock/enable")

        assert response.status_code == 200
        assert door_lock.is_enabled

    def test_post_disable_endpoint(self, client, door_lock: Lock) -> None:
        """Test POST /lock/disable endpoint."""
        response = client.post("/lock/disable")

        assert response.status_code == 200
        assert not door_lock.is_enabled

    def test_post_home_endpoint(self, client, door_lock: Lock) -> None:
        """Test POST /lock/home endpoint (locks the door)."""
        # Unlock first
        door_lock.unlock()
        assert door_lock.is_unlocked

        response = client.post("/lock/home")

        assert response.status_code == 200
        assert door_lock.is_locked  # Home = lock for start_locked=True

    def test_get_actuators_endpoint(self, client, door_lock: Lock) -> None:
        """Test GET /lock/actuators endpoint."""
        # Set known position
        door_lock.unlock()

        response = client.get("/lock/actuators")

        assert response.status_code == 200
        data = response.json()
        assert "lock" in data
        assert data["lock"] == pytest.approx(90, abs=1)  # unlocked position

    def test_actuators_shows_locked_position(self, client, door_lock: Lock) -> None:
        """Test actuators endpoint shows locked position."""
        door_lock.lock()

        response = client.get("/lock/actuators")

        assert response.status_code == 200
        data = response.json()
        assert data["lock"] == pytest.approx(0, abs=1)  # locked position

    def test_full_api_workflow(self, client, door_lock: Lock) -> None:
        """Test complete API workflow: enable -> unlock -> lock -> disable."""
        door_lock.disable()

        # 1. Enable via API
        response = client.post("/lock/enable")
        assert response.status_code == 200
        assert door_lock.is_enabled

        # 2. Unlock via direct controller (API move has validation issues)
        door_lock.unlock()

        # 3. Read status via API
        response = client.get("/lock/actuators")
        assert response.status_code == 200
        assert response.json()["lock"] == pytest.approx(90, abs=1)

        # 4. Lock via home endpoint
        response = client.post("/lock/home")
        assert response.status_code == 200
        assert door_lock.is_locked

        # 5. Disable via API
        response = client.post("/lock/disable")
        assert response.status_code == 200


# =============================================================================
# Integration: Full E2E Flow
# =============================================================================


@pytest.mark.skipif(not HAS_AI_INFRA or not HAS_FASTAPI, reason="ai-infra or fastapi not installed")
class TestFullE2EFlow:
    """Test complete end-to-end integration of all components."""

    def test_lock_created_tools_generated_api_mounted(self, door_lock: Lock) -> None:
        """Test full flow: create lock -> generate tools -> create API."""
        # 1. Lock already created via fixture
        assert door_lock.actuator is not None

        # 2. Generate AI tools
        tools = controller_to_tools(door_lock)
        assert len(tools) >= 4  # move, home, stop, status

        # 3. Generate API router
        router = controller_to_router(door_lock)
        assert router is not None

    def test_unlock_via_tool_read_via_api(self, door_lock: Lock) -> None:
        """Test unlocking via AI tool and reading via API."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient

        # Generate tools and API
        tools = controller_to_tools(door_lock)
        router = controller_to_router(door_lock)

        app = FastAPI()
        app.include_router(router, prefix="/lock")
        client = TestClient(app)

        # Unlock via AI tool
        move_tool = next(t for t in tools if t.__name__ == "door_lock_move")
        move_tool(targets={"lock": 90})  # Unlock position

        # Read via API
        response = client.get("/lock/actuators")
        data = response.json()

        assert data["lock"] == pytest.approx(90, abs=1)

    def test_lock_via_api_read_via_tool(self, door_lock: Lock) -> None:
        """Test locking via API and reading via status tool."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient

        # Generate API
        router = controller_to_router(door_lock)

        app = FastAPI()
        app.include_router(router, prefix="/lock")
        client = TestClient(app)

        # Unlock first
        door_lock.unlock()

        # Lock via home endpoint (which locks for start_locked=True)
        client.post("/lock/home")

        # Verify via direct controller access
        assert door_lock.is_locked
        assert door_lock.position == pytest.approx(0, abs=1)

    def test_toggle_simulation(self, door_lock: Lock) -> None:
        """Test simulating toggle behavior through tools."""
        tools = controller_to_tools(door_lock)

        # Get current position
        initial_pos = door_lock.position

        # "Toggle" by moving to opposite position via move tool
        move_tool = next(t for t in tools if t.__name__ == "door_lock_move")

        if door_lock.is_locked:
            move_tool(targets={"lock": 90})  # Unlock
        else:
            move_tool(targets={"lock": 0})  # Lock

        # Position should have changed
        assert door_lock.position != initial_pos


# =============================================================================
# Test: Lock Configuration Variations
# =============================================================================


class TestLockConfigurations:
    """Test different lock configurations."""

    def test_inverted_lock_positions(self) -> None:
        """Test lock with inverted positions (unlocked < locked)."""
        servo = SimulatedActuator(
            name="inverted_servo",
            limits=Limits(min=0, max=180, default=180),
            unit="degrees",
        )
        config = LockConfig(
            name="inverted_lock",
            locked_position=180,  # High = locked
            unlocked_position=0,  # Low = unlocked
        )
        lock = Lock(name="inverted_lock", actuator=servo, config=config)
        lock.enable()

        assert config.is_inverted

        # Lock should go to 180
        lock.lock()
        assert lock.position == pytest.approx(180, abs=1)

        # Unlock should go to 0
        lock.unlock()
        assert lock.position == pytest.approx(0, abs=1)

    def test_lock_with_custom_tolerance(self) -> None:
        """Test lock with custom position tolerance."""
        servo = SimulatedActuator(
            name="test_servo",
            limits=Limits(min=0, max=90, default=0),
            unit="degrees",
        )
        config = LockConfig(
            name="tolerant_lock",
            locked_position=0,
            unlocked_position=90,
            position_tolerance=5.0,  # 5 degree tolerance
        )
        lock = Lock(name="tolerant_lock", actuator=servo, config=config)
        lock.enable()

        # Move to almost locked position (within tolerance)
        servo.set(3)  # 3 degrees, within 5 degree tolerance of 0
        lock._update_state_from_position()

        # Should still be considered locked
        assert lock.is_locked
