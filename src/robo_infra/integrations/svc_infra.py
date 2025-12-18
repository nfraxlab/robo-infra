"""Integration with svc-infra for REST API robotics control.

This module provides utilities to convert robo-infra controllers and
actuators into FastAPI routers for REST/WebSocket control.

Example:
    >>> from fastapi import FastAPI
    >>> from robo_infra.core.controller import SimulatedController
    >>> from robo_infra.integrations.svc_infra import controller_to_router
    >>>
    >>> app = FastAPI()
    >>> controller = SimulatedController(name="arm")
    >>> router = controller_to_router(controller)
    >>> app.include_router(router, prefix="/v1/arm")

Note:
    This module provides basic router generation. For full svc-infra
    integration with auth, rate limiting, etc., use svc-infra's
    dual_routers and middleware.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any


if TYPE_CHECKING:
    from robo_infra.core.actuator import Actuator
    from robo_infra.core.controller import Controller


logger = logging.getLogger(__name__)


__all__ = [
    "actuator_to_router",
    "controller_to_router",
    "create_websocket_handler",
]


def controller_to_router(
    controller: Controller,
    *,
    prefix: str = "",
    tags: list[str] | None = None,
) -> Any:
    """Convert a controller to a FastAPI router.

    Creates endpoints for:
    - GET /status - Get controller status
    - POST /enable - Enable controller
    - POST /disable - Disable controller
    - POST /home - Home controller
    - POST /stop - Emergency stop
    - POST /move - Move to positions
    - GET /actuators - Get actuator values
    - GET /sensors - Read sensors
    - GET /positions - List named positions
    - POST /positions/{name} - Move to named position

    Args:
        controller: The controller to convert.
        prefix: URL prefix for the router.
        tags: OpenAPI tags for the router.

    Returns:
        FastAPI APIRouter instance.

    Raises:
        ImportError: If FastAPI is not installed.

    Example:
        >>> router = controller_to_router(arm_controller, prefix="/arm")
        >>> app.include_router(router)
    """
    try:
        from fastapi import APIRouter, HTTPException
        from pydantic import BaseModel
    except ImportError as e:
        raise ImportError(
            "FastAPI and Pydantic are required for svc-infra integration. "
            "Install with: pip install fastapi pydantic"
        ) from e

    _tags: list[str] = tags or [controller.name]
    router = APIRouter(prefix=prefix, tags=_tags)  # type: ignore[arg-type]
    name = controller.name

    # Pydantic models for request/response
    class MoveRequest(BaseModel):
        """Request body for move endpoint."""

        targets: dict[str, float]

    class StatusResponse(BaseModel):
        """Response for status endpoint."""

        state: str
        mode: str
        is_enabled: bool
        is_homed: bool
        is_running: bool
        error: str | None = None
        actuator_count: int
        sensor_count: int
        uptime: float

    @router.get("/status", response_model=StatusResponse)
    async def get_status() -> StatusResponse:
        """Get current controller status."""
        status = controller.status()
        return StatusResponse(
            state=status.state.value,
            mode=status.mode.value,
            is_enabled=status.is_enabled,
            is_homed=status.is_homed,
            is_running=status.is_running,
            error=status.error,
            actuator_count=status.actuator_count,
            sensor_count=status.sensor_count,
            uptime=status.uptime,
        )

    @router.post("/enable")
    async def enable() -> dict[str, str]:
        """Enable the controller."""
        try:
            controller.enable()
            return {"status": "enabled"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e)) from e

    @router.post("/disable")
    async def disable() -> dict[str, str]:
        """Disable the controller."""
        controller.disable()
        return {"status": "disabled"}

    @router.post("/home")
    async def home() -> dict[str, str]:
        """Home the controller."""
        try:
            controller.home()
            return {"status": "homed"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e)) from e

    @router.post("/stop")
    async def emergency_stop() -> dict[str, str]:
        """Emergency stop the controller."""
        try:
            controller.stop()
            return {"status": "stopped"}
        except Exception as e:
            # E-stop errors should be logged but still return success
            # (the stop was attempted)
            logger.error("E-stop error on %s: %s", name, e)
            raise HTTPException(
                status_code=500,
                detail=f"E-stop attempted but had errors: {e}",
            ) from e

    @router.post("/move")
    async def move(request: MoveRequest) -> dict[str, Any]:
        """Move actuators to target positions."""
        try:
            controller.move_to(request.targets)
            return {"status": "moved", "targets": request.targets}
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e)) from e
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e)) from e

    @router.get("/actuators")
    async def get_actuators() -> dict[str, float]:
        """Get current actuator values."""
        return controller.get_actuator_values()

    @router.get("/sensors")
    async def read_sensors() -> dict[str, float]:
        """Read all sensors."""
        return controller.read_sensors()

    @router.get("/positions")
    async def list_positions() -> list[str]:
        """List named positions."""
        return list(controller.positions.keys())

    @router.post("/positions/{position_name}")
    async def move_to_position(position_name: str) -> dict[str, str]:
        """Move to a named position."""
        try:
            controller.move_to_position(position_name)
            return {"status": "moved", "position": position_name}
        except KeyError as e:
            raise HTTPException(status_code=404, detail=str(e)) from e
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e)) from e

    @router.post("/positions/{position_name}/save")
    async def save_position(position_name: str) -> dict[str, Any]:
        """Save current position with the given name."""
        position = controller.save_position(position_name)
        return {"status": "saved", "name": position.name, "values": position.values}

    logger.debug("Created router for controller '%s' with prefix '%s'", name, prefix)
    return router


def actuator_to_router(
    actuator: Actuator,
    *,
    prefix: str = "",
    tags: list[str] | None = None,
) -> Any:
    """Convert a single actuator to a FastAPI router.

    Creates endpoints for:
    - GET / - Get current value
    - POST /set - Set value
    - POST /enable - Enable actuator
    - POST /disable - Disable actuator

    Args:
        actuator: The actuator to convert.
        prefix: URL prefix for the router.
        tags: OpenAPI tags for the router.

    Returns:
        FastAPI APIRouter instance.
    """
    try:
        from fastapi import APIRouter, HTTPException
        from pydantic import BaseModel, Field
    except ImportError as e:
        raise ImportError(
            "FastAPI and Pydantic are required for svc-infra integration. "
            "Install with: pip install fastapi pydantic"
        ) from e

    _tags: list[str] = tags or [actuator.name]
    router = APIRouter(prefix=prefix, tags=_tags)  # type: ignore[arg-type]
    limits = actuator.limits

    class SetRequest(BaseModel):
        """Request body for set endpoint."""

        value: float = Field(ge=limits.min, le=limits.max)

    class ActuatorStatus(BaseModel):
        """Actuator status response."""

        name: str
        value: float
        is_enabled: bool
        min: float
        max: float
        default: float | None = None

    @router.get("/", response_model=ActuatorStatus)
    async def get_value() -> ActuatorStatus:
        """Get current actuator value."""
        return ActuatorStatus(
            name=actuator.name,
            value=actuator.get(),
            is_enabled=actuator.is_enabled,
            min=limits.min,
            max=limits.max,
            default=limits.default,
        )

    @router.post("/set")
    async def set_value(request: SetRequest) -> dict[str, Any]:
        """Set actuator value."""
        try:
            actuator.set(request.value)
            return {"status": "set", "value": request.value}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e)) from e

    @router.post("/enable")
    async def enable() -> dict[str, str]:
        """Enable the actuator."""
        actuator.enable()
        return {"status": "enabled"}

    @router.post("/disable")
    async def disable() -> dict[str, str]:
        """Disable the actuator."""
        actuator.disable()
        return {"status": "disabled"}

    return router


def create_websocket_handler(controller: Controller) -> Any:
    """Create a WebSocket handler for real-time controller updates.

    Provides:
    - Real-time actuator value streaming
    - Sensor reading streaming
    - State change notifications
    - Command reception

    Args:
        controller: The controller.

    Returns:
        WebSocket route handler function.

    Example:
        >>> handler = create_websocket_handler(controller)
        >>> app.add_websocket_route("/ws/arm", handler)
    """
    try:
        from fastapi import WebSocket, WebSocketDisconnect
    except ImportError as e:
        raise ImportError(
            "FastAPI is required for WebSocket support. " "Install with: pip install fastapi"
        ) from e

    import asyncio
    import json

    async def websocket_handler(websocket: WebSocket) -> None:
        """Handle WebSocket connection for controller updates."""
        await websocket.accept()
        logger.info("WebSocket connected for controller '%s'", controller.name)

        try:
            # Start background task to send updates
            async def send_updates() -> None:
                while True:
                    data = {
                        "type": "update",
                        "state": controller.status().state.value,
                        "actuators": controller.get_actuator_values(),
                        "sensors": controller.read_sensors(),
                    }
                    await websocket.send_json(data)
                    await asyncio.sleep(0.1)  # 10 Hz updates

            update_task = asyncio.create_task(send_updates())

            try:
                # Handle incoming commands
                while True:
                    message = await websocket.receive_text()
                    try:
                        command = json.loads(message)
                        await _handle_ws_command(controller, command, websocket)
                    except json.JSONDecodeError:
                        await websocket.send_json({"error": "Invalid JSON"})

            finally:
                update_task.cancel()

        except WebSocketDisconnect:
            logger.info("WebSocket disconnected for controller '%s'", controller.name)

    return websocket_handler


async def _handle_ws_command(
    controller: Controller,
    command: dict[str, Any],
    websocket: Any,
) -> None:
    """Handle a WebSocket command."""
    cmd_type = command.get("type")
    response: dict[str, Any] = {"type": "response", "command": cmd_type}

    try:
        if cmd_type == "move":
            controller.move_to(command.get("targets", {}))
            response["status"] = "ok"

        elif cmd_type == "home":
            controller.home()
            response["status"] = "ok"

        elif cmd_type == "stop":
            controller.stop()
            response["status"] = "ok"

        elif cmd_type == "enable":
            controller.enable()
            response["status"] = "ok"

        elif cmd_type == "disable":
            controller.disable()
            response["status"] = "ok"

        else:
            response["error"] = f"Unknown command: {cmd_type}"

    except Exception as e:
        response["error"] = str(e)

    await websocket.send_json(response)
