"""Integration with svc-infra for REST API robotics control.

This module provides utilities to convert robo-infra controllers and
actuators into FastAPI routers for REST/WebSocket control.

CRITICAL: This module uses svc-infra dual routers (not generic APIRouter)
following the mandatory integration standards from robo-infra/AGENTS.md.

This module integrates with svc-infra's `router_from_object()` utility for
router creation patterns while providing robotics-specific enhancements like:
- Custom response formats with controller/actuator names
- Robotics-specific endpoints (move, positions, sensors)
- Safety-focused error handling

Example:
    >>> from fastapi import FastAPI
    >>> from robo_infra.core.controller import SimulatedController
    >>> from robo_infra.integrations.svc_infra import controller_to_router
    >>>
    >>> app = FastAPI()
    >>> controller = SimulatedController(name="arm")
    >>> router = controller_to_router(controller, auth_required=False)
    >>> app.include_router(router, prefix="/v1/arm")

Router Selection:
    - auth_required=False: Uses public_router (no auth)
    - auth_required=True: Uses user_router (requires JWT auth)
    - WebSocket: ws_public_router or ws_protected_router

Error Handling:
    Uses svc-infra FastApiException for proper error responses:
    - 400 for validation errors (invalid targets, limits exceeded)
    - 404 for not found errors (position not found)
    - 500 for system errors (hardware failure, safety errors)

Note:
    We intentionally do NOT use `from __future__ import annotations` here
    because FastAPI needs actual type objects (not string annotations) for
    Pydantic model parameter resolution in endpoint handlers.
"""

import logging
from typing import TYPE_CHECKING, Any


# Import router utilities from svc-infra for router pattern integration
try:
    from svc_infra.api.fastapi import (
        DEFAULT_EXCEPTION_MAP,
        STATUS_TITLES,
        endpoint_exclude,
        map_exception_to_http,
        router_from_object,
    )

    _SVC_INFRA_AVAILABLE = True
except ImportError:
    _SVC_INFRA_AVAILABLE = False
    DEFAULT_EXCEPTION_MAP = {}  # type: ignore[misc]
    STATUS_TITLES = {}  # type: ignore[misc]
    endpoint_exclude = None  # type: ignore[assignment]
    map_exception_to_http = None  # type: ignore[assignment]
    router_from_object = None  # type: ignore[assignment]


def _check_svc_infra() -> None:
    """Raise ImportError if svc-infra is not available."""
    if not _SVC_INFRA_AVAILABLE:
        raise ImportError(
            "svc-infra is required for API integration. Install with: pip install robo-infra[api]"
        )


if TYPE_CHECKING:
    from robo_infra.core.actuator import Actuator
    from robo_infra.core.controller import Controller


logger = logging.getLogger(__name__)


__all__ = [
    # Re-exported from svc-infra for convenience
    "DEFAULT_EXCEPTION_MAP",
    # Robotics-specific exception mapping
    "ROBOTICS_EXCEPTION_MAP",
    "STATUS_TITLES",
    "actuator_to_router",
    "controller_to_router",
    "create_websocket_router",
    "endpoint_exclude",
    "map_exception_to_http",
    "router_from_object",
]


# =============================================================================
# Exception Mapping (Robotics-specific extensions of svc-infra utilities)
# =============================================================================

# Robotics-specific exception to HTTP status code mapping
# Extends svc-infra DEFAULT_EXCEPTION_MAP with robotics exceptions
ROBOTICS_EXCEPTION_MAP: dict[type[Exception], int] = {
    **DEFAULT_EXCEPTION_MAP,
    # Additional robotics-specific exceptions can be registered here
    # via exception class (not string name) for type safety
}


def _raise_api_error(
    exc: Exception,
    *,
    default_status: int = 500,
    default_title: str = "Internal Error",
) -> None:
    """Map robo-infra exceptions to svc-infra FastApiException.

    Uses svc-infra's `map_exception_to_http()` utility for base exception
    handling, with robotics-specific extensions for hardware/safety errors.

    Exception mapping:
    - ValueError, LimitsExceededError -> 400 Validation Error
    - KeyError, PositionNotFoundError -> 404 Not Found
    - HardwareNotFoundError, CommunicationError -> 500 Hardware Error
    - SafetyError -> 500 Safety Error (critical)
    - Other -> 500 Internal Error

    Args:
        exc: The exception to map.
        default_status: Default HTTP status if exception type not recognized.
        default_title: Default error title if exception type not recognized.

    Raises:
        FastApiException with appropriate status code.
    """
    try:
        from svc_infra.exceptions import FastApiException
    except ImportError:
        # Fallback to standard HTTPException if svc-infra not available
        from fastapi import HTTPException

        raise HTTPException(status_code=default_status, detail=str(exc)) from exc

    exc_name = type(exc).__name__

    # Robotics-specific exception handling (not covered by svc-infra defaults)
    # These are matched by name to avoid importing robo-infra exception classes
    if exc_name in ("LimitsExceededError", "InvalidTargetError", "JointNotFoundError"):
        raise FastApiException(
            title="Validation Error",
            detail=str(exc),
            status_code=400,
            code="VALIDATION_ERROR",
        ) from exc

    if exc_name in ("PositionNotFoundError",):
        raise FastApiException(
            title="Not Found",
            detail=str(exc),
            status_code=404,
            code="NOT_FOUND",
        ) from exc

    if exc_name in ("HardwareNotFoundError", "CommunicationError", "DriverError"):
        raise FastApiException(
            title="Hardware Error",
            detail=str(exc),
            status_code=500,
            code="HARDWARE_ERROR",
        ) from exc

    if exc_name in ("SafetyError", "EmergencyStopError", "LimitViolationError"):
        raise FastApiException(
            title="Safety Error",
            detail=f"SAFETY: {exc}",
            status_code=500,
            code="SAFETY_ERROR",
        ) from exc

    # Use svc-infra map_exception_to_http for standard exceptions
    status, title, detail = map_exception_to_http(exc, ROBOTICS_EXCEPTION_MAP)

    raise FastApiException(
        title=title,
        detail=detail,
        status_code=status,
        code=type(exc).__name__.upper(),
    ) from exc


# =============================================================================
# Router Creation Helpers
# =============================================================================


def _create_dual_router(
    name: str,
    *,
    prefix: str = "",
    tags: list[str] | None = None,
    auth_required: bool = False,
) -> Any:
    """Create a svc-infra dual router with proper auth configuration.

    This is a helper that encapsulates the router creation pattern used
    by both controller_to_router and actuator_to_router, reducing code
    duplication.

    Args:
        name: Entity name for default tags.
        prefix: URL prefix for the router.
        tags: OpenAPI tags for the router.
        auth_required: If True, use user_router (JWT auth required).
                      If False, use public_router (no auth).

    Returns:
        DualAPIRouter instance from svc-infra, or APIRouter fallback.
    """
    default_tags = tags or [name]

    try:
        if auth_required:
            from svc_infra.api.fastapi.dual import user_router

            return user_router(prefix=prefix, tags=default_tags)
        else:
            from svc_infra.api.fastapi.dual import public_router

            return public_router(prefix=prefix, tags=default_tags)
    except ImportError:
        logger.warning(
            "svc-infra not available, using generic APIRouter. "
            "Install svc-infra for proper dual router support."
        )
        from fastapi import APIRouter

        return APIRouter(prefix=prefix, tags=list(default_tags))


def controller_to_router(
    controller: "Controller",
    *,
    prefix: str = "",
    tags: list[str] | None = None,
    auth_required: bool = False,
) -> Any:
    """Convert a controller to a FastAPI router.

    Uses svc-infra dual routers for proper auth integration and trailing
    slash handling. Select auth mode based on deployment requirements.

    This function uses the svc-infra router pattern internally while
    providing robotics-specific endpoints and response formats.

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
    - POST /positions/{name}/save - Save current position

    Args:
        controller: The controller to convert.
        prefix: URL prefix for the router.
        tags: OpenAPI tags for the router.
        auth_required: If True, use user_router (JWT auth required).
                      If False, use public_router (no auth).

    Returns:
        DualAPIRouter instance from svc-infra.

    Raises:
        ImportError: If svc-infra or FastAPI is not installed.

    Example:
        >>> # Public access (no auth)
        >>> router = controller_to_router(arm, prefix="/arm")
        >>>
        >>> # Authenticated access
        >>> router = controller_to_router(arm, prefix="/arm", auth_required=True)
    """
    _check_svc_infra()
    try:
        from pydantic import BaseModel
    except ImportError as e:
        raise ImportError(
            "Pydantic is required for svc-infra integration. Install with: pip install pydantic"
        ) from e

    # Use helper to create dual router (reduces code duplication)
    router = _create_dual_router(
        controller.name,
        prefix=prefix,
        tags=tags,
        auth_required=auth_required,
    )

    name = controller.name

    # Pydantic models for request/response
    class MoveRequest(BaseModel):
        """Request body for move endpoint."""

        targets: dict[str, float]

    class StatusResponse(BaseModel):
        """Response for status endpoint."""

        name: str
        state: str
        mode: str
        is_enabled: bool
        is_homed: bool
        is_running: bool
        error: str | None = None
        actuator_count: int
        sensor_count: int
        uptime: float

    class PositionResponse(BaseModel):
        """Response for position endpoints."""

        name: str
        values: dict[str, float]

    @router.get("/status", response_model=StatusResponse)
    async def get_status() -> StatusResponse:
        """Get current controller status."""
        status = controller.status()
        return StatusResponse(
            name=name,
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
            return {"status": "enabled", "controller": name}
        except Exception as e:
            _raise_api_error(e, default_title="Enable Failed")
            return {}  # Never reached, for type checker

    @router.post("/disable")
    async def disable() -> dict[str, str]:
        """Disable the controller."""
        try:
            controller.disable()
            return {"status": "disabled", "controller": name}
        except Exception as e:
            _raise_api_error(e, default_title="Disable Failed")
            return {}

    @router.post("/home")
    async def home() -> dict[str, str]:
        """Home the controller."""
        try:
            controller.home()
            return {"status": "homed", "controller": name}
        except Exception as e:
            _raise_api_error(e, default_title="Home Failed")
            return {}

    @router.post("/stop")
    async def emergency_stop() -> dict[str, str]:
        """Emergency stop the controller.

        Note: E-stop always attempts to stop all motion, even if errors occur.
        """
        try:
            controller.stop()
            return {"status": "stopped", "controller": name}
        except Exception as e:
            # E-stop errors should be logged but we still return success
            # because the stop was attempted
            logger.error("E-stop error on %s: %s", name, e)
            return {
                "status": "stopped_with_errors",
                "controller": name,
                "warning": str(e),
            }

    @router.post("/move")
    async def move(request: MoveRequest) -> dict[str, Any]:
        """Move actuators to target positions."""
        try:
            controller.move_to(request.targets)
            return {"status": "moved", "controller": name, "targets": request.targets}
        except Exception as e:
            _raise_api_error(e, default_status=400, default_title="Move Failed")
            return {}

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
            return {"status": "moved", "controller": name, "position": position_name}
        except KeyError as e:
            _raise_api_error(e, default_status=404, default_title="Position Not Found")
            return {}
        except Exception as e:
            _raise_api_error(e)
            return {}

    @router.post("/positions/{position_name}/save", response_model=PositionResponse)
    async def save_position(position_name: str) -> PositionResponse:
        """Save current position with the given name."""
        try:
            position = controller.save_position(position_name)
            return PositionResponse(name=position.name, values=position.values)
        except Exception as e:
            _raise_api_error(e, default_title="Save Position Failed")
            return PositionResponse(name="", values={})  # Never reached

    logger.debug("Created router for controller '%s' with prefix '%s'", name, prefix)
    return router


# =============================================================================
# Actuator Router
# =============================================================================


def actuator_to_router(
    actuator: "Actuator",
    *,
    prefix: str = "",
    tags: list[str] | None = None,
    auth_required: bool = False,
) -> Any:
    """Convert a single actuator to a FastAPI router.

    Uses svc-infra dual routers for proper auth integration.
    This function uses the svc-infra router pattern internally while
    providing actuator-specific endpoints and response formats.

    Creates endpoints for:
    - GET / - Get current value and status
    - POST /set - Set value
    - POST /enable - Enable actuator
    - POST /disable - Disable actuator

    Args:
        actuator: The actuator to convert.
        prefix: URL prefix for the router.
        tags: OpenAPI tags for the router.
        auth_required: If True, use user_router (JWT auth required).
                      If False, use public_router (no auth).

    Returns:
        DualAPIRouter instance from svc-infra.

    Raises:
        ImportError: If svc-infra is not installed.
    """
    _check_svc_infra()
    try:
        from pydantic import BaseModel, Field
    except ImportError as e:
        raise ImportError(
            "Pydantic is required for svc-infra integration. Install with: pip install pydantic"
        ) from e

    # Use helper to create dual router (reduces code duplication)
    router = _create_dual_router(
        actuator.name,
        prefix=prefix,
        tags=tags,
        auth_required=auth_required,
    )

    limits = actuator.limits
    name = actuator.name

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
        """Get current actuator value and status."""
        return ActuatorStatus(
            name=name,
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
            return {"status": "set", "actuator": name, "value": request.value}
        except Exception as e:
            _raise_api_error(e, default_status=400, default_title="Set Value Failed")
            return {}

    @router.post("/enable")
    async def enable() -> dict[str, str]:
        """Enable the actuator."""
        try:
            actuator.enable()
            return {"status": "enabled", "actuator": name}
        except Exception as e:
            _raise_api_error(e, default_title="Enable Failed")
            return {}

    @router.post("/disable")
    async def disable() -> dict[str, str]:
        """Disable the actuator."""
        try:
            actuator.disable()
            return {"status": "disabled", "actuator": name}
        except Exception as e:
            _raise_api_error(e, default_title="Disable Failed")
            return {}

    logger.debug("Created router for actuator '%s' with prefix '%s'", name, prefix)
    return router


# =============================================================================
# WebSocket Support
# =============================================================================


def _create_ws_router(
    name: str,
    *,
    prefix: str = "",
    tags: list[str] | None = None,
    auth_required: bool = False,
) -> Any:
    """Create a svc-infra WebSocket router with proper auth configuration.

    Args:
        name: Entity name for default tags.
        prefix: URL prefix for the router.
        tags: OpenAPI tags for the router.
        auth_required: If True, use ws_protected_router (JWT auth).
                      If False, use ws_public_router (no auth).

    Returns:
        WebSocket router from svc-infra, or APIRouter fallback.
    """
    default_tags = tags or [name]

    try:
        if auth_required:
            from svc_infra.api.fastapi.dual import ws_protected_router

            return ws_protected_router(prefix=prefix, tags=default_tags)
        else:
            from svc_infra.api.fastapi.dual import ws_public_router

            return ws_public_router(prefix=prefix, tags=default_tags)
    except ImportError:
        logger.warning(
            "svc-infra not available, using generic APIRouter for WebSocket. "
            "Install svc-infra for proper WebSocket router support."
        )
        from fastapi import APIRouter

        return APIRouter(prefix=prefix, tags=list(default_tags))


def create_websocket_router(
    controller: "Controller",
    *,
    prefix: str = "",
    tags: list[str] | None = None,
    auth_required: bool = False,
    update_rate_hz: float = 10.0,
) -> Any:
    """Create a WebSocket router for real-time controller updates.

    Uses svc-infra WebSocket routers for proper auth integration.

    Provides:
    - Real-time actuator value streaming
    - Sensor reading streaming
    - State change notifications
    - Command reception

    Args:
        controller: The controller.
        prefix: URL prefix for the router.
        tags: OpenAPI tags for the router.
        auth_required: If True, use ws_protected_router (JWT auth).
                      If False, use ws_public_router (no auth).
        update_rate_hz: Rate of automatic updates in Hz (default: 10).

    Returns:
        WebSocket router with /ws endpoint.

    Example:
        >>> router = create_websocket_router(arm, prefix="/arm", auth_required=False)
        >>> app.include_router(router)
        >>> # Connect to ws://host/arm/ws
    """
    _check_svc_infra()
    try:
        from fastapi import WebSocket, WebSocketDisconnect
    except ImportError as e:
        raise ImportError(
            "FastAPI is required for WebSocket support. Install with: pip install fastapi"
        ) from e

    import asyncio
    import json

    # Use helper to create WebSocket router (reduces code duplication)
    router = _create_ws_router(
        controller.name,
        prefix=prefix,
        tags=tags,
        auth_required=auth_required,
    )

    name = controller.name
    update_interval = 1.0 / update_rate_hz

    @router.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        """Handle WebSocket connection for controller updates."""
        await websocket.accept()
        logger.info("WebSocket connected for controller '%s'", name)

        try:
            # Start background task to send updates
            async def send_updates() -> None:
                while True:
                    try:
                        data = {
                            "type": "update",
                            "controller": name,
                            "state": controller.status().state.value,
                            "actuators": controller.get_actuator_values(),
                            "sensors": controller.read_sensors(),
                        }
                        await websocket.send_json(data)
                    except Exception as e:
                        logger.error("Error sending WebSocket update: %s", e)
                    await asyncio.sleep(update_interval)

            update_task = asyncio.create_task(send_updates())

            try:
                # Handle incoming commands
                while True:
                    message = await websocket.receive_text()
                    try:
                        command = json.loads(message)
                        response = await _handle_ws_command(controller, command)
                        await websocket.send_json(response)
                    except json.JSONDecodeError:
                        await websocket.send_json({"type": "error", "error": "Invalid JSON"})
            finally:
                update_task.cancel()
                try:
                    await update_task
                except asyncio.CancelledError:
                    pass

        except WebSocketDisconnect:
            logger.info("WebSocket disconnected for controller '%s'", name)

    return router


async def _handle_ws_command(
    controller: "Controller",
    command: dict[str, Any],
) -> dict[str, Any]:
    """Handle a WebSocket command.

    Args:
        controller: The controller to command.
        command: Command dict with 'type' and optional parameters.

    Returns:
        Response dict with 'type', 'status', and optional 'error'.
    """
    cmd_type = command.get("type")
    response: dict[str, Any] = {
        "type": "response",
        "command": cmd_type,
        "controller": controller.name,
    }

    try:
        if cmd_type == "move":
            targets = command.get("targets", {})
            controller.move_to(targets)
            response["status"] = "ok"
            response["targets"] = targets

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

        elif cmd_type == "status":
            status = controller.status()
            response["status"] = "ok"
            response["data"] = {
                "state": status.state.value,
                "is_enabled": status.is_enabled,
                "is_homed": status.is_homed,
            }

        else:
            response["status"] = "error"
            response["error"] = f"Unknown command: {cmd_type}"

    except Exception as e:
        response["status"] = "error"
        response["error"] = str(e)
        logger.warning("WebSocket command '%s' failed: %s", cmd_type, e)

    return response
