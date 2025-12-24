# API Integration Guide

This guide covers integrating robo-infra with svc-infra for REST API robotics control.

> **Learning svc-infra**: For comprehensive svc-infra documentation, use the **nfrax-docs MCP**
> tool with queries like `"svc-infra router_from_object"` or `"svc-infra dual routers"`.

## Overview

robo-infra provides seamless integration with svc-infra, allowing you to expose
robot controllers as REST APIs. The integration uses svc-infra's dual router system
(not generic `APIRouter`) for proper authentication handling, following nfraxlab's
integration standards.

**Key Features:**
- Automatic REST endpoint generation from controllers and actuators
- Dual router support (public/authenticated endpoints)
- svc-infra exception handling for consistent error responses
- WebSocket support for real-time control and monitoring

## Quick Start

```python
from fastapi import FastAPI
from robo_infra.core.controller import SimulatedController
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits
from robo_infra.integrations.svc_infra import controller_to_router

# Create a controller with actuators
controller = SimulatedController(name="arm")
controller.add_actuator(
    "shoulder",
    SimulatedActuator(
        name="shoulder",
        limits=Limits(min=0, max=180, default=90),
        unit="degrees",
    ),
)

# Create FastAPI app and add robot router
app = FastAPI()
router = controller_to_router(controller, prefix="/arm", auth_required=False)
app.include_router(router)

# Now you have REST endpoints:
# GET  /arm/status
# POST /arm/enable
# POST /arm/disable
# POST /arm/home
# POST /arm/stop
# POST /arm/move
# GET  /arm/actuators
# GET  /arm/sensors
# GET  /arm/positions
# POST /arm/positions/{name}
# POST /arm/positions/{name}/save
```

## Generated Endpoints

`controller_to_router()` generates the following REST endpoints:

| Method | Path | Description |
|--------|------|-------------|
| GET | `/status` | Get controller state and status |
| POST | `/enable` | Enable controller and actuators |
| POST | `/disable` | Disable controller and actuators |
| POST | `/home` | Home to default positions |
| POST | `/stop` | Emergency stop |
| POST | `/move` | Move actuators to positions |
| GET | `/actuators` | Get current actuator values |
| GET | `/sensors` | Read all sensor values |
| GET | `/positions` | List named positions |
| POST | `/positions/{name}` | Move to named position |
| POST | `/positions/{name}/save` | Save current position |

## Authentication

The integration uses svc-infra dual routers for proper authentication.

### Public Endpoints (No Auth)

For development, testing, or internal networks:

```python
# Public access - no JWT required
router = controller_to_router(controller, auth_required=False)
```

### Authenticated Endpoints (JWT Required)

For production deployments:

```python
# Authenticated access - requires valid JWT token
router = controller_to_router(controller, auth_required=True)
```

### Mixed Authentication

For complex scenarios, create separate routers:

```python
from fastapi import FastAPI
from robo_infra.integrations.svc_infra import controller_to_router

app = FastAPI()

# Status endpoint is public (for health checks)
status_router = controller_to_router(
    controller,
    prefix="/arm/public",
    auth_required=False,
    # Only expose status endpoint
)

# Control endpoints require authentication
control_router = controller_to_router(
    controller,
    prefix="/arm",
    auth_required=True,
)

app.include_router(status_router)
app.include_router(control_router)
```

## Using svc-infra Generic Utilities

robo-infra re-exports svc-infra's generic utilities for convenience:

```python
from robo_infra.integrations.svc_infra import (
    router_from_object,     # Generic object-to-router conversion
    endpoint_exclude,       # Decorator to exclude methods from router
    map_exception_to_http,  # Map exceptions to HTTP status codes
    DEFAULT_EXCEPTION_MAP,  # Standard exception mappings
    STATUS_TITLES,          # HTTP status code titles
    ROBOTICS_EXCEPTION_MAP, # Extended with robotics exceptions
)
```

### router_from_object

The `router_from_object()` utility from svc-infra automatically converts any
Python object's methods into FastAPI endpoints:

```python
from robo_infra.integrations.svc_infra import router_from_object

class Calculator:
    def get_value(self) -> float:
        """Get current value."""
        return 42.0
    
    def reset(self) -> str:
        """Reset the calculator."""
        return "reset complete"

router = router_from_object(Calculator(), prefix="/calc")
# Creates:
# GET  /calc/value  -> get_value()
# POST /calc/reset  -> reset()
```

### endpoint_exclude Decorator

Use `@endpoint_exclude` to mark methods that should not become endpoints:

```python
from robo_infra.integrations.svc_infra import endpoint_exclude, router_from_object

class Service:
    def public_action(self) -> str:
        """This becomes an endpoint."""
        return "action complete"
    
    @endpoint_exclude
    def internal_helper(self) -> str:
        """This is excluded from endpoints."""
        return "internal"

router = router_from_object(Service(), prefix="/svc")
# Only creates: POST /svc/public-action (internal_helper is excluded)
```

### Exception Mapping

Use `map_exception_to_http()` for consistent error handling:

```python
from robo_infra.integrations.svc_infra import map_exception_to_http

try:
    risky_operation()
except Exception as e:
    status, title, detail = map_exception_to_http(e)
    # ValueError -> 400, KeyError -> 404, etc.
```

## WebSocket Support

For real-time control and monitoring, use WebSocket routers:

```python
from robo_infra.integrations.svc_infra import create_websocket_router

ws_router = create_websocket_router(
    controller,
    prefix="/arm",
    auth_required=False,
    update_rate_hz=10.0,  # Send updates 10 times per second
)
app.include_router(ws_router)

# Connect to: ws://host/arm/ws
```

### WebSocket Message Format

**Incoming commands:**
```json
{"type": "move", "targets": {"shoulder": 45.0}}
{"type": "home"}
{"type": "stop"}
{"type": "enable"}
{"type": "disable"}
```

**Outgoing updates:**
```json
{
    "type": "update",
    "controller": "arm",
    "state": "idle",
    "actuators": {"shoulder": 45.0},
    "sensors": {"encoder": 180.0}
}
```

## Individual Actuator Routers

For fine-grained control, create routers for individual actuators:

```python
from robo_infra.integrations.svc_infra import actuator_to_router

actuator = SimulatedActuator(
    name="servo",
    limits=Limits(min=0, max=180, default=90),
    unit="degrees",
)

router = actuator_to_router(actuator, prefix="/servo")
# Creates:
# GET  /servo/     -> Get value and status
# POST /servo/set  -> Set value
# POST /servo/enable
# POST /servo/disable
```

## Error Handling

The integration uses svc-infra's `FastApiException` for proper error responses:

| Exception Type | HTTP Status | Use Case |
|---------------|-------------|----------|
| `ValueError` | 400 | Invalid input values |
| `LimitsExceededError` | 400 | Value out of actuator range |
| `KeyError` | 404 | Position not found |
| `PositionNotFoundError` | 404 | Named position doesn't exist |
| `HardwareNotFoundError` | 500 | Driver/device not available |
| `SafetyError` | 500 | Safety system triggered |

## Rate Limiting

Protect your robot from command flooding using svc-infra's rate limiting:

```python
from fastapi import FastAPI
from svc_infra.rate import RateLimitMiddleware, RateLimitConfig
from robo_infra.integrations.svc_infra import controller_to_router

app = FastAPI()

# Add rate limiting middleware
app.add_middleware(
    RateLimitMiddleware,
    config=RateLimitConfig(
        requests_per_second=10,  # Max 10 commands/second
        burst_size=20,           # Allow short bursts
    )
)

router = controller_to_router(arm_controller)
app.include_router(router, prefix="/v1/arm")
```

### Per-Endpoint Rate Limits

For more granular control, apply rate limits to specific endpoints:

```python
from svc_infra.rate import rate_limit

# Custom endpoint with specific rate limit
@app.post("/v1/arm/rapid-move")
@rate_limit(requests_per_second=1)  # Only 1 rapid move per second
async def rapid_move(targets: dict):
    return await arm_controller.move(targets, speed="fast")
```

## Observability

Add tracing and logging using svc-infra's observability utilities:

```python
from fastapi import FastAPI
from svc_infra.observability import (
    setup_tracing,
    TracingMiddleware,
    LoggingMiddleware,
)
from robo_infra.integrations.svc_infra import controller_to_router

app = FastAPI()

# Enable distributed tracing (Jaeger/Zipkin compatible)
setup_tracing(
    service_name="robot-api",
    endpoint="http://jaeger:14268/api/traces",
)

# Add tracing middleware
app.add_middleware(TracingMiddleware)

# Add structured logging
app.add_middleware(
    LoggingMiddleware,
    log_request_body=True,
    log_response_body=False,  # Avoid logging large position data
)

router = controller_to_router(arm_controller)
app.include_router(router, prefix="/v1/arm")
```

### Custom Metrics

Track robot-specific metrics:

```python
from svc_infra.observability import metrics

# Define custom metrics
move_counter = metrics.counter(
    "robot_moves_total",
    "Total number of robot move commands",
    labels=["controller", "status"],
)

move_latency = metrics.histogram(
    "robot_move_duration_seconds",
    "Time taken for move commands",
    labels=["controller"],
)

# Use in your handlers
@app.post("/v1/arm/move")
async def move(targets: dict):
    with move_latency.labels(controller="arm").time():
        try:
            result = await arm_controller.move(targets)
            move_counter.labels(controller="arm", status="success").inc()
            return result
        except Exception:
            move_counter.labels(controller="arm", status="error").inc()
            raise
```

## Best Practices

1. **Use authentication in production**: Always set `auth_required=True` for
   production deployments to prevent unauthorized robot control.

2. **Register error handlers**: Use svc-infra's error handlers for consistent
   error responses:
   ```python
   from svc_infra.api.fastapi.middleware.errors.handlers import register_error_handlers
   register_error_handlers(app)
   ```

3. **Add rate limiting**: For WebSocket endpoints, consider rate limiting
   commands to prevent overwhelming the robot.

4. **Monitor safety errors**: Log and alert on safety errors (HTTP 500 with
   code `SAFETY_ERROR`).

5. **Enable tracing in production**: Use distributed tracing to debug issues
   across your robot control pipeline.

6. **Use structured logging**: Log all commands with context for audit trails
   and debugging.

## See Also

- **nfrax-docs MCP**: Query with `"svc-infra router_from_object"`, `"svc-infra rate limiting"`,
  or `"svc-infra observability"` for comprehensive svc-infra documentation
- [Getting Started Guide](getting-started.md)
- [AI Integration Guide](ai-integration.md)
