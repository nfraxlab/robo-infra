# Observability

robo-infra integrates with svc-infra's observability infrastructure for metrics, logging, tracing, and health checks.

## Overview

The observability module provides:

- **Metrics** - Prometheus metrics for commands, positions, safety events
- **Health Checks** - Controller and actuator readiness monitoring
- **Structured Logging** - Context-aware logging with correlation IDs
- **Tracing** - Operation tracing with span context

All observability features work standalone but integrate seamlessly with svc-infra when available.

## Installation

```bash
# Install with observability support
pip install robo-infra[observability]

# Or with svc-infra for full integration
pip install robo-infra svc-infra[metrics,health]
```

## Metrics (via svc-infra)

### Prometheus Metrics

robo-infra exports these Prometheus metrics:

| Metric | Type | Labels | Description |
|--------|------|--------|-------------|
| `robo_commands_total` | Counter | controller, command, status | Total commands executed |
| `robo_command_duration_seconds` | Histogram | controller, command | Command execution time |
| `robo_actuator_position` | Gauge | controller, actuator | Current actuator position |
| `robo_sensor_value` | Gauge | controller, sensor, unit | Current sensor reading |
| `robo_safety_triggers_total` | Counter | controller, trigger_type | Safety events triggered |

### Recording Command Metrics

```python
from robo_infra.integrations.observability import record_command

# Record successful command
record_command(
    controller="arm",
    command="move",
    status="success",
    duration_seconds=0.150,
)

# Record failed command
record_command(
    controller="arm",
    command="move",
    status="error",
    duration_seconds=0.050,
)
```

### Using the @track_command Decorator

Automatically track async command execution:

```python
from robo_infra.integrations.observability import track_command

class ArmController:
    name = "arm"
    
    @track_command("move")
    async def move_to(self, target: float) -> dict:
        """Move to target position."""
        # Automatically tracks:
        # - robo_commands_total{controller="arm", command="move", status="success|error"}
        # - robo_command_duration_seconds{controller="arm", command="move"}
        await self._execute_move(target)
        return {"position": target}
    
    @track_command("home")
    async def home(self) -> dict:
        """Home the arm."""
        await self._execute_home()
        return {"homed": True}
```

### Recording Position Metrics

```python
from robo_infra.integrations.observability import record_position

# Update actuator position gauge
record_position(
    controller="arm",
    actuator="joint1",
    position=45.0,
)

# In a control loop
while running:
    for joint in controller.actuators:
        record_position(
            controller=controller.name,
            actuator=joint.name,
            position=joint.current_value,
        )
    await asyncio.sleep(0.02)  # 50Hz
```

### Recording Sensor Values

```python
from robo_infra.integrations.observability import record_sensor_value

# Record temperature
record_sensor_value(
    controller="arm",
    sensor="motor1_temp",
    value=45.2,
    unit="celsius",
)

# Record distance
record_sensor_value(
    controller="conveyor",
    sensor="proximity1",
    value=0.15,
    unit="meters",
)
```

### Controller Metrics

The @track_command decorator automatically records metrics for commands. For controllers with many actuators, record positions periodically:

```python
class ObservableController:
    def __init__(self, name: str, actuators: list):
        self.name = name
        self._actuators = actuators
    
    async def status_loop(self):
        """Publish status metrics at 50Hz."""
        while True:
            for actuator in self._actuators:
                record_position(
                    controller=self.name,
                    actuator=actuator.name,
                    position=actuator.current_value,
                )
            await asyncio.sleep(0.02)
```

### Sensor Metrics

Record sensor readings with units:

```python
from robo_infra.integrations.observability import record_sensor_value

# IMU data
record_sensor_value("robot", "accel_x", imu.accel_x, "m/s^2")
record_sensor_value("robot", "accel_y", imu.accel_y, "m/s^2")
record_sensor_value("robot", "accel_z", imu.accel_z, "m/s^2")

# Temperature sensors
record_sensor_value("arm", "motor1_temp", temp1.read(), "celsius")
record_sensor_value("arm", "motor2_temp", temp2.read(), "celsius")

# Distance sensors
record_sensor_value("mobile", "front_lidar", lidar.min_distance, "meters")
```

## Safety Metrics

### Safety Trigger Types

Use standard constants for consistent labeling:

```python
from robo_infra.integrations.observability import (
    SafetyTriggerType,
    record_safety_trigger,
)

# Available trigger types
SafetyTriggerType.ESTOP                # Emergency stop
SafetyTriggerType.LIMIT_EXCEEDED       # Joint/actuator limit
SafetyTriggerType.WATCHDOG_TIMEOUT     # Heartbeat timeout
SafetyTriggerType.MONITOR_ALERT        # Safety monitor alert
SafetyTriggerType.COLLISION_DETECTED   # Collision detected
SafetyTriggerType.COMMUNICATION_LOSS   # Hardware communication lost
SafetyTriggerType.OVERCURRENT          # Motor overcurrent
SafetyTriggerType.OVERTEMPERATURE      # Temperature limit exceeded
```

### Recording Safety Events

```python
from robo_infra.integrations.observability import (
    record_safety_trigger,
    record_estop_triggered,
    record_limit_exceeded,
    record_watchdog_timeout,
    record_monitor_alert,
    SafetyTriggerType,
)

# Generic safety trigger
record_safety_trigger("arm", SafetyTriggerType.COLLISION_DETECTED)

# Convenience functions with automatic logging
record_estop_triggered("arm")  # Logs warning + increments counter

record_limit_exceeded(
    "arm",
    actuator="joint1",
    limit_type="max",
)

record_watchdog_timeout("conveyor")

record_monitor_alert("arm", condition="workspace_violation")
```

## Logging

### Structured Logging Setup

```python
from robo_infra.integrations.observability import setup_robotics_logging

# Basic setup
setup_robotics_logging(level="INFO")

# Force JSON output (for production)
setup_robotics_logging(level="INFO", json_output=True)

# Debug mode with plain text
setup_robotics_logging(level="DEBUG", json_output=False)
```

### Context-Aware Logging

```python
from robo_infra.integrations.observability import log_with_context

# Log with robotics context
log_with_context(
    "info",
    "Move command completed",
    controller="arm",
    actuator="joint1",
    target=45.0,
    duration_ms=150,
)
# Output: {"message": "Move command completed", "controller": "arm", 
#          "actuator": "joint1", "target": 45.0, "duration_ms": 150, ...}

# Log errors with context
log_with_context(
    "error",
    "Position limit exceeded",
    controller="arm",
    actuator="joint1",
    current=95.0,
    limit=90.0,
)
```

### Correlation IDs

Track requests across operations:

```python
from robo_infra.integrations.observability import (
    set_robotics_request_id,
    get_robotics_request_id,
)

# Set correlation ID for a request
set_robotics_request_id("req-12345")

# All subsequent logs include the request ID
log_with_context("info", "Processing move command", controller="arm")
# Output includes: {"request_id": "req-12345", ...}

# Get current request ID
request_id = get_robotics_request_id()
```

### Log Levels

Use appropriate log levels:

```python
import logging

logger = logging.getLogger("robo_infra")

# DEBUG: Detailed diagnostic information
logger.debug("Joint position update: %s -> %s", old_pos, new_pos)

# INFO: General operational messages
logger.info("Controller '%s' initialized", controller.name)

# WARNING: Unexpected but handled situations
logger.warning("Velocity limit exceeded, clamping to %s", max_vel)

# ERROR: Errors that need attention
logger.error("Failed to communicate with motor driver: %s", error)
```

## Tracing

### Operation Tracing

Trace long-running operations:

```python
from svc_infra.obs.tracing import start_span, get_current_span

async def execute_trajectory(trajectory):
    with start_span("execute_trajectory") as span:
        span.set_attribute("trajectory.points", len(trajectory.points))
        span.set_attribute("controller", "arm")
        
        for i, point in enumerate(trajectory.points):
            with start_span(f"waypoint_{i}") as waypoint_span:
                waypoint_span.set_attribute("position", point.position)
                await move_to(point)
```

### Span Context

Pass context between operations:

```python
from svc_infra.obs.tracing import get_current_span, inject_context

# Extract context for passing to another service
span = get_current_span()
context = inject_context({})

# Pass context in HTTP headers
response = await client.post(
    "/external/service",
    headers=context,
)
```

## Health Checks

### Controller Health Checks

Create health checks for controllers:

```python
from robo_infra.integrations.observability import create_controller_health_check
from svc_infra.health import HealthRegistry

registry = HealthRegistry()

# Create health check for a controller
arm_health = create_controller_health_check(arm_controller, timeout=5.0)
registry.add("arm", arm_health, critical=True)

# Health check verifies:
# - Controller not in error state
# - Controller is enabled
# - Safety systems not triggered (e-stop not active)
```

### Actuator Health Checks

```python
from robo_infra.integrations.observability import create_actuator_health_check

# Create health check for an actuator
joint1_health = create_actuator_health_check(joint1, timeout=5.0)
registry.add("joint1", joint1_health)

# Health check verifies:
# - Communication working (can read position)
# - Actuator not in fault state
```

### Register Multiple Controllers

```python
from robo_infra.integrations.observability import register_controller_health_checks

controllers = [arm_controller, conveyor_controller, gripper_controller]

register_controller_health_checks(
    registry,
    controllers,
    timeout=5.0,
    prefix="controller:",
)
# Creates: "controller:arm", "controller:conveyor", "controller:gripper"
```

### Add Health Routes to FastAPI

```python
from fastapi import FastAPI
from robo_infra.integrations.observability import add_robotics_health_routes

app = FastAPI()

# Add health endpoints
registry = add_robotics_health_routes(
    app,
    controllers=[arm, conveyor, gripper],
    prefix="/_health",
    timeout=5.0,
)

# Creates endpoints:
# GET /_health/live   - Liveness probe
# GET /_health/ready  - Readiness probe
# GET /_health/startup - Startup probe
```

### Custom Health Checks

```python
from svc_infra.health import HealthCheckResult, HealthStatus

async def battery_health_check() -> HealthCheckResult:
    """Check battery level."""
    battery_level = await get_battery_level()
    
    if battery_level < 10:
        return HealthCheckResult(
            name="battery",
            status=HealthStatus.UNHEALTHY,
            message=f"Battery critical: {battery_level}%",
        )
    elif battery_level < 20:
        return HealthCheckResult(
            name="battery",
            status=HealthStatus.DEGRADED,
            message=f"Battery low: {battery_level}%",
        )
    else:
        return HealthCheckResult(
            name="battery",
            status=HealthStatus.HEALTHY,
            message=f"Battery OK: {battery_level}%",
        )

registry.add("battery", battery_health_check)
```

## Complete Example

```python
"""Observable robot controller example."""

import asyncio
import logging
from fastapi import FastAPI

from robo_infra.controllers import JointGroup
from robo_infra.actuators import Servo
from robo_infra.drivers import PCA9685Driver
from robo_infra.integrations.observability import (
    setup_robotics_logging,
    track_command,
    record_position,
    record_sensor_value,
    record_estop_triggered,
    add_robotics_health_routes,
    log_with_context,
    set_robotics_request_id,
)


# Setup logging
setup_robotics_logging(level="INFO", json_output=True)

logger = logging.getLogger(__name__)


class ObservableArmController:
    """Robot arm controller with full observability."""
    
    def __init__(self):
        self.name = "arm"
        self._driver = PCA9685Driver()
        self._joints = [
            Servo(f"joint{i}", channel=i, driver=self._driver)
            for i in range(6)
        ]
        self._arm = JointGroup(self.name, joints=self._joints)
        self._running = False
    
    @track_command("move")
    async def move_to(self, positions: dict[str, float]) -> dict:
        """Move to target positions."""
        log_with_context(
            "info",
            "Starting move command",
            controller=self.name,
            targets=positions,
        )
        
        await self._arm.move_to(positions)
        
        return {"success": True, "positions": positions}
    
    @track_command("home")
    async def home(self) -> dict:
        """Home all joints."""
        await self._arm.home()
        return {"homed": True}
    
    def stop(self):
        """Emergency stop."""
        self._arm.stop()
        record_estop_triggered(self.name)
    
    async def status_loop(self):
        """Publish metrics at 50Hz."""
        self._running = True
        while self._running:
            for joint in self._joints:
                record_position(
                    controller=self.name,
                    actuator=joint.name,
                    position=joint.current_value,
                )
            await asyncio.sleep(0.02)
    
    def shutdown(self):
        """Stop the controller."""
        self._running = False


# FastAPI application with health checks
app = FastAPI(title="Observable Robot API")
arm = ObservableArmController()

# Add health routes
registry = add_robotics_health_routes(
    app,
    controllers=[arm._arm],
    prefix="/_health",
)


@app.on_event("startup")
async def startup():
    """Start metrics publishing."""
    asyncio.create_task(arm.status_loop())


@app.on_event("shutdown")
async def shutdown():
    """Stop the controller."""
    arm.shutdown()


@app.post("/move")
async def move(positions: dict[str, float], request_id: str | None = None):
    """Move the arm to target positions."""
    if request_id:
        set_robotics_request_id(request_id)
    
    result = await arm.move_to(positions)
    return result


@app.post("/home")
async def home():
    """Home the arm."""
    result = await arm.home()
    return result


@app.post("/stop")
async def stop():
    """Emergency stop."""
    arm.stop()
    return {"stopped": True}
```

## Grafana Dashboards

### Example PromQL Queries

```promql
# Command rate by controller
rate(robo_commands_total[5m])

# Error rate
rate(robo_commands_total{status="error"}[5m]) 
  / rate(robo_commands_total[5m])

# Command latency P99
histogram_quantile(0.99, 
  rate(robo_command_duration_seconds_bucket[5m])
)

# Safety events
increase(robo_safety_triggers_total[1h])

# Current joint positions
robo_actuator_position{controller="arm"}
```

### Alert Examples

```yaml
# Alert on high error rate
- alert: RobotCommandErrors
  expr: |
    rate(robo_commands_total{status="error"}[5m]) > 0.1
  for: 5m
  labels:
    severity: warning
  annotations:
    summary: "High command error rate on {{ $labels.controller }}"

# Alert on safety events
- alert: RobotSafetyTrigger
  expr: |
    increase(robo_safety_triggers_total[5m]) > 0
  labels:
    severity: critical
  annotations:
    summary: "Safety event on {{ $labels.controller }}: {{ $labels.trigger_type }}"
```

## Best Practices

### 1. Instrument All Commands

```python
# [OK] Track all user-facing commands
@track_command("move")
async def move_to(self, target): ...

@track_command("grip")
async def grip(self, force): ...

# [X] Don't track internal helper methods
def _calculate_trajectory(self, ...): ...
```

### 2. Use Consistent Labels

```python
# [OK] Consistent naming
record_position("arm", "joint1", 45.0)
record_position("arm", "joint2", 30.0)

# [X] Inconsistent naming
record_position("Arm", "Joint1", 45.0)
record_position("robot_arm", "j2", 30.0)
```

### 3. Don't Log Sensitive Data

```python
# [OK] Log safe information
log_with_context("info", "Auth successful", user_id=user.id)

# [X] Never log credentials
log_with_context("info", "Auth", password=user.password)  # NEVER!
```

### 4. Set Appropriate Timeouts

```python
# [OK] Use appropriate timeouts for hardware
create_controller_health_check(arm, timeout=5.0)

# [X] Too short for hardware communication
create_controller_health_check(arm, timeout=0.1)
```

## See Also

- [Safety](safety.md) - Safety systems that trigger metrics
- [Controllers](controllers.md) - Controllers with observability
- [ROS2 Integration](ros2-integration.md) - Observability for ROS2 nodes
