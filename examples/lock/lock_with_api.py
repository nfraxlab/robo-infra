#!/usr/bin/env python3
"""REST API for smart lock control using svc-infra.

This example demonstrates API-controlled smart lock:
- Using svc-infra easy_service_app for batteries-included FastAPI
- Generating router from lock controller with controller_to_router
- Built-in health checks, metrics, and documentation

Requirements:
    pip install svc-infra robo-infra uvicorn

Run:
    python lock_with_api.py

    Then visit:
    - API docs: http://localhost:8000/docs
    - Health check: http://localhost:8000/healthz
    - Lock status: http://localhost:8000/lock/status

API Endpoints:
    GET  /lock/status     - Get lock state and position
    POST /lock/enable     - Enable the lock
    POST /lock/disable    - Disable the lock
    POST /lock/lock       - Lock the mechanism
    POST /lock/unlock     - Unlock the mechanism
    POST /lock/toggle     - Toggle lock state
    POST /lock/home       - Home the lock (go to start_locked state)
    POST /lock/stop       - Stop lock movement
"""

import uvicorn
from fastapi import FastAPI

from robo_infra.actuators.servo import Servo
from robo_infra.controllers.lock import Lock, LockConfig
from robo_infra.integrations.svc_infra import controller_to_router


def create_lock() -> Lock:
    """Create a servo-based smart lock.

    Returns:
        Configured Lock controller.
    """
    # Create a simulated servo actuator for the lock
    servo = Servo(name="lock_servo", angle_range=(0, 90))

    # Configure the lock
    config = LockConfig(
        name="door_lock",
        description="API-controlled electronic door lock",
        locked_position=0.0,
        unlocked_position=90.0,
        transition_time=0.5,
        start_locked=True,
    )

    # Create the lock controller
    lock = Lock(
        name="door_lock",
        actuator=servo,
        config=config,
    )

    return lock


def create_app() -> FastAPI:
    """Create the FastAPI application with lock control endpoints.

    Returns:
        Configured FastAPI application.
    """
    # Try to use svc-infra's easy_service_app for batteries-included setup
    # Falls back to plain FastAPI if svc-infra is not available
    try:
        from svc_infra.api.fastapi.ease import easy_service_app

        app = easy_service_app(
            name="Smart Lock API",
            release="1.0.0",
            public_cors_origins=["http://localhost:3000"],  # For web UI
        )
        print("Using svc-infra easy_service_app (with logging, metrics, health checks)")
    except ImportError:
        # Fallback to plain FastAPI
        app = FastAPI(
            title="Smart Lock API",
            description="REST API for controlling a smart lock",
            version="1.0.0",
        )
        print("Using plain FastAPI (svc-infra not installed)")

        # Add basic health check endpoint
        @app.get("/healthz")
        async def health_check() -> dict:
            return {"status": "healthy"}

    # Create and enable the lock
    lock = create_lock()
    lock.actuator.enable()
    lock.enable()

    # Generate router from the lock controller
    # NOTE: For production, use auth_required=True for security
    router = controller_to_router(
        lock,
        prefix="/lock",
        tags=["Lock Control"],
        # auth_required=True,  # Uncomment for production security
    )

    # Include the lock router
    app.include_router(router)

    # Add root endpoint with API info
    @app.get("/", tags=["Root"])
    async def root() -> dict:
        return {
            "name": "Smart Lock API",
            "version": "1.0.0",
            "lock": lock.name,
            "state": lock.state_str,
            "is_locked": lock.is_locked,
            "endpoints": {
                "docs": "/docs",
                "health": "/healthz",
                "status": "/lock/status",
                "lock": "/lock/lock",
                "unlock": "/lock/unlock",
                "toggle": "/lock/toggle",
            },
        }

    return app


def main() -> None:
    """Run the lock API server."""
    print("=" * 60)
    print("Smart Lock API Server")
    print("=" * 60)

    app = create_app()

    print("\nStarting server...")
    print("  API Docs:    http://localhost:8000/docs")
    print("  Health:      http://localhost:8000/healthz")
    print("  Lock Status: http://localhost:8000/lock/status")
    print("\nEndpoints:")
    print("  POST /lock/lock     - Lock the mechanism")
    print("  POST /lock/unlock   - Unlock the mechanism")
    print("  POST /lock/toggle   - Toggle lock state")
    print("  GET  /lock/status   - Get current state")
    print("\nPress Ctrl+C to stop")
    print("=" * 60)

    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
