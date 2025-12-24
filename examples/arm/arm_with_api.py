#!/usr/bin/env python3
"""REST API for robot arm control using svc-infra.

This example demonstrates API-controlled robot arm:
- Using svc-infra easy_service_app for batteries-included FastAPI
- Generating router from arm controller with controller_to_router
- Dual router support (public vs authenticated endpoints)
- Built-in health checks, metrics, and documentation

Requirements:
    pip install svc-infra robo-infra uvicorn

Run:
    python arm_with_api.py

    Then visit:
    - API docs: http://localhost:8000/docs
    - Health check: http://localhost:8000/healthz
    - Arm status: http://localhost:8000/arm/status

API Endpoints:
    GET  /arm/status     - Get arm state and positions
    POST /arm/enable     - Enable the arm
    POST /arm/disable    - Disable the arm
    POST /arm/home       - Home all joints
    POST /arm/stop       - Emergency stop
    POST /arm/move       - Move to target positions
    GET  /arm/actuators  - Get all actuator values
    GET  /arm/sensors    - Read all sensor values
"""

import uvicorn
from fastapi import FastAPI

from robo_infra.controllers.joint_group import JointGroup, JointGroupConfig
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits
from robo_infra.integrations.svc_infra import controller_to_router


def create_arm() -> JointGroup:
    """Create a 4-DOF robot arm with simulated servos.

    Returns:
        Configured and enabled JointGroup controller.
    """
    # Create simulated servo actuators for each joint
    joints = {
        "base": SimulatedActuator(
            name="base",
            limits=Limits(min=0, max=360, default=180),
            unit="degrees",
        ),
        "shoulder": SimulatedActuator(
            name="shoulder",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
        "elbow": SimulatedActuator(
            name="elbow",
            limits=Limits(min=0, max=150, default=75),
            unit="degrees",
        ),
        "wrist": SimulatedActuator(
            name="wrist",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
    }

    # Configure the arm with home and named positions
    config = JointGroupConfig(
        name="robot_arm",
        home_positions={
            "base": 180,
            "shoulder": 90,
            "elbow": 75,
            "wrist": 90,
        },
        named_positions={
            "ready": {"base": 180, "shoulder": 45, "elbow": 90, "wrist": 90},
            "rest": {"base": 0, "shoulder": 0, "elbow": 0, "wrist": 0},
            "pickup": {"base": 90, "shoulder": 60, "elbow": 120, "wrist": 45},
        },
    )

    # Create the arm controller
    arm = JointGroup(name="robot_arm", joints=joints, config=config)

    return arm


def create_app() -> FastAPI:
    """Create the FastAPI application with arm control endpoints.

    Returns:
        Configured FastAPI application.
    """
    # Try to use svc-infra's easy_service_app for batteries-included setup
    # Falls back to plain FastAPI if svc-infra is not available
    try:
        from svc_infra.api.fastapi.ease import easy_service_app

        app = easy_service_app(
            name="Robot Arm API",
            release="1.0.0",
            public_cors_origins=["http://localhost:3000"],  # For web UI
        )
        print("Using svc-infra easy_service_app (with logging, metrics, health checks)")
    except ImportError:
        # Fallback to plain FastAPI
        app = FastAPI(
            title="Robot Arm API",
            description="REST API for controlling a 4-DOF robot arm",
            version="1.0.0",
        )
        print("Using plain FastAPI (svc-infra not installed)")

        # Add basic health check endpoint
        @app.get("/healthz")
        async def health_check():
            return {"status": "healthy"}

    return app


def main() -> None:
    """Run the robot arm API server."""
    print("=" * 60)
    print("Robot Arm Example - REST API with svc-infra")
    print("=" * 60)

    # Create and enable the arm
    arm = create_arm()
    arm.enable()
    arm.home()
    print(f"\nCreated arm: {arm.name}")
    print(f"Initial positions: {arm.get_positions()}")

    # Create the FastAPI application
    app = create_app()

    # Generate router from the arm controller
    # auth_required=False uses public_router (no JWT required)
    # auth_required=True uses user_router (requires JWT)
    print("\n--- Generating API Router ---")
    router = controller_to_router(
        arm,
        prefix="/arm",
        tags=["Arm Control"],
        auth_required=False,  # Public access for this example
    )
    app.include_router(router)
    print("Router mounted at /arm")

    # Print available endpoints
    print("\n--- Available Endpoints ---")
    print("  GET  /arm/status     - Get arm state and positions")
    print("  POST /arm/enable     - Enable the arm")
    print("  POST /arm/disable    - Disable the arm")
    print("  POST /arm/home       - Home all joints")
    print("  POST /arm/stop       - Emergency stop")
    print("  POST /arm/move       - Move to target positions")
    print("  GET  /arm/actuators  - Get all actuator values")
    print("  GET  /arm/sensors    - Read all sensor values")

    # Example of adding authenticated endpoints (commented out)
    # For production, you would enable authentication:
    #
    # secure_router = controller_to_router(
    #     arm,
    #     prefix="/secure/arm",
    #     tags=["Secure Arm Control"],
    #     auth_required=True,  # Requires JWT token
    # )
    # app.include_router(secure_router)

    # Print startup information
    print("\n--- Starting Server ---")
    print("API server starting at http://localhost:8000")
    print("Interactive docs at http://localhost:8000/docs")
    print("Alternative docs at http://localhost:8000/redoc")
    print("Health check at http://localhost:8000/healthz")
    print("\nPress Ctrl+C to stop the server")
    print("=" * 60)

    # Run the server
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
