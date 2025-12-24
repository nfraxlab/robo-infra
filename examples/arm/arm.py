#!/usr/bin/env python3
"""Basic 4-DOF robot arm control.

This example demonstrates basic robot arm control using robo-infra:
- Creating simulated servo actuators
- Building a JointGroup controller
- Moving to specific positions
- Using named positions
- Homing the arm

Run:
    python arm.py
"""

from robo_infra.controllers.joint_group import JointGroup, JointGroupConfig
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits


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


def main() -> None:
    """Demonstrate basic arm control operations."""
    print("=" * 60)
    print("Robot Arm Example - Basic Control")
    print("=" * 60)

    # Create the arm
    arm = create_arm()
    print(f"\nCreated arm: {arm.name}")
    print(f"Joints: {list(arm.actuators.keys())}")

    # Enable the arm
    print("\n--- Enabling arm ---")
    arm.enable()
    print(f"Arm enabled: {arm.is_enabled}")

    # Home all joints
    print("\n--- Homing arm ---")
    arm.home()
    positions = arm.get_positions()
    print(f"Home positions: {positions}")

    # Move to specific positions
    print("\n--- Moving to custom position ---")
    target = {"base": 270, "shoulder": 45, "elbow": 90, "wrist": 135}
    print(f"Target: {target}")
    arm.move_joints(target)
    positions = arm.get_positions()
    print(f"Current positions: {positions}")

    # Move a single joint
    print("\n--- Moving single joint ---")
    arm.move_joints({"shoulder": 60})
    print(f"Shoulder moved to: {arm.get_positions()['shoulder']}")

    # Use named positions
    print("\n--- Moving to named position 'ready' ---")
    arm.go_to_named("ready")
    positions = arm.get_positions()
    print(f"Ready position: {positions}")

    print("\n--- Moving to named position 'pickup' ---")
    arm.go_to_named("pickup")
    positions = arm.get_positions()
    print(f"Pickup position: {positions}")

    # Save current position as a new named position
    print("\n--- Saving current position as 'custom' ---")
    arm.move_joints({"base": 120, "shoulder": 70, "elbow": 100, "wrist": 60})
    arm.save_position("custom")
    print(f"Saved position 'custom': {arm.get_positions()}")

    # Return to home
    print("\n--- Returning to home ---")
    arm.home()
    print(f"Final positions: {arm.get_positions()}")

    # Get status
    print("\n--- Arm Status ---")
    status = arm.status()
    print(f"Name: {arm.name}")
    print(f"State: {status.state}")
    print(f"Enabled: {status.is_enabled}")
    print(f"Homed: {status.is_homed}")

    # Disable when done
    print("\n--- Disabling arm ---")
    arm.disable()
    print(f"Arm disabled: {not arm.is_enabled}")

    print("\n" + "=" * 60)
    print("Example complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
