#!/usr/bin/env python3
"""Basic servo gripper control.

This example demonstrates controlling a gripper using robo-infra:
- Creating a servo actuator for the gripper mechanism
- Building a Gripper controller with open/closed positions
- Open, close, and partial position control
- State checking and grip detection

Run:
    python gripper.py
"""

from robo_infra.actuators.servo import Servo
from robo_infra.controllers.gripper import Gripper, GripperConfig


def create_gripper() -> Gripper:
    """Create a servo-based gripper.

    Returns:
        Configured Gripper controller.
    """
    # Create a simulated servo actuator for the gripper
    # In simulation mode, no driver/pins needed
    servo = Servo(name="gripper_servo", angle_range=(0, 90))

    # Configure the gripper with positions
    config = GripperConfig(
        name="parallel_gripper",
        description="Parallel jaw gripper for pick-and-place",
        open_position=0.0,  # Fully open at 0 degrees
        closed_position=90.0,  # Fully closed at 90 degrees
        grip_threshold=1.0,  # 1 degree tolerance for state detection
        default_speed=1.0,  # Full speed for normal operations
        grip_speed=0.5,  # Half speed when gripping (for force sensing)
    )

    # Create the gripper controller
    gripper = Gripper(
        name="gripper",
        actuator=servo,
        config=config,
    )

    return gripper


def main() -> None:
    """Demonstrate basic gripper control operations."""
    print("=" * 60)
    print("Gripper Example - Servo-Based Gripper Control")
    print("=" * 60)

    # Create the gripper
    gripper = create_gripper()
    print(f"\nCreated gripper: {gripper.name}")
    print(f"Actuator: {gripper.actuator.name}")
    print(f"Open position: {gripper.gripper_config.open_position}°")
    print(f"Closed position: {gripper.gripper_config.closed_position}°")

    # Enable the gripper and its actuator
    print("\n--- Enabling gripper ---")
    gripper.actuator.enable()
    gripper.enable()
    print(f"Gripper enabled: {gripper.is_enabled}")

    # Check initial state
    print("\n--- Initial State ---")
    print(f"Gripper state: {gripper.gripper_state.value}")
    print(f"Position: {gripper.position:.1f}°")
    print(f"Is open: {gripper.is_open}")
    print(f"Is closed: {gripper.is_closed}")

    # Open the gripper
    print("\n--- Opening gripper ---")
    gripper.open()
    print(f"Gripper state: {gripper.gripper_state.value}")
    print(f"Position: {gripper.position:.1f}°")
    print(f"Is open: {gripper.is_open}")

    # Close the gripper
    print("\n--- Closing gripper ---")
    gripper.close()
    print(f"Gripper state: {gripper.gripper_state.value}")
    print(f"Position: {gripper.position:.1f}°")
    print(f"Is closed: {gripper.is_closed}")

    # Partial position control
    print("\n--- Partial positions ---")

    # 25% closed (25% of 90 = 22.5°)
    position_25 = gripper.gripper_config.open_position + (
        gripper.gripper_config.range * 0.25
    )
    gripper.set(position_25)
    print(f"Set to 25%: position = {gripper.position:.1f}°")

    # 50% closed (50% of 90 = 45°)
    position_50 = gripper.gripper_config.open_position + (
        gripper.gripper_config.range * 0.50
    )
    gripper.set(position_50)
    print(f"Set to 50%: position = {gripper.position:.1f}°")

    # 75% closed (75% of 90 = 67.5°)
    position_75 = gripper.gripper_config.open_position + (
        gripper.gripper_config.range * 0.75
    )
    gripper.set(position_75)
    print(f"Set to 75%: position = {gripper.position:.1f}°")

    # Open again for next demo
    print("\n--- Pick and place demo ---")
    gripper.open()
    print("1. Gripper opened, ready to pick")

    # Simulate gripping an object
    gripper.close()
    print("2. Gripper closed, object gripped")
    print(f"   Is closed: {gripper.is_closed}")

    # Simulate placing the object
    gripper.open()
    print("3. Gripper opened, object released")
    print(f"   Is open: {gripper.is_open}")

    # Get status
    print("\n--- Gripper Status ---")
    status = gripper.status()
    print(f"Name: {gripper.name}")
    print(f"State: {status.state}")
    print(f"Enabled: {status.is_enabled}")

    # Disable when done
    print("\n--- Disabling gripper ---")
    gripper.disable()
    gripper.actuator.disable()
    print(f"Gripper disabled: {not gripper.is_enabled}")

    print("\n" + "=" * 60)
    print("Example complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
