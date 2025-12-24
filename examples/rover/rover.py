#!/usr/bin/env python3
"""Basic differential drive rover control.

This example demonstrates controlling a wheeled rover using robo-infra:
- Creating DC motor actuators for left and right wheels
- Building a DifferentialDrive controller
- Driving forward, backward, turning, and spinning
- Getting speed and position information

Run:
    python rover.py
"""

from robo_infra.actuators.dc_motor import DCMotor
from robo_infra.controllers.differential import DifferentialDrive, DifferentialDriveConfig


def create_rover() -> DifferentialDrive:
    """Create a differential drive rover with simulated DC motors.

    Returns:
        Configured and enabled DifferentialDrive controller.
    """
    # Create DC motor actuators for each wheel
    # In simulation mode, no driver/pins needed - motors operate internally
    left_motor = DCMotor(name="left_wheel")
    right_motor = DCMotor(name="right_wheel")

    # Configure the rover with physical dimensions
    config = DifferentialDriveConfig(
        name="rover",
        description="Example wheeled rover with differential drive",
        wheel_diameter=0.065,  # 65mm wheels
        track_width=0.15,  # 150mm between wheels
        max_speed=1.0,  # 1 m/s max speed
        max_angular_speed=3.14,  # ~180 deg/s max turn rate
    )

    # Create the rover controller
    rover = DifferentialDrive(
        name="rover",
        left=left_motor,
        right=right_motor,
        config=config,
    )

    return rover


def main() -> None:
    """Demonstrate basic rover control operations."""
    print("=" * 60)
    print("Rover Example - Differential Drive Control")
    print("=" * 60)

    # Create the rover
    rover = create_rover()
    print(f"\nCreated rover: {rover.name}")
    print(f"Left motor: {rover.left_motor.name}")
    print(f"Right motor: {rover.right_motor.name}")

    # Enable the rover
    print("\n--- Enabling rover ---")
    rover.enable()
    print(f"Rover enabled: {rover.is_enabled}")

    # Drive forward
    print("\n--- Driving forward ---")
    rover.forward(speed=0.5)
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Driving forward at 50% speed...")

    # Drive backward (reverse)
    print("\n--- Driving backward ---")
    rover.reverse(speed=0.3)
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Driving backward at 30% speed...")

    # Turn left (arc turn)
    print("\n--- Turning left (arc) ---")
    rover.turn_left(speed=0.4)
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Turning left with arc...")

    # Turn right (arc turn)
    print("\n--- Turning right (arc) ---")
    rover.turn_right(speed=0.4)
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Turning right with arc...")

    # Spin left (in place, counter-clockwise)
    print("\n--- Spinning left (in place) ---")
    rover.spin(speed=0.5, clockwise=False)
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Spinning counter-clockwise...")

    # Spin right (in place, clockwise)
    print("\n--- Spinning right (in place) ---")
    rover.spin(speed=0.5, clockwise=True)
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Spinning clockwise...")

    # Tank drive (direct wheel control)
    print("\n--- Tank drive (forward + turn right) ---")
    rover.tank(left_speed=0.6, right_speed=0.3)  # Left faster = turns right
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Driving forward while turning right...")

    # Arc movement
    print("\n--- Arc movement (1m radius left turn) ---")
    rover.arc(speed=0.5, radius=1.0)  # Positive radius = turn left
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Following arc with 1m radius...")

    # Stop
    print("\n--- Stopping ---")
    rover.stop()
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
    print("Stopped!")

    # Get status
    print("\n--- Rover Status ---")
    status = rover.status()
    print(f"Name: {rover.name}")
    print(f"State: {status.state}")
    print(f"Enabled: {status.is_enabled}")

    # Disable when done
    print("\n--- Disabling rover ---")
    rover.disable()
    print(f"Rover disabled: {not rover.is_enabled}")

    print("\n" + "=" * 60)
    print("Example complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
