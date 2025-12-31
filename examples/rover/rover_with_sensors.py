#!/usr/bin/env python3
"""Rover with ultrasonic distance sensor for obstacle avoidance.

This example demonstrates adding sensors to a rover:
- Creating an ultrasonic distance sensor
- Reading distance measurements
- Simple obstacle avoidance behavior
- Sensor status monitoring

Run:
    python rover_with_sensors.py
"""

from robo_infra.actuators.dc_motor import DCMotor
from robo_infra.controllers.differential import DifferentialDrive, DifferentialDriveConfig
from robo_infra.core.sensor import SimulatedSensor, Unit
from robo_infra.core.types import Limits


def create_rover() -> DifferentialDrive:
    """Create a differential drive rover with simulated DC motors.

    Returns:
        Configured and enabled DifferentialDrive controller.
    """
    # Create DC motor actuators for each wheel
    left_motor = DCMotor(name="left_wheel")
    right_motor = DCMotor(name="right_wheel")

    # Configure the rover with physical dimensions
    config = DifferentialDriveConfig(
        name="rover",
        description="Rover with obstacle avoidance",
        wheel_diameter=0.065,  # 65mm wheels
        track_width=0.15,  # 150mm between wheels
        max_speed=1.0,  # 1 m/s max speed
    )

    # Create the rover controller
    rover = DifferentialDrive(
        name="rover",
        left=left_motor,
        right=right_motor,
        config=config,
    )

    return rover


def create_distance_sensor() -> SimulatedSensor:
    """Create a simulated ultrasonic distance sensor.

    Returns:
        Configured SimulatedSensor for distance measurement.
    """
    # Simulated ultrasonic sensor (2cm to 4m range)
    sensor = SimulatedSensor(
        name="front_ultrasonic",
        unit=Unit.MILLIMETERS,
        limits=Limits(min=20, max=4000, default=1000),  # 2cm to 4m range
        noise=5.0,  # ±5mm measurement noise
    )

    return sensor


def obstacle_avoidance_demo(
    rover: DifferentialDrive,
    sensor: SimulatedSensor,
) -> None:
    """Demonstrate simple obstacle avoidance behavior.

    Args:
        rover: The rover controller.
        sensor: The distance sensor.
    """
    print("\n--- Obstacle Avoidance Demo ---")

    # Define safety threshold (in mm)
    MIN_DISTANCE_MM = 300.0  # 30cm minimum safe distance

    # Simulate different scenarios
    scenarios = [
        ("Clear path", 1500.0),  # 1.5m - clear ahead
        ("Obstacle nearby", 250.0),  # 25cm - obstacle detected
        ("Very close", 100.0),  # 10cm - very close
        ("Medium distance", 500.0),  # 50cm - getting close
        ("Far away", 3000.0),  # 3m - plenty of room
    ]

    for scenario_name, simulated_distance in scenarios:
        # Set the simulated sensor value
        sensor.set_simulated_value(simulated_distance)

        # Read the distance (includes noise)
        reading = sensor.read()
        distance = reading.value

        print(f"\nScenario: {scenario_name}")
        print(f"  Distance: {distance:.0f}mm ({distance/10:.1f}cm)")

        # Decision logic
        if distance < MIN_DISTANCE_MM:
            # Obstacle detected - stop and turn
            print(f"  [!]  Obstacle detected! Distance < {MIN_DISTANCE_MM}mm")
            rover.stop()
            print("  Action: STOP, then spin right to avoid")
            rover.spin(speed=0.5, clockwise=True)
            left_speed, right_speed = rover.current_speed
            print(f"  Motor speeds - L: {left_speed:.2f}, R: {right_speed:.2f}")
        elif distance < MIN_DISTANCE_MM * 2:
            # Getting close - slow down
            print(f"   Getting close, slowing down")
            rover.forward(speed=0.3)
            left_speed, right_speed = rover.current_speed
            print(f"  Motor speeds - L: {left_speed:.2f}, R: {right_speed:.2f}")
        else:
            # Clear ahead - full speed
            print(f"  [OK] Clear ahead, full speed")
            rover.forward(speed=0.7)
            left_speed, right_speed = rover.current_speed
            print(f"  Motor speeds - L: {left_speed:.2f}, R: {right_speed:.2f}")

    # Stop at the end
    rover.stop()


def main() -> None:
    """Demonstrate rover with sensor integration."""
    print("=" * 60)
    print("Rover Example - With Distance Sensor")
    print("=" * 60)

    # Create the rover
    rover = create_rover()
    rover.enable()
    print(f"\nCreated rover: {rover.name}")

    # Create the distance sensor
    sensor = create_distance_sensor()
    sensor.enable()
    print(f"Created sensor: {sensor.name}")
    print(f"  Unit: {sensor.unit.value}")
    print(f"  Range: {sensor.limits.min}mm - {sensor.limits.max}mm")

    # Read initial distance
    print("\n--- Sensor Reading ---")
    reading = sensor.read()
    print(f"Distance: {reading.value:.0f}mm")
    print(f"Timestamp: {reading.timestamp}")

    # Run obstacle avoidance demo
    obstacle_avoidance_demo(rover, sensor)

    # Final status
    print("\n--- Final Status ---")
    print(f"Rover enabled: {rover.is_enabled}")
    print(f"Sensor enabled: {sensor.is_enabled}")
    left_speed, right_speed = rover.current_speed
    print(f"Motor speeds - L: {left_speed:.2f}, R: {right_speed:.2f}")

    # Disable when done
    print("\n--- Cleanup ---")
    rover.disable()
    sensor.disable()
    print("Rover and sensor disabled")

    print("\n" + "=" * 60)
    print("Example complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
