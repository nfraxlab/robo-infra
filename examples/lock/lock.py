#!/usr/bin/env python3
"""Basic servo-based smart lock control.

This example demonstrates controlling a lock using robo-infra:
- Creating a servo actuator for the lock mechanism
- Building a Lock controller with locked/unlocked positions
- Lock, unlock, and toggle operations
- State checking and async operations

Run:
    python lock.py
"""

import asyncio

from robo_infra.actuators.servo import Servo
from robo_infra.controllers.lock import Lock, LockConfig


def create_lock() -> Lock:
    """Create a servo-based smart lock.

    Returns:
        Configured Lock controller.
    """
    # Create a simulated servo actuator for the lock
    # In simulation mode, no driver/pins needed
    servo = Servo(name="lock_servo", angle_range=(0, 90))

    # Configure the lock with positions and timing
    config = LockConfig(
        name="door_lock",
        description="Electronic door lock with servo mechanism",
        locked_position=0.0,  # Fully locked at 0 degrees
        unlocked_position=90.0,  # Fully unlocked at 90 degrees
        transition_time=0.5,  # 500ms to transition
        position_tolerance=1.0,  # 1 degree tolerance for state detection
        start_locked=True,  # Start in locked state
    )

    # Create the lock controller
    lock = Lock(
        name="door_lock",
        actuator=servo,
        config=config,
    )

    return lock


async def async_operations_demo(lock: Lock) -> None:
    """Demonstrate async lock operations with proper timing.

    Args:
        lock: The lock controller.
    """
    print("\n--- Async Operations Demo ---")

    # Async unlock with transition timing
    print("\nAsync unlocking (with transition time)...")
    await lock.aunlock()
    print(f"Lock state: {lock.state_str}")
    print(f"Position: {lock.position:.1f}°")

    # Wait a moment
    await asyncio.sleep(0.3)

    # Async lock
    print("\nAsync locking (with transition time)...")
    await lock.alock()
    print(f"Lock state: {lock.state_str}")
    print(f"Position: {lock.position:.1f}°")


def main() -> None:
    """Demonstrate basic lock control operations."""
    print("=" * 60)
    print("Smart Lock Example - Servo-Based Lock Control")
    print("=" * 60)

    # Create the lock
    lock = create_lock()
    print(f"\nCreated lock: {lock.name}")
    print(f"Actuator: {lock.actuator.name}")
    print(f"Locked position: {lock.lock_config.locked_position}°")
    print(f"Unlocked position: {lock.lock_config.unlocked_position}°")

    # Enable the lock and its actuator
    print("\n--- Enabling lock ---")
    lock.actuator.enable()
    lock.enable()
    print(f"Lock enabled: {lock.is_enabled}")

    # Check initial state (starts locked based on config)
    print("\n--- Initial State ---")
    print(f"Lock state: {lock.state_str}")
    print(f"Is locked: {lock.is_locked}")
    print(f"Is unlocked: {lock.is_unlocked}")
    print(f"Position: {lock.position:.1f}°")

    # Lock operation (should already be locked, but this ensures state)
    print("\n--- Locking ---")
    lock.lock()
    print(f"Lock state: {lock.state_str}")
    print(f"Is locked: {lock.is_locked}")
    print(f"Position: {lock.position:.1f}°")

    # Unlock operation
    print("\n--- Unlocking ---")
    lock.unlock()
    print(f"Lock state: {lock.state_str}")
    print(f"Is unlocked: {lock.is_unlocked}")
    print(f"Position: {lock.position:.1f}°")

    # Toggle operation (unlocked -> locked)
    print("\n--- Toggle (unlock -> lock) ---")
    lock.toggle()
    print(f"Lock state: {lock.state_str}")
    print(f"Is locked: {lock.is_locked}")
    print(f"Position: {lock.position:.1f}°")

    # Toggle again (locked -> unlocked)
    print("\n--- Toggle (lock -> unlock) ---")
    lock.toggle()
    print(f"Lock state: {lock.state_str}")
    print(f"Is unlocked: {lock.is_unlocked}")
    print(f"Position: {lock.position:.1f}°")

    # Get status
    print("\n--- Lock Status ---")
    status = lock.status()
    print(f"Name: {lock.name}")
    print(f"State: {status.state}")
    print(f"Enabled: {status.is_enabled}")

    # Run async demo
    print("\n--- Running Async Demo ---")
    asyncio.run(async_operations_demo(lock))

    # Disable when done
    print("\n--- Disabling lock ---")
    lock.disable()
    lock.actuator.disable()
    print(f"Lock disabled: {not lock.is_enabled}")

    print("\n" + "=" * 60)
    print("Example complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
