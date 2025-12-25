"""CLI entrypoint for robo-infra."""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from collections.abc import Sequence


# Available drivers in robo-infra
AVAILABLE_DRIVERS = [
    "arduino",
    "bmi270",
    "bno055",
    "dynamixel",
    "gpio",
    "icm20948",
    "l298n",
    "lsm6ds3",
    "odrive",
    "pca9685",
    "simulation",
    "step_dir",
    "tb6612",
    "tmc2209",
    "vesc",
]

# Available platforms in robo-infra
AVAILABLE_PLATFORMS = [
    "arduino",
    "beaglebone",
    "esp32",
    "jetson",
    "linux_generic",
    "raspberry_pi",
]


def print_help() -> None:
    """Print help message."""
    print("robo-infra - Universal Robotics Infrastructure")
    print()
    print("Commands:")
    print("  robo-infra version          - Show version")
    print("  robo-infra help             - Show this help message")
    print("  robo-infra info             - Show system information")
    print("  robo-infra list drivers     - List available drivers")
    print("  robo-infra list platforms   - List supported platforms")
    print("  robo-infra discover         - Discover connected hardware")
    print("  robo-infra test             - Run hardware tests")
    print("  robo-infra simulate         - Run in simulation mode")
    print()
    print("For programmatic use, import robo_infra in Python:")
    print()
    print("  from robo_infra import Servo, DCMotor, JointGroup")
    print("  servo = Servo(channel=0)")
    print("  servo.angle = 90")
    print()


def cmd_version() -> int:
    """Show version command."""
    from robo_infra import __version__

    print(f"robo-infra version {__version__}")
    return 0


def cmd_info() -> int:
    """Show system information."""
    from robo_infra import __version__

    print("robo-infra System Information")
    print("=" * 40)
    print(f"Version: {__version__}")
    print(f"Python: {sys.version}")
    print(f"Platform: {sys.platform}")
    print(f"Available drivers: {len(AVAILABLE_DRIVERS)}")
    print(f"Available platforms: {len(AVAILABLE_PLATFORMS)}")
    return 0


def cmd_list_drivers() -> int:
    """List available drivers."""
    print("Available Drivers:")
    print("-" * 30)
    for driver in sorted(AVAILABLE_DRIVERS):
        print(f"  - {driver}")
    print()
    print(f"Total: {len(AVAILABLE_DRIVERS)} drivers")
    return 0


def cmd_list_platforms() -> int:
    """List supported platforms."""
    print("Supported Platforms:")
    print("-" * 30)
    for platform in sorted(AVAILABLE_PLATFORMS):
        print(f"  - {platform}")
    print()
    print(f"Total: {len(AVAILABLE_PLATFORMS)} platforms")
    return 0


def cmd_discover() -> int:
    """Discover connected hardware."""
    print("Hardware discovery not yet implemented.")
    print("Coming in Phase 9!")
    return 0


def cmd_test() -> int:
    """Run hardware tests."""
    print("Hardware tests not yet implemented.")
    return 0


def cmd_simulate() -> int:
    """Run in simulation mode."""
    print("Simulation mode enabled.")
    print()
    print("In simulation mode, all hardware operations are mocked.")
    print("Use this for development and testing without real hardware.")
    print()
    print("To use simulation in your code:")
    print()
    print("  from robo_infra.drivers import SimulationDriver")
    print("  driver = SimulationDriver()")
    print()
    return 0


def main(args: Sequence[str] | None = None) -> int:
    """Main CLI entrypoint.

    Args:
        args: Command line arguments. If None, uses sys.argv[1:].

    Returns:
        Exit code (0 for success, non-zero for error).
    """
    if args is None:
        args = sys.argv[1:]

    # No arguments - show help
    if not args:
        print_help()
        return 0

    cmd = args[0]

    # Handle commands
    if cmd in ("version", "--version", "-v"):
        return cmd_version()
    elif cmd in ("help", "--help", "-h"):
        print_help()
        return 0
    elif cmd == "info":
        return cmd_info()
    elif cmd == "list":
        if len(args) < 2:
            print("Usage: robo-infra list <drivers|platforms>")
            return 1
        subcmd = args[1]
        if subcmd == "drivers":
            return cmd_list_drivers()
        elif subcmd == "platforms":
            return cmd_list_platforms()
        else:
            print(f"Unknown list command: {subcmd}")
            print("Available: drivers, platforms")
            return 1
    elif cmd == "discover":
        return cmd_discover()
    elif cmd == "test":
        return cmd_test()
    elif cmd in ("simulate", "sim"):
        return cmd_simulate()
    else:
        print(f"Unknown command: {cmd}")
        print("Run 'robo-infra help' for usage.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
