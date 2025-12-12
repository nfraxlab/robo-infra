"""CLI entrypoint for robo-infra."""

from __future__ import annotations

import sys


def main() -> int:
    """Main CLI entrypoint."""
    print("robo-infra - Universal Robotics Infrastructure")
    print()
    print("Commands:")
    print("  robo-infra discover   - Discover connected hardware")
    print("  robo-infra test       - Run hardware tests")
    print("  robo-infra version    - Show version")
    print()
    print("For programmatic use, import robo_infra in Python:")
    print()
    print("  from robo_infra import Servo, DCMotor, JointGroup")
    print("  servo = Servo(channel=0)")
    print("  servo.angle = 90")
    print()

    if len(sys.argv) > 1:
        cmd = sys.argv[1]
        if cmd == "version":
            from robo_infra import __version__

            print(f"robo-infra version {__version__}")
            return 0
        elif cmd == "discover":
            print("Hardware discovery not yet implemented.")
            print("Coming in Phase 9!")
            return 0
        elif cmd == "test":
            print("Hardware tests not yet implemented.")
            return 0
        else:
            print(f"Unknown command: {cmd}")
            return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
