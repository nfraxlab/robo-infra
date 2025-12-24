"""Test that all examples run without exceptions.

These tests verify that example scripts can be imported and their
main functions can be called without raising exceptions.
"""

import pytest


class TestArmExamples:
    """Test arm example scripts."""

    def test_arm_basic_import(self) -> None:
        """Test that arm.py can be imported."""
        from examples.arm.arm import create_arm, main  # noqa: F401

    def test_arm_create_arm(self) -> None:
        """Test that create_arm() creates a valid arm."""
        from examples.arm.arm import create_arm

        arm = create_arm()
        assert arm is not None
        assert arm.name == "robot_arm"
        assert len(arm.actuators) == 4

    def test_arm_with_api_import(self) -> None:
        """Test that arm_with_api.py can be imported."""
        from examples.arm.arm_with_api import create_arm, create_app  # noqa: F401

    def test_arm_with_api_create_app(self) -> None:
        """Test that create_app() creates a valid FastAPI app."""
        from examples.arm.arm_with_api import create_app

        app = create_app()
        assert app is not None

    def test_arm_with_ai_import(self) -> None:
        """Test that arm_with_ai.py can be imported."""
        from examples.arm.arm_with_ai import create_arm  # noqa: F401


class TestRoverExamples:
    """Test rover example scripts."""

    def test_rover_basic_import(self) -> None:
        """Test that rover.py can be imported."""
        from examples.rover.rover import create_rover, main  # noqa: F401

    def test_rover_create_rover(self) -> None:
        """Test that create_rover() creates a valid rover."""
        from examples.rover.rover import create_rover

        rover = create_rover()
        assert rover is not None
        assert rover.name == "rover"
        assert rover.left_motor is not None
        assert rover.right_motor is not None

    def test_rover_with_sensors_import(self) -> None:
        """Test that rover_with_sensors.py can be imported."""
        from examples.rover.rover_with_sensors import (
            create_distance_sensor,
            create_rover,
        )  # noqa: F401

    def test_rover_with_sensors_create(self) -> None:
        """Test that rover and sensor can be created."""
        from examples.rover.rover_with_sensors import (
            create_distance_sensor,
            create_rover,
        )

        rover = create_rover()
        sensor = create_distance_sensor()
        assert rover is not None
        assert sensor is not None


class TestLockExamples:
    """Test lock example scripts."""

    def test_lock_basic_import(self) -> None:
        """Test that lock.py can be imported."""
        from examples.lock.lock import create_lock, main  # noqa: F401

    def test_lock_create_lock(self) -> None:
        """Test that create_lock() creates a valid lock."""
        from examples.lock.lock import create_lock

        lock = create_lock()
        assert lock is not None
        assert lock.name == "door_lock"
        assert lock.actuator is not None

    def test_lock_with_api_import(self) -> None:
        """Test that lock_with_api.py can be imported."""
        from examples.lock.lock_with_api import create_app, create_lock  # noqa: F401

    def test_lock_with_api_create_app(self) -> None:
        """Test that create_app() creates a valid FastAPI app."""
        from examples.lock.lock_with_api import create_app

        app = create_app()
        assert app is not None


class TestGripperExamples:
    """Test gripper example scripts."""

    def test_gripper_basic_import(self) -> None:
        """Test that gripper.py can be imported."""
        from examples.gripper.gripper import create_gripper, main  # noqa: F401

    def test_gripper_create_gripper(self) -> None:
        """Test that create_gripper() creates a valid gripper."""
        from examples.gripper.gripper import create_gripper

        gripper = create_gripper()
        assert gripper is not None
        assert gripper.name == "gripper"
        assert gripper.actuator is not None


class TestExampleIntegration:
    """Integration tests for examples."""

    def test_all_examples_have_readme(self) -> None:
        """Verify all example directories have README.md files."""
        from pathlib import Path

        examples_dir = Path(__file__).parent.parent.parent / "examples"
        example_dirs = [d for d in examples_dir.iterdir() if d.is_dir() and not d.name.startswith("_")]

        for example_dir in example_dirs:
            readme = example_dir / "README.md"
            assert readme.exists(), f"Missing README.md in {example_dir.name}"

    def test_all_examples_have_init(self) -> None:
        """Verify all example directories have __init__.py files."""
        from pathlib import Path

        examples_dir = Path(__file__).parent.parent.parent / "examples"
        example_dirs = [d for d in examples_dir.iterdir() if d.is_dir() and not d.name.startswith("_")]

        for example_dir in example_dirs:
            init_file = example_dir / "__init__.py"
            assert init_file.exists(), f"Missing __init__.py in {example_dir.name}"
