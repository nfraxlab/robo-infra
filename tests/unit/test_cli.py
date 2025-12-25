"""Unit tests for CLI module.

Tests for the robo-infra command-line interface.
"""

from __future__ import annotations

import sys
from io import StringIO
from unittest.mock import patch

import pytest

from robo_infra.cli import (
    AVAILABLE_DRIVERS,
    AVAILABLE_PLATFORMS,
    cmd_discover,
    cmd_info,
    cmd_list_drivers,
    cmd_list_platforms,
    cmd_simulate,
    cmd_test,
    cmd_version,
    main,
    print_help,
)


class TestPrintHelp:
    """Tests for print_help function."""

    def test_print_help_shows_title(self) -> None:
        """Test help shows robo-infra title."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            print_help()
            output = mock_stdout.getvalue()
            assert "robo-infra" in output
            assert "Universal Robotics Infrastructure" in output

    def test_print_help_shows_commands(self) -> None:
        """Test help shows available commands."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            print_help()
            output = mock_stdout.getvalue()
            assert "version" in output
            assert "help" in output
            assert "info" in output
            assert "list drivers" in output
            assert "list platforms" in output
            assert "discover" in output
            assert "simulate" in output

    def test_print_help_shows_usage_example(self) -> None:
        """Test help shows Python usage example."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            print_help()
            output = mock_stdout.getvalue()
            assert "from robo_infra import" in output
            assert "Servo" in output


class TestCmdVersion:
    """Tests for cmd_version function."""

    def test_cmd_version_returns_zero(self) -> None:
        """Test version command returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = cmd_version()
            assert result == 0

    def test_cmd_version_shows_version_string(self) -> None:
        """Test version command shows version string."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_version()
            output = mock_stdout.getvalue()
            assert "robo-infra version" in output

    def test_cmd_version_format(self) -> None:
        """Test version output format."""
        from robo_infra import __version__

        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_version()
            output = mock_stdout.getvalue()
            assert __version__ in output


class TestCmdInfo:
    """Tests for cmd_info function."""

    def test_cmd_info_returns_zero(self) -> None:
        """Test info command returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = cmd_info()
            assert result == 0

    def test_cmd_info_shows_system_info(self) -> None:
        """Test info command shows system information."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_info()
            output = mock_stdout.getvalue()
            assert "System Information" in output
            assert "Version:" in output
            assert "Python:" in output
            assert "Platform:" in output

    def test_cmd_info_shows_driver_count(self) -> None:
        """Test info shows number of available drivers."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_info()
            output = mock_stdout.getvalue()
            assert "Available drivers:" in output
            assert str(len(AVAILABLE_DRIVERS)) in output

    def test_cmd_info_shows_platform_count(self) -> None:
        """Test info shows number of available platforms."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_info()
            output = mock_stdout.getvalue()
            assert "Available platforms:" in output
            assert str(len(AVAILABLE_PLATFORMS)) in output


class TestCmdListDrivers:
    """Tests for cmd_list_drivers function."""

    def test_cmd_list_drivers_returns_zero(self) -> None:
        """Test list drivers command returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = cmd_list_drivers()
            assert result == 0

    def test_cmd_list_drivers_shows_header(self) -> None:
        """Test list drivers shows header."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_drivers()
            output = mock_stdout.getvalue()
            assert "Available Drivers:" in output

    def test_cmd_list_drivers_shows_all_drivers(self) -> None:
        """Test list drivers shows all available drivers."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_drivers()
            output = mock_stdout.getvalue()
            for driver in AVAILABLE_DRIVERS:
                assert driver in output

    def test_cmd_list_drivers_shows_total(self) -> None:
        """Test list drivers shows total count."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_drivers()
            output = mock_stdout.getvalue()
            assert f"Total: {len(AVAILABLE_DRIVERS)} drivers" in output


class TestCmdListPlatforms:
    """Tests for cmd_list_platforms function."""

    def test_cmd_list_platforms_returns_zero(self) -> None:
        """Test list platforms command returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = cmd_list_platforms()
            assert result == 0

    def test_cmd_list_platforms_shows_header(self) -> None:
        """Test list platforms shows header."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_platforms()
            output = mock_stdout.getvalue()
            assert "Supported Platforms:" in output

    def test_cmd_list_platforms_shows_all_platforms(self) -> None:
        """Test list platforms shows all supported platforms."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_platforms()
            output = mock_stdout.getvalue()
            for platform in AVAILABLE_PLATFORMS:
                assert platform in output

    def test_cmd_list_platforms_shows_total(self) -> None:
        """Test list platforms shows total count."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_platforms()
            output = mock_stdout.getvalue()
            assert f"Total: {len(AVAILABLE_PLATFORMS)} platforms" in output


class TestCmdDiscover:
    """Tests for cmd_discover function."""

    def test_cmd_discover_returns_zero(self) -> None:
        """Test discover command returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = cmd_discover()
            assert result == 0

    def test_cmd_discover_shows_not_implemented(self) -> None:
        """Test discover shows not implemented message."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_discover()
            output = mock_stdout.getvalue()
            assert "not yet implemented" in output


class TestCmdTest:
    """Tests for cmd_test function."""

    def test_cmd_test_returns_zero(self) -> None:
        """Test test command returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = cmd_test()
            assert result == 0

    def test_cmd_test_shows_not_implemented(self) -> None:
        """Test test command shows not implemented message."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_test()
            output = mock_stdout.getvalue()
            assert "not yet implemented" in output


class TestCmdSimulate:
    """Tests for cmd_simulate function."""

    def test_cmd_simulate_returns_zero(self) -> None:
        """Test simulate command returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = cmd_simulate()
            assert result == 0

    def test_cmd_simulate_shows_simulation_message(self) -> None:
        """Test simulate shows simulation mode message."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_simulate()
            output = mock_stdout.getvalue()
            assert "Simulation mode enabled" in output

    def test_cmd_simulate_shows_usage(self) -> None:
        """Test simulate shows usage example."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_simulate()
            output = mock_stdout.getvalue()
            assert "SimulationDriver" in output


class TestMainNoArgs:
    """Tests for main function with no arguments."""

    def test_main_no_args_returns_zero(self) -> None:
        """Test main with no args returns 0."""
        with patch("sys.stdout", new_callable=StringIO):
            result = main([])
            assert result == 0

    def test_main_no_args_shows_help(self) -> None:
        """Test main with no args shows help."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main([])
            output = mock_stdout.getvalue()
            assert "robo-infra" in output
            assert "Commands:" in output


class TestMainVersion:
    """Tests for main function version command."""

    def test_main_version_command(self) -> None:
        """Test main with version command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["version"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "robo-infra version" in output

    def test_main_version_flag(self) -> None:
        """Test main with --version flag."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["--version"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "version" in output

    def test_main_version_short_flag(self) -> None:
        """Test main with -v flag."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["-v"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "version" in output


class TestMainHelp:
    """Tests for main function help command."""

    def test_main_help_command(self) -> None:
        """Test main with help command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["help"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "Commands:" in output

    def test_main_help_flag(self) -> None:
        """Test main with --help flag."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["--help"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "Commands:" in output

    def test_main_help_short_flag(self) -> None:
        """Test main with -h flag."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["-h"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "Commands:" in output


class TestMainInfo:
    """Tests for main function info command."""

    def test_main_info_command(self) -> None:
        """Test main with info command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["info"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "System Information" in output


class TestMainListDrivers:
    """Tests for main function list drivers command."""

    def test_main_list_drivers(self) -> None:
        """Test main with list drivers command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["list", "drivers"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "Available Drivers:" in output

    def test_main_list_no_subcommand(self) -> None:
        """Test main with list but no subcommand returns error."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["list"])
            assert result == 1
            output = mock_stdout.getvalue()
            assert "Usage:" in output


class TestMainListPlatforms:
    """Tests for main function list platforms command."""

    def test_main_list_platforms(self) -> None:
        """Test main with list platforms command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["list", "platforms"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "Supported Platforms:" in output

    def test_main_list_unknown_subcommand(self) -> None:
        """Test main with list unknown subcommand returns error."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["list", "unknown"])
            assert result == 1
            output = mock_stdout.getvalue()
            assert "Unknown list command" in output


class TestMainSimulate:
    """Tests for main function simulate command."""

    def test_main_simulate_command(self) -> None:
        """Test main with simulate command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["simulate"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "Simulation mode enabled" in output

    def test_main_sim_short_command(self) -> None:
        """Test main with sim short command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["sim"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "Simulation mode enabled" in output


class TestMainDiscover:
    """Tests for main function discover command."""

    def test_main_discover_command(self) -> None:
        """Test main with discover command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["discover"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "not yet implemented" in output


class TestMainTest:
    """Tests for main function test command."""

    def test_main_test_command(self) -> None:
        """Test main with test command."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            result = main(["test"])
            assert result == 0
            output = mock_stdout.getvalue()
            assert "not yet implemented" in output


class TestMainUnknownCommand:
    """Tests for main function with unknown commands."""

    def test_main_unknown_command_returns_error(self) -> None:
        """Test main with unknown command returns non-zero."""
        with patch("sys.stdout", new_callable=StringIO):
            result = main(["nonexistent"])
            assert result == 1

    def test_main_unknown_command_shows_message(self) -> None:
        """Test main with unknown command shows error message."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main(["foobar"])
            output = mock_stdout.getvalue()
            assert "Unknown command: foobar" in output
            assert "robo-infra help" in output


class TestMainDefaultArgs:
    """Tests for main function using sys.argv."""

    def test_main_uses_sysargv_when_no_args(self) -> None:
        """Test main uses sys.argv when args is None."""
        original_argv = sys.argv
        try:
            sys.argv = ["robo-infra", "version"]
            with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
                result = main(None)
                assert result == 0
                output = mock_stdout.getvalue()
                assert "version" in output
        finally:
            sys.argv = original_argv


class TestAvailableDrivers:
    """Tests for AVAILABLE_DRIVERS list."""

    def test_drivers_list_not_empty(self) -> None:
        """Test drivers list is not empty."""
        assert len(AVAILABLE_DRIVERS) > 0

    def test_drivers_contains_expected(self) -> None:
        """Test drivers list contains expected drivers."""
        expected = ["arduino", "pca9685", "tmc2209", "dynamixel", "simulation"]
        for driver in expected:
            assert driver in AVAILABLE_DRIVERS

    def test_drivers_are_sorted(self) -> None:
        """Test drivers are listed in sorted order when displayed."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_drivers()
            output = mock_stdout.getvalue()
            lines = [l.strip() for l in output.split("\n") if l.strip().startswith("-")]
            drivers_in_output = [l.replace("- ", "") for l in lines]
            assert drivers_in_output == sorted(drivers_in_output)


class TestAvailablePlatforms:
    """Tests for AVAILABLE_PLATFORMS list."""

    def test_platforms_list_not_empty(self) -> None:
        """Test platforms list is not empty."""
        assert len(AVAILABLE_PLATFORMS) > 0

    def test_platforms_contains_expected(self) -> None:
        """Test platforms list contains expected platforms."""
        expected = ["raspberry_pi", "jetson", "arduino", "esp32"]
        for platform in expected:
            assert platform in AVAILABLE_PLATFORMS

    def test_platforms_are_sorted(self) -> None:
        """Test platforms are listed in sorted order when displayed."""
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            cmd_list_platforms()
            output = mock_stdout.getvalue()
            lines = [l.strip() for l in output.split("\n") if l.strip().startswith("-")]
            platforms_in_output = [l.replace("- ", "") for l in lines]
            assert platforms_in_output == sorted(platforms_in_output)


class TestCLIIntegration:
    """Integration tests for CLI as a whole."""

    def test_full_workflow_info_then_list(self) -> None:
        """Test running multiple commands in sequence."""
        with patch("sys.stdout", new_callable=StringIO):
            result1 = main(["info"])
            result2 = main(["list", "drivers"])
            result3 = main(["list", "platforms"])
            assert result1 == 0
            assert result2 == 0
            assert result3 == 0

    def test_cli_module_callable(self) -> None:
        """Test CLI module can be called."""
        from robo_infra import cli

        assert hasattr(cli, "main")
        assert callable(cli.main)

    def test_all_success_commands_return_zero(self) -> None:
        """Test all valid commands return 0."""
        commands = [
            [],
            ["help"],
            ["--help"],
            ["-h"],
            ["version"],
            ["--version"],
            ["-v"],
            ["info"],
            ["list", "drivers"],
            ["list", "platforms"],
            ["discover"],
            ["test"],
            ["simulate"],
            ["sim"],
        ]
        with patch("sys.stdout", new_callable=StringIO):
            for cmd in commands:
                result = main(cmd)
                assert result == 0, f"Command {cmd} should return 0"

    def test_all_error_commands_return_nonzero(self) -> None:
        """Test invalid commands return non-zero."""
        commands = [
            ["unknown"],
            ["list"],
            ["list", "unknown"],
            ["xyz123"],
        ]
        with patch("sys.stdout", new_callable=StringIO):
            for cmd in commands:
                result = main(cmd)
                assert result != 0, f"Command {cmd} should return non-zero"
