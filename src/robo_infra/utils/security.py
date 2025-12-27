"""
Security utilities for robo-infra.

This module provides:
- Input validation for robotics parameters
- Privilege checking for hardware access
- Sanitization utilities for external input

Example:
    >>> from robo_infra.utils.security import (
    ...     validate_joint_angle,
    ...     validate_speed,
    ...     check_gpio_access,
    ...     sanitize_name,
    ... )
    >>>
    >>> # Validate joint angle
    >>> validate_joint_angle(1.5, min_angle=-3.14, max_angle=3.14)
    True
    >>>
    >>> # Check GPIO access before using hardware
    >>> try:
    ...     check_gpio_access()
    ... except PermissionError as e:
    ...     print(f"Need permissions: {e}")
    >>>
    >>> # Sanitize user-provided names
    >>> sanitize_name("robot_1")  # Valid
    'robot_1'
    >>> sanitize_name("../etc/passwd")  # Raises ValidationError

Note:
    These utilities are designed for robotics applications where invalid
    inputs can cause physical damage or safety issues.
"""

from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Final


if TYPE_CHECKING:
    from collections.abc import Sequence


__all__ = [
    "AddressRange",
    "HardwareAccess",
    "InputValidator",
    "JointLimits",
    "PrivilegeError",
    "SpeedLimits",
    "ValidationError",
    "check_all_hardware_access",
    "check_can_access",
    "check_gpio_access",
    "check_i2c_access",
    "check_serial_access",
    "check_spi_access",
    "get_required_groups",
    "sanitize_name",
    "sanitize_serial_command",
    "validate_acceleration",
    "validate_can_id",
    "validate_i2c_address",
    "validate_joint_angle",
    "validate_joint_angles",
    "validate_port_name",
    "validate_speed",
]

# ===========================================================================
# Constants
# ===========================================================================

# Default joint limits (radians)
DEFAULT_MIN_ANGLE: Final[float] = -2 * math.pi
DEFAULT_MAX_ANGLE: Final[float] = 2 * math.pi

# Default speed limits (radians/second for rotation, m/s for linear)
DEFAULT_MIN_SPEED: Final[float] = 0.0
DEFAULT_MAX_SPEED: Final[float] = 10.0  # Conservative default

# Default acceleration limits (rad/s² or m/s²)
DEFAULT_MIN_ACCEL: Final[float] = 0.0
DEFAULT_MAX_ACCEL: Final[float] = 50.0

# I2C address range (7-bit addressing)
I2C_MIN_ADDRESS: Final[int] = 0x08  # Reserved 0x00-0x07
I2C_MAX_ADDRESS: Final[int] = 0x77  # Reserved 0x78-0x7F

# CAN ID ranges
CAN_STANDARD_MAX: Final[int] = 0x7FF  # 11-bit
CAN_EXTENDED_MAX: Final[int] = 0x1FFFFFFF  # 29-bit

# Name validation pattern (alphanumeric, underscore, hyphen)
NAME_PATTERN: Final[re.Pattern[str]] = re.compile(r"^[a-zA-Z][a-zA-Z0-9_-]*$")

# Port name patterns for different platforms
PORT_PATTERNS: Final[dict[str, re.Pattern[str]]] = {
    "linux": re.compile(r"^/dev/(tty(USB|ACM|AMA|S)[0-9]+|serial/by-id/.+)$"),
    "darwin": re.compile(r"^/dev/(tty\..*|cu\..*)$"),
    "windows": re.compile(r"^COM[0-9]+$", re.IGNORECASE),
}

# Serial command forbidden patterns (prevent injection)
SERIAL_FORBIDDEN: Final[tuple[str, ...]] = (
    "\x00",  # Null byte
    "\x03",  # ETX (Ctrl+C)
    "\x04",  # EOT (Ctrl+D)
    "\x1b",  # ESC
    "&&",  # Command chaining
    "||",  # Command chaining
    "|",  # Pipe
    ";",  # Command separator
    "`",  # Command substitution
    "$(",  # Command substitution
)


# ===========================================================================
# Exceptions
# ===========================================================================


class ValidationError(ValueError):
    """Input validation error.

    Raised when user input fails validation checks.

    Attributes:
        field: Name of the field that failed validation.
        value: The invalid value.
        constraint: Description of the constraint that was violated.
    """

    def __init__(
        self,
        message: str,
        *,
        field: str | None = None,
        value: object = None,
        constraint: str | None = None,
    ) -> None:
        super().__init__(message)
        self.field = field
        self.value = value
        self.constraint = constraint


class PrivilegeError(PermissionError):
    """Privilege/permission error with remediation instructions.

    Raised when the current user lacks permissions for hardware access.

    Attributes:
        resource: The resource that couldn't be accessed.
        required_group: The group needed for access.
        fix_command: Command to fix the permission issue.
    """

    def __init__(
        self,
        message: str,
        *,
        resource: str | None = None,
        required_group: str | None = None,
        fix_command: str | None = None,
    ) -> None:
        super().__init__(message)
        self.resource = resource
        self.required_group = required_group
        self.fix_command = fix_command


# ===========================================================================
# Data Classes
# ===========================================================================


@dataclass(frozen=True, slots=True)
class JointLimits:
    """Joint angle limits for validation.

    Attributes:
        min_angle: Minimum allowed angle in radians.
        max_angle: Maximum allowed angle in radians.
        name: Optional joint name for error messages.
    """

    min_angle: float = DEFAULT_MIN_ANGLE
    max_angle: float = DEFAULT_MAX_ANGLE
    name: str = "joint"

    def __post_init__(self) -> None:
        if self.min_angle >= self.max_angle:
            raise ValueError(
                f"min_angle ({self.min_angle}) must be less than max_angle ({self.max_angle})"
            )


@dataclass(frozen=True, slots=True)
class SpeedLimits:
    """Speed limits for validation.

    Attributes:
        min_speed: Minimum allowed speed (must be >= 0).
        max_speed: Maximum allowed speed.
        unit: Unit description for error messages.
    """

    min_speed: float = DEFAULT_MIN_SPEED
    max_speed: float = DEFAULT_MAX_SPEED
    unit: str = "rad/s"

    def __post_init__(self) -> None:
        if self.min_speed < 0:
            raise ValueError(f"min_speed must be >= 0, got {self.min_speed}")
        if self.min_speed >= self.max_speed:
            raise ValueError(
                f"min_speed ({self.min_speed}) must be less than max_speed ({self.max_speed})"
            )


@dataclass(frozen=True, slots=True)
class AddressRange:
    """Address range for I2C/CAN validation.

    Attributes:
        min_address: Minimum valid address.
        max_address: Maximum valid address.
        name: Address type name for error messages.
    """

    min_address: int = I2C_MIN_ADDRESS
    max_address: int = I2C_MAX_ADDRESS
    name: str = "I2C address"


# ===========================================================================
# Input Validation Functions
# ===========================================================================


def validate_joint_angle(
    angle: float,
    *,
    min_angle: float = DEFAULT_MIN_ANGLE,
    max_angle: float = DEFAULT_MAX_ANGLE,
    joint_name: str = "joint",
) -> bool:
    """Validate a joint angle is within limits.

    Args:
        angle: The angle to validate in radians.
        min_angle: Minimum allowed angle (default: -2π).
        max_angle: Maximum allowed angle (default: 2π).
        joint_name: Name of the joint for error messages.

    Returns:
        True if valid.

    Raises:
        ValidationError: If angle is outside limits or not a finite number.

    Example:
        >>> validate_joint_angle(1.5)
        True
        >>> validate_joint_angle(10.0)  # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ValidationError: joint angle 10.0 exceeds maximum 6.28...
    """
    if not isinstance(angle, (int, float)):
        raise ValidationError(
            f"{joint_name} angle must be a number, got {type(angle).__name__}",
            field=joint_name,
            value=angle,
            constraint="must be numeric",
        )

    if not math.isfinite(angle):
        raise ValidationError(
            f"{joint_name} angle must be finite, got {angle}",
            field=joint_name,
            value=angle,
            constraint="must be finite",
        )

    if angle < min_angle:
        raise ValidationError(
            f"{joint_name} angle {angle:.4f} below minimum {min_angle:.4f}",
            field=joint_name,
            value=angle,
            constraint=f">= {min_angle}",
        )

    if angle > max_angle:
        raise ValidationError(
            f"{joint_name} angle {angle:.4f} exceeds maximum {max_angle:.4f}",
            field=joint_name,
            value=angle,
            constraint=f"<= {max_angle}",
        )

    return True


def validate_joint_angles(
    angles: Sequence[float],
    limits: Sequence[JointLimits] | None = None,
) -> bool:
    """Validate multiple joint angles.

    Args:
        angles: Sequence of joint angles in radians.
        limits: Optional sequence of JointLimits for each joint.
                If None, uses default limits for all joints.

    Returns:
        True if all angles are valid.

    Raises:
        ValidationError: If any angle is invalid.

    Example:
        >>> validate_joint_angles([0.5, 1.0, -0.5])
        True
        >>> limits = [JointLimits(-1, 1, "j1"), JointLimits(-2, 2, "j2")]
        >>> validate_joint_angles([0.5, 1.5], limits)
        True
    """
    if limits is None:
        limits = [JointLimits(name=f"joint_{i}") for i in range(len(angles))]
    elif len(limits) != len(angles):
        raise ValidationError(
            f"Expected {len(limits)} angles, got {len(angles)}",
            field="angles",
            value=len(angles),
            constraint=f"length == {len(limits)}",
        )

    for angle, limit in zip(angles, limits, strict=True):
        validate_joint_angle(
            angle,
            min_angle=limit.min_angle,
            max_angle=limit.max_angle,
            joint_name=limit.name,
        )

    return True


def validate_speed(
    speed: float,
    *,
    min_speed: float = DEFAULT_MIN_SPEED,
    max_speed: float = DEFAULT_MAX_SPEED,
    name: str = "speed",
) -> bool:
    """Validate a speed value is within limits.

    Args:
        speed: The speed to validate.
        min_speed: Minimum allowed speed (default: 0).
        max_speed: Maximum allowed speed.
        name: Name for error messages.

    Returns:
        True if valid.

    Raises:
        ValidationError: If speed is outside limits.

    Example:
        >>> validate_speed(5.0)
        True
        >>> validate_speed(-1.0)  # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ValidationError: speed -1.0 below minimum 0.0
    """
    if not isinstance(speed, (int, float)):
        raise ValidationError(
            f"{name} must be a number, got {type(speed).__name__}",
            field=name,
            value=speed,
            constraint="must be numeric",
        )

    if not math.isfinite(speed):
        raise ValidationError(
            f"{name} must be finite, got {speed}",
            field=name,
            value=speed,
            constraint="must be finite",
        )

    if speed < min_speed:
        raise ValidationError(
            f"{name} {speed:.4f} below minimum {min_speed:.4f}",
            field=name,
            value=speed,
            constraint=f">= {min_speed}",
        )

    if speed > max_speed:
        raise ValidationError(
            f"{name} {speed:.4f} exceeds maximum {max_speed:.4f}",
            field=name,
            value=speed,
            constraint=f"<= {max_speed}",
        )

    return True


def validate_acceleration(
    accel: float,
    *,
    min_accel: float = DEFAULT_MIN_ACCEL,
    max_accel: float = DEFAULT_MAX_ACCEL,
    name: str = "acceleration",
) -> bool:
    """Validate an acceleration value is within limits.

    Args:
        accel: The acceleration to validate.
        min_accel: Minimum allowed acceleration (default: 0).
        max_accel: Maximum allowed acceleration.
        name: Name for error messages.

    Returns:
        True if valid.

    Raises:
        ValidationError: If acceleration is outside limits.
    """
    return validate_speed(
        accel,
        min_speed=min_accel,
        max_speed=max_accel,
        name=name,
    )


def validate_i2c_address(
    address: int,
    *,
    allow_reserved: bool = False,
) -> bool:
    """Validate an I2C 7-bit address.

    Args:
        address: The I2C address (0x00-0x7F).
        allow_reserved: Allow reserved addresses (0x00-0x07, 0x78-0x7F).

    Returns:
        True if valid.

    Raises:
        ValidationError: If address is invalid.

    Example:
        >>> validate_i2c_address(0x20)
        True
        >>> validate_i2c_address(0x00)  # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ValidationError: I2C address 0x00 is in reserved range
    """
    if not isinstance(address, int):
        raise ValidationError(
            f"I2C address must be an integer, got {type(address).__name__}",
            field="i2c_address",
            value=address,
            constraint="must be integer",
        )

    if address < 0 or address > 0x7F:
        raise ValidationError(
            f"I2C address 0x{address:02X} out of range (0x00-0x7F)",
            field="i2c_address",
            value=address,
            constraint="0x00 <= address <= 0x7F",
        )

    if not allow_reserved:
        if address < I2C_MIN_ADDRESS:
            raise ValidationError(
                f"I2C address 0x{address:02X} is in reserved range (0x00-0x07)",
                field="i2c_address",
                value=address,
                constraint=f">= 0x{I2C_MIN_ADDRESS:02X}",
            )
        if address > I2C_MAX_ADDRESS:
            raise ValidationError(
                f"I2C address 0x{address:02X} is in reserved range (0x78-0x7F)",
                field="i2c_address",
                value=address,
                constraint=f"<= 0x{I2C_MAX_ADDRESS:02X}",
            )

    return True


def validate_can_id(
    can_id: int,
    *,
    extended: bool = False,
) -> bool:
    """Validate a CAN bus identifier.

    Args:
        can_id: The CAN identifier.
        extended: If True, allow 29-bit extended IDs.

    Returns:
        True if valid.

    Raises:
        ValidationError: If CAN ID is invalid.

    Example:
        >>> validate_can_id(0x100)
        True
        >>> validate_can_id(0x800)  # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ValidationError: CAN ID 0x800 exceeds maximum for standard...
    """
    if not isinstance(can_id, int):
        raise ValidationError(
            f"CAN ID must be an integer, got {type(can_id).__name__}",
            field="can_id",
            value=can_id,
            constraint="must be integer",
        )

    if can_id < 0:
        raise ValidationError(
            f"CAN ID cannot be negative: {can_id}",
            field="can_id",
            value=can_id,
            constraint=">= 0",
        )

    max_id = CAN_EXTENDED_MAX if extended else CAN_STANDARD_MAX
    id_type = "extended" if extended else "standard"

    if can_id > max_id:
        raise ValidationError(
            f"CAN ID 0x{can_id:X} exceeds maximum for {id_type} (0x{max_id:X})",
            field="can_id",
            value=can_id,
            constraint=f"<= 0x{max_id:X}",
        )

    return True


def validate_port_name(
    port: str,
    *,
    platform: str | None = None,
) -> bool:
    """Validate a serial port name.

    Args:
        port: The port name to validate.
        platform: Platform name ('linux', 'darwin', 'windows').
                  Auto-detected if None.

    Returns:
        True if valid.

    Raises:
        ValidationError: If port name is invalid.

    Example:
        >>> validate_port_name("/dev/ttyUSB0", platform="linux")
        True
        >>> validate_port_name("/etc/passwd", platform="linux")
        Traceback (most recent call last):
            ...
        robo_infra.utils.security.ValidationError: Invalid...
    """
    import sys

    if platform is None:
        platform = sys.platform

    if not isinstance(port, str):
        raise ValidationError(
            f"Port name must be a string, got {type(port).__name__}",
            field="port",
            value=port,
            constraint="must be string",
        )

    if not port:
        raise ValidationError(
            "Port name cannot be empty",
            field="port",
            value=port,
            constraint="non-empty",
        )

    # Path traversal check
    if ".." in port:
        raise ValidationError(
            f"Port name cannot contain '..': {port}",
            field="port",
            value=port,
            constraint="no path traversal",
        )

    # Get pattern for platform
    pattern_key = "linux" if platform.startswith("linux") else platform
    pattern = PORT_PATTERNS.get(pattern_key)

    if pattern and not pattern.match(port):
        raise ValidationError(
            f"Invalid port name for {platform}: {port}",
            field="port",
            value=port,
            constraint=f"matches {pattern.pattern}",
        )

    return True


def sanitize_name(
    name: str,
    *,
    max_length: int = 64,
    allow_dots: bool = False,
) -> str:
    """Sanitize a user-provided name.

    Validates that the name contains only safe characters:
    - Starts with a letter
    - Contains only alphanumeric, underscore, hyphen
    - Optionally allows dots (for file extensions)

    Args:
        name: The name to sanitize.
        max_length: Maximum allowed length.
        allow_dots: Allow dots in the name.

    Returns:
        The validated name (unchanged if valid).

    Raises:
        ValidationError: If name contains invalid characters.

    Example:
        >>> sanitize_name("robot_1")
        'robot_1'
        >>> sanitize_name("my-robot")
        'my-robot'
        >>> sanitize_name("../etc/passwd")  # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ValidationError: Name contains invalid characters: ../etc/passwd
    """
    if not isinstance(name, str):
        raise ValidationError(
            f"Name must be a string, got {type(name).__name__}",
            field="name",
            value=name,
            constraint="must be string",
        )

    if not name:
        raise ValidationError(
            "Name cannot be empty",
            field="name",
            value=name,
            constraint="non-empty",
        )

    if len(name) > max_length:
        raise ValidationError(
            f"Name too long: {len(name)} > {max_length}",
            field="name",
            value=name,
            constraint=f"<= {max_length} characters",
        )

    # Use appropriate pattern
    if allow_dots:
        pattern = re.compile(r"^[a-zA-Z][a-zA-Z0-9_.-]*$")
    else:
        pattern = NAME_PATTERN

    if not pattern.match(name):
        raise ValidationError(
            f"Name contains invalid characters: {name}",
            field="name",
            value=name,
            constraint="alphanumeric, underscore, hyphen only; must start with letter",
        )

    return name


def sanitize_serial_command(
    command: str,
    *,
    max_length: int = 256,
    allowed_chars: str | None = None,
) -> str:
    """Sanitize a serial command before sending.

    Removes or rejects dangerous characters that could cause
    command injection or protocol issues.

    Args:
        command: The command to sanitize.
        max_length: Maximum allowed length.
        allowed_chars: If provided, only allow these characters.

    Returns:
        The sanitized command.

    Raises:
        ValidationError: If command contains forbidden patterns.

    Example:
        >>> sanitize_serial_command("G1 X10 Y20")
        'G1 X10 Y20'
        >>> sanitize_serial_command("echo; rm -rf /")
        Traceback (most recent call last):
            ...
        robo_infra.utils.security.ValidationError: Command contains forbidden...
    """
    if not isinstance(command, str):
        raise ValidationError(
            f"Command must be a string, got {type(command).__name__}",
            field="command",
            value=command,
            constraint="must be string",
        )

    if len(command) > max_length:
        raise ValidationError(
            f"Command too long: {len(command)} > {max_length}",
            field="command",
            value=command,
            constraint=f"<= {max_length} characters",
        )

    # Check for forbidden patterns
    for forbidden in SERIAL_FORBIDDEN:
        if forbidden in command:
            # Don't include the actual forbidden char in error (might be control char)
            raise ValidationError(
                f"Command contains forbidden pattern at position {command.index(forbidden)}",
                field="command",
                value=command,
                constraint="no shell metacharacters",
            )

    # If specific chars allowed, validate against that
    if allowed_chars is not None:
        allowed_set = set(allowed_chars)
        for i, char in enumerate(command):
            if char not in allowed_set:
                raise ValidationError(
                    f"Command contains forbidden character at position {i}: {char!r}",
                    field="command",
                    value=command,
                    constraint=f"only {allowed_chars!r}",
                )

    return command


# ===========================================================================
# Privilege Checking Functions
# ===========================================================================


class HardwareAccess(Enum):
    """Types of hardware access that may require privileges."""

    GPIO = "gpio"
    I2C = "i2c"
    SPI = "spi"
    SERIAL = "serial"
    CAN = "can"


# Group mappings for different access types (Linux)
REQUIRED_GROUPS: dict[HardwareAccess, list[str]] = {
    HardwareAccess.GPIO: ["gpio", "dialout"],
    HardwareAccess.I2C: ["i2c"],
    HardwareAccess.SPI: ["spi", "gpio"],
    HardwareAccess.SERIAL: ["dialout", "tty"],
    HardwareAccess.CAN: ["can", "dialout"],
}

# Device paths for checking access
DEVICE_PATHS: dict[HardwareAccess, list[str]] = {
    HardwareAccess.GPIO: ["/dev/gpiochip0", "/dev/gpiomem", "/sys/class/gpio/export"],
    HardwareAccess.I2C: ["/dev/i2c-0", "/dev/i2c-1"],
    HardwareAccess.SPI: ["/dev/spidev0.0", "/dev/spidev0.1"],
    HardwareAccess.SERIAL: ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyAMA0"],
    HardwareAccess.CAN: ["/dev/can0", "/dev/vcan0"],
}


def _check_device_access(
    device_paths: list[str],
    access_type: HardwareAccess,
    required_groups: list[str],
) -> None:
    """Check if any of the device paths are accessible.

    Args:
        device_paths: List of device paths to check.
        access_type: Type of hardware access for error messages.
        required_groups: Groups that provide access.

    Raises:
        PrivilegeError: If no devices are accessible.
    """
    import sys

    # Only check on Linux
    if sys.platform != "linux":
        return

    for path in device_paths:
        if os.path.exists(path) and os.access(path, os.R_OK | os.W_OK):
            return  # Found accessible device

    # None accessible - build helpful error
    groups_str = ", ".join(required_groups)
    fix_commands = [f"sudo usermod -a -G {g} $USER" for g in required_groups]
    fix_str = " && ".join(fix_commands)

    raise PrivilegeError(
        f"Cannot access {access_type.value} devices. "
        f"Add user to groups ({groups_str}): {fix_str}. "
        f"Then log out and back in.",
        resource=access_type.value,
        required_group=required_groups[0] if required_groups else None,
        fix_command=fix_str,
    )


def check_gpio_access() -> None:
    """Check if the current user can access GPIO.

    Raises:
        PrivilegeError: If GPIO is not accessible, with fix instructions.

    Example:
        >>> try:
        ...     check_gpio_access()
        ... except PrivilegeError as e:
        ...     print(f"Fix: {e.fix_command}")
    """
    _check_device_access(
        DEVICE_PATHS[HardwareAccess.GPIO],
        HardwareAccess.GPIO,
        REQUIRED_GROUPS[HardwareAccess.GPIO],
    )


def check_i2c_access(bus: int = 1) -> None:
    """Check if the current user can access I2C.

    Args:
        bus: I2C bus number to check.

    Raises:
        PrivilegeError: If I2C is not accessible, with fix instructions.
    """
    import sys

    if sys.platform != "linux":
        return

    device = f"/dev/i2c-{bus}"
    if os.path.exists(device) and not os.access(device, os.R_OK | os.W_OK):
        raise PrivilegeError(
            f"Cannot access I2C bus {bus}. Add user to 'i2c' group: sudo usermod -a -G i2c $USER",
            resource=f"i2c-{bus}",
            required_group="i2c",
            fix_command="sudo usermod -a -G i2c $USER",
        )


def check_serial_access(port: str | None = None) -> None:
    """Check if the current user can access serial ports.

    Args:
        port: Specific port to check, or None to check common ports.

    Raises:
        PrivilegeError: If serial is not accessible, with fix instructions.
    """
    import sys

    if sys.platform != "linux":
        return

    ports_to_check = [port] if port else DEVICE_PATHS[HardwareAccess.SERIAL]

    for p in ports_to_check:
        if p and os.path.exists(p) and not os.access(p, os.R_OK | os.W_OK):
            raise PrivilegeError(
                f"Cannot access serial port {p}. "
                f"Add user to 'dialout' group: sudo usermod -a -G dialout $USER",
                resource=p,
                required_group="dialout",
                fix_command="sudo usermod -a -G dialout $USER",
            )


def check_spi_access(bus: int = 0, device: int = 0) -> None:
    """Check if the current user can access SPI.

    Args:
        bus: SPI bus number.
        device: SPI device number.

    Raises:
        PrivilegeError: If SPI is not accessible, with fix instructions.
    """
    import sys

    if sys.platform != "linux":
        return

    spi_device = f"/dev/spidev{bus}.{device}"
    if os.path.exists(spi_device) and not os.access(spi_device, os.R_OK | os.W_OK):
        raise PrivilegeError(
            f"Cannot access SPI device {spi_device}. "
            f"Add user to 'spi' group: sudo usermod -a -G spi $USER",
            resource=spi_device,
            required_group="spi",
            fix_command="sudo usermod -a -G spi $USER",
        )


def check_can_access(interface: str = "can0") -> None:
    """Check if the current user can access CAN bus.

    Args:
        interface: CAN interface name.

    Raises:
        PrivilegeError: If CAN is not accessible, with fix instructions.
    """
    import sys

    if sys.platform != "linux":
        return

    # CAN uses network interfaces, check /sys
    can_path = f"/sys/class/net/{interface}"
    if os.path.exists(can_path):
        # CAN interface exists, typically needs can group or root
        can_dev = f"/dev/{interface}"
        if os.path.exists(can_dev) and not os.access(can_dev, os.R_OK | os.W_OK):
            raise PrivilegeError(
                f"Cannot access CAN interface {interface}. "
                f"Add user to 'can' group or use 'sudo ip link set {interface} up'",
                resource=interface,
                required_group="can",
                fix_command=f"sudo usermod -a -G can $USER && sudo ip link set {interface} up",
            )


def check_all_hardware_access() -> dict[HardwareAccess, bool]:
    """Check access to all hardware types.

    Returns:
        Dictionary mapping hardware type to accessibility (True = accessible).

    Example:
        >>> access = check_all_hardware_access()
        >>> for hw, ok in access.items():
        ...     print(f"{hw.value}: {'✓' if ok else '✗'}")
    """
    results: dict[HardwareAccess, bool] = {}

    for access_type in HardwareAccess:
        try:
            _check_device_access(
                DEVICE_PATHS[access_type],
                access_type,
                REQUIRED_GROUPS[access_type],
            )
            results[access_type] = True
        except PrivilegeError:
            results[access_type] = False

    return results


def get_required_groups() -> dict[str, list[HardwareAccess]]:
    """Get a mapping of groups to the hardware they provide access to.

    Returns:
        Dictionary mapping group name to list of hardware types.

    Example:
        >>> groups = get_required_groups()
        >>> print(groups['dialout'])  # doctest: +SKIP
        [<HardwareAccess.GPIO: 'gpio'>, <HardwareAccess.SERIAL: 'serial'>]
    """
    result: dict[str, list[HardwareAccess]] = {}

    for access_type, groups in REQUIRED_GROUPS.items():
        for group in groups:
            if group not in result:
                result[group] = []
            result[group].append(access_type)

    return result


# ===========================================================================
# Composite Validator
# ===========================================================================


@dataclass
class InputValidator:
    """Composite input validator with configurable limits.

    A convenience class that bundles validation functions with
    pre-configured limits for a specific robot.

    Example:
        >>> validator = InputValidator(
        ...     joint_limits=[
        ...         JointLimits(-1.57, 1.57, "shoulder"),
        ...         JointLimits(-2.0, 0.5, "elbow"),
        ...     ],
        ...     max_speed=5.0,
        ... )
        >>> validator.validate_angles([0.5, -0.3])
        True
        >>> validator.validate_speed(2.0)
        True
    """

    joint_limits: list[JointLimits] = field(default_factory=list)
    speed_limits: SpeedLimits = field(default_factory=SpeedLimits)
    accel_limits: SpeedLimits = field(
        default_factory=lambda: SpeedLimits(
            min_speed=DEFAULT_MIN_ACCEL,
            max_speed=DEFAULT_MAX_ACCEL,
            unit="rad/s²",
        )
    )
    max_speed: float = DEFAULT_MAX_SPEED
    max_accel: float = DEFAULT_MAX_ACCEL
    allowed_i2c_addresses: set[int] = field(default_factory=set)

    def validate_angle(self, joint_index: int, angle: float) -> bool:
        """Validate a single joint angle by index."""
        if joint_index < 0 or joint_index >= len(self.joint_limits):
            raise ValidationError(
                f"Invalid joint index: {joint_index}",
                field="joint_index",
                value=joint_index,
                constraint=f"0 <= index < {len(self.joint_limits)}",
            )

        limit = self.joint_limits[joint_index]
        return validate_joint_angle(
            angle,
            min_angle=limit.min_angle,
            max_angle=limit.max_angle,
            joint_name=limit.name,
        )

    def validate_angles(self, angles: Sequence[float]) -> bool:
        """Validate all joint angles."""
        return validate_joint_angles(angles, self.joint_limits or None)

    def validate_speed(self, speed: float) -> bool:
        """Validate a speed value."""
        return validate_speed(
            speed,
            min_speed=self.speed_limits.min_speed,
            max_speed=min(self.speed_limits.max_speed, self.max_speed),
        )

    def validate_accel(self, accel: float) -> bool:
        """Validate an acceleration value."""
        return validate_acceleration(
            accel,
            min_accel=self.accel_limits.min_speed,
            max_accel=min(self.accel_limits.max_speed, self.max_accel),
        )

    def validate_i2c_address(self, address: int) -> bool:
        """Validate an I2C address, optionally against allowed list."""
        validate_i2c_address(address)

        if self.allowed_i2c_addresses and address not in self.allowed_i2c_addresses:
            raise ValidationError(
                f"I2C address 0x{address:02X} not in allowed list",
                field="i2c_address",
                value=address,
                constraint=f"in {[hex(a) for a in sorted(self.allowed_i2c_addresses)]}",
            )

        return True
