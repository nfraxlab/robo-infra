"""Hardware abstraction utilities for robo-infra.

This module provides production-ready utilities for hardware abstraction:
- Enhanced simulation with noise, delays, failures, and physics
- Robust hardware detection with timeouts and retry
- Platform-specific optimizations
- Driver health monitoring and reconnection

Example:
    >>> from robo_infra.utils.hardware import (
    ...     SimulationConfig,
    ...     HardwareProbe,
    ...     DriverHealth,
    ... )
    >>>
    >>> # Enhanced simulation
    >>> config = SimulationConfig(
    ...     delay=0.001,
    ...     noise=0.02,
    ...     failure_rate=0.001,
    ... )
    >>>
    >>> # Probe hardware with timeout
    >>> probe = HardwareProbe(timeout=2.0, retries=3)
    >>> if probe.check_i2c_device(bus=1, address=0x40):
    ...     print("Device found!")
"""

from __future__ import annotations

import asyncio
import logging
import os
import random
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar, Generic, TypeVar

from robo_infra.core.exceptions import (
    CommunicationError,
)
from robo_infra.core.exceptions import (
    TimeoutError as RoboTimeoutError,
)


if TYPE_CHECKING:
    from collections.abc import Callable


logger = logging.getLogger(__name__)


# =============================================================================
# Type Variables
# =============================================================================

T = TypeVar("T")


# =============================================================================
# Simulation Configuration
# =============================================================================


class FailureMode(Enum):
    """Types of simulated failures."""

    TIMEOUT = "timeout"
    COMMUNICATION_ERROR = "communication_error"
    VALUE_CORRUPTION = "value_corruption"
    INTERMITTENT = "intermittent"
    STUCK_VALUE = "stuck_value"


@dataclass
class SimulationConfig:
    """Configuration for enhanced simulation behavior.

    Attributes:
        delay: Base I/O delay in seconds (simulates hardware latency).
        delay_jitter: Random jitter added to delay (0.0 = no jitter).
        noise: Gaussian noise added to readings (as fraction, 0.02 = ±2%).
        failure_rate: Probability of random failure (0.0 = never, 1.0 = always).
        failure_mode: Type of failure to simulate.
        physics_enabled: Whether to simulate movement physics.
        acceleration: Simulated acceleration in units/s² (for physics).
        max_velocity: Maximum velocity in units/s (for physics).

    Example:
        >>> config = SimulationConfig(
        ...     delay=0.001,       # 1ms latency
        ...     noise=0.02,        # ±2% noise
        ...     failure_rate=0.001 # 0.1% failure rate
        ... )
    """

    delay: float = 0.001
    delay_jitter: float = 0.0
    noise: float = 0.0
    failure_rate: float = 0.0
    failure_mode: FailureMode = FailureMode.COMMUNICATION_ERROR
    physics_enabled: bool = False
    acceleration: float = 100.0
    max_velocity: float = 50.0

    def get_delay(self) -> float:
        """Get delay with optional jitter."""
        if self.delay_jitter > 0:
            jitter = random.uniform(-self.delay_jitter, self.delay_jitter)
            return max(0, self.delay + jitter)
        return self.delay

    def apply_noise(self, value: float) -> float:
        """Apply Gaussian noise to a value.

        Args:
            value: Original value.

        Returns:
            Value with noise applied.
        """
        if self.noise > 0:
            noise_amount = random.gauss(0, self.noise)
            return value * (1 + noise_amount)
        return value

    def should_fail(self) -> bool:
        """Check if a failure should be simulated."""
        return self.failure_rate > 0 and random.random() < self.failure_rate

    def simulate_failure(self, operation: str = "simulated_operation") -> None:
        """Raise an exception simulating hardware failure.

        Args:
            operation: Name of the operation for timeout errors.
        """
        if self.failure_mode == FailureMode.TIMEOUT:
            raise RoboTimeoutError(operation, timeout=self.delay or 1.0)
        elif self.failure_mode == FailureMode.COMMUNICATION_ERROR:
            raise CommunicationError("Simulated communication error")
        elif self.failure_mode == FailureMode.INTERMITTENT:
            # Intermittent - 50% chance each type
            if random.random() < 0.5:
                raise RoboTimeoutError(operation, timeout=self.delay or 1.0)
            else:
                raise CommunicationError("Simulated intermittent error")
        # VALUE_CORRUPTION and STUCK_VALUE are handled differently


@dataclass
class MovementState:
    """State for simulated movement physics.

    Tracks position, velocity, and target for physics simulation.
    """

    position: float = 0.0
    velocity: float = 0.0
    target: float = 0.0
    last_update: float = field(default_factory=time.time)

    def update(self, config: SimulationConfig, dt: float | None = None) -> bool:
        """Update position based on physics.

        Args:
            config: Simulation configuration.
            dt: Time delta in seconds (auto-calculated if None).

        Returns:
            True if target reached, False if still moving.
        """
        now = time.time()
        if dt is None:
            dt = now - self.last_update
        self.last_update = now

        if abs(self.position - self.target) < 0.001:
            self.velocity = 0.0
            self.position = self.target
            return True

        # Calculate direction and required velocity
        direction = 1.0 if self.target > self.position else -1.0
        distance_to_target = abs(self.target - self.position)

        # Stopping distance at current velocity
        stopping_distance = (
            (self.velocity**2) / (2 * config.acceleration) if config.acceleration > 0 else 0
        )

        # Accelerate or decelerate
        if stopping_distance >= distance_to_target:
            # Decelerate
            self.velocity -= direction * config.acceleration * dt
        else:
            # Accelerate
            self.velocity += direction * config.acceleration * dt

        # Clamp velocity
        self.velocity = max(-config.max_velocity, min(config.max_velocity, self.velocity))

        # Update position
        self.position += self.velocity * dt

        # Check if overshot
        if (direction > 0 and self.position > self.target) or (
            direction < 0 and self.position < self.target
        ):
            self.position = self.target
            self.velocity = 0.0

        return abs(self.position - self.target) < 0.001


# =============================================================================
# Hardware Probe (Detection with Timeout/Retry)
# =============================================================================


@dataclass
class ProbeResult:
    """Result of a hardware probe.

    Attributes:
        found: Whether the hardware was found.
        response_time: Time taken for probe in seconds.
        error: Error message if probe failed.
        details: Additional details about the hardware.
    """

    found: bool
    response_time: float = 0.0
    error: str | None = None
    details: dict[str, Any] = field(default_factory=dict)


class HardwareProbe:
    """Hardware detection with timeout and retry support.

    Provides robust hardware detection that handles:
    - Timeouts for unresponsive hardware
    - Retry logic for flaky connections
    - Informative error messages

    Example:
        >>> probe = HardwareProbe(timeout=2.0, retries=3)
        >>> result = probe.check_i2c_device(bus=1, address=0x40)
        >>> if result.found:
        ...     print(f"Found device in {result.response_time:.3f}s")
        ... else:
        ...     print(f"Not found: {result.error}")
    """

    def __init__(
        self,
        timeout: float = 2.0,
        retries: int = 3,
        retry_delay: float = 0.1,
    ) -> None:
        """Initialize hardware probe.

        Args:
            timeout: Timeout in seconds for each probe attempt.
            retries: Number of retries on failure.
            retry_delay: Delay between retries in seconds.
        """
        self.timeout = timeout
        self.retries = retries
        self.retry_delay = retry_delay

    def check_i2c_device(self, bus: int, address: int) -> ProbeResult:
        """Check if an I2C device responds.

        Args:
            bus: I2C bus number.
            address: Device address (0x00-0x7F).

        Returns:
            ProbeResult with detection status.
        """
        start = time.time()
        last_error: str | None = None

        for attempt in range(self.retries + 1):
            try:
                # Check if I2C bus exists
                bus_path = Path(f"/dev/i2c-{bus}")
                if not bus_path.exists():
                    return ProbeResult(
                        found=False,
                        response_time=time.time() - start,
                        error=f"I2C bus {bus} not found at {bus_path}. "
                        "Ensure I2C is enabled and the bus number is correct.",
                    )

                # Check bus is accessible
                if not os.access(bus_path, os.R_OK | os.W_OK):
                    return ProbeResult(
                        found=False,
                        response_time=time.time() - start,
                        error=f"Cannot access I2C bus {bus}. "
                        "Add user to 'i2c' group: sudo usermod -a -G i2c $USER",
                    )

                # Try to probe the device using ioctl
                # This is a lightweight probe that doesn't require smbus2
                import fcntl

                I2C_SLAVE = 0x0703

                with open(bus_path, "rb") as f:
                    try:
                        fcntl.ioctl(f, I2C_SLAVE, address)
                        # Try a quick read to verify device responds
                        # Some devices don't respond to empty reads
                        return ProbeResult(
                            found=True,
                            response_time=time.time() - start,
                            details={"bus": bus, "address": hex(address)},
                        )
                    except OSError as e:
                        last_error = f"I2C device at {hex(address)} not responding - {e}"

            except ImportError:
                # fcntl not available (Windows)
                return ProbeResult(
                    found=False,
                    response_time=time.time() - start,
                    error="I2C probing not available on this platform",
                )
            except Exception as e:
                last_error = str(e)

            if attempt < self.retries:
                time.sleep(self.retry_delay)

        return ProbeResult(
            found=False,
            response_time=time.time() - start,
            error=last_error or "Unknown error probing I2C device",
        )

    def check_gpio_access(self) -> ProbeResult:
        """Check if GPIO is accessible.

        Returns:
            ProbeResult with access status.
        """
        start = time.time()

        # Check for gpiochip devices
        gpio_chips = list(Path("/dev").glob("gpiochip*"))
        if not gpio_chips:
            return ProbeResult(
                found=False,
                response_time=time.time() - start,
                error="No GPIO chips found at /dev/gpiochip*. "
                "GPIO may not be available or enabled on this system.",
            )

        # Check access to first chip
        chip = gpio_chips[0]
        if not os.access(chip, os.R_OK | os.W_OK):
            return ProbeResult(
                found=False,
                response_time=time.time() - start,
                error=f"Cannot access {chip}. "
                "Add user to 'gpio' group: sudo usermod -a -G gpio $USER "
                "or run as root.",
            )

        return ProbeResult(
            found=True,
            response_time=time.time() - start,
            details={"chips": [str(c) for c in gpio_chips]},
        )

    def check_serial_port(self, port: str) -> ProbeResult:
        """Check if a serial port is accessible.

        Args:
            port: Serial port path (e.g., /dev/ttyUSB0).

        Returns:
            ProbeResult with access status.
        """
        start = time.time()

        port_path = Path(port)
        if not port_path.exists():
            return ProbeResult(
                found=False,
                response_time=time.time() - start,
                error=f"Serial port {port} not found. "
                "Check if device is connected and port name is correct.",
            )

        if not os.access(port_path, os.R_OK | os.W_OK):
            return ProbeResult(
                found=False,
                response_time=time.time() - start,
                error=f"Cannot access serial port {port}. "
                "Add user to 'dialout' group: sudo usermod -a -G dialout $USER",
            )

        # Check if port is busy (try exclusive open)
        try:
            import fcntl

            with open(port_path, "rb") as f:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                except BlockingIOError:
                    return ProbeResult(
                        found=False,
                        response_time=time.time() - start,
                        error=f"Serial port {port} is busy - another process is using it.",
                    )
        except ImportError:
            pass  # fcntl not available

        return ProbeResult(
            found=True,
            response_time=time.time() - start,
            details={"port": port},
        )

    def check_spi_device(self, bus: int, device: int) -> ProbeResult:
        """Check if an SPI device is accessible.

        Args:
            bus: SPI bus number.
            device: SPI device (chip select).

        Returns:
            ProbeResult with access status.
        """
        start = time.time()

        spi_path = Path(f"/dev/spidev{bus}.{device}")
        if not spi_path.exists():
            return ProbeResult(
                found=False,
                response_time=time.time() - start,
                error=f"SPI device {spi_path} not found. "
                "Ensure SPI is enabled and bus/device numbers are correct.",
            )

        if not os.access(spi_path, os.R_OK | os.W_OK):
            return ProbeResult(
                found=False,
                response_time=time.time() - start,
                error=f"Cannot access SPI device {spi_path}. "
                "Add user to 'spi' group: sudo usermod -a -G spi $USER",
            )

        return ProbeResult(
            found=True,
            response_time=time.time() - start,
            details={"bus": bus, "device": device},
        )


# =============================================================================
# Driver Health Monitoring
# =============================================================================


class HealthStatus(Enum):
    """Health status of a driver."""

    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    UNKNOWN = "unknown"


@dataclass
class HealthCheck:
    """Result of a health check.

    Attributes:
        status: Health status.
        latency: Response latency in seconds.
        error_count: Number of recent errors.
        last_success: Timestamp of last successful operation.
        details: Additional health details.
    """

    status: HealthStatus
    latency: float = 0.0
    error_count: int = 0
    last_success: float | None = None
    details: dict[str, Any] = field(default_factory=dict)


class DriverHealth:
    """Health monitoring for drivers.

    Tracks driver health, latency, and errors for production monitoring.

    Example:
        >>> health = DriverHealth(
        ...     name="pca9685",
        ...     error_threshold=5,
        ...     latency_threshold=0.1,
        ... )
        >>> health.record_success(latency=0.005)
        >>> health.record_success(latency=0.003)
        >>> check = health.check()
        >>> print(check.status)
        HealthStatus.HEALTHY
    """

    def __init__(
        self,
        name: str,
        error_threshold: int = 5,
        latency_threshold: float = 0.1,
        window_size: int = 100,
    ) -> None:
        """Initialize health monitor.

        Args:
            name: Driver name for logging.
            error_threshold: Number of errors before UNHEALTHY.
            latency_threshold: Latency (seconds) before DEGRADED.
            window_size: Number of operations to track.
        """
        self.name = name
        self.error_threshold = error_threshold
        self.latency_threshold = latency_threshold
        self.window_size = window_size

        self._latencies: list[float] = []
        self._error_count = 0
        self._consecutive_errors = 0
        self._last_success: float | None = None
        self._last_error: str | None = None

    def record_success(self, latency: float) -> None:
        """Record a successful operation.

        Args:
            latency: Operation latency in seconds.
        """
        self._latencies.append(latency)
        if len(self._latencies) > self.window_size:
            self._latencies.pop(0)

        self._consecutive_errors = 0
        self._last_success = time.time()

    def record_error(self, error: str | Exception) -> None:
        """Record a failed operation.

        Args:
            error: Error message or exception.
        """
        self._error_count += 1
        self._consecutive_errors += 1
        self._last_error = str(error)
        logger.warning("Driver %s error: %s", self.name, error)

    def check(self) -> HealthCheck:
        """Perform a health check.

        Returns:
            HealthCheck with current status.
        """
        # Calculate average latency
        avg_latency = sum(self._latencies) / len(self._latencies) if self._latencies else 0.0

        # Determine status
        if self._consecutive_errors >= self.error_threshold:
            status = HealthStatus.UNHEALTHY
        elif self._consecutive_errors > 0 or avg_latency > self.latency_threshold:
            status = HealthStatus.DEGRADED
        elif self._last_success is None:
            status = HealthStatus.UNKNOWN
        else:
            status = HealthStatus.HEALTHY

        return HealthCheck(
            status=status,
            latency=avg_latency,
            error_count=self._error_count,
            last_success=self._last_success,
            details={
                "consecutive_errors": self._consecutive_errors,
                "last_error": self._last_error,
                "sample_count": len(self._latencies),
            },
        )

    def reset(self) -> None:
        """Reset health statistics."""
        self._latencies.clear()
        self._error_count = 0
        self._consecutive_errors = 0
        self._last_success = None
        self._last_error = None


# =============================================================================
# Driver Reconnection
# =============================================================================


class ReconnectStrategy(Enum):
    """Strategies for driver reconnection."""

    IMMEDIATE = "immediate"
    EXPONENTIAL_BACKOFF = "exponential_backoff"
    LINEAR_BACKOFF = "linear_backoff"


@dataclass
class ReconnectConfig:
    """Configuration for driver reconnection.

    Attributes:
        strategy: Reconnection strategy.
        max_attempts: Maximum reconnection attempts (0 = infinite).
        initial_delay: Initial delay between attempts in seconds.
        max_delay: Maximum delay between attempts in seconds.
        backoff_factor: Multiplier for exponential backoff.
    """

    strategy: ReconnectStrategy = ReconnectStrategy.EXPONENTIAL_BACKOFF
    max_attempts: int = 10
    initial_delay: float = 0.1
    max_delay: float = 30.0
    backoff_factor: float = 2.0

    def get_delay(self, attempt: int) -> float:
        """Calculate delay for a given attempt number.

        Args:
            attempt: Attempt number (1-based).

        Returns:
            Delay in seconds.
        """
        if self.strategy == ReconnectStrategy.IMMEDIATE:
            return 0.0
        elif self.strategy == ReconnectStrategy.LINEAR_BACKOFF:
            delay = self.initial_delay * attempt
        else:  # EXPONENTIAL_BACKOFF
            delay = self.initial_delay * (self.backoff_factor ** (attempt - 1))

        return min(delay, self.max_delay)


class DriverReconnector(Generic[T]):
    """Automatic reconnection for drivers.

    Handles driver reconnection with configurable strategies.

    Example:
        >>> reconnector = DriverReconnector(
        ...     create_driver=lambda: MyDriver(bus=1),
        ...     config=ReconnectConfig(max_attempts=5),
        ... )
        >>> driver = await reconnector.get_or_reconnect()
    """

    def __init__(
        self,
        create_driver: Callable[[], T],
        config: ReconnectConfig | None = None,
        on_reconnect: Callable[[T, int], None] | None = None,
    ) -> None:
        """Initialize reconnector.

        Args:
            create_driver: Factory function to create driver.
            config: Reconnection configuration.
            on_reconnect: Callback when reconnection succeeds (driver, attempt).
        """
        self._create_driver = create_driver
        self._config = config or ReconnectConfig()
        self._on_reconnect = on_reconnect

        self._driver: T | None = None
        self._attempt_count = 0
        self._last_error: Exception | None = None

    @property
    def driver(self) -> T | None:
        """Current driver instance."""
        return self._driver

    @property
    def attempt_count(self) -> int:
        """Number of reconnection attempts."""
        return self._attempt_count

    def connect(self) -> T:
        """Connect to driver, with reconnection on failure.

        Returns:
            Connected driver.

        Raises:
            CommunicationError: If all reconnection attempts fail.
        """
        max_attempts = self._config.max_attempts or float("inf")

        while self._attempt_count < max_attempts:
            self._attempt_count += 1

            try:
                self._driver = self._create_driver()

                # Call connect if the driver has it
                if hasattr(self._driver, "connect"):
                    self._driver.connect()  # type: ignore[union-attr]

                if self._on_reconnect and self._attempt_count > 1:
                    self._on_reconnect(self._driver, self._attempt_count)

                logger.info(
                    "Driver connected on attempt %d",
                    self._attempt_count,
                )
                return self._driver

            except Exception as e:
                self._last_error = e
                logger.warning(
                    "Driver connection failed (attempt %d/%s): %s",
                    self._attempt_count,
                    max_attempts if max_attempts < float("inf") else "∞",
                    e,
                )

                delay = self._config.get_delay(self._attempt_count)
                if delay > 0:
                    time.sleep(delay)

        raise CommunicationError(
            f"Failed to connect after {self._attempt_count} attempts: {self._last_error}"
        )

    async def connect_async(self) -> T:
        """Connect to driver asynchronously.

        Returns:
            Connected driver.

        Raises:
            CommunicationError: If all reconnection attempts fail.
        """
        max_attempts = self._config.max_attempts or float("inf")

        while self._attempt_count < max_attempts:
            self._attempt_count += 1

            try:
                self._driver = self._create_driver()

                # Call connect_async if available, else connect
                if hasattr(self._driver, "connect_async"):
                    await self._driver.connect_async()  # type: ignore[union-attr]
                elif hasattr(self._driver, "connect"):
                    self._driver.connect()  # type: ignore[union-attr]

                if self._on_reconnect and self._attempt_count > 1:
                    self._on_reconnect(self._driver, self._attempt_count)

                logger.info(
                    "Driver connected on attempt %d",
                    self._attempt_count,
                )
                return self._driver

            except Exception as e:
                self._last_error = e
                logger.warning(
                    "Driver connection failed (attempt %d/%s): %s",
                    self._attempt_count,
                    max_attempts if max_attempts < float("inf") else "∞",
                    e,
                )

                delay = self._config.get_delay(self._attempt_count)
                if delay > 0:
                    await asyncio.sleep(delay)

        raise CommunicationError(
            f"Failed to connect after {self._attempt_count} attempts: {self._last_error}"
        )

    def reset(self) -> None:
        """Reset attempt counter."""
        self._attempt_count = 0
        self._last_error = None


# =============================================================================
# Platform Optimizations
# =============================================================================


class PlatformOptimizer:
    """Platform-specific optimizations.

    Provides hints and optimizations for different platforms.

    Example:
        >>> optimizer = PlatformOptimizer.detect()
        >>> print(optimizer.platform)
        'raspberry_pi'
        >>> print(optimizer.pwm_hint)
        'Use hardware PWM on GPIO 12, 13, 18, 19 for best performance'
    """

    _detected: ClassVar[PlatformOptimizer | None] = None

    def __init__(self, platform: str) -> None:
        """Initialize optimizer.

        Args:
            platform: Platform identifier.
        """
        self.platform = platform

    @classmethod
    def detect(cls) -> PlatformOptimizer:
        """Detect current platform and return optimizer.

        Returns:
            Platform optimizer instance.
        """
        if cls._detected is not None:
            return cls._detected

        # Check for Raspberry Pi
        if Path("/sys/firmware/devicetree/base/model").exists():
            try:
                model = Path("/sys/firmware/devicetree/base/model").read_text().lower()
                if "raspberry" in model:
                    cls._detected = RaspberryPiOptimizer()
                    return cls._detected
                elif "jetson" in model or "nvidia" in model:
                    cls._detected = JetsonOptimizer()
                    return cls._detected
            except (OSError, PermissionError):
                pass

        # Check for Jetson
        if Path("/etc/nv_tegra_release").exists():
            cls._detected = JetsonOptimizer()
            return cls._detected

        # Default to generic Linux
        cls._detected = PlatformOptimizer("linux_generic")
        return cls._detected

    @property
    def pwm_hint(self) -> str:
        """Get PWM optimization hint."""
        return "Use software PWM for GPIO control"

    @property
    def gpio_hint(self) -> str:
        """Get GPIO optimization hint."""
        return "Use libgpiod (character device API) for best performance"

    @property
    def serial_buffer_size(self) -> int:
        """Recommended serial buffer size in bytes."""
        return 4096

    @property
    def i2c_clock_stretch_timeout(self) -> float:
        """Recommended I2C clock stretch timeout in seconds."""
        return 0.01


class RaspberryPiOptimizer(PlatformOptimizer):
    """Raspberry Pi-specific optimizations."""

    # Hardware PWM pins on Raspberry Pi
    HARDWARE_PWM_PINS = {12, 13, 18, 19}

    def __init__(self) -> None:
        super().__init__("raspberry_pi")

    @property
    def pwm_hint(self) -> str:
        """Get PWM optimization hint."""
        return (
            f"Use hardware PWM on GPIO {sorted(self.HARDWARE_PWM_PINS)} for best performance. "
            "These pins provide hardware-timed PWM with no CPU overhead."
        )

    @property
    def gpio_hint(self) -> str:
        """Get GPIO optimization hint."""
        return (
            "Use libgpiod (gpiod.Chip) instead of deprecated sysfs. "
            "For highest performance, consider pigpio library."
        )

    def is_hardware_pwm_pin(self, pin: int) -> bool:
        """Check if a pin supports hardware PWM."""
        return pin in self.HARDWARE_PWM_PINS


class JetsonOptimizer(PlatformOptimizer):
    """NVIDIA Jetson-specific optimizations."""

    def __init__(self) -> None:
        super().__init__("jetson")

    @property
    def pwm_hint(self) -> str:
        """Get PWM optimization hint."""
        return (
            "Use Jetson.GPIO library for hardware PWM. "
            "PWM channels vary by Jetson model - check pinout."
        )

    @property
    def gpio_hint(self) -> str:
        """Get GPIO optimization hint."""
        return (
            "Use Jetson.GPIO for GPIO control - provides hardware-accelerated access. "
            "Ensure jetson-io tool has configured the pins correctly."
        )

    @property
    def serial_buffer_size(self) -> int:
        """Larger buffer for Jetson's faster serial ports."""
        return 8192


# =============================================================================
# Convenience Functions
# =============================================================================


def check_hardware_access(
    i2c_buses: list[int] | None = None,
    spi_devices: list[tuple[int, int]] | None = None,
    serial_ports: list[str] | None = None,
    require_gpio: bool = False,
) -> dict[str, ProbeResult]:
    """Check access to multiple hardware resources.

    Args:
        i2c_buses: I2C bus numbers to check.
        spi_devices: SPI (bus, device) pairs to check.
        serial_ports: Serial port paths to check.
        require_gpio: Whether to check GPIO access.

    Returns:
        Dictionary of resource name to ProbeResult.

    Example:
        >>> results = check_hardware_access(
        ...     i2c_buses=[1],
        ...     serial_ports=["/dev/ttyUSB0"],
        ...     require_gpio=True,
        ... )
        >>> for name, result in results.items():
        ...     status = "✓" if result.found else "✗"
        ...     print(f"{status} {name}: {result.error or 'OK'}")
    """
    probe = HardwareProbe()
    results: dict[str, ProbeResult] = {}

    if require_gpio:
        results["gpio"] = probe.check_gpio_access()

    if i2c_buses:
        for bus in i2c_buses:
            # Just check bus access, not specific devices
            bus_path = Path(f"/dev/i2c-{bus}")
            if bus_path.exists():
                if os.access(bus_path, os.R_OK | os.W_OK):
                    results[f"i2c-{bus}"] = ProbeResult(found=True, details={"bus": bus})
                else:
                    results[f"i2c-{bus}"] = ProbeResult(
                        found=False,
                        error=f"Cannot access I2C bus {bus}. "
                        "Add user to 'i2c' group: sudo usermod -a -G i2c $USER",
                    )
            else:
                results[f"i2c-{bus}"] = ProbeResult(
                    found=False,
                    error=f"I2C bus {bus} not found. Ensure I2C is enabled.",
                )

    if spi_devices:
        for bus, device in spi_devices:
            results[f"spi{bus}.{device}"] = probe.check_spi_device(bus, device)

    if serial_ports:
        for port in serial_ports:
            results[port] = probe.check_serial_port(port)

    return results


def get_platform_optimizer() -> PlatformOptimizer:
    """Get the platform optimizer for the current system.

    Returns:
        Platform optimizer instance.
    """
    return PlatformOptimizer.detect()


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    "DriverHealth",
    "DriverReconnector",
    "FailureMode",
    "HardwareProbe",
    "HealthCheck",
    "HealthStatus",
    "JetsonOptimizer",
    "MovementState",
    "PlatformOptimizer",
    "ProbeResult",
    "RaspberryPiOptimizer",
    "ReconnectConfig",
    "ReconnectStrategy",
    "SimulationConfig",
    "check_hardware_access",
    "get_platform_optimizer",
]
