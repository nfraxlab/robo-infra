"""Sensor abstractions for robotics input.

This module provides abstract base classes and utilities for building
sensors like encoders, limit switches, distance sensors, IMUs, and more.

Example:
    >>> from robo_infra.core.sensor import Sensor, SimulatedSensor
    >>> from robo_infra.core.types import Limits, Unit
    >>>
    >>> # Create a simulated temperature sensor
    >>> temp_sensor = SimulatedSensor(
    ...     name="ambient_temp",
    ...     unit=Unit.CELSIUS,
    ...     limits=Limits(min=-40, max=125, default=25),
    ... )
    >>> temp_sensor.enable()
    >>> reading = temp_sensor.read()
    >>> print(f"{reading.value} {reading.unit}")
    25.0 Unit.CELSIUS
    >>>
    >>> # With a real driver
    >>> from robo_infra.core.driver import SimulatedDriver
    >>> driver = SimulatedDriver(channels=8)
    >>> driver.connect()
    >>> encoder = SimulatedSensor(
    ...     name="wheel_encoder",
    ...     driver=driver,
    ...     channel=0,
    ...     unit=Unit.DEGREES,
    ... )
"""

from __future__ import annotations

import asyncio
import logging
import statistics
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.exceptions import (
    CalibrationError,
    DisabledError,
    NotCalibratedError,
)
from robo_infra.core.types import Limits, Reading, Unit


if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Iterator

    from robo_infra.core.driver import Driver

logger = logging.getLogger(__name__)


# =============================================================================
# Enums
# =============================================================================


class SensorState(Enum):
    """Sensor operational state."""

    IDLE = "idle"
    READING = "reading"
    STREAMING = "streaming"
    ERROR = "error"
    DISABLED = "disabled"


class SensorType(Enum):
    """Common sensor types."""

    # Position/motion sensors
    ENCODER = "encoder"
    POTENTIOMETER = "potentiometer"
    IMU = "imu"
    ACCELEROMETER = "accelerometer"
    GYROSCOPE = "gyroscope"
    MAGNETOMETER = "magnetometer"

    # Distance/proximity sensors
    ULTRASONIC = "ultrasonic"
    LIDAR = "lidar"
    INFRARED = "infrared"
    TOF = "tof"  # Time of flight

    # Contact sensors
    LIMIT_SWITCH = "limit_switch"
    BUMPER = "bumper"
    PRESSURE = "pressure"
    FORCE = "force"
    TOUCH = "touch"

    # Environment sensors
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"
    LIGHT = "light"
    COLOR = "color"
    GAS = "gas"

    # Electrical sensors
    CURRENT = "current"
    VOLTAGE = "voltage"

    # Generic
    ANALOG = "analog"
    DIGITAL = "digital"
    CUSTOM = "custom"


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class SensorStatus:
    """Current status of a sensor."""

    state: SensorState
    value: float | None
    unit: Unit
    is_enabled: bool
    is_calibrated: bool
    error: str | None = None
    sample_count: int = 0
    last_read_time: float | None = None


@dataclass
class FilterConfig:
    """Configuration for sensor value filtering."""

    # Moving average
    window_size: int = 1
    # Exponential moving average (0 = disabled, 1 = only newest)
    ema_alpha: float = 0.0
    # Median filter window (0 = disabled)
    median_window: int = 0
    # Low-pass filter cutoff (Hz, 0 = disabled)
    lowpass_cutoff: float = 0.0
    # Deadband (ignore changes smaller than this)
    deadband: float = 0.0


# =============================================================================
# Pydantic Config Model
# =============================================================================


class SensorConfig(BaseModel):
    """Pydantic configuration model for sensors.

    Allows loading sensor configuration from YAML/JSON/dict.

    Example:
        >>> config = SensorConfig(
        ...     name="wheel_encoder",
        ...     sensor_type=SensorType.ENCODER,
        ...     unit=Unit.DEGREES,
        ...     limits=Limits(min=0, max=360),
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Unique sensor identifier")
    sensor_type: SensorType = Field(default=SensorType.CUSTOM, description="Type of sensor")
    unit: Unit = Field(default=Unit.RAW, description="Unit of measurement")
    limits: Limits = Field(default_factory=lambda: Limits(min=0, max=1), description="Value limits")
    channel: int = Field(default=0, ge=0, description="Driver channel number")
    sample_rate: float = Field(default=100.0, gt=0, description="Sample rate in Hz")

    # Transform pipeline
    scale: float = Field(default=1.0, description="Scale factor (applied first)")
    offset: float = Field(default=0.0, description="Offset (applied after scale)")
    inverted: bool = Field(default=False, description="Invert the value")

    # Filtering
    filter_window: int = Field(default=1, ge=1, description="Moving average window size")
    ema_alpha: float = Field(default=0.0, ge=0, le=1, description="EMA smoothing factor")

    # Calibration
    requires_calibration: bool = Field(default=False, description="Whether calibration is required")

    # Metadata
    description: str = Field(default="", description="Human-readable description")
    metadata: dict[str, Any] = Field(default_factory=dict, description="Additional metadata")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SensorConfig:
        """Create config from dictionary."""
        # Handle limits if passed as dict
        if "limits" in data and isinstance(data["limits"], dict):
            data["limits"] = Limits(**data["limits"])
        # Handle enums if passed as strings
        if "sensor_type" in data and isinstance(data["sensor_type"], str):
            data["sensor_type"] = SensorType(data["sensor_type"])
        if "unit" in data and isinstance(data["unit"], str):
            data["unit"] = Unit(data["unit"])
        return cls(**data)


# =============================================================================
# Abstract Base Class
# =============================================================================


class Sensor(ABC):
    """Abstract base class for all sensors.

    Sensors are input devices that read values from the physical world.
    They support:
    - Single reads and continuous streaming
    - Optional driver integration
    - Value transformation (scale, offset, invert)
    - Filtering (moving average, EMA, median)
    - Calibration

    Subclasses must implement:
    - `_read_raw()`: Read raw value from hardware
    - Optionally `_run_calibration()`: Perform calibration routine

    Example:
        >>> class MyEncoder(Sensor):
        ...     def _read_raw(self) -> int:
        ...         return self._driver.get_channel(self._channel)
    """

    def __init__(
        self,
        name: str,
        *,
        driver: Driver | None = None,
        channel: int = 0,
        unit: Unit = Unit.RAW,
        limits: Limits | None = None,
        scale: float = 1.0,
        offset: float = 0.0,
        inverted: bool = False,
        requires_calibration: bool = False,
        filter_config: FilterConfig | None = None,
    ) -> None:
        """Initialize sensor.

        Args:
            name: Unique sensor identifier
            driver: Optional driver for hardware communication
            channel: Driver channel number
            unit: Unit of measurement
            limits: Value limits (optional)
            scale: Scale factor applied to raw value
            offset: Offset added after scaling
            inverted: Whether to invert the transformed value
            requires_calibration: Whether calibration is required before use
            filter_config: Filtering configuration
        """
        self._name = name
        self._driver = driver
        self._channel = channel
        self._unit = unit
        self._limits = limits or Limits(min=float("-inf"), max=float("inf"))
        self._scale = scale
        self._offset = offset
        self._inverted = inverted
        self._requires_calibration = requires_calibration
        self._filter_config = filter_config or FilterConfig()

        # State
        self._state = SensorState.DISABLED
        self._is_enabled = False
        self._is_calibrated = not requires_calibration
        self._error: str | None = None
        self._last_value: float | None = None
        self._last_raw: int | None = None
        self._last_read_time: float | None = None
        self._sample_count = 0

        # Filter buffers
        self._value_buffer: deque[float] = deque(maxlen=max(1, self._filter_config.window_size))
        self._median_buffer: deque[float] = deque(
            maxlen=max(1, self._filter_config.median_window)
            if self._filter_config.median_window > 0
            else 1
        )
        self._ema_value: float | None = None
        self._last_filtered_value: float | None = None

        # Calibration data
        self._calibration_offset: float = 0.0
        self._calibration_scale: float = 1.0

        logger.debug("Sensor '%s' initialized", name)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def name(self) -> str:
        """Sensor name."""
        return self._name

    @property
    def driver(self) -> Driver | None:
        """Associated driver, if any."""
        return self._driver

    @property
    def channel(self) -> int:
        """Driver channel number."""
        return self._channel

    @property
    def unit(self) -> Unit:
        """Unit of measurement."""
        return self._unit

    @property
    def limits(self) -> Limits:
        """Value limits."""
        return self._limits

    @property
    def is_enabled(self) -> bool:
        """Whether sensor is enabled."""
        return self._is_enabled

    @property
    def is_calibrated(self) -> bool:
        """Whether sensor is calibrated."""
        return self._is_calibrated

    @property
    def state(self) -> SensorState:
        """Current sensor state."""
        return self._state

    @property
    def last_value(self) -> float | None:
        """Last read value (transformed and filtered)."""
        return self._last_value

    @property
    def last_raw(self) -> int | None:
        """Last raw reading."""
        return self._last_raw

    @property
    def sample_count(self) -> int:
        """Number of samples taken since enable."""
        return self._sample_count

    # -------------------------------------------------------------------------
    # Abstract Methods
    # -------------------------------------------------------------------------

    @abstractmethod
    def _read_raw(self) -> int:
        """Read raw value from hardware.

        This must be implemented by subclasses.

        Returns:
            Raw integer value from sensor/ADC.
        """

    def _run_calibration(self) -> None:  # noqa: B027
        """Run calibration routine.

        Override this in subclasses that support calibration.
        Default implementation does nothing.
        """

    # -------------------------------------------------------------------------
    # Enable/Disable
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Enable the sensor for reading."""
        if self._is_enabled:
            return

        self._is_enabled = True
        self._state = SensorState.IDLE
        self._error = None
        self._sample_count = 0
        self._clear_filter_buffers()
        logger.debug("Sensor '%s' enabled", self._name)

    def disable(self) -> None:
        """Disable the sensor."""
        if not self._is_enabled:
            return

        self._is_enabled = False
        self._state = SensorState.DISABLED
        logger.debug("Sensor '%s' disabled", self._name)

    # -------------------------------------------------------------------------
    # Reading
    # -------------------------------------------------------------------------

    def read(self) -> Reading:
        """Read current value from sensor.

        Returns:
            Reading with transformed and filtered value.

        Raises:
            DisabledError: If sensor is disabled.
            NotCalibratedError: If calibration is required but not done.
        """
        if not self._is_enabled:
            raise DisabledError(self._name)

        if self._requires_calibration and not self._is_calibrated:
            raise NotCalibratedError(self._name)

        try:
            self._state = SensorState.READING

            # Read raw value
            raw = self._read_raw()
            self._last_raw = raw
            self._last_read_time = time.time()

            # Transform: raw -> scaled -> offset -> calibration -> invert
            value = float(raw) * self._scale
            value += self._offset
            value = value * self._calibration_scale + self._calibration_offset
            if self._inverted:
                # Invert around the midpoint of limits
                mid = (self._limits.min + self._limits.max) / 2
                value = 2 * mid - value

            # Apply filtering
            value = self._apply_filters(value)

            # Clamp to limits
            value = max(self._limits.min, min(self._limits.max, value))

            self._last_value = value
            self._sample_count += 1
            self._state = SensorState.IDLE

            return Reading(
                value=value,
                unit=self._unit,
                timestamp=self._last_read_time,
                raw=raw,
            )

        except Exception as e:
            self._state = SensorState.ERROR
            self._error = str(e)
            raise

    def read_raw(self) -> int:
        """Read raw value without transformation or filtering.

        Returns:
            Raw integer value from sensor.

        Raises:
            DisabledError: If sensor is disabled.
        """
        if not self._is_enabled:
            raise DisabledError(self._name)

        raw = self._read_raw()
        self._last_raw = raw
        return raw

    async def stream(
        self,
        rate: float = 100.0,
        count: int | None = None,
        duration: float | None = None,
        *,
        max_samples: int = 1_000_000,  # Safety limit: 1M samples max
    ) -> AsyncIterator[Reading]:
        """Stream readings asynchronously.

        Args:
            rate: Sample rate in Hz (default 100)
            count: Stop after this many samples (None = use max_samples)
            duration: Stop after this many seconds (None = no time limit)
            max_samples: Safety limit - maximum samples before stopping (default 1M)

        Yields:
            Reading objects at the specified rate.

        Raises:
            DisabledError: If sensor is disabled.

        Note:
            For infinite streaming, explicitly set max_samples to a very high value
            or use duration to limit by time.
        """
        if not self._is_enabled:
            raise DisabledError(self._name)

        # Determine actual count limit
        effective_count = count if count is not None else max_samples

        self._state = SensorState.STREAMING
        interval = 1.0 / rate
        start_time = time.time()
        samples = 0

        try:
            while samples < effective_count:
                # Check time limit
                if duration is not None and (time.time() - start_time) >= duration:
                    break

                yield self.read()
                samples += 1

                await asyncio.sleep(interval)

            if samples >= effective_count and count is None:
                logger.warning(
                    "Sensor stream reached max_samples limit (%d) - stopping",
                    max_samples,
                )
        finally:
            self._state = SensorState.IDLE

    # -------------------------------------------------------------------------
    # Filtering
    # -------------------------------------------------------------------------

    def _apply_filters(self, value: float) -> float:
        """Apply configured filters to value.

        Args:
            value: Raw transformed value

        Returns:
            Filtered value
        """
        cfg = self._filter_config

        # Deadband filter
        if (
            cfg.deadband > 0
            and self._last_filtered_value is not None
            and abs(value - self._last_filtered_value) < cfg.deadband
        ):
            return self._last_filtered_value

        # Moving average
        if cfg.window_size > 1:
            self._value_buffer.append(value)
            value = statistics.mean(self._value_buffer)

        # Median filter
        if cfg.median_window > 0:
            self._median_buffer.append(value)
            value = statistics.median(self._median_buffer)

        # Exponential moving average
        if cfg.ema_alpha > 0:
            if self._ema_value is None:
                self._ema_value = value
            else:
                self._ema_value = cfg.ema_alpha * value + (1 - cfg.ema_alpha) * self._ema_value
            value = self._ema_value

        self._last_filtered_value = value
        return value

    def _clear_filter_buffers(self) -> None:
        """Clear all filter buffers."""
        self._value_buffer.clear()
        self._median_buffer.clear()
        self._ema_value = None
        self._last_filtered_value = None

    def reset_filters(self) -> None:
        """Reset filter state (useful when starting a new measurement)."""
        self._clear_filter_buffers()

    # -------------------------------------------------------------------------
    # Calibration
    # -------------------------------------------------------------------------

    def calibrate(
        self, *, zero_point: float | None = None, reference_value: float | None = None
    ) -> None:
        """Calibrate the sensor.

        Args:
            zero_point: Set current reading as this value (offset calibration)
            reference_value: If provided with zero_point, calculate scale

        Raises:
            CalibrationError: If calibration fails.
            DisabledError: If sensor is disabled.
        """
        if not self._is_enabled:
            raise DisabledError(self._name)

        try:
            # Run hardware calibration routine if defined
            self._run_calibration()

            # Simple offset calibration: make current value equal zero_point
            if zero_point is not None:
                current = self.read().value
                self._calibration_offset = zero_point - current

                # Scale calibration if reference provided
                if reference_value is not None and reference_value != zero_point:
                    # Read again after offset
                    new_current = self.read().value
                    if new_current != zero_point:
                        self._calibration_scale = (reference_value - zero_point) / (
                            new_current - zero_point
                        )

            self._is_calibrated = True
            logger.info(
                "Sensor '%s' calibrated (offset=%.4f, scale=%.4f)",
                self._name,
                self._calibration_offset,
                self._calibration_scale,
            )

        except Exception as e:
            self._is_calibrated = False
            raise CalibrationError(self._name, str(e)) from e

    def reset_calibration(self) -> None:
        """Reset calibration to defaults."""
        self._calibration_offset = 0.0
        self._calibration_scale = 1.0
        self._is_calibrated = not self._requires_calibration

    # -------------------------------------------------------------------------
    # Status and Info
    # -------------------------------------------------------------------------

    def status(self) -> SensorStatus:
        """Get current sensor status.

        Returns:
            SensorStatus with current state information.
        """
        return SensorStatus(
            state=self._state,
            value=self._last_value,
            unit=self._unit,
            is_enabled=self._is_enabled,
            is_calibrated=self._is_calibrated,
            error=self._error,
            sample_count=self._sample_count,
            last_read_time=self._last_read_time,
        )

    def __repr__(self) -> str:
        """String representation."""
        return f"<{self.__class__.__name__} name='{self._name}' unit={self._unit} enabled={self._is_enabled}>"

    # -------------------------------------------------------------------------
    # Context Manager
    # -------------------------------------------------------------------------

    def __enter__(self) -> Sensor:
        """Enter context manager - enable sensor."""
        self.enable()
        return self

    def __exit__(self, exc_type: type | None, exc_val: Exception | None, exc_tb: Any) -> None:
        """Exit context manager - disable sensor."""
        self.disable()


# =============================================================================
# Simulated Implementation
# =============================================================================


class SimulatedSensor(Sensor):
    """Simulated sensor for testing and development.

    Provides a full sensor implementation that can be used without hardware.
    Supports optional noise simulation and value patterns.

    Example:
        >>> sensor = SimulatedSensor(
        ...     name="test_encoder",
        ...     unit=Unit.DEGREES,
        ...     limits=Limits(min=0, max=360),
        ... )
        >>> sensor.enable()
        >>> reading = sensor.read()
        >>> print(reading.value)
        180.0
    """

    def __init__(
        self,
        name: str,
        *,
        driver: Driver | None = None,
        channel: int = 0,
        unit: Unit = Unit.RAW,
        limits: Limits | None = None,
        scale: float = 1.0,
        offset: float = 0.0,
        inverted: bool = False,
        requires_calibration: bool = False,
        filter_config: FilterConfig | None = None,
        # Simulation-specific
        initial_value: float | None = None,
        noise: float = 0.0,
        drift: float = 0.0,
    ) -> None:
        """Initialize simulated sensor.

        Args:
            name: Sensor name
            driver: Optional driver
            channel: Driver channel
            unit: Unit of measurement
            limits: Value limits
            scale: Scale factor
            offset: Offset value
            inverted: Invert output
            requires_calibration: Require calibration
            filter_config: Filter configuration
            initial_value: Starting value (default: middle of limits or 0)
            noise: Random noise amplitude (±noise)
            drift: Value drift per read (cumulative)
        """
        # Set limits before calling super().__init__
        self._sim_limits = limits or Limits(min=0, max=1000)

        super().__init__(
            name,
            driver=driver,
            channel=channel,
            unit=unit,
            limits=self._sim_limits,
            scale=scale,
            offset=offset,
            inverted=inverted,
            requires_calibration=requires_calibration,
            filter_config=filter_config,
        )

        # Simulation state
        if initial_value is not None:
            self._simulated_value = initial_value
        elif self._sim_limits.default is not None:
            self._simulated_value = self._sim_limits.default
        else:
            self._simulated_value = (self._sim_limits.min + self._sim_limits.max) / 2

        self._noise = noise
        self._drift = drift
        self._drift_accumulator = 0.0

    def _read_raw(self) -> int:
        """Read simulated raw value.

        If a driver is connected, reads from the driver channel.
        Otherwise returns the simulated value with optional noise and drift.
        """
        if self._driver is not None:
            return int(self._driver.get_channel(self._channel))

        import random

        value = self._simulated_value

        # Add drift
        if self._drift != 0:
            self._drift_accumulator += self._drift
            value += self._drift_accumulator

        # Add noise
        if self._noise > 0:
            value += random.uniform(-self._noise, self._noise)

        return int(value)

    def set_simulated_value(self, value: float) -> None:
        """Set the simulated sensor value.

        Args:
            value: Value to simulate
        """
        self._simulated_value = value
        self._drift_accumulator = 0.0

    def reset_drift(self) -> None:
        """Reset drift accumulator."""
        self._drift_accumulator = 0.0

    @classmethod
    def from_config(cls, config: SensorConfig, driver: Driver | None = None) -> SimulatedSensor:
        """Create simulated sensor from configuration.

        Args:
            config: Sensor configuration
            driver: Optional driver to use

        Returns:
            Configured SimulatedSensor instance
        """
        return cls(
            name=config.name,
            driver=driver,
            channel=config.channel,
            unit=config.unit,
            limits=config.limits,
            scale=config.scale,
            offset=config.offset,
            inverted=config.inverted,
            requires_calibration=config.requires_calibration,
            filter_config=FilterConfig(
                window_size=config.filter_window,
                ema_alpha=config.ema_alpha,
            ),
        )


# =============================================================================
# Sensor Group
# =============================================================================


class SensorGroup:
    """Group of sensors for coordinated reading.

    Allows reading multiple sensors together and provides
    aggregate operations.

    Example:
        >>> group = SensorGroup("arm_sensors")
        >>> group.add("encoder1", encoder1)
        >>> group.add("encoder2", encoder2)
        >>> group.add("limit_switch", limit_sw)
        >>> with group:
        ...     readings = group.read_all()
        >>> print(readings)
        {'encoder1': Reading(...), 'encoder2': Reading(...), 'limit_switch': Reading(...)}
    """

    def __init__(self, name: str) -> None:
        """Initialize sensor group.

        Args:
            name: Group name
        """
        self.name = name
        self.sensors: dict[str, Sensor] = {}

    def add(self, name: str, sensor: Sensor) -> None:
        """Add sensor to group.

        Args:
            name: Name to use in group (can differ from sensor.name)
            sensor: Sensor instance
        """
        self.sensors[name] = sensor

    def remove(self, name: str) -> Sensor | None:
        """Remove sensor from group.

        Args:
            name: Sensor name in group

        Returns:
            Removed sensor or None if not found
        """
        return self.sensors.pop(name, None)

    def get(self, name: str) -> Sensor | None:
        """Get sensor by name.

        Args:
            name: Sensor name

        Returns:
            Sensor or None if not found
        """
        return self.sensors.get(name)

    def enable_all(self) -> None:
        """Enable all sensors in group."""
        for sensor in self.sensors.values():
            sensor.enable()

    def disable_all(self) -> None:
        """Disable all sensors in group."""
        for sensor in self.sensors.values():
            sensor.disable()

    def read_all(self) -> dict[str, Reading]:
        """Read all sensors.

        Returns:
            Dictionary mapping sensor names to readings.
            Disabled sensors are skipped.
        """
        readings = {}
        for name, sensor in self.sensors.items():
            if sensor.is_enabled:
                try:
                    readings[name] = sensor.read()
                except Exception as e:
                    logger.warning("Failed to read sensor '%s': %s", name, e)
        return readings

    def read_all_raw(self) -> dict[str, int]:
        """Read raw values from all sensors.

        Returns:
            Dictionary mapping sensor names to raw values.
        """
        values = {}
        for name, sensor in self.sensors.items():
            if sensor.is_enabled:
                try:
                    values[name] = sensor.read_raw()
                except Exception as e:
                    logger.warning("Failed to read sensor '%s': %s", name, e)
        return values

    def status_all(self) -> dict[str, SensorStatus]:
        """Get status of all sensors.

        Returns:
            Dictionary mapping sensor names to status.
        """
        return {name: sensor.status() for name, sensor in self.sensors.items()}

    def calibrate_all(self) -> dict[str, bool]:
        """Calibrate all sensors.

        Returns:
            Dictionary mapping sensor names to success status.
        """
        results = {}
        for name, sensor in self.sensors.items():
            try:
                sensor.calibrate()
                results[name] = True
            except Exception as e:
                logger.warning("Failed to calibrate sensor '%s': %s", name, e)
                results[name] = False
        return results

    def reset_all_filters(self) -> None:
        """Reset filters on all sensors."""
        for sensor in self.sensors.values():
            sensor.reset_filters()

    def __len__(self) -> int:
        """Number of sensors in group."""
        return len(self.sensors)

    def __iter__(self) -> Iterator[str]:
        """Iterate over sensor names."""
        return iter(self.sensors)

    def __contains__(self, name: str) -> bool:
        """Check if sensor exists in group."""
        return name in self.sensors

    def __enter__(self) -> SensorGroup:
        """Enter context - enable all sensors."""
        self.enable_all()
        return self

    def __exit__(self, exc_type: type | None, exc_val: Exception | None, exc_tb: Any) -> None:
        """Exit context - disable all sensors."""
        self.disable_all()


# =============================================================================
# Reading Utilities
# =============================================================================


class ReadingBuffer:
    """Buffer for accumulating and analyzing readings.

    Useful for calculating statistics over a series of readings.

    Example:
        >>> buffer = ReadingBuffer(max_size=100)
        >>> for _ in range(10):
        ...     buffer.add(sensor.read())
        >>> print(buffer.mean())
        25.5
    """

    def __init__(self, max_size: int = 1000) -> None:
        """Initialize reading buffer.

        Args:
            max_size: Maximum number of readings to store
        """
        self._readings: deque[Reading] = deque(maxlen=max_size)
        self._max_size = max_size

    def add(self, reading: Reading) -> None:
        """Add a reading to the buffer.

        Args:
            reading: Reading to add
        """
        self._readings.append(reading)

    def clear(self) -> None:
        """Clear all readings."""
        self._readings.clear()

    @property
    def values(self) -> list[float]:
        """List of all values."""
        return [r.value for r in self._readings]

    @property
    def timestamps(self) -> list[float]:
        """List of all timestamps."""
        return [r.timestamp for r in self._readings]

    def mean(self) -> float:
        """Calculate mean of values."""
        if not self._readings:
            return 0.0
        return statistics.mean(self.values)

    def median(self) -> float:
        """Calculate median of values."""
        if not self._readings:
            return 0.0
        return statistics.median(self.values)

    def std_dev(self) -> float:
        """Calculate standard deviation."""
        if len(self._readings) < 2:
            return 0.0
        return statistics.stdev(self.values)

    def variance(self) -> float:
        """Calculate variance."""
        if len(self._readings) < 2:
            return 0.0
        return statistics.variance(self.values)

    def min(self) -> float:
        """Get minimum value."""
        if not self._readings:
            return 0.0
        return min(self.values)

    def max(self) -> float:
        """Get maximum value."""
        if not self._readings:
            return 0.0
        return max(self.values)

    def range(self) -> float:
        """Get range (max - min)."""
        return self.max() - self.min()

    def rate(self) -> float:
        """Calculate sample rate in Hz."""
        if len(self._readings) < 2:
            return 0.0
        duration = self._readings[-1].timestamp - self._readings[0].timestamp
        if duration <= 0:
            return 0.0
        return (len(self._readings) - 1) / duration

    def latest(self) -> Reading | None:
        """Get most recent reading."""
        return self._readings[-1] if self._readings else None

    def oldest(self) -> Reading | None:
        """Get oldest reading."""
        return self._readings[0] if self._readings else None

    def __len__(self) -> int:
        """Number of readings in buffer."""
        return len(self._readings)

    def __iter__(self) -> Iterator[Reading]:
        """Iterate over readings."""
        return iter(self._readings)


# =============================================================================
# Factory Functions
# =============================================================================


def create_sensor(
    name: str,
    sensor_type: SensorType = SensorType.CUSTOM,
    *,
    driver: Driver | None = None,
    channel: int = 0,
    unit: Unit = Unit.RAW,
    limits: Limits | None = None,
    **kwargs: Any,
) -> SimulatedSensor:
    """Factory function to create a sensor.

    Args:
        name: Sensor name
        sensor_type: Type of sensor
        driver: Optional driver
        channel: Driver channel
        unit: Unit of measurement
        limits: Value limits
        **kwargs: Additional arguments passed to SimulatedSensor

    Returns:
        Configured SimulatedSensor instance
    """
    return SimulatedSensor(
        name=name,
        driver=driver,
        channel=channel,
        unit=unit,
        limits=limits,
        **kwargs,
    )


def create_encoder(
    name: str,
    *,
    driver: Driver | None = None,
    channel: int = 0,
    resolution: int = 360,
    **kwargs: Any,
) -> SimulatedSensor:
    """Create an encoder sensor.

    Args:
        name: Sensor name
        driver: Optional driver
        channel: Driver channel
        resolution: Encoder resolution (counts per revolution)
        **kwargs: Additional arguments

    Returns:
        Encoder sensor
    """
    return SimulatedSensor(
        name=name,
        driver=driver,
        channel=channel,
        unit=Unit.DEGREES,
        limits=Limits(min=0, max=360, default=0),
        scale=360.0 / resolution,
        **kwargs,
    )


def create_temperature_sensor(
    name: str,
    *,
    driver: Driver | None = None,
    channel: int = 0,
    min_temp: float = -40.0,
    max_temp: float = 125.0,
    **kwargs: Any,
) -> SimulatedSensor:
    """Create a temperature sensor.

    Args:
        name: Sensor name
        driver: Optional driver
        channel: Driver channel
        min_temp: Minimum temperature
        max_temp: Maximum temperature
        **kwargs: Additional arguments

    Returns:
        Temperature sensor
    """
    return SimulatedSensor(
        name=name,
        driver=driver,
        channel=channel,
        unit=Unit.CELSIUS,
        limits=Limits(min=min_temp, max=max_temp, default=25.0),
        **kwargs,
    )


def create_distance_sensor(
    name: str,
    *,
    driver: Driver | None = None,
    channel: int = 0,
    max_distance: float = 4000.0,  # 4 meters in mm
    **kwargs: Any,
) -> SimulatedSensor:
    """Create a distance sensor.

    Args:
        name: Sensor name
        driver: Optional driver
        channel: Driver channel
        max_distance: Maximum distance in mm
        **kwargs: Additional arguments

    Returns:
        Distance sensor
    """
    return SimulatedSensor(
        name=name,
        driver=driver,
        channel=channel,
        unit=Unit.MILLIMETERS,
        limits=Limits(min=0, max=max_distance, default=0),
        **kwargs,
    )


def create_limit_switch(
    name: str,
    *,
    driver: Driver | None = None,
    channel: int = 0,
    normally_open: bool = True,
    **kwargs: Any,
) -> SimulatedSensor:
    """Create a limit switch sensor.

    Args:
        name: Sensor name
        driver: Optional driver
        channel: Driver channel
        normally_open: True if switch is normally open
        **kwargs: Additional arguments

    Returns:
        Limit switch sensor (0 = open, 1 = closed)
    """
    return SimulatedSensor(
        name=name,
        driver=driver,
        channel=channel,
        unit=Unit.RAW,
        limits=Limits(min=0, max=1, default=0 if normally_open else 1),
        inverted=not normally_open,
        **kwargs,
    )
