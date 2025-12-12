"""IMU (Inertial Measurement Unit) sensor implementations.

Phase 4.2 provides IMU sensors:
- Accelerometer (linear acceleration in g)
- Gyroscope (angular velocity in °/s)
- Magnetometer (magnetic field strength, compass heading)
- Combined IMU (fusion of accelerometer, gyroscope, magnetometer)

All sensors extend `Sensor` via a shared `IMUSensor` base class.

Notes:
- The core `Sensor` abstraction expects `_read_raw() -> int`.
- IMU sensors return `Vector3` data via convenience methods.
- `read()` returns a scalar `Reading` (magnitude for IMU sensors).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.exceptions import CalibrationError, CommunicationError
from robo_infra.core.sensor import Sensor
from robo_infra.core.types import Limits, Unit, Vector3


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus
    from robo_infra.core.driver import Driver


# -----------------------------------------------------------------------------
# Base class
# -----------------------------------------------------------------------------


class IMUSensor(Sensor):
    """Base class for IMU sensors.

    IMU sensors read 3-axis data (X, Y, Z) and provide:
    - `read_vector()`: Returns Vector3 with scaled values
    - `read()`: Returns Reading with magnitude as value
    """

    def read_vector(self) -> Vector3:
        """Read 3-axis data as a Vector3.

        Subclasses should override this with their specific implementation.
        Default implementation returns the raw values as a vector.
        """
        raw = self._read_raw()
        return Vector3(x=float(raw), y=0.0, z=0.0)


# -----------------------------------------------------------------------------
# Accelerometer
# -----------------------------------------------------------------------------


class AccelerometerConfig(BaseModel):
    """Configuration for accelerometers."""

    name: str = "Accelerometer"
    unit: Unit = Unit.METERS_PER_SECOND  # Actually g, but using m/s² compatible

    # I2C settings
    i2c_address: int = Field(default=0x68, description="I2C address")
    data_register: int = Field(default=0x00, description="Data register start address")

    # Range and resolution
    range_g: float = Field(default=2.0, description="Full scale range in g (±)")
    resolution_bits: int = Field(default=16, description="ADC resolution")

    # Offsets for calibration
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}

    @property
    def scale_factor(self) -> float:
        """Calculate scale factor from raw to g."""
        max_raw = (1 << (self.resolution_bits - 1)) - 1
        return self.range_g / max_raw


@dataclass
class AccelerometerReading:
    """Reading from an accelerometer."""

    vector: Vector3
    magnitude: float
    unit: Unit
    timestamp: float
    raw: tuple[int, int, int]


class Accelerometer(IMUSensor):
    """3-axis accelerometer sensor.

    Measures linear acceleration in g (1g ≈ 9.81 m/s²).

    Can be backed by:
    - I2C bus (typical: MPU6050, ADXL345, LIS3DH)
    - Generic driver (for ADC-based accelerometers)

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> accel = Accelerometer(bus=bus, config=AccelerometerConfig())
        >>> accel.enable()
        >>> reading = accel.read_acceleration()
        >>> print(f"X: {reading.vector.x:.2f}g")
    """

    def __init__(
        self,
        *,
        bus: I2CBus | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        config: AccelerometerConfig | None = None,
    ) -> None:
        """Initialize accelerometer.

        Args:
            bus: I2C bus for communication (preferred)
            driver: Alternative driver for ADC-based sensors
            channel: Driver channel if using driver
            config: Accelerometer configuration
        """
        self._config = config or AccelerometerConfig()
        self._bus = bus

        # Calculate limits based on range
        max_g = self._config.range_g
        limits = Limits(min=-max_g, max=max_g)

        super().__init__(
            name=self._config.name,
            driver=driver,
            channel=channel,
            unit=Unit.METERS_PER_SECOND,
            limits=limits,
            requires_calibration=False,
        )

        # Internal state
        self._last_raw_xyz: tuple[int, int, int] = (0, 0, 0)
        self._last_vector: Vector3 = Vector3()

    def _read_raw(self) -> int:
        """Read raw magnitude from sensor."""
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz
        # Return magnitude of raw values
        return int(math.sqrt(raw_xyz[0] ** 2 + raw_xyz[1] ** 2 + raw_xyz[2] ** 2))

    def _read_raw_xyz(self) -> tuple[int, int, int]:
        """Read raw X, Y, Z values from sensor.

        Returns:
            Tuple of (x, y, z) raw integer values.
        """
        if self._bus is not None:
            # Read from I2C bus (6 bytes: X_H, X_L, Y_H, Y_L, Z_H, Z_L)
            try:
                data = self._bus.read_register(
                    self._config.i2c_address,
                    self._config.data_register,
                    6,
                )
                x = (data[0] << 8) | data[1]
                y = (data[2] << 8) | data[3]
                z = (data[4] << 8) | data[5]

                # Convert to signed 16-bit
                if x > 32767:
                    x -= 65536
                if y > 32767:
                    y -= 65536
                if z > 32767:
                    z -= 65536

                return (x, y, z)
            except Exception as e:
                raise CommunicationError(f"Failed to read accelerometer: {e}") from e

        elif self._driver is not None:
            # Read from driver (3 consecutive channels)
            try:
                x = int(self._driver.get_channel(self._channel))
                y = int(self._driver.get_channel(self._channel + 1))
                z = int(self._driver.get_channel(self._channel + 2))
                return (x, y, z)
            except Exception as e:
                raise CommunicationError(f"Failed to read accelerometer: {e}") from e

        # Simulated: return zero (at rest, should show ~1g on Z if calibrated)
        return (0, 0, 0)

    def read_vector(self) -> Vector3:
        """Read acceleration as a Vector3 in g."""
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz

        scale = self._config.scale_factor
        x = raw_xyz[0] * scale - self._config.offset_x
        y = raw_xyz[1] * scale - self._config.offset_y
        z = raw_xyz[2] * scale - self._config.offset_z

        self._last_vector = Vector3(x=x, y=y, z=z)
        return self._last_vector

    def read_acceleration(self) -> AccelerometerReading:
        """Read full acceleration data.

        Returns:
            AccelerometerReading with vector, magnitude, and metadata.
        """
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz

        scale = self._config.scale_factor
        x = raw_xyz[0] * scale - self._config.offset_x
        y = raw_xyz[1] * scale - self._config.offset_y
        z = raw_xyz[2] * scale - self._config.offset_z

        vec = Vector3(x=x, y=y, z=z)
        self._last_vector = vec

        return AccelerometerReading(
            vector=vec,
            magnitude=vec.magnitude(),
            unit=Unit.METERS_PER_SECOND,
            timestamp=time.time(),
            raw=raw_xyz,
        )

    def _run_calibration(self) -> None:
        """Calibrate accelerometer offsets.

        Assumes sensor is stationary and level (Z axis pointing up).
        After calibration, Z should read ~1g and X, Y should read ~0.
        """
        if not self._is_enabled:
            raise CalibrationError("Sensor must be enabled for calibration")

        # Take multiple samples
        samples = 100
        sum_x = sum_y = sum_z = 0.0
        scale = self._config.scale_factor

        for _ in range(samples):
            raw = self._read_raw_xyz()
            sum_x += raw[0] * scale
            sum_y += raw[1] * scale
            sum_z += raw[2] * scale
            time.sleep(0.01)

        # Calculate offsets (expect X=0, Y=0, Z=1g when level)
        self._config.offset_x = sum_x / samples
        self._config.offset_y = sum_y / samples
        self._config.offset_z = (sum_z / samples) - 1.0  # Subtract 1g

        self._is_calibrated = True


# -----------------------------------------------------------------------------
# Gyroscope
# -----------------------------------------------------------------------------


class GyroscopeConfig(BaseModel):
    """Configuration for gyroscopes."""

    name: str = "Gyroscope"
    unit: Unit = Unit.DEGREES_PER_SECOND

    # I2C settings
    i2c_address: int = Field(default=0x68, description="I2C address")
    data_register: int = Field(default=0x00, description="Data register start address")

    # Range and resolution
    range_dps: float = Field(default=250.0, description="Full scale range in °/s (±)")
    resolution_bits: int = Field(default=16, description="ADC resolution")

    # Offsets for calibration
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}

    @property
    def scale_factor(self) -> float:
        """Calculate scale factor from raw to °/s."""
        max_raw = (1 << (self.resolution_bits - 1)) - 1
        return self.range_dps / max_raw


@dataclass
class GyroscopeReading:
    """Reading from a gyroscope."""

    vector: Vector3
    magnitude: float
    unit: Unit
    timestamp: float
    raw: tuple[int, int, int]


class Gyroscope(IMUSensor):
    """3-axis gyroscope sensor.

    Measures angular velocity in degrees per second (°/s).

    Can be backed by:
    - I2C bus (typical: MPU6050, L3G4200D, ITG3200)
    - Generic driver

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> gyro = Gyroscope(bus=bus, config=GyroscopeConfig())
        >>> gyro.enable()
        >>> reading = gyro.read_angular_velocity()
        >>> print(f"Z rotation: {reading.vector.z:.2f}°/s")
    """

    def __init__(
        self,
        *,
        bus: I2CBus | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        config: GyroscopeConfig | None = None,
    ) -> None:
        """Initialize gyroscope.

        Args:
            bus: I2C bus for communication (preferred)
            driver: Alternative driver
            channel: Driver channel if using driver
            config: Gyroscope configuration
        """
        self._config = config or GyroscopeConfig()
        self._bus = bus

        # Calculate limits based on range
        max_dps = self._config.range_dps
        limits = Limits(min=-max_dps, max=max_dps)

        super().__init__(
            name=self._config.name,
            driver=driver,
            channel=channel,
            unit=Unit.DEGREES_PER_SECOND,
            limits=limits,
            requires_calibration=True,  # Gyros drift and need calibration
        )

        # Internal state
        self._last_raw_xyz: tuple[int, int, int] = (0, 0, 0)
        self._last_vector: Vector3 = Vector3()

    def _read_raw(self) -> int:
        """Read raw magnitude from sensor."""
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz
        return int(math.sqrt(raw_xyz[0] ** 2 + raw_xyz[1] ** 2 + raw_xyz[2] ** 2))

    def _read_raw_xyz(self) -> tuple[int, int, int]:
        """Read raw X, Y, Z values from sensor."""
        if self._bus is not None:
            try:
                data = self._bus.read_register(
                    self._config.i2c_address,
                    self._config.data_register,
                    6,
                )
                x = (data[0] << 8) | data[1]
                y = (data[2] << 8) | data[3]
                z = (data[4] << 8) | data[5]

                if x > 32767:
                    x -= 65536
                if y > 32767:
                    y -= 65536
                if z > 32767:
                    z -= 65536

                return (x, y, z)
            except Exception as e:
                raise CommunicationError(f"Failed to read gyroscope: {e}") from e

        elif self._driver is not None:
            try:
                x = int(self._driver.get_channel(self._channel))
                y = int(self._driver.get_channel(self._channel + 1))
                z = int(self._driver.get_channel(self._channel + 2))
                return (x, y, z)
            except Exception as e:
                raise CommunicationError(f"Failed to read gyroscope: {e}") from e

        return (0, 0, 0)

    def read_vector(self) -> Vector3:
        """Read angular velocity as a Vector3 in °/s."""
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz

        scale = self._config.scale_factor
        x = raw_xyz[0] * scale - self._config.offset_x
        y = raw_xyz[1] * scale - self._config.offset_y
        z = raw_xyz[2] * scale - self._config.offset_z

        self._last_vector = Vector3(x=x, y=y, z=z)
        return self._last_vector

    def read_angular_velocity(self) -> GyroscopeReading:
        """Read full angular velocity data.

        Returns:
            GyroscopeReading with vector, magnitude, and metadata.
        """
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz

        scale = self._config.scale_factor
        x = raw_xyz[0] * scale - self._config.offset_x
        y = raw_xyz[1] * scale - self._config.offset_y
        z = raw_xyz[2] * scale - self._config.offset_z

        vec = Vector3(x=x, y=y, z=z)
        self._last_vector = vec

        return GyroscopeReading(
            vector=vec,
            magnitude=vec.magnitude(),
            unit=Unit.DEGREES_PER_SECOND,
            timestamp=time.time(),
            raw=raw_xyz,
        )

    def calibrate_zero(self) -> None:
        """Calibrate the gyroscope to zero.

        Convenience method that calls the internal calibration routine.
        Sensor should be stationary during calibration.
        """
        self._run_calibration()

    def _run_calibration(self) -> None:
        """Calibrate gyroscope offsets.

        Assumes sensor is stationary. After calibration,
        all axes should read ~0 when not moving.
        """
        if not self._is_enabled:
            raise CalibrationError("Sensor must be enabled for calibration")

        # Take multiple samples
        samples = 100
        sum_x = sum_y = sum_z = 0.0
        scale = self._config.scale_factor

        for _ in range(samples):
            raw = self._read_raw_xyz()
            sum_x += raw[0] * scale
            sum_y += raw[1] * scale
            sum_z += raw[2] * scale
            time.sleep(0.01)

        # Calculate offsets (expect all axes = 0 when stationary)
        self._config.offset_x = sum_x / samples
        self._config.offset_y = sum_y / samples
        self._config.offset_z = sum_z / samples

        self._is_calibrated = True


# -----------------------------------------------------------------------------
# Magnetometer
# -----------------------------------------------------------------------------


class MagnetometerConfig(BaseModel):
    """Configuration for magnetometers."""

    name: str = "Magnetometer"
    unit: Unit = Unit.RAW  # Typically microtesla (μT) but using RAW

    # I2C settings
    i2c_address: int = Field(default=0x1E, description="I2C address (e.g., HMC5883L)")
    data_register: int = Field(default=0x00, description="Data register start address")

    # Range and resolution
    range_gauss: float = Field(default=1.3, description="Full scale range in Gauss (±)")
    resolution_bits: int = Field(default=12, description="ADC resolution")

    # Calibration (hard/soft iron)
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0
    scale_x: float = 1.0
    scale_y: float = 1.0
    scale_z: float = 1.0

    # Declination for true north
    declination: float = Field(default=0.0, description="Magnetic declination in degrees")

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}

    @property
    def scale_factor(self) -> float:
        """Calculate scale factor from raw to Gauss."""
        max_raw = (1 << (self.resolution_bits - 1)) - 1
        return self.range_gauss / max_raw


@dataclass
class MagnetometerReading:
    """Reading from a magnetometer."""

    vector: Vector3
    magnitude: float
    heading: float  # Compass heading in degrees (0-360)
    unit: Unit
    timestamp: float
    raw: tuple[int, int, int]


class Magnetometer(IMUSensor):
    """3-axis magnetometer sensor.

    Measures magnetic field strength and provides compass heading.

    Can be backed by:
    - I2C bus (typical: HMC5883L, QMC5883L, LIS3MDL)
    - Generic driver

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> mag = Magnetometer(bus=bus, config=MagnetometerConfig())
        >>> mag.enable()
        >>> heading = mag.heading()
        >>> print(f"Heading: {heading:.1f}°")
    """

    def __init__(
        self,
        *,
        bus: I2CBus | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        config: MagnetometerConfig | None = None,
    ) -> None:
        """Initialize magnetometer.

        Args:
            bus: I2C bus for communication (preferred)
            driver: Alternative driver
            channel: Driver channel if using driver
            config: Magnetometer configuration
        """
        self._config = config or MagnetometerConfig()
        self._bus = bus

        # Calculate limits based on range
        max_gauss = self._config.range_gauss
        limits = Limits(min=-max_gauss, max=max_gauss)

        super().__init__(
            name=self._config.name,
            driver=driver,
            channel=channel,
            unit=Unit.RAW,
            limits=limits,
            requires_calibration=True,
        )

        # Internal state
        self._last_raw_xyz: tuple[int, int, int] = (0, 0, 0)
        self._last_vector: Vector3 = Vector3()
        self._last_heading: float = 0.0

    def _read_raw(self) -> int:
        """Read raw magnitude from sensor."""
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz
        return int(math.sqrt(raw_xyz[0] ** 2 + raw_xyz[1] ** 2 + raw_xyz[2] ** 2))

    def _read_raw_xyz(self) -> tuple[int, int, int]:
        """Read raw X, Y, Z values from sensor."""
        if self._bus is not None:
            try:
                data = self._bus.read_register(
                    self._config.i2c_address,
                    self._config.data_register,
                    6,
                )
                x = (data[0] << 8) | data[1]
                y = (data[2] << 8) | data[3]
                z = (data[4] << 8) | data[5]

                if x > 32767:
                    x -= 65536
                if y > 32767:
                    y -= 65536
                if z > 32767:
                    z -= 65536

                return (x, y, z)
            except Exception as e:
                raise CommunicationError(f"Failed to read magnetometer: {e}") from e

        elif self._driver is not None:
            try:
                x = int(self._driver.get_channel(self._channel))
                y = int(self._driver.get_channel(self._channel + 1))
                z = int(self._driver.get_channel(self._channel + 2))
                return (x, y, z)
            except Exception as e:
                raise CommunicationError(f"Failed to read magnetometer: {e}") from e

        return (0, 0, 0)

    def read_vector(self) -> Vector3:
        """Read magnetic field as a Vector3."""
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz

        scale = self._config.scale_factor
        x = (raw_xyz[0] * scale - self._config.offset_x) * self._config.scale_x
        y = (raw_xyz[1] * scale - self._config.offset_y) * self._config.scale_y
        z = (raw_xyz[2] * scale - self._config.offset_z) * self._config.scale_z

        self._last_vector = Vector3(x=x, y=y, z=z)
        return self._last_vector

    def read_magnetic_field(self) -> MagnetometerReading:
        """Read full magnetic field data.

        Returns:
            MagnetometerReading with vector, heading, and metadata.
        """
        raw_xyz = self._read_raw_xyz()
        self._last_raw_xyz = raw_xyz

        scale = self._config.scale_factor
        x = (raw_xyz[0] * scale - self._config.offset_x) * self._config.scale_x
        y = (raw_xyz[1] * scale - self._config.offset_y) * self._config.scale_y
        z = (raw_xyz[2] * scale - self._config.offset_z) * self._config.scale_z

        vec = Vector3(x=x, y=y, z=z)
        self._last_vector = vec

        heading = self._calculate_heading(x, y)
        self._last_heading = heading

        return MagnetometerReading(
            vector=vec,
            magnitude=vec.magnitude(),
            heading=heading,
            unit=Unit.RAW,
            timestamp=time.time(),
            raw=raw_xyz,
        )

    def heading(self) -> float:
        """Calculate compass heading in degrees (0-360).

        Returns:
            Heading in degrees where:
            - 0° = North
            - 90° = East
            - 180° = South
            - 270° = West
        """
        vec = self.read_vector()
        heading = self._calculate_heading(vec.x, vec.y)
        self._last_heading = heading
        return heading

    def _calculate_heading(self, x: float, y: float) -> float:
        """Calculate heading from X and Y magnetic components.

        Args:
            x: X-axis magnetic field
            y: Y-axis magnetic field

        Returns:
            Heading in degrees (0-360)
        """
        # Calculate heading in radians
        heading_rad = math.atan2(y, x)

        # Convert to degrees
        heading_deg = math.degrees(heading_rad)

        # Apply declination
        heading_deg += self._config.declination

        # Normalize to 0-360
        if heading_deg < 0:
            heading_deg += 360
        elif heading_deg >= 360:
            heading_deg -= 360

        return heading_deg

    def _run_calibration(self) -> None:
        """Calibrate magnetometer (hard iron calibration).

        Note: Full calibration requires rotating the sensor through
        all orientations. This simplified version just zeros the offsets.
        """
        if not self._is_enabled:
            raise CalibrationError("Sensor must be enabled for calibration")

        # Take samples to find center offset
        samples = 100
        sum_x = sum_y = sum_z = 0.0
        scale = self._config.scale_factor

        for _ in range(samples):
            raw = self._read_raw_xyz()
            sum_x += raw[0] * scale
            sum_y += raw[1] * scale
            sum_z += raw[2] * scale
            time.sleep(0.01)

        self._config.offset_x = sum_x / samples
        self._config.offset_y = sum_y / samples
        self._config.offset_z = sum_z / samples

        self._is_calibrated = True


# -----------------------------------------------------------------------------
# Combined IMU
# -----------------------------------------------------------------------------


class IMUConfig(BaseModel):
    """Configuration for combined IMU."""

    name: str = "IMU"

    # I2C settings
    i2c_address: int = Field(default=0x68, description="Primary I2C address")
    magnetometer_address: int = Field(default=0x1E, description="Magnetometer I2C address")

    # Sub-sensor configs
    accelerometer: AccelerometerConfig = Field(default_factory=AccelerometerConfig)
    gyroscope: GyroscopeConfig = Field(default_factory=GyroscopeConfig)
    magnetometer: MagnetometerConfig = Field(default_factory=MagnetometerConfig)

    # Sample rate
    sample_rate_hz: float = Field(default=100.0, description="Sample rate in Hz")

    metadata: dict[str, Any] = Field(default_factory=dict)

    model_config = {"frozen": False, "extra": "allow"}


@dataclass
class IMUReading:
    """Complete reading from a combined IMU."""

    acceleration: Vector3  # in g
    angular_velocity: Vector3  # in °/s
    magnetic_field: Vector3  # in Gauss
    heading: float  # in degrees
    timestamp: float

    # Optional fused orientation (roll, pitch, yaw in degrees)
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None


class IMU(IMUSensor):
    """Combined 9-DOF IMU sensor.

    Combines accelerometer, gyroscope, and magnetometer into a single
    sensor with optional sensor fusion for orientation estimation.

    Can be backed by:
    - I2C bus (typical: MPU9250, MPU6050+HMC5883L, LSM9DS1)
    - Individual drivers for each axis

    Example:
        >>> from robo_infra.core.bus import SimulatedI2CBus
        >>> bus = SimulatedI2CBus()
        >>> bus.connect()
        >>> imu = IMU(bus=bus, config=IMUConfig())
        >>> imu.enable()
        >>> reading = imu.read_all()
        >>> print(f"Accel Z: {reading.acceleration.z:.2f}g")
        >>> print(f"Heading: {reading.heading:.1f}°")
    """

    def __init__(
        self,
        *,
        bus: I2CBus | None = None,
        driver: Driver | None = None,
        channel: int = 0,
        config: IMUConfig | None = None,
    ) -> None:
        """Initialize combined IMU.

        Args:
            bus: I2C bus for communication
            driver: Alternative driver
            channel: Base driver channel (uses channel, channel+3, channel+6)
            config: IMU configuration
        """
        self._config = config or IMUConfig()
        self._bus = bus

        super().__init__(
            name=self._config.name,
            driver=driver,
            channel=channel,
            unit=Unit.RAW,
            requires_calibration=True,
        )

        # Create sub-sensors
        self._accelerometer = Accelerometer(
            bus=bus,
            driver=driver,
            channel=channel,
            config=self._config.accelerometer,
        )
        self._gyroscope = Gyroscope(
            bus=bus,
            driver=driver,
            channel=channel + 3 if driver else channel,
            config=self._config.gyroscope,
        )
        self._magnetometer = Magnetometer(
            bus=bus,
            driver=driver,
            channel=channel + 6 if driver else channel,
            config=self._config.magnetometer,
        )

        # Fusion state
        self._last_reading: IMUReading | None = None
        self._complementary_alpha: float = 0.98

    def _read_raw(self) -> int:
        """Read raw value (accelerometer magnitude)."""
        return self._accelerometer._read_raw()

    def enable(self) -> None:
        """Enable the IMU and all sub-sensors."""
        super().enable()
        self._accelerometer.enable()
        self._gyroscope.enable()
        self._magnetometer.enable()

    def disable(self) -> None:
        """Disable the IMU and all sub-sensors."""
        super().disable()
        self._accelerometer.disable()
        self._gyroscope.disable()
        self._magnetometer.disable()

    @property
    def accelerometer(self) -> Accelerometer:
        """Access the accelerometer sub-sensor."""
        return self._accelerometer

    @property
    def gyroscope(self) -> Gyroscope:
        """Access the gyroscope sub-sensor."""
        return self._gyroscope

    @property
    def magnetometer(self) -> Magnetometer:
        """Access the magnetometer sub-sensor."""
        return self._magnetometer

    def read_all(self) -> IMUReading:
        """Read all IMU data at once.

        Returns:
            IMUReading with acceleration, angular velocity, magnetic field,
            and optional fused orientation.
        """
        accel = self._accelerometer.read_vector()
        gyro = self._gyroscope.read_vector()
        mag = self._magnetometer.read_vector()
        heading = self._magnetometer._calculate_heading(mag.x, mag.y)

        # Calculate orientation from accelerometer (tilt-only)
        roll = math.degrees(math.atan2(accel.y, accel.z))
        pitch = math.degrees(math.atan2(-accel.x, math.sqrt(accel.y**2 + accel.z**2)))

        reading = IMUReading(
            acceleration=accel,
            angular_velocity=gyro,
            magnetic_field=mag,
            heading=heading,
            timestamp=time.time(),
            roll=roll,
            pitch=pitch,
            yaw=heading,  # Use magnetometer heading for yaw
        )

        self._last_reading = reading
        return reading

    def read_vector(self) -> Vector3:
        """Read acceleration vector (from accelerometer)."""
        return self._accelerometer.read_vector()

    def calibrate_all(self) -> None:
        """Calibrate all IMU sub-sensors.

        Sensor should be stationary and level during calibration.
        """
        self._run_calibration()

    def _run_calibration(self) -> None:
        """Run calibration on all sub-sensors."""
        if not self._is_enabled:
            raise CalibrationError("IMU must be enabled for calibration")

        self._accelerometer._run_calibration()
        self._gyroscope._run_calibration()
        self._magnetometer._run_calibration()

        self._is_calibrated = True
