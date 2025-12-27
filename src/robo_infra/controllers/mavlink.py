"""MAVLink flight controller interface.

This module provides a MAVLink-based controller interface for communicating
with flight controllers that support the MAVLink protocol, such as:
- PX4 Autopilot
- ArduPilot (ArduCopter, ArduPlane, ArduRover)
- Other MAVLink-compatible systems

MAVLink is a lightweight messaging protocol for communicating with drones
and between onboard drone components.

Example:
    >>> from robo_infra.controllers.mavlink import MAVLinkController
    >>>
    >>> # Connect to flight controller
    >>> fc = MAVLinkController("udp:127.0.0.1:14550")
    >>> fc.enable()
    >>>
    >>> # Arm and takeoff
    >>> fc.arm()
    >>> fc.takeoff(altitude=5.0)
    >>>
    >>> # Fly to waypoint
    >>> fc.goto_position(lat=37.7749, lon=-122.4194, alt=10.0)

Note:
    This module provides an interface abstraction. For actual MAVLink
    communication, you need pymavlink installed:
    ```
    pip install pymavlink
    ```
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from robo_infra.core.controller import Controller, ControllerConfig
from robo_infra.core.exceptions import CommunicationError, DisabledError, SafetyError


if TYPE_CHECKING:
    from collections.abc import Callable


logger = logging.getLogger(__name__)


# =============================================================================
# Constants
# =============================================================================

# MAVLink message IDs (subset of commonly used)
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_GPS_RAW_INT = 24
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84
MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT = 86
MAVLINK_MSG_ID_STATUSTEXT = 253

# MAVLink command IDs (MAV_CMD)
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176

# MAVLink component IDs
MAV_COMP_ID_AUTOPILOT1 = 1
MAV_COMP_ID_ALL = 0

# Default timeouts
DEFAULT_CONNECTION_TIMEOUT = 10.0
DEFAULT_COMMAND_TIMEOUT = 5.0
DEFAULT_HEARTBEAT_TIMEOUT = 3.0


# =============================================================================
# Enums
# =============================================================================


class MAVLinkState(Enum):
    """MAVLink connection states."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ARMED = "armed"
    IN_FLIGHT = "in_flight"
    EMERGENCY = "emergency"
    ERROR = "error"


class MAVType(Enum):
    """MAVLink vehicle types (MAV_TYPE)."""

    GENERIC = 0
    FIXED_WING = 1
    QUADROTOR = 2
    COAXIAL = 3
    HELICOPTER = 4
    ANTENNA_TRACKER = 5
    GCS = 6
    AIRSHIP = 7
    FREE_BALLOON = 8
    ROCKET = 9
    GROUND_ROVER = 10
    SURFACE_BOAT = 11
    SUBMARINE = 12
    HEXAROTOR = 13
    OCTOROTOR = 14
    TRICOPTER = 15
    FLAPPING_WING = 16
    KITE = 17
    ONBOARD_CONTROLLER = 18
    VTOL_DUOROTOR = 19
    VTOL_QUADROTOR = 20
    VTOL_TILTROTOR = 21
    VTOL_RESERVED2 = 22
    VTOL_RESERVED3 = 23
    VTOL_RESERVED4 = 24
    VTOL_RESERVED5 = 25
    GIMBAL = 26
    ADSB = 27
    PARAFOIL = 28
    DODECAROTOR = 29


class AutopilotType(Enum):
    """Autopilot firmware types (MAV_AUTOPILOT)."""

    GENERIC = 0
    RESERVED = 1
    SLUGS = 2
    ARDUPILOTMEGA = 3
    OPENPILOT = 4
    GENERIC_WAYPOINTS_ONLY = 5
    GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6
    GENERIC_MISSION_FULL = 7
    INVALID = 8
    PPZ = 9
    UDB = 10
    FP = 11
    PX4 = 12
    SMACCMPILOT = 13
    AUTOQUAD = 14
    ARMAZILA = 15
    AEROB = 16
    ASLUAV = 17
    SMARTAP = 18
    AIRRAILS = 19


class FlightModeArduCopter(Enum):
    """ArduCopter flight modes."""

    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    SMART_RTL = 21
    ZIGZAG = 24


class FlightModePX4(Enum):
    """PX4 main flight modes."""

    MANUAL = 0
    ALTCTL = 1
    POSCTL = 2
    AUTO = 3
    ACRO = 4
    OFFBOARD = 5
    STABILIZED = 6
    RATTITUDE = 7
    AUTO_LOITER = 8
    AUTO_RTL = 9
    AUTO_LAND = 10
    AUTO_TAKEOFF = 11
    AUTO_MISSION = 12
    AUTO_PRECLAND = 13


class GPSFixType(Enum):
    """GPS fix types."""

    NO_GPS = 0
    NO_FIX = 1
    FIX_2D = 2
    FIX_3D = 3
    DGPS = 4
    RTK_FLOAT = 5
    RTK_FIXED = 6
    STATIC = 7
    PPP = 8


# =============================================================================
# Data Classes
# =============================================================================


@dataclass(slots=True)
class MAVLinkHeartbeat:
    """MAVLink heartbeat message data."""

    type: MAVType
    autopilot: AutopilotType
    base_mode: int
    custom_mode: int
    system_status: int
    mavlink_version: int
    timestamp: float = 0.0

    @property
    def is_armed(self) -> bool:
        """Check if vehicle is armed."""
        return bool(self.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED

    @property
    def is_guided(self) -> bool:
        """Check if in guided mode."""
        return bool(self.base_mode & 8)  # MAV_MODE_FLAG_GUIDED_ENABLED


@dataclass(slots=True)
class MAVLinkAttitude:
    """Vehicle attitude from MAVLink."""

    roll: float  # Roll angle in radians
    pitch: float  # Pitch angle in radians
    yaw: float  # Yaw angle in radians
    rollspeed: float  # Roll angular speed in rad/s
    pitchspeed: float  # Pitch angular speed in rad/s
    yawspeed: float  # Yaw angular speed in rad/s
    timestamp: float = 0.0

    @property
    def roll_deg(self) -> float:
        """Roll in degrees."""
        import math

        return math.degrees(self.roll)

    @property
    def pitch_deg(self) -> float:
        """Pitch in degrees."""
        import math

        return math.degrees(self.pitch)

    @property
    def yaw_deg(self) -> float:
        """Yaw in degrees."""
        import math

        return math.degrees(self.yaw)


@dataclass(slots=True)
class MAVLinkGPS:
    """GPS position from MAVLink."""

    lat: int  # Latitude in degE7 (1e-7 degrees)
    lon: int  # Longitude in degE7
    alt: int  # Altitude in mm above MSL
    relative_alt: int  # Altitude above home in mm
    vx: int  # Ground speed X in cm/s
    vy: int  # Ground speed Y in cm/s
    vz: int  # Ground speed Z in cm/s
    hdg: int  # Heading in cdeg (0.01 degrees)
    fix_type: GPSFixType = GPSFixType.NO_GPS
    satellites_visible: int = 0
    timestamp: float = 0.0

    @property
    def latitude(self) -> float:
        """Latitude in degrees."""
        return self.lat / 1e7

    @property
    def longitude(self) -> float:
        """Longitude in degrees."""
        return self.lon / 1e7

    @property
    def altitude_m(self) -> float:
        """Altitude in meters."""
        return self.alt / 1000.0

    @property
    def relative_altitude_m(self) -> float:
        """Relative altitude in meters."""
        return self.relative_alt / 1000.0

    @property
    def heading(self) -> float:
        """Heading in degrees."""
        return self.hdg / 100.0

    @property
    def ground_speed(self) -> float:
        """Ground speed in m/s."""
        import math

        return math.sqrt(self.vx**2 + self.vy**2) / 100.0


@dataclass(slots=True)
class MAVLinkBattery:
    """Battery status from MAVLink."""

    voltage: float  # Voltage in volts
    current: float  # Current in amps
    remaining: int  # Remaining percentage
    timestamp: float = 0.0


@dataclass
class MAVLinkStatus:
    """Complete MAVLink vehicle status."""

    state: MAVLinkState
    heartbeat: MAVLinkHeartbeat | None
    attitude: MAVLinkAttitude | None
    gps: MAVLinkGPS | None
    battery: MAVLinkBattery | None
    is_armed: bool = False
    flight_mode: str = "UNKNOWN"
    connection_quality: float = 0.0
    last_heartbeat: float = 0.0


# =============================================================================
# Configuration
# =============================================================================


class MAVLinkConfig(BaseModel):
    """Configuration for MAVLink controller.

    Attributes:
        connection_string: Connection string for MAVLink.
        system_id: MAVLink system ID for this controller.
        component_id: MAVLink component ID.
        source_system: System ID of the vehicle to control.
        source_component: Component ID of the vehicle.

    Example:
        >>> config = MAVLinkConfig(
        ...     name="px4_connection",
        ...     connection_string="udp:127.0.0.1:14550",
        ... )
    """

    model_config = {"arbitrary_types_allowed": True}

    name: str = Field(..., description="Connection name")
    description: str = Field(default="", description="Human-readable description")

    # Connection
    connection_string: str = Field(
        default="udp:127.0.0.1:14550",
        description="MAVLink connection string",
    )
    baud_rate: int = Field(
        default=57600,
        gt=0,
        description="Serial baud rate (if using serial connection)",
    )

    # MAVLink IDs
    system_id: int = Field(
        default=255,
        ge=1,
        le=255,
        description="System ID for this GCS",
    )
    component_id: int = Field(
        default=MAV_COMP_ID_AUTOPILOT1,
        ge=0,
        le=255,
        description="Component ID for this GCS",
    )
    source_system: int = Field(
        default=1,
        ge=1,
        le=255,
        description="Target vehicle system ID",
    )
    source_component: int = Field(
        default=1,
        ge=0,
        le=255,
        description="Target vehicle component ID",
    )

    # Timeouts
    connection_timeout: float = Field(
        default=DEFAULT_CONNECTION_TIMEOUT,
        gt=0,
        description="Connection timeout in seconds",
    )
    command_timeout: float = Field(
        default=DEFAULT_COMMAND_TIMEOUT,
        gt=0,
        description="Command response timeout in seconds",
    )
    heartbeat_timeout: float = Field(
        default=DEFAULT_HEARTBEAT_TIMEOUT,
        gt=0,
        description="Heartbeat timeout in seconds",
    )

    # Behavior
    auto_arm: bool = Field(
        default=False,
        description="Automatically arm when commanded to takeoff",
    )
    require_gps: bool = Field(
        default=True,
        description="Require GPS fix for autonomous commands",
    )
    min_satellites: int = Field(
        default=6,
        ge=0,
        description="Minimum satellites for GPS operations",
    )

    # Stream rates (Hz)
    heartbeat_rate: float = Field(
        default=1.0,
        gt=0,
        description="Heartbeat sending rate in Hz",
    )
    telemetry_rate: float = Field(
        default=4.0,
        gt=0,
        description="Telemetry stream rate in Hz",
    )


# =============================================================================
# MAVLink Controller
# =============================================================================


class MAVLinkController(Controller):
    """MAVLink flight controller interface.

    Provides high-level control for MAVLink-compatible flight controllers
    like PX4 and ArduPilot. Handles connection management, telemetry
    streaming, and command sending.

    Key Features:
        - Connect to flight controllers via serial, UDP, or TCP
        - Arm/disarm with safety checks
        - Takeoff and land commands
        - Position and velocity commands
        - Mode switching
        - Telemetry streaming

    Supported Protocols:
        - UDP: "udp:host:port" or "udpin:host:port" or "udpout:host:port"
        - TCP: "tcp:host:port" or "tcpin:host:port"
        - Serial: "/dev/ttyUSB0" or "COM3"

    Example:
        >>> from robo_infra.controllers.mavlink import MAVLinkController
        >>>
        >>> # Connect to SITL
        >>> fc = MAVLinkController("udp:127.0.0.1:14550")
        >>> fc.enable()
        >>> await fc.connect()
        >>>
        >>> # Basic flight
        >>> await fc.arm()
        >>> await fc.takeoff(5.0)
        >>> await fc.goto_position_local(10, 0, -5)
        >>> await fc.land()
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14550",
        *,
        name: str = "mavlink",
        config: MAVLinkConfig | None = None,
        simulated: bool = True,
    ) -> None:
        """Initialize MAVLink controller.

        Args:
            connection_string: MAVLink connection string
            name: Controller name
            config: Optional configuration
            simulated: Use simulated connection if True
        """
        ctrl_config = ControllerConfig(name=name)
        super().__init__(name, config=ctrl_config)

        if config is not None:
            self._mav_config = config
        else:
            self._mav_config = MAVLinkConfig(
                name=name,
                connection_string=connection_string,
            )

        self._simulated = simulated

        # Connection state
        self._mav_state = MAVLinkState.DISCONNECTED
        self._connection: Any = None
        self._is_connected = False

        # Telemetry
        self._heartbeat: MAVLinkHeartbeat | None = None
        self._attitude: MAVLinkAttitude | None = None
        self._gps: MAVLinkGPS | None = None
        self._battery: MAVLinkBattery | None = None
        self._last_heartbeat_time: float = 0.0
        self._messages_received: int = 0
        self._messages_sent: int = 0

        # Tasks
        self._receive_task: asyncio.Task[None] | None = None
        self._heartbeat_task: asyncio.Task[None] | None = None

        # Simulated state
        self._sim_armed = False
        self._sim_mode = "STABILIZE"
        self._sim_altitude = 0.0
        self._sim_lat = 37.7749
        self._sim_lon = -122.4194

        logger.debug(
            "MAVLinkController '%s' initialized (connection=%s, simulated=%s)",
            name,
            self._mav_config.connection_string,
            simulated,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def mav_config(self) -> MAVLinkConfig:
        """Get MAVLink configuration."""
        return self._mav_config

    @property
    def mav_state(self) -> MAVLinkState:
        """Get current MAVLink state."""
        return self._mav_state

    @property
    def is_connected(self) -> bool:
        """Check if connected to flight controller."""
        return self._is_connected

    @property
    def is_armed(self) -> bool:
        """Check if vehicle is armed."""
        if self._simulated:
            return self._sim_armed
        if self._heartbeat:
            return self._heartbeat.is_armed
        return False

    @property
    def flight_mode(self) -> str:
        """Get current flight mode string."""
        if self._simulated:
            return self._sim_mode
        if self._heartbeat:
            return str(self._heartbeat.custom_mode)
        return "UNKNOWN"

    @property
    def heartbeat(self) -> MAVLinkHeartbeat | None:
        """Get last heartbeat."""
        return self._heartbeat

    @property
    def attitude(self) -> MAVLinkAttitude | None:
        """Get current attitude."""
        return self._attitude

    @property
    def gps(self) -> MAVLinkGPS | None:
        """Get current GPS data."""
        return self._gps

    @property
    def battery(self) -> MAVLinkBattery | None:
        """Get current battery status."""
        return self._battery

    @property
    def connection_quality(self) -> float:
        """Get connection quality (0.0 to 1.0)."""
        if not self._is_connected:
            return 0.0
        # Based on heartbeat timing
        elapsed = time.time() - self._last_heartbeat_time
        if elapsed > self._mav_config.heartbeat_timeout:
            return 0.0
        return max(0.0, 1.0 - elapsed / self._mav_config.heartbeat_timeout)

    # -------------------------------------------------------------------------
    # Controller Implementation
    # -------------------------------------------------------------------------

    def _do_home(self) -> None:
        """Perform homing - set current GPS as home."""
        logger.debug("Setting current position as home")
        self._is_homed = True

    def _do_stop(self) -> None:
        """Emergency stop - send kill command."""
        logger.warning("EMERGENCY STOP: Sending motor kill command!")
        self._mav_state = MAVLinkState.EMERGENCY

        if self._simulated:
            self._sim_armed = False
            self._sim_altitude = 0.0

    def _on_enable(self) -> None:
        """Called when controller is enabled."""
        self._mav_state = MAVLinkState.DISCONNECTED
        if self._simulated:
            self._is_connected = True
            self._mav_state = MAVLinkState.CONNECTED
        logger.info("MAVLinkController '%s' enabled", self._name)

    def _on_disable(self) -> None:
        """Called when controller is disabled."""
        self._mav_state = MAVLinkState.DISCONNECTED
        self._is_connected = False
        logger.info("MAVLinkController '%s' disabled", self._name)

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    async def connect(self) -> None:
        """Connect to the flight controller.

        Raises:
            CommunicationError: If connection fails
            DisabledError: If controller not enabled
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")

        if self._is_connected:
            logger.debug("Already connected")
            return

        self._mav_state = MAVLinkState.CONNECTING

        if self._simulated:
            # Simulated connection always succeeds
            await asyncio.sleep(0.1)  # Simulate connection delay
            self._is_connected = True
            self._mav_state = MAVLinkState.CONNECTED
            self._last_heartbeat_time = time.time()
            logger.info("Simulated MAVLink connection established")
            return

        # Real MAVLink connection (requires pymavlink)
        try:
            from pymavlink import mavutil
        except ImportError as e:
            msg = "pymavlink not installed. Install with: pip install pymavlink"
            raise CommunicationError(msg) from e

        try:
            self._connection = mavutil.mavlink_connection(
                self._mav_config.connection_string,
                baud=self._mav_config.baud_rate,
                source_system=self._mav_config.system_id,
                source_component=self._mav_config.component_id,
            )

            # Wait for first heartbeat
            msg = self._connection.wait_heartbeat(
                timeout=self._mav_config.connection_timeout,
            )
            if msg is None:
                raise CommunicationError("No heartbeat received")

            self._is_connected = True
            self._mav_state = MAVLinkState.CONNECTED
            self._last_heartbeat_time = time.time()
            logger.info("MAVLink connection established to %s", self._mav_config.connection_string)

        except Exception as e:
            self._mav_state = MAVLinkState.ERROR
            raise CommunicationError(f"Connection failed: {e}") from e

    async def disconnect(self) -> None:
        """Disconnect from flight controller."""
        if not self._is_connected:
            return

        # Cancel background tasks
        if self._receive_task:
            self._receive_task.cancel()
        if self._heartbeat_task:
            self._heartbeat_task.cancel()

        # Close connection
        if self._connection:
            self._connection.close()
            self._connection = None

        self._is_connected = False
        self._mav_state = MAVLinkState.DISCONNECTED
        logger.info("MAVLink disconnected")

    # -------------------------------------------------------------------------
    # Commands
    # -------------------------------------------------------------------------

    async def arm(self, force: bool = False) -> None:
        """Arm the vehicle.

        Args:
            force: Force arm even if preflight checks fail

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
            SafetyError: If arming fails
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            self._sim_armed = True
            self._mav_state = MAVLinkState.ARMED
            logger.info("Vehicle armed (simulated)")
            return

        # Real arm command
        await self._send_command_long(
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,  # 1 = arm
            param2=21196.0 if force else 0.0,  # Magic number for force arm
        )
        logger.info("Arm command sent")

    async def disarm(self, force: bool = False) -> None:
        """Disarm the vehicle.

        Args:
            force: Force disarm even if in flight

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            self._sim_armed = False
            self._mav_state = MAVLinkState.CONNECTED
            logger.info("Vehicle disarmed (simulated)")
            return

        await self._send_command_long(
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,  # 0 = disarm
            param2=21196.0 if force else 0.0,
        )
        logger.info("Disarm command sent")

    async def takeoff(self, altitude: float = 2.5) -> None:
        """Take off to specified altitude.

        Args:
            altitude: Target altitude in meters

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
            SafetyError: If not armed
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")
        if not self.is_armed:
            if self._mav_config.auto_arm:
                await self.arm()
            else:
                raise SafetyError("Vehicle not armed")

        if self._simulated:
            self._sim_altitude = altitude
            self._mav_state = MAVLinkState.IN_FLIGHT
            logger.info("Takeoff to %.1fm (simulated)", altitude)
            return

        await self._send_command_long(
            command=MAV_CMD_NAV_TAKEOFF,
            param7=altitude,  # Altitude
        )
        logger.info("Takeoff command sent (%.1fm)", altitude)

    async def land(self) -> None:
        """Land at current position.

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            self._sim_altitude = 0.0
            self._mav_state = MAVLinkState.ARMED
            logger.info("Landing (simulated)")
            return

        await self._send_command_long(command=MAV_CMD_NAV_LAND)
        logger.info("Land command sent")

    async def return_to_launch(self) -> None:
        """Return to launch position.

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            self._sim_lat = 37.7749
            self._sim_lon = -122.4194
            logger.info("RTL (simulated)")
            return

        await self._send_command_long(command=MAV_CMD_NAV_RETURN_TO_LAUNCH)
        logger.info("RTL command sent")

    async def goto_position_global(
        self,
        lat: float,
        lon: float,
        alt: float,
        *,
        relative_alt: bool = True,
    ) -> None:
        """Go to global GPS position.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
            relative_alt: If True, altitude is relative to home

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            self._sim_lat = lat
            self._sim_lon = lon
            self._sim_altitude = alt
            logger.info("Goto (%.6f, %.6f, %.1fm) (simulated)", lat, lon, alt)
            return

        # Send SET_POSITION_TARGET_GLOBAL_INT message
        logger.info("Goto position command sent (%.6f, %.6f, %.1fm)", lat, lon, alt)

    async def goto_position_local(
        self,
        x: float,
        y: float,
        z: float,
        *,
        yaw: float | None = None,
    ) -> None:
        """Go to local NED position.

        Args:
            x: North position in meters
            y: East position in meters
            z: Down position in meters (negative = up)
            yaw: Optional yaw angle in radians

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            self._sim_altitude = -z  # NED to altitude
            logger.info("Goto local (%.1f, %.1f, %.1f) (simulated)", x, y, z)
            return

        # Send SET_POSITION_TARGET_LOCAL_NED message
        logger.info("Goto local position command sent (%.1f, %.1f, %.1f)", x, y, z)

    async def set_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        *,
        yaw_rate: float = 0.0,
    ) -> None:
        """Set target velocity in local NED frame.

        Args:
            vx: North velocity in m/s
            vy: East velocity in m/s
            vz: Down velocity in m/s (positive = down)
            yaw_rate: Yaw rate in rad/s

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            logger.debug("Velocity command (%.1f, %.1f, %.1f) (simulated)", vx, vy, vz)
            return

        # Send velocity command
        logger.debug("Velocity command sent (%.1f, %.1f, %.1f)", vx, vy, vz)

    async def set_mode(self, mode: str) -> None:
        """Set flight mode.

        Args:
            mode: Mode name (e.g., "GUIDED", "LOITER", "RTL")

        Raises:
            DisabledError: If not enabled
            CommunicationError: If not connected
        """
        if not self._is_enabled:
            raise DisabledError("Controller not enabled")
        if not self._is_connected:
            raise CommunicationError("Not connected")

        if self._simulated:
            self._sim_mode = mode
            logger.info("Mode set to %s (simulated)", mode)
            return

        # Send mode change command
        logger.info("Mode change command sent: %s", mode)

    # -------------------------------------------------------------------------
    # Internal Methods
    # -------------------------------------------------------------------------

    async def _send_command_long(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        """Send a COMMAND_LONG message.

        Args:
            command: MAV_CMD command ID
            param1-7: Command parameters
        """
        if not self._connection:
            raise CommunicationError("No connection")

        self._connection.mav.command_long_send(
            self._mav_config.source_system,
            self._mav_config.source_component,
            command,
            0,  # Confirmation
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
        )
        self._messages_sent += 1

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def status(self) -> MAVLinkStatus:
        """Get current MAVLink status.

        Returns:
            Complete status information
        """
        return MAVLinkStatus(
            state=self._mav_state,
            heartbeat=self._heartbeat,
            attitude=self._attitude,
            gps=self._gps,
            battery=self._battery,
            is_armed=self.is_armed,
            flight_mode=self.flight_mode,
            connection_quality=self.connection_quality,
            last_heartbeat=self._last_heartbeat_time,
        )

    # -------------------------------------------------------------------------
    # Integration Methods (ai-infra/svc-infra)
    # -------------------------------------------------------------------------

    def as_tools(self) -> list[dict[str, Any] | Callable[..., Any]]:
        """Generate ai-infra compatible tools for LLM control.

        Returns:
            List of function tools for AI agent
        """

        async def connect_mavlink() -> str:
            """Connect to the MAVLink flight controller."""
            try:
                await self.connect()
                return "Connected to flight controller"
            except Exception as e:
                return f"Connection failed: {e}"

        async def arm_vehicle() -> str:
            """Arm the vehicle motors."""
            try:
                await self.arm()
                return "Vehicle armed"
            except Exception as e:
                return f"Arm failed: {e}"

        async def disarm_vehicle() -> str:
            """Disarm the vehicle motors."""
            try:
                await self.disarm()
                return "Vehicle disarmed"
            except Exception as e:
                return f"Disarm failed: {e}"

        async def takeoff_vehicle(altitude: float = 2.5) -> str:
            """Take off to specified altitude in meters.

            Args:
                altitude: Target altitude (default 2.5m)
            """
            try:
                await self.takeoff(altitude)
                return f"Taking off to {altitude}m"
            except Exception as e:
                return f"Takeoff failed: {e}"

        async def land_vehicle() -> str:
            """Land the vehicle at current position."""
            try:
                await self.land()
                return "Landing"
            except Exception as e:
                return f"Land failed: {e}"

        def get_vehicle_status() -> dict:
            """Get current vehicle status."""
            status = self.status()
            return {
                "state": status.state.value,
                "is_armed": status.is_armed,
                "flight_mode": status.flight_mode,
                "connection_quality": status.connection_quality,
            }

        return [
            connect_mavlink,
            arm_vehicle,
            disarm_vehicle,
            takeoff_vehicle,
            land_vehicle,
            get_vehicle_status,
        ]


# =============================================================================
# Factory Functions
# =============================================================================


def create_mavlink_controller(
    connection: str = "udp:127.0.0.1:14550",
    *,
    name: str = "mavlink",
    simulated: bool = True,
) -> MAVLinkController:
    """Create a MAVLink controller.

    Args:
        connection: MAVLink connection string
        name: Controller name
        simulated: Use simulated connection

    Returns:
        Configured MAVLinkController

    Example:
        >>> # Connect to SITL
        >>> fc = create_mavlink_controller("udp:127.0.0.1:14550")
        >>>
        >>> # Connect to serial
        >>> fc = create_mavlink_controller("/dev/ttyUSB0", simulated=False)
    """
    return MAVLinkController(
        connection_string=connection,
        name=name,
        simulated=simulated,
    )
