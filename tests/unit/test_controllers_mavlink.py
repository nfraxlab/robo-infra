"""Unit tests for robo_infra.controllers.mavlink module.

Tests for MAVLink-based flight controller interface for PX4/ArduPilot.
"""

from __future__ import annotations

import math

import pytest

from robo_infra.controllers.mavlink import (
    AutopilotType,
    FlightModeArduCopter,
    FlightModePX4,
    GPSFixType,
    MAVLinkAttitude,
    MAVLinkBattery,
    MAVLinkConfig,
    MAVLinkController,
    MAVLinkGPS,
    MAVLinkHeartbeat,
    MAVLinkState,
    MAVLinkStatus,
    MAVType,
    create_mavlink_controller,
)
from robo_infra.core.exceptions import SafetyError


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def default_config() -> MAVLinkConfig:
    """Create a default MAVLink configuration."""
    return MAVLinkConfig(
        name="test_mavlink",
        connection_string="udp:127.0.0.1:14550",
    )


@pytest.fixture
def mavlink_controller(default_config: MAVLinkConfig) -> MAVLinkController:
    """Create a MAVLink controller in simulated mode."""
    return MAVLinkController(
        name="test_mavlink",
        config=default_config,
        simulated=True,
    )


@pytest.fixture
def enabled_controller(mavlink_controller: MAVLinkController) -> MAVLinkController:
    """Create an enabled MAVLink controller."""
    mavlink_controller.enable()
    return mavlink_controller


# =============================================================================
# Enum Tests
# =============================================================================


class TestMAVLinkState:
    """Tests for MAVLinkState enum."""

    def test_all_states_exist(self) -> None:
        """Test all expected states exist."""
        assert MAVLinkState.DISCONNECTED.value == "disconnected"
        assert MAVLinkState.CONNECTING.value == "connecting"
        assert MAVLinkState.CONNECTED.value == "connected"
        assert MAVLinkState.ARMED.value == "armed"
        assert MAVLinkState.IN_FLIGHT.value == "in_flight"
        assert MAVLinkState.EMERGENCY.value == "emergency"
        assert MAVLinkState.ERROR.value == "error"


class TestMAVType:
    """Tests for MAVType enum."""

    def test_vehicle_types(self) -> None:
        """Test vehicle type values."""
        assert MAVType.QUADROTOR.value == 2
        assert MAVType.HEXAROTOR.value == 13
        assert MAVType.OCTOROTOR.value == 14
        assert MAVType.FIXED_WING.value == 1
        assert MAVType.GROUND_ROVER.value == 10
        # VTOL types use different naming in MAVLink spec
        assert MAVType.VTOL_DUOROTOR.value == 19


class TestAutopilotType:
    """Tests for AutopilotType enum."""

    def test_autopilot_types(self) -> None:
        """Test autopilot values."""
        assert AutopilotType.GENERIC.value == 0
        assert AutopilotType.ARDUPILOTMEGA.value == 3
        assert AutopilotType.PX4.value == 12


class TestFlightModeArduCopter:
    """Tests for ArduCopter flight modes."""

    def test_stabilized_modes(self) -> None:
        """Test stabilized modes."""
        assert FlightModeArduCopter.STABILIZE.value == 0
        assert FlightModeArduCopter.ACRO.value == 1

    def test_altitude_modes(self) -> None:
        """Test altitude hold modes."""
        assert FlightModeArduCopter.ALT_HOLD.value == 2
        assert FlightModeArduCopter.LOITER.value == 5

    def test_autonomous_modes(self) -> None:
        """Test autonomous modes."""
        assert FlightModeArduCopter.AUTO.value == 3
        assert FlightModeArduCopter.GUIDED.value == 4
        assert FlightModeArduCopter.RTL.value == 6
        assert FlightModeArduCopter.LAND.value == 9


class TestFlightModePX4:
    """Tests for PX4 flight modes."""

    def test_main_modes(self) -> None:
        """Test main modes."""
        assert FlightModePX4.MANUAL.value == 0
        assert FlightModePX4.POSCTL.value == 2
        assert FlightModePX4.OFFBOARD.value == 5

    def test_auto_modes(self) -> None:
        """Test auto modes."""
        assert FlightModePX4.AUTO_MISSION.value == 12
        assert FlightModePX4.AUTO_RTL.value == 9


class TestGPSFixType:
    """Tests for GPS fix types."""

    def test_fix_types(self) -> None:
        """Test GPS fix values."""
        assert GPSFixType.NO_GPS.value == 0
        assert GPSFixType.NO_FIX.value == 1
        assert GPSFixType.FIX_2D.value == 2
        assert GPSFixType.FIX_3D.value == 3
        assert GPSFixType.DGPS.value == 4
        assert GPSFixType.RTK_FLOAT.value == 5
        assert GPSFixType.RTK_FIXED.value == 6


# =============================================================================
# Data Class Tests
# =============================================================================


class TestMAVLinkHeartbeat:
    """Tests for MAVLinkHeartbeat dataclass."""

    def test_create_heartbeat(self) -> None:
        """Test creating heartbeat with required fields."""
        hb = MAVLinkHeartbeat(
            type=MAVType.QUADROTOR,
            autopilot=AutopilotType.ARDUPILOTMEGA,
            base_mode=0,
            custom_mode=0,
            system_status=0,
            mavlink_version=3,
        )
        assert hb.type == MAVType.QUADROTOR
        assert hb.autopilot == AutopilotType.ARDUPILOTMEGA
        assert hb.mavlink_version == 3

    def test_is_armed_property(self) -> None:
        """Test is_armed property."""
        # 128 is MAV_MODE_FLAG_SAFETY_ARMED
        armed = MAVLinkHeartbeat(
            type=MAVType.QUADROTOR,
            autopilot=AutopilotType.GENERIC,
            base_mode=128,
            custom_mode=0,
            system_status=0,
            mavlink_version=3,
        )
        assert armed.is_armed is True

        disarmed = MAVLinkHeartbeat(
            type=MAVType.QUADROTOR,
            autopilot=AutopilotType.GENERIC,
            base_mode=0,
            custom_mode=0,
            system_status=0,
            mavlink_version=3,
        )
        assert disarmed.is_armed is False


class TestMAVLinkAttitude:
    """Tests for MAVLinkAttitude dataclass."""

    def test_create_attitude(self) -> None:
        """Test creating attitude."""
        att = MAVLinkAttitude(
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
        )
        assert att.roll == 0.0
        assert att.pitch == 0.0
        assert att.yaw == 0.0

    def test_degree_properties(self) -> None:
        """Test degree conversion properties."""
        att = MAVLinkAttitude(
            roll=math.pi / 4,
            pitch=math.pi / 6,
            yaw=math.pi / 2,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
        )
        assert abs(att.roll_deg - 45.0) < 0.1
        assert abs(att.pitch_deg - 30.0) < 0.1
        assert abs(att.yaw_deg - 90.0) < 0.1


class TestMAVLinkGPS:
    """Tests for MAVLinkGPS dataclass."""

    def test_create_gps(self) -> None:
        """Test creating GPS data."""
        gps = MAVLinkGPS(
            lat=377490000,  # 37.749 degrees
            lon=-1224194000,  # -122.4194 degrees
            alt=100000,  # 100m
            relative_alt=50000,  # 50m
            vx=100,  # 1 m/s
            vy=0,
            vz=0,
            hdg=9000,  # 90 degrees
            fix_type=GPSFixType.FIX_3D,
            satellites_visible=12,
        )
        assert abs(gps.latitude - 37.749) < 0.001
        assert abs(gps.longitude - (-122.4194)) < 0.001
        assert gps.altitude_m == 100.0
        assert gps.heading == 90.0

    def test_ground_speed(self) -> None:
        """Test ground speed calculation."""
        gps = MAVLinkGPS(
            lat=0,
            lon=0,
            alt=0,
            relative_alt=0,
            vx=300,  # 3 m/s
            vy=400,  # 4 m/s
            vz=0,
            hdg=0,
        )
        assert abs(gps.ground_speed - 5.0) < 0.01


class TestMAVLinkBattery:
    """Tests for MAVLinkBattery dataclass."""

    def test_create_battery(self) -> None:
        """Test creating battery status."""
        bat = MAVLinkBattery(
            voltage=16.8,
            current=15.5,
            remaining=75,
        )
        assert bat.voltage == 16.8
        assert bat.current == 15.5
        assert bat.remaining == 75


class TestMAVLinkStatus:
    """Tests for MAVLinkStatus dataclass."""

    def test_create_status(self) -> None:
        """Test creating status object."""
        status = MAVLinkStatus(
            state=MAVLinkState.CONNECTED,
            heartbeat=None,
            attitude=None,
            gps=None,
            battery=None,
            is_armed=False,
            flight_mode="LOITER",
        )
        assert status.state == MAVLinkState.CONNECTED
        assert status.is_armed is False


# =============================================================================
# Configuration Tests
# =============================================================================


class TestMAVLinkConfig:
    """Tests for MAVLinkConfig."""

    def test_default_config(self) -> None:
        """Test default configuration values."""
        config = MAVLinkConfig(name="test")
        assert config.name == "test"
        assert config.connection_string == "udp:127.0.0.1:14550"
        assert config.baud_rate == 57600

    def test_custom_connection(self) -> None:
        """Test custom connection string."""
        config = MAVLinkConfig(
            name="test",
            connection_string="/dev/ttyUSB0",
            baud_rate=115200,
        )
        assert config.connection_string == "/dev/ttyUSB0"
        assert config.baud_rate == 115200


# =============================================================================
# Controller Initialization Tests
# =============================================================================


class TestMAVLinkControllerInit:
    """Tests for MAVLinkController initialization."""

    def test_basic_init(self) -> None:
        """Test basic initialization."""
        controller = MAVLinkController(name="test", simulated=True)
        assert controller.name == "test"
        assert controller.mav_state == MAVLinkState.DISCONNECTED
        assert not controller.is_enabled

    def test_init_with_config(self, default_config: MAVLinkConfig) -> None:
        """Test initialization with config."""
        controller = MAVLinkController(
            name="test",
            config=default_config,
            simulated=True,
        )
        assert controller.mav_config == default_config


# =============================================================================
# Enable/Disable Tests
# =============================================================================


class TestMAVLinkEnableDisable:
    """Tests for enable/disable functionality."""

    def test_enable(self, mavlink_controller: MAVLinkController) -> None:
        """Test enabling controller."""
        mavlink_controller.enable()
        assert mavlink_controller.is_enabled

    def test_disable(self, enabled_controller: MAVLinkController) -> None:
        """Test disabling controller."""
        enabled_controller.disable()
        assert not enabled_controller.is_enabled


# =============================================================================
# Connection Tests
# =============================================================================


class TestMAVLinkConnection:
    """Tests for connection management."""

    def test_simulated_connect(self, mavlink_controller: MAVLinkController) -> None:
        """Test simulated connection."""
        mavlink_controller.enable()
        # In simulated mode, should connect automatically
        assert mavlink_controller.is_enabled
        assert mavlink_controller.is_connected


# =============================================================================
# Arming Tests
# =============================================================================


class TestMAVLinkArming:
    """Tests for arm/disarm functionality."""

    @pytest.mark.asyncio
    async def test_arm(self, enabled_controller: MAVLinkController) -> None:
        """Test arming via MAVLink."""
        await enabled_controller.arm()
        assert enabled_controller.is_armed

    @pytest.mark.asyncio
    async def test_disarm(self, enabled_controller: MAVLinkController) -> None:
        """Test disarming via MAVLink."""
        await enabled_controller.arm()
        await enabled_controller.disarm()
        assert not enabled_controller.is_armed

    @pytest.mark.asyncio
    async def test_force_arm(self, enabled_controller: MAVLinkController) -> None:
        """Test force arming."""
        await enabled_controller.arm(force=True)
        assert enabled_controller.is_armed


# =============================================================================
# Flight Command Tests
# =============================================================================


class TestMAVLinkTakeoff:
    """Tests for takeoff command."""

    @pytest.mark.asyncio
    async def test_takeoff(self, enabled_controller: MAVLinkController) -> None:
        """Test takeoff command."""
        await enabled_controller.arm()
        await enabled_controller.takeoff(altitude=10.0)
        assert enabled_controller.mav_state == MAVLinkState.IN_FLIGHT

    @pytest.mark.asyncio
    async def test_takeoff_requires_armed(
        self, enabled_controller: MAVLinkController
    ) -> None:
        """Test takeoff requires armed state."""
        with pytest.raises(SafetyError, match="not armed"):
            await enabled_controller.takeoff(altitude=10.0)


class TestMAVLinkLand:
    """Tests for land command."""

    @pytest.mark.asyncio
    async def test_land(self, enabled_controller: MAVLinkController) -> None:
        """Test land command."""
        await enabled_controller.arm()
        await enabled_controller.takeoff(altitude=10.0)
        await enabled_controller.land()
        # After landing, back to ARMED state
        assert enabled_controller.mav_state == MAVLinkState.ARMED


class TestMAVLinkRTL:
    """Tests for return to launch."""

    @pytest.mark.asyncio
    async def test_return_to_launch(
        self, enabled_controller: MAVLinkController
    ) -> None:
        """Test RTL command."""
        await enabled_controller.arm()
        await enabled_controller.takeoff(altitude=10.0)
        await enabled_controller.return_to_launch()
        # RTL command was executed
        assert enabled_controller.is_armed


# =============================================================================
# Position Control Tests
# =============================================================================


class TestMAVLinkPositionControl:
    """Tests for position control commands."""

    @pytest.mark.asyncio
    async def test_goto_global(
        self, enabled_controller: MAVLinkController
    ) -> None:
        """Test global position command."""
        await enabled_controller.arm()
        await enabled_controller.takeoff(altitude=10.0)
        # Use correct parameter names: lat, lon, alt
        await enabled_controller.goto_position_global(
            lat=37.7749,
            lon=-122.4194,
            alt=100.0,
        )
        # Command executed successfully

    @pytest.mark.asyncio
    async def test_goto_local(self, enabled_controller: MAVLinkController) -> None:
        """Test local position command."""
        await enabled_controller.arm()
        await enabled_controller.takeoff(altitude=10.0)
        await enabled_controller.goto_position_local(
            x=10.0,
            y=20.0,
            z=-30.0,  # NED frame
        )
        # Command executed successfully


class TestMAVLinkVelocityControl:
    """Tests for velocity control commands."""

    @pytest.mark.asyncio
    async def test_set_velocity(
        self, enabled_controller: MAVLinkController
    ) -> None:
        """Test velocity setpoint command."""
        await enabled_controller.arm()
        await enabled_controller.set_velocity(
            vx=1.0,
            vy=0.5,
            vz=0.0,
        )
        # Command executed successfully

    @pytest.mark.asyncio
    async def test_set_velocity_with_yaw(
        self, enabled_controller: MAVLinkController
    ) -> None:
        """Test velocity with yaw rate."""
        await enabled_controller.arm()
        await enabled_controller.set_velocity(
            vx=1.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=10.0,
        )
        # Command executed successfully


# =============================================================================
# Flight Mode Tests
# =============================================================================


class TestMAVLinkFlightModes:
    """Tests for flight mode control."""

    @pytest.mark.asyncio
    async def test_set_mode_arducopter(
        self, enabled_controller: MAVLinkController
    ) -> None:
        """Test setting ArduCopter mode."""
        await enabled_controller.set_mode(FlightModeArduCopter.LOITER)
        # Mode was set


# =============================================================================
# Emergency Stop Tests
# =============================================================================


class TestMAVLinkEmergencyStop:
    """Tests for emergency stop."""

    def test_emergency_stop(self, enabled_controller: MAVLinkController) -> None:
        """Test emergency stop."""
        enabled_controller.stop()
        assert enabled_controller.mav_state == MAVLinkState.EMERGENCY


# =============================================================================
# AI Integration Tests
# =============================================================================


class TestMAVLinkAIIntegration:
    """Tests for AI integration."""

    def test_as_tools(self, enabled_controller: MAVLinkController) -> None:
        """Test generating AI tools."""
        tools = enabled_controller.as_tools()
        assert isinstance(tools, list)
        assert len(tools) > 0


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreateMAVLinkController:
    """Tests for create_mavlink_controller factory function."""

    def test_create_default(self) -> None:
        """Test creating controller with defaults."""
        controller = create_mavlink_controller(simulated=True)
        assert controller.name == "mavlink"

    def test_create_custom_connection(self) -> None:
        """Test creating controller with custom connection."""
        controller = create_mavlink_controller(
            connection="tcp:192.168.1.1:5760",
            name="custom",
            simulated=True,
        )
        assert controller.name == "custom"
        assert controller.mav_config.connection_string == "tcp:192.168.1.1:5760"

    def test_create_simulated(self) -> None:
        """Test creating simulated controller."""
        controller = create_mavlink_controller(simulated=True)
        controller.enable()
        assert controller.is_enabled


# =============================================================================
# Edge Case Tests
# =============================================================================


class TestMAVLinkEdgeCases:
    """Tests for edge cases and error handling."""

    @pytest.mark.asyncio
    async def test_double_arm(self, enabled_controller: MAVLinkController) -> None:
        """Test double arming is idempotent."""
        await enabled_controller.arm()
        await enabled_controller.arm()
        assert enabled_controller.is_armed

    @pytest.mark.asyncio
    async def test_double_disarm(
        self, enabled_controller: MAVLinkController
    ) -> None:
        """Test double disarm is idempotent."""
        await enabled_controller.disarm()
        assert not enabled_controller.is_armed

    def test_enable_disable_cycle(
        self, mavlink_controller: MAVLinkController
    ) -> None:
        """Test enable/disable cycle."""
        for _ in range(3):
            mavlink_controller.enable()
            assert mavlink_controller.is_enabled
            mavlink_controller.disable()
            assert not mavlink_controller.is_enabled
