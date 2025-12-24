"""Unit tests for robo_infra.controllers.mavlink module.

Tests for MAVLink-based flight controller interface for PX4/ArduPilot.
"""

from __future__ import annotations

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


@pytest.fixture
def connected_controller(enabled_controller: MAVLinkController) -> MAVLinkController:
    """Create a connected MAVLink controller (simulated)."""
    # In simulated mode, enable connects automatically
    return enabled_controller


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
        assert MAVType.VTOL.value == 19
        assert MAVType.GROUND_ROVER.value == 10


class TestAutopilotType:
    """Tests for AutopilotType enum."""

    def test_autopilot_types(self) -> None:
        """Test autopilot values."""
        assert AutopilotType.GENERIC.value == 0
        assert AutopilotType.ARDUPILOT.value == 3
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
        assert FlightModePX4.MANUAL.value == 1
        assert FlightModePX4.POSITION.value == 3
        assert FlightModePX4.OFFBOARD.value == 6

    def test_mission_modes(self) -> None:
        """Test mission modes."""
        assert FlightModePX4.MISSION.value == 4
        assert FlightModePX4.RTL.value == 5


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

    def test_default_heartbeat(self) -> None:
        """Test default heartbeat."""
        hb = MAVLinkHeartbeat()
        assert hb.mav_type == MAVType.QUADROTOR
        assert hb.autopilot == AutopilotType.GENERIC
        assert hb.base_mode == 0
        assert hb.custom_mode == 0
        assert hb.system_status == 0
        assert hb.mavlink_version == 3

    def test_is_armed_property(self) -> None:
        """Test is_armed property."""
        # 128 is MAV_MODE_FLAG_SAFETY_ARMED
        armed = MAVLinkHeartbeat(base_mode=128)
        assert armed.is_armed is True

        disarmed = MAVLinkHeartbeat(base_mode=0)
        assert disarmed.is_armed is False


class TestMAVLinkAttitude:
    """Tests for MAVLinkAttitude dataclass."""

    def test_default_attitude(self) -> None:
        """Test default attitude is level."""
        att = MAVLinkAttitude()
        assert att.roll == 0.0
        assert att.pitch == 0.0
        assert att.yaw == 0.0
        assert att.roll_speed == 0.0
        assert att.pitch_speed == 0.0
        assert att.yaw_speed == 0.0

    def test_roll_deg_property(self) -> None:
        """Test degree conversion properties."""
        import math

        att = MAVLinkAttitude(roll=math.pi / 4)
        assert abs(att.roll_deg - 45.0) < 0.1


class TestMAVLinkGPS:
    """Tests for MAVLinkGPS dataclass."""

    def test_default_gps(self) -> None:
        """Test default GPS values."""
        gps = MAVLinkGPS()
        assert gps.fix_type == GPSFixType.NO_GPS
        assert gps.satellites == 0
        assert gps.latitude == 0.0
        assert gps.longitude == 0.0
        assert gps.altitude == 0.0

    def test_has_fix_property(self) -> None:
        """Test has_fix property."""
        no_fix = MAVLinkGPS(fix_type=GPSFixType.NO_FIX)
        assert no_fix.has_fix is False

        has_fix = MAVLinkGPS(fix_type=GPSFixType.FIX_3D)
        assert has_fix.has_fix is True


class TestMAVLinkBattery:
    """Tests for MAVLinkBattery dataclass."""

    def test_default_battery(self) -> None:
        """Test default battery values."""
        bat = MAVLinkBattery()
        assert bat.voltage == 0.0
        assert bat.current == 0.0
        assert bat.remaining == -1

    def test_valid_battery(self) -> None:
        """Test battery with valid readings."""
        bat = MAVLinkBattery(voltage=16.8, current=15.5, remaining=75)
        assert bat.voltage == 16.8
        assert bat.current == 15.5
        assert bat.remaining == 75


class TestMAVLinkStatus:
    """Tests for MAVLinkStatus dataclass."""

    def test_default_status(self) -> None:
        """Test default status."""
        status = MAVLinkStatus()
        assert status.state == MAVLinkState.DISCONNECTED
        assert status.is_armed is False
        assert status.is_connected is False


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
        assert config.timeout == 5.0
        assert config.heartbeat_interval == 1.0

    def test_serial_connection(self) -> None:
        """Test serial connection string."""
        config = MAVLinkConfig(
            name="test",
            connection_string="/dev/ttyUSB0",
            baud_rate=115200,
        )
        assert config.is_serial is True
        assert config.is_udp is False

    def test_udp_connection(self) -> None:
        """Test UDP connection string."""
        config = MAVLinkConfig(
            name="test",
            connection_string="udp:192.168.1.1:14550",
        )
        assert config.is_serial is False
        assert config.is_udp is True

    def test_tcp_connection(self) -> None:
        """Test TCP connection string."""
        config = MAVLinkConfig(
            name="test",
            connection_string="tcp:192.168.1.1:5760",
        )
        assert config.is_serial is False
        assert config.is_udp is False
        assert config.is_tcp is True

    def test_stream_rates(self) -> None:
        """Test stream rate configuration."""
        config = MAVLinkConfig(
            name="test",
            attitude_stream_rate=50,
            position_stream_rate=10,
            rc_stream_rate=20,
        )
        assert config.attitude_stream_rate == 50
        assert config.position_stream_rate == 10
        assert config.rc_stream_rate == 20


# =============================================================================
# Controller Initialization Tests
# =============================================================================


class TestMAVLinkControllerInit:
    """Tests for MAVLinkController initialization."""

    def test_basic_init(self) -> None:
        """Test basic initialization."""
        controller = MAVLinkController(name="test", simulated=True)
        assert controller.name == "test"
        assert controller.mavlink_state == MAVLinkState.DISCONNECTED
        assert not controller.is_armed
        assert not controller.is_connected
        assert not controller.is_enabled

    def test_init_with_config(self, default_config: MAVLinkConfig) -> None:
        """Test initialization with config."""
        controller = MAVLinkController(
            name="test",
            config=default_config,
            simulated=True,
        )
        assert controller.mavlink_config == default_config
        assert controller.mavlink_config.connection_string == "udp:127.0.0.1:14550"

    def test_initial_telemetry(self, mavlink_controller: MAVLinkController) -> None:
        """Test initial telemetry values."""
        assert mavlink_controller.attitude.roll == 0.0
        assert mavlink_controller.gps.fix_type == GPSFixType.NO_GPS


# =============================================================================
# Enable/Disable Tests
# =============================================================================


class TestMAVLinkEnableDisable:
    """Tests for enable/disable functionality."""

    def test_enable(self, mavlink_controller: MAVLinkController) -> None:
        """Test enabling controller."""
        mavlink_controller.enable()
        assert mavlink_controller.is_enabled
        # In simulated mode, connects automatically
        assert mavlink_controller.is_connected

    def test_disable(self, enabled_controller: MAVLinkController) -> None:
        """Test disabling controller."""
        enabled_controller.disable()
        assert not enabled_controller.is_enabled
        assert not enabled_controller.is_connected


# =============================================================================
# Connection Tests
# =============================================================================


class TestMAVLinkConnection:
    """Tests for connection management."""

    def test_simulated_connect(self, mavlink_controller: MAVLinkController) -> None:
        """Test simulated connection."""
        mavlink_controller.enable()
        assert mavlink_controller.mavlink_state == MAVLinkState.CONNECTED
        assert mavlink_controller.is_connected

    def test_simulated_disconnect(self, enabled_controller: MAVLinkController) -> None:
        """Test simulated disconnection."""
        enabled_controller.disable()
        assert mavlink_controller.mavlink_state == MAVLinkState.DISCONNECTED

    def test_heartbeat_on_connect(self, enabled_controller: MAVLinkController) -> None:
        """Test heartbeat available after connect."""
        hb = enabled_controller.heartbeat
        assert isinstance(hb, MAVLinkHeartbeat)
        # Simulated defaults
        assert hb.mav_type == MAVType.QUADROTOR


# =============================================================================
# Arming Tests
# =============================================================================


class TestMAVLinkArming:
    """Tests for arm/disarm functionality."""

    @pytest.mark.asyncio
    async def test_arm(self, connected_controller: MAVLinkController) -> None:
        """Test arming via MAVLink."""
        result = await connected_controller.arm()
        assert result is True
        assert connected_controller.is_armed
        assert connected_controller.mavlink_state == MAVLinkState.ARMED

    @pytest.mark.asyncio
    async def test_disarm(self, connected_controller: MAVLinkController) -> None:
        """Test disarming via MAVLink."""
        await connected_controller.arm()
        result = await connected_controller.disarm()
        assert result is True
        assert not connected_controller.is_armed
        assert connected_controller.mavlink_state == MAVLinkState.CONNECTED

    @pytest.mark.asyncio
    async def test_arm_requires_connection(
        self, mavlink_controller: MAVLinkController
    ) -> None:
        """Test arming requires connection."""
        with pytest.raises(RuntimeError, match="Not connected"):
            await mavlink_controller.arm()

    @pytest.mark.asyncio
    async def test_force_arm(self, connected_controller: MAVLinkController) -> None:
        """Test force arming."""
        result = await connected_controller.arm(force=True)
        assert result is True
        assert connected_controller.is_armed


# =============================================================================
# Flight Command Tests
# =============================================================================


class TestMAVLinkTakeoff:
    """Tests for takeoff command."""

    @pytest.mark.asyncio
    async def test_takeoff(self, connected_controller: MAVLinkController) -> None:
        """Test takeoff command."""
        await connected_controller.arm()
        result = await connected_controller.takeoff(altitude=10.0)
        assert result is True
        assert connected_controller.mavlink_state == MAVLinkState.IN_FLIGHT

    @pytest.mark.asyncio
    async def test_takeoff_requires_armed(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test takeoff requires armed state."""
        with pytest.raises(RuntimeError, match="Must be armed"):
            await connected_controller.takeoff(altitude=10.0)

    @pytest.mark.asyncio
    async def test_takeoff_validates_altitude(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test takeoff validates altitude."""
        await connected_controller.arm()
        with pytest.raises(ValueError, match="positive"):
            await connected_controller.takeoff(altitude=-5.0)


class TestMAVLinkLand:
    """Tests for land command."""

    @pytest.mark.asyncio
    async def test_land(self, connected_controller: MAVLinkController) -> None:
        """Test land command."""
        await connected_controller.arm()
        await connected_controller.takeoff(altitude=10.0)
        result = await connected_controller.land()
        assert result is True


class TestMAVLinkRTL:
    """Tests for return to launch."""

    @pytest.mark.asyncio
    async def test_return_to_launch(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test RTL command."""
        await connected_controller.arm()
        await connected_controller.takeoff(altitude=10.0)
        result = await connected_controller.return_to_launch()
        assert result is True


# =============================================================================
# Position Control Tests
# =============================================================================


class TestMAVLinkPositionControl:
    """Tests for position control commands."""

    @pytest.mark.asyncio
    async def test_goto_global(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test global position command."""
        await connected_controller.arm()
        await connected_controller.takeoff(altitude=10.0)
        result = await connected_controller.goto_position_global(
            latitude=37.7749,
            longitude=-122.4194,
            altitude=100.0,
        )
        assert result is True

    @pytest.mark.asyncio
    async def test_goto_local(self, connected_controller: MAVLinkController) -> None:
        """Test local position command."""
        await connected_controller.arm()
        await connected_controller.takeoff(altitude=10.0)
        result = await connected_controller.goto_position_local(
            x=10.0,
            y=20.0,
            z=-30.0,  # NED frame
        )
        assert result is True


class TestMAVLinkVelocityControl:
    """Tests for velocity control commands."""

    @pytest.mark.asyncio
    async def test_set_velocity(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test velocity setpoint command."""
        await connected_controller.arm()
        result = await connected_controller.set_velocity(
            vx=1.0,
            vy=0.5,
            vz=0.0,
        )
        assert result is True

    @pytest.mark.asyncio
    async def test_set_velocity_with_yaw(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test velocity with yaw rate."""
        await connected_controller.arm()
        result = await connected_controller.set_velocity(
            vx=1.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=10.0,
        )
        assert result is True


# =============================================================================
# Flight Mode Tests
# =============================================================================


class TestMAVLinkFlightModes:
    """Tests for flight mode control."""

    @pytest.mark.asyncio
    async def test_set_mode_arducopter(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test setting ArduCopter mode."""
        result = await connected_controller.set_mode(FlightModeArduCopter.LOITER)
        assert result is True

    @pytest.mark.asyncio
    async def test_set_mode_px4(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test setting PX4 mode."""
        # Configure for PX4
        connected_controller._heartbeat.autopilot = AutopilotType.PX4
        result = await connected_controller.set_mode(FlightModePX4.POSITION)
        assert result is True

    @pytest.mark.asyncio
    async def test_get_mode(self, connected_controller: MAVLinkController) -> None:
        """Test getting current mode."""
        mode = connected_controller.current_mode
        assert mode is not None


# =============================================================================
# Telemetry Tests
# =============================================================================


class TestMAVLinkTelemetry:
    """Tests for telemetry data."""

    def test_attitude_telemetry(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test attitude telemetry."""
        att = connected_controller.attitude
        assert isinstance(att, MAVLinkAttitude)

    def test_gps_telemetry(self, connected_controller: MAVLinkController) -> None:
        """Test GPS telemetry."""
        gps = connected_controller.gps
        assert isinstance(gps, MAVLinkGPS)
        # Simulated has fix
        assert gps.has_fix is True

    def test_battery_telemetry(self, connected_controller: MAVLinkController) -> None:
        """Test battery telemetry."""
        bat = connected_controller.battery
        assert isinstance(bat, MAVLinkBattery)
        # Simulated has values
        assert bat.voltage > 0


# =============================================================================
# Status Tests
# =============================================================================


class TestMAVLinkStatusReporting:
    """Tests for status reporting."""

    def test_status(self, connected_controller: MAVLinkController) -> None:
        """Test status reporting."""
        status = connected_controller.status()
        assert isinstance(status, MAVLinkStatus)
        assert status.state == MAVLinkState.CONNECTED
        assert status.is_connected is True

    def test_status_includes_telemetry(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test status includes telemetry data."""
        status = connected_controller.status()
        assert status.attitude is not None
        assert status.gps is not None
        assert status.battery is not None


# =============================================================================
# Emergency Stop Tests
# =============================================================================


class TestMAVLinkEmergencyStop:
    """Tests for emergency stop."""

    def test_emergency_stop(self, connected_controller: MAVLinkController) -> None:
        """Test emergency stop."""
        connected_controller.stop()
        assert connected_controller.mavlink_state == MAVLinkState.EMERGENCY
        assert not connected_controller.is_armed

    @pytest.mark.asyncio
    async def test_emergency_stop_during_flight(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test emergency stop during flight."""
        await connected_controller.arm()
        await connected_controller.takeoff(altitude=10.0)
        connected_controller.stop()
        assert connected_controller.mavlink_state == MAVLinkState.EMERGENCY


# =============================================================================
# AI Integration Tests
# =============================================================================


class TestMAVLinkAIIntegration:
    """Tests for AI integration."""

    def test_as_tools(self, connected_controller: MAVLinkController) -> None:
        """Test generating AI tools."""
        tools = connected_controller.as_tools()
        assert isinstance(tools, list)
        assert len(tools) > 0

    def test_as_tools_names(self, connected_controller: MAVLinkController) -> None:
        """Test tool names."""
        tools = connected_controller.as_tools()
        names = [t.__name__ for t in tools]
        assert "arm_vehicle" in names
        assert "disarm_vehicle" in names
        assert "takeoff" in names
        assert "land" in names


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestCreateMAVLinkController:
    """Tests for create_mavlink_controller factory function."""

    def test_create_default(self) -> None:
        """Test creating controller with defaults."""
        controller = create_mavlink_controller()
        assert controller.name == "mavlink"
        assert controller.mavlink_config.connection_string == "udp:127.0.0.1:14550"

    def test_create_custom_connection(self) -> None:
        """Test creating controller with custom connection."""
        controller = create_mavlink_controller(
            connection="tcp:192.168.1.1:5760",
            name="custom",
        )
        assert controller.name == "custom"
        assert controller.mavlink_config.connection_string == "tcp:192.168.1.1:5760"

    def test_create_simulated(self) -> None:
        """Test creating simulated controller."""
        controller = create_mavlink_controller(simulated=True)
        controller.enable()
        assert controller.is_connected


# =============================================================================
# Edge Case Tests
# =============================================================================


class TestMAVLinkEdgeCases:
    """Tests for edge cases and error handling."""

    @pytest.mark.asyncio
    async def test_double_arm(self, connected_controller: MAVLinkController) -> None:
        """Test double arming is idempotent."""
        await connected_controller.arm()
        result = await connected_controller.arm()
        assert result is True
        assert connected_controller.is_armed

    @pytest.mark.asyncio
    async def test_double_disarm(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test double disarm is idempotent."""
        result = await connected_controller.disarm()
        assert result is True
        assert not connected_controller.is_armed

    def test_enable_disable_cycle(
        self, mavlink_controller: MAVLinkController
    ) -> None:
        """Test enable/disable cycle."""
        for _ in range(3):
            mavlink_controller.enable()
            assert mavlink_controller.is_enabled
            mavlink_controller.disable()
            assert not mavlink_controller.is_enabled

    @pytest.mark.asyncio
    async def test_commands_without_gps_fix(
        self, connected_controller: MAVLinkController
    ) -> None:
        """Test commands work in simulated mode without real GPS."""
        await connected_controller.arm()
        await connected_controller.takeoff(altitude=10.0)
        # Should work in simulated mode
        assert connected_controller.mavlink_state == MAVLinkState.IN_FLIGHT
