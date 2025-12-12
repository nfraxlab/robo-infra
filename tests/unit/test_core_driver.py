"""Tests for robo_infra.core.driver module."""

from __future__ import annotations

import pytest

from robo_infra.core.driver import (
    ChannelConfig,
    ChannelMode,
    Driver,
    DriverConfig,
    DriverManager,
    DriverState,
    SimulatedDriver,
    clear_driver_registry,
    get_driver,
    list_drivers,
    register_driver,
)
from robo_infra.core.exceptions import DisabledError, HardwareNotFoundError


class TestEnums:
    """Tests for driver enums."""

    def test_driver_state_values(self) -> None:
        """Test DriverState enum values."""
        assert DriverState.DISCONNECTED.value == "disconnected"
        assert DriverState.CONNECTING.value == "connecting"
        assert DriverState.CONNECTED.value == "connected"
        assert DriverState.ERROR.value == "error"
        assert DriverState.DISABLED.value == "disabled"

    def test_channel_mode_values(self) -> None:
        """Test ChannelMode enum values."""
        assert ChannelMode.OUTPUT.value == "output"
        assert ChannelMode.INPUT.value == "input"
        assert ChannelMode.PWM.value == "pwm"
        assert ChannelMode.ANALOG.value == "analog"
        assert ChannelMode.DISABLED.value == "disabled"


class TestConfigs:
    """Tests for configuration dataclasses."""

    def test_channel_config_defaults(self) -> None:
        """Test ChannelConfig default values."""
        config = ChannelConfig()
        assert config.mode == ChannelMode.OUTPUT
        assert config.min_value == 0.0
        assert config.max_value == 1.0
        assert config.default_value == 0.0
        assert config.inverted is False
        assert config.name is None
        assert config.metadata == {}

    def test_channel_config_custom(self) -> None:
        """Test ChannelConfig with custom values."""
        config = ChannelConfig(
            mode=ChannelMode.PWM,
            min_value=-1.0,
            max_value=1.0,
            default_value=0.5,
            inverted=True,
            name="servo1",
            metadata={"offset": 10},
        )
        assert config.mode == ChannelMode.PWM
        assert config.min_value == -1.0
        assert config.max_value == 1.0
        assert config.default_value == 0.5
        assert config.inverted is True
        assert config.name == "servo1"
        assert config.metadata == {"offset": 10}

    def test_driver_config_defaults(self) -> None:
        """Test DriverConfig default values."""
        config = DriverConfig()
        assert config.name == "Driver"
        assert config.channels == 1
        assert config.frequency is None
        assert config.channel_configs == {}
        assert config.auto_connect is False
        assert config.metadata == {}

    def test_driver_config_custom(self) -> None:
        """Test DriverConfig with custom values."""
        ch_config = ChannelConfig(name="ch0")
        config = DriverConfig(
            name="MyDriver",
            channels=8,
            frequency=50,
            channel_configs={0: ch_config},
            auto_connect=True,
            metadata={"model": "v2"},
        )
        assert config.name == "MyDriver"
        assert config.channels == 8
        assert config.frequency == 50
        assert config.channel_configs[0] == ch_config
        assert config.auto_connect is True
        assert config.metadata == {"model": "v2"}


class TestDriverRegistry:
    """Tests for driver registry functions."""

    def setup_method(self) -> None:
        """Clear registry before each test."""
        clear_driver_registry()

    def teardown_method(self) -> None:
        """Clear registry after each test."""
        clear_driver_registry()

    def test_register_driver(self) -> None:
        """Test registering a driver."""

        @register_driver("test_driver")
        class TestDriver(SimulatedDriver):
            pass

        assert "test_driver" in list_drivers()

    def test_get_driver(self) -> None:
        """Test getting a registered driver."""

        @register_driver("my_driver")
        class MyDriver(SimulatedDriver):
            pass

        driver_cls = get_driver("my_driver")
        assert driver_cls is MyDriver

    def test_get_driver_not_found(self) -> None:
        """Test getting unregistered driver raises error."""
        with pytest.raises(HardwareNotFoundError) as exc_info:
            get_driver("nonexistent")
        assert "nonexistent" in str(exc_info.value)

    def test_list_drivers_empty(self) -> None:
        """Test listing drivers when empty."""
        assert list_drivers() == []

    def test_list_drivers(self) -> None:
        """Test listing registered drivers."""

        @register_driver("driver_a")
        class DriverA(SimulatedDriver):
            pass

        @register_driver("driver_b")
        class DriverB(SimulatedDriver):
            pass

        drivers = list_drivers()
        assert "driver_a" in drivers
        assert "driver_b" in drivers

    def test_register_overwrites(self) -> None:
        """Test that registering same name overwrites."""

        @register_driver("same_name")
        class Driver1(SimulatedDriver):
            pass

        @register_driver("same_name")
        class Driver2(SimulatedDriver):
            pass

        driver_cls = get_driver("same_name")
        assert driver_cls is Driver2


class TestSimulatedDriver:
    """Tests for SimulatedDriver."""

    def test_create_driver(self) -> None:
        """Test creating a simulated driver."""
        driver = SimulatedDriver()
        assert driver.name == "SimulatedDriver"
        assert driver.channels == 16
        assert driver.state == DriverState.DISCONNECTED
        assert driver.is_connected is False

    def test_create_driver_with_name(self) -> None:
        """Test creating driver with custom name."""
        driver = SimulatedDriver(name="TestDriver", channels=8)
        assert driver.name == "TestDriver"
        assert driver.channels == 8

    def test_create_driver_with_config(self) -> None:
        """Test creating driver with config."""
        config = DriverConfig(name="ConfiguredDriver", channels=4, frequency=100)
        driver = SimulatedDriver(config=config)
        assert driver.name == "ConfiguredDriver"
        assert driver.channels == 4
        assert driver.frequency == 100

    def test_connect_disconnect(self) -> None:
        """Test connecting and disconnecting."""
        driver = SimulatedDriver()
        assert driver.is_connected is False

        driver.connect()
        assert driver.is_connected is True
        assert driver.state == DriverState.CONNECTED

        driver.disconnect()
        assert driver.is_connected is False
        assert driver.state == DriverState.DISCONNECTED

    def test_context_manager(self) -> None:
        """Test using driver as context manager."""
        driver = SimulatedDriver()

        with driver:
            assert driver.is_connected is True

        assert driver.is_connected is False

    def test_enable_disable(self) -> None:
        """Test enabling and disabling driver."""
        driver = SimulatedDriver()
        driver.connect()

        assert driver.is_enabled is True

        driver.disable()
        assert driver.is_enabled is False
        assert driver.state == DriverState.DISABLED

        driver.enable()
        assert driver.is_enabled is True
        assert driver.state == DriverState.CONNECTED

    def test_set_channel(self) -> None:
        """Test setting channel value."""
        driver = SimulatedDriver(channels=8)
        driver.connect()

        driver.set_channel(0, 0.5)
        assert driver.get_channel(0) == 0.5

    def test_set_channel_clamps_value(self) -> None:
        """Test that set_channel clamps to min/max."""
        driver = SimulatedDriver(channels=8)
        driver.connect()

        # Default config is 0.0 to 1.0
        driver.set_channel(0, 1.5)
        assert driver.get_channel(0) == 1.0

        driver.set_channel(0, -0.5)
        assert driver.get_channel(0) == 0.0

    def test_set_channel_with_custom_range(self) -> None:
        """Test set_channel with custom min/max."""
        config = ChannelConfig(min_value=-1.0, max_value=1.0)
        driver = SimulatedDriver(channels=8)
        driver.set_channel_config(0, config)
        driver.connect()

        driver.set_channel(0, -0.5)
        assert driver.get_channel(0) == -0.5

        driver.set_channel(0, -2.0)
        assert driver.get_channel(0) == -1.0

    def test_set_channel_inverted(self) -> None:
        """Test set_channel with inverted config."""
        config = ChannelConfig(inverted=True)
        driver = SimulatedDriver(channels=8)
        driver.set_channel_config(0, config)
        driver.connect()

        driver.set_channel(0, 0.25)
        assert driver.get_channel(0) == 0.75  # Inverted: 1.0 - 0.25 = 0.75

    def test_set_channel_disabled_raises(self) -> None:
        """Test that set_channel raises when disabled."""
        driver = SimulatedDriver()
        driver.connect()
        driver.disable()

        with pytest.raises(DisabledError):
            driver.set_channel(0, 0.5)

    def test_set_channel_invalid_raises(self) -> None:
        """Test that invalid channel raises ValueError."""
        driver = SimulatedDriver(channels=8)
        driver.connect()

        with pytest.raises(ValueError):
            driver.set_channel(10, 0.5)

        with pytest.raises(ValueError):
            driver.set_channel(-1, 0.5)

    def test_get_channel_invalid_raises(self) -> None:
        """Test that invalid channel raises ValueError."""
        driver = SimulatedDriver(channels=8)

        with pytest.raises(ValueError):
            driver.get_channel(10)

    def test_set_all_channels(self) -> None:
        """Test setting all channels."""
        driver = SimulatedDriver(channels=4)
        driver.connect()

        driver.set_all_channels(0.75)

        for ch in range(4):
            assert driver.get_channel(ch) == 0.75

    def test_get_all_channels(self) -> None:
        """Test getting all channel values."""
        driver = SimulatedDriver(channels=4)
        driver.connect()

        driver.set_channel(0, 0.1)
        driver.set_channel(1, 0.2)
        driver.set_channel(2, 0.3)
        driver.set_channel(3, 0.4)

        values = driver.get_all_channels()
        assert values == {0: 0.1, 1: 0.2, 2: 0.3, 3: 0.4}

    def test_get_channel_config(self) -> None:
        """Test getting channel config."""
        config = ChannelConfig(name="test_channel")
        driver = SimulatedDriver(channels=8)
        driver.set_channel_config(0, config)

        result = driver.get_channel_config(0)
        assert result.name == "test_channel"

    def test_set_frequency(self) -> None:
        """Test setting frequency."""
        driver = SimulatedDriver()
        driver.connect()

        driver.set_frequency(50)
        assert driver.frequency == 50

    def test_default_channel_values(self) -> None:
        """Test channels initialize to default values."""
        ch_config = ChannelConfig(default_value=0.5)
        config = DriverConfig(
            channels=4,
            channel_configs={0: ch_config, 1: ch_config},
        )
        driver = SimulatedDriver(config=config)

        assert driver._channel_values[0] == 0.5
        assert driver._channel_values[1] == 0.5
        assert driver._channel_values[2] == 0.0  # Default
        assert driver._channel_values[3] == 0.0


class TestDriverManager:
    """Tests for DriverManager."""

    def test_create_manager(self) -> None:
        """Test creating empty manager."""
        manager = DriverManager()
        assert len(manager) == 0
        assert manager.list_drivers() == []

    def test_add_driver(self) -> None:
        """Test adding driver to manager."""
        manager = DriverManager()
        driver = SimulatedDriver(name="test")

        manager.add_driver("test", driver)

        assert len(manager) == 1
        assert "test" in manager
        assert manager.list_drivers() == ["test"]

    def test_remove_driver(self) -> None:
        """Test removing driver from manager."""
        manager = DriverManager()
        driver = SimulatedDriver(name="test")
        manager.add_driver("test", driver)

        removed = manager.remove_driver("test")

        assert removed is driver
        assert "test" not in manager
        assert len(manager) == 0

    def test_remove_nonexistent_driver(self) -> None:
        """Test removing nonexistent driver returns None."""
        manager = DriverManager()
        result = manager.remove_driver("nonexistent")
        assert result is None

    def test_get_driver(self) -> None:
        """Test getting driver by name."""
        manager = DriverManager()
        driver = SimulatedDriver(name="test")
        manager.add_driver("test", driver)

        result = manager.get_driver("test")
        assert result is driver

    def test_get_driver_not_found(self) -> None:
        """Test getting nonexistent driver raises error."""
        manager = DriverManager()

        with pytest.raises(HardwareNotFoundError):
            manager.get_driver("nonexistent")

    def test_connect_all(self) -> None:
        """Test connecting all drivers."""
        manager = DriverManager()
        driver1 = SimulatedDriver(name="d1")
        driver2 = SimulatedDriver(name="d2")
        manager.add_driver("d1", driver1)
        manager.add_driver("d2", driver2)

        manager.connect_all()

        assert driver1.is_connected
        assert driver2.is_connected

    def test_disconnect_all(self) -> None:
        """Test disconnecting all drivers."""
        manager = DriverManager()
        driver1 = SimulatedDriver(name="d1")
        driver2 = SimulatedDriver(name="d2")
        manager.add_driver("d1", driver1)
        manager.add_driver("d2", driver2)
        manager.connect_all()

        manager.disconnect_all()

        assert not driver1.is_connected
        assert not driver2.is_connected

    def test_enable_disable_all(self) -> None:
        """Test enabling/disabling all drivers."""
        manager = DriverManager()
        driver1 = SimulatedDriver(name="d1")
        driver2 = SimulatedDriver(name="d2")
        manager.add_driver("d1", driver1)
        manager.add_driver("d2", driver2)
        manager.connect_all()

        manager.disable_all()
        assert not driver1.is_enabled
        assert not driver2.is_enabled

        manager.enable_all()
        assert driver1.is_enabled
        assert driver2.is_enabled

    def test_set_channel(self) -> None:
        """Test setting channel through manager."""
        manager = DriverManager()
        driver = SimulatedDriver(name="test", channels=8)
        manager.add_driver("test", driver)
        driver.connect()

        manager.set_channel("test", 0, 0.5)

        assert driver.get_channel(0) == 0.5

    def test_get_channel(self) -> None:
        """Test getting channel through manager."""
        manager = DriverManager()
        driver = SimulatedDriver(name="test", channels=8)
        manager.add_driver("test", driver)
        driver.connect()
        driver.set_channel(0, 0.75)

        value = manager.get_channel("test", 0)

        assert value == 0.75

    def test_context_manager(self) -> None:
        """Test using manager as context manager."""
        manager = DriverManager()
        driver1 = SimulatedDriver(name="d1")
        driver2 = SimulatedDriver(name="d2")
        manager.add_driver("d1", driver1)
        manager.add_driver("d2", driver2)

        with manager:
            assert driver1.is_connected
            assert driver2.is_connected

        assert not driver1.is_connected
        assert not driver2.is_connected

    def test_iteration(self) -> None:
        """Test iterating over manager."""
        manager = DriverManager()
        manager.add_driver("a", SimulatedDriver(name="a"))
        manager.add_driver("b", SimulatedDriver(name="b"))

        names = list(manager)
        assert "a" in names
        assert "b" in names

    def test_contains(self) -> None:
        """Test 'in' operator on manager."""
        manager = DriverManager()
        manager.add_driver("test", SimulatedDriver())

        assert "test" in manager
        assert "other" not in manager


class TestDriverBaseClass:
    """Tests for Driver abstract base class behavior."""

    def test_driver_is_abstract(self) -> None:
        """Test that Driver methods are properly abstract."""
        # SimulatedDriver is our concrete implementation
        driver = SimulatedDriver()
        assert isinstance(driver, Driver)

    def test_driver_properties(self) -> None:
        """Test driver property accessors."""
        driver = SimulatedDriver(name="Test", channels=8)

        assert driver.name == "Test"
        assert driver.channels == 8
        assert driver.state == DriverState.DISCONNECTED
        assert driver.is_connected is False
        assert driver.is_enabled is True
        assert driver.frequency is None

    def test_driver_config_property(self) -> None:
        """Test driver config property."""
        config = DriverConfig(name="Configured", channels=4)
        driver = SimulatedDriver(config=config)

        assert driver.config is config
        assert driver.config.name == "Configured"
