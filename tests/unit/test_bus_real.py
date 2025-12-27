"""Unit tests for real hardware bus implementations.

These tests verify the class structure and initialization of real bus
implementations. They do NOT require actual hardware - they test the
code paths that would run before touching hardware.
"""

from __future__ import annotations

import pytest

from robo_infra.core.bus import (
    I2CConfig,
    PySerialBus,
    SerialConfig,
    SimulatedI2CBus,
    SimulatedSerialBus,
    SimulatedSPIBus,
    SMBus2I2CBus,
    SPIConfig,
    SpiDevSPIBus,
    get_i2c,
    get_serial,
    get_spi,
)
from robo_infra.core.can_bus import (
    CANConfig,
    PythonCANBus,
    SimulatedCANBus,
    get_can,
)


# =============================================================================
# SMBus2I2CBus Tests
# =============================================================================


class TestSMBus2I2CBus:
    """Tests for real I2C bus using smbus2."""

    def test_smbus2_i2c_bus_init(self) -> None:
        """Test SMBus2I2CBus initialization."""
        config = I2CConfig(bus_number=1)
        bus = SMBus2I2CBus(config)

        assert bus.name == "I2C-1"
        assert bus.config == config
        assert bus.config.bus_number == 1
        assert not bus.is_open

    def test_smbus2_i2c_bus_custom_name(self) -> None:
        """Test SMBus2I2CBus with custom name."""
        config = I2CConfig(bus_number=0)
        bus = SMBus2I2CBus(config, name="CustomI2C")

        assert bus.name == "CustomI2C"
        assert bus.config.bus_number == 0

    def test_smbus2_i2c_bus_requires_smbus2(self) -> None:
        """Test that SMBus2I2CBus.open() requires smbus2 library."""
        config = I2CConfig(bus_number=1)
        bus = SMBus2I2CBus(config)

        # Try to open - should fail gracefully if smbus2 not installed
        # or if we're not on Linux with I2C hardware
        try:
            bus.open()
            # If we got here, smbus2 is installed - close it
            bus.close()
        except ImportError as e:
            assert "smbus2" in str(e)
        except OSError:
            # Expected on non-Linux or without hardware
            pass

    def test_smbus2_i2c_bus_not_open_error(self) -> None:
        """Test that operations fail when bus is not open."""
        config = I2CConfig(bus_number=1)
        bus = SMBus2I2CBus(config)

        with pytest.raises(RuntimeError, match="not open"):
            bus.scan()

        with pytest.raises(RuntimeError, match="not open"):
            bus.read(0x40, 1)

        with pytest.raises(RuntimeError, match="not open"):
            bus.write(0x40, b"\x00")


# =============================================================================
# SpiDevSPIBus Tests
# =============================================================================


class TestSpiDevSPIBus:
    """Tests for real SPI bus using spidev."""

    def test_spidev_spi_bus_init(self) -> None:
        """Test SpiDevSPIBus initialization."""
        config = SPIConfig(bus=0, device=0)
        bus = SpiDevSPIBus(config)

        assert bus.name == "SPI-0:0"
        assert bus.config == config
        assert bus.config.bus == 0
        assert bus.config.device == 0
        assert not bus.is_open

    def test_spidev_spi_bus_custom_name(self) -> None:
        """Test SpiDevSPIBus with custom name."""
        config = SPIConfig(bus=1, device=2)
        bus = SpiDevSPIBus(config, name="CustomSPI")

        assert bus.name == "CustomSPI"
        assert bus.config.bus == 1
        assert bus.config.device == 2

    def test_spidev_spi_bus_requires_spidev(self) -> None:
        """Test that SpiDevSPIBus.open() requires spidev library."""
        config = SPIConfig(bus=0, device=0)
        bus = SpiDevSPIBus(config)

        try:
            bus.open()
            bus.close()
        except ImportError as e:
            assert "spidev" in str(e)
        except (OSError, FileNotFoundError):
            # Expected on non-Linux or without hardware
            pass

    def test_spidev_spi_bus_not_open_error(self) -> None:
        """Test that operations fail when bus is not open."""
        config = SPIConfig(bus=0, device=0)
        bus = SpiDevSPIBus(config)

        with pytest.raises(RuntimeError, match="not open"):
            bus.transfer(b"\x00")

        with pytest.raises(RuntimeError, match="not open"):
            bus.set_speed(1000000)


# =============================================================================
# PySerialBus Tests
# =============================================================================


class TestPySerialBus:
    """Tests for real serial bus using pyserial."""

    def test_pyserial_bus_init(self) -> None:
        """Test PySerialBus initialization."""
        config = SerialConfig(port="/dev/ttyUSB0", baudrate=115200)
        bus = PySerialBus(config)

        assert bus.name == "Serial-/dev/ttyUSB0"
        assert bus.config == config
        assert bus.config.port == "/dev/ttyUSB0"
        assert bus.config.baudrate == 115200
        assert not bus.is_open

    def test_pyserial_bus_custom_name(self) -> None:
        """Test PySerialBus with custom name."""
        config = SerialConfig(port="/dev/ttyAMA0", baudrate=9600)
        bus = PySerialBus(config, name="CustomSerial")

        assert bus.name == "CustomSerial"
        assert bus.config.port == "/dev/ttyAMA0"

    def test_pyserial_bus_requires_pyserial(self) -> None:
        """Test that PySerialBus.open() requires pyserial library."""
        config = SerialConfig(port="/dev/null", baudrate=9600)
        bus = PySerialBus(config)

        try:
            bus.open()
            bus.close()
        except ImportError as e:
            assert "pyserial" in str(e)
        except (OSError, FileNotFoundError):
            # Expected if port doesn't exist
            pass

    def test_pyserial_bus_not_open_error(self) -> None:
        """Test that operations fail when bus is not open."""
        config = SerialConfig(port="/dev/ttyUSB0")
        bus = PySerialBus(config)

        with pytest.raises(RuntimeError, match="not open"):
            bus.write(b"test")

        with pytest.raises(RuntimeError, match="not open"):
            bus.read(10)

        with pytest.raises(RuntimeError, match="not open"):
            bus.readline()


# =============================================================================
# PythonCANBus Tests
# =============================================================================


class TestPythonCANBus:
    """Tests for real CAN bus using python-can."""

    def test_python_can_bus_init(self) -> None:
        """Test PythonCANBus initialization."""
        config = CANConfig(channel="can0")
        bus = PythonCANBus(config)

        assert bus.name == "CAN-can0"
        assert bus.config == config
        assert bus.config.channel == "can0"
        assert not bus.is_open

    def test_python_can_bus_custom_name(self) -> None:
        """Test PythonCANBus with custom name."""
        config = CANConfig(channel="vcan0")
        bus = PythonCANBus(config, name="VirtualCAN")

        assert bus.name == "VirtualCAN"
        assert bus.config.channel == "vcan0"

    def test_python_can_bus_requires_python_can(self) -> None:
        """Test that PythonCANBus.open() requires python-can library."""
        from robo_infra.core.exceptions import HardwareNotFoundError

        config = CANConfig(channel="can0")
        bus = PythonCANBus(config)

        try:
            bus.open()
            bus.close()
        except (ImportError, HardwareNotFoundError) as e:
            assert "python-can" in str(e).lower() or "can" in str(e).lower()
        except OSError:
            # Expected if CAN interface doesn't exist
            pass

    def test_python_can_bus_not_open_error(self) -> None:
        """Test that operations fail when bus is not open."""
        from robo_infra.core.exceptions import CommunicationError

        config = CANConfig(channel="can0")
        bus = PythonCANBus(config)

        with pytest.raises(CommunicationError, match="not open"):
            bus.send(arbitration_id=0x123, data=b"\x01\x02")


# =============================================================================
# Factory Function Tests
# =============================================================================


class TestFactoryReturnsRealBus:
    """Tests that factory functions return real bus when library available."""

    def test_get_i2c_returns_real_when_available(self) -> None:
        """Test get_i2c returns SMBus2I2CBus when smbus2 is available."""
        try:
            import smbus2  # noqa: F401

            bus = get_i2c(1, simulate=False)
            assert isinstance(bus, SMBus2I2CBus)
        except ImportError:
            # smbus2 not installed, should get simulated
            import os

            os.environ["ROBO_SIMULATION"] = "true"
            try:
                bus = get_i2c(1)
                assert isinstance(bus, SimulatedI2CBus)
            finally:
                del os.environ["ROBO_SIMULATION"]

    def test_get_spi_returns_real_when_available(self) -> None:
        """Test get_spi returns SpiDevSPIBus when spidev is available."""
        try:
            import spidev  # noqa: F401

            bus = get_spi(0, 0, simulate=False)
            assert isinstance(bus, SpiDevSPIBus)
        except ImportError:
            # spidev not installed, should get simulated
            import os

            os.environ["ROBO_SIMULATION"] = "true"
            try:
                bus = get_spi(0, 0)
                assert isinstance(bus, SimulatedSPIBus)
            finally:
                del os.environ["ROBO_SIMULATION"]

    def test_get_serial_returns_real_when_available(self) -> None:
        """Test get_serial returns PySerialBus when pyserial is available."""
        try:
            import serial  # noqa: F401

            bus = get_serial("/dev/ttyUSB0", simulate=False)
            assert isinstance(bus, PySerialBus)
        except ImportError:
            # pyserial not installed, should get simulated
            import os

            os.environ["ROBO_SIMULATION"] = "true"
            try:
                bus = get_serial("/dev/ttyUSB0")
                assert isinstance(bus, SimulatedSerialBus)
            finally:
                del os.environ["ROBO_SIMULATION"]

    def test_get_can_returns_real_when_available(self) -> None:
        """Test get_can returns PythonCANBus when python-can is available."""
        try:
            import can  # noqa: F401

            bus = get_can("can0", simulate=False)
            assert isinstance(bus, PythonCANBus)
        except ImportError:
            # python-can not installed, should get simulated
            import os

            os.environ["ROBO_SIMULATION"] = "true"
            try:
                bus = get_can("can0")
                assert isinstance(bus, SimulatedCANBus)
            finally:
                del os.environ["ROBO_SIMULATION"]


class TestFactoryForceSimulation:
    """Tests that factory functions respect simulate=True flag."""

    def test_get_i2c_force_simulated(self) -> None:
        """Test get_i2c returns simulated when simulate=True."""
        bus = get_i2c(1, simulate=True)
        assert isinstance(bus, SimulatedI2CBus)

    def test_get_spi_force_simulated(self) -> None:
        """Test get_spi returns simulated when simulate=True."""
        bus = get_spi(0, 0, simulate=True)
        assert isinstance(bus, SimulatedSPIBus)

    def test_get_serial_force_simulated(self) -> None:
        """Test get_serial returns simulated when simulate=True."""
        bus = get_serial("/dev/ttyUSB0", simulate=True)
        assert isinstance(bus, SimulatedSerialBus)

    def test_get_can_force_simulated(self) -> None:
        """Test get_can returns simulated when simulation=True."""
        bus = get_can(channel="can0", simulation=True)
        assert isinstance(bus, SimulatedCANBus)


# =============================================================================
# Export Verification Tests
# =============================================================================


class TestBusExports:
    """Tests that all real bus classes are properly exported."""

    def test_smbus2_i2c_bus_exported_from_core(self) -> None:
        """Test SMBus2I2CBus is exported from robo_infra.core."""
        from robo_infra.core import SMBus2I2CBus as ExportedClass

        assert ExportedClass is SMBus2I2CBus

    def test_spidev_spi_bus_exported_from_core(self) -> None:
        """Test SpiDevSPIBus is exported from robo_infra.core."""
        from robo_infra.core import SpiDevSPIBus as ExportedClass

        assert ExportedClass is SpiDevSPIBus

    def test_pyserial_bus_exported_from_core(self) -> None:
        """Test PySerialBus is exported from robo_infra.core."""
        from robo_infra.core import PySerialBus as ExportedClass

        assert ExportedClass is PySerialBus

    def test_python_can_bus_exported_from_core(self) -> None:
        """Test PythonCANBus is exported from robo_infra.core."""
        from robo_infra.core import PythonCANBus as ExportedClass

        assert ExportedClass is PythonCANBus

    def test_get_can_exported_from_core(self) -> None:
        """Test get_can is exported from robo_infra.core."""
        from robo_infra.core import get_can as exported_func

        assert exported_func is get_can
