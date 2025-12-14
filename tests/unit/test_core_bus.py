"""Tests for robo_infra.core.bus module."""

from __future__ import annotations

import os

import pytest

from robo_infra.core.bus import (
    Bus,
    BusType,
    ByteOrder,
    I2CBus,
    I2CConfig,
    Parity,
    SerialBus,
    SerialConfig,
    SimulatedI2CBus,
    SimulatedSerialBus,
    SimulatedSPIBus,
    SPIBus,
    SPIConfig,
    SPIMode,
    StopBits,
    get_i2c,
    get_serial,
    get_spi,
)


@pytest.fixture(autouse=True)
def _enable_simulation(monkeypatch: pytest.MonkeyPatch) -> None:
    """Enable simulation mode for all tests in this module."""
    monkeypatch.setenv("ROBO_SIMULATION", "true")


class TestEnums:
    """Tests for bus enums."""

    def test_bus_type_values(self) -> None:
        """Test BusType enum values."""
        assert BusType.I2C.value == "i2c"
        assert BusType.SPI.value == "spi"
        assert BusType.UART.value == "uart"
        assert BusType.ONEWIRE.value == "onewire"
        assert BusType.CAN.value == "can"

    def test_spi_mode_values(self) -> None:
        """Test SPIMode enum values."""
        assert SPIMode.MODE_0.value == 0
        assert SPIMode.MODE_1.value == 1
        assert SPIMode.MODE_2.value == 2
        assert SPIMode.MODE_3.value == 3

    def test_parity_values(self) -> None:
        """Test Parity enum values."""
        assert Parity.NONE.value == "N"
        assert Parity.EVEN.value == "E"
        assert Parity.ODD.value == "O"
        assert Parity.MARK.value == "M"
        assert Parity.SPACE.value == "S"

    def test_stop_bits_values(self) -> None:
        """Test StopBits enum values."""
        assert StopBits.ONE.value == 1
        assert StopBits.ONE_POINT_FIVE.value == 1.5
        assert StopBits.TWO.value == 2

    def test_byte_order_values(self) -> None:
        """Test ByteOrder enum values."""
        assert ByteOrder.BIG_ENDIAN.value == "big"
        assert ByteOrder.LITTLE_ENDIAN.value == "little"


class TestConfigs:
    """Tests for bus configuration dataclasses."""

    def test_i2c_config_defaults(self) -> None:
        """Test I2CConfig default values."""
        config = I2CConfig()
        assert config.bus_number == 1
        assert config.frequency == 100_000
        assert config.timeout == 1.0

    def test_i2c_config_custom(self) -> None:
        """Test I2CConfig with custom values."""
        config = I2CConfig(bus_number=2, frequency=400_000, timeout=2.0)
        assert config.bus_number == 2
        assert config.frequency == 400_000
        assert config.timeout == 2.0

    def test_spi_config_defaults(self) -> None:
        """Test SPIConfig default values."""
        config = SPIConfig()
        assert config.bus == 0
        assert config.device == 0
        assert config.max_speed_hz == 1_000_000
        assert config.mode == SPIMode.MODE_0
        assert config.bits_per_word == 8
        assert config.lsb_first is False

    def test_spi_config_custom(self) -> None:
        """Test SPIConfig with custom values."""
        config = SPIConfig(
            bus=1,
            device=2,
            max_speed_hz=500_000,
            mode=SPIMode.MODE_3,
            bits_per_word=16,
            lsb_first=True,
        )
        assert config.bus == 1
        assert config.device == 2
        assert config.max_speed_hz == 500_000
        assert config.mode == SPIMode.MODE_3
        assert config.bits_per_word == 16
        assert config.lsb_first is True

    def test_serial_config_defaults(self) -> None:
        """Test SerialConfig default values."""
        config = SerialConfig()
        assert config.port == "/dev/ttyUSB0"
        assert config.baudrate == 9600
        assert config.bytesize == 8
        assert config.parity == Parity.NONE
        assert config.stopbits == StopBits.ONE
        assert config.timeout == 1.0
        assert config.write_timeout == 1.0
        assert config.xonxoff is False
        assert config.rtscts is False

    def test_serial_config_custom(self) -> None:
        """Test SerialConfig with custom values."""
        config = SerialConfig(
            port="/dev/ttyACM0",
            baudrate=115200,
            bytesize=7,
            parity=Parity.EVEN,
            stopbits=StopBits.TWO,
            timeout=5.0,
        )
        assert config.port == "/dev/ttyACM0"
        assert config.baudrate == 115200
        assert config.bytesize == 7
        assert config.parity == Parity.EVEN
        assert config.stopbits == StopBits.TWO
        assert config.timeout == 5.0


class TestSimulatedI2CBus:
    """Tests for SimulatedI2CBus."""

    def test_create_bus(self) -> None:
        """Test creating simulated I2C bus."""
        bus = SimulatedI2CBus()
        assert bus.name == "SimulatedI2C"
        assert bus.bus_type == BusType.I2C
        assert bus.is_open is False

    def test_create_bus_with_config(self) -> None:
        """Test creating bus with custom config."""
        config = I2CConfig(bus_number=2, frequency=400_000)
        bus = SimulatedI2CBus(config, name="TestBus")
        assert bus.name == "TestBus"
        assert bus.config.bus_number == 2
        assert bus.config.frequency == 400_000

    def test_open_close(self) -> None:
        """Test opening and closing bus."""
        bus = SimulatedI2CBus()
        assert bus.is_open is False

        bus.open()
        assert bus.is_open is True

        bus.close()
        assert bus.is_open is False

    def test_context_manager(self) -> None:
        """Test using bus as context manager."""
        bus = SimulatedI2CBus()

        with bus:
            assert bus.is_open is True

        assert bus.is_open is False

    def test_add_device(self) -> None:
        """Test adding simulated device."""
        bus = SimulatedI2CBus()
        device = bus.add_device(0x40, {0x00: 0x12, 0x01: 0x34}, name="TestDevice")

        assert device.address == 0x40
        assert device.registers[0x00] == 0x12
        assert device.registers[0x01] == 0x34
        assert device.name == "TestDevice"

    def test_remove_device(self) -> None:
        """Test removing simulated device."""
        bus = SimulatedI2CBus()
        bus.add_device(0x40)

        assert bus.get_device(0x40) is not None
        bus.remove_device(0x40)
        assert bus.get_device(0x40) is None

    def test_scan_empty(self) -> None:
        """Test scanning empty bus."""
        bus = SimulatedI2CBus()
        bus.open()
        assert bus.scan() == []

    def test_scan_with_devices(self) -> None:
        """Test scanning bus with devices."""
        bus = SimulatedI2CBus()
        bus.add_device(0x40)
        bus.add_device(0x70)
        bus.add_device(0x20)
        bus.open()

        devices = bus.scan()
        assert devices == [0x20, 0x40, 0x70]  # Sorted

    def test_write(self) -> None:
        """Test writing to device."""
        bus = SimulatedI2CBus()
        bus.open()

        result = bus.write(0x40, bytes([0x01, 0x02, 0x03]))
        assert result == 3

    def test_read(self) -> None:
        """Test reading from device."""
        bus = SimulatedI2CBus()
        bus.open()

        result = bus.read(0x40, 4)
        assert len(result) == 4
        assert result == bytes([0x00, 0x00, 0x00, 0x00])

    def test_write_read_byte(self) -> None:
        """Test write_byte and read_byte convenience methods."""
        bus = SimulatedI2CBus()
        bus.open()

        bus.write_byte(0x40, 0xAB)
        result = bus.read_byte(0x40)
        assert result == 0x00  # Simulated, returns 0

    def test_write_register(self) -> None:
        """Test writing to device register."""
        bus = SimulatedI2CBus()
        device = bus.add_device(0x40)
        bus.open()

        bus.write_register(0x40, 0x10, bytes([0xAB, 0xCD]))

        assert device.registers[0x10] == 0xAB
        assert device.registers[0x11] == 0xCD

    def test_read_register(self) -> None:
        """Test reading from device register."""
        bus = SimulatedI2CBus()
        bus.add_device(0x40, {0x00: 0x12, 0x01: 0x34, 0x02: 0x56})
        bus.open()

        result = bus.read_register(0x40, 0x00, 3)
        assert result == bytes([0x12, 0x34, 0x56])

    def test_read_register_byte(self) -> None:
        """Test read_register_byte convenience method."""
        bus = SimulatedI2CBus()
        bus.add_device(0x40, {0x00: 0xAB})
        bus.open()

        result = bus.read_register_byte(0x40, 0x00)
        assert result == 0xAB

    def test_write_register_byte(self) -> None:
        """Test write_register_byte convenience method."""
        bus = SimulatedI2CBus()
        device = bus.add_device(0x40)
        bus.open()

        bus.write_register_byte(0x40, 0x10, 0xCD)
        assert device.registers[0x10] == 0xCD

    def test_read_register_word_big_endian(self) -> None:
        """Test reading 16-bit word in big endian."""
        bus = SimulatedI2CBus()
        bus.add_device(0x40, {0x00: 0x12, 0x01: 0x34})
        bus.open()

        result = bus.read_register_word(0x40, 0x00, ByteOrder.BIG_ENDIAN)
        assert result == 0x1234

    def test_read_register_word_little_endian(self) -> None:
        """Test reading 16-bit word in little endian."""
        bus = SimulatedI2CBus()
        bus.add_device(0x40, {0x00: 0x34, 0x01: 0x12})
        bus.open()

        result = bus.read_register_word(0x40, 0x00, ByteOrder.LITTLE_ENDIAN)
        assert result == 0x1234

    def test_write_register_word_big_endian(self) -> None:
        """Test writing 16-bit word in big endian."""
        bus = SimulatedI2CBus()
        device = bus.add_device(0x40)
        bus.open()

        bus.write_register_word(0x40, 0x10, 0x1234, ByteOrder.BIG_ENDIAN)
        assert device.registers[0x10] == 0x12
        assert device.registers[0x11] == 0x34

    def test_write_register_word_little_endian(self) -> None:
        """Test writing 16-bit word in little endian."""
        bus = SimulatedI2CBus()
        device = bus.add_device(0x40)
        bus.open()

        bus.write_register_word(0x40, 0x10, 0x1234, ByteOrder.LITTLE_ENDIAN)
        assert device.registers[0x10] == 0x34
        assert device.registers[0x11] == 0x12

    def test_read_unknown_device(self) -> None:
        """Test reading from unknown device address."""
        bus = SimulatedI2CBus()
        bus.open()

        result = bus.read_register(0x99, 0x00, 2)
        assert result == bytes([0x00, 0x00])


class TestSimulatedSPIBus:
    """Tests for SimulatedSPIBus."""

    def test_create_bus(self) -> None:
        """Test creating simulated SPI bus."""
        bus = SimulatedSPIBus()
        assert bus.name == "SimulatedSPI"
        assert bus.bus_type == BusType.SPI
        assert bus.is_open is False

    def test_create_bus_with_config(self) -> None:
        """Test creating bus with custom config."""
        config = SPIConfig(bus=1, device=2, max_speed_hz=500_000)
        bus = SimulatedSPIBus(config, name="TestSPI")
        assert bus.name == "TestSPI"
        assert bus.config.bus == 1
        assert bus.config.device == 2

    def test_open_close(self) -> None:
        """Test opening and closing bus."""
        bus = SimulatedSPIBus()
        assert bus.is_open is False

        bus.open()
        assert bus.is_open is True

        bus.close()
        assert bus.is_open is False

    def test_transfer_loopback(self) -> None:
        """Test transfer returns input in loopback mode."""
        bus = SimulatedSPIBus()
        bus.open()

        data = bytes([0x01, 0x02, 0x03])
        result = bus.transfer(data)
        assert result == data

    def test_transfer_with_response(self) -> None:
        """Test transfer with configured response."""
        bus = SimulatedSPIBus()
        bus.open()
        bus.set_response(bytes([0xAA, 0xBB, 0xCC]))

        result = bus.transfer(bytes([0x01, 0x02, 0x03]))
        assert result == bytes([0xAA, 0xBB, 0xCC])

    def test_transfer_response_shorter_than_input(self) -> None:
        """Test transfer with shorter response pads with zeros."""
        bus = SimulatedSPIBus()
        bus.open()
        bus.set_response(bytes([0xAA]))

        result = bus.transfer(bytes([0x01, 0x02, 0x03]))
        assert result == bytes([0xAA, 0x00, 0x00])

    def test_write_convenience(self) -> None:
        """Test write convenience method."""
        bus = SimulatedSPIBus()
        bus.open()
        bus.write(bytes([0x01, 0x02]))  # No return value, just shouldn't error

    def test_read_convenience(self) -> None:
        """Test read convenience method."""
        bus = SimulatedSPIBus()
        bus.open()
        bus.set_response(bytes([0xAA, 0xBB, 0xCC]))

        result = bus.read(3)
        assert result == bytes([0xAA, 0xBB, 0xCC])

    def test_read_with_fill_byte(self) -> None:
        """Test read sends fill bytes."""
        bus = SimulatedSPIBus()
        bus.open()

        # In loopback, we get back the fill bytes
        result = bus.read(3, fill=0xFF)
        assert result == bytes([0xFF, 0xFF, 0xFF])

    def test_set_speed(self) -> None:
        """Test setting clock speed."""
        bus = SimulatedSPIBus()
        bus.open()

        bus.set_speed(500_000)
        assert bus.config.max_speed_hz == 500_000

    def test_set_mode(self) -> None:
        """Test setting SPI mode."""
        bus = SimulatedSPIBus()
        bus.open()

        bus.set_mode(SPIMode.MODE_2)
        assert bus.config.mode == SPIMode.MODE_2


class TestSimulatedSerialBus:
    """Tests for SimulatedSerialBus."""

    def test_create_bus(self) -> None:
        """Test creating simulated serial bus."""
        bus = SimulatedSerialBus()
        assert bus.name == "SimulatedSerial"
        assert bus.bus_type == BusType.UART
        assert bus.is_open is False

    def test_create_bus_with_config(self) -> None:
        """Test creating bus with custom config."""
        config = SerialConfig(port="/dev/ttyACM0", baudrate=115200)
        bus = SimulatedSerialBus(config, name="TestSerial")
        assert bus.name == "TestSerial"
        assert bus.config.port == "/dev/ttyACM0"
        assert bus.config.baudrate == 115200

    def test_open_close(self) -> None:
        """Test opening and closing bus."""
        bus = SimulatedSerialBus()
        assert bus.is_open is False

        bus.open()
        assert bus.is_open is True

        bus.close()
        assert bus.is_open is False

    def test_write_bytes(self) -> None:
        """Test writing bytes."""
        bus = SimulatedSerialBus()
        bus.open()

        result = bus.write(bytes([0x01, 0x02, 0x03]))
        assert result == 3

        tx_data = bus.get_tx_data()
        assert tx_data == bytes([0x01, 0x02, 0x03])

    def test_write_string(self) -> None:
        """Test writing string."""
        bus = SimulatedSerialBus()
        bus.open()

        result = bus.write("AT\r\n")
        assert result == 4

        tx_data = bus.get_tx_data()
        assert tx_data == b"AT\r\n"

    def test_read(self) -> None:
        """Test reading bytes."""
        bus = SimulatedSerialBus()
        bus.open()
        bus.set_rx_data(bytes([0xAA, 0xBB, 0xCC, 0xDD]))

        result = bus.read(2)
        assert result == bytes([0xAA, 0xBB])

        # Reading more gets remaining data
        result = bus.read(2)
        assert result == bytes([0xCC, 0xDD])

    def test_readline(self) -> None:
        """Test reading line."""
        bus = SimulatedSerialBus()
        bus.open()
        bus.set_rx_data(b"OK\r\nERROR\r\n")

        line1 = bus.readline()
        assert line1 == b"OK\r\n"

        line2 = bus.readline()
        assert line2 == b"ERROR\r\n"

    def test_read_until(self) -> None:
        """Test reading until terminator."""
        bus = SimulatedSerialBus()
        bus.open()
        bus.set_rx_data(b"DATA>>>END")

        result = bus.read_until(b">>>")
        assert result == b"DATA>>>"

    def test_read_until_no_terminator(self) -> None:
        """Test read_until returns all data if no terminator."""
        bus = SimulatedSerialBus()
        bus.open()
        bus.set_rx_data(b"NOTERM")

        result = bus.read_until(b"\n")
        assert result == b"NOTERM"

    def test_in_waiting(self) -> None:
        """Test in_waiting property."""
        bus = SimulatedSerialBus()
        bus.open()

        assert bus.in_waiting == 0

        bus.set_rx_data(b"12345")
        assert bus.in_waiting == 5

        bus.read(2)
        assert bus.in_waiting == 3

    def test_out_waiting(self) -> None:
        """Test out_waiting property."""
        bus = SimulatedSerialBus()
        bus.open()

        assert bus.out_waiting == 0

        bus.write(b"12345")
        assert bus.out_waiting == 5

    def test_flush(self) -> None:
        """Test flush clears TX buffer."""
        bus = SimulatedSerialBus()
        bus.open()
        bus.write(b"DATA")

        bus.flush()
        assert bus.out_waiting == 0

    def test_reset_input_buffer(self) -> None:
        """Test reset_input_buffer clears RX."""
        bus = SimulatedSerialBus()
        bus.open()
        bus.set_rx_data(b"DATA")

        bus.reset_input_buffer()
        assert bus.in_waiting == 0

    def test_reset_output_buffer(self) -> None:
        """Test reset_output_buffer clears TX."""
        bus = SimulatedSerialBus()
        bus.open()
        bus.write(b"DATA")

        bus.reset_output_buffer()
        assert bus.out_waiting == 0

    def test_set_baudrate(self) -> None:
        """Test setting baud rate."""
        bus = SimulatedSerialBus()
        bus.open()

        bus.set_baudrate(115200)
        assert bus.config.baudrate == 115200


class TestFactoryFunctions:
    """Tests for bus factory functions."""

    def test_get_i2c_default(self) -> None:
        """Test get_i2c with defaults."""
        bus = get_i2c()
        assert isinstance(bus, I2CBus)
        assert bus.config.bus_number == 1
        assert bus.config.frequency == 100_000

    def test_get_i2c_custom(self) -> None:
        """Test get_i2c with custom values."""
        bus = get_i2c(bus_number=2, frequency=400_000)
        assert bus.config.bus_number == 2
        assert bus.config.frequency == 400_000

    def test_get_i2c_force_simulate(self) -> None:
        """Test get_i2c with forced simulation."""
        bus = get_i2c(simulate=True)
        assert isinstance(bus, SimulatedI2CBus)

    def test_get_spi_default(self) -> None:
        """Test get_spi with defaults."""
        bus = get_spi()
        assert isinstance(bus, SPIBus)
        assert bus.config.bus == 0
        assert bus.config.device == 0

    def test_get_spi_custom(self) -> None:
        """Test get_spi with custom values."""
        bus = get_spi(bus=1, device=2, max_speed_hz=500_000, mode=SPIMode.MODE_3)
        assert bus.config.bus == 1
        assert bus.config.device == 2
        assert bus.config.max_speed_hz == 500_000
        assert bus.config.mode == SPIMode.MODE_3

    def test_get_spi_force_simulate(self) -> None:
        """Test get_spi with forced simulation."""
        bus = get_spi(simulate=True)
        assert isinstance(bus, SimulatedSPIBus)

    def test_get_serial_default(self) -> None:
        """Test get_serial with defaults."""
        bus = get_serial()
        assert isinstance(bus, SerialBus)
        assert bus.config.port == "/dev/ttyUSB0"
        assert bus.config.baudrate == 9600

    def test_get_serial_custom(self) -> None:
        """Test get_serial with custom values."""
        bus = get_serial(port="/dev/ttyACM0", baudrate=115200, timeout=5.0)
        assert bus.config.port == "/dev/ttyACM0"
        assert bus.config.baudrate == 115200
        assert bus.config.timeout == 5.0

    def test_get_serial_force_simulate(self) -> None:
        """Test get_serial with forced simulation."""
        bus = get_serial(simulate=True)
        assert isinstance(bus, SimulatedSerialBus)


class TestBusBaseClass:
    """Tests for Bus abstract base class behavior."""

    def test_bus_is_abstract(self) -> None:
        """Test that Bus cannot be instantiated directly."""
        # We can't instantiate Bus directly, but we test via SimulatedI2CBus
        bus = SimulatedI2CBus()
        assert isinstance(bus, Bus)

    def test_bus_name_property(self) -> None:
        """Test name property."""
        bus = SimulatedI2CBus(name="MyBus")
        assert bus.name == "MyBus"

    def test_bus_default_name(self) -> None:
        """Test default name uses class name."""
        bus = SimulatedI2CBus()
        assert "Simulated" in bus.name
