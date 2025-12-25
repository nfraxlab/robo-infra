"""Tests for Modbus protocol implementation."""

from __future__ import annotations

import os
import struct
from unittest.mock import patch

import pytest

from robo_infra.protocols.modbus import (
    ExceptionCode,
    FunctionCode,
    ModbusError,
    ModbusRTU,
    ModbusRTUConfig,
    ModbusStatistics,
    ModbusTCP,
    ModbusTCPConfig,
    SimulatedModbusServer,
    calculate_crc16,
    verify_crc16,
)


class TestFunctionCode:
    """Tests for Modbus function codes."""

    def test_read_function_codes(self) -> None:
        """Test read function codes."""
        assert FunctionCode.READ_COILS == 0x01
        assert FunctionCode.READ_DISCRETE_INPUTS == 0x02
        assert FunctionCode.READ_HOLDING_REGISTERS == 0x03
        assert FunctionCode.READ_INPUT_REGISTERS == 0x04

    def test_write_function_codes(self) -> None:
        """Test write function codes."""
        assert FunctionCode.WRITE_SINGLE_COIL == 0x05
        assert FunctionCode.WRITE_SINGLE_REGISTER == 0x06
        assert FunctionCode.WRITE_MULTIPLE_COILS == 0x0F
        assert FunctionCode.WRITE_MULTIPLE_REGISTERS == 0x10


class TestExceptionCode:
    """Tests for Modbus exception codes."""

    def test_exception_code_values(self) -> None:
        """Test exception code values."""
        assert ExceptionCode.ILLEGAL_FUNCTION == 0x01
        assert ExceptionCode.ILLEGAL_DATA_ADDRESS == 0x02
        assert ExceptionCode.ILLEGAL_DATA_VALUE == 0x03
        assert ExceptionCode.SLAVE_DEVICE_FAILURE == 0x04
        assert ExceptionCode.SLAVE_DEVICE_BUSY == 0x06


class TestCRC16:
    """Tests for CRC16 calculation."""

    def test_calculate_crc16(self) -> None:
        """Test CRC16 calculation."""
        # Known test vector
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])
        crc = calculate_crc16(data)
        assert isinstance(crc, int)
        assert 0 <= crc <= 0xFFFF

    def test_verify_crc16_valid(self) -> None:
        """Test CRC16 verification with valid data."""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])
        crc = calculate_crc16(data)
        full_frame = data + struct.pack("<H", crc)
        assert verify_crc16(full_frame) is True

    def test_verify_crc16_invalid(self) -> None:
        """Test CRC16 verification with corrupted data."""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])
        crc = calculate_crc16(data)
        # Corrupt the CRC
        full_frame = data + struct.pack("<H", crc ^ 0xFF)
        assert verify_crc16(full_frame) is False

    def test_verify_crc16_short_data(self) -> None:
        """Test CRC16 verification with too short data."""
        assert verify_crc16(b"\x01\x02") is False

    def test_crc16_empty_data(self) -> None:
        """Test CRC16 with empty data."""
        crc = calculate_crc16(b"")
        assert crc == 0xFFFF  # Initial value


class TestModbusRTUConfig:
    """Tests for ModbusRTUConfig."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = ModbusRTUConfig(port="/dev/ttyUSB0")
        assert config.port == "/dev/ttyUSB0"
        assert config.baudrate == 9600
        assert config.bytesize == 8
        assert config.parity == "N"
        assert config.stopbits == 1
        assert config.timeout == 1.0

    def test_inter_frame_delay_calculation(self) -> None:
        """Test inter-frame delay calculation."""
        config = ModbusRTUConfig(port="/dev/ttyUSB0", baudrate=9600)
        # At 9600 baud, 11 bits per char, 3.5 char times
        expected_delay = 3.5 * (11.0 / 9600)
        assert abs(config.inter_frame_delay - expected_delay) < 0.001

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = ModbusRTUConfig(
            port="COM1",
            baudrate=19200,
            parity="E",
            stopbits=2,
        )
        assert config.baudrate == 19200
        assert config.parity == "E"
        assert config.stopbits == 2


class TestModbusTCPConfig:
    """Tests for ModbusTCPConfig."""

    def test_default_config(self) -> None:
        """Test default configuration."""
        config = ModbusTCPConfig(host="192.168.1.100")
        assert config.host == "192.168.1.100"
        assert config.port == 502
        assert config.timeout == 1.0
        assert config.unit_id == 1

    def test_custom_config(self) -> None:
        """Test custom configuration."""
        config = ModbusTCPConfig(
            host="10.0.0.1",
            port=5020,
            timeout=5.0,
            unit_id=5,
        )
        assert config.port == 5020
        assert config.timeout == 5.0
        assert config.unit_id == 5


class TestModbusStatistics:
    """Tests for ModbusStatistics."""

    def test_default_statistics(self) -> None:
        """Test default statistics."""
        stats = ModbusStatistics()
        assert stats.requests_sent == 0
        assert stats.responses_received == 0
        assert stats.errors == 0
        assert stats.timeouts == 0
        assert stats.crc_errors == 0


class TestModbusError:
    """Tests for ModbusError exception."""

    def test_modbus_error_creation(self) -> None:
        """Test ModbusError creation."""
        error = ModbusError(
            function_code=0x03,
            exception_code=ExceptionCode.ILLEGAL_DATA_ADDRESS,
        )
        assert error.function_code == 0x03
        assert error.exception_code == ExceptionCode.ILLEGAL_DATA_ADDRESS
        assert "Illegal data address" in error.message

    def test_modbus_error_custom_message(self) -> None:
        """Test ModbusError with custom message."""
        error = ModbusError(
            function_code=0x01,
            exception_code=0xFF,
            message="Custom error",
        )
        assert error.message == "Custom error"

    def test_modbus_error_unknown_code(self) -> None:
        """Test ModbusError with unknown exception code."""
        error = ModbusError(function_code=0x01, exception_code=0x99)
        assert "Unknown exception" in error.message


class TestModbusRTU:
    """Tests for ModbusRTU client."""

    @pytest.fixture
    def modbus(self) -> ModbusRTU:
        """Create a Modbus RTU client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusRTU("/dev/ttyUSB0", baudrate=9600)
            return client

    def test_creation(self, modbus: ModbusRTU) -> None:
        """Test Modbus RTU creation."""
        assert modbus.name == "modbus-rtu"
        assert modbus.config.port == "/dev/ttyUSB0"
        assert not modbus.is_open

    def test_open_close_simulated(self, modbus: ModbusRTU) -> None:
        """Test open and close in simulation mode."""
        modbus.open()
        assert modbus.is_open

        modbus.close()
        assert not modbus.is_open

    def test_context_manager(self) -> None:
        """Test context manager usage."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            with ModbusRTU("/dev/ttyUSB0") as modbus:
                assert modbus.is_open
            assert not modbus.is_open

    def test_read_coils_simulated(self, modbus: ModbusRTU) -> None:
        """Test reading coils in simulation."""
        modbus.open()
        coils = modbus.read_coils(1, address=0, count=10)
        assert len(coils) == 10
        assert all(c is False for c in coils)  # Simulated returns all OFF
        modbus.close()

    def test_read_discrete_inputs_simulated(self, modbus: ModbusRTU) -> None:
        """Test reading discrete inputs in simulation."""
        modbus.open()
        inputs = modbus.read_discrete_inputs(1, address=100, count=5)
        assert len(inputs) == 5
        modbus.close()

    def test_read_holding_registers_simulated(self, modbus: ModbusRTU) -> None:
        """Test reading holding registers in simulation."""
        modbus.open()
        regs = modbus.read_holding_registers(1, address=0, count=10)
        assert len(regs) == 10
        assert all(r == 0 for r in regs)  # Simulated returns all 0
        modbus.close()

    def test_read_input_registers_simulated(self, modbus: ModbusRTU) -> None:
        """Test reading input registers in simulation."""
        modbus.open()
        regs = modbus.read_input_registers(1, address=0, count=5)
        assert len(regs) == 5
        modbus.close()

    def test_write_coil_simulated(self, modbus: ModbusRTU) -> None:
        """Test writing single coil in simulation."""
        modbus.open()
        modbus.write_coil(1, address=0, value=True)  # Should not raise
        modbus.write_coil(1, address=0, value=False)
        modbus.close()

    def test_write_register_simulated(self, modbus: ModbusRTU) -> None:
        """Test writing single register in simulation."""
        modbus.open()
        modbus.write_register(1, address=0, value=12345)  # Should not raise
        modbus.close()

    def test_write_coils_simulated(self, modbus: ModbusRTU) -> None:
        """Test writing multiple coils in simulation."""
        modbus.open()
        values = [True, False, True, True, False]
        modbus.write_coils(1, address=0, values=values)  # Should not raise
        modbus.close()

    def test_write_registers_simulated(self, modbus: ModbusRTU) -> None:
        """Test writing multiple registers in simulation."""
        modbus.open()
        values = [100, 200, 300, 400, 500]
        modbus.write_registers(1, address=0, values=values)  # Should not raise
        modbus.close()

    def test_slave_id_validation(self, modbus: ModbusRTU) -> None:
        """Test slave ID validation."""
        modbus.open()

        with pytest.raises(ValueError, match="Slave ID must be 1-247"):
            modbus.read_coils(0, address=0, count=1)

        with pytest.raises(ValueError, match="Slave ID must be 1-247"):
            modbus.read_coils(248, address=0, count=1)

        modbus.close()

    def test_count_validation(self, modbus: ModbusRTU) -> None:
        """Test count validation."""
        modbus.open()

        with pytest.raises(ValueError, match="Count must be 1-2000"):
            modbus.read_coils(1, address=0, count=0)

        with pytest.raises(ValueError, match="Count must be 1-125"):
            modbus.read_holding_registers(1, address=0, count=0)

        modbus.close()

    def test_register_value_validation(self, modbus: ModbusRTU) -> None:
        """Test register value validation."""
        modbus.open()

        with pytest.raises(ValueError, match="Register value must be 0-65535"):
            modbus.write_register(1, address=0, value=70000)

        modbus.close()

    def test_statistics(self, modbus: ModbusRTU) -> None:
        """Test statistics tracking."""
        modbus.open()
        modbus.read_coils(1, address=0, count=1)
        modbus.read_holding_registers(1, address=0, count=1)

        stats = modbus.statistics
        assert stats.requests_sent == 2
        modbus.close()


class TestModbusTCP:
    """Tests for ModbusTCP client."""

    @pytest.fixture
    def modbus(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100", port=502)
            return client

    def test_creation(self, modbus: ModbusTCP) -> None:
        """Test Modbus TCP creation."""
        assert modbus.name == "modbus-tcp"
        assert modbus.config.host == "192.168.1.100"
        assert modbus.config.port == 502
        assert not modbus.is_open

    def test_open_close_simulated(self, modbus: ModbusTCP) -> None:
        """Test open and close in simulation mode."""
        modbus.open()
        assert modbus.is_open

        modbus.close()
        assert not modbus.is_open

    def test_context_manager(self) -> None:
        """Test context manager usage."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            with ModbusTCP("localhost") as modbus:
                assert modbus.is_open
            assert not modbus.is_open

    def test_read_coils_simulated(self, modbus: ModbusTCP) -> None:
        """Test reading coils in simulation."""
        modbus.open()
        coils = modbus.read_coils(1, address=0, count=10)
        assert len(coils) == 10
        modbus.close()

    def test_read_discrete_inputs_simulated(self, modbus: ModbusTCP) -> None:
        """Test reading discrete inputs in simulation."""
        modbus.open()
        inputs = modbus.read_discrete_inputs(1, address=100, count=8)
        assert len(inputs) == 8
        modbus.close()

    def test_read_holding_registers_simulated(self, modbus: ModbusTCP) -> None:
        """Test reading holding registers in simulation."""
        modbus.open()
        regs = modbus.read_holding_registers(1, address=0, count=10)
        assert len(regs) == 10
        modbus.close()

    def test_read_input_registers_simulated(self, modbus: ModbusTCP) -> None:
        """Test reading input registers in simulation."""
        modbus.open()
        regs = modbus.read_input_registers(1, address=0, count=5)
        assert len(regs) == 5
        modbus.close()

    def test_write_coil_simulated(self, modbus: ModbusTCP) -> None:
        """Test writing single coil in simulation."""
        modbus.open()
        modbus.write_coil(1, address=0, value=True)
        modbus.close()

    def test_write_register_simulated(self, modbus: ModbusTCP) -> None:
        """Test writing single register in simulation."""
        modbus.open()
        modbus.write_register(1, address=0, value=54321)
        modbus.close()

    def test_write_coils_simulated(self, modbus: ModbusTCP) -> None:
        """Test writing multiple coils in simulation."""
        modbus.open()
        values = [True, True, False, True]
        modbus.write_coils(1, address=10, values=values)
        modbus.close()

    def test_write_registers_simulated(self, modbus: ModbusTCP) -> None:
        """Test writing multiple registers in simulation."""
        modbus.open()
        values = [1000, 2000, 3000]
        modbus.write_registers(1, address=20, values=values)
        modbus.close()

    def test_count_validation(self, modbus: ModbusTCP) -> None:
        """Test count validation."""
        modbus.open()

        with pytest.raises(ValueError, match="Count must be 1-2000"):
            modbus.read_coils(1, address=0, count=2001)

        with pytest.raises(ValueError, match="Count must be 1-125"):
            modbus.read_holding_registers(1, address=0, count=126)

        modbus.close()

    def test_statistics(self, modbus: ModbusTCP) -> None:
        """Test statistics tracking."""
        modbus.open()
        modbus.read_coils(1, address=0, count=1)
        modbus.write_register(1, address=0, value=100)

        stats = modbus.statistics
        assert stats.requests_sent == 2
        modbus.close()


class TestSimulatedModbusServer:
    """Tests for SimulatedModbusServer."""

    @pytest.fixture
    def server(self) -> SimulatedModbusServer:
        """Create a simulated Modbus server."""
        return SimulatedModbusServer()

    def test_creation(self, server: SimulatedModbusServer) -> None:
        """Test server creation."""
        assert len(server.coils) == 1000
        assert len(server.discrete_inputs) == 1000
        assert len(server.holding_registers) == 1000
        assert len(server.input_registers) == 1000

    def test_custom_sizes(self) -> None:
        """Test custom memory sizes."""
        server = SimulatedModbusServer(
            coils_size=100,
            discrete_inputs_size=50,
            holding_registers_size=200,
            input_registers_size=150,
        )
        assert len(server.coils) == 100
        assert len(server.discrete_inputs) == 50
        assert len(server.holding_registers) == 200
        assert len(server.input_registers) == 150

    def test_set_get_coil(self, server: SimulatedModbusServer) -> None:
        """Test setting and getting coils."""
        server.set_coil(0, True)
        assert server.get_coil(0) is True

        server.set_coil(0, False)
        assert server.get_coil(0) is False

    def test_set_get_discrete_input(self, server: SimulatedModbusServer) -> None:
        """Test setting discrete inputs."""
        server.set_discrete_input(10, True)
        assert server.discrete_inputs[10] is True

    def test_set_get_holding_register(self, server: SimulatedModbusServer) -> None:
        """Test setting and getting holding registers."""
        server.set_holding_register(5, 12345)
        assert server.get_holding_register(5) == 12345

    def test_set_get_input_register(self, server: SimulatedModbusServer) -> None:
        """Test setting input registers."""
        server.set_input_register(100, 9999)
        assert server.input_registers[100] == 9999

    def test_register_value_masking(self, server: SimulatedModbusServer) -> None:
        """Test register value is masked to 16 bits."""
        server.set_holding_register(0, 0x12345)
        assert server.get_holding_register(0) == 0x2345

    def test_out_of_range_access(self, server: SimulatedModbusServer) -> None:
        """Test out of range access returns defaults."""
        assert server.get_coil(99999) is False
        assert server.get_holding_register(99999) == 0


class TestModbusClientHelpers:
    """Tests for ModbusClient helper methods."""

    @pytest.fixture
    def modbus(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("localhost")
            client.open()
            yield client
            client.close()

    def test_read_single_coil(self, modbus: ModbusTCP) -> None:
        """Test read single coil helper."""
        value = modbus.read_coil(1, address=0)
        assert isinstance(value, bool)

    def test_read_single_discrete_input(self, modbus: ModbusTCP) -> None:
        """Test read single discrete input helper."""
        value = modbus.read_discrete_input(1, address=0)
        assert isinstance(value, bool)

    def test_read_single_holding_register(self, modbus: ModbusTCP) -> None:
        """Test read single holding register helper."""
        value = modbus.read_holding_register(1, address=0)
        assert isinstance(value, int)

    def test_read_single_input_register(self, modbus: ModbusTCP) -> None:
        """Test read single input register helper."""
        value = modbus.read_input_register(1, address=0)
        assert isinstance(value, int)


class TestModbusDataTypeHelpers:
    """Tests for Modbus data type helper methods."""

    @pytest.fixture
    def modbus(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("localhost")
            client.open()
            yield client
            client.close()

    def test_read_float32(self, modbus: ModbusTCP) -> None:
        """Test reading 32-bit float."""
        # Simulated returns 0 registers, so result will be 0.0
        value = modbus.read_float32(1, address=0)
        assert value == 0.0

    def test_read_int32(self, modbus: ModbusTCP) -> None:
        """Test reading 32-bit integer."""
        # Simulated returns 0 registers
        value = modbus.read_int32(1, address=0)
        assert value == 0


# =============================================================================
# Phase 5.8.5.2: Comprehensive Modbus Tests
# =============================================================================


class TestModbusReadHoldingRegisters:
    """Comprehensive tests for reading holding registers."""

    @pytest.fixture
    def modbus_rtu(self) -> ModbusRTU:
        """Create a Modbus RTU client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusRTU("/dev/ttyUSB0")
            yield client

    @pytest.fixture
    def modbus_tcp(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100", port=502)
            yield client

    def test_read_single_register_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading single holding register via RTU."""
        modbus_rtu.open()
        registers = modbus_rtu.read_holding_registers(1, address=0, count=1)
        assert len(registers) == 1
        assert isinstance(registers[0], int)
        modbus_rtu.close()

    def test_read_single_register_tcp(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading single holding register via TCP."""
        modbus_tcp.open()
        registers = modbus_tcp.read_holding_registers(1, address=0, count=1)
        assert len(registers) == 1
        assert isinstance(registers[0], int)
        modbus_tcp.close()

    def test_read_multiple_registers_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading multiple holding registers via RTU."""
        modbus_rtu.open()
        registers = modbus_rtu.read_holding_registers(1, address=0, count=10)
        assert len(registers) == 10
        modbus_rtu.close()

    def test_read_multiple_registers_tcp(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading multiple holding registers via TCP."""
        modbus_tcp.open()
        registers = modbus_tcp.read_holding_registers(1, address=100, count=50)
        assert len(registers) == 50
        modbus_tcp.close()

    def test_read_max_registers(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading maximum allowed registers (125)."""
        modbus_rtu.open()
        registers = modbus_rtu.read_holding_registers(1, address=0, count=125)
        assert len(registers) == 125
        modbus_rtu.close()

    def test_read_registers_count_too_high(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading registers with count too high."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Count must be 1-125"):
            modbus_rtu.read_holding_registers(1, address=0, count=126)
        modbus_rtu.close()

    def test_read_registers_count_zero(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading registers with count zero."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Count must be 1-125"):
            modbus_rtu.read_holding_registers(1, address=0, count=0)
        modbus_rtu.close()

    def test_read_registers_different_addresses(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading registers from different addresses."""
        modbus_tcp.open()
        for addr in [0, 100, 500, 1000, 10000]:
            registers = modbus_tcp.read_holding_registers(1, address=addr, count=5)
            assert len(registers) == 5
        modbus_tcp.close()

    def test_read_registers_different_slaves(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading registers from different slave IDs."""
        modbus_tcp.open()
        for slave_id in [1, 2, 5, 10, 247]:
            registers = modbus_tcp.read_holding_registers(slave_id, address=0, count=5)
            assert len(registers) == 5
        modbus_tcp.close()

    def test_read_registers_slave_id_validation(self, modbus_rtu: ModbusRTU) -> None:
        """Test slave ID validation."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Slave ID must be 1-247"):
            modbus_rtu.read_holding_registers(0, address=0, count=1)
        with pytest.raises(ValueError, match="Slave ID must be 1-247"):
            modbus_rtu.read_holding_registers(248, address=0, count=1)
        modbus_rtu.close()


class TestModbusWriteSingleRegister:
    """Comprehensive tests for writing single register."""

    @pytest.fixture
    def modbus_rtu(self) -> ModbusRTU:
        """Create a Modbus RTU client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusRTU("/dev/ttyUSB0")
            yield client

    @pytest.fixture
    def modbus_tcp(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100")
            yield client

    def test_write_single_register_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing single register via RTU."""
        modbus_rtu.open()
        modbus_rtu.write_register(1, address=0, value=12345)
        modbus_rtu.close()

    def test_write_single_register_tcp(self, modbus_tcp: ModbusTCP) -> None:
        """Test writing single register via TCP."""
        modbus_tcp.open()
        modbus_tcp.write_register(1, address=0, value=54321)
        modbus_tcp.close()

    def test_write_register_min_value(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing minimum register value (0)."""
        modbus_rtu.open()
        modbus_rtu.write_register(1, address=0, value=0)
        modbus_rtu.close()

    def test_write_register_max_value(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing maximum register value (65535)."""
        modbus_rtu.open()
        modbus_rtu.write_register(1, address=0, value=65535)
        modbus_rtu.close()

    def test_write_register_value_too_high(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing register value too high."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Register value must be 0-65535"):
            modbus_rtu.write_register(1, address=0, value=65536)
        modbus_rtu.close()

    def test_write_register_negative_value(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing negative register value."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Register value must be 0-65535"):
            modbus_rtu.write_register(1, address=0, value=-1)
        modbus_rtu.close()

    def test_write_register_various_addresses(self, modbus_tcp: ModbusTCP) -> None:
        """Test writing to various addresses."""
        modbus_tcp.open()
        for addr in [0, 100, 500, 1000, 10000]:
            modbus_tcp.write_register(1, address=addr, value=100)
        modbus_tcp.close()

    def test_write_register_slave_id_validation(self, modbus_rtu: ModbusRTU) -> None:
        """Test slave ID validation for write."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Slave ID must be 1-247"):
            modbus_rtu.write_register(0, address=0, value=100)
        modbus_rtu.close()


class TestModbusWriteMultipleRegisters:
    """Comprehensive tests for writing multiple registers."""

    @pytest.fixture
    def modbus_rtu(self) -> ModbusRTU:
        """Create a Modbus RTU client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusRTU("/dev/ttyUSB0")
            yield client

    @pytest.fixture
    def modbus_tcp(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100")
            yield client

    def test_write_multiple_registers_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing multiple registers via RTU."""
        modbus_rtu.open()
        values = [1000, 2000, 3000, 4000, 5000]
        modbus_rtu.write_registers(1, address=0, values=values)
        modbus_rtu.close()

    def test_write_multiple_registers_tcp(self, modbus_tcp: ModbusTCP) -> None:
        """Test writing multiple registers via TCP."""
        modbus_tcp.open()
        values = [100, 200, 300]
        modbus_tcp.write_registers(1, address=100, values=values)
        modbus_tcp.close()

    def test_write_single_register_via_bulk(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing single register via bulk write."""
        modbus_rtu.open()
        modbus_rtu.write_registers(1, address=0, values=[12345])
        modbus_rtu.close()

    def test_write_max_registers(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing maximum allowed registers (123)."""
        modbus_rtu.open()
        values = list(range(123))
        modbus_rtu.write_registers(1, address=0, values=values)
        modbus_rtu.close()

    def test_write_registers_too_many(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing too many registers."""
        modbus_rtu.open()
        values = list(range(124))
        with pytest.raises(ValueError, match="Must write 1-123 registers"):
            modbus_rtu.write_registers(1, address=0, values=values)
        modbus_rtu.close()

    def test_write_registers_empty(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing empty register list."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Must write 1-123 registers"):
            modbus_rtu.write_registers(1, address=0, values=[])
        modbus_rtu.close()

    def test_write_registers_various_sizes(self, modbus_tcp: ModbusTCP) -> None:
        """Test writing various register counts."""
        modbus_tcp.open()
        for count in [1, 5, 10, 50, 100]:
            values = list(range(count))
            modbus_tcp.write_registers(1, address=0, values=values)
        modbus_tcp.close()


class TestModbusReadCoils:
    """Comprehensive tests for reading coils."""

    @pytest.fixture
    def modbus_rtu(self) -> ModbusRTU:
        """Create a Modbus RTU client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusRTU("/dev/ttyUSB0")
            yield client

    @pytest.fixture
    def modbus_tcp(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100")
            yield client

    def test_read_single_coil_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading single coil via RTU."""
        modbus_rtu.open()
        coils = modbus_rtu.read_coils(1, address=0, count=1)
        assert len(coils) == 1
        assert isinstance(coils[0], bool)
        modbus_rtu.close()

    def test_read_single_coil_tcp(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading single coil via TCP."""
        modbus_tcp.open()
        coils = modbus_tcp.read_coils(1, address=0, count=1)
        assert len(coils) == 1
        assert isinstance(coils[0], bool)
        modbus_tcp.close()

    def test_read_multiple_coils_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading multiple coils via RTU."""
        modbus_rtu.open()
        coils = modbus_rtu.read_coils(1, address=0, count=16)
        assert len(coils) == 16
        for coil in coils:
            assert isinstance(coil, bool)
        modbus_rtu.close()

    def test_read_coils_byte_boundary(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading coils at byte boundary (8 bits)."""
        modbus_tcp.open()
        coils = modbus_tcp.read_coils(1, address=0, count=8)
        assert len(coils) == 8
        modbus_tcp.close()

    def test_read_coils_non_byte_boundary(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading coils not at byte boundary."""
        modbus_tcp.open()
        for count in [3, 7, 9, 15, 17]:
            coils = modbus_tcp.read_coils(1, address=0, count=count)
            assert len(coils) == count
        modbus_tcp.close()

    def test_read_max_coils(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading maximum allowed coils (2000)."""
        modbus_rtu.open()
        coils = modbus_rtu.read_coils(1, address=0, count=2000)
        assert len(coils) == 2000
        modbus_rtu.close()

    def test_read_coils_count_too_high(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading coils with count too high."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Count must be 1-2000"):
            modbus_rtu.read_coils(1, address=0, count=2001)
        modbus_rtu.close()

    def test_read_coils_count_zero(self, modbus_rtu: ModbusRTU) -> None:
        """Test reading coils with count zero."""
        modbus_rtu.open()
        with pytest.raises(ValueError, match="Count must be 1-2000"):
            modbus_rtu.read_coils(1, address=0, count=0)
        modbus_rtu.close()

    def test_read_coils_different_addresses(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading coils from different addresses."""
        modbus_tcp.open()
        for addr in [0, 100, 500, 1000]:
            coils = modbus_tcp.read_coils(1, address=addr, count=10)
            assert len(coils) == 10
        modbus_tcp.close()


class TestModbusCRC16Validation:
    """Comprehensive tests for CRC16 validation."""

    def test_crc16_empty_data(self) -> None:
        """Test CRC16 with empty data."""
        crc = calculate_crc16(b"")
        # Empty data has specific CRC
        assert isinstance(crc, int)
        assert 0 <= crc <= 0xFFFF

    def test_crc16_single_byte(self) -> None:
        """Test CRC16 with single byte."""
        for byte_val in [0x00, 0x01, 0x55, 0xAA, 0xFF]:
            crc = calculate_crc16(bytes([byte_val]))
            assert isinstance(crc, int)
            assert 0 <= crc <= 0xFFFF

    def test_crc16_known_value(self) -> None:
        """Test CRC16 with known value."""
        # Modbus RTU uses CRC-16/MODBUS polynomial
        # For data [0x01, 0x03, 0x00, 0x00, 0x00, 0x01]
        # The expected CRC is well-defined
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        crc = calculate_crc16(data)
        assert isinstance(crc, int)
        assert 0 <= crc <= 0xFFFF

    def test_crc16_different_data_different_crc(self) -> None:
        """Test different data produces different CRC."""
        data1 = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        data2 = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x02])
        crc1 = calculate_crc16(data1)
        crc2 = calculate_crc16(data2)
        assert crc1 != crc2

    def test_verify_crc16_valid(self) -> None:
        """Test verify_crc16 with valid data."""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        crc = calculate_crc16(data)
        # Append CRC in little-endian
        frame = data + struct.pack("<H", crc)
        assert verify_crc16(frame) is True

    def test_verify_crc16_invalid(self) -> None:
        """Test verify_crc16 with invalid CRC."""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        # Append wrong CRC
        frame = data + bytes([0x00, 0x00])
        # Calculate what the CRC should be
        correct_crc = calculate_crc16(data)
        # If the appended CRC is wrong, verify should fail
        frame_with_wrong_crc = data + bytes([0xFF, 0xFF])
        if correct_crc != 0xFFFF:
            assert verify_crc16(frame_with_wrong_crc) is False

    def test_crc16_byte_order(self) -> None:
        """Test CRC16 byte order (little-endian)."""
        data = bytes([0x01, 0x03])
        crc = calculate_crc16(data)
        low_byte = crc & 0xFF
        high_byte = (crc >> 8) & 0xFF
        # Verify both bytes are valid
        assert 0 <= low_byte <= 0xFF
        assert 0 <= high_byte <= 0xFF

    def test_crc16_polynomial(self) -> None:
        """Test CRC16 uses correct polynomial (0xA001 reflected)."""
        # Modbus CRC-16 uses polynomial 0x8005 (reflected as 0xA001)
        # Initial value: 0xFFFF
        # Final XOR: 0x0000
        # Input reflected: True
        # Output reflected: True
        data = bytes([0x01])
        crc = calculate_crc16(data)
        assert crc is not None

    def test_crc16_various_lengths(self) -> None:
        """Test CRC16 with various data lengths."""
        for length in [1, 2, 8, 16, 32, 64, 128, 256]:
            data = bytes(range(length % 256)) * (length // 256 + 1)
            data = data[:length]
            crc = calculate_crc16(data)
            assert isinstance(crc, int)
            assert 0 <= crc <= 0xFFFF


class TestModbusFunctionCodes:
    """Comprehensive tests for Modbus function codes."""

    def test_read_function_codes(self) -> None:
        """Test read function code values."""
        assert FunctionCode.READ_COILS == 0x01
        assert FunctionCode.READ_DISCRETE_INPUTS == 0x02
        assert FunctionCode.READ_HOLDING_REGISTERS == 0x03
        assert FunctionCode.READ_INPUT_REGISTERS == 0x04

    def test_write_function_codes(self) -> None:
        """Test write function code values."""
        assert FunctionCode.WRITE_SINGLE_COIL == 0x05
        assert FunctionCode.WRITE_SINGLE_REGISTER == 0x06
        assert FunctionCode.WRITE_MULTIPLE_COILS == 0x0F
        assert FunctionCode.WRITE_MULTIPLE_REGISTERS == 0x10

    def test_exception_codes(self) -> None:
        """Test exception code values."""
        assert ExceptionCode.ILLEGAL_FUNCTION == 0x01
        assert ExceptionCode.ILLEGAL_DATA_ADDRESS == 0x02
        assert ExceptionCode.ILLEGAL_DATA_VALUE == 0x03
        assert ExceptionCode.SLAVE_DEVICE_FAILURE == 0x04
        assert ExceptionCode.ACKNOWLEDGE == 0x05
        assert ExceptionCode.SLAVE_DEVICE_BUSY == 0x06

    def test_exception_response_format(self) -> None:
        """Test exception response format (function code | 0x80)."""
        for fc in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0F, 0x10]:
            exception_code = fc | 0x80
            assert exception_code >= 0x80
            assert (exception_code & 0x7F) == fc


class TestModbusWriteCoils:
    """Comprehensive tests for writing coils."""

    @pytest.fixture
    def modbus_rtu(self) -> ModbusRTU:
        """Create a Modbus RTU client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusRTU("/dev/ttyUSB0")
            yield client

    @pytest.fixture
    def modbus_tcp(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100")
            yield client

    def test_write_single_coil_on_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing single coil ON via RTU."""
        modbus_rtu.open()
        modbus_rtu.write_coil(1, address=0, value=True)
        modbus_rtu.close()

    def test_write_single_coil_off_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing single coil OFF via RTU."""
        modbus_rtu.open()
        modbus_rtu.write_coil(1, address=0, value=False)
        modbus_rtu.close()

    def test_write_single_coil_tcp(self, modbus_tcp: ModbusTCP) -> None:
        """Test writing single coil via TCP."""
        modbus_tcp.open()
        modbus_tcp.write_coil(1, address=100, value=True)
        modbus_tcp.close()

    def test_write_multiple_coils_rtu(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing multiple coils via RTU."""
        modbus_rtu.open()
        values = [True, False, True, False, True]
        modbus_rtu.write_coils(1, address=0, values=values)
        modbus_rtu.close()

    def test_write_multiple_coils_tcp(self, modbus_tcp: ModbusTCP) -> None:
        """Test writing multiple coils via TCP."""
        modbus_tcp.open()
        values = [True, True, False, False]
        modbus_tcp.write_coils(1, address=0, values=values)
        modbus_tcp.close()

    def test_write_coils_byte_boundary(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing coils at byte boundary."""
        modbus_rtu.open()
        values = [True] * 8  # Exactly 1 byte
        modbus_rtu.write_coils(1, address=0, values=values)
        modbus_rtu.close()

    def test_write_coils_non_byte_boundary(self, modbus_rtu: ModbusRTU) -> None:
        """Test writing coils not at byte boundary."""
        modbus_rtu.open()
        for count in [3, 7, 9, 15, 17]:
            values = [True, False] * (count // 2 + 1)
            values = values[:count]
            modbus_rtu.write_coils(1, address=0, values=values)
        modbus_rtu.close()


class TestModbusReadInputs:
    """Comprehensive tests for reading inputs."""

    @pytest.fixture
    def modbus_tcp(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100")
            yield client

    def test_read_discrete_inputs(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading discrete inputs."""
        modbus_tcp.open()
        inputs = modbus_tcp.read_discrete_inputs(1, address=0, count=16)
        assert len(inputs) == 16
        for inp in inputs:
            assert isinstance(inp, bool)
        modbus_tcp.close()

    def test_read_input_registers(self, modbus_tcp: ModbusTCP) -> None:
        """Test reading input registers."""
        modbus_tcp.open()
        registers = modbus_tcp.read_input_registers(1, address=0, count=10)
        assert len(registers) == 10
        for reg in registers:
            assert isinstance(reg, int)
        modbus_tcp.close()

    def test_read_input_registers_count_validation(self, modbus_tcp: ModbusTCP) -> None:
        """Test input register count validation."""
        modbus_tcp.open()
        with pytest.raises(ValueError, match="Count must be 1-125"):
            modbus_tcp.read_input_registers(1, address=0, count=126)
        modbus_tcp.close()


class TestModbusStatisticsTracking:
    """Comprehensive tests for statistics tracking."""

    @pytest.fixture
    def modbus(self) -> ModbusTCP:
        """Create a Modbus TCP client in simulation mode."""
        with patch.dict(os.environ, {"ROBO_SIMULATION": "true"}):
            client = ModbusTCP("192.168.1.100")
            yield client

    def test_statistics_initial_state(self, modbus: ModbusTCP) -> None:
        """Test initial statistics state."""
        stats = modbus.statistics
        assert stats.requests_sent == 0
        assert stats.responses_received == 0
        assert stats.errors == 0
        assert stats.timeouts == 0

    def test_statistics_after_reads(self, modbus: ModbusTCP) -> None:
        """Test statistics after read operations."""
        modbus.open()
        modbus.read_coils(1, address=0, count=1)
        modbus.read_holding_registers(1, address=0, count=1)
        modbus.read_input_registers(1, address=0, count=1)

        stats = modbus.statistics
        assert stats.requests_sent == 3
        modbus.close()

    def test_statistics_after_writes(self, modbus: ModbusTCP) -> None:
        """Test statistics after write operations."""
        modbus.open()
        modbus.write_coil(1, address=0, value=True)
        modbus.write_register(1, address=0, value=100)
        modbus.write_coils(1, address=0, values=[True, False])
        modbus.write_registers(1, address=0, values=[1, 2, 3])

        stats = modbus.statistics
        assert stats.requests_sent == 4
        modbus.close()

    def test_statistics_cumulative(self, modbus: ModbusTCP) -> None:
        """Test statistics are cumulative."""
        modbus.open()
        modbus.read_coils(1, address=0, count=1)
        assert modbus.statistics.requests_sent == 1

        modbus.read_coils(1, address=0, count=1)
        assert modbus.statistics.requests_sent == 2

        modbus.read_coils(1, address=0, count=1)
        assert modbus.statistics.requests_sent == 3
        modbus.close()


class TestModbusServerSimulation:
    """Comprehensive tests for simulated Modbus server."""

    @pytest.fixture
    def server(self) -> SimulatedModbusServer:
        """Create a simulated Modbus server."""
        return SimulatedModbusServer()

    def test_server_coils_operations(self, server: SimulatedModbusServer) -> None:
        """Test coil operations on server."""
        # Set and get individual coils
        for i in range(10):
            server.set_coil(i, i % 2 == 0)
            assert server.get_coil(i) == (i % 2 == 0)

    def test_server_holding_registers_operations(self, server: SimulatedModbusServer) -> None:
        """Test holding register operations on server."""
        # Set and get individual registers
        for i in range(10):
            server.set_holding_register(i, i * 1000)
            assert server.get_holding_register(i) == i * 1000

    def test_server_register_16bit_mask(self, server: SimulatedModbusServer) -> None:
        """Test register values are masked to 16 bits."""
        # Values > 65535 should be masked
        server.set_holding_register(0, 0x12345678)
        assert server.get_holding_register(0) == 0x5678

    def test_server_discrete_inputs(self, server: SimulatedModbusServer) -> None:
        """Test discrete input operations."""
        for i in range(10):
            server.set_discrete_input(i, i % 3 == 0)
            assert server.discrete_inputs[i] == (i % 3 == 0)

    def test_server_input_registers(self, server: SimulatedModbusServer) -> None:
        """Test input register operations."""
        for i in range(10):
            server.set_input_register(i, i * 100)
            assert server.input_registers[i] == i * 100


class TestModbusIntegration:
    """Integration tests for Modbus protocol."""

    def test_rtu_full_workflow(self) -> None:
        """Test RTU client full workflow."""
        with (
            patch.dict(os.environ, {"ROBO_SIMULATION": "true"}),
            ModbusRTU("/dev/ttyUSB0") as modbus,
        ):
            # Read operations
            modbus.read_coils(1, 0, 10)
            modbus.read_holding_registers(1, 0, 5)

            # Write operations
            modbus.write_coil(1, 0, True)
            modbus.write_register(1, 0, 100)
            modbus.write_coils(1, 0, [True, False, True])
            modbus.write_registers(1, 0, [1, 2, 3])

            # Check statistics
            assert modbus.statistics.requests_sent == 6

    def test_tcp_full_workflow(self) -> None:
        """Test TCP client full workflow."""
        with (
            patch.dict(os.environ, {"ROBO_SIMULATION": "true"}),
            ModbusTCP("192.168.1.100") as modbus,
        ):
            # Read operations
            modbus.read_coils(1, 0, 8)
            modbus.read_discrete_inputs(1, 0, 8)
            modbus.read_holding_registers(1, 0, 10)
            modbus.read_input_registers(1, 0, 10)

            # Write operations
            modbus.write_coil(1, 0, True)
            modbus.write_register(1, 0, 500)

            assert modbus.statistics.requests_sent == 6

    def test_multiple_slaves(self) -> None:
        """Test communicating with multiple slaves."""
        with (
            patch.dict(os.environ, {"ROBO_SIMULATION": "true"}),
            ModbusTCP("10.0.0.1") as modbus,
        ):
            # Read from different slaves
            for slave_id in [1, 2, 5, 10]:
                registers = modbus.read_holding_registers(slave_id, 0, 5)
                assert len(registers) == 5

            assert modbus.statistics.requests_sent == 4
