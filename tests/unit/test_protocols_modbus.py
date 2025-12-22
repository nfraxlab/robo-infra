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
