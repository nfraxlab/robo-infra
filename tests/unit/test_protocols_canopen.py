"""Tests for CANopen protocol implementation."""

from __future__ import annotations

import struct
import time

import pytest

from robo_infra.core.can_bus import CANConfig, CANMessage, SimulatedCANBus
from robo_infra.protocols.canopen import (
    COB_ID,
    CANOpenMaster,
    CANOpenNode,
    EMCYMessage,
    NMTCommand,
    NMTState,
    ObjectEntry,
    PDOMapping,
    SDOAbortCode,
    SDOCommand,
    SDOError,
)


class TestNMTCommand:
    """Tests for NMT commands."""

    def test_nmt_command_values(self) -> None:
        """Test NMT command values."""
        assert NMTCommand.START_REMOTE_NODE == 0x01
        assert NMTCommand.STOP_REMOTE_NODE == 0x02
        assert NMTCommand.ENTER_PRE_OPERATIONAL == 0x80
        assert NMTCommand.RESET_NODE == 0x81
        assert NMTCommand.RESET_COMMUNICATION == 0x82


class TestNMTState:
    """Tests for NMT states."""

    def test_nmt_state_values(self) -> None:
        """Test NMT state values."""
        assert NMTState.INITIALIZING == 0
        assert NMTState.STOPPED == 4
        assert NMTState.OPERATIONAL == 5
        assert NMTState.PRE_OPERATIONAL == 127


class TestCOB_ID:
    """Tests for COB-ID function codes."""

    def test_cob_id_values(self) -> None:
        """Test COB-ID values."""
        assert COB_ID.NMT == 0x000
        assert COB_ID.SYNC == 0x080
        assert COB_ID.TPDO1 == 0x180
        assert COB_ID.RPDO1 == 0x200
        assert COB_ID.SDO_TX == 0x580
        assert COB_ID.SDO_RX == 0x600
        assert COB_ID.HEARTBEAT == 0x700

    def test_cob_id_with_node_id(self) -> None:
        """Test COB-ID calculation with node ID."""
        node_id = 5
        assert COB_ID.TPDO1 + node_id == 0x185
        assert COB_ID.SDO_TX + node_id == 0x585
        assert COB_ID.SDO_RX + node_id == 0x605
        assert COB_ID.HEARTBEAT + node_id == 0x705


class TestSDOAbortCode:
    """Tests for SDO abort codes."""

    def test_abort_code_values(self) -> None:
        """Test SDO abort code values."""
        assert SDOAbortCode.OBJECT_DOES_NOT_EXIST == 0x06020000
        assert SDOAbortCode.SUBINDEX_DOES_NOT_EXIST == 0x06090011
        assert SDOAbortCode.GENERAL_ERROR == 0x08000000


class TestSDOError:
    """Tests for SDOError exception."""

    def test_sdo_error_with_abort_code(self) -> None:
        """Test SDOError creation."""
        error = SDOError(abort_code=SDOAbortCode.OBJECT_DOES_NOT_EXIST)
        assert error.abort_code == SDOAbortCode.OBJECT_DOES_NOT_EXIST
        assert "Object does not exist" in str(error.message)

    def test_sdo_error_with_custom_message(self) -> None:
        """Test SDOError with custom message."""
        error = SDOError(abort_code=0x08000000, message="Custom error")
        assert error.message == "Custom error"

    def test_sdo_error_unknown_code(self) -> None:
        """Test SDOError with unknown abort code."""
        error = SDOError(abort_code=0x12345678)
        assert "Unknown error" in error.message


class TestObjectEntry:
    """Tests for ObjectEntry dataclass."""

    def test_object_entry_creation(self) -> None:
        """Test object entry creation."""
        entry = ObjectEntry(
            index=0x1000,
            subindex=0,
            data=b"\x92\x01\x00\x00",
            name="Device Type",
        )
        assert entry.index == 0x1000
        assert entry.subindex == 0
        assert entry.data == b"\x92\x01\x00\x00"
        assert entry.name == "Device Type"

    def test_object_entry_defaults(self) -> None:
        """Test object entry defaults."""
        entry = ObjectEntry(index=0x2000, subindex=1)
        assert entry.data == b""
        assert entry.name == ""
        assert entry.access == "rw"


class TestPDOMapping:
    """Tests for PDOMapping dataclass."""

    def test_pdo_mapping_creation(self) -> None:
        """Test PDO mapping creation."""
        mapping = PDOMapping(index=0x6040, subindex=0, bit_length=16)
        assert mapping.index == 0x6040
        assert mapping.subindex == 0
        assert mapping.bit_length == 16

    def test_pdo_mapping_to_bytes(self) -> None:
        """Test PDO mapping serialization."""
        mapping = PDOMapping(index=0x6041, subindex=0, bit_length=16)
        data = mapping.to_bytes()
        assert len(data) == 4
        # Format: index (16 bits) | subindex (8 bits) | bitlength (8 bits)
        value = struct.unpack("<I", data)[0]
        assert (value >> 16) & 0xFFFF == 0x6041
        assert (value >> 8) & 0xFF == 0
        assert value & 0xFF == 16

    def test_pdo_mapping_from_bytes(self) -> None:
        """Test PDO mapping deserialization."""
        data = struct.pack("<I", (0x6040 << 16) | (1 << 8) | 8)
        mapping = PDOMapping.from_bytes(data)
        assert mapping.index == 0x6040
        assert mapping.subindex == 1
        assert mapping.bit_length == 8

    def test_pdo_mapping_roundtrip(self) -> None:
        """Test PDO mapping roundtrip."""
        original = PDOMapping(index=0x6064, subindex=0, bit_length=32)
        restored = PDOMapping.from_bytes(original.to_bytes())
        assert restored.index == original.index
        assert restored.subindex == original.subindex
        assert restored.bit_length == original.bit_length


class TestEMCYMessage:
    """Tests for EMCYMessage dataclass."""

    def test_emcy_message_creation(self) -> None:
        """Test emergency message creation."""
        emcy = EMCYMessage(
            node_id=1,
            error_code=0x1000,
            error_register=0x01,
            manufacturer_data=b"\x00\x00\x00\x00\x00",
        )
        assert emcy.node_id == 1
        assert emcy.error_code == 0x1000
        assert emcy.error_register == 0x01
        assert len(emcy.manufacturer_data) == 5

    def test_emcy_message_timestamp(self) -> None:
        """Test emergency message timestamp."""
        before = time.time()
        emcy = EMCYMessage(
            node_id=1,
            error_code=0x0000,
            error_register=0x00,
            manufacturer_data=b"",
        )
        after = time.time()
        assert before <= emcy.timestamp <= after


class TestCANOpenNode:
    """Tests for CANOpenNode."""

    @pytest.fixture
    def can_bus(self) -> SimulatedCANBus:
        """Create a simulated CAN bus."""
        config = CANConfig(channel="vcan0")
        bus = SimulatedCANBus(config)
        bus.open()
        yield bus
        bus.close()

    @pytest.fixture
    def node(self, can_bus: SimulatedCANBus) -> CANOpenNode:
        """Create a CANopen node."""
        return CANOpenNode(can_bus, node_id=1, sdo_timeout=0.1)

    def test_node_creation(self, can_bus: SimulatedCANBus) -> None:
        """Test node creation."""
        node = CANOpenNode(can_bus, node_id=5, name="TestNode")
        assert node.node_id == 5
        assert node.name == "TestNode"
        assert node.state == NMTState.INITIALIZING

    def test_node_id_validation(self, can_bus: SimulatedCANBus) -> None:
        """Test node ID validation."""
        with pytest.raises(ValueError, match="Node ID must be 1-127"):
            CANOpenNode(can_bus, node_id=0)

        with pytest.raises(ValueError, match="Node ID must be 1-127"):
            CANOpenNode(can_bus, node_id=128)

    def test_node_default_name(self, can_bus: SimulatedCANBus) -> None:
        """Test node default name."""
        node = CANOpenNode(can_bus, node_id=10)
        assert node.name == "Node-10"

    def test_nmt_start(self, node: CANOpenNode, can_bus: SimulatedCANBus) -> None:
        """Test NMT start command."""
        node.nmt_start()
        assert node.state == NMTState.OPERATIONAL

    def test_nmt_stop(self, node: CANOpenNode, can_bus: SimulatedCANBus) -> None:
        """Test NMT stop command."""
        node.nmt_stop()
        assert node.state == NMTState.STOPPED

    def test_nmt_pre_operational(self, node: CANOpenNode, can_bus: SimulatedCANBus) -> None:
        """Test NMT pre-operational command."""
        node.nmt_pre_operational()
        assert node.state == NMTState.PRE_OPERATIONAL

    def test_nmt_reset(self, node: CANOpenNode, can_bus: SimulatedCANBus) -> None:
        """Test NMT reset command."""
        node.nmt_start()
        node.nmt_reset()
        assert node.state == NMTState.INITIALIZING

    def test_process_heartbeat(self, node: CANOpenNode) -> None:
        """Test heartbeat processing."""
        before = time.time()
        node.process_heartbeat(bytes([NMTState.OPERATIONAL]))
        after = time.time()

        assert node.state == NMTState.OPERATIONAL
        assert before <= node.last_heartbeat <= after

    def test_is_alive(self, node: CANOpenNode) -> None:
        """Test is_alive check."""
        # No heartbeat yet
        assert not node.is_alive(timeout=1.0)

        # Process heartbeat
        node.process_heartbeat(bytes([NMTState.OPERATIONAL]))
        assert node.is_alive(timeout=1.0)

    def test_pdo_number_validation(self, node: CANOpenNode) -> None:
        """Test PDO number validation."""
        with pytest.raises(ValueError, match="PDO number must be 1-4"):
            node.pdo_write(0, b"\x00")

        with pytest.raises(ValueError, match="PDO number must be 1-4"):
            node.pdo_write(5, b"\x00")


class TestCANOpenMaster:
    """Tests for CANOpenMaster."""

    @pytest.fixture
    def can_bus(self) -> SimulatedCANBus:
        """Create a simulated CAN bus."""
        config = CANConfig(channel="vcan0")
        bus = SimulatedCANBus(config)
        bus.open()
        yield bus
        bus.close()

    @pytest.fixture
    def master(self, can_bus: SimulatedCANBus) -> CANOpenMaster:
        """Create a CANopen master."""
        return CANOpenMaster(can_bus, sdo_timeout=0.1)

    def test_master_creation(self, can_bus: SimulatedCANBus) -> None:
        """Test master creation."""
        master = CANOpenMaster(can_bus, sdo_timeout=0.5)
        assert master.bus is can_bus
        assert len(master.nodes) == 0

    def test_get_node(self, master: CANOpenMaster) -> None:
        """Test getting/creating a node."""
        node = master.get_node(1, name="Servo1")
        assert node.node_id == 1
        assert node.name == "Servo1"

        # Same node on second call
        node2 = master.get_node(1)
        assert node2 is node

    def test_remove_node(self, master: CANOpenMaster) -> None:
        """Test removing a node."""
        master.get_node(1)
        master.get_node(2)
        assert len(master.nodes) == 2

        master.remove_node(1)
        assert len(master.nodes) == 1
        assert 1 not in master.nodes

    def test_nmt_broadcast_start(self, master: CANOpenMaster) -> None:
        """Test NMT broadcast start all."""
        master.nmt_start_all()
        # Just verify it doesn't raise

    def test_nmt_broadcast_stop(self, master: CANOpenMaster) -> None:
        """Test NMT broadcast stop all."""
        master.nmt_stop_all()
        # Just verify it doesn't raise

    def test_nmt_broadcast_reset(self, master: CANOpenMaster) -> None:
        """Test NMT broadcast reset all."""
        master.nmt_reset_all()
        # Just verify it doesn't raise

    def test_send_sync(self, master: CANOpenMaster) -> None:
        """Test SYNC message."""
        master.send_sync()
        master.send_sync(counter=42)
        # Just verify it doesn't raise

    def test_process_heartbeat_message(self, master: CANOpenMaster, can_bus: SimulatedCANBus) -> None:
        """Test processing heartbeat message."""
        node = master.get_node(5)

        # Create heartbeat message
        msg = CANMessage(
            arbitration_id=COB_ID.HEARTBEAT + 5,
            data=bytes([NMTState.OPERATIONAL]),
        )

        master.process_message(msg)
        assert node.state == NMTState.OPERATIONAL

    def test_process_emcy_message(self, master: CANOpenMaster) -> None:
        """Test processing emergency message."""
        received_emcy = []

        def emcy_callback(emcy: EMCYMessage) -> None:
            received_emcy.append(emcy)

        master.set_emcy_callback(emcy_callback)

        # Create emergency message
        # Error code (2) + Error register (1) + Manufacturer data (5) = 8 bytes
        emcy_data = struct.pack("<H", 0x1000) + bytes([0x01]) + bytes(5)
        msg = CANMessage(
            arbitration_id=COB_ID.EMCY + 3,
            data=emcy_data,
        )

        master.process_message(msg)

        assert len(received_emcy) == 1
        assert received_emcy[0].node_id == 3
        assert received_emcy[0].error_code == 0x1000


class TestCANOpenIntegration:
    """Integration tests for CANopen."""

    @pytest.fixture
    def can_bus(self) -> SimulatedCANBus:
        """Create a simulated CAN bus."""
        config = CANConfig(channel="vcan0")
        bus = SimulatedCANBus(config)
        bus.open()
        yield bus
        bus.close()

    def test_nmt_state_transitions(self, can_bus: SimulatedCANBus) -> None:
        """Test NMT state transitions."""
        master = CANOpenMaster(can_bus)
        node = master.get_node(1)

        assert node.state == NMTState.INITIALIZING

        node.nmt_start()
        assert node.state == NMTState.OPERATIONAL

        node.nmt_pre_operational()
        assert node.state == NMTState.PRE_OPERATIONAL

        node.nmt_stop()
        assert node.state == NMTState.STOPPED

        node.nmt_reset()
        assert node.state == NMTState.INITIALIZING

    def test_multiple_nodes(self, can_bus: SimulatedCANBus) -> None:
        """Test multiple nodes on network."""
        master = CANOpenMaster(can_bus)

        # Create multiple nodes
        nodes = [master.get_node(i) for i in range(1, 6)]
        assert len(master.nodes) == 5

        # Start all
        master.nmt_start_all()

        # Individual operations
        nodes[0].nmt_stop()
        nodes[2].nmt_pre_operational()

        # Check states (local tracking)
        assert nodes[0].state == NMTState.STOPPED
        assert nodes[2].state == NMTState.PRE_OPERATIONAL


class TestSDOCommands:
    """Tests for SDO command specifiers."""

    def test_sdo_command_values(self) -> None:
        """Test SDO command values."""
        assert SDOCommand.DOWNLOAD_INITIATE_REQUEST == 0x20
        assert SDOCommand.UPLOAD_INITIATE_REQUEST == 0x40
        assert SDOCommand.ABORT_TRANSFER == 0x80
