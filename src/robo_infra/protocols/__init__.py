"""Industrial communication protocols for robo-infra.

This package provides implementations of common industrial protocols
used in robotics and automation:

- CANopen: CAN-based protocol for industrial automation
- Modbus: Serial and TCP protocol for PLCs and sensors

All protocols support simulation mode for testing without hardware.
"""

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
from robo_infra.protocols.modbus import (
    ExceptionCode,
    FunctionCode,
    ModbusClient,
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


__all__ = [
    "COB_ID",
    "CANOpenMaster",
    # CANopen
    "CANOpenNode",
    "EMCYMessage",
    "ExceptionCode",
    "FunctionCode",
    # Modbus
    "ModbusClient",
    "ModbusError",
    "ModbusRTU",
    "ModbusRTUConfig",
    "ModbusStatistics",
    "ModbusTCP",
    "ModbusTCPConfig",
    "NMTCommand",
    "NMTState",
    "ObjectEntry",
    "PDOMapping",
    "SDOAbortCode",
    "SDOCommand",
    "SDOError",
    "SimulatedModbusServer",
    "calculate_crc16",
    "verify_crc16",
]
