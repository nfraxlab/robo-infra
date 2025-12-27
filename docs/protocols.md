# Protocols

Industrial protocols enable communication with PLCs, motor controllers, and other automation equipment. This guide covers CANopen and Modbus.

## Overview

The protocols module provides:

- **CANopen** - CAN-based protocol for industrial automation and robotics
- **Modbus** - Serial/TCP protocol for PLCs and industrial devices

```python
from robo_infra.protocols import (
    CANOpenMaster, CANOpenNode,
    ModbusRTU, ModbusTCP,
)
```

---

## CANopen

CANopen is a high-level communication protocol built on CAN (Controller Area Network). It's widely used in industrial automation, robotics, and motion control.

### Key Concepts

| Term | Description |
|------|-------------|
| **Node** | A device on the CAN network (1-127) |
| **Object Dictionary** | Database of device parameters and data |
| **NMT** | Network Management (start, stop, reset) |
| **SDO** | Service Data Object (read/write parameters) |
| **PDO** | Process Data Object (real-time data) |
| **SYNC** | Synchronization trigger |
| **EMCY** | Emergency messages |
| **Heartbeat** | Node alive monitoring |

### Object Dictionary

The Object Dictionary is the core of CANopen—a standardized database structure:

```
0x0000-0x0FFF: Data types
0x1000-0x1FFF: Communication profile (standard)
0x2000-0x5FFF: Manufacturer-specific
0x6000-0x9FFF: Device profile (CiA 402 for drives, etc.)
0xA000-0xFFFF: Reserved
```

Common objects:
- `0x1000` - Device type
- `0x1008` - Manufacturer device name
- `0x1018` - Identity object (vendor, product, revision)
- `0x6040` - Control word (CiA 402)
- `0x6041` - Status word (CiA 402)
- `0x607A` - Target position (CiA 402)

### CANopen Master

```python
from robo_infra.core.can_bus import get_can
from robo_infra.protocols.canopen import CANOpenMaster

# Create CAN bus
can = get_can("socketcan", "can0", bitrate=500000)
can.open()

# Create CANopen master
master = CANOpenMaster(can)

# Scan for nodes
nodes = master.scan_nodes()
print(f"Found nodes: {nodes}")

# Get specific node
node = master.get_node(1)

# Clean up
can.close()
```

### PDO/SDO Communication

**SDO (Service Data Object)** - For configuration and parameter access:

```python
from robo_infra.protocols.canopen import CANOpenNode

# Get node
node = master.get_node(1)

# Read object dictionary entry
# SDO read: index, subindex
vendor_id = node.sdo_read(0x1018, 1)  # Identity object, vendor ID
product_code = node.sdo_read(0x1018, 2)
revision = node.sdo_read(0x1018, 3)

print(f"Vendor: 0x{vendor_id:08X}")
print(f"Product: 0x{product_code:08X}")

# Write parameter
# Control word: enable operation
node.sdo_write(0x6040, 0, struct.pack('<H', 0x0006))

# Read with specific data type
position = node.sdo_read_int32(0x6064, 0)  # Position actual value
velocity = node.sdo_read_int32(0x606C, 0)  # Velocity actual value

# Write with specific data type
node.sdo_write_int32(0x607A, 0, 10000)  # Target position
node.sdo_write_int32(0x6081, 0, 5000)   # Profile velocity
```

**PDO (Process Data Object)** - For real-time data exchange:

```python
# Configure RPDO (Receive PDO - commands to device)
node.configure_rpdo(
    pdo_number=1,
    cob_id=0x200 + node.node_id,
    mapping=[
        (0x6040, 0, 16),  # Control word, 16 bits
        (0x607A, 0, 32),  # Target position, 32 bits
    ],
)

# Configure TPDO (Transmit PDO - data from device)
node.configure_tpdo(
    pdo_number=1,
    cob_id=0x180 + node.node_id,
    mapping=[
        (0x6041, 0, 16),  # Status word, 16 bits
        (0x6064, 0, 32),  # Position actual, 32 bits
    ],
)

# Send PDO data
node.send_rpdo(1, struct.pack('<HI', 0x000F, 50000))

# Receive PDO data
data = node.receive_tpdo(1, timeout=0.1)
status, position = struct.unpack('<HI', data)
```

### Node Management

```python
# NMT state machine
node.nmt_start()              # Enter operational state
node.nmt_stop()               # Enter stopped state
node.nmt_pre_operational()    # Enter pre-operational state
node.nmt_reset()              # Reset node
node.nmt_reset_communication()  # Reset communication

# Check node state
state = node.get_state()
print(f"Node state: {state}")  # NMTState.OPERATIONAL

# Heartbeat monitoring
node.set_heartbeat_period(100)  # 100ms heartbeat
node.enable_heartbeat_monitoring()

def on_heartbeat_lost(node_id):
    print(f"WARNING: Node {node_id} heartbeat lost!")
    
master.on_heartbeat_lost = on_heartbeat_lost
```

### Motor Control via CANopen (CiA 402)

```python
from robo_infra.protocols.canopen import CANOpenMaster, CiA402Node

# Create CiA 402 motor node
motor = CiA402Node(master, node_id=1)

# Initialize drive
motor.reset_fault()
motor.enable()

# Wait for ready
motor.wait_for_state(CiA402State.OPERATION_ENABLED, timeout=5.0)

# Profile position mode
motor.set_mode(CiA402Mode.PROFILE_POSITION)

# Set motion parameters
motor.set_profile_velocity(10000)      # counts/s
motor.set_profile_acceleration(50000)  # counts/s²
motor.set_profile_deceleration(50000)

# Move to position
motor.move_to(100000)  # Move to 100000 counts
motor.wait_for_motion_complete(timeout=10.0)

print(f"Position: {motor.actual_position}")

# Homing
motor.set_mode(CiA402Mode.HOMING)
motor.start_homing()
motor.wait_for_homing_complete(timeout=30.0)

# Disable when done
motor.disable()
```

---

## Modbus

Modbus is a simple, robust protocol widely used for PLC communication. It supports serial (RTU) and Ethernet (TCP) transport.

### Data Types

| Type | Description | Function Codes |
|------|-------------|----------------|
| **Coils** | Single-bit read/write | 0x01, 0x05, 0x0F |
| **Discrete Inputs** | Single-bit read-only | 0x02 |
| **Holding Registers** | 16-bit read/write | 0x03, 0x06, 0x10 |
| **Input Registers** | 16-bit read-only | 0x04 |

### Modbus RTU (Serial)

```python
from robo_infra.protocols.modbus import ModbusRTU

# Connect to serial device
modbus = ModbusRTU(
    port="/dev/ttyUSB0",
    baudrate=9600,
    parity="N",     # N=None, E=Even, O=Odd
    stopbits=1,
    timeout=1.0,
)

modbus.open()

# Read holding registers from slave 1
# Slave ID, start address, count
registers = modbus.read_holding_registers(1, address=0, count=10)
print(f"Registers: {registers}")

# Read input registers
inputs = modbus.read_input_registers(1, address=100, count=5)

# Write single register
modbus.write_register(1, address=0, value=1234)

# Write multiple registers
modbus.write_registers(1, address=0, values=[100, 200, 300])

modbus.close()
```

### Modbus TCP (Ethernet)

```python
from robo_infra.protocols.modbus import ModbusTCP

# Connect to TCP device
modbus = ModbusTCP(
    host="192.168.1.100",
    port=502,
    timeout=1.0,
)

modbus.open()

# Same interface as RTU
registers = modbus.read_holding_registers(1, address=0, count=10)
modbus.write_register(1, address=0, value=5678)

modbus.close()
```

### Coils and Discrete Inputs

```python
# Read coils (digital outputs)
coils = modbus.read_coils(1, address=0, count=8)
print(f"Coils: {coils}")  # [True, False, True, ...]

# Read discrete inputs (digital inputs)
inputs = modbus.read_discrete_inputs(1, address=0, count=8)

# Write single coil
modbus.write_coil(1, address=0, value=True)

# Write multiple coils
modbus.write_coils(1, address=0, values=[True, False, True, True])
```

### Register Mapping

Create named mappings for readable code:

```python
from robo_infra.protocols.modbus import ModbusRegisterMap, Register

# Define register map
class MyPLCRegisters(ModbusRegisterMap):
    # Holding registers
    setpoint = Register(address=0, type="int16")
    output = Register(address=1, type="int16")
    mode = Register(address=2, type="uint16")
    
    # 32-bit values (2 registers)
    position = Register(address=10, type="int32")
    velocity = Register(address=12, type="float32")
    
    # Coils
    enable = Register(address=0, type="coil")
    alarm = Register(address=1, type="coil")

# Use the map
registers = MyPLCRegisters(modbus, slave_id=1)

# Read values
position = registers.position.read()
velocity = registers.velocity.read()

# Write values
registers.setpoint.write(1000)
registers.enable.write(True)

# Batch read (more efficient)
registers.batch_read(["position", "velocity", "setpoint"])
print(f"Position: {registers.position.value}")
```

### PLC Integration Example

```python
from robo_infra.protocols.modbus import ModbusTCP
import time

# Connect to PLC
plc = ModbusTCP("192.168.1.10", port=502)
plc.open()

try:
    # Read PLC status
    status = plc.read_holding_registers(1, address=0, count=1)[0]
    print(f"PLC Status: {status}")
    
    # Check if ready
    if status == 1:  # Ready
        # Send start command
        plc.write_coil(1, address=0, value=True)
        
        # Monitor progress
        while True:
            progress = plc.read_holding_registers(1, address=10, count=1)[0]
            print(f"Progress: {progress}%")
            
            if progress >= 100:
                break
            time.sleep(0.5)
        
        # Read results
        results = plc.read_holding_registers(1, address=100, count=10)
        print(f"Results: {results}")
        
finally:
    plc.close()
```

---

## Error Handling

### CANopen Errors

```python
from robo_infra.protocols.canopen import SDOAbortError, NMTError

try:
    value = node.sdo_read(0x6000, 0)
except SDOAbortError as e:
    print(f"SDO abort: {e.abort_code:#010x}")
    if e.abort_code == 0x06020000:
        print("Object does not exist")
    elif e.abort_code == 0x06090011:
        print("Subindex does not exist")
except TimeoutError:
    print("No response from node")
```

### Modbus Errors

```python
from robo_infra.protocols.modbus import ModbusError, ExceptionCode

try:
    registers = modbus.read_holding_registers(1, address=0, count=10)
except ModbusError as e:
    print(f"Modbus error: {e.message}")
    if e.exception_code == ExceptionCode.ILLEGAL_DATA_ADDRESS:
        print("Invalid register address")
except TimeoutError:
    print("No response from device")
```

---

## Next Steps

- [Power](power.md) - Power management for robots
- [Drivers](drivers.md) - Motor drivers using protocols
- [Controllers](controllers.md) - Controllers with protocol support
- [Safety](safety.md) - Safety systems for industrial robots
