# API Reference

This documentation has moved to the interactive API reference on nfrax.com.

## Browse API Documentation

Visit the full API reference at **[nfrax.com/robo-infra/api](https://nfrax.com/robo-infra/api)** for:

- Complete class documentation with type signatures
- Method parameters and return types
- Code examples and usage patterns
- Dark mode support

## Quick Links

### Controllers

| Class | Description |
|-------|-------------|
| [DifferentialDrive](https://nfrax.com/robo-infra/api/DifferentialDrive) | Differential drive controller |
| [Gripper](https://nfrax.com/robo-infra/api/Gripper) | Gripper controller |
| [JointGroup](https://nfrax.com/robo-infra/api/JointGroup) | Joint group controller |
| [Lock](https://nfrax.com/robo-infra/api/Lock) | Hardware lock abstraction |

### CAN Bus

| Class | Description |
|-------|-------------|
| [CANBus](https://nfrax.com/robo-infra/api/CANBus) | CAN bus interface |
| [CANConfig](https://nfrax.com/robo-infra/api/CANConfig) | CAN configuration |
| [CANMessage](https://nfrax.com/robo-infra/api/CANMessage) | CAN message model |
| [SimulatedCANBus](https://nfrax.com/robo-infra/api/SimulatedCANBus) | Simulated CAN bus for testing |

### Core Types

| Class | Description |
|-------|-------------|
| [Limits](https://nfrax.com/robo-infra/api/Limits) | Joint/actuator limits |
| [Position](https://nfrax.com/robo-infra/api/Position) | Position representation |

### Protocols

| Class | Description |
|-------|-------------|
| [CANOpenMaster](https://nfrax.com/robo-infra/api/CANOpenMaster) | CANopen master node |
| [CANOpenNode](https://nfrax.com/robo-infra/api/CANOpenNode) | CANopen node interface |
| [ModbusRTU](https://nfrax.com/robo-infra/api/ModbusRTU) | Modbus RTU protocol |
| [ModbusTCP](https://nfrax.com/robo-infra/api/ModbusTCP) | Modbus TCP protocol |

### Exceptions

| Class | Description |
|-------|-------------|
| [RoboInfraError](https://nfrax.com/robo-infra/api/RoboInfraError) | Base exception class |
| [HardwareNotFoundError](https://nfrax.com/robo-infra/api/HardwareNotFoundError) | Hardware not found |
| [CommunicationError](https://nfrax.com/robo-infra/api/CommunicationError) | Communication failure |
| [SafetyError](https://nfrax.com/robo-infra/api/SafetyError) | Safety violation |
| [CalibrationError](https://nfrax.com/robo-infra/api/CalibrationError) | Calibration failure |
