"""Hardware driver implementations.

This package provides driver implementations for common hardware:
- SimulationDriver: Enhanced simulation driver for testing
- PCA9685: 16-channel 12-bit PWM driver (I2C)
- L298N: Dual H-bridge motor driver
- TB6612: Dual motor driver (more efficient than L298N)
- GPIODriver: Direct GPIO control with software PWM
- ArduinoDriver: Serial communication with Arduino/microcontrollers
"""

from robo_infra.drivers.arduino import (
    ArduinoConfig,
    ArduinoDriver,
    ArduinoPinState,
    ArduinoProtocol,
    PinMode,
    SerialConfig,
    get_arduino_driver,
    list_arduino_ports,
)
from robo_infra.drivers.gpio import (
    GPIOConfig,
    GPIODirection,
    GPIODriver,
    GPIOEdge,
    GPIOPinConfig,
    GPIOPinState,
    GPIOPull,
    Platform,
    SoftwarePWMConfig,
    get_gpio_driver,
)
from robo_infra.drivers.l298n import (
    L298N,
    BrakeMode,
    L298NConfig,
    MotorChannel,
    MotorConfig,
    MotorDirection,
    MotorState,
)
from robo_infra.drivers.pca9685 import (
    PCA9685,
    PCA9685Config,
    PCA9685Mode1,
    PCA9685Mode2,
    PCA9685Register,
)
from robo_infra.drivers.simulation import (
    ChannelHistory,
    OperationRecord,
    OperationType,
    SimulationDriver,
)
from robo_infra.drivers.tb6612 import (
    TB6612,
    TB6612BrakeMode,
    TB6612Channel,
    TB6612Config,
    TB6612Direction,
    TB6612MotorConfig,
    TB6612MotorState,
)


__all__ = [
    "L298N",
    "PCA9685",
    "TB6612",
    "ArduinoConfig",
    "ArduinoDriver",
    "ArduinoPinState",
    "ArduinoProtocol",
    "BrakeMode",
    "ChannelHistory",
    "GPIOConfig",
    "GPIODirection",
    "GPIODriver",
    "GPIOEdge",
    "GPIOPinConfig",
    "GPIOPinState",
    "GPIOPull",
    "L298NConfig",
    "MotorChannel",
    "MotorConfig",
    "MotorDirection",
    "MotorState",
    "OperationRecord",
    "OperationType",
    "PCA9685Config",
    "PCA9685Mode1",
    "PCA9685Mode2",
    "PCA9685Register",
    "PinMode",
    "Platform",
    "SerialConfig",
    "SimulationDriver",
    "SoftwarePWMConfig",
    "TB6612BrakeMode",
    "TB6612Channel",
    "TB6612Config",
    "TB6612Direction",
    "TB6612MotorConfig",
    "TB6612MotorState",
    "get_arduino_driver",
    "get_gpio_driver",
    "list_arduino_ports",
]
