"""Hardware driver implementations.

This package provides driver implementations for common hardware:
- SimulationDriver: Enhanced simulation driver for testing
- PCA9685: 16-channel 12-bit PWM driver (I2C)
- L298N: Dual H-bridge motor driver
- TB6612: Dual motor driver (more efficient than L298N)
- GPIODriver: Direct GPIO control with software PWM
- ArduinoDriver: Serial communication with Arduino/microcontrollers
- TMC2209: UART-based stepper motor driver
- A4988/DRV8825: Step/direction stepper drivers
- ODriveDriver: ODrive brushless motor controller
- VESCDriver: VESC brushless motor controller
- DynamixelDriver: Dynamixel smart servo controller
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
from robo_infra.drivers.bmi270 import (
    AccelBWP as BMI270AccelBWP,
)
from robo_infra.drivers.bmi270 import (
    AccelODR as BMI270AccelODR,
)
from robo_infra.drivers.bmi270 import (
    AccelRange as BMI270AccelRange,
)
from robo_infra.drivers.bmi270 import (
    BMI270Config,
    BMI270Driver,
    BMI270Reading,
    BMI270Register,
    BMI270Status,
)
from robo_infra.drivers.bmi270 import (
    GyroBWP as BMI270GyroBWP,
)
from robo_infra.drivers.bmi270 import (
    GyroODR as BMI270GyroODR,
)
from robo_infra.drivers.bmi270 import (
    GyroRange as BMI270GyroRange,
)
from robo_infra.drivers.bmi270 import (
    PowerMode as BMI270PowerMode,
)
from robo_infra.drivers.bno055 import (
    BNO055CalibrationData,
    BNO055CalibrationStatus,
    BNO055Config,
    BNO055Driver,
    BNO055OperationMode,
    BNO055PowerMode,
    BNO055Reading,
    BNO055Register,
    BNO055SystemStatus,
)
from robo_infra.drivers.dynamixel import (
    ControlTable,
    DynamixelConfig,
    DynamixelDriver,
    HardwareErrorStatus,
    Instruction,
    OperatingMode,
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
from robo_infra.drivers.icm20948 import (
    AccelRange as ICM20948AccelRange,
)
from robo_infra.drivers.icm20948 import (
    GyroRange as ICM20948GyroRange,
)
from robo_infra.drivers.icm20948 import (
    ICM20948Config,
    ICM20948Driver,
    ICM20948Reading,
    ICM20948Register,
)
from robo_infra.drivers.icm20948 import (
    MagMode as ICM20948MagMode,
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
from robo_infra.drivers.lsm6ds3 import (
    AccelODR as LSM6DS3AccelODR,
)
from robo_infra.drivers.lsm6ds3 import (
    AccelScale as LSM6DS3AccelScale,
)
from robo_infra.drivers.lsm6ds3 import (
    GyroODR as LSM6DS3GyroODR,
)
from robo_infra.drivers.lsm6ds3 import (
    GyroScale as LSM6DS3GyroScale,
)
from robo_infra.drivers.lsm6ds3 import (
    LSM6DS3Config,
    LSM6DS3Driver,
    LSM6DS3Reading,
    LSM6DS3Register,
)
from robo_infra.drivers.odrive import (
    AxisState,
    ControlMode,
    EncoderMode,
    InputMode,
    MotorType,
    ODriveConfig,
    ODriveDriver,
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
from robo_infra.drivers.step_dir import (
    A4988Driver,
    DRV8825Driver,
    StepDirConfig,
    StepDirDriver,
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
from robo_infra.drivers.tmc2209 import (
    DRVStatusBits,
    GCONFBits,
    TMC2209Config,
    TMC2209Driver,
    TMC2209Register,
)
from robo_infra.drivers.vesc import (
    VESCConfig,
    VESCControlMode,
    VESCDriver,
    VESCFaultCode,
    VESCPacketID,
    VESCState,
)


__all__ = [
    "L298N",
    "PCA9685",
    "TB6612",
    # Step/Dir drivers (A4988, DRV8825)
    "A4988Driver",
    "ArduinoConfig",
    "ArduinoDriver",
    "ArduinoPinState",
    "ArduinoProtocol",
    # ODrive brushless motor controller
    "AxisState",
    "BMI270AccelBWP",
    "BMI270AccelODR",
    "BMI270AccelRange",
    "BMI270Config",
    # BMI270 6-DOF IMU
    "BMI270Driver",
    "BMI270GyroBWP",
    "BMI270GyroODR",
    "BMI270GyroRange",
    "BMI270PowerMode",
    "BMI270Reading",
    "BMI270Register",
    "BMI270Status",
    "BNO055CalibrationData",
    "BNO055CalibrationStatus",
    "BNO055Config",
    # BNO055 9-DOF IMU
    "BNO055Driver",
    "BNO055OperationMode",
    "BNO055PowerMode",
    "BNO055Reading",
    "BNO055Register",
    "BNO055SystemStatus",
    "BrakeMode",
    "ChannelHistory",
    "ControlMode",
    # Dynamixel smart servo
    "ControlTable",
    "DRV8825Driver",
    # TMC2209 stepper driver
    "DRVStatusBits",
    "DynamixelConfig",
    "DynamixelDriver",
    "EncoderMode",
    "GCONFBits",
    "GPIOConfig",
    "GPIODirection",
    "GPIODriver",
    "GPIOEdge",
    "GPIOPinConfig",
    "GPIOPinState",
    "GPIOPull",
    "HardwareErrorStatus",
    "ICM20948AccelRange",
    "ICM20948Config",
    # ICM-20948 9-DOF IMU
    "ICM20948Driver",
    "ICM20948GyroRange",
    "ICM20948MagMode",
    "ICM20948Reading",
    "ICM20948Register",
    "InputMode",
    "Instruction",
    "L298NConfig",
    "LSM6DS3AccelODR",
    "LSM6DS3AccelScale",
    "LSM6DS3Config",
    # LSM6DS3 6-DOF IMU
    "LSM6DS3Driver",
    "LSM6DS3GyroODR",
    "LSM6DS3GyroScale",
    "LSM6DS3Reading",
    "LSM6DS3Register",
    "MotorChannel",
    "MotorConfig",
    "MotorDirection",
    "MotorState",
    "MotorType",
    "ODriveConfig",
    "ODriveDriver",
    "OperatingMode",
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
    "StepDirConfig",
    "StepDirDriver",
    "TB6612BrakeMode",
    "TB6612Channel",
    "TB6612Config",
    "TB6612Direction",
    "TB6612MotorConfig",
    "TB6612MotorState",
    "TMC2209Config",
    "TMC2209Driver",
    "TMC2209Register",
    # VESC brushless motor controller
    "VESCConfig",
    "VESCControlMode",
    "VESCDriver",
    "VESCFaultCode",
    "VESCPacketID",
    "VESCState",
    "get_arduino_driver",
    "get_gpio_driver",
    "list_arduino_ports",
]
