"""Actuator implementations.

This package provides concrete actuator implementations for common
robotics hardware like servos, DC motors, and stepper motors.

Available Actuators:
    - Servo: PWM-controlled servo motors
    - SimulatedServo: Servo simulation for testing

Example:
    >>> from robo_infra.actuators import Servo, SimulatedServo, create_servo
    >>>
    >>> # Create a simulated servo for testing
    >>> servo = SimulatedServo(name="shoulder", angle_range=(0, 180))
    >>> servo.enable()
    >>> servo.set(90)  # Move to 90 degrees
"""

from robo_infra.actuators.brushless import (
    Brushless,
    BrushlessConfig,
    BrushlessStatus,
    SimulatedBrushless,
    create_brushless,
)
from robo_infra.actuators.dc_motor import (
    DCMotor,
    DCMotorConfig,
    DCMotorStatus,
    SimulatedDCMotor,
    create_dc_motor,
)
from robo_infra.actuators.linear import (
    LinearActuator,
    LinearActuatorConfig,
    LinearActuatorStatus,
    SimulatedLinearActuator,
    create_linear_actuator,
)
from robo_infra.actuators.servo import (
    HIGH_FREQUENCY,
    STANDARD_FREQUENCY,
    STANDARD_PULSE_CENTER,
    STANDARD_PULSE_MAX,
    STANDARD_PULSE_MIN,
    Servo,
    ServoConfig,
    ServoRange,
    ServoStatus,
    ServoType,
    SimulatedServo,
    create_servo,
)
from robo_infra.actuators.solenoid import (
    Relay,
    SimulatedSolenoid,
    Solenoid,
    SolenoidConfig,
    SolenoidStatus,
    create_solenoid,
)
from robo_infra.actuators.stepper import (
    SimulatedStepper,
    Stepper,
    StepperConfig,
    StepperStatus,
    create_stepper,
)


__all__ = [
    "HIGH_FREQUENCY",
    "STANDARD_FREQUENCY",
    "STANDARD_PULSE_CENTER",
    "STANDARD_PULSE_MAX",
    "STANDARD_PULSE_MIN",
    "Brushless",
    "BrushlessConfig",
    "BrushlessStatus",
    "DCMotor",
    "DCMotorConfig",
    "DCMotorStatus",
    "LinearActuator",
    "LinearActuatorConfig",
    "LinearActuatorStatus",
    "Relay",
    "Servo",
    "ServoConfig",
    "ServoRange",
    "ServoStatus",
    "ServoType",
    "SimulatedBrushless",
    "SimulatedDCMotor",
    "SimulatedLinearActuator",
    "SimulatedServo",
    "SimulatedSolenoid",
    "SimulatedStepper",
    "Solenoid",
    "SolenoidConfig",
    "SolenoidStatus",
    "Stepper",
    "StepperConfig",
    "StepperStatus",
    "create_brushless",
    "create_dc_motor",
    "create_linear_actuator",
    "create_servo",
    "create_solenoid",
    "create_stepper",
]
