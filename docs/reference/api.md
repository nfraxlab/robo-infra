# API Reference

This section contains auto-generated API documentation from the robo-infra source code.

## Core Modules

### Core

::: robo_infra.core
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - Actuator
        - Sensor
        - Controller
        - Driver
        - exceptions

### Actuators

::: robo_infra.actuators
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - Servo
        - DCMotor
        - Stepper
        - BrushlessMotor
        - LinearActuator
        - Gripper

### Sensors

::: robo_infra.sensors
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - IMU
        - Encoder
        - DistanceSensor
        - ForceSensor
        - TemperatureSensor
        - CurrentSensor

### Controllers

::: robo_infra.controllers
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - JointGroup
        - DifferentialDrive
        - MecanumDrive
        - OmniDrive
        - ArmController
        - ConveyorController

### Drivers

::: robo_infra.drivers
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - PCA9685Driver
        - DynamixelDriver
        - ODriveDriver
        - VESCDriver
        - TMC2209Driver
        - L298NDriver
        - TB6612Driver
        - ArduinoDriver
        - SimulationDriver

## Motion & Kinematics

### Motion

::: robo_infra.motion
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - TrajectoryGenerator
        - LinearInterpolation
        - CubicInterpolation
        - TrapezoidalProfile
        - SCurveProfile

### Vision

::: robo_infra.vision
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - Camera
        - ObjectDetector
        - MarkerDetector
        - CoordinateTransform

## Safety & Control

### Safety

::: robo_infra.safety
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - EStop
        - SafetyMonitor
        - JointLimiter
        - Watchdog
        - SafetyZone

## Protocols

### Protocols

::: robo_infra.protocols
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - CANopenMaster
        - CANopenNode
        - ModbusRTU
        - ModbusTCP

## Power

### Power

::: robo_infra.power
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - BatteryMonitor
        - PowerRail
        - PowerDistributionBoard
        - INA219Driver
        - INA226Driver

## Platforms

### Platforms

::: robo_infra.platforms
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - RaspberryPi
        - Jetson
        - BeagleBone
        - Arduino
        - ESP32

## Integrations

### Integrations

::: robo_infra.integrations
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - controller_to_ros2_node
        - ROS2NodeConfig
        - track_command
        - record_position
        - create_controller_health_check

## Utilities

### Utils

::: robo_infra.utils
    options:
      show_root_heading: true
      heading_level: 3
      members:
        - with_retry
        - with_timeout
        - CircuitBreaker
        - DegradedModeController
        - ConnectionPool
        - ResourceManager
        - validate_joint_angle
        - sanitize_name
        - check_gpio_access
