# Changelog

All notable changes to this project will be documented in this file.

This file is auto-generated from conventional commits using [git-cliff](https://git-cliff.org/).

## [0.2.0] - 2025-12-26

### 🎉 Highlights

This release marks the transition from Alpha to **Beta** with major improvements in testing infrastructure, observability, resilience, and production readiness.

- **5,200+ tests** with 74%+ coverage
- **190 integration tests** for end-to-end validation
- **Prometheus metrics** and health check integration
- **Resilience utilities** with circuit breakers, retries, and timeouts
- **Battery monitoring** with chemistry-aware state tracking

### Features

- **Observability Integration**: Add Prometheus metrics and health checks for production monitoring
  - Prometheus counter, histogram, and gauge metrics
  - `/health`, `/ready`, `/live` endpoints for Kubernetes
  - Structured logging with correlation IDs
  - Span tracing for operation timing

- **Resilience Utilities**: Add robust error handling for production systems
  - Circuit breaker pattern for failing dependencies
  - Exponential backoff retry with jitter
  - Configurable timeouts for all operations
  - Graceful degradation patterns

- **Battery Monitoring**: Add comprehensive power management
  - Support for LiPo, Li-ion, NiMH, Lead-acid chemistries
  - State-of-charge estimation
  - Low battery warnings and critical alerts
  - Current draw and capacity tracking

- **Integration Testing Enhancement**: Add 95 new integration tests
  - Sensor fusion E2E tests (IMU → orientation)
  - Safety integration tests (E-stop, watchdog, limits)
  - API integration tests (REST API → controller)
  - AI integration tests (tool calls → actuator)
  - Mock external systems for testing

### Testing

- Add unit tests for resource management and security utilities
- Add unit tests for watchdog module to improve coverage
- Add unit tests for Turntable and ROS2 integration
- Add unit tests for Leg controller and related components
- Add unit tests for Quadcopter controller and related components
- Add unit tests for power distribution and drivers modules
- Add unit tests for Modbus, GPS, and LIDAR sensor implementations
- Add unit tests for VESC brushless motor controller driver
- Add unit tests for vision marker detection and processing utilities
- Add unit tests for SCARA and Stewart platform kinematics
- Add unit tests for IK solvers and motion transforms
- Add unit tests for ESP32 and Jetson platform implementations
- Add unit tests for platform abstraction layer and Raspberry Pi platform
- Add unit tests for PID controller and trajectory generation

### Documentation

- Add examples for Gripper, Lock, and Rover with comprehensive documentation
- Add integration guides for AI and API usage
- Enhanced test coverage documentation

### Other Changes

- Update package versions for ai-infra (0.1.166) and svc-infra (0.1.716)
- Add optional name parameter to StepDirConfig
- Fix CI race condition in docs-changelog workflow
- Register error handlers for FastAPI integration

---

## [0.1.12] - 2025-12-20

### Other Changes

- Minor version bump for CI release process improvements

---

## [0.1.11] - 2025-12-19


### Bug Fixes

- Streamline code formatting and improve readability in controllers and drivers
- Update __all__ exports in controllers and improve test comments
- Improve error handling when stopping motors in L298N and TB6612 drivers


### Other Changes

- Add unit tests for gripper, joint group, and lock controllers

- Implement comprehensive tests for the Gripper controller, covering initialization, open/close operations, state management, enable/disable functionality, configuration, and additional methods.
- Create unit tests for the JointGroup controller, including initialization, movement, state management, named positions, interpolation, and configuration.
- Develop tests for the Lock controller, focusing on initialization, lock/unlock operations, state management, async operations, enable/disable functionality, and configuration.
- Ensure all tests validate expected behaviors and raise appropriate exceptions when necessary.

## [0.1.10] - 2025-12-18


### Bug Fixes

- Update pre-commit hooks and improve code formatting in stepper and arduino drivers

## [0.1.9] - 2025-12-18


### Bug Fixes

- Update workflow trigger to run after CI completion

## [0.1.8] - 2025-12-18


### Bug Fixes

- Restrict Python version and improve type hinting in router integrations
- Update repository links and improve documentation clarity


### Other Changes

- Add documentation and improve type hints across the project

- Created API reference documentation in `docs/reference/api.md` for core modules including Actuators, Sensors, Controllers, Drivers, Safety, Core, and Motion.
- Added `mkdocs.yml` configuration for documentation generation using MkDocs and Material theme.
- Updated `mypy.ini` to ignore missing imports for various modules and adjusted type checking settings for specific driver modules.
- Enhanced type hints in `src/robo_infra/drivers/arduino.py`, `gpio.py`, `limits.py`, and `switches.py` for better clarity and type safety.
- Refactored timeout parameters in `LimitSwitch` methods to have a default value of 30 seconds.
- Cleaned up imports in `svc_infra.py` and removed unnecessary comments.
- Improved test cases in `test_drivers_arduino.py`, `test_drivers_gpio.py`, `test_drivers_l298n.py`, `test_drivers_pca9685.py`, and `test_drivers_tb6612.py` for better readability and maintainability.
- Removed unused imports and fixed formatting issues in various test files.
- Update dependency version constraints in pyproject.toml for better compatibility
- Add unit tests for TB6612FNG dual motor driver

- Implement comprehensive test suite for TB6612 driver including:
  - Enum and constant tests for direction, channel, and brake mode
  - Configuration and motor state validation
  - Initialization and lifecycle management tests
  - Standby mode functionality
  - Motor control methods and speed limits
  - Direction inversion handling
  - Stop methods and enable/disable functionality
  - Channel interface compatibility
  - Validation of input parameters
  - Driver registration and property tests
  - Edge case scenarios for motor operations


### Refactor

- Streamline code formatting and improve readability across multiple files
- Enhance Makefile help commands and improve formatting checks

## [0.1.7] - 2025-12-13


### Other Changes

- Add SimulationDriver implementation and unit tests

## [0.1.6] - 2025-12-13


### Other Changes

- Add unit tests for environmental and switch sensors

- Implemented comprehensive unit tests for environmental sensors including Temperature, Humidity, Pressure, and Light sensors.
- Added tests for configuration classes, reading methods, calibration, and status tracking.
- Created unit tests for switch sensors including Limit Switch, Button, and Hall Effect sensors.
- Included tests for configuration, state detection, trigger counting, and asynchronous wait methods.

## [0.1.5] - 2025-12-12


### Other Changes

- Add unit tests for distance and IMU sensors

- Implemented tests for Ultrasonic, ToF, and IRDistance sensors in `test_sensors_distance.py`.
- Added comprehensive tests for IMU components including Accelerometer, Gyroscope, and Magnetometer in `test_sensors_imu.py`.
- Ensured coverage for various functionalities such as reading values, calibration, and error handling.

## [0.1.4] - 2025-12-12


### Other Changes

- Add unit tests for various actuators in robo_infra

- Implement tests for Brushless actuators, including basic functionality, driver interactions, and factory creation.
- Add comprehensive tests for DC Motors, covering configuration, status, pin interactions, and error handling.
- Create tests for Linear Actuators, validating motor and solenoid-based operations, including movement and feedback mechanisms.
- Introduce extensive tests for Servo actuators, focusing on initialization, angle/pulse conversion, operations, and calibration.
- Develop tests for Solenoids, ensuring activation, deactivation, and driver channel updates are functioning correctly.
- Implement tests for Stepper motors, validating movement, error handling, and driver channel interactions.

## [0.1.3] - 2025-12-12


### Documentation

- Add PyPI badge to README

## [0.1.1] - 2025-12-12


### Bug Fixes

- Remove poetry.lock from .gitignore for CI


### Features

- Add Makefile for project setup and development commands
- Phase 1 - Project Foundation


### Other Changes

- Add unit tests for pin and sensor abstractions

- Implemented comprehensive unit tests for pin abstractions including SimulatedDigitalPin, SimulatedPWMPin, and SimulatedAnalogPin.
- Added tests for PinMode and PinState enums to ensure all modes and states are defined.
- Created unit tests for the sensor module, covering SensorState, SensorType, SensorConfig, and various sensor functionalities.
- Included tests for filtering mechanisms, sensor calibration, and sensor group management.
- Established factory functions for creating different types of sensors with associated tests.

<!-- Generated by git-cliff -->
