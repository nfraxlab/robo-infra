# Changelog

All notable changes to this project will be documented in this file.

This file is auto-generated from conventional commits using [git-cliff](https://git-cliff.org/).

## [Unreleased]


### Bug Fixes

- Detect x.y.0 releases and skip auto-bump to create GitHub Release
- Only release x.y.0 versions, no auto-bump

## [0.2.3] - 2025-12-27


### Documentation

- Add testing guidelines for safety systems


### Other Changes

- Simplify patch statements and improve readability in unit tests
- Clean up imports and simplify boolean checks in platform detection
- Remove deprecated functions and update bus access methods
- Add unit tests for platform detection and factory functions

- Implemented comprehensive unit tests for the `robo_infra.platforms.detection` module, covering platform detection functions, simulation mode detection, environment variable overrides, and platform info retrieval.
- Added tests for individual platform detectors including Raspberry Pi, Jetson, Beaglebone, Arduino, ESP32, and others.
- Created tests for factory functions in the `robo_infra.platforms.factory` module, including `get_gpio`, `get_i2c`, `get_spi`, and `get_uart`.
- Included tests for error handling and integration scenarios to ensure robustness of platform factory functions.
- Established fixtures for resetting platform state and simulating environments to enhance test reliability.
- Enhance unit tests for SafetyMonitor and Watchdog

- Updated test suite for SafetyMonitor to improve coverage from 28% to over 80%.
- Added new test classes for state transitions, status dataclasses, limit configurations, hysteresis behavior, and violation tracking.
- Extended tests for CollisionDetector to cover custom thresholds and multiple sensors.
- Introduced additional tests for WatchdogError, WatchdogConfig, and WatchdogStatus to ensure robustness.
- Enhanced tests for Watchdog initialization, heartbeat, timeout handling, and lifecycle management.
- Improved logging and callback behavior tests for Watchdog.

## [0.2.2] - 2025-12-27


### Other Changes

- Add comprehensive utilities and vision documentation

- Introduced a new utilities module covering resilience, graceful degradation, hardware utilities, resource management, and security.
- Documented features such as retry mechanisms, circuit breakers, degraded mode controllers, and input validation.
- Added vision module documentation detailing color detection, marker tracking, image processing, camera calibration, and coordinate transforms.
- Included examples for practical usage of utilities and vision functionalities.

## [0.2.1] - 2025-12-27


### Bug Fixes

- Update numpy dependency version in pyproject.toml
- Prevent docs-changelog race condition with publish workflow


### Documentation

- Update changelog [skip ci]
- Update changelog [skip ci]
- Update changelog [skip ci]


### Features

- Update test configurations and improve test coverage with simulation mode
- Enhance optional dependencies and installation instructions for ai-infra, svc-infra, and hardware support
- Add real hardware bus implementations for I2C, SPI, and Serial with tests
- Enhance observability integration with Prometheus metrics and health checks


### Miscellaneous

- Update package versions for ai-infra and svc-infra to 0.1.166 and 0.1.716 respectively


### Other Changes

- Refactor test code for improved readability and consistency

- Adjusted formatting in various test files to enhance clarity, including consistent parameter alignment and spacing.
- Simplified list comprehensions and function calls for better readability.
- Ensured consistent use of line breaks and indentation across test cases.
- Updated comments for clarity where necessary.
- Implement feature X to enhance user experience and optimize performance
- Enhance type checking and linting configurations

- Updated mypy.ini to disable additional error codes related to type checking for hardware-interfacing code.
- Modified ruff.toml to ignore more linting rules for tests and specific files, improving flexibility in test cases.
- Added missing imports and restructured __all__ exports in various modules for better clarity and compliance with linting rules.
- Cleaned up imports and ensured consistent formatting across test files, enhancing readability and maintainability.
- Adjusted assertions in unit tests for better clarity and accuracy, ensuring tests remain robust against future changes.
- Add integration tests for API, safety system, and sensor fusion

- Implemented comprehensive integration tests for the API, covering controller to FastAPI router conversion, endpoint functionality, state synchronization, and error handling.
- Developed integration tests for the safety system, including E-stop triggering, limit checking, watchdog monitoring, and safety monitor integration with controllers.
- Created integration tests for sensor fusion, validating IMU data collection, filter processing, orientation estimation, and integration with controllers for feedback control.
- Add comprehensive unit tests for resource management and security utilities

- Implemented unit tests for resource management utilities including AsyncContextManager, ConnectionPool, ManagedResource, and ResourceManager.
- Added tests for LimitedBuffer functionality.
- Created tests for security utilities covering input validation for joint angles, speeds, addresses, and names.
- Included privilege checking tests for GPIO, I2C, SPI, serial, and CAN access.
- Validated sanitization functions for names and serial commands.
- Ensured thorough coverage of validation errors and edge cases.
- Add resilience utilities and tests for robust robotics applications

- Implemented resilience utilities in `robo_infra.utils.resilience` including retry, circuit breaker, and timeout functionalities.
- Created `with_timeout` and `run_with_timeout` for managing operation timeouts.
- Added robotics-specific circuit breaker configurations for drivers and sensors.
- Developed comprehensive unit tests for resilience utilities covering retry logic, circuit breaker behavior, and timeout handling.
- Introduced tests for `DegradedModeController` and exception hierarchy to ensure graceful degradation and error management.
- Ensured all new functionalities are covered with appropriate unit tests in `tests/unit/test_utils_resilience.py`, `tests/unit/test_degraded.py`, and `tests/unit/test_exceptions.py`.
- Add unit tests for watchdog module to improve coverage

- Implement tests for WatchdogError exception, WatchdogState enum,
  WatchdogConfig model, and WatchdogStatus dataclass.
- Cover initialization, lifecycle, heartbeat operations, timeout handling,
  and recovery after timeout.
- Validate start, stop, pause, and resume functionalities.
- Ensure thread safety and handle edge cases effectively.
- Target coverage increased from 59% to over 85%.
- Add examples for Gripper, Lock, and Rover with comprehensive documentation

- Implemented Gripper example with basic control, configuration, and state tracking.
- Created Lock example demonstrating basic lock control and REST API integration.
- Developed Rover example showcasing differential drive control and obstacle avoidance with sensors.
- Added README files for each example detailing usage, requirements, and configurations.
- Included test cases to ensure all examples run without exceptions and have necessary files.
- Add integration guides for AI and API, enhance test coverage, and register error handlers

- Introduced `ai-integration.md` to document integration with ai-infra for LLM-controlled robotics.
- Added `api-integration.md` to outline integration with svc-infra for REST API robotics control.
- Enhanced unit tests in `test_integrations_svc_infra.py` to cover new router functionalities and error handling.
- Registered error handlers in FastAPI applications to ensure consistent error responses.
- Add unit tests for Turntable and ROS2 integration

- Implement comprehensive unit tests for the Turntable controller, covering configuration, initialization, lifecycle, rotation operations, index positions, status checks, and tool generation.
- Introduce unit tests for ROS2 integration, including mock message types, mock ROS2 node functionality, service callbacks, launch file generation, and error handling.
- Ensure tests validate expected behaviors and edge cases for both Turntable and ROS2 components.
- Add unit tests for Leg controller and related components

- Implement tests for LegState, LegPosition, JointType enums.
- Add tests for FootPosition, JointAngles, LegDimensions, and LegStatus data classes.
- Create tests for LegConfig model, including default and custom configurations.
- Develop inverse kinematics and forward kinematics tests to validate calculations.
- Introduce tests for Leg controller methods, including enable/disable, homing, and joint angle settings.
- Validate leg creation with custom actuators and position attributes.
- Ensure comprehensive coverage of Leg.move_to() functionality.
- Attitude, Velocity, Position3D, MotorOutputs
  - QuadcopterConfig validation and properties
  - Motor mixing functions and normalization
  - Quadcopter initialization, enabling/disabling, arming, and flight commands
  - Emergency stop functionality and status reporting
  - Integration with AI tools and factory function for quadcopter creation
- Add unit tests for power distribution and drivers modules

- Implemented comprehensive unit tests for the power distribution module, covering PowerRailState, ShutdownPriority, PowerRailConfig, PowerRailReading, PowerRail, PowerDistributionConfig, and PowerDistributionBoard classes.
- Added unit tests for the power drivers module, including tests for PowerReading, INA219 and INA226 configurations, and their respective driver classes.
- Included integration tests to validate the interaction between components in both modules.
- Ensured all tests validate expected behavior, including edge cases and error handling.
- Add battery monitoring module with support for various chemistries and state tracking
- Add unit tests for Modbus, GPS, and LIDAR sensor implementations

- Implement comprehensive unit tests for Modbus protocol including function codes, exception codes, CRC calculations, and ModbusRTU/TCP client functionalities.
- Create unit tests for GPS sensor functionalities covering GPSReading, NMEA parsing, simulated GPS driver, and related configurations.
- Develop unit tests for LIDAR sensor implementations focusing on LIDARScan dataclass, simulated LIDAR driver, and configuration settings.
- Add optional name parameter to StepDirConfig and update unit tests for configuration
- Add unit tests for VESC brushless motor controller driver
- Add unit tests for vision marker detection and processing utilities

- Implemented comprehensive tests for fiducial marker detection in `test_vision_markers.py`, covering MarkerPose, ArUcoMarker, AprilTag, and their respective detectors.
- Added tests for various image processing functions in `test_vision_processing.py`, including resizing, cropping, rotating, thresholding, blurring, edge detection, morphological operations, and histogram equalization.
- Created utility functions for generating test frames with different patterns and formats to facilitate testing.
- Ensured compatibility with grayscale and color frames across all tests.
- Add unit tests for SCARA and Stewart platform kinematics

- Implement comprehensive unit tests for SCARA arm kinematics, covering:
  - Creation and validation of SCARAArm, SCARAConfiguration, SCARAJoints, and SCARALimits
  - Forward and inverse kinematics, including joint limits and workspace detection
  - Jacobian computation and factory functions

- Introduce unit tests for Stewart platform kinematics, including:
  - Creation and validation of StewartPlatform, StewartJoints, and StewartLimits
  - Forward and inverse kinematics, workspace validation, and singularity detection
  - Jacobian computation and factory functions
- Add unit tests for IK solvers and motion transforms

- Implemented comprehensive unit tests for various inverse kinematics solvers including JacobianIKSolver, DampedLeastSquaresIK, CCDIKSolver, and GradientDescentIK.
- Added tests for IKSolverConfig and IKResult dataclasses to ensure proper configuration and result handling.
- Developed tests for motion transforms, covering rotation and transformation functionalities such as Euler angles, quaternions, and axis-angle conversions.
- Included edge case tests to validate behavior under unusual conditions, ensuring robustness of the implementations.
- Add unit tests for ESP32 and Jetson platform implementations

- Implement comprehensive unit tests for the ESP32 platform, covering chip enums, capabilities, digital, PWM, analog, DAC, touch, and hall sensor pins.
- Include integration-style tests for ESP32 platform functionalities such as LED control, PWM motor control, DAC audio output, and touch button reading.
- Create unit tests for the Jetson platform, including tests for pin numbering enums, digital and PWM pin functionalities, platform-specific features, and model detection.
- Ensure simulation mode is properly handled in both platform tests.
- Add unit tests for platform abstraction layer and Raspberry Pi platform

- Implement comprehensive unit tests for the platform abstraction layer, covering:
  - Platform protocol and base class
  - PlatformRegistry with auto-detection
  - SimulationPlatform functionality
  - Detection functions with mocking for various platforms

- Introduce unit tests for the Raspberry Pi platform, including:
  - GPIO backend detection
  - RaspberryPiDigitalPin and RaspberryPiPWMPin functionality
  - Model detection and bus creation
  - Edge cases and error handling for pin and bus operations
- Add unit tests for PID controller and trajectory generation

- Implement comprehensive unit tests for the PID controller in `test_motion_pid.py`, covering initialization, term calculations, output limiting, anti-windup, derivative filtering, control methods, step response, and configuration validation.
- Create unit tests for trajectory generation classes in `test_motion_trajectory.py`, including tests for `TrajectoryPoint`, `MultiAxisTrajectoryPoint`, `LinearInterpolator`, `TrapezoidalProfile`, and `Trajectory`.
- Ensure all tests validate expected behaviors and edge cases, enhancing code reliability and maintainability.


### Styling

- Fix import sorting in integration tests


### Testing

- Fix CI failures for optional dependencies
- Skip integration tests when optional deps not installed

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
