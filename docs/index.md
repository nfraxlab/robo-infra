# robo-infra

**Robotics SDK for prototyping and development**

> **Beta Software**: APIs may change. Real hardware validation is ongoing.

robo-infra is a hardware-agnostic robotics SDK designed for prototyping, development, and educational projects. It provides a simulation-first development experience with drivers for common robotics hardware.

## Features

<div class="grid cards" markdown>

-   :material-robot-industrial:{ .lg .middle } **Actuators**

    ---

    Servo, DC Motor, Stepper, Brushless, Linear, Solenoid, Valve.

    [:octicons-arrow-right-24: Actuators docs](actuators.md)

-   :material-eye:{ .lg .middle } **Sensors**

    ---

    Distance, IMU, Encoder, Force, Temperature, Camera, GPS.

    [:octicons-arrow-right-24: Sensors docs](sensors.md)

-   :material-chip:{ .lg .middle } **Drivers**

    ---

    PCA9685, L298N, MPU6050, and plugin system for custom hardware.

    [:octicons-arrow-right-24: Drivers docs](drivers.md)

-   :material-shield:{ .lg .middle } **Safety**

    ---

    E-Stop, Watchdog, Current Monitoring, Limit Enforcement.

    [:octicons-arrow-right-24: Safety docs](safety/index.md)

-   :material-controller:{ .lg .middle } **Controllers**

    ---

    JointGroup, DifferentialDrive, Quadcopter, and more.

    [:octicons-arrow-right-24: Controllers docs](controllers.md)

-   :material-brain:{ .lg .middle } **AI Integration**

    ---

    LLM-controllable robots via ai-infra tools.

    [:octicons-arrow-right-24: AI Integration docs](ai-integration.md)

</div>

## Quick Start

```python
from robo_infra import Servo, JointGroup

# Create a simple servo
servo = Servo(name="gripper", angle_range=(0, 90))
servo.enable()
servo.set(45)  # Move to 45 degrees

# Create a robot arm
arm = JointGroup([
    Servo(name="base", angle_range=(0, 180)),
    Servo(name="shoulder", angle_range=(0, 180)),
    Servo(name="elbow", angle_range=(0, 180)),
    Servo(name="wrist", angle_range=(0, 180)),
])
arm.enable_all()
arm.home()  # Move all to default positions
```

## Installation

```bash
pip install robo-infra
```

## Simulation First

Everything works without real hardware:

```bash
# Enable simulation mode
export ROBO_SIMULATION=true

# Now all your code runs in simulation
python my_robot.py
```

## Safety First

robo-infra is built with safety as a core principle:

```python
from robo_infra.safety import EStop, Watchdog

# Create E-stop
estop = EStop()
estop.register_actuator(servo)
estop.register_actuator(arm)

# Trigger E-stop (disables ALL actuators immediately)
estop.trigger("User pressed emergency button")

# Watchdog for control loops
watchdog = Watchdog(timeout=0.1, estop=estop)
watchdog.start()

while running:
    do_control_update()
    watchdog.feed()  # Must call every 100ms
```

## Documentation

### Getting Started

<div class="grid cards" markdown>

-   :material-rocket-launch:{ .lg .middle } **Getting Started**

    ---

    Installation, first robot, connecting to hardware.

    [:octicons-arrow-right-24: Getting Started](getting-started.md)

-   :material-floor-plan:{ .lg .middle } **Architecture**

    ---

    System architecture, module hierarchy, data flow, and extension points.

    [:octicons-arrow-right-24: Architecture](architecture.md)

-   :material-book-open-variant:{ .lg .middle } **Core Concepts**

    ---

    Controller pattern, driver abstraction, bus system, simulation mode.

    [:octicons-arrow-right-24: Core Concepts](core-concepts.md)

-   :material-tools:{ .lg .middle } **Platforms**

    ---

    Raspberry Pi, Jetson, BeagleBone, Arduino integration.

    [:octicons-arrow-right-24: Platforms](platforms.md)

</div>

### Hardware Interfaces

| Guide | Description |
|-------|-------------|
| [Actuators](actuators.md) | Servo, DC Motor, Stepper, Brushless, Linear, Gripper |
| [Sensors](sensors.md) | IMU, Encoder, Distance, Force, Temperature, Current |
| [Drivers](drivers.md) | PCA9685, Dynamixel, ODrive, VESC, TMC2209, L298N |
| [Controllers](controllers.md) | JointGroup, DifferentialDrive, MecanumDrive, ArmController |

### Motion & Vision

| Guide | Description |
|-------|-------------|
| [Motion Planning](motion.md) | Trajectories, interpolation, trapezoidal/S-curve profiles |
| [Kinematics](kinematics.md) | Forward/inverse kinematics, DH parameters, Jacobian |
| [Vision](vision.md) | Camera, object detection, ArUco markers, coordinate transforms |

### Safety & Protocols

| Guide | Description |
|-------|-------------|
| [Safety](safety.md) | E-Stop, SafetyMonitor, Watchdog, JointLimiter, SafetyZone |
| [Protocols](protocols.md) | CANopen, Modbus RTU/TCP, industrial communication |
| [Power Management](power.md) | Battery monitoring, power distribution, INA219/INA226 |

### Integrations

| Guide | Description |
|-------|-------------|
| [ROS2 Integration](ros2-integration.md) | ROS2 nodes, publishers, subscribers, action servers |
| [AI Integration](ai-integration.md) | LLM tools, function calling, natural language control |
| [API Integration](api-integration.md) | FastAPI routers, REST endpoints, health checks |
| [Observability](observability.md) | Prometheus metrics, structured logging, health checks |

### Utilities & Reference

| Guide | Description |
|-------|-------------|
| [Utilities](utils.md) | Resilience, graceful degradation, resource management, security |
| [CLI](cli.md) | Command line interface, driver listing, simulation mode |
| [Error Handling](error-handling.md) | Exception types, error recovery, debugging |
| [API Reference](reference/api.md) | Full API documentation for all modules |

### Hardware Guides

| Guide | Description |
|-------|-------------|
| [Hardware Wiring](hardware-wiring.md) | Wiring diagrams, pin connections, safety considerations |
| [Hardware Testing](hardware-testing.md) | Testing procedures, calibration, validation |
| [Bill of Materials](hardware-bom.md) | Recommended components, suppliers, specifications |

## Architecture

```mermaid
graph TB
    subgraph "robo-infra"
        Actuators[Actuators]
        Sensors[Sensors]
        Controllers[Controllers]
        Safety[Safety]
        Drivers[Drivers]
    end
    
    subgraph "Hardware"
        Servos[Servos]
        Motors[Motors]
        IMU[IMU]
        Encoder[Encoder]
    end
    
    subgraph "Integration"
        AI[ai-infra]
        API[svc-infra]
    end
    
    App[Your Robot] --> Controllers
    Controllers --> Actuators
    Controllers --> Sensors
    Controllers --> Safety
    
    Actuators --> Drivers
    Sensors --> Drivers
    Drivers --> Servos
    Drivers --> Motors
    
    Controllers -.->|.as_tools| AI
    Controllers -.->|.as_router| API
```

## Supported Hardware

| Category | Hardware |
|----------|----------|
| PWM Drivers | PCA9685, GPIO PWM |
| Motor Drivers | L298N, TB6612, TMC2209 |
| IMUs | MPU6050, BNO055 |
| Distance | HC-SR04, VL53L0X |
| Platforms | Raspberry Pi, Jetson, Arduino, ESP32 |

## License

MIT License - see [LICENSE](https://github.com/nfraxlab/robo-infra/blob/main/LICENSE) for details.
