# Controllers

Controllers coordinate multiple actuators and sensors as unified robotic systems. They provide high-level abstractions for common robot configurations.

## Overview

All controllers extend the base `Controller` class and share common patterns:

- **Enable/disable lifecycle** - Must enable before operation
- **State management** - Track operational state
- **Actuator coordination** - Manage multiple actuators together
- **Tool generation** - Export as AI-controllable tools

```python
from robo_infra.controllers import JointGroup
from robo_infra.actuators import Servo

# Create actuators
joints = [
    Servo(name="base", channel=0),
    Servo(name="shoulder", channel=1),
    Servo(name="elbow", channel=2),
]

# Create controller
arm = JointGroup(name="robot_arm", joints=joints)

# Enable and use
arm.enable()
arm.home()
arm.move_to({"base": 90, "shoulder": 45, "elbow": 90})

# Clean up
arm.disable()
```

---

## Arm Controllers

### JointGroup

Coordinates multiple joints as a robot arm. Supports named positions, motion sequences, and interpolated movement.

```python
from robo_infra.controllers import JointGroup, JointGroupConfig
from robo_infra.actuators import Servo

# Create joint actuators
joints = {
    "base": Servo(name="base", channel=0, angle_range=(0, 180)),
    "shoulder": Servo(name="shoulder", channel=1, angle_range=(0, 180)),
    "elbow": Servo(name="elbow", channel=2, angle_range=(0, 180)),
    "wrist": Servo(name="wrist", channel=3, angle_range=(0, 180)),
    "gripper": Servo(name="gripper", channel=4, angle_range=(0, 90)),
}

# Create controller with configuration
config = JointGroupConfig(
    name="robot_arm",
    home_positions={"base": 90, "shoulder": 90, "elbow": 90, "wrist": 90, "gripper": 0},
    named_positions={
        "ready": {"base": 90, "shoulder": 45, "elbow": 90, "wrist": 90, "gripper": 0},
        "pick": {"base": 90, "shoulder": 120, "elbow": 60, "wrist": 45, "gripper": 0},
        "place": {"base": 45, "shoulder": 100, "elbow": 80, "wrist": 60, "gripper": 0},
    },
    interpolation_steps=50,
    default_speed=0.5,
)

arm = JointGroup(name="robot_arm", joints=joints, config=config)
```

**Methods:**

```python
arm.enable()

# Move all joints at once
arm.move_to({"base": 90, "shoulder": 45, "elbow": 90, "wrist": 90})

# Move with interpolation (smooth motion)
await arm.move_to_interpolated(
    {"base": 45, "shoulder": 60, "elbow": 120, "wrist": 45},
    duration=2.0,  # 2 seconds
)

# Home position
arm.home()

# Named positions
arm.save_position("custom_pose")
arm.go_to_named("ready")
arm.go_to_named("pick")

# Get current positions
positions = arm.get_positions()
print(positions)  # {"base": 90, "shoulder": 45, ...}

# Execute sequences
arm.add_sequence("pick_cycle", [
    {"base": 90, "shoulder": 120, "elbow": 60, "wrist": 45, "gripper": 0},
    {"base": 90, "shoulder": 120, "elbow": 60, "wrist": 45, "gripper": 90},  # Close gripper
    {"base": 90, "shoulder": 45, "elbow": 90, "wrist": 90, "gripper": 90},   # Lift
    {"base": 45, "shoulder": 100, "elbow": 80, "wrist": 60, "gripper": 90},  # Move to place
    {"base": 45, "shoulder": 100, "elbow": 80, "wrist": 60, "gripper": 0},   # Release
])
await arm.run_sequence("pick_cycle", step_delay=0.5)

arm.disable()
```

### Named Positions and Sequences

Store and recall common poses:

```python
# Add named position from current state
arm.save_position("inspection")

# Add named position explicitly
arm.add_position("folded", {
    "base": 90,
    "shoulder": 180,
    "elbow": 0,
    "wrist": 90,
})

# List all named positions
print(arm.named_positions.keys())

# Execute sequence with timing
await arm.run_sequence("pick_cycle", step_delay=1.0)
```

---

## Mobile Controllers

### DifferentialDrive

Two-wheel differential drive for tank-style robots.

```python
from robo_infra.controllers import DifferentialDrive, DifferentialDriveConfig
from robo_infra.actuators import DCMotor

# Create motors
left_motor = DCMotor(name="left_wheel", pin_a=17, pin_b=18, enable=12)
right_motor = DCMotor(name="right_wheel", pin_a=22, pin_b=23, enable=13)

# Create controller
config = DifferentialDriveConfig(
    name="rover",
    wheel_diameter=0.065,  # 65mm wheels
    track_width=0.15,      # 150mm between wheels
    max_speed=1.0,         # 1 m/s max
    max_angular_speed=3.14,  # ~180°/s
)

rover = DifferentialDrive(
    name="my_rover",
    left=left_motor,
    right=right_motor,
    config=config,
)
```

**Methods:**

```python
rover.enable()

# Basic movement
rover.forward(speed=0.5)       # 50% forward
rover.reverse(speed=0.3)       # 30% reverse
rover.turn_left(speed=0.4)     # Spin left in place
rover.turn_right(speed=0.4)    # Spin right in place
rover.stop()                   # Stop all motors

# Arcade drive (joystick-style)
rover.arcade_drive(
    forward=0.5,   # Forward/reverse (-1 to 1)
    turn=0.2,      # Left/right turn (-1 to 1)
)

# Tank drive (left/right independent)
rover.tank_drive(
    left=0.6,      # Left motor speed (-1 to 1)
    right=0.4,     # Right motor speed (-1 to 1)
)

# Velocity control (requires encoders)
rover.drive(
    linear=0.3,    # 0.3 m/s forward
    angular=0.5,   # 0.5 rad/s turn
)

# Odometry (if encoders available)
pose = rover.get_pose()
print(f"X: {pose.x}, Y: {pose.y}, Theta: {pose.theta}")

rover.disable()
```

### Hexapod

Six-legged walking robot with various gait patterns.

```python
from robo_infra.controllers import Hexapod, HexapodConfig, GaitType
from robo_infra.controllers import create_hexapod

# Create simulated hexapod
hexapod = create_hexapod(name="spider", simulated=True)

# Or configure manually
config = HexapodConfig(
    name="hexapod",
    leg_length=0.15,      # 150mm legs
    body_radius=0.1,      # 100mm body radius
    default_height=0.08,  # 80mm standing height
)
```

**Methods:**

```python
hexapod.enable()

# Stand up
hexapod.stand(height=0.1)  # 100mm standing height

# Walk with different gaits
hexapod.walk(direction=0, speed=0.5)      # Walk forward
hexapod.walk(direction=90, speed=0.3)     # Walk right
hexapod.walk(direction=180, speed=0.4)    # Walk backward

# Set gait pattern
hexapod.set_gait(GaitType.TRIPOD)   # Fast (3 legs at a time)
hexapod.set_gait(GaitType.WAVE)     # Slow (1 leg at a time)
hexapod.set_gait(GaitType.RIPPLE)   # Medium (2 legs at a time)

# Body pose adjustments
hexapod.set_body_pose(
    roll=5.0,    # Lean right 5°
    pitch=10.0,  # Lean forward 10°
    yaw=0.0,
)

# Rotate in place
hexapod.rotate(angular_velocity=0.5)  # rad/s

# Stop
hexapod.stop()
hexapod.sit()  # Lower to ground

hexapod.disable()
```

---

## Aerial Controllers

### Quadcopter

Four-motor flight controller with multiple flight modes.

```python
from robo_infra.controllers import Quadcopter, QuadcopterConfig, FlightMode
from robo_infra.actuators import Brushless

# Create motors (FL, FR, RL, RR)
motors = [Brushless(name=f"motor_{i}", channel=i) for i in range(4)]

# Create controller
config = QuadcopterConfig(
    name="drone",
    arm_length=0.25,      # 250mm arm length
    frame_type="X",       # X, +, or H configuration
    motor_kv=2300,
    prop_diameter=5.0,    # 5-inch props
    max_tilt_angle=35.0,  # Max tilt in degrees
)

quad = Quadcopter(name="my_drone", motors=motors, config=config)
```

**Methods:**

```python
quad.enable()

# Arm (enable motors)
quad.arm()

# Basic flight
await quad.takeoff(altitude=2.0)  # Takeoff to 2m
await quad.hover()                 # Maintain position
await quad.land()                  # Land

# Manual control
quad.set_attitude(
    throttle=0.5,  # 0-1
    roll=0.0,      # -1 to 1
    pitch=0.0,     # -1 to 1
    yaw=0.0,       # -1 to 1
)

# Flight modes
quad.set_mode(FlightMode.STABILIZE)   # Attitude hold
quad.set_mode(FlightMode.ALT_HOLD)    # Altitude hold
quad.set_mode(FlightMode.LOITER)      # Position hold
quad.set_mode(FlightMode.AUTO)        # Autonomous waypoints

# Waypoint navigation (requires GPS)
quad.add_waypoint(lat=37.7749, lon=-122.4194, alt=10.0)
await quad.fly_to_waypoint(0)

# Emergency
quad.emergency_stop()  # Kill motors immediately

quad.disarm()
quad.disable()
```

### MAVLink

Interface with ArduPilot/PX4 flight controllers via MAVLink protocol.

```python
from robo_infra.controllers import MAVLinkController

mavlink = MAVLinkController(
    connection_string="/dev/ttyUSB0",
    baudrate=57600,
)

mavlink.connect()

# Request data streams
mavlink.request_data_stream(rate=10)

# Read telemetry
attitude = mavlink.get_attitude()
print(f"Roll: {attitude.roll}, Pitch: {attitude.pitch}, Yaw: {attitude.yaw}")

position = mavlink.get_global_position()
battery = mavlink.get_battery_status()

# Send commands
mavlink.arm()
mavlink.takeoff(10.0)
mavlink.goto(lat=37.7749, lon=-122.4194, alt=50.0)
mavlink.land()
mavlink.disarm()

mavlink.disconnect()
```

---

## Mechanism Controllers

### Gripper

Open/close gripper with optional force sensing.

```python
from robo_infra.controllers import Gripper, GripperConfig
from robo_infra.actuators import Servo

# Create gripper actuator
actuator = Servo(name="gripper_servo", channel=0, angle_range=(0, 90))

# Create controller
config = GripperConfig(
    name="gripper",
    open_position=0,
    closed_position=90,
    grip_threshold=5.0,        # Position tolerance
    grip_force_threshold=2.0,  # Force threshold (if sensor)
)

gripper = Gripper(name="my_gripper", actuator=actuator, config=config)
```

**Methods:**

```python
gripper.enable()

# Basic operations
gripper.open()
gripper.close()
gripper.toggle()

# Grip with detection
await gripper.grip()  # Close until object detected or fully closed
gripper.release()

# Partial positions
gripper.set_position(0.5)  # 50% closed

# Check state
print(gripper.state)       # GripperState.OPEN, CLOSED, GRIPPING
print(gripper.is_gripping)
print(gripper.position)

gripper.disable()
```

### PanTilt

Two-axis camera/sensor positioning.

```python
from robo_infra.controllers import PanTilt, PanTiltConfig
from robo_infra.actuators import Servo

# Create servos
pan_servo = Servo(name="pan", channel=0, angle_range=(-90, 90))
tilt_servo = Servo(name="tilt", channel=1, angle_range=(-45, 45))

# Create controller
config = PanTiltConfig(
    name="camera_head",
    pan_range=(-90, 90),
    tilt_range=(-45, 45),
    center_pan=0,
    center_tilt=0,
    pan_speed=60,        # deg/s
    tilt_speed=45,       # deg/s
    image_width=640,     # For tracking
    image_height=480,
    fov_horizontal=60,   # Camera FOV
    fov_vertical=45,
)

pt = PanTilt(
    name="camera_head",
    pan_actuator=pan_servo,
    tilt_actuator=tilt_servo,
    config=config,
)
```

**Methods:**

```python
pt.enable()

# Direct positioning
pt.look_at(pan=45.0, tilt=30.0)
pt.center()
pt.pan_to(90.0)
pt.tilt_to(-30.0)

# Relative movement
pt.pan_by(10.0)   # Pan right 10°
pt.tilt_by(-5.0)  # Tilt down 5°

# Track pixel coordinates
pt.track((320, 240))  # Track center of image
pt.track_object(bbox=(100, 100, 200, 200))  # Track bounding box center

# Scanning pattern
await pt.scan(
    pan_start=-45,
    pan_end=45,
    tilt_start=-20,
    tilt_end=20,
    step=10,
    dwell=0.5,  # Pause at each position
)

pt.disable()
```

### Turntable

Rotating platform for object scanning or positioning.

```python
from robo_infra.controllers import Turntable, TurntableConfig
from robo_infra.actuators import Stepper

motor = Stepper(step_pin=17, dir_pin=18, steps_per_rev=200, microsteps=16)

config = TurntableConfig(
    name="scanner_table",
    steps_per_revolution=3200,  # 200 * 16 microsteps
    max_speed=60.0,             # degrees/second
)

table = Turntable(name="scanner", motor=motor, config=config)
```

**Methods:**

```python
table.enable()

# Rotate to absolute angle
table.rotate_to(90.0)
table.rotate_to(180.0)

# Rotate by relative amount
table.rotate_by(45.0)
table.rotate_by(-90.0)

# Continuous rotation
table.spin(speed=30.0)   # 30°/s
table.spin(speed=-15.0)  # Reverse
table.stop()

# Home position
table.home()

# Indexed rotation (for scanning)
for _ in range(12):
    table.rotate_by(30.0)  # 12 positions
    capture_image()

table.disable()
```

### Conveyor

Linear belt motion for material handling.

```python
from robo_infra.controllers import Conveyor, ConveyorConfig
from robo_infra.actuators import DCMotor

motor = DCMotor(name="conveyor_motor", pin_a=17, pin_b=18, enable=12)

config = ConveyorConfig(
    name="main_conveyor",
    belt_length=2.0,       # 2 meters
    speed_max=0.5,         # 0.5 m/s max
    index_distance=0.1,    # 10cm per index
    acceleration=0.5,      # m/s²
)

conveyor = Conveyor(name="main_conveyor", motor=motor, config=config)
```

**Methods:**

```python
conveyor.enable()

# Continuous run
conveyor.run(speed=0.3)                 # 0.3 m/s forward
conveyor.run(speed=0.3, reverse=True)   # Reverse
conveyor.stop()

# Move specific distance
conveyor.jog(distance=0.5)              # Move 50cm

# Index by increments
conveyor.index(count=3)                 # Move 3 index positions

# Wait for sensor
conveyor.run(speed=0.2)
await conveyor.wait_for_sensor(sensor)  # Stop when triggered

conveyor.disable()
```

---

## Utility Controllers

### Lock

Electronic lock/latch mechanism.

```python
from robo_infra.controllers import Lock, LockConfig
from robo_infra.actuators import Servo

actuator = Servo(name="lock_servo", channel=0, angle_range=(0, 90))

config = LockConfig(
    name="door_lock",
    locked_position=0,
    unlocked_position=90,
    transition_time=0.5,
)

lock = Lock(name="my_lock", actuator=actuator, config=config)
```

**Methods:**

```python
lock.enable()

lock.unlock()
lock.lock()
lock.toggle()

# Check state
print(lock.state)      # LockState.LOCKED, UNLOCKED
print(lock.is_locked)

# Timed unlock
await lock.unlock_for(duration=5.0)  # Unlock for 5 seconds

lock.disable()
```

### Leg

Single robotic leg for walking robots (used by Hexapod/Quadruped).

```python
from robo_infra.controllers import Leg, LegConfig
from robo_infra.actuators import Servo

# 3-DOF leg (coxa, femur, tibia)
joints = {
    "coxa": Servo(name="coxa", channel=0, angle_range=(-45, 45)),
    "femur": Servo(name="femur", channel=1, angle_range=(0, 90)),
    "tibia": Servo(name="tibia", channel=2, angle_range=(0, 135)),
}

config = LegConfig(
    name="front_left",
    coxa_length=0.03,   # 30mm
    femur_length=0.08,  # 80mm
    tibia_length=0.12,  # 120mm
)

leg = Leg(name="front_left", joints=joints, config=config)
```

**Methods:**

```python
leg.enable()

# Inverse kinematics - move foot to position
leg.move_to(x=0.1, y=0.0, z=-0.1)  # 10cm forward, 10cm down

# Forward kinematics - get current foot position
position = leg.get_foot_position()
print(f"Foot at: ({position.x}, {position.y}, {position.z})")

# Home position
leg.home()

leg.disable()
```

---

## Creating Custom Controllers

Extend the `Controller` base class for custom robot types.

```python
from robo_infra.core.controller import Controller, ControllerConfig
from robo_infra.core.actuator import Actuator

class MyRobotConfig(ControllerConfig):
    """Configuration for MyRobot."""
    max_speed: float = 1.0
    safety_enabled: bool = True

class MyRobot(Controller):
    """Custom robot controller."""
    
    def __init__(
        self,
        name: str,
        actuators: list[Actuator],
        config: MyRobotConfig | None = None,
    ):
        super().__init__(name=name, config=config)
        self._actuators = {a.name: a for a in actuators}
        self._config = config or MyRobotConfig()
    
    def enable(self) -> None:
        """Enable all actuators."""
        for actuator in self._actuators.values():
            actuator.enable()
        self._enabled = True
    
    def disable(self) -> None:
        """Disable all actuators."""
        for actuator in self._actuators.values():
            actuator.disable()
        self._enabled = False
    
    def home(self) -> None:
        """Move to home position."""
        for actuator in self._actuators.values():
            actuator.set(actuator.limits.default)
    
    def do_something(self) -> None:
        """Custom behavior."""
        # Implement custom robot behavior
        pass
```

### Lifecycle Management

Controllers follow a standard lifecycle:

```python
class MyRobot(Controller):
    
    def enable(self) -> None:
        """Called when controller is enabled."""
        super().enable()
        self._initialize_hardware()
    
    def disable(self) -> None:
        """Called when controller is disabled."""
        self._stop_all()
        super().disable()
    
    def emergency_stop(self) -> None:
        """Called on emergency stop."""
        for actuator in self._actuators.values():
            actuator.set(0)
            actuator.disable()
        self._state = ControllerState.STOPPED
```

### Tool Generation for AI

Export controller methods as AI-controllable tools:

```python
from robo_infra.integrations.ai import as_tools

# Generate tools from controller
tools = as_tools(arm)

# Use with ai-infra Agent
from ai_infra import Agent

agent = Agent(
    model="gpt-4",
    tools=tools,
)

response = await agent.run("Move the robot arm to the ready position")
```

---

## Next Steps

- [Actuators](actuators.md) - Actuators used by controllers
- [Sensors](sensors.md) - Sensors for feedback
- [Drivers](drivers.md) - Hardware drivers
- [AI Integration](ai-integration.md) - LLM-controlled robots
- [Motion](motion.md) - Motion planning and kinematics
