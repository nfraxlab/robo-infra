# Motion Planning

Motion planning provides algorithms for generating smooth, physically realizable robot motion. This includes PID control, trajectory generation, path planning, and sensor fusion.

## Overview

The motion module provides:

- **PID Controller** - Closed-loop feedback control
- **Trajectory Planning** - Smooth motion profiles (linear, trapezoidal, S-curve)
- **Path Planning** - Waypoint following and obstacle avoidance
- **Sensor Fusion** - Combine sensor data for accurate state estimation

```python
from robo_infra.motion import (
    PID, PIDConfig,
    TrajectoryGenerator, TrajectoryProfile,
    LinearPathPlanner, RRTPathPlanner,
    MadgwickFilter, MahonyFilter,
)
```

---

## PID Controller

The PID (Proportional-Integral-Derivative) controller is fundamental for closed-loop control in robotics.

```python
from robo_infra.motion.pid import PID, PIDConfig

# Basic PID
pid = PID(kp=2.0, ki=0.5, kd=0.1)

# Configure with limits
config = PIDConfig(
    kp=2.0,
    ki=0.5,
    kd=0.1,
    output_min=-100,
    output_max=100,
    integral_min=-10,
    integral_max=10,
    derivative_filter=0.1,
    sample_time=0.01,
)
pid = PID(config=config)
```

### Control Loop

```python
import time

setpoint = 100.0  # Target value

while True:
    measurement = sensor.read()
    output = pid.update(setpoint=setpoint, measurement=measurement)
    actuator.set(output)
    time.sleep(0.01)  # 100 Hz control loop
```

### Tuning Gains (Kp, Ki, Kd)

| Gain | Effect | Too Low | Too High |
|------|--------|---------|----------|
| **Kp** | Response speed | Slow response, steady-state error | Oscillation, overshoot |
| **Ki** | Steady-state error | Residual error | Windup, oscillation |
| **Kd** | Damping | Overshoot | Noise amplification |

**Tuning Method (Ziegler-Nichols):**

1. Set Ki = 0, Kd = 0
2. Increase Kp until sustained oscillation (critical gain Ku)
3. Measure oscillation period Tu
4. Calculate: Kp = 0.6 × Ku, Ki = 2 × Kp / Tu, Kd = Kp × Tu / 8

```python
# Typical starting values for position control
pid = PID(kp=1.0, ki=0.01, kd=0.1)

# Aggressive response
pid = PID(kp=5.0, ki=0.5, kd=0.2)

# Smooth, slow response
pid = PID(kp=0.5, ki=0.05, kd=0.1)
```

### Anti-Windup

Integral windup occurs when the actuator saturates but the integral term keeps accumulating. The PID controller includes automatic anti-windup:

```python
config = PIDConfig(
    kp=2.0,
    ki=0.5,
    kd=0.1,
    integral_min=-10,  # Clamp integral term
    integral_max=10,
    output_min=-100,   # Clamp output
    output_max=100,
)
pid = PID(config=config)
```

### Rate Limiting

For smooth actuator control, limit the rate of change:

```python
from robo_infra.motion.pid import RateLimitedPID

pid = RateLimitedPID(
    kp=2.0,
    ki=0.5,
    kd=0.1,
    max_rate=10.0,  # Max change per second
)
```

---

## Trajectory Planning

Trajectories define how a robot moves from one position to another over time, specifying position, velocity, and acceleration at each instant.

### TrajectoryPoint

```python
from robo_infra.motion.trajectory import TrajectoryPoint

point = TrajectoryPoint(
    position=50.0,
    velocity=25.0,
    acceleration=0.0,
    time=1.0,
    jerk=0.0,
)

print(f"At t={point.time}s: pos={point.position}")
print(f"Stationary: {point.is_stationary}")
print(f"Kinetic energy factor: {point.kinetic_energy_factor}")
```

### Linear Interpolation

Simple constant-velocity motion:

```python
from robo_infra.motion.trajectory import LinearInterpolator

# Move from 0 to 100 in 2 seconds
traj = LinearInterpolator(start=0.0, end=100.0, duration=2.0)

# Sample at t=1.0
point = traj.sample(1.0)
print(f"Position: {point.position}")  # 50.0
print(f"Velocity: {point.velocity}")  # 50.0 (constant)
```

### Trapezoidal Velocity Profile

Accelerate -> constant velocity -> decelerate:

```python
from robo_infra.motion.trajectory import TrapezoidalProfile

traj = TrapezoidalProfile(
    start=0.0,
    end=100.0,
    max_velocity=50.0,      # Max speed
    max_acceleration=100.0,  # Max accel/decel
)

print(f"Duration: {traj.duration}s")

# Sample trajectory
for t in [0.0, 0.5, 1.0, 1.5]:
    point = traj.sample(t)
    print(f"t={t:.1f}: pos={point.position:.1f}, vel={point.velocity:.1f}")
```

### S-Curve Profile

Jerk-limited motion for smooth acceleration:

```python
from robo_infra.motion.trajectory import SCurveProfile

traj = SCurveProfile(
    start=0.0,
    end=100.0,
    max_velocity=50.0,
    max_acceleration=100.0,
    max_jerk=500.0,  # Limits rate of acceleration change
)

point = traj.sample(0.5)
print(f"Jerk: {point.jerk}")  # Non-zero during transitions
```

### Polynomial Trajectories

Cubic and quintic polynomials for specified boundary conditions:

```python
from robo_infra.motion.trajectory import CubicPolynomial, QuinticPolynomial

# Cubic: specify position and velocity at start/end
cubic = CubicPolynomial(
    start_pos=0.0, start_vel=0.0,
    end_pos=100.0, end_vel=0.0,
    duration=2.0,
)

# Quintic: also specify acceleration
quintic = QuinticPolynomial(
    start_pos=0.0, start_vel=0.0, start_acc=0.0,
    end_pos=100.0, end_vel=0.0, end_acc=0.0,
    duration=2.0,
)
```

### TrajectoryGenerator

High-level interface for generating trajectories:

```python
from robo_infra.motion.trajectory import TrajectoryGenerator, TrajectoryProfile

gen = TrajectoryGenerator(
    profile=TrajectoryProfile.TRAPEZOIDAL,
    max_velocity=50.0,
    max_acceleration=100.0,
)

# Generate trajectory
traj = gen.generate(start=0.0, end=100.0)

# Sample at regular intervals
points = traj.sample_all(dt=0.01)
for point in points:
    actuator.set(point.position)
    time.sleep(0.01)
```

---

## Path Planning

Path planning finds a sequence of configurations to move from start to goal, optionally avoiding obstacles.

### Linear Path Planner

Simple interpolation between points:

```python
from robo_infra.motion.path_planning import LinearPathPlanner

planner = LinearPathPlanner(num_points=50)

# Plan path in joint space
path = planner.plan(
    start=[0.0, 0.0, 0.0],
    goal=[1.57, 0.78, 1.0],
)

print(f"Path has {len(path)} points")
for point in path:
    arm.move_to(point.position)
```

### Waypoint Following

Execute a series of waypoints:

```python
from robo_infra.motion.path_planning import WaypointPlanner

waypoints = [
    [0.0, 0.0, 0.0],
    [1.0, 0.5, 0.2],
    [1.5, 1.0, 0.5],
    [2.0, 1.5, 1.0],
]

planner = WaypointPlanner(interpolation_points=10)
path = planner.plan(waypoints)

for point in path:
    robot.move_to(point.position)
    time.sleep(0.1)
```

### Obstacle Avoidance

Define obstacles for collision-aware planning:

```python
from robo_infra.motion.path_planning import SphereObstacle, BoxObstacle

# Define obstacles
obstacles = [
    SphereObstacle(center=(1.0, 0.5, 0.0), radius=0.2),
    BoxObstacle(min_corner=(0.5, 0.0, 0.0), max_corner=(0.7, 0.3, 0.5)),
]

# Check collision
point = (0.6, 0.15, 0.25)
for obs in obstacles:
    if obs.contains(point):
        print("Collision!")
```

### RRT Path Planner

Rapidly-exploring Random Tree for complex environments:

```python
from robo_infra.motion.path_planning import RRTPathPlanner, SphereObstacle

# Define workspace and obstacles
obstacles = [
    SphereObstacle(center=(0.5, 0.5, 0.0), radius=0.2),
]

planner = RRTPathPlanner(
    obstacles=obstacles,
    bounds=[(-1, 1), (-1, 1), (-1, 1)],  # Workspace bounds
    step_size=0.1,
    max_iterations=1000,
    goal_tolerance=0.05,
)

# Plan path
path = planner.plan(
    start=(0.0, 0.0, 0.0),
    goal=(1.0, 1.0, 0.0),
)

if path.is_valid:
    print(f"Found path with {len(path)} waypoints")
else:
    print(f"Planning failed: {path.status}")
```

### A* Path Planner

Grid-based planning for 2D navigation:

```python
from robo_infra.motion.path_planning import AStarPlanner

# Create grid map (0 = free, 1 = obstacle)
grid = np.zeros((100, 100))
grid[30:70, 40:60] = 1  # Obstacle

planner = AStarPlanner(grid=grid, resolution=0.1)

path = planner.plan(
    start=(0.5, 0.5),
    goal=(9.5, 9.5),
)
```

### Path Smoothing

Smooth paths using B-splines or Bezier curves:

```python
from robo_infra.motion.path_planning import PathSmoother

smoother = PathSmoother(method="bspline", smoothing_factor=0.5)

# Smooth a rough path
smooth_path = smoother.smooth(path)
```

---

## Sensor Fusion

Combine data from multiple sensors for accurate state estimation.

### Complementary Filter

Simple fusion of accelerometer and gyroscope:

```python
from robo_infra.motion.fusion import ComplementaryFilter

filter = ComplementaryFilter(
    alpha=0.98,  # Trust gyro 98%, accel 2%
    sample_rate=100.0,
)

while True:
    accel = imu.read_accelerometer()
    gyro = imu.read_gyroscope()
    
    orientation = filter.update(accel, gyro)
    print(f"Roll: {orientation.euler.x:.1f}°, Pitch: {orientation.euler.y:.1f}°")
```

### Madgwick Filter

Gradient descent-based AHRS (Attitude and Heading Reference System):

```python
from robo_infra.motion.fusion import MadgwickFilter, MadgwickConfig

config = MadgwickConfig(
    beta=0.1,  # Filter gain (higher = faster, noisier)
    sample_rate=100.0,
)

filter = MadgwickFilter(config=config)

# 6-DOF update (accelerometer + gyroscope)
orientation = filter.update(accel=accel, gyro=gyro)

# 9-DOF update (+ magnetometer)
orientation = filter.update(accel=accel, gyro=gyro, mag=mag)

# Get orientation
print(f"Quaternion: {orientation.quaternion}")
print(f"Euler: roll={orientation.euler.x:.1f}°, pitch={orientation.euler.y:.1f}°, yaw={orientation.euler.z:.1f}°")
```

### Mahony Filter

Complementary filter with integral feedback:

```python
from robo_infra.motion.fusion import MahonyFilter, MahonyConfig

config = MahonyConfig(
    kp=0.5,       # Proportional gain
    ki=0.01,      # Integral gain
    sample_rate=100.0,
)

filter = MahonyFilter(config=config)

orientation = filter.update(accel=accel, gyro=gyro)
```

### Kalman Filter

Extended Kalman Filter for optimal state estimation:

```python
from robo_infra.motion.fusion import KalmanFilter

# Initialize with state dimension and measurement dimension
kf = KalmanFilter(state_dim=6, measurement_dim=3)

# Configure process and measurement noise
kf.set_process_noise(Q=np.eye(6) * 0.01)
kf.set_measurement_noise(R=np.eye(3) * 0.1)

# Prediction step
kf.predict(dt=0.01)

# Update with measurement
kf.update(measurement=np.array([x, y, z]))

# Get estimated state
state = kf.state
covariance = kf.covariance
```

### IMU Fusion Example

Complete IMU fusion pipeline:

```python
from robo_infra.motion.fusion import MadgwickFilter
from robo_infra.sensors import BNO055

# Initialize IMU and filter
imu = BNO055(i2c_bus=1)
filter = MadgwickFilter(sample_rate=100.0, beta=0.1)

imu.enable()

while True:
    # Read IMU data
    accel = imu.read_accelerometer()
    gyro = imu.read_gyroscope()
    mag = imu.read_magnetometer()
    
    # Update filter
    orientation = filter.update(accel=accel, gyro=gyro, mag=mag)
    
    # Use orientation for control
    if abs(orientation.euler.x) > 30:  # Roll > 30°
        print("Warning: Robot tilting!")
    
    time.sleep(0.01)  # 100 Hz
```

---

## Next Steps

- [Kinematics](kinematics.md) - Forward and inverse kinematics
- [Controllers](controllers.md) - Robot controllers
- [Sensors](sensors.md) - IMU and other sensors
- [Safety](safety.md) - Limit and watchdog systems
