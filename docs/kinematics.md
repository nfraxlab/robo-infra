# Kinematics

Kinematics deals with the geometry of robot motion—converting between joint angles and end-effector positions without considering forces or torques.

## Overview

- **Forward Kinematics (FK)**: Joint angles → End-effector position
- **Inverse Kinematics (IK)**: End-effector position → Joint angles

```python
from robo_infra.motion import (
    TwoLinkArm, ThreeLinkArm,
    DHParameter, DHChain,
    DeltaRobot, StewartPlatform, SCARAArm,
    Transform, Rotation,
    JacobianIKSolver, DampedLeastSquaresIK,
)
```

---

## DH Parameters

Denavit-Hartenberg (DH) parameters are the standard method for describing serial link manipulators.

### Parameter Definition

Each link is described by 4 parameters:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| **d** | Link offset | Distance along Z from X_{i-1} to X_i |
| **θ** | Joint angle | Rotation around Z from X_{i-1} to X_i |
| **a** | Link length | Distance along X from Z_{i-1} to Z_i |
| **α** | Link twist | Rotation around X from Z_{i-1} to Z_i |

For revolute joints, **θ** is the variable. For prismatic joints, **d** is the variable.

### Creating a DH Chain

```python
from robo_infra.motion.dh_parameters import DHParameter, DHChain, JointType

# Define a 3-DOF planar arm
params = [
    DHParameter(d=0, theta=0, a=0.1, alpha=0, name="shoulder"),
    DHParameter(d=0, theta=0, a=0.1, alpha=0, name="elbow"),
    DHParameter(d=0, theta=0, a=0.05, alpha=0, name="wrist"),
]

chain = DHChain(parameters=params)

# Forward kinematics
joint_angles = [0.0, 0.0, 0.0]
transform = chain.forward(joint_angles)

print(f"End effector position: {transform.position}")
print(f"End effector orientation: {transform.rotation}")
```

### Joint Limits

```python
from robo_infra.motion.dh_parameters import DHParameter, JointLimit

param = DHParameter(
    d=0,
    theta=0,
    a=0.1,
    alpha=0,
    limit=JointLimit(min_val=-1.57, max_val=1.57),  # ±90°
    name="shoulder",
)

# Check if angle is valid
print(param.limit.is_within(0.5))  # True
print(param.limit.clamp(3.0))      # 1.57
```

### Convention Types

```python
from robo_infra.motion.dh_parameters import DHConvention

# Standard DH (Craig's textbook)
chain = DHChain(parameters=params, convention=DHConvention.STANDARD)

# Modified DH (some manufacturers)
chain = DHChain(parameters=params, convention=DHConvention.MODIFIED)
```

---

## Serial Arm Kinematics

### 2-DOF Planar Arm

Simple two-link arm with analytical solutions:

```python
from robo_infra.motion.kinematics import TwoLinkArm, EndEffectorPose, ElbowConfiguration

# Create arm with 100mm links
arm = TwoLinkArm(l1=0.1, l2=0.1)

# Forward kinematics
pose = arm.forward([0.0, 0.0])  # Both joints at 0
print(f"Position: ({pose.x:.3f}, {pose.y:.3f})")  # (0.2, 0.0)

# Inverse kinematics
target = EndEffectorPose(x=0.15, y=0.05)
angles = arm.inverse(target, config=ElbowConfiguration.UP)
print(f"Joint angles: {[math.degrees(a) for a in angles]}")

# Check reachability
print(arm.is_reachable(EndEffectorPose(x=0.3, y=0.0)))  # False (too far)
print(arm.is_reachable(EndEffectorPose(x=0.1, y=0.1)))  # True
```

### 3-DOF Arm

Three-link planar arm with wrist:

```python
from robo_infra.motion.kinematics import ThreeLinkArm

arm = ThreeLinkArm(l1=0.1, l2=0.1, l3=0.05)

# Forward kinematics
pose = arm.forward([0.0, 0.0, 0.0])
print(f"Position: ({pose.x:.3f}, {pose.y:.3f})")
print(f"Orientation: {math.degrees(pose.orientation):.1f}°")

# Inverse kinematics with desired orientation
target = EndEffectorPose(x=0.15, y=0.05, orientation=math.radians(45))
angles = arm.inverse(target)
```

### 6-DOF Arm

Using DH parameters for a 6-axis robot:

```python
from robo_infra.motion.dh_parameters import DHParameter, DHChain, create_puma560

# Create PUMA 560 robot (classic 6-DOF arm)
puma = create_puma560()

# Forward kinematics
joint_angles = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
transform = puma.forward(joint_angles)

print(f"Position: {transform.position}")
print(f"Rotation matrix:\n{transform.rotation.matrix}")
```

---

## Parallel Kinematics

### Delta Robot

3-DOF parallel manipulator for high-speed pick-and-place:

```python
from robo_infra.motion.delta import DeltaRobot, DeltaJoints

# Create delta robot
delta = DeltaRobot(
    base_radius=0.2,       # Fixed base platform radius
    effector_radius=0.05,  # Moving platform radius
    upper_arm_length=0.2,  # Upper arm (attached to motor)
    lower_arm_length=0.4,  # Lower arm (parallelogram)
)

# Forward kinematics: motor angles → effector position
x, y, z = delta.forward(theta1=0.0, theta2=0.0, theta3=0.0)
print(f"Effector at: ({x:.3f}, {y:.3f}, {z:.3f})")

# Inverse kinematics: position → motor angles
joints = delta.inverse(x=0.0, y=0.0, z=-0.3)
print(f"Motor angles: θ1={math.degrees(joints.theta1):.1f}°")

# Check workspace
print(delta.is_reachable(0.0, 0.0, -0.2))  # True
print(delta.is_reachable(0.0, 0.0, -0.5))  # False (too far)

# Get Jacobian for velocity kinematics
J = delta.jacobian(joints)
```

### Stewart Platform

6-DOF parallel manipulator (hexapod):

```python
from robo_infra.motion.stewart import StewartPlatform, StewartPose
from robo_infra.motion.transforms import Transform

# Create symmetric hexapod
stewart = StewartPlatform.create_symmetric(
    base_radius=0.3,          # Base platform radius
    platform_radius=0.15,     # Moving platform radius
    leg_length_range=(0.4, 0.6),
)

# Inverse kinematics: pose → leg lengths (closed-form)
pose = Transform.from_translation(0, 0, 0.5)
leg_lengths = stewart.inverse(pose)
print(f"Leg lengths: {leg_lengths}")

# Forward kinematics: leg lengths → pose (iterative)
pose = stewart.forward(leg_lengths)
print(f"Platform position: {pose.position}")
print(f"Platform orientation: {pose.euler_angles}")

# Check singularity
singularity = stewart.check_singularity(pose)
if singularity != StewartSingularityType.NONE:
    print(f"Warning: Near singularity ({singularity})")
```

### SCARA

4-DOF arm for assembly and pick-and-place:

```python
from robo_infra.motion.scara import SCARAArm, SCARAConfiguration

# Create SCARA with 200mm and 150mm arm lengths
scara = SCARAArm(
    l1=0.2,
    l2=0.15,
    z_range=(0.0, 0.1),
)

# Forward kinematics
pose = scara.forward(
    theta1=0.0,   # Shoulder
    theta2=0.0,   # Elbow
    z=0.05,       # Vertical position
    theta4=0.0,   # Wrist rotation
)
print(f"End effector: {pose.position}")

# Inverse kinematics
joints = scara.inverse(
    pose,
    config=SCARAConfiguration.RIGHT_ARM,
)
print(f"Joints: θ1={math.degrees(joints.theta1):.1f}°, θ2={math.degrees(joints.theta2):.1f}°")

# Workspace analysis
print(f"Reach: {scara.min_reach:.3f} to {scara.max_reach:.3f} m")
```

---

## Transforms

### Rotation Matrices

```python
from robo_infra.motion.transforms import Rotation, EulerOrder

# Create from Euler angles
rot = Rotation.from_euler(
    angles=(0, 0, 90),
    order=EulerOrder.XYZ,
    degrees=True,
)

# Convert to different representations
print(f"Matrix:\n{rot.matrix}")
print(f"Euler (deg): {rot.as_euler(degrees=True)}")
print(f"Quaternion: {rot.as_quaternion()}")
print(f"Axis-angle: {rot.as_axis_angle()}")

# Create from quaternion (x, y, z, w)
rot = Rotation.from_quaternion((0, 0, 0.707, 0.707))

# Create from axis-angle
rot = Rotation.from_axis_angle(axis=(0, 0, 1), angle=1.57)

# Compose rotations
rot3 = rot1 @ rot2
```

### Homogeneous Transforms

4×4 matrices representing position and orientation:

```python
from robo_infra.motion.transforms import Transform

# Create from position and Euler angles
t1 = Transform.from_euler(
    position=(1.0, 2.0, 3.0),
    angles=(0, 0, 90),
    degrees=True,
)

# Create from position and quaternion
t2 = Transform.from_quaternion(
    position=(0, 0, 0),
    quaternion=(0, 0, 0.707, 0.707),
)

# Create from translation only
t3 = Transform.from_translation(1.0, 2.0, 3.0)

# Access components
print(f"Position: {t1.position}")
print(f"Rotation: {t1.rotation}")
print(f"4x4 Matrix:\n{t1.matrix}")

# Compose transforms
t4 = t1 @ t2  # Apply t2 then t1

# Inverse
t5 = t1.inverse()

# Transform a point
point = np.array([1.0, 0.0, 0.0])
transformed = t1.transform_point(point)
```

### Coordinate Frames

```python
# Robot base frame
base_frame = Transform.identity()

# Tool frame (offset from end effector)
tool_frame = Transform.from_translation(0, 0, 0.1)

# World to base transform
world_to_base = Transform.from_euler(
    position=(1.0, 2.0, 0.0),
    angles=(0, 0, 45),
    degrees=True,
)

# Transform point from tool frame to world frame
point_in_tool = np.array([0.0, 0.0, 0.0])
end_effector = arm.forward(joint_angles)

point_in_world = world_to_base @ end_effector @ tool_frame
print(f"Tool position in world: {point_in_world.position}")
```

---

## IK Solvers

For complex robots where analytical IK isn't available.

### Jacobian-Based (Pseudo-Inverse)

```python
from robo_infra.motion.ik_solvers import JacobianIKSolver
from robo_infra.motion.dh_parameters import create_planar_3dof

chain = create_planar_3dof(l1=0.1, l2=0.1, l3=0.05)
solver = JacobianIKSolver(chain)

target = Transform.from_translation(0.15, 0.05, 0)
solution = solver.solve(target, initial_guess=[0.0, 0.0, 0.0])

if solution is not None:
    print(f"Joint angles: {solution}")
```

### Damped Least Squares (DLS)

Singularity-robust method:

```python
from robo_infra.motion.ik_solvers import DampedLeastSquaresIK, IKSolverConfig

config = IKSolverConfig(
    max_iterations=100,
    position_tolerance=1e-4,
    orientation_tolerance=1e-3,
    damping=0.01,  # Damping factor (λ)
)

solver = DampedLeastSquaresIK(chain, config=config)

# Get detailed result
result = solver.solve_detailed(target)
print(f"Success: {result.success}")
print(f"Status: {result.status}")
print(f"Iterations: {result.iterations}")
print(f"Position error: {result.error_position:.6f} m")
print(f"Joint values: {result.joint_values}")
```

### Cyclic Coordinate Descent (CCD)

Good for long kinematic chains:

```python
from robo_infra.motion.ik_solvers import CCDIKSolver

solver = CCDIKSolver(
    chain,
    max_iterations=50,
    tolerance=1e-4,
)

solution = solver.solve(target)
```

### Multiple Solutions

Handle multiple IK solutions:

```python
from robo_infra.motion.ik_solvers import MultiSolutionIK

solver = MultiSolutionIK(chain)
solutions = solver.solve_all(target)

print(f"Found {len(solutions)} solutions")
for i, sol in enumerate(solutions):
    print(f"Solution {i}: {sol}")

# Select best solution (closest to current)
current_joints = [0.5, 0.3, 0.1]
best = solver.select_nearest(solutions, current_joints)
```

---

## Practical Examples

### Arm Pick-and-Place

```python
from robo_infra.motion.kinematics import ThreeLinkArm, EndEffectorPose
from robo_infra.motion.trajectory import TrajectoryGenerator, TrajectoryProfile
import time

# Create arm
arm = ThreeLinkArm(l1=0.15, l2=0.15, l3=0.05)
trajectory_gen = TrajectoryGenerator(
    profile=TrajectoryProfile.TRAPEZOIDAL,
    max_velocity=1.0,
    max_acceleration=2.0,
)

# Define positions
home = EndEffectorPose(x=0.2, y=0.0, orientation=0)
pick = EndEffectorPose(x=0.25, y=0.1, orientation=math.radians(-90))
place = EndEffectorPose(x=0.25, y=-0.1, orientation=math.radians(-90))

# Get joint angles
home_angles = arm.inverse(home)
pick_angles = arm.inverse(pick)
place_angles = arm.inverse(place)

# Generate trajectory from home to pick
def move_to(start_angles, end_angles, duration=1.0):
    trajectories = [
        trajectory_gen.generate(s, e, duration=duration)
        for s, e in zip(start_angles, end_angles)
    ]
    
    t = 0
    dt = 0.01
    while t < duration:
        joint_cmds = [traj.sample(t).position for traj in trajectories]
        robot.move_joints(joint_cmds)
        t += dt
        time.sleep(dt)

# Execute pick and place
move_to(home_angles, pick_angles)
gripper.close()
move_to(pick_angles, place_angles)
gripper.open()
move_to(place_angles, home_angles)
```

### Delta Robot Calibration

```python
from robo_infra.motion.delta import DeltaRobot
import numpy as np

# Create delta robot
delta = DeltaRobot(
    base_radius=0.2,
    effector_radius=0.05,
    upper_arm_length=0.2,
    lower_arm_length=0.4,
)

# Calibration routine
def calibrate_delta():
    """Move to known positions and verify."""
    test_positions = [
        (0.0, 0.0, -0.3),   # Center
        (0.1, 0.0, -0.3),   # X offset
        (0.0, 0.1, -0.3),   # Y offset
        (0.0, 0.0, -0.25),  # Z up
    ]
    
    for target_pos in test_positions:
        # Get motor angles
        joints = delta.inverse(*target_pos)
        
        # Move motors
        motor1.set_angle(math.degrees(joints.theta1))
        motor2.set_angle(math.degrees(joints.theta2))
        motor3.set_angle(math.degrees(joints.theta3))
        
        time.sleep(1.0)
        
        # Verify with forward kinematics
        actual_pos = delta.forward(joints.theta1, joints.theta2, joints.theta3)
        error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
        print(f"Target: {target_pos}, Actual: {actual_pos}, Error: {error:.4f}")

calibrate_delta()
```

---

## Next Steps

- [Motion Planning](motion.md) - Trajectory and path planning
- [Controllers](controllers.md) - Arm controllers with kinematics
- [Drivers](drivers.md) - Motor drivers for actuators
- [Safety](safety.md) - Joint limits and safety systems
