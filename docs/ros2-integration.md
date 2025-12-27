# ROS2 Integration

robo-infra provides optional ROS2 integration, allowing controllers to be exposed as ROS2 nodes with standard message types.

## Overview

The ROS2 integration bridges robo-infra controllers with ROS2:

- Convert controllers to ROS2 nodes
- Publish status via standard messages (JointState, Twist, Pose)
- Subscribe to command topics
- Expose services for control operations
- Auto-generate launch files

**Key Feature**: All robo-infra functionality works without ROS2 installed. When ROS2 is unavailable, a mock mode enables testing.

## Installation

```bash
# ROS2 dependencies (optional)
pip install robo-infra[ros2]

# Or install ROS2 messages manually
# Requires: rclpy, geometry_msgs, sensor_msgs, std_msgs, std_srvs
```

## Node Creation

### Convert Controller to ROS2 Node

```python
from robo_infra.controllers import JointGroup
from robo_infra.integrations.ros2 import controller_to_ros2_node
import rclpy

# Create controller
arm = JointGroup("arm", joints=[joint1, joint2, joint3])

# Convert to ROS2 node
node = controller_to_ros2_node(
    arm,
    node_name="arm_controller",
    namespace="/robot",
    publish_rate_hz=50.0,
    use_sim_time=False,
)

# Spin the node
rclpy.init()
rclpy.spin(node.get_node())
```

### Node Configuration

```python
from robo_infra.integrations.ros2 import ROS2NodeConfig, ControllerROS2Node

config = ROS2NodeConfig(
    node_name="arm_controller",
    namespace="/robot",
    publish_rate_hz=50.0,        # Status publish rate
    use_sim_time=False,           # Use simulation time
    qos_depth=10,                 # QoS history depth
    qos_reliable=True,            # Reliable vs best-effort
    enable_services=True,         # Create ROS2 services
    enable_subscribers=True,      # Create command subscribers
    frame_id="base_link",         # TF frame ID
)

node = ControllerROS2Node(arm, config)
```

## Publishing Sensor Data

### JointState Messages

Controllers automatically publish JointState messages:

```
Topic: /{controller_name}/joint_states
Type: sensor_msgs/JointState

Fields:
  - header.frame_id: TF frame (configurable)
  - header.stamp: Current time
  - name: Joint names from actuators
  - position: Current joint positions
  - velocity: Current velocities (if available)
  - effort: Current efforts (if available)
```

Example message:

```yaml
header:
  frame_id: "base_link"
  stamp:
    sec: 1704067200
    nanosec: 0
name: ["joint1", "joint2", "joint3"]
position: [0.0, 45.0, 90.0]
velocity: [0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0]
```

### Status Messages

```
Topic: /{controller_name}/status
Type: std_msgs/String

Content: Controller state ("idle", "moving", "error", etc.)
```

### Custom Publishing

For custom sensors, create your own publisher:

```python
from robo_infra.integrations.ros2 import is_ros2_available
from sensor_msgs.msg import Imu

class SensorNode:
    def __init__(self, node, sensor):
        self._sensor = sensor
        self._imu_pub = node.create_publisher(
            Imu,
            "/robot/imu",
            10,
        )
    
    def publish_imu(self):
        data = self._sensor.read()
        msg = Imu()
        msg.linear_acceleration.x = data.accel_x
        msg.linear_acceleration.y = data.accel_y
        msg.linear_acceleration.z = data.accel_z
        msg.angular_velocity.x = data.gyro_x
        msg.angular_velocity.y = data.gyro_y
        msg.angular_velocity.z = data.gyro_z
        self._imu_pub.publish(msg)
```

## Subscribing to Commands

### Velocity Commands

For mobile robots, subscribe to velocity commands:

```
Topic: /{controller_name}/cmd_vel
Type: geometry_msgs/Twist

Fields:
  linear:
    x: Forward/backward velocity
    y: Left/right velocity (holonomic)
    z: Up/down velocity (drones)
  angular:
    x: Roll rate
    y: Pitch rate
    z: Yaw rate (turn)
```

### Joint Commands

For manipulators, subscribe to joint commands:

```
Topic: /{controller_name}/joint_commands
Type: std_msgs/Float64MultiArray

Content: Target positions for each joint
```

### Custom Subscribers

```python
from geometry_msgs.msg import Twist

class MobileRobotNode:
    def __init__(self, node, controller):
        self._controller = controller
        node.create_subscription(
            Twist,
            "/robot/cmd_vel",
            self._cmd_vel_callback,
            10,
        )
    
    def _cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        self._controller.set_velocity(linear_vel, angular_vel)
```

## Message Types

### Standard Messages

robo-infra works with these ROS2 message types:

| Message | Package | Use Case |
|---------|---------|----------|
| `JointState` | sensor_msgs | Joint positions, velocities, efforts |
| `Twist` | geometry_msgs | Velocity commands |
| `Pose` | geometry_msgs | Position and orientation |
| `Point` | geometry_msgs | 3D position |
| `Quaternion` | geometry_msgs | Orientation |
| `Float64` | std_msgs | Single values |
| `Bool` | std_msgs | Boolean states |
| `String` | std_msgs | Text status |

### Mock Messages for Testing

When ROS2 is unavailable, mock messages are used:

```python
from robo_infra.integrations.ros2 import (
    MockJointState,
    MockTwist,
    MockPose,
    is_ros2_available,
    is_ros2_mock_mode,
)

# Check availability
if is_ros2_available():
    from sensor_msgs.msg import JointState
else:
    JointState = MockJointState

# Mock message usage
joint_state = MockJointState(
    name=["joint1", "joint2"],
    position=[0.0, 45.0],
    velocity=[0.0, 0.0],
    effort=[0.0, 0.0],
)
```

### Custom Messages

Define custom messages in your ROS2 package:

```python
# my_robot_msgs/msg/ArmStatus.msg
# Header header
# float64[] joint_temperatures
# bool[] joint_faults
# string error_message

from my_robot_msgs.msg import ArmStatus

class ArmNode:
    def __init__(self, node, controller):
        self._controller = controller
        self._status_pub = node.create_publisher(
            ArmStatus,
            "/arm/detailed_status",
            10,
        )
    
    def publish_status(self):
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_temperatures = [temp.read() for temp in self._temps]
        msg.joint_faults = [j.is_fault for j in self._controller.actuators]
        msg.error_message = self._controller.status().error or ""
        self._status_pub.publish(msg)
```

## Action Servers

### Motion Actions

Expose long-running motion as actions:

```python
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

class ArmActionNode:
    def __init__(self, node, controller):
        self._controller = controller
        self._action_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{controller.name}/follow_joint_trajectory",
            self._execute_trajectory,
        )
    
    async def _execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        
        for point in trajectory.points:
            # Check for cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            
            # Move to point
            await self._controller.move_to(
                {name: pos for name, pos in zip(
                    trajectory.joint_names,
                    point.positions
                )}
            )
            
            # Publish feedback
            feedback = FollowJointTrajectory.Feedback()
            feedback.actual = self._get_current_state()
            goal_handle.publish_feedback(feedback)
        
        goal_handle.succeed()
        return FollowJointTrajectory.Result()
```

### Gripper Actions

```python
from control_msgs.action import GripperCommand

class GripperActionNode:
    def __init__(self, node, gripper):
        self._gripper = gripper
        self._action_server = ActionServer(
            node,
            GripperCommand,
            "/gripper/gripper_command",
            self._execute_gripper,
        )
    
    async def _execute_gripper(self, goal_handle):
        command = goal_handle.request.command
        target_position = command.position
        max_effort = command.max_effort
        
        await self._gripper.move_to(
            position=target_position,
            force_limit=max_effort,
        )
        
        result = GripperCommand.Result()
        result.position = self._gripper.current_position
        result.stalled = self._gripper.is_stalled
        result.reached_goal = abs(
            self._gripper.current_position - target_position
        ) < 0.01
        
        goal_handle.succeed()
        return result
```

## Services

Controllers expose these services automatically:

### Home Service

```
Service: /{controller_name}/home
Type: std_srvs/Trigger

Request: (empty)
Response:
  success: bool
  message: string
```

```bash
ros2 service call /arm/home std_srvs/srv/Trigger
```

### Stop Service

```
Service: /{controller_name}/stop
Type: std_srvs/Trigger

Request: (empty)
Response:
  success: bool
  message: string
```

```bash
ros2 service call /arm/stop std_srvs/srv/Trigger
```

### Enable/Disable Service

```
Service: /{controller_name}/enable
Type: std_srvs/SetBool

Request:
  data: bool  # true=enable, false=disable
Response:
  success: bool
  message: string
```

```bash
# Enable
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"

# Disable
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: false}"
```

## Example: Complete ROS2 Robot Node

```python
#!/usr/bin/env python3
"""Complete ROS2 robot node example."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from robo_infra.controllers import JointGroup
from robo_infra.actuators import Servo, DCMotor
from robo_infra.drivers import PCA9685Driver
from robo_infra.integrations.ros2 import (
    controller_to_ros2_node,
    ROS2NodeConfig,
)


class RobotArmNode(Node):
    """Complete robot arm node with ROS2 interface."""
    
    def __init__(self):
        super().__init__("robot_arm")
        
        # Initialize hardware
        self._driver = PCA9685Driver()
        self._joints = [
            Servo("base", channel=0, driver=self._driver),
            Servo("shoulder", channel=1, driver=self._driver),
            Servo("elbow", channel=2, driver=self._driver),
            Servo("wrist", channel=3, driver=self._driver),
            Servo("gripper", channel=4, driver=self._driver),
        ]
        
        # Create controller
        self._arm = JointGroup("arm", joints=self._joints)
        
        # Publishers
        self._joint_state_pub = self.create_publisher(
            JointState,
            "/arm/joint_states",
            10,
        )
        
        # Subscribers
        self.create_subscription(
            JointState,
            "/arm/joint_commands",
            self._joint_command_callback,
            10,
        )
        
        # Services
        self.create_service(
            Trigger,
            "/arm/home",
            self._home_callback,
        )
        self.create_service(
            Trigger,
            "/arm/stop",
            self._stop_callback,
        )
        
        # Action server
        self._trajectory_action = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm/follow_joint_trajectory",
            self._execute_trajectory,
        )
        
        # Timer for status publishing
        self.create_timer(0.02, self._publish_joint_states)  # 50Hz
        
        self.get_logger().info("Robot arm node initialized")
    
    def _publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = [j.name for j in self._joints]
        msg.position = [j.current_value for j in self._joints]
        msg.velocity = [0.0] * len(self._joints)
        msg.effort = [0.0] * len(self._joints)
        self._joint_state_pub.publish(msg)
    
    def _joint_command_callback(self, msg):
        """Handle joint command messages."""
        for name, position in zip(msg.name, msg.position):
            for joint in self._joints:
                if joint.name == name:
                    joint.target = position
    
    def _home_callback(self, request, response):
        """Handle home service."""
        try:
            self._arm.home()
            response.success = True
            response.message = "Arm homed successfully"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def _stop_callback(self, request, response):
        """Handle stop service."""
        try:
            self._arm.stop()
            response.success = True
            response.message = "Arm stopped"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def _execute_trajectory(self, goal_handle):
        """Execute trajectory action."""
        trajectory = goal_handle.request.trajectory
        
        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            
            # Move to waypoint
            targets = dict(zip(
                trajectory.joint_names,
                point.positions,
            ))
            await self._arm.move_to(targets)
            
            # Feedback
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = trajectory.joint_names
            feedback.actual.positions = [j.current_value for j in self._joints]
            goal_handle.publish_feedback(feedback)
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result


def main():
    rclpy.init()
    node = RobotArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## Mock Mode for Testing

Enable mock mode for unit tests:

```python
import os
os.environ["ROS2_MOCK"] = "true"

from robo_infra.integrations.ros2 import (
    controller_to_ros2_node,
    is_ros2_mock_mode,
    MockROS2Node,
)

def test_ros2_node():
    assert is_ros2_mock_mode()
    
    arm = JointGroup("arm", joints=[...])
    node = controller_to_ros2_node(arm)
    
    # Access mock node
    mock_node = node.get_node()
    assert isinstance(mock_node, MockROS2Node)
    
    # Check publishers created
    publishers = mock_node.get_publishers()
    assert "arm/joint_states" in publishers
    
    # Check services created
    services = mock_node.get_services()
    assert "arm/home" in services
    
    # Simulate service call
    home_srv = services["arm/home"]
    response = home_srv.call({})
    assert response.success
```

## Launch Files

Generate launch files for your robot:

```python
# robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_robot",
            executable="arm_node",
            name="arm_controller",
            namespace="/robot",
            parameters=[{
                "publish_rate": 50.0,
                "use_sim_time": False,
            }],
            remappings=[
                ("/robot/arm/joint_states", "/joint_states"),
            ],
        ),
    ])
```

## Best Practices

### 1. Use Standard Messages

Prefer standard message types for compatibility:

```python
# ✅ Standard messages work with existing tools
from sensor_msgs.msg import JointState  # rviz, moveit compatible

# ⚠️ Custom messages require additional setup
from my_robot_msgs.msg import CustomJointState
```

### 2. Set QoS Appropriately

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For sensor data (lossy OK)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)

# For commands (must be reliable)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)
```

### 3. Handle ROS2 Unavailability

```python
from robo_infra.integrations.ros2 import is_ros2_available

if is_ros2_available():
    # Full ROS2 integration
    node = controller_to_ros2_node(arm)
else:
    # Standalone operation
    print("ROS2 not available, running standalone")
```

### 4. Use Namespaces

```python
# Organize topics with namespaces
node = controller_to_ros2_node(
    arm,
    namespace="/robot/manipulation",
)
# Creates: /robot/manipulation/arm/joint_states
```

## See Also

- [Controllers](controllers.md) - Controller types for ROS2 nodes
- [Observability](observability.md) - Monitoring ROS2 robots
- [Safety](safety.md) - Safety integration with ROS2
