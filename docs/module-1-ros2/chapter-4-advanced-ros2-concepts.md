---
sidebar_position: 4
---

# Chapter 4: Advanced ROS 2 Concepts for Humanoid Robots

## Action Servers and Clients - Long-Running Tasks

In humanoid robotics, many operations take a significant amount of time to complete, such as walking to a location, grasping an object, or performing a complex manipulation task. For these scenarios, ROS 2 provides **Actions** - a communication pattern for long-running tasks with feedback.

### Understanding Actions

Actions are ideal for humanoid robot behaviors because they provide:
- **Goal**: What you want the robot to do
- **Feedback**: Continuous updates on progress
- **Result**: Final outcome of the task

### Action Structure for Humanoid Robots

For a humanoid robot walking action:
- **Goal**: Target position and orientation
- **Feedback**: Current position, step count, estimated time remaining
- **Result**: Success/failure status, final position

```bash
# Example: Sending a navigation goal to a humanoid robot
ros2 action send_goal /humanoid/walk_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    position: {x: 1.0, y: 2.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## Lifecycle Nodes - Managing Robot States

Humanoid robots have complex state management requirements. Lifecycle nodes provide a structured way to manage different operational states:

### State Transitions
- **Unconfigured** → **Inactive** → **Active** → **Finalized**
- Each state has specific behaviors and resource management

### Practical Example
```python
# Lifecycle node for humanoid robot controller
from lifecycle_msgs.msg import Transition

class HumanoidControllerLifecycle(LifecycleNode):
    def on_configure(self, state):
        # Initialize hardware interfaces
        self.initialize_joints()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        # Start control loops
        self.start_control_loops()
        return TransitionCallbackReturn.SUCCESS
```

## Parameters and Dynamic Reconfiguration

Humanoid robots often need to adjust parameters during operation, such as gait parameters, balance thresholds, or safety limits.

### Parameter Management
```bash
# Set a parameter for humanoid gait
ros2 param set /humanoid_locomotion step_height 0.05

# List all parameters for a node
ros2 param list /humanoid_controller
```

## Quality of Service (QoS) for Real-time Control

For humanoid robots, timing is critical. QoS profiles ensure messages are delivered with appropriate reliability and latency:

### QoS Settings for Different Data Types
- **Joint commands**: Reliable delivery, low latency
- **Sensor data**: Best effort for high-frequency streams
- **Safety messages**: Reliable with high durability

```python
# QoS for critical humanoid control commands
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

control_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## Multi-Robot Communication

Advanced humanoid systems may involve coordination between multiple robots or robot components:

### DDS Configuration for Multi-Robot Systems
- Partition configuration for team coordination
- Discovery management for large robot teams
- Security considerations for multi-robot communication

## Exercise: Implement an Action Server for Humanoid Arm Control

1. Create an action definition for humanoid arm movement
2. Implement an action server that controls the robot's arm
3. Add feedback to report joint positions during movement
4. Test the action client to command arm movements

---

This chapter covered advanced ROS 2 concepts essential for humanoid robot development. The next module will explore digital twins and simulation environments for humanoid robots.