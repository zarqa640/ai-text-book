---
sidebar_position: 2
---

# Chapter 2: Python Agents with rclpy

## Connecting AI Agents to ROS 2

The rclpy library is the Python client library for ROS 2. It allows Python programs to interface with the ROS 2 communication system, making it perfect for connecting AI agents to robot control systems.

## Creating Publishers and Subscribers

### Publisher Example
Here's how to create a publisher that sends commands to move a humanoid joint:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(Float64, 'joint_position', 10)

    def publish_command(self, position):
        msg = Float64()
        msg.data = position
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint position: {position}')

def main():
    rclpy.init()
    publisher = JointCommandPublisher()

    # Send a command to move the joint
    publisher.publish_command(1.5)  # Move to 1.5 radians

    rclpy.spin_once(publisher, timeout_sec=1)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example
Here's how to create a subscriber that receives sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position}')

def main():
    rclpy.init()
    subscriber = JointStateSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simple Controller Example

Here's a simple controller that moves a humanoid joint to a target position:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class SimpleJointController(Node):
    def __init__(self):
        super().__init__('simple_joint_controller')
        self.command_publisher = self.create_publisher(Float64, 'target_joint_position', 10)
        self.state_subscriber = self.create_subscription(JointState, 'joint_states', self.state_callback, 10)
        self.current_position = 0.0
        self.target_position = 0.0

    def state_callback(self, msg):
        # Update current position (simplified - in real usage, you'd match joint names)
        if len(msg.position) > 0:
            self.current_position = msg.position[0]

    def move_to_position(self, target):
        self.target_position = target
        cmd_msg = Float64()
        cmd_msg.data = target
        self.command_publisher.publish(cmd_msg)

def main():
    rclpy.init()
    controller = SimpleJointController()

    # Move joint to 2.0 radians
    controller.move_to_position(2.0)

    # Run for a few seconds to allow movement
    for i in range(10):
        rclpy.spin_once(controller, timeout_sec=0.5)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise: Create Your Own Controller

1. Create a Python script that publishes commands to move a humanoid joint
2. Add a subscriber to monitor the joint's current position
3. Implement a simple control loop that moves the joint to a target position

---

This chapter showed how to connect Python AI agents to ROS 2 using rclpy. In the next chapter, we'll explore URDF for defining humanoid robot structure.