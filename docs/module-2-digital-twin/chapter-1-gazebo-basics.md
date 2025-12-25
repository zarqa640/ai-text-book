---
sidebar_position: 1
---

# Chapter 1: Gazebo Basics

## Introduction to Gazebo Physics Simulation

Gazebo is a physics simulation environment that allows you to test and develop humanoid robots in a safe, virtual environment. It provides realistic physics simulation including gravity, collisions, and contact forces. Gazebo is essential for humanoid robotics development as it allows you to test complex behaviors without risk of damaging physical hardware.

## Installing Gazebo for Humanoid Robotics

First, install Gazebo with ROS 2 integration:

```bash
# For ROS 2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control ros-humble-gazebo-plugins

# Install Gazebo Fortress (recommended version)
sudo apt install gazebo-fortress libgazebo-dev
```

## Basic Gazebo World Setup for Humanoid Robots

Create a basic world file for humanoid robot testing:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include outdoor world with sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics engine -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>
```

## Creating a Humanoid Robot Model

Create a URDF model for your humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
</robot>
```

## Launching Gazebo with Your Robot

Create a launch file to spawn your humanoid robot in Gazebo:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description')

    # World file
    world = os.path.join(
        get_package_share_directory('robot_gazebo'),
        'worlds',
        'humanoid_world.sdf'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'world': world}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(
                os.path.join(pkg_robot_description, 'urdf', 'humanoid.urdf')
            ).read()
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## Gazebo Controllers for Humanoid Robots

To control your humanoid robot in simulation, you'll need ROS 2 controllers:

```yaml
# config/humanoid_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_arm_controller:
      type: position_controllers/JointGroupPositionController

    right_arm_controller:
      type: position_controllers/JointGroupPositionController

left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint

right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_joint
      - right_elbow_joint
```

## Testing Robot Behavior in Gazebo

Create a simple test to verify your robot works in simulation:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class HumanoidTestNode(Node):
    def __init__(self):
        super().__init__('humanoid_test_node')

        # Publishers for joint controllers
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/left_arm_controller/commands',
            10
        )

        self.right_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/right_arm_controller/commands',
            10
        )

        # Test timer
        self.timer = self.create_timer(2.0, self.test_movement)
        self.test_step = 0

    def test_movement(self):
        """Test different arm positions"""
        if self.test_step == 0:
            # Move left arm up
            msg = Float64MultiArray()
            msg.data = [1.0, 0.5]  # shoulder, elbow angles
            self.left_arm_pub.publish(msg)
            self.get_logger().info("Moving left arm up")
        elif self.test_step == 1:
            # Move right arm up
            msg = Float64MultiArray()
            msg.data = [1.0, 0.5]  # shoulder, elbow angles
            self.right_arm_pub.publish(msg)
            self.get_logger().info("Moving right arm up")
        elif self.test_step == 2:
            # Return to neutral position
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            self.left_arm_pub.publish(msg)
            self.right_arm_pub.publish(msg)
            self.get_logger().info("Returning to neutral position")

        self.test_step = (self.test_step + 1) % 3

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise: Launch Your Humanoid Robot in Gazebo

1. Create the URDF model for your humanoid robot based on the example above
2. Set up the world file with appropriate physics parameters
3. Create a launch file to spawn your robot in Gazebo
4. Implement basic controllers to move the robot's joints
5. Test the simulation by sending commands to move the robot's arms

---
This chapter introduced Gazebo simulation for humanoid robots. In the next chapter, we'll explore Unity integration.