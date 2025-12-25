---
sidebar_position: 3
---

# Chapter 3: URDF for Humanoids

## What is URDF?

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. For humanoid robots, URDF defines:
- Physical structure (links)
- Joints connecting the links
- Visual and collision properties
- Inertial properties

## Basic URDF Structure for Humanoids

Here's a simplified URDF for a basic humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting head to base -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Visualizing URDF Models

### In RViz
1. Launch RViz: `ros2 run rviz2 rviz2`
2. Add a RobotModel display
3. Set the Robot Description parameter to your URDF parameter name

### In Gazebo
1. Launch Gazebo with your URDF: `ros2 launch gazebo_ros spawn_entity.py -entity robot -file path/to/your/robot.urdf`
2. The robot will appear in the simulation environment

## Exercise: Create and Visualize Your Humanoid Model

1. Create a URDF file for a simple humanoid robot with at least 3 links and 2 joints
2. Launch RViz and visualize your robot model
3. Try adding more joints to create a more complete humanoid structure

---

This chapter covered URDF for defining humanoid robot structure. This completes Module 1 of the Physical AI & Humanoid Robotics book.