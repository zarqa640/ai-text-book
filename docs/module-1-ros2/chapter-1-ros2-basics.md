---
sidebar_position: 1
---

# Chapter 1: ROS 2 Basics

## What is ROS 2 and Why Humanoid Robots Need It

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. Think of it as the "nervous system" for robots - it handles communication between different parts of the robot, manages hardware interfaces, and provides tools for development and debugging.

For humanoid robots, ROS 2 is essential because it allows different subsystems (like vision, movement, and decision-making) to communicate seamlessly. Whether your robot is processing camera data, moving its arms, or planning a path, ROS 2 provides the infrastructure for these components to work together.

## Nodes, Topics, and Services - The Communication Framework

### Nodes
A node is a process that performs computation. In a humanoid robot, you might have:
- A node for controlling the robot's joints
- A node for processing camera images
- A node for making high-level decisions

### Topics
Topics enable asynchronous communication through a publish/subscribe model. For example:
- A camera node publishes images to the `/camera/image_raw` topic
- A perception node subscribes to this topic to process the images

### Services
Services enable synchronous request/response communication. For example:
- A node requests the robot to move its arm to a specific position
- The arm controller responds with success or failure

## Basic CLI Commands

Here are essential ROS 2 command-line tools:

```bash
# List all active nodes
ros2 node list

# List all active topics
ros2 topic list

# Echo messages from a topic (e.g., camera feed)
ros2 topic echo /camera/image_raw

# List all available services
ros2 service list
```

## Exercise: Explore Your Robot's ROS 2 System

1. Launch your humanoid robot simulation
2. Use the CLI commands above to explore the running system
3. Identify at least 3 nodes and 2 topics in your robot's system

---

This chapter introduced the fundamental concepts of ROS 2. In the next chapter, we'll explore how to connect AI agents to ROS 2 using Python.