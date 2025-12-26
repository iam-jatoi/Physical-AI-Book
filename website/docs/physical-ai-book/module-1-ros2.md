---
title: Module 1 - The Robotic Nervous System (ROS 2)
description: Middleware for robot control, including ROS 2 Nodes, Topics, Services, and bridging Python Agents to ROS controllers
sidebar_position: 6
last_updated: 2025-12-24
---

# Module 1: The Robotic Nervous System (ROS 2)

## Overview

ROS 2 (Robot Operating System 2) serves as the middleware for robot control, providing a communication framework that allows different parts of a robot system to work together seamlessly. This module covers the fundamental concepts and practical implementation of ROS 2 in Physical AI systems.

## Learning Objectives

By the end of this module, you will be able to:

- Understand the architecture and core concepts of ROS 2
- Implement ROS 2 Nodes, Topics, and Services
- Bridge Python Agents to ROS controllers using rclpy
- Create and manage ROS 2 packages with Python
- Configure launch files and parameter management

## Prerequisites

Before starting this module, you should have:

- Basic understanding of Physical AI concepts (covered in Chapter 1)
- Familiarity with Python programming
- Basic knowledge of distributed systems concepts

## Module Content

### 1. ROS 2 Architecture and Core Concepts

ROS 2 is built on the Data Distribution Service (DDS) standard, which provides a middleware layer for communication between different parts of a robot system. The core concepts include:

- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication

### 2. ROS 2 Nodes, Topics, and Services

#### Nodes
Nodes are the fundamental building blocks of ROS 2 applications. Each node performs a specific task and communicates with other nodes through topics, services, or actions.

#### Topics
Topics enable asynchronous communication between nodes using a publish-subscribe model. Multiple nodes can publish to or subscribe to the same topic.

#### Services
Services provide synchronous request-response communication between nodes, useful for operations that require a guaranteed response.

### 3. Bridging Python Agents to ROS Controllers using rclpy

The `rclpy` library provides Python bindings for ROS 2, allowing Python-based AI agents to communicate with ROS-based robot controllers.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Understanding URDF (Unified Robot Description Format) for Humanoids

URDF is an XML format for representing a robot model. For humanoid robots, URDF describes the physical structure, kinematic chains, and visual/collision properties.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.8"/>
      </geometry>
    </visual>
  </link>

  <!-- Additional links and joints for arms, legs, head -->
</robot>
```

## Hands-On Example: Creating a Simple ROS 2 Publisher

Let's create a simple ROS 2 publisher that simulates sending sensor data from a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import random

class HumanoidSensorPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_publisher')
        
        # Create publisher for joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish data at regular intervals
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.publish_joint_states)
        
        # Simulate joint names for a simple humanoid
        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist'
        ]
        
        self.time = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Simulate joint positions with some oscillation
        positions = []
        for i, name in enumerate(self.joint_names):
            # Different oscillation patterns for different joints
            if 'hip' in name:
                pos = 0.2 * math.sin(self.time + i)
            elif 'knee' in name:
                pos = 0.15 * math.sin(self.time * 1.2 + i)
            elif 'ankle' in name:
                pos = 0.1 * math.sin(self.time * 0.8 + i)
            elif 'shoulder' in name:
                pos = 0.3 * math.sin(self.time * 0.5 + i)
            elif 'elbow' in name:
                pos = 0.25 * math.sin(self.time * 0.7 + i)
            else:  # wrist
                pos = 0.2 * math.sin(self.time * 0.9 + i)
            
            positions.append(pos)
        
        msg.position = positions
        
        # Add some velocity and effort data
        velocities = [0.1 * random.uniform(-1, 1) for _ in positions]
        efforts = [0.05 * random.uniform(-1, 1) for _ in positions]
        
        msg.velocity = velocities
        msg.effort = efforts
        
        self.joint_publisher.publish(msg)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = HumanoidSensorPublisher()
    
    print("Humanoid Sensor Publisher started. Publishing joint states...")
    print("Press Ctrl+C to stop.")
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Exercise**: Modify the joint oscillation patterns to simulate a walking motion for the humanoid robot. Adjust the phase and amplitude of different joints to create a more realistic gait pattern.

**Expected Outcome**: After running the code, you should see the joint states being published at 10Hz, simulating sensor data from a humanoid robot with oscillating joints.

## Next Steps

After completing this module, continue with Module 2: The Digital Twin (Gazebo & Unity) to learn about physics simulation and environment building.

[Table of Contents](../intro.md) | [Previous Lesson](../chapter-1/lesson-3.md) | [Next Module: The Digital Twin](../module-2-gazebo-unity.md)