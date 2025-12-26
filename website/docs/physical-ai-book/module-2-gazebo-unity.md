---
title: Module 2 - The Digital Twin (Gazebo & Unity)
description: Physics simulation and environment building, including simulating physics, gravity, collisions, and sensor simulation
sidebar_position: 7
last_updated: 2025-12-24
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Overview

The Digital Twin concept involves creating a virtual replica of a physical system. In Physical AI, this means simulating robots and their environments with high fidelity physics, enabling safe testing and training before deployment to real hardware. This module covers Gazebo for physics simulation and Unity for high-fidelity rendering and human-robot interaction.

## Learning Objectives

By the end of this module, you will be able to:

- Set up and configure Gazebo simulation environments
- Simulate physics, gravity, and collisions in Gazebo
- Create high-fidelity rendering and human-robot interaction in Unity
- Simulate various sensors: LiDAR, Depth Cameras, and IMUs
- Transfer learned behaviors from simulation to real robots (Sim-to-Real)

## Prerequisites

Before starting this module, you should have:

- Understanding of Physical AI concepts (covered in Chapter 1)
- Knowledge of ROS 2 fundamentals (covered in Module 1)
- Basic understanding of physics concepts (force, motion, gravity)

## Module Content

### 1. Gazebo Simulation Environment Setup

Gazebo is a physics-based simulation environment that allows you to test robot algorithms in realistic scenarios. It provides:

- High-fidelity physics simulation
- Sensor simulation
- Realistic rendering
- Multiple physics engines (ODE, Bullet, Simbody)

#### Installing Gazebo with ROS 2

```bash
# Install Gazebo Harmonic (or latest version)
sudo apt install ros-humble-gazebo-*

# Source ROS 2 and Gazebo
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
```

### 2. Simulating Physics, Gravity, and Collisions in Gazebo

Gazebo simulates realistic physics including gravity, friction, and collisions. The physics engine calculates the motion of objects based on forces applied to them.

#### Basic World File Example

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Define a simple box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.083</iyy>
            <iyz>0.0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 3. High-Fidelity Rendering and Human-Robot Interaction in Unity

Unity provides high-fidelity rendering capabilities that complement Gazebo's physics simulation. It's particularly useful for:

- Creating realistic visual environments
- Simulating camera sensors
- Designing human-robot interaction scenarios
- VR/AR applications for robot teleoperation

### 4. Simulating Sensors: LiDAR, Depth Cameras, and IMUs

#### LiDAR Sensor Simulation

LiDAR sensors provide 2D or 3D distance measurements. In Gazebo, you can simulate LiDAR using the libgazebo_ros_ray.so plugin:

```xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray.so">
    <ros>
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

#### Depth Camera Simulation

Depth cameras provide both RGB and depth information:

```xml
<sensor name="depth_camera" type="depth">
  <pose>0.1 0 0.1 0 0 0</pose>
  <camera name="depth_cam">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>15</update_rate>
  <visualize>true</visualize>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
      <remapping>image_raw:=image_color</remapping>
      <remapping>camera_info:=camera_info</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_camera_frame</frame_name>
  </plugin>
</sensor>
```

#### IMU Simulation

IMUs measure acceleration and angular velocity:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>false</visualize>
  <topic>__default_topic__</topic>
  <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
    <ros>
      <namespace>/imu</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
  <pose>0 0 0 0 0 0</pose>
</sensor>
```

## Hands-On Example: Creating a Simple Gazebo World with Sensors

Let's create a simple Gazebo world with a robot that has multiple sensors:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="sensor_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
      </visual>
    </link>

    <!-- LiDAR sensor -->
    <sensor name="lidar" type="ray">
      <pose>0.1 0 0.1 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray.so">
        <ros>
          <namespace>/robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>

    <!-- IMU sensor -->
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <topic>__default_topic__</topic>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>/robot</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
      <pose>0 0 0 0 0 0</pose>
    </sensor>
  </model>
</sdf>
```

## Python Code Example: Processing Sensor Data from Gazebo

Here's a Python example that subscribes to the sensor data from our simulated robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Subscribe to LiDAR data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.lidar_callback,
            10)
        
        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10)
        
        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for processing loop
        self.timer = self.create_timer(0.1, self.process_sensors)
        
        # Store sensor data
        self.lidar_data = None
        self.imu_data = None
        
        self.get_logger().info('Sensor Processor initialized')

    def lidar_callback(self, msg):
        self.lidar_data = msg.ranges  # Store the LiDAR ranges

    def imu_callback(self, msg):
        self.imu_data = {
            'orientation': (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            'angular_velocity': (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            'linear_acceleration': (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        }

    def process_sensors(self):
        if self.lidar_data is not None:
            # Process LiDAR data to detect obstacles
            # Convert to numpy array for easier processing
            ranges = np.array(self.lidar_data)
            
            # Remove invalid readings (inf or nan)
            valid_ranges = ranges[np.isfinite(ranges)]
            
            if len(valid_ranges) > 0:
                # Find the closest obstacle
                min_distance = np.min(valid_ranges)
                
                # Create a movement command based on sensor data
                cmd = Twist()
                
                if min_distance < 1.0:  # If obstacle is closer than 1 meter
                    # Stop and turn
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5  # Turn right
                    self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m, turning')
                else:
                    # Move forward
                    cmd.linear.x = 0.5
                    cmd.angular.z = 0.0
                    self.get_logger().info(f'Moving forward, closest obstacle at {min_distance:.2f}m')
                
                # Publish the command
                self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()
    
    print("Sensor Processor started. Processing LiDAR and IMU data...")
    print("The robot will move forward and turn when obstacles are detected.")
    
    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Exercise**: Modify the sensor processing code to implement a more sophisticated navigation algorithm, such as a wall-following behavior or a simple mapping algorithm that builds a map of the environment based on LiDAR readings.

**Expected Outcome**: The robot should navigate through the simulated environment, using its LiDAR sensor to detect obstacles and adjust its path accordingly.

## Next Steps

After completing this module, continue with Module 3: The AI-Robot Brain (NVIDIA Isaac™) to learn about advanced perception and training using NVIDIA Isaac Sim and Isaac ROS.

[Table of Contents](../intro.md) | [Previous Module: The Robotic Nervous System](../module-1-ros2.md) | [Next Module: The AI-Robot Brain](../module-3-isaac.md)