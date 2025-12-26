---
title: Physical AI Curriculum Overview
description: Overview of the complete Physical AI curriculum including all modules
sidebar_position: 5
last_updated: 2025-12-24
---

# Physical AI Curriculum Overview

This curriculum provides a comprehensive introduction to Physical AI - AI systems that interact with the physical world through robotics, sensors, and real-world applications.

## Course Structure

The curriculum is organized into multiple modules that build upon each other to provide a complete understanding of Physical AI systems:

### Module 1: The Robotic Nervous System (ROS 2)
- **Focus**: Middleware for robot control
- **Topics Covered**:
  - ROS 2 Nodes, Topics, and Services
  - Bridging Python Agents to ROS controllers using rclpy
  - Understanding URDF (Unified Robot Description Format) for humanoids

### Module 2: The Digital Twin (Gazebo & Unity)
- **Focus**: Physics simulation and environment building
- **Topics Covered**:
  - Simulating physics, gravity, and collisions in Gazebo
  - High-fidelity rendering and human-robot interaction in Unity
  - Simulating sensors: LiDAR, Depth Cameras, and IMUs

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- **Focus**: Advanced perception and training
- **Topics Covered**:
  - NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation
  - Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation
  - Nav2: Path planning for bipedal humanoid movement

### Module 4: Vision-Language-Action (VLA)
- **Focus**: The convergence of LLMs and Robotics
- **Topics Covered**:
  - Voice-to-Action: Using OpenAI Whisper for voice commands
  - Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions
  - Capstone Project: The Autonomous Humanoid

## Learning Outcomes

Upon completing this curriculum, students will be able to:

1. Understand Physical AI principles and embodied intelligence
2. Master ROS 2 (Robot Operating System) for robotic control
3. Simulate robots with Gazebo and Unity
4. Develop with NVIDIA Isaac AI robot platform
5. Design humanoid robots for natural interactions
6. Integrate GPT models for conversational robotics

## Why Physical AI Matters

Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments. This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space.

## Weekly Breakdown

- **Weeks 1-2**: Introduction to Physical AI
  - Foundations of Physical AI and embodied intelligence
  - From digital AI to robots that understand physical laws
  - Overview of humanoid robotics landscape
  - Sensor systems: LIDAR, cameras, IMUs, force/torque sensors

- **Weeks 3-5**: ROS 2 Fundamentals
  - ROS 2 architecture and core concepts
  - Nodes, topics, services, and actions
  - Building ROS 2 packages with Python
  - Launch files and parameter management

- **Weeks 6-7**: Robot Simulation with Gazebo
  - Gazebo simulation environment setup
  - URDF and SDF robot description formats
  - Physics simulation and sensor simulation
  - Introduction to Unity for robot visualization

- **Weeks 8-10**: NVIDIA Isaac Platform
  - NVIDIA Isaac SDK and Isaac Sim
  - AI-powered perception and manipulation
  - Reinforcement learning for robot control
  - Sim-to-real transfer techniques

- **Weeks 11-12**: Humanoid Robot Development
  - Humanoid robot kinematics and dynamics
  - Bipedal locomotion and balance control
  - Manipulation and grasping with humanoid hands
  - Natural human-robot interaction design

- **Week 13**: Conversational Robotics
  - Integrating GPT models for conversational AI in robots
  - Speech recognition and natural language understanding
  - Multi-modal interaction: speech, gesture, vision

## Hardware Requirements

This course is technically demanding. It sits at the intersection of three heavy computational loads: Physics Simulation (Isaac Sim/Gazebo), Visual Perception (SLAM/Computer Vision), and Generative AI (LLMs/VLA).

### The "Digital Twin" Workstation (Required per Student)
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher
- **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9
- **RAM**: 64 GB DDR5 (32 GB is the absolute minimum)
- **OS**: Ubuntu 22.04 LTS

### The "Physical AI" Edge Kit
- **The Brain**: NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
- **The Eyes (Vision)**: Intel RealSense D435i or D455
- **The Inner Ear (Balance)**: Generic USB IMU (BNO055)
- **Voice Interface**: USB Microphone/Speaker array (e.g., ReSpeaker)

## Assessments

- ROS 2 package development project
- Gazebo simulation implementation
- Isaac-based perception pipeline
- Capstone: Simulated humanoid robot with conversational AI