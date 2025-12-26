# Physical AI Book

Welcome to the Physical AI Book, an accessible learning resource designed to demystify AI systems that interact with the physical world through robotics, sensors, and real-world applications.

## About the Book

This comprehensive resource covers the fundamentals of Physical AI - AI systems that interact directly with the physical world. Unlike traditional AI systems that operate primarily in digital spaces, Physical AI systems must perceive, reason about, and act upon real-world environments. This includes:

- Robots that navigate physical spaces
- Autonomous vehicles that respond to traffic conditions
- Smart sensors that monitor environmental conditions
- Industrial systems that control physical processes
- Assistive technologies that interact with humans in physical space

## Target Audience

This book is designed for beginner to intermediate learners who want to understand, implement, and experiment with AI systems that interact with the physical world. We assume you have basic programming knowledge but provide comprehensive introductions to specialized topics in Physical AI.

## Book Structure

The book is organized into chapters and lessons that build upon each other:

- **Chapter 1**: Introduction to Physical AI - Understanding the fundamentals of AI systems that interact with the physical world, including sensors, actuators, and real-world applications
  - Lesson 1.1: Foundations of Physical AI
  - Lesson 1.2: Sensors and Perception
  - Lesson 1.3: Actuators and Control

## Curriculum Structure

This book serves as an introduction to a comprehensive Physical AI curriculum that includes additional modules:

- **Chapter 1**: Introduction to Physical AI (Foundations, Sensors, Actuators)
- **Module 1**: The Robotic Nervous System (ROS 2) - Middleware for robot control, including ROS 2 Nodes, Topics, Services, and bridging Python Agents to ROS controllers
- **Module 2**: The Digital Twin (Gazebo & Unity) - Physics simulation and environment building, including simulating physics, gravity, collisions, and sensor simulation
- **Module 3**: The AI-Robot Brain (NVIDIA Isaac™) - Advanced perception and training using NVIDIA Isaac Sim and Isaac ROS for hardware-accelerated VSLAM and navigation
- **Module 4**: Vision-Language-Action (VLA) - The convergence of LLMs and Robotics, including voice-to-action systems and cognitive planning

Each module includes detailed lessons, hands-on examples, exercises, and expected outcomes to ensure comprehensive learning.

## Getting Started

To run this documentation site locally:

### Installation

```bash
yarn
```

### Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.

## Contributing

We welcome contributions to improve the Physical AI Book. Please see our contributing guidelines in the documentation.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
