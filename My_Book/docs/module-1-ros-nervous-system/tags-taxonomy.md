# Tags Taxonomy for ROS 2 Educational Module

## Purpose
This document defines the standardized tags taxonomy for organizing documentation content in the ROS 2 educational module.

## Tag Categories

### Core ROS 2 Concepts
- `ros2`: General ROS 2 concepts and fundamentals
- `dds`: Data Distribution Service concepts
- `middleware`: Middleware concepts and architecture
- `nervous-system`: ROS 2 as the nervous system for robots

### Communication Patterns
- `nodes`: ROS 2 nodes and node management
- `topics`: Topic-based communication
- `services`: Service-based communication
- `actions`: Action-based communication
- `publish-subscribe`: Publish-subscribe pattern
- `client-server`: Client-server pattern

### Programming and Integration
- `rclpy`: Python ROS client library
- `rclcpp`: C++ ROS client library
- `python`: Python-specific content
- `cpp`: C++-specific content
- `integration`: System integration concepts

### Robot Description and Modeling
- `urdf`: Unified Robot Description Format
- `xacro`: XML Macros for URDF
- `robot-modeling`: Robot modeling concepts
- `simulation`: Simulation readiness and concepts
- `humanoid-robotics`: Humanoid-specific content

### Educational Levels
- `beginner`: Content for beginners
- `intermediate`: Content for intermediate learners
- `advanced`: Advanced concepts
- `tutorial`: Step-by-step tutorials
- `reference`: Reference material

### Application Domains
- `physical-ai`: Physical AI applications
- `humanoid-robotics`: Humanoid robotics applications
- `autonomous-systems`: Autonomous systems
- `multi-robot`: Multi-robot systems

## Tagging Guidelines

1. Use 3-5 tags per document
2. Always include at least one core concept tag (e.g., `ros2`, `nodes`, `urdf`)
3. Include an educational level tag when appropriate
4. Use specific tags rather than general ones when possible
5. Combine domain-specific tags with core concept tags (e.g., `urdf` + `humanoid-robotics`)

## Examples

### ROS 2 Basics Chapter
Tags: `ros2`, `dds`, `middleware`, `beginner`, `introduction`

### Nodes/Topics/Services Chapter
Tags: `nodes`, `topics`, `services`, `rclpy`, `intermediate`, `communication`

### URDF & Python–ROS Integration Chapter
Tags: `urdf`, `rclpy`, `python`, `robot-modeling`, `humanoid-robotics`, `intermediate`