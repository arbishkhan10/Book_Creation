---
title: ROS 2 Basics
description: Introduction to ROS 2 fundamentals, its importance for humanoid robotics, and DDS concepts
tags: [ros2, basics, dds, middleware, beginner, introduction, physical-ai, humanoid-robotics]
---

# ROS 2 Basics

## Overview

This chapter introduces the fundamental concepts of ROS 2 (Robot Operating System 2), explaining what it is, why it matters for humanoid robotics, and the underlying DDS (Data Distribution Service) concepts. ROS 2 serves as the middleware "nervous system" that enables different components of a robotic system to communicate and coordinate effectively.

## Learning Objectives

By the end of this chapter, you should be able to:
- Understand what ROS 2 is and its role in robotics
- Explain why ROS 2 is important for humanoid robotics
- Describe the key DDS concepts underlying ROS 2
- Identify the core components and architecture of ROS 2

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. Despite its name, it's not an actual operating system but rather a collection of tools, libraries, and conventions that help developers create complex robotic systems more easily.

Think of ROS 2 as the "nervous system" of a robot - it helps different parts of the robot communicate with each other, just like how your nervous system helps different parts of your body communicate.

ROS 2 is designed for:

- **Production environments**: With improved security, reliability, and real-time capabilities
- **Industry applications**: Supporting commercial and enterprise use cases
- **Distributed systems**: Enabling communication across multiple machines and networks

### Analogy: A Restaurant Kitchen

Imagine a restaurant kitchen as a robotic system:
- The **order system** (a ROS node) sends orders to different stations
- The **chef station** (another node) receives orders and sends completion messages
- The **dishwasher** (another node) communicates when dishes are clean
- The **waiter** (another node) coordinates between all these components

In this restaurant, ROS 2 would be like the communication system that allows all these different "workers" to coordinate effectively without needing to know exactly who they're communicating with.

### Key Characteristics

ROS 2 is built on a **data-centric** approach, meaning it focuses on the data flowing between different components rather than just the components themselves. This approach enables:

- **Decoupled development**: Components can be developed independently
- **Flexible deployment**: Components can run on different machines
- **Robust communication**: Reliable data exchange even in complex systems

### Example: Simple ROS 2 Node Structure

Here's a basic example of a ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

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

This example demonstrates the basic structure of a ROS 2 node that publishes messages to a topic.

## Why ROS 2 Matters for Humanoid Robotics

Humanoid robots present unique challenges that make ROS 2 particularly valuable:

### Complexity Management
Humanoid robots typically have many subsystems (sensors, actuators, perception, planning, control) that need to work together seamlessly. ROS 2 provides:

- **Modular architecture**: Each subsystem can be developed as a separate node
- **Standardized interfaces**: Common message formats for communication
- **Scalable communication**: Handles the high volume of data in humanoid systems

### Real-time Requirements
Humanoid robots often require real-time performance for stability and safety. ROS 2 supports:

- **Real-time capable systems**: Integration with real-time operating systems
- **Deterministic communication**: Predictable timing for critical functions
- **Quality of Service (QoS)**: Configurable reliability and performance settings

### Research and Development
The humanoid robotics community benefits from ROS 2's:

- **Extensive ecosystem**: Large collection of packages and tools
- **Community support**: Active development and documentation
- **Cross-platform compatibility**: Works across different hardware and operating systems

## DDS Concepts

ROS 2 is built on DDS (Data Distribution Service), a middleware standard for real-time systems. Understanding DDS concepts is crucial for mastering ROS 2.

### Data-Centric Architecture

Think of DDS like a bulletin board system in a community center:

- **Data-centric**: Instead of sending messages directly to specific people, you post information on a bulletin board
- **Publish-Subscribe**: Anyone interested in a particular topic (like "community events") can check the appropriate bulletin board
- **Decentralized**: No central authority controls who posts or reads; the system works based on shared topics

This approach means that if a robot has sensor data to share, it doesn't need to know which other parts of the robot need that data. It just "posts" the data to a specific topic, and any component interested in that topic can "read" it.

### Quality of Service (QoS)

QoS policies in DDS are like different types of mail delivery:

- **Reliability**: Like choosing between regular mail (best-effort) and certified mail (reliable)
- **Durability**: Like deciding whether to keep old announcements posted or remove them after a certain time
- **History**: Like determining how many past announcements to keep available
- **Deadline**: Like setting expectations for how quickly responses are needed
- **Liveliness**: Like checking if the post office is still open and accepting mail

DDS provides QoS policies that define how data is communicated:

- **Reliability**: Reliable (all data delivered) vs. Best-effort (data may be lost)
- **Durability**: Keep data for late-joining subscribers vs. Volatile
- **History**: How much data to keep in the system
- **Deadline**: Expected update frequency
- **Liveliness**: How to detect if publishers/subscribers are active

### DDS Entities

The core DDS entities include:

- **Domain**: A communication space where participants communicate
- **Participant**: An application participating in a domain
- **Topic**: A named data channel for specific data types
- **Publisher**: An entity that sends data
- **Subscriber**: An entity that receives data
- **DataWriter**: Interface for publishing data
- **DataReader**: Interface for subscribing to data

### Example: DDS Communication Pattern

In DDS, communication follows a publish-subscribe pattern:

```
Publisher (DataWriter) → Network → Subscriber (DataReader)
     ↓                     ↓           ↓
  Topic A              Domain     Topic A
```

This pattern enables decoupled communication where publishers don't need to know about specific subscribers, and subscribers don't need to know about specific publishers. This is fundamental to ROS 2's communication model.

## Core Components of ROS 2

### Nodes
Nodes are the fundamental execution units in ROS 2. Each node:
- Performs a specific function within the robotic system
- Communicates with other nodes through topics, services, or actions
- Has a unique name within the ROS graph
- Can be written in different programming languages (C++, Python, etc.)

### Communication Primitives

ROS 2 provides three main communication patterns:

1. **Topics** (publish-subscribe): Asynchronous, one-way communication
2. **Services** (request-response): Synchronous, two-way communication
3. **Actions** (goal-result-feedback): Asynchronous with feedback for long-running tasks

### Packages and Build System

ROS 2 uses ament as its build system, which:
- Supports multiple programming languages
- Manages dependencies between packages
- Handles installation and distribution
- Integrates with standard build tools

## Summary

ROS 2 provides a robust middleware framework for developing complex robotic systems, particularly valuable for humanoid robotics due to its ability to manage complexity, support real-time requirements, and facilitate research and development. Built on DDS, it offers data-centric communication with configurable Quality of Service policies that make it suitable for both research and production environments.

Understanding these fundamentals provides the foundation for exploring more advanced ROS 2 concepts in subsequent chapters.

## Additional Resources

- [Official ROS 2 Documentation](https://docs.ros.org/en/rolling/)
- [ROS 2 Design Documents](https://design.ros2.org/)
- [DDS Foundation](https://www.dds-foundation.org/)

## Related Concepts

For more information on how these core concepts are implemented in practice, see:
- [Communication Patterns (Nodes/Topics/Services)](./nodes-topics-services): Learn how to implement the publish-subscribe and request-response patterns discussed in this chapter
- [Robot Modeling with URDF](./urdf-python-ros-integration): Understand how to describe robot structure using URDF, which is essential for complex robotic systems

## References

[^1]: ROS 2 Documentation. (2024). *About ROS 2*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-ROS-2/
[^2]: ROS 2 Documentation. (2024). *Middleware Implementation*. Retrieved from https://docs.ros.org/en/rolling/Concepts/Middleware/Overview/
[^3]: ROS 2 Documentation. (2024). *DDS Quality of Service*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings/
[^4]: DDS Foundation. (2024). *DDS Primer*. Retrieved from https://www.dds-foundation.org/dds-primer/

---

**Page Feedback**

Was this chapter helpful? [Yes/No] [Provide feedback]

[Back to Module Overview](../intro) | [Next Chapter: Nodes/Topics/Services](./nodes-topics-services)