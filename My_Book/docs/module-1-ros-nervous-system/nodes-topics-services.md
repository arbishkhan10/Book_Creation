---
title: Nodes/Topics/Services
description: Understanding ROS 2 communication patterns including Nodes, Topics, Services, and rclpy-based agent ↔ controller flow
tags: [ros2, nodes, topics, services, communication, rclpy, python, intermediate]
---

# Nodes/Topics/Services

## Overview

This chapter explores the core communication patterns in ROS 2: Nodes, Topics, and Services. These communication primitives form the foundation of how different components in a robotic system interact. We'll also cover practical examples using rclpy (Python ROS client library) to demonstrate agent ↔ controller flow.

## Learning Objectives

By the end of this chapter, you should be able to:
- Explain the roles and characteristics of ROS 2 nodes
- Implement topic-based publish-subscribe communication
- Create and use service-based request-response communication
- Develop practical examples using rclpy
- Understand agent ↔ controller communication patterns

## Understanding ROS 2 Nodes

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system, and they:

- Perform specific functions within the robotic system
- Communicate with other nodes through topics, services, or actions
- Have a unique name within the ROS graph
- Can be written in different programming languages (C++, Python, etc.)

### Node Lifecycle

ROS 2 nodes have a well-defined lifecycle that allows for more sophisticated state management than ROS 1:

- **Unconfigured**: Initial state after creation
- **Inactive**: Node is configured but not yet active
- **Active**: Node is running and performing its function
- **Finalized**: Node is shutting down

### Creating a Node with rclpy

Here's an example of a simple ROS 2 node using rclpy:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    # Spin the node so the callback function is called
    rclpy.spin(minimal_node)

    # Destroy the node explicitly
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topic-Based Communication (Publish-Subscribe)

Topics enable asynchronous, one-way communication using a publish-subscribe pattern. This is ideal for:

- Sensor data streams (camera images, LIDAR scans, IMU data)
- Status updates (battery level, system status)
- Continuous data streams that multiple nodes might need

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service-Based Communication (Request-Response)

Services provide synchronous, two-way communication using a request-response pattern. This is ideal for:

- Actions that require confirmation (moving to a position, taking a picture)
- Querying information (current position, system status)
- Operations with clear start and end states

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Agent ↔ Controller Communication Patterns

In humanoid robotics, communication patterns between agents (decision-making components) and controllers (actuation components) are crucial for coordinated behavior.

### Hierarchical Control Pattern

```
High-Level Agent (Planning)
         ↓ (goals, commands)
Low-Level Controller (Execution)
         ↓ (feedback, status)
High-Level Agent
```

### Example: Movement Command Pattern

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MovementAgent(Node):
    """
    An agent that sends movement commands to a controller
    """
    def __init__(self):
        super().__init__('movement_agent')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_subscriber = self.create_subscription(
            String,
            'movement_status',
            self.status_callback,
            10)
        self.timer = self.create_timer(1.0, self.send_command)
        self.command_counter = 0

    def send_command(self):
        msg = Twist()
        # Send a simple movement command
        msg.linear.x = 1.0  # Move forward
        msg.angular.z = 0.5  # Turn slightly
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent movement command #{self.command_counter}')
        self.command_counter += 1

    def status_callback(self, msg):
        self.get_logger().info(f'Received status: {msg.data}')

class MovementController(Node):
    """
    A controller that executes movement commands
    """
    def __init__(self):
        super().__init__('movement_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.command_callback,
            10)
        self.status_publisher = self.create_publisher(String, 'movement_status', 10)

    def command_callback(self, msg):
        # In a real robot, this would interface with hardware
        self.get_logger().info(f'Executing: linear={msg.linear.x}, angular={msg.angular.z}')

        # Send status update
        status_msg = String()
        status_msg.data = f'Executed command: linear={msg.linear.x}, angular={msg.angular.z}'
        self.status_publisher.publish(status_msg)

# Example of how to run both nodes in the same process
def main(args=None):
    rclpy.init(args=args)

    agent = MovementAgent()
    controller = MovementController()

    # Run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(agent)
    executor.add_node(controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    agent.destroy_node()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Considerations

When implementing communication patterns, consider the appropriate QoS settings:

- **Reliable vs Best Effort**: Use reliable for critical data, best effort for streaming data
- **Durability**: Use transient local for data that new subscribers should receive immediately
- **History**: Keep last N messages vs. keep all messages
- **Deadline**: Set expectations for message delivery timing

## Hands-on Exercises

### Exercise 1: Simple Publisher-Subscriber
Create a publisher that sends temperature readings and a subscriber that logs these readings to the console. Use appropriate QoS settings for sensor data.

### Exercise 2: Service for Robot Control
Create a service that accepts a robot movement command (forward, backward, turn) and returns whether the command was executed successfully.

### Exercise 3: Agent-Controller Pattern
Implement a simple agent that decides when to move based on sensor input and a controller that executes the movement commands.

### Exercise 4: Quality of Service Experiment
Create two publishers with different QoS settings (reliable vs. best-effort) and observe the differences in communication behavior.

## Summary

This chapter covered the fundamental communication patterns in ROS 2: Nodes, Topics, and Services. Understanding these concepts is crucial for building robust robotic systems. Topics provide asynchronous, one-way communication ideal for sensor streams and status updates. Services provide synchronous, two-way communication perfect for request-response interactions. The agent ↔ controller pattern demonstrates how these primitives work together in humanoid robotics applications.

## Additional Resources

- [ROS 2 Topics Documentation](https://docs.ros.org/en/rolling/Concepts/About-Topics/)
- [ROS 2 Services Documentation](https://docs.ros.org/en/rolling/Concepts/About-Services/)
- [ROS 2 Quality of Service Documentation](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings/)
- [rclpy Tutorials](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Node-Construction-And-Destruction.html)

## Related Concepts

For more information on related topics, see:
- [ROS 2 Fundamentals](./ros2-basics): Review the foundational concepts of ROS 2 architecture and DDS
- [Robot Modeling with URDF](./urdf-python-ros-integration): Learn how to describe robot structure, which is essential for creating robot models that use these communication patterns

## References

[^1]: ROS 2 Documentation. (2024). *About Topics*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Topics/
[^2]: ROS 2 Documentation. (2024). *About Services*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Services/
[^3]: ROS 2 Documentation. (2024). *Writing a Simple Publisher and Subscriber (Python)*. Retrieved from https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber/
[^4]: ROS 2 Documentation. (2024). *Writing a Simple Service and Client (Python)*. Retrieved from https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client/
[^5]: ROS 2 Documentation. (2024). *Using Parameters in a Class (Python)*. Retrieved from https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python/
[^6]: ROS 2 Documentation. (2024). *Writing a Simple Publisher and Subscriber (C++)*. Retrieved from https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber/

---

**Page Feedback**

Was this chapter helpful? [Yes/No] [Provide feedback]

[Back to Module Overview](../intro) | [Previous Chapter: ROS 2 Basics](./ros2-basics) | [Next Chapter: URDF & Python–ROS Integration](./urdf-python-ros-integration)