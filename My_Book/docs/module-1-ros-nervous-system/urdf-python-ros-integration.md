---
title: URDF & Python–ROS Integration
description: Understanding URDF for humanoid robots and Python-ROS integration, including simulation readiness
tags: [urdf, python, ros-integration, robot-modeling, humanoid-robotics, simulation, intermediate]
---

# URDF & Python–ROS Integration

## Overview

This chapter covers URDF (Unified Robot Description Format) for humanoid robots and how to integrate it with Python-based ROS systems. URDF is an XML-based format that describes robot models, including their physical properties, kinematic structure, and visual appearance. Understanding URDF is crucial for humanoid robotics and simulation readiness.

## Learning Objectives

By the end of this chapter, you should be able to:
- Understand the fundamentals of URDF and its role in robotics
- Create and modify URDF files for humanoid robots
- Integrate URDF with Python-based ROS systems
- Prepare robot models for simulation
- Implement practical examples using URDF and Python

## URDF Fundamentals

URDF (Unified Robot Description Format) is an XML-based format that describes robot models. It defines:

- **Physical structure**: Links (rigid bodies) and joints (connections between links)
- **Visual appearance**: How the robot looks in simulation and visualization tools
- **Collision properties**: How the robot interacts with the environment in simulation
- **Kinematic properties**: Mass, inertia, and other physical characteristics

### Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0.0 0.0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots have specific requirements in their URDF descriptions:

### Key Components of Humanoid URDF

1. **Trunk**: The main body including torso and pelvis
2. **Head**: Including sensors like cameras and IMUs
3. **Arms**: Shoulders, elbows, wrists, and hands
4. **Legs**: Hips, knees, ankles, and feet
5. **Actuators**: Joint specifications for movement

### Example: Simple Humanoid Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Trunk -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0.0 0.0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0.0 0.0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Python-ROS Integration with URDF

ROS provides several ways to work with URDF files from Python:

### Loading URDF in Python

```python
import xml.etree.ElementTree as ET

def load_urdf_from_file(file_path):
    """Load URDF from a file and parse it"""
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Extract robot name
    robot_name = root.get('name')
    print(f"Robot name: {robot_name}")

    # Extract links
    links = root.findall('link')
    for link in links:
        link_name = link.get('name')
        print(f"Found link: {link_name}")

        # Check for visual and collision elements
        visual = link.find('visual')
        if visual is not None:
            geometry = visual.find('geometry')
            if geometry is not None:
                shape = geometry[0]  # First geometry element
                print(f"  Visual geometry: {shape.tag}")

    # Extract joints
    joints = root.findall('joint')
    for joint in joints:
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        print(f"Found joint: {joint_name} ({joint_type}) from {parent} to {child}")

# Example usage:
# load_urdf_from_file('path/to/robot.urdf')
```

### Working with Robot State Publisher

The robot_state_publisher package in ROS converts joint states to transforms and publishes them to tf. Here's how to work with it in Python:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):
    """
    Publishes joint states for visualization and simulation
    """
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create timer to publish joint states at regular intervals
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Initialize joint names
        self.joint_names = [
            'torso_to_head',
            'torso_to_left_shoulder',
            'left_shoulder_to_elbow'
        ]

        # Initialize joint angles
        self.joint_angles = [0.0] * len(self.joint_names)
        self.time = 0.0

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names

        # Update joint angles with some oscillating motion
        self.time += 0.1
        for i in range(len(self.joint_angles)):
            self.joint_angles[i] = 0.5 * math.sin(self.time + i)

        msg.position = self.joint_angles
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Publish the message
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Readiness

For a robot model to be ready for simulation, it needs several components:

### 1. Complete Physical Description
- All links must have mass and inertia properties
- Collision geometries should be properly defined
- Visual geometries for rendering

### 2. Kinematic Chain
- Complete kinematic tree from base to all end effectors
- Proper joint limits and types
- Correct parent-child relationships

### 3. Transmission Definitions
For controlling the robot in simulation:

```xml
<transmission name="trans_torso_to_head">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_to_head">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="head_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### 4. Gazebo-Specific Elements
For simulation in Gazebo:

```xml
<gazebo reference="torso">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>torso_to_head</joint_name>
  </plugin>
</gazebo>
```

## Practical Examples

### Example 1: URDF Parser in Python

```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import xml.etree.ElementTree as ET

class URDFVisualizer(Node):
    """
    Visualizes URDF elements as markers in RViz
    """
    def __init__(self):
        super().__init__('urdf_visualizer')
        self.marker_pub = self.create_publisher(Marker, 'urdf_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_urdf_markers)

        # Load URDF from parameter or file
        self.urdf_path = self.declare_parameter('urdf_path', '').value
        if not self.urdf_path:
            self.get_logger().info('No URDF path specified, using default')
            return

        self.urdf_tree = ET.parse(self.urdf_path)
        self.urdf_root = self.urdf_tree.getroot()

    def publish_urdf_markers(self):
        # Create a marker array for all links
        for link in self.urdf_root.findall('link'):
            link_name = link.get('name')
            visual = link.find('visual')

            if visual is not None:
                marker = Marker()
                marker.header.frame_id = link_name
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "urdf"
                marker.id = hash(link_name) % 10000
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Set marker properties based on URDF geometry
                geometry = visual.find('geometry')
                if geometry is not None:
                    shape = geometry[0]
                    if shape.tag == 'box':
                        size = [float(x) for x in shape.get('size').split()]
                        marker.scale.x = size[0]
                        marker.scale.y = size[1]
                        marker.scale.z = size[2]
                    elif shape.tag == 'sphere':
                        radius = float(shape.get('radius'))
                        marker.scale.x = marker.scale.y = marker.scale.z = 2 * radius
                    elif shape.tag == 'cylinder':
                        radius = float(shape.get('radius'))
                        length = float(shape.get('length'))
                        marker.scale.x = marker.scale.y = 2 * radius
                        marker.scale.z = length

                marker.color.a = 0.8  # Alpha
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0

                self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = URDFVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Robot Description Publisher

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

class RobotDescriptionPublisher(Node):
    """
    Publishes robot description to parameter server
    """
    def __init__(self):
        super().__init__('robot_description_publisher')

        # Declare parameter for URDF file path
        self.declare_parameter(
            'urdf_file_path',
            '',
            ParameterDescriptor(description='Path to URDF file')
        )

        # Get URDF file path from parameter
        urdf_file_path = self.get_parameter('urdf_file_path').value

        if urdf_file_path:
            # Read URDF file
            with open(urdf_file_path, 'r') as file:
                urdf_content = file.read()

            # Set robot_description parameter
            self.set_parameters([rclpy.parameter.Parameter(
                'robot_description',
                rclpy.Parameter.Type.STRING,
                urdf_content
            )])

            self.get_logger().info(f'Loaded URDF from {urdf_file_path}')
        else:
            self.get_logger().warn('No URDF file path specified')

def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()

    # Keep the node alive
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for URDF

1. **Use consistent naming**: Follow ROS naming conventions for links and joints
2. **Proper mass properties**: Ensure all links have realistic mass and inertia values
3. **Collision vs visual**: Use simple geometries for collision, detailed for visual
4. **Joint limits**: Always specify appropriate limits for safety
5. **Inertial properties**: Calculate or estimate realistic inertia tensors
6. **Xacro for complex models**: Use Xacro to simplify complex URDF files

## Hands-on Exercises

### Exercise 1: URDF Parser
Create a Python script that parses a URDF file and prints information about all links and joints.

### Exercise 2: Joint State Publisher
Implement a joint state publisher that simulates a simple oscillating motion for a humanoid robot.

### Exercise 3: URDF to TF Publisher
Create a node that reads joint states and publishes the corresponding transforms using tf2.

### Exercise 4: Simulation Setup
Create a complete URDF file for a simple humanoid robot with all necessary elements for Gazebo simulation.

## Summary

URDF is a crucial component in robotics, especially for humanoid robots. It provides a standardized way to describe robot models including their physical properties, kinematic structure, and visual appearance. When integrated with Python-based ROS systems, URDF enables powerful capabilities for simulation, visualization, and control. Understanding how to create, modify, and work with URDF files is essential for humanoid robotics development.

## Additional Resources

- [URDF Documentation](https://wiki.ros.org/urdf)
- [URDF Tutorials](https://wiki.ros.org/urdf/Tutorials)
- [Xacro Documentation](https://wiki.ros.org/xacro)
- [Robot State Publisher](https://wiki.ros.org/robot_state_publisher)

## Related Concepts

For more information on related topics, see:
- [ROS 2 Fundamentals](./ros2-basics): Review the foundational concepts of ROS 2 architecture that underpin robot description systems
- [Communication Patterns](./nodes-topics-services): Learn how URDF models interact with the communication systems through robot_state_publisher and joint state messages

## References

[^1]: ROS Documentation. (2024). *URDF*. Retrieved from https://wiki.ros.org/urdf
[^2]: ROS Documentation. (2024). *URDF/XML/robot*. Retrieved from https://wiki.ros.org/urdf/XML
[^3]: ROS Documentation. (2024). *robot_state_publisher*. Retrieved from https://wiki.ros.org/robot_state_publisher
[^4]: ROS Documentation. (2024). *Working with Transmissions*. Retrieved from https://wiki.ros.org/urdf/XML/Transmission
[^5]: ROS Documentation. (2024). *Xacro*. Retrieved from https://wiki.ros.org/xacro
[^6]: ROS 2 Documentation. (2024). *Using Parameters in a Class (Python)*. Retrieved from https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python/
[^7]: ROS 2 Documentation. (2024). *Using tf2 with Python*. Retrieved from https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Using-Tf2-With-Py/

---

**Page Feedback**

Was this chapter helpful? [Yes/No] [Provide feedback]

[Back to Module Overview](../intro) | [Previous Chapter: Nodes/Topics/Services](./nodes-topics-services)