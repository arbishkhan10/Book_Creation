---
title: Physics Simulation with Gazebo
description: Learn how to create physics-based simulations using Gazebo for humanoid environments
tags: [gazebo, physics, simulation, humanoid, robotics, beginner, tutorial]
---

# Physics Simulation with Gazebo

## Overview

This chapter introduces physics-based simulation using Gazebo, the standard simulation environment for robotics development. You'll learn how to create realistic physics simulations for humanoid environments, including model setup, physics properties, and environment interaction.

## Learning Objectives

By the end of this chapter, you should be able to:
- Set up basic physics simulations in Gazebo
- Configure humanoid models with appropriate physics properties
- Create realistic environments for humanoid simulation
- Understand physics constraints and their impact on humanoid movement

## Introduction to Gazebo Physics Simulation

Gazebo is a robot simulation environment that provides realistic physics simulation, high-quality graphics, and programmatic interfaces. For humanoid robotics, Gazebo provides the essential physics engine that simulates real-world interactions including gravity, collisions, and joint constraints.

### Key Physics Concepts in Gazebo

Gazebo uses several physics engines including ODE (Open Dynamics Engine), Bullet, and DART. These engines handle:

- **Collision detection**: Identifying when objects make contact
- **Dynamics simulation**: Computing forces, torques, and resulting motions
- **Contact processing**: Managing friction, restitution, and other contact properties

### Why Gazebo for Humanoid Simulation

Humanoid robots present unique challenges that make Gazebo particularly valuable:

- **Complex kinematics**: Multiple degrees of freedom and articulated structures
- **Balance and stability**: Critical for humanoid locomotion
- **Environment interaction**: Walking, manipulation, and navigation
- **Sensor simulation**: Integration with realistic sensor models

## Humanoid Model Setup in Gazebo

### Basic Model Structure

A humanoid model in Gazebo consists of multiple links connected by joints:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
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
</robot>
```

### Physics Properties and Constraints

#### Mass and Inertia
Each link must have realistic mass and inertia properties. These values significantly impact how the model responds to forces and torques:

- **Mass**: Should reflect the actual weight of the physical component
- **Inertia**: Describes how mass is distributed around the center of mass
- **Center of Mass**: Critical for balance and stability calculations

#### Joint Constraints
Joints define how links can move relative to each other:

- **Revolute**: Rotational motion around a single axis
- **Prismatic**: Linear motion along a single axis
- **Fixed**: No motion allowed
- **Continuous**: Unlimited rotation around an axis
- **Floating**: Six degrees of freedom

## Environment Setup and Interaction Patterns

### Creating Realistic Environments

Gazebo worlds define the environment in which your humanoid operates. A basic world file includes:

```xml
<sdf version='1.7'>
  <world name='humanoid_world'>
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sky -->
    <include>
      <uri>model://sky</uri>
    </include>

    <!-- Physics properties -->
    <physics type='ode'>
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Your humanoid model would be included here -->
  </world>
</sdf>
```

### Physics Parameters for Humanoid Simulation

#### Time Step Configuration
- **Max step size**: Smaller values provide more accurate simulation but require more computation
- **Real-time factor**: Controls how fast simulation runs relative to real time
- **Update rate**: How frequently physics calculations are performed

#### Collision Parameters
- **Friction**: Determines how objects interact when in contact
- **Restitution**: Controls "bounciness" of collisions
- **Contact properties**: Define how contacts are processed

## Practical Examples and Illustrations

### Example 1: Simple Humanoid Walking Simulation

Here's a complete example of a simple humanoid model with basic walking capabilities:

```xml
<?xml version="1.0"?>
<robot name="walking_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.5 0.2 1.0"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.5 0.2 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Legs -->
  <joint name="left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="-0.15 -0.1 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin rpy="1.57 0 0"/>
    </collision>
  </link>
</robot>
```

### Example 2: Environment with Obstacles

Creating a world with obstacles to test navigation:

```xml
<sdf version='1.7'>
  <world name='obstacle_course'>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type='ode'>
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Obstacles -->
    <model name='box_obstacle'>
      <pose>2 0 0.5 0 0 0</pose>
      <link name='box_link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
```

## Hands-on Exercises for Gazebo Physics Simulation

### Exercise 1: Basic Humanoid Model
Create a simple humanoid model with a torso, head, and basic limbs. Configure the physics properties appropriately and test it in a basic environment.

### Exercise 2: Walking Pattern
Implement a simple walking pattern by controlling the joint angles in a rhythmic fashion. Observe how the physics simulation responds to these commands.

### Exercise 3: Balance Challenge
Create an environment with uneven terrain and implement basic balance control for your humanoid model.

## References

[^1]: Gazebo Documentation. (2024). *Physics Simulation*. Retrieved from http://gazebosim.org/
[^2]: Gazebo Documentation. (2024). *Model Format*. Retrieved from http://gazebosim.org/
[^3]: Open Robotics. (2024). *URDF Integration*. Retrieved from https://classic.gazebosim.org/tutorials?tut=ros_urdf
[^4]: Gazebo Documentation. (2024). *World Files*. Retrieved from http://gazebosim.org/
[^5]: Gazebo Documentation. (2024). *Physics Engine*. Retrieved from http://gazebosim.org/

## Related Concepts

For more information on related topics, see:
- [Digital Twins & HRI in Unity](./digital-twins-hri-unity): Learn how to visualize physics simulation results in Unity
- [Sensor Simulation & Validation](./sensor-simulation-validation): Understand how sensors interact with physics simulations

---

**Page Feedback**

Was this chapter helpful? [Yes/No] [Provide feedback]

[Back to Module Overview](../intro) | [Next Chapter: Digital Twins & HRI in Unity](./digital-twins-hri-unity)