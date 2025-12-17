---
title: URDF Modeling for Humanoid Robots
sidebar_position: 7
---

# URDF Modeling for Humanoid Robots

To simulate and control robots, especially complex ones like humanoids, we need a way to describe their physical structure, including their components, how they are connected, and their physical properties. The **Unified Robot Description Format (URDF)** is an XML-based file format used by ROS to represent a robot's kinematics and dynamics.

## What is URDF?

URDF allows you to define a robot as a collection of interconnected **links** and **joints**. Each link represents a physical component of the robot (e.g., a leg, a torso, a finger), and each joint defines the articulation between two links (e.g., a revolute joint for a knee, a fixed joint for a rigid connection).

A URDF file typically describes:
*   **Links**: Their geometry (visual and collision shapes), visual properties (colors, textures), and inertial properties (mass, inertia tensor).
*   **Joints**: The type of joint (revolute, prismatic, fixed, etc.), the parent and child links they connect, their limits (e.g., angle range for a revolute joint), and their axes of motion.

## Core URDF Elements

A URDF file is structured hierarchically, starting with a root `<robot>` element.

```xml
<robot name="my_humanoid_robot">
  <!-- Links -->
  <link name="base_link">
    <!-- Inertial properties -->
    <inertial>...</inertial>
    <!-- Visual representation -->
    <visual>...</visual>
    <!-- Collision geometry -->
    <collision>...</collision>
  </link>

  <link name="leg_front_left_link">
    ...
  </link>

  <!-- Joints -->
  <joint name="base_to_leg_front_left_joint" type="revolute">
    <!-- Parent and child links -->
    <parent link="base_link"/>
    <child link="leg_front_left_link"/>
    <!-- Joint limits and properties -->
    <limit lower="..." upper="..." effort="..." velocity="..."/>
    <axis xyz="..."/>
    <!-- Joint origin transformation -->
    <origin xyz="..." rpy="..."/>
  </joint>

  <!-- More links and joints -->
</robot>
```

### `<link>` Element
A `<link>` element defines a rigid body. It can contain:
*   `<inertial>`: Mass and inertia tensor, crucial for physics simulation.
*   `<visual>`: Defines how the link is rendered in simulation and visualization tools (geometry, material).
*   `<collision>`: Defines the collision shape used by the physics engine.

### `<joint>` Element
A `<joint>` element describes the connection between two links. It defines:
*   `name`: A unique name for the joint.
*   `type`: The type of motion allowed (e.g., `revolute` for rotation, `prismatic` for linear motion, `fixed` for no relative motion, `continuous` for unlimited rotation).
*   `<parent>` and `<child>`: The links this joint connects.
*   `<origin>`: The transformation (translation and rotation) of the joint relative to its parent link.
*   `<axis>`: The axis of rotation or translation for non-fixed joints.
*   `<limit>`: Defines the operational range (e.g., `lower` and `upper` bounds for revolute/prismatic joints), maximum effort, and velocity.

## Modeling a Humanoid Robot

Creating a URDF for a humanoid robot involves defining each body segment as a link and the connections between them as joints. A typical structure might include:

*   **Base Link**: The main body or torso.
*   **Head**: Connected to the torso via a `revolute` or `fixed` joint.
*   **Arms**: Each arm (upper arm, forearm, hand) connected via `revolute` joints.
*   **Legs**: Each leg (hip, thigh, shin, foot) connected via `revolute` or `continuous` joints.

You'll need to carefully define the `origin` and `axis` for each joint to correctly represent the robot's pose and degrees of freedom.

### Inertial, Visual, and Collision Properties

*   **Inertial**: Accurately defining mass and inertia is vital for realistic physics simulations. While approximate values can be used initially, precise values improve simulation accuracy.
*   **Visual**: This section defines how the robot looks. You can use primitive shapes (cubes, cylinders, spheres) or meshes (e.g., `.dae`, `.stl` files) and assign materials with colors.
*   **Collision**: Defines the geometric shape used for physics calculations. It should be a simplified representation of the visual shape to ensure efficient collision detection.

## Visualizing URDF Models

Tools like `RViz` (ROS Visualization) and dedicated URDF viewers can load and display URDF models, allowing you to inspect your robot's structure, joint configurations, and ensure the model is correctly defined before integrating it into Gazebo.

*   **RViz**: You can add a "URDF" display to RViz and point it to your URDF file or a running ROS node that advertises the robot description topic.
*   **`check_urdf` Tool**: A command-line utility that checks for basic errors in your URDF file.

Defining an accurate and detailed URDF is a foundational step for simulating and controlling humanoid robots, enabling complex behaviors and interactions in the virtual world.
---
