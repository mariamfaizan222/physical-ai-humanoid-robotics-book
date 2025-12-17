---
title: ROS 2 Architecture
sidebar_position: 3
---

# Chapter 3: ROS 2 Architecture

Understanding the underlying architecture of ROS 2 is crucial for designing, developing, and debugging complex robotic systems. ROS 2 is designed to be a flexible, distributed middleware that facilitates communication between various software components (nodes) running on potentially different machines. This chapter will explore the core components of the ROS 2 architecture and how they enable powerful robotic applications.

## The Pillars of ROS 2 Architecture

At its heart, ROS 2 is a communication system. It provides standardized ways for different parts of a robot's software to talk to each other. The main communication paradigms and components are:

### 1. Nodes

**Nodes** are the fundamental building blocks of a ROS 2 system. Each node is typically an executable that performs a specific task, such as reading sensor data, processing images, planning paths, or controlling motors. A robot system is composed of many nodes working together. For example, a robot might have a `camera_node` for image capture, an `image_processor_node` for object detection, a `path_planner_node` for navigation, and a `motor_controller_node` for movement.

Nodes are designed to be independent and reusable, allowing for modular development and easy substitution of components.

### 2. DDS (Data Distribution Service)

Unlike ROS 1, which relied on a central master node, ROS 2 is inherently **masterless**. This is achieved through its underlying communication middleware: **Data Distribution Service (DDS)**. DDS is an industry standard for real-time, publish-subscribe data sharing.

*   **Real-Time Communication**: DDS is designed for high-performance, low-latency communication, essential for robotics.
*   **Decentralized Discovery**: Nodes can discover each other automatically over the network without a central coordinator.
*   **Quality of Service (QoS)**: DDS provides fine-grained control over communication reliability, durability, and liveliness, allowing developers to tailor communication to specific needs (e.g., reliable delivery for commands, best-effort for sensor data). ROS 2 exposes these QoS settings to the user.

### 3. Communication Paradigms

ROS 2 employs several primary communication mechanisms, built on top of DDS:

*   **Topics (Publish/Subscribe)**: This is the most common paradigm, used for asynchronous, one-to-many communication. A **publisher** node sends data messages on a named **topic**, and any number of **subscriber** nodes interested in that topic can receive the data. This is ideal for streaming data like sensor readings (images, LiDAR scans, joint states) or continuous state updates.
    *   *Example*: A `camera_driver` node publishes images on the `/camera/image_raw` topic. A `perception_node` subscribes to this topic to process the images.
*   **Services (Request/Reply)**: Used for synchronous, one-to-one communication where a **client** requests a specific operation from a **server**, and the server performs the task and sends back a response. This is suitable for discrete operations that require a definite result before proceeding.
    *   *Example*: A `moveit_node` might offer a `compute_ik` service that a `planning_node` calls to get inverse kinematics solutions.
*   **Actions**: Designed for long-running, pre-emptible tasks that may require feedback during execution and a final result. An **action client** sends a **goal** to an **action server**, which can provide **feedback** during execution and eventually return a **result**.
    *   *Example*: A `navigation_node` might offer a navigation action. A client node could send a goal to navigate to a specific pose. The action server would provide feedback on the robot's progress and eventually report success or failure.
*   **Parameters**: Nodes can expose parameters that can be dynamically set or queried at runtime. This allows for easy configuration of node behavior without recompiling code.

## A Typical ROS 2 System Diagram

Imagine a humanoid robot navigating a room:

*   **Sensor Nodes**:
    *   A `camera_node` publishes images and camera info on `/camera/image_raw` and `/camera/camera_info` topics.
    *   A `lidar_node` publishes `sensor_msgs/msg/LaserScan` data on the `/scan` topic.
    *   An `imu_node` publishes `sensor_msgs/msg/Imu` data on the `/imu/data` topic.
    *   A `joint_state_publisher` node publishes `sensor_msgs/msg/JointState` on `/joint_states`.

*   **Perception Nodes**:
    *   A `pointcloud_processor` node subscribes to `/scan` and camera data to generate a 3D point cloud, publishing it on `/perception/pointcloud`.
    *   An `object_detector` node subscribes to `/camera/image_raw` and publishes detected objects on `/perception/detected_objects`.

*   **Planning Nodes**:
    *   A `slam_node` subscribes to `/scan`, `/imu/data`, and potentially `/joint_states` to build a map and estimate the robot's pose, publishing map data and odometry.
    *   A `navigation_node` (using Nav2) subscribes to the map, robot pose, and `/scan` to plan paths. It might offer navigation goals as an action.

*   **Control Nodes**:
    *   A `controller_node` subscribes to `cmd_vel` topic for base movement commands or potentially to action goals from `navigation_node`.
    *   A `joint_controller_node` subscribes to desired joint states or action goals and publishes commands to the robot's low-level motor controllers.

All these nodes communicate using ROS 2's communication paradigms (topics, actions, services) facilitated by DDS. The DDS middleware handles the discovery and reliable (or best-effort) transmission of messages between nodes, whether they are on the same machine or distributed across a network.

## Practical Use Cases

The ROS 2 architecture is foundational for a wide range of robotic applications:

*   **Distributed Systems**: Nodes can run on different computers (e.g., an embedded robot computer, a powerful workstation for AI processing), communicating over a network.
*   **Real-Time Performance**: DDS enables low-latency communication suitable for time-critical tasks like motor control and sensor fusion.
*   **Modularity and Reusability**: Nodes can be developed independently and reused across different robot platforms and projects.
*   **Scalability**: The masterless, distributed nature allows systems to scale from simple robots to complex multi-robot fleets.
*   **Fault Tolerance**: If one node fails, it doesn't necessarily bring down the entire system, allowing for more resilient robot designs.

## Summary

The ROS 2 architecture is built upon a foundation of nodes communicating via topics, services, and actions, all powered by the DDS middleware. This masterless, distributed design provides flexibility, scalability, and real-time performance essential for modern robotics. By understanding these core components—nodes, DDS, and the various communication paradigms—developers can effectively build complex, intelligent, and robust robotic systems, from simple sensor processing to sophisticated AI-driven humanoid robots.
---