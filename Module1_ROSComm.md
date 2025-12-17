---
title: ROS 2 Communication Patterns and Best Practices
sidebar_position: 5
---

# Chapter 5: ROS 2 Communication Patterns and Best Practices

Effective communication between the various software components (nodes) of a robotic system is paramount for its correct functioning. ROS 2 provides a robust and flexible middleware built on DDS (Data Distribution Service) that offers several communication paradigms: Topics, Services, and Actions. Each pattern is designed for specific types of interaction, and understanding when to use which, along with adhering to best practices, is key to building reliable and efficient robotic applications.

## Introduction to ROS 2 Communication

In a complex robot, different processes perform distinct tasks—sensors gather data, perception algorithms process it, planning modules decide on actions, and control systems execute them. These processes must communicate data and commands efficiently. ROS 2's architecture, built on DDS, provides the tools to manage this distributed communication. The choice of communication pattern significantly impacts the system's performance, responsiveness, and predictability.

## Core Communication Paradigms

ROS 2 offers three primary patterns for inter-process communication:

### 1. Topics (Publish/Subscribe)

Topics are the most common communication mechanism in ROS 2, facilitating asynchronous, one-to-many communication.

*   **How it works**: A **publisher** node sends messages of a specific type on a named **topic**. Any number of **subscriber** nodes interested in that topic can receive these messages. There is no direct connection between publishers and subscribers; they communicate indirectly through the DDS middleware, which handles message routing and discovery.
*   **When to use**: Ideal for streaming continuous data or broadcasting state information. Examples include:
    *   Publishing sensor readings (camera images, LiDAR scans, IMU data, joint states).
    *   Broadcasting robot odometry or pose estimates.
    *   Publishing control commands that are intended to be continuously applied (e.g., velocity commands to a robot base).
*   **Quality of Service (QoS)**: ROS 2 allows fine-grained control over QoS profiles for topics, enabling developers to specify reliability (e.g., best-effort vs. reliable), durability (e.g., transient local, to keep history), and liveliness settings, tailoring communication to specific needs.

**Example: Robot Velocity Commands**

A common use case is publishing velocity commands to control a robot's movement.

*   **Publisher Node**: A node (e.g., `teleop_twist_keyboard` or a higher-level planner) publishes `geometry_msgs/msg/Twist` messages on the `/cmd_vel` topic.
*   **Subscriber Node**: A `robot_driver` node subscribes to `/cmd_vel` and translates these commands into actual motor commands for the robot.

```python
# Example Publisher (conceptual snippet)
# ... inside a ROS 2 node ...
self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
msg = Twist()
msg.linear.x = 0.2  # Move forward at 0.2 m/s
self.cmd_vel_publisher.publish(msg)
```

### 2. Services (Request/Reply)

Services are used for synchronous, one-to-one communication, where a client requests an operation from a server and waits for a response.

*   **How it works**: A **client** node calls a **service server** node, sending a request. The server node performs a specific task based on the request and then sends a **response** back to the client. The client typically blocks until the response is received.
*   **When to use**: Suitable for operations that require a definite outcome or are computationally intensive and should not be interrupted. Examples include:
    *   Requesting a specific calculation (e.g., inverse kinematics, path validation).
    *   Calling a function that requires exclusive access or a definitive result (e.g., saving robot configuration, requesting a complex query).
    *   Performing a discrete, blocking task.

**Example: Simple Service for Addition**

A service might be defined to add two integers.

*   **Service Server Node**: Listens for requests on a service named `add_two_ints`, performs addition, and returns the sum.
*   **Service Client Node**: Calls the `add_two_ints` service with two numbers and waits for the sum.

```python
# Example Service Server (conceptual snippet)
# ... inside a ROS 2 node ...
self.add_two_ints_server = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    return response
```

### 3. Actions

Actions are designed for longer-running tasks that may require feedback during execution and can be pre-empted or cancelled.

*   **How it works**: An **action client** sends a **goal** to an **action server**. The server begins working on the goal and can periodically send **feedback** to the client about its progress. Once the task is complete, the server sends a final **result**. The client can also cancel the goal at any time.
*   **When to use**: Ideal for tasks that take time, involve sequential steps, or require monitoring. Examples include:
    *   Navigation (e.g., "Go to location X" - feedback is current pose, result is success/failure).
    *   Manipulation (e.g., "Pick up object Y" - feedback might be arm joint angles, result is successful grasp).
    *   Complex computations or deployments that can take seconds or minutes.

**Conceptual Diagram of Communication Patterns:**

Imagine a ROS 2 system as a graph where nodes are circles and communication links are arrows:

*   **Topics (Pub/Sub)**: One publisher node sending data on a topic. Multiple subscriber nodes receiving that data. Arrows originate from the publisher and point to each subscriber. This represents a one-to-many broadcast.
*   **Services (Request/Reply)**: One client node making a request to one server node. Arrows go from client to server for the request and back from server to client for the response. This is a one-to-one, synchronous interaction.
*   **Actions (Client/Server with Feedback)**: One action client sending a goal to one action server. Arrows go from client to server for the goal. Arrows can go from server to client for feedback during execution. A final arrow from server to client carries the result. This is a one-to-one, asynchronous interaction with progress reporting.

## Best Practices for ROS 2 Communication

*   **Choose the Right Pattern**: Select the communication paradigm that best fits the task: topics for continuous data, services for quick, discrete operations, and actions for long-running, feedback-driven tasks.
*   **Quality of Service (QoS)**: Carefully configure QoS settings for topics. For critical commands, use reliable transport. For high-frequency sensor data where occasional drops are acceptable, use best-effort. Use transient local QoS to ensure new subscribers receive the latest data published before they connected.
*   **Node Design**: Keep nodes focused on a single responsibility. Avoid creating monolithic nodes that do too many things. This promotes reusability and simplifies debugging.
*   **Clear Naming**: Use consistent and descriptive names for topics, services, actions, and parameters. This makes systems easier to understand and integrate.
*   **Error Handling**: Implement robust error handling in clients (services and actions) to manage cases where servers are unavailable, requests time out, or operations fail.
*   **Asynchronous Operations**: For nodes that need to perform multiple tasks concurrently (e.g., publishing data while also responding to service calls), use asynchronous patterns and multi-threading where appropriate.
*   **Message Definitions**: Define custom ROS 2 messages, services, and actions for application-specific data structures to ensure type safety and clarity.

## Practical Use Cases

*   **Robot Navigation**: A navigation stack typically uses topics for sensor data (LiDAR, odometry), services for map-related queries, and actions for sending navigation goals.
*   **Manipulation**: A manipulation system might use topics for joint states, services for kinematics computations, and actions for executing complex pick-and-place routines.
*   **Humanoid Robot Control**: A humanoid robot's locomotion and manipulation might be controlled via actions, with joint states and sensor data published on topics, and high-level commands (like "go to charging station") being handled by services or interpreted from voice commands.

## Summary

ROS 2 offers a powerful and flexible set of communication tools—Topics, Services, and Actions—built upon the reliable DDS middleware. By understanding the purpose of each paradigm and adhering to best practices, developers can construct robust, efficient, and scalable robotic systems. Mastering these communication patterns is fundamental to unlocking the full potential of ROS 2 for building intelligent and interactive robots.
---