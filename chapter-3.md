---
sidebar_position: 3
---

# Chapter 3: ROS 2 Architecture

## What is ROS 2?

ROS 2 (Robot Operating System 2) is middleware that provides:
- Communication infrastructure
- Hardware abstraction
- Package management
- Development tools

## Core Concepts

### Nodes

A **node** is a single executable that performs computation.
mermaid
graph LR
    A[Camera Node] --> B[Image Topic]
    B --> C[Vision Node]
    C --> D[Detection Results]


### Topics

Topics enable **publish-subscribe** communication:
python
# Publisher example (conceptual)
publisher = node.create_publisher(topic_name, message_type)
publisher.publish(message)

# Subscriber example (conceptual)
subscriber = node.create_subscription(topic_name, callback_function)


### Services

Services provide **request-response** communication:
mermaid
sequenceDiagram
    Client->>Server: Request
    Server->>Server: Process
    Server->>Client: Response


### Actions

Actions for **long-running tasks** with feedback:
mermaid
sequenceDiagram
    Client->>Server: Goal
    Server->>Client: Feedback (ongoing)
    Server->>Client: Feedback (ongoing)
    Server->>Client: Result (final)


## ROS 2 Graph

The computation graph shows all nodes and their connections:
mermaid
graph TD
    A[Camera Node] -->|/image| B[Object Detector]
    B -->|/detections| C[Planner Node]
    C -->|/cmd_vel| D[Motor Controller]
    E[LiDAR Node] -->|/scan| C


## Communication Patterns

### Pub/Sub (Topics)

Best for:
- Sensor data streams
- Continuous data flow
- One-to-many communication

### Request/Response (Services)

Best for:
- Occasional operations
- Computation requests
- Configuration changes

### Goal-Oriented (Actions)

Best for:
- Navigation commands
- Long computations
- Cancellable operations

## ROS 2 Distributions

| Distribution | Release Date | Support Until |
|--------------|--------------|---------------|
| Foxy | June 2020 | May 2023 |
| Humble | May 2022 | May 2027 (LTS) |
| Iron | May 2023 | Nov 2024 |

**Recommendation**: Use Humble (Long Term Support)

## ROS 2 vs ROS 1
mermaid
graph LR
    A[ROS 1] -->|Single Master| B[Centralized]
    C[ROS 2] -->|DDS| D[Decentralized]
    D --> E[Better for Multi-Robot]


### Key Improvements in ROS 2
- No single point of failure
- Real-time capable
- Better security
- Multi-platform support (Windows, macOS)
- Improved Python and C++ APIs

## Data Distribution Service (DDS)

ROS 2 uses DDS for communication:
mermaid
graph TD
    A[Node A] -->|Publish| B[DDS Layer]
    B -->|Discovery| C[DDS Layer]
    C -->|Subscribe| D[Node B]


### DDS Benefits
- Automatic discovery
- Quality of Service (QoS) policies
- Reliable data delivery
- Scalable architecture

## Quality of Service (QoS)

QoS policies control communication behavior:

| Policy | Options | Use Case |
|--------|---------|----------|
| Reliability | Reliable, Best Effort | Sensor data vs Commands |
| Durability | Volatile, Transient Local | Temporary vs Persistent |
| History | Keep Last N, Keep All | Buffer size |

## Chapter Summary

- ROS 2 is middleware for robot software
- Nodes communicate via topics, services, actions
- DDS provides robust communication
- QoS policies control data delivery
- Humble is the recommended LTS version

## Exercise

Design a ROS 2 graph for a delivery robot that:
1. Reads camera and LiDAR data
2. Detects obstacles
3. Plans a path
4. Controls motors

Draw the nodes, topics, and data flow.

---

**Next**: [Chapter 4: Python-ROS Integration →](chapter-4)