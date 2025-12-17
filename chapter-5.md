---
title: ROS 2 Communication Patterns and Best Practices
sidebar_position: 5
---

# ROS 2 Communication Patterns and Best Practices

Effective communication between different parts of a robotic system is fundamental. ROS 2 provides several powerful communication mechanisms, each suited for different types of data exchange. Understanding these patterns—Topics, Services, and Actions—and employing best practices will lead to more robust, efficient, and maintainable robotic software.

## The Role of DDS

At its core, ROS 2 leverages the Data Distribution Service (DDS) standard for its data-centric publish-subscribe communication. DDS handles the complexities of real-time data sharing, discovery, and Quality of Service (QoS) settings, enabling ROS 2 to operate reliably in distributed and challenging network environments.

## Topics: Publish/Subscribe for Continuous Data

**Topics** are used for asynchronous, one-to-many communication. A **publisher** sends messages of a specific type on a named **topic**, and any number of **subscribers** can receive these messages. This pattern is ideal for streaming data like sensor readings, robot state, or diagnostic information where data is continuously updated.

### When to Use Topics:
*   Streaming sensor data (e.g., camera images, LiDAR scans, IMU readings).
*   Broadcasting robot odometry or state information.
*   Publishing control commands that are meant to be continuously applied.

### Example: Publishing Robot Velocity

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Velocity publisher started')

    def timer_callback(self):
        msg = Twist()
        # Example: move forward at 0.2 m/s
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

A subscriber node would then listen to the `cmd_vel` topic to control the robot's movement.

## Services: Request/Reply for Synchronous Operations

**Services** are used for synchronous, one-to-one communication. A **client** sends a request to a **server**, and the server performs a specific task and sends back a response. This is suitable for operations that require a definitive answer or completion before proceeding.

### When to Use Services:
*   Requesting a specific computation or transformation (e.g., inverse kinematics).
*   Setting a configuration parameter that requires immediate confirmation.
*   Performing a discrete action that should not be interrupted once started.

### Example: Adding Two Numbers Service

A service definition typically involves a request message and a response message. For an "add two numbers" service:

```python
# Service definition (e.g., in a file like 'srv/AddTwoInts.srv')
# int64 a
# int64 b
# ---
# int64 sum
```

**Server Node:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Assuming this service type exists

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add two ints server ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a} b={request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    rclpy.spin(add_two_ints_server)
    rclpy.shutdown()
```

**Client Node:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
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

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_client = AddTwoIntsClient()
    response = add_two_ints_client.send_request(3, 7)
    add_two_ints_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The client waits for the server to be available, sends the request, and blocks until it receives the response.

## Actions: For Long-Running, Pre-emptible Tasks

**Actions** are designed for tasks that take a significant amount of time to complete and may require feedback during execution or the ability to be cancelled. They provide a structured way to manage goals, feedback, and results asynchronously.

### When to Use Actions:
*   Navigation tasks (e.g., "go to location X").
*   Manipulation tasks (e.g., "pick up object Y").
*   Complex motion planning or trajectory execution.

### Key Components of an Action:
*   **Goal**: The request sent by the client.
*   **Feedback**: Information sent by the server to the client during execution.
*   **Result**: The final outcome of the task.

ROS 2 provides action libraries and templates to help define and implement custom actions.

## Best Practices for ROS 2 Communication

*   **Choose the Right Pattern**: Use topics for streaming data, services for quick requests/responses, and actions for long-running tasks.
*   **Quality of Service (QoS)**: Understand and configure QoS settings (reliability, durability, history) for topics to match your application's needs. For example, sensor data might benefit from best-effort, transient-local QoS, while critical commands might need reliable, persistent QoS.
*   **Avoid Blocking Calls**: In nodes that perform continuous operations (like publishers or subscribers), avoid blocking calls in their main threads. Use separate threads or asynchronous operations for services or actions.
*   **Error Handling**: Implement robust error handling for service clients and action clients to manage communication failures or unsuccessful operations.
*   **Message Serialization Efficiency**: Use efficient message types and consider serialization formats if dealing with very large data payloads.
*   **Clear Naming Conventions**: Use consistent and descriptive names for topics, services, and actions.

By mastering these communication patterns, you can build sophisticated and reliable ROS 2 systems for your humanoid robots.
---
