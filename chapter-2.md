---
sidebar_position: 2
---

# Chapter 2: Sensor Systems

## Introduction to Robot Perception

Robots perceive the world through sensors. This chapter covers the primary sensor types used in humanoid robotics.

## LiDAR (Light Detection and Ranging)

### How LiDAR Works

LiDAR uses laser pulses to measure distances and create 3D maps of the environment.
mermaid
graph TD
    A[Laser Emitter] --> B[Laser Pulse]
    B --> C[Object/Surface]
    C --> D[Reflected Pulse]
    D --> E[Detector]
    E --> F[Calculate Distance]


### Applications
- Obstacle detection
- 3D mapping (SLAM)
- Navigation planning
- Object recognition

### LiDAR Specifications

| Parameter | Typical Value |
|-----------|--------------|
| Range | 0.1m - 100m |
| Accuracy | ±2cm |
| Scan Rate | 5-20 Hz |
| Angular Resolution | 0.1° - 1° |

## Camera Systems

### RGB Cameras

Standard color cameras for visual perception:
- Object detection
- Face recognition
- Scene understanding

### Depth Cameras

Measure distance to each pixel:
- **Stereo Vision**: Two cameras for depth
- **Structured Light**: Projects patterns
- **Time-of-Flight**: Measures light travel time

### Camera Comparison
mermaid
graph LR
    A[RGB Camera] -->|Color Info| D[Vision System]
    B[Depth Camera] -->|Distance Info| D
    C[Thermal Camera] -->|Heat Info| D
    D --> E[Fused Perception]


## IMU (Inertial Measurement Unit)

### Components

An IMU combines:
1. **Accelerometer**: Measures acceleration (3 axes)
2. **Gyroscope**: Measures rotation rate (3 axes)
3. **Magnetometer**: Measures magnetic field (optional)

### Applications in Robotics

- Balance control
- Orientation estimation
- Fall detection
- Motion tracking

### IMU Data Fusion
python
# Conceptual IMU data reading
class IMUData:
    def _init_(self):
        self.acceleration = [ax, ay, az]  # m/s²
        self.angular_velocity = [gx, gy, gz]  # rad/s
        self.orientation = [roll, pitch, yaw]  # radians


## Force/Torque Sensors

### Purpose

Measure forces and moments applied to robot joints or end-effectors.

### Applications
- Compliant grasping
- Contact detection
- Force-controlled assembly
- Human-robot collaboration safety

## Sensor Fusion

### Why Combine Sensors?

No single sensor provides complete information. Sensor fusion combines multiple sensor inputs:
mermaid
graph TD
    A[LiDAR Data] --> E[Sensor Fusion]
    B[Camera Data] --> E
    C[IMU Data] --> E
    D[Force Sensors] --> E
    E --> F[Robust Perception]


### Benefits
- **Redundancy**: Backup if one sensor fails
- **Accuracy**: Cross-validation of measurements
- **Completeness**: Different sensors for different aspects

## Sensor Placement on Humanoids
mermaid
graph TD
    A[Head] -->|Cameras, LiDAR| B[Vision System]
    C[Torso] -->|IMU| D[Balance Control]
    E[Hands] -->|Force/Torque| F[Manipulation]
    G[Feet] -->|Pressure Sensors| H[Stability]


## Chapter Summary

- LiDAR provides accurate 3D mapping
- Cameras enable visual perception
- IMUs track orientation and motion
- Force sensors enable safe interaction
- Sensor fusion combines strengths of each sensor

## Exercise

Design a sensor suite for a humanoid robot working in a warehouse. Which sensors would you prioritize and why?

---

**Next**: [Chapter 3: ROS 2 Architecture →](chapter-3)