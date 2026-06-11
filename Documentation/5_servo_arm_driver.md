# ROS 2 Driver for a 6-DOF Servo Robot Arm using Arduino Nano ESP32

## Overview

This driver allows a 6-DOF educational robotic arm built with SG90 servos and an Arduino Nano ESP32 to be controlled from ROS 2 Humble using the standard ROS trajectory interface.

The architecture follows the same philosophy used by industrial robot drivers:

```text
MoveIt2 / Kinematics Nodes
            │
            ▼
/arm_controller/joint_trajectory
            │
            ▼
serial_trajectory_bridge_node
            │ USB Serial
            ▼
Arduino Nano ESP32
            │
            ▼
SG90 Servos
```

The objective is to make the educational arm compatible with the same ROS 2 nodes already developed for industrial robots such as the UR5e and the PUMA manipulator.

---

# System Architecture

The system is divided into three layers:

## Layer 1: Motion Planning

This layer generates robot motions.

Examples:

* Forward Kinematics (FK)
* Inverse Kinematics (IK)
* MoveIt2 motion planning
* Cartesian trajectories
* Joint trajectories

Nodes from:

```text
my_arm_kinematics
my_arm_motion
```

publish standard ROS trajectories to:

```text
/arm_controller/joint_trajectory
```

---

## Layer 2: ROS 2 Driver

The driver subscribes to:

```text
/arm_controller/joint_trajectory
```

and converts ROS joint positions into servo angles.

The driver acts as a simplified hardware interface.

### Input

```text
trajectory_msgs/JointTrajectory
```

### Output

```text
90,120,45,90,90,90
```

sent through the serial port.

---

## Layer 3: Arduino Controller

The Arduino receives six servo angles and updates the servos.

Example received message:

```text
90,120,45,90,90,90
```

The Arduino parses the six values and executes:

```cpp
servo.write(angle);
```

for each servo.

---

# ROS Driver Node

## Node Name

```text
serial_trajectory_bridge_node
```

## Subscription

```text
/arm_controller/joint_trajectory
```

Type:

```cpp
trajectory_msgs/msg/JointTrajectory
```

---

## Processing Pipeline

### Step 1

Receive a trajectory:

```text
JointTrajectory
 ├── joint_names
 └── points[]
```

---

### Step 2

Extract target positions:

```python
point = msg.points[-1]
```

The last point corresponds to the final target configuration.

---

### Step 3

Convert radians to degrees:

```python
joint_deg = math.degrees(joint_rad)
```

---

### Step 4

Apply servo calibration:

```python
servo_deg =
offset[i] + sign[i] * joint_deg
```

Example:

```python
offset = [90,90,90,90,90,90]

sign = [1,-1,1,1,-1,1]
```

This allows each servo to have its own:

* mechanical zero
* rotation direction

---

### Step 5

Limit output range:

```python
servo_deg = max(0,min(180,servo_deg))
```

---

### Step 6

Send serial message:

```python
90,120,45,90,90,90\n
```

---

# Arduino Firmware

## Responsibilities

The Arduino acts only as a servo controller.

It does not perform:

* kinematics
* inverse kinematics
* trajectory generation
* motion planning

Those tasks remain inside ROS 2.

---

## Arduino Program

The firmware:

1. Receives serial data.
2. Parses six comma-separated values.
3. Updates each servo position.

Example:

```cpp
servo.write(angle);
```

---

# Using the Driver with FK Nodes

A simple example is:

```text
puma_fkine.py
```

The node publishes a target joint configuration.

Example:

```python
target_joints_deg =
[
    0.0,
    -45.0,
    90.0,
    0.0,
    45.0,
    0.0
]
```

The node converts degrees to radians and publishes:

```text
/arm_controller/joint_trajectory
```

The driver receives:

```text
JointTrajectory
```

and converts it into:

```text
90,45,180,90,135,90
```

which is sent to the Arduino.

---

# Example Launch Sequence

## Terminal 1

Start the ROS driver:

```bash
ros2 run my_arm_driver serial_trajectory_bridge_node
```

---

## Terminal 2

Run FK example:

```bash
ros2 launch my_arm_kinematics puma_fkine.launch.py
```

---

Expected result:

```text
puma_fkine
        │
        ▼
/arm_controller/joint_trajectory
        │
        ▼
serial_trajectory_bridge_node
        │
        ▼
Arduino Nano ESP32
        │
        ▼
6 SG90 Servos
```

---

# Compatibility with Existing Packages

The driver is compatible with any node that publishes:

```text
/arm_controller/joint_trajectory
```

including:

## my_arm_kinematics

* puma_fkine.py
* puma_ikine_position.py
* ur5e_fkine.py
* ur5e_ikine_position.py
* trajectory generators

## my_arm_motion

* move_to_pose.py
* move_to_configuration.py
* arm_pose_sequence.py
* send_pose_trajectory.py

## MoveIt2

Any MoveIt2 planning pipeline publishing standard trajectories.

---

# Future Improvements

## Trajectory Execution

Current implementation:

```python
point = msg.points[-1]
```

Only the final point is executed.

Future implementation:

```python
for point in msg.points:
```

Execute every trajectory point using:

```python
point.time_from_start
```

This would allow smoother motions and better compatibility with MoveIt2 trajectory planning.

---

# Educational Value

This architecture reproduces the same software layers found in industrial robot systems:

```text
Application Layer
     ↓
Motion Planning
     ↓
ROS Driver
     ↓
Hardware Interface
     ↓
Robot Controller
     ↓
Actuators
```

allowing students to learn ROS 2, robot kinematics, trajectory generation, and hardware integration using a low-cost educational robot arm.
