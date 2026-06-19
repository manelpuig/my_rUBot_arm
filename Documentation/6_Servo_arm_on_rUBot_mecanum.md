# ROS 2 Driver for a 6-DOF Servo Arm Mounted on a Mecanum Robot

## Overview

This package provides a ROS 2 driver for a 6-DOF educational robot arm built with SG90 servos and an Arduino Nano ESP32.

The arm is mounted on the **rUBot mecanum platform**, forming a mobile manipulator that can control both the mobile base and the arm simultaneously.

The driver follows the same ROS 2 architecture used by industrial robots:

```text
MoveIt2 / Motion Nodes
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

---

# System Architecture

The complete robot consists of two independent subsystems:

## Mobile Base

```text
/cmd_vel
    │
    ▼
my_robot_driver
    │
    ▼
Arduino Nano ESP32
    │
    ▼
Mecanum wheels
```

## Robot Arm

```text
MoveIt2 / Kinematics Nodes
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
6 SG90 servos
```

---

# Driver Node

## Node

```text
serial_trajectory_bridge_node
```

## Subscription

```text
/arm_controller/joint_trajectory
```

Type:

```text
trajectory_msgs/msg/JointTrajectory
```

## Publication

```text
/joint_states
```

Type:

```text
sensor_msgs/msg/JointState
```

The driver publishes the commanded joint positions so that RViz, TF, and robot_state_publisher can represent the current arm configuration.

---

# Driver Operation

The driver performs the following steps:

1. Receive a JointTrajectory message.
2. Execute each trajectory point sequentially.
3. Convert joint positions from radians to servo angles.
4. Apply servo calibration offsets and sign corrections.
5. Limit the servo range.
6. Send six angles through the serial port.
7. Update and publish `/joint_states`.

Example serial message:

```text
90,120,45,90,90,90
```

---

# Arduino Firmware

The Arduino acts only as a low-level servo controller.

Its responsibilities are:

* Receive serial commands.
* Parse six comma-separated angles.
* Update the six SG90 servos.

The Arduino does not perform:

* Forward kinematics
* Inverse kinematics
* Motion planning
* Trajectory generation

All robot intelligence remains inside ROS 2.

---

# Motion Layer

Motion generation is implemented in:

```text
my_arm_motion
my_arm_kinematics
```

These nodes can perform:

* Forward kinematics
* Inverse kinematics
* Cartesian motions
* MoveIt2 planning
* Joint trajectories

All commands are published to:

```text
/arm_controller/joint_trajectory
```

---

# Example Architecture

```text
            MoveIt2
               │
               ▼
      /arm_controller/joint_trajectory
               │
               ▼
   serial_trajectory_bridge_node
               │
          USB Serial
               │
               ▼
        Arduino Nano ESP32
               │
               ▼
            SG90 Servos

               ▲
               │
           /joint_states
```

---

# Launch Driver

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py \
    serial_port:=/dev/ttyUSB0
```

---

# Test with a Single Target

Launch the driver:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py
```

Send a target configuration:

```bash
ros2 run my_arm_driver send_joint_target_node \
    --ros-args \
    -p target_joints_deg:="[0,0,0,0,0,0]"
```

Another example:

```bash
ros2 run my_arm_driver send_joint_target_node \
    --ros-args \
    -p target_joints_deg:="[20,-30,40,0,0,0]"
```

---

# Test with a Trajectory

Launch the driver:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py
```

Run:

```bash
ros2 run my_arm_driver send_joint_trajectory_node
```

The node sends multiple trajectory points, and the driver executes them sequentially.

---

# Typical Integration

The arm driver is intended to be integrated into the complete robot bringup:

```text
my_robot_bringup
│
├── my_robot_driver          (mecanum base)
├── serial_trajectory_bridge_node (robot arm)
├── robot_state_publisher
└── RViz
```

This allows simultaneous control of the mobile base and the 6-DOF robot arm, creating a complete ROS 2 mobile manipulator.
